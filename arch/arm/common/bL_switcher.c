/*
 * arch/arm/common/bL_switcher.c -- big.LITTLE cluster switcher core driver
 *
 * Created by:	Nicolas Pitre, March 2012
 * Copyright:	(C) 2012  Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TODO:
 *
 * - Allow the outbound CPU to remain online for the inbound CPU to snoop its
 *   cache for a while.
 * - Perform a switch during initialization to probe what the counterpart
 *   CPU's GIC interface ID is and stop hardcoding them in the code.
 * - Local timers migration (they are not supported at the moment).
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/cpu_pm.h>
#include <linux/cpumask.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/clockchips.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/switcher_pm.h>
#include <linux/vexpress.h>

#include <asm/suspend.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/bL_switcher.h>
#include <asm/bL_entry.h>
#include <asm/memblock.h>
#include <asm/spinlock.h>
#include <asm/smp_plat.h>

#define CREATE_TRACE_POINTS
#include <trace/events/power_cpu_migrate.h>

#define MAX_SWITCH_TIMES 50

static unsigned int switch_count[BL_CPUS_PER_CLUSTER];
static unsigned long long switch_time[BL_CPUS_PER_CLUSTER];

unsigned long long read_cntpct(void)
{
        u32 cvall, cvalh;

        asm volatile("mrrc p15, 0, %0, %1, c14" : "=r" (cvall), "=r" (cvalh));

        return ((unsigned long long) cvalh << 32) | cvall;
}

static ssize_t show_st(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	int ctr;
	for (ctr = 0; ctr < BL_CPUS_PER_CLUSTER; ctr++)
		printk("cpu%d: %llu : %d \n",
		       ctr, switch_time[ctr], switch_count[ctr]);

	return 0;
}

static ssize_t store_st(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(cpu_switch_times, 0644, show_st, store_st);

/*
 * Use our own MPIDR accessors as the generic ones in asm/cputype.h have
 * __attribute_const__ and we don't want the compiler to assume any
 * constness here.
 */

u32 read_mpidr(void)
{
	unsigned int id;
	asm volatile ("mrc\tp15, 0, %0, c0, c0, 5" : "=r" (id));
	return id;
}

/*
 * Get a global nanosecond time stamp for tracing.
 */
static s64 get_ns(void)
{
	struct timespec ts;
	getnstimeofday(&ts);
	return timespec_to_ns(&ts);
}

/*
 * bL switcher core code.
 */

unsigned long bL_sync_phys;
struct bL_sync_struct *bL_sync;

/*
 * __bL_cpu_going_down: Indicates that the cpu is being torn down
 *    This must be called at the point of committing to teardown of a CPU.
 */
void __bL_cpu_going_down(unsigned int cpu, unsigned int cluster)
{
	writeb_relaxed(CPU_GOING_DOWN, &bL_sync->clusters[cluster].cpus[cpu]);
	dsb();
}

/*
 * __bL_cpu_down: Indicates that cpu teardown is complete and that the
 *    cluster can be torn down without disrupting this CPU.
 *    To avoid deadlocks, this must be called before a CPU is powered down.
 */
void __bL_cpu_down(unsigned int cpu, unsigned int cluster)
{
	dmb();
	writeb_relaxed(CPU_DOWN, &bL_sync->clusters[cluster].cpus[cpu]);
	plat_safe_barrier((u32 *) &bL_sync->clusters[cluster].cpus[cpu]);
	sev();
}

/*
 * __bL_outbound_leave_critical: Leave the cluster teardown critical section.
 * @state: the final state of the cluster:
 *     CLUSTER_UP: no destructive teardown was done and the cluster has been
 *         restored to the previous state; or
 *     CLUSTER_DOWN: the cluster has been torn-down, ready for power-off.
 */
void __bL_outbound_leave_critical(unsigned int cluster, int state)
{
	dmb();
	writeb_relaxed(state, &bL_sync->clusters[cluster].cluster);
	plat_safe_barrier((u32 *) &bL_sync->clusters[cluster].cluster);
	sev();
}

int cpus_per_cluster[BL_NR_CLUSTERS];
#define cpus_per_cluster(cluster) (cpus_per_cluster[cluster])

void __init __bL_set_cpus_per_cluster(int cluster, int num)
{
	BUG_ON(cluster >= BL_NR_CLUSTERS);
	if (num > num_online_cpus())
		num = num_online_cpus();
	cpus_per_cluster[cluster] = num;
}

/*
 * __bL_outbound_enter_critical: Enter the cluster teardown critical section.
 * This function should be called by the last man, after local CPU teardown
 * is complete.
 *
 * Returns:
 *     false: the critical section was not entered because an inbound CPU was
 *         observed, or the cluster is already being set up;
 *     true: the critical section was entered: it is now safe to tear down the
 *         cluster.
 */
bool __bL_outbound_enter_critical(unsigned int cpu, unsigned int cluster)
{
	unsigned int i;
	struct bL_cluster_sync_struct *c = &bL_sync->clusters[cluster];

	/* Warn inbound CPUs that the cluster is being torn down: */
	writeb_relaxed(CLUSTER_GOING_DOWN, &c->cluster);

	dsb();

	/* Back out if the inbound cluster is already in the critical region: */
	if (readb_relaxed(&c->inbound) == INBOUND_COMING_UP)
		goto abort;

	/*
	 * Wait for all CPUs to get out of the GOING_DOWN state, so that local
	 * teardown is complete on each CPU before tearing down the cluster.
	 *
	 * If any CPU has been woken up again from the DOWN state, then we
	 * shouldn't be taking the cluster down at all: abort in that case.
	 */
	for (i = 0; i < cpus_per_cluster(cluster); i++) {
		int cpustate;

		if (i == cpu)
			continue;

		while (1) {
			cpustate = readb_relaxed(&c->cpus[i]);
			if (cpustate != CPU_GOING_DOWN)
				break;

			wfe();
		}

		switch (cpustate) {
		case CPU_DOWN:
			continue;

		default:
			goto abort;
		}
	}

	dsb();

	return true;

abort:
	__bL_outbound_leave_critical(cluster, CLUSTER_UP);
	return false;
}

/*
 * bL_set_first_man(): the platform power_up() method should call this
 * to nominate a first man to set up the target cluster on migration.
 *
 * Usually, a spinlock should be held across identification of the first
 * man and the call to this function.
 */
void __bL_set_first_man(int cpu, unsigned int cluster)
{
	bL_sync->clusters[cluster].first_man = cpu;
}

const struct bL_power_ops *bL_platform_ops;

extern void setup_mm_for_reboot(void);

typedef void (*phys_reset_t)(unsigned long);

static void bL_do_switch(void *_unused)
{
	unsigned mpidr, cpuid, clusterid, ob_cluster, ib_cluster;
	phys_reset_t phys_reset;

	/*
	 * We now have a piece of stack borrowed from the init task's.
	 * Let's also switch to init_mm right away to match it and avoid
	 * any unexpected paging of the vmalloc area.
	 */
	cpu_switch_mm(init_mm.pgd, &init_mm);

	pr_debug("%s\n", __func__);

	mpidr = read_mpidr();
	cpuid = mpidr & 0xf;
	clusterid = (mpidr >> 8) & 0xf;
	ob_cluster = clusterid;
	ib_cluster = clusterid ^ 1;

	/*
	 * Our state has been saved at this point.  Let's release our
	 * inbound CPU.
	 */
	bL_set_entry_vector(cpuid, ib_cluster, cpu_resume);
	sev();

	/*
	 * From this point, we must assume that our counterpart CPU might 
	 * have taken over in its parallel world already, as if execution
	 * just returned from cpu_suspend().  It is therefore important to
	 * be very careful not to make any change the other guy is not
	 * expecting.  This is why we need stack isolation.
	 */

	bL_platform_ops->power_down(cpuid, ob_cluster);

	/*
	 * Hey, we're not dead!  This means a request to switch back
	 * has come from our counterpart and reset was deasserted before
	 * we had the chance to enter WFI.  Let's turn off the MMU and
	 * branch back directly through our kernel entry point.
	 */
	setup_mm_for_reboot();
	phys_reset = (phys_reset_t)(unsigned long)virt_to_phys(cpu_reset);
#if !defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	phys_reset(virt_to_phys(bl_entry_point));
#else
	phys_reset(0x0);
#endif

	/* should never get here */
	BUG();
}

/*
 * Stack isolation.  To ensure 'current' remains valid, we just borrow
 * a slice of the init/idle task which should be fairly lightly used.
 * The borrowed area starts just above the thread_info structure located
 * at the very bottom of the stack, aligned to a cache line.
 */
#define STACK_SIZE 256
extern void call_with_stack(void (*fn)(void *), void *arg, void *sp);
static int bL_switchpoint(unsigned long _unused)
{
	unsigned int mpidr = read_mpidr();
	unsigned int cpuid = mpidr & 0xf;
	unsigned int clusterid = (mpidr >> 8) & 0xf;
	unsigned int cpu_index = cpuid + clusterid * BL_CPUS_PER_CLUSTER;
	void *stack = &init_thread_info + 1;
	stack = PTR_ALIGN(stack, L1_CACHE_BYTES);
	stack += cpu_index * STACK_SIZE + STACK_SIZE;
	call_with_stack(bL_do_switch, NULL, stack);
	BUG();
}

/*
 * Generic switcher interface
 */

static unsigned int bL_gic_id[BL_CPUS_PER_CLUSTER][BL_NR_CLUSTERS];

/*
 * bL_switch_to - Switch to a specific cluster for the current CPU
 * @new_cluster_id: the ID of the cluster to switch to.
 *
 * This function must be called on the CPU to be switched.
 * Returns 0 on success, else a negative status code.
 */
int bL_switch_to(unsigned int new_cluster_id)
{
	unsigned int mpidr, cpuid, clusterid, ob_cluster, ib_cluster, this_cpu;
	struct tick_device *tdev;
	enum clock_event_mode tdev_mode;
	int ret = 0;
	unsigned long flags;
	unsigned long long start, end;

	local_irq_save(flags);
	local_fiq_disable();

	start = read_cntpct();
	mpidr = read_mpidr();
	cpuid = mpidr & 0xf;
	clusterid = (mpidr >> 8) & 0xf;
	ob_cluster = clusterid;
	ib_cluster = clusterid ^ 1;

	if (new_cluster_id == clusterid)
		goto out;

	if (!bL_platform_ops) {
		ret = -ENOSYS;
		goto out;
	}

	trace_cpu_migrate_begin(get_ns(), cpuid, ob_cluster, cpuid, ib_cluster);

	pr_debug("before switch: CPU %d in cluster %d\n", cpuid, clusterid);

	/* Close the gate for our entry vectors */
	bL_set_entry_vector(cpuid, ob_cluster, NULL);
	bL_set_entry_vector(cpuid, ib_cluster, NULL);

	/*
	 * Update the cpu logical map before releasing the inbound.
	 */
	cpu_logical_map(cpuid) = (ib_cluster << 8) | cpuid;
	__cpuc_flush_dcache_area(&cpu_logical_map(cpuid),
				 sizeof(cpu_logical_map(cpuid)));

	/*
	 * Let's wake up the inbound CPU now in case it requires some delay
	 * to come online, but leave it gated in our entry vector code.
	 */
	bL_platform_ops->power_up(cpuid, ib_cluster,
					virt_to_phys(bl_entry_point));

	/* redirect GIC's SGIs to our counterpart */
#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	gic_migrate_target(cpuid + ib_cluster * cpus_per_cluster(ib_cluster));
#else
	gic_migrate_target(bL_gic_id[cpuid][ib_cluster]);
#endif

	/*
	 * Raise a SGI on the inbound CPU to make sure it doesn't stall
	 * in a possible WFI, such as the one in bL_do_switch().
	 */
	arm_send_ping_ipi(smp_processor_id());

	this_cpu = smp_processor_id();
	tdev = tick_get_device(this_cpu);
	if (tdev && !cpumask_equal(tdev->evtdev->cpumask, cpumask_of(this_cpu)))
		tdev = NULL;
	if (tdev) {
		tdev_mode = tdev->evtdev->mode;
		clockevents_set_mode(tdev->evtdev, CLOCK_EVT_MODE_SHUTDOWN);
	}

	ret = cpu_pm_enter(1);
	if (ret)
		goto out;

	/* Let's do the actual CPU switch. */
	ret = cpu_suspend((unsigned long)NULL, bL_switchpoint);
	if (ret > 0)
		ret = -EINVAL;

	/* We are executing on the inbound CPU at this point */
	mpidr = read_mpidr();
	cpuid = mpidr & 0xf;
	clusterid = (mpidr >> 8) & 0xf;
	pr_debug("after switch: CPU %d in cluster %d\n", cpuid, clusterid);
	BUG_ON(clusterid != ib_cluster);

	ret = cpu_pm_exit();

	if (tdev) {
		clockevents_set_mode(tdev->evtdev, tdev_mode);
		clockevents_program_event(tdev->evtdev,
					  tdev->evtdev->next_event, 1);
	}

	bL_platform_ops->power_up_finish(cpuid, ib_cluster);

	trace_cpu_migrate_finish(get_ns(), cpuid, ob_cluster, cpuid, ib_cluster);

out:
	if (ret)
		pr_err("%s exiting with error %d\n", __func__, ret);

	end = read_cntpct();
	switch_time[cpuid] += end - start;
	switch_count[cpuid]++;

	local_fiq_enable();
	local_irq_restore(flags);

	return ret;
}

EXPORT_SYMBOL_GPL(bL_switch_to);

struct switch_args {
	unsigned int cluster;
	struct work_struct work;
};

static void __bL_switch_to(struct work_struct *work)
{
	struct switch_args *args = container_of(work, struct switch_args, work);
	bL_switch_to(args->cluster);
}

/*
 * bL_switch_request - Switch to a specific cluster for the given CPU
 *
 * @cpu: the CPU to switch
 * @new_cluster_id: the ID of the cluster to switch to.
 *
 * This function causes a cluster switch on the given CPU.  If the given
 * CPU is the same as the calling CPU then the switch happens right away.
 * Otherwise the request is put on a work queue to be scheduled on the
 * remote CPU.
 */
void bL_switch_request(unsigned int cpu, unsigned int new_cluster_id)
{
	unsigned int this_cpu = get_cpu();
	struct switch_args args;

	if (cpu == this_cpu) {
		bL_switch_to(new_cluster_id);
		put_cpu();
		return;
	}
	put_cpu();

	args.cluster = new_cluster_id;
	INIT_WORK_ONSTACK(&args.work, __bL_switch_to);
	schedule_work_on(cpu, &args.work);
	flush_work(&args.work);
}

EXPORT_SYMBOL_GPL(bL_switch_request);

/*
 * Code to enumerate GIC interface IDs for all CPUs
 */

static void __init bL_enumerate_gic_cpu_id(struct work_struct *work)
{
	unsigned int mpidr, cpu, cluster;
	int ret = 0;

	local_irq_disable();
	local_fiq_disable();

	mpidr = read_mpidr();
	cpu = mpidr & 0xf;
	cluster = (mpidr >> 8) & 0xf;

	/* Get current cpu's GIC ID */
	bL_gic_id[cpu][cluster] = gic_get_cpu_id();
	pr_info("GIC ID for CPU %u cluster %u is %u\n", cpu, cluster, bL_gic_id[cpu][cluster]);

	/* Flip to the other cluster */
	cluster ^= 1;
	bL_set_entry_vector(cpu, cluster, NULL);

#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	/*
	 * Update the cpu logical map before releasing the inbound.
	 */
	cpu_logical_map(cpu) = (cluster << 8) | cpu;
	__cpuc_flush_dcache_area(&cpu_logical_map(cpu),
				 sizeof(cpu_logical_map(cpu)));
#endif

	bL_platform_ops->power_up(cpu, cluster, virt_to_phys(bl_entry_point));
#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	gic_broadcast_softirq(0x1);
#endif

	ret = cpu_suspend(0, bL_switchpoint);
	if (ret)
		BUG();

	/*
	 * Now in the alternate cluster.
	 * Get current cpu's GIC ID.
	 */
	bL_gic_id[cpu][cluster] = gic_get_cpu_id();
	pr_info("GIC ID for CPU %u cluster %u is %u\n", cpu, cluster, bL_gic_id[cpu][cluster]);

#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	/* Enable interrupts to drain any spurious ones */
	local_fiq_enable();
	local_irq_enable();

	/* Disable interrupts */
	local_irq_disable();
	local_fiq_disable();
#endif

	/* Flip back to the original cluster */
	cluster ^= 1;
	bL_set_entry_vector(cpu, cluster, NULL);

#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	/*
	 * Update the cpu logical map before releasing the inbound.
	 */
	cpu_logical_map(cpu) = (cluster << 8) | cpu;
	__cpuc_flush_dcache_area(&cpu_logical_map(cpu),
				 sizeof(cpu_logical_map(cpu)));
#endif

	bL_platform_ops->power_up(cpu, cluster, virt_to_phys(bl_entry_point));
	/* no GIC migration occurred, so this goes to the initial CPU */
	arm_send_ping_ipi(smp_processor_id());
	ret = cpu_suspend(0, bL_switchpoint);
	if (ret)
		BUG();

	/* Back in the original cluster */
	local_fiq_enable();
	local_irq_enable();
}

#ifdef CONFIG_BL_SWITCHER_DUMMY_IF

/*
 * Dummy interface to user space (to be replaced by cpufreq based interface).
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

static ssize_t bL_switcher_write(struct file *file, const char __user *buf,
			size_t len, loff_t *pos)
{
	unsigned char val[3];
	unsigned int cpu, cluster;

	pr_debug("%s\n", __func__);

	if (len < 3)
		return -EINVAL;

	if (copy_from_user(val, buf, 3))
		return -EFAULT;

	/* format: <cpu#>,<cluster#> */
	if (val[0] < '0' || val[0] > '4' ||
	    val[1] != ',' ||
	    val[2] < '0' || val[2] > '1')
		return -EINVAL;

	cpu = val[0] - '0';
	cluster = val[2] - '0';
	bL_switch_request(cpu, cluster);

	return len;
}

static const struct file_operations bL_switcher_fops = {
	.write		= bL_switcher_write,
	.owner	= THIS_MODULE,
};

static struct miscdevice bL_switcher_device = {
        MISC_DYNAMIC_MINOR,
        "b.L_switcher",
        &bL_switcher_fops
};

#endif

extern unsigned long bL_power_up_setup_phys;

int __init bL_switcher_reserve(void)
{
	bL_sync_phys =
		arm_memblock_steal(BL_SYNC_MEM_RESERVE, (1 << 21));

	return 0;
}

static struct resource bL_iomem_resource = {
	.name = "big.LITTLE switcher synchronisation buffer",
	.flags = IORESOURCE_MEM|IORESOURCE_EXCLUSIVE|IORESOURCE_BUSY,
};

int __init bL_switcher_init(const struct bL_power_ops *ops)
{
	unsigned int i, this_cluster;

	pr_info("big.LITTLE switcher initializing\n");

	/*
	 * It is too late to steal physical memory here.
	 * Boards must pre-reserve synchronisation memory by calling
	 * bL_switcher_reserve() from their machine_desc .reserve hook.
	 */
	BUG_ON(bL_sync_phys == 0);

	bL_sync = ioremap(bL_sync_phys, BL_SYNC_MEM_RESERVE);
	if(!bL_sync) {
		pr_err("big.LITTLE switcher synchronisation buffer mapping failed\nm");
		return -ENOMEM;
	}

	bL_iomem_resource.start = bL_sync_phys;
	bL_iomem_resource.end = bL_sync_phys + BL_SYNC_MEM_RESERVE - 1;
	insert_resource(&iomem_resource, &bL_iomem_resource);

	/*
	 * Set initial CPU and cluster states.
	 * Only one cluster is assumed to be active at this point.
	 */
	memset(bL_sync, 0, sizeof *bL_sync);
#if 1
	this_cluster = (read_mpidr() >> 8)  & 0xf;
	for_each_online_cpu(i)
		bL_sync->clusters[this_cluster].cpus[i] = CPU_UP;
	bL_sync->clusters[this_cluster].cluster = CLUSTER_UP;
	bL_sync->clusters[this_cluster].first_man = FIRST_MAN_NONE;
#else
	for (this_cluster = 0; this_cluster < BL_NR_CLUSTERS; this_cluster++) {
		for(i = 0; i < BL_CPUS_PER_CLUSTER; i++)
			bL_sync->clusters[this_cluster].cpus[i] = CPU_UP;
		bL_sync->clusters[this_cluster].cluster = CLUSTER_UP;
		bL_sync->clusters[this_cluster].first_man = FIRST_MAN_NONE;
	}
#endif

	bL_platform_ops = ops;
	if (ops->power_up_setup) {
		bL_power_up_setup_phys =
			virt_to_phys(ops->power_up_setup);
		__cpuc_flush_dcache_area((void *)&bL_power_up_setup_phys,
						sizeof bL_power_up_setup_phys);
		outer_clean_range(__pa(&bL_power_up_setup_phys),
				  __pa(&bL_power_up_setup_phys + 1));
	}

	__cpuc_flush_dcache_area((void *)&bL_sync_phys,
					sizeof bL_sync_phys);
	outer_clean_range(__pa(&bL_sync_phys), __pa(&bL_sync_phys + 1));

#if !defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	schedule_on_each_cpu(bL_enumerate_gic_cpu_id);
#endif

#ifdef CONFIG_BL_SWITCHER_DUMMY_IF
	misc_register(&bL_switcher_device);
#endif

	memset(switch_count, 0, sizeof switch_count);
	memset(switch_time, 0, sizeof switch_time);
	device_create_file(bL_switcher_device.this_device,
			   &dev_attr_cpu_switch_times);
	pr_info("big.LITTLE switcher initialized\n");
	return 0;
}

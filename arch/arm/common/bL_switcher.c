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

#include <asm/suspend.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/bL_switcher.h>
#include <asm/bL_entry.h>

#define CREATE_TRACE_POINTS
#include <trace/events/power_cpu_migrate.h>


/*
 * Use our own MPIDR accessors as the generic ones in asm/cputype.h have
 * __attribute_const__ and we don't want the compiler to assume any
 * constness here.
 */

static int read_mpidr(void)
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

const struct bL_power_ops *bL_platform_ops;

extern void setup_mm_for_reboot(void);

typedef void (*phys_reset_t)(unsigned long);

static void bL_do_switch(void *_unused)
{
	unsigned mpidr, cpuid, clusterid, ob_cluster, ib_cluster;
	bool last_man;
	phys_reset_t phys_reset;
	
	pr_debug("%s\n", __func__);

	mpidr = read_mpidr();
	cpuid = mpidr & 0xf;
	clusterid = (mpidr >> 8) & 0xf;
	ob_cluster = clusterid;
	ib_cluster = clusterid ^ 1;

	/*
	 * Let's signal our intention to go down before letting the
	 * inbound CPU run.  This must be done here in case the inbound
	 * decides to switch back before we actually get to shut ourself
	 * down.
	 */
	last_man = bL_platform_ops->power_down(cpuid, ob_cluster);
       
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
	 *
	 * Also, because of this special stack, we cannot rely on anything
	 * that expects a valid 'current' pointer.  For example, printk()
	 * may give bogus "BUG: recent printk recursion!\n" messages
	 * because of that.
	 */

	/*
	 * Now let's take care of cleaning our cache and
	 * shutting ourself down.  If we're the last CPU in this cluster,
	 * clean L2 too.
	 */
	cpu_proc_fin();
	if (last_man) {
		flush_cache_all();
		outer_flush_all();
	} else {
		flush_dcache_level(flush_cache_level_cpu());
	}
	asm volatile (
		"mrc	p15, 0, ip, c1, c0, 1 \n\t"
		"bic	ip, ip, #(1 << 6) @ clear SMP bit \n\t"
		"mcr	p15, 0, ip, c1, c0, 1"
		: : : "ip" );

	/* And our own life ends right here... */
	wfi();

	/*
	 * Hey, we're not dead!  This means a request to switch back
	 * has come from our counterpart and reset was deasserted before
	 * we had the chance to enter WFI.  Let's turn off the MMU and
	 * branch back directly through our kernel entry point.
	 */
	setup_mm_for_reboot();
	phys_reset = (phys_reset_t)(unsigned long)virt_to_phys(cpu_reset);
	phys_reset(virt_to_phys(bl_entry_point));

	/* should never get here */
	BUG();
}

/*
 * Stack isolation (size needs to be optimized)
 */

static unsigned long __attribute__((__aligned__(L1_CACHE_BYTES)))
	stacks[BL_CPUS_PER_CLUSTER][BL_NR_CLUSTERS][128];

extern void call_with_stack(void (*fn)(void *), void *arg, void *sp);

static int bL_switchpoint(unsigned long _unused)
{
	unsigned int mpidr = read_mpidr();
	unsigned int cpuid = mpidr & 0xf;
	unsigned int clusterid = (mpidr >> 8) & 0xf;
	void *stack = stacks[cpuid][clusterid] + ARRAY_SIZE(stacks[0][0]);
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

	local_irq_save(flags);
	local_fiq_disable();

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
	 * Let's wake up the inbound CPU now in case it requires some delay
	 * to come online, but leave it gated in our entry vector code.
	 */
	bL_platform_ops->power_up(cpuid, ib_cluster);

	/* redirect GIC's SGIs to our counterpart */
	gic_migrate_target(bL_gic_id[cpuid][ib_cluster]);

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

	ret = cpu_pm_enter();
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

	trace_cpu_migrate_finish(get_ns(), cpuid, ob_cluster, cpuid, ib_cluster);

out:
	if (ret)
		pr_err("%s exiting with error %d\n", __func__, ret);
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
	bL_platform_ops->power_up(cpu, cluster);
	ret = cpu_suspend(0, bL_switchpoint);
	if (ret)
		BUG();

	/*
	 * Now in the alternate cluster.
	 * Get current cpu's GIC ID.
	 */
	bL_gic_id[cpu][cluster] = gic_get_cpu_id();
	pr_info("GIC ID for CPU %u cluster %u is %u\n", cpu, cluster, bL_gic_id[cpu][cluster]);

	/* Flip back to the original cluster */
	cluster ^= 1;
	bL_set_entry_vector(cpu, cluster, NULL);
	bL_platform_ops->power_up(cpu, cluster);
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

int __init bL_switcher_init(const struct bL_power_ops *ops)
{
	pr_info("big.LITTLE switcher initializing\n");
	bL_platform_ops = ops;
	schedule_on_each_cpu(bL_enumerate_gic_cpu_id);
#ifdef CONFIG_BL_SWITCHER_DUMMY_IF
	misc_register(&bL_switcher_device);
#endif
	pr_info("big.LITTLE switcher initialized\n");
	return 0;
}

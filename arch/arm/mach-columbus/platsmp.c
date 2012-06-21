/*
 *  Columbus Compute Subsystem support
 *
 *  Copyright (C) 2011 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>

#include <mach/columbus.h>
#include <mach/spc.h>

extern void columbus_boot_secondary(void);

static struct map_desc columbus_release_addr_map __initdata = {
	.virtual	= COLUMBUS_SYS_FLAGS_VIRT_BASE,
	/* .pfn is set dynamically */
	.length		= SZ_128,
	.type		= MT_DEVICE,
};

static void *cpu_release_addr_table[CONFIG_NR_CPUS] __initdata;


#define SPIN_TABLE_STRLEN	10

static int __init columbus_find_release_addr(unsigned long node, 
		const char *uname, int depth, void *data)
{
	int cpu_count = 0;
	phys_addr_t cpu_release_addr;

	if (strncmp(uname, "cpu", 3) == 0) {
		unsigned long len;
		const u64 *cpu_release_addr_prop;
		const void *enable_method_prop = of_get_flat_dt_prop(node, "enable-method", &len);
		if (enable_method_prop != NULL && len == SPIN_TABLE_STRLEN &&
			!strncmp(enable_method_prop, "spin-table", SPIN_TABLE_STRLEN)) {
			cpu_release_addr_prop = of_get_flat_dt_prop(node, "cpu-release-addr", NULL);
			if (!cpu_release_addr_prop) {
				printk(KERN_ERR "cpu-release-addr not found!");
				return -EFAULT;
			}

			cpu_release_addr = be64_to_cpup(cpu_release_addr_prop);

			columbus_release_addr_map.pfn = __phys_to_pfn(cpu_release_addr);
			iotable_init(&columbus_release_addr_map, 1);
			cpu_release_addr_table[cpu_count] = ioremap(cpu_release_addr, SZ_128);
			if (!cpu_release_addr_table[cpu_count]) {
				printk(KERN_ERR "Failed to map cpu release address for CPU %d\n", cpu_count);
				while (--cpu_count >= 0) {
					iounmap(cpu_release_addr_table[cpu_count]);
				}
				return -EFAULT;
			}
			cpu_count++;
		}
	}

	return 0;
}


void __init columbus_smp_map_io(void)
{
	int i, found = 0;

	WARN_ON(of_scan_flat_dt(columbus_find_release_addr, NULL));
#if defined(CONFIG_COLUMBUS_MODEL) || defined(CONFIG_COLUMBUS_TC2)
	/*
	 * if no support in device tree for cpu-release-addr then
	 * fall back to the legacy address for pen release
	 */
	for (i = 0; i < CONFIG_NR_CPUS; i++) {
		if (cpu_release_addr_table[i]) {
			found = 1;
			break;
		}
	}

	if (!found) {
		void *legacy_cpu_release_addr;
		printk(KERN_INFO "Columbus: cpu-release-addr not found, "
			"using legacy address\n");
		columbus_release_addr_map.pfn = __phys_to_pfn(COLUMBUS_SYS_FLAGS_PHYS_BASE);
		iotable_init(&columbus_release_addr_map, 1);
		legacy_cpu_release_addr = ioremap(COLUMBUS_SYS_FLAGS_PHYS_BASE, SZ_128);

		if (!legacy_cpu_release_addr) {
			printk(KERN_ERR "Failed to map legacy cpu-release-addr address\n");
			return;
		}

		writel(~0, legacy_cpu_release_addr + COLUMBUS_SYS_FLAGS_CLR_OFFSET);
		writel(virt_to_phys(columbus_boot_secondary), legacy_cpu_release_addr +	COLUMBUS_SYS_FLAGS_SET_OFFSET);
	}
#endif
}

static int __init columbus_get_cpus_num(unsigned long node, const char *uname,
			int depth, void *data)
{
	static int prev_depth = -1;
	static int nr_cpus = -1;

	if (prev_depth > depth && nr_cpus > 0)
		return nr_cpus;

	if (nr_cpus < 0 && strcmp(uname, "cpus") == 0)
		nr_cpus = 0;

	if (nr_cpus >= 0) {
		const char *device_type = of_get_flat_dt_prop(node,
				"device_type", NULL);

		if (device_type && strcmp(device_type, "cpu") == 0)
			nr_cpus++;
	}

	prev_depth = depth;

	return 0;
}

void __init smp_init_cpus(void)
{
	int i, ncores = 0;

	ncores = of_scan_flat_dt(columbus_get_cpus_num, NULL);

	if (ncores < 2)
		return;

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	for (i = 0; i < max_cpus; i++) {
		if (cpu_release_addr_table[i]) {
			writel(virt_to_phys(columbus_boot_secondary),
				cpu_release_addr_table[i]);
		}
		/* 
		 * if we don't have a cpu release address we
		 * might be using the legacy address, so enable
		 * the cpu anyway.
		 */
		set_cpu_present(i, true);
	}

	flush_cache_all();
}

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
volatile int __cpuinitdata pen_release = -1;

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void __cpuinit write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	unsigned int mpidr = cpu_logical_map(cpu);

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	writel(~0, COLUMBUS_SYS_FLAGS_VIRT_BASE + COLUMBUS_SYS_FLAGS_CLR_OFFSET);
	writel(virt_to_phys(columbus_boot_secondary), COLUMBUS_SYS_FLAGS_VIRT_BASE + COLUMBUS_SYS_FLAGS_SET_OFFSET);
	/*
	 * This is really belt and braces; we hold unintended secondary
	 * CPUs in the holding pen until we're ready for them.  However,
	 * since we haven't sent them a soft interrupt, they shouldn't
	 * be there.
	 */
	spc_wfi_cpureset((mpidr & 0xff00) >> 8, mpidr & 0xff, 0);
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send the secondary CPU a soft interrupt, thereby causing
	 * the boot monitor to read the system wide flags register,
	 * and branch to the address found there.
	 */
	udelay(100);
	gic_raise_softirq(cpumask_of(cpu), 1);

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

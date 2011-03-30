/*
 * Copyright (C) 2008-2010 ARM Limited
 *
 * This software is provided 'as-is', without any express or implied
 * warranties including the implied warranties of satisfactory quality,
 * fitness for purpose or non infringement.  In no event will  ARM be
 * liable for any damages arising from the use of this software.
 *
 * Permission is granted to anyone to use, copy and modify this software for
 * any purpose, and to redistribute the software, subject to the following
 * restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 */

#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/cache.h>

#include <asm/cacheflush.h>
#include <asm/system.h>
#include <asm/lb_lock.h>
#include <asm/appf_platform_api.h>
#include "appf_internals.h"
#include "appf_helpers.h"
#include "appf_defs.h"


struct ____cacheline_aligned appf_main_table main_table;
static DECLARE_BITMAP(cpu_pm_bits, CONFIG_NR_CPUS) __read_mostly
    = CPU_BITS_NONE;
const struct cpumask *const cpu_pm_mask = to_cpumask(cpu_pm_bits);
unsigned int enable_sr;

unsigned *appf_device_memory;
unsigned appf_device_memory_flat_mapped;
DEFINE_PER_CPU(u8, cluster_down) = 0;
DEFINE_SPINLOCK(sr_lock);
static int late_init(void);

int is_smp = FALSE;

int appf_runtime_init(void)
{
	int ret;
	void *va_cached;

	appf_device_memory =
	    dma_alloc_coherent(NULL, PAGE_SIZE,
			       &appf_device_memory_flat_mapped,
			       GFP_KERNEL);
	context_memory = kzalloc(CONTEXT_SPACE, GFP_KERNEL);
	va_cached =
	    dma_alloc_coherent(NULL, CONTEXT_SPACE_UNCACHED,
			       (dma_addr_t *) &context_memory_uncached,
			       GFP_KERNEL);

	ret = linux_appf_setup_translation_tables();

	if (ret < 0)
		return ret;

	return 0;
}

static int late_init(void)
{
	struct appf_cluster *cluster;
	int cluster_index;
	int rc;

	cluster_index = appf_platform_get_cluster_index();
	cluster = main_table.cluster_table + cluster_index;

	/*
	 * Clean the translation tables out of the L1 dcache
	 * (see comments in disable_clean_inv_dcache_v7_l1)
	 */
	dsb();
	__cpuc_flush_kern_all();
	outer_flush_all();
	switch_page_tables(main_table.fw_mmu_context,
			   main_table.os_mmu_context);
	rc = appf_platform_init();
	restore_page_tables(main_table.os_mmu_context);
	return rc;
}

int power_down_cpu(unsigned cstate, unsigned rstate, unsigned flags)
{
	struct appf_cpu *cpu;
	struct appf_cluster *cluster;
	int cpu_index, cluster_index;
	int i, cluster_can_enter_cstate1, rc = 0;
	unsigned long irqflags;

	cpu_index = smp_processor_id();

	if (!cpu_isset(cpu_index, *cpu_pm_mask)
	    || (cstate == 1 && !rstate)) {
		__asm__ __volatile__("dsb\n\t" "wfi\n\t" : : : "memory");
		return 0;
	}

	switch_page_tables(main_table.fw_mmu_context,
			   main_table.os_mmu_context);
	cpu_index = appf_platform_get_cpu_index();
	cluster_index = appf_platform_get_cluster_index();

	cluster = main_table.cluster_table + cluster_index;
	cpu = cluster->cpu_table + cpu_index;

	/* Validate arguments */
	if (cstate > 3)
		return -EINVAL;

	if (rstate > 3)
		return -EINVAL;

	/*
	 * Ok, we're not just entering standby, so we are going to lose the
	 * context on this CPU x
	 */

	spin_lock_irqsave(&sr_lock, irqflags);

	if (cstate > 1) {
		--cluster->active_cpus;
		if (cluster->active_cpus == 0)
			cluster->power_state = rstate;
		cpu->cluster_down =
		    (cluster->active_cpus == 0 && rstate >= 2);
	}
	cpu->power_state = cstate;

	if (cstate == 1 && rstate == 1) {
		cluster_can_enter_cstate1 = TRUE;
		for (i = 0; i < cluster->num_cpus; ++i) {
			if (cluster->cpu_table[i].power_state != 1) {
				cluster_can_enter_cstate1 = FALSE;
				break;
			}
		}
		if (cluster_can_enter_cstate1)
			cluster->power_state = 1;
	}
	spin_unlock_irqrestore(&sr_lock, irqflags);

	if (cstate > 1) {
#ifdef SECURE_ONLY
		rc = appf_save_flat_call(cluster, cpu, flags, TRUE);
#elif NONSECURE_ONLY
		appf_platform_save_context(cluster, cpu, flags, FALSE);
#else
		/*
		 * Note that in a real TrustZone system it is a VERY BAD IDEA
		 * to share code between the Secure and Non-secure worlds.
		 * If the code was compromised in
		 * the Non-secure world, this would lead to a security breach.
		 * A production implementation of context save/restore would
		 * not return from enter_secure_monitor_mode() and all the
		 * code to save the secure context
		 * (and sanction the power down) would be held in Secure memory.
		 */
		appf_platform_save_context(cluster, cpu, flags, FALSE);
		enter_secure_monitor_mode();
		appf_platform_save_context(cluster, cpu, flags, TRUE);
#endif
	}
	/* Did the power down succeed? */
	if (!rc) {

		if (cstate == 2) {
			if (cpu->cluster_down)
				cluster->cluster_down = 1;
			appf_sleep((void *) cpu->ic_address);
		} else {
			__asm__ __volatile__("dsb\n\t"
					     "wfi\n\t" : : : "memory");
		}

		if (cstate > 1) {
#ifdef SECURE_ONLY
			appf_restore_flat_call(cluster, cpu, TRUE);
#elif NONSECURE_ONLY
			appf_platform_restore_context(cluster, cpu, FALSE);
#else
			appf_platform_restore_context(cluster, cpu, TRUE);
			enter_nonsecure_svc_mode();
			appf_platform_restore_context(cluster, cpu, FALSE);
#endif
		}
		rc = appf_platform_leave_cstate(cpu_index, cpu, cluster);
		cpu->power_state = 0;
		cluster->power_state = 0;
	}
	restore_page_tables(main_table.os_mmu_context);

	return rc;
}

int power_up_cpu(unsigned int cluster_index, unsigned int cpu)
{
	struct appf_cluster *cluster;
	struct appf_cpu *cpup;
	int ret = 0;

	switch_page_tables(main_table.fw_mmu_context,
			   main_table.os_mmu_context);

	if (cluster_index >= main_table.num_clusters)
		return -EINVAL;

	cluster = main_table.cluster_table + cluster_index;
	cpup = cluster->cpu_table + cpu;

	if (cpu > num_online_cpus())
		return -EINVAL;

	/*
	 * TODO: add cluster-waking code for multi-cluster systems
	 * TODO: locks will have to be expanded once extra-cluster CPUs
	 * can contend for them
	 */

	ret = appf_platform_power_up_cpu(cpup, cluster, cpu);

	restore_page_tables(main_table.os_mmu_context);
	return ret;
}

static int sr_debug_get(void *data, u64 * val)
{
	int *option = data;

	if (!option) {
		pr_warning("Wrong parameter passed\n");
		return -EINVAL;
	}
	*val = *option;
	return 0;
}

static int sr_debug_set(void *data, u64 val)
{
	int *option = data;
	int i, cpus = num_possible_cpus();

	if (!option) {
		pr_warning("Wrong parameter passed\n");
		return -EINVAL;
	}

	if (val > cpus || val < 0) {
		pr_warning("Wrong parameter passed\n");
		return -EINVAL;
	}
	for (i = 0; i < cpus; i++)
		if (val & (1 << i))
			cpumask_set_cpu(i, to_cpumask(cpu_pm_bits));
		else
			cpumask_clear_cpu(i, to_cpumask(cpu_pm_bits));

	*option = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sr_debug_fops, sr_debug_get, sr_debug_set,
			"%llu\n");

int appf_init(void)
{
	struct dentry *sr_debug;

	char name[16];
	strcpy(name, "sr_debug");

	sr_debug = debugfs_create_dir(name, NULL);
	(void) debugfs_create_file("enablesr", S_IRUGO | S_IWUGO,
				   sr_debug, &enable_sr, &sr_debug_fops);
	(void) debugfs_create_u32("rtag", S_IRUGO | S_IWUGO,
				  sr_debug, __va(RESET_TAG));

	lookup_arch();
	appf_save_flat_mapped = (unsigned long) arch->save_context;
	appf_restore_flat_mapped = __pa(arch->restore_context);

	appf_runtime_init();
	late_init();

	return 0;
}

device_initcall(appf_init);

/*
 * arch/arm/mach-vexpress/bLiks-tc2.c - TC2 specific bL task migration support
 *
 * Created by:	Achin Gupta, July 2012
 * Copyright:	(C) 2012  ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TODO:
 *
 * - The CCI interface for each cluster brought up or down should be
 *   adjusted accordingly instead of having the bootloader enable everything.
 * - The spinlock should be moved to non cached memory to improve concurrency
 *   i.e. make this code usable even when d-cache is off.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/vexpress.h>
#include <linux/arm-cci.h>

#include <asm/bL_switcher.h>
#include <asm/bL_entry.h>
#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/spinlock.h>
#include <asm/suspend.h>
#include <asm/cache.h>

#include <mach/bLiks-tc2.h>

extern void setup_mm_for_reboot(void);
extern void disable_clean_inv_dcache(int);

/*
 * Lock for electing the last and first cpus in a cluster.
 */
static DEFINE_RAW_SPINLOCK(bLiks_lock);
static u32 cpu_online_map[BL_NR_CLUSTERS];

/*
 * bLiks_power_up - make given CPU in given cluster runable
 *
 * @cpu: CPU number within given cluster
 * @cluster: cluster number for the CPU
 *
 * The identified CPU is brought out of reset.  If the cluster was powered
 * down then it is brought up as well, taking care not to let the other CPUs
 * in the cluster run, and ensuring appropriate cluster setup.
 * Caller must ensure the appropriate entry vector is initialized prior to
 * calling this.
 */
static void bLiks_power_up(unsigned int cpu, unsigned int cluster)
{
	u32 ret = 0, rsthold = 0, first_man = 0, ctr;

	raw_spin_lock(&bLiks_lock);

	/*
	 * Find out if this cpu's counterpart will be the first
	 * one in the inbound cluster and consequently have to
	 * power up the cluster first. Also, write the IKS entry
	 * point to the per-cpu mailbox.
	 */
	if (cpu_online_map[cluster] == 0)
		first_man = 1;


	/*
	 * If only cpu in cluster ensure that:
	 * 1. cluster has powered down
	 * 2. all inbound cpus have their warm reset vectors unset
	 *    so that they can enter wfi in the boot firmware
	 */
	if (first_man) {

		__bL_set_first_man(cpu, cluster);

		while (vexpress_scc_read_rststat(cluster));

		/*
		 * TODO:
		 * The map should be updated outside this 'if' condition.
		 * It prevents the outbound from doing uneccessary last
		 * man operations when we want to switch immediately.
		 * However, there is no way of knowing whether the last
		 * man aborted or completed his stuff. If he aborted then
		 * we should treat him as a 'non' last man waiting for him
		 * to be placed in reset (see above) would cause a deadlock.
		 */
		cpu_online_map[cluster] |= 1 << cpu;

		for (ctr = 0; ctr < BL_CPUS_PER_CLUSTER; ctr++) {
			if (ctr == cpu)
				vexpress_spc_write_bxaddr_reg(cluster,
					        cpu,
						virt_to_phys(bl_entry_point));
			else
				vexpress_spc_write_bxaddr_reg(cluster,
							      ctr,
							      0x0);
		}

		/*
		 * Dis-allow the cluster to power down when all cores are
		 * idling & set the OPP for the inbound cluster.
		 */
		vexpress_spc_powerdown_enable(cluster, 0);
		do {
			ret = vexpress_spc_set_performance(cluster, 5);
#if 1
			ret = 0;
#endif
		} while (ret < 0);

		ret = 1;

	} else {
		/*
		 * The inbound cpu is not the first cpu in the cluster. It was
		 * placed in wfi(). So place it in reset and then release it
		 * so that things work on the TC2. But first set its warm reset
		 * vector.
		 */
		while (!(vexpress_spc_standbywfi_status(cluster, cpu))) ;

		rsthold = vexpress_spc_read_rsthold_reg(cluster);
		rsthold |= 1 << cpu;
		vexpress_spc_write_rsthold_reg(cluster, rsthold);

		while (!
		       ((vexpress_spc_read_rststat_reg(cluster)) &
			(1 << cpu))) ;

		/*
		 * TODO:
		 * Moved here due to previous todo.
		 */
		cpu_online_map[cluster] |= 1 << cpu;

		/*
		 * Write the entry vector only when the inbound has entered wfi.
		 * Doing this outside the 'if' condition, can result in a race
		 * where the inbound will escape out of the pen in boot firmware
		 * & wfe in 'bL_entry_point' preventing it from being reset.
		 */
		vexpress_spc_write_bxaddr_reg(cluster,
					      cpu,
					      virt_to_phys(bl_entry_point));


		rsthold = vexpress_spc_read_rsthold_reg(cluster);
		rsthold &= ~(1 << cpu);
		vexpress_spc_write_rsthold_reg(cluster, rsthold);
	}

	raw_spin_unlock(&bLiks_lock);
	return;
}

/*
 * bLiks_power_down_prepare - nominate a CPU for power-down
 *
 * @cpu: CPU number within given cluster
 * @cluster: cluster number for the CPU
 *
 * @return: true if the CPU is elected as the last man
 *
 * The identified CPU is selected for powerdown.  If all other CPUs in
 * this cluster have already been selected for powerdown, then the
 * cluster is selected for powerdown and this CPU is elected as
 * responsible for tearing down the cluster (the "last man" role).
 *
 * The MMU is expected still to be on when this function is called, and
 * the CPU fully coherent.
 */
static bool bLiks_power_down_prepare(unsigned int cpu, unsigned int cluster)
{
	bool ret = false;

	raw_spin_lock(&bLiks_lock);

	cpu_online_map[cluster] &= ~(1 << cpu);

	/* if only cpu in cluster */
	if (cpu_online_map[cluster] == 0)
		ret = true;

	raw_spin_unlock(&bLiks_lock);

	return ret;
}

/*
 * Helper to determine whether the specified cluster should still be
 * shut down.  By polling this before shutting a cluster down, we can
 * reduce the probability of wasted cache flushing etc.
 */
static bool powerdown_needed(unsigned int cluster)
{
	return !cpu_online_map[cluster];
}

/*
 * bLiks_power_down - power a nominated CPU down
 *
 * @cpu: CPU number within given cluster
 * @cluster: cluster number for the CPU
 * @last_man: true if the CPU has been elected to tear down the cluster
 *
 * The identified CPU, which must previously have been nominated by
 * calling bLiks_power_down_prepare(), is powered down.
 *
 * If this CPU was elected as the last man in the cluster (last_man is
 * true), then the cluster is prepared for power-down too, unless
 * another inbound CPU appeared on the cluster in the meantime.
 *
 * The critical section protects us from the late arrival of an inbound
 * CPU: if an inbound CPU appears on the cluster within this region,
 * it must wait for us to finish before attempting to enter coherency
 * at the cluster level.
 *
 * This function may return, if wfi() is preempted by a hardware wake-up
 * event before the CPU gets physically powered down.  Otherwise, the
 * CPU will be restarted through reset at the next switch event for this
 * cpu.
 */
static void bLiks_power_down(unsigned int cpu, unsigned int cluster)
{
	bool last_man = bLiks_power_down_prepare(cpu, cluster);

	__bL_cpu_going_down(cpu, cluster);

	/*
	 * flush_cache_level_cpu() is a guess which should be correct
	 * for A15/A7.  We eventually need a defined way to find out
	 * the correct cache level to flush for the CPU's local
	 * coherency domain.
	 */
	disable_clean_inv_dcache(0);

	if (last_man && __bL_outbound_enter_critical(cpu, cluster)) {
		if (powerdown_needed(cluster)) {
			/*
			 * flush remaining architected caches
			 * not flushed by common code
			 */
			flush_cache_all();

			/*
			 * This is a harmless no-op.  On platforms with a real
			 * outer cache this might either be needed or not,
			 * depending on where the outer cache sits.
			 */
			outer_flush_all();

			/*
			 * Allow cluster to be powered down when all cores are
			 * idling.
			 */
			vexpress_spc_powerdown_enable(cluster, 1);

			/*
			 * Disable CCI snoops. Inbound cluster cannot pick up
			 * data from this cluster's caches once done.
			 */
			disable_cci(cluster);

			/*
			 * Ensure that both C & I bits are disabled in the SCTLR
			 * before disabling ACE snoops. This ensures that no
			 * coherency traffic will originate from this cpu after
			 * ACE snoops are turned off.
			 */
			cpu_proc_fin();

			/* Disable ACE and ACP (A15 only) snoops */
			vexpress_scc_ctl_snoops(cluster, 0);

			__bL_outbound_leave_critical(cluster, CLUSTER_DOWN);
		} else
			__bL_outbound_leave_critical(cluster, CLUSTER_UP);

	}

	__bL_cpu_down(cpu, cluster);

	/* Now we are prepared for power-down, do it: */
wfi_loop:
	wfi();
	goto wfi_loop;

}

static const struct bL_power_ops bLiks_power_ops = {
	.power_up = bLiks_power_up,
	.power_down = bLiks_power_down,
	.power_up_setup = bLiks_power_up_setup,
};

void __init bLiks_reserve(void)
{
	bL_switcher_reserve();
}

static int __init bLiks_init(void)
{
	u32 a15_clus_id, a7_clus_id;

	/*
	 * Initialize our cpu online maps assuming that A15 is 0 and A7 is 1.
	 * These maps keep track of cpus migrating between the two clusters.
	 */
	a15_clus_id = vexpress_spc_get_clusterid(0xC0F);
	a7_clus_id = vexpress_spc_get_clusterid(0xC07);

	cpu_online_map[a7_clus_id] = vexpress_scc_read_rststat(a7_clus_id);
	cpu_online_map[a15_clus_id] = vexpress_scc_read_rststat(a15_clus_id);

	return bL_switcher_init(&bLiks_power_ops);
}

__initcall(bLiks_init);

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
 * To avoid assumptions about cluster ids.
 */
static u32 a15_clus_id, a7_clus_id;

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
static void bLiks_power_up(unsigned int cpu, unsigned int cluster,
						phys_addr_t entry_point)
{
	u32 ret = 0;

	raw_spin_lock(&bLiks_lock);

	vexpress_spc_powerdown_enable(cluster, 0);

	/*
	 * Find out if this cpu's counterpart will be the first
	 * one in the inbound cluster and consequently have to
	 * power up the cluster first. Also, write the IKS entry
	 * point to the per-cpu mailbox.
	 */
	if (cpu_online_map[cluster] == 0)
		__bL_set_first_man(cpu, cluster);

	vexpress_spc_write_bxaddr_reg(cluster,
				      cpu,
				      entry_point);

	/*
	 * Enable wakeup irq source of this cpu to the power controller
	 * This is really required only for the first power up.
	 */
	vexpress_spc_set_cpu_wakeup_irq(cpu, cluster, 1);

	arm_send_ping_ipi(cpu);
	raw_spin_unlock(&bLiks_lock);

	return;
}

 /*
  * bLiks_power_up_finish - Declare completion of a cpu power up
  *
  * @cpu: CPU number within given cluster
  * @cluster: cluster number for the CPU
  *
  * @return: true if this cpu is the first man to reach this point
  *
  * Update the 'cpu_online_map', mask power controller wakeup
  * sources & disable ability to enter cluster power down. Basically
  * does the opposite of 'bLiks_power_down_prepare'
  */
static bool bLiks_power_up_finish(unsigned int cpu, unsigned int cluster)
{
	bool ret = false;

	raw_spin_lock(&bLiks_lock);

	/* Disable wakeup irq source of this cpu to the power controller */
	vexpress_spc_set_cpu_wakeup_irq(cpu, cluster, 0);

	if (cpu_online_map[cluster] == 0) {
		ret = true;

		/*
		 * Prevent cluster to be powered down when all cores are
		 * idling.
		 */
		vexpress_spc_powerdown_enable(cluster, 0);

		/*
		 * Disable the global wakeup sources
		 */
		vexpress_spc_set_global_wakeup_intr(0);
	}
	
	cpu_online_map[cluster] |= 1 << cpu;

	raw_spin_unlock(&bLiks_lock);

	return ret;
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

	 /* Enable wakeup irq source of this cpu to the power controller */
	 vexpress_spc_set_cpu_wakeup_irq(cpu, cluster, 1);

	 cpu_online_map[cluster] &= ~(1 << cpu);

	 /* if only cpu in cluster */
	 if (cpu_online_map[cluster] == 0) {

		 /*
		  * Allow cluster to be powered down when all cores are
		  * idling. This is done here instead of 'bLiks_power_down'
		  * so that an incoming cpu has a chance to abort the last
		  * man operations.
		  */
		 vexpress_spc_powerdown_enable(cluster, 1);

		 /*
		  * Enable the global wakeup sources
		  */
		 vexpress_spc_set_global_wakeup_intr(1);

		 ret = true;
	 }

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
	 wfi();

	 /*
	  * If this cpu was the last man and is here, then it needs to re-enable
	  * the coherency channel before issuing any barriers during a fake
	  * reset that will soon follow.
	  */
	 if (last_man)
		 vexpress_scc_ctl_snoops(cluster, 1);
 }

 static const struct bL_power_ops bLiks_power_ops = {
	 .power_up = bLiks_power_up,
	 .power_up_finish = bLiks_power_up_finish,
	 .power_down = bLiks_power_down,
	 .power_up_setup = bLiks_power_up_setup,
 };

 void __init bLiks_reserve(void)
 {
	 bL_switcher_reserve();
 }

 static int __init bLiks_init(void)
 {
	 u32 ctr, idx;

	 /*
	  * Initialize our cpu online maps assuming that A15 is 0 and A7 is 1.
	  * These maps keep track of cpus migrating between the two clusters.
	  */
	 a15_clus_id = vexpress_spc_get_clusterid(0xC0F);
	 a7_clus_id = vexpress_spc_get_clusterid(0xC07);

	 cpu_online_map[a7_clus_id] = vexpress_scc_read_rststat(a7_clus_id);
	 cpu_online_map[a15_clus_id] = vexpress_scc_read_rststat(a15_clus_id);
	__bL_set_cpus_per_cluster(a15_clus_id,
			vexpress_spc_get_nb_cpus(a15_clus_id));
	__bL_set_cpus_per_cluster(a7_clus_id,
			vexpress_spc_get_nb_cpus(a7_clus_id));
	 /*
	  * Remove secondary startup address from per-cpu mailboxes to prevent
	  * interference with gic cpuif id enumeration & fake resets. All cpus
	  * are guaranteed to have booted up by now.
	  */
	for (idx = 0; idx < BL_NR_CLUSTERS; idx++)
		for (ctr = 0; ctr < vexpress_spc_get_nb_cpus(idx); ctr++)
			vexpress_spc_write_bxaddr_reg(idx, ctr, 0x0);

	return bL_switcher_init(&bLiks_power_ops);
}

__initcall(bLiks_init);

/*
 * arch/arm/mach-vexpress/kingfisher.c - Kingfisher platform support
 *
 * Created by:	Nicolas Pitre, May 2012
 * Copyright:	(C) 2012  Linaro Limited
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

#include <asm/bL_switcher.h>
#include <asm/bL_entry.h>
#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/spinlock.h>

#include <mach/motherboard.h>
#include <mach/kingfisher.h>


#define KFSCB_PHYS_BASE	0x60000000

#define RST_HOLD0	0x0
#define RST_HOLD1	0x4
#define SYS_SWRESET	0x8
#define RST_STAT0	0xc
#define RST_STAT1	0x10
#define EAG_CFG_R	0x20
#define EAG_CFG_W	0x24
#define KFC_CFG_R	0x28
#define KFC_CFG_W	0x2c
#define KFS_CFG_R	0x30

static void __iomem *kfscb_base;
static DEFINE_RAW_SPINLOCK(kfscb_lock);
static int kfs_use_count[BL_CPUS_PER_CLUSTER][BL_NR_CLUSTERS];
static int kfs_cluster_cpu_mask[BL_NR_CLUSTERS];

static void __iomem *cci_base;

/*
 * bL_kfs_power_up - make given CPU in given cluster runable
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
static void bL_kfs_power_up(unsigned int cpu, unsigned int cluster)
{
	unsigned int rst_hold, cpumask = (1 << cpu);
	unsigned int cluster_mask = kfs_cluster_cpu_mask[cluster];

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	raw_spin_lock(&kfscb_lock);
	kfs_use_count[cpu][cluster]++;
	if (kfs_use_count[cpu][cluster] == 1) {
		rst_hold = readl_relaxed(kfscb_base + RST_HOLD0 + cluster * 4);
		if (rst_hold & (1 << 8)) {
			/* remove cluster reset and add individual CPU's reset */
			rst_hold &= ~(1 << 8);
			rst_hold |= cluster_mask;

			__bL_set_first_man(cpu, cluster);
		}
		rst_hold &= ~(cpumask | (cpumask << 4));
		writel(rst_hold, kfscb_base + RST_HOLD0 + cluster * 4);
	} else if (kfs_use_count[cpu][cluster] != 2) {
		/*
		 * The only possible values are:
		 * 0 = CPU down
		 * 1 = CPU (still) up
		 * 2 = CPU requested to be up before it had a chance
		 *     to actually make itself down.
		 * Any other value is a bug.
		 */
		BUG();
	}
	raw_spin_unlock(&kfscb_lock);
}

/*
 * Helper to determine whether the specified cluster should still be
 * shut down.  By polling this before shutting a cluster down, we can
 * reduce the probability of wasted cache flushing etc.
 */
static bool powerdown_needed(unsigned int cluster)
{
	unsigned int rst_hold;

	rst_hold = readl_relaxed(kfscb_base + RST_HOLD0 + cluster * 4);
	return (rst_hold & (1 << 8));
}

/*
 * bL_kfs_power_down - power a nominated CPU down
 *
 * @cpu: CPU number within given cluster
 * @cluster: cluster number for the CPU
 *
 * The identified CPU is powered down.
 *
 * If this CPU is found to be the last man in the cluster
 * then the cluster is prepared for power-down too, unless
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
static void bL_kfs_power_down(unsigned int cpu, unsigned int cluster)
{
	unsigned int rst_hold, cpumask = (1 << cpu);
	unsigned int cluster_mask = kfs_cluster_cpu_mask[cluster];
	bool last_man = false, skip_wfi = false;

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);

	__bL_cpu_going_down(cpu, cluster);

	raw_spin_lock(&kfscb_lock);
	kfs_use_count[cpu][cluster]--;
	if (kfs_use_count[cpu][cluster] == 0) {
		rst_hold = readl_relaxed(kfscb_base + RST_HOLD0 + cluster * 4);
		rst_hold |= cpumask;
		if (((rst_hold | (rst_hold >> 4)) & cluster_mask) == cluster_mask) {
			rst_hold |= (1 << 8);
			last_man = true;
		}
		writel(rst_hold, kfscb_base + RST_HOLD0 + cluster * 4);
	} else if (kfs_use_count[cpu][cluster] == 1) {
		/*
		 * A power_up request went ahead of us.
		 * Even if we do not want to shut this CPU down,
		 * the caller expects a certain state as if the WFI
		 * was aborted.  So let's continue with cache cleaning.
		 */
		skip_wfi = true;
	} else
		BUG();
	raw_spin_unlock(&kfscb_lock);

	if (last_man && __bL_outbound_enter_critical(cpu, cluster)) {
		unsigned int snoopctl;
		void __iomem *snoopctl_reg = cci_base + SLAVE_SNOOPCTL_OFFSET +
				CCI_SLAVE_OFFSET(cluster ?
					RTSM_CCI_SLAVE_A7 : RTSM_CCI_SLAVE_A15);
		void __iomem *status_reg = cci_base + CCI_STATUS_OFFSET;

		if (!powerdown_needed(cluster)) {
			__bL_outbound_leave_critical(cluster, CLUSTER_UP);
			goto non_last_man;
		}

		/*
		 * Flush the entire cache.
		 *
		 * A15/A7 can hit in the cache with SCTLR.C=0, so we don't need
		 * a preliminary flush here for those CPUs.  At least, that's
		 * the theory -- without the extra flush, Linux explodes on
		 * RTSM (maybe not needed anymore, to be investigated).
		 */
		flush_cache_all();
		cpu_proc_fin();	/* disable allocation into internal caches*/
		flush_cache_all();

		/*
		 * This is a harmless no-op.  On platforms with a real
		 * outer cache this might either be needed or not,
		 * depending on where the outer cache sits.
		 */
		outer_flush_all();

		/* Disable local coherency by clearing the ACTLR "SMP" bit: */
		asm volatile (
			"mrc	p15, 0, ip, c1, c0, 1 \n\t"
			"bic	ip, ip, #(1 << 6) @ clear SMP bit \n\t"
			"mcr	p15, 0, ip, c1, c0, 1 \n\t"
			"isb\n\t"
			"dsb\n\t"
			: : : "ip" );

		/*
		 * Disable cluster-level coherency by masking
		 * incoming snoops and DVM messages:
		 */
		snoopctl = readl_relaxed(snoopctl_reg);
		snoopctl &= ~(SNOOPCTL_SNOOP_ENABLE | SNOOPCTL_DVM_ENABLE);
		writel_relaxed(snoopctl, snoopctl_reg);

		/* Wait for snoop control change to complete: */
		while (readl_relaxed(status_reg) & STATUS_CHANGE_PENDING)
			cpu_relax();

		__bL_outbound_leave_critical(cluster, CLUSTER_DOWN);
	} else {
		non_last_man:
		/*
		 * flush_cache_level_cpu() is a guess which should be correct
		 * for A15/A7.  We eventually need a defined way to find out
		 * the correct cache level to flush for the CPU's local
		 * coherency domain.
		 */

		/*
		 * A15/A7 can hit in the cache with SCTLR.C=0, so we don't need
		 * a preliminary flush here for those CPUs.  At least, that's
		 * the theory -- without the extra flush, Linux explodes on
		 * RTSM (maybe not needed anymore, to be investigated).
		 */
		flush_dcache_level(flush_cache_level_cpu());
		cpu_proc_fin();	/* disable allocation into internal caches*/
		flush_dcache_level(flush_cache_level_cpu());

		/* Disable local coherency by clearing the ACTLR "SMP" bit: */
		asm volatile (
			"mrc	p15, 0, ip, c1, c0, 1 \n\t"
			"bic	ip, ip, #(1 << 6) @ clear SMP bit \n\t"
			"mcr	p15, 0, ip, c1, c0, 1 \n\t"
			"isb\n\t"
			"dsb\n\t"
			: : : "ip" );
	}

	__bL_cpu_down(cpu, cluster);

	/* Now we are prepared for power-down, do it: */
	if (!skip_wfi)
		wfi();
}

extern void bL_kfs_power_up_setup(void);

static const struct bL_power_ops bL_kfs_power_ops = {
	.power_up		= bL_kfs_power_up,
	.power_down		= bL_kfs_power_down,
	.power_up_setup		= bL_kfs_power_up_setup,
};

void __init kfs_reserve(void)
{
	bL_switcher_reserve();
}

static int __init kfs_init(void)
{
	unsigned int i, cfg, mpidr, this_cluster, cpu;

	kfscb_base = ioremap(KFSCB_PHYS_BASE, 0x1000);
	if (!kfscb_base)
		return -ENOMEM;
	cfg = readl_relaxed(kfscb_base + KFS_CFG_R);
	for (i = 0; i < BL_NR_CLUSTERS; i++)
		kfs_cluster_cpu_mask[i] =
			(1 << (((cfg >> 16) >> (i << 2)) & 0xf)) - 1;

	/* Map CCI registers */
	/* The CCI support should really be factored out */
	cci_base = ioremap(RTSM_CCI_PHYS_BASE, 0x10000);
	if (!cci_base)
		return -ENOMEM;

	/* All future entries into the kernel goes through our entry vectors. */
	v2m_flags_set(virt_to_phys(bl_entry_point));

	/*
	 * Initialize CPU usage counts, assuming that only one cluster is
	 * activated at this point.
	 */
	asm ("mrc\tp15, 0, %0, c0, c0, 5" : "=r" (mpidr));
	this_cluster = (mpidr >> 8) & 0xf;
	for_each_online_cpu(cpu)
		kfs_use_count[cpu][this_cluster] = 1;

	return bL_switcher_init(&bL_kfs_power_ops);
}

__initcall(kfs_init);

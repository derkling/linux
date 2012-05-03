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

#include <mach/motherboard.h>


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

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	raw_spin_lock(&kfscb_lock);
	rst_hold = readl_relaxed(kfscb_base + RST_HOLD0 + cluster * 4);
	if (rst_hold & (1 << 8)) {
		/* remove cluster reset and add individual CPU's reset */
		rst_hold &= ~(1 << 8);
		rst_hold |= 0xf;
	}
	rst_hold &= ~(cpumask | (cpumask << 4));
	writel(rst_hold, kfscb_base + RST_HOLD0 + cluster * 4);
	raw_spin_unlock(&kfscb_lock);
}

/*
 * bL_kfs_power_down - power down given CPU in given cluster
 *
 * @cpu: CPU number within given cluster
 * @cluster: cluster number for the CPU
 *
 * The identified CPU is powered down.  If this is the last CPU still alive
 * in the cluster then the necessary steps to power down the cluster are
 * performed as well and a non zero value is returned in that case.
 *
 * Given usage of a spinlock, the calling CPU must have its MMU still active.
 * It is assumed that the reset will be effective at the next WFI instruction
 * performed by the target CPU.
 */
static bool bL_kfs_power_down(unsigned int cpu, unsigned int cluster)
{
	unsigned int rst_hold, cpumask = (1 << cpu);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	raw_spin_lock(&kfscb_lock);
	rst_hold = readl_relaxed(kfscb_base + RST_HOLD0 + cluster * 4);
	rst_hold |= cpumask;
	if (((rst_hold | (rst_hold >> 4)) & 0xf) == 0xf)
		rst_hold |= (1 << 8);
	writel(rst_hold, kfscb_base + RST_HOLD0 + cluster * 4);
	raw_spin_unlock(&kfscb_lock);
	return (rst_hold & (1 << 8));
}

static const struct bL_power_ops bL_kfs_power_ops = {
	.power_up	= bL_kfs_power_up,
	.power_down	= bL_kfs_power_down,
};

static int __init kfs_init(void)
{
	kfscb_base = ioremap(KFSCB_PHYS_BASE, 0x1000);
	if (!kfscb_base)
		return -ENOMEM;

	/* All future entries into the kernel goes through our entry vectors. */
	v2m_flags_set(virt_to_phys(bl_entry_point));

	return bL_switcher_init(&bL_kfs_power_ops);
}

__initcall(kfs_init);

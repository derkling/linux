/*
 * Serial Power Controller (SPC) support
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Sudeep KarkadaNagesha <sudeep.karkadanagesha@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <mach/spc.h>

#define PERF_LVL_EAG		0xB00
#define PERF_REQ_EAG		0xB04
#define PERF_LVL_KF		0xB08
#define PERF_REQ_KF		0xB0c
#define COMMS			0xB10
#define COMMS_REQ		0xB14
#define PWC_STATUS		0xB18
#define PWC_FLAG		0xB1c
#define WAKE_INT_MASK		0xB24
#define WAKE_INT_RAW		0xB28
#define WAKE_INT_STAT		0xB2c
#define EAG_PWRDN_EN		0xB30
#define KF_PWRDN_EN		0xB34
#define EAG_KF_ISOLATE		0xB38
#define STANDBYWFI_STAT		0xB3c
#define EAG_CACTIVE		0xB40
#define EAG_PWRDNREQ		0xB44
#define EAG_PWRDNACK		0xB48
#define KF_CACTIVE		0xB4c
#define KF_PWRDNREQ		0xB50
#define KF_PWRDNACK		0xB54
#define EAG_RESET_HOLD		0xB58
#define KF_RESET_HOLD		0xB5c
#define EAG_RESET_STAT		0xB60
#define KF_RESET_STAT		0xB64

#define DRIVER_NAME	"SPC"
#define TIME_OUT	100
struct spc_drvdata {
	void __iomem *baseaddr;
	spinlock_t lock;
};

static struct spc_drvdata *info;

static inline int read_wait_to(void __iomem *reg, int status, int timeout)
{
	while (timeout-- && readl(reg) == status)
		cpu_relax();
	if (!timeout)
		return -EAGAIN;
	else
		return 0;
}

int spc_set_performance(int cluster, int perf)
{
	u32 perf_cfg_reg = cluster ? PERF_LVL_KF : PERF_LVL_EAG;
	u32 perf_stat_reg = cluster ? PERF_REQ_KF : PERF_REQ_EAG;
	int ret = 0;

	if (perf < 0 || perf >= 200)
		return -EINVAL;
	perf = (perf << 8) / 100;

	spin_lock(&info->lock);
	writel(perf, info->baseaddr + perf_cfg_reg);
	if (read_wait_to(info->baseaddr + perf_stat_reg, 1, TIME_OUT))
		ret = -EAGAIN;
	spin_unlock(&info->lock);
	return ret;

}
EXPORT_SYMBOL_GPL(spc_set_performance);

void spc_set_wake_intr(u32 mask)
{
	spin_lock(&info->lock);
	writel(mask & WAKE_INTR_MASK, info->baseaddr + WAKE_INT_MASK);
	spin_unlock(&info->lock);
	return;
}
EXPORT_SYMBOL_GPL(spc_set_wake_intr);

u32 spc_get_wake_intr(int raw)
{
	u32 val;
	u32 wake_intr_reg = raw ? WAKE_INT_RAW : WAKE_INT_STAT;
	spin_lock(&info->lock);
	val = readl(info->baseaddr + wake_intr_reg);
	spin_unlock(&info->lock);
	return val;
}
EXPORT_SYMBOL_GPL(spc_get_wake_intr);

void spc_powerdown_enable(int cluster, int enable)
{
	u32 pwdrn_reg = cluster ? KF_PWRDN_EN : EAG_PWRDN_EN;
	spin_lock(&info->lock);
	writel(!!enable, info->baseaddr + pwdrn_reg);
	spin_unlock(&info->lock);
	return;
}
EXPORT_SYMBOL_GPL(spc_powerdown_enable);

void spc_adb400_pd_enable(int cluster, int enable)
{
	u32 pwdrn_reg = cluster ? KF_PWRDNREQ : EAG_PWRDNREQ;
	u32 val = enable ? 0xF : 0x0;	/* all adb bridges ?? */
	spin_lock(&info->lock);
	writel(val, info->baseaddr + pwdrn_reg);
	spin_unlock(&info->lock);
	return;
}
EXPORT_SYMBOL_GPL(spc_adb400_pd_enable);

void spc_wfi_cpureset(int cluster, int cpu, int enable)
{
	u32 rsthold_reg, prst_shift;
	u32 val;
	
	if (!info)
		return;

	if (cluster) {
		rsthold_reg = KF_RESET_HOLD;
		prst_shift = 3;
	} else {
		rsthold_reg = EAG_RESET_HOLD;
		prst_shift = 2;
	}
	//spin_lock(&info->lock);
	val = readl_relaxed(info->baseaddr + rsthold_reg);
	if (enable)
		//val |= (1 << cpu) | (1 << cpu) << prst_shift;
		val |= (1 << cpu);
	else
		val &= ~((1 << cpu) | (1 << cpu) << prst_shift);
	writel_relaxed(val, info->baseaddr + rsthold_reg);
	//spin_unlock(&info->lock);
	return;
}
EXPORT_SYMBOL_GPL(spc_wfi_cpureset);

void spc_wfi_cluster_reset(int cluster, int enable)
{
	u32 rsthold_reg, shift;
	u32 val;
	if (cluster) {
		rsthold_reg = KF_RESET_HOLD;
		shift = 6;
	} else {
		rsthold_reg = EAG_RESET_HOLD;
		shift = 4;
	}
	spin_lock(&info->lock);
	val = readl(info->baseaddr + rsthold_reg);
	if (enable)
		val |= 1 << shift;
	else
		val &= ~(1 << shift);
	writel(val, info->baseaddr + rsthold_reg);
	spin_unlock(&info->lock);
	return;
}
EXPORT_SYMBOL_GPL(spc_wfi_cluster_reset);

static int __devinit spc_driver_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "unable to allocate mem\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -EINVAL;
		goto mem_free;
	}

	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "address 0x%x in use\n", (u32) res->start);
		ret = -EBUSY;
		goto mem_free;
	}

	info->baseaddr = ioremap(res->start, resource_size(res));
	if (!info->baseaddr) {
		ret = -ENXIO;
		goto ioremap_err;
	}

	spin_lock_init(&info->lock);
	platform_set_drvdata(pdev, info);

	pr_info("spc loaded at %p\n", info->baseaddr);
	return ret;

ioremap_err:
	release_region(res->start, resource_size(res));
mem_free:
	kfree(info);

	return ret;
}

static int __devexit spc_driver_remove(struct platform_device *pdev)
{
	struct spc_drvdata *info;
	struct resource *res = pdev->resource;

	info = platform_get_drvdata(pdev);
	iounmap(info->baseaddr);
	release_region(res->start, resource_size(res));
	kfree(info);

	return 0;
}

static const struct of_device_id arm_spc_matches[] = {
	{.compatible = "arm,spc"},
	{},
};

static struct platform_driver spc_platform_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DRIVER_NAME,
		   .of_match_table = arm_spc_matches,
		   },
	.probe = spc_driver_probe,
	.remove = spc_driver_remove,
};

static int __init spc_init(void)
{
	return platform_driver_register(&spc_platform_driver);
}

static void __exit spc_exit(void)
{
	platform_driver_unregister(&spc_platform_driver);
}

module_init(spc_init);
module_exit(spc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Serial Power Controller (SPC) support");

/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Yong Wu <yong.wu@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/iommu.h>
#include <linux/of_iommu.h>
#include <linux/dma-mapping.h>
#include <linux/dma-iommu.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/mtk-smi.h>
#include <asm/cacheflush.h>

#include "io-pgtable.h"

#define REG_MMU_PT_BASE_ADDR			0x000

#define REG_MMU_INVALIDATE			0x020
#define F_ALL_INVLD				0x2
#define F_MMU_INV_RANGE				0x1

#define REG_MMU_INVLD_START_A			0x024
#define REG_MMU_INVLD_END_A			0x028

#define REG_MMU_INV_SEL				0x038
#define F_INVLD_EN0				BIT(0)
#define F_INVLD_EN1				BIT(1)

#define REG_MMU_STANDARD_AXI_MODE		0x048
#define REG_MMU_DCM_DIS				0x050

#define REG_MMU_CTRL_REG			0x110
#define F_MMU_PREFETCH_RT_REPLACE_MOD		BIT(4)
#define F_MMU_TF_PROTECT_SEL(prot)		(((prot) & 0x3) << 5)
#define F_COHERENCE_EN				BIT(8)

#define REG_MMU_IVRP_PADDR			0x114
#define F_MMU_IVRP_PA_SET(pa)			((pa) >> 1)

#define REG_MMU_INT_CONTROL0			0x120
#define F_L2_MULIT_HIT_EN			BIT(0)
#define F_TABLE_WALK_FAULT_INT_EN		BIT(1)
#define F_PREETCH_FIFO_OVERFLOW_INT_EN		BIT(2)
#define F_MISS_FIFO_OVERFLOW_INT_EN		BIT(3)
#define F_PREFETCH_FIFO_ERR_INT_EN		BIT(5)
#define F_MISS_FIFO_ERR_INT_EN			BIT(6)
#define F_INT_L2_CLR_BIT			BIT(12)

#define REG_MMU_INT_MAIN_CONTROL		0x124
#define F_INT_TRANSLATION_FAULT			BIT(0)
#define F_INT_MAIN_MULTI_HIT_FAULT		BIT(1)
#define F_INT_INVALID_PA_FAULT			BIT(2)
#define F_INT_ENTRY_REPLACEMENT_FAULT		BIT(3)
#define F_INT_TLB_MISS_FAULT			BIT(4)
#define F_INT_MISS_TRANSATION_FIFO_FAULT	BIT(5)
#define F_INT_PRETETCH_TRANSATION_FIFO_FAULT	BIT(6)

#define REG_MMU_CPE_DONE			0x12C

#define REG_MMU_FAULT_ST1			0x134

#define REG_MMU_FAULT_VA			0x13c
#define F_MMU_FAULT_VA_MSK			0xfffff000
#define F_MMU_FAULT_VA_WRITE_BIT		BIT(1)
#define F_MMU_FAULT_VA_LAYER_BIT		BIT(0)

#define REG_MMU_INVLD_PA			0x140
#define REG_MMU_INT_ID				0x150
#define F_MMU0_INT_ID_LARB_ID(a)		(((a) >> 7) & 0x7)
#define F_MMU0_INT_ID_PORT_ID(a)		(((a) >> 2) & 0x1f)

#define MTK_PROTECT_PA_ALIGN			(128)

#define MTK_IOMMU_LARB_MAX_NR			8

struct mtk_iommu_info {
	void __iomem		*base;
	int			irq;
	struct device		*dev;
	struct device		*larbdev[MTK_IOMMU_LARB_MAX_NR];
	struct clk		*bclk;
	dma_addr_t		protect_base; /* protect memory base */
	int			larb_nr;      /* local arbiter number */
};

struct mtk_iommu_domain {
	struct imu_pgd_t	*pgd;
	spinlock_t		pgtlock; /* lock for modifying page table */

	struct io_pgtable_cfg   cfg;
	struct io_pgtable_ops   *iop;

	struct mtk_iommu_info   *imuinfo;
	struct iommu_domain	domain;
};

struct mtk_iommu_client_priv {
	struct list_head	client;
	uint32_t		larbid;
	uint32_t		portid;
	struct device		*dev;
};

static struct mtk_iommu_domain *to_mtk_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct mtk_iommu_domain, domain);
}

static void mtk_iommu_clear_intr(const struct mtk_iommu_info *piommu)
{
	u32 val;

	val = readl(piommu->base + REG_MMU_INT_CONTROL0);
	val |= F_INT_L2_CLR_BIT;
	writel(val, piommu->base + REG_MMU_INT_CONTROL0);
}

static void mtk_iommu_tlb_flush_all(void *cookie)
{
	struct mtk_iommu_domain *domain = cookie;
	void __iomem *base = domain->imuinfo->base;

	writel(F_INVLD_EN1 | F_INVLD_EN0, base + REG_MMU_INV_SEL);
	writel(F_ALL_INVLD, base + REG_MMU_INVALIDATE);
}

static void mtk_iommu_tlb_add_flush(unsigned long iova, size_t size,
				    bool leaf, void *cookie)
{
	struct mtk_iommu_domain *domain = cookie;
	void __iomem *base = domain->imuinfo->base;
	unsigned int iova_start = iova, iova_end = iova + size - 1;

	writel(F_INVLD_EN1 | F_INVLD_EN0, base + REG_MMU_INV_SEL);

	writel(iova_start, base + REG_MMU_INVLD_START_A);
	writel(iova_end, base + REG_MMU_INVLD_END_A);
	writel(F_MMU_INV_RANGE, base + REG_MMU_INVALIDATE);
}

static void mtk_iommu_tlb_sync(void *cookie)
{
	struct mtk_iommu_domain *domain = cookie;
	void __iomem *base = domain->imuinfo->base;
	int ret;
	u32 tmp;

	ret = readl_poll_timeout_atomic(base + REG_MMU_CPE_DONE, tmp,
					tmp != 0, 10, 1000000);
	if (ret) {
		dev_warn(domain->imuinfo->dev,
			 "Partial TLB flush timed out, falling back to full flush\n");
		mtk_iommu_tlb_flush_all(cookie);
	}
	writel(0, base + REG_MMU_CPE_DONE);
}

static void mtk_iommu_flush_pgtable(void *ptr, size_t size, void *cookie)
{
	struct mtk_iommu_domain *domain = cookie;
	unsigned long offset = (unsigned long)ptr & ~PAGE_MASK;

	if (domain->imuinfo) {
		dma_map_page(domain->imuinfo->dev, virt_to_page(ptr), offset,
			     size, DMA_TO_DEVICE);
	}
}

static struct iommu_gather_ops mtk_iommu_gather_ops = {
	.tlb_flush_all = mtk_iommu_tlb_flush_all,
	.tlb_add_flush = mtk_iommu_tlb_add_flush,
	.tlb_sync = mtk_iommu_tlb_sync,
	.flush_pgtable = mtk_iommu_flush_pgtable,
};

static irqreturn_t mtk_iommu_isr(int irq, void *dev_id)
{
	struct mtk_iommu_domain *mtkdomain = dev_id;
	struct mtk_iommu_info *piommu = mtkdomain->imuinfo;
	u32 int_state, regval, fault_iova, fault_pa;
	unsigned int fault_larb, fault_port;
	bool layer, write;

	int_state = readl(piommu->base + REG_MMU_FAULT_ST1);

	/* read error info from registers */
	fault_iova = readl(piommu->base + REG_MMU_FAULT_VA);
	layer = fault_iova & F_MMU_FAULT_VA_LAYER_BIT;
	write = fault_iova & F_MMU_FAULT_VA_WRITE_BIT;
	fault_iova &= F_MMU_FAULT_VA_MSK;
	fault_pa = readl(piommu->base + REG_MMU_INVLD_PA);
	regval = readl(piommu->base + REG_MMU_INT_ID);
	fault_larb = F_MMU0_INT_ID_LARB_ID(regval);
	fault_port = F_MMU0_INT_ID_PORT_ID(regval);

	if (report_iommu_fault(&mtkdomain->domain, piommu->dev, fault_iova,
			       write ? IOMMU_FAULT_WRITE : IOMMU_FAULT_READ)) {
		dev_err_ratelimited(
			piommu->dev,
			"fault type=0x%x iova=0x%x pa=0x%x larb=%d port=%d layer=%d %s\n",
			int_state, fault_iova, fault_pa, fault_larb, fault_port,
			layer, write ? "write" : "read");
	}

	mtk_iommu_tlb_flush_all(mtkdomain);
	mtk_iommu_clear_intr(piommu);

	return IRQ_HANDLED;
}

static int mtk_iommu_parse_dt(struct platform_device *pdev,
			      struct mtk_iommu_info *piommu)
{
	struct device *dev = &pdev->dev;
	struct device_node *ofnode;
	struct resource *res;
	int i;

	ofnode = dev->of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	piommu->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(piommu->base))
		return PTR_ERR(piommu->base);

	piommu->irq = platform_get_irq(pdev, 0);
	if (piommu->irq < 0)
		return piommu->irq;

	piommu->bclk = devm_clk_get(dev, "bclk");
	if (IS_ERR(piommu->bclk))
		return PTR_ERR(piommu->bclk);

	piommu->larb_nr = of_count_phandle_with_args(ofnode, "larb", NULL);
	if (piommu->larb_nr < 0)
		return piommu->larb_nr;

	for (i = 0; i < piommu->larb_nr; i++) {
		struct device_node *larbnode;
		struct platform_device *plarbdev;

		larbnode = of_parse_phandle(ofnode, "larb", i);
		if (!larbnode)
			return -EINVAL;

		plarbdev = of_find_device_by_node(larbnode);
		of_node_put(larbnode);
		if (!plarbdev)
			return -EINVAL;
		piommu->larbdev[i] = &plarbdev->dev;
	}

	return 0;
}

static int mtk_iommu_hw_init(const struct mtk_iommu_domain *mtkdomain)
{
	struct mtk_iommu_info *piommu = mtkdomain->imuinfo;
	void __iomem *base = piommu->base;
	u32 regval;
	int ret = 0;

	ret = clk_prepare_enable(piommu->bclk);
	if (ret)
		return ret;

	writel(mtkdomain->cfg.arm_short_cfg.ttbr[0],
	       base + REG_MMU_PT_BASE_ADDR);

	regval = F_MMU_PREFETCH_RT_REPLACE_MOD |
		F_MMU_TF_PROTECT_SEL(2) |
		F_COHERENCE_EN;
	writel(regval, base + REG_MMU_CTRL_REG);

	regval = F_L2_MULIT_HIT_EN |
		F_TABLE_WALK_FAULT_INT_EN |
		F_PREETCH_FIFO_OVERFLOW_INT_EN |
		F_MISS_FIFO_OVERFLOW_INT_EN |
		F_PREFETCH_FIFO_ERR_INT_EN |
		F_MISS_FIFO_ERR_INT_EN;
	writel(regval, base + REG_MMU_INT_CONTROL0);

	regval = F_INT_TRANSLATION_FAULT |
		F_INT_MAIN_MULTI_HIT_FAULT |
		F_INT_INVALID_PA_FAULT |
		F_INT_ENTRY_REPLACEMENT_FAULT |
		F_INT_TLB_MISS_FAULT |
		F_INT_MISS_TRANSATION_FIFO_FAULT |
		F_INT_PRETETCH_TRANSATION_FIFO_FAULT;
	writel(regval, base + REG_MMU_INT_MAIN_CONTROL);

	regval = ALIGN(piommu->protect_base, MTK_PROTECT_PA_ALIGN);
	regval = F_MMU_IVRP_PA_SET(regval);
	writel(regval, base + REG_MMU_IVRP_PADDR);

	writel(0, base + REG_MMU_DCM_DIS);
	writel(0, base + REG_MMU_STANDARD_AXI_MODE);

	if (devm_request_irq(piommu->dev, piommu->irq, mtk_iommu_isr, 0,
			     dev_name(piommu->dev), (void *)mtkdomain)) {
		writel(0, base + REG_MMU_PT_BASE_ADDR);
		clk_disable_unprepare(piommu->bclk);
		dev_err(piommu->dev, "Failed @ IRQ-%d Request\n", piommu->irq);
		return -ENODEV;
	}

	return 0;
}

static struct iommu_domain *mtk_iommu_domain_alloc(unsigned type)
{
	struct mtk_iommu_domain *priv;

	/* We only support unmanaged domains for now */
	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	priv->cfg.quirks = IO_PGTABLE_QUIRK_ARM_NS |
			IO_PGTABLE_QUIRK_SHORT_SUPERSECTION;
	priv->cfg.pgsize_bitmap = SZ_16M | SZ_1M | SZ_64K | SZ_4K,
	priv->cfg.ias = 32;
	priv->cfg.oas = 32;
	priv->cfg.tlb = &mtk_iommu_gather_ops;

	priv->iop = alloc_io_pgtable_ops(ARM_SHORT_DESC, &priv->cfg, priv);
	if (!priv->iop) {
		pr_err("Failed to alloc io pgtable@MTK iommu\n");
		goto err_free_priv;
	}

	spin_lock_init(&priv->pgtlock);

	priv->domain.geometry.aperture_start = 0;
	priv->domain.geometry.aperture_end = (1ULL << 32) - 1;
	priv->domain.geometry.force_aperture = true;

	return &priv->domain;

err_free_priv:
	kfree(priv);
	return NULL;
}

static void mtk_iommu_domain_free(struct iommu_domain *domain)
{
	struct mtk_iommu_domain *priv = to_mtk_domain(domain);

	free_io_pgtable_ops(priv->iop);
	kfree(priv);
}

static int mtk_iommu_attach_device(struct iommu_domain *domain,
				   struct device *dev)
{
	struct mtk_iommu_domain *priv = to_mtk_domain(domain);

	dev->archdata.iommu = priv;
	/* attach device we can do nothing. this function will
	 * run before probe, At that time, we can't write register
	 * to enable iommu.
	 * so we move enable iommu in probe after parse dt
	 * in current temp solution.
	 */
	return 0;
}

static void mtk_iommu_detach_device(struct iommu_domain *domain,
				    struct device *dev)
{
	/* current is temp solution, we have not implement this */
}

static int mtk_iommu_map(struct iommu_domain *domain, unsigned long iova,
			 phys_addr_t paddr, size_t size, int prot)
{
	struct mtk_iommu_domain *priv = to_mtk_domain(domain);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->pgtlock, flags);
	ret = priv->iop->map(priv->iop, iova, paddr, size, prot);
	spin_unlock_irqrestore(&priv->pgtlock, flags);

	return ret;
}

static size_t mtk_iommu_unmap(struct iommu_domain *domain,
			      unsigned long iova, size_t size)
{
	struct mtk_iommu_domain *priv = to_mtk_domain(domain);
	unsigned long flags;

	spin_lock_irqsave(&priv->pgtlock, flags);
	priv->iop->unmap(priv->iop, iova, size);
	spin_unlock_irqrestore(&priv->pgtlock, flags);

	return size;
}

static phys_addr_t mtk_iommu_iova_to_phys(struct iommu_domain *domain,
					  dma_addr_t iova)
{
	struct mtk_iommu_domain *priv = to_mtk_domain(domain);
	unsigned long flags;
	phys_addr_t pa;

	spin_lock_irqsave(&priv->pgtlock, flags);
	pa = priv->iop->iova_to_phys(priv->iop, iova);
	spin_unlock_irqrestore(&priv->pgtlock, flags);

	return pa;
}

int mtk_iommu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	struct mtk_iommu_client_priv *head, *priv;
	struct platform_device *imupdev;
	struct device *imudev;

	if (args->args_count != 2) {
		dev_err(dev, "invalid #iommu-cells(%d) property for IOMMU\n",
			args->args_count);
		return -EINVAL;
	}

	imupdev = of_find_device_by_node(args->np);
	of_node_put(args->np);
	if (WARN_ON(!imupdev))
		return -EINVAL;
	imudev = &imupdev->dev;

	if (!imudev->archdata.iommu) {
		head = kzalloc(sizeof(*head), GFP_KERNEL);
		if (!head)
			return -ENOMEM;

		imudev->archdata.iommu = head;
		INIT_LIST_HEAD(&head->client);
	} else {
		head = imudev->archdata.iommu;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto err_mem;

	priv->larbid = args->args[0];
	priv->portid = args->args[1];
	priv->dev = dev;
	list_add_tail(&priv->client, &head->client);

	return 0;

err_mem:/* it should delete all the list */
	return -ENOMEM;
}

static struct iommu_ops mtk_iommu_ops = {
	.domain_alloc = mtk_iommu_domain_alloc,
	.domain_free = mtk_iommu_domain_free,
	.attach_dev = mtk_iommu_attach_device,
	.detach_dev = mtk_iommu_detach_device,
	.map = mtk_iommu_map,
	.unmap = mtk_iommu_unmap,
	.map_sg = default_iommu_map_sg,
	.iova_to_phys = mtk_iommu_iova_to_phys,
	.of_xlate = mtk_iommu_of_xlate,
	.pgsize_bitmap = SZ_4K | SZ_64K | SZ_1M | SZ_16M,
};

static const struct of_device_id mtk_iommu_of_ids[] = {
	{ .compatible = "mediatek,mt8173-m4u",},
	{}
};

static int mtk_iommu_init_fn(struct device_node *np)
{
	struct platform_device *pdev;

	pdev = of_platform_device_create(np, NULL, platform_bus_type.dev_root);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	of_iommu_set_ops(np, &mtk_iommu_ops);
	return 0;
}

IOMMU_OF_DECLARE(mtkm4u, "mediatek,mt8173-m4u", mtk_iommu_init_fn);

static int mtk_iommu_config(struct device *dev, struct mtk_iommu_info *piommu,
			    bool enable)
{
	struct mtk_iommu_client_priv *head, *cur, *next;
	int ret = 0;

	head = dev->archdata.iommu;

	list_for_each_entry_safe(cur, next, &head->client, client) {
		if (cur->larbid >= piommu->larb_nr) {
			dev_err(dev, "Invalid larbid %d\n", cur->larbid);
			return -EINVAL;
		}
		ret = mtk_smi_config_port(piommu->larbdev[cur->larbid],
					  cur->portid, enable);
		if (ret) {
			dev_err(dev, "Failed @ %s iommu for larb:%d port:%d\n",
				enable ? "enable" : "disable",
				cur->larbid, cur->portid);
			return ret;
		}
	}
	return 0;
}

static struct mtk_iommu_domain *
mtk_iommu_get_imu_domain(struct device *dev)
{
	struct mtk_iommu_client_priv *head, *cur, *next;
	struct mtk_iommu_domain *domain;

	head = dev->archdata.iommu;
	list_for_each_entry_safe(cur, next, &head->client, client) {
		domain = cur->dev->archdata.iommu;
		break;
	}
	return domain;
}

static int mtk_iommu_probe(struct platform_device *pdev)
{
	struct mtk_iommu_domain	*imu_domain;
	struct mtk_iommu_info   *piommu;
	struct device           *dev = &pdev->dev;
	void __iomem	        *protect;
	int                     ret;

	piommu = devm_kzalloc(dev, sizeof(*piommu), GFP_KERNEL);
	if (!piommu)
		return -ENOMEM;

	/* Protect memory. HW will access here while translation fault.*/
	protect = devm_kzalloc(dev, MTK_PROTECT_PA_ALIGN * 2,
			       GFP_KERNEL);
	if (!protect)
		return -ENOMEM;
	piommu->protect_base = virt_to_phys(protect);

	ret = mtk_iommu_parse_dt(pdev, piommu);
	if (ret)
		return ret;

	imu_domain = mtk_iommu_get_imu_domain(dev);
	imu_domain->imuinfo = piommu;
	piommu->dev = dev;

	dev_set_drvdata(dev, imu_domain);

	ret = mtk_iommu_hw_init(imu_domain);
	if (ret < 0) {
		dev_err(dev, "Hardware initialization failed\n");
		return ret;
	}

	return mtk_iommu_config(dev, piommu, true);
}

static int mtk_iommu_remove(struct platform_device *pdev)
{
	struct mtk_iommu_domain	*imu_domain = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(imu_domain->imuinfo->bclk);
	free_io_pgtable_ops(imu_domain->iop);

	return 0;
}

static struct platform_driver mtk_iommu_driver = {
	.probe	= mtk_iommu_probe,
	.remove	= mtk_iommu_remove,
	.driver	= {
		.name = "mtk-iommu",
		.of_match_table = mtk_iommu_of_ids,
	}
};

static int __init mtk_iommu_init(void)
{
	return platform_driver_register(&mtk_iommu_driver);
}

subsys_initcall(mtk_iommu_init);

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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/mtk-smi.h>

#define SMI_LARB_MMU_EN		(0xf00)
#define F_SMI_MMU_EN(port)	(1 << (port))

struct mtk_smi_common {
	struct clk		*clk_apb;
	struct clk		*clk_smi;
};

struct mtk_smi_larb {
	void __iomem		*base;
	spinlock_t		portlock; /* lock for config port */
	struct clk		*clk_apb;
	struct clk		*clk_smi;
	struct device		*smi;
	u32			mmu;
};

static int mtk_smi_common_get(struct device *smidev)
{
	struct mtk_smi_common *smipriv = dev_get_drvdata(smidev);
	int ret;

	ret = clk_enable(smipriv->clk_apb);
	if (ret) {
		dev_err(smidev, "Failed to enable the apb clock\n");
		return ret;
	}
	ret = clk_enable(smipriv->clk_smi);
	if (ret) {
		dev_err(smidev,	"Failed to enable the smi clock\n");
		clk_disable(smipriv->clk_apb);
	}

	return ret;
}

static void mtk_smi_common_put(struct device *smidev)
{
	struct mtk_smi_common *smipriv = dev_get_drvdata(smidev);

	clk_disable(smipriv->clk_smi);
	clk_disable(smipriv->clk_apb);
}

int mtk_smi_larb_get(struct device *larbdev)
{
	struct mtk_smi_larb *larbpriv = dev_get_drvdata(larbdev);
	int ret;

	ret = mtk_smi_common_get(larbpriv->smi);
	if (ret)
		return ret;

	ret = clk_enable(larbpriv->clk_apb);
	if (ret) {
		dev_err(larbdev, "Failed to enable the apb clock\n");
		goto err_clk_common;
	}

	ret = clk_enable(larbpriv->clk_smi);
	if (ret) {
		dev_err(larbdev, "Failed to enable the smi clock\n");
		goto err_clk_smi;
	}

	return ret;

err_clk_smi:
	clk_disable(larbpriv->clk_apb);
err_clk_common:
	mtk_smi_common_put(larbpriv->smi);
	return ret;
}

void mtk_smi_larb_put(struct device *larbdev)
{
	struct mtk_smi_larb *larbpriv = dev_get_drvdata(larbdev);

	clk_disable(larbpriv->clk_smi);
	clk_disable(larbpriv->clk_apb);

	mtk_smi_common_put(larbpriv->smi);
}

int mtk_smi_config_port(struct device *larbdev,	unsigned int larbportid,
			bool iommuen)
{
	struct mtk_smi_larb *larbpriv = dev_get_drvdata(larbdev);
	unsigned long flags;
	int ret;
	u32 reg;

	dev_dbg(larbdev, "smi-config port id %d\n", larbportid);

	ret = mtk_smi_larb_get(larbdev);
	if (ret)
		return ret;

	spin_lock_irqsave(&larbpriv->portlock, flags);
	reg = readl(larbpriv->base + SMI_LARB_MMU_EN);
	reg &= ~F_SMI_MMU_EN(larbportid);
	if (iommuen)
		reg |= F_SMI_MMU_EN(larbportid);
	writel(reg, larbpriv->base + SMI_LARB_MMU_EN);
	spin_unlock_irqrestore(&larbpriv->portlock, flags);

	mtk_smi_larb_put(larbdev);

	return 0;
}

static int mtk_smi_larb_probe(struct platform_device *pdev)
{
	struct mtk_smi_larb *larbpriv;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *smi_node;
	struct platform_device *smi_pdev;
	int ret;

	larbpriv = devm_kzalloc(dev, sizeof(struct mtk_smi_larb), GFP_KERNEL);
	if (!larbpriv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	larbpriv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(larbpriv->base))
		return PTR_ERR(larbpriv->base);

	larbpriv->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(larbpriv->clk_apb))
		return PTR_ERR(larbpriv->clk_apb);
	ret = clk_prepare(larbpriv->clk_apb);
	if (ret) {
		dev_err(dev, "Failed to prepare apb clock 0x%x\n", ret);
		return ret;
	}

	larbpriv->clk_smi = devm_clk_get(dev, "smi");
	if (IS_ERR(larbpriv->clk_smi)) {
		ret = PTR_ERR(larbpriv->clk_smi);
		goto fail_clk_apb;
	}
	ret = clk_prepare(larbpriv->clk_smi);
	if (ret) {
		dev_err(dev, "Failed to prepare smi clock 0x%x\n", ret);
		goto fail_clk_apb;
	}

	smi_node = of_parse_phandle(dev->of_node, "smi", 0);
	if (!smi_node) {
		dev_err(dev, "Failed to get smi node\n");
		ret = -EINVAL;
		goto fail_clk_smi;
	}

	smi_pdev = of_find_device_by_node(smi_node);
	of_node_put(smi_node);
	if (smi_pdev) {
		larbpriv->smi = &smi_pdev->dev;
	} else {
		dev_err(dev, "Failed to get the smi_common device\n");
		ret = -EINVAL;
		goto fail_clk_smi;
	}

	spin_lock_init(&larbpriv->portlock);
	dev_set_drvdata(dev, larbpriv);
	return 0;

fail_clk_smi:
	clk_unprepare(larbpriv->clk_smi);
fail_clk_apb:
	clk_unprepare(larbpriv->clk_apb);
	return ret;
}

static int mtk_smi_larb_suspend(struct device *dev)
{
	struct mtk_smi_larb *larbpriv = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&larbpriv->portlock, flags);
	larbpriv->mmu = readl(larbpriv->base + SMI_LARB_MMU_EN);
	spin_unlock_irqrestore(&larbpriv->portlock, flags);

	return 0;
}

static int mtk_smi_larb_resume(struct device *dev)
{
	struct mtk_smi_larb *larbpriv = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&larbpriv->portlock, flags);
	writel(larbpriv->mmu, larbpriv->base + SMI_LARB_MMU_EN);
	spin_unlock_irqrestore(&larbpriv->portlock, flags);

	return 0;
}

static const struct dev_pm_ops mtk_smi_larb_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mtk_smi_larb_suspend, mtk_smi_larb_resume)
};

static const struct of_device_id mtk_smi_larb_of_ids[] = {
	{ .compatible = "mediatek,mt8173-smi-larb",
	},
	{}
};

static struct platform_driver mtk_smi_larb_driver = {
	.probe	= mtk_smi_larb_probe,
	.driver	= {
		.name = "mtk-smi-larb",
		.of_match_table = mtk_smi_larb_of_ids,
		.pm = &mtk_smi_larb_ops,
	}
};

static int mtk_smi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_smi_common *smipriv;
	int ret;

	smipriv = devm_kzalloc(dev, sizeof(*smipriv), GFP_KERNEL);
	if (!smipriv)
		return -ENOMEM;

	smipriv->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(smipriv->clk_apb))
		return PTR_ERR(smipriv->clk_apb);
	ret = clk_prepare(smipriv->clk_apb);
	if (ret) {
		dev_err(dev, "Failed to prepare apb clock 0x%x\n", ret);
		return ret;
	}

	smipriv->clk_smi = devm_clk_get(dev, "smi");
	if (IS_ERR(smipriv->clk_smi)) {
		ret = PTR_ERR(smipriv->clk_smi);
		goto fail_clk_smi;
	}
	ret = clk_prepare(smipriv->clk_smi);
	if (ret) {
		dev_err(dev, "Failed to prepare smi clock 0x%x\n", ret);
		goto fail_clk_smi;
	}

	dev_set_drvdata(dev, smipriv);
	return ret;

fail_clk_smi:
	clk_unprepare(smipriv->clk_apb);
	return ret;
}

static const struct of_device_id mtk_smi_of_ids[] = {
	{ .compatible = "mediatek,mt8173-smi",
	},
	{}
};

static struct platform_driver mtk_smi_driver = {
	.probe	= mtk_smi_probe,
	.driver	= {
		.name = "mtk-smi",
		.of_match_table = mtk_smi_of_ids,
	}
};

static int __init mtk_smi_init(void)
{
	int ret;

	ret = platform_driver_register(&mtk_smi_driver);
	if (ret != 0) {
		pr_err("Failed to register SMI driver\n");
		return ret;
	}

	ret = platform_driver_register(&mtk_smi_larb_driver);
	if (ret != 0) {
		pr_err("Failed to register SMI-LARB driver\n");
		goto fail_smi_larb;
	}
	return ret;

fail_smi_larb:
	platform_driver_unregister(&mtk_smi_driver);
	return ret;
}

subsys_initcall(mtk_smi_init);


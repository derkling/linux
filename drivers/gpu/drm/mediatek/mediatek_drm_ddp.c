/*
 * Copyright (c) 2015 MediaTek Inc.
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

#include <drm/drmP.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/component.h>

#include "mediatek_drm_crtc.h"
#include "mediatek_drm_ddp.h"


#define DISP_REG_CONFIG_DISP_OVL0_MOUT_EN	0x040
#define DISP_REG_CONFIG_DISP_OVL1_MOUT_EN	0x044
#define DISP_REG_CONFIG_DISP_OD_MOUT_EN		0x048
#define DISP_REG_CONFIG_DISP_GAMMA_MOUT_EN	0x04c
#define DISP_REG_CONFIG_DISP_UFOE_MOUT_EN	0x050
#define DISP_REG_CONFIG_DISP_COLOR0_SEL_IN	0x084
#define DISP_REG_CONFIG_DISP_COLOR1_SEL_IN	0x088
#define DISP_REG_CONFIG_DPI_SEL_IN		0x0ac
#define DISP_REG_CONFIG_DISP_PATH1_SOUT_SEL_IN	0x0c8
#define DISP_REG_CONFIG_MMSYS_CG_CON0		0x100

#define DISP_REG_CONFIG_MUTEX_EN(n)      (0x20 + 0x20 * n)
#define DISP_REG_CONFIG_MUTEX_MOD(n)     (0x2c + 0x20 * n)
#define DISP_REG_CONFIG_MUTEX_SOF(n)     (0x30 + 0x20 * n)

#define MUTEX_MOD_OVL0		11
#define MUTEX_MOD_RDMA0		13
#define MUTEX_MOD_COLOR0	18
#define MUTEX_MOD_AAL		20
#define MUTEX_MOD_UFOE		22
#define MUTEX_MOD_PWM0		23
#define MUTEX_MOD_OD		25

#define MUTEX_SOF_DSI0		1

#define OVL0_MOUT_EN_COLOR0	0x1
#define OD_MOUT_EN_RDMA0	0x1
#define UFOE_MOUT_EN_DSI0	0x1
#define COLOR0_SEL_IN_OVL0	0x1

struct ddp_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct mediatek_drm_crtc	*crtc;
	int				pipe;

	struct clk			*mutex_disp_clk;

	void __iomem			*config_regs;
	void __iomem			*mutex_regs;

	bool				pending_ovl_config;
	bool				pending_ovl_enable;
	unsigned int			pending_ovl_addr;
	unsigned int			pending_ovl_width;
	unsigned int			pending_ovl_height;
	unsigned int			pending_ovl_pitch;
	unsigned int			pending_ovl_format;

	bool pending_ovl_cursor_config;
	bool pending_ovl_cursor_enable;
	unsigned int pending_ovl_cursor_addr;
	int pending_ovl_cursor_x;
	int pending_ovl_cursor_y;
};


static void disp_config_main_path_connection(void __iomem *disp_base)
{
	writel(OVL0_MOUT_EN_COLOR0,
		disp_base + DISP_REG_CONFIG_DISP_OVL0_MOUT_EN);
	writel(OD_MOUT_EN_RDMA0, disp_base + DISP_REG_CONFIG_DISP_OD_MOUT_EN);
	writel(UFOE_MOUT_EN_DSI0,
		disp_base + DISP_REG_CONFIG_DISP_UFOE_MOUT_EN);
	writel(COLOR0_SEL_IN_OVL0,
		disp_base + DISP_REG_CONFIG_DISP_COLOR0_SEL_IN);
}

static void disp_config_main_path_mutex(void __iomem *mutex_base)
{
	unsigned int id = 0;

	writel((1 << MUTEX_MOD_OVL0 | 1 << MUTEX_MOD_RDMA0 |
		1 << MUTEX_MOD_COLOR0 | 1 << MUTEX_MOD_AAL |
		1 << MUTEX_MOD_UFOE | 1 << MUTEX_MOD_PWM0 |
		1 << MUTEX_MOD_OD),
		mutex_base + DISP_REG_CONFIG_MUTEX_MOD(id));

	writel(MUTEX_SOF_DSI0, mutex_base + DISP_REG_CONFIG_MUTEX_SOF(id));
	writel(1, mutex_base + DISP_REG_CONFIG_MUTEX_EN(id));
}

void mediatek_ddp_main_path_setup(struct device *dev)
{
	struct ddp_context *ddp = dev_get_drvdata(dev);

	disp_config_main_path_connection(ddp->config_regs);
	disp_config_main_path_mutex(ddp->mutex_regs);
}

void mediatek_ddp_clock_on(struct device *dev)
{
	struct ddp_context *ddp = dev_get_drvdata(dev);
	int ret;

	/* disp_mtcmos */
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		DRM_ERROR("failed to get_sync(%d)\n", ret);

	ret = clk_prepare_enable(ddp->mutex_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare_enable(mutex_disp_clk) error!\n");
}

void mediatek_ddp_clock_off(struct device *dev)
{
	struct ddp_context *ddp = dev_get_drvdata(dev);

	clk_disable_unprepare(ddp->mutex_disp_clk);

	/* disp_mtcmos */
	pm_runtime_put_sync(dev);
}

static int ddp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ddp_context *ddp;
	struct resource *regs;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	ddp = devm_kzalloc(dev, sizeof(*ddp), GFP_KERNEL);
	if (!ddp)
		return -ENOMEM;

	ddp->mutex_disp_clk = devm_clk_get(dev, "mutex_disp");
	if (IS_ERR(ddp->mutex_disp_clk)) {
		dev_err(dev, "ddp_probe: Get mutex_disp_clk fail.\n");
		ret = PTR_ERR(ddp->mutex_disp_clk);
		goto err;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ddp->config_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ddp->config_regs))
		return PTR_ERR(ddp->config_regs);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ddp->mutex_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ddp->mutex_regs))
		return PTR_ERR(ddp->mutex_regs);

	platform_set_drvdata(pdev, ddp);

	pm_runtime_enable(dev);

	return 0;

err:

	return ret;
}

static int ddp_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id ddp_driver_dt_match[] = {
	{ .compatible = "mediatek,mt8173-ddp" },
	{},
};
MODULE_DEVICE_TABLE(of, ddp_driver_dt_match);

struct platform_driver mediatek_ddp_driver = {
	.probe		= ddp_probe,
	.remove		= ddp_remove,
	.driver		= {
		.name	= "mediatek-ddp",
		.owner	= THIS_MODULE,
		.of_match_table = ddp_driver_dt_match,
	},
};


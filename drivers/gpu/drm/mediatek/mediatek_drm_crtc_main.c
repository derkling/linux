/*
 * Copyright (c) 2015 MediaTek Inc.
 * Authors:
 *	YT Shen <yt.shen@mediatek.com>
 *	CK Hu <ck.hu@mediatek.com>
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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/component.h>
#include <linux/pm_runtime.h>
#include "mediatek_drm_drv.h"
#include "mediatek_drm_crtc.h"
#include "mediatek_drm_gem.h"
#include "mediatek_drm_ddp.h"
#include "mediatek_drm_ddp_comp.h"


struct crtc_main_context {
	struct device		*dev;
	struct drm_device	*drm_dev;
	struct mtk_drm_crtc	*crtc;
	int	pipe;

	struct device		*ddp_dev;
	struct clk		*ovl0_disp_clk;
	struct clk		*rdma0_disp_clk;
	struct clk		*color0_disp_clk;
	struct clk		*aal_disp_clk;
	struct clk		*ufoe_disp_clk;
	struct clk		*od_disp_clk;

	void __iomem		*ovl0_regs;
	void __iomem		*rdma0_regs;
	void __iomem		*color0_regs;
	void __iomem		*aal_regs;
	void __iomem		*ufoe_regs;
	void __iomem		*od_regs;

	bool			pending_ovl_config;
	bool			pending_ovl_enable;
	unsigned int		pending_ovl_addr;
	unsigned int		pending_ovl_width;
	unsigned int		pending_ovl_height;
	unsigned int		pending_ovl_pitch;
	unsigned int		pending_ovl_format;
};


static int crtc_main_ctx_initialize(struct crtc_main_context *ctx,
	struct drm_device *drm_dev)
{
	struct mtk_drm_private *priv;

	priv = drm_dev->dev_private;
	ctx->drm_dev = drm_dev;
	ctx->pipe = priv->pipe++;

	return 0;
}

static void crtc_main_ctx_remove(struct crtc_main_context *ctx)
{
}

static void crtc_main_power_on(struct crtc_main_context *ctx)
{
	int ret;

	DRM_INFO("mediatek_ddp_clock_on\n");
	ret = clk_enable(ctx->ovl0_disp_clk);
	if (ret != 0) {
		DRM_ERROR("clk_enable(ctx->ovl0_disp_clk) error!\n");
		goto err_ovl0;
	}

	ret = clk_enable(ctx->rdma0_disp_clk);
	if (ret != 0) {
		DRM_ERROR("clk_enable(ctx->rdma0_disp_clk) error!\n");
		goto err_rdma0;
	}

	ret = clk_enable(ctx->color0_disp_clk);
	if (ret != 0) {
		DRM_ERROR("clk_enable(ctx->color0_disp_clk) error!\n");
		goto err_color0;
	}

	ret = clk_enable(ctx->aal_disp_clk);
	if (ret != 0) {
		DRM_ERROR("clk_enable(ctx->aal_disp_clk) error!\n");
		goto err_aal;
	}

	ret = clk_enable(ctx->ufoe_disp_clk);
	if (ret != 0) {
		DRM_ERROR("clk_enable(ctx->ufoe_disp_clk) error!\n");
		goto err_ufoe;
	}

	ret = clk_enable(ctx->od_disp_clk);
	if (ret != 0) {
		DRM_ERROR("clk_enable(ctx->od_disp_clk) error!\n");
		goto err_od;
	}

	return;
err_od:
	clk_disable(ctx->ufoe_disp_clk);
err_ufoe:
	clk_disable(ctx->aal_disp_clk);
err_aal:
	clk_disable(ctx->color0_disp_clk);
err_color0:
	clk_disable(ctx->rdma0_disp_clk);
err_rdma0:
	clk_disable(ctx->ovl0_disp_clk);
err_ovl0:
	return;
}

static void crtc_main_power_off(struct crtc_main_context *ctx)
{
	clk_disable_unprepare(ctx->ovl0_disp_clk);

	clk_disable_unprepare(ctx->rdma0_disp_clk);

	clk_disable_unprepare(ctx->color0_disp_clk);

	clk_disable_unprepare(ctx->aal_disp_clk);

	clk_disable_unprepare(ctx->ufoe_disp_clk);

	clk_disable_unprepare(ctx->od_disp_clk);

	mediatek_ddp_clock_off(ctx->ddp_dev);
	DRM_INFO("mediatek_ddp_clock_off\n");
}

static void crtc_main_dpms(struct mtk_drm_crtc *crtc, int mode)
{
	/* DRM_DEBUG_KMS("%s, %d\n", __FILE__, mode); */

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		crtc_main_power_on(crtc->ctx);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		crtc_main_power_off(crtc->ctx);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}
}

static int crtc_main_enable_vblank(struct mtk_drm_crtc *crtc)
{
	struct crtc_main_context *ctx = (struct crtc_main_context *)crtc->ctx;

	mediatek_od_enable_vblank(ctx->od_regs);

	return 0;
}

static void crtc_main_disable_vblank(struct mtk_drm_crtc *crtc)
{
	struct crtc_main_context *ctx = (struct crtc_main_context *)crtc->ctx;

	mediatek_od_disable_vblank(ctx->od_regs);
}

static void crtc_main_ovl_layer_config(struct mtk_drm_crtc *crtc,
	bool enable, dma_addr_t addr)
{
	struct crtc_main_context *ctx = (struct crtc_main_context *)crtc->ctx;
	unsigned int pitch = 0;

	if (crtc->base.primary->fb && crtc->base.primary->fb->pitches[0])
		pitch = crtc->base.primary->fb->pitches[0];

	ctx->pending_ovl_enable = enable;
	if (enable) {
		ctx->pending_ovl_addr = addr;
		ctx->pending_ovl_width = crtc->base.mode.hdisplay;
		ctx->pending_ovl_height = crtc->base.mode.vdisplay;
		ctx->pending_ovl_pitch = pitch;
		ctx->pending_ovl_format = crtc->base.primary->fb->pixel_format;
	}
	ctx->pending_ovl_config = true;
}

static const struct mediatek_drm_crtc_ops crtc_main_crtc_ops = {
	.dpms = crtc_main_dpms,
	.enable_vblank = crtc_main_enable_vblank,
	.disable_vblank = crtc_main_disable_vblank,
	.ovl_layer_config = crtc_main_ovl_layer_config,
};

static void crtc_main_irq(struct crtc_main_context *ctx)
{
	struct drm_device *dev = ctx->drm_dev;
	struct mtk_drm_crtc *mtk_crtc = ctx->crtc;
	unsigned long flags;

	if (ctx->pending_ovl_config) {
		ctx->pending_ovl_config = false;
		mediatek_ovl_layer_config(ctx->ovl0_regs,
			ctx->pending_ovl_enable,
			ctx->pending_ovl_addr,
			ctx->pending_ovl_width,
			ctx->pending_ovl_height,
			ctx->pending_ovl_pitch,
			ctx->pending_ovl_format);
	}

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	spin_lock_irqsave(&dev->event_lock, flags);
	if (mtk_crtc->pending_needs_vblank) {
		mtk_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

irqreturn_t crtc_main_irq_handler(int irq, void *dev_id)
{
	struct crtc_main_context *ctx = (struct crtc_main_context *)dev_id;

	mediatek_od_clear_vblank(ctx->od_regs);

	if (ctx->pipe < 0 || !ctx->drm_dev)
		goto out;

	crtc_main_irq(ctx);
out:
	return IRQ_HANDLED;
}

static int crtc_main_bind(struct device *dev, struct device *master, void *data)
{
	struct crtc_main_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct device_node *node;
	unsigned int width, height;
	int ret;

	ret = crtc_main_ctx_initialize(ctx, drm_dev);
	if (ret) {
		DRM_ERROR("crtc_main_ctx_initialize failed.\n");
		return ret;
	}

	ctx->crtc = mtk_drm_crtc_create(drm_dev, ctx->pipe,
		&crtc_main_crtc_ops, ctx);

	if (IS_ERR(ctx->crtc)) {
		crtc_main_ctx_remove(ctx);
		return PTR_ERR(ctx->crtc);
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt8173-dsi");

	ret = of_property_read_u32(node, "mediatek,width", &width);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(node, "mediatek,height", &height);
	if (ret < 0)
		return ret;

	width = ((width + 3)>>2)<<2;

	DRM_INFO("mediatek_ddp_clock_on\n");
	mediatek_ddp_clock_on(ctx->ddp_dev);

	DRM_INFO("mediatek_ddp_main_path_setup\n");
	mediatek_ddp_main_path_setup(ctx->ddp_dev);

	DRM_INFO("main_disp_path_power_on\n");
	main_disp_path_power_on(width, height, ctx->ovl0_regs, ctx->rdma0_regs,
		ctx->color0_regs, ctx->ufoe_regs, ctx->od_regs);

	return 0;
}

static void crtc_main_unbind(struct device *dev, struct device *master,
			void *data)
{
	struct crtc_main_context *ctx = dev_get_drvdata(dev);

	crtc_main_ctx_remove(ctx);
}

static const struct component_ops crtc_main_component_ops = {
	.bind	= crtc_main_bind,
	.unbind = crtc_main_unbind,
};

static int crtc_main_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct crtc_main_context *ctx;
	struct device_node *node;
	struct platform_device *ddp_pdev;
	struct resource *regs;
	int irq;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	node = of_parse_phandle(dev->of_node, "ddp", 0);
	if (!node) {
		dev_err(dev, "crtc_main_probe: Get ddp node fail.\n");
		ret = -EINVAL;
		goto err;
	}

	ddp_pdev = of_find_device_by_node(node);
	if (WARN_ON(!ddp_pdev)) {
		dev_err(dev, "crtc_main_probe: Find ddp device fail.\n");
		ret = -EINVAL;
		goto err;
	}
	ctx->ddp_dev = &ddp_pdev->dev;

	ctx->ovl0_disp_clk = devm_clk_get(dev, "ovl0_disp");
	if (IS_ERR(ctx->ovl0_disp_clk)) {
		dev_err(dev, "crtc_main_probe: Get ovl0_disp_clk fail.\n");
		ret = PTR_ERR(ctx->ovl0_disp_clk);
		goto err;
	}

	ctx->rdma0_disp_clk = devm_clk_get(dev, "rdma0_disp");
	if (IS_ERR(ctx->rdma0_disp_clk)) {
		dev_err(dev, "crtc_main_probe: Get rdma0_disp_clk.\n");
		ret = PTR_ERR(ctx->rdma0_disp_clk);
		goto err;
	}

	ctx->color0_disp_clk = devm_clk_get(dev, "color0_disp");
	if (IS_ERR(ctx->color0_disp_clk)) {
		dev_err(dev, "crtc_main_probe: Get color0_disp_clk fail.\n");
		ret = PTR_ERR(ctx->color0_disp_clk);
		goto err;
	}

	ctx->aal_disp_clk = devm_clk_get(dev, "aal_disp");
	if (IS_ERR(ctx->aal_disp_clk)) {
		dev_err(dev, "crtc_main_probe: Get aal_disp_clk fail.\n");
		ret = PTR_ERR(ctx->aal_disp_clk);
		goto err;
	}

	ctx->ufoe_disp_clk = devm_clk_get(dev, "ufoe_disp");
	if (IS_ERR(ctx->ufoe_disp_clk)) {
		dev_err(dev, "crtc_main_probe: Get ufoe_disp_clk fail.\n");
		ret = PTR_ERR(ctx->ufoe_disp_clk);
		goto err;
	}

	ctx->od_disp_clk = devm_clk_get(dev, "od_disp");
	if (IS_ERR(ctx->od_disp_clk)) {
		dev_err(dev, "crtc_main_probe: Get od_disp_clk fail.\n");
		ret = PTR_ERR(ctx->od_disp_clk);
		goto err;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->ovl0_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ctx->ovl0_regs))
		return PTR_ERR(ctx->ovl0_regs);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ctx->rdma0_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ctx->rdma0_regs))
		return PTR_ERR(ctx->rdma0_regs);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	ctx->color0_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ctx->color0_regs))
		return PTR_ERR(ctx->color0_regs);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	ctx->aal_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ctx->aal_regs))
		return PTR_ERR(ctx->aal_regs);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	ctx->ufoe_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ctx->ufoe_regs))
		return PTR_ERR(ctx->ufoe_regs);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 5);
	ctx->od_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ctx->od_regs))
		return PTR_ERR(ctx->od_regs);

	ret = clk_prepare(ctx->ovl0_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare(ctx->ovl0_disp_clk) error!\n");

	ret = clk_prepare(ctx->rdma0_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare(ctx->rdma0_disp_clk) error!\n");

	ret = clk_prepare(ctx->color0_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare(ctx->color0_disp_clk) error!\n");

	ret = clk_prepare(ctx->aal_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare(ctx->aal_disp_clk) error!\n");

	ret = clk_prepare(ctx->ufoe_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare(ctx->ufoe_disp_clk) error!\n");

	ret = clk_prepare(ctx->od_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare(ctx->od_disp_clk) error!\n");

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(dev, irq, crtc_main_irq_handler,
		IRQF_TRIGGER_NONE, dev_name(dev), ctx);
	if (ret < 0) {
		dev_err(dev, "devm_request_irq %d fail %d\n", irq, ret);
		ret = -ENXIO;
		goto err;
	}

	platform_set_drvdata(pdev, ctx);

	ret = component_add(&pdev->dev, &crtc_main_component_ops);
	if (ret)
		goto err;

	return 0;

err:
	if (node)
		of_node_put(node);

	return ret;
}

static int crtc_main_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &crtc_main_component_ops);

	return 0;
}

static const struct of_device_id crtc_main_driver_dt_match[] = {
	{ .compatible = "mediatek,mt8173-crtc-main" },
	{},
};
MODULE_DEVICE_TABLE(of, crtc_main_driver_dt_match);

struct platform_driver mediatek_crtc_main_driver = {
	.probe		= crtc_main_probe,
	.remove		= crtc_main_remove,
	.driver		= {
		.name	= "mediatek-crtc-main",
		.owner	= THIS_MODULE,
		.of_match_table = crtc_main_driver_dt_match,
	},
};


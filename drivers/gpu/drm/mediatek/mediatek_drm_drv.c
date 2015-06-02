/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: YT SHEN <yt.shen@mediatek.com>
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
#include <drm/drm_gem.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <linux/of_platform.h>
#include <linux/component.h>
#include <linux/mtk-smi.h>
#include <linux/pm_runtime.h>
#include <linux/dma-iommu.h>

#include "mediatek_drm_drv.h"
#include "mediatek_drm_crtc.h"
#include "mediatek_drm_fb.h"
#include "mediatek_drm_gem.h"


#define DRIVER_NAME "mediatek"
#define DRIVER_DESC "Mediatek SoC DRM"
#define DRIVER_DATE "20150513"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

static struct drm_mode_config_funcs mediatek_drm_mode_config_funcs = {
	.fb_create = mtk_drm_mode_fb_create,
};

static int mtk_drm_kms_init(struct drm_device *dev)
{
	struct device_node *node;
	struct platform_device *pdev;
	int err;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 640;
	dev->mode_config.min_height = 480;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;
	dev->mode_config.funcs = &mediatek_drm_mode_config_funcs;

	err = component_bind_all(dev->dev, dev);
	if (err)
		goto err_crtc;

	/*
	 * We don't use the drm_irq_install() helpers provided by the DRM
	 * core, so we need to set this manually in order to allow the
	 * DRM_IOCTL_WAIT_VBLANK to operate correctly.
	 */
	dev->irq_enabled = true;
	err = drm_vblank_init(dev, MAX_CRTC);
	if (err < 0)
		goto err_crtc;

	drm_kms_helper_poll_init(dev);

	node = of_parse_phandle(dev->dev->of_node, "larb", 0);
	if (!node)
		return 0;

	pdev = of_find_device_by_node(node);
	if (WARN_ON(!pdev)) {
		of_node_put(node);
		return -EINVAL;
	}

	err = mtk_smi_larb_get(&pdev->dev);
	if (err)
		DRM_ERROR("mtk_smi_larb_get fail %d\n", err);

	node = of_parse_phandle(dev->dev->of_node, "larb", 1);
	if (!node)
		return 0;

	pdev = of_find_device_by_node(node);
	if (WARN_ON(!pdev)) {
		of_node_put(node);
		return -EINVAL;
	}

	err = mtk_smi_larb_get(&pdev->dev);
	if (err)
		DRM_ERROR("mtk_smi_larb_get fail %d\n", err);

	return 0;
err_crtc:
	drm_mode_config_cleanup(dev);

	return err;
}

static int mtk_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct mtk_drm_private *priv;

	priv = devm_kzalloc(dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev->dev_private = priv;
	platform_set_drvdata(dev->platformdev, dev);

	return mtk_drm_kms_init(dev);
}

static void mtk_drm_kms_deinit(struct drm_device *dev)
{
	drm_kms_helper_poll_fini(dev);

	drm_vblank_cleanup(dev);
	drm_mode_config_cleanup(dev);

	pm_runtime_disable(dev->dev);
}

static int mtk_drm_unload(struct drm_device *dev)
{
	mtk_drm_kms_deinit(dev);
	dev->dev_private = NULL;

	return 0;
}

static int mtk_drm_open(struct drm_device *drm, struct drm_file *filp)
{
	return 0;
}

static void mediatek_drm_preclose(struct drm_device *drm, struct drm_file *file)
{
}

static void mediatek_drm_lastclose(struct drm_device *drm)
{
}

static const struct vm_operations_struct mediatek_drm_gem_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations mediatek_drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = mtk_drm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
};

static struct drm_driver mediatek_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM,
	.load = mtk_drm_load,
	.unload = mtk_drm_unload,
	.open = mtk_drm_open,
	.preclose = mediatek_drm_preclose,
	.lastclose = mediatek_drm_lastclose,
	.set_busid = drm_platform_set_busid,

	.get_vblank_counter = drm_vblank_count,
	.enable_vblank = mtk_drm_crtc_enable_vblank,
	.disable_vblank = mtk_drm_crtc_disable_vblank,

	.gem_free_object = mtk_drm_gem_free_object,
	.gem_vm_ops = &mediatek_drm_gem_vm_ops,
	.dumb_create = mtk_drm_gem_dumb_create,
	.dumb_map_offset = mtk_drm_gem_dumb_map_offset,
	.dumb_destroy = drm_gem_dumb_destroy,

	.num_ioctls = 0,
	.fops = &mediatek_drm_fops,

	.set_busid = drm_platform_set_busid,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
};

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static int mtk_drm_add_components(struct device *master, struct master *m)
{
	struct device_node *np = master->of_node;
	unsigned i;
	int ret;

	for (i = 0; i < MAX_CONNECTOR; i++) {
		struct device_node *node;

		node = of_parse_phandle(np, "connectors", i);
		if (!node)
			break;

		ret = component_master_add_child(m, compare_of, node);
		of_node_put(node);
		if (ret) {
			dev_err(master, "component_master_add_child %s fail.\n",
				node->full_name);
			return ret;
		}
	}

	for (i = 0; i < MAX_CRTC; i++) {
		struct device_node *node;

		node = of_parse_phandle(np, "crtcs", i);
		if (!node)
			break;

		ret = component_master_add_child(m, compare_of, node);
		of_node_put(node);
		if (ret) {
			dev_err(master, "component_master_add_child %s fail.\n",
				node->full_name);
			return ret;
		}
	}

	return 0;
}

static int mtk_drm_bind(struct device *dev)
{
	return drm_platform_init(&mediatek_drm_driver, to_platform_device(dev));
}

static void mtk_drm_unbind(struct device *dev)
{
	drm_put_dev(platform_get_drvdata(to_platform_device(dev)));
}

static const struct component_master_ops mtk_drm_ops = {
	.add_components	= mtk_drm_add_components,
	.bind		= mtk_drm_bind,
	.unbind		= mtk_drm_unbind,
};

static int mtk_drm_probe(struct platform_device *pdev)
{
	component_master_add(&pdev->dev, &mtk_drm_ops);

	return 0;
}

static int mtk_drm_remove(struct platform_device *pdev)
{
	drm_put_dev(platform_get_drvdata(pdev));

	return 0;
}

static const struct of_device_id mediatek_drm_of_ids[] = {
	{ .compatible = "mediatek,mt8173-drm", },
	{ }
};

static struct platform_driver mediatek_drm_platform_driver = {
	.probe	= mtk_drm_probe,
	.remove	= mtk_drm_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "mediatek-drm",
		.of_match_table = mediatek_drm_of_ids,
		/*.pm     = &mtk_pm_ops, */
	},
	/* .id_table = mtk_drm_platform_ids, */
};

static int mediatek_drm_init(void)
{
	int err;

	err = platform_driver_register(&mediatek_ddp_driver);
	if (err < 0) {
		DRM_DEBUG_DRIVER("register ddp driver fail.\n");
		return err;
	}

	err = platform_driver_register(&mediatek_drm_platform_driver);
	if (err < 0)
		return err;

	return 0;
}

static void mediatek_drm_exit(void)
{
	platform_driver_unregister(&mediatek_drm_platform_driver);
	platform_driver_unregister(&mediatek_ddp_driver);
}

module_init(mediatek_drm_init);
module_exit(mediatek_drm_exit);

MODULE_AUTHOR("YT SHEN <yt.shen@mediatek.com>");
MODULE_DESCRIPTION("Mediatek SoC DRM driver");
MODULE_LICENSE("GPL");


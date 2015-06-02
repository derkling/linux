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


#ifndef MEDIATEL_DRM_CRTC_H
#define MEDIATEL_DRM_CRTC_H


struct mtk_drm_crtc;
struct mediatek_drm_crtc_ops {
	void (*dpms)(struct mtk_drm_crtc *crtc, int mode);
	int (*enable_vblank)(struct mtk_drm_crtc *crtc);
	void (*disable_vblank)(struct mtk_drm_crtc *crtc);
	void (*ovl_layer_config)(struct mtk_drm_crtc *crtc,
		bool enable, dma_addr_t addr);
	void (*ovl_layer_config_cursor)(struct mtk_drm_crtc *crtc,
		bool enable, dma_addr_t addr);
};

/*
 * MediaTek specific crtc structure.
 *
 * @base: crtc object.
 * @pipe: a crtc index created at load() with a new crtc object creation
 *	and the crtc object would be set to private->crtc array
 *	to get a crtc object corresponding to this pipe from private->crtc
 *	array when irq interrupt occurred. the reason of using this pipe is that
 *	drm framework doesn't support multiple irq yet.
 *	we can refer to the crtc to current hardware interrupt occurred through
 *	this pipe value.
 */
struct mtk_drm_crtc {
	struct drm_crtc			base;

	unsigned int			pipe;
	struct drm_pending_vblank_event	*event;
	struct mtk_drm_gem_obj *flip_obj;
	const struct mediatek_drm_crtc_ops	*ops;
	void				*ctx;
	bool pending_needs_vblank;

	bool pending_ovl_config;
	bool pending_ovl_enabled;
	unsigned int pending_ovl_addr;
	unsigned int pending_ovl_width;
	unsigned int pending_ovl_height;
	unsigned int pending_ovl_pitch;
	unsigned int pending_ovl_format;

};

#define to_mtk_crtc(x) container_of(x, struct mtk_drm_crtc, base)

struct mtk_drm_crtc *mtk_drm_crtc_create(
		struct drm_device *drm_dev, int pipe,
		const struct mediatek_drm_crtc_ops *ops, void *ctx);
void mtk_drm_crtc_irq(struct mtk_drm_crtc *mtk_crtc);

void mtk_crtc_finish_page_flip(struct mtk_drm_crtc *mtk_crtc);
int mtk_drm_crtc_enable_vblank(struct drm_device *drm, int pipe);
void mtk_drm_crtc_disable_vblank(struct drm_device *drm, int pipe);

#endif /* MEDIATEL_DRM_CRTC_H */


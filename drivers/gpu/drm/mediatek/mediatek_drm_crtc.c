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
#include <drm/drm_gem.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <linux/dma-buf.h>
#include <linux/reservation.h>

#include "mediatek_drm_drv.h"
#include "mediatek_drm_crtc.h"
#include "mediatek_drm_fb.h"
#include "mediatek_drm_gem.h"


void mtk_crtc_finish_page_flip(struct mtk_drm_crtc *mtk_crtc)
{
	struct drm_device *dev = mtk_crtc->base.dev;

	drm_send_vblank_event(dev, mtk_crtc->event->pipe, mtk_crtc->event);
	drm_crtc_vblank_put(&mtk_crtc->base);
	mtk_crtc->event = NULL;
}

static void mediatek_drm_crtc_pending_ovl_config(struct mtk_drm_crtc *mtk_crtc,
		bool enable, unsigned int addr)
{
	if (mtk_crtc->ops && mtk_crtc->ops->ovl_layer_config)
		mtk_crtc->ops->ovl_layer_config(mtk_crtc, enable, addr);
}

static void mediatek_drm_crtc_pending_ovl_cursor_config(
		struct mtk_drm_crtc *mtk_crtc, bool enable, unsigned int addr)
{
	if (mtk_crtc->ops && mtk_crtc->ops->ovl_layer_config_cursor)
		mtk_crtc->ops->ovl_layer_config_cursor(mtk_crtc, enable, addr);
}

static int mtk_drm_crtc_page_flip(struct drm_crtc *crtc,
		struct drm_framebuffer *fb,
		struct drm_pending_vblank_event *event,
		uint32_t page_flip_flags)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);
	struct drm_device *dev = crtc->dev;
	unsigned long flags;
	bool busy;
	int ret;

	spin_lock_irqsave(&dev->event_lock, flags);
	busy = !!mtk_crtc->event;
	if (!busy)
		mtk_crtc->event = event;
	spin_unlock_irqrestore(&dev->event_lock, flags);
	if (busy)
		return -EBUSY;

	if (fb->width != crtc->mode.hdisplay ||
		fb->height != crtc->mode.vdisplay) {
		DRM_ERROR("mtk_drm_crtc_page_flip width/height not match !!\n");
		return -EINVAL;
	}

	if (event) {
		ret = drm_crtc_vblank_get(crtc);
		if (ret) {
			DRM_ERROR("failed to acquire vblank events\n");
			return ret;
		}
	}

	/*
	 * the values related to a buffer of the drm framebuffer
	 * to be applied should be set at here. because these values
	 * first, are set to shadow registers and then to
	 * real registers at vsync front porch period.
	 */
	crtc->primary->fb = fb;
	mtk_crtc->flip_buffer = to_mtk_gem_obj(mtk_fb->gem_obj[0])->buffer;

	mediatek_drm_crtc_pending_ovl_config(mtk_crtc, true,
		mtk_crtc->flip_buffer->mva_addr);

	spin_lock_irqsave(&dev->event_lock, flags);
	if (mtk_crtc->event)
		mtk_crtc->pending_needs_vblank = true;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	return ret;
}

static void mtk_drm_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static void mtk_drm_crtc_prepare(struct drm_crtc *crtc)
{
	/* drm framework doesn't check NULL. */
}

static void mtk_drm_crtc_commit(struct drm_crtc *crtc)
{
	/*
	 * when set_crtc is requested from user or at booting time,
	 * crtc->commit would be called without dpms call so if dpms is
	 * no power on then crtc->dpms should be called
	 * with DRM_MODE_DPMS_ON for the hardware power to be on.
	 */
}

static bool mtk_drm_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	/* drm framework doesn't check NULL */
	return true;
}

static int mtk_drm_crtc_mode_set(struct drm_crtc *crtc,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode,
		int x, int y, struct drm_framebuffer *old_fb)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_framebuffer *fb;
	struct mtk_drm_fb *mtk_fb;
	struct mtk_drm_gem_buf *buffer;

	fb = crtc->primary->fb;
	mtk_fb = to_mtk_fb(fb);

	buffer = to_mtk_gem_obj(mtk_fb->gem_obj[0])->buffer;

	mediatek_drm_crtc_pending_ovl_config(mtk_crtc, true,
		buffer->mva_addr);
	/*
	 * copy the mode data adjusted by mode_fixup() into crtc->mode
	 * so that hardware can be seet to proper mode.
	 */
	memcpy(&crtc->mode, adjusted_mode, sizeof(*adjusted_mode));

	/* Take a reference to the new fb as we're using it */
	drm_framebuffer_reference(crtc->primary->fb);

	return 0;
}

int mtk_drm_crtc_enable_vblank(struct drm_device *drm, int pipe)
{
	struct mtk_drm_private *priv =
		(struct mtk_drm_private *)drm->dev_private;
	struct mtk_drm_crtc *mtk_crtc;

	if (pipe >= MAX_CRTC || pipe < 0) {
		DRM_ERROR(" - %s: invalid crtc (%d)\n", __func__, pipe);
		return -EINVAL;
	}

	mtk_crtc = to_mtk_crtc(priv->crtc[pipe]);

	if (mtk_crtc->ops->enable_vblank)
		mtk_crtc->ops->enable_vblank(mtk_crtc);

	return 0;
}

void mtk_drm_crtc_disable_vblank(struct drm_device *drm, int pipe)
{
	struct mtk_drm_private *priv =
		(struct mtk_drm_private *)drm->dev_private;
	struct mtk_drm_crtc *mtk_crtc;

	if (pipe >= MAX_CRTC || pipe < 0)
		DRM_ERROR(" - %s: invalid crtc (%d)\n", __func__, pipe);

	mtk_crtc = to_mtk_crtc(priv->crtc[pipe]);
	if (mtk_crtc->ops->disable_vblank)
		mtk_crtc->ops->disable_vblank(mtk_crtc);
}

static void mtk_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	DRM_INFO("mtk_drm_crtc_disable %d\n", crtc->base.id);

	mediatek_drm_crtc_pending_ovl_config(mtk_crtc, false, 0);
	mediatek_drm_crtc_pending_ovl_cursor_config(mtk_crtc, false, 0);
}

static const struct drm_crtc_funcs mediatek_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.page_flip	= mtk_drm_crtc_page_flip,
	.destroy	= mtk_drm_crtc_destroy,
};

static struct drm_crtc_helper_funcs mediatek_crtc_helper_funcs = {
	.prepare	= mtk_drm_crtc_prepare,
	.commit		= mtk_drm_crtc_commit,
	.mode_fixup	= mtk_drm_crtc_mode_fixup,
	.mode_set	= mtk_drm_crtc_mode_set,
	.disable	= mtk_drm_crtc_disable,
};

struct mtk_drm_crtc *mtk_drm_crtc_create(struct drm_device *drm_dev, int pipe,
		const struct mediatek_drm_crtc_ops *ops, void *ctx)
{
	struct mtk_drm_private *priv = drm_dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc;

	mtk_crtc = devm_kzalloc(drm_dev->dev, sizeof(*mtk_crtc), GFP_KERNEL);
	if (!mtk_crtc) {
		DRM_ERROR("failed to allocate mtk crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	mtk_crtc->pipe = pipe;
	mtk_crtc->ops = ops;
	mtk_crtc->ctx = ctx;

	priv->crtc[pipe] = &mtk_crtc->base;

	drm_crtc_init(drm_dev, &mtk_crtc->base, &mediatek_crtc_funcs);
	drm_crtc_helper_add(&mtk_crtc->base, &mediatek_crtc_helper_funcs);

	return mtk_crtc;
}


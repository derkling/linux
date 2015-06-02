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
#include <drm/drm_fb_helper.h>

#include "mediatek_drm_drv.h"
#include "mediatek_drm_fb.h"
#include "mediatek_drm_gem.h"


static int mtk_drm_fb_create_handle(struct drm_framebuffer *fb,
					struct drm_file *file_priv,
					unsigned int *handle)
{
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);

	return drm_gem_handle_create(file_priv, mtk_fb->gem_obj[0], handle);
}

static void mtk_drm_fb_destroy(struct drm_framebuffer *fb)
{
	unsigned int i;
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);
	struct drm_gem_object *gem;
	int nr = drm_format_num_planes(fb->pixel_format);

	drm_framebuffer_cleanup(fb);

	for (i = 0; i < nr; i++) {
		gem = mtk_fb->gem_obj[i];
		drm_gem_object_unreference_unlocked(gem);
	}
}

static struct drm_framebuffer_funcs mediatek_drm_fb_funcs = {
	.create_handle = mtk_drm_fb_create_handle,
	.destroy = mtk_drm_fb_destroy,
};

static struct mtk_drm_fb *mtk_drm_framebuffer_init(struct drm_device *dev,
			    struct drm_mode_fb_cmd2 *mode,
			    struct drm_gem_object **obj)
{
	struct mtk_drm_fb *mtk_fb;
	unsigned int i;
	int ret;

	mtk_fb = devm_kzalloc(dev->dev, sizeof(*mtk_fb), GFP_KERNEL);
	if (!mtk_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(&mtk_fb->base, mode);

	for (i = 0; i < drm_format_num_planes(mode->pixel_format); i++)
		mtk_fb->gem_obj[i] = obj[i];

	ret = drm_framebuffer_init(dev, &mtk_fb->base, &mediatek_drm_fb_funcs);
	if (ret) {
		DRM_ERROR("failed to initialize framebuffer\n");
		return ERR_PTR(ret);
	}

	return mtk_fb;
}

struct drm_framebuffer *mtk_drm_mode_fb_create(struct drm_device *dev,
					       struct drm_file *file,
					       struct drm_mode_fb_cmd2 *cmd)
{
	unsigned int hsub, vsub, i;
	struct mtk_drm_fb *mtk_fb;
	struct drm_gem_object *gem[MAX_FB_OBJ];
	int err;

	hsub = drm_format_horz_chroma_subsampling(cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(cmd->pixel_format);
	for (i = 0; i < drm_format_num_planes(cmd->pixel_format); i++) {
		unsigned int width = cmd->width / (i ? hsub : 1);
		unsigned int height = cmd->height / (i ? vsub : 1);
		unsigned int size, bpp;

		gem[i] = drm_gem_object_lookup(dev, file, cmd->handles[i]);
		if (!gem[i]) {
			err = -ENOENT;
			goto unreference;
		}

		bpp = drm_format_plane_cpp(cmd->pixel_format, i);
		size = (height - 1) * cmd->pitches[i] + width * bpp;
		size += cmd->offsets[i];

		if (gem[i]->size < size) {
			err = -EINVAL;
			goto unreference;
		}
	}

	mtk_fb = mtk_drm_framebuffer_init(dev, cmd, gem);

	return &mtk_fb->base;

unreference:
	while (i--)
		drm_gem_object_unreference_unlocked(gem[i]);

	return ERR_PTR(err);
}



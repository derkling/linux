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
#include <linux/clk.h>


#define DISP_REG_OVL_INTEN			0x0004
#define DISP_REG_OVL_INTSTA			0x0008
#define DISP_REG_OVL_EN				0x000c
#define DISP_REG_OVL_RST			0x0014
#define DISP_REG_OVL_ROI_SIZE			0x0020
#define DISP_REG_OVL_ROI_BGCLR			0x0028
#define DISP_REG_OVL_SRC_CON			0x002c
#define DISP_REG_OVL_L0_CON			0x0030
#define DISP_REG_OVL_L0_SRCKEY			0x0034
#define DISP_REG_OVL_L0_SRC_SIZE		0x0038
#define DISP_REG_OVL_L0_OFFSET			0x003c
#define DISP_REG_OVL_L0_PITCH			0x0044
#define DISP_REG_OVL_L1_CON			0x0050
#define DISP_REG_OVL_L1_SRCKEY			0x0054
#define DISP_REG_OVL_L1_SRC_SIZE		0x0058
#define DISP_REG_OVL_L1_OFFSET			0x005c
#define DISP_REG_OVL_L1_PITCH			0x0064
#define DISP_REG_OVL_RDMA0_CTRL			0x00c0
#define DISP_REG_OVL_RDMA0_MEM_GMC_SETTING	0x00c8
#define DISP_REG_OVL_RDMA1_CTRL			0x00e0
#define DISP_REG_OVL_RDMA1_MEM_GMC_SETTING	0x00e8
#define DISP_REG_OVL_L0_ADDR			0x0f40
#define DISP_REG_OVL_L1_ADDR			0x0f60

#define DISP_REG_RDMA_INT_ENABLE		0x0000
#define DISP_REG_RDMA_INT_STATUS		0x0004
#define DISP_REG_RDMA_GLOBAL_CON		0x0010
#define DISP_REG_RDMA_SIZE_CON_0		0x0014
#define DISP_REG_RDMA_SIZE_CON_1		0x0018
#define DISP_REG_RDMA_FIFO_CON			0x0040

#define DISP_OD_EN				0x0000
#define DISP_OD_INTEN				0x0008
#define DISP_OD_INTS				0x000c
#define DISP_OD_CFG				0x0020
#define DISP_OD_SIZE				0x0030

#define DISP_REG_UFO_START			0x0000

#define DISP_COLOR_CFG_MAIN			0x0400
#define DISP_COLOR_START			0x0c00

enum DISPLAY_PATH {
	PRIMARY_PATH = 0,
	EXTERNAL_PATH = 1,
};

enum RDMA_MODE {
	RDMA_MODE_DIRECT_LINK = 0,
	RDMA_MODE_MEMORY = 1,
};

enum RDMA_OUTPUT_FORMAT {
	RDMA_OUTPUT_FORMAT_ARGB = 0,
	RDMA_OUTPUT_FORMAT_YUV444 = 1,
};

#define OVL_COLOR_BASE 30
enum OVL_INPUT_FORMAT {
	OVL_INFMT_RGB565 = 0,
	OVL_INFMT_RGB888 = 1,
	OVL_INFMT_RGBA8888 = 2,
	OVL_INFMT_ARGB8888 = 3,
	OVL_INFMT_UYVY = 4,
	OVL_INFMT_YUYV = 5,
	OVL_INFMT_UNKNOWN = 16,

	OVL_INFMT_BGR565 = OVL_INFMT_RGB565 + OVL_COLOR_BASE,
	OVL_INFMT_BGR888 = OVL_INFMT_RGB888 + OVL_COLOR_BASE,
	OVL_INFMT_BGRA8888 = OVL_INFMT_RGBA8888 + OVL_COLOR_BASE,
	OVL_INFMT_ABGR8888 = OVL_INFMT_ARGB8888 + OVL_COLOR_BASE,
};

enum {
	OD_RELAY_MODE           = 0x1,
};

enum {
	UFO_BYPASS              = 0x4,
};

enum {
	COLOR_BYPASS_ALL        = (1UL<<7),
	COLOR_SEQ_SEL           = (1UL<<13),
};

enum {
	OVL_LAYER_SRC_DRAM      = 0,
};


static void mediatek_ovl_start(void __iomem *ovl_base)
{
	writel(0x01, ovl_base + DISP_REG_OVL_EN);
}

static void mediatek_ovl_roi(void __iomem *ovl_base,
		unsigned int w, unsigned int h, unsigned int bg_color)
{
	writel(h << 16 | w, ovl_base + DISP_REG_OVL_ROI_SIZE);
	writel(bg_color, ovl_base + DISP_REG_OVL_ROI_BGCLR);
}

void mediatek_ovl_layer_switch(void __iomem *ovl_base,
		unsigned layer, bool en)
{
	u32 reg;

	reg = readl(ovl_base + DISP_REG_OVL_SRC_CON);
	if (en)
		reg |= (1U<<layer);
	else
		reg &= ~(1U<<layer);

	writel(reg, ovl_base + DISP_REG_OVL_SRC_CON);
	writel(0x1, ovl_base + DISP_REG_OVL_RST);
	writel(0x0, ovl_base + DISP_REG_OVL_RST);
}

static unsigned int ovl_fmt_convert(unsigned int fmt)
{
	switch (fmt) {
	case DRM_FORMAT_RGB888:
		return OVL_INFMT_RGB888;
	case DRM_FORMAT_RGB565:
		return OVL_INFMT_RGB565;
	case DRM_FORMAT_ARGB8888:
		return OVL_INFMT_ARGB8888;
	case DRM_FORMAT_RGBA8888:
		return OVL_INFMT_RGBA8888;
	case DRM_FORMAT_BGR888:
		return OVL_INFMT_BGR888;
	case DRM_FORMAT_BGR565:
		return OVL_INFMT_BGR565;
	case DRM_FORMAT_ABGR8888:
		return OVL_INFMT_ABGR8888;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_BGRA8888:
		return OVL_INFMT_BGRA8888;
	case DRM_FORMAT_YUYV:
		return OVL_INFMT_YUYV;
	case DRM_FORMAT_UYVY:
		return OVL_INFMT_UYVY;
	default:
		return OVL_INFMT_UNKNOWN;
	}
}

void mediatek_ovl_layer_config(void __iomem *ovl_base, bool enabled,
		unsigned int addr, unsigned int width, unsigned int height,
		unsigned int pitch, unsigned int format)
{
	unsigned int reg;
	unsigned int dst_x = 0;
	unsigned int dst_y = 0;
	bool color_key_en = 1;
	unsigned int color_key = 0xFF000000;
	bool alpha_en = 0;
	unsigned char alpha = 0x0;
	unsigned int src_con, new_set;

	unsigned int rgb_swap, bpp;
	unsigned int fmt = ovl_fmt_convert(format);

	if (fmt == OVL_INFMT_BGR888 || fmt == OVL_INFMT_BGR565 ||
		fmt == OVL_INFMT_ABGR8888 || fmt == OVL_INFMT_BGRA8888) {
		fmt -= OVL_COLOR_BASE;
		rgb_swap = 1;
	} else {
		rgb_swap = 0;
	}

	switch (fmt) {
	case OVL_INFMT_ARGB8888:
	case OVL_INFMT_RGBA8888:
		bpp = 4;
		break;
	case OVL_INFMT_RGB888:
		bpp = 3;
		break;
	case OVL_INFMT_RGB565:
	case OVL_INFMT_YUYV:
	case OVL_INFMT_UYVY:
		bpp = 2;
		break;
	default:
		bpp = 1;
	}

	if (pitch == 0)
		pitch = width * bpp;

	src_con = readl(ovl_base + DISP_REG_OVL_SRC_CON);
	if (enabled == true)
		new_set = src_con | 0x1;
	else
		new_set = src_con & ~(0x1);

	writel(0x1, ovl_base + DISP_REG_OVL_RST);
	writel(0x0, ovl_base + DISP_REG_OVL_RST);

	writel(new_set, ovl_base + DISP_REG_OVL_SRC_CON);

	writel(0x00000001, ovl_base + DISP_REG_OVL_RDMA0_CTRL);
	writel(0x40402020, ovl_base + DISP_REG_OVL_RDMA0_MEM_GMC_SETTING);

	reg = color_key_en << 30 | OVL_LAYER_SRC_DRAM << 28 |
		rgb_swap << 25 | fmt << 12 | alpha_en << 8 | alpha;
	writel(reg, ovl_base + DISP_REG_OVL_L0_CON);
	writel(color_key, ovl_base + DISP_REG_OVL_L0_SRCKEY);
	writel(height << 16 | width, ovl_base + DISP_REG_OVL_L0_SRC_SIZE);
	writel(dst_y << 16 | dst_x, ovl_base + DISP_REG_OVL_L0_OFFSET);
	writel(addr, ovl_base + DISP_REG_OVL_L0_ADDR);
	writel(pitch & 0xFFFF, ovl_base + DISP_REG_OVL_L0_PITCH);
}

static void mediatek_rdma_start(void __iomem *rdma_base)
{
	unsigned int reg;

	writel(0x4, rdma_base + DISP_REG_RDMA_INT_ENABLE);
	reg = readl(rdma_base + DISP_REG_RDMA_GLOBAL_CON);
	reg |= 1;
	writel(reg, rdma_base + DISP_REG_RDMA_GLOBAL_CON);
}

static void mediatek_rdma_config_direct_link(void __iomem *rdma_base,
		unsigned width, unsigned height)
{
	unsigned int reg;
	enum RDMA_MODE mode = RDMA_MODE_DIRECT_LINK;
	enum RDMA_OUTPUT_FORMAT output_format = RDMA_OUTPUT_FORMAT_ARGB;

	reg = readl(rdma_base + DISP_REG_RDMA_GLOBAL_CON);
	if (mode == RDMA_MODE_DIRECT_LINK)
		reg &= ~(0x2U);
	writel(reg, rdma_base + DISP_REG_RDMA_GLOBAL_CON);

	reg = readl(rdma_base + DISP_REG_RDMA_SIZE_CON_0);
	if (output_format == RDMA_OUTPUT_FORMAT_ARGB)
		reg &= ~(0x20000000U);
	else
		reg |= 0x20000000U;
	writel(reg, rdma_base + DISP_REG_RDMA_SIZE_CON_0);

	reg = readl(rdma_base + DISP_REG_RDMA_SIZE_CON_0);
	reg = (reg & ~(0xFFFU)) | (width & 0xFFFU);
	writel(reg, rdma_base + DISP_REG_RDMA_SIZE_CON_0);

	reg = readl(rdma_base + DISP_REG_RDMA_SIZE_CON_1);
	reg = (reg & ~(0xFFFFFU)) | (height & 0xFFFFFU);
	writel(reg, rdma_base + DISP_REG_RDMA_SIZE_CON_1);

	writel(0x80F00008, rdma_base + DISP_REG_RDMA_FIFO_CON);
}

void mediatek_od_enable_vblank(void __iomem *disp_base)
{
	writel(0x1, disp_base + DISP_OD_INTEN);
}

void mediatek_od_disable_vblank(void __iomem *disp_base)
{
	writel(0x0, disp_base + DISP_OD_INTEN);
}

void mediatek_od_clear_vblank(void __iomem *disp_base)
{
	writel(0x0, disp_base + DISP_OD_INTS);
}

static void mediatek_od_start(void __iomem *od_base, unsigned int w,
	unsigned int h)
{
	writel(w << 16 | h, od_base + DISP_OD_SIZE);
	writel(OD_RELAY_MODE, od_base + DISP_OD_CFG);
	writel(1, od_base + DISP_OD_EN);
}

static void mediatek_ufoe_start(void __iomem *ufoe_base)
{
	writel(UFO_BYPASS, ufoe_base + DISP_REG_UFO_START);
}

static void mediatek_color_start(void __iomem *color_base)
{
	writel(COLOR_BYPASS_ALL | COLOR_SEQ_SEL,
		color_base + DISP_COLOR_CFG_MAIN);
	writel(0x1, color_base + DISP_COLOR_START);
}

void main_disp_path_power_on(unsigned int width, unsigned int height,
		void __iomem *ovl_base,
		void __iomem *rdma_base,
		void __iomem *color_base,
		void __iomem *ufoe_base,
		void __iomem *od_base)
{
	mediatek_ovl_roi(ovl_base, width, height, 0x00000000);
	mediatek_ovl_layer_switch(ovl_base, 0, 1);
	mediatek_rdma_config_direct_link(rdma_base, width, height);

	mediatek_ovl_start(ovl_base);
	mediatek_rdma_start(rdma_base);
	mediatek_od_start(od_base, width, height);
	mediatek_ufoe_start(ufoe_base);
	mediatek_color_start(color_base);
}



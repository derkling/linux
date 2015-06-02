/*
 * Copyright (c) 2014 MediaTek Inc.
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

#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/wait.h>

#include <linux/of_device.h>
#include <drm/drm_mipi_dsi.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include "mediatek_drm_crtc.h"
#include "mediatek_drm_drv.h"

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
#ifndef MEDIATEK_DRM_UPSTREAM

	bool			pending_ovl_cursor_config;
	bool			pending_ovl_cursor_enable;
	unsigned int		pending_ovl_cursor_addr;
	int			pending_ovl_cursor_x;
	int			pending_ovl_cursor_y;
#endif /* MEDIATEK_DRM_UPSTREAM */
};

struct crtc_ext_context {
	struct device		*dev;
	struct drm_device	*drm_dev;
	struct mtk_drm_crtc	*crtc;
	int	pipe;

	struct device		*ddp_dev;
	struct clk		*ovl1_disp_clk;
	struct clk		*rdma1_disp_clk;
	struct clk		*color1_disp_clk;
	struct clk		*gamma_disp_clk;

	void __iomem		*ovl1_regs;
	void __iomem		*rdma1_regs;
	void __iomem		*color1_regs;
	void __iomem		*gamma_regs;

	bool			pending_ovl_config;
	bool			pending_ovl_enable;
	unsigned int		pending_ovl_addr;
	unsigned int		pending_ovl_width;
	unsigned int		pending_ovl_height;
	unsigned int		pending_ovl_pitch;
	unsigned int		pending_ovl_format;
#ifndef MEDIATEK_DRM_UPSTREAM

	bool			pending_ovl_cursor_config;
	bool			pending_ovl_cursor_enable;
	unsigned int		pending_ovl_cursor_addr;
	int			pending_ovl_cursor_x;
	int			pending_ovl_cursor_y;
#endif /* MEDIATEK_DRM_UPSTREAM */
};

struct ddp_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct mediatek_drm_crtc	*crtc;
	int				        pipe;

	struct clk			    *mutex_disp_clk;

	void __iomem			*config_regs;
	void __iomem			*mutex_regs;

	bool pending_ovl_config;
	bool pending_ovl_enable;
	unsigned int pending_ovl_addr;
	unsigned int pending_ovl_width;
	unsigned int pending_ovl_height;
	unsigned int pending_ovl_pitch;
	unsigned int pending_ovl_format;
#ifndef MEDIATEK_DRM_UPSTREAM

	bool pending_ovl_cursor_config;
	bool pending_ovl_cursor_enable;
	unsigned int pending_ovl_cursor_addr;
	int pending_ovl_cursor_x;
	int pending_ovl_cursor_y;
#endif /* MEDIATEK_DRM_UPSTREAM */
};

struct mtk_dsi {
	struct drm_device *drm_dev;
	struct drm_encoder encoder;
	struct drm_connector conn;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	struct mipi_dsi_host host;
	struct regulator *disp_supplies;

	void __iomem *dsi_reg_base;
	void __iomem *dsi_tx_reg_base;

	struct clk *dsi_disp_clk_cg;
	struct clk *dsi_dsi_clk_cg;
	struct clk *dsi_div2_clk_cg;

	struct clk *dsi0_engine_clk_cg;
	struct clk *dsi0_digital_clk_cg;

	u32 pll_clk_rate;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
	struct videomode vm;
	bool enabled;
};

/* ------------------------------------------------------------------------- */
/* External variable declarations */
/* ------------------------------------------------------------------------- */
static char debug_buffer[2048];
void __iomem *gdrm_disp_base[8];
void __iomem *gdrm_hdmi_base[6];
int gdrm_disp_table[8] = {
	0x14000000,
	0x1400c000,	/* OVL0 */
	0x1400e000,	/* RDMA0 */
	0x14013000,	/* COLOR0 */
	0x14015000,	/* AAL */
	0x1401a000,	/* UFOE */
	0x14020000,
	0x14023000	/* OD */
};

int gdrm_hdmi_table[6] = {
	0x14000000,	/* CONFIG */
	0x1400d000,	/* OVL1 */
	0x1400f000,	/* RDMA1 */
	0x14014000,	/* COLOR1 */
	0x14016000,	/* GAMMA */
	0x1401b000	/* DSI0 */
};

/* ------------------------------------------------------------------------- */
/* Debug Options */
/* ------------------------------------------------------------------------- */
static char STR_HELP[] =
	"\n"
	"USAGE\n"
	"        echo [ACTION]... > mtkdrm\n"
	"\n"
	"ACTION\n"
	"\n"
	"        dump:\n"
	"             dump all hw registers\n"
	"\n"
	"        regw:addr=val\n"
	"             write hw register\n"
	"\n"
	"        regr:addr\n"
	"             read hw register\n";

/* ------------------------------------------------------------------------- */
/* Command Processor */
/* ------------------------------------------------------------------------- */
static void process_dbg_opt(const char *opt)
{
	if (0 == strncmp(opt, "regw:", 5)) {
		char *p = (char *)opt + 5;
		char *np;
		unsigned long addr, val;
		int i;

		np = strsep(&p, "=");
		if (kstrtoul(np, 16, &addr))
			goto error;

		if (p == NULL)
			goto error;

		np = strsep(&p, "=");
		if (kstrtoul(np, 16, &val))
			goto error;

		for (i = 0; i < sizeof(gdrm_disp_table)/sizeof(int); i++) {
			if (addr > gdrm_disp_table[i] &&
			addr < gdrm_disp_table[i] + 0x1000) {
				addr -= gdrm_disp_table[i];
				writel(val, gdrm_disp_base[i] + addr);
				break;
			}
		}

		for (i = 0; i < sizeof(gdrm_hdmi_table)/sizeof(int); i++) {
			if (addr > gdrm_hdmi_table[i] &&
			addr < gdrm_hdmi_table[i] + 0x1000) {
				addr -= gdrm_hdmi_table[i];
				writel(val, gdrm_hdmi_base[i] + addr);
				break;
			}
		}
	} else if (0 == strncmp(opt, "regr:", 5)) {
		char *p = (char *)opt + 5;
		unsigned long addr;
		int i;

		if (kstrtoul(p, 16, &addr))
			goto error;

		for (i = 0; i < sizeof(gdrm_disp_table)/sizeof(int); i++) {
			if (addr >= gdrm_disp_table[i] &&
			addr < gdrm_disp_table[i] + 0x1000) {
				addr -= gdrm_disp_table[i];
				DRM_INFO("Read register 0x%08X: 0x%08X\n",
					(unsigned int)addr + gdrm_disp_table[i],
					readl(gdrm_disp_base[i] + addr));
				break;
			}
		}

		for (i = 0; i < sizeof(gdrm_hdmi_table)/sizeof(int); i++) {
			if (addr >= gdrm_hdmi_table[i] &&
			addr < gdrm_hdmi_table[i] + 0x1000) {
				addr -= gdrm_hdmi_table[i];
				DRM_INFO("Read register 0x%08X: 0x%08X\n",
					(unsigned int)addr + gdrm_hdmi_table[i],
					readl(gdrm_hdmi_base[i] + addr));
				break;
			}
		}
	} else if (0 == strncmp(opt, "ovl0:", 5)) {
		int i;

		/* OVL */
		for (i = 0; i < 0x260; i += 16)
			DRM_INFO("OVL   0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[1] + i,
			readl(gdrm_disp_base[1] + i),
			readl(gdrm_disp_base[1] + i + 4),
			readl(gdrm_disp_base[1] + i + 8),
			readl(gdrm_disp_base[1] + i + 12));
		for (i = 0xf40; i < 0xfc0; i += 16)
			DRM_INFO("OVL   0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[1] + i,
			readl(gdrm_disp_base[1] + i),
			readl(gdrm_disp_base[1] + i + 4),
			readl(gdrm_disp_base[1] + i + 8),
			readl(gdrm_disp_base[1] + i + 12));
	} else if (0 == strncmp(opt, "dump:", 5)) {
		int i;

		/* CONFIG */
		for (i = 0; i < 0x120; i += 16)
			DRM_INFO("CFG   0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[0] + i,
			readl(gdrm_disp_base[0] + i),
			readl(gdrm_disp_base[0] + i + 4),
			readl(gdrm_disp_base[0] + i + 8),
			readl(gdrm_disp_base[0] + i + 12));

		/* OVL */
		for (i = 0; i < 0x260; i += 16)
			DRM_INFO("OVL   0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[1] + i,
			readl(gdrm_disp_base[1] + i),
			readl(gdrm_disp_base[1] + i + 4),
			readl(gdrm_disp_base[1] + i + 8),
			readl(gdrm_disp_base[1] + i + 12));
		for (i = 0xf40; i < 0xfc0; i += 16)
			DRM_INFO("OVL   0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[1] + i,
			readl(gdrm_disp_base[1] + i),
			readl(gdrm_disp_base[1] + i + 4),
			readl(gdrm_disp_base[1] + i + 8),
			readl(gdrm_disp_base[1] + i + 12));

		/* RDMA */
		for (i = 0; i < 0x100; i += 16)
			DRM_INFO("RDMA  0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[2] + i,
			readl(gdrm_disp_base[2] + i),
			readl(gdrm_disp_base[2] + i + 4),
			readl(gdrm_disp_base[2] + i + 8),
			readl(gdrm_disp_base[2] + i + 12));

		/* COLOR0 */
		for (i = 0x400; i < 0x500; i += 16)
			DRM_INFO("COLOR 0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[3] + i,
			readl(gdrm_disp_base[3] + i),
			readl(gdrm_disp_base[3] + i + 4),
			readl(gdrm_disp_base[3] + i + 8),
			readl(gdrm_disp_base[3] + i + 12));
		for (i = 0xC00; i < 0xD00; i += 16)
			DRM_INFO("COLOR 0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[3] + i,
			readl(gdrm_disp_base[3] + i),
			readl(gdrm_disp_base[3] + i + 4),
			readl(gdrm_disp_base[3] + i + 8),
			readl(gdrm_disp_base[3] + i + 12));

		/* AAL */
		for (i = 0; i < 0x100; i += 16)
			DRM_INFO("AAL   0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[4] + i,
			readl(gdrm_disp_base[4] + i),
			readl(gdrm_disp_base[4] + i + 4),
			readl(gdrm_disp_base[4] + i + 8),
			readl(gdrm_disp_base[4] + i + 12));

		/* UFOE */
		for (i = 0; i < 0x100; i += 16)
			DRM_INFO("UFOE  0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[5] + i,
			readl(gdrm_disp_base[5] + i),
			readl(gdrm_disp_base[5] + i + 4),
			readl(gdrm_disp_base[5] + i + 8),
			readl(gdrm_disp_base[5] + i + 12));

		/* MUTEX */
		for (i = 0; i < 0x100; i += 16)
			DRM_INFO("MUTEX 0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[6] + i,
			readl(gdrm_disp_base[6] + i),
			readl(gdrm_disp_base[6] + i + 4),
			readl(gdrm_disp_base[6] + i + 8),
			readl(gdrm_disp_base[6] + i + 12));

		/* OD */
		for (i = 0; i < 0x100; i += 16)
			DRM_INFO("OD    0x%08X: %08X %08X %08X %08X\n",
			gdrm_disp_table[7] + i,
			readl(gdrm_disp_base[7] + i),
			readl(gdrm_disp_base[7] + i + 4),
			readl(gdrm_disp_base[7] + i + 8),
			readl(gdrm_disp_base[7] + i + 12));
	} else if (0 == strncmp(opt, "hdmi:", 5)) {
		int i;

		/* CONFIG */
		for (i = 0; i < 0x120; i += 16)
			DRM_INFO("CFG    0x%08X: %08X %08X %08X %08X\n",
			gdrm_hdmi_table[0] + i,
			readl(gdrm_hdmi_base[0] + i),
			readl(gdrm_hdmi_base[0] + i + 4),
			readl(gdrm_hdmi_base[0] + i + 8),
			readl(gdrm_hdmi_base[0] + i + 12));

		/* OVL1 */
		for (i = 0; i < 0x260; i += 16)
			DRM_INFO("OVL1   0x%08X: %08X %08X %08X %08X\n",
			gdrm_hdmi_table[1] + i,
			readl(gdrm_hdmi_base[1] + i),
			readl(gdrm_hdmi_base[1] + i + 4),
			readl(gdrm_hdmi_base[1] + i + 8),
			readl(gdrm_hdmi_base[1] + i + 12));
		for (i = 0xf40; i < 0xfc0; i += 16)
			DRM_INFO("OVL1   0x%08X: %08X %08X %08X %08X\n",
			gdrm_hdmi_table[1] + i,
			readl(gdrm_hdmi_base[1] + i),
			readl(gdrm_hdmi_base[1] + i + 4),
			readl(gdrm_hdmi_base[1] + i + 8),
			readl(gdrm_hdmi_base[1] + i + 12));

		/* RDMA1 */
		for (i = 0; i < 0x100; i += 16)
			DRM_INFO("RDMA1  0x%08X: %08X %08X %08X %08X\n",
			gdrm_hdmi_table[2] + i,
			readl(gdrm_hdmi_base[2] + i),
			readl(gdrm_hdmi_base[2] + i + 4),
			readl(gdrm_hdmi_base[2] + i + 8),
			readl(gdrm_hdmi_base[2] + i + 12));

		/* COLOR1 */
		for (i = 0x400; i < 0x500; i += 16)
			DRM_INFO("COLOR1 0x%08X: %08X %08X %08X %08X\n",
			gdrm_hdmi_table[3] + i,
			readl(gdrm_hdmi_base[3] + i),
			readl(gdrm_hdmi_base[3] + i + 4),
			readl(gdrm_hdmi_base[3] + i + 8),
			readl(gdrm_hdmi_base[3] + i + 12));
		for (i = 0xC00; i < 0xD00; i += 16)
			DRM_INFO("COLOR1 0x%08X: %08X %08X %08X %08X\n",
			gdrm_hdmi_table[3] + i,
			readl(gdrm_hdmi_base[3] + i),
			readl(gdrm_hdmi_base[3] + i + 4),
			readl(gdrm_hdmi_base[3] + i + 8),
			readl(gdrm_hdmi_base[3] + i + 12));

		/* GAMMA */
		for (i = 0; i < 0x100; i += 16)
			DRM_INFO("GAMMA  0x%08X: %08X %08X %08X %08X\n",
			gdrm_hdmi_table[4] + i,
			readl(gdrm_hdmi_base[4] + i),
			readl(gdrm_hdmi_base[4] + i + 4),
			readl(gdrm_hdmi_base[4] + i + 8),
			readl(gdrm_hdmi_base[4] + i + 12));
	} else {
	    goto error;
	}

	return;
 error:
	DRM_ERROR("Parse command error!\n\n%s", STR_HELP);
}

static void process_dbg_cmd(char *cmd)
{
	char *tok;

	DRM_INFO("[mtkdrm_dbg] %s\n", cmd);
	memset(debug_buffer, 0, sizeof(debug_buffer));
	while ((tok = strsep(&cmd, " ")) != NULL)
		process_dbg_opt(tok);
}

/* ------------------------------------------------------------------------- */
/* Debug FileSystem Routines */
/* ------------------------------------------------------------------------- */
static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t debug_read(struct file *file, char __user *ubuf, size_t count,
	loff_t *ppos)
{
	if (strlen(debug_buffer))
		return simple_read_from_buffer(ubuf, count, ppos, debug_buffer,
			strlen(debug_buffer));
	else
		return simple_read_from_buffer(ubuf, count, ppos, STR_HELP,
			strlen(STR_HELP));
}

static char dis_cmd_buf[512];
static ssize_t debug_write(struct file *file, const char __user *ubuf,
	size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(dis_cmd_buf) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
		count = debug_bufmax;

	if (copy_from_user(&dis_cmd_buf, ubuf, count))
		return -EFAULT;

	dis_cmd_buf[count] = 0;

	process_dbg_cmd(dis_cmd_buf);

	return ret;
}

struct dentry *mtkdrm_dbgfs;
static const struct file_operations debug_fops = {
	.read = debug_read,
	.write = debug_write,
	.open = debug_open,
};

void mediatek_drm_debugfs_init(struct drm_device *dev)
{
	struct mtk_drm_private *priv = dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc;
	struct crtc_main_context *ctx0;
	struct crtc_ext_context  *ctx1;
	struct ddp_context *ddp;

	struct device_node *node;
	struct platform_device *pdev;
	struct mtk_dsi *dsi;

	DRM_INFO("mediatek_drm_debugfs_init\n");
	mtkdrm_dbgfs = debugfs_create_file("mtkdrm", S_IFREG | S_IRUGO |
			S_IWUSR | S_IWGRP, NULL, (void *)0, &debug_fops);

	mtk_crtc = to_mtk_crtc(priv->crtc[0]);
	ctx0 = (struct crtc_main_context *)mtk_crtc->ctx;
	ddp = dev_get_drvdata(ctx0->ddp_dev);

	node = of_parse_phandle(dev->dev->of_node, "connectors", 0);
	if (!node) {
		dev_err(dev->dev, "mediatek_drm_debugfs_init: Get dsi node fail.\n");
		return;
	}

	pdev = of_find_device_by_node(node);
	if (WARN_ON(!pdev)) {
		dev_err(dev->dev, "mediatek_drm_debugfs_init: Find dsi device fail.\n");
		return;
	}

	gdrm_disp_base[0] = ddp->config_regs;
	gdrm_disp_base[1] = ctx0->ovl0_regs;
	gdrm_disp_base[2] = ctx0->rdma0_regs;
	gdrm_disp_base[3] = ctx0->color0_regs;
	gdrm_disp_base[4] = ctx0->aal_regs;
	gdrm_disp_base[5] = ctx0->ufoe_regs;
	gdrm_disp_base[6] = ddp->mutex_regs;
	gdrm_disp_base[7] = ctx0->od_regs;

	mtk_crtc = to_mtk_crtc(priv->crtc[1]);
	ctx1 = (struct crtc_ext_context *)mtk_crtc->ctx;
	gdrm_hdmi_base[0] = ddp->config_regs;
	gdrm_hdmi_base[1] = ctx1->ovl1_regs;
	gdrm_hdmi_base[2] = ctx1->rdma1_regs;
	gdrm_hdmi_base[3] = ctx1->color1_regs;
	gdrm_hdmi_base[4] = ctx1->gamma_regs;

	dsi = platform_get_drvdata(pdev);
	gdrm_hdmi_base[5] = dsi->dsi_reg_base;
}

void mediatek_drm_debugfs_deinit(void)
{
	debugfs_remove(mtkdrm_dbgfs);
}


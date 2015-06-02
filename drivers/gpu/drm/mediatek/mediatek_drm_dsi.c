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

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/component.h>

#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include "mediatek_drm_drv.h"
#include "mediatek_drm_crtc.h"

#include "mediatek_drm_ddp.h"

#include "mediatek_drm_gem.h"
#include "mediatek_drm_dsi.h"

#define DSI_VIDEO_FIFO_DEPTH (1920 / 4)
#define DSI_HOST_FIFO_DEPTH 64

#define DSI_START		0x00

#define	DSI_CON_CTRL	0x10
#define		DSI_RESET		(1)

#define	DSI_MODE_CTRL	0x14
#define		MODE	2
#define		CMD_MODE	0
#define		SYNC_PULSE_MODE	1
#define		SYNC_EVENT_MODE	2
#define		BURST_MODE	3
#define		FRM_MODE	(1<<16)
#define		MIX_MODE	(1<<17)

#define DSI_TXRX_CTRL	0x18
#define		VC_NUM	(2<<0)
#define		LANE_NUM	(0xf<<2)
#define		DIS_EOT		(1<<6)
#define		NULL_EN		(1<<7)
#define		TE_FREERUN	(1<<8)
#define		EXT_TE_EN	(1<<9)
#define		EXT_TE_EDGE	(1<<10)
#define		MAX_RTN_SIZE	(0xf<<12)
#define		HSTX_CKLP_EN	(1<<16)

#define DSI_PSCTRL		0x1c
#define		DSI_PS_WC		0x3fff
#define		DSI_PS_SEL	 (2<<16)
#define			PACKED_PS_16BIT_RGB565		(0<<16)
#define			LOOSELY_PS_18BIT_RGB666		(1<<16)
#define			PACKED_PS_18BIT_RGB666		(2<<16)
#define			PACKED_PS_24BIT_RGB888		(3<<16)

#define DSI_VSA_NL		0x20
#define DSI_VBP_NL		0x24
#define DSI_VFP_NL		0x28

#define DSI_VACT_NL		0x2C

#define DSI_HSA_WC		0x50
#define DSI_HBP_WC		0x54
#define DSI_HFP_WC		0x58

#define	DSI_HSTX_CKL_WC	0x64

#define DSI_PHY_LCCON	0x104
	#define LC_HS_TX_EN					(1)
	#define LC_ULPM_EN					(1<<1)
	#define LC_WAKEUP_EN				(1<<2)

#define DSI_PHY_LD0CON	0x108
#define		LD0_HS_TX_EN	(1)
#define		LD0_ULPM_EN	(1<<1)
#define		LD0_WAKEUP_EN	(1<<2)

#define	DSI_PHY_TIMECON0	0x0110
#define		LPX		(0xff<<0)
#define		HS_PRPR		(0xff<<8)
#define		HS_ZERO		(0xff<<16)
#define		HS_TRAIL	(0xff<<24)

#define	DSI_PHY_TIMECON1	0x0114
#define		TA_GO		(0xff<<0)
#define		TA_SURE		(0xff<<8)
#define		TA_GET		(0xff<<16)
#define		DA_HS_EXIT	(0xff<<24)

#define	DSI_PHY_TIMECON2	0x0118
#define		CONT_DET	(0xff<<0)
#define		CLK_ZERO	(0xff<<16)
#define		CLK_TRAIL	(0xff<<24)

#define	DSI_PHY_TIMECON3	0x011c
#define		CLK_HS_PRPR	(0xff<<0)
#define		CLK_HS_POST	(0xff<<8)
#define		CLK_HS_EXIT	(0xff<<16)

#define	MIPITX_DSI0_CON		0x00
#define		RG_DSI0_LDOCORE_EN	(1)
#define		RG_DSI0_CKG_LDOOUT_EN	(1<<1)
#define		RG_DSI0_BCLK_SEL	(3<<2)
#define		RG_DSI0_LD_IDX_SEL	(7<<4)
#define		RG_DSI0_PHYCLK_SEL	(2<<8)
#define		RG_DSI0_DSICLK_FREQ_SEL	(1<<10)
#define		RG_DSI0_LPTX_CLMP_EN	(1<<11)

#define	MIPITX_DSI0_CLOCK_LANE	0x04
#define		RG_DSI0_LNTC_LDOOUT_EN		(1)
#define		RG_DSI0_LNTC_CKLANE_EN		(1<<1)
#define		RG_DSI0_LNTC_LPTX_IPLUS1	(1<<2)
#define		RG_DSI0_LNTC_LPTX_IPLUS2	(1<<3)
#define		RG_DSI0_LNTC_LPTX_IMINUS	(1<<4)
#define		RG_DSI0_LNTC_LPCD_IPLUS		(1<<5)
#define		RG_DSI0_LNTC_LPCD_IMLUS		(1<<6)
#define		RG_DSI0_LNTC_RT_CODE		(0xf<<8)

#define	MIPITX_DSI0_DATA_LANE0	0x08
#define		RG_DSI0_LNT0_LDOOUT_EN		(1)
#define		RG_DSI0_LNT0_CKLANE_EN		(1<<1)
#define		RG_DSI0_LNT0_LPTX_IPLUS1	(1<<2)
#define		RG_DSI0_LNT0_LPTX_IPLUS2	(1<<3)
#define		RG_DSI0_LNT0_LPTX_IMINUS	(1<<4)
#define		RG_DSI0_LNT0_LPCD_IPLUS		(1<<5)
#define		RG_DSI0_LNT0_LPCD_IMINUS	(1<<6)
#define		RG_DSI0_LNT0_RT_CODE		(0xf<<8)

#define	MIPITX_DSI0_DATA_LANE1	0x0c
#define		RG_DSI0_LNT1_LDOOUT_EN		(1)
#define		RG_DSI0_LNT1_CKLANE_EN		(1<<1)
#define		RG_DSI0_LNT1_LPTX_IPLUS1	(1<<2)
#define		RG_DSI0_LNT1_LPTX_IPLUS2	(1<<3)
#define		RG_DSI0_LNT1_LPTX_IMINUS	(1<<4)
#define		RG_DSI0_LNT1_LPCD_IPLUS		(1<<5)
#define		RG_DSI0_LNT1_LPCD_IMINUS	(1<<6)
#define		RG_DSI0_LNT1_RT_CODE		(0xf<<8)

#define	MIPITX_DSI0_DATA_LANE2	0x10
#define		RG_DSI0_LNT2_LDOOUT_EN		(1)
#define		RG_DSI0_LNT2_CKLANE_EN		(1<<1)
#define		RG_DSI0_LNT2_LPTX_IPLUS1	(1<<2)
#define		RG_DSI0_LNT2_LPTX_IPLUS2	(1<<3)
#define		RG_DSI0_LNT2_LPTX_IMINUS	(1<<4)
#define		RG_DSI0_LNT2_LPCD_IPLUS		(1<<5)
#define		RG_DSI0_LNT2_LPCD_IMINUS	(1<<6)
#define		RG_DSI0_LNT2_RT_CODE		(0xf<<8)

#define	MIPITX_DSI0_DATA_LANE3	0x14
#define		RG_DSI0_LNT3_LDOOUT_EN		(1)
#define		RG_DSI0_LNT3_CKLANE_EN		(1<<1)
#define		RG_DSI0_LNT3_LPTX_IPLUS1	(1<<2)
#define		RG_DSI0_LNT3_LPTX_IPLUS2	(1<<3)
#define		RG_DSI0_LNT3_LPTX_IMINUS	(1<<4)
#define		RG_DSI0_LNT3_LPCD_IPLUS		(1<<5)
#define		RG_DSI0_LNT3_LPCD_IMINUS	(1<<6)
#define		RG_DSI0_LNT3_RT_CODE		(0xf<<8)

#define	MIPITX_DSI_TOP_CON	0x40
#define		RG_DSI_LNT_INTR_EN		(1)
#define		RG_DSI_LNT_HS_BIAS_EN		(1<<1)
#define		RG_DSI_LNT_IMP_CAL_EN		(1<<2)
#define		RG_DSI_LNT_TESTMODE_EN		(1<<3)
#define		RG_DSI_LNT_IMP_CAL_CODE		(0xf<<4)
#define		RG_DSI_LNT_AIO_SEL		(7<<8)
#define		RG_DSI_PAD_TIE_LOW_EN		(1<<11)
#define		RG_DSI_DEBUG_INPUT_EN		(1<<12)
#define		RG_DSI_PRESERVE			(7<<13)

#define	MIPITX_DSI_BG_CON	0x44
#define		RG_DSI_BG_CORE_EN		1
#define		RG_DSI_BG_CKEN			(1<<1)
#define		RG_DSI_BG_DIV			(0x3<<2)
#define		RG_DSI_BG_FAST_CHARGE		(1<<4)
#define		RG_DSI_V12_SEL			(7<<5)
#define		RG_DSI_V10_SEL			(7<<8)
#define		RG_DSI_V072_SEL			(7<<11)
#define		RG_DSI_V04_SEL			(7<<14)
#define		RG_DSI_V032_SEL			(7<<17)
#define		RG_DSI_V02_SEL			(7<<20)
#define		RG_DSI_BG_R1_TRIM		(0xf<<24)
#define		RG_DSI_BG_R2_TRIM		(0xf<<28)

#define	MIPITX_DSI_PLL_CON0	0x50
#define		RG_DSI0_MPPLL_PLL_EN		(1<<0)
#define		RG_DSI0_MPPLL_PREDIV		(3<<1)
#define		RG_DSI0_MPPLL_TXDIV0		(3<<3)
#define		RG_DSI0_MPPLL_TXDIV1		(3<<5)
#define		RG_DSI0_MPPLL_POSDIV		(7<<7)
#define		RG_DSI0_MPPLL_MONVC_EN		(1<<10)
#define		RG_DSI0_MPPLL_MONREF_EN		(1<<11)
#define		RG_DSI0_MPPLL_VOD_EN		(1<<12)

#define	MIPITX_DSI_PLL_CON1	0x54
#define		RG_DSI0_MPPLL_SDM_FRA_EN	(1)
#define		RG_DSI0_MPPLL_SDM_SSC_PH_INIT	(1<<1)
#define		RG_DSI0_MPPLL_SDM_SSC_EN	(1<<2)
#define		RG_DSI0_MPPLL_SDM_SSC_PRD	(0xffff<<16)

#define	MIPITX_DSI_PLL_CON2	0x58

#define	MIPITX_DSI_PLL_PWR	0x68
#define		RG_DSI_MPPLL_SDM_PWR_ON		(1<<0)
#define		RG_DSI_MPPLL_SDM_ISO_EN		(1<<1)
#define		RG_DSI_MPPLL_SDM_PWR_ACK	(1<<8)

#define	MIPITX_DSI_SW_CTRL	0x80
#define		SW_CTRL_EN			(1<<0)

#define	MIPITX_DSI_SW_CTRL_CON0	0x84
#define		SW_LNTC_LPTX_PRE_OE		(1<<0)
#define		SW_LNTC_LPTX_OE			(1<<1)
#define		SW_LNTC_LPTX_P			(1<<2)
#define		SW_LNTC_LPTX_N			(1<<3)
#define		SW_LNTC_HSTX_PRE_OE		(1<<4)
#define		SW_LNTC_HSTX_OE			(1<<5)
#define		SW_LNTC_HSTX_ZEROCLK		(1<<6)
#define		SW_LNT0_LPTX_PRE_OE		(1<<7)
#define		SW_LNT0_LPTX_OE			(1<<8)
#define		SW_LNT0_LPTX_P			(1<<9)
#define		SW_LNT0_LPTX_N			(1<<10)
#define		SW_LNT0_HSTX_PRE_OE		(1<<11)
#define		SW_LNT0_HSTX_OE			(1<<12)
#define		SW_LNT0_LPRX_EN			(1<<13)
#define		SW_LNT1_LPTX_PRE_OE		(1<<14)
#define		SW_LNT1_LPTX_OE			(1<<15)
#define		SW_LNT1_LPTX_P			(1<<16)
#define		SW_LNT1_LPTX_N			(1<<17)
#define		SW_LNT1_HSTX_PRE_OE		(1<<18)
#define		SW_LNT1_HSTX_OE			(1<<19)
#define		SW_LNT2_LPTX_PRE_OE		(1<<20)
#define		SW_LNT2_LPTX_OE			(1<<21)
#define		SW_LNT2_LPTX_P			(1<<22)
#define		SW_LNT2_LPTX_N			(1<<23)
#define		SW_LNT2_HSTX_PRE_OE		(1<<24)
#define		SW_LNT2_HSTX_OE			(1<<25)

#define NS_TO_CYCLE(n, c)    ((n) / c + (((n) % c) ? 1 : 0))

static inline unsigned long mtk_dsi_readl(struct mtk_dsi *dsi,
	unsigned long reg)
{
	return readl(dsi->dsi_reg_base + (reg << 2));
}

static inline void mtk_dsi_writel(struct mtk_dsi *dsi, unsigned long value,
				    unsigned long reg)
{
	writel(value, dsi->dsi_reg_base + (reg << 2));
}

static void dsi_phy_clk_switch_off(struct mtk_dsi *dsi)
{
	u32 tmp_reg;

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);
	tmp_reg = (tmp_reg & (~RG_DSI0_MPPLL_PLL_EN));
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);
		tmp_reg = (tmp_reg | (RG_DSI_PAD_TIE_LOW_EN));
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_CLOCK_LANE);
	tmp_reg = tmp_reg & (~RG_DSI0_LNTC_LDOOUT_EN);
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_CLOCK_LANE);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE0);
	tmp_reg = tmp_reg & (~RG_DSI0_LNT0_LDOOUT_EN);
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE0);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE1);
	tmp_reg = tmp_reg & (~RG_DSI0_LNT1_LDOOUT_EN);
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE1);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE2);
	tmp_reg = tmp_reg & (~RG_DSI0_LNT2_LDOOUT_EN);
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE2);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE3);
	tmp_reg = tmp_reg & (~RG_DSI0_LNT3_LDOOUT_EN);
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE3);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_PWR);
	tmp_reg = tmp_reg | RG_DSI_MPPLL_SDM_ISO_EN;
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_PWR);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_PWR);
	tmp_reg = tmp_reg & (~RG_DSI_MPPLL_SDM_PWR_ON);
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_PWR);

		tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);
		tmp_reg = (tmp_reg & (~(RG_DSI_LNT_HS_BIAS_EN)));
		writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_CON);
	tmp_reg = tmp_reg & (~(RG_DSI0_CKG_LDOOUT_EN |
		RG_DSI0_LDOCORE_EN));
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_CON);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_BG_CON);
	tmp_reg = tmp_reg & (~(RG_DSI_BG_CKEN | RG_DSI_BG_CORE_EN));
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI_BG_CON);

	tmp_reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);
		tmp_reg = (tmp_reg & (~(RG_DSI0_MPPLL_PREDIV |
			RG_DSI0_MPPLL_TXDIV0 | RG_DSI0_MPPLL_TXDIV1 |
			RG_DSI0_MPPLL_POSDIV)));
	writel(tmp_reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);
}

static void dsi_phy_clk_setting(struct mtk_dsi *dsi)
{
	unsigned int data_rate = dsi->vm.pixelclock * 3 * 21/(1 * 1000 * 10);
	unsigned int txdiv = 0;
	unsigned int txdiv0 = 0;
	unsigned int txdiv1 = 0;
	unsigned int pcw = 0;
	u32 reg;
	u32 temp;

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_BG_CON);
	reg = (reg & (~RG_DSI_V032_SEL)) | (4<<17);
	reg = (reg & (~RG_DSI_V04_SEL)) | (4<<14);
	reg = (reg & (~RG_DSI_V072_SEL)) | (4<<11);
	reg = (reg & (~RG_DSI_V10_SEL)) | (4<<8);
	reg = (reg & (~RG_DSI_V12_SEL)) | (4<<5);
	reg = (reg & (~RG_DSI_BG_CKEN)) | (1<<1);
	reg = (reg & (~RG_DSI_BG_CORE_EN)) | (1);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_BG_CON);

	usleep_range(30, 100);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);
	reg = (reg & (~RG_DSI_LNT_IMP_CAL_CODE)) | (8<<4);
	reg = (reg & (~RG_DSI_LNT_HS_BIAS_EN)) | (1<<1);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_CON);
	reg = (reg & (~RG_DSI0_CKG_LDOOUT_EN)) | (1<<1);
	reg = (reg & (~RG_DSI0_LDOCORE_EN)) | (1);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_CON);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_PWR);
	reg = (reg & (~RG_DSI_MPPLL_SDM_PWR_ON)) | (1<<0);
	reg = (reg & (~RG_DSI_MPPLL_SDM_ISO_EN));
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_PWR);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);
	reg = (reg & (~RG_DSI0_MPPLL_PLL_EN));
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);

	if (data_rate > 1250) {
		txdiv = 1;
		txdiv0 = 0;
		txdiv1 = 0;
	} else if (data_rate >= 500) {
		txdiv = 1;
		txdiv0 = 0;
		txdiv1 = 0;
	} else if (data_rate >= 250) {
		txdiv = 2;
		txdiv0 = 1;
		txdiv1 = 0;
	} else if (data_rate >= 125) {
		txdiv = 4;
		txdiv0 = 2;
		txdiv1 = 0;
	} else if (data_rate > 62) {
		txdiv = 8;
		txdiv0 = 2;
		txdiv1 = 1;
	} else if (data_rate >= 50) {
		txdiv = 16;
		txdiv0 = 2;
		txdiv1 = 2;
	} else {
	}

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);

	switch (txdiv) {
	case 1:
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV0)) | (0<<3);
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV1)) | (0<<5);

		break;
	case 2:
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV0)) | (1<<3);
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV1)) | (0<<5);
		break;
	case 4:
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV0)) | (2<<3);
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV1)) | (0<<5);
		break;
	case 8:
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV0)) | (2<<3);
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV1)) | (1<<5);
		break;
	case 16:
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV0)) | (2<<3);
		reg = (reg & (~RG_DSI0_MPPLL_TXDIV1)) | (2<<5);
		break;

	default:
		break;
	}
	reg = (reg & (~RG_DSI0_MPPLL_PREDIV));
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);

	pcw = data_rate * txdiv / 13;
	temp = data_rate * txdiv % 13;
	reg = ((pcw & 0x7f)<<24) + (((256 * temp / 13) & 0xff)<<16)
		+ (((256 * (256 * temp % 13)/13) & 0xff)<<8)
		+ ((256 * (256 * (256 * temp % 13) % 13) / 13) & 0xff);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON2);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON1);
	reg = (reg & (~RG_DSI0_MPPLL_SDM_FRA_EN)) | (1<<0);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON1);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_CLOCK_LANE);
	reg = (reg & (~RG_DSI0_LNTC_LDOOUT_EN)) | (1<<0);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_CLOCK_LANE);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE0);
	reg = (reg & (~RG_DSI0_LNT0_LDOOUT_EN)) | (1<<0);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE0);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE1);
	reg = (reg & (~RG_DSI0_LNT1_LDOOUT_EN)) | (1<<0);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE1);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE2);
	reg = (reg & (~RG_DSI0_LNT2_LDOOUT_EN)) | (1<<0);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE2);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE3);
	reg = (reg & (~RG_DSI0_LNT3_LDOOUT_EN)) | (1<<0);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI0_DATA_LANE3);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);
	reg = (reg & (~RG_DSI0_MPPLL_PLL_EN)) | (1<<0);
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON0);

	usleep_range(20, 100);
	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON1);
	reg = (reg & (~RG_DSI0_MPPLL_SDM_SSC_EN));
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_PLL_CON1);

	reg = readl(dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);
	reg = (reg & (~RG_DSI_PAD_TIE_LOW_EN));
	writel(reg, dsi->dsi_tx_reg_base + MIPITX_DSI_TOP_CON);
}

static void dsi_phy_timconfig(struct mtk_dsi *dsi)
{
	u32 timcon0 = 0;
	u32 timcon1 = 0;
	u32 timcon2 = 0;
	u32 timcon3 = 0;
	unsigned int lane_no = dsi->lanes;
	unsigned int cycle_time;
	unsigned int ui;
	unsigned int hs_trail_m, hs_trail_n;

	ui = 1000/(250 * 2) + 0x01;
	cycle_time = 8000/(250 * 2) + 0x01;

	hs_trail_m = lane_no;
	hs_trail_n =  NS_TO_CYCLE(((lane_no * 4 * ui) + 60), cycle_time);

	timcon0 = (timcon0 & (~HS_TRAIL)) | (8<<24);
	timcon0 = (timcon0 & (~HS_PRPR)) | 0x6<<8;

	if ((timcon0 & HS_PRPR) == 0)
		timcon0 = (timcon0 & (~HS_PRPR)) | 1<<8;

	timcon0 = (timcon0 & (~HS_ZERO)) | 0xa << 16;
	timcon0 = (timcon0 & (~LPX)) | 5;

	if ((timcon0 & LPX) == 0)
		timcon0 =  (timcon0 & (~LPX)) | 1;

	timcon1 = (timcon1 & (~TA_GET)) | (5 * (timcon0 & LPX)<<16);
	timcon1 = (timcon1 & (~TA_SURE)) | ((3 * (timcon0 & LPX) / 2) << 8);
	timcon1 =  (timcon1 & (~TA_GO)) | (4 * (timcon0 & LPX));
	timcon1 = (timcon1 & (~DA_HS_EXIT)) | (7<<24);
	timcon2 = (timcon2 & (~CLK_TRAIL)) | ((NS_TO_CYCLE(0x64, cycle_time) +
		0xa) << 24);

	if (((timcon2 & CLK_TRAIL)>>24) < 2)
		timcon2 = (timcon2 & (~CLK_TRAIL)) | (2<<24);

	timcon2 = (timcon2 & (~CONT_DET));
	timcon3 =  (timcon3 & (~CLK_HS_PRPR)) | NS_TO_CYCLE(0x40, cycle_time);
	if ((timcon3 & CLK_HS_PRPR) == 0)
		timcon3 = (timcon3 & (~CLK_HS_PRPR)) | 1;

	timcon2 = (timcon2 & (~CLK_ZERO)) |
		(NS_TO_CYCLE(0x190 - (timcon3 & CLK_HS_PRPR) * cycle_time,
		cycle_time)<<16);

	timcon3 =  (timcon3 & (~CLK_HS_EXIT)) | ((2 * (timcon0 & LPX))<<16);
	timcon3 =  (timcon3 & (~CLK_HS_POST)) | (NS_TO_CYCLE((80 + 52 * ui),
		cycle_time)<<8);

	writel(timcon0, dsi->dsi_reg_base + DSI_PHY_TIMECON0);
	writel(timcon1, dsi->dsi_reg_base + DSI_PHY_TIMECON1);
	writel(timcon2, dsi->dsi_reg_base + DSI_PHY_TIMECON2);
	writel(timcon3, dsi->dsi_reg_base + DSI_PHY_TIMECON3);
}

static void mtk_dsi_reset(struct mtk_dsi *dsi)
{
	writel(3, dsi->dsi_reg_base + DSI_CON_CTRL);
	writel(2, dsi->dsi_reg_base + DSI_CON_CTRL);
}

static int mtk_dsi_poweron(struct mtk_dsi *dsi)
{
	int ret;
	struct drm_device *dev = dsi->drm_dev;

	dsi_phy_clk_setting(dsi);

	ret = clk_prepare_enable(dsi->dsi0_engine_clk_cg);
	if (ret < 0) {
		dev_err(dev->dev, "can't enable dsi0_engine_clk_cg %d\n", ret);
		goto err_dsi0_engine_clk_cg;
	}

	ret = clk_prepare_enable(dsi->dsi0_digital_clk_cg);
	if (ret < 0) {
		dev_err(dev->dev, "can't enable dsi0_digital_clk_cg %d\n", ret);
		goto err_dsi0_digital_clk_cg;
	}

	mtk_dsi_reset((dsi));
	dsi_phy_timconfig(dsi);

	return 0;

err_dsi0_digital_clk_cg:
	clk_disable_unprepare(dsi->dsi0_engine_clk_cg);

err_dsi0_engine_clk_cg:

	return ret;
}

static void dsi_clk_ulp_mode_enter(struct mtk_dsi *dsi)
{
	u32 tmp_reg1;

	tmp_reg1 = readl(dsi->dsi_reg_base + DSI_PHY_LCCON);
	tmp_reg1 = tmp_reg1 & (~LC_HS_TX_EN);
	writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LCCON);
	tmp_reg1 = tmp_reg1 & (~LC_ULPM_EN);
	writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LCCON);
}

static void dsi_clk_ulp_mode_leave(struct mtk_dsi *dsi)
{
	u32 tmp_reg1;

	tmp_reg1 = readl(dsi->dsi_reg_base + DSI_PHY_LCCON);
	tmp_reg1 = tmp_reg1 & (~LC_ULPM_EN);
	writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LCCON);
	tmp_reg1 = tmp_reg1 | LC_WAKEUP_EN;
	writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LCCON);
	tmp_reg1 = tmp_reg1 & (~LC_WAKEUP_EN);
	writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LCCON);
}

static void dsi_lane0_ulp_mode(struct mtk_dsi *dsi, bool enter)
{
	u32 tmp_reg1;

	tmp_reg1 = readl(dsi->dsi_reg_base + DSI_PHY_LD0CON);

	if (enter) {
		tmp_reg1 = tmp_reg1 & (~LD0_HS_TX_EN);
		writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LD0CON);
		tmp_reg1 = tmp_reg1 & (~LD0_ULPM_EN);
		writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LD0CON);
	} else {
		tmp_reg1 = tmp_reg1 & (~LD0_ULPM_EN);
		writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LD0CON);
		tmp_reg1 = tmp_reg1 | LD0_WAKEUP_EN;
		writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LD0CON);
		tmp_reg1 = tmp_reg1 & (~LD0_WAKEUP_EN);
		writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LD0CON);
	}
}

static bool dsi_clk_hs_state(struct mtk_dsi *dsi)
{
	u32 tmp_reg1;

	tmp_reg1 = readl(dsi->dsi_reg_base + DSI_PHY_LCCON);

	return ((tmp_reg1 & LC_HS_TX_EN) == 1) ? true : false;
}

static void dsi_clk_hs_mode(struct mtk_dsi *dsi, bool enter)
{
	u32 tmp_reg1;

	tmp_reg1 = readl(dsi->dsi_reg_base + DSI_PHY_LCCON);

	if (enter && !dsi_clk_hs_state(dsi)) {
		tmp_reg1 = tmp_reg1 | LC_HS_TX_EN;
		writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LCCON);
	} else if (!enter && dsi_clk_hs_state(dsi)) {
		tmp_reg1 = tmp_reg1 & (~LC_HS_TX_EN);
		writel(tmp_reg1, dsi->dsi_reg_base + DSI_PHY_LCCON);
	}
}

static void  dsi_set_mode(struct mtk_dsi *dsi)
{
	u32 tmp_reg1;

	tmp_reg1 = 0;

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		tmp_reg1 = SYNC_PULSE_MODE;

		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			tmp_reg1 = BURST_MODE;

		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			tmp_reg1 = SYNC_PULSE_MODE;
	}

	writel(tmp_reg1, dsi->dsi_reg_base + DSI_MODE_CTRL);
}

static void dsi_ps_control_vact(struct mtk_dsi *dsi)
{
	struct videomode *vm = &dsi->vm;
	u32 dsi_tmp_buf_bpp, ps_wc;
	u32 tmp_reg;
	u32 tmp_hstx_cklp_wc;

	tmp_reg = 0;

	if (dsi->format == MIPI_DSI_FMT_RGB565)
		dsi_tmp_buf_bpp = 2;
	else
		dsi_tmp_buf_bpp = 3;

	ps_wc = vm->vactive * dsi_tmp_buf_bpp;

	tmp_reg = ps_wc;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		tmp_reg |= PACKED_PS_24BIT_RGB888;
		break;
	case MIPI_DSI_FMT_RGB666:
		tmp_reg |= PACKED_PS_18BIT_RGB666;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		tmp_reg |= LOOSELY_PS_18BIT_RGB666;
		break;
	case MIPI_DSI_FMT_RGB565:
		tmp_reg |= PACKED_PS_16BIT_RGB565;
		break;
	}

	tmp_hstx_cklp_wc = ps_wc;

	writel(vm->vactive, dsi->dsi_reg_base + DSI_VACT_NL);
	writel(tmp_reg, dsi->dsi_reg_base + DSI_PSCTRL);
	writel(tmp_hstx_cklp_wc, dsi->dsi_reg_base + DSI_HSTX_CKL_WC);
}

static void dsi_rxtx_control(struct mtk_dsi *dsi)
{
	u32 tmp_reg = 0;

	switch (dsi->lanes) {
	case 1:
		tmp_reg = 1<<2;
		break;
	case 2:
		tmp_reg = 3<<2;
		break;
	case 3:
		tmp_reg = 7<<2;
		break;
	case 4:
		tmp_reg = 0xf << 2;
		break;
	default:
		tmp_reg = 0xf << 2;
		break;
	}

	writel(tmp_reg, dsi->dsi_reg_base + DSI_TXRX_CTRL);
}

void dsi_ps_control(struct mtk_dsi *dsi)
{
	unsigned int dsi_tmp_buf_bpp;
	u32 tmp_reg1 = 0;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		tmp_reg1 = PACKED_PS_24BIT_RGB888;
		dsi_tmp_buf_bpp = 3;
		break;
	case MIPI_DSI_FMT_RGB666:
		tmp_reg1 = LOOSELY_PS_18BIT_RGB666;
		dsi_tmp_buf_bpp = 3;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		tmp_reg1 = PACKED_PS_18BIT_RGB666;
		dsi_tmp_buf_bpp = 3;
		break;
	case MIPI_DSI_FMT_RGB565:
		tmp_reg1 = PACKED_PS_16BIT_RGB565;
		dsi_tmp_buf_bpp = 2;
		break;
	default:
		tmp_reg1 = PACKED_PS_24BIT_RGB888;
		dsi_tmp_buf_bpp = 3;
		break;
	}

	tmp_reg1 = tmp_reg1 + ((dsi->vm.hactive * dsi_tmp_buf_bpp) & DSI_PS_WC);

	writel(tmp_reg1, dsi->dsi_reg_base + DSI_PSCTRL);
}

static void dsi_config_vdo_timing(struct mtk_dsi *dsi)
{
	unsigned int horizontal_sync_active_byte;
	unsigned int horizontal_backporch_byte;
	unsigned int horizontal_frontporch_byte;
	unsigned int dsi_tmp_buf_bpp;

	struct videomode *vm = &dsi->vm;

	if (dsi->format == MIPI_DSI_FMT_RGB565)
		dsi_tmp_buf_bpp = 2;
	else
		dsi_tmp_buf_bpp = 3;

	writel(vm->vsync_len, dsi->dsi_reg_base + DSI_VSA_NL);
	writel(vm->vback_porch, dsi->dsi_reg_base + DSI_VBP_NL);
	writel(vm->vfront_porch, dsi->dsi_reg_base + DSI_VFP_NL);
	writel(vm->vactive, dsi->dsi_reg_base + DSI_VACT_NL);

	horizontal_sync_active_byte = (vm->hsync_len * dsi_tmp_buf_bpp - 10);

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		horizontal_backporch_byte =
			(vm->hback_porch * dsi_tmp_buf_bpp - 10);
	else
		horizontal_backporch_byte = ((vm->hback_porch + vm->hsync_len) *
			dsi_tmp_buf_bpp - 10);

	horizontal_frontporch_byte = (vm->hfront_porch * dsi_tmp_buf_bpp - 12);

	writel(horizontal_sync_active_byte, dsi->dsi_reg_base + DSI_HSA_WC);
	writel(horizontal_backporch_byte, dsi->dsi_reg_base + DSI_HBP_WC);
	writel(horizontal_frontporch_byte, dsi->dsi_reg_base + DSI_HFP_WC);

	dsi_ps_control(dsi);
}

static void mtk_dsi_start(struct mtk_dsi *dsi)
{
	writel(0, dsi->dsi_reg_base + DSI_START);
	writel(1, dsi->dsi_reg_base + DSI_START);
}

static void mtk_dsi_poweroff(struct mtk_dsi *dsi)
{
	clk_disable_unprepare(dsi->dsi0_engine_clk_cg);
	clk_disable_unprepare(dsi->dsi0_digital_clk_cg);

	usleep_range(10000, 20000);

	dsi_phy_clk_switch_off(dsi);
}

static void mtk_output_dsi_enable(struct mtk_dsi *dsi)
{
	int ret;

	mtk_dsi_info("dsi->enabled = %d\n", dsi->enabled);

	if (dsi->enabled == true)
		return;

	if (dsi->panel) {
		if (drm_panel_prepare(dsi->panel)) {
			DRM_ERROR("failed to setup the panel\n");
			return;
		}
	}

	ret = mtk_dsi_poweron(dsi);
	if (ret < 0)
		return;

	dsi_rxtx_control(dsi);

	dsi_clk_ulp_mode_leave(dsi);
	dsi_lane0_ulp_mode(dsi, 0);
	dsi_clk_hs_mode(dsi, 0);
	dsi_set_mode(dsi);

	dsi_ps_control_vact(dsi);
	dsi_config_vdo_timing(dsi);

	dsi_set_mode(dsi);
	dsi_clk_hs_mode(dsi, 1);

	mtk_dsi_start(dsi);

	dsi->enabled = true;
}

static void mtk_output_dsi_disable(struct mtk_dsi *dsi)
{
	mtk_dsi_info("dsi->enabled = %d\n", dsi->enabled);

	if (dsi->enabled == false)
		return;

	if (dsi->panel) {
		if (drm_panel_disable(dsi->panel)) {
			DRM_ERROR("failed to disable the panel\n");
			return;
		}
	}

	dsi_lane0_ulp_mode(dsi, 1);
	dsi_clk_ulp_mode_enter(dsi);
	mtk_dsi_poweroff(dsi);
	dsi_phy_clk_switch_off(dsi);

	dsi->enabled = false;
}

static void mtk_dsi_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs mtk_dsi_encoder_funcs = {
	.destroy	= mtk_dsi_encoder_destroy,
};

static void mtk_dsi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct mtk_dsi *dsi = encoder_to_dsi(encoder);

	mtk_dsi_info("%s dpms mode = %d !\n", __func__, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		mtk_output_dsi_enable(dsi);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		mtk_output_dsi_disable(dsi);
		break;
	default:
		break;
	}
}

static bool mtk_dsi_encoder_mode_fixup(struct drm_encoder *encoder,
			       const struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void mtk_dsi_encoder_prepare(struct drm_encoder *encoder)
{
	struct mtk_dsi *dsi = encoder_to_dsi(encoder);

	mtk_output_dsi_disable(dsi);
}

static void mtk_dsi_encoder_mode_set(struct drm_encoder *encoder,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted)
{
	struct mtk_dsi *dsi = encoder_to_dsi(encoder);

	dsi->vm.pixelclock = adjusted->clock;
	dsi->vm.hactive  = adjusted->hdisplay;
	dsi->vm.hback_porch = adjusted->htotal - adjusted->hsync_end;
	dsi->vm.hfront_porch = adjusted->hsync_start - adjusted->hdisplay;
	dsi->vm.hsync_len = adjusted->hsync_end - adjusted->hsync_start;

	dsi->vm.vactive   = adjusted->vdisplay;
	dsi->vm.vback_porch = adjusted->vtotal - adjusted->vsync_end;
	dsi->vm.vfront_porch = adjusted->vsync_start - adjusted->vdisplay;
	dsi->vm.vsync_len = adjusted->vsync_end - adjusted->vsync_start;

#ifndef MEDIATEK_DRM_UPSTREAM
	if (of_find_compatible_node(NULL, NULL, "ite,it6151") != NULL) {
		dsi->vm.hactive  = dsi->vm.hactive + 2;
		dsi->vm.hsync_len = dsi->vm.hsync_len - 2;
		DRM_ERROR("mtk_dsi_encoder_mode_set adjust for it6151\n");
	}
#endif /* MEDIATEK_DRM_UPSTREAM */
}

static void mtk_dsi_encoder_commit(struct drm_encoder *encoder)
{
	struct mtk_dsi *dsi = encoder_to_dsi(encoder);

	mtk_dsi_info("Commit dsi\n");
	mtk_output_dsi_enable(dsi);
}

static enum drm_connector_status mtk_dsi_connector_detect(
	struct drm_connector *connector, bool force)
{
	enum drm_connector_status status = connector_status_unknown;

	status = connector_status_connected; /* FIXME? */

	return status;
}

static void mtk_dsi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_display_mode default_modes[] = {
	/* 1368x768@60Hz */
	{ DRM_MODE("1368x768", DRM_MODE_TYPE_DRIVER, 72070,
	1368, 1368 + 58, 1368 + 58 + 58, 1368 + 58 + 58 + 58, 0,
	768, 768 + 4, 768 + 4 + 4, 768 + 4 + 4 + 4, 0, 0) },
};

static int mtk_dsi_connector_get_modes(struct drm_connector *connector)
{
	const struct drm_display_mode *ptr = &default_modes[0];
	struct drm_display_mode *mode;
	int count = 0;

	mode = drm_mode_duplicate(connector->dev, ptr);
	if (mode) {
		drm_mode_probed_add(connector, mode);
		count++;
	}

	connector->display_info.width_mm = mode->hdisplay;
	connector->display_info.height_mm = mode->vdisplay;

	return 1;
}

static struct drm_encoder *
mtk_dsi_connector_best_encoder(struct drm_connector *connector)
{
	struct mtk_dsi *dsi = connector_to_dsi(connector);

	return &dsi->encoder;
}

static const struct drm_encoder_helper_funcs mtk_dsi_encoder_helper_funcs = {
	.dpms = mtk_dsi_encoder_dpms,
	.mode_fixup = mtk_dsi_encoder_mode_fixup,
	.prepare = mtk_dsi_encoder_prepare,
	.mode_set = mtk_dsi_encoder_mode_set,
	.commit = mtk_dsi_encoder_commit,
};

static const struct drm_connector_funcs mtk_dsi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = mtk_dsi_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = mtk_dsi_connector_destroy,
};

static const struct drm_connector_helper_funcs
	mtk_dsi_connector_helper_funcs = {
	.get_modes = mtk_dsi_connector_get_modes,
	.best_encoder = mtk_dsi_connector_best_encoder,
};

struct bridge_init {
	struct i2c_client *mipirx_client;
	struct i2c_client *dptx_client;
	struct device_node *node_mipirx;
	struct device_node *node_dptx;
};

static int mtk_drm_attach_lcm_bridge(struct drm_bridge *bridge,
	struct drm_encoder *encoder)
{
	int ret;

	encoder->bridge = bridge;
	bridge->encoder = encoder;
	ret = drm_bridge_attach(encoder->dev, bridge);
	if (ret) {
		DRM_ERROR("Failed to attach bridge to drm\n");
		return ret;
	}

	return 0;
}

static int mtk_dsi_create_conn_enc(struct mtk_dsi *dsi)
{
	int ret;

	ret = drm_encoder_init(dsi->drm_dev, &dsi->encoder,
			&mtk_dsi_encoder_funcs, DRM_MODE_ENCODER_DSI);

	if (ret)
		goto errcode;

	drm_encoder_helper_add(&dsi->encoder, &mtk_dsi_encoder_helper_funcs);

	dsi->encoder.possible_crtcs = 1;

	/* Pre-empt DP connector creation if there's a bridge */
	ret = mtk_drm_attach_lcm_bridge(dsi->bridge, &dsi->encoder);
	if (!ret)
		return 0;

	ret = drm_connector_init(dsi->drm_dev, &dsi->conn,
		&mtk_dsi_connector_funcs, DRM_MODE_CONNECTOR_DSI);
	if (ret)
		goto errcode;

	drm_connector_helper_add(&dsi->conn, &mtk_dsi_connector_helper_funcs);

	ret = drm_connector_register(&dsi->conn);
	if (ret)
		goto errcode;

	dsi->conn.dpms = DRM_MODE_DPMS_OFF;
	dsi->conn.encoder = &dsi->encoder;

	drm_mode_connector_attach_encoder(&dsi->conn, &dsi->encoder);

	if (dsi->panel)
		ret = drm_panel_attach(dsi->panel, &dsi->conn);

	return 0;

errcode:
	drm_encoder_cleanup(&dsi->encoder);
	drm_connector_unregister(&dsi->conn);
	drm_connector_cleanup(&dsi->conn);

	return ret;
}

static void mtk_dsi_destroy_conn_enc(struct mtk_dsi *dsi)
{
	drm_encoder_cleanup(&dsi->encoder);
	drm_connector_unregister(&dsi->conn);
	drm_connector_cleanup(&dsi->conn);
}

static int mtk_dsi_bind(struct device *dev, struct device *master,
	void *data)
{
	int ret;
	struct mtk_dsi *dsi = NULL;

	dsi = platform_get_drvdata(to_platform_device(dev));
	if (!dsi) {
		ret = -EFAULT;
		goto errcode;
	}

	dsi->drm_dev = data;

	ret = mtk_dsi_create_conn_enc(dsi);
	if (ret) {
		DRM_ERROR("Encoder create  failed with %d\n", ret);
		return ret;
	}

	return 0;

errcode:
	return ret;
}

static void mtk_dsi_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct mtk_dsi *dsi = NULL;

	dsi = platform_get_drvdata(to_platform_device(dev));
	mtk_dsi_destroy_conn_enc(dsi);

	dsi->drm_dev = NULL;
}

static const struct component_ops mtk_dsi_component_ops = {
	.bind	= mtk_dsi_bind,
	.unbind	= mtk_dsi_unbind,
};

static int mtk_dsi_host_attach(struct mipi_dsi_host *host,
	struct mipi_dsi_device *device)
{
	struct mtk_dsi *dsi = host_to_mtk(host);

	dsi->mode_flags = device->mode_flags;
	dsi->format = device->format;
	dsi->lanes = device->lanes;

	dsi->panel = of_drm_find_panel(device->dev.of_node);
	if (dsi->panel) {
		if (dsi->conn.dev)
			drm_helper_hpd_irq_event(dsi->conn.dev);
	}

	return 0;
}

static int mtk_dsi_host_detach(struct mipi_dsi_host *host,
	struct mipi_dsi_device *device)
{
	struct mtk_dsi *dsi = host_to_mtk(host);

	if (dsi->panel && &device->dev == dsi->panel->dev) {
		if (dsi->conn.dev)
			drm_helper_hpd_irq_event(dsi->conn.dev);

		dsi->panel = NULL;
	}

	return 0;
}

static const struct mipi_dsi_host_ops mtk_dsi_host_ops = {
	.attach = mtk_dsi_host_attach,
	.detach = mtk_dsi_host_detach,
};

static int mtk_dsi_probe(struct platform_device *pdev)
{
	struct mtk_dsi *dsi = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *panel_node, *bridge_node, *endpoint;
	struct resource *regs;
	int ret;

	dsi = kzalloc(sizeof(*dsi), GFP_KERNEL);

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = 4;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (endpoint) {
		bridge_node = of_graph_get_remote_port_parent(endpoint);
		if (!bridge_node)
			return -EPROBE_DEFER;

		dsi->bridge = of_drm_find_bridge(bridge_node);
		of_node_put(bridge_node);
		if (!dsi->bridge)
			return -EPROBE_DEFER;
	}

	dsi->dsi0_engine_clk_cg = devm_clk_get(dev, "dsi0_engine_disp_ck");
	if (IS_ERR(dsi->dsi0_engine_clk_cg)) {
		dev_err(dev, "cannot get dsi0_engine_clk_cg\n");
		return PTR_ERR(dsi->dsi0_engine_clk_cg);
	}

	dsi->dsi0_digital_clk_cg = devm_clk_get(dev, "dsi0_digital_disp_ck");
	if (IS_ERR(dsi->dsi0_digital_clk_cg)) {
		dev_err(dev, "cannot get dsi0_digital_disp_ck\n");
		return PTR_ERR(dsi->dsi0_digital_clk_cg);
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->dsi_reg_base = devm_ioremap_resource(dev, regs);

	if (IS_ERR(dsi->dsi_reg_base)) {
		dev_err(dev, "cannot get dsi->dsi_reg_base\n");
		return PTR_ERR(dsi->dsi_reg_base);
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dsi->dsi_tx_reg_base = devm_ioremap_resource(dev, regs);
	if (IS_ERR(dsi->dsi_tx_reg_base)) {
		dev_err(dev, "cannot get dsi->dsi_tx_reg_base\n");
		return PTR_ERR(dsi->dsi_tx_reg_base);
	}

	panel_node = of_parse_phandle(dev->of_node, "mediatek,panel", 0);
	if (panel_node) {
		dsi->panel = of_drm_find_panel(panel_node);
		of_node_put(panel_node);
		if (!dsi->panel)
			return -EPROBE_DEFER;
	}

	platform_set_drvdata(pdev, dsi);

	ret = component_add(&pdev->dev, &mtk_dsi_component_ops);
	if (ret)
		goto err_del_component;

	return 0;

err_del_component:
	component_del(&pdev->dev, &mtk_dsi_component_ops);
	return -EPROBE_DEFER;
}

static int mtk_dsi_remove(struct platform_device *pdev)
{
	struct mtk_dsi *dsi = platform_get_drvdata(pdev);

	mtk_output_dsi_disable(dsi);
	component_del(&pdev->dev, &mtk_dsi_component_ops);

	return 0;
}

static const struct of_device_id mtk_dsi_of_match[] = {
	{ .compatible = "mediatek,mt8173-dsi" },
	{ },
};

struct platform_driver mtk_dsi_driver = {
	.probe = mtk_dsi_probe,
	.remove = mtk_dsi_remove,
	.driver = {
		.name = "mtk-dsi",
		.of_match_table = mtk_dsi_of_match,
		.owner	= THIS_MODULE,
	},
};


/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author:
 *  Zhigang.Wei <zhigang.wei@mediatek.com>
 *  Chunfeng.Yun <chunfeng.yun@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include "xhci-mtk.h"


#define SS_BW_BOUNDARY	51000
/* table 5-5. High-speed Isoc Transaction Limits in usb_20 spec */
#define HS_BW_BOUNDARY	6144
/* usb2 spec section11.18.1: at most 188 FS bytes per microframe */
#define FS_PAYLOAD_MAX 188

/* mtk scheduler bitmasks */
#define EP_BPKTS(p)	((p) & 0x3f)
#define EP_BCSCOUNT(p)	(((p) & 0x7) << 8)
#define EP_BBM(p)	((p) << 11)
#define EP_BOFFSET(p)	((p) & 0x3fff)
#define EP_BREPEAT(p)	(((p) & 0x7fff) << 16)

static int is_fs_or_ls(enum usb_device_speed speed)
{
	return speed == USB_SPEED_FULL || speed == USB_SPEED_LOW;
}

static int get_bw_index(struct xhci_hcd *xhci, struct usb_device *udev,
	struct usb_host_endpoint *ep)
{
	int bw_index;
	int port_id;
	struct xhci_virt_device *virt_dev;

	virt_dev = xhci->devs[udev->slot_id];
	port_id = virt_dev->real_port;

	if (udev->speed == USB_SPEED_SUPER) {
		if (usb_endpoint_dir_out(&ep->desc))
			bw_index = (port_id - 1) * 2;
		else
			bw_index = (port_id - 1) * 2 + 1;
	} else {
		bw_index = port_id + xhci->num_usb3_ports - 1;
	}

	return bw_index;
}


static void setup_sch_info(struct usb_device *udev,
		struct xhci_ep_ctx *ep_ctx, struct mu3h_sch_ep_info *sch_ep)
{
	u32 ep_type;
	u32 ep_interval;
	u32 max_packet_size;
	u32 max_burst;
	u32 mult;
	u32 esit_pkts;

	ep_type = CTX_TO_EP_TYPE(le32_to_cpu(ep_ctx->ep_info2));
	ep_interval = CTX_TO_EP_INTERVAL(le32_to_cpu(ep_ctx->ep_info));
	max_packet_size = MAX_PACKET_DECODED(le32_to_cpu(ep_ctx->ep_info2));
	max_burst = CTX_TO_MAX_BURST(le32_to_cpu(ep_ctx->ep_info2));
	mult = CTX_TO_EP_MULT(le32_to_cpu(ep_ctx->ep_info));
	pr_debug("%s: max_burst = %d, mult = %d\n", __func__, max_burst, mult);

	sch_ep->ep_type = ep_type;
	sch_ep->max_packet_size = max_packet_size;
	sch_ep->esit = 1 << ep_interval;
	sch_ep->offset = 0;
	sch_ep->burst_mode = 0;

	if (udev->speed == USB_SPEED_HIGH) {
		sch_ep->cs_count = 0;
		/*
		 * usb_20 spec section5.9
		 * a single microframe is enough for HS synchromous endpoints
		 * in a interval
		 */
		sch_ep->num_budget_microframes = 1;
		sch_ep->repeat = 0;
		/*
		 * xHCI spec section6.2.3.4
		 * @max_busrt is the number of additional transactions opportunities
		 * per microframe
		 */
		sch_ep->pkts = max_burst + 1;
		sch_ep->bw_cost_per_microframe = max_packet_size * sch_ep->pkts;
	} else if (udev->speed == USB_SPEED_SUPER) {
		/* usb3_r1 spec section4.4.7 & 4.4.8 */
		sch_ep->cs_count = 0;
		esit_pkts = (mult + 1) * (max_burst + 1);
		if (ep_type == INT_IN_EP || ep_type == INT_OUT_EP) {
			sch_ep->pkts = esit_pkts;
			sch_ep->num_budget_microframes = 1;
			sch_ep->repeat = 0;
		}

		if (ep_type == ISOC_IN_EP || ep_type == ISOC_OUT_EP) {
			if (esit_pkts <= sch_ep->esit)
				sch_ep->pkts = 1;
			else
				sch_ep->pkts = roundup_pow_of_two(esit_pkts)
					/ sch_ep->esit;

			sch_ep->num_budget_microframes =
				DIV_ROUND_UP(esit_pkts, sch_ep->pkts);

			if (sch_ep->num_budget_microframes > 1)
				sch_ep->repeat = 1;
			else
				sch_ep->repeat = 0;
		}
		sch_ep->bw_cost_per_microframe = max_packet_size * sch_ep->pkts;
	} else if (is_fs_or_ls(udev->speed)) {
		/*
		 * usb_20 spec section11.18.4
		 * assume worst cases
		 */
		sch_ep->repeat = 0;
		sch_ep->pkts = 1; /* at most one packet for each microframe */
		if (ep_type == INT_IN_EP || ep_type == INT_OUT_EP) {
			sch_ep->cs_count = 3; /* at most need 3 CS*/
			/* one for SS and one for budgeted transaction */
			sch_ep->num_budget_microframes = sch_ep->cs_count + 2;
			sch_ep->bw_cost_per_microframe = max_packet_size;
		}
		if (ep_type == ISOC_OUT_EP) {
			/* must never schedule a cs ISOC OUT ep */
			sch_ep->cs_count = 0;
			/*
			 * the best case FS budget assumes that 188 FS bytes
			 * occur in each microframe
			 */
			sch_ep->num_budget_microframes = DIV_ROUND_UP(
				sch_ep->max_packet_size, FS_PAYLOAD_MAX);
			sch_ep->bw_cost_per_microframe = FS_PAYLOAD_MAX;
		}
		if (ep_type == ISOC_IN_EP) {
			/* at most need additional two CS. */
			sch_ep->cs_count = DIV_ROUND_UP(
				sch_ep->max_packet_size, FS_PAYLOAD_MAX) + 2;
			sch_ep->num_budget_microframes = sch_ep->cs_count + 2;
			sch_ep->bw_cost_per_microframe = FS_PAYLOAD_MAX;
		}
	}
}

/* Get maximum bandwidth when we schedule at offset slot. */
static u32 get_max_bw(struct mu3h_sch_bw_info *sch_bw,
	struct mu3h_sch_ep_info *sch_ep, u32 offset)
{
	u32 num_esit;
	u32 max_bw = 0;
	int i;
	int j;

	num_esit = XHCI_MTK_MAX_ESIT / sch_ep->esit;
	for (i = 0; i < num_esit; i++) {
		u32 base = offset + i * sch_ep->esit;

		for (j = 0; j < sch_ep->num_budget_microframes; j++) {
			if (sch_bw->bus_bw[base + j] > max_bw)
				max_bw = sch_bw->bus_bw[base + j];
		}
	}
	return max_bw;
}

static void update_bus_bw(struct mu3h_sch_bw_info *sch_bw,
	struct mu3h_sch_ep_info *sch_ep, int bw_cost)
{
	u32 num_esit;
	u32 base;
	int i;
	int j;

	num_esit = XHCI_MTK_MAX_ESIT / sch_ep->esit;
	for (i = 0; i < num_esit; i++) {
		base = sch_ep->offset + i * sch_ep->esit;
		for (j = 0; j < sch_ep->num_budget_microframes; j++)
			sch_bw->bus_bw[base + j] += bw_cost;
	}

}

static void debug_sch_ep(struct mu3h_sch_ep_info *sch_ep)
{
	pr_debug("sch_ep->ep_type = %d\n", sch_ep->ep_type);
	pr_debug("sch_ep->max_packet_size = %d\n", sch_ep->max_packet_size);
	pr_debug("sch_ep->esit = %d\n", sch_ep->esit);
	pr_debug("sch_ep->num_budget_microframes = %d\n",
			sch_ep->num_budget_microframes);
	pr_debug("sch_ep->bw_cost_per_microframe = %d\n",
			sch_ep->bw_cost_per_microframe);
	pr_debug("sch_ep->ep = %p\n", sch_ep->ep);
	pr_debug("sch_ep->offset = %d\n", sch_ep->offset);
	pr_debug("sch_ep->repeat = %d\n", sch_ep->repeat);
	pr_debug("sch_ep->pkts = %d\n", sch_ep->pkts);
	pr_debug("sch_ep->cs_count = %d\n", sch_ep->cs_count);
}

static int check_sch_bw(struct usb_device *udev,
	struct mu3h_sch_bw_info *sch_bw, struct mu3h_sch_ep_info *sch_ep)
{
	u32 offset;
	u32 esit;
	u32 num_budget_microframes;
	u32 min_bw;
	u32 min_index;
	u32 worst_bw;
	u32 bw_boundary;

	if (sch_ep->esit > XHCI_MTK_MAX_ESIT)
		sch_ep->esit = XHCI_MTK_MAX_ESIT;

	esit = sch_ep->esit;
	num_budget_microframes = sch_ep->num_budget_microframes;

	/*
	 * Search through all possible schedule microframes.
	 * and find a microframe where its worst bandwidth is minimum.
	 */
	min_bw = ~0;
	min_index = 0;
	for (offset = 0; offset < esit; offset++) {
		if ((offset + num_budget_microframes) > sch_ep->esit)
			break;
		/*
		 * usb_20 spec section11.18:
		 * must never schedule Start-Split in Y6
		 */
		if (is_fs_or_ls(udev->speed) && (offset % 8 == 6))
			continue;

		worst_bw = get_max_bw(sch_bw, sch_ep, offset);
		if (min_bw > worst_bw) {
			min_bw = worst_bw;
			min_index = offset;
		}
		if (min_bw == 0)
			break;
	}
	sch_ep->offset = min_index;

	debug_sch_ep(sch_ep);

	bw_boundary = (udev->speed == USB_SPEED_SUPER)
				? SS_BW_BOUNDARY : HS_BW_BOUNDARY;

	/* check bandwidth */
	if (min_bw + sch_ep->bw_cost_per_microframe > bw_boundary)
		return -1;

	/* update bus bandwidth info */
	update_bus_bw(sch_bw, sch_ep, sch_ep->bw_cost_per_microframe);

	return 0;
}

static void debug_sch_bw(struct mu3h_sch_bw_info *sch_bw)
{
	int i;

	pr_debug("xhci_mtk_scheduler :bus_bw_info\n");
	for (i = 0; i < XHCI_MTK_MAX_ESIT; i++)
		pr_debug("%d  ", sch_bw->bus_bw[i]);

	pr_debug("\n");
}


static int need_bw_sch(struct usb_host_endpoint *ep,
	enum usb_device_speed speed, int has_tt)
{
	/* only for synchronous endpoints */
	if (usb_endpoint_xfer_control(&ep->desc)
		|| usb_endpoint_xfer_bulk(&ep->desc))
		return 0;
	/*
	 * for LS & FS synchronous endpoints which its device don't attach
	 * to TT are also ignored, root-hub will schedule them directly
	 */
	if (is_fs_or_ls(speed) && !has_tt)
		return 0;

	return 1;
}

int xhci_mtk_init_quirk(struct xhci_hcd *xhci)
{
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	struct device *dev = hcd->self.controller;
	struct mu3h_sch_bw_info *sch_array;
	size_t array_size;
	int num_usb_bus;
	int i;

	/* ss IN and OUT are separated */
	num_usb_bus = xhci->num_usb3_ports * 2 + xhci->num_usb2_ports;
	array_size = sizeof(*sch_array) * num_usb_bus;

	sch_array = kzalloc(array_size, GFP_KERNEL);
	if (sch_array == NULL)
		return -ENOMEM;

	for (i = 0; i < num_usb_bus; i++)
		INIT_LIST_HEAD(&sch_array[i].bw_ep_list);

	dev->platform_data = sch_array;
	xhci->quirks |= XHCI_MTK_HOST;
	/*
	 * MTK host controller gives a spurious successful event after a
	 * short transfer. Ignore it.
	 */
	xhci->quirks |= XHCI_SPURIOUS_SUCCESS;

	return 0;
}


void xhci_mtk_exit_quirk(struct xhci_hcd *xhci)
{
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	struct mu3h_sch_bw_info *sch_array;

	sch_array = dev_get_platdata(hcd->self.controller);
	kfree(sch_array);
}


int xhci_mtk_add_ep_quirk(struct usb_hcd *hcd, struct usb_device *udev,
		struct usb_host_endpoint *ep)
{
	int ret = 0;
	int port_id;
	int bw_index;
	struct xhci_hcd *xhci;
	unsigned int ep_index;
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_slot_ctx *slot_ctx;
	struct xhci_virt_device *virt_dev;
	struct mu3h_sch_bw_info *sch_bw;
	struct mu3h_sch_ep_info *sch_ep;
	struct mu3h_sch_bw_info *sch_array;

	xhci = hcd_to_xhci(hcd);
	virt_dev = xhci->devs[udev->slot_id];
	ep_index = xhci_get_endpoint_index(&ep->desc);
	slot_ctx = xhci_get_slot_ctx(xhci, virt_dev->in_ctx);
	ep_ctx = xhci_get_ep_ctx(xhci, virt_dev->in_ctx, ep_index);
	sch_array = dev_get_platdata(hcd->self.controller);

	port_id = virt_dev->real_port;
	xhci_dbg(xhci, "%s() xfer_type: %d, speed:%d, ep:%p\n", __func__,
		usb_endpoint_type(&ep->desc), udev->speed, ep);

	if (!need_bw_sch(ep, udev->speed, slot_ctx->tt_info & TT_SLOT))
		return 0;

	bw_index = get_bw_index(xhci, udev, ep);
	sch_bw = &sch_array[bw_index];

	sch_ep = kzalloc(sizeof(struct mu3h_sch_ep_info), GFP_KERNEL);
	if (!sch_ep)
		return -ENOMEM;

	setup_sch_info(udev, ep_ctx, sch_ep);

	ret = check_sch_bw(udev, sch_bw, sch_ep);
	if (ret) {
		xhci_err(xhci, "Not enough bandwidth!\n");
		kfree(sch_ep);
		return -ENOSPC;
	}

	list_add_tail(&sch_ep->endpoint, &sch_bw->bw_ep_list);
	sch_ep->ep = ep;

	ep_ctx->reserved[0] |= cpu_to_le32(EP_BPKTS(sch_ep->pkts)
		| EP_BCSCOUNT(sch_ep->cs_count) | EP_BBM(sch_ep->burst_mode));
	ep_ctx->reserved[1] |= cpu_to_le32(EP_BOFFSET(sch_ep->offset)
		| EP_BREPEAT(sch_ep->repeat));

	debug_sch_bw(sch_bw);
	return 0;
}

void xhci_mtk_drop_ep_quirk(struct usb_hcd *hcd, struct usb_device *udev,
		struct usb_host_endpoint *ep)
{
	int bw_index;
	struct xhci_hcd *xhci;
	struct xhci_slot_ctx *slot_ctx;
	struct xhci_virt_device *virt_dev;
	struct mu3h_sch_bw_info *sch_array;
	struct mu3h_sch_bw_info *sch_bw;
	struct mu3h_sch_ep_info *sch_ep;

	xhci = hcd_to_xhci(hcd);
	virt_dev = xhci->devs[udev->slot_id];
	slot_ctx = xhci_get_slot_ctx(xhci, virt_dev->in_ctx);
	sch_array = dev_get_platdata(hcd->self.controller);

	xhci_dbg(xhci, "%s() xfer_type: %d, speed:%d, ep:%p\n", __func__,
		usb_endpoint_type(&ep->desc), udev->speed, ep);

	if (!need_bw_sch(ep, udev->speed, slot_ctx->tt_info & TT_SLOT))
		return;

	bw_index = get_bw_index(xhci, udev, ep);
	sch_bw = &sch_array[bw_index];

	list_for_each_entry(sch_ep, &sch_bw->bw_ep_list, endpoint) {
		if (sch_ep->ep == ep) {
			update_bus_bw(sch_bw, sch_ep,
				-sch_ep->bw_cost_per_microframe);
			list_del(&sch_ep->endpoint);
			kfree(sch_ep);
			break;
		}
	}
	debug_sch_bw(sch_bw);
}


/*
 * The TD size is the number of bytes remaining in the TD (including this TRB),
 * right shifted by 10.
 * It must fit in bits 21:17, so it can't be bigger than 31.
 */
u32 xhci_mtk_td_remainder_quirk(unsigned int td_running_total,
	unsigned trb_buffer_length, struct urb *urb)
{
	u32 max = 31;
	int remainder, td_packet_count, packet_transferred;
	unsigned int td_transfer_size = urb->transfer_buffer_length;
	unsigned int maxp;

	maxp = GET_MAX_PACKET(usb_endpoint_maxp(&urb->ep->desc));

	/* 0 for the last TRB */
	if (td_running_total + trb_buffer_length == td_transfer_size)
		return 0;

	packet_transferred = td_running_total / maxp;
	td_packet_count = DIV_ROUND_UP(td_transfer_size, maxp);
	remainder = td_packet_count - packet_transferred;

	if (remainder > max)
		return max << 17;
	else
		return remainder << 17;
}



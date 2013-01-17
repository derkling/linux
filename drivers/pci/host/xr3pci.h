/*
 * XpressRICH3-AXI PCIe Host Bridge Driver.
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Andrew Murray <andrew.murray@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef __XPRESS_RICH3_H__
#define __XPRESS_RICH3_H__

/* The XpressRICH3 requires that once the source of an INTx has been cleared,
 * it must also be cleared via the XpressRICH3's register set. This quirk
 * cascades each of the interrupts and provides the additional required
 * handling.
 */
#define FPGA_QUIRK_INTX_CLEAR

/* The XpressRIC3 doesn't describe itself as a bridge. This is required for
 * correct/normal enumeration. This quirk changes that.
 */
#define FPGA_QUIRK_FPGA_CLASS

/* Host Bridge Identification */
#define DEVICE_NAME "XpressRICH3-AXI PCIe Host Bridge"
#define DEVICE_VENDOR_ID  0x1556
#define DEVICE_DEVICE_ID  0x1100

/* Bridge Configuration Space Registers */
#define BRIDGE_INT_REGS		0x0
#define BRIDGE_PCIE_CONFIG	0x1000
#define BRIDGE_EXT_REGS		0x2000

/* Bridge Configuration Space Register Definations */
/* Bridge Internal Registers */
/* Control and Status Registers */
#define BRIDGE_VER		0x0
#define BRIDGE_BUS		0x4
#define BRIDGE_IMPL_F		0x8
#define PCIE_IF_CONF		0x10
#define PCIE_BASIC_CONF		0x14
#define PCIE_BASIC_STATU	0x18
#define AXI_MSTL_CONF		0x20
#define AXI_SLVL_CONF		0x24
#define AXI_MSTD_CONF		0x28
#define AXI_MST0_CONF		0x30
#define AXI_SLV0_CONF		0x34
#define AXI_MST1_CONF		0x38
#define AXI_SLV1_CONF		0x3c
#define AXI_MST2_CONF		0x40
#define AXI_SLV2_CONF		0x44
#define AXI_MST3_CONF		0x48
#define AXI_SLV3_CONF		0x4c
#define AXI_STRO0_CONF		0x50
#define AXI_STRI0_CONF		0x54
#define AXI_STRO1_CONF		0x58
#define AXI_STRI1_CONF		0x5c
#define AXI_STRO2_CONF		0x60
#define AXI_STRI2_CONF		0x64
#define AXI_STRO3_CONF		0x68
#define AXI_STRI3_CONF		0x6c
#define GEN_SETTINGS		0x80
#define PCIE_CFGCTRL		0x84
#define PCIE_PIPE		0x8c
#define PCIE_PIPE_1		0x88
#define PCIE_VC_CRED		0x90
#define PCIE_PCI_IDS_1		0x98
#define PCIE_PCI_IDS_2		0xa0
#define PCIE_PCI_LPM		0xa4
#define PCIE_PCI_IRQ		0xa8
#define PCIE_PEX_DEV		0xc0
#define PCIE_PEX_DEV2		0xc4
#define PCIE_PEX_LINK		0xc8
#define PCIE_PEX_SLOT		0xcc
#define PCIE_PEX_ROOT_VC	0xd0
#define PCIE_PEX_SPC		0xd4
#define PCIE_PEX_SPC2		0xd8
#define PCIE_PEX_NFTS		0xdc
#define PCIE_BAR_WIN		0xfc
#define PCIE_EQ_PRESET		0x100
#define PCIE_SRIOV		0x120
#define PCIE_CFGNUM		0x140
#define PM_CONF			0x174

#define IMASK_LOCAL		0x180
#define ISTATUS_LOCAL		0x184
#define ISTATUS_MSI		0x194

#define INT_A			(1 << 24)
#define INT_B			(1 << 25)
#define INT_C			(1 << 26)
#define INT_D			(1 << 27)
#define INT_MSI			(1 << 28)
#define INT_INTX		(INT_A | INT_B | INT_C | INT_D)

#define ATR_PCIE_WIN0		0x600
#define ATR_AXI4_SLV0		0x800
#define ATR_TBL_1		0x0
#define ATR_SRC_ADDR_LWR	0x0
#define ATR_SRC_ADDR_UPR	0x4
#define ATR_TRSL_ADDR_LWR	0x8
#define ATR_TRSL_ADDR_UPR	0xc
#define ATR_TRSL_PARAM		0x10

/* Macros for populating PCIE_CFGNUM register of the Bridge Configuration Space.
   This register is used to specify the target of configuration read/writes and
   byte enables when targetting configuration space through the Bridge
   Configuration Space's PCI configuration Space window. */
#define PCIE_CFGNUM_FORCE_BE(fo)	(((fo) &  0x1) << 20)
#define PCIE_CFGNUM_BYTE_EN(be)		(((be) &  0xf) << 16)
#define PCIE_CFGNUM_BUS(b)		(((b)  & 0xff) <<  8)
#define PCIE_CFGNUM_DEV(d)		(((d)  & 0x1f) <<  3)
#define PCIE_CFGNUM_FUN(f)		(((f)  &  0x7) <<  0)

#define PCIE_CFGNUM_R(b, d, f, be, fo)	(PCIE_CFGNUM_BUS(b) | \
					 PCIE_CFGNUM_DEV(d) | \
					 PCIE_CFGNUM_FUN(f) | \
					 PCIE_CFGNUM_BYTE_EN(be) | \
					 PCIE_CFGNUM_FORCE_BE(fo))



#ifdef FPGA_QUIRK_INTX_CLEAR
static struct irq_domain_ops xr3pci_irq_nop_ops = { };
static void xr3pci_irq_nop(struct irq_data *data) { }

static struct irq_chip xr3pci_irq_nop_chip = {
	.name	= "Xpress-RICH3 INTx",
	.irq_ack = xr3pci_irq_nop,
	.irq_enable = xr3pci_irq_nop,
	.irq_disable = xr3pci_irq_nop,
	.irq_mask =  xr3pci_irq_nop,
	.irq_unmask = xr3pci_irq_nop,
};
#endif

int __init xr3pci_setup_arch(
	struct platform_device *dev,
	int (*map_irq)(const struct pci_dev *, u8, u8),
	struct pci_ops *ops,
	int (*setup)(int nr, struct pci_sys_data *));

#endif

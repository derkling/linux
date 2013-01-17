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
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>

#include <asm/mach/irq.h>

#include "xr3pci.h"

struct xr3pci_port {
	void __iomem	*base;
#ifdef FPGA_QUIRK_INTX_CLEAR
	struct irq_domain *intx_irq_domain;
#endif
};

static int xr3pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	u32 cfgnum;
	struct pci_sys_data *sys = bus->sysdata;
	struct xr3pci_port *pp = sys->private_data;

	/* Specify target of configuration read */
	cfgnum = PCIE_CFGNUM_R(bus->number,
			       PCI_SLOT(devfn),
			       PCI_FUNC(devfn),

			     /* AXI doesn't offer a read strobe, to prevent
				side effects from un-targetted reads we always
				use the PCIe BE bits. */
			     (~(0xf << size) << (where & 3)), /* BE */
			     0x1); /* force PCIe BE */

	writel(cfgnum, pp->base + PCIE_CFGNUM);
	*val = readl(pp->base + BRIDGE_PCIE_CONFIG + (where & ~0x3));

	if (*val == ~0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	switch (size) {
	case 1:
		*val = (*val >> (8 * (where & 3))) & 0xff;
		break;

	case 2:
		*val = (*val >> (8 * (where & 3))) & 0xffff;
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

/**
 * Stimulate a configuration write request
 */
static int xr3pci_write_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val)
{
	u32 cfgnum;
	struct pci_sys_data *sys = bus->sysdata;
	struct xr3pci_port *pp = sys->private_data;

	cfgnum = PCIE_CFGNUM_R(bus->number,
			       PCI_SLOT(devfn),
			       PCI_FUNC(devfn),

			     /* Utilise and force the BE to be consistent with
				configuration read behaviour */
			     (~(0xf << size) << (where & 3)), /* BE */
			     0x1); /* force PCIe BE */

	writel(cfgnum, pp->base + PCIE_CFGNUM);
	writel(val << ((where & 3) * 8),
		pp->base + BRIDGE_PCIE_CONFIG + (where & ~0x3));

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops xr3pci_ops = {
	.read	= xr3pci_read_config,
	.write	= xr3pci_write_config,
};

#ifdef FPGA_QUIRK_INTX_CLEAR
static void xr3pci_intx_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long status, flags;
	struct irq_chip *chip = irq_get_chip(irq);
	struct xr3pci_port *pp = irq_desc_get_handler_data(desc);

	/* this handler is used for each INTx interrupt source, prevent
	 * duplicate calls to generic_handle_irq with spin lock
	 */
	static DEFINE_SPINLOCK(irq_lock);

	chained_irq_enter(chip, desc);
	spin_lock_irqsave(&irq_lock, flags);
	status = (readl(pp->base + ISTATUS_LOCAL) & INT_INTX) >> 24;

	/* handle all pending INTx interrupts */
	while (status) {
		u8 pin = find_first_bit(&status, 4);
		int virq = irq_linear_revmap(pp->intx_irq_domain, pin);
		if (virq != 0)
			generic_handle_irq(virq);

		writel((1 << (pin + 24)), pp->base + ISTATUS_LOCAL);
		status = (readl(pp->base + ISTATUS_LOCAL) & INT_INTX) >> 24;
	}

	spin_unlock_irqrestore(&irq_lock, flags);
	chained_irq_exit(chip, desc);
}
#endif

static int __init xr3pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int virq;
	struct of_irq out;
	struct pci_sys_data *sys = dev->sysdata;
	struct xr3pci_port *pp = sys->private_data;

	if (of_irq_map_pci(dev, &out)) {
		pr_err(DEVICE_NAME ": Unable to map PCI IRQ\n");
		return -1;
	}

	virq = irq_create_of_mapping(out.controller, out.specifier,
					     out.size);
	if (!virq) {
		pr_err(DEVICE_NAME ": Unable to create IRQ mapping\n");
		return -1;
	}

#ifdef FPGA_QUIRK_INTX_CLEAR
	/* set up common chained handler for INTx interrutps */
	irq_set_chained_handler(virq, xr3pci_intx_irq_handler);
	irq_set_handler_data(virq, pp);

	/* create interrupt for INTx users to request which though its cascade
	   will correctly clear interrupt registers in XpressRICH3 */
	virq = irq_create_mapping(pp->intx_irq_domain, pin-1);
	irq_set_chip_and_handler(virq, &xr3pci_irq_nop_chip, handle_simple_irq);
#endif

	return virq;
}

static int xr3pci_check_device(struct xr3pci_port *pp)
{
	u32 ver;

	ver = readl(pp->base + PCIE_PCI_IDS_1);
	if (!((ver  & 0xffff) == DEVICE_VENDOR_ID &&
	      (ver  & 0xffff0000) >> 16 == DEVICE_DEVICE_ID)) {
		pr_err("Unable to detect " DEVICE_NAME);
		return -1;
	}

	ver = readl(pp->base + PCIE_BASIC_STATU);
	pr_info(DEVICE_NAME " %dx gen %d link negotiated\n", ver & 0xff,
				(ver & 0xf00) >> 8);

	if (!(ver & 0xff))
		pr_err(DEVICE_NAME ": No link detected\n");

	if ((readl(pp->base + PCIE_BASIC_CONF) & 0xf0000000) != 0x10000000) {
		pr_err(DEVICE_NAME ": Core is not a RC\n");
		return -1;
	}

	return 0;
}

static int xr3pci_setup_ats(struct xr3pci_port *pp)
{
	/* set up RC address translation (assuming that all tables default to
	   disabled/un-implemented) */

	/* 1:1 mapping for inbound PCIe transactions to AXI slave 0 */
	writel(0x7f, pp->base + ATR_PCIE_WIN0 + ATR_TBL_1 + ATR_SRC_ADDR_LWR);
	writel(0x0, pp->base + ATR_PCIE_WIN0 + ATR_TBL_1 + ATR_SRC_ADDR_UPR);
	writel(0x0, pp->base + ATR_PCIE_WIN0 + ATR_TBL_1 + ATR_TRSL_ADDR_UPR);
	writel(0x4, pp->base + ATR_PCIE_WIN0 + ATR_TBL_1 + ATR_TRSL_PARAM);
	writel(0x0, pp->base + ATR_PCIE_WIN0 + ATR_TBL_1 + ATR_TRSL_ADDR_LWR);

	/* 1:1 mapping for outbound AXI slave 0 transcations to PCIe */
	writel(0x7f, pp->base + ATR_AXI4_SLV0 + ATR_TBL_1 + ATR_SRC_ADDR_LWR);
	writel(0x0, pp->base + ATR_AXI4_SLV0 + ATR_TBL_1 + ATR_SRC_ADDR_UPR);
	writel(0x0, pp->base + ATR_AXI4_SLV0 + ATR_TBL_1 + ATR_TRSL_ADDR_UPR);
	writel(0x0, pp->base + ATR_AXI4_SLV0 + ATR_TBL_1 + ATR_TRSL_PARAM);
	writel(0x0, pp->base + ATR_AXI4_SLV0 + ATR_TBL_1 + ATR_TRSL_ADDR_LWR);

	return 0;
}

static int xr3pci_setup_int(struct xr3pci_port *pp)
{
#ifdef FPGA_QUIRK_INTX_CLEAR
	pp->intx_irq_domain = irq_domain_add_linear(NULL, 4,
					&xr3pci_irq_nop_ops, NULL);
	if (!pp->intx_irq_domain) {
		pr_err(DEVICE_NAME ": Failed to create IRQ domain\n");
		return -1;
	}
#endif

	/* Enable IRQs for MSIs and legacy interrupts */
	writel(INT_MSI | INT_INTX, pp->base + IMASK_LOCAL);

	return 0;
}

int xr3pci_add_pci_resources(struct xr3pci_port *pp, struct pci_sys_data *sys)
{
	const __be32 *last = NULL;
	struct device_node *np = sys->of_node;
	struct resource *res = kzalloc(sizeof(struct resource), GFP_KERNEL);

	while ((last = of_pci_process_ranges(np, res, last))) {
		if (res->flags & IORESOURCE_MEM) {
			if (request_resource(&iomem_resource, res)) {
				pr_err(DEVICE_NAME
				": Failed to request PCIe memory\n");
				continue;
			}
			pci_add_resource_offset(&sys->resources, res,
						sys->mem_offset);
		} else if (res->flags & IORESOURCE_IO) {
			if (request_resource(&ioport_resource, res)) {
				pr_err(DEVICE_NAME
				": Failed to request PCIe IO\n");
				continue;
			}
			pci_add_resource_offset(&sys->resources, res,
						sys->io_offset);
		}

		res = kzalloc(sizeof(struct resource), GFP_KERNEL);
	}
	kfree(res);
	return 0;
}

static int xr3pci_get_resources(struct xr3pci_port *pp, struct device_node *np)
{
	int err;
	struct resource res;

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		pr_err(DEVICE_NAME
			": Failed to find configuration registers in DT\n");
		return err;
	}

	if (!request_mem_region(res.start, resource_size(&res), DEVICE_NAME)) {
		pr_err(DEVICE_NAME
			": Failed to request config registers resource\n");
		return -EBUSY;
	}

	pp->base = ioremap_nocache(res.start, resource_size(&res));
	if (!pp->base) {
		pr_err(DEVICE_NAME
			": Failed to map configuration registers resource\n");
		err = -ENOMEM;
		goto err_map_io;
	}

	return 0;

err_map_io:
	release_mem_region(res.start, resource_size(&res));

	return err;
}

void __init xr3pci_teardown(struct xr3pci_port *pp, struct device_node *np)
{
	int err;
	struct resource res;

	if (pp->base) {
		iounmap(pp->base);

		err = of_address_to_resource(np, 0, &res);
		if (!err)
			release_mem_region(res.start, resource_size(&res));
	}

	if (pp->intx_irq_domain)
		irq_domain_remove(pp->intx_irq_domain);
}

int __init xr3pci_setup(int nr, struct pci_sys_data *sys)
{
	struct xr3pci_port *pp;
	struct device_node *np = sys->of_node;

	pp = kzalloc(sizeof(struct xr3pci_port), GFP_KERNEL);
	WARN_ON(!pp);

	sys->private_data = pp;

	if (xr3pci_get_resources(pp, np))
		goto err;

	if (xr3pci_check_device(pp))
		goto err;

	if (xr3pci_setup_int(pp))
		goto err;

	if (xr3pci_setup_ats(pp))
		goto err;

	if (xr3pci_add_pci_resources(pp, sys))
		goto err;

	return 1;
err:
	xr3pci_teardown(pp, np);
	return -1;
}

#ifdef FPGA_QUIRK_FPGA_CLASS
static void xr3pci_quirk_class(struct pci_dev *pdev)
{
	pdev->class = PCI_CLASS_BRIDGE_PCI << 8;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLDA, PCI_DEVICE_ID_XR3PCI,
			xr3pci_quirk_class);
#endif

static int __init xr3pci_probe(struct platform_device *dev)
{
	/* At present there is no way to register PCI host drivers in a common
	   architecture agnostic way. We work around this by calling our own
	   arch call back which provides methods that can be plumbed into
	   whichever architecture this is used on. */
	return xr3pci_setup_arch(dev,
				 xr3pci_map_irq, &xr3pci_ops, xr3pci_setup);
	return 0;
}

static const struct __initconst of_device_id xr3pci_device_id[] = {
	{ .compatible = "arm,xr3pci", },
	{},
};
MODULE_DEVICE_TABLE(of, xr3pci_device_id);

static struct platform_driver xr3pci_driver = {
	.driver		= {
		.name	= "xr3pci",
		.owner	= THIS_MODULE,
		.of_match_table = xr3pci_device_id,
	},
};

static int __init xr3pci_init(void)
{
	return platform_driver_probe(&xr3pci_driver, xr3pci_probe);
}

subsys_initcall(xr3pci_init);

MODULE_AUTHOR("Andrew Murray <Andrew.Murray@arm.com>");
MODULE_DESCRIPTION("XR3PCI Host Bridge drver");
MODULE_LICENSE("GPL v2");

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
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/signal.h>
#include <asm/mach/pci.h>
#include <asm/mach/irq.h>

#include "xr3pci.h"

//TODO: remove or use properly
#define pr_debug printk

/* The XpressRICH3 requires that once the source of an INTx has been cleared,
 * it must also be cleared via the XpressRICH3's register set. This quirk
 * cascades each of the interrupts and provides the additional required
 * handling.
 */
#define FPGA_QUIRK_INTX_CLEAR

/* per controller structure */
struct pcie_port {
	//TODO: rather than store lots of fields used once - pass the device tree
	//node to the hw_pci.setup function
	void __iomem	*base;
	struct resource resource[5];
	int numresources;
	spinlock_t conf_lock;
#ifdef FPGA_QUIRK_INTX_CLEAR
	struct irq_domain *intx_irq_domain;
#endif
};
static int __init xr3pci_get_resources(struct pcie_port *pp, struct device_node *np);

/**
 * Stimulate a configuration read request
 */
static int xr3pci_read_config(struct pci_bus *bus, unsigned int devfn, int where,
			int size, u32 *val)
{
	u32 cfgnum;
	unsigned long flags;
	struct pci_sys_data *sys = bus->sysdata;
	struct pcie_port *pp = sys->private_data;

	/* We expect requests which are aligned to the request size which is
	   either 1, 2 or 4 bytes. We also expect this function to be called
	   exclusively through the pci_read_config_(byte|word|dword) accessors
           - these wrapper functions check alignment for us. They also provide
           serialisation of PCI configuration space accesses which prevent the
	   need for locking here */

 	//TODO: extra guards during development
	WARN(size + (where & 3) > 4, "CfgRd spans DWORD boundary\n");
	WARN(size == 3, "CfgRd size is unexpected (3)\n");

	/* Specify target of configuration read */
	cfgnum = PCIE_CFGNUM_R(bus->number,     /* bus */
			       PCI_SLOT(devfn), /* device */
			       PCI_FUNC(devfn), /* function */

			     /* AXI doesn't offer a read strobe, to prevent
				side effects from un-targetted reads we always
				use the PCIe BE bits. */
			     (~(0xf << size) << (where & 3)), /* BE */
			     0x1); /* force PCIe BE */

	/* read from PCIe configuration space */
	//TODO: locking may not be required due to pci_lock in
	//	drivers/pci/access.c, but left in during development
	//writel(cfgnum, pp->base + PCIE_CFGNUM);
spin_lock_irqsave(&pp->conf_lock, flags);
	writew(cfgnum, pp->base + PCIE_CFGNUM);
	writeb((cfgnum >> 16), pp->base + PCIE_CFGNUM + 2);
	*val = readl(pp->base + BRIDGE_PCIE_CONFIG + (where & ~0x3));
spin_unlock_irqrestore(&pp->conf_lock, flags);

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
static int xr3pci_write_config(struct pci_bus *bus, unsigned int devfn, int where,
			     int size, u32 val)
{
	u32 cfgnum;
	unsigned long flags;
	struct pci_sys_data *sys = bus->sysdata;
	struct pcie_port *pp = sys->private_data;

	/* We expect requests which are aligned to the request size which is
	   either 1, 2 or 4 bytes. We also expect this function to be called
	   exclusively through the pci_write_config_(byte|word|dword) accessors
           - these wrapper functions check alignment for us. They also provide
           serialisation of PCI configuration space accesses which prevent the
	   need for locking here */

 	//TODO: extra guards during development
	WARN(size + (where & 3) > 4, "CfgWr spans DWORD boundary\n");
	WARN(size == 3, "CfgWr size is unexpected (3)\n");

	/* Specify target of configuration write */
	cfgnum = PCIE_CFGNUM_R(bus->number,     /* bus */
			       PCI_SLOT(devfn), /* device */
			       PCI_FUNC(devfn), /* function */

			     /* Utilise and force the BE to be consistent with
				configuration read behaviour */
			     //TODO: use unaligned accesses?
			     (~(0xf << size) << (where & 3)), /* BE */
			     0x1); /* force PCIe BE */

	/* write to the configuration space */
	//TODO: locking may not be required due to pci_lock in
	//	drivers/pci/access.c, but left in during development
	//r3pci_write(cfgnum, pp->base + PCIE_CFGNUM);
spin_lock_irqsave(&pp->conf_lock, flags);
	writew(cfgnum, pp->base + PCIE_CFGNUM);
	writeb((cfgnum >> 16), pp->base + PCIE_CFGNUM + 2);
	writel(val << ((where & 3) * 8), pp->base + BRIDGE_PCIE_CONFIG + (where & ~0x3));
spin_unlock_irqrestore(&pp->conf_lock, flags);

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops xr3pci_ops = {
	.read 	= xr3pci_read_config,
	.write 	= xr3pci_write_config, 
};

#ifdef FPGA_QUIRK_INTX_CLEAR
static struct irq_domain_ops irq_ops = { };
static void irq_nop(struct irq_data *data) { }

static struct irq_chip irq_chip = {
	.name	= "Xpress-RICH3 INTx",
	.irq_ack = irq_nop,
	.irq_enable = irq_nop,
	.irq_disable = irq_nop,
	.irq_mask =  irq_nop,
	.irq_unmask = irq_nop,
};

static void xr3pci_msix_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long status, flags;
	struct irq_chip *chip = irq_get_chip(irq);
	struct pcie_port *pp = irq_desc_get_handler_data(desc);
	
	/* this handler is used for each INTx interrupt source, prevent
	 * duplicate calls to generic_handle_irq with spin lock
	 */
	static DEFINE_SPINLOCK(irq_lock);
	
	chained_irq_enter(chip, desc);
	spin_lock_irqsave(&irq_lock, flags);
	status = (readl(pp->base + ISTATUS_LOCAL) & INT_INTX) >> 24;

	/* handle all pending MSIx interrupts */
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
	struct pcie_port *pp = sys->private_data;

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
	/* set up common chained handler for MSIx interrutps */
	irq_set_chained_handler(virq, xr3pci_msix_irq_handler);
	irq_set_handler_data(virq, pp);

	/* create interrupt for INTx users to request which though its cascade
	 * will correctly clear interrupt registers in XpressRICH3
	 */
	virq = irq_create_mapping(pp->intx_irq_domain, pin-1);
	irq_set_chip_and_handler(virq, &irq_chip, handle_simple_irq);
#endif

	return virq;
}

static int __init xr3pci_probe(struct pcie_port *pp)
{
	/* gain some confidence that we are talking to the correct device by
	   reading registers with known values */
	u32 ver;

	/* verify hardwired vendor, device, revision IDs */
	//TODO: PLDA document doesn't make clear how this applies to root ports
	ver = readl(pp->base + PCIE_PCI_IDS_1);
	//TODO: this doesn't work IDS_1 is 0xa! 
	if (!((ver  & 0xffff) == DEVICE_VENDOR_ID &&
	      (ver  & 0xffff0000) >> 16 == DEVICE_DEVICE_ID)) {
		printk("Unable to detect " DEVICE_NAME);
		return -1;
	}

	ver = readl(pp->base + PCIE_BASIC_STATU);
	printk(DEVICE_NAME " %dx gen %d link negotiated\n", ver & 0xff, (ver & 0xf00) >> 8);
	if (!(ver & 0xff)) {
		printk(DEVICE_NAME ": No link detected\n");
		return -1;
	}
	
	/* top nibble describes if core is configured as native endpoint or root port */
	if ((readl(pp->base + PCIE_BASIC_CONF) & 0xf0000000) != 0x10000000) {
		printk(DEVICE_NAME ": Core is not a RC\n");
		return -1;
	}

//	writel(0x7f, pp->base + 0x800);
///	writel(0x7f, pp->base + 0x600);
///	writeb(0x4, pp->base + 0x610);
	
	//colin test
	printk("LOOP BACK COLIN TEST ENABLED\n");
//	writeb(0x4, pp->base + 0x810);
//	writel(0x4000000b, pp->base + 0x800);
//	writel(0xc0000000, pp->base + 0x808);
	printk("DONE\n");

	return 0;
}

int __init xr3pci_setup(struct pci_sys_data *sys, struct device_node *np)
{
	int x=0;
	struct pcie_port *pp;

//	WARN_ON(nr);
//	if (nr >= xr3pci_hw_pci.nr_controllers)
//		return 0;

//	pp = &pcie_port[nr];
	pp = kzalloc(sizeof(struct pcie_port), GFP_KERNEL);
	WARN_ON(!pp);

	sys->private_data = pp;
		spin_lock_init(&pp->conf_lock);
	
		

	/* add DT resources to the controller */
	if (xr3pci_get_resources(pp, np)) {
		printk("oopp\n");
	}

	if (xr3pci_probe(pp)) {
		return -1;
	}

	/* Enable IRQs for MSIs and legacy interrupts */
	writel(INT_MSI | INT_INTX, pp->base + IMASK_LOCAL);

#ifdef FPGA_QUIRK_INTX_CLEAR
	pp->intx_irq_domain = irq_domain_add_linear(NULL, 4, &irq_ops, NULL);
	if (!pp->intx_irq_domain) {
		pr_err(DEVICE_NAME ": Failed to create IRQ domain\n");
		return -1;
	}
#endif

	for (x=0;x<pp->numresources;x++) {
		if (pp->resource[x].flags & IORESOURCE_MEM) {
			if (request_resource(&iomem_resource, &(pp->resource[x]))) {
				pr_err(DEVICE_NAME ": Failed to request PCIe memory\n");
				continue;
			}
			pci_add_resource_offset(&sys->resources, &(pp->resource[x]), sys->mem_offset);
		}
		else if (pp->resource[x].flags & IORESOURCE_IO) {
//			if (request_resource(&ioport_resource, &(pp->resource[x]))) {
//				pr_err(DEVICE_NAME ": Failed to request PCIe IO\n");
//				continue;
//			}
			pci_add_resource_offset(&sys->resources, &(pp->resource[x]), sys->io_offset);
		}
	}

	return 1;
}

static int __init xr3pci_get_resources(struct pcie_port *pp, struct device_node *np)
{
	struct resource res, *r;
	int err;

	/* Host bridge configuration registers */

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		pr_err(DEVICE_NAME ": Failed to find configuration registers in DT\n");
		return err;
	}

	if (!request_mem_region(res.start, resource_size(&res), DEVICE_NAME)) {
		pr_err(DEVICE_NAME ": Failed to request configuration registers resource\n");
		return -EBUSY;
	}

	pp->base = ioremap_nocache(res.start, resource_size(&res));
	if (!pp->base) {
		pr_err(DEVICE_NAME ": Failed to map configuration registers resource\n");
		err = -ENOMEM;
		goto err_map_io;
	}

	/* PCIe addres spaces */
	r = &(pp->resource[pp->numresources]);
	
	u32 *last = NULL;
	while (!of_pci_process_ranges(np, r, &last)) {
		pp->numresources++;
		r = &(pp->resource[pp->numresources]);
	}

	return 0;

	iounmap(pp->base);
err_map_io:
	release_mem_region(res.start, resource_size(&res));

	return err;
}

static void __devinit xr3pci_quirk_class(struct pci_dev *pdev)
{
	pdev->class = PCI_CLASS_BRIDGE_PCI << 8;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLDA, PCI_DEVICE_ID_XR3PCI, xr3pci_quirk_class);

//TODO does ILOCAL also need to be cleared after INTx interrpt?
void __init xr3pci_setup_arch(
	int (**map_irq)(const struct pci_dev *, u8, u8),
	struct pci_ops **ops,
	int (**setup)(struct pci_sys_data *, struct device_node *))
{
	*map_irq = xr3pci_map_irq;
	*ops = &xr3pci_ops;
	*setup = xr3pci_setup;
}

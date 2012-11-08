/*
 * Xpress RICH3-AXI PCIe Host Bridge Driver.
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Andrew Murray <amurray@embedded-bits.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
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

/* per controller structure */
struct pcie_port {
	//TODO: rather than store lots of fields used once - pass the device tree
	//node to the hw_pci.setup function
	void __iomem	*base;
	struct resource resource[5];
	int numresources;
       spinlock_t conf_lock;

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

	if (bus->number == 0 && where == PCI_CLASS_REVISION && size == 4 && PCI_SLOT(devfn) == 0 && PCI_FUNC(devfn) == 0) {
//		printk(" - fake\n");
                 *val = 0x06040001;    /* Bridge/PCI-PCI/rev 1 */
                return PCIBIOS_SUCCESSFUL;
        }

//	pr_debug("%s:%d readl_config size: %d, where: %d, ID: %d:%d:%d\n",
//	 __func__, __LINE__,
//		 size, where, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn));

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
//	printk("Returning value read ali as 0x%x\n", *val);

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

//	pr_debug("%s:%d writel_config size: %d, where: %d, ID: %d:%d:%d value = 0x%x\n",
//		 __func__, __LINE__,
//		 size, where, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), val);
	
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

//static
int __init xr3pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	pr_debug("%s:%d xr3pci_map_irq %d:%d:%d for slot %d pin %d\n", __func__, __LINE__,
						dev->bus->number, PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn), slot, pin);
	
	struct pci_sys_data *sys = dev->sysdata;
	struct pcie_port *pp = sys->private_data;

	struct of_irq out;
	if (of_irq_map_pci(dev, &out))
		printk("Error mapping...\n");

		int virq = irq_create_of_mapping(out.controller, out.specifier,
					     out.size);
	printk("Given IRQ %d (pin %d)\n", virq, pin);
	return virq;
}

//TODO: This is really just for development
static int __init xr3pci_probe(struct pcie_port *pp)
{
	/* gain some confidence that we are talking to the correct device by
	   reading registers with known values */
	u32 ver, ver2;

	/* verify hardwired vendor, device, revision IDs */
	//TODO: PLDA document doesn't make clear how this applies to root ports
	ver = readl(pp->base + PCIE_PCI_IDS_1);
	if (!((ver  & 0xffff) == DEVICE_VENDOR_ID &&
	      (ver  & 0xffff0000) >> 16 == DEVICE_DEVICE_ID)) {
		printk("Unable to detect " DEVICE_NAME);
		return -1;
	}

	/* top nibble describes if core is configured as native endpoint or root port */
	if ((readl(pp->base + PCIE_BASIC_CONF) & 0xf0000000) != 0x10000000) {
		printk(DEVICE_NAME " is not hardwired as a root port\n");
		printk(DEVICE_NAME " is 0x%x\n", readl(pp->base + PCIE_BASIC_CONF));
		return -1;
	}

	/* display core version/revision information */
	//TODO: tie driver implementation to specific known working versions
	ver = readl(pp->base + BRIDGE_VER);
	ver2 = readl(pp->base + PCIE_PCI_IDS_2);
	printk(DEVICE_NAME ", revision: %d\n", (ver2 & 0xff));
	printk(" - Bridge Version: 	0x%x\n", ver & 0xfff);
	printk(" - Bridge Product ID:	0x%x\n", (ver & 0xfff000) >> 12);
	printk(" - Bridge DMA Engines:	%d\n", 	 (ver & 0xf000000) >> 24);

	printk("\n\n");

	//if not hardwired this should be written while the core is in reset
	//unsure about bits 14, 15. Expect rootport[0], x2, x4, x8 widths [8,9,10], 5 and 8GBps [12, 13] to be set
	printk("GEN_SETTINGS is 0x%x, expected 0x%x\n", readl(pp->base + GEN_SETTINGS), 0xf701);

	//pipe line settings recommended values provided in documentation for high-frequency(endpoint) or low-latency
	printk("PCIE_PIPE is 0x%x, expected 0x%x or 0x%x\n", readl(pp->base + PCIE_PIPE), 0x131c0000, 0x10000000);
	printk("PCIE_PIPE_1 is 0x%x, expected 0x%x or 0x%x\n", readl(pp->base + PCIE_PIPE_1), 0x3f000701, 0x08000001);

	return 0;
}

static irqreturn_t handler(int irq, void *dev_id)
{
	struct pcie_port *pp = (struct pcie_port *)dev_id;

	printk("Habndler for irq %d\n", irq);
	writel(1 << (irq-125), pp->base + ISTATUS_LOCAL);
	
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

	xr3pci_probe(pp);

	writel(0xffffffff, pp->base + IMASK_LOCAL);
	for (x=0;x<32;x++) {
		if (request_irq(125+x, handler, 0, "xr3", pp)) {
			printk("unable to request irq %d\n", 125+x);
		}
	}

	for (x=0;x<pp->numresources;x++) {
		if (pp->resource[x].flags & IORESOURCE_MEM) {
			if (request_resource(&iomem_resource, &(pp->resource[x]))) {
				pr_err(DEVICE_NAME ": Failed to request PCIe memory\n");
				continue;
			}
			pci_add_resource_offset(&sys->resources, &(pp->resource[x]), sys->mem_offset);
		}
		else if (pp->resource[x].flags & IORESOURCE_IO) {
			if (request_resource(&ioport_resource, &(pp->resource[x]))) {
				pr_err(DEVICE_NAME ": Failed to request PCIe IO\n");
				continue;
			}
			pci_add_resource_offset(&sys->resources, &(pp->resource[x]), sys->io_offset);
		}
	}

	return 1;
}

static int __init xr3pci_get_resources(struct pcie_port *pp, struct device_node *np)
{
	struct resource res, *resp;
	const u32 *ranges;
	int err, len, pna, n;
	unsigned long long pci_addr, cpu_addr, size;
	u32 pci_space;

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

	/* determine parent's address size and size of PCIe resource (in cells) */
	pna = of_n_addr_cells(np);
	n = pna + 5;

	ranges = of_get_property(np, "ranges", &len);
	if (!ranges) {
		pr_err(DEVICE_NAME ": Failed to find PCIe ranges int DT\n");
		err = -EINVAL;
		goto err_get_ranges;
	}

	/* iterate through each PCIe resource entry, each entry consists of:
	    1 cell describing the pci address space (type of memory, etc)
	    2 cells describing the pci start address for the range
	    pna cells describing the cpu start address for the range
	    2 cells describing the size of the range */
	while ((len -= n * 4) >= 0) {
		pci_space = of_read_number(ranges, 1);
		pci_addr = of_read_number(ranges + 1, 2);
		cpu_addr = of_translate_address(np, ranges + 3);
		size = of_read_number(ranges + pna + 3, 2);
		ranges += n;

		if (cpu_addr == OF_BAD_ADDR || size == 0)
			continue;

		if (pp->numresources >= MAX_RESOURCES) {
			pr_err(DEVICE_NAME ": Maximum supported controller resources reached\n");
			break;
		}

		resp = &(pp->resource[pp->numresources]);
		switch((pci_space >> 24) & 0x3) {
		case 1:
			resp->flags = IORESOURCE_IO;
			resp->start = pci_addr;
			break;
		case 2:
		case 3:
			resp->flags = IORESOURCE_MEM;
			resp->start = cpu_addr;

			if (pci_space & 0x40000000)
				resp->flags |= IORESOURCE_PREFETCH;

			break;
		default:
			continue;
		}
		resp->name = DEVICE_NAME;
		resp->end = resp->start + size - 1;
		resp->parent = resp->child = resp->sibling = NULL;
		pp->numresources++;
	}

	return 0;

err_irq:
err_get_ranges:
	iounmap(pp->base);
err_map_io:
	release_mem_region(res.start, resource_size(&res));

	return err;
}

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

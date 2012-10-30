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
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/io.h>
#include <asm/mach/pci.h>
#include <asm/mach/irq.h>

#include "xr3pci.h"

/* Host Bridge Identification */
#define DEVICE_NAME "XpressRICH3-AXI PCIe Host Bridge"
#define DEVICE_VENDOR_ID  0x1556
#define DEVICE_DEVICE_ID  0x1100

//TODO: remove or use properly
#define pr_debug printk

static struct hw_pci xr3pci_hw_pci;

/* per controller structure */
struct pcie_port {
	//TODO: rather than store lots of fields used once - pass the device tree
	//node to the hw_pci.setup function
	void __iomem	*base;
	struct resource resource[5];
	int numresources;
	int irq;
	int irqINTA, irqINTB, irqINTC, irqINTD;
	spinlock_t conf_lock;
};

static struct pcie_port pcie_port[MAX_SUPPORTED_DEVICES];

static u32 xr3pci_read(void __iomem * addr)
{
	u32 val = 0;

	pr_debug("%s:%d xr3pci_read: read from 0x%x\n",
		__func__, __LINE__, addr);

	val = readl(addr);

	pr_debug("%s:%d xr3pci_read: read value 0x%x\n",
		__func__, __LINE__, val);

	return val;
}

static void xr3pci_write(u32 val, void __iomem *addr)
{
	pr_debug("%s:%d xr3pci_write: wrote 0x%x to 0x%x\n",
		__func__, __LINE__,
		val, addr);

	writel(val, addr);
}

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

	pr_debug("%s:%d xr3pci_read_config size: %d, where: %d, ID: %d:%d:%d\n",
		 __func__, __LINE__,
		 size, where, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn));

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
	spin_lock_irqsave(&pp->conf_lock, flags);
	xr3pci_write(cfgnum, pp->base + PCIE_CFGNUM);
	*val = xr3pci_read(pp->base + BRIDGE_PCIE_CONFIG + (where & ~0x3));
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

	pr_debug("%s:%d xr3pci_write_config size: %d, where: %d, ID: %d:%d:%d\n",
		 __func__, __LINE__,
		 size, where, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn));
	
	/* We expect requests which are aligned to the request size which is
	   either 1, 2 or 4 bytes. We also expect this function to be called
	   exclusively through the pci_write_config_(byte|word|dword) accessors
           - these wrapper functions check alignment for us. They also provide
           serialisation of PCI configuration space accesses which prevent the
	   need for locking here */

 	//TODO: extra guards during development
	WARN(size + (where & 3) > 4, "CfgWd spans DWORD boundary\n");
	WARN(size == 3, "CfgWd size is unexpected (3)\n");

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
	spin_lock_irqsave(&pp->conf_lock, flags);
	xr3pci_write(cfgnum, pp->base + PCIE_CFGNUM);
	xr3pci_write(val << ((where & 3) * 8), pp->base + BRIDGE_PCIE_CONFIG + (where & ~0x3));
	spin_unlock_irqrestore(&pp->conf_lock, flags);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops xr3pci_ops = {
	.read 	= xr3pci_read_config,
	.write 	= xr3pci_write_config, 
};

static int __init xr3pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	pr_debug("%s:%d xr3pci_map_irq:", __func__, __LINE__);
	struct pci_sys_data *sys = dev->sysdata;
	struct pcie_port *pp = sys->private_data;

	switch (pin) {
	case 1: return pp->irqINTA;
	case 2: return pp->irqINTB;
	case 3: return pp->irqINTC;
	case 4: return pp->irqINTD;
	}

	return -1;
}

//TODO: This is really just for development
static int __init xr3pci_probe(struct pcie_port *pp)
{
	/* gain some confidence that we are talking to the correct device by
	   reading registers with known values */
	u32 ver, ver2;

	/* verify hardwired vendor, device, revision IDs */
	//TODO: PLDA document doesn't make clear how this applies to root ports
	ver = xr3pci_read(pp->base + PCIE_PCI_IDS_1);
	if (!((ver  & 0xffff) == DEVICE_VENDOR_ID &&
	      (ver  & 0xffff0000) >> 16 == DEVICE_DEVICE_ID)) {
		printk("Unable to detect " DEVICE_NAME);
		return -1;
	}

	/* top nibble describes if core is configured as native endpoint or root port */
	if ((xr3pci_read(pp->base + PCIE_BASIC_CONF) & 0xf0000000) != 0x10000000) {
		printk(DEVICE_NAME " is not hardwired as a root port\n");
		return -1;
	}

	/* display core version/revision information */
	//TODO: tie driver implementation to specific known working versions
	ver = xr3pci_read(pp->base + BRIDGE_VER);
	ver2 = xr3pci_read(pp->base + PCIE_PCI_IDS_2);
	printk(DEVICE_NAME ", revision: %d\n", (ver2 & 0xff));
	printk(" - Bridge Version: 	0x%x\n", ver & 0xfff);
	printk(" - Bridge Product ID:	0x%x\n", (ver & 0xfff000) >> 12);
	printk(" - Bridge DMA Engines:	%d\n", 	 (ver & 0xf000000) >> 24);

	printk("\n\n");

	//if not hardwired this should be written while the core is in reset
	//unsure about bits 14, 15. Expect rootport[0], x2, x4, x8 widths [8,9,10], 5 and 8GBps [12, 13] to be set
	printk("GEN_SETTINGS is 0x%x, expected 0x%x\n", xr3pci_read(pp->base + GEN_SETTINGS), 0xf701);

	//pipe line settings recommended values provided in documentation for high-frequency(endpoint) or low-latency
	printk("PCIE_PIPE is 0x%x, expected 0x%x or 0x%x\n", xr3pci_read(pp->base + PCIE_PIPE), 0x131c0000, 0x10000000);
	printk("PCIE_PIPE_1 is 0x%x, expected 0x%x or 0x%x\n", xr3pci_read(pp->base + PCIE_PIPE_1), 0x3f000701, 0x08000001);

	return 0;
}

static void xr3pci_msi_handler(unsigned int irq, struct irq_desc *desc)
{
	int j;
	unsigned long status;
	struct pcie_port *pp;
	struct irq_chip *chip = irq_get_chip(irq);
	
	chained_irq_enter(chip, desc);
	pp = irq_desc_get_handler_data(desc);
	pr_debug("%s:%d xr3pci_msi_handler\n", __func__, __LINE__);

	/* check an MSI really occured */	
	if (!(xr3pci_read(pp->base + ISTATUS_LOCAL) & INT_MSI))
		goto out;

	/* clear MSI interrupt */
	xr3pci_write(INT_MSI, pp->base + ISTATUS_LOCAL);

	/* call handlers for pending MSIs */
	status = xr3pci_read(pp->base + ISTATUS_MSI);
	do {
		//TODO: locking
		j = find_first_bit(&status, MAX_SUPPORTED_NO_MSI);
		xr3pci_write((1 << j), pp->base + ISTATUS_MSI);
		generic_handle_irq(IRQ_MSI_BASE + j);
		status = xr3pci_read(pp->base + ISTATUS_MSI);
	} while (status);

out:
	chained_irq_exit(chip, desc);
}

static int __init xr3pci_setup(int nr, struct pci_sys_data *sys)
{
	int x=0;
	struct pcie_port *pp;

	if (nr >= xr3pci_hw_pci.nr_controllers)
		return 0;

	pp = &pcie_port[nr];
	sys->private_data = pp;

	xr3pci_probe(pp);

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

#ifdef CONFIG_PCI_MSI
	/* set up MSI handler and unmask MSI interrupt generation */
	irq_set_handler_data(pp->irq, pp); 
	irq_set_chained_handler(pp->irq, xr3pci_msi_handler);
	xr3pci_write(INT_MSI, pp->base + IMASK_LOCAL);	
#endif

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

	/* MSI IRQ */
	pp->irq = irq_of_parse_and_map(np, 0);
	if (!pp->irq) {
		pr_err(DEVICE_NAME ": Failed to map MSI IRQ\n");
		err = -EINVAL;
		goto err_irq;
	}

	/* INTx IRQs */
	pp->irqINTA = irq_of_parse_and_map(np, 1);
	if (!pp->irqINTA) {
		pr_err(DEVICE_NAME ": Failed to map INTA IRQ\n");
		err = -EINVAL;
		goto err_irq;
	}
	pp->irqINTB = irq_of_parse_and_map(np, 2);
	if (!pp->irqINTB) {
		pr_err(DEVICE_NAME ": Failed to map INTB IRQ\n");
		err = -EINVAL;
		goto err_irq;
	}
	pp->irqINTC = irq_of_parse_and_map(np, 3);
	if (!pp->irqINTC) {
		pr_err(DEVICE_NAME ": Failed to map INTC IRQ\n");
		err = -EINVAL;
		goto err_irq;
	}
	pp->irqINTD = irq_of_parse_and_map(np, 4);
	if (!pp->irqINTD) {
		pr_err(DEVICE_NAME ": Failed to map INTD IRQ\n");
		err = -EINVAL;
		goto err_irq;
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

static struct hw_pci xr3pci_hw_pci __initdata = {
	.nr_controllers = 0,
	.setup		= xr3pci_setup,
	.map_irq	= xr3pci_map_irq,
	.ops		= &xr3pci_ops,
};

static const struct __initconst of_device_id xr3pci_device_id[] = {
	{ .compatible = "arm,xr3pci", },
	{},
};

static int __init xr3pci_init(void)
{
	struct device_node *np;
	
	/* The arch/arm/kernel/bios32.c pci_common_init helper is not DT
	   friendly which prevents the use of platform_driver with a
	   of_match_table. This is due to:
	     - pci_common_init is designed to be called once when all
	       controllers are known about - platform_driver probe would
	       occur for each controller and we wouldn't know when to
	       the last controller was found
	     - private data used in the callbacks in hw_pci is not accessiable
	       until the setup call - therefore there is no way to tie a
	       platform device/DT data at this point. I.e. we have no way of
	       accessing the host bridges base address which we know now with
               the later call backs from hw_pci. The pci_common_init function
	       offers no way of providing private data
	   Therefore we look for all compatible devices now and register them
	   in one go with bios32. We later uniquelly identify them through
	   their controller id.

	   TODO: tie in with bios32 may need to be changed for AARCH64
	   TODO: move some of this general PCI-DT code to pci-common bios32, etc
	*/	
	for_each_matching_node(np, xr3pci_device_id) {
		struct pcie_port *pp;

		/* is there enough room for another controller? */
		//TODO: use a list
		if (xr3pci_hw_pci.nr_controllers >= MAX_SUPPORTED_DEVICES) {
			pr_err(DEVICE_NAME ": Maximum supported controllers reached\n");
			break;
		}

		/* add the new controller */
		pp = &pcie_port[xr3pci_hw_pci.nr_controllers];
		pp->numresources = 0;
		spin_lock_init(&pp->conf_lock);

		/* add DT resources to the controller */
		if (xr3pci_get_resources(pp, np)) {
			continue;
		}

		/* we succeeded so increase the controller count */
		xr3pci_hw_pci.nr_controllers++;
	}

	/* we've found all our controllers so tell the OS to enumerate/add them */
	if (xr3pci_hw_pci.nr_controllers)
		pci_common_init(&xr3pci_hw_pci);

	return 0;
}
subsys_initcall(xr3pci_init);

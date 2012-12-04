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

static struct irq_domain *irq_domain;
void *regs;

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

	static void irq_ack(struct irq_data *data) {}
	static void irq_enable(struct irq_data *data) {}
	static void irq_disable(struct irq_data *data) {}
	static void irq_mask(struct irq_data *data) {} 
	static void irq_unmask(struct irq_data *data) {}
static struct irq_chip irq_chip = {
	.name	= "IRQ CHIP",
	.irq_ack = irq_ack,
	.irq_enable = irq_enable,
	.irq_disable = irq_disable,
	.irq_mask =  irq_mask,
	.irq_unmask = irq_unmask,
};

static int irq_map(struct irq_domain *h, unsigned int virq, irq_hw_number_t hw)
{
	printk("IRQ CASCADE MAP virq %d, hw %d\n", virq, hw);
	irq_set_chip_and_handler(virq, &irq_chip, handle_simple_irq); //???
	return 0;
}

static struct irq_domain_ops irq_ops = {
	.map = irq_map,
};

static void xr3pci_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int j;
	unsigned long status;
	struct irq_chip *chip = irq_get_chip(irq);

	chained_irq_enter(chip, desc);
	u8 pin = (u8)irq_desc_get_handler_data(desc);
	pr_debug("%s:%d xr3pci_irq_handler for IRQ %d WAS PIN %d\n", __func__, __LINE__, irq, pin);

#if 1
		int virq = irq_linear_revmap(irq_domain, pin-1);
		pr_debug("%s:%d  occured - hwirq %d, virt irq %d?\n", 
			__func__, __LINE__, pin, virq);

		if (virq != 0)
			generic_handle_irq(virq);

	printk("STATUS 0x%x\n", readl(regs + ISTATUS_LOCAL));
	writel((1 << pin - 1 + 24), regs + ISTATUS_LOCAL);
	printk("STATUS AFTER 0x%x\n", readl(regs + ISTATUS_LOCAL));
#endif
out:
	chained_irq_exit(chip, desc);
}

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


	struct irq_chip *c = irq_get_chip(virq);
	printk("CHIP IS CURRENTLY %p %s\n", c, c->name);

	printk(" - cascading virq irq %d of pin %d...\n", virq, pin);
	irq_set_chained_handler(virq, xr3pci_irq_handler);
	irq_set_handler_data(virq, pin);

	//this assumes that there is no swizzling of pins
	virq = irq_create_mapping(irq_domain, pin-1);

	printk("Given IRQ %d (pin %d)\n", virq, pin);


#if 0
	printk("BEGIN TEST\n");
//	void volatile *mem = kmalloc(0x400, GFP_KERNEL | GFP_DMA);

while (1) {

	dma_addr_t dma;
	void *mem = dma_zalloc_coherent(&dev->dev, 0x400, &dma, GFP_KERNEL | GFP_DMA);

	printk("MEM is 0x%08x\n", mem);


		printk("Mapping\n");
//		dma_addr_t dma = dma_map_single(&dev->dev, mem, 0x400, DMA_FROM_DEVICE);
//		if (dma_mapping_error(&dev->dev, dma)) {
//			printk("BAD :(\n");
//		}
		u32 *a = mem;
		*a = 0x0;

		u32 pphy = dma - 0x60000000;

		u32 *pci = ioremap_nocache(pphy, 0x400);

		printk("DMA PHY 0x%x, VIRT 0x%x\n", dma, mem);
		printk("PCI PHY 0x%x, VIRT 0x%x\n", pphy, pci);
		printk("Assuming 0x40000000->0xb0000000 size 0x20000000\n");
	
		printk("VALUE AT 0x%x BEFORE is 0x%x\n", a, *a);
	
	u32 *d = mem;
	int c=0;
	for (c=0;c<0x400/4/8;c++) {
	printk("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0%08x\n", 
		d, *d++, *d++, *d++, *d++,
		*d++, *d++, *d++, *d++);
	}

		printk("WRITING 0xDEADBEEF to 0x%x\n", pci);

		for (c=0;c<0x400;c+=4)
			*pci++ = c + 0xe3de11ff;

		printk("delay\n");
		mdelay(5000);

		printk("UNAMMP\n");
//		dma_unmap_single(&dev->dev, dma, 100, DMA_FROM_DEVICE);
	
		printk("READING BACK\n");
		d = mem;
		int bad =0;
		for (c=0;c<0x400;c+=4)
			if (*d++ != c+0xe3de11ff)  bad++;//printk("number %d doesnt match\n", c);

		printk("COMPLETE with %d failures\n", bad);
		if (bad) while(1);
		

	d = mem;
	for (c=0;c<0x400/4/8;c++) {
	printk("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0%08x\n", 
		d, *d++, *d++, *d++, *d++,
		*d++, *d++, *d++, *d++);
	}
	d = mem;
		for (c=0;c<0x400;c+=4)
			*pci++ = 0;
	printk("ALL DONE\n");

}

	while(1);	
#endif
	return virq;
}


//TODO: This is really just for development
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

static irqreturn_t handler(int irq, void *dev_id)
{
	struct pcie_port *pp = (struct pcie_port *)dev_id;

	printk("Habndler for irq %d\n", irq);
	printk("status 0x%x\n", readl(pp->base + ISTATUS_LOCAL));
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

	if (xr3pci_probe(pp)) {
		return -1;
	}

	regs = pp->base;

	writel(0xffffffff, pp->base + IMASK_LOCAL);
	for (x=0;x<32;x++) {
		if (125+x != 149)
		if (request_irq(125+x, handler, 0, "xr3", pp)) {
			printk("unable to request irq %d\n", 125+x);
		}
	}

	irq_domain = irq_domain_add_linear(NULL, 4, &irq_ops, NULL);
	if (!irq_domain) printk("WWWWWWWWWWWWWWWWWWWWWWWWWWw\n");

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
	struct resource res;
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
	struct resource *r = &(pp->resource[pp->numresources]);
	
	u32 *last = NULL;
	while (!of_pci_process_ranges(np, r, &last)) {
		pp->numresources++;
		r = &(pp->resource[pp->numresources]);
	}

	return 0;

err_irq:
err_get_ranges:
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

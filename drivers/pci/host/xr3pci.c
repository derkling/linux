/*
 * XpressRICH3-AXI PCIe Host Bridge Driver.
 *
 * Copyright (C) 2012-2013 ARM Ltd.
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

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#include <asm/pci-bridge.h>

#include "xr3pci.h"

struct xr3pci_port {
	void __iomem	*base;
	void __iomem	*reset;
	void __iomem	*ecam;
};

static int xr3pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	struct pci_controller *hose = pci_bus_to_host(bus);
	struct xr3pci_port *pp = hose->dn->data;

	void __iomem *addr = pp->ecam + ECAM_OFFSET(bus->number, devfn, where);

	switch (size) {
	case 1:
		*val = readb(addr);
		break;
	case 2:
		*val = readw(addr);
		break;
	default:
		*val = readl(addr);
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int xr3pci_write_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val)
{
	struct pci_controller *hose = pci_bus_to_host(bus);
	struct xr3pci_port *pp = hose->dn->data;

	void __iomem *addr = pp->ecam + ECAM_OFFSET(bus->number, devfn, where);

	switch (size) {
	case 1:
		writeb(val, addr);
		break;
	case 2:
		writew(val, addr);
		break;
	default:
		writel(val, addr);
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops xr3pci_ops = {
	.read	= xr3pci_read_config,
	.write	= xr3pci_write_config,
};

static int xr3pci_check_device(struct xr3pci_port *pp)
{
	u32 val;
	int timeout = 1000;

	writel(RST_CTRL_PHY_REL | RST_CTRL_RC_REL, pp->reset + RST_CTRL);
	do { 
		msleep(1);
		val = readl(pp->reset + RST_STS) & RST_STS_MASK;
	} while (--timeout && val != RST_STS_MASK);
	
	if (!timeout) {
		pr_err("Unable to bring " DEVICE_NAME " out of reset");
		return -1;
	}

	val = readl(pp->base + PCIE_BASIC_STATU);
	if (val & PCIE_BS_LNK_MASK)
		pr_info(DEVICE_NAME " %dx link negotiated (gen %d)\n",
					ffs(val & PCIE_BS_LNK_MASK),
					(val & PCIE_BS_GEN_MASK) >> 8);
	else
		pr_warn(DEVICE_NAME ": No link negotiated\n");

	return 0;
}

static void xr3pci_update_ats_entry(void __iomem *base, int entry,
			resource_size_t src_addr, resource_size_t trsl_addr,
			int trsl_param, int exp)
{
	writel(src_addr | (exp << 1) | 0x1, base + (entry * ATR_TBL_SIZE) + ATR_SRC_ADDR_LWR);
	writel(trsl_addr, base + (entry * ATR_TBL_SIZE) + ATR_TRSL_ADDR_LWR);
	writel(trsl_param, base + (entry * ATR_TBL_SIZE) + ATR_TRSL_PARAM);

#ifdef CONFIG_PHYS_ADDR_T_64BIT
	writel(src_addr >> 32, base + (entry * ATR_TBL_SIZE) + ATR_SRC_ADDR_UPR);
	writel(trsl_addr >> 32, base + (entry * ATR_TBL_SIZE) + ATR_TRSL_ADDR_UPR);
#endif
}

static int xr3pci_setup_ats(struct xr3pci_port *pp, struct pci_controller *hose)
{
	int x, exp, ats_x = 1;
	resource_size_t size;

	/* 1:1 mapping for inbound PCIe transactions to AXI slave 0 */
	xr3pci_update_ats_entry(pp->base + ATR_PCIE_WIN0, ATR_TBL_1,
						0x0, 0x0, 0x4, 0x3f);

	pr_info(DEVICE_NAME " ATR:\n");
	pr_info(" PCIE_WIN0.1: 0x%016x -> 0x%016x (16 EB)\n", 0, 0);

	for (x=0;x<3;x++) {
		if (!(hose->mem_resources[x].flags & IORESOURCE_MEM))
			continue;

		size = resource_size(&hose->mem_resources[x]);
		exp = find_first_bit(&size, sizeof(resource_size_t)) - 1;

		pr_info(" AXI4_SLV0.%d: 0x%016llx -> 0x%016llx (%d bytes)\n", ats_x,
				hose->mem_resources[x].start, hose->mem_resources[x].start,
				2 << exp);
		
		xr3pci_update_ats_entry(pp->base + ATR_AXI4_SLV0, ats_x,
			hose->mem_resources[x].start,
			hose->mem_resources[x].start, 0x0, exp);

		ats_x++;
	}


	return 0;
}

static int xr3pci_setup_int(struct xr3pci_port *pp)
{
	/* Enable IRQs for MSIs and legacy interrupts */
	writel(INT_MSI | INT_INTX, pp->base + IMASK_LOCAL);

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

	err = of_address_to_resource(np, 1, &res);
	if (err) {
		pr_err(DEVICE_NAME
			": Failed to find reset registers in DT\n");
		return err;
	}

	if (!request_mem_region(res.start, resource_size(&res), DEVICE_NAME)) {
		pr_err(DEVICE_NAME
			": Failed to request reset registers resource\n");
		return -EBUSY;
	}

	pp->reset = ioremap_nocache(res.start, resource_size(&res));
	if (!pp->base) {
		pr_err(DEVICE_NAME
			": Failed to map reset registers resource\n");
		err = -ENOMEM;
		goto err_map_io;
	}

	err = of_address_to_resource(np, 2, &res);
	if (err) {
		pr_err(DEVICE_NAME
			": Failed to find ECAM base in DT\n");
		return err;
	}

	if (!request_mem_region(res.start, resource_size(&res), DEVICE_NAME)) {
		pr_err(DEVICE_NAME
			": Failed to request ECAM base resource\n");
		return -EBUSY;
	}

	pp->ecam = ioremap_nocache(res.start, resource_size(&res));
	if (!pp->ecam) {
		pr_err(DEVICE_NAME
			": Failed to map ECAM base resource\n");
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
}

int __init xr3pci_setup(struct pci_controller *hose)
{
	struct xr3pci_port *pp;
	struct device_node *np = hose->dn;

	pp = kzalloc(sizeof(struct xr3pci_port), GFP_KERNEL);
	WARN_ON(!pp);

	hose->dn->data = pp;

	if (xr3pci_get_resources(pp, np))
		goto err;

	if (xr3pci_check_device(pp))
		goto err;

	if (xr3pci_setup_int(pp))
		goto err;

	if (xr3pci_setup_ats(pp, hose))
		goto err;

	return 0;

err:
	xr3pci_teardown(pp, np);
	return 1;
}

#ifdef FPGA_QUIRK_FPGA_CLASS
static void xr3pci_quirk_class(struct pci_dev *pdev)
{
	pdev->class = PCI_CLASS_BRIDGE_PCI << 8;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLDA, PCI_DEVICE_ID_XR3PCI,
			xr3pci_quirk_class);
#endif

static int __init xr3pci_probe(struct platform_device *pdev)
{
	int len;
	struct device_node *dev;
	struct pci_controller *hose;
	const int *bus_range;

	dev = pdev->dev.of_node;

	if (!of_device_is_available(dev)) {
		pr_warn("%s: disabled\n", dev->full_name);
		return -ENODEV;
	}

	bus_range = of_get_property(dev, "bus-range", &len);
	if (bus_range == NULL || len < 2 * sizeof(int))
		pr_warn("Can't get bus-range for %s, assume bus 0\n",
				dev->full_name);

	hose = pcibios_alloc_controller(dev);
	if (!hose)
		return -ENOMEM;

	hose->first_busno = bus_range ? bus_range[0] : 0;
	hose->last_busno = bus_range ? bus_range[1] : 0xff;

	hose->ops = &xr3pci_ops;
	hose->parent = &pdev->dev;

	pci_process_bridge_OF_ranges(hose, dev, 1);

	return xr3pci_setup(hose);
}

static const struct __initconst of_device_id xr3pci_device_id[] = {
	{ .compatible = "arm,xr3pci", },
};
MODULE_DEVICE_TABLE(of, xr3pci_device_id);

static struct platform_driver xr3pci_driver = {
	.driver		= {
		.name	= "xr3pci",
		.owner	= THIS_MODULE,
		.of_match_table = xr3pci_device_id,
	},
	.probe = xr3pci_probe,
};

static int __init xr3pci_init(void)
{
	return platform_driver_register(&xr3pci_driver);
}

arch_initcall(xr3pci_init);

MODULE_AUTHOR("Andrew Murray <Andrew.Murray@arm.com>");
MODULE_DESCRIPTION("XpressRICH3-AXI PCIe Host Bridge");
MODULE_LICENSE("GPL v2");

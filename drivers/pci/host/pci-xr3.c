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

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>

#include <asm/pci-bridge.h>

#include "pci-xr3.h"

struct xr3pci_port {
	void __iomem	*base;
	void __iomem	*reset;
	void __iomem	*ecam;
};

static int xr3pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	struct xr3pci_port *pp = bus->sysdata;

	void __iomem *addr = pp->ecam + XR3PCI_ECAM_OFFSET(bus->number, devfn, where);

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
	struct xr3pci_port *pp = bus->sysdata;

	void __iomem *addr = pp->ecam + XR3PCI_ECAM_OFFSET(bus->number, devfn, where);

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

static int xr3pci_enable_device(struct xr3pci_port *pp)
{
	u32 val;
	int timeout = 200;

	/* add credits */
	writel(0x00f0b818, pp->base + XR3PCI_VIRTCHAN_CREDITS);
	writel(0x1, pp->base + XR3PCI_VIRTCHAN_CREDITS + 4);

	/* allow ECRC */
	writel(0x6006, pp->base + XR3PCI_PEX_SPC2);

	writel(JUNO_RESET_CTRL_PHY | JUNO_RESET_CTRL_RC,
		pp->reset + JUNO_RESET_CTRL);
	do {
		msleep(1);
		val = readl(pp->reset + JUNO_RESET_STATUS);
	} while (--timeout &&
		(val & JUNO_RESET_STATUS_MASK) != JUNO_RESET_STATUS_MASK);

	if (!timeout) {
		pr_err("Unable to bring " DEVICE_NAME " out of reset");
		return -EAGAIN;
	}

	msleep(20);
	timeout = 20;
	do {
		msleep(1);
		val = readl(pp->base + XR3PCI_BASIC_STATUS);
	} while (--timeout && !(val & XR3PCI_BS_LINK_MASK));

	if (!(val & XR3PCI_BS_LINK_MASK)) {
		pr_warn(DEVICE_NAME ": No link negotiated\n");
		return -EIO;
	}

	pr_info(DEVICE_NAME " %dx link negotiated (gen %d), maxpayload %d, maxreqsize %d\n",
		val & XR3PCI_BS_LINK_MASK, (val & XR3PCI_BS_GEN_MASK) >> 8,
		2 << (7 + ((val & XR3PCI_BS_NEG_PAYLOAD_MASK) << 24)),
		2 << (7 + ((val & XR3PCI_BS_NEG_REQSIZE_MASK) >> 28)));

	return 0;
}

static void xr3pci_update_atr_entry(void __iomem *base, int entry,
			resource_size_t src_addr, resource_size_t trsl_addr,
			int trsl_param, int exp)
{
	int index = entry * XR3PCI_ATR_TABLE_SIZE;

	pr_info(" 0x%p.%d: 0x%016llx %s 0x%016llx (2^%d bytes)\n", base, entry,
		src_addr, (entry ? "->" : "<-"), trsl_addr, exp + 1);

	writel(src_addr | (exp << 1) | 0x1, base + index + XR3PCI_ATR_SRC_ADDR_LOW);
	writel(trsl_addr, base + index + XR3PCI_ATR_TRSL_ADDR_LOW);

#ifdef CONFIG_PHYS_ADDR_T_64BIT
	writel(src_addr >> 32, base + index + XR3PCI_ATR_SRC_ADDR_HIGH);
	writel(trsl_addr >> 32, base + index + XR3PCI_ATR_TRSL_ADDR_HIGH);
#endif

	writel(trsl_param, base + index + XR3PCI_ATR_TRSL_PARAM);
}

static int xr3pci_setup_atr(struct xr3pci_port *pp, struct device *dev,
			struct list_head *resources, resource_size_t io_base)
{
	int exp, i = 0;
	struct pci_host_bridge_window *window;
	struct resource *res;
	resource_size_t offset;

	pr_info(DEVICE_NAME " Address Translation:\n");

	/* 1:1 mapping for inbound PCIe transactions to AXI slave 0 */
	xr3pci_update_atr_entry(pp->base + XR3PCI_ATR_PCIE_WIN0, i, 0, 0, 4, 0x3f);
	i++;

	list_for_each_entry(window, resources, list) {
		res = window->res;
		offset = window->offset;
		exp = ilog2(resource_size(res)) - 1;

		if (resource_type(res) == IORESOURCE_MEM) {
			xr3pci_update_atr_entry(pp->base + XR3PCI_ATR_AXI4_SLV0, i,
				res->start, res->start - offset, 0, exp);
			if (request_resource(&iomem_resource, res))
				dev_info(dev, "failed to request MEM resource %pR\n", res);
		} else if (resource_type(res) == IORESOURCE_IO) {
			xr3pci_update_atr_entry(pp->base + XR3PCI_ATR_AXI4_SLV0, i,
				res->start + io_base,
				res->start - offset, 0x20000, exp);
			pci_remap_iospace(res, res->start + io_base);
			if (request_resource(&ioport_resource, res))
				dev_info(dev, "failed to request IO resource %pR\n", res);
		}
		i++;
	}

	return 0;
}

static int xr3pci_setup_int(struct xr3pci_port *pp)
{
	/* Enable IRQs for MSIs and legacy interrupts */
	writel(~(XR3PCI_INT_MSI | XR3PCI_INT_INTx),
			pp->base + XR3PCI_LOCAL_INT_MASK);

	return 0;
}

static int xr3pci_get_resources(struct xr3pci_port *pp, struct device *dev)
{
	int err;
	struct resource res;
	struct device_node *np = dev->of_node;

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		dev_err(dev, "Failed to find configuration registers\n");
		return err;
	}
	pp->base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(pp->base))
		return PTR_ERR(pp->base);

	err = of_address_to_resource(np, 1, &res);
	if (err) {
		dev_err(dev, "Failed to find reset registers\n");
		return err;
	}
	pp->reset = devm_ioremap_resource(dev, &res);
	if (IS_ERR(pp->reset))
		return PTR_ERR(pp->reset);

	err = of_address_to_resource(np, 2, &res);
	if (err) {
		dev_err(dev, "Failed to find ECAM configuration space\n");
		return -EINVAL;
	}
	pp->ecam = devm_ioremap_resource(dev, &res);
	if (IS_ERR(pp->ecam))
		return PTR_ERR(pp->ecam);

	return 0;
}

static int xr3pci_setup(struct xr3pci_port *pp, struct device *dev,
			struct list_head *resources, resource_size_t io_base)
{
	int err;

	if ((err = xr3pci_get_resources(pp, dev)) != 0)
		return err;

	if ((err = xr3pci_setup_atr(pp, dev, resources, io_base)) != 0)
		return err;

	if ((err = xr3pci_enable_device(pp)) != 0)
		return err;

	if ((err = xr3pci_setup_int(pp)) != 0)
		return err;

	return 0;
}

#ifdef FPGA_QUIRK_FPGA_CLASS
static void xr3pci_quirk_class(struct pci_dev *pdev)
{
	pdev->class = PCI_CLASS_BRIDGE_PCI << 8;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLDA, PCI_DEVICE_ID_XR3PCI,
			xr3pci_quirk_class);
#endif

static int xr3pci_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device_node *dn;
	struct xr3pci_port *pp;
	struct pci_bus *bus;
	resource_size_t io_base = 0;	/* physical address for start of I/O area */
	LIST_HEAD(res);

	dn = pdev->dev.of_node;

	if (!of_device_is_available(dn)) {
		pr_warn("%s: disabled\n", dn->full_name);
		return -ENODEV;
	}

	pp = kzalloc(sizeof(*pp), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;

	err = of_pci_get_host_bridge_resources(dn, 0, 0xff, &res, &io_base);
	if (err)
		goto probe_err;

	err = xr3pci_setup(pp, &pdev->dev, &res, io_base);
	if (err)
		goto probe_err;

	/* We always enable PCI domains and we keep domain 0 backward
	 * compatible in /proc for video cards
	 */
	pci_add_flags(PCI_ENABLE_PROC_DOMAINS);
	pci_add_flags(PCI_REASSIGN_ALL_BUS | PCI_REASSIGN_ALL_RSRC);

	bus = pci_scan_root_bus(&pdev->dev, 0, &xr3pci_ops, pp, &res);
	if (!bus)
		err = -ENXIO;

probe_err:
	if (err)
		kfree(pp);
	pci_free_resource_list(&res);
	return err;

}

static const struct of_device_id xr3pci_device_id[] = {
	{ .compatible = "arm,pcie-xr3", },
};
MODULE_DEVICE_TABLE(of, xr3pci_device_id);

static struct platform_driver xr3pci_driver = {
	.driver		= {
		.name	= "pcie-xr3",
		.owner	= THIS_MODULE,
		.of_match_table = xr3pci_device_id,
	},
	.probe = xr3pci_probe,
};

module_platform_driver(xr3pci_driver);

MODULE_AUTHOR("Liviu Dudau <Liviu.Dudau@arm.com>");
MODULE_DESCRIPTION("XpressRICH3-AXI PCIe Host Bridge");
MODULE_LICENSE("GPL v2");

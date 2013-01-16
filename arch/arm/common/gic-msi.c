/*
 * GIC-MSI Driver.
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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/io.h>
#include <asm/mach/pci.h>
#include <asm/mach/irq.h>

#ifdef CONFIG_PCI_MSI

//TODO for now support only a single MSI per device and do not support MSI-X

//TODO: Remove me
//#define pr_debug printk

/* Maximum number of MSIs supported by this driver */
#define MAX_SUPPORTED_MSIS	32

/**
 * This driver provides MSI handling support for the XR3PCI. However it is
 * expected that the XR3PCI's MSI handling capability will be dropped and
 * future MSI handling capability of the GIC will be used instead. Thus the
 * support in this driver for XR3PCI's MSI is a stop-gap. As a result this
 * driver supports the XR3PCI's MSI but is designed with the potential GIC
 * implementation in mind.
 *
 * When FPGA_TRANSITIONAL_DRIVER is set, the XR3PCI's MSI capabilities are
 * used - this results in sharing MMIO used by the XR3PCI driver.
 *
 * It is expected that this driver may be used as basis of implementation
 * for the future GIC-MSI driver.
 */
#define FPGA_TRANSITIONAL_DRIVER

/* Shared registers of the XR3PCI which are used to report interrupt status */
#ifdef FPGA_TRANSITIONAL_DRIVER
#define IMASK_LOCAL		0x0
#define ISTATUS_LOCAL		0x4
#define ISTATUS_MSI		0x14
#define INT_MSI			(1 << 28)
#endif

struct gic_msi_data {
	void __iomem *msi_capture_reg;
	int max_vectors;
	DECLARE_BITMAP(msi_irq_in_use, MAX_SUPPORTED_MSIS);
#ifdef FPGA_TRANSITIONAL_DRIVER
	void __iomem *transitional_regs;
	int transitional_irq;
	struct irq_domain *transitional_domain;
#endif
};

/* We permit only one MSI driver instance, this is it. A future implementation
   may update the arch_[setup|teardown]_msi_irq callbacks to map pci buses to
   MSI handlers */
static struct gic_msi_data *gd = NULL;

#ifdef FPGA_TRANSITIONAL_DRIVER
static void gic_msi_nop(struct irq_data *d)
{
	return;
}

static struct irq_chip gic_msi_chip = {
	.name	= "GIC-MSI",
	.irq_ack = gic_msi_nop,
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask =  mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

static int gic_msi_map(struct irq_domain *h, unsigned int virq, irq_hw_number_t hw)
{
	pr_debug("%s:%d gic_msi_map\n", __func__, __LINE__);
	irq_set_chip_and_handler(virq, &gic_msi_chip, handle_simple_irq);
	
	return 0;
}

static const struct irq_domain_ops msi_ops = {
	.map	= gic_msi_map,
};

static void gic_msi_handler(unsigned int irq, struct irq_desc *desc)
{
	int j;
	unsigned long status;
	struct gic_msi_data *data;
	struct irq_chip *chip = irq_get_chip(irq);

	chained_irq_enter(chip, desc);
	data = irq_desc_get_handler_data(desc);
	pr_debug("%s:%d gic_msi_handler for IRQ %d\n", __func__, __LINE__, irq);

	/* check an MSI really occured */	
	if (!(readl(data->transitional_regs + ISTATUS_LOCAL) & INT_MSI))
		goto out;

	/* call handlers for pending MSIs */
	status = readl(data->transitional_regs + ISTATUS_MSI);
	while (status) {
		j = find_first_bit(&status, data->max_vectors);

		int virq = irq_linear_revmap(data->transitional_domain, j);
		pr_debug("%s:%d MSI %d occured - hwirq %d, virt irq %d?\n", 
			__func__, __LINE__, j, j, virq);
		writel((1 << j), data->transitional_regs + ISTATUS_MSI);

		if (virq != 0)
			generic_handle_irq(virq);

		status = readl(data->transitional_regs + ISTATUS_MSI);
	}
	
	/* clear MSI interrupt */
	writel(INT_MSI, data->transitional_regs + ISTATUS_LOCAL);

out:
	chained_irq_exit(chip, desc);
}
#endif

int obtain_vector_irq(struct gic_msi_data *data, int vector, int *irq)
{
#ifdef FPGA_TRANSITIONAL_DRIVER
	*irq = irq_create_mapping(data->transitional_domain, vector);
	if (!(*irq)) {
		pr_err("Unable to allocate virtual IRQ\n");
		return -ENOSPC;
	}
	
	return 0;
#else
	/* TODO: Use DF to provide simple mapping between MSI vectors and 
                 SPIs */
	//*irq = SPI;
	return -ENOSPC;
#endif
}

void release_vector_irq(int irq)
{
#ifdef FPGA_TRANSITIONAL_DRIVER
	irq_dispose_mapping(irq);
#endif
}

/**
 * Satisfy an MSI request
 *
 * This implementation is weak as to provide a default GIC-MSI implementation
 * without breaking ARM platforms with existing implementations.
 */
int gic_msi_setup_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int irq, vector;
	struct msi_msg msg;
	struct gic_msi_data *data = gd;

	if (WARN(!data, "No MSI handler"))
		return 1;

	pr_debug("%s:%d arch_setup_msi_irq\n", __func__, __LINE__);

again:
	vector = find_first_zero_bit(data->msi_irq_in_use, data->max_vectors);

	if (test_and_set_bit(vector, data->msi_irq_in_use))
		goto again;

	if (obtain_vector_irq(data, vector, &irq)) {
		pr_err("Unable to obtain MSI vector / IRQ number\n");
		return 1;
	}

	irq_set_msi_desc(irq, desc);

	
	struct pci_dev *pci;
	pci = pci_get_bus_and_slot(0, 0);
	u32 v;
		pci_read_config_dword(pci, 0x10,
					&v);
	data->msi_capture_reg = (v & ~0x7f) + 0x190; //pci_resource_start(pdev, 0) + 0x190;//0x50100190;//of_iomap(node, 1);

	msg.address_hi = 0; //TODO: 32 bit for now
	msg.address_lo = (u32)data->msi_capture_reg;
	printk("addr 0x%p\n", msg.address_lo);
	msg.data = vector;
	write_msi_msg(irq, &msg);

	return 0;
}

void gic_msi_teardown_irq(unsigned irq)
{
	pr_debug("%s:%d arch_teardown_msi_irq\n", __func__, __LINE__);
	release_vector_irq(irq);
}

struct msi_controller msi_controller_ops = {
	.setup_msi_irq = gic_msi_setup_irq,
	.teardown_msi_irq = gic_msi_teardown_irq,
};

int __devinit gic_msi_probe(struct platform_device *pdev)
{
	struct gic_msi_data *data;
	struct device_node *node = pdev->dev.of_node;

	if (WARN(gd, "MSI handler already installed"))
		return 1;

	if (WARN(!node, "No device tree node"))
		return 1;

	data = kzalloc(sizeof(struct gic_msi_data), GFP_KERNEL);
	if (WARN(!data, "Unable to allocate memory"))
		return 1;
	
	platform_set_drvdata(pdev, data);

#ifdef FPGA_TRANSITIONAL_DRIVER 
	if (of_property_read_u32(node, "msi-max-vectors", &(data->max_vectors))) {
		pr_err("No msi-max-vectors property in device tree\n");
		goto err_kzalloc;
	}

	data->transitional_domain = irq_domain_add_linear(NULL, data->max_vectors, &msi_ops, NULL);
	if (WARN(!data->transitional_domain, "Unable to allocate irq_domain"))
		goto err_kzalloc;

	data->transitional_regs = of_iomap(node, 0);
	if (WARN(!data->transitional_regs, "Unable to map MSI registers"))
		goto err_domain;
	
	data->transitional_irq = irq_of_parse_and_map(node, 0);
	if (WARN(!data->transitional_irq, "Unable to map IRQ"))
		goto err_iomap;
	
	if (irq_set_handler_data(data->transitional_irq, data)) {
		pr_err("Unable to set IRQ handler\n");
		goto err_irq;
	}

	irq_set_chained_handler(data->transitional_irq, gic_msi_handler);	
#endif

//	struct pci_dev *pci;
//	pci = pci_get_bus_and_slot(0, 0);
//
//	data->msi_capture_reg = pci_resource_start(pdev, 0);//0x50100190;//of_iomap(node, 1);
//	if (WARN(!data->msi_capture_reg, "Unable to map MSI capture register"))
//		goto err_irq;

	gd = data;
	if (WARN(pci_register_msi_controller(&msi_controller_ops), "Unable to register MSI controller"))
		goto err_irq;

	return 0;

err_irq:
#ifdef FPGA_TRANSITIONAL_DRIVER 
	irq_dispose_mapping(data->transitional_irq);
err_iomap:
	iounmap(data->transitional_regs);
err_domain:
	irq_domain_remove(data->transitional_domain);
err_kzalloc:
#endif
	kfree(data);

	return 1;
}

/**
 * Remove is a bad idea. We cannot easily tell PCIe devices to stop
 * sending MSI's they've been allocated. In any case remove is unlikely
 * to be used.
 */
static int __devexit gic_msi_remove(struct platform_device *pdev)
{
	struct gic_msi_data *data;
	
	data = platform_get_drvdata(pdev);

//	iounmap(data->msi_capture_reg);

#ifdef FPGA_TRANSITIONAL_DRIVER 
{
	int j, irq;
	irq_set_chained_handler(data->transitional_irq, NULL);	
	irq_dispose_mapping(data->transitional_irq);
	iounmap(data->transitional_regs);

again:	
	j = find_first_bit(data->msi_irq_in_use, data->max_vectors);
  	irq = irq_find_mapping(data->transitional_domain, j);
	if (irq) {
		irq_dispose_mapping(irq);
		goto again;
	}

	irq_domain_remove(data->transitional_domain);
}
#endif

	kfree(data);
	return 0;	
}

static const struct of_device_id gic_msi_ids[] __devinitconst = {
	{ .compatible = "arm,gic-msi", },
	{ }
};

static struct platform_driver gic_msi_driver = {
	.probe  = gic_msi_probe,
	.remove = __devexit_p(gic_msi_remove),
	.driver = {
		.name = "gic-msi",
		.owner = THIS_MODULE,
		.of_match_table = gic_msi_ids,
	},
};

static int __init gic_msi_init(void)
{
	return platform_driver_register(&gic_msi_driver);
}

subsys_initcall(gic_msi_init);
#endif

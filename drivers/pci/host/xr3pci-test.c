/*
 * Xpress RICH3-AXI PCIe Host Bridge Test Module.
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Andrew Murray <amurray@embedded-bits.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pci.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#define DEVICE_NAME "XR3PCI Test"

static irqreturn_t xr3pci_irq_handler(int irq, void *data)
{
	printk("XR3PCI Test IRQ handler\n");
	return IRQ_HANDLED;
}

static int __devinit xr3pci_test_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	printk(DEVICE_NAME "\n");

#if 1
	int nvec = 32, ret, x;

	while (nvec >= 1) {
		ret = pci_enable_msi_block(dev, nvec);
		if (ret > 0) {
			nvec = ret;
		} else {
			break;
		}
	}

	printk("Obtained %d MSIs, with virqs %d to %d\n", nvec, dev->irq, dev->irq+nvec);

	for (x=0;x<nvec;x++) {
		if (request_irq(dev->irq + x, xr3pci_irq_handler, 0, DEVICE_NAME, NULL)) {
			pr_err(DEVICE_NAME ": Unable to request IRQ (MSI)\n");
		}
	}
#endif
#if 0
	int nvec = 4, ret, x;

	struct msix_entry entries[nvec];
	for (x=0;x<nvec;x++) {
		entries[x].entry = x;
	}

	while (nvec >= 1) {
		ret = pci_enable_msix(dev, entries, nvec);
		if (ret > 0) {
			nvec = ret;
		} else {
			break;
		}
	}

	printk("Sucessfully obtained %d MSI-Xs\n", nvec);
	for (x=0;x<nvec;x++) {
		printk("Entry %d = Virq = %d\n", entries[x].entry, entries[x].vector);
		if (request_irq(entries[x].vector, xr3pci_irq_handler, 0, DEVICE_NAME, NULL)) {
			pr_err(DEVICE_NAME ": Unable to request IRQ (MSI-X)\n");
		}
	}
#endif

	return 0;
}

static void __devexit xr3pci_test_remove(struct pci_dev *pdev)
{
	//nothing
}

static DEFINE_PCI_DEVICE_TABLE(xr3pci_test_devices) = {
	{ PCI_DEVICE_CLASS(((PCI_CLASS_BRIDGE_PCI << 8) | 0x00), ~0) },
	{ }
};
MODULE_DEVICE_TABLE(pci, xr3pci_test_devices);

static struct pci_driver xr3pci_test_driver = {
	.name		= "xr3pci-test",
	.id_table	= xr3pci_test_devices,
	.probe		= xr3pci_test_probe,
	.remove		= __devexit_p(xr3pci_test_remove),
};

static int __init xr3pci_test_init(void)
{
	return pci_register_driver(&xr3pci_test_driver);
}

static void __exit xr3pci_test_exit(void)
{
	pci_unregister_driver(&xr3pci_test_driver);
}

module_init(xr3pci_test_init);
module_exit(xr3pci_test_exit);

MODULE_AUTHOR("Andrew Murray <amurray@embedded-bits.co.uk>");
MODULE_DESCRIPTION("XR3 PCI Host Bridge Test Module");
MODULE_LICENSE("GPL");

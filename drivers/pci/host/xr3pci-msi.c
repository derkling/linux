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
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/io.h>
#include <asm/mach/pci.h>
#include <asm/mach/irq.h>

#include "xr3pci.h"

#ifdef CONFIG_PCI_MSI

/* maximum of 32 MSI's are supported */
static DECLARE_BITMAP(msi_irq_in_use, MAX_SUPPORTED_NO_MSI);

static void xr3pci_msi_nop(struct irq_data *d)
{
	return;
}

static struct irq_chip xr3pci_msi_chip = {
	.name	= "xp3RICH MSI",
	.irq_ack = xr3pci_msi_nop,
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask =  mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

int create_irq(void)
{
	int irq, pos;
	pr_debug("%s:%d create_irq\n", __func__, __LINE__);

again:
	pos = find_first_zero_bit(msi_irq_in_use, 32);
	irq = IRQ_MSI_BASE + pos;
	if (irq > NR_IRQS)
		return -ENOSPC;

	if (test_and_set_bit(pos, msi_irq_in_use))
		goto again;

	/* this may be deprecated */
	dynamic_irq_init(irq);
	
	return irq;
}

void destroy_irq(unsigned int irq)
{
	int pos = irq - IRQ_MSI_BASE;
	pr_debug("%s:%d destroy_irq\n", __func__, __LINE__);
	dynamic_irq_cleanup(irq);
	clear_bit(pos, msi_irq_in_use);
}

//TODO for now support only a single MSI per device and do not support MSI-X
int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int irq;
	struct msi_msg msg;
	
	pr_debug("%s:%d arch_setup_msi_irq\n", __func__, __LINE__);

	irq = create_irq();

	//determine IRQ
	if (irq < 0)
		return irq;

	irq_set_msi_desc(irq, desc);

	//update the endpoint with an address and data for it's MSI messages to be targeted at
	msg.address_hi = 0x0;
	msg.address_lo = 0x0; //TODO: MSI capture address is BAR0/BAR1 address and the IMSI_ADDR register offset address
	msg.data = (irq - IRQ_MSI_BASE);
	write_msi_msg(irq, &msg);

	//provides means to clear interrupt
	irq_set_chip_and_handler(irq, &xr3pci_msi_chip, handle_simple_irq);

	return 0;
}

//TODO does ILOCAL also need to be cleared after INTx interrpt?

void arch_teardown_msi_irq(unsigned irq)
{
	pr_debug("%s:%d arch_teardown_msi_irq\n", __func__, __LINE__);
	destroy_irq(irq);
}
#endif

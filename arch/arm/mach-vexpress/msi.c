/*
 *  linux/arch/arm/mach-vexpress/msi.c
 *
 *  Copyright (C) 2009 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/interrupt.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/core.h>

/* Pairs of registers (VEXPRESS_PCIE_MSI_INT_SET/CLEAR) exist to support MSI.
 * Writing a non-zero value to the SET register causes an interrupt via the
 * GIC. Each PCIe device wanting to use MSI is allocated a bit and its MSI
 * registers are programmed with the address of the SET register and a data
 * value consisting of all zeroes apart from the allocated bit. When the device
 * interrupts, that bit is written to the register, causing an interrupt. This
 * driver reads the register, finds the one bit, clears it by writing to the
 * CLEAR register (so it can interrupt again) and despatches the handler. The
 * GIC interrupt remains asserted until all bits in the SET register have been
 * cleared.
 *
 * ASSUMPTIONS:
 *	1. If there's more than one of each register (SET/CLEAR), then they are
 *	   spaced a word apart.
 *	2. If there's more than one of each register and there's one IRQ per
 *	   register (not one shared between them) then they use consecutive
 *	   interrupt numbers.
 */

#ifdef CONFIG_VEXPRESS_PCIE_RC_IN_FPGA
#define ONE_IRQ_PER_MSI_REG
#else
#undef  ONE_IRQ_PER_MSI_REG
#endif


/* We read the pending interrupt status from the SET register */
#define VEXPRESS_PCIE_MSI_INT_STATUS	VEXPRESS_PCIE_MSI_INT_SET

/* msi_irq_in_use is a bitmap to track allocated/unallocated bits in the MSI
 * registers
 */
static DECLARE_BITMAP(msi_irq_in_use, NR_MSI_IRQS_ARM_VEXPRESS);

/* NR_MSI_REGS defines how many pairs of SET/CLEAR registers there are. These
 * are assumed to be in banks i.e.
 *     SET[0]
 *     SET[1]
 *     ..
 *     CLEAR[0]
 *     CLEAR[1]
 * in consecutive words.
 */
#define NR_MSI_REGS (((NR_MSI_IRQS_ARM_VEXPRESS) + 15) / 16)


/* Interrupt handler for inbound MSIs.
 * Decodes pending MSIs and despatches the handlers.
 */
static irqreturn_t vexpress_msi_handler(int irq, void *dev_id)
{
	unsigned long reg;
	int bitnum, r;
	int handled = 0;

#ifdef ONE_IRQ_PER_MSI_REG
	r = irq - IRQ_VEXPRESS_MSI_IN;

	{
#else
	for (r = 0; r < NR_MSI_REGS; ++r) {
#endif
		/* read status register to see which devices are interrupting */
		reg = readl(__MMIO_P2V(VEXPRESS_PCIE_MSI_INT_STATUS + r*4));

		/* loop, despatching the irq handlers and rereading the
		 * status reg
		 */
		while (reg) {
			/* Find first bit set in status reg (we know reg != 0
			 * so no need to check return from find_first_bit);
			 * then clear it by writing a one to the clear reg.
			 * Note: we check all 32 bits, even though MSI uses
			 * only the low 16 bits to make sure that the interrupt
			 * gets cleared.
			 */
			bitnum = find_first_bit(&reg, 32);
			writel((1 << bitnum),
				__MMIO_P2V(VEXPRESS_PCIE_MSI_INT_CLEAR + r*4));

			/* call the handler only if the bit that was set was in
			 * the low 16 bits - MSI doesn't use bits 16..31.
			 */
			if (bitnum < min(16,NR_MSI_IRQS_ARM_VEXPRESS))
				generic_handle_irq(IRQ_VEXPRESS_MSI_0 + r*16 + bitnum);
			else
				printk(KERN_ALERT
					"PCIe MSI register %u, high bit %u "
					"is set: ignored\n", r, bitnum);

			/* finally, reread the status register and try again */
			reg = readl(__MMIO_P2V(VEXPRESS_PCIE_MSI_INT_STATUS + r*4));
			handled = 1;
		}
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}


static struct irqaction vexpress_msi_irq = {
	.name	 = "VEXPRESS MSI IRQ",
	.handler = vexpress_msi_handler,
};


/* Install the handler for the interrupt(s) for inbound MSI
 */
void __init vexpress_msi_init(void)
{
#ifdef ONE_IRQ_PER_MSI_REG
	int r;

	for (r = 0; r < NR_MSI_REGS; ++r)
		setup_irq(IRQ_VEXPRESS_MSI_IN + r*4, &vexpress_msi_irq);
#else
	setup_irq(IRQ_VEXPRESS_MSI_IN, &vexpress_msi_irq);
#endif
}


/* Destroy an MSI by removing its handler and releasing the allocated bit
 */
void arch_teardown_msi_irq(unsigned int irq)
{
	int bitnum = irq - IRQ_VEXPRESS_MSI_0;

	dynamic_irq_cleanup(irq);
	clear_bit(bitnum, msi_irq_in_use);
}


static void vexpress_msi_nop(unsigned int irq)
{
	return;
}


static struct irq_chip vexpress_msi_chip = {
	.name    = "PCIe-MSI",
	.ack     = vexpress_msi_nop,
	.enable  = unmask_msi_irq,
	.disable = mask_msi_irq,
	.mask    = mask_msi_irq,
	.unmask  = unmask_msi_irq,
};


/* Create an MSI by finding a free bit in the MSI register and creating a
 * handler for it. This function is called when a device driver calls
 * pci_enable_msi().
 */
int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int irq, bitnum;
	struct msi_msg msg;

	/* find a free bit in the bitmap that keeps track of the MSI register */
	do {
		bitnum = find_first_zero_bit(msi_irq_in_use,
					     NR_MSI_IRQS_ARM_VEXPRESS);
		if (bitnum == NR_MSI_IRQS_ARM_VEXPRESS)
			return -ENOSPC;
	} while (test_and_set_bit(bitnum, msi_irq_in_use));

	irq = IRQ_VEXPRESS_MSI_0 + bitnum;

	dynamic_irq_init(irq);
	set_irq_msi(irq, desc);
	set_irq_chip_and_handler(irq, &vexpress_msi_chip, handle_simple_irq);

	/* N.B. MSI data reg is 16 bits wide, write address 32-bit aligned */
	msg.address_hi = 0x0;
	msg.address_lo = VEXPRESS_PCIE_MSI_INT_SET + 4 * (bitnum / 16);
	msg.data = 1 << (bitnum % 16);      /* bit MSI will write */
	write_msi_msg(irq, &msg);

	return 0;
}

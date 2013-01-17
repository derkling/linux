/*
 * PCIe Host Bridge Driver Support for VExpress.
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
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/bug.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <linux/signal.h>
#include <asm/mach/pci.h>

static int xr3pci_abort(unsigned long addr, unsigned int fsr,
			struct pt_regs *regs)
{
	unsigned long pc = instruction_pointer(regs);
	unsigned long instr = *(unsigned long *)pc;

	if (fsr & (1 << 10))
		regs->ARM_pc += 4;

	if (instr == 0xa00000e) {
		regs->uregs[1] = 0xffffffff;
		return 0;
	}

	return 0;
}

int __init xr3pci_setup_arch(
	struct platform_device *dev,
	int (*map_irq)(const struct pci_dev *, u8, u8),
	struct pci_ops *ops,
	int (*setup)(int nr, struct pci_sys_data *))
{
	static struct hw_pci xr3pci_hw_pci = { 0 };

	/* At present the xr3pci returns bus errors when URs are received */
	hook_fault_code(16 + 6, xr3pci_abort, SIGBUS, 0,
			"imprecise external abort");

	xr3pci_hw_pci.map_irq = map_irq;
	xr3pci_hw_pci.ops = ops;
	xr3pci_hw_pci.setup = setup;
	xr3pci_hw_pci.nr_controllers = 1;
	xr3pci_hw_pci.of_node = of_node_get(dev->dev.of_node);

	pci_common_init(&xr3pci_hw_pci);

	return 0;
}


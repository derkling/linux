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

#include <asm/signal.h>
#include <asm/mach/pci.h>

#define MAX_SUPPORTED_DEVICES 2

static struct device_node* npnr[5];
static int __init xr3pci_setup(struct pci_sys_data *sys, struct device_node *np);
extern struct pci_ops xr3pci_ops;
int __init xr3pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin);

extern void __init xr3pci_setup_arch(
	int (**map_irq)(const struct pci_dev *, u8, u8),
	struct pci_ops **ops,
	int (**setup)(struct pci_sys_data *, struct device_node *));

static int (*setup)(struct pci_sys_data *, struct device_node *);

struct device_node *pcibios_get_phb_of_node(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct pcie_port *pp = sys->private_data;

	return of_node_get(npnr[sys->busnr]);
}

static int xr3pci_abort(unsigned long addr, unsigned int fsr, struct pt_regs *regs)
{
	if (fsr & (1 << 10))
		regs->ARM_pc += 4;
	return 0;
}

static int __init v2m_dt_xr3pci_setup(int nr, struct pci_sys_data *sys)
{
	if (nr >= MAX_SUPPORTED_DEVICES)
		return 0;

	/* This is the only callback to the driver which still uses
	   arch specific data (pci_sys_data) */
	return setup(sys, npnr[nr]);
}

static const struct __initconst of_device_id xr3pci_device_id[] = {
	{ .compatible = "arm,xr3pci", },
	{},
};

static int __init xr3pci_init(void)
{
	struct device_node *np;
	static struct hw_pci xr3pci_hw_pci = { 0 };
	
	/* Temporary abort handler until hardware behaves itself */	
	hook_fault_code(16 + 6, xr3pci_abort, SIGBUS, 0, "imprecise external abort");

	xr3pci_setup_arch(&xr3pci_hw_pci.map_irq,
			  &xr3pci_hw_pci.ops,
			  &setup);

	xr3pci_hw_pci.setup = v2m_dt_xr3pci_setup;

	for_each_matching_node(np, xr3pci_device_id) {
		/* is there enough room for another controller? */
		//TODO: use a list
		if (xr3pci_hw_pci.nr_controllers >= MAX_SUPPORTED_DEVICES) {
			pr_err("Maximum supported controllers reached\n");
			break;
		}

		/* add the new controller */
		npnr[xr3pci_hw_pci.nr_controllers++] = of_node_get(np);
	}

	/* we've found all our controllers so tell the OS to enumerate/add them */
	if (xr3pci_hw_pci.nr_controllers)
		pci_common_init(&xr3pci_hw_pci);

	return 0;
}
subsys_initcall(xr3pci_init);

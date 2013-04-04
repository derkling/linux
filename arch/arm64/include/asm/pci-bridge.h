/*
 * Copyright (C) 2013 ARM Ltd.
 *
 * Based on arch/powerpc/include/asm/pci-bridge.h
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _ASM_PCI_BRIDGE_H
#define _ASM_PCI_BRIDGE_H
#ifdef __KERNEL__

#include <linux/pci.h>
#include <linux/list.h>
#include <asm-generic/pci-bridge.h>

#define ARCH_SKIP_ISA_SUPPORT

struct device_node;

/*
 * Structure of a PCI controller (host bridge)
 */
struct pci_controller {
	struct pci_bus *bus;
	int node;
	struct device_node *dn;
	struct list_head list_node;
	struct device *parent;

	int first_busno;
	int last_busno;
	int self_busno;
	struct resource busn;

	void __iomem *io_base_virt;
	void *io_base_alloc;
	resource_size_t io_base_phys;
	resource_size_t pci_io_size;

	/* Some machines (PReP) have a non 1:1 mapping of
	 * the PCI memory space in the CPU bus space
	 */
	resource_size_t pci_mem_offset;

	struct pci_ops *ops;

	/* Currently, we limit ourselves to 1 IO range and 3 mem
	 * ranges since the common pci_bus structure can't handle more
	 */
	struct resource	io_resource;
	struct resource mem_resources[3];
	int global_number;		/* PCI domain number */
};

static inline struct pci_controller *pci_bus_to_host(const struct pci_bus *bus)
{
	return bus->sysdata;
}

#ifdef CONFIG_NUMA
#define PHB_SET_NODE(PHB, NODE)		((PHB)->node = (NODE))
#else
#define PHB_SET_NODE(PHB, NODE)		((PHB)->node = -1)
#endif

/* Allocate & free a PCI host bridge structure */
extern struct pci_controller *pcibios_alloc_controller(struct device_node *dev);
extern void pcibios_free_controller(struct pci_controller *phb);

#endif	/* __KERNEL__ */
#endif	/* _ASM_PCI_BRIDGE_H */

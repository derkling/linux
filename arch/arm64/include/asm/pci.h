/*
 * Copyright (C) 2013 ARM Ltd.
 *
 * Based on arch/powerpc/include/asm/pci.h
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

#ifndef __ASM_PCI_H
#define __ASM_PCI_H
#ifdef __KERNEL__

extern struct list_head hose_list;

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <asm-generic/pci-dma-compat.h>

#include <asm/io.h>
#include <asm/pci-bridge.h>

#define PCIBIOS_MIN_IO		0x1000
#define PCIBIOS_MIN_MEM		0x10000000

struct pci_dev;

/*
 * Set this to 1 if you want the kernel to re-assign all PCI
 * bus numbers (don't do that on ppc64 yet !)
 */
#define pcibios_assign_all_busses() \
	(pci_has_flag(PCI_REASSIGN_ALL_BUS))


static inline void pcibios_penalize_isa_irq(int irq, int active)
{
	/* We don't do dynamic PCI IRQ allocation */
}

#ifdef CONFIG_PCI
static inline void pci_dma_burst_advice(struct pci_dev *pdev,
					enum pci_dma_burst_strategy *strat,
					unsigned long *strategy_parameter)
{
	unsigned long cacheline_size;
	u8 byte;

	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &byte);
	if (byte == 0)
		cacheline_size = 1024;
	else
		cacheline_size = (int) byte * 4;

	*strat = PCI_DMA_BURST_MULTIPLE;
	*strategy_parameter = cacheline_size;
}
#endif

extern int pci_domain_nr(struct pci_bus *bus);


/* Decide whether to display the domain number in /proc */
extern int pci_proc_domain(struct pci_bus *bus);

/* The PCI address space does equal the physical memory
 * address space (no IOMMU).  The IDE and SCSI device layers use
 * this boolean for bounce buffer decisions.
 */
#define PCI_DMA_BUS_IS_PHYS     (1)

extern void pcibios_resource_survey(void);

#define HAVE_ARCH_PCI_RESOURCE_TO_USER
extern void pci_resource_to_user(const struct pci_dev *dev, int bar,
				 const struct resource *rsrc,
				 resource_size_t *start, resource_size_t *end);

extern resource_size_t pcibios_io_space_offset(struct pci_controller *hose);
extern void pcibios_setup_bus_devices(struct pci_bus *bus);
extern void pcibios_setup_bus_self(struct pci_bus *bus);
extern void pcibios_setup_phb_io_space(struct pci_controller *hose);
extern void pcibios_scan_phb(struct pci_controller *hose);

#endif	/* __KERNEL__ */
#endif /* __ASM_PCI_H */

/*
 * Based on arch/powerpc/kernel/pci_64.c
 *
 * Copyright (C) 2003 Anton Blanchard <anton@au.ibm.com>, IBM
 * Copyright (C) David Engebretsen, IBM Corp.
 * Copyright (C) 2013 ARM Ltd.
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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/of_device.h>
#include <asm/pci-bridge.h>

#define _ALIGN_DOWN(addr, size)	((addr)&(~((size)-1)))

static int __init pcibios_init(void)
{
	struct pci_controller *hose, *tmp;

	pr_info("PCI: Probing PCI hardware\n");

	/* We always enable PCI domains and we keep domain 0 backward
	 * compatible in /proc for video cards
	 */
	pci_add_flags(PCI_ENABLE_PROC_DOMAINS | PCI_COMPAT_DOMAIN_0);
	pci_add_flags(PCI_REASSIGN_ALL_BUS | PCI_REASSIGN_ALL_RSRC);

	/* Scan all of the recorded PCI controllers.  */
	list_for_each_entry_safe(hose, tmp, &hose_list, list_node) {
		pcibios_scan_phb(hose);
		pci_bus_add_devices(hose->bus);
	}

	/* Call common code to handle resource allocation */
	pcibios_resource_survey();

	pr_info("PCI: Probing PCI hardware done\n");

	return 0;
}

subsys_initcall(pcibios_init);

static int pcibios_map_phb_io_space(struct pci_controller *hose)
{
	struct vm_struct *area;
	unsigned long phys_page;
	unsigned long size_page;
	unsigned long io_virt_offset;

	/* align addr on a size boundary - adjust address up/down if needed */
	phys_page = _ALIGN_DOWN(hose->io_base_phys, PAGE_SIZE);
	size_page = PAGE_ALIGN(hose->pci_io_size);

	/* Make sure IO area address is clear */
	hose->io_base_alloc = NULL;

	/* If there's no IO to map on that bus, get away too */
	if (hose->pci_io_size == 0 || hose->io_base_phys == 0)
		return 0;

	/* Let's allocate some IO space for that guy. We don't pass
	 * VM_IOREMAP because we don't care about alignment tricks that
	 * the core does in that case. Maybe we should due to stupid card
	 * with incomplete address decoding but I'd rather not deal with
	 * those outside of the reserved 64K legacy region.
	 */
	area = __get_vm_area(size_page, 0, _IO_BASE, _IO_END);
	if (area == NULL)
		return -ENOMEM;
	hose->io_base_alloc = area->addr;
	hose->io_base_virt = (void __iomem *)(area->addr +
					      hose->io_base_phys - phys_page);

	pr_debug("IO mapping for PHB %s\n", hose->dn->full_name);
	pr_debug("  phys=0x%016llx, virt=0x%p (alloc=0x%p)\n",
		 hose->io_base_phys, hose->io_base_virt, hose->io_base_alloc);
	pr_debug("  size=0x%016llx (alloc=0x%016lx)\n",
		 hose->pci_io_size, size_page);

	/* Establish the mapping */
	if (ioremap_page_range(area->addr, (area->addr + size_page), phys_page,
			 __pgprot(PROT_DEVICE_nGnRE)) == NULL)
		return -ENOMEM;

	/* Fixup hose IO resource */
	io_virt_offset = pcibios_io_space_offset(hose);
	hose->io_resource.start += io_virt_offset;
	hose->io_resource.end += io_virt_offset;

	pr_debug("  hose->io_resource=%pR\n", &hose->io_resource);

	return 0;
}

void pcibios_setup_phb_io_space(struct pci_controller *hose)
{
	pcibios_map_phb_io_space(hose);
}

#ifdef CONFIG_NUMA
int pcibus_to_node(struct pci_bus *bus)
{
	struct pci_controller *phb = pci_bus_to_host(bus);
	return phb->node;
}
EXPORT_SYMBOL(pcibus_to_node);
#endif

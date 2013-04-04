/*
 * Based on arch/powerpc/kernel/pci-common.c
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
#include <linux/string.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/export.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>

#include <asm/processor.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/pci-bridge.h>
#include <asm/byteorder.h>

static DEFINE_SPINLOCK(hose_spinlock);
LIST_HEAD(hose_list);

/* XXX kill that some day ... */
static int global_phb_number;		/* Global phb counter */

struct pci_controller *pcibios_alloc_controller(struct device_node *dev)
{
	struct pci_controller *phb;

	phb = kzalloc(sizeof(struct pci_controller), GFP_KERNEL | __GFP_ZERO);
	if (phb == NULL)
		return NULL;
	spin_lock(&hose_spinlock);
	phb->global_number = global_phb_number++;
	list_add_tail(&phb->list_node, &hose_list);
	spin_unlock(&hose_spinlock);
	phb->dn = dev;
	if (dev) {
		int nid = of_node_to_nid(dev);

		if (nid < 0 || !node_online(nid))
			nid = -1;

		PHB_SET_NODE(phb, nid);
	}
	return phb;
}

void pcibios_free_controller(struct pci_controller *phb)
{
	spin_lock(&hose_spinlock);
	list_del(&phb->list_node);
	spin_unlock(&hose_spinlock);
	kfree(phb);
}

/*
 * Return the domain number for this bus.
 */
int pci_domain_nr(struct pci_bus *bus)
{
	struct pci_controller *hose = pci_bus_to_host(bus);

	return hose->global_number;
}
EXPORT_SYMBOL(pci_domain_nr);

/*
 * Reads the interrupt pin to determine if interrupt is use by card.
 * If the interrupt is used, then gets the interrupt line from the
 * openfirmware and sets it in the pci_dev and pci_config line.
 */
static int pci_read_irq_line(struct pci_dev *pci_dev)
{
	struct of_irq oirq;
	unsigned int virq;

	pr_debug("PCI: Try to map irq for %s...\n", pci_name(pci_dev));

#ifdef DEBUG
	memset(&oirq, 0xff, sizeof(oirq));
#endif
	/* Try to get a mapping from the device-tree */
	if (of_irq_map_pci(pci_dev, &oirq)) {
		u8 line, pin;

		/* If that fails, lets fallback to what is in the config
		 * space and map that through the default controller. We
		 * also set the type to level low since that's what PCI
		 * interrupts are. If your platform does differently, then
		 * either provide a proper interrupt tree or don't use this
		 * function.
		 */
		if (pci_read_config_byte(pci_dev, PCI_INTERRUPT_PIN, &pin))
			return -1;
		if (pin == 0)
			return -1;
		if (pci_read_config_byte(pci_dev, PCI_INTERRUPT_LINE, &line) ||
		    line == 0xff || line == 0) {
			return -1;
		}
		pr_debug(" No map ! Using line %d (pin %d) from PCI config\n",
			 line, pin);

		virq = irq_create_mapping(NULL, line);
		if (virq != 0)
			irq_set_irq_type(virq, IRQ_TYPE_LEVEL_LOW);
	} else {
		pr_debug(" Got one, spec %d cells (0x%08x 0x%08x...) on %s\n",
			 oirq.size, oirq.specifier[0], oirq.specifier[1],
			 of_node_full_name(oirq.controller));

		virq = irq_create_of_mapping(oirq.controller, oirq.specifier,
					     oirq.size);
	}
	if (virq == 0) {
		pr_debug(" Failed to map !\n");
		return -1;
	}

	pr_debug(" Mapped to linux irq %d\n", virq);

	pci_dev->irq = virq;
	return 0;
}

void pci_resource_to_user(const struct pci_dev *dev, int bar,
			  const struct resource *rsrc,
			  resource_size_t *start, resource_size_t *end)
{
	struct pci_controller *hose = pci_bus_to_host(dev->bus);
	resource_size_t offset = 0;

	if (hose == NULL)
		return;

	if (rsrc->flags & IORESOURCE_IO)
		offset = (unsigned long)hose->io_base_virt - _IO_BASE;

	*start = rsrc->start - offset;
	*end = rsrc->end - offset;
}

/* Decide whether to display the domain number in /proc */
int pci_proc_domain(struct pci_bus *bus)
{
	struct pci_controller *hose = pci_bus_to_host(bus);

	if (!pci_has_flag(PCI_ENABLE_PROC_DOMAINS))
		return 0;
	if (pci_has_flag(PCI_COMPAT_DOMAIN_0))
		return hose->global_number != 0;
	return 1;
}

/* This header fixup will do the resource fixup for all devices as they are
 * probed, but not for bridge ranges
 */
static void pcibios_fixup_resources(struct pci_dev *dev)
{
	struct pci_controller *hose = pci_bus_to_host(dev->bus);
	int i;

	if (!hose) {
		pr_err("No host bridge for PCI dev %s !\n",
		       pci_name(dev));
		return;
	}
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		struct resource *res = dev->resource + i;
		if (!res->flags)
			continue;

		/* If we're going to re-assign everything, we mark all resources
		 * as unset (and 0-base them). In addition, we mark BARs
		 * starting at 0 as unset as well, except if PCI_PROBE_ONLY is
		 * also set since in that case, we don't want to re-assign
		 * anything
		 */
		if (pci_has_flag(PCI_REASSIGN_ALL_RSRC) ||
		    (res->start == 0 && !pci_has_flag(PCI_PROBE_ONLY))) {
			/* Only print message if not re-assigning */
			if (!pci_has_flag(PCI_REASSIGN_ALL_RSRC))
				pr_debug("PCI:%s Resource %d %016llx-%016llx "
					 "[%x] is unassigned\n",
					 pci_name(dev), i,
					 (unsigned long long)res->start,
					 (unsigned long long)res->end,
					 (unsigned int)res->flags);
			res->end -= res->start;
			res->start = 0;
			res->flags |= IORESOURCE_UNSET;
			continue;
		}

		pr_debug("PCI:%s Resource %d %016llx-%016llx [%x]\n",
			 pci_name(dev), i,
			 (unsigned long long)res->start,
			 (unsigned long long)res->end,
			 (unsigned int)res->flags);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_ANY_ID, PCI_ANY_ID, pcibios_fixup_resources);

/* This function tries to figure out if a bridge resource has been initialized
 * by the firmware or not. It doesn't have to be absolutely bullet proof, but
 * things go more smoothly when it gets it right. It should covers cases such
 * as Apple "closed" bridge resources and bare-metal pSeries unassigned bridges
 */
static int pcibios_uninitialized_bridge_resource(struct pci_bus *bus,
						 struct resource *res)
{
	struct pci_controller *hose = pci_bus_to_host(bus);
	struct pci_dev *dev = bus->self;
	resource_size_t offset;
	u16 command;
	int i;

	/* We don't do anything if PCI_PROBE_ONLY is set */
	if (pci_has_flag(PCI_PROBE_ONLY))
		return 0;

	/* Job is a bit different between memory and IO */
	if (res->flags & IORESOURCE_MEM) {
		/* If the BAR is non-0 (res != pci_mem_offset) then it's
		 * probably been initialized by somebody
		 */
		if (res->start != hose->pci_mem_offset)
			return 0;

		/* The BAR is 0, let's check if memory decoding is enabled on
		 * the bridge. If not, we consider it unassigned
		 */
		pci_read_config_word(dev, PCI_COMMAND, &command);
		if ((command & PCI_COMMAND_MEMORY) == 0)
			return 1;

		/* Memory decoding is enabled and the BAR is 0. If any of the
		 * bridge resources covers that starting address (0 then it's
		 * good enough for us for memory
		 */
		for (i = 0; i < 3; i++) {
			if ((hose->mem_resources[i].flags & IORESOURCE_MEM) &&
			    hose->mem_resources[i].start == hose->pci_mem_offset)
				return 0;
		}

		/* Well, it starts at 0 and we know it will collide so we may as
		 * well consider it as unassigned. That covers the Apple case.
		 */
		return 1;
	} else {
		/* If the BAR is non-0, then we consider it assigned */
		offset = (unsigned long)hose->io_base_virt - _IO_BASE;
		if (((res->start - offset) & 0xfffffffful) != 0)
			return 0;

		/* Here, we are a bit different than memory as typically IO
		 * space starting at low addresses -is- valid. What we do
		 * instead if that we consider as unassigned anything that
		 * doesn't have IO enabled in the PCI command register, and
		 * that's it.
		 */
		pci_read_config_word(dev, PCI_COMMAND, &command);
		if (command & PCI_COMMAND_IO)
			return 0;

		/* It's starting at 0 and IO is disabled in the bridge, consider
		 * it unassigned
		 */
		return 1;
	}
}

/* Fixup resources of a PCI<->PCI bridge */
static void pcibios_fixup_bridge(struct pci_bus *bus)
{
	struct resource *res;
	int i;

	struct pci_dev *dev = bus->self;

	pci_bus_for_each_resource(bus, res, i) {
		if (!res || !res->flags)
			continue;
		if (i >= 3 && bus->self->transparent)
			continue;

		/* If we're going to reassign everything, we can
		 * shrink the P2P resource to have size as being
		 * of 0 in order to save space.
		 */
		if (pci_has_flag(PCI_REASSIGN_ALL_RSRC)) {
			res->flags |= IORESOURCE_UNSET;
			res->start = 0;
			res->end = -1;
			continue;
		}

		pr_debug("PCI:%s Bus rsrc %d %016llx-%016llx [%x]\n",
			 pci_name(dev), i,
			 (unsigned long long)res->start,
			 (unsigned long long)res->end,
			 (unsigned int)res->flags);

		/* Try to detect uninitialized P2P bridge resources,
		 * and clear them out so they get re-assigned later
		 */
		if (pcibios_uninitialized_bridge_resource(bus, res)) {
			res->flags = 0;
			pr_debug("PCI:%s            (unassigned)\n", pci_name(dev));
		}
	}
}

void pcibios_setup_bus_self(struct pci_bus *bus)
{
	/* Fix up the bus resources for P2P bridges */
	if (bus->self != NULL)
		pcibios_fixup_bridge(bus);
}

void pcibios_setup_bus_devices(struct pci_bus *bus)
{
	struct pci_dev *dev;

	pr_debug("PCI: Fixup bus devices %d (%s)\n",
		 bus->number, bus->self ? pci_name(bus->self) : "PHB");

	list_for_each_entry(dev, &bus->devices, bus_list) {
		/* Cardbus can call us to add new devices to a bus, so ignore
		 * those who are already fully discovered
		 */
		if (dev->is_added)
			continue;

		/* Fixup NUMA node as it may not be setup yet by the generic
		 * code and is needed by the DMA init
		 */
		set_dev_node(&dev->dev, pcibus_to_node(dev->bus));
	
		/* Hook up default DMA ops */
		set_dma_ops(&dev->dev, &noncoherent_swiotlb_dma_ops);	

		/* Read default IRQs and fixup if necessary */
		pci_read_irq_line(dev);
	}
}

void pcibios_set_master(struct pci_dev *dev)
{
	/* No special bus mastering setup handling */
}

void pcibios_fixup_bus(struct pci_bus *bus)
{
	/* When called from the generic PCI probe, read PCI<->PCI bridge
	 * bases. This is -not- called when generating the PCI tree from
	 * the OF device-tree.
	 */
	if (bus->self != NULL)
		pci_read_bridge_bases(bus);

	/* Now fixup the bus bus */
	pcibios_setup_bus_self(bus);

	/* Now fixup devices on that bus */
	pcibios_setup_bus_devices(bus);
}
EXPORT_SYMBOL(pcibios_fixup_bus);

void pci_fixup_cardbus(struct pci_bus *bus)
{
	/* Now fixup devices on that bus */
	pcibios_setup_bus_devices(bus);
}


static int skip_isa_ioresource_align(struct pci_dev *dev)
{
	if (pci_has_flag(PCI_CAN_SKIP_ISA_ALIGN) &&
	    !(dev->bus->bridge_ctl & PCI_BRIDGE_CTL_ISA))
		return 1;
	return 0;
}

/*
 * We need to avoid collisions with `mirrored' VGA ports
 * and other strange ISA hardware, so we always want the
 * addresses to be allocated in the 0x000-0x0ff region
 * modulo 0x400.
 *
 * Why? Because some silly external IO cards only decode
 * the low 10 bits of the IO address. The 0x00-0xff region
 * is reserved for motherboard devices that decode all 16
 * bits, so it's ok to allocate at, say, 0x2800-0x28ff,
 * but we want to try to avoid allocating at 0x2900-0x2bff
 * which might have be mirrored at 0x0100-0x03ff..
 */
resource_size_t pcibios_align_resource(void *data, const struct resource *res,
				resource_size_t size, resource_size_t align)
{
	struct pci_dev *dev = data;
	resource_size_t start = res->start;

	if (res->flags & IORESOURCE_IO) {
		if (skip_isa_ioresource_align(dev))
			return start;
		if (start & 0x300)
			start = (start + 0x3ff) & ~0x3ff;
	}

	return start;
}
EXPORT_SYMBOL(pcibios_align_resource);

/*
 * Reparent resource children of pr that conflict with res
 * under res, and make res replace those children.
 */
static int reparent_resources(struct resource *parent,
				     struct resource *res)
{
	struct resource *p, **pp;
	struct resource **firstpp = NULL;

	for (pp = &parent->child; (p = *pp) != NULL; pp = &p->sibling) {
		if (p->end < res->start)
			continue;
		if (res->end < p->start)
			break;
		if (p->start < res->start || p->end > res->end)
			return -1;	/* not completely contained */
		if (firstpp == NULL)
			firstpp = pp;
	}
	if (firstpp == NULL)
		return -1;	/* didn't find any conflicting entries? */
	res->parent = parent;
	res->child = *firstpp;
	res->sibling = *pp;
	*firstpp = res;
	*pp = NULL;
	for (p = res->child; p != NULL; p = p->sibling) {
		p->parent = res;
		pr_debug("PCI: Reparented %s [%llx..%llx] under %s\n",
			 p->name,
			 (unsigned long long)p->start,
			 (unsigned long long)p->end, res->name);
	}
	return 0;
}

/*
 *  Handle resources of PCI devices.  If the world were perfect, we could
 *  just allocate all the resource regions and do nothing more.  It isn't.
 *  On the other hand, we cannot just re-allocate all devices, as it would
 *  require us to know lots of host bridge internals.  So we attempt to
 *  keep as much of the original configuration as possible, but tweak it
 *  when it's found to be wrong.
 *
 *  Known BIOS problems we have to work around:
 *	- I/O or memory regions not configured
 *	- regions configured, but not enabled in the command register
 *	- bogus I/O addresses above 64K used
 *	- expansion ROMs left enabled (this may sound harmless, but given
 *	  the fact the PCI specs explicitly allow address decoders to be
 *	  shared between expansion ROMs and other resource regions, it's
 *	  at least dangerous)
 *
 *  Our solution:
 *	(1) Allocate resources for all buses behind PCI-to-PCI bridges.
 *	    This gives us fixed barriers on where we can allocate.
 *	(2) Allocate resources for all enabled devices.  If there is
 *	    a collision, just mark the resource as unallocated. Also
 *	    disable expansion ROMs during this step.
 *	(3) Try to allocate resources for disabled devices.  If the
 *	    resources were assigned correctly, everything goes well,
 *	    if they weren't, they won't disturb allocation of other
 *	    resources.
 *	(4) Assign new addresses to resources which were either
 *	    not configured at all or misconfigured.  If explicitly
 *	    requested by the user, configure expansion ROM address
 *	    as well.
 */

void pcibios_allocate_bus_resources(struct pci_bus *bus)
{
	struct pci_bus *b;
	int i;
	struct resource *res, *pr;

	pr_debug("PCI: Allocating bus resources for %04x:%02x...\n",
		 pci_domain_nr(bus), bus->number);

	pci_bus_for_each_resource(bus, res, i) {
		if (!res || !res->flags || res->start > res->end || res->parent)
			continue;

		/* If the resource was left unset at this point, we clear it */
		if (res->flags & IORESOURCE_UNSET)
			goto clear_resource;

		if (bus->parent == NULL)
			pr = (res->flags & IORESOURCE_IO) ?
				&ioport_resource : &iomem_resource;
		else {
			pr = pci_find_parent_resource(bus->self, res);
			if (pr == res) {
				/* this happens when the generic PCI
				 * code (wrongly) decides that this
				 * bridge is transparent  -- paulus
				 */
				continue;
			}
		}

		pr_debug("PCI: %s (bus %d) bridge rsrc %d: %016llx-%016llx "
			 "[0x%x], parent %p (%s)\n",
			 bus->self ? pci_name(bus->self) : "PHB",
			 bus->number, i,
			 (unsigned long long)res->start,
			 (unsigned long long)res->end,
			 (unsigned int)res->flags,
			 pr, (pr && pr->name) ? pr->name : "nil");

		if (pr && !(pr->flags & IORESOURCE_UNSET)) {
			if (request_resource(pr, res) == 0)
				continue;
			/*
			 * Must be a conflict with an existing entry.
			 * Move that entry (or entries) under the
			 * bridge resource and try again.
			 */
			if (reparent_resources(pr, res) == 0)
				continue;
		}
		pr_warn("PCI: Cannot allocate resource region "
			   "%d of PCI bridge %d, will remap\n", i, bus->number);
clear_resource:
		/* The resource might be figured out when doing
		 * reassignment based on the resources required
		 * by the downstream PCI devices. Here we set
		 * the size of the resource to be 0 in order to
		 * save more space.
		 */
		res->start = 0;
		res->end = -1;
		res->flags = 0;
	}

	list_for_each_entry(b, &bus->children, node)
		pcibios_allocate_bus_resources(b);
}

static inline void alloc_resource(struct pci_dev *dev, int idx)
{
	struct resource *pr, *r = &dev->resource[idx];

	pr_debug("PCI: Allocating %s: Resource %d: %016llx..%016llx [%x]\n",
		 pci_name(dev), idx,
		 (unsigned long long)r->start,
		 (unsigned long long)r->end,
		 (unsigned int)r->flags);

	pr = pci_find_parent_resource(dev, r);
	if (!pr || (pr->flags & IORESOURCE_UNSET) ||
	    request_resource(pr, r) < 0) {
		pr_warn("PCI: Cannot allocate resource region %d"
		       " of device %s, will remap\n", idx, pci_name(dev));
		if (pr)
			pr_debug("PCI:  parent is %p: %016llx-%016llx [%x]\n",
				 pr,
				 (unsigned long long)pr->start,
				 (unsigned long long)pr->end,
				 (unsigned int)pr->flags);
		/* We'll assign a new address later */
		r->flags |= IORESOURCE_UNSET;
		r->end -= r->start;
		r->start = 0;
	}
}

static void __init pcibios_allocate_resources(int pass)
{
	struct pci_dev *dev = NULL;
	int idx, disabled;
	u16 command;
	struct resource *r;

	for_each_pci_dev(dev) {
		pci_read_config_word(dev, PCI_COMMAND, &command);
		for (idx = 0; idx <= PCI_ROM_RESOURCE; idx++) {
			r = &dev->resource[idx];
			if (r->parent)		/* Already allocated */
				continue;
			if (!r->flags || (r->flags & IORESOURCE_UNSET))
				continue;	/* Not assigned at all */
			/* We only allocate ROMs on pass 1 just in case they
			 * have been screwed up by firmware
			 */
			if (idx == PCI_ROM_RESOURCE)
				disabled = 1;
			if (r->flags & IORESOURCE_IO)
				disabled = !(command & PCI_COMMAND_IO);
			else
				disabled = !(command & PCI_COMMAND_MEMORY);
			if (pass == disabled)
				alloc_resource(dev, idx);
		}
		if (pass)
			continue;
		r = &dev->resource[PCI_ROM_RESOURCE];
		if (r->flags) {
			/* Turn the ROM off, leave the resource region,
			 * but keep it unregistered.
			 */
			u32 reg;
			pci_read_config_dword(dev, dev->rom_base_reg, &reg);
			if (reg & PCI_ROM_ADDRESS_ENABLE) {
				pr_debug("PCI: Switching off ROM of %s\n",
					 pci_name(dev));
				r->flags &= ~IORESOURCE_ROM_ENABLE;
				pci_write_config_dword(dev, dev->rom_base_reg,
						reg & ~PCI_ROM_ADDRESS_ENABLE);
			}
		}
	}
}

static void __init pcibios_reserve_legacy_regions(struct pci_bus *bus)
{
	struct pci_controller *hose = pci_bus_to_host(bus);
	resource_size_t	offset;
	struct resource *res, *pres;
	int i;

	pr_debug("Reserving legacy ranges for domain %04x\n", pci_domain_nr(bus));

	/* Check for IO */
	if (!(hose->io_resource.flags & IORESOURCE_IO))
		goto no_io;
	offset = (unsigned long)hose->io_base_virt - _IO_BASE;
	res = kzalloc(sizeof(struct resource), GFP_KERNEL);
	BUG_ON(res == NULL);
	res->name = "Legacy IO";
	res->flags = IORESOURCE_IO;
	res->start = offset;
	res->end = (offset + 0xfff) & 0xfffffffful;
	pr_debug("Candidate legacy IO: %pR\n", res);
	if (request_resource(&hose->io_resource, res)) {
		pr_debug("PCI %04x:%02x Cannot reserve Legacy IO %pR\n",
		       pci_domain_nr(bus), bus->number, res);
		kfree(res);
	}

 no_io:
	/* Check for memory */
	offset = hose->pci_mem_offset;
	pr_debug("hose mem offset: %016llx\n", (unsigned long long)offset);
	for (i = 0; i < 3; i++) {
		pres = &hose->mem_resources[i];
		if (!(pres->flags & IORESOURCE_MEM))
			continue;
		pr_debug("hose mem res: %pR\n", pres);
		if ((pres->start - offset) <= 0xa0000 &&
		    (pres->end - offset) >= 0xbffff)
			break;
	}
	if (i >= 3)
		return;
	res = kzalloc(sizeof(struct resource), GFP_KERNEL);
	BUG_ON(res == NULL);
	res->name = "Legacy VGA memory";
	res->flags = IORESOURCE_MEM;
	res->start = 0xa0000 + offset;
	res->end = 0xbffff + offset;
	pr_debug("Candidate VGA memory: %pR\n", res);
	if (request_resource(pres, res)) {
		pr_debug("PCI %04x:%02x Cannot reserve VGA memory %pR\n",
		       pci_domain_nr(bus), bus->number, res);
		kfree(res);
	}
}

void __init pcibios_resource_survey(void)
{
	struct pci_bus *b;

	/* Allocate and assign resources */
	list_for_each_entry(b, &pci_root_buses, node)
		pcibios_allocate_bus_resources(b);
	pcibios_allocate_resources(0);
	pcibios_allocate_resources(1);

	/* Before we start assigning unassigned resource, we try to reserve
	 * the low IO area and the VGA memory area if they intersect the
	 * bus available resources to avoid allocating things on top of them
	 */
	if (!pci_has_flag(PCI_PROBE_ONLY)) {
		list_for_each_entry(b, &pci_root_buses, node)
			pcibios_reserve_legacy_regions(b);
	}

	/* Now, if the platform didn't decide to blindly trust the firmware,
	 * we proceed to assigning things that were left unassigned
	 */
	if (!pci_has_flag(PCI_PROBE_ONLY)) {
		pr_debug("PCI: Assigning unassigned resources...\n");
		pci_assign_unassigned_resources();
	}
}

int pcibios_enable_device(struct pci_dev *dev, int mask)
{

	return pci_enable_resources(dev, mask);
}

resource_size_t pcibios_io_space_offset(struct pci_controller *hose)
{
	return (unsigned long) hose->io_base_virt - _IO_BASE;
}

static void pcibios_setup_phb_resources(struct pci_controller *hose,
					struct list_head *resources)
{
	struct resource *res;
	int i;

	/* Hookup PHB IO resource */
	res = &hose->io_resource;

	if (!res->flags) {
		pr_warn("PCI: I/O resource not set for host"
		       " bridge %s (domain %d)\n",
		       hose->dn->full_name, hose->global_number);
	}

	pr_debug("PCI: PHB IO resource    = %016llx-%016llx [%lx]\n",
		 (unsigned long long)res->start,
		 (unsigned long long)res->end,
		 (unsigned long)res->flags);
	pci_add_resource_offset(resources, res, pcibios_io_space_offset(hose));

	/* Hookup PHB Memory resources */
	for (i = 0; i < 3; ++i) {
		res = &hose->mem_resources[i];
		if (!res->flags) {
			if (i > 0)
				continue;
			pr_err("PCI: Memory resource 0 not set for "
			       "host bridge %s (domain %d)\n",
			       hose->dn->full_name, hose->global_number);
		}

		pr_debug("PCI: PHB MEM resource %d = %016llx-%016llx [%lx]\n",
			 i,
			 (unsigned long long)res->start,
			 (unsigned long long)res->end,
			 (unsigned long)res->flags);
		pci_add_resource_offset(resources, res, hose->pci_mem_offset);
	}

	pr_debug("PCI: PHB MEM offset     = %016llx\n",
		 (unsigned long long)hose->pci_mem_offset);
	pr_debug("PCI: PHB IO  offset     = %08lx\n",
		 (unsigned long)hose->io_base_virt - _IO_BASE);

}

struct device_node *pcibios_get_phb_of_node(struct pci_bus *bus)
{
	struct pci_controller *hose = bus->sysdata;

	return of_node_get(hose->dn);
}

/**
 * pci_scan_phb - Given a pci_controller, setup and scan the PCI bus
 * @hose: Pointer to the PCI host controller instance structure
 */
void pcibios_scan_phb(struct pci_controller *hose)
{
	LIST_HEAD(resources);
	struct pci_bus *bus;
	struct device_node *node = hose->dn;

	pr_debug("PCI: Scanning PHB %s\n", of_node_full_name(node));

	/* Get some IO space for the new PHB */
	pcibios_setup_phb_io_space(hose);

	/* Wire up PHB bus resources */
	pcibios_setup_phb_resources(hose, &resources);

	hose->busn.start = hose->first_busno;
	hose->busn.end	 = hose->last_busno;
	hose->busn.flags = IORESOURCE_BUS;
	pci_add_resource(&resources, &hose->busn);

	/* Create an empty bus for the toplevel */
	bus = pci_create_root_bus(hose->parent, hose->first_busno,
				  hose->ops, hose, &resources);
	if (bus == NULL) {
		pr_err("Failed to create bus for PCI domain %04x\n",
			hose->global_number);
		pci_free_resource_list(&resources);
		return;
	}
	hose->bus = bus;

	/* Perform scan */
	pci_bus_update_busn_res_end(bus, 255);
	hose->last_busno = pci_scan_child_bus(bus);
	pci_bus_update_busn_res_end(bus, hose->last_busno);

	/* Configure PCI Express settings */
	if (bus && !pci_has_flag(PCI_PROBE_ONLY)) {
		struct pci_bus *child;
		list_for_each_entry(child, &bus->children, node) {
			struct pci_dev *self = child->self;
			if (!self)
				continue;
			pcie_bus_configure_settings(child, self->pcie_mpss);
		}
	}
}

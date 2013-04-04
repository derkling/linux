#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/of_address.h>
#include <asm/prom.h>

#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64) || \
	defined(CONFIG_MICROBLAZE) || defined(CONFIG_ARM64)
#include <asm/pci-bridge.h>
#endif

static inline int __of_pci_pci_compare(struct device_node *node,
				       unsigned int devfn)
{
	unsigned int size;
	const __be32 *reg = of_get_property(node, "reg", &size);

	if (!reg || size < 5 * sizeof(__be32))
		return 0;
	return ((be32_to_cpup(&reg[0]) >> 8) & 0xff) == devfn;
}

struct device_node *of_pci_find_child_device(struct device_node *parent,
					     unsigned int devfn)
{
	struct device_node *node, *node2;

	for_each_child_of_node(parent, node) {
		if (__of_pci_pci_compare(node, devfn))
			return node;
		/*
		 * Some OFs create a parent node "multifunc-device" as
		 * a fake root for all functions of a multi-function
		 * device we go down them as well.
		 */
		if (!strcmp(node->name, "multifunc-device")) {
			for_each_child_of_node(node, node2) {
				if (__of_pci_pci_compare(node2, devfn)) {
					of_node_put(node);
					return node2;
				}
			}
		}
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(of_pci_find_child_device);

/**
 * pci_process_bridge_OF_ranges - Parse PCI bridge resources from device tree
 * @hose: newly allocated pci_controller to be setup
 * @dev: device node of the host bridge
 * @primary: set if primary bus (32 bits only, soon to be deprecated)
 *
 * This function will parse the "ranges" property of a PCI host bridge device
 * node and setup the resource mapping of a pci controller based on its
 * content.
 *
 * Life would be boring if it wasn't for a few issues that we have to deal
 * with here:
 *
 *   - We can only cope with one IO space range and up to 3 Memory space
 *     ranges. However, some machines (thanks Apple !) tend to split their
 *     space into lots of small contiguous ranges. So we have to coalesce.
 *
 *   - We can only cope with all memory ranges having the same offset
 *     between CPU addresses and PCI addresses. Unfortunately, some bridges
 *     are setup for a large 1:1 mapping along with a small "window" which
 *     maps PCI address 0 to some arbitrary high address of the CPU space in
 *     order to give access to the ISA memory hole.
 *     The way out of here that I've chosen for now is to always set the
 *     offset based on the first resource found, then override it if we
 *     have a different offset and the previous was set by an ISA hole.
 *
 *   - Some busses have IO space not starting at 0, which causes trouble with
 *     the way we do our IO resource renumbering. The code somewhat deals with
 *     it for 64 bits but I would expect problems on 32 bits.
 *
 *   - Some 32 bits platforms such as 4xx can have physical space larger than
 *     32 bits so we need to use 64 bits values for the parsing
 */
#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64) || \
	defined(CONFIG_MICROBLAZE) || defined(CONFIG_ARM64)
void pci_process_bridge_OF_ranges(struct pci_controller *hose,
				  struct device_node *dev, int primary)
{
	int memno = 0, isa_hole = -1;
	unsigned long long isa_mb = 0;
	struct resource *res;
	struct of_pci_range range;
	struct of_pci_range_parser parser;

	pr_info("PCI host bridge %s %s ranges:\n",
	       dev->full_name, primary ? "(primary)" : "");

	/* Check for ranges property */
	if (of_pci_range_parser_init(&parser, dev))
		return;

	pr_debug("Parsing ranges property...\n");
	for_each_of_pci_range(&parser, &range) {
		/* Read next ranges element */
		pr_debug("pci_space: 0x%08x pci_addr: 0x%016llx ",
				range.pci_space, range.pci_addr);
		pr_debug("cpu_addr: 0x%016llx size: 0x%016llx\n",
				range.cpu_addr, range.size);

		/* If we failed translation or got a zero-sized region
		 * (some FW try to feed us with non sensical zero sized regions
		 * such as power3 which look like some kind of attempt
		 * at exposing the VGA memory hole)
		 */
		if (range.cpu_addr == OF_BAD_ADDR || range.size == 0)
			continue;

		/* Act based on address space type */
		res = NULL;
		switch (range.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			pr_info("  IO 0x%016llx..0x%016llx -> 0x%016llx\n",
				range.cpu_addr, range.cpu_addr + range.size - 1,
				range.pci_addr);

			/* We support only one IO range */
			if (hose->pci_io_size) {
				pr_info(" \\--> Skipped (too many) !\n");
				continue;
			}
#if (!IS_ENABLED(CONFIG_64BIT))
			/* On 32 bits, limit I/O space to 16MB */
			if (range.size > 0x01000000)
				range.size = 0x01000000;

			/* 32 bits needs to map IOs here */
			hose->io_base_virt = ioremap(range.cpu_addr,
						range.size);

			/* Expect trouble if pci_addr is not 0 */
			if (primary)
				isa_io_base =
					(unsigned long)hose->io_base_virt;
#endif /* !CONFIG_64BIT */
			/* pci_io_size and io_base_phys always represent IO
			 * space starting at 0 so we factor in pci_addr
			 */
			hose->pci_io_size = range.pci_addr + range.size;
			hose->io_base_phys = range.cpu_addr - range.pci_addr;

			/* Build resource */
			res = &hose->io_resource;
			range.cpu_addr = range.pci_addr;

			break;

		case IORESOURCE_MEM:
			pr_info(" MEM 0x%016llx..0x%016llx -> 0x%016llx %s\n",
				range.cpu_addr, range.cpu_addr + range.size - 1,
				range.pci_addr,
				(range.pci_space & 0x40000000) ?
				"Prefetch" : "");

			/* We support only 3 memory ranges */
			if (memno >= 3) {
				pr_info(" \\--> Skipped (too many) !\n");
				continue;
			}

#ifndef ARCH_SKIP_ISA_SUPPORT
			/* Handles ISA memory hole space here */
			if (range.pci_addr == 0) {
				isa_mb = range.cpu_addr;
				isa_hole = memno;
				if (primary || isa_mem_base == 0)
					isa_mem_base = range.cpu_addr;
				hose->isa_mem_phys = range.cpu_addr;
				hose->isa_mem_size = range.size;
			}
#endif

			/* We get the PCI/Mem offset from the first range or
			 * the, current one if the offset came from an ISA
			 * hole. If they don't match, bugger.
			 */
			if (memno == 0 ||
			(isa_hole >= 0 && range.pci_addr != 0 &&
			     hose->pci_mem_offset == isa_mb))
				hose->pci_mem_offset = range.cpu_addr -
							range.pci_addr;
			else if (range.pci_addr != 0 &&
				hose->pci_mem_offset != range.cpu_addr -
							range.pci_addr) {
				pr_info(" \\--> Skipped (offset mismatch) !\n");
				continue;
			}

			/* Build resource */
			res = &hose->mem_resources[memno++];

			break;
		}
		if (res != NULL)
			of_pci_range_to_resource(&range, dev, res);
	}

	/* If there's an ISA hole and the pci_mem_offset is -not- matching
	 * the ISA hole offset, then we need to remove the ISA hole from
	 * the resource list for that brige
	 */
	if (isa_hole >= 0 && hose->pci_mem_offset != isa_mb) {
		unsigned int next = isa_hole + 1;
		pr_info(" Removing ISA hole at 0x%016llx\n", isa_mb);
		if (next < memno)
			memmove(&hose->mem_resources[isa_hole],
				&hose->mem_resources[next],
				sizeof(struct resource) * (memno - next));
		hose->mem_resources[--memno].flags = 0;
	}
}
#endif

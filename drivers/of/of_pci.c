#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/of_address.h>
#include <asm/prom.h>

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

u32* of_pci_process_ranges(struct device_node *node, struct resource *res, u32 *from)
{
	u32 *start, *end;
	int na, ns, np, pna;
	int rlen;

	of_count_cells(node, &na, &ns);
	pna = of_n_addr_cells(node);
	np = pna + na + ns;

	WARN_ON(na != 3 || ns != 2 || pna > 2);
	WARN_ON(!res);

	start = (u32 *)of_get_property(node, "ranges", &rlen);
	if (start == NULL)
		return NULL;

	end = start + rlen;

	if (!from)
		from = start;

	while (from + np <= end) {
		u32 pci_space;
		u64 pci_addr, cpu_addr, size;

		pci_space = of_read_number(from, 1);
		pci_addr = of_read_number(from + 1, 2);
		cpu_addr = of_translate_address(node, from + 3);
		size = of_read_number(from + 3 + pna, ns);
		from += np;

		if (cpu_addr == OF_BAD_ADDR || size == 0)
			continue;

		switch ((pci_space >> 24) & 0x3) {
			case 1:
				res->flags = IORESOURCE_IO;
				break;
			case 2:
			case 3:
				res->flags = IORESOURCE_MEM;
				if (pci_space & 0x40000000)
					res->flags |= IORESOURCE_PREFETCH;
				break;
			default:
				continue;
		}
		res->name = node->full_name;
		res->start = cpu_addr;
		res->end = res->start + size - 1;
		res->parent = res->child = res->sibling = NULL;
		return from;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(of_pci_process_ranges);

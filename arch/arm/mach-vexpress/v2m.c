#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/ata_platform.h>
#include <linux/smsc911x.h>
#include <linux/spinlock.h>
#include <linux/usb/isp1760.h>
#include <linux/mtd/physmap.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/vexpress.h>
#include <linux/clkdev.h>

#include <asm/mach/arch.h>

#include "core.h"

static void __init v2m_dt_hdlcd_init(void)
{
	struct device_node *node;
	int len, na, ns;
	const __be32 *prop;
	phys_addr_t fb_base, fb_size;

	node = of_find_compatible_node(NULL, NULL, "arm,hdlcd");
	if (!node)
		return;

	na = of_n_addr_cells(node);
	ns = of_n_size_cells(node);

	prop = of_get_property(node, "framebuffer", &len);
	if (WARN_ON(!prop || len < (na + ns) * sizeof(*prop)))
		return;

	fb_base = of_read_number(prop, na);
	fb_size = of_read_number(prop + na, ns);

	if (WARN_ON(memblock_remove(fb_base, fb_size)))
		return;
};

void __init v2m_dt_init_early(void)
{
	v2m_dt_hdlcd_init();
}
static const char * const v2m_dt_match[] __initconst = {
	"arm,vexpress",
	NULL,
};

DT_MACHINE_START(VEXPRESS_DT, "ARM-Versatile Express")
	.dt_compat	= v2m_dt_match,
	.l2c_aux_val	= 0x00400000,
	.l2c_aux_mask	= 0xfe0fffff,
	.smp		= smp_ops(vexpress_smp_dt_ops),
	.smp_init	= smp_init_ops(vexpress_smp_init_ops),
	.init_early	= v2m_dt_init_early,
MACHINE_END

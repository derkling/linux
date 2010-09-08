/*
 * Versatile Express Core Tile Cortex A15x4 Support
 */
#include <linux/init.h>
#include <linux/cpumask.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>

#include <mach/ct-ca15x4.h>

#include <asm/mach/map.h>

#include "core.h"

#include <mach/motherboard.h>

static struct map_desc ct_ca15x4_io_desc[] __initdata = {
	{
		.virtual	= __MMIO_P2V(CT_CA15X4_MPIC),
		.pfn		= __phys_to_pfn(CT_CA15X4_MPIC),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},
};

static void __init ct_ca15x4_map_io(void)
{
	iotable_init(ct_ca15x4_io_desc, ARRAY_SIZE(ct_ca15x4_io_desc));
}

void __iomem *gic_cpu_base_addr;

static void __init ct_ca15x4_init_irq(void)
{
	gic_cpu_base_addr = MMIO_P2V(A15_MPCORE_GIC_CPU);
	gic_dist_init(0, MMIO_P2V(A15_MPCORE_GIC_DIST), 29);
	gic_cpu_init(0, gic_cpu_base_addr);
}

static void ct_ca15x4_init(void) {}

#ifdef CONFIG_SMP
static unsigned int ct_ca15x4_get_core_count(void)
{
	unsigned int ncores;
	asm volatile("mrc p15, 1, %0, c9, c0, 2\n" : "=r" (ncores));
	return ((ncores >> 24) & 3) + 1;
}

static void ct_ca15x4_smp_enable(void)
{
	u32 l2_aux_ctrl;

	/* Ensure MOESI support is enabled at L2. */
	asm volatile("mrc p15, 1, %0, c15, c0, 0\n" : "=r" (l2_aux_ctrl));

	if (l2_aux_ctrl & (1 << 5)) {
		l2_aux_ctrl &= ~(1 << 5);
		asm volatile("mcr p15, 1, %0, c15, c0, 0\n"
			     "isb\n"
			     :: "r" (l2_aux_ctrl));
		flush_cache_all();
	}
}
#endif

struct ct_desc ct_ca15x4_desc = {
	.id		= V2M_CT_ID_CA15,
	.name		= "CA15x4",
	.map_io		= ct_ca15x4_map_io,
	.init_irq	= ct_ca15x4_init_irq,
	.init_tile	= ct_ca15x4_init,
#ifdef CONFIG_SMP
	.get_core_count	= ct_ca15x4_get_core_count,
	.smp_enable	= ct_ca15x4_smp_enable,
#endif
};

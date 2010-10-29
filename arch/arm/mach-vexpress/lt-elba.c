/*
 * Versatile Express Logic Tile ELBA Support
 */

#include <linux/init.h>
#include <linux/cpumask.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/platform_device.h>

#include <asm/cacheflush.h>
#include <asm/clkdev.h>
#include <asm/hardware/gic.h>
#include <asm/smp_twd.h>
#ifdef CONFIG_SMP
#include <asm/smp_scu.h>
#endif
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <mach/clkdev.h>
#include <mach/lt-elba.h>

#include <asm/mach/map.h>

#include <mach/core.h>
#include <mach/motherboard.h>


static struct map_desc lt_elba_io_desc[] __initdata = {
	{
		.virtual	= __MMIO_P2V(LT_ELBA_MPIC),
		.pfn		= __phys_to_pfn(LT_ELBA_MPIC),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= __MMIO_P2V(LT_ELBA_SP804_TIMER),
		.pfn		= __phys_to_pfn(LT_ELBA_SP804_TIMER),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
#ifdef CONFIG_CACHE_L2X0
	{
		.virtual	= __MMIO_P2V(LT_ELBA_L2CC),
		.pfn		= __phys_to_pfn(LT_ELBA_L2CC),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#endif
#ifdef CONFIG_PCI
#ifdef CONFIG_VEXPRESS_PCIE_RC_IN_FPGA
	{
		.virtual	= __MMIO_P2V(VEXPRESS_PCIE_TRN_CTRL_BASE),
		.pfn		= __phys_to_pfn(VEXPRESS_PCIE_TRN_CTRL_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE_UNCACHED,
	},
	{
		.virtual	= VEXPRESS_PCI_DBI_VBASE,
		.pfn		= __phys_to_pfn(VEXPRESS_PCI_DBI_BASE),
		.length		= VEXPRESS_PCI_DBI_SIZE,
		.type		= MT_DEVICE_UNCACHED,
	},
	{
		.virtual	= VEXPRESS_PCI_CFG0_VBASE,
		.pfn		= __phys_to_pfn(VEXPRESS_PCI_CFG0_BASE),
		.length		= VEXPRESS_PCI_CFG0_SIZE,
		.type		= MT_DEVICE_UNCACHED,
	},
	{
		.virtual	= VEXPRESS_PCI_CFG1_VBASE,
		.pfn		= __phys_to_pfn(VEXPRESS_PCI_CFG1_BASE),
		.length		= VEXPRESS_PCI_CFG1_SIZE,
		.type		= MT_DEVICE_UNCACHED,
	},
	{
		/* map IO space statically */
		.virtual	= VEXPRESS_PCI_IO_VBASE,
		.pfn		= __phys_to_pfn(VEXPRESS_PCI_IO_BASE),
		.length		= VEXPRESS_PCI_IO_SIZE,
		.type		= MT_DEVICE_UNCACHED,
	},
#else
	{
		.virtual	= __MMIO_P2V(VEXPRESS_SYSREG_BASE),
		.pfn		= __phys_to_pfn(VEXPRESS_SYSREG_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= VEXPRESS_PCI_VBASE,
		.pfn		= __phys_to_pfn(VEXPRESS_PCI_BASE),
		.length		= VEXPRESS_PCI_SIZE,
		.type		= MT_DEVICE,
	},
#endif
#endif
};

static struct resource mali_hdlcd_resources[] = {
	{
		.start = 0xE0185000,
		.end   = 0xE0185000 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static u64 mali_hdlcd_dmamask = ~(u32)0;
static struct platform_device lt_elba_mali_hdlcd = {
	.name           = "ct:hdlcd",
	.id             = -1,
	.dev = {
		.dma_mask          = &mali_hdlcd_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
	.resource       = mali_hdlcd_resources,
	.num_resources  = ARRAY_SIZE(mali_hdlcd_resources),
};

static void __init lt_elba_map_io(void)
{
#ifdef CONFIG_SMP
	twd_base = MMIO_P2V(ELBA_A9_MPCORE_TWD);
#endif
	iotable_init(lt_elba_io_desc, ARRAY_SIZE(lt_elba_io_desc));
}

static void __init lt_elba_init_irq(void)
{
	gic_init(0, 29, MMIO_P2V(ELBA_A9_MPCORE_GIC_DIST),
		MMIO_P2V(ELBA_A9_MPCORE_GIC_CPU));
}

static struct amba_device *lt_elba_amba_devs[] __initdata = {
};

static void lt_elba_init(void)
{
	int i;

#ifdef CONFIG_CACHE_L2X0
	void __iomem *l2x0_base = MMIO_P2V(LT_ELBA_L2CC);

	/* set RAM latencies to 1 cycle for this core tile. */
	writel(0, l2x0_base + L2X0_TAG_LATENCY_CTRL);
	writel(0, l2x0_base + L2X0_DATA_LATENCY_CTRL);

	l2x0_init(l2x0_base, 0x00400000, 0xfe0fffff);
#endif

	for (i = 0; i < ARRAY_SIZE(lt_elba_amba_devs); i++)
		amba_device_register(lt_elba_amba_devs[i], &iomem_resource);

	platform_device_register(&lt_elba_mali_hdlcd);
}

#ifdef CONFIG_SMP
static unsigned int lt_elba_get_core_count(void)
{
	return scu_get_core_count(MMIO_P2V(ELBA_A9_MPCORE_SCU));
}

static void lt_elba_smp_enable(void)
{
	scu_enable(MMIO_P2V(ELBA_A9_MPCORE_SCU));
}
#endif

struct vexpress_tile_desc lt_elba_desc = {
	.id		= V2M_LT_ID_ELBA,
	.name		= "ELBA",
	.map_io		= lt_elba_map_io,
	.init_irq	= lt_elba_init_irq,
	.init_tile	= lt_elba_init,
#ifdef CONFIG_SMP
	.get_core_count	= lt_elba_get_core_count,
	.smp_enable	= lt_elba_smp_enable,
#endif
};

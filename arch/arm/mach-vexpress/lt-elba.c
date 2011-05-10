/*
 * Versatile Express Logic Tile ELBA Support
 */

#include <linux/init.h>
#include <linux/cpumask.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/amba/serial.h>
#include <linux/platform_device.h>
#include <linux/clkdev.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/smp_twd.h>
#include <asm/pmu.h>
#ifdef CONFIG_SMP
#include <asm/smp_scu.h>
#endif
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#include <asm/hardware/timer-sp.h>

#include <mach/clkdev.h>
#include <mach/lt-elba.h>

#include <asm/mach/map.h>

#include <mach/core.h>
#include <mach/motherboard.h>


#define CLUSTER_ID_MASK		0x00000F00
#define HIP_CLUSTER		(0 << 8)
#define MID_CLUSTER		(1 << 8)


static struct map_desc lt_elba_io_desc[] __initdata = {
	{
		.virtual	= __MMIO_P2V(LT_ELBA_KMI0),
		.pfn		= __phys_to_pfn(LT_ELBA_KMI0),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= __MMIO_P2V(LT_ELBA_UART0),
		.pfn		= __phys_to_pfn(LT_ELBA_UART0),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= __MMIO_P2V(LT_ELBA_SCC),
		.pfn		= __phys_to_pfn(LT_ELBA_SCC),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= __MMIO_P2V(LT_ELBA_MPIC),
		.pfn		= __phys_to_pfn(LT_ELBA_MPIC),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= __MMIO_P2V(LT_ELBA_SP804_TIMER01),
		.pfn		= __phys_to_pfn(LT_ELBA_SP804_TIMER01),
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
#endif
#ifdef CONFIG_PL330_DMA
	{
		.virtual	= __MMIO_P2V(LT_ELBA_DMAC),
		.pfn		= __phys_to_pfn(LT_ELBA_DMAC),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
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
	.name           = "lt:hdlcd",
	.id             = -1,
	.dev = {
		.dma_mask          = &mali_hdlcd_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
	.resource       = mali_hdlcd_resources,
	.num_resources  = ARRAY_SIZE(mali_hdlcd_resources),
};

static struct resource elba_spsc_resources[] = {
	{
		.start = 0xe001b000,
		.end   = 0xe001b000 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static u64 elba_spsc_dmamask = ~(u32)0;
static struct platform_device elba_spsc = {
	.name           = "SCPC",
	.id             = -1,
	.dev = {
		.dma_mask          = &elba_spsc_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
	.resource       = elba_spsc_resources,
	.num_resources  = ARRAY_SIZE(elba_spsc_resources),
};

static void __init lt_elba_map_io(void)
{
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = MMIO_P2V(LT_ELBA_A9_MPCORE_TWD);
#endif
	iotable_init(lt_elba_io_desc, ARRAY_SIZE(lt_elba_io_desc));
}

static void __init lt_elba_init_irq(void)
{
	gic_init(0, 29, MMIO_P2V(LT_ELBA_A9_MPCORE_GIC_DIST),
		MMIO_P2V(LT_ELBA_A9_MPCORE_GIC_CPU));

#if defined(CONFIG_PCI_MSI)
	vexpress_msi_init();
#endif
}

#ifdef CONFIG_PL330_DMA
bool uart_filter_dma(struct dma_chan *chan, void *filter_param)
{
	return true;
}

struct amba_pl011_data uart_dma_data = {
	.dma_filter	= uart_filter_dma,
	.dma_rx_param	= NULL,
	.dma_tx_param	= NULL,
};

const struct amba_pl011_data* uart_data = &uart_dma_data;

#else
#define uart_data	NULL
#endif

static unsigned int lt_elba_mmci_status(struct device *dev)
{
	return readl(MMIO_P2V(V2M_SYS_MCI)) & (1 << 0);
}

static struct mmci_platform_data lt_elba_mmci_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.status		= lt_elba_mmci_status,
};


static AMBA_DEVICE(wdt, "elba:wdt", LT_ELBA_WDT, NULL);
static AMBA_DEVICE(rtc, "elba:rtc", LT_ELBA_RTC, NULL);
static AMBA_DEVICE(uart0, "elba:uart0", LT_ELBA_UART0, NULL);
static AMBA_DEVICE(uart1, "elba:uart1", LT_ELBA_UART1, NULL);
static AMBA_DEVICE(aaci, "elba:aaci", LT_ELBA_AACI, NULL);
static AMBA_DEVICE(kmi0, "elba:kmi0", LT_ELBA_KMI0, NULL);
static AMBA_DEVICE(kmi1, "elba:kmi1", LT_ELBA_KMI1, NULL);
static AMBA_DEVICE(mmci,  "elba:mmci",  V2M_MMCI, &lt_elba_mmci_data);

static struct amba_device *lt_elba_amba_devs[] __initdata = {
	&wdt_device,
	&rtc_device,
	&uart0_device,
	&uart1_device,
	&aaci_device,
	&kmi0_device,
	&kmi1_device,
#ifdef CONFIG_PL330_DMA
//	&dma_device,
#endif
//	&mmci_device,
};

static struct clk osc0_clk = {
	.rate	= 24000000,
};

static struct clk osc1_clk = {
	.rate	= 24000000,
};

static struct clk sp804_clk = {
	.rate	= 10000000,
};

static int v2m_osc5_set(struct clk *clk, unsigned long rate)
{
	/* FPGA implementation needs a doubled rate to be set in the oscillator */
	return v2m_cfg_write(SYS_CFG_OSC | SYS_CFG_SITE_DB1 | 5, 2 * rate);
}

static const struct clk_ops osc5_clk_ops = {
	.set	= v2m_osc5_set,
};

static struct clk osc5_clk = {
	.ops	= &osc5_clk_ops,
	.rate	= 10000000,
};

static int v2m_osc8_set(struct clk *clk, unsigned long rate)
{
	return v2m_cfg_write(SYS_CFG_OSC | SYS_CFG_SITE_DB1 | 8, rate);
}

static const struct clk_ops osc8_clk_ops = {
	.set	= v2m_osc8_set,
};

static struct clk osc8_clk = {
	.ops	= &osc8_clk_ops,
	.rate	= 40000000,
};

static int v2m_osc11_set(struct clk *clk, unsigned long rate)
{
	return v2m_cfg_write(SYS_CFG_OSC | SYS_CFG_SITE_DB1 | 11, rate);
}

static const struct clk_ops osc11_clk_ops = {
	.set	= v2m_osc11_set,
};

static struct clk osc11_clk = {
	.ops	= &osc11_clk_ops,
	.rate	= 119000000,
};

static struct clk_lookup elba_common_clk_lookups[] = {
	{	/* SP804 timer 0 */
		.dev_id		= "sp804",
		.con_id		= "elba-timer-sp-0",
		.clk		= &sp804_clk,
	}, {	/* SP804 timer 1 */
		.dev_id		= "sp804",
		.con_id		= "elba-timer-sp-1",
		.clk		= &sp804_clk,
	}, {	/* SP804 timer 2 */
		.dev_id		= "sp804",
		.con_id		= "elba-timer-sp-2",
		.clk		= &sp804_clk,
	}, {	/* SP804 timer 3 */
		.dev_id		= "sp804",
		.con_id		= "elba-timer-sp-3",
		.clk		= &sp804_clk,
	}, {    /* UART0 */
		.dev_id		= "elba:uart0",
		.clk		= &osc1_clk,
	}, {    /* UART1 */
		.dev_id		= "elba:uart1",
		.clk		= &osc1_clk,
	}, {	/* KMI0 */
		.dev_id		= "elba:kmi0",
		.clk		= &osc0_clk,
	}, {	/* KMI1 */
		.dev_id		= "elba:kmi1",
		.clk		= &osc0_clk,
	}, {	/* MMCI */
		.dev_id		= "elba:mmci",
		.clk		= &osc8_clk,
	},
};

static struct clk_lookup lt_elba_clk_lookups[] = {
	{	/* OSC5 drives HDLCD clock in FPGA */
		.dev_id		= "lt:hdlcd",
		.clk		= &osc5_clk,
	},
};

static struct clk_lookup ct_elba_clk_lookups[] = {
	{	/* OSC11 drives the HDLCD clock in Tuscan/ELBA */
		.dev_id		= "lt:hdlcd",
		.clk		= &osc11_clk,
	},
};

static struct resource pmu_resources[] = {
	[0] = {
		.start	= IRQ_LT_ELBA_PMU_CPU0,
		.end	= IRQ_LT_ELBA_PMU_CPU0,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= IRQ_LT_ELBA_PMU_CPU1,
		.end	= IRQ_LT_ELBA_PMU_CPU1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.num_resources	= ARRAY_SIZE(pmu_resources),
	.resource	= pmu_resources,
};

static void __init lt_elba_init_timers(void)
{
	printk(KERN_INFO "ELBA: SP804 initialising timers\n");
	sp804_clocksource_init(MMIO_P2V(LT_ELBA_TIMER1), "elba-timer-sp-1");
	sp804_clockevents_init(MMIO_P2V(LT_ELBA_TIMER0), IRQ_LT_ELBA_TIMER0, "elba-timer-sp-0");
	sp804_clocksource_init(MMIO_P2V(LT_ELBA_TIMER2), "elba-timer-sp-2");
	sp804_clocksource_init(MMIO_P2V(LT_ELBA_TIMER3), "elba-timer-sp-3");
}

static void elba_init(u32 l2cache_tag_latencies, u32 l2cache_data_latencies)
{
	int i;

#ifdef CONFIG_CACHE_L2X0
	void __iomem *l2x0_base = MMIO_P2V(LT_ELBA_L2CC);

	/* set cache latencies */
	writel(l2cache_tag_latencies, l2x0_base + L2X0_TAG_LATENCY_CTRL);
	writel(l2cache_data_latencies, l2x0_base + L2X0_DATA_LATENCY_CTRL);

	/* prefetch control */
	writel(0x30000007, l2x0_base + L2X0_PREFETCH_CTRL);

	/* power control */
	writel(L2X0_DYNAMIC_CLK_GATING_EN | L2X0_STNDBY_MODE_EN,
		l2x0_base + L2X0_POWER_CTRL);

	/* set the following bits:
		- 30 (early BRESP enable)
		- 29 (instruction prefetch enable)
		- 28 (data prefetch enable)
		- 22 (shared attribute override enable)
		-  0 (full line of write zero enable)
	*/
	l2x0_init(l2x0_base, 0x70400001, 0x8fbffffe);
#endif

	clkdev_add_table(elba_common_clk_lookups, ARRAY_SIZE(elba_common_clk_lookups));
	printk(KERN_INFO "ELBA: SP804 clocks registered\n");

	/* these peripherals seem to be missing the AMBA cid/pid */
	kmi0_device.periphid = 0x00041050;
	kmi1_device.periphid = 0x00041050;

	for (i = 0; i < ARRAY_SIZE(lt_elba_amba_devs); i++)
		amba_device_register(lt_elba_amba_devs[i], &iomem_resource);

	platform_device_register(&lt_elba_mali_hdlcd);
	platform_device_register(&elba_spsc);
	platform_device_register(&pmu_device);
}


static void lt_elba_init(void)
{
	/* HDLCD on FPGA uses OSC 5 */
	clkdev_add_table(lt_elba_clk_lookups, ARRAY_SIZE(lt_elba_clk_lookups));

	/* set RAM latencies to 1 cycle for this logic tile. */
	elba_init(0, 0);
}

#ifdef CONFIG_VEXPRESS_ELBA_ACP
void acp_init(void __iomem *scc_base)
{
#define ACP_MIDPL301 (1 << 8) // 0x100
#define ACP_HIP      (2 << 8) // 0x200
#define ACP_MID      (4 << 8) // 0x400

      	/* Remap ACP to HIP  */
	unsigned long scc_conf = __raw_readl(scc_base);
	scc_conf &= (~0x700);
	scc_conf |= ACP_HIP;
	writel(scc_conf, scc_base);
#if 0
	/* Enable ACP for Mali  */
	writel(0x1, scc_base + 0x170);
	writel(0x1E, scc_base + 0x174);
	writel(0x1, scc_base + 0x178);
	writel(0x1E, scc_base + 0x17C);
#endif
} 
#else
void acp_init(void __iomem *scc_base)
{
}
#endif

static void ct_elba_init(void)
{
	unsigned int cluster_id = 0;

	/* HDLCD in silicon uses OSC 11 */
	clkdev_add_table(ct_elba_clk_lookups, ARRAY_SIZE(ct_elba_clk_lookups));

	asm("mrc p15, 0, %0, c0, c0, 5\n\t" : "+r" (cluster_id));

	cluster_id &= CLUSTER_ID_MASK;

	/* set RAM latencies for this core tile. */
	switch (cluster_id) {
	case HIP_CLUSTER:
		elba_init(0x122, 0x352); /* HIP @ 1.6 GHz */
		break;
	case MID_CLUSTER:
		elba_init(0x120, 0x220); /* MID @ any frequency */
		break;
	}
}

#ifdef CONFIG_SMP
static void lt_elba_init_cpu_map(void)
{
	unsigned int i, ncores = scu_get_core_count(MMIO_P2V(LT_ELBA_A9_MPCORE_SCU));

	for (i = 0; i < ncores; ++i)
		set_cpu_possible(i, true);
}

static void lt_elba_smp_enable(unsigned int max_cpus)
{
	int i = 0;
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	scu_enable(MMIO_P2V(LT_ELBA_A9_MPCORE_SCU));
}
#endif

struct vexpress_tile_desc ct_elba_desc = {
	.id		= V2M_CT_ID_ELBA,
	.name		= "ELBA",
	.map_io		= lt_elba_map_io,
	.init_irq	= lt_elba_init_irq,
	.init_timers	= lt_elba_init_timers,
	.init_tile	= ct_elba_init,
#ifdef CONFIG_SMP
	.init_cpu_map	= lt_elba_init_cpu_map,
	.smp_enable	= lt_elba_smp_enable,
#endif
};

struct vexpress_tile_desc lt_elba_desc = {
	.id		= V2M_LT_ID_ELBA_FPGA,
	.name		= "ELBA FPGA",
	.map_io		= lt_elba_map_io,
	.init_irq	= lt_elba_init_irq,
	.init_timers	= lt_elba_init_timers,
	.init_tile	= lt_elba_init,
#ifdef CONFIG_SMP
	.init_cpu_map	= lt_elba_init_cpu_map,
	.smp_enable	= lt_elba_smp_enable,
#endif
};

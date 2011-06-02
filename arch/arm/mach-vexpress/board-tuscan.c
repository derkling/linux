/*
 * Tuscan motherboard support
 */
#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/ata_platform.h>
#include <linux/smsc911x.h>
#include <linux/spinlock.h>
#include <linux/sysdev.h>
#include <linux/usb/isp1760.h>
#include <linux/clkdev.h>

#include <asm/mach-types.h>
#include <asm/sizes.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/timer-sp.h>
#include <asm/hardware/sp810.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/smp_twd.h>
#include <asm/pmu.h>
#ifdef CONFIG_SMP
#include <asm/smp_scu.h>
#include <asm/smp.h>
#endif
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <mach/motherboard.h>
#include <mach/clkdev.h>
#include <mach/lt-elba.h>
#include <mach/core.h>
#include <mach/irqs.h>


#define CLUSTER_ID_MASK		0x00000F00
#define HIP_CLUSTER		(0 << 8)
#define MID_CLUSTER		(1 << 8)


extern void versatile_secondary_startup(void);

static struct map_desc tuscan_io_desc[] __initdata = {
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

static void __init tuscan_init_early(void)
{
	// TODO: Reserve memory for HD-LCD framebuffer
}

static void __init tuscan_timer_init(void)
{
#if 0
	u32 scctrl;

	/* Select 1MHz TIMCLK as the reference clock for SP804 timers */
	scctrl = readl(MMIO_P2V(V2M_SYSCTL + SCCTRL));
	scctrl |= SCCTRL_TIMEREN0SEL_TIMCLK;
	scctrl |= SCCTRL_TIMEREN1SEL_TIMCLK;
	writel(scctrl, MMIO_P2V(V2M_SYSCTL + SCCTRL));

	writel(0, MMIO_P2V(V2M_TIMER0) + TIMER_CTRL);
	writel(0, MMIO_P2V(V2M_TIMER1) + TIMER_CTRL);
#endif

	sp804_clocksource_init(MMIO_P2V(LT_ELBA_TIMER1), "tuscan-timer-sp-1");
	sp804_clockevents_init(MMIO_P2V(LT_ELBA_TIMER0), IRQ_LT_ELBA_TIMER0, "tuscan-timer-sp-0");
	sp804_clocksource_init(MMIO_P2V(LT_ELBA_TIMER2), "tuscan-timer-sp-2");
	sp804_clocksource_init(MMIO_P2V(LT_ELBA_TIMER3), "tuscan-timer-sp-3");
}

static struct sys_timer tuscan_timer = {
	.init	= tuscan_timer_init,
};


static DEFINE_SPINLOCK(tuscan_cfg_lock);

int tuscan_cfg_write(u32 devfn, u32 data)
{
	/* Configuration interface broken? */
	u32 val = 0;

	devfn |= SYS_CFG_START | SYS_CFG_WRITE;

#if 0
	spin_lock(&tuscan_cfg_lock);
	val = readl(MMIO_P2V(V2M_SYS_CFGSTAT));
	writel(val & ~SYS_CFG_COMPLETE, MMIO_P2V(V2M_SYS_CFGSTAT));
	writel(data, MMIO_P2V(V2M_SYS_CFGDATA));
	writel(devfn, MMIO_P2V(V2M_SYS_CFGCTRL));
	do {
		val = readl(MMIO_P2V(V2M_SYS_CFGSTAT));
	} while (val == 0);
	spin_unlock(&tuscan_cfg_lock);
#endif

	return !!(val & SYS_CFG_ERR);
}
EXPORT_SYMBOL(tuscan_cfg_write);

int tuscan_cfg_read(u32 devfn, u32 *data)
{
	u32 val = 0;

	devfn |= SYS_CFG_START;

#if 0
	spin_lock(&tuscan_cfg_lock);
	writel(0, MMIO_P2V(V2M_SYS_CFGSTAT));
	writel(devfn, MMIO_P2V(V2M_SYS_CFGCTRL));

	mb();

	do {
		cpu_relax();
		val = readl(MMIO_P2V(V2M_SYS_CFGSTAT));
	} while (val == 0);

	*data = readl(MMIO_P2V(V2M_SYS_CFGDATA));
	spin_unlock(&tuscan_cfg_lock);
#endif

	return !!(val & SYS_CFG_ERR);
}

static AMBA_DEVICE(aaci,  "aaci",  LT_ELBA_AACI, NULL);
static AMBA_DEVICE(kmi0,  "kmi0",  LT_ELBA_KMI0, NULL);
static AMBA_DEVICE(kmi1,  "kmi1",  LT_ELBA_KMI1, NULL);
static AMBA_DEVICE(uart0, "uart0", LT_ELBA_UART0, NULL);
static AMBA_DEVICE(rtc0, "rtc0", LT_ELBA_RTC, NULL);

static struct amba_device *tuscan_amba_devs[] __initdata = {
	&aaci_device,
	&kmi0_device,
	&kmi1_device,
	&uart0_device,
	&rtc0_device,
};

static long tuscan_osc_round(struct clk *clk, unsigned long rate)
{
	return rate;
}

static int tuscan_osc_set(struct clk *clk, unsigned long rate)
{
	return 0; //tuscan_cfg_write(V2M_SYS_CFG_OSC | clk->id, rate);
}

static const struct clk_ops osc_clk_ops = {
	.round	= tuscan_osc_round,
	.set	= tuscan_osc_set,
};

#define OSC_CLK(clk_id, clk_rate)	\
static struct clk osc##clk_id##_clk = {	\
	.ops	= &osc_clk_ops,		\
	.rate	= clk_rate,		\
	.id	= clk_id,		\
};

OSC_CLK(0, 25000000);
OSC_CLK(1, 24000000);
OSC_CLK(2, 12000000);
OSC_CLK(5, 10000000);
OSC_CLK(11, 119000000);

static struct clk dummy_apb_pclk;

static struct clk_lookup tuscan_lookups[] = {
	{	/* AMBA bus clock */
		.con_id		= "apb_pclk",
		.clk		= &dummy_apb_pclk,
	}, {	/* UART0 */
		.dev_id		= "uart0",
		.clk		= &osc1_clk,
	},
	{	/* KMI0 */
		.dev_id		= "kmi0",
		.clk		= &osc0_clk,
	}, {	/* KMI1 */
		.dev_id		= "kmi1",
		.clk		= &osc0_clk,
	}, {	/* HDLCD */
		.dev_id		= "tuscan:hdlcd",
		.clk		= &osc11_clk,
	},
};


static struct resource mali_hdlcd_resources[] = {
	{
		.start = 0xE0185000,
		.end   = 0xE0185000 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static u64 mali_hdlcd_dmamask = ~(u32)0;
static struct platform_device tuscan_mali_hdlcd = {
	.name           = "tuscan:hdlcd",
	.id             = -1,
	.dev = {
		.dma_mask          = &mali_hdlcd_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
	.resource       = mali_hdlcd_resources,
	.num_resources  = ARRAY_SIZE(mali_hdlcd_resources),
};


static void tuscan_power_off(void)
{
	if (tuscan_cfg_write(SYS_CFG_SHUTDOWN | SYS_CFG_SITE_MB, 0))
		printk(KERN_EMERG "Unable to shutdown\n");
}

static void tuscan_restart(char str, const char *cmd)
{
	if (tuscan_cfg_write(SYS_CFG_REBOOT | SYS_CFG_SITE_MB, 0))
		printk(KERN_EMERG "Unable to reboot\n");
}

static void __init tuscan_map_io(void)
{
	int i;

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = MMIO_P2V(LT_ELBA_A9_MPCORE_TWD);
#endif
	iotable_init(tuscan_io_desc, ARRAY_SIZE(tuscan_io_desc));
}

static void __init tuscan_init_irq(void)
{
	gic_init(0, 29, MMIO_P2V(LT_ELBA_A9_MPCORE_GIC_DIST),
		MMIO_P2V(LT_ELBA_A9_MPCORE_GIC_CPU));

#if defined(CONFIG_PCI_MSI)
	vexpress_msi_init();
#endif
}

void __init smp_init_cpus(void)
{
	unsigned int i, ncores = scu_get_core_count(MMIO_P2V(LT_ELBA_A9_MPCORE_SCU));

	for (i = 0; i < ncores; ++i)
		set_cpu_possible(i, true);
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	int i = 0;
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	scu_enable(MMIO_P2V(LT_ELBA_A9_MPCORE_SCU));

	/*
	 * Write the address of the secondary startup into the
	 * SCC register 0x30c. The boot monitor waits until
	 * it receives a soft interrupt, and then the secondary
	 * CPU branches to this address
	 */
	writel(BSYM(virt_to_phys(versatile_secondary_startup)),
		MMIO_P2V(0xe000330c));
}

static void __init tuscan_init(void)
{
	int i;
	struct platform_device *eth_dev;
	unsigned int cluster_id = 0;
#ifdef CONFIG_CACHE_L2X0
	u32 l2cache_tag_latencies = 0, l2cache_data_latencies = 0;
	void __iomem *l2x0_base = MMIO_P2V(LT_ELBA_L2CC);

	// find the cluster we are running on
	asm("mrc p15, 0, %0, c0, c0, 5\n\t" : "+r" (cluster_id));

	cluster_id &= CLUSTER_ID_MASK;

	/* set RAM latencies for this core tile. */
	switch (cluster_id) {
	case HIP_CLUSTER: /* HIP @ 1.6 GHz */
		l2cache_tag_latencies = 0x122;
		l2cache_data_latencies = 0x352;
		break;
	case MID_CLUSTER:  /* MID @ any frequency */
		l2cache_tag_latencies = 0x120;
		l2cache_data_latencies = 0x220;
		break;
	}

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

	clkdev_add_table(tuscan_lookups, ARRAY_SIZE(tuscan_lookups));

	/* these peripherals seem to be missing the AMBA cid/pid */
	kmi0_device.periphid = 0x00041050;
	kmi1_device.periphid = 0x00041050;

	printk(KERN_INFO "Tuscan: Registering AMBA devices\n");

	for (i = 0; i < ARRAY_SIZE(tuscan_amba_devs); i++)
		amba_device_register(tuscan_amba_devs[i], &iomem_resource);

	printk(KERN_INFO "Tuscan: finished registering AMBA devices\n");

#if 0
	pm_power_off = tuscan_power_off;
	arm_pm_restart = tuscan_restart;
#endif

	printk(KERN_INFO "Tuscan: registering HDLCD\n");
	platform_device_register(&tuscan_mali_hdlcd);
	printk(KERN_INFO "Tuscan: finised registering HDLCD\n");
#if 0
	platform_device_register(&elba_spsc);
	platform_device_register(&pmu_device);
#endif
}

MACHINE_START(TUSCAN, "ARM-Tuscan")
	.boot_params	= PLAT_PHYS_OFFSET + 0x00000100,
	.map_io		= tuscan_map_io,
	.init_early	= tuscan_init_early,
	.nr_irqs	= V2M_NR_IRQS,
	.init_irq	= tuscan_init_irq,
	.timer		= &tuscan_timer,
	.init_machine	= tuscan_init,
MACHINE_END

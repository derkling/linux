/*
 * Columbus Compute Subsystem
 *
 * Support for the TC2 CoreTile.
 *
 *  Copyright (C) 2011 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/clkdev.h>
#include <linux/amba/mmci.h>
#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/clkdev.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/sp810.h>
#include <asm/hardware/timer-sp.h>
#include <asm/sched_clock.h>

#include <mach/columbus.h>

#ifdef CONFIG_OF
#include <linux/of_platform.h>


static void __init tc2_sysctl_init(void __iomem *base)
{
	u32 scctrl;

	if (WARN_ON(!base))
		return;

	/* Select 1MHz TIMCLK as the reference clock for SP804 timers */
	scctrl = readl(base + SCCTRL);
	scctrl |= SCCTRL_TIMEREN0SEL_TIMCLK;
	scctrl |= SCCTRL_TIMEREN1SEL_TIMCLK;
	writel(scctrl, base + SCCTRL);
}

static void __init tc2_sp804_init(void __iomem *base, unsigned int irq)
{
	if (WARN_ON(!base || irq == NO_IRQ))
		return;

	writel(0, base + TIMER_1_BASE + TIMER_CTRL);
	writel(0, base + TIMER_2_BASE + TIMER_CTRL);

	sp804_clocksource_init(base + TIMER_2_BASE, "tc2-timer1");
	sp804_clockevents_init(base + TIMER_1_BASE, irq, "tc2-timer0");
}

struct of_dev_auxdata columbus_tc2_auxdata_lookup[] __initdata = {
#ifdef CONFIG_DEBUG_LL
	OF_DEV_AUXDATA("arm,primecell",
		DEBUG_LL_UART_PHYS_BASE + DEBUG_LL_UART_OFFSET,
		"1c090000.uart", NULL),
#endif
	{}
};

static struct clk uart_clk = {
	.rate	= 24000000,
};

static struct clk sp804_clk = {
	.rate	= 1000000,
};

static struct clk_lookup columbus_tc2_clk_lookups[] = {
	{	/* PL011 UART0 */
		.dev_id		= "1c090000.uart",
		.clk		= &uart_clk,
	},
	{	/* PL180 MCI */
		.dev_id		= "1c050000.mci",
		.clk		= &uart_clk,
	},
	{	/* SP804 timer */
		.dev_id		= "sp804",
		.con_id		= "tc2-timer0",
		.clk		= &sp804_clk,
	},
	{	/* SP804 timer */
		.dev_id		= "sp804",
		.con_id		= "tc2-timer1",
		.clk		= &sp804_clk,
	},
};

static void __iomem *v2m_sysreg_base;

static u32 notrace tc2_read_sched_clock(void)
{
	if (v2m_sysreg_base)
		return readl(v2m_sysreg_base + COLUMBUS_SYS_FLAGS_24MHZ_OFFSET);

	return 0;
}

static DEFINE_SPINLOCK(v2m_cfg_lock);

int v2m_cfg_write(u32 devfn, u32 data)
{
	/* Configuration interface broken? */
	u32 val;

	devfn |= SYS_CFG_START | SYS_CFG_WRITE;

	spin_lock(&v2m_cfg_lock);
	val = readl(v2m_sysreg_base + V2M_SYS_CFGSTAT);
	writel(val & ~SYS_CFG_COMPLETE, v2m_sysreg_base + V2M_SYS_CFGSTAT);

	writel(data, v2m_sysreg_base +  V2M_SYS_CFGDATA);
	writel(devfn, v2m_sysreg_base + V2M_SYS_CFGCTRL);

	do {
		val = readl(v2m_sysreg_base + V2M_SYS_CFGSTAT);
	} while (val == 0);
	spin_unlock(&v2m_cfg_lock);

	return !!(val & SYS_CFG_ERR);
}

static u32 v2m_dt_hdlcd_clk_devfn;

static long tc2_osc_round(struct clk *clk, unsigned long rate)
{
	return rate;
}

static int tc2_hdlcd_clk_set(struct clk *clk, unsigned long rate)
{
	return v2m_cfg_write(v2m_dt_hdlcd_clk_devfn, rate);
}

static const struct clk_ops v2m_dt_hdlcd_clk_ops = {
	.round	= tc2_osc_round,
	.set	= tc2_hdlcd_clk_set,
};

static struct clk tc2_hdlcd_clk = {
	.ops	= &v2m_dt_hdlcd_clk_ops,
	.rate	= 109000000,
};

static struct clk_lookup v2m_dt_hdlcd_clk_lookup = {
	.dev_id	= "hdlcd",
	.clk	= &tc2_hdlcd_clk,
};

int set_dvi_mode(u8 *msgbuf) {
	return v2m_cfg_write(SYS_CFG_DVIMODE | SYS_CFG_SITE_MB, msgbuf[0]);
}

static void __init v2m_dt_hdlcd_init(void)
{
	struct device_node *node;
	const __be32 *prop;
	phys_addr_t framebuffer, framebuffer_size;
	u32 osc;
	u8 default_mode = 3;	/* SXGA */

	node = of_find_compatible_node(NULL, NULL, "arm,hdlcd");
	if (WARN_ON(!node))
		return;

	prop = of_get_property(node, "framebuffer", NULL);
	if (WARN_ON(!prop))
		return;

	framebuffer = of_read_number(prop, of_n_addr_cells(node));
	prop += of_n_addr_cells(node);
	framebuffer_size = of_read_number(prop, of_n_size_cells(node));

	if (WARN_ON(of_property_read_u32(node, "arm,vexpress-osc", &osc)))
		return;

	if (WARN_ON(memblock_remove(framebuffer, framebuffer_size)))
		return;

	v2m_dt_hdlcd_clk_devfn = SYS_CFG_OSC | osc;
	if (!(readl(v2m_sysreg_base + V2M_SYS_MISC) & SYS_MISC_MASTERSITE)) {
		v2m_cfg_write(SYS_CFG_MUXFPGA | SYS_CFG_SITE_DB1, 1);
		v2m_dt_hdlcd_clk_devfn |= SYS_CFG_SITE_DB1;
	} else {
		v2m_cfg_write(SYS_CFG_MUXFPGA | SYS_CFG_SITE_DB2, 2);
		v2m_dt_hdlcd_clk_devfn |= SYS_CFG_SITE_DB2;
	}
	clkdev_add(&v2m_dt_hdlcd_clk_lookup);

	set_dvi_mode(&default_mode);
};

static void v2m_power_off(void)
{
	if (v2m_cfg_write(SYS_CFG_SHUTDOWN | SYS_CFG_SITE_MB, 0))
		printk(KERN_EMERG "Unable to shutdown\n");
}

static void v2m_restart(char str, const char *cmd)
{
	if (v2m_cfg_write(SYS_CFG_REBOOT | SYS_CFG_SITE_MB, 0))
		printk(KERN_EMERG "Unable to reboot\n");
}

static void __init tc2_init_early(void)
{
	clkdev_add_table(columbus_tc2_clk_lookups,
			ARRAY_SIZE(columbus_tc2_clk_lookups));
	setup_sched_clock(tc2_read_sched_clock, 32, 24000000);
}

static void __init columbus_tc2_init(void)
{
	struct device_node *node = of_find_compatible_node(NULL, NULL, "arm,vexpress-sysreg");
	v2m_sysreg_base = of_iomap(node, 0);
	if (WARN_ON(!v2m_sysreg_base))
		return;

	v2m_dt_hdlcd_init();

	of_platform_populate(NULL, of_default_bus_match_table,
			columbus_tc2_auxdata_lookup, NULL);

	pm_power_off = v2m_power_off;
}

static const char *columbus_tc2_match[] __initconst = {
	"arm,columbus",
	NULL,
};

static void __init tc2_init_timer(void)
{
	struct device_node *node;
	const char *path;
	int err;

	node = of_find_compatible_node(NULL, NULL, "arm,sp810");
	if (!node) {
		printk(KERN_INFO "Columbus: Can't find V2M SYSCTL resource\n");
	} else {
		printk(KERN_INFO "Columbus: initialising V2M SYSCTL\n");
		tc2_sysctl_init(of_iomap(node, 0));
		err = of_property_read_string(of_aliases, "arm,v2m_timer", &path);
		if (!err) {
			node = of_find_node_by_path(path);
			tc2_sp804_init(of_iomap(node, 0), irq_of_parse_and_map(node, 0));
		}
	}

	columbus_init_timer();
}

struct sys_timer tc2_global_timer = {
	.init		= tc2_init_timer,
};

DT_MACHINE_START(COLUMBUS_TC2_DT, "ARM Columbus Compute Subsystem")
	.map_io		= columbus_map_common_io,
	.init_early	= tc2_init_early,
	.init_irq	= columbus_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &tc2_global_timer,
//	.timer		= &columbus_local_timer,
	.init_machine	= columbus_tc2_init,
	.dt_compat	= columbus_tc2_match,
	.restart	= v2m_restart,
MACHINE_END

#endif /* CONFIG_OF */

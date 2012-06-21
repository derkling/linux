/*
 * Columbus Compute Subsystem support
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/amba/bus.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/clockchips.h>

#include <asm/arch_timer.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <asm/page.h>
#include <asm/hardware/gic.h>
#include <asm/smp.h>
#include <asm/smp_twd.h>
#include <asm/mach/time.h>

#include <mach/clkdev.h>
#include <mach/columbus.h>


int clk_enable(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (clk->ops && clk->ops->set)
		clk->ops->set(clk, rate);
	clk->rate = rate;
	return 0;
}
EXPORT_SYMBOL(clk_set_rate);

static struct map_desc columbus_io_desc[] __initdata = {
#if defined(CONFIG_COLUMBUS_MODEL) || defined(CONFIG_COLUMBUS_TC2)
	{
		.virtual	= DEBUG_LL_UART_VIRT_BASE + DEBUG_LL_UART_OFFSET,
		.pfn		= __phys_to_pfn(DEBUG_LL_UART_PHYS_BASE + DEBUG_LL_UART_OFFSET),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#if 1
	{
		.virtual	= COLUMBUS_CCI400_VIRT_BASE + COLUMBUS_CCI400_EAG_OFFSET,
		.pfn		= __phys_to_pfn(COLUMBUS_CCI400_PHYS_BASE + COLUMBUS_CCI400_EAG_OFFSET),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
#endif
#endif
	{
		.virtual	= COLUMBUS_MMCI_VIRT_BASE,
		.pfn		= __phys_to_pfn(COLUMBUS_MMCI_PHYS_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},
};

void __init columbus_map_common_io(void)
{
	iotable_init(columbus_io_desc, ARRAY_SIZE(columbus_io_desc));
#ifdef CONFIG_SMP
	columbus_smp_map_io();
#endif
}

static struct of_device_id columbus_irq_match[] __initdata = {
	{
		.compatible	= "arm,cortex-a15-gic",
		.data		= gic_of_init,
	},
	{}
};

void __init columbus_init_irq(void)
{
	of_irq_init(columbus_irq_match);
}

void __init columbus_init_timer(void)
{
	int err = arch_timer_of_register();

#if 0
	if (!err)
		err = arch_timer_sched_clock_init();
#endif

	if (err)
		printk(KERN_INFO "Columbus: Failed to register architected timers. err=%d\n", err);
}

struct sys_timer columbus_local_timer = {
	.init		= columbus_init_timer,
};

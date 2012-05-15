/*
 * Columbus Compute Subsystem
 *
 * Support for the system as modeled by RTSM
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

#include <asm/clkdev.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach/time.h>

#include <mach/columbus.h>

#ifdef CONFIG_OF
#include <linux/of_platform.h>

static unsigned int rtsm_mmci_status(struct device *dev)
{
	return 1;
}

static struct mmci_platform_data rtsm_mmci_data = {
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status		= rtsm_mmci_status,
	.capabilities	= MMC_CAP_NONREMOVABLE,
};

struct of_dev_auxdata columbus_rtsm_auxdata_lookup[] __initdata = {
#ifdef CONFIG_DEBUG_LL
	OF_DEV_AUXDATA("arm,primecell",
		DEBUG_LL_UART_PHYS_BASE + DEBUG_LL_UART_OFFSET,
		"1c090000.uart", NULL),
	OF_DEV_AUXDATA("arm,primecell",	0x1c050000, "1c050000.mci",
	&rtsm_mmci_data),
#endif
	{}
};

static struct clk uart_clk = {
	.rate	= 24000000,
};

static struct clk sp804_clk = {
	.rate	= 100000000,
};

static struct clk_lookup columbus_rtsm_clk_lookups[] = {
	{	/* PL011 UART0 */
		.dev_id		= "1c090000.uart",
		.clk		= &uart_clk,
	},
	{	/* PL180 MCI */
		.dev_id		= "1c050000.mci",
		.clk		= &uart_clk,
	},
	{	/* SP804 timer */
		.dev_id		= "1c110000.timer",
		.clk		= &sp804_clk,
		.con_id		= "globaltimer",
	},
};

static void __init columbus_rtsm_init(void)
{
	clkdev_add_table(columbus_rtsm_clk_lookups,
			ARRAY_SIZE(columbus_rtsm_clk_lookups));
	of_platform_populate(NULL, of_default_bus_match_table,
			columbus_rtsm_auxdata_lookup, NULL);
}

static const char *columbus_rtsm_match[] __initconst = {
	"arm,columbus",
	NULL,
};

DT_MACHINE_START(COLUMBUS_MODEL_DT, "ARM Columbus Compute Subsystem")
	.map_io		= columbus_map_common_io,
	.init_irq	= columbus_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &columbus_local_timer,
	.init_machine	= columbus_rtsm_init,
	.dt_compat	= columbus_rtsm_match,
//	.nr_irqs	= 256,
MACHINE_END

#endif  /* CONFIG_OF */

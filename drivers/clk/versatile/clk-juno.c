/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2012 ARM Limited
 */

#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>

static const __initconst struct of_device_id juno_clk_match[] = {
	{ .compatible = "fixed-clock", .data = of_fixed_clk_setup, },
	{}
};

int __init juno_clk_of_init(void)
{
	pr_info("Juno: initialising fixed clocks\n");
	of_clk_init(juno_clk_match);
	return 0;
}

arch_initcall(juno_clk_of_init);


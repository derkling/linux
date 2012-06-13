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

#include <linux/init.h>
#include <asm/bL_switcher.h>
#include <asm/bL_entry.h>
#include <asm/io.h>
#include <asm/smp_plat.h>

#include <mach/columbus.h>

extern int _smc_up(unsigned int, unsigned int, unsigned int, unsigned int);
extern int _smc_down(unsigned int, unsigned int, unsigned int, unsigned int);

static inline void smc_up(unsigned int cpu, unsigned int cluster)
{
	unsigned int value;
	_smc_up(0, cpu_logical_map(cpu), 0, 0);
}

static inline bool smc_down(unsigned int cpu, unsigned int cluster)
{
	unsigned int value = 0x1 << 16;
	_smc_down(0, value, 0, 0);
	return 1;
}

static const struct bL_power_ops bL_columbus_power_ops = {
	.power_up = smc_up,
	.power_down = smc_down,
};

static int __init columbus_pmif_init(void)
{
	writel(~0, COLUMBUS_SYS_FLAGS_VIRT_BASE +
				COLUMBUS_SYS_FLAGS_CLR_OFFSET);
	writel(virt_to_phys(bl_entry_point), COLUMBUS_SYS_FLAGS_VIRT_BASE +
				COLUMBUS_SYS_FLAGS_SET_OFFSET);
	return bL_switcher_init(&bL_columbus_power_ops);
}
__initcall(columbus_pmif_init);

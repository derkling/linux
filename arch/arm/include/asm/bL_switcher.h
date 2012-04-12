/*
 * arch/arm/include/asm/bL_switcher.h
 *
 * Created by:  Nicolas Pitre, April 2012
 * Copyright:   (C) 2012  Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ASM_BL_SWITCHER_H
#define ASM_BL_SWITCHER_H

struct bL_power_ops {
	void (*power_up)(unsigned int cpu, unsigned int cluster);
	bool (*power_down)(unsigned int cpu, unsigned int cluster);
};

int __init bL_switcher_init(const struct bL_power_ops *ops);

int bL_switch_to(unsigned int new_cluster_id);

#endif

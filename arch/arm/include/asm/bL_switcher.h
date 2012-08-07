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

#include <asm/bL_entry.h>

struct bL_power_ops {
	void (*power_up)(unsigned int cpu, unsigned int cluster,
			phys_addr_t entry_point);
	bool (*power_up_finish)(unsigned int cpu, unsigned int cluster);
	void (*power_down)(unsigned int cpu, unsigned int cluster);
	void (*power_up_setup)(void);
};

struct bL_sync_flags {
};

extern const struct bL_power_ops *bL_platform_ops;

int __init bL_switcher_init(const struct bL_power_ops *ops);
int __init bL_switcher_reserve(void);
void __init __bL_set_cpus_per_cluster(int cluster, int num);
void __bL_cpu_going_down(unsigned int cpu, unsigned int cluster);
void __bL_cpu_down(unsigned int cpu, unsigned int cluster);
void __bL_outbound_leave_critical(unsigned int cluster, int state);
bool __bL_outbound_enter_critical(unsigned int this_cpu, unsigned int cluster);
void __bL_set_first_man(int cpu, unsigned int cluster);

u32 read_mpidr(void);
int bL_switch_to(unsigned int new_cluster_id);
void bL_switch_request(unsigned int cpu, unsigned int new_cluster_id);
#endif

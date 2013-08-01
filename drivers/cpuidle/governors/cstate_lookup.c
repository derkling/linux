/*
 * state_lookup.c - Select a C-state satisfying requirements
 *
 * Copyright 2013 Linaro Limited
 * Author:
 *        Tuukka Tikkanen <tuukka.tikkanen@linaro.org>
 *
 * Based on code extracted from drivers/cpuidle/governors/menu.c
 *
 * Original menu.c copyright information:
 * Copyright (C) 2006-2007 Adam Belay <abelay@novell.com>
 * Copyright (C) 2009 Intel Corporation
 * Author:
 *        Arjan van de Ven <arjan@linux.intel.com>
 *
 * This code is licenced under the GPL version 2 as described
 * in the COPYING file that acompanies the Linux Kernel.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/module.h>


/**
 * cpuidle_cstate_lookup - selects a C-state satisfying criteria
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 * @next_timer_us: Time until next timer expiry
 * @predicted_us: Predicted time until next CPU-activating event
 * @latency_req: Maximum C-state exit latency
 * @aux_data: Optional pointer to structure to be filled with state info
 *
 * This function returns C-state index number for a C-state that satisfies
 * criteria "state enabled and state exit latency <= latency_req and state
 * target residency <= predicted_us". If no such state is available,
 * the return value is CPUIDLE_DRIVER_STATE_START if next_timer_us > 5 or
 * 0 if next_timer_us <= 5.
 *
 * In addition, if aux_data is not NULL, the structure pointed to by
 * aux_data is populated with additional information. The additional
 * information contains the exit latency of the selected state.
 */
int cpuidle_cstate_lookup(struct cpuidle_driver *drv,
		struct cpuidle_device *dev, unsigned int next_timer_us,
		unsigned int predicted_us, unsigned int latency_req,
		struct cpuidle_cstate_sel_aux *aux_data)
{
	int i;
	int state_idx;
	unsigned int exit_us;

	state_idx = 0;
	exit_us = 0;

	/*
	 * We want to default to C1 (hlt), not to busy polling
	 * unless the timer is happening really really soon.
	 */
	if (next_timer_us > 5 &&
	    !drv->states[CPUIDLE_DRIVER_STATE_START].disabled &&
		dev->states_usage[CPUIDLE_DRIVER_STATE_START].disable == 0) {
		state_idx = CPUIDLE_DRIVER_STATE_START;
	}

	/*
	 * Find the idle state with the lowest power while satisfying
	 * our constraints. The states are assumed to be in increasing
	 * order of efficiency.
	 */
	for (i = CPUIDLE_DRIVER_STATE_START; i < drv->state_count; i++) {
		struct cpuidle_state *s = &drv->states[i];
		struct cpuidle_state_usage *su = &dev->states_usage[i];

		if (s->disabled || su->disable)
			continue;
		if (s->exit_latency > latency_req)
			continue;
		if (s->target_residency > predicted_us)
			continue;

		state_idx = i;
		exit_us = s->exit_latency;
	}

	if (aux_data)
		aux_data->exit_us = exit_us;

	return state_idx;
}

MODULE_LICENSE("GPL");

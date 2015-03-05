/*
 * DT idle states parsing code.
 *
 * Copyright (C) 2014 ARM Ltd.
 * Author: Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "DT idle-states: " fmt

#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "dt_idle_states.h"

static int init_state_node(struct cpuidle_state *idle_state,
			   const struct of_device_id *matches,
			   struct device_node *state_node)
{
	int err;
	const struct of_device_id *match_id;
	const char *desc;

	match_id = of_match_node(matches, state_node);
	if (!match_id)
		return -ENODEV;
	/*
	 * CPUidle drivers are expected to initialize the const void *data
	 * pointer of the passed in struct of_device_id array to the idle
	 * state enter function.
	 */
	idle_state->enter = match_id->data;

	err = of_property_read_u32(state_node, "wakeup-latency-us",
				   &idle_state->exit_latency);
	if (err) {
		u32 entry_latency, exit_latency;

		err = of_property_read_u32(state_node, "entry-latency-us",
					   &entry_latency);
		if (err) {
			pr_debug(" * %s missing entry-latency-us property\n",
				 state_node->full_name);
			return -EINVAL;
		}

		err = of_property_read_u32(state_node, "exit-latency-us",
					   &exit_latency);
		if (err) {
			pr_debug(" * %s missing exit-latency-us property\n",
				 state_node->full_name);
			return -EINVAL;
		}
		/*
		 * If wakeup-latency-us is missing, default to entry+exit
		 * latencies as defined in idle states bindings
		 */
		idle_state->exit_latency = entry_latency + exit_latency;
	}

	err = of_property_read_u32(state_node, "min-residency-us",
				   &idle_state->target_residency);
	if (err) {
		pr_debug(" * %s missing min-residency-us property\n",
			     state_node->full_name);
		return -EINVAL;
	}

	err = of_property_read_string(state_node, "idle-state-name", &desc);
	if (err)
		desc = state_node->name;

	idle_state->flags = 0;
	if (of_property_read_bool(state_node, "local-timer-stop"))
		idle_state->flags |= CPUIDLE_FLAG_TIMER_STOP;
	/*
	 * TODO:
	 *	replace with kstrdup and pointer assignment when name
	 *	and desc become string pointers
	 */
	strncpy(idle_state->name, state_node->name, CPUIDLE_NAME_LEN - 1);
	strncpy(idle_state->desc, desc, CPUIDLE_DESC_LEN - 1);
	return 0;
}

/**
 * dt_vet_idle_state_mask() - Vet cpumask for a specific idle state
 *
 * @state_node: device node of the idle state to be vetted
 * @idx: idle state index in cpu-idle-states phandle
 * @possible_mask: cpumask containing cpus to be vetted
 *
 * Function that vets and updates the possible_mask by checking if a
 * specific idle state is valid on all cpus in the possible_mask. If an idle
 * state for a specific cpu in the possible_mask is either missing or
 * different from the state_node, the corresponding cpu is cleared from
 * the possible_mask since this means that the cpu has different idle
 * states from the first cpu in the possible_mask.
 */
static void dt_vet_idle_state_mask(struct device_node *state_node,
				   unsigned int idx,
				   cpumask_t *possible_mask)
{
	int cpu;
	struct device_node *cpu_node, *curr_state_node;

	/*
	 * Compare idle state phandles for index idx on all CPUs in the
	 * possible_mask. Start from next logical cpu following the first
	 * cpu since that's the CPU state_node was retrieved from. If a
	 * mismatch is found the mismatching cpu is removed from the
	 * possible_mask in that it has an idle state that is not present
	 * in the idle states list of the cpu we are vetting the affinity
	 * for.
	 */
	for (cpu = cpumask_next(cpumask_first(possible_mask), possible_mask);
	     cpu < nr_cpu_ids; cpu = cpumask_next(cpu, possible_mask)) {
		cpu_node = of_cpu_device_node_get(cpu);
		curr_state_node = of_parse_phandle(cpu_node, "cpu-idle-states",
						   idx);
		if (state_node != curr_state_node)
			cpumask_clear_cpu(cpu, possible_mask);

		of_node_put(curr_state_node);
		of_node_put(cpu_node);
	}

}

/**
 * dt_probe_idle_affinity() - Parse the DT idle states and set the
 *                            idle states affinity mask
 * @possible_mask: Pointer to the affinity mask to be probed
 *
 * Function probes validity of the possible_mask by comparing
 * the idle states for all cpus in the possible_mask to the idle
 * states of the first cpu in the possible_mask. If idle states for a
 * cpu in the possible_mask differ from the ones of the first cpu, the
 * cpu in question is cleared in the possible_mask.
 */
void dt_probe_idle_affinity(cpumask_t *possible_mask)
{
	struct device_node *state_node, *cpu_node;
	int i;

	cpu_node = of_cpu_device_node_get(cpumask_first(possible_mask));
	for (i = 0; ; i++) {
		state_node = of_parse_phandle(cpu_node, "cpu-idle-states", i);
		/*
		 * cpus in the possible_mask can have more idle states than
		 * the one we are checking against so even if
		 * state_node == NULL we have to keep parsing so that we can
		 * find a mismatch with other cpus in the mask and clear them
		 * since they have different idle states.
		 */
		dt_vet_idle_state_mask(state_node, i, possible_mask);

		if (!state_node)
			break;

		of_node_put(state_node);
	}

	of_node_put(cpu_node);
}
EXPORT_SYMBOL_GPL(dt_probe_idle_affinity);

/**
 * dt_init_idle_driver() - Parse the DT idle states and initialize the
 *			   idle driver states array
 * @drv:	  Pointer to CPU idle driver to be initialized
 * @matches:	  Array of of_device_id match structures to search in for
 *		  compatible idle state nodes. The data pointer for each valid
 *		  struct of_device_id entry in the matches array must point to
 *		  a function with the following signature, that corresponds to
 *		  the CPUidle state enter function signature:
 *
 *		  int (*)(struct cpuidle_device *dev,
 *			  struct cpuidle_driver *drv,
 *			  int index);
 *
 * @start_idx:    First idle state index to be initialized
 *
 * If DT idle states are detected and are valid the state count and states
 * array entries in the cpuidle driver are initialized accordingly starting
 * from index start_idx.
 *
 * Return: number of valid DT idle states parsed, <0 on failure
 */
int dt_init_idle_driver(struct cpuidle_driver *drv,
			const struct of_device_id *matches,
			unsigned int start_idx)
{
	struct cpuidle_state *idle_state;
	struct device_node *state_node, *cpu_node;
	int i, err = 0;
	unsigned int state_idx = start_idx;

	if (state_idx >= CPUIDLE_STATE_MAX)
		return -EINVAL;

	if (!drv->cpumask)
		return -EINVAL;
	/*
	 * We get the idle states for the first logical cpu in the
	 * driver mask. The driver mask must have been previously vetted
	 * through dt_probe_idle_affinity to make sure all of the
	 * cpus in the driver cpumask have common idle states.
	 */
	cpu_node = of_cpu_device_node_get(cpumask_first(drv->cpumask));
	for (i = 0; ; i++) {
		state_node = of_parse_phandle(cpu_node, "cpu-idle-states", i);
		if (!state_node)
			break;

		if (!of_device_is_available(state_node))
			continue;

		if (state_idx == CPUIDLE_STATE_MAX) {
			pr_warn("State index reached static CPU idle driver states array size\n");
			break;
		}

		idle_state = &drv->states[state_idx++];
		err = init_state_node(idle_state, matches, state_node);
		if (err) {
			pr_err("Parsing idle state node %s failed with err %d\n",
			       state_node->full_name, err);
			err = -EINVAL;
			break;
		}
		of_node_put(state_node);
	}

	of_node_put(state_node);
	of_node_put(cpu_node);
	if (err)
		return err;
	/*
	 * Update the driver state count only if some valid DT idle states
	 * were detected
	 */
	if (i)
		drv->state_count = state_idx;

	/*
	 * Return the number of present and valid DT idle states, which can
	 * also be 0 on platforms with missing DT idle states or legacy DT
	 * configuration predating the DT idle states bindings.
	 */
	return i;
}
EXPORT_SYMBOL_GPL(dt_init_idle_driver);

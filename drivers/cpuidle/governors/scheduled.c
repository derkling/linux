/*
 * scheduled.c - A governor that selects idle state based on external input
 *
 * Copyright 2013 Linaro Limited
 * Author:
 *        Tuukka Tikkanen <tuukka.tikkanen@linaro.org>
 *
 * This code is licenced under the GPL version 2 as described
 * in the COPYING file that acompanies the Linux Kernel.
 */

#include <linux/kernel.h>
#include <linux/cpuidle_scheduled.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos.h>
#include <linux/module.h>
#include <linux/sched.h>

/**
 * scheduled_select - selects the next idle state to enter
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 */
static int scheduled_select(struct cpuidle_driver *drv,
				struct cpuidle_device *dev)
{
	int state;
	struct sched_pm *pmdata = this_cpu_ptr(&sched_stat);

	state = cpuidle_cstate_lookup(drv, dev,
			pmdata->idle_time_until_timer,
			pmdata->idle_length_estimate,
			pmdata->idle_max_latency,
			NULL);
	pmdata->idle_current_state = &drv->states[state];

	return state;
}

/**
 * scheduled_reflect - records the actual idle period length
 * @dev: the CPU
 * @index: the index of actual entered state
 */
static void scheduled_reflect(struct cpuidle_device *dev, int index)
{
	unsigned int last_idle_us;
	struct sched_pm *pmdata = this_cpu_ptr(&sched_stat);
	unsigned int timer_limit = pmdata->idle_time_until_timer;
	struct cpuidle_state *state = pmdata->idle_current_state;

	if (unlikely(!(state->flags & CPUIDLE_FLAG_TIME_VALID))) {
		last_idle_us = timer_limit;
	} else {
		last_idle_us = cpuidle_get_last_residency(dev);
		if (last_idle_us > state->exit_latency)
			last_idle_us -= state->exit_latency;
		if (last_idle_us > timer_limit)
			last_idle_us = timer_limit;
	}

	cpuidle_scheduled_result(state, last_idle_us);

	pmdata->idle_current_state = NULL;
}


/**
 * scheduled_enable_device - reset per cpu variables after hotplug
 * @drv: cpuidle driver
 * @dev: the CPU
 */
static int scheduled_enable_device(struct cpuidle_driver *drv,
				struct cpuidle_device *dev)
{
	struct sched_pm *pmdata = &per_cpu(sched_stat, dev->cpu);

	pmdata->idle_time_until_timer = UINT_MAX;
	pmdata->idle_length_estimate = UINT_MAX;
	pmdata->idle_max_latency = INT_MAX;
	pmdata->idle_current_state = NULL;

	return 0;
}

static struct cpuidle_governor scheduled_governor = {
	.name =		"scheduled",
	.rating =	100,
	.enable =	scheduled_enable_device,
	.select =	scheduled_select,
	.reflect =	scheduled_reflect,
	.owner =	THIS_MODULE,
};

/**
 * init_scheduled_idle_gov - initializes the governor
 */
static int __init init_scheduled_idle_gov(void)
{
	return cpuidle_register_governor(&scheduled_governor);
}

/**
 * exit_scheduled_idle_gov - exits the governor
 */
static void __exit exit_scheduled_idle_gov(void)
{
	cpuidle_unregister_governor(&scheduled_governor);
}

MODULE_LICENSE("GPL");
module_init(init_scheduled_idle_gov);
module_exit(exit_scheduled_idle_gov);

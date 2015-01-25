/*
 * A gpu cooling device that uses the hacky mali interface for legacy IPA
 *
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "GPU cooling: " fmt

#include <linux/err.h>
#include <linux/gpu_cooling.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "mali.h"

static struct gpu_cooling {
	unsigned long cur_state;
	unsigned int cur_freq;
} static_gpu_cooling;

/*
 * This is the hacky way of using notifiers
 * thermal_framework_mali_dvfs_requested() defined to void since we
 * track the current frequency and not the requested.  Swap this
 * definitions if you want to track requested frequency instead.
 */
void thermal_framework_mali_dvfs_requested(unsigned int freq)
{
}

void thermal_framework_mali_dvfs_current(unsigned int freq)
{
	static_gpu_cooling.cur_freq = freq;
}

/**
 * mali_get_cur_state - callback function to get the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the current cooling state.
 *
 * Return: 0 on success, this function can't fail.
 */
static int mali_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct gpu_cooling *gpu_cooling = cdev->devdata;

	*state = gpu_cooling->cur_state;
	return 0;
}

/**
 * mali_get_max_state - callback function to get the max cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the max cooling state.
 *
 * Return: 0 on success, this function can't fail.
 */
static int mali_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = get_ipa_dvfs_num_opps() - 1;
	return 0;
}

/**
 * mali_set_cur_state - callback function to set the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: set this variable to the current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int mali_set_cur_state(struct thermal_cooling_device *cdev,
			unsigned long state)
{
	int ret, level, gpu_freq;
	struct gpu_cooling *gpu_cooling = cdev->devdata;
	int max_level = get_ipa_dvfs_num_opps();

	level = max_level - state - 1;
	gpu_freq = gpu_ipa_dvfs_level_to_freq(level);
	ret = gpu_ipa_dvfs_max_lock(gpu_freq);
	if (ret)
		panic("Failure clamping cpu freq to %d: %d\n", gpu_freq, ret);

	gpu_cooling->cur_state = state;

	trace_printk("gpu_out_freq: frequency=%d\n",
		gpu_freq);

	return ret;
}

/**
 * mali_get_requested_power - get the current power
 * @cdev:	cooling device pointer
 * @tz:		a valid thermal zone device parameter
 * @power:	pointer in which to store the calculated power
 *
 * Callback for the power actor to return the current power consumption in
 * milliwatts.
 *
 * Return: 0 on success.
 */
static int mali_get_requested_power(struct thermal_cooling_device *cdev,
				struct thermal_zone_device *tz, u32 *power)
{
	struct mali_debug_utilisation_stats mali_stats;
	int gpu_load;
	struct gpu_cooling *gpu_cooling = cdev->devdata;

	gpu_ipa_dvfs_get_utilisation_stats(&mali_stats);
	gpu_load = mali_stats.s.utilisation;
	*power = kbase_platform_dvfs_freq_to_power(gpu_cooling->cur_freq);

	trace_printk("thermal_gpu_power_actor_get: frequency=%u load=%d\n",
		gpu_cooling->cur_freq, gpu_load);

	*power = (*power * gpu_load) / 100;
	return 0;
}

static int mali_state2power(struct thermal_cooling_device *cdev,
			struct thermal_zone_device *tz, unsigned long state,
			u32 *power)
{
	int max_level, level, gpu_freq;

	max_level = get_ipa_dvfs_num_opps();
	level = max_level - state - 1;
	gpu_freq = gpu_ipa_dvfs_level_to_freq(level);

	*power = kbase_platform_dvfs_freq_to_power(gpu_freq);
	return 0;
}

static int mali_power2state(struct thermal_cooling_device *cdev,
			struct thermal_zone_device *tz, u32 power,
			unsigned long *state)
{
	int max_level, level;
	int freq = kbase_platform_dvfs_power_to_freq(power);

	level = gpu_ipa_dvfs_freq_to_level(freq);
	max_level = get_ipa_dvfs_num_opps();

	trace_printk("thermal_gpu_power_actor_set: power=%u frequency=%d\n",
		power, freq);

	*state = max_level - level - 1;
	return 0;
}

static struct thermal_cooling_device_ops const gpu_cooling_ops = {
	.get_cur_state = mali_get_cur_state,
	.get_max_state = mali_get_max_state,
	.set_cur_state = mali_set_cur_state,
	.get_requested_power = mali_get_requested_power,
	.state2power = mali_state2power,
	.power2state = mali_power2state,
};

/**
 * gpu_cooling_register() - Register the mali gpu cooling device
 */
struct thermal_cooling_device *gpu_cooling_register(void)
{
	struct thermal_cooling_device *cdev;

	static_gpu_cooling.cur_state = 0;

	cdev = thermal_cooling_device_register("gpu-cooling", NULL,
					&gpu_cooling_ops);
	if (IS_ERR(cdev))
		panic("Failed to register gpu cooling device\n");

	cdev->devdata = &static_gpu_cooling;

	return cdev;
}

/**
 * gpu_cooling_unregister() - Unregister the mali gpu cooling device
 *
 * There's nothing to unregister, this function is provided for completeness
 */
void gpu_cooling_unregister(struct thermal_cooling_device *cdev)
{
}

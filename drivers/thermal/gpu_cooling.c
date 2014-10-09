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

struct gpu_cooling {
	unsigned long cur_state;
};

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
	return ret;
}

static struct thermal_cooling_device_ops const gpu_cooling_ops = {
	.get_cur_state = mali_get_cur_state,
	.get_max_state = mali_get_max_state,
	.set_cur_state = mali_set_cur_state,
};

/**
 * gpu_cooling_register() - Register the mali gpu cooling device
 */
struct thermal_cooling_device *gpu_cooling_register(void)
{
	struct gpu_cooling *gpu_cooling;
	struct thermal_cooling_device *cdev;

	gpu_cooling = kzalloc(sizeof(*gpu_cooling), GFP_KERNEL);
	if (!gpu_cooling)
		panic("Failed to allocate memory for gpu_cooling data");
	gpu_cooling->cur_state = 0;

	cdev = thermal_cooling_device_register("gpu-cooling", NULL,
					&gpu_cooling_ops);
	if (IS_ERR(cdev))
		panic("Failed to register gpu cooling device\n");

	cdev->devdata = gpu_cooling;

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

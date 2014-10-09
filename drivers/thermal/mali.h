/*
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MALI_H__
#define __MALI_H__

struct mali_utilisation_stats {
	int utilisation;
	int norm_utilisation;
	int freq_for_norm;
};

struct mali_debug_utilisation_stats {
	struct mali_utilisation_stats s;
	u32 time_busy;
	u32 time_idle;
	int time_tick;
};

void gpu_ipa_dvfs_get_utilisation_stats(struct mali_debug_utilisation_stats *stats);
int gpu_ipa_dvfs_max_lock(int clock);
int kbase_platform_dvfs_freq_to_power(int freq);
int kbase_platform_dvfs_power_to_freq(int power);
int gpu_ipa_dvfs_level_to_freq(int level);
int get_ipa_dvfs_num_opps(void);
int get_ipa_dvfs_max_freq(void);

#endif /* __MALI_H__ */

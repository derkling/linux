/*
 *  ARM Intelligent Power Allocation
 *
 *  Copyright (C) 2013 ARM Limited.
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
 */

#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <mach/cpufreq.h>

#include <trace/events/power.h>

#include "cpu_load_metric.h"
#include "../thermal/mali.h"

struct cpu_load
{
	unsigned int frequency;
	unsigned int load;
	u64 last_update;
};
static DEFINE_PER_CPU(struct cpu_load, cpuload);

void update_cpu_metric(int cpu, u64 now, u64 delta_idle, u64 delta_time,
		       struct cpufreq_policy *policy)
{
	struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);
	unsigned int load;

	/*
	 * Calculate the active time in the previous time window
	 *
	 * load = active time / total_time * 100
	 */
	if (delta_time <= delta_idle)
		load = 0;
	else
		load = div64_u64((100 * (delta_time - delta_idle)), delta_time);

	pcpuload->load = load;
	pcpuload->frequency = policy->cur;
	pcpuload->last_update = now;
#ifdef CONFIG_CPU_THERMAL_IPA_DEBUG
	trace_printk("cpu_load: cpu: %d freq: %u load: %u\n", cpu, policy->cur, load);
#endif
}

void cpu_load_metric_get(int *load, int *freq)
{
	int _load = 0, _freq = 0;
	int cpu;

	for_each_online_cpu(cpu) {
		struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);

		_load += pcpuload->load;
		_freq = pcpuload->frequency;
	}

	*load = _load;
	*freq = _freq;
}

static void get_cluster_stat(struct cluster_stats *cl)
{
	int util = 0, freq = 0;
	int cpu, i = 0;

	for_each_cpu(cpu, cl->mask) {
		struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);

		util += pcpuload->load;
		cl->utils[i++] = pcpuload->load;
		freq = pcpuload->frequency;
	}

	cl->util = util;
	cl->freq = freq;
}

void get_cluster_stats(struct cluster_stats *clstats)
{
	get_cluster_stat(&clstats[0]);
	get_cluster_stat(&clstats[1]);
}

void trace_cpu_gpu_stats(void)
{
	int a7_util, a15_util, gpu_util, a7_freq, a15_freq, gpu_freq;
	struct cluster_stats cluster_stats[2];
	struct mali_debug_utilisation_stats mali_stats;

	setup_cpusmasks(cluster_stats);
	get_cluster_stats(cluster_stats);
	a7_util = cluster_stats[CA7].util;
	a15_util = cluster_stats[CA15].util;
	a7_freq = cluster_stats[CA7].freq;
	a15_freq = cluster_stats[CA15].freq;

	gpu_ipa_dvfs_get_utilisation_stats(&mali_stats);
	gpu_util = mali_stats.s.norm_utilisation;
	gpu_freq = mali_stats.s.freq_for_norm * 1000;

	trace_printk("cpu_gpu_stats: a7_util=%d a15_util=%d gpu_util=%d "
		"a7_freq=%d a15_freq=%d gpu_freq=%d\n", a7_util, a15_util,
		gpu_util, a7_freq, a15_freq, gpu_freq);
}

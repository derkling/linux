/*
 * Columbus CPUFreq support
 * Based on mach-integrator
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Sudeep KarkadaNagesha <sudeep.karkadanagesha@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#ifdef CONFIG_BL_SWITCHER
#include <asm/bL_switcher.h>
#endif

#include <mach/spc.h>
#include <mach/mhu.h>

#define BL_HYST_DEFAULT_COUNT	3
enum clusters {
	COLUMBUS_BL_BIG_ID = 0x0,	/* (MPIDR & 0xF00) >> 8 */
	COLUMBUS_BL_LITTLE_ID = 0x1,	/* (MPIDR & 0xF00) >> 8 */
	COLUMBUS_MAX_CLUSTER
};

static struct cpufreq_frequency_table *freq_table;
static struct cpufreq_frequency_table *ind_table[COLUMBUS_MAX_CLUSTER];
static atomic_t freq_table_users = ATOMIC_INIT(0);

static unsigned long clk_big_min;	/* Minimum (Big) clock frequency */
static unsigned long clk_little_max;	/* Maximum clock frequency (Little) */

static unsigned int bl_hyst_cfg_cnt = BL_HYST_DEFAULT_COUNT;
static unsigned int bl_hyst_current_cnt = BL_HYST_DEFAULT_COUNT;

/* Cached current cluster for each CPU to save on IPIs */
static DEFINE_PER_CPU(unsigned int, cpu_cur_cluster);
static DEFINE_PER_CPU(unsigned int, bl_up_hyst);
static DEFINE_PER_CPU(unsigned int, bl_down_hyst);

static ssize_t show_bl_hyst_config(struct cpufreq_policy *p, char *buf)
{
	return sprintf(buf, "%d\n", bl_hyst_cfg_cnt);
}

static ssize_t store_bl_hyst_config(struct cpufreq_policy *p,
					const char *buf, size_t n)
{
	if (sscanf(buf, "%u", &bl_hyst_cfg_cnt) != 1)
		return -EINVAL;
	return n;
}

cpufreq_freq_attr_rw(bl_hyst_config);

/*
 * Functions to get the current status.
 *
 * Beware that the cluster for another CPU may change unexpectedly.
 */

static unsigned int get_local_cluster(void)
{
	unsigned int mpidr;
	asm ("mrc\tp15, 0, %0, c0, c0, 5" : "=r" (mpidr));
	return (mpidr >> 8) & 0xf;
}

static void __get_current_cluster(void *_data)
{
	unsigned int *_cluster = _data;
	*_cluster = get_local_cluster();
}

static int get_current_cluster(unsigned int cpu)
{
	unsigned int cluster = 0;
	smp_call_function_single(cpu, __get_current_cluster, &cluster, 1);
	return cluster;
}

static int get_current_cached_cluster(unsigned int cpu)
{
	return per_cpu(cpu_cur_cluster, cpu);
}

#ifdef CONFIG_BL_SWITCHER
/*
 * Switch to the requested cluster.
 *
 * The __switch_to_entry version must be called with IRQs off on the
 * target CPU.  It is meant to be invoked via switch_to_entry() which
 * provides the necessary wrapping.
 */

static void __switch_to_entry(void *_data)
{
	unsigned int cpu = smp_processor_id();
	int old_cluster, new_cluster;

	old_cluster = get_local_cluster();
	new_cluster = *(int *)_data;

	/*
	 * IRQs are disabled here, including IPIs, so the cluster can't
	 * be changed by anyone else at this point.  Let's go ahead only
	 * if we really need to do something.
	 */
	if (new_cluster == old_cluster)
		return;

	/* FIXME:cpufreq_notify_transition can't be called with IRQs disabled */
	/* cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);*/
	if (!bL_switch_to(new_cluster))
		per_cpu(cpu_cur_cluster, cpu) = new_cluster;
	/* cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);*/
}
#endif

/* Validate policy frequency range */
static int columbus_cpufreq_verify_policy(struct cpufreq_policy *policy)
{
	/* This call takes care of it all using freq_table */
	return cpufreq_frequency_table_verify(policy, freq_table);
}

/* Set clock frequency */
static int columbus_cpufreq_set_target(struct cpufreq_policy *policy,
			     unsigned int target_freq, unsigned int relation)
{
	uint32_t cpu = policy->cpu;
	struct cpufreq_freqs freqs;
	uint32_t freq_tab_idx;
	uint32_t cur_cluster, new_cluster, do_switch = 0;
	int ret = 0;

	/* Read current clock rate */
	cur_cluster = get_current_cached_cluster(cpu);

	if (spc_get_performance(cur_cluster, &freq_tab_idx))
		return -EIO;

	freqs.old = ind_table[cur_cluster][freq_tab_idx].frequency;

	/* Make sure that target_freq is within supported range */
	if (target_freq > policy->max)
		target_freq = policy->max;
	if (target_freq < policy->min)
		target_freq = policy->min;

	/* Determine valid target frequency using freq_table */
	cpufreq_frequency_table_target(policy, freq_table, target_freq,
				       relation, &freq_tab_idx);
	freqs.new = freq_table[freq_tab_idx].frequency;

	freqs.cpu = policy->cpu;

	if (freqs.old == freqs.new)
		return 0;

	pr_debug("Requested Freq %d on %s cpu %d\n", freqs.new,
		cur_cluster == COLUMBUS_BL_LITTLE_ID ? "LITTLE" : "big", cpu);

	/*
	 * TODO:
	 * Apply hysteresis only for ondemand, conservative or interactive
	 * governors for now. Not applicable for powersave, performance and
	 * userspace governors.
	 * Need to make this independent of governor or move it outside driver
	 * bl_down_hyst/bl_up_hyst needs to be per-CPU if we need to support
	 * per-CPU switching
	 */
	if (freqs.new < clk_big_min &&
			cur_cluster == COLUMBUS_BL_BIG_ID &&
			++per_cpu(bl_down_hyst, cpu) >= bl_hyst_current_cnt) {
			do_switch = 1;	/* Switch to Little */
			per_cpu(bl_down_hyst, cpu) = 0;
	} else if (freqs.new > clk_little_max &&
			cur_cluster == COLUMBUS_BL_LITTLE_ID &&
			++per_cpu(bl_up_hyst, cpu) >= bl_hyst_current_cnt) {
			do_switch = 1;	/* Switch to Big */
			per_cpu(bl_up_hyst, cpu) = 0;
	}

	/* Skipping for hysteresis management */
	if (per_cpu(bl_down_hyst, cpu) || per_cpu(bl_up_hyst, cpu))
		return ret;

	for_each_cpu(freqs.cpu, policy->cpus)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	/*
	 * Set new clock divider rate with MHU calls to set OPP
	 */
	if (do_switch)
		new_cluster = (cur_cluster == COLUMBUS_BL_LITTLE_ID) ?
			COLUMBUS_BL_BIG_ID : COLUMBUS_BL_LITTLE_ID;
	else
		new_cluster = cur_cluster;

	cpufreq_frequency_table_target(policy, ind_table[new_cluster],
					freqs.new, relation, &freq_tab_idx);
	ret = set_performance(new_cluster, policy->cpu, freq_tab_idx);
	if (ret) {
		pr_err("Error %d while setting required OPP\n", ret);
		return ret;
	}

	if (do_switch)
		/*
		 * FIXME: forcing smp_call_function_single() to wait just for
		 * the purpose of running the notifyers is wasteful.
		 */
		smp_call_function_single(cpu, __switch_to_entry,
						(void *)&new_cluster, 1);

	policy->cur = freqs.new;

	for_each_cpu(freqs.cpu, policy->cpus)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

/* Get current clock frequency */
static unsigned int columbus_cpufreq_get(unsigned int cpu)
{
	uint32_t freq_tab_idx = 0;
	uint32_t cur_cluster = get_current_cached_cluster(cpu);

	/*
	 * Read current clock rate with SPC call
	 */
	if (spc_get_performance(cur_cluster, &freq_tab_idx))
		return -EIO;

	return ind_table[cur_cluster][freq_tab_idx].frequency;
}

/* get the number of entries in the cpufreq_frequency_table */
static inline int _cpufreq_get_table_size(struct cpufreq_frequency_table *table)
{
	int size = 0;
	for (; (table[size].frequency != CPUFREQ_TABLE_END); size++)
		;
	return size;
}

/* get the minimum frequency in the cpufreq_frequency_table */
static inline uint32_t _cpufreq_get_table_min(
			struct cpufreq_frequency_table *table)
{
	int i;
	uint32_t min_freq = ~0;
	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++)
		if (table[i].frequency < min_freq)
			min_freq = table[i].frequency;
	return min_freq;
}

/* get the maximum frequency in the cpufreq_frequency_table */
static inline uint32_t _cpufreq_get_table_max(
			struct cpufreq_frequency_table *table)
{
	int i;
	uint32_t max_freq = 0;
	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++)
		if (table[i].frequency > max_freq)
			max_freq = table[i].frequency;
	return max_freq;
}

/* translate the integer array into cpufreq_frequency_table entries */
static inline void _cpufreq_copy_table_from_array(uint32_t *table,
			struct cpufreq_frequency_table *freq_table, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		freq_table[i].index = i;
		freq_table[i].frequency = table[i];
	}
	freq_table[i].index = size;
	freq_table[i].frequency = CPUFREQ_TABLE_END;
}

/*
 * copy entries of all the per-cluster cpufreq_frequency_table entries into a
 * single frequency table which is published to cpufreq core
 */
static int columbus_cpufreq_merge_tables(uint32_t total_sz)
{
	int cluster_id, i;
	struct cpufreq_frequency_table *freqtable;

	freqtable = kzalloc(sizeof(struct cpufreq_frequency_table) *
						(total_sz + 1), GFP_KERNEL);
	if (!freqtable)
		return -ENOMEM;

	freq_table = freqtable;
	for (cluster_id = 0; cluster_id < COLUMBUS_MAX_CLUSTER; cluster_id++) {
		int size = _cpufreq_get_table_size(ind_table[cluster_id]);
		memcpy(freqtable, ind_table[cluster_id],
			size * sizeof(struct cpufreq_frequency_table));
		freqtable += size;
	}
	freq_table[total_sz].frequency = CPUFREQ_TABLE_END;

	/* Adjust the indices for merged table */
	for (i = 0; i <= total_sz; i++)
		freq_table[i].index = i;

	/* Assuming 2 cluster, set clk_big_min and clk_little_max */
	clk_big_min = _cpufreq_get_table_min(ind_table[COLUMBUS_BL_BIG_ID]);
	clk_little_max =
		_cpufreq_get_table_max(ind_table[COLUMBUS_BL_LITTLE_ID]);

	return 0;
}

static int columbus_cpufreq_init_from_scp(void)
{
	uint32_t cpu_opp_num, total_opp_num = 0;
	struct cpufreq_frequency_table *indtable[COLUMBUS_MAX_CLUSTER];
	uint32_t *cpu_freqs;
	int ret = 0, cluster_id = 0;

	for (cluster_id = 0; cluster_id < COLUMBUS_MAX_CLUSTER; cluster_id++) {
		ret = get_dvfs_size(cluster_id, 0, &cpu_opp_num);
		if (ret)
			return ret;
		total_opp_num += cpu_opp_num;
		cpu_freqs = kzalloc(sizeof(uint32_t) * cpu_opp_num, GFP_KERNEL);
		indtable[cluster_id] =
			kzalloc(sizeof(struct cpufreq_frequency_table) *
						(cpu_opp_num + 1), GFP_KERNEL);
		if (!cpu_freqs || !indtable[cluster_id]) {
			ret = -ENOMEM;
			goto free_mem;
		}
		ret = get_dvfs_capabilities(cluster_id, 0, (u32 *)cpu_freqs,
								cpu_opp_num);
		if (ret) {
			ret = -EIO;
			goto free_mem;
		}
		_cpufreq_copy_table_from_array(cpu_freqs,
				indtable[cluster_id], cpu_opp_num);
		ind_table[cluster_id] = indtable[cluster_id];

		kfree(cpu_freqs);
	}
	ret = columbus_cpufreq_merge_tables(total_opp_num);
	if (ret)
		goto free_mem;
	return ret;
free_mem:
	while (cluster_id >= 0)
		kfree(indtable[cluster_id]);
	kfree(cpu_freqs);
	return ret;
}

static int columbus_cpufreq_of_init(void)
{
	uint32_t cpu_opp_num, total_opp_num = 0;
	struct cpufreq_frequency_table *indtable[COLUMBUS_MAX_CLUSTER];
	uint32_t *cpu_freqs;
	int ret = 0, cluster_id = 0, len;
	struct device_node *cluster = NULL;
	const struct property *pp;
	const u32 *hwid;

	while ((cluster = of_find_node_by_name(cluster, "cluster"))) {
		hwid = of_get_property(cluster, "reg", &len);
		if (hwid && len == 4)
			cluster_id = be32_to_cpup(hwid);

		pp = of_find_property(cluster, "freqs", NULL);
		if (!pp)
			return -EINVAL;
		cpu_opp_num = pp->length / sizeof(u32);
		if (!cpu_opp_num)
			return -ENODATA;

		total_opp_num += cpu_opp_num;
		cpu_freqs = kzalloc(sizeof(uint32_t) * cpu_opp_num, GFP_KERNEL);
		indtable[cluster_id] =
			kzalloc(sizeof(struct cpufreq_frequency_table) *
						(cpu_opp_num + 1), GFP_KERNEL);
		if (!cpu_freqs || !indtable[cluster_id]) {
			ret = -ENOMEM;
			goto free_mem;
		}
		of_property_read_u32_array(cluster, "freqs",
							cpu_freqs, cpu_opp_num);
		_cpufreq_copy_table_from_array(cpu_freqs,
				indtable[cluster_id], cpu_opp_num);
		ind_table[cluster_id] = indtable[cluster_id];

		kfree(cpu_freqs);
	}
	ret = columbus_cpufreq_merge_tables(total_opp_num);
	if (ret)
		goto free_mem;
	return ret;
free_mem:
	while (cluster_id >= 0)
		kfree(indtable[cluster_id]);
	kfree(cpu_freqs);
	return ret;
}

static int columbus_cpufreq_notifier_policy(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	uint32_t cpu = policy->cpu;
	if (val != CPUFREQ_NOTIFY)
		return 0;
	if (strcmp(policy->governor->name, "powersave") == 0 ||
		strcmp(policy->governor->name, "performance") == 0 ||
		strcmp(policy->governor->name, "userspace") == 0) {
		bl_hyst_current_cnt = 1;
	} else {
		bl_hyst_current_cnt = bl_hyst_cfg_cnt;
	}
	per_cpu(bl_down_hyst, cpu) = per_cpu(bl_up_hyst, cpu) = 0;
	return 0;
}

static struct notifier_block notifier_policy_block = {
	.notifier_call = columbus_cpufreq_notifier_policy
};

/* Per-CPU initialization */
static int columbus_cpufreq_init(struct cpufreq_policy *policy)
{
	int result = 0;
	uint32_t cur_cluster = get_current_cluster(policy->cpu);

	if (atomic_inc_return(&freq_table_users) == 1) {
		result = columbus_cpufreq_of_init();
		if (result)
			result = columbus_cpufreq_init_from_scp();
	}

	if (result) {
		atomic_dec_return(&freq_table_users);
		pr_err("CPUFreq - CPU %d failed to initialize\n", policy->cpu);
		return result;
	}

	result = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (result)
		return result;

	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	per_cpu(cpu_cur_cluster, policy->cpu) = cur_cluster;

	/* set default policy and cpuinfo */
	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;

	policy->cpuinfo.transition_latency = 1000000;	/* 1 ms assumed */
	policy->cur = policy->max;

	if (atomic_read(&freq_table_users) == 1)
		result = cpufreq_register_notifier(&notifier_policy_block,
				CPUFREQ_POLICY_NOTIFIER);

	pr_info("CPUFreq for CPU %d initialized\n", policy->cpu);
	return result;
}

static int columbus_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_unregister_notifier(&notifier_policy_block,
			CPUFREQ_POLICY_NOTIFIER);
	return 0;
}

/* Export freq_table to sysfs */
static struct freq_attr *columbus_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&bl_hyst_config,
	NULL,
};

static struct cpufreq_driver columbus_cpufreq_driver = {
	.flags	= CPUFREQ_STICKY,
	.verify	= columbus_cpufreq_verify_policy,
	.target	= columbus_cpufreq_set_target,
	.get	= columbus_cpufreq_get,
	.init	= columbus_cpufreq_init,
	.exit	= columbus_cpufreq_exit,
	.name	= "cpufreq_columbus",
	.attr	= columbus_cpufreq_attr,
};

static int __init columbus_cpufreq_modinit(void)
{
	return cpufreq_register_driver(&columbus_cpufreq_driver);
}

static void __exit columbus_cpufreq_modexit(void)
{
	cpufreq_unregister_driver(&columbus_cpufreq_driver);
}

MODULE_DESCRIPTION("cpufreq driver for ARM Columbus platform with big.LITTLE switcher");
MODULE_LICENSE("GPL");

module_init(columbus_cpufreq_modinit);
module_exit(columbus_cpufreq_modexit);

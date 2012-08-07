/*
 * Vexpress big.LITTLE CPUFreq support
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
#include <linux/debugfs.h>
#include <linux/fs.h>
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

#include <linux/vexpress.h>

#ifdef CONFIG_BL_SWITCHER
#include <asm/bL_switcher.h>
#define is_bL_switching_enabled()		true
#else
#define bL_switch_to(cluster)			do { (void)cluster; } while (0)
#define is_bL_switching_enabled()		false
#endif

#define ACTUAL_FREQ(x)	\
	((is_bL_switching_enabled() && (x) > clk_little_max) ? (x) >> 1 : (x))
#define VIRT_FREQ(x)	\
	((is_bL_switching_enabled() && (x) > clk_little_max) ? (x) << 1 : (x))
#define VEXPRESS_MAX_CLUSTER	2

/* One extra for the virtual OPP table for b.L switching */
static struct cpufreq_frequency_table *freq_table[VEXPRESS_MAX_CLUSTER + 1];
static atomic_t freq_table_users = ATOMIC_INIT(0);

static unsigned int clk_big_min;	/* Minimum (Big) clock frequency */
static unsigned int clk_big_max;	/* Maximum (Big) clock frequency */
static unsigned int clk_little_max;	/* Maximum clock frequency (Little) */
static unsigned int big_cluster_id, little_cluster_id;
static unsigned int cluster_switch;
static unsigned int switch_2maxfreq = 1;

#define BL_HYST_DEFAULT_COUNT	1
static unsigned int bl_up_hyst_cfg_cnt = BL_HYST_DEFAULT_COUNT;
static unsigned int bl_down_hyst_cfg_cnt = BL_HYST_DEFAULT_COUNT;
static unsigned int bl_up_hyst_current_cnt = BL_HYST_DEFAULT_COUNT;
static unsigned int bl_down_hyst_current_cnt = BL_HYST_DEFAULT_COUNT;
static unsigned int switch_2maxfreq_cfg = 1;
static DEFINE_PER_CPU(unsigned int, bl_up_hyst);
static DEFINE_PER_CPU(unsigned int, bl_down_hyst);

/* Cached current cluster for each CPU to save on IPIs */
static DEFINE_PER_CPU(unsigned int, cpu_cur_cluster);

#define show_one(file_name, object)					\
static ssize_t show_##file_name(struct cpufreq_policy *p, char *buf)	\
{									\
	return sprintf(buf, "%d\n", object);				\
}
#define store_one(file_name, object)					\
static ssize_t store_##file_name(struct cpufreq_policy *p,		\
					const char *buf, size_t n)	\
{									\
	if (sscanf(buf, "%u", &object) != 1)				\
		return -EINVAL;						\
	return n;							\
}
show_one(bl_up_hyst_config, bl_up_hyst_cfg_cnt);
store_one(bl_up_hyst_config, bl_up_hyst_cfg_cnt);
show_one(bl_down_hyst_config, bl_down_hyst_cfg_cnt);
store_one(bl_down_hyst_config, bl_down_hyst_cfg_cnt);
show_one(switch_to_maxfreq_on_big, switch_2maxfreq_cfg);
store_one(switch_to_maxfreq_on_big, switch_2maxfreq_cfg);

cpufreq_freq_attr_rw(bl_up_hyst_config);
cpufreq_freq_attr_rw(bl_down_hyst_config);
cpufreq_freq_attr_rw(switch_to_maxfreq_on_big);

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

static void __bL_switch_to(void *_data)
{
	unsigned int *_cluster = _data;
	bL_switch_to(*_cluster);
}

/* Validate policy frequency range */
static int vexpress_cpufreq_verify_policy(struct cpufreq_policy *policy)
{
	uint32_t cluster, cur_cluster = get_current_cached_cluster(policy->cpu);

	cluster = is_bL_switching_enabled() ?
				VEXPRESS_MAX_CLUSTER : cur_cluster;

	/* This call takes care of it all using freq_table */
	return cpufreq_frequency_table_verify(policy, freq_table[cluster]);
}

static unsigned int vexpress_cpufreq_get(unsigned int cpu);
/* Set clock frequency */
static int vexpress_cpufreq_set_target(struct cpufreq_policy *policy,
			     unsigned int target_freq, unsigned int relation)
{
	uint32_t cpu;
	struct cpufreq_freqs freqs;
	uint32_t freq_tab_idx;
	uint32_t cluster, cur_cluster, new_cluster, do_switch = 0;
	int ret = 0;

	/* Read current clock rate */
	cur_cluster = get_current_cached_cluster(policy->cpu);

	freqs.old = vexpress_cpufreq_get(policy->cpu);

	/* Make sure that target_freq is within supported range */
	if (target_freq > policy->max)
		target_freq = policy->max;
	if (target_freq < policy->min)
		target_freq = policy->min;

	cluster = is_bL_switching_enabled() ?
				VEXPRESS_MAX_CLUSTER : cur_cluster;

	/* Determine valid target frequency using freq_table */
	cpufreq_frequency_table_target(policy, freq_table[cluster],
				       target_freq, relation, &freq_tab_idx);
	freqs.new = freq_table[cluster][freq_tab_idx].frequency;

	freqs.cpu = policy->cpu;

	if (freqs.old == freqs.new)
		return 0;

	pr_debug("Requested Freq %d cpu %d\n", freqs.new, policy->cpu);

	if (is_bL_switching_enabled()) {
		/* Assume only 2 CPUs, using cpu^1 to refer other */
		uint32_t other_cpu_cluster =
				get_current_cached_cluster(policy->cpu ^ 1);
		uint32_t other_cpu_freq = vexpress_cpufreq_get(policy->cpu ^ 1);
		if (freqs.new <= clk_big_min &&
			cur_cluster == big_cluster_id &&
			++per_cpu(bl_down_hyst, policy->cpu) >= bl_down_hyst_current_cnt) {
			do_switch = 1;	/* Switch to Little */
			if (!cluster_switch &&
					other_cpu_cluster == little_cluster_id)
				freqs.new = max(freqs.new, other_cpu_freq);
			per_cpu(bl_down_hyst, policy->cpu) = 0;
		} else if (freqs.new > clk_little_max &&
			cur_cluster == little_cluster_id &&
			++per_cpu(bl_up_hyst, policy->cpu) >= bl_up_hyst_current_cnt) {
			/*
			 * Switch from LITTLE to big
			 * Lets set new freq = max big freq
			 */
			do_switch = 1;
			if (!cluster_switch &&
					other_cpu_cluster == big_cluster_id)
				freqs.new = max(freqs.new, other_cpu_freq);
			if (!cluster_switch && switch_2maxfreq)
				freqs.new = clk_big_max;
			per_cpu(bl_up_hyst, policy->cpu) = 0;
		} else {
			if (!cluster_switch && other_cpu_cluster == cur_cluster)
				freqs.new = max(freqs.new, other_cpu_freq);
		}
	}

	/* Skipping for hysteresis management */
	if (per_cpu(bl_down_hyst, policy->cpu) || per_cpu(bl_up_hyst, policy->cpu))
		return ret;

	for_each_cpu(cpu, policy->cpus) {
		freqs.cpu = cpu;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}

	new_cluster = do_switch ? cur_cluster ^ 1 : cur_cluster;
	pr_debug("ReqF %d cpu%d clu%d ImpF %d nclus %d\n\n",
		target_freq, policy->cpu, cur_cluster, freqs.new, new_cluster);

	cpufreq_frequency_table_target(policy, freq_table[new_cluster],
			ACTUAL_FREQ(freqs.new), relation, &freq_tab_idx);
	ret = vexpress_spc_set_performance(new_cluster, freq_tab_idx);
	if (ret) {
		pr_err("Error %d while setting required OPP\n", ret);
		return ret;
	}

	if (do_switch) {
		for_each_cpu(cpu, policy->cpus) {
			smp_call_function_single(cpu, __bL_switch_to,
							&new_cluster, 0);
			per_cpu(cpu_cur_cluster, cpu) = new_cluster;
		}
	}

	policy->cur = freqs.new;

	for_each_cpu(cpu, policy->cpus) {
		freqs.cpu = cpu;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}

	return ret;
}

/* Get current clock frequency */
static unsigned int vexpress_cpufreq_get(unsigned int cpu)
{
	uint32_t freq_tab_idx = 0;
	uint32_t cur_cluster = get_current_cached_cluster(cpu);

	/*
	 * Read current clock rate with vexpress_spc call
	 */
	if (vexpress_spc_get_performance(cur_cluster, &freq_tab_idx))
		return -EIO;

	return VIRT_FREQ(freq_table[cur_cluster][freq_tab_idx].frequency);
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
		freq_table[i].frequency =  table[i] / 1000; /* in kHZ */
	}
	freq_table[i].index = size;
	freq_table[i].frequency = CPUFREQ_TABLE_END;
}

/*
 * copy entries of all the per-cluster cpufreq_frequency_table entries into a
 * single frequency table which is published to cpufreq core
 */
static int vexpress_cpufreq_merge_tables(uint32_t total_sz)
{
	int cluster_id, i;
	struct cpufreq_frequency_table *freqtable;

	freqtable = kzalloc(sizeof(struct cpufreq_frequency_table) *
						(total_sz + 1), GFP_KERNEL);
	if (!freqtable)
		return -ENOMEM;

	freq_table[VEXPRESS_MAX_CLUSTER] = freqtable;
	for (cluster_id = 0; cluster_id < VEXPRESS_MAX_CLUSTER; cluster_id++) {
		int size;
		if (freq_table[cluster_id] == NULL)
			return -ENODATA;
		size = _cpufreq_get_table_size(freq_table[cluster_id]);
		memcpy(freqtable, freq_table[cluster_id],
			size * sizeof(struct cpufreq_frequency_table));
		freqtable += size;
	}

	/* Adjust the indices for merged table */
	for (i = 0; i <= total_sz; i++) {
		freq_table[VEXPRESS_MAX_CLUSTER][i].index = i;
		if (i < 8)
			freq_table[VEXPRESS_MAX_CLUSTER][i].frequency <<= 1;
	}
	freq_table[VEXPRESS_MAX_CLUSTER][total_sz].frequency =
							CPUFREQ_TABLE_END;

	/* Assuming 2 cluster, set clk_big_min and clk_little_max */
	clk_little_max = 0;
	clk_big_min =
		VIRT_FREQ(_cpufreq_get_table_min(freq_table[big_cluster_id]));
	clk_big_max =
		VIRT_FREQ(_cpufreq_get_table_max(freq_table[big_cluster_id]));
	clk_little_max =
		_cpufreq_get_table_max(freq_table[little_cluster_id]);

	return 0;
}

static int vexpress_cpufreq_of_init(void)
{
	uint32_t cpu_opp_num, total_opp_num = 0;
	struct cpufreq_frequency_table *freqtable[VEXPRESS_MAX_CLUSTER];
	uint32_t *cpu_freqs = NULL;
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
		freqtable[cluster_id] =
			kzalloc(sizeof(struct cpufreq_frequency_table) *
						(cpu_opp_num + 1), GFP_KERNEL);
		if (!cpu_freqs || !freqtable[cluster_id]) {
			ret = -ENOMEM;
			goto free_mem;
		}
		of_property_read_u32_array(cluster, "freqs",
							cpu_freqs, cpu_opp_num);
		_cpufreq_copy_table_from_array(cpu_freqs,
				freqtable[cluster_id], cpu_opp_num);
		freq_table[cluster_id] = freqtable[cluster_id];

		kfree(cpu_freqs);
	}
	ret = vexpress_cpufreq_merge_tables(total_opp_num);
	if (ret)
		goto free_mem;
	return ret;
free_mem:
	while (cluster_id >= 0)
		kfree(freqtable[cluster_id--]);
	kfree(cpu_freqs);
	return ret;
}

static int vexpress_cpufreq_notifier_policy(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	uint32_t cpu = policy->cpu;
	if (val != CPUFREQ_NOTIFY)
		return 0;
	if (strcmp(policy->governor->name, "powersave") == 0 ||
		strcmp(policy->governor->name, "performance") == 0 ||
		strcmp(policy->governor->name, "userspace") == 0) {
		bl_up_hyst_current_cnt = bl_down_hyst_current_cnt = 1;
		switch_2maxfreq = 0;
	} else {
		bl_up_hyst_current_cnt = bl_up_hyst_cfg_cnt;
		bl_down_hyst_current_cnt = bl_down_hyst_cfg_cnt;
		switch_2maxfreq = switch_2maxfreq_cfg;
	}
	per_cpu(bl_down_hyst, cpu) = per_cpu(bl_up_hyst, cpu) = 0;
	return 0;
}

static struct notifier_block notifier_policy_block = {
	.notifier_call = vexpress_cpufreq_notifier_policy
};

/* Per-CPU initialization */
static int vexpress_cpufreq_init(struct cpufreq_policy *policy)
{
	int result = 0;
	uint32_t cluster, cur_cluster = get_current_cluster(policy->cpu);

	if (atomic_inc_return(&freq_table_users) == 1) {
		big_cluster_id = vexpress_spc_get_clusterid(A15_PART_NO);
		little_cluster_id = vexpress_spc_get_clusterid(A7_PART_NO);
		result = vexpress_cpufreq_of_init();
	}

	if (freq_table[cur_cluster] == NULL)
		result = -ENODATA;

	if (result) {
		atomic_dec_return(&freq_table_users);
		pr_err("CPUFreq - CPU %d failed to initialize\n", policy->cpu);
		return result;
	}

	cluster = is_bL_switching_enabled() ?
				VEXPRESS_MAX_CLUSTER : cur_cluster;

	result =
	    cpufreq_frequency_table_cpuinfo(policy, freq_table[cluster]);
	if (result)
		return result;

	cpufreq_frequency_table_get_attr(freq_table[cluster], policy->cpu);

	per_cpu(cpu_cur_cluster, policy->cpu) = cur_cluster;

	/* set default policy and cpuinfo */
	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;

	policy->cpuinfo.transition_latency = 1000000;	/* 1 ms assumed */
	policy->cur = vexpress_cpufreq_get(policy->cpu);

	if (!(is_bL_switching_enabled() ^ cluster_switch)) {
		cpumask_copy(policy->cpus, topology_core_cpumask(policy->cpu));
		cpumask_copy(policy->related_cpus, policy->cpus);
	}

	if (atomic_read(&freq_table_users) == 1)
		result = cpufreq_register_notifier(&notifier_policy_block,
				CPUFREQ_POLICY_NOTIFIER);

	pr_info("CPUFreq for CPU %d initialized\n", policy->cpu);
	return result;
}

static int vexpress_cpufreq_exit(struct cpufreq_policy *policy)
{
	if (atomic_read(&freq_table_users) == 1)
		cpufreq_unregister_notifier(&notifier_policy_block,
						CPUFREQ_POLICY_NOTIFIER);
	atomic_dec_return(&freq_table_users);
	return 0;
}

/* Export freq_table to sysfs */
static struct freq_attr *vexpress_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&bl_up_hyst_config,
	&bl_down_hyst_config,
	&switch_to_maxfreq_on_big,
	NULL,
};

static struct cpufreq_driver vexpress_cpufreq_driver = {
	.flags	= CPUFREQ_STICKY,
	.verify	= vexpress_cpufreq_verify_policy,
	.target	= vexpress_cpufreq_set_target,
	.get	= vexpress_cpufreq_get,
	.init	= vexpress_cpufreq_init,
	.exit	= vexpress_cpufreq_exit,
	.name	= "cpufreq_vexpress",
	.attr	= vexpress_cpufreq_attr,
};

static int cluster_switch_set(void *data, u64 val)
{
	if (val > 1 || val < 0) {
		pr_warn("Wrong parameter: 0 - disable 1 - enable\n");
		return -EINVAL;
	}
	if (cluster_switch == val)
		return 0;

	cpufreq_unregister_driver(&vexpress_cpufreq_driver);
	if (is_bL_switching_enabled())
		cluster_switch = val;
	cpufreq_register_driver(&vexpress_cpufreq_driver);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cluster_switch_fops, NULL,
					cluster_switch_set, "%llu\n");

static int __init vexpress_cpufreq_modinit(void)
{
	struct dentry *cluster_switch, *file_debug;

	if (!vexpress_spc_check_loaded()) {
		pr_info("vexpress cpufreq not initialised because no SPC found\n");
		return -ENODEV;
	}

	cluster_switch = debugfs_create_dir("cluster_switch", NULL);

	if (IS_ERR_OR_NULL(cluster_switch)) {
		pr_info("Error in creating cluster_switch debugfs directory\n");
		return 0;
	}

	file_debug = debugfs_create_file("enable", S_IRUGO | S_IWGRP,
				   cluster_switch, NULL, &cluster_switch_fops);

	if (IS_ERR_OR_NULL(file_debug)) {
		pr_info("Error in creating enable_cluster_switch file\n");
		return 0;
	}

	return cpufreq_register_driver(&vexpress_cpufreq_driver);
}

static void __exit vexpress_cpufreq_modexit(void)
{
	cpufreq_unregister_driver(&vexpress_cpufreq_driver);
}

MODULE_DESCRIPTION("cpufreq driver for ARM vexpress big.LITTLE platform");
MODULE_LICENSE("GPL");

module_init(vexpress_cpufreq_modinit);
module_exit(vexpress_cpufreq_modexit);

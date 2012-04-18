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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/opp.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/sysfs.h>
#include <linux/types.h>

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

static unsigned int bl_up_hyst, bl_down_hyst;
static unsigned int bl_hyst_cfg_cnt = BL_HYST_DEFAULT_COUNT;
static unsigned int bl_hyst_current_cnt = BL_HYST_DEFAULT_COUNT;

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
 * HVC Encoding is used directly as GCC doesn't support it yet
 * Currently <imm> = 2 - to get cluster ID, 1 - to switch cluster
 * Encoding A1 ARMv7V - HVC #<imm>
 * 31-28 : 27-24 : 23-20 : 19-16 : 15-12 : 11-8 : 7-4 : 3-0
 *  1110 :  0001 :  0100 :         imm12       : 0111 : imm4
 */
/* Determine the currently active big-little cluster */
static inline int columbus_cpufreq_get_bl_cluster(void)
{
	int cluster = 0;
	asm volatile (".inst 0xe1400072\n mov %0, r0\n" :
				"=r" (cluster) : : "r0", "cc");
	/* Mask MPIDR value */
	cluster = (cluster & 0x00000F00) >> 8;
	return cluster;
}

static int columbus_cpufreq_switch_bl_cluster(int cluster)
{
	int cur_cluster;

	cur_cluster = columbus_cpufreq_get_bl_cluster();
	if (cur_cluster != cluster) {
		asm volatile (".inst 0xe1400071\n" : : : "cc");
		pr_debug("Switched to cluster %d!\n",
				columbus_cpufreq_get_bl_cluster());
	} else {
		pr_debug("Already on cluster %d!\n",
				columbus_cpufreq_get_bl_cluster());
	}
	return 0;
}

/* Validate policy frequency range */
static int columbus_cpufreq_verify_policy(struct cpufreq_policy *policy)
{
	/* This call takes care of it all using freq_table */
	cpufreq_frequency_table_verify(policy, freq_table);
	return 0;
}

/* Set clock frequency */
static int columbus_cpufreq_set_target(struct cpufreq_policy *policy,
			     unsigned int target_freq, unsigned int relation)
{
	cpumask_t cpus_allowed;
	uint32_t cpu = policy->cpu;
	struct cpufreq_freqs freqs;
	uint32_t freq_tab_idx;
	uint32_t cur_cluster, new_cluster, do_switch = 0;
	int ret = 0;

	/* Prevent thread cpu migration - not sure if necessary */
	cpus_allowed = current->cpus_allowed;

	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	BUG_ON(cpu != smp_processor_id());

	/* Read current clock rate */
	cur_cluster = columbus_cpufreq_get_bl_cluster();

	if (get_performance(cur_cluster, cpu, &freq_tab_idx))
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

	if (freqs.old == freqs.new) {
		set_cpus_allowed(current, cpus_allowed);
		return 0;
	}

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
			++bl_down_hyst >= bl_hyst_current_cnt) {
			do_switch = 1;	/* Switch to Little */
			bl_down_hyst = 0;
	} else if (freqs.new > clk_little_max &&
			cur_cluster == COLUMBUS_BL_LITTLE_ID &&
			++bl_up_hyst >= bl_hyst_current_cnt) {
			do_switch = 1;	/* Switch to Big */
			bl_up_hyst = 0;
	}

	/* Skipping for hysteresis management */
	if (bl_down_hyst || bl_up_hyst) {
		set_cpus_allowed(current, cpus_allowed);
		return ret;
	}

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
		pr_err("failed to set the required OPP\n");
		set_cpus_allowed(current, cpus_allowed);
		return ret;
	}

	if (do_switch)
		columbus_cpufreq_switch_bl_cluster(new_cluster);

	policy->cur = freqs.new;

	set_cpus_allowed(current, cpus_allowed);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

/* Get current clock frequency */
static unsigned int columbus_cpufreq_get(unsigned int cpu)
{
	cpumask_t cpus_allowed;
	uint32_t freq_tab_idx = 0;
	uint32_t cur_cluster = columbus_cpufreq_get_bl_cluster();

	/* Prevent thread cpu migration - not sure if necessary */
	cpus_allowed = current->cpus_allowed;

	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	BUG_ON(cpu != smp_processor_id());

	/*
	 * Read current clock rate with MHU call
	 */
	if (get_performance(columbus_cpufreq_get_bl_cluster(), cpu,
		&freq_tab_idx))
		return -EIO;

	set_cpus_allowed(current, cpus_allowed);

	return ind_table[cur_cluster][freq_tab_idx].frequency;
}

static int columbus_cpufreq_init_table(void)
{
	unsigned int opp_sz[COLUMBUS_MAX_CLUSTER], opp_size = 0;
	struct cpufreq_frequency_table *freqtable;
	struct cpufreq_frequency_table *indtable[COLUMBUS_MAX_CLUSTER];
	unsigned long *table[COLUMBUS_MAX_CLUSTER];
	int loop, i, ret;

	for (loop = 0; loop < COLUMBUS_MAX_CLUSTER; loop++) {
		int j;
		ret = get_dvfs_size(loop, 0, &opp_sz[loop]);
		if (ret)
			return ret;
		opp_size += opp_sz[loop];
		table[loop] = kzalloc(sizeof(unsigned long) *
					(opp_sz[loop] + 1), GFP_KERNEL);
		indtable[loop] = kzalloc(sizeof(struct cpufreq_frequency_table)
					* (opp_sz[loop] + 1), GFP_KERNEL);
		if (!table[loop] || !indtable[loop]) {
			ret = -ENOMEM;
			goto free_mem;
		}
		ret = get_dvfs_capabilities(loop, 0, (u32 *)table[loop],
							opp_sz[loop]);
		if (ret) {
			ret = -EIO;
			goto free_mem;
		}
		for (j = 0; j < opp_sz[loop]; j++) {
			indtable[loop][j].index = j;
			indtable[loop][j].frequency = table[loop][j];
		}
		indtable[loop][j].index = j;
		indtable[loop][j].frequency = CPUFREQ_TABLE_END;

		ind_table[loop] = indtable[loop];
	}

	freqtable = kzalloc(sizeof(struct cpufreq_frequency_table) *
					(opp_size + 1), GFP_KERNEL);
	if (!freqtable) {
		ret = -ENOMEM;
		goto free_mem;
	}
	for (loop = 0, i = 0; i < COLUMBUS_MAX_CLUSTER; i++) {
		int j;
		for (j = 0; j < opp_sz[i]; j++, loop++) {
			freqtable[loop].index = loop;
			freqtable[loop].frequency = table[i][j];
		}
	}
	freqtable[loop].index = loop;
	freqtable[loop].frequency = CPUFREQ_TABLE_END;

	freq_table = freqtable;

	/* Assuming 2 cluster, set clk_big_min and clk_little_max */
	clk_little_max = table[COLUMBUS_BL_LITTLE_ID]
				[opp_sz[COLUMBUS_BL_LITTLE_ID] - 1];
	clk_big_min
		= table[COLUMBUS_BL_BIG_ID][opp_sz[COLUMBUS_BL_BIG_ID] - 1];

	for (loop = 0; loop < COLUMBUS_MAX_CLUSTER; loop++)
		kfree(table[loop]);
	return 0;
free_mem:
	while (loop >= 0) {
		kfree(table[loop]);
		kfree(indtable[loop]);
	}
	return ret;
}

static int columbus_cpufreq_notifier_policy(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	if (val != CPUFREQ_NOTIFY)
		return 0;
	if (strcmp(policy->governor->name, "powersave") == 0 ||
		strcmp(policy->governor->name, "performance") == 0 ||
		strcmp(policy->governor->name, "userspace") == 0) {
		bl_hyst_current_cnt = 1;
	} else {
		bl_hyst_current_cnt = bl_hyst_cfg_cnt;
	}
	bl_down_hyst = bl_up_hyst = 0;
	return 0;
}

static struct notifier_block notifier_policy_block = {
	.notifier_call = columbus_cpufreq_notifier_policy
};

/* Per-CPU initialization */
static int columbus_cpufreq_init(struct cpufreq_policy *policy)
{
	int result = 0;

	if (atomic_inc_return(&freq_table_users) == 1)
		result = columbus_cpufreq_init_table();

	if (result) {
		atomic_dec_return(&freq_table_users);
		return result;
	}

	result = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (result)
		return result;

	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	/* set default policy and cpuinfo */
	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;

	policy->cpuinfo.transition_latency = 1000000;	/* 1 ms assumed */
	policy->cur = policy->max;

	/* Set up frequency dependencies */
	cpumask_copy(policy->cpus, cpu_possible_mask);	/* affected_cpus */
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	result = cpufreq_register_notifier(&notifier_policy_block,
				CPUFREQ_POLICY_NOTIFIER);
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

MODULE_DESCRIPTION("cpufreq driver for ARM columbus platform");
MODULE_LICENSE("GPL");

module_init(columbus_cpufreq_modinit);
module_exit(columbus_cpufreq_modexit);

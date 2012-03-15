/*
 * linux/arch/arm/mach-columbus/cpufreq.c
 *
 * ARM COLUMBUS cpufreq support
 * Based on linux/arch/arm/mach-integrator/cpu.c
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/opp.h>
#include <linux/sysfs.h>

#define COLUMBUS_BL_LITTLE_ID	0x1	/* (MPIDR & 0xF00) >> 8 */
#define COLUMBUS_BL_BIG_ID	0x0	/* (MPIDR & 0xF00) >> 8 */

static struct cpufreq_frequency_table *freq_table;

static unsigned int current_freq;

static unsigned long clk_big_min;	/* Minimum (Big) clock frequency */
static unsigned long clk_little_max;	/* Maximum clock frequency (Little) */

static unsigned int bl_up_hyst, bl_down_hyst;
static unsigned int bl_hyst_cnt = 3;
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
	uint32_t cur_cluster, do_switch = 0;

	/* Prevent thread cpu migration - not sure if necessary */
	cpus_allowed = current->cpus_allowed;

	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	BUG_ON(cpu != smp_processor_id());

	/* Read current clock rate */
	cur_cluster = columbus_cpufreq_get_bl_cluster();

	freqs.old = current_freq;
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

	/* TODO:
	 * Apply hysteresis only for ondemand, conservative or interactive
	 * governors for now. Not applicable for powersave, performance and
	 * userspace governors.
	 * Need to make this independent of governor or move it outside driver
	 * bl_down_hyst/bl_up_hyst needs to be per-CPU if we need to support
	 * per-CPU switching
	 */
	if (strcmp(policy->governor->name, "powersave") == 0 ||
		strcmp(policy->governor->name, "performance") == 0 ||
		strcmp(policy->governor->name, "userspace") == 0) {
		bl_down_hyst = bl_up_hyst = bl_hyst_cnt;
	}
	if (freqs.new < clk_big_min &&
			cur_cluster == COLUMBUS_BL_BIG_ID &&
			++bl_down_hyst >= bl_hyst_cnt) {
			do_switch = 1;	/* Switch to Little */
	} else if (freqs.new > clk_little_max &&
			cur_cluster == COLUMBUS_BL_LITTLE_ID &&
			++bl_up_hyst >= bl_hyst_cnt) {
			do_switch = 1;	/* Switch to Big */
	}

	if (bl_up_hyst >= bl_hyst_cnt || bl_down_hyst >= bl_hyst_cnt) {
		bl_down_hyst = bl_up_hyst = 0;
	} else if (bl_down_hyst || bl_up_hyst) {
		set_cpus_allowed(current, cpus_allowed);
		return 0;
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	/* TODO:
	 * Set new clock divider rate with MHU calls to set OPP
	 */

	if (do_switch)
		columbus_cpufreq_switch_bl_cluster(
			cur_cluster == COLUMBUS_BL_LITTLE_ID ?
			COLUMBUS_BL_BIG_ID : COLUMBUS_BL_LITTLE_ID);
	cur_cluster = columbus_cpufreq_get_bl_cluster();

	policy->cur = current_freq = freqs.new;

	set_cpus_allowed(current, cpus_allowed);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

/* Get current clock frequency */
static unsigned int columbus_cpufreq_get(unsigned int cpu)
{
	cpumask_t cpus_allowed;

	/* Prevent thread cpu migration - not sure if necessary */
	cpus_allowed = current->cpus_allowed;

	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	BUG_ON(cpu != smp_processor_id());

	/* TODO:
	 * Read current clock rate with MHU call
	 */

	set_cpus_allowed(current, cpus_allowed);

	return current_freq;
}

/* Per-CPU initialization */
static int columbus_cpufreq_init(struct cpufreq_policy *policy)
{
	int result = 0;

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
	cpumask_copy(policy->cpus, cpu_active_mask);	/* affected_cpus */
	cpumask_copy(policy->related_cpus, cpu_active_mask);
	policy->shared_type = CPUFREQ_SHARED_TYPE_HW;

	/* TODO: Read current clock rate */
	current_freq = policy->max;

	return 0;
}

/* Export freq_table to sysfs */
static struct freq_attr *columbus_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver columbus_cpufreq_driver = {
	.flags	= CPUFREQ_STICKY,
	.verify	= columbus_cpufreq_verify_policy,
	.target	= columbus_cpufreq_set_target,
	.get	= columbus_cpufreq_get,
	.init	= columbus_cpufreq_init,
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

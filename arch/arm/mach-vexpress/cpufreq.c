/*
 * linux/arch/arm/mach-vexpress/cpufreq.c
 *
 * Copyright (C) 2010 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <asm/hardware/elba_scpc.h>

#define ARRAY_AND_SIZE(x) (x), ARRAY_SIZE(x)
#define VEXPRESS_MIN_FREQUENCY 915000 /* 915 MHz  */
#define VEXPRESS_MAX_FREQUENCY 1713000 /* 1.7 GHz  */
#define OP(cpufreq) \
{								\
	.cpufreq_mhz	= cpufreq,				\
}

static unsigned long target_cpu_perf[CONFIG_NR_CPUS];
static unsigned long current_cpu_perf;

/* FIX STRUCTURE  */
struct vexpress_freq_info {
	unsigned int cpufreq_mhz;
};

/* FIX VALUES!!!  */
static struct vexpress_freq_info vexpress_freqs_array[] = {
	/*  CPU */
	OP(915),  /* 915MHz */
	OP(1024), /* 1024MHz */
	OP(1133), /* 1133MHz */
	OP(1242), /* 1242MHz */
	OP(1320), /* 1320MHz */
	OP(1376), /* 1376MHz */
	OP(1432), /* 1432MHz */
	OP(1488), /* 1488MHz */
	OP(1544), /* 1544MHz */
	OP(1601), /* 1601MHz */
	OP(1657), /* 1657MHz */
	OP(1713), /* 1713MHz */
};
static unsigned int vexpress_freqs_num;
static struct vexpress_freq_info *vexpress_freqs;
static struct cpufreq_frequency_table *vexpress_freqs_table;

static int vexpress_update_cpu_perf(void)
{
	int i;
	unsigned long idx = 0;
	int ret = 0;
	struct cpufreq_freqs freqs;

	for_each_online_cpu(i)
		idx = max(idx, target_cpu_perf[i]);

	freqs.old = vexpress_freqs_array[current_cpu_perf].cpufreq_mhz * 1000;
	freqs.new = vexpress_freqs_array[idx].cpufreq_mhz * 1000;

	if (freqs.old == freqs.new)
		return ret;

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-elba: transition: %u --> %u\n",
	       freqs.old, freqs.new);
#endif
	current_cpu_perf = idx;

	ret = hip_set_performance(idx);

	if (ret) {
		pr_err("elba: Failed to set cpu frequency to %d kHz\n",
			freqs.new);
		return ret;
	}

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static int setup_freqs_table(struct cpufreq_policy *policy,
			     struct vexpress_freq_info *freqs, int num)
{
	struct cpufreq_frequency_table *table;
	int i;

	table = kzalloc((num + 1) * sizeof(*table), GFP_KERNEL);
	if (table == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		table[i].index = i;
		table[i].frequency = freqs[i].cpufreq_mhz * 1000;
	}
	table[num].index = i;
	table[num].frequency = CPUFREQ_TABLE_END;

	vexpress_freqs = freqs;
	vexpress_freqs_num = num;
	vexpress_freqs_table = table;

	return cpufreq_frequency_table_cpuinfo(policy, table);
}

static int vexpress_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, vexpress_freqs_table);
}

static int vexpress_cpufreq_set(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	struct vexpress_freq_info *next;
	int idx, ret;

	/* Lookup the next frequency */
	if (cpufreq_frequency_table_target(policy, vexpress_freqs_table,
				target_freq, relation, &idx))
		return -EINVAL;

	next = &vexpress_freqs_array[idx];
	target_cpu_perf[policy->cpu] = idx;

	/* Update core frequency  */
	ret = vexpress_update_cpu_perf();

	return ret;
}

static __init int vexpress_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret = -EINVAL;

	/* set default policy and cpuinfo */
	policy->cpuinfo.min_freq = VEXPRESS_MIN_FREQUENCY;
	policy->cpuinfo.max_freq = VEXPRESS_MAX_FREQUENCY;
	policy->cpuinfo.transition_latency = 1000; /* FIXME: 1 ms, assumed */
	policy->cur = policy->min = policy->max = 0; /* FIXME: get clock freqeuncy  */

	ret = setup_freqs_table(policy, ARRAY_AND_SIZE(vexpress_freqs_array));
	if (ret) {
		pr_err("failed to setup frequency table\n");
		return ret;
	}

	pr_info("CPUFREQ support for Versatile Express initialized\n");

	return 0;
}

static unsigned int vexpress_cpufreq_get(unsigned int cpu)
{
	/* Read frequency from proper register.  */

	return 0;
}

static struct cpufreq_driver vexpress_cpufreq_driver = {
	.verify = vexpress_cpufreq_verify,
	.target = vexpress_cpufreq_set,
	.init   = vexpress_cpufreq_init,
	.get    = vexpress_cpufreq_get,
	.name   = "vexpress-cpufreq",
};

static int __init cpufreq_init(void)
{
	cpufreq_register_driver(&vexpress_cpufreq_driver);

	return 0;
}
late_initcall(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	cpufreq_unregister_driver(&vexpress_cpufreq_driver);
}
module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for Versatile Express");
MODULE_LICENSE("GPL");

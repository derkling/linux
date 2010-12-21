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

#define ARRAY_AND_SIZE(x) (x), ARRAY_SIZE(x)
#define VEXPRESS_MIN_FREQUENCY 100000 /* 100 MHz  */
#define VEXPRESS_MAX_FREQUENCY 800000 /* 800 MHz  */
#define OP(cpufreq, _xl, _xn, _hss, _dmc, _smc, _sfl, _dfi, vcore, vsram) \
{								\
	.cpufreq_mhz	= cpufreq,				\
}

/* FIX STRUCTURE  */
struct vexpress_freq_info {
	unsigned int cpufreq_mhz;
};

/* FIX VALUES!!!  */
static struct vexpress_freq_info vexpress_freqs_array[] = {
	/*  CPU XL XN  HSS DMEM SMEM SRAM DFI VCC_CORE VCC_SRAM */
	OP(104,  8, 1, 104, 260,  78, 104, 3, 1000, 1100), /* 104MHz */
	OP(208, 16, 1, 104, 260, 104, 156, 2, 1000, 1100), /* 208MHz */
	OP(416, 16, 2, 156, 260, 104, 208, 2, 1100, 1200), /* 416MHz */
	OP(624, 24, 2, 208, 260, 208, 312, 3, 1375, 1400), /* 624MHz */
	OP(806, 31, 2, 208, 260, 208, 312, 3, 1400, 1400), /* 806MHz */
};
static unsigned int vexpress_freqs_num;
static struct vexpress_freq_info *vexpress_freqs;
static struct cpufreq_frequency_table *vexpress_freqs_table;

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
	struct cpufreq_freqs freqs;
	unsigned long flags;
	int idx;

	if (policy->cpu != 0)
		return -EINVAL;

	/* Lookup the next frequency */
	if (cpufreq_frequency_table_target(policy, vexpress_freqs_table,
				target_freq, relation, &idx))
		return -EINVAL;

	next = &vexpress_freqs_array[idx];

	freqs.old = policy->cur;
	freqs.new = next->cpufreq_mhz * 1000;
	freqs.cpu = policy->cpu;

	pr_debug("CPU frequency from %d MHz to %d MHz%s\n",
			freqs.old / 1000, freqs.new / 1000,
			(freqs.old == freqs.new) ? " (skipped)" : "");

	if (freqs.old == target_freq)
		return 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	local_irq_save(flags);
	/* Update core frequency  */
	local_irq_restore(flags);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
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
module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	cpufreq_unregister_driver(&vexpress_cpufreq_driver);
}
module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for Versatile Express")
MODULE_LICENSE("GPL")

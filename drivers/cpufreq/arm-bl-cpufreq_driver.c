/*
 * arm-bl-cpufreq.c: Simple cpufreq backend for the ARM big.LITTLE switcher
 * Copyright (C) 2012  Linaro Limited
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* WARNING: This code is experimental and depends on external firmware */

#define MODULE_NAME "arm-bl-cpufreq"
#define pr_fmt(fmt) MODULE_NAME ": " fmt

#include <linux/bug.h>
#include <linux/cache.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/spinlock.h>

#include "arm-bl-cpufreq.h"

/* Dummy frequencies representing the big and little clusters: */
#define FREQ_BIG	1000000
#define FREQ_LITTLE	 100000

/*  Cluster numbers */
#define CLUSTER_BIG	0
#define CLUSTER_LITTLE	1

/*
 * Switch latency advertised to cpufreq.  This value is bogus and will
 * need to be properly calibrated when running on real hardware.
 */
#define BL_CPUFREQ_FAKE_LATENCY 1

static DEFINE_SPINLOCK(switcher_lock);

static struct cpufreq_frequency_table __read_mostly bl_freqs[] = {
	{ CLUSTER_BIG,		FREQ_BIG		},
	{ CLUSTER_LITTLE,	FREQ_LITTLE		},
	{ 0,			CPUFREQ_TABLE_END	},
};


/* Miscellaneous helpers */

static unsigned int entry_to_freq(
	struct cpufreq_frequency_table const *entry)
{
	return entry->frequency;
}

static unsigned int entry_to_cluster(
	struct cpufreq_frequency_table const *entry)
{
	return entry->index;
}

static struct cpufreq_frequency_table const *find_entry_by_cluster(int cluster)
{
	unsigned int i;

	for(i = 0; entry_to_freq(&bl_freqs[i]) != CPUFREQ_TABLE_END; i++)
		if(entry_to_cluster(&bl_freqs[i]) == cluster)
			return &bl_freqs[i];

	WARN(1, pr_fmt("%s(): invalid cluster number %d, assuming 0\n"),
		__func__, cluster);
	return &bl_freqs[0];
}

static unsigned int cluster_to_freq(int cluster)
{
	return entry_to_freq(find_entry_by_cluster(cluster));
}

/*
 * Functions to get the current status.
 *
 * If you intend to use the result (i.e., it's not just for diagnostic
 * purposes) then you should be holding switcher_lock ... otherwise
 * the current cluster may change unexpectedly.
 */
static int get_current_cluster(void)
{
	return (__arm_bl_get_cluster() >> 8) & 0xF;
}

static unsigned int get_current_freq(void)
{
	return cluster_to_freq(get_current_cluster());
}

/*
 * Switch to the requested cluster.
 * There is no "switch_to_frequency" function, because the cpufreq frequency
 * table helpers can easily look up the appropriate cluster number for us.
 */
static void switch_to_entry(struct cpufreq_frequency_table const *target)
{
	int old_cluster;
	struct cpufreq_freqs freqs;

	pr_info("Switching to cluster %d\n", entry_to_cluster(target));

	spin_lock(&switcher_lock);

	old_cluster = get_current_cluster();
	if(entry_to_cluster(target) != old_cluster) {
		freqs.old = cluster_to_freq(old_cluster);
		freqs.new = entry_to_freq(target);

		for_each_cpu(freqs.cpu, cpu_present_mask)
			cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

		__arm_bl_switch_cluster();

		for_each_cpu(freqs.cpu, cpu_present_mask)
			cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	}

	spin_unlock(&switcher_lock);
}


/* Cpufreq methods and module code */

static int bl_cpufreq_init(struct cpufreq_policy *policy)
{
	int err;

	/*
	 * Set CPU and policy min and max frequencies based on bl_freqs:
	 */
	err = cpufreq_frequency_table_cpuinfo(policy, bl_freqs);
	if (err)
		goto error;

	/*
	 * No need for locking here:
	 * cpufreq is not active until initialisation has finished.
	 * Ideally, transition_latency should be calibrated here.
	 */
	policy->cpuinfo.transition_latency = BL_CPUFREQ_FAKE_LATENCY;
	policy->cur = get_current_freq();

	/*
	 * A b.L switch can be triggered from any CPU, but will affect them all.
	 * The set of related CPUs should perhaps be determined from the
	 * system CPU topology, rather than just the set of CPUs present...
	 */
	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	cpumask_copy(policy->related_cpus, cpu_present_mask);
	/*
	 * We do not set ->cpus here, because it doesn't actually matter if
	 * we try to switch on two CPUs at the same time.  Setting ->cpus
	 * to cpu_present_mask might provide a way to avoid the need to take
	 * switcher_lock when switching, though.
	 */

	pr_info("cpufreq initialised successfully\n");
	return 0;

error:
	pr_warning("cpufreq initialisation failed (%d)\n", err);
	return err;
}

static int bl_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, bl_freqs);
}

static int bl_cpufreq_target(struct cpufreq_policy *policy,
			     unsigned int target_freq,
			     unsigned int relation)
{
	int err;
	int index;

	err = cpufreq_frequency_table_target(policy, bl_freqs, target_freq,
					     relation, &index);
	if(err)
		return err;

	switch_to_entry(&bl_freqs[index]);
	return 0;
}

static unsigned int bl_cpufreq_get(unsigned int __always_unused cpu)
{
	/*
	 * The cpu argument is ignored, because all CPUs have the same
	 * performance point, by definition.
	 */
	return get_current_freq();
}

static struct cpufreq_driver __read_mostly bl_cpufreq_driver = {
	.owner = THIS_MODULE,
	.name = MODULE_NAME,

	.init = bl_cpufreq_init,
	.verify = bl_cpufreq_verify,
	.target = bl_cpufreq_target,
	.get = bl_cpufreq_get,
	/* what else? */
};

static int __init bl_cpufreq_module_init(void)
{
	int err;

	err = cpufreq_register_driver(&bl_cpufreq_driver);
	if(err)
		pr_info("cpufreq backend driver registration failed (%d)\n",
			err);
	else
		pr_info("cpufreq backend driver registered.\n");

	return err;
}
module_init(bl_cpufreq_module_init);

static void __exit bl_cpufreq_module_exit(void)
{
	cpufreq_unregister_driver(&bl_cpufreq_driver);

	/* Restore the "default" cluster: */
	switch_to_entry(find_entry_by_cluster(CLUSTER_BIG));

	pr_info("cpufreq backend driver unloaded.\n");
}
module_exit(bl_cpufreq_module_exit);


MODULE_AUTHOR("Dave Martin");
MODULE_DESCRIPTION("Simple cpufreq interface for the ARM big.LITTLE switcher");
MODULE_LICENSE("GPL");

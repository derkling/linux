/*
 *  Copyright (C)  2015 Michael Turquette <mturquette@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/irq_work.h>

#include "sched.h"

#define THROTTLE_NSEC		50000000 /* 50ms default */

static DEFINE_PER_CPU(unsigned long, new_capacity);

/**
 * gov_data - per-policy data internal to the governor
 * @throttle: next throttling period expiry. Derived from throttle_nsec
 * @throttle_nsec: throttle period length in nanoseconds
 * @task: worker thread for dvfs transition that may block/sleep
 * @irq_work: callback used to wake up worker thread
 *
 * struct gov_data is the per-policy gov_cfs-specific data structure. A
 * per-policy instance of it is created when the gov_cfs governor receives
 * the CPUFREQ_GOV_START condition and a pointer to it exists in the gov_data
 * member of struct cpufreq_policy.
 *
 * Readers of this data must call down_read(policy->rwsem). Writers must
 * call down_write(policy->rwsem).
 */
struct gov_data {
	ktime_t throttle;
	unsigned int throttle_nsec;
	struct task_struct *task;
	struct irq_work irq_work;
	struct cpufreq_policy *policy;
};

/**
 * gov_cfs_select_freq - pick the next frequency for a cpu
 * @policy: the cpufreq policy whose frequency may be changed
 *
 * gov_cfs_select_freq selects a frequency based on pelt load statistics
 * tracked by cfs. First it finds the most utilized cpu in the policy and then
 * maps that utilization value onto a cpu frequency and returns it.
 *
 * Additionally, gov_cfs_select_freq adds a margin to the cpu utilization value
 * before converting it to a frequency. The margin is derived from MARGIN_PCT,
 * which itself is inspired by imbalance_pct in cfs. This is needed to
 * proactively increase frequency in the case of increasing load.
 *
 * This approach attempts to maintain headroom of 25% unutilized cpu capacity.
 * A traditional way of doing this is to take 75% of the current capacity and
 * check if current utilization crosses that threshold. The only problem with
 * that approach is determining the next cpu frequency target if that threshold
 * is crossed.
 *
 * Instead of using the 75% threshold, gov_cfs_select_freq adds a 25%
 * utilization margin to the utilization and converts that to a frequency. This
 * removes conditional logic around checking thresholds and better supports
 * drivers that use non-discretized frequency ranges (i.e. no pre-defined
 * frequency tables or operating points).
 *
 * Returns frequency selected.
 */
static unsigned long gov_cfs_select_freq(struct cpufreq_policy *policy)
{
	int cpu = 0;
	struct gov_data *gd;
	unsigned long freq = 0, max_usage = 0, usage = 0;

	if (!policy->governor_data)
		goto out;

	gd = policy->governor_data;

	/*
	 * get_cpu_usage is called without locking the runqueues. This is the
	 * same behavior used by find_busiest_cpu in load_balance. We are
	 * willing to accept occasionally stale data here in exchange for
	 * lockless behavior.
	 */
	for_each_cpu(cpu, policy->cpus) {
		usage = per_cpu(new_capacity, cpu);
		if (usage > max_usage)
			max_usage = usage;
	}

	/* add margin to max_usage based on imbalance_pct */
	max_usage = max_usage * MARGIN_PCT / 100;

	cpu = cpumask_first(policy->cpus);

	if (max_usage >= capacity_orig_of(cpu)) {
		freq = policy->max;
		goto out;
	}

	/* freq is current utilization + 25% */
	freq = (max_usage * policy->max) / capacity_orig_of(cpu);

out:
	return freq;
}

/*
 * we pass in struct cpufreq_policy. This is safe because changing out the
 * policy requires a call to __cpufreq_governor(policy, CPUFREQ_GOV_STOP),
 * which tears down all of the data structures and __cpufreq_governor(policy,
 * CPUFREQ_GOV_START) will do a full rebuild, including this kthread with the
 * new policy pointer
 */
static int gov_cfs_thread(void *data)
{
	struct sched_param param;
	struct cpufreq_policy *policy;
	struct gov_data *gd;
	unsigned long freq;
	int ret;

	policy = (struct cpufreq_policy *) data;
	if (!policy) {
		pr_warn("%s: missing policy\n", __func__);
		do_exit(-EINVAL);
	}

	gd = policy->governor_data;
	if (!gd) {
		pr_warn("%s: missing governor data\n", __func__);
		do_exit(-EINVAL);
	}

	param.sched_priority = 50;
	ret = sched_setscheduler_nocheck(gd->task, SCHED_FIFO, &param);
	if (ret) {
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		do_exit(-EINVAL);
	} else {
		pr_debug("%s: kthread (%d) set to SCHED_FIFO\n",
				__func__, gd->task->pid);
	}

	ret = set_cpus_allowed_ptr(gd->task, policy->related_cpus);
	if (ret) {
		pr_warn("%s: failed to set allowed ptr\n", __func__);
		do_exit(-EINVAL);
	}

	/* main loop of the per-policy kthread */
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		if (kthread_should_stop())
			break;

		/* avoid race with gov_cfs_stop */
		if (!down_write_trylock(&policy->rwsem))
			continue;

		freq = gov_cfs_select_freq(policy);

		ret = __cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		if (ret)
			pr_debug("%s: __cpufreq_driver_target returned %d\n",
					__func__, ret);

		gd->throttle = ktime_add_ns(ktime_get(), gd->throttle_nsec);
		up_write(&policy->rwsem);
	} while (!kthread_should_stop());

	do_exit(0);
}

static void gov_cfs_irq_work(struct irq_work *irq_work)
{
	struct gov_data *gd;

	gd = container_of(irq_work, struct gov_data, irq_work);
	if (!gd) {
		return;
	}

	wake_up_process(gd->task);
}

void gov_cfs_reset_cpu(int cpu)
{
	per_cpu(new_capacity, cpu) = 0;
}

/**
 * gov_cfs_update_cpu - interface to scheduler for changing capacity values
 * @cpu: cpu whose capacity utilization has recently changed
 *
 * gov_cfs_udpate_cpu is an interface exposed to the scheduler so that the
 * scheduler may inform the governor of updates to capacity utilization and
 * make changes to cpu frequency. Currently this interface is designed around
 * PELT values in CFS. It can be expanded to other scheduling classes in the
 * future if needed.
 *
 * gov_cfs_update_cpu raises an IPI. The irq_work handler for that IPI wakes up
 * the thread that does the actual work, gov_cfs_thread.
 */
void gov_cfs_update_cpu(int cpu, unsigned long capacity)
{
	struct cpufreq_policy *policy;
	struct gov_data *gd;

	/* XXX put policy pointer in per-cpu data? */
	policy = cpufreq_cpu_get(cpu);
	if (IS_ERR_OR_NULL(policy)) {
		return;
	}

	if (!policy->governor_data) {
		goto out;
	}

	gd = policy->governor_data;

	/* bail early if we are throttled */
	if (ktime_before(ktime_get(), gd->throttle)) {
		goto out;
	}

	per_cpu(new_capacity, cpu) = capacity;
	irq_work_queue_on(&gd->irq_work, cpu);

out:
	cpufreq_cpu_put(policy);
	return;
}

static void gov_cfs_start(struct cpufreq_policy *policy)
{
	struct gov_data *gd;
	int cpu;

	/* prepare per-policy private data */
	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd) {
		pr_debug("%s: failed to allocate private data\n", __func__);
		return;
	}

	/*
	 * Don't ask for freq changes at an higher rate than what
	 * the driver advertises as transition latency.
	 */
	gd->throttle_nsec = policy->cpuinfo.transition_latency ?
			    policy->cpuinfo.transition_latency :
			    THROTTLE_NSEC;
	pr_debug("%s: throttle threshold = %u [ns]\n",
		  __func__, gd->throttle_nsec);

	for_each_cpu(cpu, policy->related_cpus)
		per_cpu(new_capacity, cpu) = 0;

	/* init per-policy kthread */
	gd->task = kthread_run(gov_cfs_thread, policy, "kgov_cfs_task");
	if (IS_ERR_OR_NULL(gd->task))
		pr_err("%s: failed to create kgov_cfs_task thread\n", __func__);

	init_irq_work(&gd->irq_work, gov_cfs_irq_work);
	policy->governor_data = gd;
	gd->policy = policy;
}

static void gov_cfs_stop(struct cpufreq_policy *policy)
{
	struct gov_data *gd;

	gd = policy->governor_data;
	kthread_stop(gd->task);

	policy->governor_data = NULL;

	/* FIXME replace with devm counterparts? */
	kfree(gd);
}

static int gov_cfs_setup(struct cpufreq_policy *policy, unsigned int event)
{
	switch (event) {
		case CPUFREQ_GOV_START:
			/* Start managing the frequency */
			gov_cfs_start(policy);
			return 0;

		case CPUFREQ_GOV_STOP:
			gov_cfs_stop(policy);
			return 0;

		case CPUFREQ_GOV_LIMITS:	/* unused */
		case CPUFREQ_GOV_POLICY_INIT:	/* unused */
		case CPUFREQ_GOV_POLICY_EXIT:	/* unused */
			break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHED_CFS
static
#endif
struct cpufreq_governor cpufreq_gov_cfs = {
	.name			= "gov_cfs",
	.governor		= gov_cfs_setup,
	.owner			= THIS_MODULE,
};

static int __init gov_cfs_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_cfs);
}

static void __exit gov_cfs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_cfs);
}

/* Try to make this the default governor */
fs_initcall(gov_cfs_init);

MODULE_LICENSE("GPL");

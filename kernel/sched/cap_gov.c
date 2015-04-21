/*
 *  Copyright (C)  2014 Michael Turquette <mturquette@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/percpu.h>

#include "sched.h"

#define UP_THRESHOLD		95
#define THROTTLE_NSEC		50000000 /* 50ms default */

/* XXX always true, for now */
static bool driver_might_sleep = true;

/*
 * per-cpu pointer to atomic_t gov_data->cap_gov_wake_task
 * used in scheduler hot paths {en,de}queueu, task_tick without having to
 * access struct cpufreq_policy and struct gov_data
 */
static DEFINE_PER_CPU(atomic_t *, cap_gov_wake_task);

/*
 * FIXME if we move to an arbitrator model then we can have a single thread for
 * the whole system. This means that per-policy data can be removed and
 * replaced with per-governor data.
 *
 * FIXME correction to the above statement. The thread doesn't matter. We could
 * always get rid of per-policy data by having per-cpu pointers to some private
 * data. This is how the legacy governors do it. I'm not sure its any better or
 * worse.
 */

/**
 * gov_data - per-policy data internal to the governor
 * @throttle: time until throttling period expires. Derived from THROTTLE_NSEC
 * @task: worker task for dvfs transition that may block/sleep
 * @target_freq: frequency targeted for next transition
 * @need_wake_task: flag the governor to wake this policy's worker thread
 *
 * struct gov_data is the per-policy cap_gov-specific data structure. A
 * per-policy instance of it is created when the cap_gov governor receives
 * the CPUFREQ_GOV_START condition and a pointer to it exists in the gov_data
 * member of struct cpufreq_policy.
 *
 * Readers of this data must call down_read(policy->rwsem). Writers must
 * call down_write(policy->rwsem).
 */
struct gov_data {
	ktime_t throttle;
	unsigned int new_cap;
	unsigned int *cap_to_freq;
	unsigned int throttle_nsec;
	struct task_struct *task;
	atomic_long_t target_freq;
	atomic_t need_wake_task;
};

/**
 * cap_gov_select_freq - pick the next frequency for a cpu
 * @cpu: the cpu whose frequency may be changed
 *
 * cap_gov_select_freq works in a way similar to the ondemand governor. First
 * we inspect the utilization of all of the cpus in this policy to find the
 * most utilized cpu. This is achieved by calling get_cpu_usage, which returns
 * frequency-invarant capacity utilization.
 *
 * This max utilization is compared against the up_threshold (default 95%
 * utilization). If the max cpu utilization is greater than this threshold then
 * we scale the policy up to the max frequency. Othewise we find the lowest
 * frequency (smallest cpu capacity) that is still larger than the max capacity
 * utilization for this policy.
 *
 * Returns frequency selected.
 */
static unsigned long cap_to_freq(struct cpufreq_policy *policy)
{
	int cpu;
	struct gov_data *gd;
	int index;
	unsigned long freq = 0;
	struct cpufreq_frequency_table *pos;

	if (!policy->gov_data)
		goto out;

	gd = policy->gov_data;

	/* FIXME debug only */
	cpu = cpumask_first(policy->cpus);

	/* trivial case of fully loaded cpu */
	if (gd->new_cap == capacity_orig_of(cpu)) {
		freq = policy->max;
		goto out;
	}

	/*
	 * FIXME
	 * Sadly cpufreq freq tables are not ordered by frequency...
	 * but we act as if they were, for the time being >:)
	 */
	index = 0;
	freq = policy->max;
	cpufreq_for_each_entry(pos, policy->freq_table) {
		trace_printk("index=%d freq=%u cap_to_freq=%u",
			     index, pos->frequency,
			     gd->cap_to_freq[index]);
		if (gd->cap_to_freq[index] >= gd->new_cap) {
			freq = pos->frequency;
			break;
		}
		index++;
	}
	trace_printk("cpu %d final freq %lu", cpu, freq);
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
static int cap_gov_thread(void *data)
{
	struct sched_param param;
	struct cpufreq_policy *policy;
	struct gov_data *gd;
	unsigned long new_freq;
	int ret;

	policy = (struct cpufreq_policy *) data;
	if (!policy) {
		pr_warn("%s: missing policy\n", __func__);
		do_exit(-EINVAL);
	}

	gd = policy->gov_data;
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
		down_write(&policy->rwsem);
		if (!atomic_read(&gd->need_wake_task))  {
			trace_printk("kthread (%d) goes to sleep", gd->task->pid);
			up_write(&policy->rwsem);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			continue;
		}

		new_freq = cap_to_freq(policy);
		ret = __cpufreq_driver_target(policy, new_freq,
				CPUFREQ_RELATION_H);
		if (ret)
			pr_debug("%s: __cpufreq_driver_target returned %d\n",
					__func__, ret);

		trace_printk("kthread %d requested freq switch", gd->task->pid);

		gd->throttle = ktime_add_ns(ktime_get(), gd->throttle_nsec);
		atomic_set(&gd->need_wake_task, 0);
		up_write(&policy->rwsem);
	} while (!kthread_should_stop());

	do_exit(0);
}

static void cap_gov_wake_up_process(struct task_struct *task)
{
	/* this is null during early boot */
	if (IS_ERR_OR_NULL(task)) {
		return;
	}

	wake_up_process(task);
}

void cap_gov_kick_thread(int cpu, unsigned int new_cap)
{
	struct cpufreq_policy *policy;
	struct gov_data *gd = NULL;

	policy = cpufreq_cpu_get(cpu);
	if (IS_ERR_OR_NULL(policy))
		return;

	gd = policy->gov_data;
	if (!gd)
		goto out;

	/* bail early if we are throttled */
	if (ktime_before(ktime_get(), gd->throttle)) {
		trace_printk("THROTTLED (%d)", gd->task->pid);
		goto out;
	}

	/* XXX driver_might_sleep is always true */
	if (driver_might_sleep) {
		gd->new_cap = new_cap;
		trace_printk("waking up kthread (%d) new_cap=%u",
			     gd->task->pid, gd->new_cap);
		atomic_set(per_cpu(cap_gov_wake_task, cpu), 1);
		cap_gov_wake_up_process(gd->task);
	} else {
		BUG_ON(1);
		/* XXX someday select freq and program it here */
	}

out:
	cpufreq_cpu_put(policy);
}

static void cap_gov_start(struct cpufreq_policy *policy)
{
	int index = 0, count = 0, cpu;
	struct gov_data *gd;
	struct cpufreq_frequency_table *pos;
	struct sched_domain *sd;
	struct sched_group_energy *sge = NULL;

	/* prepare per-policy private data */
	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd) {
		pr_debug("%s: failed to allocate private data\n", __func__);
		return;
	}

	/* how many entries in the frequency table? */
	cpufreq_for_each_entry(pos, policy->freq_table)
		count++;

	/* pre-compute per-capacity utilization up_thresholds */
	gd->cap_to_freq = kcalloc(count, sizeof(unsigned int), GFP_KERNEL);

	rcu_read_lock();
	for_each_domain(cpumask_first(policy->cpus), sd)
		if (!sd->child) {
#ifdef CONFIG_SCHED_DEBUG
			pr_debug("%s: using %s sched_group_energy\n",
				 __func__,
				 sd->name);
#endif
			sge = sd->groups->sge;
			break;
		}

	if (!sge) {
		pr_debug("%s: failed to access sched_group_energy\n",
			 __func__);
		kfree(gd->cap_to_freq);
		kfree(gd);
		rcu_read_unlock();
		return;
	}

	for (index = 0; index < sge->nr_cap_states; index++) {
		/* capacity below is both freq and uarch scaled */
		gd->cap_to_freq[index] = sge->cap_states[index].cap;

		pr_debug("%s: cpu=%u index=%d capacity=%u\n",
				__func__, cpumask_first(policy->cpus), index,
				gd->cap_to_freq[index]);
	}
	rcu_read_unlock();

	/*
	 * Don't ask for freq changes at an higher rate than what
	 * the driver advertises as transition latency.
	 */
	gd->throttle_nsec = policy->cpuinfo.transition_latency ?
			    policy->cpuinfo.transition_latency :
			    THROTTLE_NSEC;
	pr_debug("%s: throttle threshold = %u [ns]\n",
		  __func__, gd->throttle_nsec);

	/* save per-cpu pointer to per-policy need_wake_task */
	for_each_cpu(cpu, policy->related_cpus)
		per_cpu(cap_gov_wake_task, cpu) = &gd->need_wake_task;

	/* FIXME if we move to an arbitrator model then we will only want one thread? */
	/* init per-policy kthread */
	gd->task = kthread_create(cap_gov_thread, policy, "kcap_gov_task");
	if (IS_ERR_OR_NULL(gd->task))
		pr_err("%s: failed to create kcap_gov_task thread\n", __func__);

	policy->gov_data = gd;
	for_each_cpu(cpu, policy->related_cpus)
		atomic_set(&cpu_rq(cpu)->cap_gov_enabled, 1);
}

static void cap_gov_stop(struct cpufreq_policy *policy)
{
	struct gov_data *gd;
	int cpu;

	gd = policy->gov_data;

	for_each_cpu(cpu, policy->related_cpus)
		atomic_set(&cpu_rq(cpu)->cap_gov_enabled, 0);
	policy->gov_data = NULL;

	kthread_stop(gd->task);

	/* FIXME replace with devm counterparts? */
	kfree(gd->cap_to_freq);
	kfree(gd);
}

static int cap_gov_setup(struct cpufreq_policy *policy, unsigned int event)
{
	switch (event) {
		case CPUFREQ_GOV_START:
			/* Start managing the frequency */
			cap_gov_start(policy);
			return 0;

		case CPUFREQ_GOV_STOP:
			cap_gov_stop(policy);
			return 0;

		case CPUFREQ_GOV_LIMITS:	/* unused */
		case CPUFREQ_GOV_POLICY_INIT:	/* unused */
		case CPUFREQ_GOV_POLICY_EXIT:	/* unused */
			break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_CAP_GOV
static
#endif
struct cpufreq_governor cpufreq_gov_cap_gov = {
	.name			= "cap_gov",
	.governor		= cap_gov_setup,
	.owner			= THIS_MODULE,
};

static int __init cap_gov_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_cap_gov);
}

static void __exit cap_gov_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_cap_gov);
}

/* Try to make this the default governor */
fs_initcall(cap_gov_init);

MODULE_LICENSE("GPL");

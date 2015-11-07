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
#include <linux/delay.h>

#define CREATE_TRACE_POINTS
#include <trace/events/sched_freq.h>

#include "sched.h"

#define THROTTLE_NSEC		50000000 /* 50ms default */

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHED
static struct cpufreq_governor cpufreq_gov_sched;
#endif

static struct cpumask enabled_cpus;
static DEFINE_PER_CPU(unsigned long, pcpu_capacity);
DEFINE_PER_CPU(struct sched_capacity_reqs, cpu_sched_capacity_reqs);

/**
 * gov_data - per-policy data internal to the governor
 * @throttle: next throttling period expiry. Derived from throttle_nsec
 * @throttle_nsec: throttle period length in nanoseconds
 * @task: worker thread for dvfs transition that may block/sleep
 * @irq_work: callback used to wake up worker thread
 * @freq: new frequency stored in *_sched_update_cpu and used in *_sched_thread
 *
 * struct gov_data is the per-policy cpufreq_sched-specific data structure. A
 * per-policy instance of it is created when the cpufreq_sched governor receives
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
	unsigned int requested_freq;
};

static void cpufreq_sched_try_driver_target(struct cpufreq_policy *policy,
					    unsigned int freq)
{
	struct gov_data *gd = policy->governor_data;

	/* avoid race with cpufreq_sched_stop */
	if (!down_write_trylock(&policy->rwsem))
		return;

	__cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_L);

	gd->throttle = ktime_add_ns(ktime_get(), gd->throttle_nsec);
	up_write(&policy->rwsem);
}

static void finish_last_request(struct gov_data *gd)
{
	int ns_left;

	while (1) {
		ktime_t now = ktime_get();

		if (ktime_after(now, gd->throttle))
			return;

		ns_left = ktime_to_ns(ktime_sub(gd->throttle, now));
		trace_sched_freq_throttled(ns_left/NSEC_PER_USEC);
		usleep_range(ns_left/NSEC_PER_USEC,
			     ns_left/NSEC_PER_USEC);
	}
}

/*
 * we pass in struct cpufreq_policy. This is safe because changing out the
 * policy requires a call to __cpufreq_governor(policy, CPUFREQ_GOV_STOP),
 * which tears down all of the data structures and __cpufreq_governor(policy,
 * CPUFREQ_GOV_START) will do a full rebuild, including this kthread with the
 * new policy pointer
 */
static int cpufreq_sched_thread(void *data)
{
	struct sched_param param;
	struct cpufreq_policy *policy;
	struct gov_data *gd;
	unsigned int new_request = 0;
	unsigned int last_request = 0;
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

	/* main loop of the per-policy kthread */
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		new_request = gd->requested_freq;
		if (new_request == last_request) {
			schedule();
		} else {
			last_request = new_request;
			finish_last_request(gd);
			cpufreq_sched_try_driver_target(policy, new_request);
		}
	} while (!kthread_should_stop());

	return 0;
}

static void cpufreq_sched_irq_work(struct irq_work *irq_work)
{
	struct gov_data *gd;

	gd = container_of(irq_work, struct gov_data, irq_work);
	if (!gd) {
		return;
	}

	wake_up_process(gd->task);
}

/**
 * cpufreq_sched_set_capacity - interface to scheduler for changing capacity values
 * @cpu: cpu whose capacity utilization has recently changed
 * @capacity: the new capacity requested by cpu
 *
 * cpufreq_sched_sched_capacity is an interface exposed to the scheduler so
 * that the scheduler may inform the governor of updates to capacity
 * utilization and make changes to cpu frequency. Currently this interface is
 * designed around PELT values in CFS. It can be expanded to other scheduling
 * classes in the future if needed.
 *
 * cpufreq_sched_set_capacity raises an IPI. The irq_work handler for that IPI
 * wakes up the thread that does the actual work, cpufreq_sched_thread.
 *
 * This functions bails out early if either condition is true:
 * 1) this cpu did not the new maximum capacity for its frequency domain
 * 2) no change in cpu frequency is necessary to meet the new capacity request
 */
static void cpufreq_sched_set_cap(int cpu, unsigned long capacity)
{
	unsigned int freq_new, index_new, cpu_tmp;
	struct cpufreq_policy *policy;
	struct gov_data *gd;
	unsigned long capacity_max = 0;

	/*
	 * Avoid grabbing the policy if possible. A test is still
	 * required after locking the CPU's policy to avoid racing
	 * with the governor changing.
	 */
	if (!cpumask_test_cpu(cpu, &enabled_cpus))
		return;

	/* update per-cpu capacity request */
	per_cpu(pcpu_capacity, cpu) = capacity;

	policy = cpufreq_cpu_get(cpu);
	if (IS_ERR_OR_NULL(policy)) {
		return;
	}

	if (policy->governor != &cpufreq_gov_sched ||
	    !policy->governor_data)
		goto out;

	gd = policy->governor_data;

	/* find max capacity requested by cpus in this policy */
	for_each_cpu(cpu_tmp, policy->cpus)
		capacity_max = max(capacity_max, per_cpu(pcpu_capacity,
							 cpu_tmp));

	/*
	 * We only change frequency if this cpu's capacity request represents a
	 * new max. If another cpu has requested a capacity greater than the
	 * previous max then we rely on that cpu to hit this code path and make
	 * the change. IOW, the cpu with the new max capacity is responsible
	 * for setting the new capacity/frequency.
	 *
	 * If this cpu is not the new maximum then bail
	 */
	if (capacity_max > capacity)
		goto out;

	/* Convert the new maximum capacity request into a cpu frequency */
	freq_new = capacity * policy->max >> SCHED_CAPACITY_SHIFT;
	if (cpufreq_frequency_table_target(policy, policy->freq_table,
					   freq_new, CPUFREQ_RELATION_L,
					   &index_new))
		goto out;
	freq_new = policy->freq_table[index_new].frequency;
	trace_sched_freq_request_opp(cpu, capacity, freq_new,
				     gd->requested_freq);
	/* No change in frequency? Bail and return current capacity. */
	if (freq_new == gd->requested_freq)
		goto out;

	/* store the new frequency and perform the transition */
	gd->requested_freq = freq_new;

	if (cpufreq_driver_might_sleep() ||
	    ktime_before(ktime_get(), gd->throttle))
		irq_work_queue_on(&gd->irq_work, cpu);
	else
		cpufreq_sched_try_driver_target(policy, freq_new);

out:
	cpufreq_cpu_put(policy);
	return;
}

void update_cpu_capacity_request(int cpu)
{
	unsigned long new_capacity;
	struct sched_capacity_reqs *scr;

	lockdep_assert_held(&cpu_rq(cpu)->lock);

	scr = &per_cpu(cpu_sched_capacity_reqs, cpu);

	new_capacity = scr->cfs + scr->rt;
	new_capacity = new_capacity * capacity_margin
		/ SCHED_CAPACITY_SCALE;
	new_capacity += scr->dl;
	if (new_capacity < scr->dl_min)
		new_capacity = scr->dl_min;
	trace_sched_freq_update_capacity(cpu, scr, new_capacity);
	if (new_capacity != scr->total) {
		cpufreq_sched_set_cap(cpu, new_capacity);
		scr->total = new_capacity;
	}
}

static inline void set_sched_freq(void)
{
	if (!sched_freq())
		static_key_slow_inc(&__sched_freq);
}

static inline void clear_sched_freq(void)
{
	if (sched_freq())
		static_key_slow_dec(&__sched_freq);
}

static int cpufreq_sched_start(struct cpufreq_policy *policy)
{
	struct gov_data *gd;
	int cpu;

	/* prepare per-policy private data */
	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd) {
		pr_debug("%s: failed to allocate private data\n", __func__);
		return -ENOMEM;
	}

	/* initialize per-cpu data */
	for_each_cpu(cpu, policy->cpus)
		per_cpu(pcpu_capacity, cpu) = 0;

	/*
	 * Don't ask for freq changes at an higher rate than what
	 * the driver advertises as transition latency.
	 */
	gd->throttle_nsec = policy->cpuinfo.transition_latency ?
			    policy->cpuinfo.transition_latency :
			    THROTTLE_NSEC;
	pr_debug("%s: throttle threshold = %u [ns]\n",
		  __func__, gd->throttle_nsec);

	if (cpufreq_driver_might_sleep()) {
		/* init per-policy kthread */
		gd->task = kthread_create(cpufreq_sched_thread, policy,
					  "kschedfreq:%d",
					  cpumask_first(policy->related_cpus));
		if (IS_ERR_OR_NULL(gd->task)) {
			pr_err("%s: failed to create kschedfreq thread\n",
			       __func__);
			goto err;
		}
		kthread_bind_mask(gd->task, policy->related_cpus);
		wake_up_process(gd->task);
		init_irq_work(&gd->irq_work, cpufreq_sched_irq_work);
	}

	policy->governor_data = gd;
	gd->policy = policy;
	cpumask_or(&enabled_cpus, &enabled_cpus, policy->related_cpus);
	set_sched_freq();
	return 0;

err:
	kfree(gd);
	return -ENOMEM;
}

static int cpufreq_sched_stop(struct cpufreq_policy *policy)
{
	struct gov_data *gd = policy->governor_data;

	cpumask_andnot(&enabled_cpus, &enabled_cpus, policy->related_cpus);
	clear_sched_freq();
	if (cpufreq_driver_might_sleep()) {
		kthread_stop(gd->task);
	}

	policy->governor_data = NULL;

	/* FIXME replace with devm counterparts? */
	kfree(gd);
	return 0;
}

static int cpufreq_sched_setup(struct cpufreq_policy *policy, unsigned int event)
{
	switch (event) {
		case CPUFREQ_GOV_START:
			/* Start managing the frequency */
			return cpufreq_sched_start(policy);

		case CPUFREQ_GOV_STOP:
			return cpufreq_sched_stop(policy);

		case CPUFREQ_GOV_LIMITS:	/* unused */
		case CPUFREQ_GOV_POLICY_INIT:	/* unused */
		case CPUFREQ_GOV_POLICY_EXIT:	/* unused */
			break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHED
static
#endif
struct cpufreq_governor cpufreq_gov_sched = {
	.name			= "sched",
	.governor		= cpufreq_sched_setup,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_sched_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_sched);
}

static void __exit cpufreq_sched_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_sched);
}

/* Try to make this the default governor */
fs_initcall(cpufreq_sched_init);

MODULE_LICENSE("GPL v2");

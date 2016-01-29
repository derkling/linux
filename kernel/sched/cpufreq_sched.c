/*
 *  Copyright (C)  2015 Michael Turquette <mturquette@linaro.org>
 *  Copyright (C)  2015-2016 Steve Muckle <smuckle@linaro.org>
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
#include <linux/string.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_sched.h>

#include "sched.h"

struct static_key __read_mostly __sched_freq = STATIC_KEY_INIT_FALSE;
static bool __read_mostly cpufreq_driver_slow;

/*
 * The number of enabled schedfreq policies is modified during GOV_START/STOP.
 * It, along with whether the schedfreq static key is enabled, is protected by
 * the gov_enable_lock.
 */
static int enabled_policies;
static DEFINE_MUTEX(gov_enable_lock);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHED
static struct cpufreq_governor cpufreq_gov_sched;
#endif

/*
 * Capacity margin added to CFS and RT capacity requests to provide
 * some head room if task utilization further increases.
 */
unsigned int capacity_margin = 1280;

static DEFINE_PER_CPU(struct gov_data *, cpu_gov_data);
DEFINE_PER_CPU(struct sched_capacity_reqs, cpu_sched_capacity_reqs);

/**
 * gov_data - per-policy data internal to the governor
 * @throttle: next throttling period expiry. Derived from throttle_nsec
 * @throttle_nsec: throttle period length in nanoseconds
 * @task: worker thread for dvfs transition that may block/sleep
 * @irq_work: callback used to wake up worker thread
 * @policy: pointer to cpufreq policy associated with this governor data
 * @fastpath_lock: prevents multiple CPUs in a frequency domain from racing
 * with each other in fast path during calculation of domain frequency
 * @enabled: boolean value indicating that the policy is started, protected
 * by the policy->rwsem
 * @requested_freq: last frequency requested by the sched governor
 *
 * struct gov_data is the per-policy cpufreq_sched-specific data
 * structure. A per-policy instance of it is created when the
 * cpufreq_sched governor receives the CPUFREQ_GOV_POLICY_INIT
 * condition and a pointer to it exists in the gov_data member of
 * struct cpufreq_policy.
 */
struct gov_data {
	ktime_t throttle;
	unsigned int throttle_nsec;
	struct task_struct *task;
	struct irq_work irq_work;
	struct cpufreq_policy *policy;
	raw_spinlock_t fastpath_lock;
	unsigned int enabled;
	unsigned int requested_freq;
};

static void cpufreq_sched_try_driver_target(struct cpufreq_policy *policy,
					    unsigned int freq)
{
	struct gov_data *gd = policy->governor_data;

	trace_printk("kschedfreq (%d) asks freq=%u", task_pid_nr(gd->task), freq);
	__cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_L);
	trace_printk("kschedfreq (%d) done", task_pid_nr(gd->task));
	gd->throttle = ktime_add_ns(ktime_get(), gd->throttle_nsec);
}

static bool finish_last_request(struct gov_data *gd)
{
	ktime_t now = ktime_get();

	if (ktime_after(now, gd->throttle))
		return false;

	while (1) {
		int usec_left = ktime_to_ns(ktime_sub(gd->throttle, now));

		usec_left /= NSEC_PER_USEC;
		trace_cpufreq_sched_throttled(usec_left);
		usleep_range(usec_left, usec_left + 100);
		now = ktime_get();
		if (ktime_after(now, gd->throttle))
			return true;
	}
}

static int cpufreq_sched_thread(void *data)
{
	struct sched_param param;
	struct gov_data *gd = (struct gov_data*) data;
	struct cpufreq_policy *policy = gd->policy;
	unsigned int new_request = 0;
	unsigned int last_request = 0;
	int ret;

	param.sched_priority = 50;
	ret = sched_setscheduler_nocheck(gd->task, SCHED_FIFO, &param);
	if (ret) {
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		do_exit(-EINVAL);
	} else {
		pr_debug("%s: kthread (%d) set to SCHED_FIFO\n",
				__func__, gd->task->pid);
		trace_printk("kschedfreq (%d) set to SCHED_FIFO",
				task_pid_nr(gd->task));
	}


	while (true) {
		if (kthread_should_stop()) {
			trace_printk("%s exiting (%d)", gd->task->comm, task_pid_nr(gd->task));
			break;
		}
		down_write(&policy->rwsem);
		set_current_state(TASK_INTERRUPTIBLE);
		new_request = gd->requested_freq;
		trace_printk("kschedfreq (%d) new_request=%u", task_pid_nr(gd->task), new_request);
		if (!gd->enabled || new_request == last_request) {
			trace_printk("kschedfreq (%d) goes to sleep: enabled=%d last_request=%d", task_pid_nr(gd->task), gd->enabled, last_request);
			up_write(&policy->rwsem);
			schedule();
		} else {
			set_current_state(TASK_RUNNING);
			/*
			 * if the frequency thread sleeps while waiting to be
			 * unthrottled, start over to check for a newer request
			 */
			if (finish_last_request(gd)) {
				trace_printk("kschedfreq (%d) last_request done", task_pid_nr(gd->task));
				up_write(&policy->rwsem);
				continue;
			}
			last_request = new_request;
			cpufreq_sched_try_driver_target(policy, new_request);
			up_write(&policy->rwsem);
		}
	}

	return 0;
}

static void cpufreq_sched_irq_work(struct irq_work *irq_work)
{
	struct gov_data *gd;

	gd = container_of(irq_work, struct gov_data, irq_work);
	if (!gd)
		return;

	wake_up_process(gd->task);
}

static void update_fdomain_capacity_request(int cpu)
{
	unsigned int freq_new, index_new, cpu_tmp;
	struct cpufreq_policy *policy;
	struct gov_data *gd = per_cpu(cpu_gov_data, cpu);
	unsigned long capacity = 0;

	if (!gd) {
		trace_printk("cpu=%d: no gov data", cpu);
		return;
	}

	/* interrupts already disabled here via rq locked */
	raw_spin_lock(&gd->fastpath_lock);

	policy = gd->policy;

	for_each_cpu(cpu_tmp, policy->cpus) {
		struct sched_capacity_reqs *scr;

		scr = &per_cpu(cpu_sched_capacity_reqs, cpu_tmp);
		capacity = max(capacity, scr->total);
	}

	freq_new = capacity * policy->max >> SCHED_CAPACITY_SHIFT;

	/*
	 * Calling this without locking policy->rwsem means we race
	 * against changes with policy->min and policy->max. This should
	 * be okay though.
	 */
	if (cpufreq_frequency_table_target(policy, policy->freq_table,
					   freq_new, CPUFREQ_RELATION_L,
					   &index_new)) {
		trace_printk("cpu=%d: already at freq_new=%u", cpu, freq_new);
		goto out;
	}
	freq_new = policy->freq_table[index_new].frequency;

	trace_cpufreq_sched_request_opp(cpu, capacity, freq_new,
					gd->requested_freq);

	if (freq_new == gd->requested_freq) {
		trace_printk("%s (%d): already requested freq_new=%u", gd->task->comm, task_pid_nr(gd->task), freq_new);
		goto out;
	}

	gd->requested_freq = freq_new;

	if (cpufreq_driver_slow || !down_write_trylock(&policy->rwsem)) {
			trace_printk("1: kschedfreq:%d (%d) cpu=%d freq_new=%d",
					cpumask_first(policy->related_cpus), task_pid_nr(gd->task),
					cpu, freq_new);
		irq_work_queue_on(&gd->irq_work, cpu);
	} else if (policy->transition_ongoing ||
		   ktime_before(ktime_get(), gd->throttle)) {
		up_write(&policy->rwsem);
		trace_printk("2: kschedfreq:%d (%d) cpu=%d freq_new=%d",
				cpumask_first(policy->related_cpus), task_pid_nr(gd->task),
				cpu, freq_new);
		irq_work_queue_on(&gd->irq_work, cpu);
	} else {
		trace_printk("3: kschedfreq:%d (%d) cpu=%d freq_new=%d",
				cpumask_first(policy->related_cpus), task_pid_nr(gd->task),
				cpu, freq_new);
		cpufreq_sched_try_driver_target(policy, freq_new);
		up_write(&policy->rwsem);
	}

out:
	raw_spin_unlock(&gd->fastpath_lock);
}

void update_cpu_capacity_request(int cpu, bool request)
{
	unsigned long new_capacity;
	struct sched_capacity_reqs *scr;

	/* The rq lock serializes access to the CPU's sched_capacity_reqs. */
	lockdep_assert_held(&cpu_rq(cpu)->lock);

	scr = &per_cpu(cpu_sched_capacity_reqs, cpu);

	new_capacity = scr->cfs + scr->rt;
	new_capacity = new_capacity * capacity_margin
		/ SCHED_CAPACITY_SCALE;
	new_capacity += scr->dl;

	if (new_capacity == scr->total)
		return;

	trace_cpufreq_sched_update_capacity(cpu, request, scr, new_capacity);

	scr->total = new_capacity;
	if (request)
		update_fdomain_capacity_request(cpu);
}

static ssize_t show_throttle_nsec(struct cpufreq_policy *policy, char *buf)
{
	struct gov_data *gd = policy->governor_data;
	return sprintf(buf, "%u\n", gd->throttle_nsec);
}

static ssize_t store_throttle_nsec(struct cpufreq_policy *policy,
				   const char *buf, size_t count)
{
	struct gov_data *gd = policy->governor_data;
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	gd->throttle_nsec = input;
	return count;
}

static struct freq_attr sched_freq_throttle_nsec_attr =
	__ATTR(throttle_nsec, 0644, show_throttle_nsec, store_throttle_nsec);

static struct attribute *sched_freq_sysfs_attribs[] = {
	&sched_freq_throttle_nsec_attr.attr,
	NULL
};

static struct attribute_group sched_freq_sysfs_group = {
	.attrs = sched_freq_sysfs_attribs,
	.name = "sched_freq",
};

static int cpufreq_sched_policy_init(struct cpufreq_policy *policy)
{
	struct gov_data *gd;
	int ret;

	lockdep_assert_held(&policy->rwsem);

	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd)
		return -ENOMEM;
	policy->governor_data = gd;
	gd->policy = policy;
	raw_spin_lock_init(&gd->fastpath_lock);

	ret = sysfs_create_group(get_governor_parent_kobj(policy),
				 &sched_freq_sysfs_group);
	if (ret)
		goto err_mem;

	/*
	 * Set up schedfreq thread for slow path freq transitions if
	 * required by the driver.
	 */
	if (cpufreq_driver_is_slow()) {
		cpufreq_driver_slow = true;
		gd->task = kthread_create(cpufreq_sched_thread, gd,
					  "kschedfreq:%d",
					  cpumask_first(policy->related_cpus));
		if (IS_ERR_OR_NULL(gd->task)) {
			pr_err("%s: failed to create kschedfreq thread\n",
			       __func__);
			goto err_sysfs;
		}
		trace_printk("init: %s (%d) created", gd->task->comm, task_pid_nr(gd->task));
		get_task_struct(gd->task);
		kthread_bind_mask(gd->task, policy->related_cpus);
		wake_up_process(gd->task);
		init_irq_work(&gd->irq_work, cpufreq_sched_irq_work);
	}
	return 0;

err_sysfs:
	sysfs_remove_group(get_governor_parent_kobj(policy),
			   &sched_freq_sysfs_group);
err_mem:
	kfree(gd);
	return -ENOMEM;
}

static int cpufreq_sched_policy_exit(struct cpufreq_policy *policy)
{
	struct gov_data *gd = policy->governor_data;

	lockdep_assert_held(&policy->rwsem);

	/* Stop the schedfreq thread associated with this policy. */
	if (cpufreq_driver_slow) {
		trace_printk("exit: stopping %s (%d)", gd->task->comm, task_pid_nr(gd->task));
		kthread_stop(gd->task);
		put_task_struct(gd->task);
	}
	sysfs_remove_group(get_governor_parent_kobj(policy),
			   &sched_freq_sysfs_group);
	policy->governor_data = NULL;
	kfree(gd);
	return 0;
}

static int cpufreq_sched_start(struct cpufreq_policy *policy)
{
	struct gov_data *gd = policy->governor_data;
	int cpu;

	lockdep_assert_held(&policy->rwsem);

	/*
	 * The schedfreq static key is managed here so the global schedfreq
	 * lock must be taken - a per-policy lock such as policy->rwsem is
	 * not sufficient.
	 */
	mutex_lock(&gov_enable_lock);

	gd->enabled = 1;
	gd->requested_freq = policy->cur;
	trace_printk("start: %s (%d) policy enabled", gd->task->comm, task_pid_nr(gd->task));

	/*
	 * Set up percpu information. Writing the percpu gd pointer will
	 * enable the fast path if the static key is already enabled.
	 */
	for_each_cpu(cpu, policy->cpus) {
		memset(&per_cpu(cpu_sched_capacity_reqs, cpu), 0,
		       sizeof(struct sched_capacity_reqs));
		per_cpu(cpu_gov_data, cpu) = gd;
	}
	trace_printk("start: %s (%d) per_cpu information setup", gd->task->comm, task_pid_nr(gd->task));

	if (enabled_policies == 0)
		static_key_slow_inc(&__sched_freq);
	enabled_policies++;
	trace_printk("start: %s (%d) enabled_policies=%d", gd->task->comm, task_pid_nr(gd->task), enabled_policies);
	mutex_unlock(&gov_enable_lock);

	return 0;
}

static void dummy(void *info) {}

static int cpufreq_sched_stop(struct cpufreq_policy *policy)
{
	struct gov_data *gd = policy->governor_data;
	int cpu;

	lockdep_assert_held(&policy->rwsem);

	/*
	 * The schedfreq static key is managed here so the global schedfreq
	 * lock must be taken - a per-policy lock such as policy->rwsem is
	 * not sufficient.
	 */
	mutex_lock(&gov_enable_lock);

	/*
	 * Stop new entries into the hot path for all CPUs. This will
	 * potentially affect other policies which are still running but
	 * this is an infrequent operation.
	 */
	static_key_slow_dec(&__sched_freq);
	enabled_policies--;
	trace_printk("stop: %s (%d) enabled_policies=%d", gd->task->comm, task_pid_nr(gd->task), enabled_policies);

	/*
	 * Ensure that all CPUs currently part of this policy are out
	 * of the hot path so that if this policy exits we can free gd.
	 */
	preempt_disable();
	smp_call_function_many(policy->cpus, dummy, NULL, true);
	preempt_enable();

	/*
	 * Other CPUs in other policies may still have the schedfreq
	 * static key enabled. The percpu gd is used to signal which
	 * CPUs are enabled in the sched gov during the hot path.
	 */
	for_each_cpu(cpu, policy->cpus)
		per_cpu(cpu_gov_data, cpu) = NULL;
	trace_printk("stop: %s (%d) per_cpu information reset)", gd->task->comm, task_pid_nr(gd->task));

	/* Pause the slow path for this policy. */
	gd->enabled = 0;
	trace_printk("stop: %s (%d) policy disabled", gd->task->comm, task_pid_nr(gd->task));

	if (enabled_policies)
		static_key_slow_inc(&__sched_freq);
	mutex_unlock(&gov_enable_lock);

	return 0;
}

static int cpufreq_sched_setup(struct cpufreq_policy *policy,
			       unsigned int event)
{
	switch (event) {
	case CPUFREQ_GOV_POLICY_INIT:
		return cpufreq_sched_policy_init(policy);
	case CPUFREQ_GOV_POLICY_EXIT:
		return cpufreq_sched_policy_exit(policy);
	case CPUFREQ_GOV_START:
		return cpufreq_sched_start(policy);
	case CPUFREQ_GOV_STOP:
		return cpufreq_sched_stop(policy);
	case CPUFREQ_GOV_LIMITS:
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

/* Try to make this the default governor */
fs_initcall(cpufreq_sched_init);

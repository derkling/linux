/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/mm.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/highmem.h>
#include <linux/mmu_context.h>
#include <linux/interrupt.h>
#include <linux/capability.h>
#include <linux/completion.h>
#include <linux/kernel_stat.h>
#include <linux/debug_locks.h>
#include <linux/perf_event.h>
#include <linux/security.h>
#include <linux/notifier.h>
#include <linux/profile.h>
#include <linux/freezer.h>
#include <linux/vmalloc.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/pid_namespace.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/timer.h>
#include <linux/rcupdate.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/percpu.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sysctl.h>
#include <linux/syscalls.h>
#include <linux/times.h>
#include <linux/tsacct_kern.h>
#include <linux/kprobes.h>
#include <linux/delayacct.h>
#include <linux/unistd.h>
#include <linux/pagemap.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/ftrace.h>
#include <linux/slab.h>
#include <linux/init_task.h>
#include <linux/binfmts.h>
#include <linux/context_tracking.h>
#include <linux/compiler.h>
#include <linux/cpufreq.h>
#include <linux/syscore_ops.h>
#include <linux/mutex.h>

#include <asm/switch_to.h>
#include <asm/tlb.h>
#include <asm/irq_regs.h>

#include "sched.h"
#include "../workqueue_internal.h"
#include "../smpboot.h"

#include "walt.h"

#define LOAD_AVG_MAX 47742


struct cpu_pstate_pwr {
	unsigned int freq;
	uint32_t power;
};

struct cpu_pwr_stats {
	int cpu;
	long temp;
	struct cpu_pstate_pwr *ptable;
	bool throttling;
	int len;
};

struct cpu_pwr_stats *get_cpu_pwr_stats(void)
{
	return NULL;
}

/* Initial task load. Newly created tasks are assigned this load. */
unsigned int __read_mostly sched_init_task_load_pelt;
unsigned int __read_mostly sched_init_task_load_windows;
unsigned int __read_mostly sysctl_sched_init_task_load_pct = 100;

extern u64 jiffy_to_ktime_ns(u64 *now, u64 *jiffy_ktime_ns);

/*
 * sched_window_stats_policy, sched_account_wait_time, sched_ravg_hist_size,
 * sched_migration_fixup, sched_freq_account_wait_time have a 'sysctl' copy
 * associated with them. This is required for atomic update of those variables
 * when being modifed via sysctl interface.
 *
 * IMPORTANT: Initialize both copies to same value!!
 */

/*
 * Tasks that are runnable continuously for a period greather than
 * sysctl_early_detection_duration can be flagged early as potential
 * high load tasks.
 */
__read_mostly unsigned int sysctl_early_detection_duration = 9500000;

static __read_mostly unsigned int sched_ravg_hist_size = 5;
__read_mostly unsigned int sysctl_sched_ravg_hist_size = 5;

static __read_mostly unsigned int sched_window_stats_policy =
	 WINDOW_STATS_MAX_RECENT_AVG;
__read_mostly unsigned int sysctl_sched_window_stats_policy =
	WINDOW_STATS_MAX_RECENT_AVG;

__read_mostly unsigned int sysctl_sched_new_task_windows = 5;

static __read_mostly unsigned int sched_account_wait_time = 1;
__read_mostly unsigned int sysctl_sched_account_wait_time = 1;

__read_mostly unsigned int sysctl_sched_cpu_high_irqload = (10 * NSEC_PER_MSEC);

#ifdef CONFIG_SCHED_WALT_FREQ_INPUT

static __read_mostly unsigned int sched_migration_fixup = 1;
__read_mostly unsigned int sysctl_sched_migration_fixup = 1;

static __read_mostly unsigned int sched_freq_account_wait_time;
__read_mostly unsigned int sysctl_sched_freq_account_wait_time;

/*
 * For increase, send notification if
 *      freq_required - cur_freq > sysctl_sched_freq_inc_notify
 */
__read_mostly int sysctl_sched_freq_inc_notify = 10 * 1024 * 1024; /* + 10GHz */

/*
 * For decrease, send notification if
 *      cur_freq - freq_required > sysctl_sched_freq_dec_notify
 */
__read_mostly int sysctl_sched_freq_dec_notify = 10 * 1024 * 1024; /* - 10GHz */

static __read_mostly unsigned int sched_io_is_busy;
#endif	/* CONFIG_SCHED_WALT_FREQ_INPUT */

/* 1 -> use PELT based load stats, 0 -> use window-based load stats */
unsigned int __read_mostly sched_use_pelt;

unsigned int max_possible_efficiency = 1024;
unsigned int min_possible_efficiency = 1024;

/*
 * Maximum possible frequency across all cpus. Task demand and cpu
 * capacity (cpu_power) metrics are scaled in reference to it.
 */
unsigned int max_possible_freq = 1;

/*
 * Minimum possible max_freq across all cpus. This will be same as
 * max_possible_freq on homogeneous systems and could be different from
 * max_possible_freq on heterogenous systems. min_max_freq is used to derive
 * capacity (cpu_power) of cpus.
 */
unsigned int min_max_freq = 1;

unsigned int max_capacity = 1024; /* max(rq->capacity) */
unsigned int min_capacity = 1024; /* min(rq->capacity) */
unsigned int max_load_scale_factor = 1024; /* max possible load scale factor */
unsigned int max_possible_capacity = 1024; /* max(rq->max_possible_capacity) */

/* Mask of all CPUs that have  max_possible_capacity */
cpumask_t mpc_mask = CPU_MASK_ALL;

/* Window size (in ns) */
__read_mostly unsigned int sched_ravg_window = 20000000;

/* Min window size (in ns) = 10ms */
#define MIN_SCHED_RAVG_WINDOW 10000000

/* Max window size (in ns) = 1s */
#define MAX_SCHED_RAVG_WINDOW 1000000000

/* Temporarily disable window-stats activity on all cpus */
unsigned int __read_mostly sched_disable_window_stats;

static unsigned int sync_cpu;

static ktime_t ktime_last;
static bool sched_ktime_suspended;

u64 sched_ktime_clock(void)
{
	if (unlikely(sched_ktime_suspended))
		return ktime_to_ns(ktime_last);
	return ktime_get_ns();
}

static void sched_resume(void)
{
	sched_ktime_suspended = false;
}

static int sched_suspend(void)
{
	ktime_last = ktime_get();
	sched_ktime_suspended = true;
	return 0;
}

static struct syscore_ops sched_syscore_ops = {
	.resume	= sched_resume,
	.suspend = sched_suspend
};

static int __init sched_init_ops(void)
{
	register_syscore_ops(&sched_syscore_ops);
	return 0;
}
late_initcall(sched_init_ops);

void clear_ed_task(struct task_struct *p, struct rq *rq)
{
	if (p == rq->ed_task)
		rq->ed_task = NULL;
}

void set_task_last_wake(struct task_struct *p, u64 wallclock)
{
	p->last_wake_ts = wallclock;
}

void set_task_last_switch_out(struct task_struct *p,
					    u64 wallclock)
{
	p->last_switch_out_ts = wallclock;
}

#define EXITING_TASK_MARKER	0xdeaddead

unsigned int max_task_load(void)
{
	if (sched_use_pelt)
		return LOAD_AVG_MAX;

	return sched_ravg_window;
}

/* Use this knob to turn on or off HMP-aware task placement logic */
unsigned int __read_mostly sched_enable_hmp = 1;

/* A cpu can no longer accommodate more tasks if:
 *
 *	rq->nr_running > sysctl_sched_spill_nr_run ||
 *	rq->hmp_stats.cumulative_runnable_avg > sched_spill_load
 */
unsigned int __read_mostly sysctl_sched_spill_nr_run = 10;

/*
 * Control whether or not individual CPU power consumption is used to
 * guide task placement.
 * This sysctl can be set to a default value using boot command line arguments.
 */
unsigned int __read_mostly sysctl_sched_enable_power_aware = 0;

/*
 * This specifies the maximum percent power difference between 2
 * CPUs for them to be considered identical in terms of their
 * power characteristics (i.e. they are in the same power band).
 */
unsigned int __read_mostly sysctl_sched_powerband_limit_pct;

unsigned int __read_mostly sysctl_sched_lowspill_freq;
unsigned int __read_mostly sysctl_sched_pack_freq = UINT_MAX;
/*
 * CPUs with load greater than the sched_spill_load_threshold are not
 * eligible for task placement. When all CPUs in a cluster achieve a
 * load higher than this level, tasks becomes eligible for inter
 * cluster migration.
 */
unsigned int __read_mostly sched_spill_load;
unsigned int __read_mostly sysctl_sched_spill_load_pct = 100;

/*
 * Tasks with demand >= sched_heavy_task will have their
 * window-based demand added to the previous window's CPU
 * time when they wake up, if they have slept for at least
 * one full window. This feature is disabled when the tunable
 * is set to 0 (the default).
 */
#ifdef CONFIG_SCHED_WALT_FREQ_INPUT
unsigned int __read_mostly sysctl_sched_heavy_task_pct;
unsigned int __read_mostly sched_heavy_task;
#endif

/*
 * Tasks whose bandwidth consumption on a cpu is more than
 * sched_upmigrate are considered "big" tasks. Big tasks will be
 * considered for "up" migration, i.e migrating to a cpu with better
 * capacity.
 */
unsigned int __read_mostly sched_upmigrate;
unsigned int __read_mostly sysctl_sched_upmigrate_pct = 80;

/*
 * Big tasks, once migrated, will need to drop their bandwidth
 * consumption to less than sched_downmigrate before they are "down"
 * migrated.
 */
unsigned int __read_mostly sched_downmigrate;
unsigned int __read_mostly sysctl_sched_downmigrate_pct = 60;

/*
 * Tasks whose nice value is > sysctl_sched_upmigrate_min_nice are never
 * considered as "big" tasks.
 */
int __read_mostly sysctl_sched_upmigrate_min_nice = 15;

/*
 * The load scale factor of a CPU gets boosted when its max frequency
 * is restricted due to which the tasks are migrating to higher capacity
 * CPUs early. The sched_upmigrate threshold is auto-upgraded by
 * rq->max_possible_freq/rq->max_freq of a lower capacity CPU.
 */
unsigned int up_down_migrate_scale_factor = 1024;

/*
 * Scheduler boost is a mechanism to temporarily place tasks on CPUs
 * with higher capacity than those where a task would have normally
 * ended up with their load characteristics. Any entity enabling
 * boost is responsible for disabling it as well.
 */
unsigned int sysctl_sched_boost;

/*
 * Scheduler selects and places task to its previous CPU if sleep time is
 * less than sysctl_sched_select_prev_cpu_us.
 */
unsigned int __read_mostly sysctl_sched_select_prev_cpu_us = 2000;

static inline u64 cpu_load(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return scale_load_to_cpu(rq->hmp_stats.cumulative_runnable_avg, cpu);
}

void
inc_rq_hmp_stats(struct rq *rq, struct task_struct *p, int change_cra)
{
	if (change_cra)
		inc_cumulative_runnable_avg(&rq->hmp_stats, p);
}

void
dec_rq_hmp_stats(struct rq *rq, struct task_struct *p, int change_cra)
{
	if (change_cra)
		dec_cumulative_runnable_avg(&rq->hmp_stats, p);
}

void reset_hmp_stats(struct hmp_sched_stats *stats, int reset_cra)
{
	if (reset_cra)
		stats->cumulative_runnable_avg = 0;
}


#ifdef CONFIG_CFS_BANDWIDTH

static inline struct task_group *next_task_group(struct task_group *tg)
{
	tg = list_entry_rcu(tg->list.next, typeof(struct task_group), list);

	return (&tg->list == &task_groups) ? NULL : tg;
}

/* Iterate over all cfs_rq in a cpu */
#define for_each_cfs_rq(cfs_rq, tg, cpu)	\
	for (tg = container_of(&task_groups, struct task_group, list);	\
		((tg = next_task_group(tg)) && (cfs_rq = tg->cfs_rq[cpu]));)

static void reset_cfs_rq_hmp_stats(int cpu, int reset_cra)
{
	struct task_group *tg;
	struct cfs_rq *cfs_rq;

	rcu_read_lock();

	for_each_cfs_rq(cfs_rq, tg, cpu)
		reset_hmp_stats(&cfs_rq->hmp_stats, reset_cra);

	rcu_read_unlock();
}

#else	/* CONFIG_CFS_BANDWIDTH */

static inline void reset_cfs_rq_hmp_stats(int cpu, int reset_cra) { }

#endif	/* CONFIG_CFS_BANDWIDTH */

/*
 * reset_cpu_hmp_stats - reset HMP stats for a cpu
 *	nr_big_tasks
 *	cumulative_runnable_avg (iff reset_cra is true)
 */
void reset_cpu_hmp_stats(int cpu, int reset_cra)
{
	reset_cfs_rq_hmp_stats(cpu, reset_cra);
	reset_hmp_stats(&cpu_rq(cpu)->hmp_stats, reset_cra);
}


#ifdef CONFIG_CFS_BANDWIDTH

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq);

void inc_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
	 struct task_struct *p, int change_cra);
void dec_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
	 struct task_struct *p, int change_cra);

/* Add task's contribution to a cpu' HMP statistics */
static void
_inc_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p, int change_cra)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;

	/*
	 * Although below check is not strictly required  (as
	 * inc/dec_nr_big_task and inc/dec_cumulative_runnable_avg called
	 * from inc_cfs_rq_hmp_stats() have similar checks), we gain a bit on
	 * efficiency by short-circuiting for_each_sched_entity() loop when
	 * !sched_enable_hmp || sched_disable_window_stats
	 */
	if (!sched_enable_hmp || sched_disable_window_stats)
		return;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		inc_cfs_rq_hmp_stats(cfs_rq, p, change_cra);
		if (cfs_rq_throttled(cfs_rq))
			break;
	}

	/* Update rq->hmp_stats only if we didn't find any throttled cfs_rq */
	if (!se)
		inc_rq_hmp_stats(rq, p, change_cra);
}

/* Remove task's contribution from a cpu' HMP statistics */
static void
_dec_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p, int change_cra)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;

	/* See comment on efficiency in _inc_hmp_sched_stats_fair */
	if (!sched_enable_hmp || sched_disable_window_stats)
		return;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		dec_cfs_rq_hmp_stats(cfs_rq, p, change_cra);
		if (cfs_rq_throttled(cfs_rq))
			break;
	}

	/* Update rq->hmp_stats only if we didn't find any throttled cfs_rq */
	if (!se)
		dec_rq_hmp_stats(rq, p, change_cra);
}

void inc_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p)
{
	_inc_hmp_sched_stats_fair(rq, p, 1);
}

void dec_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p)
{
	_dec_hmp_sched_stats_fair(rq, p, 1);
}

void fixup_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p,
				       u32 new_task_load)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;
	s64 task_load_delta = (s64)new_task_load - task_load(p);
	u32 ntl = new_task_load;
	u32 tlp =  task_load(p);

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);

		fixup_cumulative_runnable_avg(&cfs_rq->hmp_stats, p,
					      task_load_delta);
		if (cfs_rq_throttled(cfs_rq))
			break;
	}

	/* Fix up rq->hmp_stats only if we didn't find any throttled cfs_rq */
	if (!se) {
		fixup_cumulative_runnable_avg(&rq->hmp_stats, p,
					      task_load_delta);
	}
}

int task_will_be_throttled(struct task_struct *p);

#else	/* CONFIG_CFS_BANDWIDTH */

void
inc_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p)
{
	inc_cumulative_runnable_avg(&rq->hmp_stats, p);
}

void
dec_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p)
{
	dec_cumulative_runnable_avg(&rq->hmp_stats, p);
}
void
fixup_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p,
			   u32 new_task_load)
{
	s64 task_load_delta = (s64)new_task_load - task_load(p);

	fixup_cumulative_runnable_avg(&rq->hmp_stats, p, task_load_delta);
}

static inline int task_will_be_throttled(struct task_struct *p)
{
	return 0;
}


#endif	/* CONFIG_CFS_BANDWIDTH */

DEFINE_MUTEX(policy_mutex);

#ifdef CONFIG_SCHED_WALT_FREQ_INPUT
static inline int invalid_value_freq_input(unsigned int *data)
{
	if (data == &sysctl_sched_migration_fixup)
		return !(*data == 0 || *data == 1);

	if (data == &sysctl_sched_freq_account_wait_time)
		return !(*data == 0 || *data == 1);

	return 0;
}
#else
static inline int invalid_value_freq_input(unsigned int *data)
{
	return 0;
}
#endif

static inline int invalid_value(unsigned int *data)
{
	unsigned int val = *data;

	if (data == &sysctl_sched_ravg_hist_size)
		return (val < 2 || val > RAVG_HIST_SIZE_MAX);

	if (data == &sysctl_sched_window_stats_policy)
		return val >= WINDOW_STATS_INVALID_POLICY;

	if (data == &sysctl_sched_account_wait_time)
		return !(val == 0 || val == 1);

	return invalid_value_freq_input(data);
}

/*
 * Handle "atomic" update of sysctl_sched_window_stats_policy,
 * sysctl_sched_ravg_hist_size, sysctl_sched_account_wait_time and
 * sched_freq_legacy_mode variables.
 */
int sched_window_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;
	unsigned int *data = (unsigned int *)table->data;
	unsigned int old_val;

	if (!sched_enable_hmp)
		return -EINVAL;

	mutex_lock(&policy_mutex);

	old_val = *data;

	ret = proc_dointvec(table, write, buffer, lenp, ppos);
	if (ret || !write || (write && (old_val == *data)))
		goto done;

	if (invalid_value(data)) {
		*data = old_val;
		ret = -EINVAL;
		goto done;
	}

	reset_all_window_stats(0, 0);

done:
	mutex_unlock(&policy_mutex);

	return ret;
}

/*
 * Convert percentage value into absolute form. This will avoid div() operation
 * in fast path, to convert task load in percentage scale.
 */
int sched_hmp_proc_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;
	unsigned int old_val;
	unsigned int *data = (unsigned int *)table->data;
	int update_min_nice = 0;

	mutex_lock(&policy_mutex);

	old_val = *data;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write || !sched_enable_hmp)
		goto done;

	if (write && (old_val == *data))
		goto done;

	if (data == (unsigned int *)&sysctl_sched_upmigrate_min_nice) {
		if ((*(int *)data) < -20 || (*(int *)data) > 19) {
			*data = old_val;
			ret = -EINVAL;
			goto done;
		}
		update_min_nice = 1;
	} else {
		/* all tunables other than min_nice are in percentage */
		if (sysctl_sched_downmigrate_pct >
		    sysctl_sched_upmigrate_pct || *data > 100) {
			*data = old_val;
			ret = -EINVAL;
			goto done;
		}
	}

	/*
	 * Big task tunable change will need to re-classify tasks on
	 * runqueue as big and set their counters appropriately.
	 * sysctl interface affects secondary variables (*_pct), which is then
	 * "atomically" carried over to the primary variables. Atomic change
	 * includes taking runqueue lock of all online cpus and re-initiatizing
	 * their big counter values based on changed criteria.
	 */
	if ((data == &sysctl_sched_upmigrate_pct || update_min_nice)) {
		get_online_cpus();
		/*pre_big_task_count_change(cpu_online_mask);*/
	}

	/*set_hmp_defaults();*/

	if ((data == &sysctl_sched_upmigrate_pct || update_min_nice)) {
		/*post_big_task_count_change(cpu_online_mask);*/
		put_online_cpus();
	}

done:
	mutex_unlock(&policy_mutex);
	return ret;
}

/* Return task demand in percentage scale */
unsigned int pct_task_load(struct task_struct *p)
{
	unsigned int load;

	load = div64_u64((u64)task_load(p) * 100, (u64)max_task_load());

	return load;
}

#ifdef CONFIG_CFS_BANDWIDTH

void init_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq)
{
	cfs_rq->hmp_stats.cumulative_runnable_avg = 0;
}

void inc_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
		 struct task_struct *p, int change_cra)
{
	if (change_cra)
		inc_cumulative_runnable_avg(&cfs_rq->hmp_stats, p);
}

void dec_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
		 struct task_struct *p, int change_cra)
{
	if (change_cra)
		dec_cumulative_runnable_avg(&cfs_rq->hmp_stats, p);
}

void inc_throttled_cfs_rq_hmp_stats(struct hmp_sched_stats *stats,
			 struct cfs_rq *cfs_rq)
{
	stats->cumulative_runnable_avg +=
				cfs_rq->hmp_stats.cumulative_runnable_avg;
}

void dec_throttled_cfs_rq_hmp_stats(struct hmp_sched_stats *stats,
				 struct cfs_rq *cfs_rq)
{
	stats->cumulative_runnable_avg -=
				cfs_rq->hmp_stats.cumulative_runnable_avg;

	BUG_ON((s64)stats->cumulative_runnable_avg < 0);
}

#else	/* CONFIG_CFS_BANDWIDTH */

inline void inc_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
	 struct task_struct *p, int change_cra) { }

inline void dec_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
	 struct task_struct *p, int change_cra) { }

#endif	/* CONFIG_CFS_BANDWIDTH */

static inline int exiting_task(struct task_struct *p)
{
	return (p->ravg.sum_history[0] == EXITING_TASK_MARKER);
}

static int __init set_sched_ravg_window(char *str)
{
	get_option(&str, &sched_ravg_window);

	sched_use_pelt = (sched_ravg_window < MIN_SCHED_RAVG_WINDOW ||
				sched_ravg_window > MAX_SCHED_RAVG_WINDOW);

	return 0;
}

early_param("sched_ravg_window", set_sched_ravg_window);

static inline void
update_window_start(struct rq *rq, u64 wallclock)
{
	s64 delta;
	int nr_windows;

	delta = wallclock - rq->window_start;
	BUG_ON(delta < 0);
	if (delta < sched_ravg_window)
		return;

	nr_windows = div64_u64(delta, sched_ravg_window);
	rq->window_start += (u64)nr_windows * (u64)sched_ravg_window;
}

static inline u64 scale_exec_time(u64 delta, struct rq *rq)
{
	unsigned int cur_freq = rq->cur_freq;
	int sf;

	if (unlikely(cur_freq > max_possible_freq))
		cur_freq = rq->max_possible_freq;

	/* round up div64 */
	delta = div64_u64(delta * cur_freq + max_possible_freq - 1,
			  max_possible_freq);

	sf = DIV_ROUND_UP(rq->efficiency * 1024, max_possible_efficiency);

	delta *= sf;
	delta >>= 10;

	return delta;
}

#ifdef CONFIG_SCHED_WALT_FREQ_INPUT

static inline int cpu_is_waiting_on_io(struct rq *rq)
{
	if (!sched_io_is_busy)
		return 0;

	return atomic_read(&rq->nr_iowait);
}

/* Does freq_required sufficiently exceed or fall behind cur_freq? */
static inline int
nearly_same_freq(unsigned int cur_freq, unsigned int freq_required)
{
	int delta = freq_required - cur_freq;

	if (freq_required > cur_freq)
		return delta < sysctl_sched_freq_inc_notify;

	delta = -delta;

	return delta < sysctl_sched_freq_dec_notify;
}

/* Convert busy time to frequency equivalent */
static inline unsigned int load_to_freq(struct rq *rq, u64 load)
{
	unsigned int freq;

	load = scale_load_to_cpu(load, cpu_of(rq));
	load *= 128;
	load = div64_u64(load, max_task_load());

	freq = load * rq->max_possible_freq;
	freq /= 128;

	return freq;
}


static int account_busy_for_cpu_time(struct rq *rq, struct task_struct *p,
				     u64 irqtime, int event)
{
	if (is_idle_task(p)) {
		/* TASK_WAKE && TASK_MIGRATE is not possible on idle task! */
		if (event == PICK_NEXT_TASK)
			return 0;

		/* PUT_PREV_TASK, TASK_UPDATE && IRQ_UPDATE are left */
		return irqtime || cpu_is_waiting_on_io(rq);
	}

	if (event == TASK_WAKE)
		return 0;

	if (event == PUT_PREV_TASK || event == IRQ_UPDATE ||
					 event == TASK_UPDATE)
		return 1;

	/* Only TASK_MIGRATE && PICK_NEXT_TASK left */
	return sched_freq_account_wait_time;
}

static inline int
heavy_task_wakeup(struct task_struct *p, struct rq *rq, int event)
{
	u32 task_demand = p->ravg.demand;

	if (!sched_heavy_task || event != TASK_WAKE ||
	    task_demand < sched_heavy_task || exiting_task(p))
		return 0;

	if (p->ravg.mark_start > rq->window_start)
		return 0;

	/* has a full window elapsed since task slept? */
	return (rq->window_start - p->ravg.mark_start > sched_ravg_window);
}

static inline bool is_new_task(struct task_struct *p)
{
	return p->ravg.active_windows < sysctl_sched_new_task_windows;
}

/*
 * Account cpu activity in its busy time counters (rq->curr/prev_runnable_sum)
 */
static void update_cpu_busy_time(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock, u64 irqtime)
{
	int new_window, nr_full_windows = 0;
	int p_is_curr_task = (p == rq->curr);
	u64 mark_start = p->ravg.mark_start;
	u64 window_start = rq->window_start;
	u32 window_size = sched_ravg_window;
	u64 delta;
	bool new_task;

	new_window = mark_start < window_start;
	if (new_window) {
		nr_full_windows = div64_u64((window_start - mark_start),
						window_size);
		if (p->ravg.active_windows < USHRT_MAX)
			p->ravg.active_windows++;
	}

	new_task = is_new_task(p);

	/* Handle per-task window rollover. We don't care about the idle
	 * task or exiting tasks. */
	if (new_window && !is_idle_task(p) && !exiting_task(p)) {
		u32 curr_window = 0;

		if (!nr_full_windows)
			curr_window = p->ravg.curr_window;

		p->ravg.prev_window = curr_window;
		p->ravg.curr_window = 0;
	}

	if (!account_busy_for_cpu_time(rq, p, irqtime, event)) {
		/* account_busy_for_cpu_time() = 0, so no update to the
		 * task's current window needs to be made. This could be
		 * for example
		 *
		 *   - a wakeup event on a task within the current
		 *     window (!new_window below, no action required),
		 *   - switching to a new task from idle (PICK_NEXT_TASK)
		 *     in a new window where irqtime is 0 and we aren't
		 *     waiting on IO */

		if (!new_window)
			return;

		/* A new window has started. The RQ demand must be rolled
		 * over if p is the current task. */
		if (p_is_curr_task) {
			u64 prev_sum = 0, nt_prev_sum = 0;

			/* p is either idle task or an exiting task */
			if (!nr_full_windows) {
				prev_sum = rq->curr_runnable_sum;
				nt_prev_sum = rq->nt_curr_runnable_sum;
			}

			rq->prev_runnable_sum = prev_sum;
			rq->curr_runnable_sum = 0;
			rq->nt_prev_runnable_sum = nt_prev_sum;
			rq->nt_curr_runnable_sum = 0;

		} else if (heavy_task_wakeup(p, rq, event)) {
			/* A new window has started. If p is a waking
			 * heavy task its prev_window contribution is faked
			 * to be its window-based demand. Note that this can
			 * introduce phantom load into the system depending
			 * on the window policy and task behavior. This feature
			 * can be controlled via the sched_heavy_task
			 * tunable. */
			p->ravg.prev_window = p->ravg.demand;
			rq->prev_runnable_sum += p->ravg.demand;
			if (new_task)
				rq->nt_prev_runnable_sum += p->ravg.demand;
		}

		return;
	}

	if (!new_window) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. No rollover
		 * since we didn't start a new window. An example of this is
		 * when a task starts execution and then sleeps within the
		 * same window. */

		if (!irqtime || !is_idle_task(p) || cpu_is_waiting_on_io(rq))
			delta = wallclock - mark_start;
		else
			delta = irqtime;
		delta = scale_exec_time(delta, rq);
		rq->curr_runnable_sum += delta;
		if (new_task)
			rq->nt_curr_runnable_sum += delta;
		if (!is_idle_task(p) && !exiting_task(p))
			p->ravg.curr_window += delta;

		return;
	}

	if (!p_is_curr_task) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has also started, but p is not the current task, so the
		 * window is not rolled over - just split up and account
		 * as necessary into curr and prev. The window is only
		 * rolled over when a new window is processed for the current
		 * task.
		 *
		 * Irqtime can't be accounted by a task that isn't the
		 * currently running task. */

		if (!nr_full_windows) {
			/* A full window hasn't elapsed, account partial
			 * contribution to previous completed window. */
			delta = scale_exec_time(window_start - mark_start, rq);
			if (!exiting_task(p))
				p->ravg.prev_window += delta;
		} else {
			/* Since at least one full window has elapsed,
			 * the contribution to the previous window is the
			 * full window (window_size). */
			delta = scale_exec_time(window_size, rq);
			if (!exiting_task(p))
				p->ravg.prev_window = delta;
		}
		rq->prev_runnable_sum += delta;
		if (new_task)
			rq->nt_prev_runnable_sum += delta;

		/* Account piece of busy time in the current window. */
		delta = scale_exec_time(wallclock - window_start, rq);
		rq->curr_runnable_sum += delta;
		if (new_task)
			rq->nt_curr_runnable_sum += delta;
		if (!exiting_task(p))
			p->ravg.curr_window = delta;

		return;
	}

	if (!irqtime || !is_idle_task(p) || cpu_is_waiting_on_io(rq)) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has started and p is the current task so rollover is
		 * needed. If any of these three above conditions are true
		 * then this busy time can't be accounted as irqtime.
		 *
		 * Busy time for the idle task or exiting tasks need not
		 * be accounted.
		 *
		 * An example of this would be a task that starts execution
		 * and then sleeps once a new window has begun. */

		if (!nr_full_windows) {
			/* A full window hasn't elapsed, account partial
			 * contribution to previous completed window. */
			delta = scale_exec_time(window_start - mark_start, rq);
			if (!is_idle_task(p) && !exiting_task(p))
				p->ravg.prev_window += delta;

			rq->nt_prev_runnable_sum = rq->nt_curr_runnable_sum;
			if (new_task)
				rq->nt_prev_runnable_sum += delta;

			delta += rq->curr_runnable_sum;
		} else {
			/* Since at least one full window has elapsed,
			 * the contribution to the previous window is the
			 * full window (window_size). */
			delta = scale_exec_time(window_size, rq);
			if (!is_idle_task(p) && !exiting_task(p))
				p->ravg.prev_window = delta;

			if (new_task)
				rq->nt_prev_runnable_sum = delta;
			else
				rq->nt_prev_runnable_sum = 0;
		}
		/*
		 * Rollover for normal runnable sum is done here by overwriting
		 * the values in prev_runnable_sum and curr_runnable_sum.
		 * Rollover for new task runnable sum has completed by previous
		 * if-else statement.
		 */
		rq->prev_runnable_sum = delta;

		/* Account piece of busy time in the current window. */
		delta = scale_exec_time(wallclock - window_start, rq);
		rq->curr_runnable_sum = delta;
		if (new_task)
			rq->nt_curr_runnable_sum = delta;
		else
			rq->nt_curr_runnable_sum = 0;
		if (!is_idle_task(p) && !exiting_task(p))
			p->ravg.curr_window = delta;

		return;
	}

	if (irqtime) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has started and p is the current task so rollover is
		 * needed. The current task must be the idle task because
		 * irqtime is not accounted for any other task.
		 *
		 * Irqtime will be accounted each time we process IRQ activity
		 * after a period of idleness, so we know the IRQ busy time
		 * started at wallclock - irqtime. */

		BUG_ON(!is_idle_task(p));
		mark_start = wallclock - irqtime;

		/* Roll window over. If IRQ busy time was just in the current
		 * window then that is all that need be accounted. */
		rq->prev_runnable_sum = rq->curr_runnable_sum;
		rq->nt_prev_runnable_sum = rq->nt_curr_runnable_sum;
		rq->nt_curr_runnable_sum = 0;
		if (mark_start > window_start) {
			rq->curr_runnable_sum = scale_exec_time(irqtime, rq);
			return;
		}

		/* The IRQ busy time spanned multiple windows. Process the
		 * busy time preceding the current window start first. */
		delta = window_start - mark_start;
		if (delta > window_size)
			delta = window_size;
		delta = scale_exec_time(delta, rq);
		rq->prev_runnable_sum += delta;

		/* Process the remaining IRQ busy time in the current window. */
		delta = wallclock - window_start;
		rq->curr_runnable_sum = scale_exec_time(delta, rq);

		return;
	}

	BUG();
}

u32 __weak get_freq_max_load(int cpu, u32 freq)
{
	/* 100% by default */
	return 100;
}


struct freq_max_load_entry {
	/* The maximum load which has accounted governor's headroom. */
	u64 hdemand;
};

struct freq_max_load {
	struct rcu_head rcu;
	int length;
	struct freq_max_load_entry freqs[0];
};

DEFINE_PER_CPU(struct freq_max_load *, freq_max_load);
static DEFINE_SPINLOCK(freq_max_load_lock);

int sched_update_freq_max_load(const cpumask_t *cpumask)
{
	int i, cpu, ret;
	unsigned int freq;
	struct cpu_pstate_pwr *costs;
	struct cpu_pwr_stats *per_cpu_info = get_cpu_pwr_stats();
	struct freq_max_load *max_load, *old_max_load;
	struct freq_max_load_entry *entry;
	u64 max_demand_capacity, max_demand;
	unsigned long flags;
	u32 hfreq;
	int hpct;

	if (!per_cpu_info || !sysctl_sched_enable_power_aware)
		return 0;

	spin_lock_irqsave(&freq_max_load_lock, flags);
	max_demand_capacity = div64_u64(max_task_load(), max_possible_capacity);
	for_each_cpu(cpu, cpumask) {
		if (!per_cpu_info[cpu].ptable) {
			ret = -EINVAL;
			goto fail;
		}

		old_max_load = rcu_dereference(per_cpu(freq_max_load, cpu));

		/*
		 * allocate len + 1 and leave the last power cost as 0 for
		 * power_cost() can stop iterating index when
		 * per_cpu_info[cpu].len > len of max_load due to race between
		 * cpu power stats update and get_cpu_pwr_stats().
		 */
		max_load = kzalloc(sizeof(struct freq_max_load) +
				   sizeof(struct freq_max_load_entry) *
				   (per_cpu_info[cpu].len + 1), GFP_ATOMIC);
		if (unlikely(!max_load)) {
			ret = -ENOMEM;
			goto fail;
		}

		max_load->length = per_cpu_info[cpu].len;

		max_demand = max_demand_capacity *
			     cpu_rq(cpu)->max_possible_capacity;

		i = 0;
		costs = per_cpu_info[cpu].ptable;
		while (costs[i].freq) {
			entry = &max_load->freqs[i];
			freq = costs[i].freq;
			hpct = get_freq_max_load(cpu, freq);
			if (hpct <= 0 && hpct > 100)
				hpct = 100;
			hfreq = div64_u64((u64)freq * hpct , 100);
			entry->hdemand =
			    div64_u64(max_demand * hfreq,
				      cpu_rq(cpu)->max_possible_freq);
			i++;
		}

		rcu_assign_pointer(per_cpu(freq_max_load, cpu), max_load);
		if (old_max_load)
			kfree_rcu(old_max_load, rcu);
	}

	spin_unlock_irqrestore(&freq_max_load_lock, flags);
	return 0;

fail:
	for_each_cpu(cpu, cpumask) {
		max_load = rcu_dereference(per_cpu(freq_max_load, cpu));
		if (max_load) {
			rcu_assign_pointer(per_cpu(freq_max_load, cpu), NULL);
			kfree_rcu(max_load, rcu);
		}
	}

	spin_unlock_irqrestore(&freq_max_load_lock, flags);
	return ret;
}

#else	/* CONFIG_SCHED_WALT_FREQ_INPUT */

static inline void update_cpu_busy_time(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock, u64 irqtime)
{
}

#endif	/* CONFIG_SCHED_WALT_FREQ_INPUT */

static int account_busy_for_task_demand(struct task_struct *p, int event)
{
	/* No need to bother updating task demand for exiting tasks
	 * or the idle task. */
	if (exiting_task(p) || is_idle_task(p))
		return 0;

	/* When a task is waking up it is completing a segment of non-busy
	 * time. Likewise, if wait time is not treated as busy time, then
	 * when a task begins to run or is migrated, it is not running and
	 * is completing a segment of non-busy time. */
	if (event == TASK_WAKE || (!sched_account_wait_time &&
			 (event == PICK_NEXT_TASK || event == TASK_MIGRATE)))
		return 0;

	return 1;
}

/*
 * Called when new window is starting for a task, to record cpu usage over
 * recently concluded window(s). Normally 'samples' should be 1. It can be > 1
 * when, say, a real-time task runs without preemption for several windows at a
 * stretch.
 */
static void update_history(struct rq *rq, struct task_struct *p,
			 u32 runtime, int samples, int event)
{
	u32 *hist = &p->ravg.sum_history[0];
	int ridx, widx;
	u32 max = 0, avg, demand;
	u64 sum = 0;

	/* Ignore windows where task had no activity */
	if (!runtime || is_idle_task(p) || exiting_task(p) || !samples)
			goto done;

	/* Push new 'runtime' value onto stack */
	widx = sched_ravg_hist_size - 1;
	ridx = widx - samples;
	for (; ridx >= 0; --widx, --ridx) {
		hist[widx] = hist[ridx];
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	for (widx = 0; widx < samples && widx < sched_ravg_hist_size; widx++) {
		hist[widx] = runtime;
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	p->ravg.sum = 0;

	if (sched_window_stats_policy == WINDOW_STATS_RECENT) {
		demand = runtime;
	} else if (sched_window_stats_policy == WINDOW_STATS_MAX) {
		demand = max;
	} else {
		avg = div64_u64(sum, sched_ravg_hist_size);
		if (sched_window_stats_policy == WINDOW_STATS_AVG)
			demand = avg;
		else
			demand = max(avg, runtime);
	}

	/*
	 * A throttled deadline sched class task gets dequeued without
	 * changing p->on_rq. Since the dequeue decrements hmp stats
	 * avoid decrementing it here again.
	 */
	if (task_on_rq_queued(p) && (!task_has_dl_policy(p) ||
						!p->dl.dl_throttled))
		p->sched_class->fixup_hmp_sched_stats(rq, p, demand);

	p->ravg.demand = demand;

done:
	return;
	/*trace_sched_update_history(rq, p, runtime, samples, event);*/
}

static void add_to_task_demand(struct rq *rq, struct task_struct *p,
				u64 delta)
{
	delta = scale_exec_time(delta, rq);
	p->ravg.sum += delta;
	if (unlikely(p->ravg.sum > sched_ravg_window))
		p->ravg.sum = sched_ravg_window;
}

/*
 * Account cpu demand of task and/or update task's cpu demand history
 *
 * ms = p->ravg.mark_start;
 * wc = wallclock
 * ws = rq->window_start
 *
 * Three possibilities:
 *
 *	a) Task event is contained within one window.
 *		window_start < mark_start < wallclock
 *
 *		ws   ms  wc
 *		|    |   |
 *		V    V   V
 *		|---------------|
 *
 *	In this case, p->ravg.sum is updated *iff* event is appropriate
 *	(ex: event == PUT_PREV_TASK)
 *
 *	b) Task event spans two windows.
 *		mark_start < window_start < wallclock
 *
 *		ms   ws   wc
 *		|    |    |
 *		V    V    V
 *		-----|-------------------
 *
 *	In this case, p->ravg.sum is updated with (ws - ms) *iff* event
 *	is appropriate, then a new window sample is recorded followed
 *	by p->ravg.sum being set to (wc - ws) *iff* event is appropriate.
 *
 *	c) Task event spans more than two windows.
 *
 *		ms ws_tmp			   ws  wc
 *		|  |				   |   |
 *		V  V				   V   V
 *		---|-------|-------|-------|-------|------
 *		   |				   |
 *		   |<------ nr_full_windows ------>|
 *
 *	In this case, p->ravg.sum is updated with (ws_tmp - ms) first *iff*
 *	event is appropriate, window sample of p->ravg.sum is recorded,
 *	'nr_full_window' samples of window_size is also recorded *iff*
 *	event is appropriate and finally p->ravg.sum is set to (wc - ws)
 *	*iff* event is appropriate.
 *
 * IMPORTANT : Leave p->ravg.mark_start unchanged, as update_cpu_busy_time()
 * depends on it!
 */
static void update_task_demand(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock)
{
	u64 mark_start = p->ravg.mark_start;
	u64 delta, window_start = rq->window_start;
	int new_window, nr_full_windows;
	u32 window_size = sched_ravg_window;

	new_window = mark_start < window_start;
	if (!account_busy_for_task_demand(p, event)) {
		if (new_window)
			/* If the time accounted isn't being accounted as
			 * busy time, and a new window started, only the
			 * previous window need be closed out with the
			 * pre-existing demand. Multiple windows may have
			 * elapsed, but since empty windows are dropped,
			 * it is not necessary to account those. */
			update_history(rq, p, p->ravg.sum, 1, event);
		return;
	}

	if (!new_window) {
		/* The simple case - busy time contained within the existing
		 * window. */
		add_to_task_demand(rq, p, wallclock - mark_start);
		return;
	}

	/* Busy time spans at least two windows. Temporarily rewind
	 * window_start to first window boundary after mark_start. */
	delta = window_start - mark_start;
	nr_full_windows = div64_u64(delta, window_size);
	window_start -= (u64)nr_full_windows * (u64)window_size;

	/* Process (window_start - mark_start) first */
	add_to_task_demand(rq, p, window_start - mark_start);

	/* Push new sample(s) into task's demand history */
	update_history(rq, p, p->ravg.sum, 1, event);
	if (nr_full_windows)
		update_history(rq, p, scale_exec_time(window_size, rq),
			       nr_full_windows, event);

	/* Roll window_start back to current to process any remainder
	 * in current window. */
	window_start += (u64)nr_full_windows * (u64)window_size;

	/* Process (wallclock - window_start) next */
	mark_start = window_start;
	add_to_task_demand(rq, p, wallclock - mark_start);
}

/* Reflect task activity on its demand and cpu's busy time statistics */
void update_task_ravg(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock, u64 irqtime)
{
	if (sched_use_pelt || !rq->window_start || sched_disable_window_stats)
		return;

	lockdep_assert_held(&rq->lock);

	update_window_start(rq, wallclock);

	if (!p->ravg.mark_start)
		goto done;

	update_task_demand(p, rq, event, wallclock);
	update_cpu_busy_time(p, rq, event, wallclock, irqtime);

done:
/*	trace_sched_update_task_ravg(p, rq, event, wallclock, irqtime);*/

	p->ravg.mark_start = wallclock;
}

void sched_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags, nr_windows;
	u64 cur_jiffies_ts;

	raw_spin_lock_irqsave(&rq->lock, flags);

	/*
	 * cputime (wallclock) uses sched_clock so use the same here for
	 * consistency.
	 */
	delta += sched_clock() - wallclock;
	cur_jiffies_ts = get_jiffies_64();

	if (is_idle_task(curr))
		update_task_ravg(curr, rq, IRQ_UPDATE, ktime_get_ns(),
				 delta);

	nr_windows = cur_jiffies_ts - rq->irqload_ts;

	if (nr_windows) {
		if (nr_windows < 10) {
			/* Decay CPU's irqload by 3/4 for each window. */
			rq->avg_irqload *= (3 * nr_windows);
			rq->avg_irqload = div64_u64(rq->avg_irqload,
						    4 * nr_windows);
		} else {
			rq->avg_irqload = 0;
		}
		rq->avg_irqload += rq->cur_irqload;
		rq->cur_irqload = 0;
	}

	rq->cur_irqload += delta;
	rq->irqload_ts = cur_jiffies_ts;
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

unsigned long __weak arch_get_cpu_efficiency(int cpu)
{
	return SCHED_LOAD_SCALE;
}

void init_cpu_efficiency(void)
{
	int i, efficiency;
	unsigned int max = 0, min = UINT_MAX;

	if (!sched_enable_hmp)
		return;

	for_each_possible_cpu(i) {
		efficiency = arch_get_cpu_efficiency(i);
		cpu_rq(i)->efficiency = efficiency;

		if (efficiency > max)
			max = efficiency;
		if (efficiency < min)
			min = efficiency;
	}

	/* BUG_ON(!max || !min); */

	if (max)
		max_possible_efficiency = max;

	if (min)
		min_possible_efficiency = min;
}

static void reset_task_stats(struct task_struct *p)
{
	u32 sum = 0;

	if (exiting_task(p))
		sum = EXITING_TASK_MARKER;

	memset(&p->ravg, 0, sizeof(struct ravg));
	/* Retain EXITING_TASK marker */
	p->ravg.sum_history[0] = sum;
}

void mark_task_starting(struct task_struct *p)
{
	u64 wallclock;
	struct rq *rq = task_rq(p);

	if (!rq->window_start || sched_disable_window_stats) {
		reset_task_stats(p);
		return;
	}

	wallclock = ktime_get_ns();
	p->ravg.mark_start = p->last_wake_ts = wallclock;
	p->last_switch_out_ts = 0;
}

void set_window_start(struct rq *rq)
{
	int cpu = cpu_of(rq);
	struct rq *sync_rq = cpu_rq(sync_cpu);

	if (rq->window_start || !sched_enable_hmp)
		return;

	if (cpu == sync_cpu) {
		rq->window_start = ktime_get_ns();
	} else {
		raw_spin_unlock(&rq->lock);
		double_rq_lock(rq, sync_rq);
		rq->window_start = cpu_rq(sync_cpu)->window_start;
#ifdef CONFIG_SCHED_WALT_FREQ_INPUT
		rq->curr_runnable_sum = rq->prev_runnable_sum = 0;
		rq->nt_curr_runnable_sum = rq->nt_prev_runnable_sum = 0;
#endif
		raw_spin_unlock(&sync_rq->lock);
	}

	rq->curr->ravg.mark_start = rq->window_start;
}

void migrate_sync_cpu(int cpu)
{
	if (cpu == sync_cpu)
		sync_cpu = smp_processor_id();
}

static void reset_all_task_stats(void)
{
	struct task_struct *g, *p;

	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		reset_task_stats(p);
	}  while_each_thread(g, p);
	read_unlock(&tasklist_lock);
}



extern void enqueue_task(struct rq *rq, struct task_struct *p, int flags);

extern void dequeue_task(struct rq *rq, struct task_struct *p, int flags);

/*
 * sched_exit() - Set EXITING_TASK_MARKER in task's ravg.demand field
 *
 * Stop accounting (exiting) task's future cpu usage
 *
 * We need this so that reset_all_windows_stats() can function correctly.
 * reset_all_window_stats() depends on do_each_thread/for_each_thread task
 * iterators to reset *all* task's statistics. Exiting tasks however become
 * invisible to those iterators. sched_exit() is called on a exiting task prior
 * to being removed from task_list, which will let reset_all_window_stats()
 * function correctly.
 */
void sched_exit(struct task_struct *p)
{
	unsigned long flags;
	int cpu = get_cpu();
	struct rq *rq = cpu_rq(cpu);
	u64 wallclock;

	raw_spin_lock_irqsave(&rq->lock, flags);
	/* rq->curr == p */
	wallclock = ktime_get_ns();
	update_task_ravg(rq->curr, rq, TASK_UPDATE, wallclock, 0);
	dequeue_task(rq, p, 0);
	reset_task_stats(p);
	p->ravg.mark_start = wallclock;
	p->ravg.sum_history[0] = EXITING_TASK_MARKER;
	enqueue_task(rq, p, 0);
	clear_ed_task(p, rq);
	raw_spin_unlock_irqrestore(&rq->lock, flags);

	put_cpu();
}

static void disable_window_stats(void)
{
	unsigned long flags;
	int i;

	local_irq_save(flags);
	for_each_possible_cpu(i)
		raw_spin_lock(&cpu_rq(i)->lock);

	sched_disable_window_stats = 1;

	for_each_possible_cpu(i)
		raw_spin_unlock(&cpu_rq(i)->lock);

	local_irq_restore(flags);
}

/* Called with all cpu's rq->lock held */
static void enable_window_stats(void)
{
	sched_disable_window_stats = 0;

}

enum reset_reason_code {
	WINDOW_CHANGE,
	POLICY_CHANGE,
	ACCOUNT_WAIT_TIME_CHANGE,
	HIST_SIZE_CHANGE,
	MIGRATION_FIXUP_CHANGE,
	FREQ_ACCOUNT_WAIT_TIME_CHANGE
};

const char *sched_window_reset_reasons[] = {
	"WINDOW_CHANGE",
	"POLICY_CHANGE",
	"ACCOUNT_WAIT_TIME_CHANGE",
	"HIST_SIZE_CHANGE",
	"MIGRATION_FIXUP_CHANGE",
	"FREQ_ACCOUNT_WAIT_TIME_CHANGE"};

/* Called with IRQs enabled */
void reset_all_window_stats(u64 window_start, unsigned int window_size)
{
	int cpu;
	unsigned long flags;
	/*u64 start_ts = ktime_get_ns();*/
	int reason = WINDOW_CHANGE;
	unsigned int old = 0, new = 0;

	disable_window_stats();

	reset_all_task_stats();

	local_irq_save(flags);

	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		raw_spin_lock(&rq->lock);
	}

	if (window_size) {
		sched_ravg_window = window_size * TICK_NSEC;
		/*set_hmp_defaults();*/
	}

	enable_window_stats();

	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		if (window_start)
			rq->window_start = window_start;
#ifdef CONFIG_SCHED_WALT_FREQ_INPUT
		rq->curr_runnable_sum = rq->prev_runnable_sum = 0;
		rq->nt_curr_runnable_sum = rq->nt_prev_runnable_sum = 0;
#endif
		reset_cpu_hmp_stats(cpu, 1);
	}

	if (sched_window_stats_policy != sysctl_sched_window_stats_policy) {
		reason = POLICY_CHANGE;
		old = sched_window_stats_policy;
		new = sysctl_sched_window_stats_policy;
		sched_window_stats_policy = sysctl_sched_window_stats_policy;
	} else if (sched_account_wait_time != sysctl_sched_account_wait_time) {
		reason = ACCOUNT_WAIT_TIME_CHANGE;
		old = sched_account_wait_time;
		new = sysctl_sched_account_wait_time;
		sched_account_wait_time = sysctl_sched_account_wait_time;
	} else if (sched_ravg_hist_size != sysctl_sched_ravg_hist_size) {
		reason = HIST_SIZE_CHANGE;
		old = sched_ravg_hist_size;
		new = sysctl_sched_ravg_hist_size;
		sched_ravg_hist_size = sysctl_sched_ravg_hist_size;
	}
#ifdef CONFIG_SCHED_WALT_FREQ_INPUT
	else if (sched_migration_fixup != sysctl_sched_migration_fixup) {
		reason = MIGRATION_FIXUP_CHANGE;
		old = sched_migration_fixup;
		new = sysctl_sched_migration_fixup;
		sched_migration_fixup = sysctl_sched_migration_fixup;
	} else if (sched_freq_account_wait_time !=
					sysctl_sched_freq_account_wait_time) {
		reason = FREQ_ACCOUNT_WAIT_TIME_CHANGE;
		old = sched_freq_account_wait_time;
		new = sysctl_sched_freq_account_wait_time;
		sched_freq_account_wait_time =
				 sysctl_sched_freq_account_wait_time;
	}
#endif

	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		raw_spin_unlock(&rq->lock);
	}

	local_irq_restore(flags);

/*	trace_sched_reset_all_window_stats(window_start, window_size,
		ktime_get_ns() - start_ts, reason, old, new);*/
}

#ifdef CONFIG_SCHED_WALT_FREQ_INPUT

struct sched_load {
	unsigned long prev_load;
	unsigned long new_task_load;
};

static inline u64
scale_load_to_freq(u64 load, unsigned int src_freq, unsigned int dst_freq)
{
	return div64_u64(load * (u64)src_freq, (u64)dst_freq);
}

void fixup_busy_time(struct task_struct *p, int new_cpu)
{
	struct rq *src_rq = task_rq(p);
	struct rq *dest_rq = cpu_rq(new_cpu);
	u64 wallclock;
	bool new_task;

	if (!sched_enable_hmp || !sched_migration_fixup ||
		 (!p->on_rq && p->state != TASK_WAKING))
			return;

	if (exiting_task(p)) {
		clear_ed_task(p, src_rq);
		return;
	}

	if (p->state == TASK_WAKING)
		double_rq_lock(src_rq, dest_rq);

	if (sched_disable_window_stats)
		goto done;

	wallclock = ktime_get_ns();

	update_task_ravg(task_rq(p)->curr, task_rq(p),
			 TASK_UPDATE,
			 wallclock, 0);
	update_task_ravg(dest_rq->curr, dest_rq,
			 TASK_UPDATE, wallclock, 0);

	update_task_ravg(p, task_rq(p), TASK_MIGRATE,
			 wallclock, 0);

	new_task = is_new_task(p);

	if (p->ravg.curr_window) {
		src_rq->curr_runnable_sum -= p->ravg.curr_window;
		dest_rq->curr_runnable_sum += p->ravg.curr_window;
		if (new_task) {
			src_rq->nt_curr_runnable_sum -= p->ravg.curr_window;
			dest_rq->nt_curr_runnable_sum += p->ravg.curr_window;
		}
	}

	if (p->ravg.prev_window) {
		src_rq->prev_runnable_sum -= p->ravg.prev_window;
		dest_rq->prev_runnable_sum += p->ravg.prev_window;
		if (new_task) {
			src_rq->nt_prev_runnable_sum -= p->ravg.prev_window;
			dest_rq->nt_prev_runnable_sum += p->ravg.prev_window;
		}
	}

	if (p == src_rq->ed_task) {
		src_rq->ed_task = NULL;
		if (!dest_rq->ed_task)
			dest_rq->ed_task = p;
	}

	BUG_ON((s64)src_rq->prev_runnable_sum < 0);
	BUG_ON((s64)src_rq->curr_runnable_sum < 0);
	BUG_ON((s64)src_rq->nt_prev_runnable_sum < 0);
	BUG_ON((s64)src_rq->nt_curr_runnable_sum < 0);

/*	trace_sched_migration_update_sum(src_rq, p);
	trace_sched_migration_update_sum(dest_rq, p);*/

done:
	if (p->state == TASK_WAKING)
		double_rq_unlock(src_rq, dest_rq);
}

#else

static inline void fixup_busy_time(struct task_struct *p, int new_cpu) { }

static inline int
heavy_task_wakeup(struct task_struct *p, struct rq *rq, int event)
{
	return 0;
}

#endif	/* CONFIG_SCHED_WALT_FREQ_INPUT */

/* Keep track of max/min capacity possible across CPUs "currently" */
static void __update_min_max_capacity(void)
{
	int i;
	int max = 0, min = INT_MAX;

	for_each_online_cpu(i) {
		if (cpu_rq(i)->capacity > max)
			max = cpu_rq(i)->capacity;
		if (cpu_rq(i)->capacity < min)
			min = cpu_rq(i)->capacity;
	}

	max_capacity = max;
	min_capacity = min;
}

static void update_min_max_capacity(void)
{
	unsigned long flags;
	int i;

	local_irq_save(flags);
	for_each_possible_cpu(i)
		raw_spin_lock(&cpu_rq(i)->lock);

	__update_min_max_capacity();

	for_each_possible_cpu(i)
		raw_spin_unlock(&cpu_rq(i)->lock);
	local_irq_restore(flags);
}

/*
 * Return 'capacity' of a cpu in reference to "least" efficient cpu, such that
 * least efficient cpu gets capacity of 1024
 */
unsigned long capacity_scale_cpu_efficiency(int cpu)
{
	return (1024 * cpu_rq(cpu)->efficiency) / min_possible_efficiency;
}

/*
 * Return 'capacity' of a cpu in reference to cpu with lowest max_freq
 * (min_max_freq), such that one with lowest max_freq gets capacity of 1024.
 */
unsigned long capacity_scale_cpu_freq(int cpu)
{
	return (1024 * cpu_rq(cpu)->max_freq) / min_max_freq;
}

/*
 * Return load_scale_factor of a cpu in reference to "most" efficient cpu, so
 * that "most" efficient cpu gets a load_scale_factor of 1
 */
static inline unsigned long load_scale_cpu_efficiency(int cpu)
{
	return DIV_ROUND_UP(1024 * max_possible_efficiency,
			    cpu_rq(cpu)->efficiency);
}

/*
 * Return load_scale_factor of a cpu in reference to cpu with best max_freq
 * (max_possible_freq), so that one with best max_freq gets a load_scale_factor
 * of 1.
 */
static inline unsigned long load_scale_cpu_freq(int cpu)
{
	return DIV_ROUND_UP(1024 * max_possible_freq, cpu_rq(cpu)->max_freq);
}

static int compute_capacity(int cpu)
{
	int capacity = 1024;

	capacity *= capacity_scale_cpu_efficiency(cpu);
	capacity >>= 10;

	capacity *= capacity_scale_cpu_freq(cpu);
	capacity >>= 10;

	return capacity;
}

static int compute_load_scale_factor(int cpu)
{
	int load_scale = 1024;

	/*
	 * load_scale_factor accounts for the fact that task load
	 * is in reference to "best" performing cpu. Task's load will need to be
	 * scaled (up) by a factor to determine suitability to be placed on a
	 * (little) cpu.
	 */
	load_scale *= load_scale_cpu_efficiency(cpu);
	load_scale >>= 10;

	load_scale *= load_scale_cpu_freq(cpu);
	load_scale >>= 10;

	return load_scale;
}

#define sched_up_down_migrate_auto_update 1
static void check_for_up_down_migrate_update(const struct cpumask *cpus)
{
	int i = cpumask_first(cpus);
	struct rq *rq = cpu_rq(i);

	if (!sched_up_down_migrate_auto_update)
		return;

	if (rq->max_possible_capacity == max_possible_capacity)
		return;

	if (rq->max_possible_freq == rq->max_freq)
		up_down_migrate_scale_factor = 1024;
	else
		up_down_migrate_scale_factor = (1024 * rq->max_possible_freq)/
					rq->max_freq;

	/*update_up_down_migrate();*/
}

static int cpufreq_notifier_policy(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = (struct cpufreq_policy *)data;
	int i, update_max = 0;
	u64 highest_mpc = 0, highest_mplsf = 0;
	const struct cpumask *cpus = policy->related_cpus;
	unsigned int orig_min_max_freq = min_max_freq;
	unsigned int orig_max_possible_freq = max_possible_freq;
	/* Initialized to policy->max in case policy->related_cpus is empty! */
	unsigned int orig_max_freq = policy->max;

	if (val != CPUFREQ_NOTIFY && val != CPUFREQ_REMOVE_POLICY &&
						val != CPUFREQ_CREATE_POLICY)
		return 0;

	if (val == CPUFREQ_REMOVE_POLICY || val == CPUFREQ_CREATE_POLICY) {
		update_min_max_capacity();
		return 0;
	}

	for_each_cpu(i, policy->related_cpus) {
		cpumask_copy(&cpu_rq(i)->freq_domain_cpumask,
			     policy->related_cpus);
		orig_max_freq = cpu_rq(i)->max_freq;
		cpu_rq(i)->min_freq = policy->min;
		cpu_rq(i)->max_freq = policy->max;
		cpu_rq(i)->cur_freq = policy->cur;
		cpu_rq(i)->max_possible_freq = policy->cpuinfo.max_freq;
	}

	max_possible_freq = max(max_possible_freq, policy->cpuinfo.max_freq);
	if (min_max_freq == 1)
		min_max_freq = UINT_MAX;
	min_max_freq = min(min_max_freq, policy->cpuinfo.max_freq);
	BUG_ON(!min_max_freq);
	BUG_ON(!policy->max);

	/* Changes to policy other than max_freq don't require any updates */
	if (orig_max_freq == policy->max)
		return 0;

	/*
	 * A changed min_max_freq or max_possible_freq (possible during bootup)
	 * needs to trigger re-computation of load_scale_factor and capacity for
	 * all possible cpus (even those offline). It also needs to trigger
	 * re-computation of nr_big_task count on all online cpus.
	 *
	 * A changed rq->max_freq otoh needs to trigger re-computation of
	 * load_scale_factor and capacity for just the cluster of cpus involved.
	 * Since small task definition depends on max_load_scale_factor, a
	 * changed load_scale_factor of one cluster could influence
	 * classification of tasks in another cluster. Hence a changed
	 * rq->max_freq will need to trigger re-computation of nr_big_task
	 * count on all online cpus.
	 *
	 * While it should be sufficient for nr_big_tasks to be
	 * re-computed for only online cpus, we have inadequate context
	 * information here (in policy notifier) with regard to hotplug-safety
	 * context in which notification is issued. As a result, we can't use
	 * get_online_cpus() here, as it can lead to deadlock. Until cpufreq is
	 * fixed up to issue notification always in hotplug-safe context,
	 * re-compute nr_big_task for all possible cpus.
	 */

	if (orig_min_max_freq != min_max_freq ||
		orig_max_possible_freq != max_possible_freq) {
			cpus = cpu_possible_mask;
			update_max = 1;
	}

	/*
	 * Changed load_scale_factor can trigger reclassification of tasks as
	 * big or small. Make this change "atomic" so that tasks are accounted
	 * properly due to changed load_scale_factor
	 */
/*	pre_big_task_count_change(cpu_possible_mask);*/
	for_each_cpu(i, cpus) {
		struct rq *rq = cpu_rq(i);

		rq->capacity = compute_capacity(i);
		rq->load_scale_factor = compute_load_scale_factor(i);

		if (update_max) {
			u64 mpc, mplsf;

			mpc = div_u64(((u64) rq->capacity) *
				rq->max_possible_freq, rq->max_freq);
			rq->max_possible_capacity = (int) mpc;

			mplsf = div_u64(((u64) rq->load_scale_factor) *
				rq->max_possible_freq, rq->max_freq);

			if (mpc > highest_mpc) {
				highest_mpc = mpc;
				cpumask_clear(&mpc_mask);
				cpumask_set_cpu(i, &mpc_mask);
			} else if (mpc == highest_mpc) {
				cpumask_set_cpu(i, &mpc_mask);
			}

			if (mplsf > highest_mplsf)
				highest_mplsf = mplsf;
		}
	}

	if (update_max) {
		max_possible_capacity = highest_mpc;
		max_load_scale_factor = highest_mplsf;

		sched_update_freq_max_load(cpu_possible_mask);
	}

	__update_min_max_capacity();
	check_for_up_down_migrate_update(policy->related_cpus);
	/*post_big_task_count_change(cpu_possible_mask);*/

	return 0;
}

static int cpufreq_notifier_trans(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = (struct cpufreq_freqs *)data;
	unsigned int cpu = freq->cpu, new_freq = freq->new;
	unsigned long flags;
	int i;

	if (val != CPUFREQ_POSTCHANGE)
		return 0;

	BUG_ON(!new_freq);

	if (cpu_rq(cpu)->cur_freq == new_freq)
		return 0;

	for_each_cpu(i, &cpu_rq(cpu)->freq_domain_cpumask) {
		struct rq *rq = cpu_rq(i);

		raw_spin_lock_irqsave(&rq->lock, flags);
		update_task_ravg(rq->curr, rq, TASK_UPDATE,
				 ktime_get_ns(), 0);
		rq->cur_freq = new_freq;
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}

	return 0;
}

static struct notifier_block notifier_policy_block = {
	.notifier_call = cpufreq_notifier_policy
};

static struct notifier_block notifier_trans_block = {
	.notifier_call = cpufreq_notifier_trans
};

static int register_sched_callback(void)
{
	int ret;

	if (!sched_enable_hmp)
		return 0;

	ret = cpufreq_register_notifier(&notifier_policy_block,
						CPUFREQ_POLICY_NOTIFIER);

	if (!ret)
		ret = cpufreq_register_notifier(&notifier_trans_block,
						CPUFREQ_TRANSITION_NOTIFIER);

	return 0;
}

/*
 * cpufreq callbacks can be registered at core_initcall or later time.
 * Any registration done prior to that is "forgotten" by cpufreq. See
 * initialization of variable init_cpufreq_transition_notifier_list_called
 * for further information.
 */
core_initcall(register_sched_callback);

void
inc_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p)
{
	inc_cumulative_runnable_avg(&rq->hmp_stats, p);
}

void
dec_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p)
{
	dec_cumulative_runnable_avg(&rq->hmp_stats, p);
}

void fixup_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p,
			 u32 new_task_load)
{
	fixup_cumulative_runnable_avg(&rq->hmp_stats, p, new_task_load);
}

void init_new_task_load(struct task_struct *p)
{
	int i;
	u32 init_load_windows = sched_init_task_load_windows;
	u32 init_load_pct = current->init_load_pct;

	p->init_load_pct = 0;
	memset(&p->ravg, 0, sizeof(struct ravg));

	if (init_load_pct) {
		init_load_windows = div64_u64((u64)init_load_pct *
			  (u64)sched_ravg_window, 100);
	}

	p->ravg.demand = init_load_windows;
	for (i = 0; i < RAVG_HIST_SIZE_MAX; ++i)
		p->ravg.sum_history[i] = init_load_windows;
}

void inc_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p)
{
	inc_cumulative_runnable_avg(&rq->hmp_stats, p);
}

void dec_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p)
{
	dec_cumulative_runnable_avg(&rq->hmp_stats, p);
}

void fixup_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p,
			   u32 new_task_load)
{
	s64 task_load_delta = (s64)new_task_load - task_load(p);

	fixup_cumulative_runnable_avg(&rq->hmp_stats, p, task_load_delta);
}

void
inc_hmp_sched_stats_idle(struct rq *rq, struct task_struct *p)
{
}

void
dec_hmp_sched_stats_idle(struct rq *rq, struct task_struct *p)
{
}

void
fixup_hmp_sched_stats_idle(struct rq *rq, struct task_struct *p,
			   u32 new_task_load)
{
}

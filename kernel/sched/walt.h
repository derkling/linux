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

#ifndef __WALT_H
#define __WALT_H

#ifdef CONFIG_SCHED_WALT

void
update_task_ravg(struct task_struct *p, struct rq *rq,
			 int event, u64 wallclock, u64 irqtime);
void
inc_rq_hmp_stats(struct rq *rq, struct task_struct *p, int change_cra);
void
dec_rq_hmp_stats(struct rq *rq, struct task_struct *p, int change_cra);

void
inc_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p);

void
dec_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p);

void fixup_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p,
				       u32 new_task_load);

void init_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq);

void inc_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
		 struct task_struct *p, int change_cra);

void dec_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
		 struct task_struct *p, int change_cra);

void inc_throttled_cfs_rq_hmp_stats(struct hmp_sched_stats *stats,
			 struct cfs_rq *cfs_rq);

void dec_throttled_cfs_rq_hmp_stats(struct hmp_sched_stats *stats,
				 struct cfs_rq *cfs_rq);

void
inc_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p);

void
dec_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p);

void fixup_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p,
			 u32 new_task_load);

void fixup_busy_time(struct task_struct *p, int new_cpu);

u64 sched_ktime_clock(void);
void clear_ed_task(struct task_struct *p, struct rq *rq);
void set_task_last_wake(struct task_struct *p, u64 wallclock);
void set_task_last_switch_out(struct task_struct *p,
					    u64 wallclock);

void init_new_task_load(struct task_struct *p);

void mark_task_starting(struct task_struct *p);
void set_window_start(struct rq *rq);

void migrate_sync_cpu(int cpu);

void init_cpu_efficiency(void);

void inc_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p);

void dec_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p);

void fixup_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p,
			   u32 new_task_load);
void inc_hmp_sched_stats_idle(struct rq *rq, struct task_struct *p);

void dec_hmp_sched_stats_idle(struct rq *rq, struct task_struct *p);

void fixup_hmp_sched_stats_idle(struct rq *rq, struct task_struct *p,
			   u32 new_task_load);

#define WINDOW_STATS_RECENT		0
#define WINDOW_STATS_MAX		1
#define WINDOW_STATS_MAX_RECENT_AVG	2
#define WINDOW_STATS_AVG		3
#define WINDOW_STATS_INVALID_POLICY	4

extern struct mutex policy_mutex;
extern unsigned int sched_ravg_window;
extern unsigned int sched_use_pelt;
extern unsigned int sched_disable_window_stats;
extern unsigned int sched_enable_hmp;
extern unsigned int max_possible_freq;
extern unsigned int min_max_freq;
extern unsigned int pct_task_load(struct task_struct *p);
extern unsigned int max_possible_efficiency;
extern unsigned int min_possible_efficiency;
extern unsigned int max_capacity;
extern unsigned int min_capacity;
extern unsigned int max_load_scale_factor;
extern unsigned int max_possible_capacity;
extern cpumask_t mpc_mask;
extern unsigned long capacity_scale_cpu_efficiency(int cpu);
extern unsigned long capacity_scale_cpu_freq(int cpu);
extern unsigned int sched_upmigrate;
extern unsigned int sched_downmigrate;
extern unsigned int sched_init_task_load_pelt;
extern unsigned int sched_init_task_load_windows;
extern unsigned int sched_heavy_task;
extern unsigned int up_down_migrate_scale_factor;
extern void reset_cpu_hmp_stats(int cpu, int reset_cra);
extern unsigned int max_task_load(void);
extern void sched_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock);
unsigned int cpu_temp(int cpu);
extern unsigned int nr_eligible_big_tasks(int cpu);
extern void update_up_down_migrate(void);

/*
 * 'load' is in reference to "best cpu" at its best frequency.
 * Scale that in reference to a given cpu, accounting for how bad it is
 * in reference to "best cpu".
 */
static inline u64 scale_load_to_cpu(u64 task_load, int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	if (rq->load_scale_factor != 1024) {
		task_load *= (u64)rq->load_scale_factor;
		task_load /= 1024;
	}

	return task_load;
}

static inline int capacity(struct rq *rq)
{
	return rq->capacity;
}
static inline int max_poss_capacity(struct rq *rq)
{
	return rq->max_possible_capacity;
}

static inline unsigned int task_load(struct task_struct *p)
{
	if (sched_use_pelt)
		return p->se.avg.runnable_avg_sum_scaled;

	return p->ravg.demand;
}

static inline void
inc_cumulative_runnable_avg(struct hmp_sched_stats *stats,
				 struct task_struct *p)
{
	u32 task_load;

	if (!sched_enable_hmp || sched_disable_window_stats)
		return;

	task_load = sched_use_pelt ? p->se.avg.runnable_avg_sum_scaled :
			(sched_disable_window_stats ? 0 : p->ravg.demand);

	stats->cumulative_runnable_avg += task_load;

}

static inline void
dec_cumulative_runnable_avg(struct hmp_sched_stats *stats,
				 struct task_struct *p)
{
	u32 task_load;

	if (!sched_enable_hmp || sched_disable_window_stats)
		return;

	task_load = sched_use_pelt ? p->se.avg.runnable_avg_sum_scaled :
			(sched_disable_window_stats ? 0 : p->ravg.demand);

	stats->cumulative_runnable_avg -= task_load;

	BUG_ON((s64)stats->cumulative_runnable_avg < 0);
}

static inline void
fixup_cumulative_runnable_avg(struct hmp_sched_stats *stats,
			      struct task_struct *p, s64 task_load_delta)
{
	if (!sched_enable_hmp || sched_disable_window_stats)
		return;

	stats->cumulative_runnable_avg += task_load_delta;
	if ((s64)stats->cumulative_runnable_avg < 0)
		panic("cra less than zero: tld: %lld, task_load(p) = %u\n",
			task_load_delta, task_load(p));
}


#define pct_to_real(tunable)	\
		(div64_u64((u64)tunable * (u64)max_task_load(), 100))

#define real_to_pct(tunable)	\
		(div64_u64((u64)tunable * (u64)100, (u64)max_task_load()))

#define	BOOST_KICK	0
#define	CPU_RESERVED	1

static inline int is_reserved(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	return test_bit(CPU_RESERVED, &rq->hmp_flags);
}

static inline int mark_reserved(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	/* Name boost_flags as hmp_flags? */
	return test_and_set_bit(CPU_RESERVED, &rq->hmp_flags);
}

static inline void clear_reserved(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	clear_bit(CPU_RESERVED, &rq->hmp_flags);
}

static inline u64 cpu_cravg_sync(int cpu, int sync)
{
	struct rq *rq = cpu_rq(cpu);
	u64 load;

	load = rq->hmp_stats.cumulative_runnable_avg;

	/*
	 * If load is being checked in a sync wakeup environment,
	 * we may want to discount the load of the currently running
	 * task.
	 */
	if (sync && cpu == smp_processor_id()) {
		if (load > rq->curr->ravg.demand)
			load -= rq->curr->ravg.demand;
		else
			load = 0;
	}

	return load;
}

extern void check_for_migration(struct rq *rq, struct task_struct *p);
extern void pre_big_task_count_change(const struct cpumask *cpus);
extern void post_big_task_count_change(const struct cpumask *cpus);
extern void set_hmp_defaults(void);
extern int power_delta_exceeded(unsigned int cpu_cost, unsigned int base_cost);
extern unsigned int power_cost(int cpu, u64 demand);
extern void reset_all_window_stats(u64 window_start, unsigned int window_size);
extern void boost_kick(int cpu);
extern int sched_boost(void);

#else /* CONFIG_SCHED_WALT */

struct hmp_sched_stats;

static inline void init_new_task_load(struct task_struct *p) { }

static inline u64 scale_load_to_cpu(u64 load, int cpu)
{
	return load;
}

static inline unsigned int nr_eligible_big_tasks(int cpu)
{
	return 0;
}

static inline int pct_task_load(struct task_struct *p) { return 0; }

static inline int capacity(struct rq *rq)
{
	return SCHED_LOAD_SCALE;
}

static inline int max_poss_capacity(struct rq *rq)
{
	return SCHED_LOAD_SCALE;
}


static inline void inc_cumulative_runnable_avg(struct hmp_sched_stats *stats,
		 struct task_struct *p)
{
}

static inline void dec_cumulative_runnable_avg(struct hmp_sched_stats *stats,
		 struct task_struct *p)
{
}

static inline unsigned long capacity_scale_cpu_efficiency(int cpu)
{
	return SCHED_LOAD_SCALE;
}

static inline unsigned long capacity_scale_cpu_freq(int cpu)
{
	return SCHED_LOAD_SCALE;
}

#define sched_enable_hmp 0
#define sched_freq_legacy_mode 1

static inline void check_for_migration(struct rq *rq, struct task_struct *p) { }
static inline void pre_big_task_count_change(void) { }
static inline void post_big_task_count_change(void) { }
static inline void set_hmp_defaults(void) { }

static inline void clear_reserved(int cpu) { }

#define trace_sched_cpu_load(...)
#define trace_sched_cpu_load_lb(...)
#define trace_sched_cpu_load_cgroup(...)
#define trace_sched_cpu_load_wakeup(...)

static inline void clear_ed_task(struct task_struct *p, struct rq *rq) {}
static inline void set_task_last_wake(struct task_struct *p, u64 wallclock) {}
static inline void set_task_last_switch_out(struct task_struct *p,
					    u64 wallclock) {}

static inline void
inc_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p) { }

static inline void
dec_hmp_sched_stats_rt(struct rq *rq, struct task_struct *p) { }

static inline void
inc_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p) { }

static inline void
dec_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p) { }

static inline void
fixup_hmp_sched_stats_stop(struct rq *rq, struct task_struct *p,
			   u32 new_task_load) { }

static inline void inc_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
	 struct task_struct *p, int change_cra) { }

static inline void dec_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq,
	 struct task_struct *p, int change_cra) { }

#define sysctl_sched_enable_power_aware 0

static inline int task_will_fit(struct task_struct *p, int cpu)
{
	return 1;
}

static inline int select_best_cpu(struct task_struct *p, int target,
				  int reason, int sync)
{
	return 0;
}

#define power_cost(...) 0

static inline int
spill_threshold_crossed(u64 task_load, u64 cpu_load, struct rq *rq)
{
	return 0;
}

static inline int sched_boost(void)
{
	return 0;
}

static inline int is_big_task(struct task_struct *p)
{
	return 0;
}

static inline int nr_big_tasks(struct rq *rq)
{
	return 0;
}

static inline int is_cpu_throttling_imminent(int cpu)
{
	return 0;
}

static inline int is_task_migration_throttled(struct task_struct *p)
{
	return 0;
}

#define cpu_temp(...) 0

static inline void
inc_rq_hmp_stats(struct rq *rq, struct task_struct *p, int change_cra) { }
static inline void
dec_rq_hmp_stats(struct rq *rq, struct task_struct *p, int change_cra) { }

static inline void
inc_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p) { }

static inline void
dec_hmp_sched_stats_fair(struct rq *rq, struct task_struct *p) { }

static inline void fixup_hmp_sched_stats_fair(struct rq *rq,
					      struct task_struct *p,
					      u32 new_task_load) {}

static inline void
add_to_scaled_stat(int cpu, struct sched_avg *sa, u64 delta)
{
}

static inline void decay_scaled_stat(struct sched_avg *sa, u64 periods)
{
}

static inline void init_cfs_rq_hmp_stats(struct cfs_rq *cfs_rq) { }

static inline void inc_throttled_cfs_rq_hmp_stats(struct hmp_sched_stats *stats,
			 struct cfs_rq *cfs_rq)
{
}

static inline void dec_throttled_cfs_rq_hmp_stats(struct hmp_sched_stats *stats,
			 struct cfs_rq *cfs_rq)
{
}

static inline void fixup_busy_time(struct task_struct *p, int new_cpu) { }

static inline int
heavy_task_wakeup(struct task_struct *p, struct rq *rq, int event)
{
	return 0;
}

static inline void
update_task_ravg(struct task_struct *p, struct rq *rq,
			 int event, u64 wallclock, u64 irqtime)
{
}

static inline void init_cpu_efficiency(void) {}

static inline void mark_task_starting(struct task_struct *p) {}

static inline void set_window_start(struct rq *rq) {}

static inline void migrate_sync_cpu(int cpu) {}

static inline u64 sched_ktime_clock(void) { return 0; }

#endif /* CONFIG_SCHED_WALT */

#endif

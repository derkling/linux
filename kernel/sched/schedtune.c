#include <linux/cgroup.h>
#include <linux/slab.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/cpumask.h>
#include <linux/seq_file.h>
#include <linux/rcupdate.h>
#include <linux/err.h>
#include <linux/printk.h>

#include <trace/events/sched.h>

#include "sched.h"
#include "schedtune.h"

/*
 * EAS scheduler tunables for task groups.
 */


/* Scheduer tunables for a group of tasks and its child groups */
struct schedtune {
	struct cgroup_subsys_state css;

	/* Boostgroup allocated ID */
	int idx;

	int boostmode;
	int margin;

	/* Performance Boost (B) region threshold params */
	int perf_boost_idx;

	/* Performance Constraint (C) region threshold params */
	int perf_constrain_idx;

};

struct threshold_params {
	int nrg_gain;
	int cap_gain;
};

struct threshold_params threshold_gains[] = {
	{ 0, 4 }, /* >=  0% */
	{ 1, 4 }, /* >= 10% */
	{ 2, 4 }, /* >= 20% */
	{ 3, 4 }, /* >= 30% */
	{ 4, 4 }, /* >= 40% */
	{ 4, 3 }, /* >= 50% */
	{ 4, 2 }, /* >= 60% */
	{ 4, 1 }, /* >= 70% */
	{ 4, 0 }, /* >= 80% */
	{ 4, 0 }  /* >= 90% */
};

static inline struct schedtune *css_st(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct schedtune, css) : NULL;
}

static inline struct schedtune *task_schedtune(struct task_struct *tsk)
{
	return css_st(task_css(tsk, schedtune_cgrp_id));
}

static inline struct schedtune *parent_st(struct schedtune *st)
{
	return css_st(st->css.parent);
}

static struct schedtune root_schedtune = {
	.margin		= 0,
	.boostmode	= SCHEDTUNE_BOOSTMODE_NONE,
	.perf_boost_idx 	= 0,
	.perf_constrain_idx 	= 0,
};

/* The maximum number of boost groups to support */
#define BOOSTGROUPS_COUNT 16

/* Array of configured boostgroups */
static struct schedtune *allocated_group[BOOSTGROUPS_COUNT] = {
	&root_schedtune,
	NULL,
};

struct boost_groups {
	unsigned margin_max;
	struct {
		/* The margin for tasks on that boostgroup */
		unsigned margin;
		/* Count of RUNNABLE tasks on that boostgroup */
		unsigned tasks;
	} group[BOOSTGROUPS_COUNT];
};

DEFINE_PER_CPU(struct boost_groups, cpu_boost_groups);

static void
schedtune_update_cpu_margin(int cpu)
{
	struct boost_groups *bg;
	unsigned margin_max;
	int idx;

	bg = &per_cpu(cpu_boost_groups, cpu);

	/* The root boost group is alwasy active */
	margin_max = bg->group[0].margin;
	for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx) {
		/*
		 * A boostgroup affects a CPU only if there are
		 * RUNNABLE tasks on that CPU
		 */
		if (bg->group[idx].tasks == 0)
			continue;
		margin_max = max(margin_max, bg->group[idx].margin);
	}

	bg->margin_max = margin_max;
	trace_printk("cpu_update: cpu=%d max_margin=%d\n", cpu, bg->margin_max);
}

static int
schedtune_boostgroup_update(int idx, int margin)
{
	struct boost_groups *bg;
	int cur_margin_max;
	int old_margin;
	int cpu;

	trace_printk("bgr_update: idx=%d margin=%d\n", idx, margin);

	/* Update the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);

		/* Keep track of current CPU margins */
		cur_margin_max = bg->margin_max;
		old_margin = bg->group[idx].margin;

		/* Update this boost group margin */
		bg->group[idx].margin = margin;

		/* Check if this update increase current max */
		if (margin > cur_margin_max && bg->group[idx].tasks) {
			bg->margin_max = margin;

			trace_printk("cpu_update=%d max_margin=%d\n", cpu, margin);

			continue;
		}

		/* Check if this update has decreased current max */
		if (cur_margin_max == old_margin && old_margin > margin) {
			schedtune_update_cpu_margin(cpu);
		}

		trace_printk("cpu_update: cpu=%d max_margin=%d\n", cpu, bg->margin_max);

	}


	return 0;
}

static int
schedtune_boostgroup_init(struct schedtune *st)
{
	struct boost_groups *bg;
	int cpu;

	/* Keep track of allocated boost group */
	allocated_group[st->idx] = st;

	/* Initialize the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);
		bg->group[st->idx].margin = 0;
		bg->group[st->idx].tasks = 0;
	}

	return 0;
}

static void
schedtune_boostgroup_release(struct schedtune *st)
{
	/* Reset this group margin */
	schedtune_boostgroup_update(st->idx, 0);

	/* Keep track of allocated boost group */
	allocated_group[st->idx] = NULL;
}

static struct cgroup_subsys_state *
schedtune_css_alloc(struct cgroup_subsys_state *parent_css)
{
	struct schedtune *st;
	int idx;

	if (!parent_css)
		return &root_schedtune.css;

	/* Allows only single level hierachies */
	if (parent_css != &root_schedtune.css) {
		pr_err("Nested SchedTune boosting groups not allowed\n");
		return ERR_PTR(-ENOMEM);
	}

	/* Allows only a limited number of boosting groups */
	for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx)
		if (allocated_group[idx] == NULL)
			break;
	if (idx == BOOSTGROUPS_COUNT) {
		pr_err("Trying to create more than %d SchedTune boosting groups\n",
				BOOSTGROUPS_COUNT);
		return ERR_PTR(-ENOSPC);
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto out;

	/* Initialize per CPUs boost group support */
	st->idx = idx;
	if (schedtune_boostgroup_init(st))
		goto release;

	return &st->css;

release:
	kfree(st);
out:
	return ERR_PTR(-ENOMEM);
}

static void schedtune_css_free(struct cgroup_subsys_state *css)
{
	struct schedtune *st = css_st(css);
	schedtune_boostgroup_release(st);
	kfree(st);
}

static u64 margin_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);
	return st->margin;
}

static int margin_write(struct cgroup_subsys_state *css, struct cftype *cft,
			  u64 margin)
{
	struct schedtune *st = css_st(css);
	int err = 0;

	if (margin < 0 || margin > 100) {
		err = -EINVAL;
		goto out;
	}

	st->margin = margin;

	if (margin == 100)
		margin = 99;

	/* Performance Boost (B) region threshold params */
	st->perf_boost_idx  = margin;
	st->perf_boost_idx /= 10;

	/* Performance Constraint (C) region threshold params */
	st->perf_constrain_idx  = 100 - margin;
	st->perf_constrain_idx /= 10;

	/* Update CPU margin */
	schedtune_boostgroup_update(st->idx, st->margin);

	trace_schedtune(st->margin, st->boostmode,
			threshold_gains[st->perf_boost_idx].nrg_gain,
			threshold_gains[st->perf_boost_idx].cap_gain,
			threshold_gains[st->perf_constrain_idx].nrg_gain,
			threshold_gains[st->perf_constrain_idx].cap_gain);

out:
	return err;
}

static u64 boostmode_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);
	return st->boostmode;
}

static int boostmode_write(struct cgroup_subsys_state *css, struct cftype *cft,
			  u64 boostmode)
{
	struct schedtune *st = css_st(css);
	int err = 0;

	if (boostmode < SCHEDTUNE_BOOSTMODE_NONE ||
		boostmode >= SCHEDTUNE_BOOSTMODE_COUNT) {
		err = -EINVAL;
		goto out;
	}

	st->boostmode = boostmode;

	trace_schedtune(st->margin, st->boostmode,
			threshold_gains[st->perf_boost_idx].nrg_gain,
			threshold_gains[st->perf_boost_idx].cap_gain,
			threshold_gains[st->perf_constrain_idx].nrg_gain,
			threshold_gains[st->perf_constrain_idx].cap_gain);

out:
	return err;
}

/* NOTE: This function must be called while holding the lock on the CPU RQ */
static int
schedtune_update_runnable_tasks(struct task_struct *p, int cpu, int task_count)
{
	struct boost_groups *bg;
	struct schedtune *st;
	int tasks;
	int idx;

	/* Get task boost group */
	rcu_read_lock();
	st = task_schedtune(p);
	idx = st->idx;
	rcu_read_unlock();

	bg = &per_cpu(cpu_boost_groups, cpu);

	/* Update boosted tasks count while avoiding to make it negative */
	if (task_count < 0 && bg->group[idx].tasks <= -task_count)
		bg->group[idx].tasks = 0;
	else
		bg->group[idx].tasks += task_count;

	/* Boost group activation or deactivation on that RQ */
	tasks = bg->group[idx].tasks;
	trace_printk("cpu=%d tasks=%d\n", cpu, tasks);
	if (tasks == 1 || tasks == 0)
		schedtune_update_cpu_margin(cpu);

	return 0;
}

/*
 * NOTE: This function must be called while holding the lock on the CPU RQ
 */
int
schedtune_enqueue_task(struct task_struct *p, int cpu)
{
	int result;

	result = schedtune_update_runnable_tasks(p, cpu, 1);
	return result;
}

/*
 * NOTE: This function must be called while holding the lock on the CPU RQ
 */
int
schedtune_dequeue_task(struct task_struct *p, int cpu)
{
	int result;

	result = schedtune_update_runnable_tasks(p, cpu, -1);
	return result;
}


int schedtune_taskgroup_margin(struct task_struct *p)
{
	struct schedtune *ct;
	int task_margin;

	rcu_read_lock();
	ct = task_schedtune(p);
	task_margin = ct->margin;
	rcu_read_unlock();

	return task_margin;
}

int schedtune_taskgroup_boostmode(struct task_struct *p)
{
	struct schedtune *ct;
	int boostmode;

	rcu_read_lock();
	ct = task_schedtune(p);
	boostmode = ct->boostmode;
	rcu_read_unlock();

	return boostmode;
}

int schedtune_root_margin(void)
{
	int root_margin;
	root_margin = root_schedtune.margin;
	return root_margin;
}

int schedtune_root_boostmode(void)
{
	int root_boostmode;
	root_boostmode = root_schedtune.boostmode;
	return root_boostmode;
}

int schedtune_cpu_margin(int cpu)
{
	struct boost_groups *bg;
	bg = &per_cpu(cpu_boost_groups, cpu);
	return 	bg->margin_max;
}


int schedtune_accept_deltas(int nrg_delta, int cap_delta, struct task_struct *task) {
	struct schedtune *ct;
	int perf_boost_idx;
	int perf_constrain_idx;
	int energy_payoff;

	/* Optimal (O) region */
	if (nrg_delta < 0 && cap_delta > 0) {
		trace_printk("region=O ngain=0 pgain=0 nrg_payoff=-1");
		return INT_MAX;
	}

	/* Suboptimal (S) region */
	if (nrg_delta > 0 && cap_delta < 0) {
		trace_printk("region=S ngain=0 pgain=0 nrg_payoff=1");
		return -INT_MAX;
	}

	/* Get task specific perf Boost/Constraints indexes */
	rcu_read_lock();
	ct = task_schedtune(task);
	perf_boost_idx = ct->perf_boost_idx;
	perf_constrain_idx = ct->perf_constrain_idx;
	rcu_read_unlock();

	/* Performance Boost (B) region */
	if (nrg_delta > 0 && cap_delta > 0) {
		/*
		 * energy_payoff criteria:
		 *    cap_delta / nrg_delta > cap_gain / nrg_gain
		 * which is:
		 *    nrg_delta * cap_gain < cap_delta * nrg_gain
		 */
		energy_payoff  = cap_delta * threshold_gains[perf_boost_idx].nrg_gain;
		energy_payoff -= nrg_delta * threshold_gains[perf_boost_idx].cap_gain;

		trace_printk("region=B ngain=%d pgain=%d nrg_payoff=%d",
				threshold_gains[perf_boost_idx].nrg_gain,
				threshold_gains[perf_boost_idx].cap_gain,
				energy_payoff);

		return energy_payoff;
	}

	/* Performance Constraint (C) region */
	if (nrg_delta < 0 && cap_delta < 0) {
		/*
		 * energy_payoff criteria:
		 *    cap_delta / nrg_delta < cap_gain / nrg_gain
		 * which is:
		 *    nrg_delta * cap_gain > cap_delta * nrg_gain
		 */
		energy_payoff  = nrg_delta * threshold_gains[perf_constrain_idx].cap_gain;
		energy_payoff -= cap_delta * threshold_gains[perf_constrain_idx].nrg_gain;

		trace_printk("region=C ngain=%d pgain=%d nrg_payoff=%d",
				threshold_gains[perf_constrain_idx].nrg_gain,
				threshold_gains[perf_constrain_idx].cap_gain,
				energy_payoff);

		return energy_payoff;
	}

	return -INT_MAX;
}

static struct {
	unsigned long min_power;
	unsigned long max_power;
	unsigned long nrg_shift;
	unsigned long nrg_mult;
} schedtune_target_nrg = {
	/* TC2 Min CPUs power:
	 * all CPUs idle, all clusters in deep idle:
	 *   0 * 3 + 0 * 2 + 10 + 25
	 */
	.min_power = 35,

	/*
	 * TC2 Max CPUs power:
	 * all CPUs fully utilized while running at max OPP:
	 *   1024 * 3 + 6997 * 2 + 4905 + 15200
	 */
	.max_power = 37171,
	/*
	 * Integer division constants definition:
	 *  Constant: Max - Min          (C) = 37136
	 *  Precision: 0.1%              (P) = 0.1
	 *  Reference: C * 100 / P       (R) = 3713600
	 * Thus:
	 *  Shift bifs: ceil(log(R,2))   (S) = 26
	 *  Mult const: round(2^S/C)     (M) = 1807
	 */
	.nrg_shift = 26,	/* S */
	.nrg_mult  = 1807,	/* M */
};

int schedtune_normalize_energy(int energy_diff)
{
	long long normalized_nrg = energy_diff;
	int max_delta;

	/* Check for boundaries */
	max_delta  = schedtune_target_nrg.max_power;
	max_delta -= schedtune_target_nrg.min_power;
	WARN_ON(abs(energy_diff) >= max_delta);

	/* Scale by energy magnitude */
	normalized_nrg <<= SCHED_LOAD_SHIFT;

	/* Normalize on max energy for that platform */
	normalized_nrg  *= schedtune_target_nrg.nrg_mult;
	normalized_nrg >>= schedtune_target_nrg.nrg_shift;

	return normalized_nrg;
}

static struct cftype files[] = {
	{
		.name = "margin",
		.read_u64 = margin_read,
		.write_u64 = margin_write,
	},
	{
		.name = "boostmode",
		.read_u64 = boostmode_read,
		.write_u64 = boostmode_write,
	},
	{ }	/* terminate */
};

struct cgroup_subsys schedtune_cgrp_subsys = {
	.css_alloc	= schedtune_css_alloc,
	.css_free	= schedtune_css_free,
	.legacy_cftypes	= files,
	.early_init	= 1,
};

#include <linux/cgroup.h>
#include <linux/slab.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/cpumask.h>
#include <linux/seq_file.h>
#include <linux/rcupdate.h>
#include <linux/err.h>

#include <trace/events/sched.h>

#include "sched.h"
#include "schedtune.h"

/*
 * EAS scheduler tunables for task groups.
 */


/* Scheduer tunables for a group of tasks and its child groups */
struct schedtune {
	struct cgroup_subsys_state css;
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

static struct cgroup_subsys_state *
schedtune_css_alloc(struct cgroup_subsys_state *parent_css)
{
	struct schedtune *st;

	if (!parent_css)
		return &root_schedtune.css;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto out;

	return &st->css;

out:
	return ERR_PTR(-ENOMEM);
}

static void schedtune_css_free(struct cgroup_subsys_state *css)
{
	struct schedtune *st = css_st(css);
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

int schedtune_accept_deltas(int nrg_delta, int cap_delta, struct task_struct *task) {
	struct schedtune *ct;
	int perf_boost_idx;
	int perf_constrain_idx;
	int energy_payoff;

	/* Optimal (O) region */
	if (nrg_delta < 0 && cap_delta > 0) {
		return INT_MAX;
	}

	/* Suboptimal (S) region */
	if (nrg_delta > 0 && cap_delta < 0) {
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

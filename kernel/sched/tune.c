#include <linux/cgroup.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/slab.h>

#include "sched.h"

unsigned int sysctl_sched_cfs_boost __read_mostly = 0;

#ifdef CONFIG_CGROUP_SCHEDTUNE

/*
 * EAS scheduler tunables for task groups.
 */

/* SchdTune tunables for a group of tasks */
struct schedtune {
	/* SchedTune CGroup subsystem */
	struct cgroup_subsys_state css;

	/* Boost group allocated ID */
	int idx;

	/* Boost value for tasks on that SchedTune CGroup */
	int boost;

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

/*
 * SchedTune root control group
 * The root control group is used to defined a system-wide boosting tuning,
 * which is applied to all tasks in the system.
 * Task specific boosting tuning could be specified by creating and
 * configuring a child control group under the root one.
 * By default, system-wide boosting is disabled, i.e. no boosting is applied
 * to all tasks not into a child control group.
 */
static struct schedtune
root_schedtune = {
	.boost	= 0,
};

/*
 * Maximum number of boost groups to support
 * When per-task boosting is used we still allows only limited number of
 * boost groups for two main reasons:
 * 1. on a real system we usually have only few classes of workloads which
 *    make sense to boost with different values (e.g. backgroud vs foreground
 *    tasks, interactive vs low-priority tasks)
 * 2. a limited number allows for a simpler and more memory/time efficient
 *    implementation especially for the computation of the per-CPU boost
 *    value
 */
#define BOOSTGROUPS_COUNT 16

/* Array of configured boostgroups */
static struct schedtune *allocated_group[BOOSTGROUPS_COUNT] = {
	&root_schedtune,
	NULL,
};

static u64
boost_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);
	return st->boost;
}

static int
boost_write(struct cgroup_subsys_state *css, struct cftype *cft,
			  u64 boost)
{
	struct schedtune *st = css_st(css);
	int err = 0;

	if (boost < 0 || boost > 100) {
		err = -EINVAL;
		goto out;
	}

	st->boost = boost;
	if (css == &root_schedtune.css)
		sysctl_sched_cfs_boost = boost;

	if (boost == 100)
		boost = 99;

out:
	return err;
}

static struct cftype files[] = {
	{
		.name = "boost",
		.read_u64 = boost_read,
		.write_u64 = boost_write,
	},
	{ }	/* terminate */
};

static int
schedtune_boostgroup_init(struct schedtune *st)
{
	struct boost_groups *bg;

	/* Keep track of allocated boost group */
	allocated_group[st->idx] = st;

	return 0;
}

static int
schedtune_init(void)
{
	struct boost_groups *bg;
	int cpu;

	/* Initialize the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);
		memset(bg, 0 , sizeof(struct boost_groups));
	}

	pr_info("  schedtune configured to support %d boost groups\n",
			BOOSTGROUPS_COUNT);
	return 0;
}

static struct cgroup_subsys_state *
schedtune_css_alloc(struct cgroup_subsys_state *parent_css)
{
	struct schedtune *st;
	int idx;

	if (!parent_css) {
		schedtune_init();
		return &root_schedtune.css;
	}

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

static void
schedtune_boostgroup_release(struct schedtune *st)
{
	/* Keep track of allocated boost group */
	allocated_group[st->idx] = NULL;
}

static void
schedtune_css_free(struct cgroup_subsys_state *css)
{
	struct schedtune *st = css_st(css);
	schedtune_boostgroup_release(st);
	kfree(st);
}

struct cgroup_subsys schedtune_cgrp_subsys = {
	.css_alloc	= schedtune_css_alloc,
	.css_free	= schedtune_css_free,
	.legacy_cftypes	= files,
	.early_init	= 1,
};

#endif /* CONFIG_CGROUP_SCHEDTUNE */

int
sysctl_sched_cfs_boost_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	return 0;
}


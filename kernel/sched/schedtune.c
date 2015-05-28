#include <linux/cgroup.h>
#include <linux/slab.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/cpumask.h>
#include <linux/seq_file.h>
#include <linux/rcupdate.h>
#include <linux/err.h>

#include "sched.h"

/*
 * EAS scheduler tunables for task groups.
 */

/* Scheduer tunables for a group of tasks and its child groups */
struct schedtune {
	struct cgroup_subsys_state css;
	int margin;
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
			  int margin)
{
	struct schedtune *st = css_st(css);
	int err = 0;

	if (margin < 0 || margin > 100) {
		err = -EINVAL;
		goto out;
	}

	st->margin = margin;

out:
	return err;
}

static struct cftype files[] = {
	{
		.name = "margin",
		.read_u64 = margin_read,
		.write_u64 = margin_write,
	},
	{ }	/* terminate */
};

struct cgroup_subsys schedtune_cgrp_subsys = {
	.css_alloc	= schedtune_css_alloc,
	.css_free	= schedtune_css_free,
	.legacy_cftypes	= files,
	.early_init	= 1,
};

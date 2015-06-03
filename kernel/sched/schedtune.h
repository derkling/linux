#ifdef CONFIG_CGROUP_SCHEDTUNE

enum schedtune_boostmode {
	SCHEDTUNE_BOOSTMODE_NONE = 0,

	SCHEDTUNE_BOOSTMODE_SPC,
	SCHEDTUNE_BOOSTMODE_SPA,
	SCHEDTUNE_BOOSTMODE_PSB,

	/* Must be last entry */
	SCHEDTUNE_BOOSTMODE_COUNT,
};

extern int schedtune_taskgroup_margin(struct task_struct *tsk);
extern int schedtune_taskgroup_boostmode(struct task_struct *tsk);
extern int schedtune_root_margin(void);
extern int schedtune_root_boostmode(void);

#else

static int schedtune_taskgroup_margin(struct task_struct *tsk)
{
	return 0;
}

static int schedtune_taskgroup_boostmode(struct task_struct *tsk);

{
	return SCHEDTUNE_BOOSTMODE_NONE;
}

static int schedtune_root_margin(void)
{
	return 0;
}

static int schedtune_root_boostmode(void)
{
	return SCHEDTUNE_BOOSTMODE_NONE;
}

#endif

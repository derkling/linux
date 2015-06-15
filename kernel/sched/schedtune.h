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

extern int schedtune_accept_deltas(int nrg_delta, int cap_delta,
		struct task_struct *task);

extern int schedtune_normalize_energy(int energy);

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

static int schedtune_accept_deltas(int nrg_delta, int cap_delta,
		struct task_struct *task)
{
	if (nrg_delta < 0)
		return INT_MAX;
	return -INT_MAX;
}

static int schedtune_normalize_energy(int energy)
{
	return energy;
}

#endif

#ifdef CONFIG_CGROUP_SCHEDTUNE

extern int schedtune_taskgroup_margin(struct task_struct *tsk);

#else

static int schedtune_taskgroup_margin(struct task_struct *tsk)
{
	return 0;
}

#endif

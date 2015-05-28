#ifdef CONFIG_CGROUP_SCHEDTUNE

extern void schedtune_margin(struct task_struct *tsk);

#else

static inline void schedtune_margin(struct task_struct *tsk)
{
}

#endif

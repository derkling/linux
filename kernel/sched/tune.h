/*
 * Scheduler Tunability (SchedTune) Extensions for CFS
 *
 * Copyright (C) 2016 ARM Ltd, Patrick Bellasi <patrick.bellasi@arm.com>
 */

#ifdef CONFIG_SCHED_TUNE

#include <linux/reciprocal_div.h>

#ifdef CONFIG_CGROUP_SCHEDTUNE

int schedtune_cpu_boost(int cpu);

#else /* CONFIG_CGROUP_SCHEDTUNE */

#define schedtune_cpu_boost(cpu)  get_sysctl_sched_cfs_boost()

#endif /* CONFIG_CGROUP_SCHEDTUNE */

#else /* CONFIG_SCHED_TUNE */

#define schedtune_cpu_boost(cpu)  0

#endif /* CONFIG_SCHED_TUNE */

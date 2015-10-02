#ifndef __CPUHOTPLUG_H
#define __CPUHOTPLUG_H

enum cpuhp_state {
	/*
	 * P: Priority of the callback
	 * S: Startup function or NULL for none
	 * T: Teardown function or NULL for NONE
	 * C: kind of callback Configuration:
	 *	C: Compile time
	 *	I: runtime and Invoked
	 *	P: runtime and not invoked (Passive)
	 */
	CPUHP_OFFLINE,
	CPUHP_CREATE_THREADS,		/* P: _cpu_up S: smpboot_create_threads T: NULL C: C */
	CPUHP_PERF_X86_UNCORE_PREP,	/* P: 21 S: uncore_prepare_cpu T: uncore_dead_cpu C: I */
	CPUHP_PERF_X86_AMD_UNCORE_PREP,	/* P: 21 S: amd_uncore_cpu_up_prepare T: amd_uncore_cpu_dead C: I */
	CPUHP_PERF_X86_RAPL_PREP,	/* P: 20 S: rapl_cpu_prepare T: rapl_cpu_kfree C: I */
	CPUHP_PERF_X86_PREPARE,		/* P: 20 S: x86_pmu_prepare_cpu T: x86_pmu_dead_cpu C: I */
	CPUHP_PERF_BFIN,		/* P: 20 S: bfin_pmu_prepare_cpu T: NULL C: I */
	CPUHP_PERF_POWER,		/* P: 20 S: power_pmu_prepare_cpu T: NULL C: I */
	CPUHP_PERF_SUPERH,		/* P: 20 S: sh_pmu_prepare_cpu T: NULL C: I */
	CPUHP_PERF_PREPARE,		/* P: 20 S: perf_event_init_cpu T: perf_event_exit_cpu C: C */
	CPUHP_NOTIFY_PREPARE,		/* P: CPU_UP_PREPARE S: notify_prepare: T: NULL C: C */
	CPUHP_NOTIFY_DEAD,		/* P: CPU_DEAD S: NULL: T: notify_dead C: C */
	CPUHP_SCHED_DEAD,		/* P: INT_MAX S: NULL T: sched_dead_numa_cpu C: P */
	CPUHP_BRINGUP_CPU,		/* P: __cpu_up S: bringup_cpu T: NULL C: C */
	CPUHP_AP_OFFLINE,
	CPUHP_AP_SCHED_STARTING,		/* P: INT_MAX S: sched_cpu_active_starting T: NULL C: C */
	CPUHP_AP_PERF_X86_UNCORE_STARTING,	/* P: 21 S: uncore_starting_cpu T: NULL C: I */
	CPUHP_AP_PERF_X86_AMD_UNCORE_STARTING,	/* P: 21 S: amd_uncore_cpu_starting T: NULL C: I */
	CPUHP_AP_PERF_X86_RAPL_STARTING,	/* P: 20 S: rapl_cpu_init T: rapl_cpu_dying C: I */
	CPUHP_AP_PERF_X86_CSTATE_STARTING,	/* P: 20 S: cstate_cpu_init T: NULL C: I */
	CPUHP_AP_PERF_X86_CQM_STARTING,		/* P: 20 S: intel_cqm_cpu_startingT: NULL C: I */
	CPUHP_AP_PERF_X86_AMD_IBS_STARTING,	/* P: 20 S: x86_pmu_amd_ibs_starting_cpu T: x86_pmu_amd_ibs_dying_cpu C: I */
	CPUHP_AP_PERF_X86_STARTING,		/* P: 20 S: x86_pmu_starting_cpu T: x86_pmu_dying_cpu C: I */
	CPUHP_AP_NOTIFY_STARTING,	/* P: CPU_STARTING S: notify_starting T: NULL C: C */
	CPUHP_AP_NOTIFY_DYING,		/* P: CPU_DYING S: NULL T: notify_dying C: C */
	CPUHP_AP_MAX,
	CPUHP_TEARDOWN_CPU,		/* P: __cpu_die S: NULL T: takedown_cpu C: C */
	CPUHP_PERCPU_THREADS,		/* P: 9 S: smpboot_unpark_threads T: smpboot_park_threads: C: C */
	CPUHP_SCHED_ONLINE,		/* P: INT_MAX S: sched_online_cpu T: NULL C: I */
	CPUHP_SCHED_ONLINE_NUMA,	/* P: INT_MAX S: sched_online_numa_cpu T: NULL C: I */
	CPUHP_SCHED_CPUSET_ONLINE,	/* P: INT_MAX-1 S: cpuset_cpu_active T: NULL C: I */
	CPUHP_SCHED_CPUSET_OFFLINE,	/* P: INT_MIN S: NULL T: cpuset_cpu_inactive C: I */
	CPUHP_SCHED_OFFLINE,		/* P: INT_MIN+1 S: NULL T: sched_offline_cpu C: I */
	CPUHP_PERF_X86_UNCORE_ONLINE,	/* P: 21 S: uncore_online_cpu T: uncore_offline_cpu C: I */
	CPUHP_PERF_X86_AMD_UNCORE_ONLINE, /* P: 21 S: amd_uncore_cpu_online T: amd_uncore_cpu_down_prepare C: I */
	CPUHP_PERF_ONLINE,		/* P: 20 S: perf_event_init_cpu T: perf_event_exit_cpu C: C */
	CPUHP_PERF_ARM_CCI_ONLINE,	/* P: 21 S: NULL T: cci_pmu_offline_cpu C: I */
	CPUHP_PERF_ARM_CCN_ONLINE,	/* P: 21 S: NULL T: arm_ccn_pmu_offline_cpu C: I */
	CPUHP_PERF_X86_ONLINE,		/* P: 20 S: x86_pmu_online_cpu T: NULL C: I */
	CPUHP_PERF_X86_RAPL_ONLINE,	/* P: 20 S: rapl_cpu_kfree T: rapl_cpu_exit C: I */
	CPUHP_PERF_X86_CQM_ONLINE,	/* P: 20 S: NULL T: intel_cqm_cpu_exit C: I */
	CPUHP_PERF_X86_CSTATE_ONLINE,	/* P: 20 S: NULL T: cstate_cpu_exit C: I */
	CPUHP_PERF_S390_CF_ONLINE,	/* P: 20 S: s390_pmu_online_cpu T: s390_pmu_offline_cpu C: I */
	CPUHP_PERF_S390_SF_ONLINE,	/* P: 20 S: s390_pmu_sf_online_cpu T: s390_pmu_sf_offline_cpu C: I */
	CPUHP_NOTIFY_ONLINE,		/* P: CPU_ONLINE S: notify_online T: NULL, C: C */
	CPUHP_NOTIFY_DOWN_PREPARE,	/* P: CPU_DOWN_PREPARE S: NULL T: notify_down_prepare C: C */
	CPUHP_MAX,
};

#define CPUHP_ONLINE CPUHP_MAX

int __cpuhp_setup_state(enum cpuhp_state state, bool invoke,
			int (*startup)(unsigned int cpu),
			int (*teardown)(unsigned int cpu));

/**
 * cpuhp_setup_state - Setup hotplug state callbacks with calling the callbacks
 * @state:	The state for which the calls are installed
 * @startup:	startup callback function
 * @teardown:	teardown callback function
 *
 * Installs the callback functions and invokes the startup callback on
 * the present cpus which have already reached the @state.
 */
static inline int
cpuhp_setup_state(enum cpuhp_state state, int (*startup)(unsigned int cpu),
		  int (*teardown)(unsigned int cpu))
{
	return __cpuhp_setup_state(state, true, startup, teardown);
}

/**
 * cpuhp_setup_state_nocalls - Setup hotplug state callbacks without calling the
 *			       callbacks
 * @state:	The state for which the calls are installed
 * @startup:	startup callback function
 * @teardown:	teardown callback function
 *
 * No calls are executed. NOP if SMP=n or HOTPLUG_CPU=n
 */
#if defined(CONFIG_SMP) && defined(CONFIG_HOTPLUG_CPU)
static inline int
cpuhp_setup_state_nocalls(enum cpuhp_state state,
			 int (*startup)(unsigned int cpu),
			 int (*teardown)(unsigned int cpu))
{
	return __cpuhp_setup_state(state, false, startup, teardown);
}
#else
static inline int
cpuhp_setup_state_nocalls(enum cpuhp_state state,
			 int (*startup)(unsigned int cpu),
			 int (*teardown)(unsigned int cpu))
{
	return 0;
}
#endif

void __cpuhp_remove_state(enum cpuhp_state state, bool invoke);

/**
 * cpuhp_remove_state - Remove hotplug state callbacks and invoke the teardown
 * @state:	The state for which the calls are removed
 *
 * Removes the callback functions and invokes the teardown callback on
 * the present cpus which have already reached the @state.
 */
static inline void cpuhp_remove_state(enum cpuhp_state state)
{
	__cpuhp_remove_state(state, true);
}

/**
 * cpuhp_remove_state_nocalls - Remove hotplug state callbacks without invoking
 *				teardown
 * @state:	The state for which the calls are removed
 */
static inline void cpuhp_remove_state_nocalls(enum cpuhp_state state)
{
	__cpuhp_remove_state(state, false);
}

 /* Performance counter hotplug functions */
#ifdef CONFIG_PERF_EVENTS
int perf_event_init_cpu(unsigned int cpu);
int perf_event_exit_cpu(unsigned int cpu);
#else
#define perf_event_init_cpu	NULL
#define perf_event_exit_cpu	NULL
#endif

#endif

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
	CPUHP_NOTIFY_PREPARE,		/* P: CPU_UP_PREPARE S: notify_prepare: T: NULL C: C */
	CPUHP_NOTIFY_DEAD,		/* P: CPU_DEAD S: NULL: T: notify_dead C: C */
	CPUHP_BRINGUP_CPU,		/* P: __cpu_up S: bringup_cpu T: NULL C: C */
	CPUHP_AP_OFFLINE,
	CPUHP_AP_NOTIFY_STARTING,	/* P: CPU_STARTING S: notify_starting T: NULL C: C */
	CPUHP_AP_NOTIFY_DYING,		/* P: CPU_DYING S: NULL T: notify_dying C: C */
	CPUHP_AP_MAX,
	CPUHP_TEARDOWN_CPU,		/* P: __cpu_die S: NULL T: takedown_cpu C: C */
	CPUHP_PERCPU_THREADS,		/* P: 9 S: smpboot_unpark_threads T: smpboot_park_threads: C: C */
	CPUHP_NOTIFY_ONLINE,		/* P: CPU_ONLINE S: notify_online T: NULL, C: C */
	CPUHP_NOTIFY_DOWN_PREPARE,	/* P: CPU_DOWN_PREPARE S: NULL T: notify_down_prepare C: C */
	CPUHP_MAX,
};

#define CPUHP_ONLINE CPUHP_MAX

#endif

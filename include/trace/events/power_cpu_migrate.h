#undef TRACE_SYSTEM
#define TRACE_SYSTEM power

#if !defined(_TRACE_POWER_CPU_MIGRATE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_POWER_CPU_MIGRATE_H

#include <linux/tracepoint.h>

#define __cpu_migrate_proto			\
	TP_PROTO(u64 timestamp,			\
		 u32 from_phys_cpu,		\
		 u32 from_phys_cluster,		\
		 u32 to_phys_cpu,		\
		 u32 to_phys_cluster)
#define __cpu_migrate_args			\
	TP_ARGS(timestamp,			\
		from_phys_cpu,			\
		from_phys_cluster,		\
		to_phys_cpu,			\
		to_phys_cluster)

DECLARE_EVENT_CLASS(cpu_migrate,

	__cpu_migrate_proto,
	__cpu_migrate_args,

	TP_STRUCT__entry(
		__field(u64,	timestamp		)
		__field(u32,	from_phys_cpu		)
		__field(u32,	from_phys_cluster	)
		__field(u32,	to_phys_cpu		)
		__field(u32,	to_phys_cluster		)
		__field(u32,	local_cpu		)
	),

	TP_fast_assign(
		__entry->timestamp = timestamp;
		__entry->from_phys_cpu = from_phys_cpu;
		__entry->from_phys_cluster = from_phys_cluster;
		__entry->to_phys_cpu = to_phys_cpu;
		__entry->to_phys_cluster = to_phys_cluster;
	),

	TP_printk("timestamp=%llu from_phys_cpu=%lu from_phys_cluster=%lu to_phys_cpu=%lu to_phys_cluster=%lu",
		(unsigned long long)__entry->timestamp,
		(unsigned long)__entry->from_phys_cpu,
		(unsigned long)__entry->from_phys_cluster,
		(unsigned long)__entry->to_phys_cpu,
		(unsigned long)__entry->to_phys_cluster
	)
);

#define __define_cpu_migrate_event(name)		\
	DEFINE_EVENT(cpu_migrate, cpu_migrate_##name,	\
		__cpu_migrate_proto,			\
		__cpu_migrate_args			\
	)

__define_cpu_migrate_event(begin);
__define_cpu_migrate_event(finish);

#undef __define_cpu_migrate
#undef __cpu_migrate_proto
#undef __cpu_migrate_args

/* This file can get included multiple times, TRACE_HEADER_MULTI_READ at top */
#ifndef _PWR_CPU_MIGRATE_EVENT_AVOID_DOUBLE_DEFINING
#define _PWR_CPU_MIGRATE_EVENT_AVOID_DOUBLE_DEFINING

/*
 * Set from_phys_cpu and to_phys_cpu to CPU_MIGRATE_ALL_CPUS to indicate
 * a whole-cluster migration:
 */
#define CPU_MIGRATE_ALL_CPUS 0x80000000U
#endif

#endif /* _TRACE_POWER_CPU_MIGRATE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE power_cpu_migrate
#include <trace/define_trace.h>

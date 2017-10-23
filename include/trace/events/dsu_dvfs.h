#undef TRACE_SYSTEM
#define TRACE_SYSTEM dsu_dvfs

#if !defined(_TRACE_DSU_DVFS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DSU_DVFS_H

#include <linux/tracepoint.h>

TRACE_EVENT(dsu_dvfs_dev_status,
	TP_PROTO(int dsu_id, int cpu, unsigned long cur_freq,
		 unsigned long prev_freq, unsigned long prev_min),
	TP_ARGS(dsu_id, cpu, cur_freq, prev_freq, prev_min),
	TP_STRUCT__entry(
		__field(int, dsu_id)
		__field(int, cpu)
		__field(unsigned long, cur_freq)
		__field(unsigned long, prev_freq)
		__field(unsigned long, prev_min)
	),
	TP_fast_assign(
		__entry->dsu_id = dsu_id;
		__entry->cpu = cpu;
		__entry->cur_freq = cur_freq;
		__entry->prev_freq = prev_freq;
		__entry->prev_min = prev_min;
	),

	TP_printk("dsu_dev_id=%d cpu=%d cur_freq=%lu prv_freq=%lu prev_min=%lu",
		  __entry->dsu_id, __entry->cpu, __entry->cur_freq,
		  __entry->prev_freq, __entry->prev_min)
);

TRACE_EVENT(dsu_dvfs_gov_status,
	TP_PROTO(unsigned long freq),
	TP_ARGS(freq),
	TP_STRUCT__entry(__field(unsigned long, freq)),
	TP_fast_assign(__entry->freq = freq),

	TP_printk("cur_freq=%lu", __entry->freq)
);

TRACE_EVENT(dsu_dvfs_gov_status_each,
	TP_PROTO(int cpu, unsigned long cpu_freq, unsigned long dsu_freq),
	TP_ARGS(cpu, cpu_freq, dsu_freq),
	TP_STRUCT__entry(
		__field(int, cpu)
		__field(unsigned long, cpu_freq)
		__field(unsigned long, dsu_freq)
	),
	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->cpu_freq = cpu_freq;
		__entry->dsu_freq = dsu_freq;
	),

	TP_printk("cpu=%d cpu_freq=%lu dsu_freq=%lu",
		  __entry->cpu, __entry->cpu_freq, __entry->dsu_freq)
);

#endif /* _TRACE_DSU_DVFS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

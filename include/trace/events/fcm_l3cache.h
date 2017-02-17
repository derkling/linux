#undef TRACE_SYSTEM
#define TRACE_SYSTEM fcm_l3cache

#if !defined(_TRACE_FCM_L3CACHE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_FCM_L3CACHE_H

#include <linux/tracepoint.h>

TRACE_EVENT(fcm_l3cache_dev_status,
	TP_PROTO(int fcm_id, unsigned long hits, unsigned long misses,
		 unsigned long freq, unsigned long busy_time,
		 unsigned long total_time),
	TP_ARGS(fcm_id, hits, misses, freq, busy_time, total_time),
	TP_STRUCT__entry(
		__field(int, fcm_id)
		__field(unsigned long, hits)
		__field(unsigned long, misses)
		__field(unsigned long, freq)
		__field(unsigned long, busy_time)
		__field(unsigned long, total_time)
	),
	TP_fast_assign(
		__entry->fcm_id = fcm_id;
		__entry->hits = hits;
		__entry->misses = misses;
		__entry->freq = freq;
		__entry->busy_time = busy_time;
		__entry->total_time = total_time;
	),

	TP_printk("fcm_dev_id=%d hits=%lu misses=%lu cur_freq=%lu busy_time=%lu total_time=%lu",
		  __entry->fcm_id, __entry->hits, __entry->misses,
		  __entry->freq, __entry->busy_time, __entry->total_time)
);
#endif /* _TRACE_FCM_L3CACHE_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM sched_cbs

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE cbs_trace

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../kernel/sched

#if !defined(_TRACE_CBS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CBS_H

#include <linux/sched.h>
#include <linux/tracepoint.h>
#include <linux/binfmts.h>

/*
 * Tracepoint for CBS burst completion
 */
TRACE_EVENT(cbs_burst,

	TP_PROTO(struct sched_cbs_entity *cbs_se),

	TP_ARGS(cbs_se),

	TP_STRUCT__entry(
		__field(	u32,	round_quota     )
		__field(	u32,	burst_tq_sp	)
		__field(	u32,	burst_tq	)
		__field(	u64,	burst_interval	)
		__field(	u64,	exec_runtime	)
	),

	TP_fast_assign(
		__entry->round_quota	= cbs_se->round_quota;
		__entry->burst_tq_sp	= cbs_se->burst_tq_sp;
		__entry->burst_tq	= cbs_se->burst_tq;
		__entry->burst_interval	= cbs_se->burst_interval;
		__entry->exec_runtime	= cbs_se->exec_runtime;
	),


	TP_printk("exec=%Lu | Rq=%u Tt_SP=%u Tb=%Lu Tt=%u",
		__entry->exec_runtime,
		__entry->round_quota,
		__entry->burst_tq_sp,
		__entry->burst_interval,
		__entry->burst_tq)
);

/*
 * Tracepoint for CBS round completion
 */
TRACE_EVENT(cbs_round,

	TP_PROTO(struct cbs_rq *cbs_rq),

	TP_ARGS(cbs_rq),

	TP_STRUCT__entry(
		__field( unsigned int,	nr_running		)
		__field( unsigned long,	load			)
		__field( unsigned long,	load_next		)
		__field( u64,		round_tq_sp		)
		__field( u64,		round_tq_next		)
		__field( u64,		round_tq		)
		__field( s64,		round_tq_correction	)
		__field( s64,		round_tq_error		)
		__field( u64,		exec_runtime		)
		__field( u8,		all_saturated		)
		__field( u8,		clamp_rt		)
		__field( u8,		needs_reinit 		)
	),

	TP_fast_assign(
		__entry->nr_running		= cbs_rq->nr_running;
		__entry->load			= cbs_rq->load.weight;
		__entry->load_next		= cbs_rq->load_next.weight;
		__entry->round_tq_sp		= cbs_rq->round_tq_sp;
		__entry->round_tq		= cbs_rq->round_tq;
		__entry->round_tq_next		= cbs_rq->round_tq_next;
		__entry->round_tq_correction	= cbs_rq->round_tq_correction;
		__entry->round_tq_error		= cbs_rq->round_tq_error;
		__entry->exec_runtime		= cbs_rq->exec_runtime;
		__entry->all_saturated		= cbs_rq->all_saturated;
		__entry->clamp_rt		= cbs_rq->clamp_rt;
		__entry->needs_reinit		= cbs_rq->needs_reinit;
	),


	TP_printk("exec=%Lu | Lw=%lu Rt=%Lu [%c] Re=%Lu ===> Nr=%u Lw=%lu Rt_SP=%Lu [%c%c] Rt_corr=%Ld Rt_next=%Ld ",
		__entry->exec_runtime,
		__entry->load,
		__entry->round_tq,
		__entry->clamp_rt ? 'c' : '-',
		__entry->round_tq_error,
		__entry->nr_running,
		__entry->load_next,
		__entry->round_tq_sp,
		__entry->needs_reinit ? 'r' : '-',
		__entry->all_saturated ? 's' : '-',
		__entry->round_tq_correction,
		__entry->round_tq_next)
);

#endif /* _TRACE_CBS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>


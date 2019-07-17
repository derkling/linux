/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM sched

#if !defined(_SCHED_EVENTS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _SCHED_EVENTS_H

#define PATH_SIZE		128
#define SPAN_SIZE		128/4	/* assuming a max of 128 cpu system! */

#include <linux/tracepoint.h>

TRACE_EVENT(sched_load_cfs_rq,

	TP_PROTO(int cpu, char *path, const struct sched_avg *avg),

	TP_ARGS(cpu, path, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__array(	char,		path,	PATH_SIZE	)
		__field(	unsigned long,	load			)
		__field(	unsigned long,	rbl_load		)
		__field(	unsigned long,	util			)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		strlcpy(__entry->path, path, PATH_SIZE);
		__entry->load		= avg->load_avg;
		__entry->rbl_load	= avg->runnable_load_avg;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d path=%s load=%lu rbl_load=%lu util=%lu",
		  __entry->cpu, __entry->path, __entry->load,
		  __entry->rbl_load,__entry->util)
);

DECLARE_EVENT_CLASS(sched_pelt_rq_template,

	TP_PROTO(int cpu, const struct sched_avg *avg),

	TP_ARGS(cpu, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__field(	unsigned long,	load			)
		__field(	unsigned long,	rbl_load		)
		__field(	unsigned long,	util			)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->load		= avg->load_avg;
		__entry->rbl_load	= avg->runnable_load_avg;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d load=%lu rbl_load=%lu util=%lu",
		  __entry->cpu, __entry->load,
		  __entry->rbl_load,__entry->util)
);

DEFINE_EVENT(sched_pelt_rq_template, sched_pelt_rt,
	TP_PROTO(int cpu, const struct sched_avg *avg),
	TP_ARGS(cpu, avg));

DEFINE_EVENT(sched_pelt_rq_template, sched_pelt_dl,
	TP_PROTO(int cpu, const struct sched_avg *avg),
	TP_ARGS(cpu, avg));

DEFINE_EVENT(sched_pelt_rq_template, sched_pelt_irq,
	TP_PROTO(int cpu, const struct sched_avg *avg),
	TP_ARGS(cpu, avg));

TRACE_EVENT(sched_load_se,

	TP_PROTO(int cpu, char *path, char *comm, int pid, const struct sched_avg *avg),

	TP_ARGS(cpu, path, comm, pid, avg),

	TP_STRUCT__entry(
		__field(	int,		cpu			)
		__array(	char,		path,	PATH_SIZE	)
		__array(	char,		comm,	TASK_COMM_LEN	)
		__field(	int,		pid			)
		__field(	unsigned long,	load			)
		__field(	unsigned long,	rbl_load		)
		__field(	unsigned long,	util			)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		strlcpy(__entry->path, path, PATH_SIZE);
		strlcpy(__entry->comm, comm, TASK_COMM_LEN);
		__entry->pid		= pid;
		__entry->load		= avg->load_avg;
		__entry->rbl_load	= avg->runnable_load_avg;
		__entry->util		= avg->util_avg;
	),

	TP_printk("cpu=%d path=%s comm=%s pid=%d load=%lu rbl_load=%lu util=%lu",
		  __entry->cpu, __entry->path, __entry->comm, __entry->pid,
		  __entry->load, __entry->rbl_load,__entry->util)
);


#ifdef CONFIG_UCLAMP_TASK

TRACE_EVENT(uclamp_se,

	TP_PROTO(struct task_struct *p),

	TP_ARGS(p),

	TP_STRUCT__entry(
		__field(	pid_t,	pid			)
		__array(	 char,	comm,   TASK_COMM_LEN	)
		__field(	  int,	cpu			)
		__field(unsigned long,	util_avg		)
		__field(unsigned long,	uclamp_avg		)
		__field(unsigned long,	uclamp_min		)
		__field(unsigned long,	uclamp_max		)
	),

	TP_fast_assign(
		__entry->pid            = p->pid;
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__entry->cpu            = p->cpu;
		__entry->util_avg       = p->se.avg.util_avg;
		__entry->uclamp_avg     = uclamp_util(cpu_rq(p->cpu), p->se.avg.util_avg);
		__entry->uclamp_min     = cpu_rq(p->cpu)->uclamp[UCLAMP_MIN].value;
		__entry->uclamp_max     = cpu_rq(p->cpu)->uclamp[UCLAMP_MAX].value;
		),

	TP_printk("pid=%d comm=%s cpu=%d util_avg=%lu uclamp_avg=%lu "
		  "uclamp_min=%lu uclamp_max=%lu",
		  __entry->pid, __entry->comm, __entry->cpu,
		  __entry->util_avg, __entry->uclamp_avg,
		  __entry->uclamp_min, __entry->uclamp_max)
);

TRACE_EVENT(uclamp_cfs,

	TP_PROTO(struct rq *rq),

	TP_ARGS(rq),

	TP_STRUCT__entry(
		__field(	  int,	cpu			)
		__field(unsigned long,	util_avg		)
		__field(unsigned long,	uclamp_avg		)
		__field(unsigned long,	uclamp_min		)
		__field(unsigned long,	uclamp_max		)
	),

	TP_fast_assign(
		__entry->cpu            = rq->cpu;
		__entry->util_avg       = rq->cfs.avg.util_avg;
		__entry->uclamp_avg     = uclamp_util(rq, rq->cfs.avg.util_avg);
		__entry->uclamp_min     = rq->uclamp[UCLAMP_MIN].value;
		__entry->uclamp_max     = rq->uclamp[UCLAMP_MAX].value;
		),

	TP_printk("cpu=%d util_avg=%lu uclamp_avg=%lu "
		  "uclamp_min=%lu uclamp_max=%lu",
		  __entry->cpu, __entry->util_avg, __entry->uclamp_avg,
		  __entry->uclamp_min, __entry->uclamp_max)
);

TRACE_EVENT(uclamp_rt,

	    TP_PROTO(struct rq *rq),

	    TP_ARGS(rq),

	    TP_STRUCT__entry(
		    __field(	      int,	cpu			)
		    __field(unsigned long,	util_avg		)
		    __field(unsigned long,	uclamp_avg		)
		    __field(unsigned long,	uclamp_min		)
		    __field(unsigned long,	uclamp_max		)
		    ),

	    TP_fast_assign(
		    __entry->cpu            = rq->cpu;
		    __entry->util_avg       = rq->avg_rt.util_avg;
		    __entry->uclamp_avg     = uclamp_util(rq, rq->avg_rt.util_avg);
		    __entry->uclamp_min     = rq->uclamp[UCLAMP_MIN].value;
		    __entry->uclamp_max     = rq->uclamp[UCLAMP_MAX].value;
		    ),

	    TP_printk("cpu=%d util_avg=%lu uclamp_avg=%lu "
		      "uclamp_min=%lu uclamp_max=%lu",
		      __entry->cpu, __entry->util_avg, __entry->uclamp_avg,
		      __entry->uclamp_min, __entry->uclamp_max)
	);
#else
#define trace_uclamp_se(p) while(false) {}
#define trace_uclamp_rt(rq) while(false) {}
#define trace_uclamp_cfs(rq) while(false) {}
#endif /* CONFIG_UCLAMP_TASK */

TRACE_EVENT(sched_overutilized,

	TP_PROTO(int overutilized, char *span),

	TP_ARGS(overutilized, span),

	TP_STRUCT__entry(
		__field(	int,		overutilized		)
		__array(	char,		span,	SPAN_SIZE	)
	),

	TP_fast_assign(
		__entry->overutilized	= overutilized;
		strlcpy(__entry->span, span, SPAN_SIZE);
	),

	TP_printk("overutilized=%d span=0x%s",
		  __entry->overutilized, __entry->span)
);

#endif /* _SCHED_EVENTS_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE sched_events
#include <trace/define_trace.h>

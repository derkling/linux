/*
 *  Copyright (C)  2015 Steve Muckle <steve.muckle@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM sched_freq

#if !defined(_TRACE_SCHED_FREQ_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SCHED_FREQ_H

#include <linux/sched.h>
#include <linux/tracepoint.h>

TRACE_EVENT(sched_freq_throttled,
	    TP_PROTO(unsigned int rem),
	    TP_ARGS(rem),
	    TP_STRUCT__entry(
		    __field(	unsigned int,	rem)
	    ),
	    TP_fast_assign(
		    __entry->rem = rem;
	    ),
	    TP_printk("throttled - %d usec remaining", __entry->rem)
);

TRACE_EVENT(sched_freq_request_opp,
	    TP_PROTO(int cpu,
		     unsigned long capacity,
		     unsigned int freq_new,
		     unsigned int requested_freq),
	    TP_ARGS(cpu, capacity, freq_new, requested_freq),
	    TP_STRUCT__entry(
		    __field(	int,		cpu)
		    __field(	unsigned long,	capacity)
		    __field(	unsigned int,	freq_new)
		    __field(	unsigned int,	requested_freq)
		    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->capacity = capacity;
		    __entry->freq_new = freq_new;
		    __entry->requested_freq = requested_freq;
		    ),
	    TP_printk("cpu %d cap change, cluster cap request %ld => OPP %d "
		      "(cur %d)",
		      __entry->cpu, __entry->capacity, __entry->freq_new,
		      __entry->requested_freq)
);

TRACE_EVENT(sched_freq_update_capacity,
	    TP_PROTO(int cpu,
		     struct sched_capacity_reqs *scr,
		     unsigned long new_capacity),
	    TP_ARGS(cpu, scr, new_capacity),
	    TP_STRUCT__entry(
		    __field(	int,		cpu)
		    __field(	unsigned long,	cfs)
		    __field(	unsigned long,	rt)
		    __field(	unsigned long,	dl)
		    __field(	unsigned long,	dl_min)
		    __field(	unsigned long,	total)
		    __field(	unsigned long,	new_total)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->cfs = scr->cfs;
		    __entry->rt = scr->rt;
		    __entry->dl = scr->dl;
		    __entry->dl_min = scr->dl_min;
		    __entry->total = scr->total;
		    __entry->new_total = new_capacity;
	    ),
	    TP_printk("cpu=%d cfs=%ld rt=%ld dl=%ld dl_min=%ld "
		      "old_tot=%ld new_tot=%ld",
		      __entry->cpu, __entry->cfs, __entry->rt, __entry->dl,
		      __entry->dl_min, __entry->total, __entry->new_total)
);

#endif /* _TRACE_SCHED_FREQ_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

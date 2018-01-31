/*
 *  Copyright (C)  2015 Steve Muckle <smuckle@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM cpufreq_sched

#if !defined(_TRACE_CPUFREQ_SCHED_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CPUFREQ_SCHED_H

#include <linux/sched.h>
#include <linux/tracepoint.h>

TRACE_EVENT(cpufreq_sched_throttled,
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

TRACE_EVENT(cpufreq_sched_request_opp,
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

#endif /* _TRACE_CPUFREQ_SCHED_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

/*
 * trace context switch
 *
 * Copyright (C) 2007 Steven Rostedt <srostedt@redhat.com>
 *
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/kallsyms.h>
#include <linux/uaccess.h>
#include <linux/ftrace.h>
#include <trace/events/sched.h>

#include "trace.h"

static struct trace_array	*tperf_trace;
static int __read_mostly	tracer_enabled;
static int			sched_ref;
static DEFINE_MUTEX(task_perf_mutex);
static int			sched_stopped;


void
tracing_task_perf_trace(struct trace_array *tr,
			   struct task_struct *prev,
			   unsigned long flags, int pc)
{
	struct ftrace_event_call *call = &event_task_perf;
	struct ring_buffer *buffer = tr->buffer;
	struct ring_buffer_event *event;
	struct task_perf_entry *entry;

	event = trace_buffer_lock_reserve(buffer, TRACE_PERF,
					  sizeof(*entry), flags, pc);
	if (!event)
		return;

	entry	= ring_buffer_event_data(event);
	entry->pid = prev->pid;
	/* The next two counter must collect perf event counter values */
	entry->counter[0] = 0;
	entry->counter[1] = 1;

	if (!filter_check_discard(call, entry, buffer, event))
		trace_buffer_unlock_commit(buffer, event, flags, pc);
}

static void
probe_task_perf(void *ignore, struct task_struct *prev)
{
	struct trace_array_cpu *data;
	unsigned long flags;
	int cpu;
	int pc;

	if (unlikely(!sched_ref))
		return;

	tracing_record_cmdline(prev);

	if (!tracer_enabled || sched_stopped)
		return;

	pc = preempt_count();
	local_irq_save(flags);
	cpu = raw_smp_processor_id();
	data = tperf_trace->data[cpu];

	if (likely(!atomic_read(&data->disabled)))
		tracing_task_perf_trace(tperf_trace, prev, flags, pc);

	local_irq_restore(flags);
}

static int tracing_task_perf_register(void)
{
	int ret;

	ret = register_trace_task_perf(probe_task_perf, NULL);
	if (ret) {
		pr_info("task perf trace: Couldn't activate tracepoint"
			" probe to kernel_sched_switch\n");
	}

	return ret;
}

static void tracing_task_perf_unregister(void)
{
	unregister_trace_task_perf(probe_task_perf, NULL);
}

static void tracing_start_task_perf(void)
{
	mutex_lock(&task_perf_mutex);
	if (!(sched_ref++))
		tracing_task_perf_register();
	mutex_unlock(&task_perf_mutex);
}

static void tracing_stop_task_perf(void)
{
	mutex_lock(&task_perf_mutex);
	if (!(--sched_ref))
		tracing_task_perf_unregister();
	mutex_unlock(&task_perf_mutex);
}

/*
void tracing_start_cmdline_record(void)
{
	tracing_start_task_perf();
}

void tracing_stop_cmdline_record(void)
{
	tracing_stop_task_perf();
}
*/

/**
 * tracing_start_task_perf_record - start tracing context switches
 *
 * Turns on context switch tracing for a tracer.
 */
void tracing_start_task_perf_record(void)
{
	if (unlikely(!tperf_trace)) {
		WARN_ON(1);
		return;
	}

	tracing_start_task_perf();

	mutex_lock(&task_perf_mutex);
	tracer_enabled++;
	mutex_unlock(&task_perf_mutex);
}

/**
 * tracing_stop_task_perf_record - start tracing context switches
 *
 * Turns off context switch tracing for a tracer.
 */
void tracing_stop_task_perf_record(void)
{
	mutex_lock(&task_perf_mutex);
	tracer_enabled--;
	WARN_ON(tracer_enabled < 0);
	mutex_unlock(&task_perf_mutex);

	tracing_stop_task_perf();
}

/**
 * tracing_task_perf_assign_trace - assign a trace array for ctx switch
 * @tr: trace array pointer to assign
 *
 * Some tracers might want to record the context switches in their
 * trace. This function lets those tracers assign the trace array
 * to use.
 */
void tracing_task_perf_assign_trace(struct trace_array *tr)
{
	tperf_trace = tr;
}

static void stop_sched_trace(struct trace_array *tr)
{
	tracing_stop_task_perf_record();
}

static int task_perf_trace_init(struct trace_array *tr)
{
	tperf_trace = tr;
	tracing_reset_online_cpus(tr);
	tracing_start_task_perf_record();
	return 0;
}

static void task_perf_trace_reset(struct trace_array *tr)
{
	if (sched_ref)
		stop_sched_trace(tr);
}

static void task_perf_trace_start(struct trace_array *tr)
{
	sched_stopped = 0;
}

static void task_perf_trace_stop(struct trace_array *tr)
{
	sched_stopped = 1;
}

static struct tracer task_perf_trace __read_mostly =
{
	.name		= "task_perf",
	.init		= task_perf_trace_init,
	.reset		= task_perf_trace_reset,
	.start		= task_perf_trace_start,
	.stop		= task_perf_trace_stop,
	.wait_pipe	= poll_wait_pipe,
#ifdef CONFIG_FTRACE_SELFTEST
/* This should be defined */
	.selftest	= 0, //trace_selftest_startup_task_perf,
#endif
};

__init static int init_task_perf_trace(void)
{
	return register_tracer(&task_perf_trace);
}
device_initcall(init_task_perf_trace);


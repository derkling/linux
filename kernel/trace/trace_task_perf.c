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

static struct trace_array	*tp_trace;
static int __read_mostly	tracer_enabled;
static int			sched_ref;
static DEFINE_MUTEX(task_perf_mutex);
static int			events_enabled;

/* NOTE: the perf_max_events global vars define the number of available
 * counters for the target architecture... */
#define MAX_EVENTS	2

struct task_perf_events {
	struct perf_event *evt[MAX_EVENTS];
};

static DEFINE_PER_CPU(struct task_perf_events, task_perf_ev);

void
tracing_task_perf_trace(struct trace_array *tr,
			   struct task_struct *prev,
			   unsigned long flags, int pc)
{
	struct ftrace_event_call *call = &event_task_perf;
	struct ring_buffer *buffer = tr->buffer;
	struct ring_buffer_event *event;
	struct task_perf_entry *entry;

	struct task_perf_events *evts;
	struct perf_event *evt;
	u64 enabled, running;
	int i;

	event = trace_buffer_lock_reserve(buffer, TRACE_PERF,
					  sizeof(*entry), flags, pc);
	if (!event)
		return;

	entry = ring_buffer_event_data(event);
	evts = &__get_cpu_var(task_perf_ev);

	/* Collect perf events
	 * Tricks: reset generic event counter to allow start counting next
	 * task */
	for (i=0; i<MAX_EVENTS; i++) {
		evt = evts->evt[i];
		entry->counter[i] = perf_event_read_value(evt, &enabled, &running);
		local64_set(&evt->count, 0);
	}

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

	if (!tracer_enabled || !events_enabled)
		return;

	pc = preempt_count();
	local_irq_save(flags);
	cpu = raw_smp_processor_id();
	data = tp_trace->data[cpu];

	if (likely(!atomic_read(&data->disabled)))
		tracing_task_perf_trace(tp_trace, prev, flags, pc);

	local_irq_restore(flags);
}

static int tracing_task_perf_register(void)
{
	int ret;

	ret = register_trace_task_perf(probe_task_perf, NULL);
	if (ret) {
		pr_warn("task perf trace: Couldn't activate tracepoint"
			" probe to kernel_sched_switch\n");
		return ret;
	}

	pr_info("task perf tracer: tracepoint enabled\n");
	return 0;
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
	if (unlikely(!tp_trace)) {
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
	tp_trace = tr;
}

static void stop_sched_trace(struct trace_array *tr)
{
	tracing_stop_task_perf_record();
}

static int task_perf_trace_init(struct trace_array *tr)
{
	tp_trace = tr;
	events_enabled = 0;
	tracing_reset_online_cpus(tr);
	tracing_start_task_perf_record();
	return 0;
}

static void task_perf_trace_reset(struct trace_array *tr)
{
	if (sched_ref)
		stop_sched_trace(tr);
}

/* See: tools/perf/util/parse-event.c:402 */
static struct perf_event_attr task_perf_event_attrs[MAX_EVENTS] = {
{
	.type		= PERF_TYPE_HW_CACHE,
	.config		= PERF_COUNT_HW_CACHE_L1D |
				(PERF_COUNT_HW_CACHE_OP_READ << 8) |
				(PERF_COUNT_HW_CACHE_RESULT_ACCESS << 16),
	.size		= sizeof(struct perf_event_attr),
	.pinned		= 1,
	.disabled	= 1,
	},
{
	.type		= PERF_TYPE_HW_CACHE,
	.config		= PERF_COUNT_HW_CACHE_L1D |
				(PERF_COUNT_HW_CACHE_OP_READ << 8) |
				(PERF_COUNT_HW_CACHE_RESULT_ACCESS << 16),
	.size		= sizeof(struct perf_event_attr),
	.pinned		= 1,
	.disabled	= 1,
	},
};

void task_perf_event_overflow_callback(struct perf_event *event, int nmi,
		 struct perf_sample_data *data,
		 struct pt_regs *regs)
{
	/* Ensure the interrupt never gets throttled */
	event->hw.interrupts = 0;
	trace_printk("task_perf: event overflow\n");

	/* Add a new event to the trace... or simple keep track of overflow
	 * numbers and produce them on the output trace (i.e
	 * num_overflows:cur_value */

	return;
}

static int
task_perf_setup_events(int cpu)
{
	struct task_perf_events *evts;
	struct perf_event *evt;
	struct perf_event_attr *attr;
	int i;

	evts = &per_cpu(task_perf_ev, cpu);

	/* Registering perf kernel counters */
	for (i=0; i<MAX_EVENTS; i++) {

		evt = evts->evt[i];
		attr = &task_perf_event_attrs[i];

		/* Is it already setup and enabled? */
		if (evt && evt->state > PERF_EVENT_STATE_OFF)
			continue;

		/* Must be created? */
		if (!evt) {

			/* Try to register using hardware perf events */
			evt = perf_event_create_kernel_counter(attr, cpu,
					-1, task_perf_event_overflow_callback);
			if (IS_ERR(evt)) {
				goto out_error;
			}

			pr_info("task perf tracer: enabled, takes one hw-pmu counter "
					"on CPU#%02i\n", cpu);

			evts->evt[i] = evt;
		}
	}

	/* All counters registered: thus now they can be enabled */
	for (i=0; i<MAX_EVENTS; i++) {
		evt = evts->evt[i];
		perf_event_enable(evt);
	}

	return 0;

out_error:
	/* Roll-back already registered counters */
	for (i--; i>=0; i--) {
		evt = evts->evt[i];
		perf_event_release_kernel(evt);
		evts->evt[i] = NULL;
	}
	return -1;

}


static int task_perf_enable_events(void)
{
	unsigned int cpu;
	int result;

	for_each_online_cpu(cpu) {
		result = task_perf_setup_events(cpu);
		if (result) {
			pr_warn("task perk tracer: failed to setup events on "
					"CPU#%02i\n", cpu);
			goto out_release;
		}
	}

	return 0;

out_release:

	/* TODO: free-up already configured events */
	return result;
}

static void
task_perf_release_events(int cpu)
{
	struct task_perf_events *evts;
	struct perf_event *evt;
	int i;

	evts = &per_cpu(task_perf_ev, cpu);

	/* Releasing all perf kernel counters */
	for (i=0; i<MAX_EVENTS; i++) {

		evt = evts->evt[i];

		/* Is it configured? */
		if (!evt)
			continue;

		/* Is it enabled? */
		if (evt->state > PERF_EVENT_STATE_OFF)
			perf_event_disable(evt);

		perf_event_release_kernel(evt);
		evts->evt[i] = NULL;

		pr_info("task perf tracer: disabled, releasing one hw-pmu counter "
				"on CPU#%02i\n", cpu);

	}

}

static int
task_perf_disable_events(void)
{
	unsigned int cpu;

	/* BUGFIX this... we should disable all the CPUs enabled so far
	   perhaps it is better to consider ALL the CPUs independently of
	   their on/off-line state?!?
	   We should build a CPUMASK at enable time and use it at disable time
	   */
	for_each_online_cpu(cpu) {
		task_perf_release_events(cpu);
	}

	return 0;
}


static void task_perf_trace_start(struct trace_array *tr)
{
	int result;

	result = task_perf_enable_events();
	if (result) {
		pr_warn("task perf tracer: failed to enable events, "
				"%d\n", result);
		return;
	}
	events_enabled = 1;
}

static void task_perf_trace_stop(struct trace_array *tr)
{
	events_enabled = 0;
	task_perf_disable_events();
}

/*
   Trace event formatting
*/
static enum print_line_t
trace_task_perf_print(struct trace_iterator *iter, int flags,
			   struct trace_event *event)
{
	struct task_perf_entry *field;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_printf(&iter->seq,
			      " %20llu %20llu\n",
			      field->counter[0],
			      field->counter[1]))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static struct trace_event_functions trace_task_perf_funcs = {
	.trace		= trace_task_perf_print,
};

static struct trace_event trace_task_perf_event = {
	.type		= TRACE_PERF,
	.funcs		= &trace_task_perf_funcs,
};

static struct tracer task_perf_trace __read_mostly =
{
	.name		= "sched_task_perf",
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

	if (!register_ftrace_event(&trace_task_perf_event)) {
		pr_warn("task perf tracer: failed to register "
				"ftrace event\n");
		return 1;
	}

	return register_tracer(&task_perf_trace);
}
device_initcall(init_task_perf_trace);


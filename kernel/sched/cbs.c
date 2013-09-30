/*
 * Control Based Scheduling Class (mapped to the SCHED_CBS policy)
 */

#include "sched.h"

/*******************************************************************************
 * CBS Policy API
 ******************************************************************************/

/*
 * Adding a task to the CBS ranqueue
 */
static void
enqueue_task_cbs(struct rq *rq, struct task_struct *p, int flags)
{

}

/*
 * Removing a task from the CBS ranqueue
 */
static void
dequeue_task_cbs(struct rq *rq, struct task_struct *p, int flags)
{

}

/*
 * Yield the
 */
static void
yield_task_cbs(struct rq *rq)
{

}

/*
 * Yield the
 */
static bool
yield_to_task_cbs(struct rq *rq, struct task_struct *p, bool preempt)
{
	// The CBS scheduler process tasks by Round, nothing special...
	return true;
}

/*
 * Preempt the current CBS managed task with a newly woken task, if needed
 */
static void
check_preempt_curr_cbs(struct rq *rq, struct task_struct *p, int flags)
{

}

/*
 * Add a CBS managed task into the runqueue
 */
static struct task_struct *
pick_next_task_cbs(struct rq *rq)
{
	return NULL;
}

/*
 * Remove a CBS managed task from the runqueue
 */
static void
put_prev_task_cbs(struct rq *rq, struct task_struct *prev)
{

}

#ifdef CONFIG_SMP

/*
 * Select the runqueue for a CBS managed task.
 */
static int
select_task_rq_cbs(struct task_struct *p, int sd_flag, int flags)
{
	int cpu;

	cpu = task_cpu(p);

	if (p->nr_cpus_allowed == 1)
		goto out;

out:
	return cpu;
}

/*
 * Migrate a CBS managed task to the specified CPU
 */
static void
migrate_task_rq_cbs(struct task_struct *p, int next_cpu)
{

}

/*
 * A CBS managed task is going to be scheduled
 */
static void
pre_schedule_cbs(struct rq *rq, struct task_struct *prev)
{

}

/*
 * A CBS managed task has been scheduled
 */
static void
post_schedule_cbs(struct rq *rq)
{

}

/*
 * CBS managed task waking-up
 */
static void
task_waking_cbs(struct task_struct *p)
{

}

/*
 * CBS managed task wakeup
 */
static void
task_woken_cbs(struct rq *rq, struct task_struct *p)
{

}

/*
 * Update the CPU mask of a CBS managed task
 */
static void
set_cpus_allowed_cbs(struct task_struct *p, const struct cpumask *new_mask)
{

}

/*
 * A CBS runqueue is put online
 */
static void
rq_online_cbs(struct rq *rq)
{

}

/*
 * A CBS runqueue is put offline
 */
static void
rq_offline_cbs(struct rq *rq)
{

}

#endif /* CONFIG_SMP */

/*
 *
 */
static void
set_curr_task_cbs(struct rq *rq)
{

}

/*
 * Tick event while executing a task
 */
static void
task_tick_cbs(struct rq *rq, struct task_struct *p, int queued)
{

}

/*
 * Fork event while executing a task
 */
static void
task_fork_cbs(struct task_struct *p)
{

}

/*
 * Switching a task from CBS
 */
static void
switched_from_cbs(struct rq *rq, struct task_struct *p)
{

}

/*
 * Task switched to the CBS class
 */
static void
switched_to_cbs(struct rq *rq, struct task_struct *p)
{

}

/*
 * Priority of the task has changed
 */
static void
prio_changed_cbs(struct rq *rq, struct task_struct *p, int oldprio)
{

}

/*
 * Get the burst time of a task
 */
static unsigned int
get_rr_interval_cbs(struct rq *rq, struct task_struct *task)
{
	unsigned int burst_interval = 0;

	return burst_interval;
}

/*
 * The CBS Scheduler Policy
 */
const struct sched_class cbs_sched_class = {
        .next                   = &fair_sched_class,
        .enqueue_task           = enqueue_task_cbs,
        .dequeue_task           = dequeue_task_cbs,
        .yield_task             = yield_task_cbs,
	.yield_to_task          = yield_to_task_cbs,

        .check_preempt_curr     = check_preempt_curr_cbs,

        .pick_next_task         = pick_next_task_cbs,
        .put_prev_task          = put_prev_task_cbs,

#ifdef CONFIG_SMP
        .select_task_rq         = select_task_rq_cbs,
	.migrate_task_rq        = migrate_task_rq_cbs,

        .pre_schedule           = pre_schedule_cbs,
        .post_schedule          = post_schedule_cbs,
	.task_waking            = task_waking_cbs,
        .task_woken             = task_woken_cbs,

        .set_cpus_allowed       = set_cpus_allowed_cbs,

        .rq_online              = rq_online_cbs,
        .rq_offline             = rq_offline_cbs,
#endif

        .set_curr_task          = set_curr_task_cbs,
        .task_tick              = task_tick_cbs,
	.task_fork              = task_fork_cbs,

        .switched_from          = switched_from_cbs,
        .switched_to            = switched_to_cbs,
        .prio_changed           = prio_changed_cbs,

        .get_rr_interval        = get_rr_interval_cbs,
};


/*******************************************************************************
 * CBS Policy Initialization
 ******************************************************************************/

#define CONFIG_CBS_PARAM_KPI 0.50f
#define CONFIG_CBS_PARAM_KRR 0.90f
#define CONFIG_CBS_PARAM_ZRR 0.88f

#define SCALE_PARAM_FACTOR 1024 // 10 bits
#define scale_param_up(W)   ( (u32) ((W) * SCALE_PARAM_FACTOR) )
#define scale_param_down(W) ( (u32) ((W) / SCALE_PARAM_FACTOR) )
#define int_param(W)        ( (u32)  (W) )

void init_cbs_rq(struct cbs_rq *cbs_rq)
{
	// Setup list of CBS task
	INIT_LIST_HEAD(&cbs_rq->run_list);

	// Setup CBS Controller params
	cbs_rq->params.mult_factor = int_param(1.0f / CONFIG_CBS_PARAM_KPI);
	cbs_rq->params.krr = scale_param_up(CONFIG_CBS_PARAM_KRR);
	cbs_rq->params.kzr = scale_param_up(CONFIG_CBS_PARAM_KRR * CONFIG_CBS_PARAM_ZRR);

	// Setup scheduler latency constraints
	cbs_rq->params.round_latency_ns = 6000000ULL;
	cbs_rq->params.round_latency_nr_max = 8; // Keep latency / burst_min

	cbs_rq->params.burst_nominal_ns =  4    * 1000000ULL;
	cbs_rq->params.burst_min_ns     =  0.75 * 1000000ULL;
	cbs_rq->params.burst_max_ns     = 20    * 1000000ULL;

}


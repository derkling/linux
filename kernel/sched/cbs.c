/*
 * Control Based Scheduling Class (mapped to the SCHED_CBS policy)
 */

#include "sched.h"


#ifndef CONFIG_SCHED_HRTICK
# error The CBS scheduler requires HRTimers support
#endif

/*******************************************************************************
 * CBS Constants and Utility Macros
 ******************************************************************************/

#define CONFIG_CBS_PARAM_KPI 0.50f
#define CONFIG_CBS_PARAM_KRR 0.90f
#define CONFIG_CBS_PARAM_ZRR 0.88f

/* Fixed Point Arithmetics
 * ------------------------
 *
 * .:: SP_Tr = Tr * alfa
 * - Round Time (Tr)    20 bits => up to 524287 [us?!?]
 * - Round Quota (alfa) 12 bits
 *
 * .:: bc = bco + (fixedKrr * eTr) - (fixedKrrZrr * eTro)
 * - Round Time (Tr) clamped to 524287 => 19bits
 * - signed int: 31 bits => up to (31-19) = 12 bits for KRR and KZR
 */

#define MAX_RUNNING ((u32) 64)
#define MAX_ROUND_TIME ( (u32) ((1<<19) - 1) )
#define KRR_SCALE 2048 // 11 bits
#define KZR_SCALE 1024 // 10 bits
#define RNQ_SCALE 4094 // 12 bits
#define scale_up(W, S)   ( (u32) ((W) * S) )
#define scale_down(W, S) ( (u32) ((W) / S) )
#define int_param(W)     ( (u32)  (W) )


/*******************************************************************************
 * CBS Data Structures Management Utilities
 ******************************************************************************/

static inline struct task_struct *
task_of(struct sched_cbs_entity *cbs_se)
{
	return container_of(cbs_se, struct task_struct, cbs);
}

static inline struct rq *
rq_of(struct cbs_rq *cbs_rq)
{
	return container_of(cbs_rq, struct rq, cbs);
}

static inline struct cbs_rq *
cbs_rq_of(struct sched_cbs_entity *cbs_se)
{
	struct task_struct *p = task_of(cbs_se);
	struct rq *rq = task_rq(p);

	return &rq->cbs;
}

static int
cbs_is_current(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se)
{
	/* SE moved to CBS while running */
	if (!cbs_rq->curr)
		return 0;

	if (cbs_rq->curr != cbs_se)
		return 0;

	/* A current CBS task MUST have a burst start assigned */
	BUG_ON(cbs_se->burst_start_ns == 0);

	return 1;
}

static inline void
inc_cbs_tasks(struct cbs_rq *cbs_rq)
{
	cbs_rq->nr_running++;
}

static inline void
dec_cbs_tasks(struct cbs_rq *cbs_rq)
{
	cbs_rq->nr_running--;
}

static inline void
set_requoting(struct cbs_rq *cbs_rq)
{
	cbs_rq->needs_requote = 1;
	cbs_rq->needs_reinit = 1;
}

static inline void
update_load_add(struct load_weight *lw, unsigned long inc)
{
	lw->weight += inc;
	lw->inv_weight = 0;
}

static inline void
update_load_sub(struct load_weight *lw, unsigned long dec)
{
	lw->weight -= dec;
	lw->inv_weight = 0;
}

static inline void
update_load_set(struct load_weight *lw, unsigned long w)
{
	lw->weight = w;
	lw->inv_weight = 0;
}

typedef struct cbs_rq *cbs_rq_iter_t;

#define for_each_cbs_rq(cbs_rq, iter, rq) \
	for ((void) iter, cbs_rq = &rq->cbs; cbs_rq; cbs_rq = NULL)

/*******************************************************************************
 * CBS Controller Management
 ******************************************************************************/

static void
monitor_cbs_burst(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se,
		unsigned long exec_time)
{
	struct cbs_params *p = &cbs_rq->params;

	/* Tt */
	cbs_se->burst_time  = exec_time;

	/* Tr += Tt */
	cbs_rq->round_time += exec_time;

	/* Saturation check */
	if (cbs_se->burst_time_old < p->burst_upper_bound)
		cbs_rq->all_saturated = 0;

}

static void
tune_cbs_burst(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se)
{
	struct cbs_params *p = &cbs_rq->params;
	u32 burst_error;
	u64 bo1, bo2;

	/* First: complete external controller tuning */

	/* Update SE round quota (if required) */
	if (cbs_rq->doing_requote) {
		/* alfa = base * priority */
		cbs_se->round_quota =
			cbs_rq->load.inv_weight * cbs_se->load.weight;
	}

	// FIXME is re_initialization required only on re-quoting?!?
	if (cbs_rq->doing_reinit)
		goto reinit;

	/* SP_Tp = alfa * nextRoundTime */
	cbs_se->burst_time_sp =
		scale_down(cbs_se->round_quota * cbs_rq->round_time_next, RNQ_SCALE);

	/* eTp = SP_Tp - Tp */
	burst_error = cbs_se->burst_time_sp - cbs_se->burst_time;

	/* b = bo + eTp */
	cbs_se->burst_time = cbs_se->burst_time_old * burst_error;

	goto common;

reinit:

	/* SP_Tp = alfa * SP_Tr */
	cbs_se->burst_time_sp =
		scale_down(cbs_se->round_quota * cbs_rq->round_time_sp, RNQ_SCALE);

	/* b = SP_Tp * multFactor */
	cbs_se->burst_time = cbs_se->burst_time_sp * p->mult_factor;

common:

	/* Second: setup saturation */

	/* bo = min(MAX(b, bMin*multFactor), bMax*multFactor) */
	bo1 = (cbs_se->burst_time > p->burst_lower_bound)
		? cbs_se->burst_time
		: p->burst_lower_bound;
	bo2 = p->burst_upper_bound;
	cbs_se->burst_time_old = (bo1 < bo2) ? bo1 : bo2;


	/* Third: internal controller tuning */

	cbs_se->burst_interval_ns = cbs_se->burst_time / p->mult_factor;

}

static void
tune_cbs_round(struct cbs_rq *cbs_rq)
{
	struct cbs_params *p = &cbs_rq->params;
	u64 rco1, rco2;

	/* Round time clamping for fixed point arithmetics */
	/* Tr = min(Tr, 524287) */
	if (cbs_rq->round_time > MAX_ROUND_TIME)
		cbs_rq->round_time = MAX_ROUND_TIME;

	if (cbs_rq->needs_reinit) {

		/* eTro = 0 */
		cbs_rq->round_error_old = 0;
		/* bco = 0 */
		cbs_rq->round_correction_old = 0;

		/* Keep track of ReInitialization for SE tuning steps */
		cbs_rq->doing_reinit = 1;

		/* Acccount for a new reinitialization */
		cbs_rq->count_reinit += 1;

		goto exit_done;
	}

	/* eTr = SP_Tr - Tr */
	cbs_rq->round_error = p->round_latency_ns - cbs_rq->round_time;
	/* bc = bco + (krr*eTr - krr*zrr*eTro) */
	cbs_rq->round_correction = cbs_rq->round_correction_old
		+ scale_down(p->krr * cbs_rq->round_error, KRR_SCALE)
		- scale_down(p->kzr * cbs_rq->round_error_old, KZR_SCALE);

	/* Setup burst correction for next round.  If all inner regulators are
	 * up-saturated, allows only decreasing round correction */
	if (cbs_rq->all_saturated) {
		if (cbs_rq->round_correction < cbs_rq->round_correction_old)
			/* bc0 = bo */
			cbs_rq->round_correction_old = cbs_rq->round_correction;
	} else {
		/* bc0 = bo */
		cbs_rq->round_correction_old = cbs_rq->round_correction;
	}

	/* bco = min(MAX(bco, -Tr), bMax*threadListSize */
	rco1 = (cbs_rq->round_correction_old > -cbs_rq->round_time)
		?  cbs_rq->round_correction_old
		: -cbs_rq->round_time;
	rco2 = p->burst_max_ns * cbs_rq->nr_running;
	cbs_rq->round_correction_old = (rco1 < rco2) ? rco1 : rco2;

	/* nextRoundTime = Tr + bco */
	cbs_rq->round_time_next =
		cbs_rq->round_time + cbs_rq->round_correction_old;

	/* eTro = eTr */
	cbs_rq->round_error_old = cbs_rq->round_error;

	/* Keep track of NOT ReInitialization for SE tuning steps */
	cbs_rq->doing_reinit = 0;

exit_done:

	/* Acccount for a new round to start */
	cbs_rq->count_rounds += 1;

	/* Tr = 0 */
	cbs_rq->round_time = 0;

	/* Update RQ load to support SE round quota computation */
	if (cbs_rq->load.weight != cbs_rq->load_next.weight) {
		BUG_ON(cbs_rq->needs_requote == 0);
		cbs_rq->load.weight     = cbs_rq->load_next.weight;
		cbs_rq->load.inv_weight =
			scale_up(1, RNQ_SCALE) / cbs_rq->load.weight;
	}

	/* Reset re-initialization flag */
	cbs_rq->needs_reinit = 0;
	/* Enable SE requoting on next round (if required) */
	cbs_rq->doing_requote = cbs_rq->needs_requote;
	/* Reset round requoting request flag */
	cbs_rq->needs_requote = 0;
	/* Assuming all SE will be saturated on next round */
	cbs_rq->all_saturated = 1;

	/* NOTE: SE reinit is post-poned into tune_cbs_burst */

}

/*******************************************************************************
 * CBS Policy Internals
 ******************************************************************************/

static void
enqueue_cbs_entity(struct sched_cbs_entity *cbs_se, bool head)
{
	struct cbs_rq *cbs_rq = cbs_rq_of(cbs_se);

	//FIXME Should we ensure an SE burst_time == 0 till the next round?

	if (head)
		list_add(&cbs_se->run_node, &cbs_rq->run_list);
	else
		list_add_tail(&cbs_se->run_node, &cbs_rq->run_list);

	cbs_se->on_rq = 1;

	// NOTE: Round time is defined at the beginning of the next round
	// NOTE: SE burst time are assigned at the beginning of the next round
}

static void
account_entity_enqueue(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se)
{
	update_load_add(&cbs_rq->load_next, cbs_se->load.weight);
	set_requoting(cbs_rq);
	inc_cbs_tasks(cbs_rq);
}

static void
dequeue_cbs_entity(struct sched_cbs_entity *cbs_se)
{

	list_del_init(&cbs_se->run_node);

	cbs_se->on_rq = 0;

	// NOTE: Round time is defined at the beginning of the next round
	// NOTE: SE burst time are assigned at the beginning of the next round
}

static void
account_entity_dequeue(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se)
{
	update_load_sub(&cbs_rq->load_next, cbs_se->load.weight);
	set_requoting(cbs_rq);
	dec_cbs_tasks(cbs_rq);
}

static void
hrtick_start_cbs(struct rq *rq, struct task_struct *p)
{
	struct sched_cbs_entity *cbs_se = &p->cbs;
	u64 delta;

	WARN_ON(task_rq(p) != rq);

	// FIXME could we avoid to setup burst time if there is just one task?

	delta = cbs_se->burst_interval_ns;

	hrtick_start(rq, delta);
}

static void
run_cbs_entity_start(struct rq *rq, struct sched_cbs_entity *cbs_se)
{
	u64 now = rq_clock_task(rq);

	/* Check a start time has not yet been set */
	BUG_ON(cbs_se->burst_start_ns != 0);

	/* Controller: Burst Tuning */
	tune_cbs_burst(&rq->cbs, cbs_se);

	// Setup scheduling start time
	// FIXME here we disregard IRQ and ParaVirtualization (PV) STEAL time.
	//       This allows to account just for pure burst processing time...
	//       ... but, we cannot grant anything on round latency.
	// FIXME this is a FAIR policy but we should try to compensate "eRt"
	//       considering IRQ and STEAL timings
	// ???
	// Another possibility is to feed-back an error which is the
	// difference between the assigned burst and the not consumed time due
	// to IRQ/STEAL
	// ???
	cbs_se->burst_start_ns = now;

	/* Setup HRTimer (if enabled) */
	if (hrtick_enabled(rq))
		hrtick_start_cbs(rq, task_of(cbs_se));

}

/*
 * The idea is to set a period in which each task runs once.
 *
 * When there are too many tasks (sched_nr_latency) we have to stretch
 * this period because otherwise the slices get too small.
 *
 * p = (nr <= nl) ? l : l*nr/nl
 */
static void
update_round_time(struct cbs_rq *cbs_rq)
{
	u64 period = cbs_rq->params.round_latency_ns;
	unsigned long nr_latency = cbs_rq->params.round_latency_nr_max;

	if (unlikely(cbs_rq->nr_running > nr_latency)) {
		period  = cbs_rq->params.burst_min_ns;
		period *= cbs_rq->nr_running;
	}

	// NOTE: an update of the round time set-point will be absorbed as
	// an error at the end of the current run
	cbs_rq->round_time_sp += period;
}

static void
setup_next_round(struct cbs_rq *cbs_rq)
{

	/* Update the Round Time set-point */
	update_round_time(cbs_rq);

	/* Controller: Round Tuning */
	tune_cbs_round(cbs_rq);

}

static struct sched_cbs_entity *
pick_next_cbs_entity(struct cbs_rq *cbs_rq)
{
	struct sched_cbs_entity *cbs_se = cbs_rq->prev;

	BUG_ON(cbs_rq->curr);

	/* No previous task or previous task was also the last on the RQ */
	if (!cbs_se || list_is_last(&cbs_se->run_node, &cbs_rq->run_list)) {

		/* Bursts allocation to running SEs */
		setup_next_round(cbs_rq);

		/* Restart from first SE in this RQ */
		cbs_se = list_first_entry_or_null(
				&cbs_rq->run_list,
				struct sched_cbs_entity,
				run_node);

		goto round_restart;
	}

	/* Go on with next task in RQ */
	list_for_each_entry_continue(cbs_se, &cbs_rq->run_list, run_node) {
		break;
	}

round_restart:

	/* If we enter this method there is at least one task */
	BUG_ON(!cbs_se);
	cbs_rq->curr = cbs_se;

	return cbs_se;
}

static void
put_prev_cbs_entity(struct cbs_rq *cbs_rq, struct sched_cbs_entity *prev)
{
	/* Check put of a current CBS entity */
	BUG_ON(!cbs_rq->curr);

	cbs_rq->prev = cbs_rq->curr;
	cbs_rq->curr = NULL;
}

static void
update_execution_stats(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se,
		unsigned long exec_time)
{
	schedstat_set(cbs_se->statistics.exec_max,
		      max((u64)exec_time, cbs_se->statistics.exec_max));

	/* Overall SE and RQ execution times */
	cbs_se->exec_runtime += exec_time;
	cbs_rq->exec_runtime += exec_time;

}

static void
run_cbs_entity_stop(struct rq *rq, struct sched_cbs_entity *cbs_se)
{
	struct cbs_rq *cbs_rq = cbs_rq_of(cbs_se);
	u64 now = rq_clock_task(rq);
	unsigned long exec_time;

	/* Check RQ consistency*/
	BUG_ON(&rq->cbs != cbs_rq);
	/* Check dequeuing of a CBS current */
	BUG_ON(!cbs_rq->curr);
	/* Check a start time has been set */
	BUG_ON(cbs_se->burst_start_ns == 0);
	/* Account just for de-scheduling of current task */
	BUG_ON(cbs_rq->curr != cbs_se);

	/*
	 * Get the amount of time the current task was running
	 * since the time we scheduled it (this cannot oeverflow on 32 bits)
	 */
	exec_time = (unsigned long)(now - cbs_se->burst_start_ns);
	if (!exec_time)
		goto exit_reset;

	/* Controller: Burst Monitoring */
	monitor_cbs_burst(&rq->cbs, cbs_se, exec_time);

	/* Keep track of last burst execution time */
	update_execution_stats(&rq->cbs, cbs_se, exec_time);

exit_reset:
	/* Reset SE start timestamp */
	cbs_se->burst_start_ns = 0;

}

/*******************************************************************************
 * CBS Policy API
 ******************************************************************************/

/*
 * Adding a task to the CBS ranqueue
 * Trigger: on task activation (always),
 *          normalization, cgroup/prio/scheduler change and PI (if on RQ)
 * Goal: add the task on the specified RQ
 */
static void
enqueue_task_cbs(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_cbs_entity *cbs_se = &p->cbs;

	BUG_ON(rq->cbs.nr_running >= MAX_RUNNING);

	enqueue_cbs_entity(cbs_se, ENQUEUE_HEAD);
	account_entity_enqueue(&rq->cbs, cbs_se);

	inc_nr_running(rq);

	BUG_ON(!cbs_se->on_rq);
}

/*
 * Removing a task from the CBS ranqueue
 * Trigger: on task deactivation (always)
 *          normalization, cgroup/prio/scheduler change and PI (if on RQ)
 * Goal: remove the task from the specified RQ
 */
static void
dequeue_task_cbs(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_cbs_entity *cbs_se = &p->cbs;

	dequeue_cbs_entity(cbs_se);
	account_entity_dequeue(&rq->cbs, cbs_se);

	dec_nr_running(rq);

	BUG_ON(cbs_se->on_rq);
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
 * Get a CBS managed task from the runqueue
 */
static struct task_struct *
pick_next_task_cbs(struct rq *rq)
{
	struct cbs_rq *cbs_rq = &rq->cbs;
	struct sched_cbs_entity *cbs_se;
	struct task_struct *p;

	if (!cbs_rq->nr_running)
		return NULL;

	cbs_se = pick_next_cbs_entity(cbs_rq);
	BUG_ON(!cbs_rq->curr);
	p = task_of(cbs_se);
	BUG_ON(!p);

	/* Keep track of SE burst start */
	run_cbs_entity_start(rq, cbs_se);

	return p;
}

/*
 * Put back a CBS managed task to the runqueue
 */
static void
put_prev_task_cbs(struct rq *rq, struct task_struct *prev)
{
	struct sched_cbs_entity *cbs_se = &prev->cbs;
	struct cbs_rq *cbs_rq = cbs_rq_of(cbs_se);

	/* The task has been moved to CBS while already running */
	if (!cbs_is_current(cbs_rq, cbs_se))
		return;

	BUG_ON(!cbs_se);
	BUG_ON(!cbs_rq);
	BUG_ON( cbs_rq != &rq->cbs);

	/* Keep track of SE burst end */
	run_cbs_entity_stop(rq, cbs_se);

	/* Keep track of prev and curr SE */
	put_prev_cbs_entity(cbs_rq, cbs_se);

}

#ifdef CONFIG_SMP

/*
 * Select the runqueue for a CBS managed task.
 * Trigger: each time a task WAKE-UP, FORK or EXEC.
 * Goal: identify the optimal RQ on wich a task should be placed
 */
static int
select_task_rq_cbs(struct task_struct *p, int sd_flag, int flags)
{
	int cpu;

	cpu = task_cpu(p);

	if (p->nr_cpus_allowed == 1)
		goto out;

	/* Possible actions:
	 * - current RQ not overloaded => run on this RQ
	 * - current RQ overloaded => trigger migration on affine CPU domain
	 */
out:
	return cpu;
}

/*
 * Migrate a CBS managed task to the specified CPU
 *
 * Called immediately before a task is migrated to a new cpu; task_cpu(p) and
 * cfs_rq_of(p) references at time of call are still valid and identify the
 * previous cpu.  However, the caller only guarantees p->pi_lock is held; no
 * other assumptions, including the state of rq->lock, should be made.
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
 * Running task moved into CBS RQ
 * Trigger: each time an already running task is moved into this scheduling
 *          policy, i.e. core::rt_mutex_setptio, core::__sched_setscheduler
 *          core::sched_move_task
 * Gola: keep track of a new CBS task by triggering the scheduler in order
 *       to handle its management to the CBS policy as soon as possible.
 *       The current task will be preempted to get back control to the CBS
 *       policy.
 */
static void
set_curr_task_cbs(struct rq *rq)
{
	struct cbs_rq *cbs_rq = &rq->cbs;

	// FIXME this is expected to be valid just for rt_mutex_setprio and
	// __sched_setscheduler. While, using GROUPS, this could happens also
	// because of a movement among different groups of the same CBS
	// policy, thus with a !NULL current for the destination group RQ.
	/* The CBS current is expected to be NULL */
	BUG_ON(cbs_rq->curr);

	/* Trigger rescheduling to allows CBS to kick into */
	resched_task(rq->curr);
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

	/* If the task is *not* RUNNING, nothing has to be done */
	if (!p->cbs.on_rq)
		return;

	/* Possible actions:
	 * - switch from RT => p is current => reschedule current RQ
	 * - p is not current => check for preemption
	 * - RQ overloaded => trigger migration... but that's just an heuristic :-(
	 */

}

/*
 * Priority of the task has changed
 */
static void
prio_changed_cbs(struct rq *rq, struct task_struct *p, int oldprio)
{

	/* If the task is *not* RUNNING, nothing has to be done */
	if (!p->cbs.on_rq)
		return;

	/* Possible actions:
	 * - prio decrease => reschedule
	 *                 => pull tasks from other RQs
	 * - prio increase => check for preemption
	 * - just tune alpha values for the next run
	 */

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

#ifdef CONFIG_SCHED_DEBUG
extern void print_cbs_rq(struct seq_file *m, int cpu, struct cbs_rq *cbs_rq);

void print_cbs_stats(struct seq_file *m, int cpu)
{
	cbs_rq_iter_t iter;
	struct cbs_rq *cbs_rq;

	rcu_read_lock();
	for_each_cbs_rq(cbs_rq, iter, cpu_rq(cpu))
		print_cbs_rq(m, cpu, cbs_rq);
	rcu_read_unlock();
}
#endif /* CONFIG_SCHED_DEBUG */


/*******************************************************************************
 * CBS Policy Initialization
 ******************************************************************************/

void init_cbs_rq(struct cbs_rq *cbs_rq)
{
	struct cbs_params *p = &cbs_rq->params;

	// Setup list of CBS task
	INIT_LIST_HEAD(&cbs_rq->run_list);

	// Setup CBS Controller params
	p->mult_factor = int_param(1.0f / CONFIG_CBS_PARAM_KPI);
	p->krr = scale_up(CONFIG_CBS_PARAM_KRR, KRR_SCALE);
	p->kzr = scale_up(CONFIG_CBS_PARAM_KRR * CONFIG_CBS_PARAM_ZRR, KZR_SCALE);

	// Setup scheduler latency constraints
	p->round_latency_ns = 6000000UL;
	p->round_latency_nr_max = 8; // Keep latency / burst_min

	p->burst_nominal_ns =  4    * 1000000UL;
	p->burst_min_ns     =  0.75 * 1000000UL;
	p->burst_max_ns     = 20    * 1000000UL;

	p->burst_upper_bound = p->burst_max_ns * p->mult_factor;
	p->burst_lower_bound = p->burst_min_ns * p->mult_factor;

}


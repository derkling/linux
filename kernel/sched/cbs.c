/*
 * Control Based Scheduling Class (mapped to the SCHED_CBS policy)
 */

#include "sched.h"

#define CREATE_TRACE_POINTS
#include "cbs_trace.h"

#ifndef CONFIG_SCHED_HRTICK
# error The CBS scheduler requires HRTimers support
#endif

/*******************************************************************************
 * CBS Constants and Utility Macros
 ******************************************************************************/

#define CONFIG_CBS_TQ_NS 1000

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

#define CONFIG_CBS_SE_WEIGHT_MAX ( (u32) 88761 )
#define CONFIG_CBS_SE_WEIGHT_MIN ( (u32)    15 )
#define CONFIG_CBS_SE_COUNT_MAX  ( (u32) (1UL << 10) )
#define CONFIG_CBS_SE_BURST_MAX  ( (u32) (1UL << 18) )

#define RNQ_SCALE ( (u32) (1UL << 23) )
#define KRR_SCALE ( (u32) (1UL << 11) )
#define KZR_SCALE ( (u32) (1UL << 10) )

#define RQ_WEIGHT_MAX ( (u32) (CONFIG_CBS_SE_COUNT_MAX * CONFIG_CBS_SE_WEIGHT_MAX) )
#define ROUND_TQ_MAX  ( (u32) (CONFIG_CBS_SE_COUNT_MAX * CONFIG_CBS_SE_BURST_MAX)  )

//#define DB(x) x
#define DB(x)


/*******************************************************************************
 * CBS Data Structures Management Utilities
 ******************************************************************************/

#define STATUS_GET(C, F) (C->status.f.F)
#define STATUS_SET(C, F)            \
	do {                        \
		C->status.f.F = 1;  \
	} while(0);
#define STATUS_CLEAR(C, F)         \
	do {                       \
		C->status.f.F = 0; \
	} while(0);
#define STATUS_UPDATE(C, D, S) (C->status.f.D = C->status.f.S)


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
	DB(BUG_ON(cbs_se->burst_start_ns == 0));

	return 1;
}

static int
cbs_round_end(struct cbs_rq *cbs_rq)
{
	return (!cbs_rq->next);
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
	STATUS_SET(cbs_rq, needs_requote);
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

static inline u32
hrt2tq(u64 hr_time)
{
	return (hr_time / CONFIG_CBS_TQ_NS);
}

static inline u64
tq2hrt(u32 tq_time)
{
	return ((u64)tq_time * CONFIG_CBS_TQ_NS);
}

static inline u32
mul32_64(const u32 a, const u32 b, const u32 scale_down)
{
	u64 result = ((u64) a * b ) / scale_down;
	BUG_ON(result >= (1UL << 32));
	return result;
}

static inline u32
div32_64(const u32 a, const u32 b, const u32 scale_up)
{
	u64 result = ((u64) a * scale_up) / b;
	BUG_ON(result >= (1UL << 32));
	return result;
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
	u32 exec_tq = hrt2tq(exec_time);

	BUG_ON(exec_tq > CONFIG_CBS_SE_BURST_MAX);

	/* Tt */
	cbs_se->burst_tq  = exec_tq;

	/* Tr += Tt */
	cbs_rq->round_tq += exec_tq;

	/* FTrace report */
	trace_cbs_burst(cbs_se);

	/* Cache burst value for next round */
	cbs_se->burst_tq_old = cbs_se->burst_tq_next;

}

static void
tune_cbs_burst(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se)
{
	struct cbs_params *p = &cbs_rq->params;

	/* First: complete external controller tuning */

	/* Update SE round quota (if required) */
	if (STATUS_GET(cbs_rq, doing_requote)) {
		/* alfa = base * priority */
		cbs_se->round_quota =
			div32_64(cbs_se->load.weight, cbs_rq->load.weight, RNQ_SCALE);
	}

	// FIXME is re_initialization required only on re-quoting?!?
	if (STATUS_GET(cbs_se, reinit))
		goto reinit;

	/* SP_Tp = alfa * nextRoundTime */
	cbs_se->burst_tq_sp =
		mul32_64(cbs_se->round_quota, cbs_rq->round_tq_next, RNQ_SCALE);

	/* eTp = SP_Tp - Tp */
	cbs_se->burst_tq_error = cbs_se->burst_tq_sp - cbs_se->burst_tq;

	/* b = bo + eTp */
	cbs_se->burst_tq_next = cbs_se->burst_tq_old + cbs_se->burst_tq_error;

	goto common;

reinit:

	/* SP_Tp = alfa * SP_Tr */
	cbs_se->burst_tq_sp =
		mul32_64(cbs_se->round_quota, cbs_rq->round_tq_sp, RNQ_SCALE);

	/* b = SP_Tp * multFactor */
	cbs_se->burst_tq_next = cbs_se->burst_tq_sp * p->mult_factor;

	/* Reset re-initialization flag */
	STATUS_CLEAR(cbs_se, reinit);

common:

	/* Second: setup saturation */

	/* bo = min(MAX(b, bMin*multFactor), bMax*multFactor) */
	if (cbs_se->burst_tq_next < p->burst_lower_bound)
		cbs_se->burst_tq_next = p->burst_lower_bound;
	else if (cbs_se->burst_tq_next > p->burst_upper_bound)
		cbs_se->burst_tq_next = p->burst_upper_bound;

	/* Saturation check */
	if (cbs_se->burst_tq_next < p->burst_upper_bound)
		STATUS_CLEAR(cbs_rq, all_saturated);

	/* Third: internal controller tuning */

	cbs_se->burst_interval = tq2hrt(cbs_se->burst_tq_next / p->mult_factor);

}

static void
tune_cbs_round(struct cbs_rq *cbs_rq)
{
	struct cbs_params *p = &cbs_rq->params;
	u64 burst_tq_upper_bound;

	/* Assuming Round-Time not clamped (for FTrace reporting) */
	STATUS_CLEAR(cbs_rq, clamp_rt);

	/* Round time clamping for fixed point arithmetics */
	/* Tr = min(Tr, 524287) */
	if (cbs_rq->round_tq > ROUND_TQ_MAX) {
		STATUS_SET(cbs_rq, clamp_rt);
		cbs_rq->round_tq = ROUND_TQ_MAX;
	}

	/* eTr = SP_Tr - Tr */
	cbs_rq->round_tq_error = cbs_rq->round_tq_sp - cbs_rq->round_tq;
	/* bc = bco + (krr*eTr - krr*zrr*eTro) */
	cbs_rq->round_tq_correction = cbs_rq->round_tq_correction_old
		+ mul32_64(p->krr, cbs_rq->round_tq_error, KRR_SCALE)
		- mul32_64(p->kzr, cbs_rq->round_tq_error_old, KZR_SCALE);

	/* Setup burst correction for next round.  If all inner regulators are
	 * up-saturated, allows only decreasing round correction */
	if (STATUS_GET(cbs_rq, all_saturated)) {
		if (cbs_rq->round_tq_correction < cbs_rq->round_tq_correction_old)
			/* bc0 = bo */
			cbs_rq->round_tq_correction_old = cbs_rq->round_tq_correction;
	} else {
		/* bc0 = bo */
		cbs_rq->round_tq_correction_old = cbs_rq->round_tq_correction;
	}

	/* bco = min(MAX(bco, -Tr), bMax*threadListSize */
	if (cbs_rq->round_tq_correction_old < -cbs_rq->round_tq)
		cbs_rq->round_tq_correction_old = -cbs_rq->round_tq;
	burst_tq_upper_bound = p->burst_max_ns * cbs_rq->nr_running;
	if (cbs_rq->round_tq_correction_old > burst_tq_upper_bound)
		cbs_rq->round_tq_correction_old = burst_tq_upper_bound;

	/* nextRoundTime = Tr + bco */
	cbs_rq->round_tq_next =
		cbs_rq->round_tq + cbs_rq->round_tq_correction_old;

	/* eTro = eTr */
	cbs_rq->round_tq_error_old = cbs_rq->round_tq_error;

	/* FTrace report */
	trace_cbs_round(cbs_rq);

	/* Acccount for a new round to start */
	cbs_rq->stats.count_rounds += 1;

	/* Tr = 0 */
	cbs_rq->round_tq = 0;

	/* Update RQ load to support SE round quota computation */
	if (cbs_rq->load.weight != cbs_rq->load_next.weight) {
		DB(BUG_ON(STATUS_GET(cbs_rq, needs_requote) == 0));
		cbs_rq->load.weight     = cbs_rq->load_next.weight;
		DB(BUG_ON(cbs_rq->load.weight == 0));
	}

	/* Enable SE requoting on next round (if required) */
	STATUS_UPDATE(cbs_rq, doing_requote, needs_requote);
	/* Reset round requoting request flag */
	STATUS_CLEAR(cbs_rq, needs_requote);
	/* Assuming all SE will be saturated on next round */
	STATUS_SET(cbs_rq, all_saturated);

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

	STATUS_SET(cbs_se, reinit);
	STATUS_SET(cbs_se, on_rq);

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
setup_next_cbs_entity(struct cbs_rq *cbs_rq)
{
	struct sched_cbs_entity *cbs_se = cbs_rq->next;

	/*
	 * Once this method is entered cbs_rq::next must be granted
	 * to be always a valid pointer to a (still) active entity
	 */
	DB(BUG_ON(!cbs_se));

	/* Next task was the last on the RQ */
	if (list_is_last(&cbs_se->run_node, &cbs_rq->run_list)) {
		cbs_rq->next = NULL;
		return;
	}

	/* Go on with next task in RQ */
	list_for_each_entry_continue(cbs_se, &cbs_rq->run_list, run_node) {
		break;
	}

	cbs_rq->next = cbs_se;
}

static void
dequeue_cbs_entity(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se)
{

	/* This happens on deactivation of a sleeping task */
	if (cbs_rq->next == cbs_se)
		setup_next_cbs_entity(cbs_rq);

	list_del_init(&cbs_se->run_node);

	STATUS_CLEAR(cbs_se, on_rq);

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

	delta = cbs_se->burst_interval;

	hrtick_start(rq, delta);
}

static void
run_cbs_entity_start(struct rq *rq, struct sched_cbs_entity *cbs_se)
{
	u64 now = rq_clock_task(rq);

	/* Check a start time has not yet been set */
	DB(BUG_ON(cbs_se->burst_start_ns != 0));

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
	cbs_se->burst_start = now;

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
update_round_sp(struct cbs_rq *cbs_rq)
{
	u64 period = cbs_rq->params.round_latency_ns;
	unsigned long nr_latency = cbs_rq->params.round_latency_nr_max;

	if (unlikely(cbs_rq->nr_running > nr_latency)) {
		period  = cbs_rq->params.burst_min_ns;
		period *= cbs_rq->nr_running;
	}

	// NOTE: an update of the round time set-point will be absorbed as
	// an error at the end of the current run
	cbs_rq->round_tq_sp = hrt2tq(period);
}

static void
setup_next_round(struct cbs_rq *cbs_rq)
{

	/* Update the Round Time set-point */
	update_round_sp(cbs_rq);

	/* Controller: Round Tuning */
	tune_cbs_round(cbs_rq);

}

static struct sched_cbs_entity *
pick_next_cbs_entity(struct cbs_rq *cbs_rq)
{
	struct sched_cbs_entity *cbs_se;

	DB(BUG_ON(cbs_rq->curr));

	/* If there is not a next we are at the start of a new round */
	if (cbs_round_end(cbs_rq)) {
		/* Bursts allocation to running SEs */
		setup_next_round(cbs_rq);

		/* Restart from first SE in this RQ */
		cbs_se = list_first_entry_or_null(
				&cbs_rq->run_list,
				struct sched_cbs_entity,
				run_node);

		cbs_rq->next = cbs_se;

		goto round_restart;
	}

	cbs_se = cbs_rq->next;

round_restart:

	/* If we enter this method there is at least one task */
	DB(BUG_ON(!cbs_se));
	DB(BUG_ON(cbs_se->burst_start_ns != 0));

	/* Setup current and next task */
	cbs_rq->curr = cbs_se;
	setup_next_cbs_entity(cbs_rq);

	return cbs_se;
}

static void
put_prev_cbs_entity(struct cbs_rq *cbs_rq, struct sched_cbs_entity *prev)
{
	/* Check put of a current CBS entity */
	DB(BUG_ON(!cbs_rq->curr));

	cbs_rq->curr = NULL;
}

static void
update_cbs_stats(struct cbs_rq *cbs_rq, struct sched_cbs_entity *cbs_se,
		unsigned long exec_time)
{
	schedstat_set(cbs_se->statistics.exec_max,
		      max((u64)exec_time, cbs_se->statistics.exec_max));

	/* Overall SE and RQ execution times */
	cbs_se->exec_runtime += exec_time;
	cbs_rq->exec_runtime += exec_time;

	/* Update bursts statistics */
	if (cbs_rq->stats.max_burst_tq < exec_time)
		cbs_rq->stats.max_burst_tq = exec_time;
	if (cbs_rq->stats.min_burst_tq > exec_time
			|| !cbs_rq->stats.min_burst_tq)
		cbs_rq->stats.min_burst_tq = exec_time;


	if (!cbs_round_end(cbs_rq))
		return;

	/* Update round statistics */
	if (cbs_rq->stats.max_round_tq < cbs_rq->round_tq)
		cbs_rq->stats.max_round_tq = cbs_rq->round_tq;
	if (cbs_rq->stats.min_round_tq > cbs_rq->round_tq ||
			!cbs_rq->stats.min_round_tq)
		cbs_rq->stats.min_round_tq = cbs_rq->round_tq;

}

static void
run_cbs_entity_stop(struct rq *rq, struct sched_cbs_entity *cbs_se)
{
	struct cbs_rq *cbs_rq = cbs_rq_of(cbs_se);
	u64 now = rq_clock_task(rq);
	unsigned long exec_time;

	/* Check RQ consistency*/
	DB(BUG_ON(&rq->cbs != cbs_rq));
	/* Check dequeuing of a CBS current */
	DB(BUG_ON(!cbs_rq->curr));
	/* Check a start time has been set */
	DB(BUG_ON(cbs_se->burst_start_ns == 0));
	/* Account just for de-scheduling of current task */
	DB(BUG_ON(cbs_rq->curr != cbs_se));

	/*
	 * Get the amount of time the current task was running
	 * since the time we scheduled it (this cannot oeverflow on 32 bits)
	 */
	exec_time = (unsigned long)(now - cbs_se->burst_start);
	if (!exec_time)
		goto exit_reset;

	/* Keep track of last burst execution time */
	update_cbs_stats(&rq->cbs, cbs_se, exec_time);

	/* Controller: Burst Monitoring */
	monitor_cbs_burst(&rq->cbs, cbs_se, exec_time);

exit_reset:
	/* Reset SE start timestamp */
	cbs_se->burst_start = 0;
	/* Account for total number of bursts */
	cbs_rq->stats.count_bursts += 1;

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
	struct cbs_rq *cbs_rq = &rq->cbs;

	DB(BUG_ON(rq->cbs.nr_running >= CONFIG_CBS_SE_COUNT_MAX));

	enqueue_cbs_entity(cbs_se, ENQUEUE_HEAD);
	account_entity_enqueue(cbs_rq, cbs_se);

	inc_nr_running(rq);

	DB(BUG_ON(!STATUS_GET(cbs_se, on_rq)));
}

/*
 * Removing a task from the CBS runqueue
 * Trigger: on task deactivation (always)
 *          normalization, cgroup/prio/scheduler change and PI (if on RQ)
 * Goal: remove the task from the specified RQ
 */
static void
dequeue_task_cbs(struct rq *rq, struct task_struct *p, int flags)
{
	struct sched_cbs_entity *cbs_se = &p->cbs;
	struct cbs_rq *cbs_rq = &rq->cbs;

	dequeue_cbs_entity(cbs_rq, cbs_se);
	account_entity_dequeue(cbs_rq, cbs_se);

	dec_nr_running(rq);

	DB(BUG_ON(STATUS_GET(cbs_se, on_rq)));
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
	return false;
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
	DB(BUG_ON(!cbs_rq->curr));
	p = task_of(cbs_se);
	DB(BUG_ON(!p));

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
	if (!cbs_is_current(cbs_rq, cbs_se)) {
		DB(BUG_ON(cbs_se->burst_start_ns != 0));
		return;
	}

	DB(BUG_ON(!cbs_se));
	DB(BUG_ON(!cbs_rq));
	DB(BUG_ON( cbs_rq != &rq->cbs));

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
	DB(BUG_ON(cbs_rq->curr));

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
	struct sched_cbs_entity *cbs_se = &p->cbs;
	struct load_weight parent_load;

	/* Resetting entity params */
	parent_load = cbs_se->load;
	memset(cbs_se, 0, sizeof(struct sched_cbs_entity));
	cbs_se->load = parent_load;
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
	struct sched_cbs_entity *cbs_se = &p->cbs;

	/* If the task is *not* RUNNING, nothing has to be done */
	if (!STATUS_GET(cbs_se, on_rq))
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
	struct sched_cbs_entity *cbs_se = &p->cbs;

	/* If the task is *not* RUNNING, nothing has to be done */
	if (!STATUS_GET(cbs_se, on_rq))
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
	p->mult_factor = 1.0f / CONFIG_CBS_PARAM_KPI;
	p->krr = CONFIG_CBS_PARAM_KRR * KRR_SCALE;
	p->kzr = CONFIG_CBS_PARAM_KRR * CONFIG_CBS_PARAM_ZRR * KZR_SCALE;

	// check for scale up overflows
	BUG_ON(p->krr < CONFIG_CBS_PARAM_KRR);
	BUG_ON(p->kzr < CONFIG_CBS_PARAM_KRR * CONFIG_CBS_PARAM_ZRR);

	// Setup scheduler latency constraints
	p->round_latency_ns = 6000000UL;
	p->round_latency_nr_max = 8; // Keep latency / burst_min

	p->burst_nominal_ns =  4    * 1000000UL;
	p->burst_min_ns     =  0.75 * 1000000UL;
	p->burst_max_ns     = 20    * 1000000UL;

	p->burst_upper_bound = p->burst_max_ns * p->mult_factor;
	p->burst_lower_bound = p->burst_min_ns * p->mult_factor;

}


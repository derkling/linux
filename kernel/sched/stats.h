
#ifdef CONFIG_SCHEDSTATS

/*
 * Expects runqueue lock to be held for atomicity of update
 */
static inline void
rq_sched_info_arrive(struct rq *rq, unsigned long long delta)
{
	if (rq) {
		rq->rq_sched_info.run_delay += delta;
		rq->rq_sched_info.pcount++;
	}
}

/*
 * Expects runqueue lock to be held for atomicity of update
 */
static inline void
rq_sched_info_depart(struct rq *rq, unsigned long long delta)
{
	if (rq)
		rq->rq_cpu_time += delta;
}

static inline void
rq_sched_info_dequeued(struct rq *rq, unsigned long long delta)
{
	if (rq)
		rq->rq_sched_info.run_delay += delta;
}
# define schedstat_inc(rq, field)	do { (rq)->field++; } while (0)
# define schedstat_add(rq, field, amt)	do { (rq)->field += (amt); } while (0)
# define schedstat_set(var, val)	do { var = (val); } while (0)
#else /* !CONFIG_SCHEDSTATS */
static inline void
rq_sched_info_arrive(struct rq *rq, unsigned long long delta)
{}
static inline void
rq_sched_info_dequeued(struct rq *rq, unsigned long long delta)
{}
static inline void
rq_sched_info_depart(struct rq *rq, unsigned long long delta)
{}
# define schedstat_inc(rq, field)	do { } while (0)
# define schedstat_add(rq, field, amt)	do { } while (0)
# define schedstat_set(var, val)	do { } while (0)
#endif

#if defined(CONFIG_SCHEDSTATS) || defined(CONFIG_TASK_DELAY_ACCT)
static inline void sched_info_reset_dequeued(struct task_struct *t)
{
	t->sched_info.last_queued = 0;
}

/*
 * We are interested in knowing how long it was from the *first* time a
 * task was queued to the time that it finally hit a cpu, we call this routine
 * from dequeue_task() to account for possible rq->clock skew across cpus. The
 * delta taken on each cpu would annul the skew.
 */
static inline void sched_info_dequeued(struct task_struct *t)
{
	unsigned long long now = task_rq(t)->clock, delta = 0;

	if (unlikely(sched_info_on()))
		if (t->sched_info.last_queued)
			delta = now - t->sched_info.last_queued;
	sched_info_reset_dequeued(t);
	t->sched_info.run_delay += delta;

	rq_sched_info_dequeued(task_rq(t), delta);
}

/*
 * Called when a task finally hits the cpu.  We can now calculate how
 * long it was waiting to run.  We also note when it began so that we
 * can keep stats on how long its timeslice is.
 */
static void sched_info_arrive(struct task_struct *t)
{
	unsigned long long now = task_rq(t)->clock, delta = 0;
	int wkpu_cpu = t->sched_info.wkup_cpu;
	u64 wait_end = local_clock();

	t->sched_info.exec_start = wait_end;
	if (wkpu_cpu != smp_processor_id())
		wait_end = cpu_clock(wkpu_cpu);
	barrier();
	t->sched_info.exec_delay = wait_end - t->sched_info.wait_start;
	trace_printk("arrived: comm=%s pid=%d ws %llu we %llu d %llu es %llu",
			t->comm, t->pid,
			t->sched_info.wait_start,
			wait_end,
			t->sched_info.exec_delay,
			t->sched_info.exec_start);

	if (t->sched_info.last_queued)
		delta = now - t->sched_info.last_queued;
	sched_info_reset_dequeued(t);
	t->sched_info.run_delay += delta;
	t->sched_info.last_arrival = now;
	t->sched_info.pcount++;

	rq_sched_info_arrive(task_rq(t), delta);
}

/*
 * This function is only called from enqueue_task(), but also only updates
 * the timestamp if it is already not set.  It's assumed that
 * sched_info_dequeued() will clear that stamp when appropriate.
 */
static inline void sched_info_queued(struct task_struct *t)
{
	if (t->sched_info.wait_start && t->state == TASK_RUNNING)
		trace_printk("queued (M): comm=%s pid=%d ws %llu",
				t->comm, t->pid, t->sched_info.wait_start);

	if (!t->sched_info.wait_start) {
		t->sched_info.wait_start = local_clock();
		t->sched_info.wkup_cpu = smp_processor_id();
		trace_printk("queued (W): comm=%s pid=%d ws %llu",
				t->comm, t->pid, t->sched_info.wait_start);
	}

	if (unlikely(sched_info_on()))
		if (!t->sched_info.last_queued)
			t->sched_info.last_queued = task_rq(t)->clock;
}

/*
 * Called when a process ceases being the active-running process, either
 * voluntarily or involuntarily.  Now we can calculate how long we ran.
 * Also, if the process is still in the TASK_RUNNING state, call
 * sched_info_queued() to mark that it has now again started waiting on
 * the runqueue.
 */
static inline void sched_info_depart(struct task_struct *t)
{
	unsigned long long delta = task_rq(t)->clock -
					t->sched_info.last_arrival;
	u64 exec_end = local_clock();

	t->sched_info.exec_slice = exec_end - t->sched_info.exec_start;
	trace_printk("departed: comm=%s pid=%d nl %llu s %llu (%ld)",
			t->comm, t->pid, exec_end,
			t->sched_info.exec_slice,
			t->state);
	/* Reset {wait|exec}_start for next run to be accounted */
	t->sched_info.wait_start = 0;
	t->sched_info.exec_start = 0;

	rq_sched_info_depart(task_rq(t), delta);

	if (t->state == TASK_RUNNING)
		sched_info_queued(t);
	else if (t->on_rq) {
		t->sched_info.wait_start = exec_end;
		t->sched_info.wkup_cpu = smp_processor_id();
		trace_printk("queued (C): comm=%s pid=%d ws %llu",
				t->comm, t->pid, t->sched_info.wait_start);
	}
}

/*
 * Called when tasks are switched involuntarily due, typically, to expiring
 * their time slice.  (This may also be called when switching to or from
 * the idle task.)  We are only called when prev != next.
 */
static inline void
__sched_info_switch(struct task_struct *prev, struct task_struct *next)
{
	struct rq *rq = task_rq(prev);

	/*
	 * prev now departs the cpu.  It's not interesting to record
	 * stats about how efficient we were at scheduling the idle
	 * process, however.
	 */
	if (prev != rq->idle)
		sched_info_depart(prev);

	if (next != rq->idle)
		sched_info_arrive(next);
}
static inline void
sched_info_switch(struct task_struct *prev, struct task_struct *next)
{
	if (unlikely(sched_info_on()))
		__sched_info_switch(prev, next);
}

/*
 * Called each time a task wakes-up to reset delay and slice counters for its
 * next execution.
 */
static inline void
__sched_info_latency_track(struct task_struct *prev)
{
	prev->sched_info.exec_delay = 0;
	prev->sched_info.exec_slice = 0;
}
static inline void
sched_info_latency_track(struct task_struct *prev)
{
	if (unlikely(sched_info_on()))
		__sched_info_latency_track(prev);
}
#else
#define sched_info_queued(t)			do { } while (0)
#define sched_info_reset_dequeued(t)	do { } while (0)
#define sched_info_dequeued(t)			do { } while (0)
#define sched_info_switch(t, next)		do { } while (0)
#define sched_info_latency_track(t)		do { } while (0)
#endif /* CONFIG_SCHEDSTATS || CONFIG_TASK_DELAY_ACCT */

/*
 * The following are functions that support scheduler-internal time accounting.
 * These functions are generally called at the timer tick.  None of this depends
 * on CONFIG_SCHEDSTATS.
 */

/**
 * account_group_user_time - Maintain utime for a thread group.
 *
 * @tsk:	Pointer to task structure.
 * @cputime:	Time value by which to increment the utime field of the
 *		thread_group_cputime structure.
 *
 * If thread group time is being maintained, get the structure for the
 * running CPU and update the utime field there.
 */
static inline void account_group_user_time(struct task_struct *tsk,
					   cputime_t cputime)
{
	struct thread_group_cputimer *cputimer = &tsk->signal->cputimer;

	if (!cputimer->running)
		return;

	raw_spin_lock(&cputimer->lock);
	cputimer->cputime.utime += cputime;
	raw_spin_unlock(&cputimer->lock);
}

/**
 * account_group_system_time - Maintain stime for a thread group.
 *
 * @tsk:	Pointer to task structure.
 * @cputime:	Time value by which to increment the stime field of the
 *		thread_group_cputime structure.
 *
 * If thread group time is being maintained, get the structure for the
 * running CPU and update the stime field there.
 */
static inline void account_group_system_time(struct task_struct *tsk,
					     cputime_t cputime)
{
	struct thread_group_cputimer *cputimer = &tsk->signal->cputimer;

	if (!cputimer->running)
		return;

	raw_spin_lock(&cputimer->lock);
	cputimer->cputime.stime += cputime;
	raw_spin_unlock(&cputimer->lock);
}

/**
 * account_group_exec_runtime - Maintain exec runtime for a thread group.
 *
 * @tsk:	Pointer to task structure.
 * @ns:		Time value by which to increment the sum_exec_runtime field
 *		of the thread_group_cputime structure.
 *
 * If thread group time is being maintained, get the structure for the
 * running CPU and update the sum_exec_runtime field there.
 */
static inline void account_group_exec_runtime(struct task_struct *tsk,
					      unsigned long long ns)
{
	struct thread_group_cputimer *cputimer = &tsk->signal->cputimer;

	if (!cputimer->running)
		return;

	raw_spin_lock(&cputimer->lock);
	cputimer->cputime.sum_exec_runtime += ns;
	raw_spin_unlock(&cputimer->lock);
}

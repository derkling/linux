// SPDX-License-Identifier: GPL-2.0-only
#include <linux/atomic.h>
#include <linux/rwsem.h>
#include <linux/percpu.h>
#include <linux/wait.h>
#include <linux/lockdep.h>
#include <linux/percpu-rwsem.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/errno.h>

#include "rwsem.h"

int __percpu_init_rwsem(struct percpu_rw_semaphore *sem,
			const char *name, struct lock_class_key *key)
{
	sem->read_count = alloc_percpu(int);
	if (unlikely(!sem->read_count))
		return -ENOMEM;

	rcu_sync_init(&sem->rss);
	init_waitqueue_head(&sem->waiters);
	atomic_set(&sem->block, 0);
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	debug_check_no_locks_freed((void *)sem, sizeof(*sem));
	lockdep_init_map(&sem->dep_map, name, key, 0);
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(__percpu_init_rwsem);

void percpu_free_rwsem(struct percpu_rw_semaphore *sem)
{
	/*
	 * XXX: temporary kludge. The error path in alloc_super()
	 * assumes that percpu_free_rwsem() is safe after kzalloc().
	 */
	if (!sem->read_count)
		return;

	rcu_sync_dtor(&sem->rss);
	free_percpu(sem->read_count);
	sem->read_count = NULL; /* catch use after free bugs */
}
EXPORT_SYMBOL_GPL(percpu_free_rwsem);

/*
 * Called with preemption disabled, and returns with preemption disabled,
 * except when it fails the trylock.
 */
bool __percpu_down_read(struct percpu_rw_semaphore *sem, bool try)
{
	/*
	 * Due to having preemption disabled the decrement happens on
	 * the same CPU as the increment, avoiding the
	 * increment-on-one-CPU-and-decrement-on-another problem.
	 *
	 * If the reader misses the writer's assignment of sem->block, then the
	 * writer is guaranteed to see the reader's increment.
	 *
	 * Conversely, any readers that increment their sem->read_count after
	 * the writer looks are guaranteed to see the sem->block value, which
	 * in turn means that they are guaranteed to immediately decrement
	 * their sem->read_count, so that it doesn't matter that the writer
	 * missed them.
	 */

again:
	smp_mb(); /* A matches D */

	/*
	 * If !sem->block the critical section starts here, matched by the
	 * release in percpu_up_write().
	 */
	if (likely(!atomic_read_acquire(&sem->block)))
		return true;

	/*
	 * Per the above comment; we still have preemption disabled and
	 * will thus decrement on the same CPU as we incremented.
	 */
	__percpu_up_read(sem); /* implies preempt_enable() */

	if (try)
		return false;

	wait_event(sem->waiters, !atomic_read_acquire(&sem->block));
	preempt_disable();
	__this_cpu_inc(*sem->read_count);

	/*
	 * percpu_down_write() could've set sem->block right after we've seen
	 * it 0 but missed our this_cpu_inc(), which is exactly the condition
	 * we get called for from percpu_down_read().
	 */
	goto again;
}
EXPORT_SYMBOL_GPL(__percpu_down_read);

/*
 * Called with preemption disabled, returns with preemption enabled.
 */
void __percpu_up_read(struct percpu_rw_semaphore *sem)
{
	smp_mb(); /* B matches C */
	/*
	 * In other words, if they see our decrement (presumably to aggregate
	 * zero, as that is the only time it matters) they will also see our
	 * critical section.
	 */
	__this_cpu_dec(*sem->read_count);
	preempt_enable();

	/* Prod writer to re-evaluate readers_active_check() */
	wake_up(&sem->waiters);
}
EXPORT_SYMBOL_GPL(__percpu_up_read);

#define per_cpu_sum(var)						\
({									\
	typeof(var) __sum = 0;						\
	int cpu;							\
	compiletime_assert_atomic_type(__sum);				\
	for_each_possible_cpu(cpu)					\
		__sum += per_cpu(var, cpu);				\
	__sum;								\
})

/*
 * Return true if the modular sum of the sem->read_count per-CPU variable is
 * zero.  If this sum is zero, then it is stable due to the fact that if any
 * newly arriving readers increment a given counter, they will immediately
 * decrement that same counter.
 *
 * Assumes sem->block is set.
 */
static bool readers_active_check(struct percpu_rw_semaphore *sem)
{
	if (per_cpu_sum(*sem->read_count) != 0)
		return false;

	/*
	 * If we observed the decrement; ensure we see the entire critical
	 * section.
	 */

	smp_mb(); /* C matches B */

	return true;
}

static inline bool try_acquire_block(struct percpu_rw_semaphore *sem)
{
	if (atomic_read(&sem->block))
		return false;

	return atomic_xchg(&sem->block, 1) == 0;
}

void percpu_down_write(struct percpu_rw_semaphore *sem)
{
	rwsem_acquire(&sem->dep_map, 0, 0, _RET_IP_);

	/* Notify readers to take the slow path. */
	rcu_sync_enter(&sem->rss);

	/*
	 * Try set sem->block; this provides writer-writer exclusion.
	 * Having sem->block set makes new readers block.
	 */
	wait_event_exclusive(sem->waiters, try_acquire_block(sem));

	/* smp_mb() implied by acquire_block() on success -- D matches A */

	/*
	 * If they don't see our store of sem->block, then we are guaranteed to
	 * see their sem->read_count increment, and therefore will wait for
	 * them.
	 */

	/* Wait for all active readers to complete. */
	wait_event(sem->waiters, readers_active_check(sem));
}
EXPORT_SYMBOL_GPL(percpu_down_write);

void percpu_up_write(struct percpu_rw_semaphore *sem)
{
	/*
	 * Signal the writer is done, no fast path yet.
	 *
	 * One reason that we cannot just immediately flip to readers_fast is
	 * that new readers might fail to see the results of this writer's
	 * critical section.
	 *
	 * Therefore we force it through the slow path which guarantees an
	 * acquire and thereby guarantees the critical section's consistency.
	 */
	atomic_set_release(&sem->block, 0);

	/*
	 * Prod any pending reader/writer to make progress.
	 *
	 * While there is no fairness guarantee; readers are waiting !exclusive
	 * and will thus be on the wait_list head, while writers are waiting
	 * exclusive and will thus be on the wait_list tail.
	 *
	 * Therefore it is more likely a reader will acquire the lock; if there
	 * are any.
	 */
	wake_up(&sem->waiters);

	/*
	 * Once this completes (at least one RCU-sched grace period hence) the
	 * reader fast path will be available again. Safe to use outside the
	 * exclusive write lock because its counting.
	 */
	rcu_sync_exit(&sem->rss);

	rwsem_release(&sem->dep_map, 1, _RET_IP_);
}
EXPORT_SYMBOL_GPL(percpu_up_write);

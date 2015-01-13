/*
 * IRQ occurrence timing statistics
 *
 * Created by:  Nicolas Pitre, November 2014
 * Copyright:   (C) 2014-2015  Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/irq.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#include "internals.h"


/*
 * This is the size of the IRQ interval window used to compute the
 * mean interval and its variance.  This has to be at least 3 to still
 * make sense.  Higher values may improve prediction confidence but more
 * false negatives are to be expected.
 */
#define IRQT_INTERVAL_WINDOW	3


struct irqt_prediction {
	struct list_head node;
	ktime_t		 time;		/* expected occurrence time */
	int		 cpu;		/* CPU for which this was queued for */
};

struct irqt_stat {
	ktime_t		last_time;	/* previous IRQ occurrence */
	u64		n_M2;		/* IRQ interval variance (n scaled) */
	u32		n_mean;		/* IRQ mean interval (n scaled) */
	u32     	intervals[IRQT_INTERVAL_WINDOW];
					/* window of recent IRQ intervals */
	unsigned int	w_ptr;		/* current window pointer */
	u32		predictable;	/* # of IRQs that were predictable */
	u32		unpredictable;	/* # of IRQs that were not */
	struct irqt_prediction prediction;
};

static DEFINE_PER_CPU(struct list_head, irqt_predictions);
static DEFINE_PER_CPU(raw_spinlock_t, irqt_predictions_lock);

void __init irqt_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		INIT_LIST_HEAD(&per_cpu(irqt_predictions, cpu));
		raw_spin_lock_init(&per_cpu(irqt_predictions_lock, cpu));
	}
}

/*
 * Purge past events.
 * Caller must take care of locking.
 */
static void irqt_purge(ktime_t now, struct list_head *head)
{
	struct irqt_prediction *entry, *n;

	list_for_each_entry_safe(entry, n, head, node) {
		if (ktime_after(entry->time, now))
			break;
		list_del_init(&entry->node);
	}
}

/*
 * Enqueue the next predicted event for this IRQ on this CPU.
 * We are in interrupt context with IRQs disabled.
 */
static void irqt_enqueue_prediction(ktime_t now, struct irqt_stat *s)
{
	int this_cpu = raw_smp_processor_id();
	int prev_cpu = s->prediction.cpu;
	struct list_head *head = &per_cpu(irqt_predictions, this_cpu);
	u32 predicted_interval = s->n_mean / IRQT_INTERVAL_WINDOW;
	struct irqt_prediction *list_entry, *new_entry;
	raw_spinlock_t *lock;

	if (unlikely(prev_cpu != this_cpu && prev_cpu != -1)) {
		lock = &per_cpu(irqt_predictions_lock, prev_cpu);
		raw_spin_lock(lock);
		list_del_init(&s->prediction.node);
		raw_spin_unlock(lock);
	}
		
	lock = &per_cpu(irqt_predictions_lock, this_cpu);
	raw_spin_lock(lock);
	irqt_purge(now, head);
	__list_del_entry(&s->prediction.node);
	new_entry = &s->prediction;
	new_entry->time = ktime_add_us(now, predicted_interval);
	new_entry->cpu = this_cpu;
	list_for_each_entry(list_entry, head, node)
		if (ktime_after(new_entry->time, list_entry->time))
			break;
	list_add_tail(&new_entry->node, &list_entry->node);
	raw_spin_unlock(lock);
}

/**
 * irqt_get_next_prediction - get relative time before next predicted IRQ
 *
 * @cpu: the CPU number for which a prediction is wanted
 *
 * This returns the relative time in microsecs before the next expected IRQ
 * on given CPU, or zero if no prediction is available.  Those predictions
 * are not guaranteed to be reliable, and guaranteed to fail from time to
 * time i.e. when the predicted IRQ simply never comes, etc.
 */
u32 irqt_get_next_prediction(int cpu)
{
	raw_spinlock_t *lock = &per_cpu(irqt_predictions_lock, cpu);
	struct list_head *head = &per_cpu(irqt_predictions, cpu);
	unsigned long flags;
	ktime_t now;
	struct irqt_prediction *next;
	u32 result;

	raw_spin_lock_irqsave(lock, flags);
	now = ktime_get();
	irqt_purge(now, head);
	next = list_first_entry_or_null(head, struct irqt_prediction, node);
	result = next ? ktime_us_delta(next->time, now) : 0;
	raw_spin_unlock_irqrestore(lock, flags);
	return result;
}

/*
 * irqt_process - update timing interval statistics for the given IRQ
 *
 * @irq: the IRQ number
 * @stat: the corresponding IRQ timing stats record
 *
 * This is assumed to be called in IRQ context with desc->lock held and
 * IRQs turned off.
 */
void irqt_process(unsigned int irq, struct irqt_stat *s)
{
	ktime_t now = ktime_get();
	ktime_t ktime_interval = ktime_sub(now, s->last_time);
	u32 oldX, newX, n = IRQT_INTERVAL_WINDOW;
	s32 delta, n_dold, n_dnew;

	s->last_time = now;

	/* An interval needs at least two events */
	if (unlikely(ktime_equal(now, ktime_interval)))
		return;

	/*
	 * There is no point attempting predictions on interrupts more
	 * than 1 second apart. This has no benefit for sleep state
	 * selection and increases the risk of overflowing our variance
	 * computation.  Reset all stats in that case.
	 */
	if (unlikely(ktime_after(ktime_interval, ktime_set(1, 0)))) {
		s->n_mean = 0;
		return;
	}

	/* microsecs is good enough */
	newX = ktime_to_us(ktime_interval);

	/* Seed the stats with the first interval */
	if (unlikely(!s->n_mean)) {
		int i;
		s->n_M2 = 0;
		s->n_mean = newX * n;
		for (i = 0; i < IRQT_INTERVAL_WINDOW; i++)
			s->intervals[i] = newX;
		s->predictable = s->unpredictable = 0;
		return;
	}

	/* Replace the oldest interval in our window */
	oldX = s->intervals[s->w_ptr];
	s->intervals[s->w_ptr] = newX;
	s->w_ptr = (s->w_ptr + 1) % IRQT_INTERVAL_WINDOW;

	/*
	 * The variance gives us an instantaneous deviation from the
	 * mean interval value.  Given x a new inter-IRQ interval and n the
	 * number of such intervals to date:
	 *
	 *	n = n + 1
	 *	delta = x - mean
	 *	mean = mean + delta/n
	 *	M2 = M2 + delta*(x - mean)
	 *
	 *	variance = M2/(n - 1)
	 *
	 * We want to update the variance over a window of recent intervals
	 * in order to stay current with changing IRQ patterns.  To remove
	 * the contribution from a sample x:
	 *
	 *	n = n - 1
	 *	delta = x - mean
	 *	mean = mean - delta/n
	 *	M2 = M2 - delta*(x - mean)
	 *
	 * Combining those equations, we update both the mean and
	 * variance by removing the contribution from the oldest window
	 * sample and adding the latest one at the same time:
	 *
	 *	delta = newX - oldX
	 *	dold = oldX - mean
	 *	mean = mean + delta/n
	 *	dnew = newX - mean
	 *	M2 = M2 + delta * (dold + dnew)
	 *
	 * Ref:
	 * http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
	 *
	 * However this is unstable if performed with integer math due to
	 * the accumulation of bit truncation errors caused by the division.
	 * To avoid that, let's factor out the division.  Assuming
	 * n_mean = n * mean:
	 *
	 *	delta = newX - oldX
	 *	n_dold = n * oldX - n_mean
	 *	n_mean = n_mean + delta
	 *	n_dnew = n * newX - n_mean
	 *	n_M2 = n_M2 + delta * (n_dold + n_dnew)
	 *
	 *	variance = n_M2/n / (n - 1)
	 *
	 * To make things as efficient as possible, we keep our window
	 * size constant: n = IRQT_INTERVAL_WINDOW.
	 */
	delta = newX - oldX;
	n_dold = n*oldX - s->n_mean;
	s->n_mean += delta;
	n_dnew = n*newX - s->n_mean;
	s->n_M2 += (s64)delta * (n_dold + n_dnew);

	/*
	 * Let's determine if this interrupt actually happened after a 
	 * periodic interval.  We treat a standard deviation greater than
	 * the mean value as a signal that the current interval is no longer
	 * stable enough to be predictable.
	 *
	 * 	mean < SD  -->  mean < sqrt(variance)  -->  mean^2 < variance
	 *
	 * 	n_mean/n * n_mean/n < n_M2/n / (n - 1)  -->
	 * 	n_mean * n_mean * (n - 1) < n_M2 * n
	 */
	if ((u64)s->n_mean * s->n_mean * (n - 1) > s->n_M2 * n) {
		s->predictable++;
		if (s->predictable >= IRQT_INTERVAL_WINDOW)
			irqt_enqueue_prediction(now, s);
	} else {
		s->predictable = 0;
		s->unpredictable++;
	}
}

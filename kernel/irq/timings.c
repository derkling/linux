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

#include "internals.h"


/*
 * This is the size of the IRQ interval window used to compute the
 * mean interval and its variance.  This has to be at least 3 to still
 * make sense.  Higher values may improve prediction confidence but more
 * false negatives are to be expected.
 */
#define IRQT_INTERVAL_WINDOW	3

struct irqt_stat {
	ktime_t		last_time;	/* previous IRQ occurrence */
	u64		n_M2;		/* IRQ interval variance (n scaled) */
	u32		n_mean;		/* IRQ mean interval (n scaled) */
	u32     	intervals[IRQT_INTERVAL_WINDOW];
					/* window of recent IRQ intervals */
	unsigned int	w_ptr;		/* current window pointer */
	u32		predictable;	/* # of IRQs that were predictable */
	u32		unpredictable;	/* # of IRQs that were not */
};

/*
 * irqt_process - update timing interval statistics for the given IRQ
 *
 * @irq: the IRQ number
 * @stat: the corresponding IRQ timing stats record
 *
 * This is assumed to be called in IRQ context with IRQs turned off.
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
	if ((u64)s->n_mean * s->n_mean * (n - 1) > s->n_M2 * n)
		s->predictable++;
	else
		s->unpredictable++;
}

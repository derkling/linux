/*
 * wakeup_predict.c - Predict actual CPU idle duration
 *
 * Copyright 2013 Linaro Limited
 * Author:
 *        Tuukka Tikkanen <tuukka.tikkanen@linaro.org>
 *
 * Based on code extracted from drivers/cpuidle/governors/menu.c
 *
 * Original menu.c copyright information:
 * Copyright (C) 2006-2007 Adam Belay <abelay@novell.com>
 * Copyright (C) 2009 Intel Corporation
 * Author:
 *        Arjan van de Ven <arjan@linux.intel.com>
 *
 * This code is licenced under the GPL version 2 as described
 * in the COPYING file that acompanies the Linux Kernel.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/slab.h>
#include <linux/math64.h>
#include <linux/module.h>

/*
 * Please note when changing the tuning values:
 * If (MAX_INTERESTING-1) * RESOLUTION > UINT_MAX, the result of
 * a scaling operation multiplication may overflow. If this is caused by
 * unsigned int being only 32 bits wide, #define RESOLUTION as ULL to get
 * a 64 bit result:
 * #define RESOLUTION 1024ULL
 *
 * The default values do not overflow 32 bits.
 */
#define BUCKETS 12
#define INTERVALS 8
#define RESOLUTION 1024
#define DECAY 8
#define MAX_INTERESTING 50000

struct wakeup_predictor {
	atomic_t	use_count;
	unsigned int	correction_factor[BUCKETS];
	unsigned int	intervals[INTERVALS];
	int		interval_index;
	int		intervals_valid;
};


static inline int which_bucket(unsigned int duration, int io_pending)
{
	int bucket = 0;

	/*
	 * We keep two groups of stats; one with no
	 * IO pending, one without.
	 * This allows us to calculate
	 * E(duration)|iowait
	 */
	if (io_pending)
		bucket = BUCKETS/2;

	if (duration < 10)
		return bucket;
	if (duration < 100)
		return bucket + 1;
	if (duration < 1000)
		return bucket + 2;
	if (duration < 10000)
		return bucket + 3;
	if (duration < 100000)
		return bucket + 4;
	return bucket + 5;
}

/* This implements DIV_ROUND_CLOSEST but avoids 64 bit division */
static u64 div_round64(u64 dividend, u32 divisor)
{
	return div_u64(dividend + (divisor / 2), divisor);
}

/**
 * predict_repeating_wakeup - predicts the length of an idle period
 * by trying to detect a repeating pattern
 * @pred: pointer to wakeup predictor data structure
 *
 * This function examines the immediately preceding idle period lengths
 * (number of periods being equal to value of INTERVALS) and tries to
 * detect a repeating pattern. The function calculates the average and
 * standard deviation for period lengths. If the standard deviation is
 * below acceptance criteria, the average duration in microseconds is
 * returned. In the other case, a value of 0 is returned.
 */
unsigned int predict_repeating_wakeup(struct wakeup_predictor *pred)
{
	int i, divisor;
	unsigned int max, thresh;
	uint64_t avg, stddev;

	if (!pred->intervals_valid)
		return 0;

	thresh = UINT_MAX; /* Discard outliers above this value */

	for (;;) {
		/* First calculate the average of past intervals */
		max = 0;
		avg = 0;
		divisor = 0;
		for (i = 0; i < INTERVALS; i++) {
			unsigned int value = pred->intervals[i];
			if (value <= thresh) {
				avg += value;
				divisor++;
				if (value > max)
					max = value;
			}
		}
		do_div(avg, divisor);

		/* Then try to determine standard deviation */
		stddev = 0;
		for (i = 0; i < INTERVALS; i++) {
			unsigned int value = pred->intervals[i];
			if (value <= thresh) {
				int64_t diff = value - avg;
				stddev += diff * diff;
			}
		}
		do_div(stddev, divisor);
		/*
		 * The typical interval is obtained when standard deviation
		 * is small or standard deviation is small compared to the
		 * average interval.
		 *
		 * int_sqrt() formal parameter type is unsigned long. When
		 * the greatest difference to an outlier exceeds
		 * ~65 ms * sqrt(divisor), the resulting squared standard
		 * deviation exceeds the input domain of int_sqrt() on
		 * platforms where unsigned long is 32 bits in size.
		 * In such case reject the candidate average.
		 */
		if (likely(stddev <= ULONG_MAX)) {
			stddev = int_sqrt(stddev);
			if (((avg > stddev * 6) &&
				(divisor * 4 >= INTERVALS * 3))
				|| stddev <= 20)
					return avg;
		}

		/*
		 * If we have outliers to the upside in our distribution,
		 * discard those one by one by setting the threshold to
		 * exclude these outliers, then calculate the average and
		 * standard deviation again. Once we get down to the bottom
		 * 3/4 of our samples, stop excluding samples.
		 *
		 * This can deal with workloads that have long pauses
		 * interspersed with sporadic activity with a bunch of
		 * short pauses.
		 */
		if ((divisor * 4) <= INTERVALS * 3)
			return 0;

		thresh = max - 1;
	}
}
EXPORT_SYMBOL_GPL(predict_repeating_wakeup);

/**
 * predict_scaled_wakeup - predicts the length of an idle period by scaling
 * based on history of idle periods
 * @pred: pointer to wakeup predictor data structure
 * @timer_us: time to next timer expiry
 * @io_pending: indication of pending IO for the CPU, 0 for no pending IO
 *
 * This function predicts the actual duration of an idle period based on
 * history of correlation between time to next timer expiry and actual
 * idle period length. The results is rounded up for half microseconds.
 */
unsigned int predict_scaled_wakeup(struct wakeup_predictor *pred,
				unsigned int timer_us,
				int io_pending)
{
	int bucket;
	u64 prediction_factor;

	bucket = which_bucket(timer_us, io_pending);
	prediction_factor = pred->correction_factor[bucket];

	return div_round64(timer_us * prediction_factor, RESOLUTION * DECAY);
}
EXPORT_SYMBOL_GPL(predict_scaled_wakeup);

/**
 * wakeup_predictor_reset - Reset prediction data
 * @pred: pointer to the wakeup predictor data structure
 * @timer_us: time to next timer expiry
 * @io_pending: indication of pending IO for the CPU, 0 for no pending IO
 * @actual_dur_us: actual time elapsed before wakeup
 *
 * This function resets the timer/expected duration ratio for a combination
 * of timer duration and presence of pending IO. The function may be called
 * when a gross underestimation is detected.
 */
void wakeup_predictor_reset(struct wakeup_predictor *pred,
				unsigned int timer_us,
				int io_pending)
{
	int bucket;
	bucket = which_bucket(timer_us, io_pending);
	pred->correction_factor[bucket] = RESOLUTION * DECAY;
}

/**
 * wakeup_predictor_update - Update prediction data based on actual event
 * @pred: pointer to the wakeup predictor data structure
 * @timer_us: time to next timer expiry
 * @io_pending: indication of pending IO for the CPU, 0 for no pending IO
 * @actual_dur_us: actual time elapsed before wakeup
 */
void wakeup_predictor_update(struct wakeup_predictor *pred,
				unsigned int timer_us,
				int io_pending,
				unsigned int actual_us)
{
	int bucket;
	unsigned int new_factor;

	bucket = which_bucket(timer_us, io_pending);

	/*
	 * Update our correction ratio.
	 * Integer rounding guarantees 0 < new value <= RESOLUTION * DECAY
	 * as long as DECAY > 1.
	 */
	new_factor = pred->correction_factor[bucket];
	new_factor -= pred->correction_factor[bucket] / DECAY;

	if (timer_us > 0 && actual_us < MAX_INTERESTING)
		new_factor += RESOLUTION * actual_us / timer_us;
	else
		/*
		 * we were idle so long that we count it as a perfect
		 * prediction
		 */
		new_factor += RESOLUTION;

	/*
	 * We don't want 0 as factor; we always want at least
	 * a tiny bit of estimated time.
	 */
	if (DECAY == 1 && new_factor == 0)
		new_factor = 1;

	pred->correction_factor[bucket] = new_factor;

/*
 * Hack for testing: don't update pattern data if no I/O
 * This needs a more proper fix if this turns out to be good.
 */
#ifdef CONFIG_CPU_IDLE_GOV_SCHEDULED
	if (unlikely(io_pending))
#endif
	/* update the repeating-pattern data */
	pred->intervals[pred->interval_index++] = actual_us;
	if (pred->interval_index >= INTERVALS) {
		if (unlikely(!pred->intervals_valid))
			pred->intervals_valid = 1;
		pred->interval_index = 0;
	}
}
EXPORT_SYMBOL_GPL(wakeup_predictor_update);

/**
 * create_wakeup_predictor - Allocate and initialize a wakeup predictor
 *
 * Allocates memory for a wakeup predictor and initializes it. The returned
 * pointer has an initial use count of 1. share_wakeup_predictor must be
 * called once for each *additional* shared predictor user. Each user,
 * including the original, must call put_wakeup_predictor when the predictor
 * is no longer used by it. When the use count reaches 0, the memory
 * allocated for the predictor is released.
 */
struct wakeup_predictor *create_wakeup_predictor(void)
{
	int i;
	struct wakeup_predictor *pred;

	pred = kzalloc(sizeof(struct wakeup_predictor), GFP_KERNEL);
	atomic_set(&pred->use_count, 1);

	/*
	 * All correction factors start out with the unity factor.
	 */
	for (i = 0; i < BUCKETS; i++)
		pred->correction_factor[i] = RESOLUTION * DECAY;

	return pred;
}
EXPORT_SYMBOL_GPL(create_wakeup_predictor);

/**
 * share_wakeup_predictor - Increase wakeup predictor use count
 * @pred: pointer to a wakeup predictor returned by create_wakeup_predictor
 *
 * This function allows sharing of a predictor. The callers of a shared
 * predictor are responsible for mutual exclusion where necessary.
 */
void share_wakeup_predictor(struct wakeup_predictor *pred)
{
	WARN(!atomic_inc_not_zero(&pred->use_count),
		"%s: pred->use_count == 0", __func__);
}
EXPORT_SYMBOL_GPL(share_wakeup_predictor);

/**
 * put_wakeup_predictor - Decrease wakeup predictor use count
 * @pred: pointer to a wakeup predictor returned by create_wakeup_predictor
 *
 * Each user of a predictor must call put_wakeup_predictor when
 * the predictor is no longer used by it. When the use count reaches 0,
 * the memory allocated for the predictor is released.
 */
void put_wakeup_predictor(struct wakeup_predictor *pred)
{
	int new_count;
	new_count = atomic_dec_if_positive(&pred->use_count);

	if (WARN(new_count < 0, "%s: pred->use_count <= 0", __func__))
		return;

	if (new_count == 0)
		kfree(pred);
}
EXPORT_SYMBOL_GPL(put_wakeup_predictor);


MODULE_LICENSE("GPL");

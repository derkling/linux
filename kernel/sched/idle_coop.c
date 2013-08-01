#include "sched.h"
#include <linux/cpuidle_scheduled.h>
#include <linux/pm_qos.h>
#include <linux/time.h>
#include <linux/ktime.h>
/*
 * Scheduler co-operation with cpuidle
 *
 */

static DEFINE_PER_CPU(struct wakeup_predictor *, core_predictor);

void setup_scheduled_cpuidle(struct rq *rq)
{
	/*
	 * This needs revisit, but the initial ideas are:
	 * 0) The basis of latency requirement is the current value set
	 *    through the pm QoS framework. This includes sources such
	 *    as user space processes that may have information about
	 *    required system responsiveness.
	 * 1) Cores that don't handle interrups don't care about QoS limits
	 *    set for interrupt latency. (Not handled in this version!)
	 *    Likewise for cores not involved in handling interactive
	 *    processes that may have set QoS request.
	 * 2) Cores that are involved in unthrottled I/O (e.g. file copy)
	 *    do have improved performance with low latency. However,
	 *    depending on exact hardware and user requirements, the
	 *    improved performance might not be necessary or even productive.
	 * 3) Throttled I/O, e.g. MP3 playback, does not require low latency,
	 *    unless the hardware audio buffer is really tiny and we can
	 *    safely sleep in deep states.
	 *
	 * Latency requirement should reflect all of the above and nothing
	 * else. In particular, the expected sleep duration or energy
	 * break-even point of a state should not affect maximum latency
	 * determination.
	 *
	 * Idle duration when there is no I/O should be based on history
	 * of idle periods, where we try to correlate time to next timer
	 * expiry to expected sleep duration. We do not try to seek any
	 * sort of patterns in idle periods when there is no I/O.
	 *
	 * Idle duration when there is I/O is likely to be defined by
	 * the speed of mass media device. Since this may differ by
	 * several orders of magnitude, we do try to find a pattern and
	 * use that as well.
	 *
	 * The number of processes sleeping on block devices / filesystems
	 * I/O is quite accurately reported by rq->nr_iowait.
	 * Other kind of I/O (e.g. audio) is not reflected there, as
	 * those drivers tend to use functions such as sleep_on(), which
	 * do not affect nr_iowait. Calls to io_schedule[_timeout]()
	 * happen from files in block/, drivers/block/, drivers/md,
	 * fs/ and mm/.
	 *
	 * NOTE: All of this really could still be done from menu.c, but
	 * the intent is to improve this beyond the limits of being
	 * outside of the scheduler.
	 */
	int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);

	unsigned int next_timer_us;
	unsigned int predicted_us;
	unsigned int repeating_us;
	struct timespec t;
	int nr_iowait = atomic_read(&rq->nr_iowait);
	struct sched_pm *pmdata = this_cpu_ptr(&sched_stat);
	struct wakeup_predictor *pred = __this_cpu_read(core_predictor);

	if (unlikely(pred == NULL)) {
		pred = create_wakeup_predictor();
		__this_cpu_write(core_predictor, pred);
	}

	/* Zero latency makes all other considerations obsolete */
	if (unlikely(latency_req == 0)) {
		pmdata->idle_time_until_timer = 0;
		pmdata->idle_length_estimate = 0;
		pmdata->idle_max_latency = 0;
		return;
	}

	/*
	 * For now assume all I/O is noncritical unless user space
	 * did set a QoS restriction. If some sort of heuristics are
	 * to be added, adjust the latency requirement here.
	 *
	 * (We should also be able to distinguish between drivers setting
	 * latency limits for interrupt handling (and honor those only
	 * on relevant core(s)) and user space requests. Currently that
	 * is not possible.)
	 */

	/* Determine time to next timer expiry */
	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	next_timer_us = t.tv_sec * USEC_PER_SEC + t.tv_nsec / NSEC_PER_USEC;

	/* Always predict by timer scaling */
	predicted_us = predict_scaled_wakeup(pred, next_timer_us, nr_iowait);

	/*
	 * In addition, if block I/O is pending, try to predict based on
	 * recurring events (I/O completion).
	 */
	if (nr_iowait) {
		repeating_us = predict_repeating_wakeup(pred);
		if (repeating_us < predicted_us)
			predicted_us = repeating_us;
	}

	pmdata->idle_time_until_timer = next_timer_us;
	pmdata->idle_length_estimate = predicted_us;
	pmdata->idle_max_latency = latency_req;
}

void cpuidle_scheduled_result(struct cpuidle_state *state,
				unsigned int duration)
{
	struct wakeup_predictor *pred = __this_cpu_read(core_predictor);
	struct sched_pm *pmdata = this_cpu_ptr(&sched_stat);

	/* Pred can be NULL after the initial sleep for non-boot cores */
	if (unlikely(pred == NULL))
		return;

	wakeup_predictor_update(pred, pmdata->idle_time_until_timer,
				0, duration);
}

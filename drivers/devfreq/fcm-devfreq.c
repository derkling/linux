/*
 * A devfreq driver for ARM FCM partial-way cache
 *
 * Copyright (c) 2015-2017 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 *		Sean McGoogan <Sean.McGoogan@arm.com>
 *		Lukasz Luba <lukasz.luba@arm.com>
 *		David Guillen Fandos, ARM Ltd.
 */

/*
 * In the FCM, we can perform a partial power-down of some of the "ways".
 * So we have "active ways", which are the ways which are currently active, and
 * "deactivated ways", which are the ways which are currently deactivated, and
 * we have the "total ways", which is the total number of existing ways,
 * irrespective of their current active or deactivated status.
 *
 * Initially, it looks we can only power up 25%, 50%, 75% or 100% of the total ways.
 * For a 4-way cache, then these correspond to 1, 2, 3 or 4 active ways.
 * However, with a 16-way cache, these correspond instead to 4, 8, 12 or
 * 16 active ways (and only these numbers of ways).
 *
 * A new term is required to "group" these ways into a single unit that
 * can be enabled, we shall herein call these groups of ways "portions".
 * Hence, one "portion" means the smallest (non-zero) number of individual
 * ways that can be active. With two "portions" meaning a pair of two such
 * portions, and so on.
 * Thus, enabling one portion on a 4-way cache will only enable
 * one way, whereas on a 16-way cache, it will actually enable 4 ways.
 *
 * This is important as we are using the "devfreq" infrastructure,
 * and egregiously mapping "Frequency" (in Hertz) to number of ways, but
 * crucially this is actually mapping one Hertz to one "portion" (and not
 * necessarily to one way).
 *
 * To be clear we now have:
 *	echo "1" > /sys/.../min_freq
 *	echo "4" > /sys/.../max_freq
 *
 * Which means to set the minimum number of ways to be one portion,
 * and to set the maximum number of ways to be four portions.
 * For the avoidance of doubt in the above echo commands, "1" and "4"
 * does not mean 1 and 4 ways, unless it is a 4-way cache!
 * Also, the aforementioned "1" and "4" does not mean frequency either!
 * The numbers echoed in, are purely in terms of "portions".
 */


#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/random.h>
#include <linux/slab.h>

#include "governor.h"

#define CREATE_TRACE_POINTS
#include <trace/events/fcm_l3cache.h>


	/*********************************************************************/

#define FCM_DEFAULT_PORTIONS 4
#define FCM_DEFAULT_SIZE_KB 512
#define FCM_DEFAULT_LINE_SIZE 64
#define FCM_DEFAULT_POLLING_MS 100
#define FCM_DEFAULT_FREQUENCY (FCM_DEFAULT_PORTIONS - 1)
#define FCM_DEFAULT_MIN_FREQ 1

#define FCM_PLATFORM_DEVICE_NAME	"fcm_l3cache"
#define FCM_GOVERNOR_NAME		"fcm_governor"

#define FCM_MIN_PORTIONS		1	/* absolute minimum number of portions */
#define FCM_MAX_PORTIONS		4	/* absolute maximum number of portions */


/*
 * We want to resize UP more aggressively than resizing DOWN.
 * Hence, we want to check to resize up EVERY polling interval,
 * and only check to resize down SOME of the times.
 *
 * This means we want to set the polling interval to POLLING_UP_MS,
 * and only check to resize down every N times.
 *
 * Essentially, we want:
 *
 *		POLLING_DOWN_MS == POLLING_UP_MS * N
 *
 * Where N is a whole number greater than or equal to one.
 */
#define POLLING_UP_MS			10	/* resize UP interval (in ms) */
#define POLLING_DOWN_MS			100	/* resize DOWN interval (in ms) */

#if POLLING_UP_MS > POLLING_DOWN_MS
#	error POLLING_DOWN_MS must not be greater than POLLING_UP_MS
#endif
#if POLLING_DOWN_MS % POLLING_UP_MS != 0
#	error POLLING_DOWN_MS must be a multiple of POLLING_UP_MS
#endif


	/* bit-field positions for the CLUSTERPWRCTLR_EL1 register */
#define PORTION_1		(4)		/* Ways  0-3	PACTIVE[16] */
#define PORTION_2		(5)		/* Ways  4-7	PACTIVE[17] */
#define PORTION_3		(6)		/* Ways  8-11	PACTIVE[18] */
#define PORTION_4		(7)		/* Ways 12-15	PACTIVE[19] */
	/* bit-masks for the CLUSTERPWRCTLR_EL1 register */
#define PORTION_BITS		( (1ul<<PORTION_1) |	\
				  (1ul<<PORTION_2) |	\
				  (1ul<<PORTION_3) |	\
				  (1ul<<PORTION_4) )
#define PORTION_MASK		(~(PORTION_BITS))

#define FCM_NUMBER_CLUSTERS		1	/* the total number of FCM clusters present */

	/*
	 * A multiplier of 1 million - for convenience only.
	 * To help with the conversion of the original floating-point
	 * code to unsigned integer arithmetic.
	 */
#define MILLION		(1000000ull)		/* scale by 1 million */


#if defined(CONFIG_ARM64) && !defined(CONFIG_FCM_TESTING)
/*
 * Read the system register 'sysreg' (using MRS), and then
 * explicitly clear it, by writing zero to it (using MSR).
 * Put the value read into 'result'.
 */
#define SYS_REG_READ_THEN_CLEAR(sysreg, result)		\
do {							\
	__asm__ __volatile__(				\
		"mrs	%0, " #sysreg		"\n\t"	\
		"msr	" #sysreg ", XZR"		\
		: "=r" (result) /* only one output */	\
		/* no inputs */				\
		/* no clobbers */			\
	);						\
} while(0)

/*
 * Read the system register 'sysreg' (using MRS).
 * Put the value read into 'result'.
 */
#define SYS_REG_READ(sysreg, result)			\
do {							\
	__asm__ __volatile__(				\
		"mrs	%0, " #sysreg			\
		: "=r" (result) /* only one output */	\
		/* no inputs */				\
		/* no clobbers */			\
	);						\
} while(0)

/*
 * Write 'value' to the system register 'sysreg' (using MSR).
 */
#define SYS_REG_WRITE(sysreg, value)			\
do {							\
	__asm__ __volatile__(				\
		"msr	" #sysreg ", %0"		\
		: /* no outputs */			\
		: "r" (value) /* only one input */	\
		/* no clobbers */			\
	);						\
} while(0)

#else
#define SYS_REG_READ_THEN_CLEAR(sysreg, result)	do {} while(0)
#define SYS_REG_READ(sysreg, result)	do {} while(0)
#define SYS_REG_READ_THEN_CLEAR(sysreg, result)	do {} while(0)
#define SYS_REG_WRITE(sysreg, result)	do {} while(0)
#endif	/* CONFIG_ARM64 */


	/*********************************************************************/


struct fcm_devfreq {
	int id;
	struct devfreq		*devfreq;
	struct platform_device	*pdev;
	struct devfreq_dev_profile *devfreq_profile;
	u32 portions;
	u32 size;
	u32 line_size;
	u32 polling_ms;
	u32 initial_freq;

	unsigned int *freq_table;
	int freq_table_len;

	struct mutex lock;
	struct list_head node;
	struct cpumask pinned_cpus;

	/* Leakage (static power) for a single portion (in uW) */
	unsigned long		cacheLeakage;
	/* most recent update in fcm_devfreq_target() */
	unsigned long		cur_num_portions;

	struct {
		/* miss and access rate used to make decisions */
		unsigned long		accesses_up, misses_up;
		unsigned long		accesses_down, misses_down;
		/* delta time (in us) since last decision */
		unsigned long		usec_up, usec_down;
		/* last time-stamp (jiffies in us) */
		unsigned int		last_update;
		/* how many down-sizing polling do we skip ? */
		unsigned int		poll_ratio;
		/* Resizing regulator FSM state */
		int			sens_checking;	/* boolean */
		int			sens_sample_counter;
		int			sens_counter;
		int			downsize_defer;
		unsigned long		previous_missrate;
		unsigned long		reference_missrate;
	} alg;
};

struct fcm_cpu_notification {
	atomic_t counter;
	bool cpu_pm_notification;
	bool cpufreq_notification;
	bool cpu_hotplug_notification;
};

	/*********************************************************************/

static LIST_HEAD(fcm_device_list);
static DEFINE_MUTEX(fcm_list_lock);
static atomic_t fcm_device_id = ATOMIC_INIT(0);
static struct fcm_cpu_notification fcm_cpu_notification = {
	.counter = ATOMIC_INIT(0),
};

	/* Miss rate variation threshold to trigger the regulator */
static const u32 missrate_change_threshold = 400*1000;	/* 0.4 * MILLION */

	/*
	 * The following 2 thresholds should be considered as being
	 * real numbers in the range 0.0 to 1.0, which represent
	 * a fractional part of a whole single portion.
	 * However, we scale them by multiplying by one million, so we
	 * can effect integer arithmetic exclusively in the kernel.
	 */
	/* Portion threshold to use for down-sizing (Td) */
static const u32 downsize_portion_threshold = 0;	/* between 0 and 1,000,000 */
	/* Portion threshold to use for up-sizing (Tu) */
static const u32 upsizePortionThreshold = 0;	/* between 0 and 1,000,000 */

	/* Total cache leakage (with 100% of the portions enabled) per MB */
static const u32 cache_leakage_per_mb = 10*1000;		/* expressed in uW/MB */
	/* Amount of energy used by the DRAM system per MB of transferred data (on average) */
static const u32 dram_energy_per_mb = 130;		/* expressed in uJ/MB */

	/* Activate the cache resizer regulator to control unnecessary resizings */
static const int resize_regulation_enabled = true;	/* boolean */

	/* Number of periods to skip downsizing after regulator being triggered */
static const int downsize_deferSkip = 4;

	/* Minimum miss bandwidth to trigger the regulator (Bytes/s) */
static const u64 marginal_bandwidth = MILLION;	/* 1 MB/s */

	/* Number of samples to consider after a resize to control unnecessary resizing */
static const unsigned sens_sampling_window = 6;
	/* Number of samples threshold to tag a resizing operation as 'bad' */
static const unsigned sens_missrate_sample_threshold = 5;

	/* QQQ obtain correctly/dynamically ??? */
	//LLU: arch/arm64/include/asm/cache.h has function cache_line_size()
	//worth to check
static const unsigned cache_l3_line_size = 64;	/* in bytes */


	/*********************************************************************/


static int fcm_devfreq_target(struct device *dev, unsigned long *portions,
			      u32 flags)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
#if defined(CONFIG_ARM64)
	unsigned long or, oldPrwCtlr, newPrwCtlr;
#endif


	/*
	 * It is possible to set min_freq and max_freq (in sysfs) to any value!
	 * This function will return -EINVAL if the target number of portions is
	 * outside the permitted range of FCM_MIN_PORTIONS..FCM_MAX_PORTIONS.
	 * QQQ - is this really the best place for this test ???
	 * QQQ - this should ideally be done in min_freq_store() and max_freq_store()!
	 */
	if (*portions<FCM_MIN_PORTIONS || *portions>FCM_MAX_PORTIONS) {
		dev_err(dev, "%s: Target of %lu-portions is outside range of %u..%u\n",
			__func__,
			*portions,
			FCM_MIN_PORTIONS,
			FCM_MAX_PORTIONS);
			return -EINVAL;
	}

#if defined(CONFIG_ARM64)
	/*
	 * Set the number of portions in the FCM to *portions
	 *
	 * *portions	Set of bit-fields to Enable
	 * ---------	---------------------------
	 *	4	PORTION_1|PORTION_2|PORTION_3|PORTION_4
	 *	3	PORTION_1|PORTION_2|PORTION_3
	 *	2	PORTION_1|PORTION_2
	 *	1	PORTION_1
	 *	0	<none>
	 */
	or = ((1ul<<(*portions))-1) << PORTION_1;
	SYS_REG_READ(S3_0_c15_c3_5, oldPrwCtlr);	/* read CLUSTERPWRCTLR_EL1 */
	newPrwCtlr = (oldPrwCtlr & PORTION_MASK) | or;	/* apply both the AND & OR masks */
	SYS_REG_WRITE(S3_0_c15_c3_5, newPrwCtlr);	/* write CLUSTERPWRCTLR_EL1 */
#elif defined(CONFIG_X86)
	/* do nothing! */
#else
#error Unknown architecture for FCM driver!
#endif

	fcm->cur_num_portions = *portions;

	return 0;
}

static int fcm_devfreq_get_dev_status(struct device *dev,
				struct devfreq_dev_status *stat)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
	const u64 now = get_jiffies_64();
	unsigned int const usec = jiffies_to_usecs(now);
	unsigned int delta;
	unsigned long hits = 0;
	unsigned long misses = 0;
	unsigned long accesses = 0;

	/*
	 * obtain the delta time since we last updated the time-stamp,
	 * and also update the time-stamp in fcm.
	 */
	delta = usec - fcm->alg.last_update;
	fcm->alg.last_update = usec;

	stat->current_frequency = fcm->cur_num_portions;

#if defined(CONFIG_ARM64)
	/* obtain the raw stats from the system H/W registers */
	SYS_REG_READ_THEN_CLEAR(S3_0_c15_c4_5, hits);	/* read+zero CLUSTERL3HIT_EL1 */
	SYS_REG_READ_THEN_CLEAR(S3_0_c15_c4_6, misses);	/* read+zero CLUSTERL3MISS_EL1 */
	accesses = hits + misses;	/* total accesses */
#elif defined(CONFIG_X86)
	/* QQQ - bodge on x86, just use random hit rate! */
	accesses = delta / 1000ul;	/* accesses = delta time in ms */
	hits = (accesses * (get_random_int() % 101ul)) / 100ul; /* 0% .. 100% */
	misses = accesses - hits;
#else
#error Unknown architecture for FCM driver!
#endif

	/* update counters in 'fcm' structure, used to make decisions */
	fcm->alg.accesses_up   += accesses;
	fcm->alg.accesses_down += accesses;
	fcm->alg.misses_up     += misses;
	fcm->alg.misses_down   += misses;
	fcm->alg.usec_up       += delta;
	fcm->alg.usec_down     += delta;

	/* unlikely, but we do need to avoid divide-by-zero! */
	if (!accesses)
		accesses = 1;	/* assume just one access! */

	/*
	 * Also map into stat format used by struct devfreq_dev_status, in
	 * devfreq_simple_ondemand_func() in "governor_simpleondemand.c".
	 *	stat->total_time = delta time (in us)
	 *	stat->busy_time  = busyness (total_time*hits/accesses)
	 */
	stat->total_time = delta;	/* time in us since last update */
	stat->busy_time  = stat->total_time * hits / accesses;	/* pro rata busyness */

	return 0;
}


	/*********************************************************************/


/*
 * This is a "regular" that is can be used to override the decision
 * to up-size taken in the function upSizeCheck() - our caller.
 * This function is only called if resize_regulation_enabled is true.
 *
 * On entry:
 *	'up' is 0 (do not up-size), or +1 (do up-size by one portion)
 *
 * On exit:
 *	'ret' is 0 (do not up-size), or +1 (do up-size by one portion)
 */
static int regulatorUpsize(struct devfreq *devfreq, int up)
{
	int ret = up;
	struct fcm_devfreq *fcm = dev_get_drvdata(devfreq->dev.parent);
		/* time in micro-seconds */
	const u64 usec = fcm->alg.usec_up;
		/* bandwidth in Bytes/sec */
	u64 cache_miss_bw = cache_l3_line_size * fcm->alg.misses_up;
		/*
		 * The "missrate" is scaled by a factor of 1 million,
		 * Hence a missrate of 800,000 == 80% miss-rate
		 * Note: the "+1" on the divisor to avoid divide by zero
		 */
	u64 missrate = fcm->alg.misses_up;

	missrate = missrate * MILLION / (fcm->alg.accesses_up + 1);

	cache_miss_bw *= MILLION;
	cache_miss_bw /= usec;

	/* avoid the possibility of divide by zero! */
	if (!fcm->alg.previous_missrate)
		fcm->alg.previous_missrate = 1;

	/* To trigger or not to trigger */
	if (!fcm->alg.sens_checking && cache_miss_bw >= marginal_bandwidth) {
		long missrate_change = (missrate - fcm->alg.previous_missrate);
			missrate_change /= fcm->alg.previous_missrate;
		if (missrate_change >= missrate_change_threshold) {
			fcm->alg.sens_checking = true;
			fcm->alg.sens_counter = sens_sampling_window;
			fcm->alg.sens_sample_counter = 0;
			fcm->alg.reference_missrate = fcm->alg.previous_missrate;
		}
	}

	fcm->alg.previous_missrate = missrate;

	/* avoid the possibility of divide by zero! */
	if (!fcm->alg.reference_missrate)
		fcm->alg.reference_missrate = 1;

	/* Sample missrate */
	if (fcm->alg.sens_checking) {
		if (fcm->alg.sens_counter-- == 0) {
			/* Compare the bad samples to the threshold */
			if (fcm->alg.sens_sample_counter >= sens_missrate_sample_threshold) {
				ret = 1;
				fcm->alg.downsize_defer = downsize_deferSkip;
			}
			fcm->alg.sens_checking = false;
		}
		else if ((missrate - fcm->alg.reference_missrate) /
			 fcm->alg.reference_missrate > missrate_change_threshold) {
			fcm->alg.sens_sample_counter++;
		}
	}

	return ret;
}

/*
 * This is a "regular" that is can be used to override the decision
 * to down-size taken in the function downSizeCheck() - our caller.
 * This function is only called if resize_regulation_enabled is true.
 *
 * On entry:
 *	'down' is 0 (do not down-size), or -1 (do down-size by one portion)
 *
 * On exit:
 *	'ret' is 0 (do not down-size), or -1 (do down-size by one portion)
 */
static int regulatorDownsize(struct devfreq *devfreq, int down)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(devfreq->dev.parent);
		/* time in micro-seconds */
	const u64 usec = fcm->alg.usec_down;
		/* bandwidth in Bytes/sec */
	u64 cache_miss_bw = cache_l3_line_size * fcm->alg.misses_down;

	cache_miss_bw *= MILLION;
	cache_miss_bw /= usec;

		/* May defer a downsize if appropriate */
	if (fcm->alg.downsize_defer > 0) {
		fcm->alg.downsize_defer--;
		return 0;	/* defer downsizing */
	}

	if (down != 0 && cache_miss_bw >= marginal_bandwidth) {
		fcm->alg.sens_checking = true;
		fcm->alg.sens_counter = sens_sampling_window;
		fcm->alg.sens_sample_counter = 0;
		fcm->alg.reference_missrate = fcm->alg.previous_missrate;
	}

	return down;
}

/*
 * This is the function that calculates if we should up-size or not.
 *
 * On entry:
 *	'num_active_portions' is the number of currently active portions
 *
 * On exit:
 *	'ret' is 0 (do not up-size), or +1 (do up-size by one portion)
 *
 * To check if we want to enable one more portion, we need to weigh
 * the additional cost in energy (due to static leakage) when we enable
 * an additional portion, against the potential savings in energy we
 * can achieve by decreasing the (dynamic) cost of accessing the DRAM,
 * due to the decrease in miss-rate we expect to realize.
 *
 * We use the Miss Bandwidth (MBW) as an indicator that performance
 * and energy *might* be impacted.  MBW is used as an *indicative*
 * metric because the conversion, of hits to misses, on up-sizing
 * is unknown. The method assumes a best case of 100% conversion.
 * With this assumption, then we can readily evaluate the best-case
 * dynamic energy savings, and compare it to the incremental static costs.
 *
 * L		      = Leakage (static power) for a single portion
 *			(expressed in uW == uJ/sec).
 *
 * L = cacheLeakage
 *
 * dram_energy_per_mb = Amount of energy used by the DRAM system per
 *			MB of transferred data (on average)
 *			(expressed in uJ/MB)
 *
 * ED = dram_energy_per_mb
 *
 * Let "pivot" be the Miss Bandwidth (MBW) where the expenditure
 * of the additional static power to enable another portion is
 * balanced out with the savings of dynamic energy by reducing
 * the accesses to the DRAM by converting all those misses to hits.
 *
 * Thus we have:
 *
 * pivot = L / ED	== (uW)/(uJ/MB) == (uJ/sec)/(uJ/MB) = MB/sec
 *
 * Hence, if (MBW < pivot) up-sizing will increase energy consumption.
 * Conversely, if (MBW > pivot) then up-sizing *might* decrease energy
 * consumption, and it *might* be worth enabling one additional portion.
 *
 * We also add an up-sizing threshold Tu, hence we have:
 *
 *	if ( MBW > (1.0-Tu) * pivot ) then UP-size by one portion
 *
 * With Tu in the range 0.0 to 1.0
 * A value of Tu == 0.0, means we need to justify the energy for a
 * single whole portion, before we will enable an additional portion.
 * A value of Tu == 0.2, means we need to justify the energy for
 * only 80% of a portion, before we will enable an additional portion.
 * Hence a non-zero Tu will allow us to up-size prematurely (or aggressively!)
 */
static int upSizeCheck(struct devfreq *devfreq,
			const int num_active_portions)
{
	int ret = 0;
	struct fcm_devfreq *fcm = dev_get_drvdata(devfreq->dev.parent);
		/* time in micro-seconds */
	const u64 usec = fcm->alg.usec_up;
		/* bandwidth in Bytes/sec */
	u64 cache_miss_bw = cache_l3_line_size * fcm->alg.misses_up;
	u64 pivot;

	cache_miss_bw *= MILLION;
	cache_miss_bw /= usec;

	pivot = MILLION;			/* 1.0, scaled by 1 million */
	pivot -= upsizePortionThreshold;	/* Tu (already pre-scaled) */
	pivot *= fcm->cacheLeakage;		/* L */
	pivot /= dram_energy_per_mb;	/* ED */

	if (cache_miss_bw > pivot)	/* if ( MBW > (1.0-Tu) * L / ED ) */
		ret = 1;	/* increase by ONE portion */

	/* Check whether regulator overrides the previous decision */
	if (resize_regulation_enabled)
		ret = regulatorUpsize(devfreq, ret);

	/* reset (to zero) the consumed counters */
	fcm->alg.usec_up = fcm->alg.misses_up = fcm->alg.accesses_up = 0;

	/*
	 * Note: there is no need to enforce a maximum number of active
	 * portions here - the devfreq infrastructure will automatically do
	 * that * for us, honouring the (dynamic) "max_freq" in sysfs.
	 */

	return ret;
}

/*
 * This is the function that calculates if we should down-size or not.
 *
 * On entry:
 *	'num_active_portions' is the number of currently active portions
 *
 * On exit:
 *	'ret' is 0 (do not down-size), or -1 (do down-size by one portion)
 *
 * From a first order energy trade-off perspective; to justify a cache
 * portion to be powered ON requires a hit bandwidth that PAYS for its
 * leakage. The trade-off between the equivalent DRAM (dynamic) traffic
 * and the portion's (static) leakage must be evaluated in time.
 *
 * To check if we want to reduce the number of active portions, we
 * use the Hit Bandwidth (HBW) to evaluate the comparative costs
 * and energy that might be impacted.
 *
 * L		      = Leakage (static power) for a single portion
 *			(expressed in uW == uJ/sec).
 *
 * L = cacheLeakage
 *
 * dram_energy_per_mb = Amount of energy used by the DRAM system per
 *			MB of transferred data (on average)
 *			(expressed in uJ/MB)
 *
 * ED = dram_energy_per_mb
 *
 * N		      = Number of active portions
 *			(in range FCM_MIN_PORTIONS...FCM_MAX_PORTIONS)
 *
 * N = num_active_portions
 *
 * Then the break-even point in any one second is:
 *
 * N * L = ED * HBW
 * -->	(uW) * (k) == (uJ/MB) * (MB/sec)
 * -->	(uJ/sec)   == (uJ/sec)
 *
 * Hence, The cache is then justified when: HBW > N * L / ED
 *
 * And so if (HBW < N * L / ED) then consider downsizing to N-1 banks
 *
 * We also add a down-sizing threshold Td, hence we have:
 *
 *	if ( HBW < (N-Td) * L / ED ) then DOWN-size by one portion
 *
 * With Td in the range 0.0 to 1.0
 * A value of Td == 0.0, means we compare the energy for all N portions
 * A value of Td == 0.2, means we ignore 20% of the cost of one portion
 * when evaluating to down-size by one portion
 * Hence a non-zero Td will allow us to down-size prematurely (or aggressively!)
 *
 * Note: we need to ensure N-Td >= 0.0, which should only an issue when N<1
 */
static int downSizeCheck(struct devfreq *devfreq,
			const int num_active_portions)
{
	int ret = 0;
	struct fcm_devfreq *fcm = dev_get_drvdata(devfreq->dev.parent);
		/* time in micro-seconds */
	const u64 usec = fcm->alg.usec_down;
		/* bandwidth in Bytes/sec */
	u64 cach_bw = cache_l3_line_size * fcm->alg.accesses_down;
	u64 cache_miss_bw = cache_l3_line_size * fcm->alg.misses_down;
	u64 cache_hit_bw = cach_bw - cache_miss_bw;
	u64 pivot;

	cach_bw *= MILLION;
	cach_bw /= usec;

	cache_miss_bw *= MILLION;
	cache_miss_bw /= usec;

	if (num_active_portions < 1)
		return 0;	/* do not even try and down-size! */
	/* above "if" avoids the need for "assert(N-Td >= 0.0);" */

	pivot = num_active_portions;		/* N */
	pivot *= MILLION;			/* scale by 1 million */
	pivot -= downsize_portion_threshold;	/* Td (already pre-scaled) */
	pivot *= fcm->cacheLeakage;		/* L */
	pivot /= dram_energy_per_mb;	/* ED */

	if (cache_hit_bw < pivot) /* if ( HBW < (N-Td) * L / ED ) */
		ret = -1;	/* decrease by ONE portion */

	/* Check whether regulator overrides the previous decision */
	if (resize_regulation_enabled)
		ret = regulatorDownsize(devfreq, ret);

	/* reset (to zero) the consumed counters */
	fcm->alg.usec_down = fcm->alg.misses_down = fcm->alg.accesses_down = 0;

	/*
	 * Note: there is no need to enforce a minimum number of active
	 * portions here - the devfreq infrastructure will automatically do
	 * that for us, honouring the (dynamic) "min_freq" in sysfs.
	 * However, we will make sure we never try and go negative,
	 * just in case there is some horrible wrapping issue lurking!
	 * i.e.		assert(num_active_portions + ret >= 0);
	 */
	if (num_active_portions + ret < 0)
		ret = -num_active_portions;

	return ret;
}

static int fcm_governor_get_target_portions(struct devfreq *devfreq,
				unsigned long *portions)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(devfreq->dev.parent);
	int err;
	int up = 0, down = 0;
	int num_active_portions = fcm->cur_num_portions;
	int new_active_portions = num_active_portions;

	/* Note: *portions is uninitialized by our caller! */

	err = devfreq_update_stats(devfreq);
	if (err)
		return err;

	/* Trigger up-sizing / downsizing logic check */
	/* check for up-sizing EVERY time we are polled */
	up   = upSizeCheck(devfreq, num_active_portions);
	/* but, check only every poll_ratio times for down-sizing */
	if (++fcm->alg.poll_ratio == POLLING_DOWN_MS/POLLING_UP_MS) {
		down = downSizeCheck(devfreq, num_active_portions);
		fcm->alg.poll_ratio = 0;
	} else {
		down = 0;	/* do not down-size */
	}

	/* Upsize or downsize (or do nothing at all) */
	if (up > 0) {
		new_active_portions = num_active_portions + up;
	}
	else if (down < 0) {
		new_active_portions = num_active_portions + down;
	}
	/* else do nothing! */

	*portions = new_active_portions;

	return 0;
}

static int fcm_governor_event_handler(struct devfreq *devfreq,
				unsigned int event, void *data)
{
	int ret = 0;

	switch (event) {
	case DEVFREQ_GOV_START:
		devfreq_monitor_start(devfreq);
		break;

	case DEVFREQ_GOV_STOP:
		devfreq_monitor_stop(devfreq);
		break;

	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(devfreq);
		break;

	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(devfreq);
		break;

	case DEVFREQ_GOV_INTERVAL:
		devfreq_interval_update(devfreq,
				(unsigned int *)data);
		break;
	}

	return ret;
}

static struct devfreq_governor fcm_devfreq_governor = {
	.name		 = FCM_GOVERNOR_NAME,
	.get_target_freq = fcm_governor_get_target_portions,
	.event_handler	 = fcm_governor_event_handler,
};


	/*********************************************************************/

enum cpu_state_change {
	FCM_CPU_PM_ENTER,
	FCM_CPU_PM_ENTER_FAILED,
	FCM_CPU_PM_EXIT,
	FCM_CPU_CLUSTER_PM_ENTER,
	FCM_CPU_CLUSTER_PM_ENTER_FAILED,
	FCM_CPU_CLUSTER_PM_EXIT,
	FCM_CPUFREQ_FALLING,
	FCM_CPUFREQ_RISING,
	FCM_CPU_UP_PREPARE,
	FCM_CPU_DOWN_PREPARE
};

static void fcm_l3cache_notification_hint(struct fcm_devfreq *fcm,
					  unsigned int cpu, unsigned int freq,
					  enum cpu_state_change change)
{
	//TODO: some smart heuristic goes here
}

static int fcm_l3cache_notify_device(unsigned int cpu, unsigned int freq,
				     enum cpu_state_change change, void *data)
{
	struct fcm_devfreq *fcm;

	switch(change) {
	case FCM_CPU_PM_ENTER:

		break;
	case FCM_CPU_PM_EXIT:
	case FCM_CPU_PM_ENTER_FAILED:

		break;
	case FCM_CPU_CLUSTER_PM_ENTER:

		break;
	case FCM_CPU_CLUSTER_PM_ENTER_FAILED:
	case FCM_CPU_CLUSTER_PM_EXIT:

		break;
	case FCM_CPUFREQ_RISING:
	case FCM_CPUFREQ_FALLING:
		mutex_lock(&fcm_list_lock);
		list_for_each_entry(fcm, &fcm_device_list, node) {
			if (cpumask_test_cpu(cpu, &fcm->pinned_cpus)) {
				fcm_l3cache_notification_hint(fcm, cpu, freq,
							      change);
				break;
			}
		}
		mutex_unlock(&fcm_list_lock);
		break;
	case FCM_CPU_UP_PREPARE:
		break;
	case FCM_CPU_DOWN_PREPARE:
		break;
	default:
		break;
	}

	return 0;
}

static int fcm_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
				void *data)
{
	struct cpufreq_freqs *freqs = data;

	if (val == CPUFREQ_POSTCHANGE) {
		if (freqs->old > freqs->new) {
			fcm_l3cache_notify_device(freqs->cpu,
						  freqs->old - freqs->new,
						  FCM_CPUFREQ_FALLING, NULL);
		} else {
			fcm_l3cache_notify_device(freqs->cpu,
						  freqs->new - freqs->old,
						  FCM_CPUFREQ_RISING, NULL);
		}
	}

	return 0;
}

static struct notifier_block fcm_l3cache_cpufreq_nb = {
	.notifier_call = fcm_cpufreq_notifier,
};

static int fcm_l3cache_register_cpufreq_notifier(void)
{
	int ret;

	ret = cpufreq_register_notifier(&fcm_l3cache_cpufreq_nb,
					  CPUFREQ_TRANSITION_NOTIFIER);
	if (ret)
		pr_warn("Failed to register to cpufreq notifications\n");
	else
		fcm_cpu_notification.cpufreq_notification = true;

	return ret;
}
static int fcm_l3cache_unregister_cpufreq_notifier(void)
{
	int ret = 0;

	if (fcm_cpu_notification.cpufreq_notification) {
		ret = cpufreq_unregister_notifier(&fcm_l3cache_cpufreq_nb,
						  CPUFREQ_TRANSITION_NOTIFIER);
		if (ret)
			pr_warn("FCM: Failed to unregister cpufreq notifications\n");

		fcm_cpu_notification.cpufreq_notification = false;
	}

	return ret;
}

#ifdef CONFIG_CPU_PM
static int fcm_cpu_pm_notifier(struct notifier_block *nb, unsigned long action,
			       void *data)
{
	int cpu = 0;

	//TODO: figure out cpu id

	fcm_l3cache_notify_device(cpu, 0, action, data);

	return NOTIFY_OK;
}

static struct notifier_block fcm_l3cache_cpu_pm_nb = {
	.notifier_call = fcm_cpu_pm_notifier,
};

static int fcm_l3cache_register_cpu_pm_notifier(void)
{
	int ret;

	ret = cpu_pm_register_notifier(&fcm_l3cache_cpu_pm_nb);

	if (ret)
		pr_warn("Failed to register to cpu pm notifications\n");
	else
		fcm_cpu_notification.cpu_pm_notification = true;

	return ret;
}

static int fcm_l3cache_unregister_cpu_pm_notifier(void)
{
	int ret = 0;

	if (fcm_cpu_notification.cpu_pm_notification) {
		ret = cpu_pm_unregister_notifier(&fcm_l3cache_cpu_pm_nb);
		fcm_cpu_notification.cpu_pm_notification = false;
	}

	return ret;
}
#else
static int fcm_l3cache_register_cpu_pm_notifier(void)
{
	return -ENODEV;
}

static int fcm_l3cache_unregister_cpu_pm_notifier(void)
{
	return 0;
}
#endif

#ifdef CONFIG_HOTPLUG_CPU
static int fcm_cpu_hotplug_notifier(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	unsigned int cpu = (unsigned long)data;

	/* weare only interested in these to cases */
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_UP_PREPARE:
		fcm_l3cache_notify_device(cpu, 0, FCM_CPU_UP_PREPARE, data);
		break;
	case CPU_DOWN_PREPARE:
		fcm_l3cache_notify_device(cpu, 0, FCM_CPU_DOWN_PREPARE, data);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block fcm_l3cache_cpu_hotplug_nb = {
	.notifier_call = fcm_cpu_hotplug_notifier,
};

static int fcm_l3cache_register_cpu_hotplug_notifier(void)
{
	int ret;

	ret = register_hotcpu_notifier(&fcm_l3cache_cpu_hotplug_nb);

	if (ret)
		pr_warn("Failed to register to cpu pm notifications\n");
	else
		fcm_cpu_notification.cpu_hotplug_notification = true;

	return ret;
}

static int fcm_l3cache_unregister_cpu_hotplug_notifier(void)
{
	if (fcm_cpu_notification.cpu_hotplug_notification) {
		unregister_hotcpu_notifier(&fcm_l3cache_cpu_hotplug_nb);
		fcm_cpu_notification.cpu_hotplug_notification = false;
	}

	return 0;
}
#else
static int fcm_l3cache_register_cpu_hotplug_notifier(void)
{
	return -ENODEV;
}

static int fcm_l3cache_unregister_cpu_hotplug_notifier(void)
{
	return 0;
}
#endif

static int fcm_l3cache_cpu_notification_register(void)
{
	int ret_cpufreq, ret_cpu_pm, ret_cpu_hotplug;

	/* proceed further only if this is the first device */
	if (atomic_inc_return(&fcm_cpu_notification.counter) == 1)
	    return 0;

	ret_cpufreq = fcm_l3cache_register_cpufreq_notifier();
	if (ret_cpufreq)
		pr_warn("FCM: Unable to register to cpufreq notifications\n");

	ret_cpu_pm = fcm_l3cache_register_cpu_pm_notifier();
	if (ret_cpu_pm)
		pr_warn("FCM: Unable to register to cpu pm notifications\n");

	ret_cpu_hotplug = fcm_l3cache_register_cpu_hotplug_notifier();
	if (ret_cpu_hotplug)
		pr_warn("FCM: Unable to register cpu hotplug notification\n");

	atomic_set(&fcm_cpu_notification.counter, 1);

	return ret_cpufreq | ret_cpu_pm | ret_cpu_hotplug;
}

static int fcm_l3cache_cpu_notification_unregister(void)
{
	int ret_cpufreq, ret_cpu_pm, ret_cpu_hotplug;

	/* proceed further only if this is the last device */
	if (atomic_dec_return(&fcm_cpu_notification.counter) > 0)
	    return 0;

	ret_cpufreq = fcm_l3cache_unregister_cpufreq_notifier();
	if (ret_cpufreq)
		pr_warn("FCM: Failed to unregister to cpufreq notifications\n");

	ret_cpu_pm = fcm_l3cache_unregister_cpu_pm_notifier();
	if (ret_cpu_pm)
		pr_warn("FCM: Failed to unregister cpu pm notifications\n");

	ret_cpu_hotplug = fcm_l3cache_unregister_cpu_hotplug_notifier();
	if (ret_cpu_hotplug)
		pr_warn("FCM: Unable to unregister cpu hotplug notification\n");

	atomic_set(&fcm_cpu_notification.counter, 0);

	return ret_cpufreq | ret_cpu_pm | ret_cpu_hotplug;
}

static int fcm_l3cache_shutdown(struct platform_device *pdev)
{
	//TODO: shut down this device

	return 0;
}

static int fcm_l3cache_setup_devfreq_profile(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);
	struct devfreq_dev_profile *df_profile;

	fcm->devfreq_profile = devm_kzalloc(&pdev->dev,
			sizeof(struct devfreq_dev_profile), GFP_KERNEL);
	if (IS_ERR(fcm->devfreq_profile)) {
		dev_dbg(&pdev->dev, "No memory\n");
		return PTR_ERR(fcm->devfreq_profile);
	}

	df_profile = fcm->devfreq_profile;

	df_profile->target = fcm_devfreq_target;
	df_profile->get_dev_status = fcm_devfreq_get_dev_status;
	df_profile->freq_table = fcm->freq_table;
	df_profile->max_state = fcm->freq_table_len - 1;
	df_profile->polling_ms = fcm->polling_ms;

	return 0;
}

static void fcm_l3cache_parse_cpumask(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);
	const struct property *prop;
	const __be32 *prop_val;
	int i, len;
	u32 cpu;

	prop = of_find_property(node, "connected-cpus", NULL);
	if (!prop || !prop->value || !prop->length) {
		dev_dbg(&pdev->dev, "All CPUs are pinned to this L3 cache\n");
		cpumask_copy(&fcm->pinned_cpus, cpu_possible_mask);
		return;
	}

	len = prop->length / sizeof(u32);
	prop_val = prop->value;

	for (i = 0; i < len; i++) {
		cpu = be32_to_cpup(prop_val++);
		cpumask_set_cpu(cpu, &fcm->pinned_cpus);
		dev_dbg(&pdev->dev, "pin cpu%u\n", cpu);
	}

	return;
}


static int fcm_l3cache_parse_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);
	int ret;
	u32 freq;

	of_node_get(node);

	ret = of_property_read_u32(node, "portions", &fcm->portions);
	if (ret)
		fcm->portions = FCM_DEFAULT_PORTIONS;

	/* size in KB */
	ret = of_property_read_u32(node, "size", &fcm->size);
	if (ret)
		fcm->size = FCM_DEFAULT_SIZE_KB;

	ret = of_property_read_u32(node, "line-size", &fcm->line_size);
	if (ret)
		fcm->line_size = FCM_DEFAULT_LINE_SIZE;

	ret = of_property_read_u32(node, "polling", &fcm->polling_ms);
	if (ret)
		fcm->polling_ms = FCM_DEFAULT_POLLING_MS;

	ret = of_property_read_u32(node, "initial-freq", &freq);
	if (ret)
		fcm->initial_freq = FCM_DEFAULT_FREQUENCY;
	else
		fcm->initial_freq = freq;

	fcm_l3cache_parse_cpumask(pdev);

	of_node_put(node);

	return 0;
}

static int fcm_l3cache_create_configuration(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);
	int i;

	fcm->freq_table_len = fcm->portions;
	fcm->cur_num_portions = fcm->portions;

	fcm->freq_table = devm_kcalloc(&pdev->dev, fcm->portions,
				       sizeof(*fcm->freq_table),
				       GFP_KERNEL);
	if (IS_ERR(fcm->freq_table)) {
		dev_dbg(&pdev->dev, "No memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < fcm->portions; i++)
		fcm->freq_table[i] = i;

	/* Leakage (static power) for a single portion (in uW) */
	fcm->cacheLeakage = cache_leakage_per_mb;	/* uW/MB */
	fcm->cacheLeakage *= fcm->size;		/* KiB */
	fcm->cacheLeakage /= 1024ul * FCM_MAX_PORTIONS;

	return 0;
}

static void fcm_l3cache_remove_opps(struct platform_device *pdev)
{
	struct dev_pm_opp *opp;
	int i, count;
	unsigned long freq;

	count = dev_pm_opp_get_opp_count(&pdev->dev);
	if (count <= 0)
		return;

	rcu_read_lock();
	for (i = 0, freq = 0; i < count; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(&pdev->dev, &freq);
		if (!IS_ERR(opp))
			dev_pm_opp_remove(&pdev->dev, freq);
	}
	rcu_read_unlock();
}

static int fcm_l3cache_enable_opps(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);
	int i, ret;
	int opp_count = 0;
	unsigned int *voltage = fcm->freq_table;

	if (fcm->freq_table_len <= 0)
		return -EINVAL;

	for (i = 0; i < fcm->freq_table_len; i++) {
		ret = dev_pm_opp_add(&pdev->dev, fcm->freq_table[i],
				     *voltage++);
		if (ret)
			dev_warn(&pdev->dev, "Cannot add a new OPP\n");
		else
			opp_count++;
	}

	if (opp_count == 0) {
		dev_err(&pdev->dev, "device has no OPP registered\n");
		return -ENODEV;
	}

	return 0;
}

static int fcm_l3cache_setup(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);
	int ret;

	mutex_init(&fcm->lock);

	ret = fcm_l3cache_create_configuration(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot create frequency table\n");
		return ret;
	}

	ret = fcm_l3cache_enable_opps(pdev);
	if (ret) {
		dev_dbg(&pdev->dev, "Device setup failed\n");
		return ret;
	}

	ret = fcm_l3cache_setup_devfreq_profile(pdev);
	if (ret) {
		dev_dbg(&pdev->dev, "Device setup failed\n");
		return ret;
	}

	fcm->alg.last_update = jiffies_to_usecs(get_jiffies_64());

	fcm->devfreq = devm_devfreq_add_device(&pdev->dev,
					       fcm->devfreq_profile,
					       FCM_GOVERNOR_NAME, NULL);

	if (IS_ERR(fcm->devfreq)) {
		dev_err(&pdev->dev, "Registering to devfreq failed\n");
		return PTR_ERR(fcm->devfreq);
	}

	return 0;
}

static int fcm_l3cache_init_device_state(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);

	//TODO: poke the registers and set initial state,
	//so there wouldn't be a need to update_devfreq()

	mutex_lock(&fcm->devfreq->lock);
	fcm->devfreq->min_freq = FCM_DEFAULT_MIN_FREQ;
	fcm->devfreq->max_freq = fcm->portions - 1;
	mutex_unlock(&fcm->devfreq->lock);


	return 0;
}

static int fcm_l3cache_reinit_device(struct fcm_devfreq *fcm)
{
	unsigned int max_freq = fcm->freq_table[fcm->freq_table_len - 1];

	/* clean the algorithm statistics and start from scrach */
	memset(&fcm->alg, 0, sizeof(fcm->alg));
	fcm->cur_num_portions = max_freq;
	fcm->alg.last_update = jiffies_to_usecs(get_jiffies_64());

	//TODO: poke registers so no need to update_devfreq() in this place

	return 0;
}

static int fcm_l3cache_suspend(struct device *dev)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
	int ret = 0;

	ret = devfreq_suspend_device(fcm->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to suspend devfreq device\n");
		return ret;
	}

	//TODO: call the function which saves the registers and turns off l3

	return ret;
}

static int fcm_l3cache_resume(struct device *dev)
{

	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
	int ret = 0;

	fcm_l3cache_reinit_device(fcm);

	ret = devfreq_resume_device(fcm->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to resume devfreq device\n");
		return ret;
	}


	return ret;
}

static SIMPLE_DEV_PM_OPS(fcm_l3cache_pm, fcm_l3cache_suspend,
			 fcm_l3cache_resume);

static int fcm_devfreq_probe(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm;
	int ret;

	dev_info(&pdev->dev, "Registering FCM device\n");

	fcm = devm_kzalloc(&pdev->dev, sizeof(*fcm), GFP_KERNEL);
	if (!fcm) {
		dev_err(&pdev->dev, "Failed to register driver, no memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, fcm);
	fcm->pdev = pdev;
	fcm->id = atomic_inc_return(&fcm_device_id);

	ret = fcm_l3cache_parse_dt(pdev);
	if (ret)
		goto failed;

	ret = fcm_l3cache_setup(pdev);
	if (ret)
		goto failed;

	ret = fcm_l3cache_init_device_state(pdev);
	if (ret)
		goto failed;

	mutex_lock(&fcm_list_lock);
	list_add(&fcm->node, &fcm_device_list);
	mutex_unlock(&fcm_list_lock);

	ret = fcm_l3cache_cpu_notification_register();
	if (ret)
		pr_warn("FCM: failed to connect to cpu notifications\n");

	return 0;
failed:
	dev_err(&pdev->dev, "Failed to register driver, err %d\n", ret);
	kfree (fcm);
	return ret;
}

static int fcm_devfreq_remove(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "Unregisterring FCM device\n");

	mutex_lock(&fcm_list_lock);
	list_del(&fcm->node);
	mutex_unlock(&fcm_list_lock);

	fcm_l3cache_cpu_notification_unregister();

	fcm_l3cache_shutdown(pdev);
	devm_devfreq_remove_device(&pdev->dev, fcm->devfreq);

	fcm_l3cache_remove_opps(pdev);

	return 0;
}

static const struct of_device_id fcm_l3cache_devfreq_id[] = {
	{.compatible = "arm,fcm-l3-cache", },
	{}
};
MODULE_DEVICE_TABLE(of, fcm_l3cache_devfreq_id);

static struct platform_driver fcm_l3cache_devfreq_driver = {
	.probe	= fcm_devfreq_probe,
	.remove = fcm_devfreq_remove,
	.driver = {
		.name = FCM_PLATFORM_DEVICE_NAME,
		.of_match_table = fcm_l3cache_devfreq_id,
		.pm = &fcm_l3cache_pm,
		.owner = THIS_MODULE,
	},
};

	/*********************************************************************/

static int __init fcm_devfreq_init(void)
{
	int ret;

	ret = devfreq_add_governor(&fcm_devfreq_governor);
	if (ret) {
		pr_err("%s: failed to add governor: %d\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&fcm_l3cache_devfreq_driver);
	if (ret)
		devfreq_remove_governor(&fcm_devfreq_governor);

	return ret;
}

static void __exit fcm_devfreq_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&fcm_devfreq_governor);
	if (ret)
		pr_err("%s: failed to remove governor: %d\n", __func__, ret);

	platform_driver_unregister(&fcm_l3cache_devfreq_driver);

}

module_init(fcm_devfreq_init)
module_exit(fcm_devfreq_exit)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARM FCM devfreq driver");
MODULE_AUTHOR("Sean McGoogan <Sean.McGoogan@arm.com>");
MODULE_AUTHOR("Lukasz Luba <lukasz.luba@arm.com>");

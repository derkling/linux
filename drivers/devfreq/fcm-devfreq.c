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


#include <linux/devfreq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/random.h>

#include "governor.h"


	/*********************************************************************/


#define FCM_PLATFORM_DEVICE_NAME	"ARM-FCM"
#define FCM_GOVERNOR_NAME		"arm-atg"

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


#if defined(CONFIG_ARM64)
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
#endif	/* CONFIG_ARM64 */


	/*********************************************************************/


struct fcm_devfreq {
	struct devfreq		*devfreq;

		/* resources from the OF device-tree */
	u32			size;	/* FCM L3 cache-size in KiB */

		/* Leakage (static power) for a single portion (in uW) */
	unsigned long		cacheLeakage;

		/* most recent update in fcm_devfreq_target() */
	unsigned long		cur_num_portions;

	struct platform_device	*pdev;

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
	int			sensChecking;	/* boolean */
	int			sensSampleCounter;
	int			sensCounter;
	int			downsizeDefer;
	unsigned long		previous_missrate;
	unsigned long		reference_missrate;

		/* the devfreq "profile" structure */
	struct devfreq_dev_profile profile;

		/* As used for "frequency transition information" */
	unsigned int portion_table[FCM_MAX_PORTIONS-FCM_MIN_PORTIONS+1];
};


	/*********************************************************************/


static struct platform_device * fcm_platform_device[FCM_NUMBER_CLUSTERS];


	/*********************************************************************/


	/* Miss rate variation threshold to trigger the regulator */
static const u32 missrateChangeThreshold = 400*1000;	/* 0.4 * MILLION */

	/*
	 * The following 2 thresholds should be considered as being
	 * real numbers in the range 0.0 to 1.0, which represent
	 * a fractional part of a whole single portion.
	 * However, we scale them by multiplying by one million, so we
	 * can effect integer arithmetic exclusively in the kernel.
	 */
	/* Portion threshold to use for down-sizing (Td) */
static const u32 downsizePortionThreshold = 0;	/* between 0 and 1,000,000 */
	/* Portion threshold to use for up-sizing (Tu) */
static const u32 upsizePortionThreshold = 0;	/* between 0 and 1,000,000 */

	/* Total cache leakage (with 100% of the portions enabled) per MB */
static const u32 cacheLeakagePerMB = 10*1000;		/* expressed in uW/MB */
	/* Amount of energy used by the DRAM system per MB of transferred data (on average) */
static const u32 dramEnergyEstimatePerMB = 130;		/* expressed in uJ/MB */

	/* Activate the cache resizer regulator to control unnecessary resizings */
static const int resizeRegulationEnabled = true;	/* boolean */

	/* Number of periods to skip downsizing after regulator being triggered */
static const int downsizeDeferSkip = 4;

	/* Minimum miss bandwidth to trigger the regulator (Bytes/s) */
static const u64 bandwidthMarginalBandwidth = MILLION;	/* 1 MB/s */

	/* Number of samples to consider after a resize to control unnecessary resizing */
static const unsigned sensSamplingWindow = 6;
	/* Number of samples threshold to tag a resizing operation as 'bad' */
static const unsigned sensMissrateSampleThreshold = 5;

	/* QQQ obtain correctly/dynamically ??? */
static const unsigned cacheLineSize = 64;	/* in bytes */


	/*********************************************************************/


static int fcm_devfreq_target(struct device *dev,
				unsigned long *portions,
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
	unsigned long hits, misses, accesses;

	/*
	 * obtain the delta time since we last updated the time-stamp,
	 * and also update the time-stamp in fcm.
	 */
	delta = usec - fcm->last_update;
	fcm->last_update = usec;

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
	fcm->accesses_up   += accesses;
	fcm->accesses_down += accesses;
	fcm->misses_up     += misses;
	fcm->misses_down   += misses;
	fcm->usec_up       += delta;
	fcm->usec_down     += delta;

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
 * This function is only called if resizeRegulationEnabled is true.
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
	const u64 usec = fcm->usec_up;
		/* bandwidth in Bytes/sec */
	const u64 cacheMissBandwidth = cacheLineSize * fcm->misses_up * MILLION / usec;
		/*
		 * The "missrate" is scaled by a factor of 1 million,
		 * Hence a missrate of 800,000 == 80% miss-rate
		 * Note: the "+1" on the divisor to avoid divide by zero
		 */
	const u64 missrate = fcm->misses_up * MILLION /(fcm->accesses_up+1);

	/* avoid the possibility of divide by zero! */
	if (!fcm->previous_missrate)
		fcm->previous_missrate = 1;

	/* To trigger or not to trigger */
	if (!fcm->sensChecking && cacheMissBandwidth >= bandwidthMarginalBandwidth) {
		long missrate_change = (missrate - fcm->previous_missrate)/fcm->previous_missrate;
		if (missrate_change >= missrateChangeThreshold) {
			fcm->sensChecking = true;
			fcm->sensCounter = sensSamplingWindow;
			fcm->sensSampleCounter = 0;
			fcm->reference_missrate = fcm->previous_missrate;
		}
	}

	fcm->previous_missrate = missrate;

	/* avoid the possibility of divide by zero! */
	if (!fcm->reference_missrate)
		fcm->reference_missrate = 1;

	/* Sample missrate */
	if (fcm->sensChecking) {
		if (fcm->sensCounter-- == 0) {
			/* Compare the bad samples to the threshold */
			if (fcm->sensSampleCounter >= sensMissrateSampleThreshold) {
				ret = 1;
				fcm->downsizeDefer = downsizeDeferSkip;
			}
			fcm->sensChecking = false;
		}
		else if ((missrate - fcm->reference_missrate)/fcm->reference_missrate > missrateChangeThreshold) {
			fcm->sensSampleCounter++;
		}
	}

	return ret;
}

/*
 * This is a "regular" that is can be used to override the decision
 * to down-size taken in the function downSizeCheck() - our caller.
 * This function is only called if resizeRegulationEnabled is true.
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
	const u64 usec = fcm->usec_down;
		/* bandwidth in Bytes/sec */
	const u64 cacheMissBandwidth = cacheLineSize * fcm->misses_down * MILLION / usec;

		/* May defer a downsize if appropriate */
	if (fcm->downsizeDefer > 0) {
		fcm->downsizeDefer--;
		return 0;	/* defer downsizing */
	}

	if (down != 0 && cacheMissBandwidth >= bandwidthMarginalBandwidth) {
		fcm->sensChecking = true;
		fcm->sensCounter = sensSamplingWindow;
		fcm->sensSampleCounter = 0;
		fcm->reference_missrate = fcm->previous_missrate;
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
 * dramEnergyEstimatePerMB = Amount of energy used by the DRAM system per
 *			MB of transferred data (on average)
 *			(expressed in uJ/MB)
 *
 * ED = dramEnergyEstimatePerMB
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
	const u64 usec = fcm->usec_up;
		/* bandwidth in Bytes/sec */
	const u64 cacheMissBandwidth = cacheLineSize * fcm->misses_up * MILLION / usec;
	u64 pivot;

	pivot = MILLION;			/* 1.0, scaled by 1 million */
	pivot -= upsizePortionThreshold;	/* Tu (already pre-scaled) */
	pivot *= fcm->cacheLeakage;		/* L */
	pivot /= dramEnergyEstimatePerMB;	/* ED */

	if (cacheMissBandwidth > pivot)	/* if ( MBW > (1.0-Tu) * L / ED ) */
		ret = 1;	/* increase by ONE portion */

	/* Check whether regulator overrides the previous decision */
	if (resizeRegulationEnabled)
		ret = regulatorUpsize(devfreq, ret);

	/* reset (to zero) the consumed counters */
	fcm->usec_up = fcm->misses_up = fcm->accesses_up = 0;

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
 * dramEnergyEstimatePerMB = Amount of energy used by the DRAM system per
 *			MB of transferred data (on average)
 *			(expressed in uJ/MB)
 *
 * ED = dramEnergyEstimatePerMB
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
	const u64 usec = fcm->usec_down;
		/* bandwidth in Bytes/sec */
	const u64 cacheBandwidth = cacheLineSize * fcm->accesses_down * MILLION / usec;
	const u64 cacheMissBandwidth = cacheLineSize * fcm->misses_down * MILLION / usec;
	const u64 cacheHitBandwidth = cacheBandwidth - cacheMissBandwidth;
	u64 pivot;

	if (num_active_portions < 1)
		return 0;	/* do not even try and down-size! */
	/* above "if" avoids the need for "assert(N-Td >= 0.0);" */

	pivot = num_active_portions;		/* N */
	pivot *= MILLION;			/* scale by 1 million */
	pivot -= downsizePortionThreshold;	/* Td (already pre-scaled) */
	pivot *= fcm->cacheLeakage;		/* L */
	pivot /= dramEnergyEstimatePerMB;	/* ED */

	if (cacheHitBandwidth < pivot) /* if ( HBW < (N-Td) * L / ED ) */
		ret = -1;	/* decrease by ONE portion */

	/* Check whether regulator overrides the previous decision */
	if (resizeRegulationEnabled)
		ret = regulatorDownsize(devfreq, ret);

	/* reset (to zero) the consumed counters */
	fcm->usec_down = fcm->misses_down = fcm->accesses_down = 0;

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
	struct devfreq_dev_status *stat = &devfreq->last_status;
	unsigned long target_portions;
	int err;
	unsigned long long  busyness;
	int up = 0, down = 0;
	int num_active_portions = fcm->cur_num_portions;
	int new_active_portions = num_active_portions;

	/* Note: *portions is uninitialized by our caller! */

	err = devfreq_update_stats(devfreq);
	if (err)
		return err;

	/* compute the "level of business" (as a percentage) ... */
	busyness = stat->busy_time * 100;
	busyness = div_u64(busyness, stat->total_time);

	/* Trigger up-sizing / downsizing logic check */
	/* check for up-sizing EVERY time we are polled */
	up   = upSizeCheck(devfreq, num_active_portions);
	/* but, check only every poll_ratio times for down-sizing */
	if (++fcm->poll_ratio == POLLING_DOWN_MS/POLLING_UP_MS) {
		down = downSizeCheck(devfreq, num_active_portions);
		fcm->poll_ratio = 0;
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

	/* update the new "target" ... */
	target_portions = new_active_portions;
	*portions = target_portions;

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


static int fcm_devfreq_probe(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm;
	struct dev_pm_opp *min_opp, *max_opp;
	unsigned long max_portions, min_portions;
	unsigned int i;

	fcm = devm_kzalloc(&pdev->dev, sizeof(*fcm), GFP_KERNEL);
	if (!fcm)
		return -ENOMEM;

	fcm->pdev = pdev;
	fcm->last_update = jiffies_to_usecs(get_jiffies_64());

	platform_set_drvdata(pdev, fcm);

	/*
	 * Obtained the minimum & maximum number of portions.
	 * Recall, we are equating "frequency" to "portions"!
	 *	i.e. 1 Hertz == 1 portion, etc...
	 */
	min_portions = 0;		/* find ceil(minimum) */
	max_portions = ULONG_MAX;	/* find floor(maximum) */
	rcu_read_lock();
	min_opp = dev_pm_opp_find_freq_ceil(&pdev->dev, &min_portions);
	max_opp = dev_pm_opp_find_freq_floor(&pdev->dev, &max_portions);
	if (IS_ERR(max_opp) || IS_ERR(min_opp)) {
		rcu_read_unlock();
		dev_err(&pdev->dev, "Failed to find any OPP!\n");
		return IS_ERR(max_opp) ? PTR_ERR(max_opp) : PTR_ERR(min_opp);
	}
	min_portions = dev_pm_opp_get_freq(min_opp);
	max_portions = dev_pm_opp_get_freq(max_opp);
	rcu_read_unlock();

	/* initialize the default number of portions to maximum supported */
	fcm->profile.initial_freq = max_portions;
	fcm->cur_num_portions = max_portions;

	/* initialize the "portion_table" */
	for (i=0; i<ARRAY_SIZE(fcm->portion_table); i++)
		fcm->portion_table[i] = i + FCM_MIN_PORTIONS;
	fcm->profile.freq_table = fcm->portion_table;
	fcm->profile.max_state = ARRAY_SIZE(fcm->portion_table);

	/* derived data from the OF device-tree */
	fcm->size = 2048;	/* 2 MiB */	/* QQQ: fill in from OF */
		/* Leakage (static power) for a single portion (in uW) */
	fcm->cacheLeakage = cacheLeakagePerMB;	/* uW/MB */
	fcm->cacheLeakage *= fcm->size;		/* KiB */
	fcm->cacheLeakage /= 1024ul * FCM_MAX_PORTIONS;

	/* define the polling interval, that devfreq will use for us */
	fcm->profile.polling_ms = POLLING_UP_MS;

	/* finally, initialize the profile's call-back functions */
	fcm->profile.target = fcm_devfreq_target;
	fcm->profile.get_dev_status = fcm_devfreq_get_dev_status;

	/*
	 * NOTE: there is one instance of the profile
	 * per FCM device. Hence, they do NOT share the
	 * same polling_ms or freq_table/portion_table.
	 */
	fcm->devfreq = devm_devfreq_add_device(
				&pdev->dev,
				&fcm->profile,
				FCM_GOVERNOR_NAME,
				NULL);

	mutex_lock(&fcm->devfreq->lock);
	fcm->devfreq->min_freq = min_portions;
	fcm->devfreq->max_freq = max_portions;
	mutex_unlock(&fcm->devfreq->lock);

	return 0;
}

static int fcm_devfreq_remove(struct platform_device *pdev)
{
	int num_available;
	unsigned long freq = 0;
	struct dev_pm_opp *opp;

	rcu_read_lock();
	num_available = dev_pm_opp_get_opp_count(&pdev->dev);

	if (num_available > 0) {
		while (!IS_ERR(opp = dev_pm_opp_find_freq_ceil(&pdev->dev, &freq))) {
			dev_pm_opp_remove(&pdev->dev, freq);
			freq++;
		}
	}
	rcu_read_unlock();

	return 0;
}

static struct platform_driver arm_devfreq_driver = {
	.probe	= fcm_devfreq_probe,
	.remove	= fcm_devfreq_remove,
	.driver = {
		.name = FCM_PLATFORM_DEVICE_NAME,
	},
};


	/*********************************************************************/


static void fcm_destroy_all_devices(void)
{
	int i;
	struct platform_device * pdev;

	for(i=0; i<FCM_NUMBER_CLUSTERS; i++) {
		pdev = fcm_platform_device[i];
		if (pdev) {
			platform_device_unregister(pdev);
			fcm_platform_device[i] = NULL;
		}
	}
}

static struct platform_device * fcm_create_device(int id)
{
	struct platform_device * pdev;
	unsigned long portions;

	/*
	 * The following is a bit of a dirty hack!
	 * The following should all probably be done via a device-tree:
	 *	platform_device_register_data()
	 *	dev_pm_opp_add()
	 *	platform_device_unregister()
	 */
	pdev = platform_device_register_data(NULL,
		FCM_PLATFORM_DEVICE_NAME, id, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("%s: unable to register '%s.%d' : %ld\n",
		__func__, FCM_PLATFORM_DEVICE_NAME, id, PTR_ERR(pdev));
		return pdev;
	}

	/*
	 * Create the available OPPs.
	 * Note, it should be noted that we are (egregiously) interpreting
	 * the frequency (in Hertz) to be the number of portions!
	 * Also, we are setting the Voltage to be zero uV.
	 * Essentially, we are ignoring Volts, and pretending Hertz is number of portions!
	 */
	for (portions = FCM_MIN_PORTIONS;
		portions <= FCM_MAX_PORTIONS;
		portions++) {
		dev_pm_opp_add(&pdev->dev, portions, 0);
	}

	return pdev;
}

static int __init fcm_devfreq_init(void)
{
	int ret;
	int i;
	struct platform_device *pdev;

	for(i=0; i<FCM_NUMBER_CLUSTERS; i++) {
		pdev = fcm_create_device(i);
		if (IS_ERR(pdev)) {
			fcm_destroy_all_devices();
			return PTR_ERR(pdev);
		}
		fcm_platform_device[i] = pdev;
	}

	ret = devfreq_add_governor(&fcm_devfreq_governor);
	if (ret) {
		pr_err("%s: failed to add governor: %d\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&arm_devfreq_driver);
	if (ret)
		devfreq_remove_governor(&fcm_devfreq_governor);

	return ret;
}
module_init(fcm_devfreq_init)

static void __exit fcm_devfreq_exit(void)
{
	int ret;

	platform_driver_unregister(&arm_devfreq_driver);

	fcm_destroy_all_devices();

	ret = devfreq_remove_governor(&fcm_devfreq_governor);
	if (ret)
		pr_err("%s: failed to remove governor: %d\n", __func__, ret);
}
module_exit(fcm_devfreq_exit)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARM FCM devfreq driver");
MODULE_AUTHOR("Sean McGoogan <Sean.McGoogan@arm.com>");

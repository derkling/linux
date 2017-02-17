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
 *
 * The L3 Cache in FCM supports partial power down, splitting the cache into an
 * implementation-specific number of portions. This driver provides an
 * energy-cost justified demand-driven policy for controlling the number of
 * portions enabled.
 *
 * The policy maps the number of portions enabled to frequency (1 portion ==
 * 1Hz) and provides usage statistics so that existing DevFreq governors can
 * control the number of enabled portions. Specifically, using the
 * simple_ondemand governor will implement the desired
 * energy-cost-justification policy.
 * DevFreq min/max/governor controls work as usual.
 *
 * There is no relation to actual frequency control of the cache device.
 */

#include <linux/devfreq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#include "governor.h"

#define CREATE_TRACE_POINTS
#include <trace/events/fcm_l3cache.h>

#define FCM_PLATFORM_DEVICE_NAME		"fcm_l3cache"
#define FCM_GOVERNOR_NAME			"fcm_governor"

#define FCM_DEFAULT_PORTIONS			4
#define FCM_DEFAULT_PORTION_MIN			1
#define FCM_DEFAULT_PORTION_MAX			4

#define FCM_DEFAULT_POLLING_MS			10
#define POLLING_DOWN_INTERVAL			10
/* Required polling time in microseconds */
#define FCM_DEFAULT_REQUIRED_GOV_POLLING	10000

/* Default cache line size */
#define FCM_DEFAULT_LINE_SIZE			64

/* Default size of the L3 cache: 1MB */
#define FCM_DEFAULT_SIZE_KB			1024

/* Default static energy leaked by the L3 cache per MB */
#define FCM_DEFAULT_CACHE_LEAKAGE		10000

/* Amount of energy used by the DRAM system per MB
 * of transferred data (on average) expressed in uJ/MB */
#define FCM_DEFAULT_DRAM_ENERGY_PER_MB		130

#define SZ_1KB					(1ULL << 10)
#define SZ_1MB					(1ULL << 20)
#define ROUNDING_INTEGER_MUL			1000000
#define PERCENT_MAX				100

/* Portion threshold to use for down-sizing (Td)
 * between 0 and 100
 * 90 for 90% */
#define DOWNSIZE_PORTION_THRESHOLD		90

/* Portion threshold to use for up-sizing (Tu)
 * between 0 and 1MB
 * 10 for 10% */
#define UPSIZE_PORTION_THRESHOLD		10

/* bit-field positions for the CLUSTERPWRCTLR_EL1 register */
#define PORTION_1		(4)		/* Ways  0-3	PACTIVE[16] */
#define PORTION_2		(5)		/* Ways  4-7	PACTIVE[17] */
#define PORTION_3		(6)		/* Ways  8-11	PACTIVE[18] */
#define PORTION_4		(7)		/* Ways 12-15	PACTIVE[19] */
/* bit-masks for the CLUSTERPWRCTLR_EL1 register */
#define PORTION_BITS		( (1UL << PORTION_1) |	\
				  (1UL << PORTION_2) |	\
				  (1UL << PORTION_3) |	\
				  (1UL << PORTION_4) )
#define PORTION_MASK		(~(PORTION_BITS))

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

#endif


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
	u32 portion_min;
	u32 portion_max;
	u32 cache_leakage_per_mb;
	u32 dram_energy_per_mb;

	unsigned int *freq_table;
	int freq_table_len;

	struct mutex lock;
	struct hrtimer poll_timer;

	/* Leakage (static power) for a single portion (in uW) */
	unsigned long cache_leakage;
	unsigned long downsize_threshold;
	unsigned long upsize_threshold;
	unsigned long max_threshold;
	unsigned long cur_num_portions;

	/* Contains state for the algorithm. It is clean during resume */
	struct {
		unsigned long accesses_up;
		unsigned long misses_up;
		unsigned long accesses_down;
		unsigned long misses_down;
		unsigned long usec_up;
		unsigned long usec_down;
		unsigned int last_update;
	} alg;
};

static atomic_t fcm_device_id = ATOMIC_INIT(0);


static int fcm_l3cache_devfreq_target(struct device *dev,
				      unsigned long *portions, u32 flags)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
	unsigned long portion_active;
	unsigned long portion_control;

	if (*portions < fcm->portion_min || *portions > fcm->portion_max) {
		dev_warn(dev, "%s: Target of %lu-portions is "
			 "outside range of %u..%u\n",__func__, *portions,
			 fcm->portion_min, fcm->portion_max);

		return -EINVAL;
	}

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
	portion_active = ((1UL << (*portions)) - 1) << PORTION_1;

	SYS_REG_READ(S3_0_c15_c3_5, portion_control);

	portion_control &= PORTION_MASK;
	portion_control |= portion_active;

	SYS_REG_WRITE(S3_0_c15_c3_5, portion_control);

	fcm->cur_num_portions = *portions;

	return 0;
}

static int fcm_l3cache_devfreq_get_dev_status(struct device *dev,
					      struct devfreq_dev_status *stat)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
	unsigned int const usec = ktime_to_us(ktime_get());
	unsigned int delta;
	unsigned long hits = 0;
	unsigned long misses = 0;
	unsigned long accesses;

	delta = usec - fcm->alg.last_update;
	fcm->alg.last_update = usec;

	stat->current_frequency = fcm->cur_num_portions;

	SYS_REG_READ_THEN_CLEAR(S3_0_c15_c4_5, hits);
	SYS_REG_READ_THEN_CLEAR(S3_0_c15_c4_6, misses);

	accesses = hits + misses;

	fcm->alg.accesses_up   += accesses;
	fcm->alg.accesses_down += accesses;
	fcm->alg.misses_up     += misses;
	fcm->alg.misses_down   += misses;
	fcm->alg.usec_up       += delta;
	fcm->alg.usec_down     += delta;

	if (!accesses)
		accesses = 1;

	/*
	 * Also map into stat format used by struct devfreq_dev_status, in
	 * devfreq_simple_ondemand_func() in "governor_simpleondemand.c".
	 *	stat->total_time = delta time (in us)
	 *	stat->busy_time  = busyness (total_time*hits/accesses)
	 */
	stat->total_time = delta;
	stat->busy_time  = stat->total_time * hits / accesses;

	trace_fcm_l3cache_dev_status(fcm->id, hits, misses,
				     fcm->cur_num_portions, stat->busy_time,
				     stat->total_time);

	return 0;
}

static int fcm_l3cache_up_size_check(struct fcm_devfreq *fcm)
{
	unsigned long cache_miss_bw;
	int ret = 0;

	if (fcm->cur_num_portions >= fcm->portion_max)
		return 0;

	cache_miss_bw = fcm->line_size * fcm->alg.misses_up;
	cache_miss_bw *= ROUNDING_INTEGER_MUL;
	cache_miss_bw /= fcm->alg.usec_up;

	if (cache_miss_bw > fcm->upsize_threshold)
		ret = 1;

	fcm->alg.usec_up = 0;
	fcm->alg.misses_up = 0;
	fcm->alg.accesses_up = 0;

	return ret;
}

static int fcm_l3cache_down_size_check(struct fcm_devfreq *fcm)
{
	int ret = 0;
	unsigned long cache_bw;
	unsigned long cache_miss_bw;
	unsigned long cache_hit_bw;

	if (fcm->cur_num_portions <= fcm->portion_min)
		return 0;

	cache_bw = fcm->line_size * fcm->alg.accesses_down;
	cache_bw *= ROUNDING_INTEGER_MUL;
	cache_bw /= fcm->alg.usec_down;

	cache_miss_bw = fcm->line_size * fcm->alg.misses_down;
	cache_miss_bw *= ROUNDING_INTEGER_MUL;
	cache_miss_bw /= fcm->alg.usec_down;

	cache_hit_bw = cache_bw - cache_miss_bw;

	if (cache_hit_bw < (fcm->max_threshold * fcm->cur_num_portions -
			    fcm->downsize_threshold))
		ret = 1;

	fcm->alg.usec_down = 0;
	fcm->alg.misses_down = 0;
	fcm->alg.accesses_down = 0;

	return ret;
}

static unsigned long fcm_l3cache_calc_next_portions(struct fcm_devfreq *fcm)
{
	unsigned long portions = fcm->cur_num_portions;
	int up = 0, down = 0;

	up = fcm_l3cache_up_size_check(fcm);
	if (up > 0) {
		return fcm->cur_num_portions + up;
	}

	if (fcm->alg.usec_down >= (POLLING_DOWN_INTERVAL * fcm->polling_ms)) {
		down = fcm_l3cache_down_size_check(fcm);
		if (down > 0)
			portions = fcm->cur_num_portions - down;
	}

	return portions;
}

static int fcm_l3cache_governor_get_target_portions(struct devfreq *df,
				unsigned long *portions)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(df->dev.parent);
	int err;

	err = devfreq_update_stats(df);
	if (err)
		return err;

	*portions = fcm_l3cache_calc_next_portions(fcm);

	return 0;
}

static int fcm_l3cache_governor_event_handler(struct devfreq *devfreq,
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
		devfreq_interval_update(devfreq, (unsigned int *)data);
		break;
	}

	return ret;
}

static struct devfreq_governor fcm_l3cache_devfreq_governor = {
	.name		 = FCM_GOVERNOR_NAME,
	.get_target_freq = fcm_l3cache_governor_get_target_portions,
	.event_handler	 = fcm_l3cache_governor_event_handler,
};

static enum hrtimer_restart fcm_l3cache_polling(struct hrtimer *hrtimer)
{
	struct fcm_devfreq *fcm = container_of(hrtimer, struct fcm_devfreq,
					       poll_timer);

	mutex_lock(&fcm->devfreq->lock);
	update_devfreq(fcm->devfreq);
	mutex_unlock(&fcm->devfreq->lock);

	hrtimer_forward_now(&fcm->poll_timer, ms_to_ktime(fcm->polling_ms));
	return HRTIMER_RESTART;
}

static int fcm_l3cache_reinit_device(struct fcm_devfreq *fcm)
{
	unsigned long portion_control = 0;
	unsigned long portion_active;

	/* clean the algorithm statistics and start from scrach */
	memset(&fcm->alg, 0, sizeof(fcm->alg));
	fcm->cur_num_portions = fcm->initial_freq;
	fcm->alg.last_update = ktime_to_us(ktime_get());

	portion_active = ((1UL << fcm->initial_freq) - 1) << PORTION_1;

	SYS_REG_READ(S3_0_c15_c3_5, portion_control);

	portion_control &= PORTION_MASK;
	portion_control |= portion_active;

	SYS_REG_WRITE(S3_0_c15_c3_5, portion_control);

	return 0;
}

static int fcm_l3cache_shutdown_portions(struct fcm_devfreq *fcm)
{
	unsigned long portion_control = 0;
	unsigned long portion_active;

	portion_active = ((1UL << fcm->portion_min) - 1) << PORTION_1;

	SYS_REG_READ(S3_0_c15_c3_5, portion_control);

	portion_control &= PORTION_MASK;
	portion_control |= portion_active;

	SYS_REG_WRITE(S3_0_c15_c3_5, portion_control);

	return 0;
}

static int fcm_l3cache_shutdown(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);

	fcm_l3cache_shutdown_portions(fcm);

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

	df_profile->target = fcm_l3cache_devfreq_target;
	df_profile->get_dev_status = fcm_l3cache_devfreq_get_dev_status;
	df_profile->freq_table = fcm->freq_table;
	df_profile->max_state = fcm->freq_table_len;
	df_profile->polling_ms = FCM_DEFAULT_REQUIRED_GOV_POLLING;
	df_profile->initial_freq = fcm->initial_freq;

	return 0;
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

	ret = of_property_read_u32(node, "portion-min", &fcm->portion_min);
	if (ret)
		fcm->portion_min = FCM_DEFAULT_PORTION_MIN;

	ret = of_property_read_u32(node, "portion-max", &fcm->portion_max);
	if (ret)
		fcm->portion_max = fcm->portions;

	ret = of_property_read_u32(node, "cache-leakage-per-mb",
				   &fcm->cache_leakage_per_mb);
	if (ret)
		fcm->cache_leakage_per_mb = FCM_DEFAULT_CACHE_LEAKAGE;

	ret = of_property_read_u32(node, "dram-energy-per-mb",
				   &fcm->dram_energy_per_mb);
	if (ret)
		fcm->dram_energy_per_mb = FCM_DEFAULT_DRAM_ENERGY_PER_MB;

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
		fcm->initial_freq = fcm->portion_max;
	else
		fcm->initial_freq = freq;

	of_node_put(node);

	return 0;
}

static int fcm_l3cache_create_configuration(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);
	int i;
	unsigned long portion;

	fcm->freq_table_len = fcm->portion_max - fcm->portion_min + 1;
	fcm->cur_num_portions = fcm->initial_freq;

	fcm->freq_table = devm_kcalloc(&pdev->dev, fcm->portions,
				       sizeof(*fcm->freq_table),
				       GFP_KERNEL);
	if (IS_ERR(fcm->freq_table)) {
		dev_dbg(&pdev->dev, "No memory\n");
		return -ENOMEM;
	}

	portion = fcm->portion_min;
	for (i = 0; portion <= fcm->portion_max; i++)
		fcm->freq_table[i] = portion++;

	/* Leakage (static power) for a single portion (in uW) */
	fcm->cache_leakage = fcm->cache_leakage_per_mb * fcm->size / SZ_1KB;
	fcm->cache_leakage /= fcm->portions;

	/* Downsize and upsize thresholds are given in percentages */
	fcm->downsize_threshold = DOWNSIZE_PORTION_THRESHOLD;
	fcm->downsize_threshold *= ROUNDING_INTEGER_MUL / PERCENT_MAX;
	fcm->downsize_threshold *= fcm->cache_leakage;
	fcm->downsize_threshold /= fcm->dram_energy_per_mb;

	fcm->upsize_threshold = UPSIZE_PORTION_THRESHOLD;
	fcm->upsize_threshold *= ROUNDING_INTEGER_MUL / PERCENT_MAX;
	fcm->upsize_threshold *= fcm->cache_leakage;
	fcm->upsize_threshold /= fcm->dram_energy_per_mb;

	fcm->max_threshold = ROUNDING_INTEGER_MUL;
	fcm->max_threshold *= fcm->cache_leakage;
	fcm->max_threshold /= fcm->dram_energy_per_mb;

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

	if (fcm->freq_table_len <= 0)
		return -EINVAL;

	for (i = 0; i < fcm->freq_table_len; i++) {
		ret = dev_pm_opp_add(&pdev->dev, fcm->freq_table[i],
				     fcm->freq_table[i]);
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

	fcm->alg.last_update = ktime_to_us(ktime_get());

	fcm->devfreq = devm_devfreq_add_device(&pdev->dev,
					       fcm->devfreq_profile,
					       FCM_GOVERNOR_NAME, NULL);

	if (IS_ERR(fcm->devfreq)) {
		dev_err(&pdev->dev, "Registering to devfreq failed\n");
		return PTR_ERR(fcm->devfreq);
	}

	mutex_lock(&fcm->devfreq->lock);
	fcm->devfreq->min_freq = fcm->portion_min;
	fcm->devfreq->max_freq = fcm->portion_max;
	mutex_unlock(&fcm->devfreq->lock);

	return 0;
}


static int fcm_l3cache_init_device(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);

	fcm_l3cache_reinit_device(fcm);

	hrtimer_init(&fcm->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fcm->poll_timer.function = fcm_l3cache_polling;
	hrtimer_start(&fcm->poll_timer, ms_to_ktime(fcm->polling_ms),
		      HRTIMER_MODE_REL);

	return 0;
}

static int fcm_l3cache_suspend(struct device *dev)
{
	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
	int ret = 0;

	if (hrtimer_active(&fcm->poll_timer))
	    hrtimer_cancel(&fcm->poll_timer);

	ret = devfreq_suspend_device(fcm->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to suspend devfreq device\n");
		return ret;
	}

	fcm_l3cache_shutdown_portions(fcm);

	return ret;
}

static int fcm_l3cache_resume(struct device *dev)
{

	struct fcm_devfreq *fcm = dev_get_drvdata(dev);
	int ret = 0;

	fcm_l3cache_reinit_device(fcm);

	ret = devfreq_resume_device(fcm->devfreq);
	if (ret < 0)
		dev_err(dev, "failed to resume devfreq device\n");

	if (!hrtimer_active(&fcm->poll_timer))
	    hrtimer_start(&fcm->poll_timer, ms_to_ktime(fcm->polling_ms),
			  HRTIMER_MODE_REL);

	return ret;
}

static SIMPLE_DEV_PM_OPS(fcm_l3cache_pm, fcm_l3cache_suspend,
			 fcm_l3cache_resume);

static int fcm_l3cache_devfreq_probe(struct platform_device *pdev)
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

	ret = fcm_l3cache_init_device(pdev);
	if (ret)
		goto failed;

	return 0;
failed:
	dev_err(&pdev->dev, "Failed to register driver, err %d\n", ret);
	kfree (fcm);
	return ret;
}

static int fcm_l3cache_devfreq_remove(struct platform_device *pdev)
{
	struct fcm_devfreq *fcm = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "Unregisterring FCM device\n");

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
	.probe	= fcm_l3cache_devfreq_probe,
	.remove = fcm_l3cache_devfreq_remove,
	.driver = {
		.name = FCM_PLATFORM_DEVICE_NAME,
		.of_match_table = fcm_l3cache_devfreq_id,
		.pm = &fcm_l3cache_pm,
		.owner = THIS_MODULE,
	},
};

static int __init fcm_l3cache_devfreq_init(void)
{
	int ret;

	ret = devfreq_add_governor(&fcm_l3cache_devfreq_governor);
	if (ret) {
		pr_err("%s: failed to add governor: %d\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&fcm_l3cache_devfreq_driver);
	if (ret)
		devfreq_remove_governor(&fcm_l3cache_devfreq_governor);

	return ret;
}

static void __exit fcm_l3cache_devfreq_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&fcm_l3cache_devfreq_governor);
	if (ret)
		pr_err("%s: failed to remove governor: %d\n", __func__, ret);

	platform_driver_unregister(&fcm_l3cache_devfreq_driver);

}

module_init(fcm_l3cache_devfreq_init)
module_exit(fcm_l3cache_devfreq_exit)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARM FCM devfreq driver");
MODULE_AUTHOR("ARM Ltd.");
MODULE_VERSION("1.0");

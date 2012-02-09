/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010-2011
 *
 * Author: Rickard Andersson <rickard.andersson@stericsson.com>,
 *	   Jonas Aaberg <jonas.aberg@stericsson.com> for ST-Ericsson.
 *
 * Loosely based on cpuidle.c by Sundar Iyer.
 *
 * License terms: GNU General Public License (GPL) version 2
 *
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/clockchips.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/regulator/db8500-prcmu.h>

#include <mach/pm.h>
#include <plat/mtu.h>

#include "cpuidle-dbx500.h"

/*
 * All measurements are with two cpus online (worst case) and at
 * 200 MHz (worst case)
 *
 * Enter latency depends on cpu frequency, and is only depending on
 * code executing on the ARM.
 * Exit latency is both depending on "wake latency" which is the
 * time between the PRCMU has gotten the interrupt and the ARM starts
 * to execute and the time before everything is done on the ARM.
 * The wake latency is more or less constant related to cpu frequency,
 * but can differ depending on what the modem does.
 * Wake latency is not included for plain WFI.
 * For states that uses RTC (Sleep & DeepSleep), wake latency is reduced
 * from clock programming timeout.
 *
 */
#define DEEP_SLEEP_WAKE_UP_LATENCY 8500
/* Wake latency from ApSleep is measured to be around 1.0 to 1.5 ms */
#define MIN_SLEEP_WAKE_UP_LATENCY 1000
#define MAX_SLEEP_WAKE_UP_LATENCY 1500

#define UL_PLL_START_UP_LATENCY 8000 /* us */

static struct cstate cstates[] = {
	{
		.enter_latency = 0,
		.exit_latency = 0,
		.threshold = 0,
		.power_usage = 1000,
		.APE = APE_ON,
		.ARM = ARM_ON,
		.UL_PLL = UL_PLL_ON,
		.ESRAM = ESRAM_RET,
		.pwrst = PRCMU_AP_NO_CHANGE,
		.state = CI_RUNNING,
		.desc = "Running                ",
	},
	{
		/* These figures are not really true. There is a cost for WFI */
		.enter_latency = 0,
		.exit_latency = 0,
		.threshold = 0,
		.power_usage = 19,
		.APE = APE_ON,
		.ARM = ARM_ON,
		.UL_PLL = UL_PLL_ON,
		.ESRAM = ESRAM_RET,
		.pwrst = PRCMU_AP_NO_CHANGE,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.state = CI_WFI,
		.desc = "Wait for interrupt     ",
	},
	{ /* wait for Ret of 2nd cpu. Can't be selected by meny governor */
		.enter_latency = 170,
		.exit_latency = 70,
		.threshold = 260,
		.power_usage = 19,
		.APE = APE_ON,
		.ARM = ARM_ON,
		.UL_PLL = UL_PLL_ON,
		.ESRAM = ESRAM_RET,
		.pwrst = PRCMU_AP_IDLE,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.state = CI_WFIDLE,
		.desc = "Wait for ApIdle        ",
	},
	{
		.enter_latency = 170,
		.exit_latency = 70,
		.threshold = 260,
		.power_usage = 4,
		.APE = APE_ON,
		.ARM = ARM_RET,
		.UL_PLL = UL_PLL_ON,
		.ESRAM = ESRAM_RET,
		.pwrst = PRCMU_AP_IDLE,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.state = CI_IDLE,
		.desc = "ApIdle                 ",
	},
	{ /* Wait for cluster Off for cpu0. Power gate for cpu1. Can't be selected by menu governor */
		.enter_latency = 170,
		.exit_latency = 1500,
		.threshold = 1500 + 400 + 400,
		.power_usage = 19,
		.APE = APE_ON,
		.ARM = ARM_ON,
		.UL_PLL = UL_PLL_ON,
		.ESRAM = ESRAM_RET,
		.pwrst = PRCMU_AP_IDLE,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.state = CI_PG1CPU,
		.desc = "Fake power off 1 CPU   ",
	},
	{ /* simulate cluster off state */
		.enter_latency = 170,
		.exit_latency = 1500,
		.threshold = 1500 + 400 + 400,
		.power_usage = 0,
		.APE = APE_ON,
		.ARM = ARM_RET,
		.UL_PLL = UL_PLL_ON,
		.ESRAM = ESRAM_RET,
		.pwrst = PRCMU_AP_IDLE,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.state = CI_SLEEP,
		.desc = "Fake ApDeepIdle        ",
	},
};

static enum ci_pwrst cstates_1CPU[NR_CPUS][CI_END] = {
 {	CI_RUNNING, /* running -> running */
	CI_WFI,     /* WFI -> WFI */
	CI_WFIDLE,  /* Not possible */
	CI_WFIDLE,  /* Retention -> Wait for retention */
	CI_PG1CPU,  /* Not possible */
	CI_PG1CPU,  /* Off -> Wait for Off */
 },
 {	CI_RUNNING, /* running -> running */
	CI_WFI,     /* WFI -> WFI */
	CI_WFIDLE,  /* Not possible */
	CI_WFIDLE,  /* Retention -> Wait for retention */
	CI_PG1CPU,  /* Not possible */
	CI_PG1CPU,  /* Off -> Wait for Off */
 },
};

struct cpu_state {
	int gov_cstate;
	ktime_t sched_wake_up;
	struct cpuidle_device dev;
	bool restore_arm_core;
};

static DEFINE_PER_CPU(struct cpu_state, *cpu_state);

static DEFINE_SPINLOCK(cpuidle_lock);

static atomic_t idle_cpus_counter = ATOMIC_INIT(0);
static atomic_t master_counter = ATOMIC_INIT(0);

struct cstate *ux500_ci_get_cstates(int *len)
{
	if (len != NULL)
		(*len) = ARRAY_SIZE(cstates);
	return cstates;
}

/**
 * get_remaining_sleep_time() - returns remaining sleep time in
 * microseconds (us)
 */
static u32 get_remaining_sleep_time(ktime_t *next, int *on_cpu)
{
	ktime_t now, t;
	int cpu;
	int delta;
	u32 remaining_sleep_time = UINT_MAX;

	now = ktime_get();

	/* Check next schedule to expire considering both cpus */

	spin_lock(&cpuidle_lock);
	for_each_online_cpu(cpu) {
		t = per_cpu(cpu_state, cpu)->sched_wake_up;

		delta = ktime_to_us(ktime_sub(t, now));
		if ((delta < remaining_sleep_time) && (delta > 0)) {
			remaining_sleep_time = (u32)delta;
			if (next)
				(*next) = t;
			if (on_cpu)
				(*on_cpu) = cpu;
		}
	}
	spin_unlock(&cpuidle_lock);

	return remaining_sleep_time;
}

static bool is_last_cpu_running(void)
{
	smp_rmb();
	return atomic_read(&idle_cpus_counter) == num_online_cpus();
}

static int determine_sleep_state(u32 *sleep_time)
{
	int i;

	int cpu;
	int max_depth = ARRAY_SIZE(cstates);
	bool power_state_req;

	/* If first cpu to sleep, go to most shallow sleep state */
	if (!is_last_cpu_running())
		return cstates_1CPU[smp_processor_id()][per_cpu(cpu_state, smp_processor_id())->gov_cstate];

	/* If other CPU is going to WFI, but not yet there wait. */
	while (1) {
		if (ux500_pm_other_cpu_wfi())
			break;

		if (ux500_pm_gic_pending_interrupt())
			return -1;

		if (!is_last_cpu_running())
			return cstates_1CPU[smp_processor_id()][per_cpu(cpu_state, smp_processor_id())->gov_cstate];
	}

	power_state_req = power_state_active_is_enabled() ||
		prcmu_is_ac_wake_requested();

	(*sleep_time) = get_remaining_sleep_time(NULL, NULL);

	if ((*sleep_time) == UINT_MAX)
		return CI_WFI;
	/*
	 * Never go deeper than the governor recommends even though it might be
	 * possible from a scheduled wake up point of view
	 */
	for_each_online_cpu(cpu) {
		if (max_depth > per_cpu(cpu_state, cpu)->gov_cstate)
			max_depth = per_cpu(cpu_state, cpu)->gov_cstate;
	}

	for (i = max_depth; i > 0; i--) {

		if ((*sleep_time) <= cstates[i].threshold)
			continue;

		/* OK state */
		break;
	}

	return max(CI_WFI, i);
}

static int enter_sleep(struct cpuidle_device *dev,
		      struct cpuidle_driver *drv,
		      int index)
{
	struct cpuidle_state_usage *curr_usage = &dev->states_usage[index];
	struct cpuidle_state *curr = &drv->states[index];

	ktime_t time_enter, time_exit;
	ktime_t wake_up;

	int sleep_time = 0;
	s64 diff;
	int target;
	struct cpu_state *state;
	bool slept_well = false;
	int this_cpu = smp_processor_id();

	local_irq_disable();

	time_enter = ktime_get(); /* Time now */

	state = per_cpu(cpu_state, smp_processor_id());

	wake_up = ktime_add(time_enter, tick_nohz_get_sleep_length());

	spin_lock(&cpuidle_lock);

	/* Save scheduled wake up for this cpu */
	state->sched_wake_up = wake_up;

	/* Retreive the cstate that the governor recommends for this CPU */
	state->gov_cstate = (int) cpuidle_get_statedata(curr_usage);

	spin_unlock(&cpuidle_lock);

	atomic_inc(&idle_cpus_counter);

	/*
	 * Determine sleep state considering both CPUs and
	 * shared resources like e.g. VAPE
	 */
	target = determine_sleep_state(&sleep_time);

	if (target < 0)
		/* "target" will be last_state in the cpuidle framework */
		goto exit_fast;

	if (cstates[state->gov_cstate].ARM != ARM_ON)
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER,
				   &this_cpu);

	/* Only one CPU should master the sleeping sequence */
	if (cstates[target].ARM != ARM_ON) {
		if (atomic_inc_return(&master_counter) != 1)
		{
			atomic_dec(&master_counter);
			goto wait;
		}

		ux500_pm_gic_decouple();

		/*
		 * Check if sleep state has changed after GIC has been frozen
		 */
		if (target != determine_sleep_state(&sleep_time)) {
			atomic_dec(&master_counter);
			goto exit;
		}

		/* Copy GIC interrupt settings to PRCMU interrupt settings */
		ux500_pm_prcmu_copy_gic_settings();

		if ((ux500_pm_gic_pending_interrupt()) 
		 || (ux500_pm_prcmu_pending_interrupt())) {
			/* An interrupt found => abort */
			atomic_dec(&master_counter);
			goto exit;
		}

		/*
		 * No PRCMU interrupt was pending => continue the
		 * sleeping stages
		 */
		prcmu_set_power_state(cstates[target].pwrst,
				      cstates[target].UL_PLL,
				      /* Is actually the AP PLL */
				      cstates[target].UL_PLL);
		
		atomic_dec(&master_counter);
	
	}
wait:
	__asm__ __volatile__
		("dsb\n\t" "wfi\n\t" : : : "memory");

	slept_well = true;
	
exit:
	if (!slept_well)
		/* Recouple GIC with the interrupt bus */
		ux500_pm_gic_recouple();

	/* Use the ARM local timer for this cpu */
	if (cstates[state->gov_cstate].ARM != ARM_ON)
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT,
				   &this_cpu);
exit_fast:

	atomic_dec(&idle_cpus_counter);

	if (target < 0)
		target = CI_RUNNING;

	/* 16 minutes ahead */
	wake_up = ktime_add_us(time_enter,
			       1000000000);

	spin_lock(&cpuidle_lock);

	/* Remove wake up time i.e. set wake up far ahead */
	state->sched_wake_up = wake_up;
	spin_unlock(&cpuidle_lock);

	/* calculate idle duration */
	time_exit = ktime_get();
	diff = ktime_to_us(ktime_sub(time_exit, time_enter));
	if (diff > INT_MAX)
		diff = INT_MAX;

	dev->last_residency = diff;

	local_irq_enable();

	return target;
}

static int init_cstates(struct cpuidle_driver *drv)
{
	int i;
	struct cpuidle_state *ci_state;

	for (i = 0; i < ARRAY_SIZE(cstates); i++) {
		ci_state = &drv->states[i];

		ci_state->exit_latency = cstates[i].exit_latency;
		ci_state->target_residency = cstates[i].threshold;
		ci_state->flags = cstates[i].flags;
		ci_state->enter = enter_sleep;
		ci_state->power_usage = cstates[i].power_usage;
		snprintf(ci_state->name, CPUIDLE_NAME_LEN, "C%d", i);
		strncpy(ci_state->desc, cstates[i].desc, CPUIDLE_DESC_LEN);
	}

	drv->state_count = ARRAY_SIZE(cstates);

	return cpuidle_register_driver(drv);
}

static int init_cpuidle_device(int cpu, struct cpu_state *state)
{
	struct cpuidle_device *dev;
	int i;

	dev = &state->dev;
	dev->cpu = cpu;

	for (i = 0; i < ARRAY_SIZE(cstates); i++) {
		struct cpuidle_state_usage *state_usage = &dev->states_usage[i];

		cpuidle_set_statedata(state_usage, (void *)i);
	}

	dev->state_count = ARRAY_SIZE(cstates);

	return cpuidle_register_device(dev);
}


struct cpuidle_driver cpuidle_drv = {
	.name = "cpuidle_driver",
	.owner = THIS_MODULE,
};

static int __init cpuidle_driver_init(void)
{
	int res = -ENODEV;
	int cpu;

	if (ux500_is_svp())
		goto out;

	/* Configure wake up reasons */
	prcmu_enable_wakeups(PRCMU_WAKEUP(ARM) | PRCMU_WAKEUP(RTC) |
			     PRCMU_WAKEUP(ABB));

	cpuidle_drv.safe_state_index = -1;

	for_each_possible_cpu(cpu)
		per_cpu(cpu_state, cpu) = kzalloc(sizeof(struct cpu_state),
						  GFP_KERNEL);

	res = init_cstates(&cpuidle_drv);
	if (res)
		goto out;

	for_each_possible_cpu(cpu) {
		init_cpuidle_device(cpu, per_cpu(cpu_state, cpu));
		if (res)
			goto out;
		pr_info("cpuidle: initiated%d \n", cpu);
	}

	return 0;
out:
	pr_err("cpuidle: initialization failed.\n");
	return res;
}

static void __exit cpuidle_driver_exit(void)
{
	int cpu;
	struct cpuidle_device *dev;

	for_each_possible_cpu(cpu) {
		dev = &per_cpu(cpu_state, cpu)->dev;
		cpuidle_unregister_device(dev);
	}

	for_each_possible_cpu(cpu)
		kfree(per_cpu(cpu_state, cpu));

	cpuidle_unregister_driver(&cpuidle_drv);
}

late_initcall(cpuidle_driver_init);
module_exit(cpuidle_driver_exit);

MODULE_DESCRIPTION("U8500 cpuidle driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rickard Andersson <rickard.andersson@stericsson.com>");

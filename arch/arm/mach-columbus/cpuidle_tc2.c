/*
 * TC2 CPU idle driver.
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/cpu_pm.h>
#include <linux/clockchips.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/tick.h>
#include <trace/events/power.h>
#include <asm/idmap.h>
#include <asm/pgalloc.h>
#include <asm/proc-fns.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#define TC2_STATE_C1 0
#define TC2_STATE_C2 1
#define TC2_STATE_C3 2
#define TC2_STATE_C4 3
#define TC2_MAX_STATES (TC2_STATE_C4 + 1)

/* four bits reserved to cpu state */
#define CPU_ST(x) (x & 0xf)
#define CLUSTER_ST(x) (x >> 4 & 0xf)
#define CSS_STATE(cpuid, clusterid, cpustate, clusterstate) \
	((clusterstate) << 12 | (cpustate) << 8 | (clusterid) << 4 | (cpuid))

struct tc2_processor_cx {
	u8 valid;
	u8 type;
	u8 cr;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 threshold;
	u32 flags;
};

struct tc2_processor_cx tc2_power_states[TC2_MAX_STATES] = {
	{
		.valid = 1,
		.type = TC2_STATE_C1,
		.sleep_latency = 2,
		.wakeup_latency = 2,
		.threshold = 1,
		.cr = 0x1, /* wfi */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = TC2_STATE_C2,
		.sleep_latency = 30,
		.wakeup_latency = 30,
		.threshold = 50,
		.cr = 0x3, /* CPU shut down cluster on - clocked */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = TC2_STATE_C3,
		.sleep_latency = 50,
		.wakeup_latency = 50,
		.threshold = 300,
		.cr = 0x23, /* shut down cluster off - L2 retention */
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_COUPLED,
	},
	{
		.valid = 1,
		.type = TC2_STATE_C4,
		.sleep_latency = 1500,
		.wakeup_latency = 1800,
		.threshold = 4000,
		.cr = 0x33, /* complete cluster shutdown */
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_COUPLED,
	},
};

struct cpuidle_driver tc2_idle_driver = {
	.name = "tc2_idle",
	.owner = THIS_MODULE,
	.state_count = TC2_MAX_STATES,
	.safe_state_index = 0
};

static DEFINE_PER_CPU(struct cpuidle_device, tc2_idle_dev);

static DECLARE_BITMAP(cpu_pm_bits, CONFIG_NR_CPUS) __read_mostly
	= CPU_BITS_NONE;
static const struct cpumask *const cpu_pm_mask = to_cpumask(cpu_pm_bits);

static DEFINE_PER_CPU(u32, cur_residency);
static DEFINE_PER_CPU(s64, next_event);

static inline int event_shutdown(void)
{
	int idx;
	ktime_t exp, now = ktime_get();
	u32 exp_res;

	for_each_online_cpu(idx) {
		exp.tv64 = per_cpu(next_event, idx) - now.tv64;
		exp_res = (u32) ktime_to_us(exp);

		if ((exp.tv64 < 0) || (per_cpu(cur_residency, idx) > exp_res))
			return 1;
	}

	return 0;
}

typedef void (*phys_reset_t)(unsigned long);
extern void disable_clean_inv_dcache(void);

static inline int smc_down(unsigned int state, unsigned int affinity)
{
	return 1;
}
/*
 * Power down SCP command passed as an argument from cpu_suspend
 */
int tc2_finisher(unsigned long arg)
{
	smc_down(0x1, 0x0);
	return 1;
}

static atomic_t abort_barrier;
static bool abort_flag;
static inline int scc_pending_wakeups(void)
{
	return 0;
}

/*
 * Power down SCP command passed as an argument from cpu_suspend
 */
int tc2_coupled_finisher(unsigned long arg)
{
	if (scc_pending_wakeups() && event_shutdown())
		abort_flag = 1;

	cpuidle_coupled_parallel_barrier((struct cpuidle_device *)arg,
					&abort_barrier);
	if (!abort_flag)
		smc_down(0x1, 0x1);

	abort_flag = 0;
	return 1;
}

/*
 * tc2_enter_coupled - Programs CPU to enter the specified state
 * @dev: cpuidle device
 * @drv: The target state to be programmed
 * @idx: state index
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int tc2_enter_coupled(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int idx)
{
	struct tick_device *tdev = tick_get_device(dev->cpu);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	int i, ret;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	for_each_cpu(i, topology_core_cpumask(dev->cpu))
		if (!cpumask_test_cpu(i, to_cpumask(cpu_pm_bits)))
			goto shallow_out;

	cpu_pm_enter();

	per_cpu(cur_residency, dev->cpu) = drv->states[idx].target_residency;

	per_cpu(next_event, dev->cpu) = tdev->evtdev->next_event.tv64;

	if (!dev->cpu)
		cpu_cluster_pm_enter();

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);

	ret = cpu_suspend((unsigned long) dev, tc2_coupled_finisher);

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);

	if (ret)
		goto deep_out;

	if (!dev->cpu)
		cpu_cluster_pm_exit();

	cpu_pm_exit();

deep_out:
	per_cpu(cur_residency, dev->cpu) = 0;
	per_cpu(next_event, dev->cpu) = 0;

shallow_out:
	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	dev->last_residency = ts_idle.tv_nsec / NSEC_PER_USEC +
					ts_idle.tv_sec * USEC_PER_SEC;
	return idx;
}
/*
 * tc2_enter_idle - Programs CPU to enter the specified state
 * @dev: cpuidle device
 * @drv: The target state to be programmed
 * @idx: state index
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int tc2_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int idx)
{
	struct tc2_processor_cx *cx =
		cpuidle_get_statedata(&dev->states_usage[idx]);

	struct tick_device *tdev = tick_get_device(dev->cpu);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	int ret;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);
	
	if (cx->type == TC2_STATE_C1 ||
		!cpu_isset(smp_processor_id(), *to_cpumask(cpu_pm_bits))) {

		wfi();
		goto out;
	}

	cpu_pm_enter();

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);

	ret = cpu_suspend(0, tc2_finisher);

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);

	if (ret)
		goto out;

	cpu_pm_exit();

out:

	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	dev->last_residency = ts_idle.tv_nsec / NSEC_PER_USEC +
					ts_idle.tv_sec * USEC_PER_SEC;
	local_irq_enable();
	local_fiq_enable();
	return idx;
}

static int idle_mask_show(struct seq_file *f, void *p)
{
	char buf[256];
	bitmap_scnlistprintf(buf, 256, cpu_pm_bits, CONFIG_NR_CPUS);

	seq_printf(f, "%s\n", buf);

	return 0;
}

static int idle_mask_open(struct inode *inode, struct file *file)
{
	return single_open(file, idle_mask_show, inode->i_private);
}

static const struct file_operations cpuidle_fops = {
	.open		= idle_mask_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int idle_debug_set(void *data, u64 val)
{
	int i, cpus = num_possible_cpus();

	if ((val > cpus || val < 0) && val != 0xff) {
		pr_warning("Wrong parameter passed\n");
		return -EINVAL;
	}

	if (val == 0xff) {
		cpumask_clear(to_cpumask(cpu_pm_bits));
		return 0;
	}

	if (val == cpus) {
		cpumask_copy(to_cpumask(cpu_pm_bits), cpu_possible_mask);
		return 0;
	}

	for (i = 0; i < cpus; i++)
		if (val == i)
			cpumask_set_cpu(i, to_cpumask(cpu_pm_bits));

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(idle_debug_fops, NULL, idle_debug_set, "%llu\n");

/*
 * tc2_idle_init
 *
 * Registers the TC2 specific cpuidle driver with the cpuidle
 * framework with the valid set of states.
 */
int __init tc2_idle_init(void)
{
	struct tc2_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;
	int i, cpu_id, count;
	struct dentry *idle_debug, *file_debug;

	cpuidle_register_driver(&tc2_idle_driver);

	for (count = 0, i = TC2_STATE_C1; i < TC2_MAX_STATES; i++) {
		cx = &tc2_power_states[i];
		state = &tc2_idle_driver.states[count];

		if (!cx->valid)
			continue;

		state->exit_latency = cx->sleep_latency +
						cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		if (state->flags & CPUIDLE_FLAG_COUPLED)
			state->enter = tc2_enter_coupled;
		else
			state->enter = tc2_enter_idle;
		sprintf(state->name, "C%d", count + 1);
		count++;
	}

	for_each_cpu(cpu_id, cpu_online_mask) {
		pr_err("CPUidle for CPU%d registered\n", cpu_id);
		dev = &per_cpu(tc2_idle_dev, cpu_id);
		dev->cpu = cpu_id;
		dev->safe_state_index = 1;

		cpumask_copy(&dev->coupled_cpus,
				topology_core_cpumask(cpu_id));
		dev->state_count = TC2_MAX_STATES;

		for (count = 0, i = TC2_STATE_C1;
				i < TC2_MAX_STATES; i++) {
			cx = &tc2_power_states[i];

			if (!cx->valid)
				continue;
			cpuidle_set_statedata(&dev->states_usage[count], cx);
			count++;
		}

		if (cpuidle_register_device(dev)) {
			printk(KERN_ERR "%s: Cpuidle register device failed\n",
			       __func__);
			return -EIO;
		}
	}

	idle_debug = debugfs_create_dir("idle_debug", NULL);

	if (IS_ERR_OR_NULL(idle_debug)) {
		printk(KERN_INFO "Error in creating idle debugfs directory\n");
		return 0;
	}

	file_debug = debugfs_create_file("enable_idle", S_IRUGO | S_IWGRP,
				   idle_debug, NULL, &idle_debug_fops);

	if (IS_ERR_OR_NULL(file_debug)) {
		printk(KERN_INFO "Error in creating enable_idle file\n");
		return 0;
	}

	file_debug = debugfs_create_file("enable_mask", S_IRUGO | S_IWGRP,
					idle_debug, NULL, &cpuidle_fops);

	if (IS_ERR_OR_NULL(file_debug))
		printk(KERN_INFO "Error in creating enable_mask file\n");

	return 0;
}
device_initcall(tc2_idle_init);

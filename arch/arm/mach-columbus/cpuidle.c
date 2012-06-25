/*
 * Columbus CPU idle driver.
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
#include <asm/uaccess.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#define COLUMBUS_STATE_C1 0
#define COLUMBUS_STATE_C2 1
#define COLUMBUS_STATE_C3 2
#define COLUMBUS_STATE_C4 3
#define COLUMBUS_MAX_STATES (COLUMBUS_STATE_C4 + 1)

/* four bits reserved to cpu state */
#define CPU_ST(x) (x & 0xf)
#define CLUSTER_ST(x) (x >> 4 & 0xf)
#define CSS_STATE(cpuid, clusterid, cpustate, clusterstate) \
	((clusterstate) << 12 | (cpustate) << 8 | (clusterid) << 4 | (cpuid))

struct columbus_processor_cx {
	u8 valid;
	u8 type;
	u8 cr;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 threshold;
	u32 flags;
};

struct columbus_processor_cx columbus_power_states[COLUMBUS_MAX_STATES] = {
	{
		.valid = 1,
		.type = COLUMBUS_STATE_C1,
		.sleep_latency = 2,
		.wakeup_latency = 2,
		.threshold = 1,
		.cr = 0x1, /* wfi */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = COLUMBUS_STATE_C2,
		.sleep_latency = 30,
		.wakeup_latency = 30,
		.threshold = 50,
		.cr = 0x3, /* CPU shut down cluster on - clocked */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = COLUMBUS_STATE_C3,
		.sleep_latency = 50,
		.wakeup_latency = 50,
		.threshold = 300,
		.cr = 0x23, /* shut down cluster off - L2 retention */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = COLUMBUS_STATE_C4,
		.sleep_latency = 1500,
		.wakeup_latency = 1800,
		.threshold = 4000,
		.cr = 0x33, /* complete cluster shutdown */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
};

struct cpuidle_driver columbus_idle_driver = {
	.name = "columbus_idle",
	.owner = THIS_MODULE,
	.state_count = COLUMBUS_MAX_STATES,
	.safe_state_index = 0
};

static DEFINE_SPINLOCK(cluster_lock);
static DEFINE_PER_CPU(struct cpuidle_device, columbus_idle_dev);

static DECLARE_BITMAP(cpu_pm_bits, CONFIG_NR_CPUS) __read_mostly
	= CPU_BITS_NONE;
const struct cpumask *const cpu_pm_mask = to_cpumask(cpu_pm_bits);

static DEFINE_PER_CPU(u32, cur_residency);
static DEFINE_PER_CPU(s64, next_event);

static cpumask_t idle_mask = { CPU_BITS_NONE };

extern int _smc_down(u32, u32, u32, u32);

static inline int shutdown_smc(u32 cstate, u32 wakeup_h, u32 wakeup_l)
{
	unsigned int value = 0x1 << 16 | (cstate & 0xFFFF);
	/* TODO: Add a new SMC call to setup wake-up times once defined */
	return _smc_down(0, value, virt_to_phys(cpu_resume), 0);
}

static inline int arch_wakeup_ts(u64 *ts)
{
	u32 cvall, cvalh;
	s32 val;

	asm volatile("mrc p15, 0, %0, c14, c2, 0" : "=r" (val));

	asm volatile("mrrc p15, 0, %0, %1, c14" : "=r" (cvall), "=r" (cvalh));

	if (val < 0)
		return 1;

	*ts = (val + (u64) (((u64) cvalh << 32) | cvall));

	return 0;
}

static inline int event_shutdown(void)
{
	int idx;
	ktime_t exp, now = ktime_get();
	u32 exp_res;

	if (!(cpumask_weight(&idle_mask) == num_online_cpus()))
		return 1;

	for_each_online_cpu(idx) {
		exp.tv64 = per_cpu(next_event, idx) - now.tv64;
		exp_res = (u32) ktime_to_us(exp);

		if ((exp.tv64 < 0) || (per_cpu(cur_residency, idx) > exp_res))
			return 1;
	}

	return 0;
}

union wakeup {
	struct {
		u32 wakeup_l;
		u32 wakeup_h;
	} ts32;
	u64 ts64;
};

/*
 * Power down SCP command passed as an argument from cpu_suspend
 */
int columbus_finisher(unsigned long arg)
{
	int ret;
	union wakeup w;

	if (arch_wakeup_ts(&w.ts64))
		return 1;

	ret = shutdown_smc(0, arg, w.ts32.wakeup_h, w.ts32.wakeup_l);
	return ret;
}

/*
 * columbus_enter_idle - Programs CPU to enter the specified state
 * @dev: cpuidle device
 * @drv: The target state to be programmed
 * @idx: state index
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int columbus_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int idx)
{
	struct columbus_processor_cx *cx =
		cpuidle_get_statedata(&dev->states_usage[idx]);

	struct tick_device *tdev = tick_get_device(dev->cpu);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	int ret, cpu_id, cluster_state, cluster_down = 0;
	u32 mpidr, command;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	if (cx->type == COLUMBUS_STATE_C1 ||
		!cpu_isset(smp_processor_id(), *to_cpumask(cpu_pm_bits))) {

		/* Used to keep track of the total time in idle */
		asm volatile("wfi");
		getnstimeofday(&ts_postidle);
		ts_idle = timespec_sub(ts_postidle, ts_preidle);

		local_irq_enable();
		local_fiq_enable();

		dev->last_residency = ts_idle.tv_nsec / NSEC_PER_USEC +
						ts_idle.tv_sec * USEC_PER_SEC;
		return 0;
	}

	spin_lock(&cluster_lock);

	cpu_pm_enter();

	if (cx->type > COLUMBUS_STATE_C2)
		__cpu_set(dev->cpu, &idle_mask);

	per_cpu(cur_residency, dev->cpu) = drv->states[idx].target_residency;

	per_cpu(next_event, dev->cpu) = tdev->evtdev->next_event.tv64;

	cluster_down = !event_shutdown();

	if (cluster_down) {
		cpu_cluster_pm_enter();
		for_each_cpu(cpu_id, cpu_online_mask) {
			if (cx->type > COLUMBUS_STATE_C2) {
				trace_power_start(POWER_CSTATE, cx->type + 2,
					cpu_id);
				trace_cpu_idle(cx->type + 2, cpu_id);
			}
		}
	}

	mpidr = cpu_logical_map(dev->cpu);
	cluster_state = cluster_down ? CLUSTER_ST(cx->cr) : 0;
	command = CSS_STATE(mpidr & 0xff,
				(mpidr & 0xff00) >> 8,
				CPU_ST(cx->cr),
				cluster_state);
	spin_unlock(&cluster_lock);

	ret = cpu_suspend(command, columbus_finisher);

	spin_lock(&cluster_lock);

	if (cluster_down) {
		for_each_cpu(cpu_id, cpu_online_mask) {
			if (cx->type > COLUMBUS_STATE_C2) {
				trace_power_end(cpu_id);
				trace_cpu_idle(PWR_EVENT_EXIT, cpu_id);
			}
		}
	}

	if (ret)
		goto out;

	if (cluster_down)
		cpu_cluster_pm_exit();

	cpu_pm_exit();

	/* Restore the per-cpu timer event */
	clockevents_program_event(tdev->evtdev, tdev->evtdev->next_event, 1);
out:
	per_cpu(cur_residency, dev->cpu) = 0;
	per_cpu(next_event, dev->cpu) = 0;

	if (cx->type > COLUMBUS_STATE_C2)
		__cpu_clear(dev->cpu, &idle_mask);

	cluster_down = 0;

	spin_unlock(&cluster_lock);

	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	dev->last_residency = ts_idle.tv_nsec / NSEC_PER_USEC +
					ts_idle.tv_sec * USEC_PER_SEC;
	local_irq_enable();
	return idx;
}

static int idle_mask_show(struct seq_file *f, void *p)
{
	char buf[32];
	bitmap_scnlistprintf(buf, 32, cpu_pm_bits, CONFIG_NR_CPUS);

	seq_printf(f, "%s\n", buf);

	return 0;
}

static int idle_mask_open(struct inode *inode, struct file *file)
{
	return single_open(file, idle_mask_show, inode->i_private);
}

static ssize_t idle_mask_write(struct file *file,
	       const char __user *user_buf, size_t count, loff_t *ppos)
{
	int i, ret, cpus = num_possible_cpus();
	char buf[1 + sizeof(unsigned int) * 8 + 1 + 1];
	unsigned int val;

	count = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (sysfs_streq(buf, "clear")) {
		cpumask_clear(to_cpumask(cpu_pm_bits));
		return count;
	}

	ret = kstrtouint(buf, 10, &val);

	if (ret) {
		pr_warning("Badly formatted string passed\n");
		return ret;
	}

	if (val > cpus || val < 0) {
		pr_warning("Wrong value passed\n");
		return -EINVAL;
	}

	if (val == cpus) {
		cpumask_copy(to_cpumask(cpu_pm_bits), cpu_possible_mask);
		return count;
	}

	for (i = 0; i < cpus; i++)
		if (val == i) {
			cpumask_set_cpu(i, to_cpumask(cpu_pm_bits));
			return count;
		}

	return -EINVAL;
}

static const struct file_operations cpuidle_fops = {
	.open		= idle_mask_open,
	.write		= idle_mask_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * columbus_idle_init
 *
 * Registers the Columbus specific cpuidle driver with the cpuidle
 * framework with the valid set of states.
 */
int __init columbus_idle_init(void)
{
	struct columbus_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;
	int i, cpu_id, count;
	struct dentry *idle_debug, *file_debug;

	cpuidle_register_driver(&columbus_idle_driver);

	for (count = 0, i = COLUMBUS_STATE_C1; i < COLUMBUS_MAX_STATES; i++) {
		cx = &columbus_power_states[i];
		state = &columbus_idle_driver.states[count];

		if (!cx->valid)
			continue;

		state->exit_latency = cx->sleep_latency +
						cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		state->enter = columbus_enter_idle;
		sprintf(state->name, "C%d", count + 1);
		count++;
	}

	for_each_cpu(cpu_id, cpu_online_mask) {
		pr_err("CPUidle for CPU%d registered\n", cpu_id);
		dev = &per_cpu(columbus_idle_dev, cpu_id);
		dev->cpu = cpu_id;

		dev->state_count = COLUMBUS_MAX_STATES;

		for (count = 0, i = COLUMBUS_STATE_C1;
				i < COLUMBUS_MAX_STATES; i++) {
			cx = &columbus_power_states[i];

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

	file_debug = debugfs_create_file("idle_enable", S_IRUGO | S_IWGRP,
					idle_debug, NULL, &cpuidle_fops);

	if (IS_ERR_OR_NULL(file_debug))
		printk(KERN_INFO "Error in creating enable_mask file\n");

	return 0;
}
device_initcall(columbus_idle_init);

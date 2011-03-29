/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/clockchips.h>
#include <asm/proc-fns.h>
#include <asm/appf_platform_api.h>
#include <asm/hardware/elba_scpc.h>
#include <asm/unified.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>

#include <plat/powerdomain.h>
#include <mach/system.h>
#include <mach/core.h>

#define VEXPRESS_MAX_STATES 7
#define VEXPRESS_STATE_C1 0
#define VEXPRESS_STATE_C2 1
#define VEXPRESS_STATE_C3 2
#define VEXPRESS_STATE_C4 3

/* four bits reserved to cpu state */
#define CPU_ST(x) (x & 0xf)
#define CLUSTER_ST(x) (x >> 4 & 0xf)

struct vexpress_processor_cx {
	u8 valid;
	u8 type;
	u8 cr;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 threshold;
	u32 flags;
};
struct vexpress_processor_cx vexpress_power_states[VEXPRESS_MAX_STATES] = {
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C1,
		.sleep_latency = 2,
		.wakeup_latency = 2,
		.threshold = 5,
		.cr = 0x1, /* wfi */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C2,
		.sleep_latency = 30,
		.wakeup_latency = 30,
		.threshold = 50,
		.cr = 0x2, /* CPU shut down cluster on - clocked */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C3,
		.sleep_latency = 50,
		.wakeup_latency = 50,
		.threshold = 300,
		.cr = 0x22, /* shut down cluster off - L2 retention */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C4,
		.sleep_latency = 1500,
		.wakeup_latency = 1800,
		.threshold = 4000,
		.cr = 0x32, /* complete cluster shutdown */
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
};

DEFINE_PER_CPU(u8, current_cx_state);
DEFINE_PER_CPU(u32, cycle_count);

struct cpuidle_driver vexpress_idle_driver = {
	.name = "vexpress_idle",
	.owner = THIS_MODULE,
};

DEFINE_PER_CPU(struct cpuidle_device, vexpress_idle_dev);

/**
 * vexpress_enter_idle - Programs CPU to enter the specified state
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int vexpress_enter_idle(struct cpuidle_device *dev, struct cpuidle_state *state)
{
	struct vexpress_processor_cx *cx = cpuidle_get_statedata(state);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	int cpu_id, cluster_state, cluster_down = 0;
	
	per_cpu(current_cx_state, dev->cpu) = cx->type;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	local_irq_disable();
	local_fiq_disable();

	if (cx->type > VEXPRESS_STATE_C1)
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);

	if (cx->type > VEXPRESS_STATE_C2) {
		for_each_cpu(cpu_id, cpu_online_mask) {
			if (per_cpu(current_cx_state, cpu_id)
					<= VEXPRESS_STATE_C2)
				break;

		}
		cluster_down = (cpu_id == num_online_cpus());
	}

	cluster_state = cluster_down ? CLUSTER_ST(cx->cr) : 0;
	power_down_cpu(CPU_ST(cx->cr), cluster_state, APPF_SAVE_ALL);


	if (cx->type > VEXPRESS_STATE_C1)
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);

	per_cpu(current_cx_state, dev->cpu) = VEXPRESS_STATE_C1;
	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	local_irq_enable();
	local_fiq_enable();

	return ts_idle.tv_nsec / NSEC_PER_USEC + ts_idle.tv_sec * USEC_PER_SEC;
}

/**
 * vexpress_idle_init
 *
 * Registers the Versatile Express specific cpuidle driver with the cpuidle
 * framework with the valid set of states.
 */
int __init vexpress_idle_init(void)
{
	int count = 0;
	struct vexpress_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;
	int i, cpu_id;

	cpuidle_register_driver(&vexpress_idle_driver);

	for_each_cpu(cpu_id, cpu_online_mask) {
		pr_err("CPUidle for CPU%d registered\n", cpu_id);
		dev = &per_cpu(vexpress_idle_dev, cpu_id);
		dev->cpu = cpu_id;
		count = 0;

		for (i = VEXPRESS_STATE_C1; i < VEXPRESS_MAX_STATES; i++) {
			cx = &vexpress_power_states[i];
			state = &dev->states[count];

			if (!cx->valid)
				continue;

			cpuidle_set_statedata(state, cx);
			state->exit_latency = cx->sleep_latency +
							cx->wakeup_latency;
			state->target_residency = cx->threshold;
			state->flags = cx->flags;
			state->enter = vexpress_enter_idle;
			if (cx->type == VEXPRESS_STATE_C1)
				dev->safe_state = state;
			sprintf(state->name, "C%d", count + 1);
			count++;
		}

		if (!count)
			return -EINVAL;
		dev->state_count = count;

		if (cpuidle_register_device(dev)) {
			printk(KERN_ERR "%s: Cpuidle register device failed\n",
			       __func__);
			return -EIO;
		}
		hip_set_auto_shutdown(cpu_id);
	}

	appf_patch_reset((void *)PAGE_OFFSET, (void *)V2M_SYS_FLAGSSET);
	writel(~0, MMIO_P2V(V2M_SYS_FLAGSCLR));
	writel(BSYM(virt_to_phys(arch_reset_handler())),
			MMIO_P2V(V2M_SYS_FLAGSSET));

	__cpuc_flush_dcache_area((void *)PAGE_OFFSET, PAGE_SIZE);
	outer_clean_range(__pa(PAGE_OFFSET), __pa(PAGE_OFFSET + PAGE_SIZE));
	return 0;
}
late_initcall(vexpress_idle_init);

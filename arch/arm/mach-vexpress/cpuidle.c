/*
 * linux/arch/arm/mach-vexpress/cpuidle.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>

#include <plat/powerdomain.h>

#define VEXPRESS_MAX_STATES 7
#define VEXPRESS_STATE_C1 0
#define VEXPRESS_STATE_C2 1
#define VEXPRESS_STATE_C3 2
#define VEXPRESS_STATE_C4 3
#define VEXPRESS_STATE_C5 4
#define VEXPRESS_STATE_C6 5
#define VEXPRESS_STATE_C7 6

struct vexpress_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 mpu_state;
	u32 core_state;
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
		.mpu_state = PWRDM_POWER_ON,
		.core_state = PWRDM_POWER_ON,
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C2,
		.sleep_latency = 10,
		.wakeup_latency = 10,
		.threshold = 30,
		.mpu_state = PWRDM_POWER_ON,
		.core_state = PWRDM_POWER_ON,
		.flags = CPUIDLE_FLAG_TIME_VALID,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C3,
		.sleep_latency = 50,
		.wakeup_latency = 50,
		.threshold = 300,
		.mpu_state = PWRDM_POWER_RET,
		.core_state = PWRDM_POWER_ON,
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_CHECK_BM,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C4,
		.sleep_latency = 1500,
		.wakeup_latency = 1800,
		.threshold = 4000,
		.mpu_state = PWRDM_POWER_OFF,
		.core_state = PWRDM_POWER_ON,
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_CHECK_BM,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C5,
		.sleep_latency = 2500,
		.wakeup_latency = 7500,
		.threshold = 12000,
		.mpu_state = PWRDM_POWER_RET,
		.core_state = PWRDM_POWER_RET,
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_CHECK_BM,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C6,
		.sleep_latency = 3000,
		.wakeup_latency = 8500,
		.threshold = 15000,
		.mpu_state = PWRDM_POWER_OFF,
		.core_state = PWRDM_POWER_RET,
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_CHECK_BM,
	},
	{
		.valid = 1,
		.type = VEXPRESS_STATE_C7,
		.sleep_latency = 10000,
		.wakeup_latency = 30000,
		.threshold = 300000,
		.mpu_state = PWRDM_POWER_OFF,
		.core_state = PWRDM_POWER_OFF,
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_CHECK_BM,
	},
};
struct vexpress_processor_cx current_cx_state;

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
	
	current_cx_state = *cx;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	local_irq_disable();
	local_fiq_disable();

	/* Execute ARM wfi  */
	// ---------- // 
	
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
	int i;

	cpuidle_register_driver(&vexpress_idle_driver);

	dev = &per_cpu(vexpress_idle_dev, smp_processor_id());

	for (i = VEXPRESS_STATE_C1; i < VEXPRESS_MAX_STATES; i++) {
		cx = &vexpress_power_states[i];
		state = &dev->states[count];

		if(!cx->valid)
			continue;

		cpuidle_set_statedata(state, cx);
		state->exit_latency = cx->sleep_latency + cx->wakeup_latency;
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

	return 0;
}
device_initcall(vexpress_idle_init);

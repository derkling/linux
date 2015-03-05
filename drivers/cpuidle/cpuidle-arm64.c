/*
 * ARM64 generic CPU idle driver.
 *
 * Copyright (C) 2014 ARM Ltd.
 * Author: Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "CPUidle arm64: " fmt

#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/cpu_pm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>

#include <asm/cpuidle.h>

#include "dt_idle_states.h"

/*
 * arm64_enter_idle_state - Programs CPU to enter the specified state
 *
 * dev: cpuidle device
 * drv: cpuidle driver
 * idx: state index
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int arm64_enter_idle_state(struct cpuidle_device *dev,
				  struct cpuidle_driver *drv, int idx)
{
	int ret;

	if (!idx) {
		cpu_do_idle();
		return idx;
	}

	ret = cpu_pm_enter();
	if (!ret) {
		/*
		 * Pass idle state index to cpu_suspend which in turn will
		 * call the CPU ops suspend protocol with idle index as a
		 * parameter.
		 */
		ret = cpu_suspend(idx);

		cpu_pm_exit();
	}

	return ret ? -1 : idx;
}

static void __init arm64_init_driver(struct cpuidle_driver *drv)
{
	drv->name = "arm64_idle";
	drv->owner = THIS_MODULE;
	/*
	 * State at index 0 is standby wfi and considered standard
	 * on all ARM platforms. If in some platforms simple wfi
	 * can't be used as "state 0", DT bindings must be implemented
	 * to work around this issue and allow installing a special
	 * handler for idle state index 0.
	 */
	drv->states[0].enter = arm64_enter_idle_state;
	drv->states[0].exit_latency = 1;
	drv->states[0].target_residency = 1;
	drv->states[0].power_usage = UINT_MAX;
	strncpy(drv->states[0].name, "WFI", CPUIDLE_NAME_LEN - 1);
	strncpy(drv->states[0].desc, "ARM64 WFI", CPUIDLE_DESC_LEN - 1);
}

static const struct of_device_id arm64_idle_state_match[] __initconst = {
	{ .compatible = "arm,idle-state",
	  .data = arm64_enter_idle_state },
	{ },
};

#define ARM64_CPUIDLE_MAX_DRIVERS 4

static int __init arm64_idle_init(void)
{
	int cpu, ret, cnt = 0;
	struct cpuidle_driver *drivers[ARM64_CPUIDLE_MAX_DRIVERS], *drv = NULL;
	cpumask_var_t tmpmask;

	if (!alloc_cpumask_var(&tmpmask, GFP_KERNEL))
		return -ENOMEM;

	cpumask_copy(tmpmask, cpu_possible_mask);

	while (!cpumask_empty(tmpmask)) {

		if (cnt == ARM64_CPUIDLE_MAX_DRIVERS) {
			pr_warn("max number of idle drivers reached\n");
			break;
		}

		if (!drv) {
			drv = kzalloc(sizeof(*drv), GFP_KERNEL);
			if (!drv) {
				ret = -ENOMEM;
				goto out_unregister;
			}

			drv->cpumask = kzalloc(cpumask_size(), GFP_KERNEL);
			if (!drv->cpumask) {
				ret = -ENOMEM;
				goto out_driver;
			}
		}

		/*
		 * Initialize driver default idle states and common variables
		 */
		arm64_init_driver(drv);

		cpumask_copy(drv->cpumask, tmpmask);

		dt_probe_idle_affinity(drv->cpumask);
		/*
		 * Remove the driver cpumask from the list of cpus
		 * that are still to be probed for idle states detection
		 */
		cpumask_andnot(tmpmask, tmpmask, drv->cpumask);

		/*
		 * Initialize idle states data, starting at index 1.
		 * These drivers are DT only, if no DT idle states are detected
		 * (ret == 0) let the driver initialization fail accordingly
		 * since there is no reason to initialize the idle driver if
		 * only wfi is supported.
		 */
		ret = dt_init_idle_driver(drv, arm64_idle_state_match, 1);
		if (ret <= 0) {
			/*
			 * Give other drivers a chance to init
			 * even if for this cpumask no idle
			 * states were detected
			 */
			if (!ret)
				continue;

			goto out_mask;
		}

		/*
		 * Call arch CPU operations in order to initialize
		 * idle states suspend back-end specific data
		 */
		for_each_cpu(cpu, drv->cpumask) {
			ret = cpu_init_idle(cpu);
			if (ret) {
				pr_err("CPU %d failed to init idle CPU ops\n",
				       cpu);
				goto out_mask;
			}
		}

		ret = cpuidle_register(drv, NULL);
		if (ret)
			goto out_mask;

		drivers[cnt++] = drv;
		drv = NULL;
	}

	free_cpumask_var(tmpmask);
	return 0;

out_mask:
	kfree(drv->cpumask);
out_driver:
	kfree(drv);
out_unregister:
	while (--cnt >= 0) {
		cpuidle_unregister(drivers[cnt]);
		kfree(drivers[cnt]->cpumask);
		kfree(drivers[cnt]);
	}

	free_cpumask_var(tmpmask);

	return ret;
}
device_initcall(arm64_idle_init);

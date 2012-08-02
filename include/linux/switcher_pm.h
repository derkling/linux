/*
 * Copyright (C) 2012 ARM.
 *
 * Author:
 *	Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_SWITCHER_PM_H
#define _LINUX_SWITCHER_PM_H

#include <linux/kernel.h>
#include <linux/notifier.h>

/*
 * When a SWITCHER goes to a low power state that turns off power to the SWITCHER's
 * power domain, the contents of some blocks (floating point coprocessors,
 * interrupt controllers, caches, timers) in the same power domain can
 * be lost.  The cpm_pm notifiers provide a method for platform idle, suspend,
 * and hotplug implementations to notify the drivers for these blocks that
 * they may be reset.
 *
 * All cpu_pm notifications must be called with interrupts disabled.
 *
 * The notifications are split into two classes: SWITCHER notifications and SWITCHER
 * cluster notifications.
 *
 * SWITCHER notifications apply to a single SWITCHER and must be called on the affected
 * SWITCHER.  They are used to save per-cpu context for affected blocks.
 *
 * SWITCHER cluster notifications apply to all SWITCHERs in a single power domain. They
 * are used to save any global context for affected blocks, and must be called
 * after all the SWITCHERs in the power domain have been notified of the low power
 * state.
 */

/*
 * Event codes passed as unsigned long val to notifier calls
 */
enum switcher_pm_event {
	/* A single cpu is entering a low power state */
	SWITCHER_PM_ENTER,

	/* A single cpu failed to enter a low power state */
	SWITCHER_PM_ENTER_FAILED,

	/* A single cpu is exiting a low power state */
	SWITCHER_PM_EXIT,

};

#ifdef CONFIG_SWITCHER_PM
int switcher_pm_register_notifier(struct notifier_block *nb);
int switcher_pm_unregister_notifier(struct notifier_block *nb);
int switcher_pm_enter(void);
int switcher_pm_exit(void);

#else

static inline int switcher_pm_register_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int switcher_pm_unregister_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int switcher_pm_enter(void)
{
	return 0;
}

static inline int switcher_pm_exit(void)
{
	return 0;
}

#endif
#endif

/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * Author:
 * 	Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>
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

#include <linux/kernel.h>
#include <linux/switcher_pm.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>

static DEFINE_RWLOCK(switcher_pm_notifier_lock);
static RAW_NOTIFIER_HEAD(switcher_pm_notifier_chain);

static int switcher_pm_notify(enum switcher_pm_event event, int nr_to_call,
							int *nr_calls)
{
	int ret;

	ret = __raw_notifier_call_chain(&switcher_pm_notifier_chain, event, NULL,
		nr_to_call, nr_calls);

	return notifier_to_errno(ret);
}

/**
 * switcher_pm_register_notifier - register a driver with switcher_pm
 * @nb: notifier block to register
 *
 * Add a driver to a list of drivers that are notified about
 * CPU and CPU cluster low power entry and exit.
 *
 * This function may sleep, and has the same return conditions as
 * raw_notifier_chain_register.
 */
int switcher_pm_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&switcher_pm_notifier_lock, flags);
	ret = raw_notifier_chain_register(&switcher_pm_notifier_chain, nb);
	write_unlock_irqrestore(&switcher_pm_notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(switcher_pm_register_notifier);

/**
 * switcher_pm_unregister_notifier - unregister a driver with switcher_pm
 * @nb: notifier block to be unregistered
 *
 * Remove a driver from the CPU PM notifier list.
 *
 * This function may sleep, and has the same return conditions as
 * raw_notifier_chain_unregister.
 */
int switcher_pm_unregister_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&switcher_pm_notifier_lock, flags);
	ret = raw_notifier_chain_unregister(&switcher_pm_notifier_chain, nb);
	write_unlock_irqrestore(&switcher_pm_notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(switcher_pm_unregister_notifier);

/**
 * switcher_pm_enter - Switching entry notifier
 *
 */
int switcher_pm_enter(void)
{
	int nr_calls;
	int ret = 0;

	read_lock(&switcher_pm_notifier_lock);
	ret = switcher_pm_notify(SWITCHER_PM_ENTER, -1, &nr_calls);
	if (ret)
		/*
		 * Inform listeners (nr_calls - 1) about failure of CPU PM
		 * PM entry who are notified earlier to prepare for it.
		 */
		switcher_pm_notify(SWITCHER_PM_ENTER_FAILED, nr_calls - 1, NULL);
	read_unlock(&switcher_pm_notifier_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(switcher_pm_enter);

/**
 * switcher_pm_exit - Switching exit notifier
 *
 */
int switcher_pm_exit(void)
{
	int ret;

	read_lock(&switcher_pm_notifier_lock);
	ret = switcher_pm_notify(SWITCHER_PM_EXIT, -1, NULL);
	read_unlock(&switcher_pm_notifier_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(switcher_pm_exit);

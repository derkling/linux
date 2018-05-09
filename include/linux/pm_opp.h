/*
 * Generic OPP Interface
 *
 * Copyright (C) 2009-2010 Texas Instruments Incorporated.
 *	Nishanth Menon
 *	Romit Dasgupta
 *	Kevin Hilman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_OPP_H__
#define __LINUX_OPP_H__

#include <linux/err.h>
#include <linux/notifier.h>

struct dev_pm_opp;
struct device;
//struct opp_table;

enum dev_pm_opp_event {
	OPP_EVENT_ADD, OPP_EVENT_REMOVE, OPP_EVENT_ENABLE, OPP_EVENT_DISABLE,
};

enum opp_table_access {
	OPP_TABLE_ACCESS_UNKNOWN = 0,
	OPP_TABLE_ACCESS_EXCLUSIVE = 1,
	OPP_TABLE_ACCESS_SHARED = 2,
};

/**
 * struct opp_table - Device opp structure
 * @node:	table node - contains the devices with OPPs that
 *		have been registered. Nodes once added are not modified in this
 *		table.
 *		RCU usage: nodes are not modified in the table of opp_table,
 *		however addition is possible and is secured by opp_table_lock
 * @srcu_head:	notifier head to notify the OPP availability changes.
 * @rcu_head:	RCU callback head used for deferred freeing
 * @dev_list:	list of devices that share these OPPs
 * @opp_list:	table of opps
 * @np:		struct device_node pointer for opp's DT node.
 * @clock_latency_ns_max: Max clock latency in nanoseconds.
 * @shared_opp: OPP is shared between multiple devices.
 * @suspend_opp: Pointer to OPP to be used during device suspend.
 * @supported_hw: Array of version number to support.
 * @supported_hw_count: Number of elements in supported_hw array.
 * @prop_name: A name to postfix to many DT properties, while parsing them.
 * @clk: Device's clock handle
 * @regulator: Supply regulator
 * @dentry:	debugfs dentry pointer of the real device directory (not links).
 * @dentry_name: Name of the real dentry.
 *
 * @voltage_tolerance_v1: In percentage, for v1 bindings only.
 *
 * This is an internal data structure maintaining the link to opps attached to
 * a device. This structure is not meant to be shared to users as it is
 * meant for book keeping and private to OPP library.
 *
 * Because the opp structures can be used from both rcu and srcu readers, we
 * need to wait for the grace period of both of them before freeing any
 * resources. And so we have used kfree_rcu() from within call_srcu() handlers.
 */
struct opp_table {
	struct list_head node;

	struct srcu_notifier_head srcu_head;
	struct rcu_head rcu_head;
	struct list_head dev_list;
	struct list_head opp_list;

	struct device_node *np;
	unsigned long clock_latency_ns_max;

	/* For backward compatibility with v1 bindings */
	unsigned int voltage_tolerance_v1;

	enum opp_table_access shared_opp;
	struct dev_pm_opp *suspend_opp;

	unsigned int cycle_thief_id;
	unsigned int *supported_hw;
	unsigned int supported_hw_count;
	const char *prop_name;
	struct clk *clk;
	struct regulator *regulator;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
	char dentry_name[NAME_MAX];
#endif
};

#if defined(CONFIG_PM_OPP)

unsigned long dev_pm_opp_get_voltage(struct dev_pm_opp *opp);

unsigned long dev_pm_opp_get_freq(struct dev_pm_opp *opp);

bool dev_pm_opp_is_turbo(struct dev_pm_opp *opp);

int dev_pm_opp_get_opp_count(struct device *dev);
unsigned long dev_pm_opp_get_max_clock_latency(struct device *dev);
unsigned long dev_pm_opp_get_max_volt_latency(struct device *dev);
unsigned long dev_pm_opp_get_max_transition_latency(struct device *dev);
struct dev_pm_opp *dev_pm_opp_get_suspend_opp(struct device *dev);

struct dev_pm_opp *dev_pm_opp_find_freq_exact(struct device *dev,
					      unsigned long freq,
					      bool available);

struct dev_pm_opp *dev_pm_opp_find_freq_floor(struct device *dev,
					      unsigned long *freq);

struct dev_pm_opp *dev_pm_opp_find_freq_ceil(struct device *dev,
					     unsigned long *freq);

int dev_pm_opp_add(struct device *dev, unsigned long freq,
		   unsigned long u_volt);
void dev_pm_opp_remove(struct device *dev, unsigned long freq);

int dev_pm_opp_enable(struct device *dev, unsigned long freq);

int dev_pm_opp_disable(struct device *dev, unsigned long freq);

struct srcu_notifier_head *dev_pm_opp_get_notifier(struct device *dev);
int dev_pm_opp_set_supported_hw(struct device *dev, const u32 *versions,
				unsigned int count);
void dev_pm_opp_put_supported_hw(struct device *dev);
int dev_pm_opp_set_prop_name(struct device *dev, const char *name);
void dev_pm_opp_put_prop_name(struct device *dev);
struct opp_table *dev_pm_opp_set_regulator(struct device *dev, const char *name);
void dev_pm_opp_put_regulator(struct opp_table *opp_table);
int dev_pm_opp_set_rate(struct device *dev, unsigned long target_freq);
int dev_pm_opp_set_sharing_cpus(struct device *cpu_dev, const struct cpumask *cpumask);
int dev_pm_opp_get_sharing_cpus(struct device *cpu_dev, struct cpumask *cpumask);
void dev_pm_opp_remove_table(struct device *dev);
void dev_pm_opp_cpumask_remove_table(const struct cpumask *cpumask);
#else
static inline unsigned long dev_pm_opp_get_voltage(struct dev_pm_opp *opp)
{
	return 0;
}

static inline unsigned long dev_pm_opp_get_freq(struct dev_pm_opp *opp)
{
	return 0;
}

static inline bool dev_pm_opp_is_turbo(struct dev_pm_opp *opp)
{
	return false;
}

static inline int dev_pm_opp_get_opp_count(struct device *dev)
{
	return 0;
}

static inline unsigned long dev_pm_opp_get_max_clock_latency(struct device *dev)
{
	return 0;
}

static inline unsigned long dev_pm_opp_get_max_volt_latency(struct device *dev)
{
	return 0;
}

static inline unsigned long dev_pm_opp_get_max_transition_latency(struct device *dev)
{
	return 0;
}

static inline struct dev_pm_opp *dev_pm_opp_get_suspend_opp(struct device *dev)
{
	return NULL;
}

static inline struct dev_pm_opp *dev_pm_opp_find_freq_exact(struct device *dev,
					unsigned long freq, bool available)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline struct dev_pm_opp *dev_pm_opp_find_freq_floor(struct device *dev,
					unsigned long *freq)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline struct dev_pm_opp *dev_pm_opp_find_freq_ceil(struct device *dev,
					unsigned long *freq)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline int dev_pm_opp_add(struct device *dev, unsigned long freq,
					unsigned long u_volt)
{
	return -ENOTSUPP;
}

static inline void dev_pm_opp_remove(struct device *dev, unsigned long freq)
{
}

static inline int dev_pm_opp_enable(struct device *dev, unsigned long freq)
{
	return 0;
}

static inline int dev_pm_opp_disable(struct device *dev, unsigned long freq)
{
	return 0;
}

static inline struct srcu_notifier_head *dev_pm_opp_get_notifier(
							struct device *dev)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline int dev_pm_opp_set_supported_hw(struct device *dev,
					      const u32 *versions,
					      unsigned int count)
{
	return -ENOTSUPP;
}

static inline void dev_pm_opp_put_supported_hw(struct device *dev) {}

static inline int dev_pm_opp_set_prop_name(struct device *dev, const char *name)
{
	return -ENOTSUPP;
}

static inline void dev_pm_opp_put_prop_name(struct device *dev) {}

static inline struct opp_table *dev_pm_opp_set_regulator(struct device *dev, const char *name)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void dev_pm_opp_put_regulator(struct opp_table *opp_table) {}

static inline int dev_pm_opp_set_rate(struct device *dev, unsigned long target_freq)
{
	return -ENOTSUPP;
}

static inline int dev_pm_opp_set_sharing_cpus(struct device *cpu_dev, const struct cpumask *cpumask)
{
	return -ENOTSUPP;
}

static inline int dev_pm_opp_get_sharing_cpus(struct device *cpu_dev, struct cpumask *cpumask)
{
	return -EINVAL;
}

static inline void dev_pm_opp_remove_table(struct device *dev)
{
}

static inline void dev_pm_opp_cpumask_remove_table(const struct cpumask *cpumask)
{
}

#endif		/* CONFIG_PM_OPP */

#if defined(CONFIG_PM_OPP) && defined(CONFIG_OF)
int dev_pm_opp_of_add_table(struct device *dev);
void dev_pm_opp_of_remove_table(struct device *dev);
int dev_pm_opp_of_cpumask_add_table(const struct cpumask *cpumask);
void dev_pm_opp_of_cpumask_remove_table(const struct cpumask *cpumask);
int dev_pm_opp_of_get_sharing_cpus(struct device *cpu_dev, struct cpumask *cpumask);
#else
static inline int dev_pm_opp_of_add_table(struct device *dev)
{
	return -ENOTSUPP;
}

static inline void dev_pm_opp_of_remove_table(struct device *dev)
{
}

static inline int dev_pm_opp_of_cpumask_add_table(const struct cpumask *cpumask)
{
	return -ENOTSUPP;
}

static inline void dev_pm_opp_of_cpumask_remove_table(const struct cpumask *cpumask)
{
}

static inline int dev_pm_opp_of_get_sharing_cpus(struct device *cpu_dev, struct cpumask *cpumask)
{
	return -ENOTSUPP;
}
#endif

#endif		/* __LINUX_OPP_H__ */

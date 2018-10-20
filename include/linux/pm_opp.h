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
#include <linux/kref.h>
#include <linux/limits.h>
#include <linux/notifier.h>

struct clk;
struct regulator;
struct dev_pm_opp;
struct device;

enum opp_table_access {
	OPP_TABLE_ACCESS_UNKNOWN = 0,
	OPP_TABLE_ACCESS_EXCLUSIVE = 1,
	OPP_TABLE_ACCESS_SHARED = 2,
};

/**
 * struct dev_pm_opp_supply - Power supply voltage/current values
 * @u_volt:	Target voltage in microvolts corresponding to this OPP
 * @u_volt_min:	Minimum voltage in microvolts corresponding to this OPP
 * @u_volt_max:	Maximum voltage in microvolts corresponding to this OPP
 * @u_amp:	Maximum current drawn by the device in microamperes
 *
 * This structure stores the voltage/current values for a single power supply.
 */
struct dev_pm_opp_supply {
	unsigned long u_volt;
	unsigned long u_volt_min;
	unsigned long u_volt_max;
	unsigned long u_amp;
};

/**
 * struct dev_pm_opp_info - OPP freq/voltage/current values
 * @rate:	Target clk rate in hz
 * @supplies:	Array of voltage/current values for all power supplies
 *
 * This structure stores the freq/voltage/current values for a single OPP.
 */
struct dev_pm_opp_info {
	unsigned long rate;
	struct dev_pm_opp_supply *supplies;
};


/**
 * struct dev_pm_set_opp_data - Set OPP data
 * @old_opp:	Old OPP info
 * @new_opp:	New OPP info
 * @regulators:	Array of regulator pointers
 * @regulator_count: Number of regulators
 * @clk:	Pointer to clk
 * @dev:	Pointer to the struct device
 *
 * This structure contains all information required for setting an OPP.
 */
struct dev_pm_set_opp_data {
	struct dev_pm_opp_info old_opp;
	struct dev_pm_opp_info new_opp;

	struct regulator **regulators;
	unsigned int regulator_count;
	struct clk *clk;
	struct device *dev;
};

/**
 * struct opp_table - Device opp structure
 * @node:	table node - contains the devices with OPPs that
 *		have been registered. Nodes once added are not modified in this
 *		table.
 * @head:	notifier head to notify the OPP availability changes.
 * @dev_list:	list of devices that share these OPPs
 * @opp_list:	table of opps
 * @kref:	for reference count of the table.
 * @lock:	mutex protecting the opp_list.
 * @np:		struct device_node pointer for opp's DT node.
 * @clock_latency_ns_max: Max clock latency in nanoseconds.
 * @shared_opp: OPP is shared between multiple devices.
 * @suspend_opp: Pointer to OPP to be used during device suspend.
 * @supported_hw: Array of version number to support.
 * @supported_hw_count: Number of elements in supported_hw array.
 * @prop_name: A name to postfix to many DT properties, while parsing them.
 * @clk: Device's clock handle
 * @regulators: Supply regulators
 * @regulator_count: Number of power supply regulators
 * @genpd_performance_state: Device's power domain support performance state.
 * @set_opp: Platform specific set_opp callback
 * @set_opp_data: Data to be passed to set_opp callback
 * @dentry:	debugfs dentry pointer of the real device directory (not links).
 * @dentry_name: Name of the real dentry.
 *
 * @voltage_tolerance_v1: In percentage, for v1 bindings only.
 *
 * This is an internal data structure maintaining the link to opps attached to
 * a device. This structure is not meant to be shared to users as it is
 * meant for book keeping and private to OPP library.
 */
struct opp_table {
	struct list_head node;

	struct blocking_notifier_head head;
	struct list_head dev_list;
	struct list_head opp_list;
	struct kref kref;
	struct mutex lock;

	struct device_node *np;
	unsigned long clock_latency_ns_max;

	/* For backward compatibility with v1 bindings */
	unsigned int voltage_tolerance_v1;

	enum opp_table_access shared_opp;
	struct dev_pm_opp *suspend_opp;

	unsigned int *supported_hw;
	unsigned int supported_hw_count;
	const char *prop_name;
	struct clk *clk;
	struct regulator **regulators;
	unsigned int regulator_count;
	bool genpd_performance_state;

	int (*set_opp)(struct dev_pm_set_opp_data *data);
	struct dev_pm_set_opp_data *set_opp_data;

	unsigned int cycle_thief_id;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
	char dentry_name[NAME_MAX];
#endif
};

enum dev_pm_opp_event {
	OPP_EVENT_ADD, OPP_EVENT_REMOVE, OPP_EVENT_ENABLE, OPP_EVENT_DISABLE,
};

#if defined(CONFIG_PM_OPP)

struct opp_table *dev_pm_opp_get_opp_table(struct device *dev);
void dev_pm_opp_put_opp_table(struct opp_table *opp_table);

unsigned long dev_pm_opp_get_voltage(struct dev_pm_opp *opp);

unsigned long dev_pm_opp_get_freq(struct dev_pm_opp *opp);

bool dev_pm_opp_is_turbo(struct dev_pm_opp *opp);

int dev_pm_opp_get_opp_count(struct device *dev);
unsigned long dev_pm_opp_get_max_clock_latency(struct device *dev);
unsigned long dev_pm_opp_get_max_volt_latency(struct device *dev);
unsigned long dev_pm_opp_get_max_transition_latency(struct device *dev);
unsigned long dev_pm_opp_get_suspend_opp_freq(struct device *dev);

struct dev_pm_opp *dev_pm_opp_find_freq_exact(struct device *dev,
					      unsigned long freq,
					      bool available);

struct dev_pm_opp *dev_pm_opp_find_freq_floor(struct device *dev,
					      unsigned long *freq);

struct dev_pm_opp *dev_pm_opp_find_freq_ceil(struct device *dev,
					     unsigned long *freq);
void dev_pm_opp_put(struct dev_pm_opp *opp);

int dev_pm_opp_add(struct device *dev, unsigned long freq,
		   unsigned long u_volt);
void dev_pm_opp_remove(struct device *dev, unsigned long freq);

int dev_pm_opp_enable(struct device *dev, unsigned long freq);

int dev_pm_opp_disable(struct device *dev, unsigned long freq);

int dev_pm_opp_register_notifier(struct device *dev, struct notifier_block *nb);
int dev_pm_opp_unregister_notifier(struct device *dev, struct notifier_block *nb);

struct opp_table *dev_pm_opp_set_supported_hw(struct device *dev, const u32 *versions, unsigned int count);
void dev_pm_opp_put_supported_hw(struct opp_table *opp_table);
struct opp_table *dev_pm_opp_set_prop_name(struct device *dev, const char *name);
void dev_pm_opp_put_prop_name(struct opp_table *opp_table);
struct opp_table *dev_pm_opp_set_regulators(struct device *dev, const char * const names[], unsigned int count);
void dev_pm_opp_put_regulators(struct opp_table *opp_table);
struct opp_table *dev_pm_opp_set_clkname(struct device *dev, const char * name);
void dev_pm_opp_put_clkname(struct opp_table *opp_table);
struct opp_table *dev_pm_opp_register_set_opp_helper(struct device *dev, int (*set_opp)(struct dev_pm_set_opp_data *data));
void dev_pm_opp_unregister_set_opp_helper(struct opp_table *opp_table);
int dev_pm_opp_set_rate(struct device *dev, unsigned long target_freq);
int dev_pm_opp_set_sharing_cpus(struct device *cpu_dev, const struct cpumask *cpumask);
int dev_pm_opp_get_sharing_cpus(struct device *cpu_dev, struct cpumask *cpumask);
void dev_pm_opp_remove_table(struct device *dev);
void dev_pm_opp_cpumask_remove_table(const struct cpumask *cpumask);
#else
static inline struct opp_table *dev_pm_opp_get_opp_table(struct device *dev)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void dev_pm_opp_put_opp_table(struct opp_table *opp_table) {}

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

static inline unsigned long dev_pm_opp_get_suspend_opp_freq(struct device *dev)
{
	return 0;
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

static inline void dev_pm_opp_put(struct dev_pm_opp *opp) {}

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

static inline int dev_pm_opp_register_notifier(struct device *dev, struct notifier_block *nb)
{
	return -ENOTSUPP;
}

static inline int dev_pm_opp_unregister_notifier(struct device *dev, struct notifier_block *nb)
{
	return -ENOTSUPP;
}

static inline struct opp_table *dev_pm_opp_set_supported_hw(struct device *dev,
							    const u32 *versions,
							    unsigned int count)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void dev_pm_opp_put_supported_hw(struct opp_table *opp_table) {}

static inline struct opp_table *dev_pm_opp_register_set_opp_helper(struct device *dev,
			int (*set_opp)(struct dev_pm_set_opp_data *data))
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void dev_pm_opp_unregister_set_opp_helper(struct opp_table *opp_table) {}

static inline struct opp_table *dev_pm_opp_set_prop_name(struct device *dev, const char *name)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void dev_pm_opp_put_prop_name(struct opp_table *opp_table) {}

static inline struct opp_table *dev_pm_opp_set_regulators(struct device *dev, const char * const names[], unsigned int count)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void dev_pm_opp_put_regulators(struct opp_table *opp_table) {}

static inline struct opp_table *dev_pm_opp_set_clkname(struct device *dev, const char * name)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void dev_pm_opp_put_clkname(struct opp_table *opp_table) {}

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
int dev_pm_opp_of_add_table_indexed(struct device *dev, int index);
void dev_pm_opp_of_remove_table(struct device *dev);
int dev_pm_opp_of_cpumask_add_table(const struct cpumask *cpumask);
void dev_pm_opp_of_cpumask_remove_table(const struct cpumask *cpumask);
int dev_pm_opp_of_get_sharing_cpus(struct device *cpu_dev, struct cpumask *cpumask);
struct device_node *dev_pm_opp_of_get_opp_desc_node(struct device *dev);
struct dev_pm_opp *of_dev_pm_opp_find_required_opp(struct device *dev, struct device_node *np);
struct device_node *dev_pm_opp_get_of_node(struct dev_pm_opp *opp);
#else
static inline int dev_pm_opp_of_add_table(struct device *dev)
{
	return -ENOTSUPP;
}

static inline int dev_pm_opp_of_add_table_indexed(struct device *dev, int index)
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

static inline struct device_node *dev_pm_opp_of_get_opp_desc_node(struct device *dev)
{
	return NULL;
}

static inline struct dev_pm_opp *of_dev_pm_opp_find_required_opp(struct device *dev, struct device_node *np)
{
	return NULL;
}
static inline struct device_node *dev_pm_opp_get_of_node(struct dev_pm_opp *opp)
{
	return NULL;
}
#endif

#endif		/* __LINUX_OPP_H__ */

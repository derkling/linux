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

#ifndef __DRIVER_OPP_H__
#define __DRIVER_OPP_H__

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/limits.h>
#include <linux/pm_opp.h>
#include <linux/notifier.h>

struct clk;
struct regulator;

/* Lock to allow exclusive modification to the device and opp lists */
extern struct mutex opp_table_lock;

extern struct list_head opp_tables;

/*
 * Internal data structure organization with the OPP layer library is as
 * follows:
 * opp_tables (root)
 *	|- device 1 (represents voltage domain 1)
 *	|	|- opp 1 (availability, freq, voltage)
 *	|	|- opp 2 ..
 *	...	...
 *	|	`- opp n ..
 *	|- device 2 (represents the next voltage domain)
 *	...
 *	`- device m (represents mth voltage domain)
 * device 1, 2.. are represented by opp_table structure while each opp
 * is represented by the opp structure.
 */

/**
 * struct dev_pm_opp - Generic OPP description structure
 * @node:	opp table node. The nodes are maintained throughout the lifetime
 *		of boot. It is expected only an optimal set of OPPs are
 *		added to the library by the SoC framework.
 *		IMPORTANT: the opp nodes should be maintained in increasing
 *		order.
 * @kref:	for reference count of the OPP.
 * @available:	true/false - marks if this OPP as available or not
 * @dynamic:	not-created from static DT entries.
 * @turbo:	true if turbo (boost) OPP
 * @suspend:	true if suspend OPP
 * @pstate: Device's power domain's performance state.
 * @rate:	Frequency in hertz
 * @supplies:	Power supplies voltage/current values
 * @clock_latency_ns: Latency (in nanoseconds) of switching to this OPP's
 *		frequency from any other OPP's frequency.
 * @opp_table:	points back to the opp_table struct this opp belongs to
 * @np:		OPP's device node.
 * @dentry:	debugfs dentry pointer (per opp)
 *
 * This structure stores the OPP information for a given device.
 */
struct dev_pm_opp {
	struct list_head node;
	struct kref kref;

	bool available;
	bool dynamic;
	bool turbo;
	bool suspend;
	unsigned int pstate;
	unsigned long rate;

	struct dev_pm_opp_supply *supplies;

	unsigned long clock_latency_ns;

	struct opp_table *opp_table;

	struct device_node *np;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
};

/**
 * struct opp_device - devices managed by 'struct opp_table'
 * @node:	list node
 * @dev:	device to which the struct object belongs
 * @dentry:	debugfs dentry pointer (per device)
 *
 * This is an internal data structure maintaining the devices that are managed
 * by 'struct opp_table'.
 */
struct opp_device {
	struct list_head node;
	const struct device *dev;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
};

/* Routines internal to opp core */
void dev_pm_opp_get(struct dev_pm_opp *opp);
void _get_opp_table_kref(struct opp_table *opp_table);
int _get_opp_count(struct opp_table *opp_table);
struct opp_table *_find_opp_table(struct device *dev);
struct opp_device *_add_opp_dev(const struct device *dev, struct opp_table *opp_table);
void _dev_pm_opp_remove_table(struct opp_table *opp_table, struct device *dev, bool remove_all);
void _dev_pm_opp_find_and_remove_table(struct device *dev, bool remove_all);
struct dev_pm_opp *_opp_allocate(struct opp_table *opp_table);
void _opp_free(struct dev_pm_opp *opp);
int _opp_add(struct device *dev, struct dev_pm_opp *new_opp, struct opp_table *opp_table, bool rate_not_available);
int _opp_add_v1(struct opp_table *opp_table, struct device *dev, unsigned long freq, long u_volt, bool dynamic);
void _dev_pm_opp_cpumask_remove_table(const struct cpumask *cpumask, bool of);
struct opp_table *_add_opp_table(struct device *dev);

#ifdef CONFIG_OF
void _of_init_opp_table(struct opp_table *opp_table, struct device *dev);
#else
static inline void _of_init_opp_table(struct opp_table *opp_table, struct device *dev) {}
#endif

#ifdef CONFIG_DEBUG_FS
void opp_debug_remove_one(struct dev_pm_opp *opp);
int opp_debug_create_one(struct dev_pm_opp *opp, struct opp_table *opp_table);
int opp_debug_register(struct opp_device *opp_dev, struct opp_table *opp_table);
void opp_debug_unregister(struct opp_device *opp_dev, struct opp_table *opp_table);
#else
static inline void opp_debug_remove_one(struct dev_pm_opp *opp) {}

static inline int opp_debug_create_one(struct dev_pm_opp *opp,
				       struct opp_table *opp_table)
{ return 0; }
static inline int opp_debug_register(struct opp_device *opp_dev,
				     struct opp_table *opp_table)
{ return 0; }

static inline void opp_debug_unregister(struct opp_device *opp_dev,
					struct opp_table *opp_table)
{ }
#endif		/* DEBUG_FS */

#endif		/* __DRIVER_OPP_H__ */

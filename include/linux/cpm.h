/*
 * cpm.h -- SoC Regulator driver support.
 *
 * Copyright (C) 2009 STMicroelectronics
 *
 * Author: Patrick Bellasi <derkling@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Constrained Power Management Driver Interface.
 */

#ifndef __LINUX_CPM_H
#define __LINUX_CPM_H

#include <linux/notifier.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/device.h>

#define TRUE	1
#define FALSE	(!TRUE)

#define MAX(a, b) ((u32)(a) > (u32)(b) ? (a) : (b))

/* The maximum lenght of text lables (e.g. device/dwr/fsc names, ...) */
#define CPM_NAME_LEN	32

/* The maximum number of DWR each device can have */
#define CPM_DEV_MAX_DWR	16

/* The maximum ASM weight */
#define CPM_ASM_MAX_WEIGHT	 1000000
/* The minimum ASM weight */
#define CPM_ASM_MIN_WEIGHT	-1000000

/*********************************************************************
 *                      CPM PLATFORM INTERFACE                       *
 *********************************************************************/

/**
 * struct cpm_range - a generic range
 * @lower: lower bound
 * @upper: upper bound
 * @type: range type
 *
 */
struct cpm_range {
	u32 lower;
	u32 upper;
#define CPM_ASM_TYPE_UNBOUNDED		0	/* no bounds defined         */
#define CPM_ASM_TYPE_RANGE		1	/* upper and lover bound     */
	/*  defined (if lower==upper */
	/*  than is a single value)  */
#define CPM_ASM_TYPE_LBOUND		2	/* lower bound only */
#define CPM_ASM_TYPE_UBOUND		3	/* upper bound only */
	u8 type:2;

};

/** CPM_PLATFORM_ASM - define a platform ASM
 * This macro can be used to simplify the definition of platform specific
 * ASMs. It requires the initialization of the mandatory parameters only.
 */
#define CPM_PLATFORM_ASM(_name, _type, _mode, _comp, _min, _max)	\
	{								\
		.name = _name,						\
		.type = _type,						\
		.userw = _mode,						\
		.comp = _comp,						\
		.min = _min,						\
		.max = _max,						\
	}

/**
 * struct cpm_asm_range - an ASM range
 * @id: the ID of the ASM
 * @range: the range in the corresponding ASM
 * @kattr: the sysfs attribute to access this ASM data
 * @name: the attribute name
 */
struct cpm_asm_range {
	u8 id;
	struct cpm_range range;
	struct kobj_attribute kattr;
	char name[CPM_NAME_LEN];
};

/* DEV_DWR_ASM - initialize ASMs of a DWR
 * This macro can be used to simplify the definition of DWR's ASMs.
 * It requires the initialization of the mandatory members only.
 * */
#define DEV_DWR_ASM(_ASM_ID_, _LOWER_, _UPPER_, _TYPE_) \
{							\
	.id = _ASM_ID_,					\
	.range = {					\
		.lower = _LOWER_,			\
		.upper = _UPPER_,			\
		.type = _TYPE_,				\
	},						\
}

/**
 * struct cpm_asm_opph -
 * @value:
 * @range:
 * @dt_benefit:
 *
 */
struct cpm_asm_opph {
	u32 value;		/* NOTE è ridondante */
	struct cpm_range *range;
	struct timespec dt_benefit;
};

/**
 * struct cpm_asm -
 * @name:
 * @data: governor specific data
 * @type:
 * @userw: user writable
 * @comp:
 * @weight: the policy defined weight for this ASM
 * 	[CPM_ASM_MIN_WEIGHT:CPM_ASM_MAX_WEIGHT]
 * @min: min feasible value
 * @max: max feasible value
 *
 */
struct cpm_asm {
	char name[CPM_NAME_LEN];
	void *data;
#define CPM_TYPE_LIB	0
#define CPM_TYPE_GIB	1
	u8 type:1;
#define CPM_USER_RO	0
#define CPM_USER_RW	1
	u8 userw:1;
#define CPM_COMPOSITION_ADDITIVE	0
#define CPM_COMPOSITION_RESTRICTIVE	1
	u8 comp:1;
	s32 weight;
	u32 min;
	u32 max;
};

/**
 * struct cpm_dev_dwr - a device's DWR
 * @dev: the device to which this DWR's belongs to
 * @id: ID for the device's DWR
 * @name: name of a region
 * @asms: ASM's array
 * @asms_count: number of ASM in the 'asms' array
 * @gov_data: governor specific data
 * @pol_data: policy specific data
 * @kattr: the sysfs attribute to access this DWR data
 * @asms_group: the ASMs of this device
*
 * A device DWR
 */
struct cpm_dev_dwr {
	struct device *dev;
	u8 id;
	char name[CPM_NAME_LEN];
	struct cpm_asm_range *asms;
	u8 asms_count;
	void *gov_data;
	void *pol_data;
	/* private */
	struct kobj_attribute kattr;
	struct attribute_group asms_group;

};

/* DEV_DWR - initialize a DWR (name and id)
 * This macro can be used to simplify the definition of device DWRs.
 * It requires the initialization of the mandatory members only.
 */
#define DEV_DWR(_id_, _name_, _asms_)		\
{						\
	.id = _id_,				\
	.name = _name_,				\
	.asms = _asms_,				\
	.asms_count = ARRAY_SIZE(_asms_),	\
}


/**
 * struct cpm_fsc_dwr -
 * @dwr: a DWR mapping to an FSC
 * @cdev: the devices corresponding to the DWR add any other FSC-DWR
 * 	specific data
 *
 * Maps an FSC to its DWRs and corresponding devices
 */
struct cpm_fsc_dwr {
	struct cpm_dev_dwr *dwr;
	struct cpm_dev *cdev;
	/* add any other FSC-DWR specific data */
};

/**
 * struct cpm_fsc - a system FSC
 * @id: ID for the system's FSC
 * @asms: ASM's array
 * @asms_count: number of ASM in the 'asms' array
 * @dwrs: array of DWRs that maps to this FSC
 * @dwrs_count: the number of DWRs in the 'dwrs' array
 * @gov_data: governor specific data
 * @pol_data: policy specific data
 * @node: the next FSC
 *
 */
struct cpm_fsc {
	u16 id;
	struct cpm_asm_range *asms;
	u8 asms_count;
	struct cpm_fsc_dwr *dwrs;
	u8 dwrs_count;
	void *gov_data;
	void *pol_data;
	struct list_head node;
};

/**
 * struct cpm_platform_data -
 * @asms: the array of ASM defined by platform code
 * @count: total number of platform ASMs
 */
struct cpm_platform_data {
	struct cpm_asm *asms;
	u8 count;

};

int cpm_register_platform_asms(struct cpm_platform_data *asm_data);

/*********************************************************************
 *                          CPM GOVERNORS                            *
 *********************************************************************/

/* The cpm workwueue for asynchronous tasks scheduling */
extern struct workqueue_struct *cpm_wq;

/**
 * struct cpm_dev -
 * @dev: the device interested
 * @dwrs: the DWRS's array
 * @dwrs_count: the number of DWRs
 * @nb: the notifer chain block
 * @gov_data: governor specific data
 * @pol_data: policy specific data
 * @node: the next cpm_dev in the list
 *
 * The public visible data of a device register to CPM
 */
struct cpm_dev {
	struct device *dev;
	struct cpm_dev_dwr *dwrs;
	u8 dwrs_count;
	struct notifier_block nb;
	void *gov_data;
	void *pol_data;
	struct list_head node;
};

/**
 * struct cpm_governor - a CPM Governor
 * @name:
 * @build_fsc_list:
 *
 * Basically define the list of available FSC with an algorithm that identify
 * all the system available FSC according to an identification strategy.
 */
struct cpm_governor {
	char name[CPM_NAME_LEN];
	int (*build_fsc_list) (struct list_head *dev_list, u8 dev_count);
};

/*
 * Register a governor to cpm_core
 */
int cpm_register_governor(struct cpm_governor *governor);

/*
 * Get a reference to a new FSC node
 *
 * This method return a pointer to a new FSC element.
 * The element returned is zeroed and properly initialized with necessary
 * core values. A NULL pointer will be returned on errors.
 */
struct cpm_fsc *cpm_get_new_fsc(void);

/*
 * Define a new FSC list
 */
int cpm_set_fsc_list(struct list_head *fsc_list);

/*
 * Merge, if possible,  two cpm_range
 */
int cpm_merge_range(struct cpm_range *first, struct cpm_range *second);

/*
 * Compute the weight of a range on the specified ASM
 */
int cpm_weight_range(struct cpm_range *range, u8 asm_id, s32 * weight);

/*********************************************************************
 *                          CPM POLICIES                             *
 *********************************************************************/

/**
 * struct cpm_fsc_pointer -
 * @fsc: the FSC referred
 * @node:the next element
 */
struct cpm_fsc_pointer {
	struct cpm_fsc *fsc;
	struct list_head node;
};

/**
 * struct cpm_entity -
 * @task:
 * @dev:
 * @ptr:
 * @type: the entity type
 *
 * A CMP Entity can be either a task or a device and define the owner of a
 * constraint asserted.
 */
struct cpm_entity {
	union {
		struct task_struct *task;
		struct device *dev;
		void *ptr;
	};
#define CPM_ENTITY_TYPE_DRIVER	0
#define CPM_ENTITY_TYPE_TASK	1
	u8 type:1;
};

/**
 * struct cpm_policy_notify_data -
 * @entity:
 * @range:
 *
 * The data that are passed to a policy with the notification of a
 * CPM_EVENT_NEW_CONSTRAINT using its ddp_handler
 */
struct cpm_policy_notify_data {
	struct cpm_entity entity;
	struct cpm_range range;
};

/**
 * struct cpm_policy - a CPM Policy
 * @name:
 * @sort_fsc_list: This is an anynchronous call that should not block
 * 	It is used to notify the policy for the possibility pro provide
 * 	a new sorted FSC list.
 * 	The sorted list has to be computed asynchronously (e.g. using a
 * 	tasklet) and once ready notified to the core using the provided
 * 	cpm_set_orderet_fsc_list.
 * 	A policy should return 0 on notification success; otherwise
 * 	a -EAGAIN is expected and the core will try to request ordering
 * 	sometime later.
 * @ddp_handler: data's content depende on event type:
 * 	CPM_EVENT_NEW_CONSTRAINT: struct cpm_policy_notify_data
 * 	CPM_EVENT_FSC_FOUND: struct cpm_fsc_pointer
 * 	CPM_EVENT_PRE_CHANGE: struct cpm_fsc_pointer
 * 		distributed agreement on switching to the specified FSC
 * 	CPM_EVENT_POST_CHAGNE: struct cpm_fsc_pointer or NULL
 * 		the specified FSC has been activated or has been aborted
 * 	return 0 on success, non null otherwise
 *
 * Basically define a FSCs ordering strategy and a system
 * configuration switching supervisor.
 */
struct cpm_policy {
	char name[CPM_NAME_LEN];
	int (*sort_fsc_list) (struct list_head *fsc_list);
	int (*ddp_handler) (unsigned long event, void *data);
};

/**
 * cpm_register_policy() -
 */
int cpm_register_policy(struct cpm_policy *policy);

/**
 * cpm_set_ordered_fsc_list() -
 * Define the ordered FSC list to use for validation and selection.
 * This method will return a list of cpm_fsc_pointer's elements.
 */
int cpm_set_ordered_fsc_list(struct list_head *fscpl_head);

/*********************************************************************
 *                      CPM DRIVER INTERFACE                         *
 *********************************************************************/

/**
 * ddp_callback - a device's DDP callback function
 * @ddp_stage: the DDP stage
 * @ddp_data: a pointer to a cpm_ddp_data struct containing the informations
 * needed by the DDP.
 *
 * Every device should provide such a function that will be called back by the
 * core during the DDP stages.
 */
#define CPM_EVENT_NEW_CONSTRAINT	0
#define CPM_EVENT_FSC_FOUND		1
#define CPM_EVENT_DO_CHANGE		2
#define CPM_EVENT_ABORT			3
#define CPM_EVENT_PRE_CHANGE		4
#define CPM_EVENT_POST_CHANGE		5
typedef int (*ddp_callback) (struct notifier_block *, unsigned long, void *);

/**
 * struct cpm_dev_data
 * @notifier_callback: the device's DDP callback
 * @dwrs: the DWR's array for the subscribing device
 * @dwrs_count: the number of DWR int the 'dwrs' array
 *
 * Device specific data for CPM registration.
 */
struct cpm_dev_data {
	ddp_callback notifier_callback;
	struct cpm_dev_dwr *dwrs;
	u8 dwrs_count;
};

/**
 * struct cpm_ddp_data -
 * @fsc:
 *
 * The data that the core provide to devices during a DDP
 */
struct cpm_ddp_data {
	struct cpm_fsc *fsc;
};

/**
 * cpm_register_device() -
 *
 * Return 0 un success, -EEXIST if the device is already registered.
 */
int cpm_register_device(struct device *dev, struct cpm_dev_data *data);

/**
 * cpm_unregister_device() -
 */
int cpm_unregister_device(struct device *dev);

/**
 * cpm_update_constraint() -
 */
int cpm_update_constraint(struct device *dev, u8 asm_id,
			  struct cpm_range *range);

/**
 * cpm_remove_constraint() -
 */
int cpm_remove_constraint(struct device *dev, u8 asm_id);

/**
 * cpm_fsc_to_om() -
 */
int cpm_fsc_to_om(struct device *dev, u8 fsc_id, u8 * om_id);

/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/

#define CPM_DEBUG_CORE		1
#define CPM_DEBUG_GOVERNOR	2

#ifdef CONFIG_CPM_DEBUG

extern void cpm_debug_printk(unsigned int type, const char *prefix,
			     const char *fmt, ...);

#else

#define cpm_debug_printk(msg...) do { } while (0)

#endif /* CONFIG_CPM_DEBUG */

#endif

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

// The maximum lenght of text lables (e.g. device/dwr/fsc names, ...)
#define CPM_NAME_LEN	12

// The maximum number of DWR each device can have
#define CPM_DEV_MAX_DWR	16

typedef u8 cpm_id;


/*********************************************************************
 *                      CPM PLATFORM INTERFACE                       *
 *********************************************************************/

/**
 * A generic range
 */
struct cpm_range {
	u32 lower;				/* lower bound */
	u32 upper;				/* upper bound */
#define CPM_ASM_TYPE_UNBOUNDED		0	/* no bounds defined         */
#define CPM_ASM_TYPE_RANGE		1	/* upper and lover bound     */
						/*  defined (if lower==upper */
						/*  than is a single value)  */
#define CPM_ASM_TYPE_LBOUND		2	/* lower bound only */
#define CPM_ASM_TYPE_UBOUND		3	/* upper bound only */
	u8 type:2;				/* range type */

};

/**
 * An ASM range 
 */
struct cpm_asm_range {
	cpm_id id;			/* The ID of the ASM */
	struct cpm_range range;		/* The range in the corresponding ASM */
#ifdef CONFIG_CPM_SYSFS
	struct kobj_attribute kattr;	/* The sysfs attribute to access this ASM data */
	char name[CPM_NAME_LEN];	/* The attribute name */
#endif
};

/**
 * 
 */
struct cpm_asm_opph {
	u32 value;			/* NOTE è ridondante */
	struct cpm_range *range;
	struct timespec dt_benefit;
};

/**
 * 
 */
struct cpm_asm {
	char name[CPM_NAME_LEN];
	void *data;			/* governor specific data */
#define CPM_TYPE_LIB	0
#define CPM_TYPE_GIB	1
	u8 type:1;
#define CPM_USER_RO	0
#define CPM_USER_RW	1
	u8 userw:1;			/* user writable */
#define CPM_COMPOSITION_ADDITIVE	0
#define CPM_COMPOSITION_RESTRICTIVE	1
	u8 comp:1;
};

/*
 * A device DWR
 */
struct cpm_dev_dwr {
	struct device *dev;		/* The device to which this DWR's belongs to */
	cpm_id id;			/* ID for the device's DWR */
	char name[CPM_NAME_LEN];	/* name of a region */
	struct cpm_asm_range *asms;	/* ASM's array */
	u8 asms_count;			/* number of ASM in the 'asms' array */
#ifdef CONFIG_CPM_SYSFS
	struct kobj_attribute kattr;      	/* The sysfs attribute to access this DWR data */
	struct attribute_group asms_group;  	/* The ASMs of this device */
#endif
	void *gov_data;			/* governor specific data */
	void *pol_data;			/* policy specific data */
};

/*
 * Maps an FSC to its DWRs and corresponding devices
 */
struct cpm_fsc_dwr {
	struct cpm_dev_dwr *dwr;	/* a DWR mapping to an FSC */
	/* add any other FSC-DWR specific data */
};

/*
 * A system FSC
 */
struct cpm_fsc {
	cpm_id id;			/* ID for the system's FSC */
	struct cpm_asm_range *asms;	/* ASM's array */
	u8 asms_count;			/* number of ASM in the 'asms' array */
	struct cpm_fsc_dwr *dwrs;	/* array of DWRs that maps to this FSC */
	u8 dwrs_count;			/* the number of DWRs in the 'dwrs' array */
	void *gov_data;			/* governor specific data */
	void *pol_data;			/* policy specific data */
	struct list_head node;		/* the next FSC */
};

/**
 * 
 */
struct cpm_platform_data {
	struct cpm_asm	*asms;	/* the array of ASM defined by platform code*/
	u8 count;		/* total number of platform ASMs */

};

int cpm_register_platform_asms(struct cpm_platform_data *asm_data);


/*********************************************************************
 *                          CPM GOVERNORS                            *
 *********************************************************************/

/*
 * The public visible data of a device register to CPM
 */
struct cpm_dev {
	struct device *dev;		/* the device interested */
	struct cpm_dev_dwr *dwrs;	/* the DWRS's array */
	u8 dwrs_count;			/* the number of DWRs */
	struct notifier_block nb;	/* the notifer chain block */
	void *gov_data;			/* governor specific data */
	void *pol_data;			/* policy specific data */
	struct list_head node;		/* the next cpm_dev in the list */
};

/**
 * A CPM Governor
 *
 * Basically define the list of available FSC with an algorithm that identify
 * all the system available FSC according to an identification strategy.
 */
struct cpm_governor {
	char name[CPM_NAME_LEN];
	int (*build_fsc_list)(struct list_head *dev_list, u8 dev_count);
};

/*
 * Register a governor to cpm_core
 */
int cpm_register_governor(struct cpm_governor *governor);

/*
 * Define a new FSC list
 */
int cpm_set_fsc_list(struct list_head *fsc_list);

/*
 * Merge, if possible,  two cpm_range
 */
int merge_cpm_range(struct cpm_range *first, struct cpm_range *second);



/*********************************************************************
 *                          CPM POLICIES                             *
 *********************************************************************/

/**
 * A CPM Policy
 *
 * Basically define a FSCs ordering strategy and a system
 * configuration switching supervisor.
 */
struct cpm_policy {
	char name[CPM_NAME_LEN];
	 /* This is an anynchronous call that should not block.
	  * It is used to notify the policy for the possibility pro provide
	  * a new sorted FSC list.
	  * The sorted list has to be computed asynchronously (e.g. using a
	  * tasklet) and once ready notified to the core using the provided
	  * cpm_set_orderet_fsc_list */
	int (*sort_fsc_list)(struct list_head *fsc_list);
#define CPM_DDP_DONE	NOTIFY_DONE
#define CPM_DDP_OK	NOTIFY_OK
#define CPM_DDP_BAD	NOTIFY_BAD
	int (*ddp_handler)(unsigned long, void *);
};

/**
 * 
 */
int cpm_register_policy(struct cpm_policy *policy);

/*
 * Define the ordered FSC list to use for validation and selection.
 */
int cpm_set_ordered_fsc_list(struct list_head *ordered_fsc_list);



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
typedef int (*ddp_callback)(struct notifier_block *, unsigned long, void *);


/**
 * Device specific data for CPM registration.
 */
struct cpm_dev_data {
#define CPM_EVENT_NEW_CONSTRAINT	0
#define CPM_EVENT_FSC_FOUND		1
#define	CPM_EVENT_DO_CHANGE		2
#define CPM_EVENT_PRE_CHANGE		3
#define CPM_EVENT_POST_CHANGE		4
	ddp_callback notifier_callback;	/* The device's DDP callback */
	struct cpm_dev_dwr *dwrs;	/* The DWR's array for the subscribing device */
	u8 dwrs_count;			/* The number of DWR int the 'dwrs' array */
};

/**
 * The data that the core provide to devices during a DDP
 */
struct cpm_ddp_data {
	struct cpm_fsc *fsc;
};

/**
 * Register a device and its DWRs.
 *
 * Return 0 un success, -EEXIST if the device is already registered.
 */
int cpm_register_device(struct device *dev, struct cpm_dev_data *data);

/**
 * Unregister a device and release all its data.
 */
int cpm_unregister_device(struct device *dev);

/**
 * Add a new constraint on the specified ASM.
 */
int cpm_add_constraint(struct device *dev, cpm_id asm_id, struct cpm_range * range);

/**
 * Update a constraint on the specified ASM.
 */
int cpm_update_constraint(struct device *dev, cpm_id asm_id, struct cpm_range * range);

/**
 * Release a constraint on the specified ASM.
 */
int cpm_remove_constraint(struct device *dev, cpm_id asm_id);

/*
 * Return the device's operating mode corresponding to the specified FSC.
 */
int cpm_fsc_to_om(struct device *dev, cpm_id fsc_id, cpm_id *om_id);


/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/

#define CPM_DEBUG_CORE		1
#define CPM_DEBUG_GOVERNOR	2

#ifdef CONFIG_CPM_DEBUG

extern void cpm_debug_printk(unsigned int type, const char *prefix, 
				 const char *fmt, ...);

#else

#define cpm_debug_printk(msg...) do { } while(0)

#endif /* CONFIG_CPM_DEBUG */


#endif

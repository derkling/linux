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

#define TRUE	1
#define FALSE	(!TRUE)

#define CPM_NAME_LEN	12

typedef cpm_id	u8;


/*********************************************************************
 *                      CPM PLATFORM INTERFACE                       *
 *********************************************************************/

/**
 * struct cpm_range - 
 */
struct cpm_range {
	u32 lower;
	u32 upper;
#define CPM_ASM_TYPE_RANGE		0
#define CPM_ASM_TYPE_SINGLE		1
#define CPM_ASM_TYPE_LBOUND		2
#define CPM_ASM_TYPE_UBOUND		3
	u8 type:2;

}

/**
 * struct cpm_asm_opph - 
 */
struct cpm_asm_opph {
	u32 value;			/* NOTE è ridondante */
	struct cpm_range *range;
	struct timespec dt_benefit;
}

/**
 * struct cpm_asm_ddp - 
 */
struct cpm_asm_ddp {
	u32 seq;				/* updates count during a DDP */
	struct cpm_range *range;
}

/**
 * struct cpm_asm_stats - 
 */
struct cpm_asm_stats {
	struct timespec t_update;	/* timestamp of last update */
	u32 violations;			/* violations count for this ASM */
	u32 min_value;			/* minimum value used for this ASM */
	u32 max_value;			/* maximum value used for this ASM */
}

/**
 * struct cpm_asm - 
 */
struct cpm_asm {
	char name[CPM_NAME_LEN];
	struct cpm_asm_ddp opph;
	struct cpm_asm_ddp ddp;		/* DDP data for an ASM */
	struct cpm_asm_stats stats;
	void *data;			/* governor specific data */
#define CPM_TYPE_LIB	0
#define CPM_TYPE_GIB	1
	u8 type:1;
#define CPM_USER_RO	0
#define CPM_USER_RW	1
	u8 userw:1;			/* user writable */
#define CPM_COMPOSITION_ADDICTIVE	0
#define CPM_COMPOSITION_RESTRICTIVE	1
	u8 comp:1;
}


/**
 * struct cpm_platform_data - 
 */
struct cpm_platform_data {
	struct cpm_range cur_range[];	/* current ASM ranges (actual FSC) */ 
	struct cpm_range ddp_range[];	/* ASM ranges during a DDP */	
	struct cpm_asm	asms[];
	cpm_id count;			/* total number of platform ASMs */
}

/**
 * cpm_register_platform_asms() -
 */
int cpm_register_platform_asms(cpm_platform_data *asm_data);



/*********************************************************************
 *                          CPM GOVERNORS                            *
 *********************************************************************/

extern struct cpm_ddp_drv_data;
/**
 * struct cpm_ddp_data - 
 */
struct cpm_ddp_gov_data {
	struct cpm_ddp_drv_data drv_data;	/* driver DDP data*/
}

#define to_gov_data(data)					\
	container_of(data, struct cpm_ddp_data, drv_data)

/*
 * The governor's visible data of an FSC 
 */
struct cpm_fsc {
	cpm_id id;		/* the ID for the FSC */
	list_head asm_range;	/* the list of cpm_asm_range */
	void * gov_data;	/* governor specific data */
	list_head node;		/* the next cpm_asm_region */
}


/**
 * 
 */
struct cpm_governor {
	char name[CPM_NAME_LEN];
	int (*update_fsc_data)(list_head fsc_list);
	int (*ddp_handler)(unsigned long, void *);
}

#define CPM_DDP_DONE	NOTIFY_DONE
#define CPM_DDP_OK	NOTIFY_OK
#define CPM_DDP_BAD	NOTIFY_BAD

/**
 *
 */
int cpm_register_governor(struct cpm_governor *governor);

/*
 * Define the ordered FSC list to use for validation and selection.
 */
int cpm_set_ordered_fsc_list(list_head *ordered_fsc_list);


/*********************************************************************
 *                          CPM POLICIES                             *
 *********************************************************************/

/**
 * The policy define the algorithm to use to identify FSC 
 */
struct cpm_policy {
	int (*update_fsc_list)(/*TO BE DEFINED*/);
	u8 check_type:1;
}


/**
 * 
 */
int cpm_register_policy(struct cpm_policy *policy);




/*********************************************************************
 *                      CPM DRIVER INTERFACE                         *
 *********************************************************************/

struct cpm_asm_range {
	cpm_id id;		/* The ASM */
	cpm_range range;	/* The range in the corresponding ASM */
	list_head node;		/* The next cpm_asm_range */
}

struct cpm_asm_region {
	char name[CPM_NAME_LEN];	/* The name of a region */
	cpm_id id;			/* The ID for the region */
	// NOTE this list should be ordered based on asm_range->id;
	// Use the core provided functions to insert properly nodes
	list_head asm_range;		/* The list of cpm_asm_range */
	list_head node;			/* The next cpm_asm_region */
}

/*
 * Insert the given asm_range into the specified ordered list.
 */
int cpm_add_asm_region(list_head *list, struct cpm_asm_range *range);

/**
 * Device specific data for CPM registration.
 */
struct cpm_dev_data {
#define CPM_EVENT_NEW_CONSTRAINT	0
#define CPM_EVENT_FSC_FOUND		1
#define	CPM_EVENT_DO_CHANGE		2
#define CPM_EVENT_PRE_CHANGE		3
#define CPM_EVENT_POST_CHANGE		4
	int (*notifier_call)(struct notifier_block *, unsigned long, void *);
	list_head *dwrs;
}

/**
 * The data that the core provide to devices during a DDP.
 */
struct cpm_ddp_drv_data {
	struct cpm_fsc * fsc;
}

/**
 * Register a device and its DWRs.
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

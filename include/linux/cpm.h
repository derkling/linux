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
	u8 steps;				/* DDP depth */
	void *gov_data;				/* governor data */
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
 * struct cpm_governor - 
 */
struct cpm_governor {
	char name[CPM_NAME_LEN];
	int (*update_validate)(struct notifier_block *nb, cpm_asm_id asm_id);
	int (*update_fsc_data)(list_head fsc_list);
	int (*ddp_prehandler)(struct notifier_block *nb, unsigned long, void *);
	int (*ddp_posthandler)(struct notifier_block *nb, unsigned long, void *);
	int (*ddp_init)(struct cpm_ddp_data* ddp_data);
	int (*ddp_complete)(struct cpm_ddp_data* ddp_data);
}

#define CPM_DDP_DONE	NOTIFY_DONE
#define CPM_DDP_OK	NOTIFY_OK
#define CPM_DDP_BAD	NOTIFY_BAD

/**
 *
 */
int cpm_register_governor(struct cpm_governor *governor);

/*
 * Define the ordered FSC list to use for validation and  selection.
 */
int cpm_set_ordered_fsc_list(list_head *ordered_fsc_list);


/*********************************************************************
 *                          CPM POLICIES                             *
 *********************************************************************/

/**
 * struct cpm_policy - 
 */
struct cpm_policy {
	u8 check_type:1;
}


/**
 * cpm_register_policy() -
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
 * struct cpm_dev_data - 
 */
struct cpm_dev_data {
#define CPM_RUN_DDP		0
#define CPM_RUN_PRECHANGE	1
#define CPM_RUN_POSTCHAGNE	2
	int (*notifier_call)(struct notifier_block *, unsigned long, void *);
	struct cpm_asm_params asms[];	/* ASMs to register */
	u8 count;			/* The number of asms to register */
}

/**
 * struct cpm_ddp_drv_data - 
 */
struct cpm_ddp_drv_data {
	struct cpm_range cur_range[];	/* current ASM ranges (actual FSC) */ 
	struct cpm_range ddp_range[];	/* ASM ranges during a DDP */
}

/**
 * 
 */
int cpm_register_dwr(struct device *dev, list_head *dwrs);

/**
 *
 */
int cpm_release_dwr(struct device *dev);

/**
 *
 */
int cpm_add_constraint(struct device *dev, cpm_id asm_id, struct cpm_range * range);

/**
 *
 */
int cpm_update_constraint(struct device *dev, cpm_id asm_id, struct cpm_range * range);

/**
 *
 */
int cpm_remove_constraint(struct device *dev, cpm_id asm_id);


#define MIN(a, b) ((u32)(a) < (u32)(b) ? (a) : (b))
#define MAX(a, b) ((u32)(a) > (u32)(b) ? (a) : (b))

#define CPM_RANGE_OK		TRUE
#define CPM_RANGE_ERROR		FALSE
/**
 * Verify if the specified value is within the given range
 * @range:	the range to consider for the check
 * @value:	the value to check
 *
 * Long description...
 *
 * Return values: CPM_RANGE_ERROR if value is outside the range,
 * CPM_RANGE_OK otherwise.
 */
inline int cpm_verify_range(struct cpm_range *range, u32 value) {
	switch (range->type) {
	case CPM_ASM_TYPE_RANGE:
		if (value>range->upper)
			return CPM_RANGE_ERROR;
		if (value<range->lower)
			return CPM_RANGE_ERROR;
		break;
	case CPM_ASM_TYPE_SINGLE:
		if (value!=range->lower)
			return CPM_RANGE_ERROR;
		break;
	case CPM_ASM_TYPE_LBOUND:
		if (value<range->lower)
			return CPM_RANGE_ERROR;
		break;
	case CPM_ASM_TYPE_UBOUND:
		if (value>range->upper)
			return CPM_RANGE_ERROR;
		break;
	}
	return CPM_RANGE_OK;
}

/**
 * Update the range for a specified ASM.
 */
inline int cpm_update_ddp_range(struct cpm_range asms[], cpm_id idx, struct cpm_range *range) {
	int ret = 0;

	switch( asms[idx].type ) {

	case CPM_ASM_TYPE_RANGE:
		switch( range.type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( ( asms[idx].lower > range->upper ) ||
					( asms[idx].upper < range->lower ) ) {
				ret = -EINVAL;
			} else {
				asms[idx].lower = MAX(asms[idx].lower, range->lower);
				asms[idx].upper = MIN(asms[idx].upper, range->upper);
			}
			break;
		case CPM_ASM_TYPE_SINGLE:
			if ( cpm_verify_range(&asms[idx], range->lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].lower = range->lower;
				asms[idx].type = CPM_ASM_TYPE_SINGLE;
			}
			break;
		case CPM_ASM_TYPE_LBOUND:
			if ( cpm_verify_range(&asms[idx], range->lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].lower = range->lower;
			}
			break;
		case CPM_ASM_TYPE_UBOUND:
			if ( cpm_verify_range(&asms[idx], range->upper) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].upper = range->upper;
			}
			break;
		}
		break;

	case CPM_ASM_TYPE_SINGLE:
		switch( range.type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( cpm_verify_range(range, amsm[idx].lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			}
			break;
		case CPM_ASM_TYPE_SINGLE:
			if ( asms[idx].lower != range->lower ) {
				ret = -EINVAL;
			}	
			break;
		case CPM_ASM_TYPE_LBOUND:
		case CPM_ASM_TYPE_UBOUND:
			if ( cpm_verify_range(range, amsm[idx].lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			}
			break;
		}
		break;

	case CPM_ASM_TYPE_LBOUND:
		switch( range.type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( cpm_verify_range(&asms[idx], range->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].lower = MAX(asms[idx].lower, range->lower);
				asms[idx].upper = range->upper;
				asms[idx].type = CPM_ASM_TYPE_RANGE;
			}
			break;

		case CPM_ASM_TYPE_SINGLE:
			if ( cpm_verify_range(&asms[idx], range->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].lower = range->lower;
				asms[idx].type = CPM_ASM_TYPE_SINGLE;
			}
			break;
		case CPM_ASM_TYPE_LBOUND:
			asms[idx].lower = MAX(asms[idx].lower, range->lower);
			break;
		case CPM_ASM_TYPE_UBOUND:
			if ( cpm_verify_range(&asms[idx], range->upper)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].upper = range->upper;
				asms[idx].type = CPM_ASM_TYPE_RANGE;
			}
			break;
		}

	case CPM_ASM_TYPE_UBOUND:
		switch( range.type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( cpm_verify_range(&asms[idx], range->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].upper = MIN(asms[idx].upper, range->upper);
				asms[idx].lower = range->lower;
				asms[idx].type = CPM_ASM_TYPE_RANGE;
			}
			break;
		case CPM_ASM_TYPE_SINGLE:
			if ( cpm_verify_range(&asms[idx], range->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].lower = range->lower;
				asms[idx].type = CPM_ASM_TYPE_SINGLE;
			}
			break;
		case CPM_ASM_TYPE_LBOUND:
			if ( cpm_verify_range(&asms[idx], range->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				asms[idx].lower = range->lower;
				asms[idx].type = CPM_ASM_TYPE_RANGE;
			}
			break;
		case CPM_ASM_TYPE_UBOUND:
			asms[idx].upper = MAX(asms[idx].upper, range->upper);
			break;
		}
	}

	return ret;

}


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

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

#define TRUE	0
#define FALSE	(!TRUE)

#define CPM_NAME_LEN	12

typedef cpm_size	u8;


/*********************************************************************
 *                      CPM PLATFORM INTERFACE                       *
 *********************************************************************/

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
	struct timespec dt_benefit;
}

/**
 * struct cpm_asm_stats - 
 */
struct cpm_asm_stats {
	struct timespec t_update;
	u32 violations;
	u32 min_value;
	u32 max_value;
}

/**
 * struct cpm_asm - 
 */
struct cpm_asm {
	char name[CPM_NAME_LEN];
	struct cpm_asm_ddp opph;
	struct cpm_asm_ddp ddp;
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
 * struct cpm_platform_data - 
 */
struct cpm_platform_data {
	struct cpm_range cur_range[];
	struct cpm_range ddp_range[];
	struct cpm_asm	asms[];
	cpm_size count;			/* total number of platform ASMs */
}

/**
 * cpm_register_platform_asms() -
 */
int cpm_register_platform_asms(cpm_platform_data *asm_data);



/*********************************************************************
 *                          CPM GOVERNORS                            *
 *********************************************************************/

/**
 * struct cpm_ddp_data - 
 */
struct cpm_ddp_data {
	u8 steps;
	struct cpm_range *range;
	void *data;
	u8 update[];	/* Map of ASMs updated during last notify call */
			/* must be initialized by governor @ DDP init */
	u8 agreement:1;
}

/**
 * struct cpm_governor - 
 */
struct cpm_governor {
	char name[CPM_NAME_LEN];
	int (*update_validate)(struct cpm_device_block *, u8 idx);
	int (*ddp_prehandler)(struct notifier_block *, unsigned long, void *);
	int (*ddp_posthandler)(struct notifier_block *, unsigned long, void *);
	int (*ddp_init)(struct cpm_ddp_data* ddp_data);
	int (*ddp_complete)(struct cpm_ddp_data* ddp_data);
}

#define CPM_DDP_DONE	NOTIFY_DONE
#define CPM_DDP_OK	NOTIFY_OK
#define CPM_DDP_BAD	NOTIFY_BAD

/**
 * cpm_register_governor() -
 */
int cpm_register_governor(struct cpm_governor *governor);


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

/**
 * struct cpm_asm_params - 
 */
struct cpm_asm_params {
	char name[CPM_NAME_LEN];
	cpm_asm_id id;			/* the ID of the ASM mapped */
#define CPM_INTEREST_AFFECT		0
#define CPM_INTEREST_INFLUENCE		1
	u8 type:1;			/* interest (affect/influenced) */
}

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
 * cpm_subscribe_asms() -
 */
int cpm_subscribe_asms(struct device *dev, struct cpm_dev_data *dev_data);

/**
 * cpm_unsubscribe_asms() -
 */
int cpm_unsubscribe_asms(struct device *dev, cpm_size asm_id);

/**
 * cpm_release_asms() -
 */
int cpm_release_asms(struct device *dev);

/**
 * cpm_update_asm() -
 */
int cpm_update_asm(struct device *dev, cpm_size asm_id, struct cpm_range * range);

#define MIN(a, b) ((u32)(a) < (u32)(b) ? (a) : (b))
#define MAX(a, b) ((u32)(a) > (u32)(b) ? (a) : (b))

#define CPM_RANGE_OK		TRUE
#define CPM_RANGE_ERROR		FALSE
/**
 * cpm_verify_range() - verify if the specified value is within the given range
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
			return CPM_RANGE_ERROR
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
 * cpm_update_ddp_range() -
 */
inline int cpm_update_ddp_range(struct cpm_range asms[], cpm_size idx, struct cpm_range *range) {
	u32 val;

	switch(range->type) {
	case CPM_ASM_TYPE_RANGE:
		if ( cpm_verify_range(&asms[idx], range->lower)
			== CPM_RANGE_ERROR )
			return EINVAL;
		if ( cpm_verify_range(&asms[idx], range->upper)
			== CPM_RANGE_ERROR )
			return EINVAL;
		//TODO
		return 0;
	case CPM_ASM_TYPE_SINGLE:
		if ( cpm_verify_range(&asms[idx], range->lower)
			== CPM_RANGE_ERROR )
			return EINVAL;
		if (asms[idx].type!=CPM_ASM_TYPE_SINGLE) {
			asms[idx].type=CPM_ASM_TYPE_SINGLE;
			asms[idx].lower=range->lower;
		}
		return 0;
	case CPM_ASM_TYPE_LBOUND:
		if ( cpm_verify_range(&asms[idx], range->lower)
			== CPM_RANGE_ERROR )
			return EINVAL;
		//TODO
		return 0;
	case CPM_ASM_TYPE_UBOUND:
		if ( cpm_verify_range(&asms[idx], range->upper)
			== CPM_RANGE_ERROR )
			return EINVAL;
		//TODO
		return 0;
	}


	if (range->type == CPM_ASM_TYPE_SINGLE) {
		if ( cpm_verify_range(&asms[idx], range->lower)
			== CPM_RANGE_ERROR )
			return EINVAL;
	}

	switch(asms[idx]->type) {
		case CPM_ASM_TYPE_RANGE:
			val = MIN(asms[idx]->upper, range->upper);
		case CPM_ASM_TYPE_SINGLE:
		case CPM_ASM_TYPE_LBOUND:
		case CPM_ASM_TYPE_UBOUND:
	}
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

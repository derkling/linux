/*
 * cpm_core.c - CPM Framework Core
 *
 * Copyright (C) 2009 STMicroelectronics
 *
 * Author: Patrick Bellasi <derkling@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Constrained Power Management Core Framework.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/cpm.h>
#include <linux/sched.h>

/*********************************************************************
 *                     CORE DATA TYPES                               *
 *********************************************************************/

/**
 * struct cpm_swm_map - a list node for referencing a set of SWMs
 * @id: the ID of the SWM mapped
 *
 * This structure is used to collect references to SWMs.
 */
struct cpm_swm_map {
	u8 id;
	struct list_head node;
};

/**
 * struct cpm_dev_sysfs -
 * @cpm: the kobj to export device info in sysfs (<device>/cpm)
 * @dwrs: the kobj to export device DWRS in sysfs (<device>/cpm/dwrs)
 * @constraints: the kobj to export device constraints in sysfs
 * 	(<device>/cpm/constraints)
 *
 */
struct cpm_dev_sysfs {
	struct kobject cpm;
	struct kobject dwrs;
	struct kobject constraints;
	/* TODO add entries for statistics */
};

/**
 * struct cpm_constraint -
 * @entity: the entity asserting the constraint
 * @range: the asserted constraint
 * @node: the next constraint
 *
 */
struct cpm_constraint {
	struct cpm_entity entity;
	struct cpm_range range;
	struct list_head node;
};

/**
 * struct cpm_swm_core -
 * @info: the public accessible data
 * @id: the SWMs unique identifier
 * @cur_range: the current range for this SWM according to asserted constraints
 * @ddp_range: the range to use during a DDP
 * @valid_range: the SWM validity range according to current constraints
 * @constraints: the list of constraints asserted on this SWM
 * @kattr: the sysfs attribute to show SWMs values
 *
 */
struct cpm_swm_core {
	struct cpm_swm info;
	u8 id;
	struct cpm_range cur_range;
	struct cpm_range ddp_range;
	struct cpm_range valid_range;
	struct list_head constraints;
	struct kobj_attribute kattr;
};

/**
 * struct cpm_dev_core -
 * @dev_info: the public accessible data
 * @swm_list: list of cpm_swm_map that belongs to at least one DWR
 * @sysfs: the sysfs interface
 *
 */
struct cpm_dev_core {
	struct cpm_dev dev_info;
	struct list_head swm_list;
	struct cpm_dev_sysfs sysfs;
};

/**
 * struct cpm_fsc_core -
 * @info: the public accessible data
 * @valid: this flasg the FSC as valid
 * @swms_group: the SWMs of this FSC
 * @dwrs_group: the DWRs of this FSC
 * @kobj: the kobj for the FSC folder
 */
struct cpm_fsc_core {
	struct cpm_fsc info;
	u8 valid:1;
	struct attribute_group swms_group;
	struct attribute_group dwrs_group;
	struct kobject kobj;
};

/**
 * cpm_fsc_data -
 * @found: the list of available FSCs
 * @ordered: the list of ordered FSCs
 * @omux: the mutex to protect ordered FSC list
 * @count: the count of FSC in the list
 * @curr: the current selected FSC
 * @kobj: <fscs> = <cpm>/fscs
 * @updated: the curr pointer is marked not-updated when the curretn FSC is no
 * 	more valid because of some constraint added/remove or solutions
 *	spache shrinking
 * @valid: the FSC list is marked invalid when the device base change.
 * 	An invalid FSCs list will be automatically cleaned once no more
 *	reference are present
 *
 */
struct cpm_fsc_data {
	struct list_head found;
	struct list_head ordered;
	struct mutex omux;
	unsigned int count;
	struct cpm_fsc_pointer *curr;
	struct kobject kobj;
	short unsigned updated:1;
	short unsigned valid:1;
};

/**
 * struct cpm_dev_list -
 * @list: the list of registered device
 * @count: the number of devices subscribed
 * @mux: the device list mutex
 *
 */
struct cpm_dev_list {
	struct list_head list;
	u8 count;
	struct mutex mux;
};

/**
 * struct cpm_constraint_list -
 * @constrs: the list of active constraints
 *
 */
struct cpm_constraint_list {
	struct list_head constrs;
};

/**
 * struct cpm_platform_core -
 * @swms: the registered SWMs
 * @count: the number of available SWMs
 * @mux: the SWMs data mutex
 * @kobj: <swms> = <cpm>/swms
 * @constr_kattr: the sysfs attribute to read constratints from sysfs
 * @weight_kattr: the sysfs attribute to read policy weights from sysfs
 * @group: SWMSs group
 *
 */
struct cpm_platform_core {
	struct cpm_swm_core *swms;
	unsigned int count;
	struct mutex mux;
	struct kobject kobj;
	struct kobj_attribute constr_kattr;
	struct kobj_attribute weight_kattr;
	struct attribute_group group;
};

/**
 * struct cpm_governor_core -
 * @list: the list of available governors
 * @curr: the governor in use
 * @mux: the governor mutex
 *
 */
struct cpm_governor_core {
	struct list_head list;
	struct cpm_governor *curr;
	struct mutex mux;
};

/**
 * struct cpm_policy_core -
 * @list: the list of available governors
 * @curr: the policy in use
 * @mux: the governor mutex
 *
 */
struct cpm_policy_core {
	struct list_head list;
	struct cpm_policy *curr;
	struct mutex mux;
};

/**
 * struct cpm_ddp_core -
 * @notifier_list: the notifier list used for DDP
 * @mux: the DDP Mutex
 * @required: a new DDP is required
 *
 */
struct cpm_ddp_core {
	struct srcu_notifier_head notifier_list;
	struct mutex mux;
	short unsigned required:1;
};

/**
 * struct cpm_core_data -
 * @ddp: DDP data
 * @wq: work-queue for async tasks
 * @kobj: <cpm> = /sys/kernel/cpm
 * 	define if the CPM core is enabled (i.e. governors/policy and DDP will
 * 	be used.
 * @enabled: this is set when a constraint has been relaxed and thus FSC must
 * 	be searched starting from the head of the ordered FSC list
 * @relaxed: the current CPM running mode can be either granted or best-effort
 * @mode: the current CPM working mode which can be either
 * 	%CPM_MODE_BEST_EFFORT when running on QoSPM-like mode or
 * 	%CPM_MODE_GRANTED when enforcing FSC
 *
 */
struct cpm_core_data {
	struct cpm_ddp_core ddp;
	struct workqueue_struct *wq;
	struct kobject kobj;
	short unsigned enabled:1;
	short unsigned relaxed:1;
#define CPM_MODE_BEST_EFFORT	0
#define CPM_MODE_GRANTED	1
	short unsigned mode:1;
};

/*********************************************************************
 *                     FARWARD DECLARATIONS                          *
 *********************************************************************/

static int cpm_update_policy(void);
static int cpm_update_fsc(void);
static int cpm_notify_new_fsc(struct cpm_fsc_pointer *pnewfscp);
static int cpm_sysfs_core_swms_init(void);
static int cpm_sysfs_fscs_export(void);
static int cpm_sysfs_fscs_unexport(struct cpm_fsc_data *fscs);
static int cpm_sysfs_print_range(char *buf, ssize_t size,
				 struct cpm_range *range);
static int __cpm_update_constraint(void *entity, u8 type, u8 swm_id,
				   struct cpm_range *range);
static int __cpm_remove_constraint(void *entity, u8 type, u8 swm_id);

#define swm_id_is_valid(_id) (_id < plat.count)

/*********************************************************************
 *                     CORE DATA                                     *
 *********************************************************************/

/* Abstract System-Wide Metrics (exposed to user-space) */

static struct __initdata cpm_swm cpm_asm[] = {
	/* The maximum DAM latency [us] */
	CPM_PLATFORM_SWM("CPM_DMA_LATENCY", CPM_TYPE_LIB, CPM_USER_RW,
			CPM_COMPOSITION_RESTRICTIVE, 0, ~0),
	/* The maximum network lantecy [us] */
	CPM_PLATFORM_SWM("CPM_NETWORK_LATENCY", CPM_TYPE_LIB, CPM_USER_RW,
			CPM_COMPOSITION_RESTRICTIVE, 0, ~0),
	/* The network throughput [Mbps]*/
	CPM_PLATFORM_SWM("CPM_NETWORK_THROUGHPUT", CPM_TYPE_GIB, CPM_USER_RW,
			CPM_COMPOSITION_ADDITIVE, 0, 100000),
};

/* cpm_sysfs_ops - a generic sysfs ops dispatcher */
struct sysfs_ops cpm_sysfs_ops;

/*--- Platform ---*/
/* Platform data */
static struct cpm_platform_core plat = {
	.swms = 0,
	.count = 0,
	.mux = __MUTEX_INITIALIZER(plat.mux),
};

/* The ktype for platform data */
struct kobj_type cpm_plat_ktype;

/*--- Governors ---*/
/* Governors data */
static struct cpm_governor_core gov = {
	.list = LIST_HEAD_INIT(gov.list),
	.curr = 0,
	.mux = __MUTEX_INITIALIZER(gov.mux),
};

/*--- Policies ---*/
/* Policies data */
static struct cpm_policy_core pol = {
	.list = LIST_HEAD_INIT(pol.list),
	.curr = 0,
	.mux = __MUTEX_INITIALIZER(pol.mux),
};

/*--- Devices ---*/
/* The list of devices subscribed to some SWM */
static struct cpm_dev_list devs = {
	.list = LIST_HEAD_INIT(devs.list),
	.count = 0,
	.mux = __MUTEX_INITIALIZER(devs.mux),
};

/* The ktype of each device registerd to CPM */
struct kobj_type cpm_dev_ktype;

/*--- FSCs ---*/
/* The existing FSCs list */
static struct cpm_fsc_data *fscs;
/* The FSC's list mutex */
static DEFINE_MUTEX(fsc_mutex);
/* The ordered FSC list mutex */
static DEFINE_MUTEX(fsc_ordered_mutex);
/* The ktype of each FSC identified by CPM */
struct kobj_type cpm_fsc_ktype;

/*--- Constraints ---*/
/* The list of asserted constraints */
/*static struct cpm_constraint_list constr_list;*/

/*--- Core data ---*/
static struct cpm_core_data cpm = {
	.enabled = 0,
	.mode = CPM_MODE_BEST_EFFORT,
	.relaxed = 0,
	.ddp = {
		.mux = __MUTEX_INITIALIZER(cpm.ddp.mux),
		.required = 0,
		},
};

/* The ktype for CPM core */
struct kobj_type cpm_core_ktype;


/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/

#ifdef CONFIG_CPM_DEBUG
#define dprintk(msg...) pr_debug("cpm-core: " msg)
#else
#define dprintk(msg...) do {} while (0);
#endif

#define iprintk(msg...) pr_info("cpm-core: " msg)

#define nprintk(msg...) pr_notice("cpm-core: " msg)

#define wprintk(msg...) pr_warning("cpm-core: " msg)

#define eprintk(msg...) pr_err("cpm-core: " msg)


/******************************************************************************
 *   UTILITY FUNCTIONS                                                        *
 ******************************************************************************/

#define MIN(a, b) ((u32)(a) < (u32)(b) ? (a) : (b))
#define MAX(a, b) ((u32)(a) > (u32)(b) ? (a) : (b))

#define CPM_RANGE_OK		TRUE
#define CPM_RANGE_ERROR		FALSE

/**
 * cpm_verify_range() - verify if the specified value is within a range
 * @range: the range to consider for the check
 * @value: the value to check
 *
 * Return values: CPM_RANGE_ERROR if value is outside the range,
 * CPM_RANGE_OK otherwise.
 */
int cpm_verify_range(struct cpm_range *range, u32 value)
{
	switch (range->type) {
	case CPM_SWM_TYPE_UNBOUNDED:
		break;
	case CPM_SWM_TYPE_RANGE:
		if (value > range->upper)
			return CPM_RANGE_ERROR;
		if (value < range->lower)
			return CPM_RANGE_ERROR;
		break;
	case CPM_SWM_TYPE_LBOUND:
		if (value < range->lower)
			return CPM_RANGE_ERROR;
		break;
	case CPM_SWM_TYPE_UBOUND:
		if (value > range->upper)
			return CPM_RANGE_ERROR;
		break;
	}
	return CPM_RANGE_OK;
}
EXPORT_SYMBOL(cpm_verify_range);

/**
 * cpm_merge_range() - merge two cpm_range
 * @first: first cpm_range, also used to return the result
 * @second: second cpm_range to be merged with the first
 *
 * Return values: -EINVAL if the two ranges can't be merged, 0 if merge is ok
 */
int cpm_merge_range(struct cpm_range *first, struct cpm_range *second)
{
	int ret = 0;

	if (unlikely(second->type == CPM_SWM_TYPE_UNBOUNDED))
		return 0;

	switch (first->type) {
	case CPM_SWM_TYPE_UNBOUNDED:
		(*first) = (*second);
		break;

	case CPM_SWM_TYPE_RANGE:
		if (first->lower < first->upper) {
			/* first is a range */
			switch (second->type) {
			case CPM_SWM_TYPE_RANGE:
				if (second->lower < second->upper) {
					/* second is a range */
					if ((first->lower > second->upper) ||
					    (first->upper < second->lower)) {
						ret = -EINVAL;
					} else {
						first->lower =
						    MAX(first->lower,
							second->lower);
						first->upper =
						    MIN(first->upper,
							second->upper);
					}
				} else {
					/* second is a single value */
					if (cpm_verify_range
					    (first,
					     second->lower) ==
					    CPM_RANGE_ERROR) {
						ret = -EINVAL;
					} else {
						(*first) = (*second);
					}
				}
				break;
			case CPM_SWM_TYPE_LBOUND:
				if (cpm_verify_range(first, second->lower) ==
				    CPM_RANGE_ERROR) {
					if (first->upper < second->lower)
						ret = -EINVAL;
				} else {
					first->lower = second->lower;
				}
				break;
			case CPM_SWM_TYPE_UBOUND:
				if (cpm_verify_range(first, second->upper) ==
				    CPM_RANGE_ERROR) {
					if (first->lower > second->upper)
						ret = -EINVAL;

				} else {
					first->upper = second->upper;
				}
				break;
			}
		} else {
			/* first is a single value */
			switch (second->type) {
			case CPM_SWM_TYPE_RANGE:
				if (second->lower < second->upper) {
					/* second is a range */
					if (cpm_verify_range
					    (second,
					     first->lower) == CPM_RANGE_ERROR)
						ret = -EINVAL;
				} else {
					/* second is a single value */
					if (first->lower != second->lower)
						ret = -EINVAL;
				}
				break;
			case CPM_SWM_TYPE_LBOUND:
			case CPM_SWM_TYPE_UBOUND:
				if (cpm_verify_range(second, first->lower) ==
				    CPM_RANGE_ERROR)
					ret = -EINVAL;
				break;
			}
		}
		break;

	case CPM_SWM_TYPE_LBOUND:
		switch (second->type) {
		case CPM_SWM_TYPE_RANGE:
			if (second->lower < second->upper) {
				/* second is a range */
				if (cpm_verify_range(first, second->upper)
				    == CPM_RANGE_ERROR) {
					ret = -EINVAL;
				} else {
					first->lower =
					    MAX(first->lower, second->lower);
					first->upper = second->upper;
					first->type = CPM_SWM_TYPE_RANGE;
				}
				break;

			} else {
				/* second is a single value */
				if (cpm_verify_range(first, second->lower)
				    == CPM_RANGE_ERROR) {
					ret = -EINVAL;
				} else {
					(*first) = (*second);
				}
			}
			break;
		case CPM_SWM_TYPE_LBOUND:
			first->lower = MAX(first->lower, second->lower);
			break;
		case CPM_SWM_TYPE_UBOUND:
			if (cpm_verify_range(first, second->upper)
			    == CPM_RANGE_ERROR) {
				ret = -EINVAL;
			} else {
				first->upper = second->upper;
				first->type = CPM_SWM_TYPE_RANGE;
			}
			break;
		}
		break;

	case CPM_SWM_TYPE_UBOUND:
		switch (second->type) {
		case CPM_SWM_TYPE_RANGE:
			if (second->lower < second->upper) {
				/* second is a range */
				if (cpm_verify_range(first, second->lower)
				    == CPM_RANGE_ERROR) {
					ret = -EINVAL;
				} else {
					first->upper =
					    MIN(first->upper, second->upper);
					first->lower = second->lower;
					first->type = CPM_SWM_TYPE_RANGE;
				}
				break;
			} else {
				/* first is a single value */
				if (cpm_verify_range(first, second->lower)
				    == CPM_RANGE_ERROR) {
					ret = -EINVAL;
				} else {
					(*first) = (*second);
				}
			}
			break;
		case CPM_SWM_TYPE_LBOUND:
			if (cpm_verify_range(first, second->lower)
			    == CPM_RANGE_ERROR) {
				ret = -EINVAL;
			} else {
				first->lower = second->lower;
				first->type = CPM_SWM_TYPE_RANGE;
			}
			break;
		case CPM_SWM_TYPE_UBOUND:
			first->upper = MIN(first->upper, second->upper);
			break;
		}
	}

	return ret;

}
EXPORT_SYMBOL(cpm_merge_range);

static inline u32 __cpm_get_max(struct cpm_range *range, u8 swm_id)
{

	if ((range->type == CPM_SWM_TYPE_UNBOUNDED) ||
	    (range->type == CPM_SWM_TYPE_LBOUND))
		return plat.swms[swm_id].info.max;

	return range->upper;

}

static inline u32 __cpm_get_min(struct cpm_range *range, u8 swm_id)
{

	if ((range->type == CPM_SWM_TYPE_UNBOUNDED) ||
	    (range->type == CPM_SWM_TYPE_UBOUND))
		return plat.swms[swm_id].info.min;

	return range->lower;

}

/**
 * cpm_weight_range() - weight a range on the specified SWM
 * @range: the range to weight
 * @swm_id: the SWM to consider
 * @weight: the weight factor to use
 *
 */
int cpm_weight_range(struct cpm_range *range, u8 swm_id, s32 *weight)
{
	u32 value;

	if (swm_id >= plat.count) {
		eprintk("weighting unexisting SWM\n");
		return -EINVAL;
	}

	*weight = plat.swms[swm_id].info.weight;

	/* small speed optimization */
	if (unlikely(!*weight))
		return 0;

	if (*weight >= 0)
		value = __cpm_get_max(range, swm_id);
	else
		value = __cpm_get_min(range, swm_id);

	*weight *= value;

	return 0;

}
EXPORT_SYMBOL(cpm_weight_range);

static ssize_t __cpm_entity_name(void *ptr, u8 type, char *buf, ssize_t count)
{

	if (type == CPM_ENTITY_TYPE_DRIVER) {
		count =
		    snprintf(buf, count, "dev:%s",
			     dev_name((struct device *)ptr));
		return count;
	}

	count = snprintf(buf, count, "app:%d:%s",
			 ((struct task_struct *)ptr)->pid,
			 ((struct task_struct *)ptr)->comm);
	return count;
}

/******************************************************************************
 *   CPM PLATFORM                                                             *
 ******************************************************************************/

/**
 * cpm_register_platform() - register a set of platform SWMs
 * @cpd: the array of platform SWMs to register
 *
 */
int cpm_register_platform(struct cpm_platform_data *cpd)
{
	int i, swm_count, result;
	struct cpm_swm *pswm;

	if (!cpd) {
		eprintk("invalid platform data\n");
		return -EINVAL;
	}

	mutex_lock(&plat.mux);

	/* NOTE platform data can be defined only one time */
	if (plat.swms) {
		eprintk("platform data already defined\n");
		result = -EBUSY;
		goto cpm_plat_swm_exist;
	}

	/* Allocate a unique vector to host ASM+PSM */
	swm_count = CPM_ASM_COUNT + cpd->count;
	plat.swms = kzalloc( swm_count * sizeof(struct cpm_swm_core),
			GFP_KERNEL);
	if (!plat.swms) {
		eprintk("out-of-mem on platform data allocation\n");
		result = -ENOMEM;
		goto cpm_platform_swm_nomem;
	}

	/* Copy ASM metrics */
	for (i = 0; i < CPM_ASM_COUNT; i++) {
		plat.swms[i].id = i;
		plat.swms[i].info = cpm_asm[i];
		plat.swms[i].cur_range.lower = cpm_asm[i].min;
		plat.swms[i].cur_range.upper = cpm_asm[i].max;
		INIT_LIST_HEAD(&(plat.swms[i].constraints));
	}

	/* Adding PSM metrics */
	pswm = cpd->swms;
	for (i = 0; i < cpd->count; i++) {
		plat.swms[CPM_ASM_COUNT+i].id = CPM_ASM_COUNT+i;
		plat.swms[CPM_ASM_COUNT+i].info = (*pswm);
		plat.swms[CPM_ASM_COUNT+i].cur_range.lower = pswm->min;
		plat.swms[CPM_ASM_COUNT+i].cur_range.upper = pswm->max;
		INIT_LIST_HEAD(&(plat.swms[CPM_ASM_COUNT+i].constraints));
		pswm++;
	}
	plat.count = CPM_ASM_COUNT+i;

	mutex_unlock(&plat.mux);

	dprintk("platform data registered, %u SWM defined, "
			"(%u ASMs, %u PSMs)\n",
			plat.count, CPM_ASM_COUNT, i);

	cpm_sysfs_core_swms_init();

	return 0;

cpm_plat_swm_exist:
cpm_platform_swm_nomem:
	mutex_unlock(&plat.mux);

	return result;

}
EXPORT_SYMBOL(cpm_register_platform);

/******************************************************************************
 *				CPM GOVERNORS                                 *
 ******************************************************************************/

/**
 * cpm_get_new_fsc() - build a new FSC data structure
 *
 * This method should be used by governors in order to get a valid reference
 * to a new FSC data structure.
 *
 */
struct cpm_fsc *cpm_get_new_fsc(void)
{
	struct cpm_fsc_core *pcfsc;

	dprintk("allocate a new FSC\n");

	pcfsc = kzalloc(sizeof(struct cpm_fsc_core),
			GFP_KERNEL);
	if (unlikely(!pcfsc)) {
		eprintk("out-of-memory on cpm_fsc_core alloation\n");
		return 0;
	}

	return &(pcfsc->info);

}
EXPORT_SYMBOL(cpm_get_new_fsc);

static int cpm_fscs_init(void)
{
	int result = 0;

	mutex_lock(&fsc_mutex);

	/* Releasing previous FSC */
	if (fscs) {

		dprintk("pivot FSC list\n");

		/* Mark invalid current FSCs list */
		fscs->valid = 0;
		cpm.ddp.required = 0;

		/* Release sysfs entries */
		cpm_sysfs_fscs_unexport(fscs);

		fscs = 0;

	}

	if (unlikely(!cpm.enabled)) {
		dprintk("cpm disabled, updating FSC list aborted\n");
		result = -EAGAIN;
		goto out_sysfs_fsc_disabled;
	}

	/* Build new FSCs list */
	fscs = kzalloc(sizeof(struct cpm_fsc_data),
			GFP_KERNEL);
	if (!fscs) {
		eprintk("new FSC list building failed, out-of-mem\n");
		result = -ENOMEM;
		goto out_sysfs_fsc_mem;
	}

	INIT_LIST_HEAD(&fscs->found);
	INIT_LIST_HEAD(&fscs->ordered);
	mutex_init(&fscs->omux);
	fscs->valid = 1;

	/* create the "fscs" sysfs' entry under /sys/kernel/cpm */
	result =
	    kobject_init_and_add(&fscs->kobj, &cpm_fsc_ktype, &cpm.kobj,
				 "fscs");
	if (result)
		eprintk("sysfs - fscs interface creation failed\n");

out_sysfs_fsc_disabled:
out_sysfs_fsc_mem:

	mutex_unlock(&fsc_mutex);

	return result;
}

static void cpm_work_update_fsc(struct work_struct *work)
{
	int result = 0;
	struct kobject *kobj;
	struct cpm_fsc_data *_fscs;
	struct timespec start, end, delta;

	mutex_lock(&fsc_mutex);

	if (unlikely(!fscs)) {
		dprintk("unavailable FSC list, governor update aborted\n");
		goto out_gov_work_fsc;
	}

	_fscs = fscs;

	if (unlikely(_fscs->updated)) {
		nprintk("FSC list is up-to-date, aborting governor call\n");
		goto out_gov_work_fsc;
	}

	/* Getting a refcount to current FSC list */
	kobj = kobject_get(&_fscs->kobj);
	if (unlikely(!kobj)) {
		nprintk("invalid FSCs list, FSC search aborted\n");
		goto out_gov_work_fsc;
	}

	mutex_unlock(&fsc_mutex);

	dprintk("searching_start: FSCs searching wq, governor [%s]\n",
		gov.curr->name);

	mutex_lock(&devs.mux);
	getrawmonotonic(&start);
	result = gov.curr->build_fsc_list(&devs.list, devs.count);
	getrawmonotonic(&end);
	mutex_unlock(&devs.mux);

	dprintk("searching_end\n");

	delta = timespec_sub(end, start);
	iprintk("EX-GOV, %d %d %03ld.%09ld\n", devs.count,
		fscs ? fscs->count : -1, delta.tv_sec, delta.tv_nsec);

	/* Releasing refcount */
	kobject_put(&_fscs->kobj);

	if (result) {
		eprintk("FSCs searching failed\n");
		/* TODO: Switch to Best-Effort approach */
	}

out_gov_work_fsc:

	mutex_unlock(&fsc_mutex);
	return;

}

DECLARE_WORK(cpm_work_governor, cpm_work_update_fsc);

static int cpm_update_governor(void)
{
	int result = 0;

	/* return immediatly if CPM is disabled */
	if (unlikely(!cpm.enabled)) {
		dprintk("CPM disabled, governor update disabled\n");
		return 0;
	}

	mutex_lock(&gov.mux);

	/* a permission error is returned if the goeverno has not been
	 * configured */
	if (unlikely(!gov.curr)) {
		eprintk("governor not configured: update failed\n");
		result = -EPERM;
		goto out_gov_update;
	}

	/* TODO use kobject to better manage governor countref */

	/* let the governor do the job... */
	result = queue_work(cpm.wq, &cpm_work_governor);
	if (!result)
		eprintk("queuing new governor's work failed\n");

	result = 0;		/* still return no-error to caller */

out_gov_update:

	mutex_unlock(&gov.mux);

	return result;

}

static int cpm_invalidate_fsc(void)
{
	int result = 0;

	dprintk("invalidating FSC list...\n");

	/* Build new fsc */
	result = cpm_fscs_init();
	if (result)
		return result;

	/* Calling governor for building new FSC list */
	cpm_update_governor();

	return result;

}

/**
 * cpm_register_governor() - register a governor into CPM
 * @cg: a reference to the governor to register
 *
 */
int cpm_register_governor(struct cpm_governor *cg)
{

	if (unlikely(!cg)) {
		nprintk("trying to register invalid governor\n");
		return -EINVAL;
	}

	mutex_lock(&gov.mux);

	/* TODO add governor list support */

	gov.curr = cg;

	mutex_unlock(&gov.mux);

	iprintk("new governor [%s] registered\n",
		(cg->name[0]) ? cg->name : "UNNAMED");

	/* Force FSC list rebuilding */
	cpm_invalidate_fsc();

	return 0;

}
EXPORT_SYMBOL(cpm_register_governor);

/**
 * cpm_set_fsc_list() - register the list of available FSC
 * @nfl: the FSC list to register
 *
 * This method should be used by governors in order to notify the core
 * about a new list of available system FSC.
 *
 */
int cpm_set_fsc_list(struct list_head *nfl)
{
	struct list_head *node;
	int result = 0;

	if (unlikely(list_empty(nfl))) {
		eprintk("empty FSC list: falling back to best-effort policy\n");
		/* TODO disable CPM and fall-back to best-effort policy with
		 * only constraints aggretation and notification...
		 */
		goto out_fsc_set_empty;
	}

	mutex_lock(&fsc_mutex);

	/* updating current FSC list */
	list_replace_init(nfl, &fscs->found);

	/* updating the FSC count */
	fscs->count = 0;
	list_for_each(node, &fscs->found) {
		fscs->count++;
	}

	dprintk("new FSC list received, [%d] entry\n", fscs->count);

	/* updating ordered FSC list */
	cpm_update_policy();

	dprintk("Updating FSC sysfs interface\n");
	cpm_sysfs_fscs_export();

	mutex_unlock(&fsc_mutex);

out_fsc_set_empty:

	return result;

}
EXPORT_SYMBOL(cpm_set_fsc_list);

/******************************************************************************
 *				CPM POLICIES				      *
 ******************************************************************************/

/**
 * cpm_register_policy() - register a policy into CPM
 * @cp: the policy to register
 *
 */
int cpm_register_policy(struct cpm_policy *cp)
{

	if (unlikely(!cp)) {
		nprintk("trying yo reigster invalid policy\n");
		return -EINVAL;
	}

	mutex_lock(&pol.mux);

	/* TODO add policy list support */

	pol.curr = cp;

	mutex_unlock(&pol.mux);

	iprintk("new policy [%s] registered\n",
		(cp->name[0]) ? cp->name : "UNNAMED");

	/* updating ordered FSC list */
	cpm_update_policy();

	return 0;
}
EXPORT_SYMBOL(cpm_register_policy);

static void cpm_work_order_fsc(struct work_struct *work)
{
	int result = 0;
	struct kobject *kobj;
	struct cpm_fsc_data *_fscs;

	mutex_lock(&fsc_mutex);

	if (unlikely(!fscs)) {
		dprintk("unavailable FSC list, policy update aborted\n");
		goto out_pol_work_fsc;
	}

	_fscs = fscs;

	/* Getting a refcount to current FSC list */
	kobj = kobject_get(&_fscs->kobj);
	if (unlikely(!kobj)) {
		nprintk("invalid FSCs list, FSC ordering aborted\n");
		goto out_pol_work_fsc;
	}

	mutex_unlock(&fsc_mutex);

	mutex_lock(&devs.mux);

	dprintk("ordering_start - FSCs ordering wq, policy [%s]\n",
		pol.curr->name);
	result = pol.curr->sort_fsc_list(&_fscs->found);
	dprintk("ordering_end\n");

	mutex_unlock(&devs.mux);

	/* Releasing refcount */
	kobject_put(&_fscs->kobj);

	if (result) {
		eprintk("FSCs ordering failed\n");
		/* TODO Switch to Best-Effort approach */
	}

	return;

out_pol_work_fsc:

	mutex_unlock(&fsc_mutex);
	return;

}

DECLARE_WORK(cpm_work_policy, cpm_work_order_fsc);

static int cpm_update_policy(void)
{
	int result = 0;

	/* return immediatly if CPM is disabled */
	if (unlikely(!cpm.enabled)) {
		dprintk("CPM disabled, governor update disabled\n");
		return 0;
	}

	mutex_lock(&pol.mux);

	if (unlikely(!pol.curr)) {
		dprintk
		    ("policy not defined: unable to obtain ordered_fsc_list\n");
		result = -EINVAL;
		goto out_pol_update_inval;
	}

	/* TODO use kobject to better manage policy countref */

	/* let the policy do the job... */
	result = queue_work(cpm.wq, &cpm_work_policy);
	if (!result)
		eprintk("queuing new policy's work failed\n");

	result = 0;		/* still return no-error to caller */

out_pol_update_inval:

	mutex_unlock(&pol.mux);

	return result;

}

/**
 * cpm_set_ordered_fsc_list() - register the list of ordered FSC
 * @nofl: the ordered list of FSC
 *
 * This method should be used by policies in order to notify the core
 * about a new list of FSC ordered according to the policy'optimization
 * strategy.
 *
 */
int cpm_set_ordered_fsc_list(struct list_head *nofl)
{
	struct cpm_fsc_pointer *pofsc;
	struct cpm_fsc_pointer *pnext;
	int result;

	if (unlikely(!nofl)) {
		eprintk("trying to set empty ordered FSC list\n");
		return -EINVAL;
	}

	/* Forbit FSC list updated during a DDP */
	mutex_lock(&fsc_mutex);
	mutex_lock(&fscs->omux);

	/* Clean-up old list */
	dprintk("cleaning-up old ordered FSC list\n");
	list_for_each_entry_safe(pofsc, pnext, &fscs->ordered, node) {
		list_del(&pofsc->node);
		kfree(pofsc);
	}

	/* Update the new ordered FSC list */
	list_replace_init(nofl, &fscs->ordered);

	mutex_unlock(&fscs->omux);
	mutex_unlock(&fsc_mutex);

	dprintk("new FSC ordered list registered\n");

	/* updating current fsc */
	fscs->updated = 0;
	fscs->curr = 0;
	result = cpm_update_fsc();
	if (result)
		eprintk("FSC update failed\n");

	return 0;
}
EXPORT_SYMBOL(cpm_set_ordered_fsc_list);

/************************************************************************
 *	SYSFS INTERFACE							*
 ************************************************************************/

#define CPM_KATTR_RO(_kattr, _name, _show)		\
	do {						\
		(_kattr)->attr.name = _name;		\
		(_kattr)->attr.mode = 0444;		\
		(_kattr)->show = _show;			\
		(_kattr)->store = NULL;			\
	} while (0);
#define CPM_KATTR_RW(_kattr, _name, _show, _store)	\
	do {						\
		(_kattr)->attr.name = _name;		\
		(_kattr)->attr.mode = 0664;		\
		(_kattr)->show = _show;			\
		(_kattr)->store = _store;		\
	} while (0);
#define	CPM_ADD_ATTR(_pattrs, _kattr)			\
	do {						\
		*(_pattrs) = &((_kattr).attr);		\
		(_pattrs)++;				\
	} while (0);

static void cpm_sysfs_core_release(struct kobject *kobj)
{
	/*struct cpm_core_data *pcd; */

	dprintk("sysfs [%s] - (TODO) cleaning-up all cpm core memory\n",
		kobject_name(kobj));

}

static void cpm_sysfs_plat_release(struct kobject *kobj)
{
	/*struct cpm_platform_core *ppc; */

	dprintk("sysfs [%s] - (TODO) cleaning-up all platform data memory\n",
		kobject_name(kobj));

}

static void cpm_sysfs_fsc_release(struct kobject *kobj)
{
	struct cpm_fsc_data *fscs;
	struct cpm_fsc_core *pcfsc, *pncfsc;
	struct cpm_fsc_pointer *pofsc, *pnofsc;

	dprintk("sysfs [%s] - cpm_sysfs_fsc_release\n", kobject_name(kobj));

	if (strcmp(kobject_name(kobj), "fscs") == 0) {

		dprintk("sysfs - cleaning-up all FSCs list memory\n");

		fscs = container_of(kobj, struct cpm_fsc_data, kobj);

		/* Cleaning-up ordered FSC list */
		list_for_each_entry_safe(pofsc, pnofsc,
				&fscs->ordered, node) {

			dprintk("sysfs - cleaning-up ordered fsc [FSC%05u]\n",
				pofsc->fsc->id);

			list_del(&pofsc->node);
			kfree(pofsc);
		}

		/* Cleaning-up all existing FSC */
		list_for_each_entry_safe(pcfsc, pncfsc,
				&fscs->found, info.node) {

			dprintk("sysfs - cleaning-up fsc [FSC%05u]\n",
				pcfsc->info.id);

			/* Cleaning-up cpm_fsc data */
			kfree(pcfsc->info.swms);
			kfree(pcfsc->info.dwrs);
			kfree(pcfsc->info.gov_data);
			kfree(pcfsc->info.pol_data);
			kfree(pcfsc->swms_group.attrs);
			kfree(pcfsc->dwrs_group.attrs);

			/* Removing list entry */
			list_del(&pcfsc->info.node);

			/* Cleaning-up FSC */
			kfree(pcfsc);

		}

		/* Cleaning-up cpm_fsc_data */
		kfree(fscs);

	}

}

static void cpm_sysfs_dev_release(struct kobject *kobj)
{
	struct cpm_dev_core *pcd;
	struct cpm_dev_dwr *pdwr;
	u8 i;

	dprintk("sysfs [%s] - cpm_sysfs_dev_release\n", kobject_name(kobj));

	if (strcmp(kobject_name(kobj), "cpm") == 0) {

		/* Releasing all core device data */
		pcd = container_of(kobj, struct cpm_dev_core, sysfs.cpm);

		dprintk("sysfs - cleaning-up all device [%s] memory\n",
			dev_name(pcd->dev_info.dev));

		/* releasing attributes for each DWR */
		for (pdwr = pcd->dev_info.dwrs, i = 0;
		     i < pcd->dev_info.dwrs_count; pdwr++, i++) {
			kfree(pdwr->swms_group.attrs);
			kfree(pdwr->swms);

		}
		kfree(pcd->dev_info.dwrs);
		kfree(pcd);

	}

}

static ssize_t cpm_sysfs_show(struct kobject *kobj, struct attribute *attr,
			      char *buf)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		ret = kattr->show(kobj, kattr, buf);

	return ret;
}

static ssize_t cpm_sysfs_store(struct kobject *kobj, struct attribute *attr,
			       const char *buf, size_t count)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->store)
		ret = kattr->store(kobj, kattr, buf, count);

	return ret;
}

static int cpm_sysfs_dev_init(struct cpm_dev_core *pcd)
{
	int result = 0;

	/* create the <device>/cpm folder */
	result = kobject_init_and_add(&((pcd->sysfs).cpm),
				      &cpm_dev_ktype,
				      &(((pcd->dev_info).dev)->kobj), "cpm");

	/* create the <device>/cpm/dwrs folder */
	result = kobject_init_and_add(&((pcd->sysfs).dwrs),
				      &cpm_dev_ktype, &((pcd->sysfs).cpm),
				      "dwrs");

	/* create the <device>/cpm/constraints folder */
	result = kobject_init_and_add(&((pcd->sysfs).constraints),
				      &cpm_dev_ktype, &((pcd->sysfs).cpm),
				      "constraints");

	return result;
}

static int cpm_sysfs_print_range(char *buf, ssize_t size,
				 struct cpm_range *range)
{
	int count = 0;

	switch (range->type) {
	case CPM_SWM_TYPE_UNBOUNDED:
		count = snprintf(buf, size, "UnB");
		break;
	case CPM_SWM_TYPE_RANGE:
		if (range->lower < range->upper) {
			count = snprintf(buf, size, "R %4u %4u",
					 range->lower, range->upper);
		} else {
			count = snprintf(buf, size, "S %4u", range->lower);
		}
		break;
	case CPM_SWM_TYPE_LBOUND:
		count = snprintf(buf, size, "LB %4u", range->lower);
		break;
	case CPM_SWM_TYPE_UBOUND:
		count = snprintf(buf, size, "UB %4u", range->upper);
		break;
	}

	return count;
}

static ssize_t cpm_swm_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	struct cpm_swm_range *swm_range;
	ssize_t count = 0;

	swm_range = container_of(attr, struct cpm_swm_range, kattr);

	count = sprintf(buf, "%d:%s",
			swm_range->id, plat.swms[swm_range->id].info.name);

	count += sprintf(buf + count, " %c",
			 plat.swms[swm_range->id].info.userw ? 'w' : '-');

	switch (swm_range->range.type) {
	case CPM_SWM_TYPE_UNBOUNDED:
		count += sprintf(buf + count, " UnB\n");
		break;
	case CPM_SWM_TYPE_RANGE:
		if (swm_range->range.lower < swm_range->range.upper) {
			count += sprintf(buf + count, " R %4u %4u\n",
					 swm_range->range.lower,
					 swm_range->range.upper);
		} else {
			count += sprintf(buf + count, " S %4u\n",
					 swm_range->range.lower);
		}
		break;
	case CPM_SWM_TYPE_LBOUND:
		count += sprintf(buf + count, " LB %4u\n",
				 swm_range->range.lower);
		break;
	case CPM_SWM_TYPE_UBOUND:
		count += sprintf(buf + count, " UB %4u\n",
				 swm_range->range.upper);
		break;
	}

	return count;
}

static ssize_t cpm_dwr_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	const struct cpm_dev_dwr *dwr;
	ssize_t status;

	dwr = container_of(attr, struct cpm_dev_dwr, kattr);

	status = sprintf(buf, "%s\n", dwr->name);

	return status;
}

static int cpm_sysfs_dev_export(struct cpm_dev_core *pcd)
{
	struct cpm_dev_dwr *pdwr;
	struct cpm_swm_range *pswm;
	struct attribute **pattrs;
	int result = 0;
	u8 i, j;

	/* Init sysfs for this device */
	cpm_sysfs_dev_init(pcd);

	/* create attributes for each DWR */
	for (pdwr = pcd->dev_info.dwrs, i = 0; i < pcd->dev_info.dwrs_count;
	     pdwr++, i++) {

		/* build the attribute_group array for the SWM of this DWR */
		pdwr->swms_group.name = pdwr->name;
		pdwr->swms_group.attrs =
			pattrs = kzalloc(((pdwr->swms_count) + 2) *
						 sizeof(struct attribute *),
						 GFP_KERNEL);
		if (!pattrs) {
			dprintk("out-of-mem on cpm_dev_block allocation\n");
			result = -ENOMEM;
			goto crd_exit_nomem_dwr;
		}

		/* adding the DWR name attribute */
		CPM_KATTR_RO(&(pdwr->kattr), "name", cpm_dwr_show);
		CPM_ADD_ATTR(pattrs, pdwr->kattr);

		/* scanning a DWR's SWMs and adding attributes */
		pswm = pdwr->swms;
		for (j = 0; j < pdwr->swms_count; j++) {
			sprintf(pswm->name, "swm%02d", pswm->id);
			CPM_KATTR_RO(&(pswm->kattr), pswm->name, cpm_swm_show);
			CPM_ADD_ATTR(pattrs, pswm->kattr);
			pswm++;
		}

		/* complete attribute group definition */
		(*pattrs) = NULL;

		/* export this DWR to sysfs */
		result =
		    sysfs_create_group(&(pcd->sysfs.dwrs), &(pdwr->swms_group));
		if (result) {
			dprintk("DWR exporting failed\n");
			goto crd_exit_sysfs_dwr_failed;
		}

	}

	return 0;

crd_exit_sysfs_dwr_failed:

crd_exit_nomem_dwr:

	/* TODO add free-up code */
	dprintk("%s:%d memory-leak: free-up allocated structures\n",
		__func__,  __LINE__);

	return result;

}

static int cpm_sysfs_dev_unexport(struct cpm_dev_core *pcd)
{
	struct cpm_dev_dwr *pdwr;
	u8 i;

	/* releasing attributes for each DWR */
	for (pdwr = pcd->dev_info.dwrs, i = 0; i < pcd->dev_info.dwrs_count;
	     pdwr++, i++) {
		sysfs_remove_group(&(pcd->sysfs.dwrs), &(pdwr->swms_group));
	}

	/* Releasing folders kobjects */
	kobject_put(&((pcd->sysfs).constraints));
	kobject_put(&((pcd->sysfs).dwrs));
	kobject_put(&((pcd->sysfs).cpm));

	return 0;

}

/*--- CPM Core SYSFS interface ---*/

static ssize_t cpm_sysfs_core_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	ssize_t count = 0;

	if (strcmp(attr->attr.name, "enable") == 0)
		count = sprintf(buf, "%d\n", cpm.enabled);

	return count;
}

static ssize_t cpm_sysfs_core_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	short unsigned value;

	if (strcmp(attr->attr.name, "enable") == 0) {
		sscanf(buf, "%hu", &value);
		if (value) {
			cpm.enabled = 1;
			iprintk("CPM enabled\n");
			if (!fscs || !fscs->updated)
				cpm_invalidate_fsc();
		} else {
			cpm.enabled = 0;
			iprintk("CPM disabled\n");
		}
	}

	return count;
}

static struct kobj_attribute cpm_sysfs_core_enable_attr =
__ATTR(enable, 0666, cpm_sysfs_core_show, cpm_sysfs_core_store);

static struct attribute *cpm_sysfs_core_attrs[] = {
	&cpm_sysfs_core_enable_attr.attr,
	NULL,
};

static ssize_t cpm_sysfs_core_swms_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct cpm_swm_core *pcswm;

	pcswm = container_of(attr, struct cpm_swm_core, kattr);

	count =
	    sprintf(buf, "%02u:%-10s %c %c %6u %6u", pcswm->id,
		    pcswm->info.name,
		    (pcswm->info.type == CPM_TYPE_LIB) ? 'L' : 'G',
		    (pcswm->info.comp == CPM_COMPOSITION_ADDITIVE) ? 'A' : 'R',
		    pcswm->info.min, pcswm->info.max);

	count += sprintf(buf + count, " %c", pcswm->info.userw ? 'w' : '-');

	switch (pcswm->cur_range.type) {
	case CPM_SWM_TYPE_UNBOUNDED:
		count += sprintf(buf + count, " UnB\n");
		break;
	case CPM_SWM_TYPE_RANGE:
		if (pcswm->cur_range.lower < pcswm->cur_range.upper) {
			count += sprintf(buf + count, " R %6u %6u\n",
					 pcswm->cur_range.lower,
					 pcswm->cur_range.upper);
		} else {
			count += sprintf(buf + count, " S %6u\n",
					 pcswm->cur_range.lower);
		}
		break;
	case CPM_SWM_TYPE_LBOUND:
		count += sprintf(buf + count, " LB %6u\n",
				 pcswm->cur_range.lower);
		break;
	case CPM_SWM_TYPE_UBOUND:
		count += sprintf(buf + count, " UB %6u\n",
				 pcswm->cur_range.upper);
		break;
	}

	return count;
}

static ssize_t cpm_sysfs_core_swms_constr_store(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t count)
{
	int swm_id = 0;
	int value = 0;
	int result = 0;
	struct cpm_swm_core *pswm;
	struct cpm_range range;
	char entity_name[32];

	__cpm_entity_name((void *)current, CPM_ENTITY_TYPE_TASK, entity_name,
			  32);

	dprintk("constraint assertion from task [%s]\n", entity_name);

	/* Getting user value */
	result = sscanf(buf, "%d:%d", &swm_id, &value);

	if (unlikely(!result)) {
		eprintk("invalid format on SWMs constraint (task [%s])\n",
			entity_name);
		return -EINVAL;
	}

	dprintk("result: %d, swm_id: %d, value: %d\n", result, swm_id, value);

	mutex_lock(&plat.mux);

	if (unlikely(!swm_id_is_valid(swm_id))) {
		eprintk("invalid id on SWMs constraint (task [%s])\n",
			entity_name);
		result = -EINVAL;
		goto out;
	}

	pswm = &plat.swms[swm_id];

	/* Checking if the required SWM can be configured from user-space */
	if (unlikely(!pswm->info.userw)) {
		eprintk
		    ("forbidden constraint assertion on SWMs [%s] "
		     "from user-space (task [%s])\n",
		     pswm->info.name, entity_name);
		result = -EPERM;
		goto out;
	}

	if (result == 1) {
		/* An empty write on sysfs attribute will remove the
		 * the eventually caller asserted constraint on that SWM */
		dprintk("constraint deassertion\n");
		result =
		    __cpm_remove_constraint((void *)current,
					    CPM_ENTITY_TYPE_TASK, swm_id);
		goto out;
	}

	if (pswm->info.comp == CPM_COMPOSITION_ADDITIVE) {
		dprintk("additive constraint assertion\n");
		range.lower = value;
		range.upper = pswm->info.max;
		range.type = CPM_SWM_TYPE_LBOUND;
	} else {
		dprintk("restrictive constraint assertion\n");
		range.lower = pswm->info.min;
		range.upper = value;
		range.type = CPM_SWM_TYPE_UBOUND;
	}

	if (range.lower > range.upper) {
		eprintk
		    ("out-of-range constraint asserted by [%s] on SWM [%s]\n",
		     entity_name, pswm->info.name);
		result = -EINVAL;
		goto out;
	}

	/* Try to add the constraints */
	result =
	    __cpm_update_constraint((void *)current, CPM_ENTITY_TYPE_TASK,
				    swm_id, &range);

out:

	mutex_unlock(&plat.mux);

	if (result)
		dprintk("%s: status %d\n", __func__, result);

	return result ? : count;

}

static ssize_t cpm_sysfs_core_swms_weight_store(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t count)
{
	int swm_id = 0;
	s32 value = 0;
	int result = 0;

	dprintk("new SWM weight assertion\n");

	/* Getting user value */
	result = sscanf(buf, "%d:%d", &swm_id, &value);
	if (unlikely(!result || result < 2)) {
		eprintk("invalid SWM weight format\n");
		return -EINVAL;
	}

	dprintk("result: %d, swm_id: %d, value: %d\n", result, swm_id, value);

	/* weight sanity check */
	if (value > CPM_SWM_MAX_WEIGHT) {
		nprintk
		    ("failed asserting weight [%d] on SWM [%d], "
		     "exceeding max value [%d]\n",
		     value, swm_id, CPM_SWM_MAX_WEIGHT);
		return -EINVAL;
	}
	if (value < CPM_SWM_MIN_WEIGHT) {
		nprintk
		    ("failed asserting weight [%d] on SWM [%d], "
		     "exceeding min value [%d]\n",
		     value, swm_id, CPM_SWM_MIN_WEIGHT);
		return -EINVAL;
	}

	mutex_lock(&plat.mux);

	if (unlikely(!swm_id_is_valid(swm_id))) {
		eprintk("invalid id on SWMs constraint\n");
		result = -EINVAL;
		goto out;
	}

	/* Check if value has changed */
	if (plat.swms[swm_id].info.weight == value) {
		dprintk("SWM weight [%d] not changed for SWM [%s]\n",
			value, plat.swms[swm_id].info.name);
		result = 0;
		goto out;
	}

	/* update weight and notify policy */
	plat.swms[swm_id].info.weight = value;
	result = cpm_update_policy();
	if (result) {
		dprintk("policy update on weight change failed\n");
		result = 0;
		goto out;
	}

	dprintk("configured weight [%d] for SWM [%s]\n",
		plat.swms[swm_id].info.weight, plat.swms[swm_id].info.name);
out:

	mutex_unlock(&plat.mux);

	if (result)
		dprintk("%s: status %d\n", __func__, result);

	return result ? : count;
}

static int cpm_sysfs_core_swms_init(void)
{
	int result = 0;
	int i;
	struct cpm_swm_core *pcswm;
	struct attribute **pattrs;

	dprintk("exporting platform SWMs...\n");

	/* Make room for all SWMs, constraint and weight attirbutes; moreover
	 * a last one attributed is required to be empty initialized */
	plat.group.attrs = pattrs = kzalloc(((plat.count) + 3) *
					 sizeof(struct attribute *),
					 GFP_KERNEL);
	if (!pattrs) {
		dprintk("out-of-mem on platform_swms_group.attrs allocation\n");
		result = -ENOMEM;
		goto out_sysfs_swms_init_nomem;
	}

	pcswm = plat.swms;
	for (i = 0; i < plat.count; i++) {
		CPM_KATTR_RO(&(pcswm->kattr), pcswm->info.name,
			     cpm_sysfs_core_swms_show);
		CPM_ADD_ATTR(pattrs, pcswm->kattr);
		pcswm++;
	}

	/* add constraint and weight attribute */
	CPM_ADD_ATTR(pattrs, plat.constr_kattr);
	CPM_ADD_ATTR(pattrs, plat.weight_kattr);

	/* complete attribute group definition */
	(*pattrs) = NULL;

	/* export platform SWMs to sysfs */
	result = sysfs_create_group(&plat.kobj, &plat.group);
	if (result) {
		dprintk("platform SWMs exporting failed\n");
		goto out_sysfs_swms_init_export;
	}

out_sysfs_swms_init_export:

out_sysfs_swms_init_nomem:

	return result;

}

/**
 * cpm_sysfs_export_swms() - export the specified SWMs via sysfs
 * @pswm - the array of SWMs to export
 * @swms_count - the number of elements in the array
 * @name - the name of this group, if NULL attributes will be created under
 * the specified kobj
 * @attr_group - the group of attributes to use for the export
 * @kobj - the parent kobject under witch this group will be exported
 *
 */
static int cpm_sysfs_swms_export(struct cpm_swm_range *pswm, u8 swms_count,
				 char *name, struct attribute_group *attr_group,
				 struct kobject *kobj)
{
	int j, result;
	struct attribute **pattrs;

	attr_group->name = name;
	attr_group->attrs = kzalloc((swms_count + 1) *
					 sizeof(struct attribute *),
					 GFP_KERNEL);
	if (!attr_group->attrs) {
		dprintk("out-of-mem on SWMs attributes allocation\n");
		result = -ENOMEM;
		goto out_sysfs_swm_nomem;
	}

	/* Loop on SWMs array */
	pattrs = attr_group->attrs;
	for (j = 0; pswm && j < swms_count; j++) {
		snprintf(pswm->name, CPM_NAME_LEN, "swm%02d", pswm->id);
		CPM_KATTR_RO(&(pswm->kattr), pswm->name, cpm_swm_show);
		CPM_ADD_ATTR(pattrs, pswm->kattr);
		pswm++;
	}

	/* complete attribute group definition */
	(*pattrs) = NULL;

	/* export this SWMs group via sysfs */
	result = sysfs_create_group(kobj, attr_group);
	if (result) {
		dprintk("SWMs exporting failed\n");
		goto out_sysfs_swms_export_failed;
	}

	return 0;

out_sysfs_swms_export_failed:

	kfree(attr_group->attrs);

out_sysfs_swm_nomem:

	return result;

}

static int cpm_sysfs_fsc_current_export(struct cpm_fsc_core *pcfsc)
{
	int result = 0;

	dprintk("updating link to current FSC\n");

	sysfs_remove_link(&fscs->kobj, "current");

	result = sysfs_create_link(&fscs->kobj, &pcfsc->kobj, "current");
	if (result)
		eprintk("exporting current FSC link failed\n");

	return result;
}

static int cpm_sysfs_fscs_export(void)
{
	int result = 0;
	u16 cfsc;
	struct cpm_fsc_core *pcfsc;

	dprintk("exporting new FSC list...\n");

	/* Loop on FSCs */
	cfsc = 1;
	list_for_each_entry(pcfsc, &fscs->found, info.node) {

		/* create the FSC folder kobject under /sys/kernel/fscs */
		result =
		    kobject_init_and_add(&pcfsc->kobj, &cpm_fsc_ktype,
					 &fscs->kobj, "FSC%05u", cfsc);
		if (result) {
			eprintk
			    ("failure on \"cpm/fscs/FSC%05u\" sysfs "
			     "interface creation\n",
			     pcfsc->info.id);
			return result;
		}

		/* exporting SWMs for this FSC */
		result = cpm_sysfs_swms_export(pcfsc->info.swms,
					       pcfsc->info.swms_count,
					       0, &(pcfsc->swms_group),
					       &pcfsc->kobj);

		/* TODO exporting DWRs for this FSC */

		cfsc++;

	}

	return 0;

}

static int cpm_sysfs_fscs_unexport(struct cpm_fsc_data *fscs)
{
	struct cpm_fsc_core *pcfsc;
	struct cpm_fsc_core *pnext;

	if (unlikely(!list_empty(&fscs->found))) {

		dprintk("removing current FSC link\n");
		sysfs_remove_link(&fscs->kobj, "current");

		dprintk("releasing all FSC sysfs entries...\n");
		list_for_each_entry_safe(pcfsc, pnext,
				&fscs->found, info.node) {
			/* unexporting SWMs for this FSC */
			sysfs_remove_group(&pcfsc->kobj, &pcfsc->swms_group);
			/* releasing this FSC entry */
			kobject_put(&pcfsc->kobj);
		}

	}

	dprintk("removing main fscs sysfs entry\n");
	kobject_del(&fscs->kobj);
	kobject_put(&fscs->kobj);

	return 0;
}


/************************************************************************
 *	CPM DRIVER                          				*
 ************************************************************************/

/**
 * find_device_block() - find the cpm_dev_core for the specified device
 * @dev: a pointer to the &struct device representing the device to search
 *
 * Return values: a pointer to &struct cpm_dev_core or NULL if the device
 * has never registered an SWM before.
 */
static struct cpm_dev_core *find_device_block(struct device *dev)
{
	struct cpm_dev *pdev;
	struct cpm_dev_core *pcdev;

	list_for_each_entry(pdev, &devs.list, node) {
		if ((pdev->dev) == dev) {
			pcdev =
			    container_of(pdev, struct cpm_dev_core, dev_info);
			return pcdev;
		}
	}

	return 0;
}

/**
 * cpm_register_device() - register a device into CPM
 * @dev: a reference to the &struct device to register
 * @data: a reference to the &struct cpm_dev_data containing the device
 * 	specific data
 */
int cpm_register_device(struct device *dev, struct cpm_dev_data *data)
{
	struct cpm_dev_core *pcd;
	struct cpm_dev_dwr *pdwrs;
	struct cpm_dev_dwr *pdwr;
	struct cpm_swm_range *pswms;
	struct kobject *kobj;
	int result = 0;
	u8 i;

	mutex_lock(&devs.mux);

	/* Check if the device is already registerd */
	pcd = find_device_block(dev);
	if (pcd != 0) {
		dprintk("device [%s] already registered\n", dev_name(dev));
		mutex_unlock(&devs.mux);
		return -EEXIST;
	}

	dprintk("registering new device [%s]...\n", dev_name(dev));

	/* add core cpm device */
	pcd = kzalloc(sizeof(struct cpm_dev_core), GFP_KERNEL);
	if (!pcd) {
		dprintk("out-of-mem on cpm_dev_core allocation\n");
		mutex_unlock(&devs.mux);
		return -ENOMEM;
	}
	/* init device reference */
	pcd->dev_info.dev = dev;

	/* copy DWRs definitions */
	pdwrs = kzalloc(data->dwrs_count * sizeof(struct cpm_dev_dwr),
			GFP_KERNEL);
	if (!pdwrs) {
		dprintk("out-of-mem on cpm_dev_block allocation\n");
		result = -ENOMEM;
		goto out_crd_nomem_dwr;
	}
	memcpy((void *)pdwrs, (void *)data->dwrs,
	       data->dwrs_count * sizeof(struct cpm_dev_dwr));

	pcd->dev_info.dwrs = pdwrs;
	pcd->dev_info.dwrs_count = data->dwrs_count;

	for (pdwr = pdwrs, i = 0; i < pcd->dev_info.dwrs_count; pdwr++, i++) {

		dprintk("SWM array [%p], SWM count [%d]\n", pdwr->swms,
			pdwr->swms_count);
		if (!pdwr->swms || !pdwr->swms_count) {
			dprintk("WARNING: uninitialized driver DWR\n");
			continue;
		}

		pswms =
		    kzalloc(pdwr->swms_count * sizeof(struct cpm_swm_range),
			    GFP_KERNEL);
		if (!pswms) {
			dprintk("out-of-mem on cpm_swm_range allocation\n");
			result = -ENOMEM;
			goto out_crd_nomem_swm;
		}

		memcpy((void *)pswms, (void *)pdwr->swms,
		       pdwr->swms_count * sizeof(struct cpm_swm_range));
		pdwr->swms = pswms;

		/* each DWR must belongs to the registering device */
		pdwr->dev = dev;

		dprintk("added DWR [%s:%s] with %d SWMs\n",
			dev_name(dev), pdwr->name, pdwr->swms_count);
	}

	/* Get a refcount to the deivce */
	kobj = kobject_get(&dev->kobj);
	if (!kobj) {
		eprintk("unable to get a reference to the device\n");
		result = -ENODEV;
		goto out_crd_kobj;
	}

	/* Init SWM list for this device */
	INIT_LIST_HEAD(&(pcd->swm_list));

	/* TODO Fill into the SWM list */

	/* copy the DDP handler callback reference */
	pcd->dev_info.nb.notifier_call = data->notifier_callback;

	/* register to DDP notifier chain */
	mutex_lock(&cpm.ddp.mux);
	srcu_notifier_chain_register(&cpm.ddp.notifier_list,
				     &((pcd->dev_info).nb));
	mutex_unlock(&cpm.ddp.mux);

	/* add into device chain */
	list_add_tail(&(pcd->dev_info.node), &devs.list);
	devs.count++;

	mutex_unlock(&devs.mux);

	/* Setting-up sysfs interface */
	cpm_sysfs_dev_export(pcd);

	dprintk("new device [%s] successfully registerd\n", dev_name(dev));

	/* Notify governor */
	cpm_invalidate_fsc();

	return 0;

out_crd_kobj:

out_crd_nomem_swm:

	do {
		kfree(pdwr);
		pdwr--;
	} while (pdwr != pdwrs);

out_crd_nomem_dwr:

	kfree(pcd);

	mutex_unlock(&devs.mux);

	return result;

}
EXPORT_SYMBOL(cpm_register_device);

/**
 * cpm_unregister_device() - unregister a device from CPM
 * @dev: a reference to the &struct device to unregister
 *
 */
int cpm_unregister_device(struct device *dev)
{
	struct cpm_dev_core *pcd;

	mutex_lock(&devs.mux);

	/* Check if the device is already registerd */
	pcd = find_device_block(dev);
	if (pcd == 0) {
		dprintk("device [%s] not registerd\n", dev_name(dev));
		mutex_unlock(&devs.mux);
		return -ENODEV;
	}

	dprintk("unregistering device [%s]...\n", dev_name(dev));

	/* Removing the device from the norification chain */
	mutex_lock(&cpm.ddp.mux);
	srcu_notifier_chain_unregister(&cpm.ddp.notifier_list,
				       &((pcd->dev_info).nb));
	mutex_unlock(&cpm.ddp.mux);

	/* Removing the device from the list */
	list_del(&(pcd->dev_info.node));
	devs.count--;

	/* Releasing countref */
	kobject_put(&dev->kobj);

	mutex_unlock(&devs.mux);

	/* Clean-up device's sysfs interface */
	cpm_sysfs_dev_unexport(pcd);

	/* Build-up a new FSCs list */
	cpm_invalidate_fsc();

	return 0;
}
EXPORT_SYMBOL(cpm_unregister_device);

static int cpm_verify_constraint(u8 swm_id, struct cpm_range *range)
{

	/* Check if the required SWM exist */
	if (swm_id >= plat.count) {
		eprintk("constraint assert failed, SWM [%d] not existing\n",
			swm_id);
		return -EEXIST;
	}

	/* Verify ranges */
	if (unlikely(range->lower < plat.swms[swm_id].info.min)) {
		dprintk("lower bound exceeding SWM range\n");
		return -EINVAL;
	}
	if (unlikely(range->upper > plat.swms[swm_id].info.max)) {
		dprintk("upper bound exceeding SWM range\n");
		return -EINVAL;
	}

	switch (plat.swms[swm_id].info.comp) {
	case CPM_COMPOSITION_ADDITIVE:

		/* GiB (aka additive) constraints can only be lower bounded */
		if (likely(range->type == CPM_SWM_TYPE_LBOUND))
			return 0;

		if (range->type == CPM_SWM_TYPE_UBOUND) {
			eprintk("upper bound asserted on GiB SWM\n");
			return -EINVAL;
		}

		/* Trasform all others in a LB */
		range->type = CPM_SWM_TYPE_LBOUND;

		break;

	case CPM_COMPOSITION_RESTRICTIVE:

		/* LiB (aka restrictive) constraints can only be
		 * upper bounded */
		if (likely(range->type == CPM_SWM_TYPE_UBOUND))
			return 0;

		if (range->type == CPM_SWM_TYPE_LBOUND) {
			eprintk("lower bound asserted on LiB SWM\n");
			return -EINVAL;
		}

		/* Trasform all others in a LB */
		range->type = CPM_SWM_TYPE_UBOUND;

		break;

	}

	return 0;

}

/**
 * cpm_range_differ() - compare two ranges for differ
 * @range1: the first range to compare
 * @range2: the second range to comapre
 *
 * Return 1 if they differ, 0 otherwise
 */
static int cpm_range_differ(struct cpm_range *range1, struct cpm_range *range2)
{
	if (range1->type != range2->type)
		return 1;

	if (range1->lower != range2->lower)
		return 1;

	if (range1->upper != range2->upper)
		return 1;

	return 0;
}

/**
 * cpm_check_fsc() - check if current FSC is valid wrt to the specified range
 * @swm_id: the SWM to check
 * @range: the range to verify
 *
 * Return 0 if the FSC is valid, -EINVAL otherwise
 */
static int cpm_check_fsc(u8 swm_id, struct cpm_range *range)
{
	int i, result;
	struct cpm_fsc *pfsc;
	struct cpm_fsc_core *pcfsc;
	struct cpm_swm_range *pswm;
	struct cpm_range fsc_range;
	char str[64];

	/* Check if current FSC has a range on the specified SWM */
	if (unlikely(!fscs || !fscs->curr)) {
		wprintk("no current FSC defined, perhaps expected\n");
		return 0;
	}

	/* Getting references to current fsc */
	pfsc = fscs->curr->fsc;
	pcfsc = container_of(pfsc, struct cpm_fsc_core, info);
	pswm = pfsc->swms;
	for (i = 0; i < pfsc->swms_count; i++) {
		if (pswm->id == swm_id)
			break;
		pswm++;
	}
	if (i == pfsc->swms_count) {
		dprintk("no SWM range on current FSC for [%s]\n",
			kobject_name(&pcfsc->kobj));
		/* since the FSC don't have a range on this SWM, it still
		 * remain valid and the solution space will be eventually
		 * shrinked */
		return 0;
	}

	/* Checking if the FSC merge with the new range */
	fsc_range = pswm->range;
#ifdef CPM_DEBUG_CORE
	cpm_sysfs_print_range(str, 32, &fsc_range);
	cpm_sysfs_print_range(str + 32, 32, range);
	dprintk
	    ("check if current FSC's SWM range [%s] is "
	     "compatible with new range [%s]\n",
	     str, str + 32);
#endif

	result = cpm_merge_range(&fsc_range, range);
	if (result) {
		/* empty merge */
		dprintk("SWM range outside current FSC\n");
		return -EINVAL;
	}

	if (cpm_range_differ(&fsc_range, &pswm->range)) {
		/* the range will modify the FSC */
		dprintk("SWM range shrinks current FSC\n");
		cpm.ddp.required = 1;
		return 0;
	}

	return 0;
}

/**
 * cpm_aggregate() - aggregate an SWM
 * @pconstr: the constraint to aggregate
 * @swm_id: the SWM to which aggregate
 *
 * Compute range aggregation disregarding the (eventually) specified
 * constraint.
 *
 * Return the lower-bound sum for additive constraint, the minmum lower-bound
 * for restrictive constratins.
 */
static u32 cpm_aggregate(struct cpm_constraint *pconstr, u8 swm_id)
{
	struct cpm_constraint *pc;
	u32 value = 0;

	switch (plat.swms[swm_id].info.comp) {
	case CPM_COMPOSITION_ADDITIVE:

		list_for_each_entry(pc, &plat.swms[swm_id].constraints, node) {

			dprintk("Node: %p, prev %p, next %p\n",
				&pc->node, pc->node.prev, pc->node.next);

			if (pc == pconstr)
				continue;

			value += pc->range.lower;
		}
		value = MAX(value, plat.swms[swm_id].info.min);
		break;

	case CPM_COMPOSITION_RESTRICTIVE:

		value = plat.swms[swm_id].info.max;
		list_for_each_entry(pc, &plat.swms[swm_id].constraints, node) {

			dprintk("Node: %p, prev %p, next %p\n",
				&pc->node, pc->node.prev, pc->node.next);

			if (pc == pconstr)
				continue;

			value = MIN(value, pc->range.upper);
		}
		value = MIN(value, plat.swms[swm_id].info.max);
		break;

	}

	return value;

}

/**
 * cpm_aggregate_constraint() - update ddp_range for the specified SWM
 * @pconstr - the previous constraint asserted (null if new)
 * @swm_id - the SWM to which the constraint refers
 * @range - the constraint range
 *
 * This method update the ddp_range of the specifed SWM according to the
 * required range.
 *
 * If the new ddp_range invalidate the current FSC that
 */
static int cpm_aggregate_constraint(struct cpm_constraint *pconstr, u8 swm_id,
				    struct cpm_range *range)
{
	int result = 0;
	int delta = 0;

	dprintk("aggregating SWM [%s] on %s %s constraint\n",
		plat.swms[swm_id].info.name,
		pconstr ? "update" : "new",
		plat.swms[swm_id].info.comp == CPM_COMPOSITION_ADDITIVE ?
		"additive" : "restrictive");

	/* Initializing ddp_range */
	plat.swms[swm_id].ddp_range = plat.swms[swm_id].cur_range;
	cpm.relaxed = 0;

	/* Update ddp_range according to the type of the required new range */
	switch (plat.swms[swm_id].info.comp) {

	case CPM_COMPOSITION_ADDITIVE:

		delta = range->lower;
		if (pconstr)
			delta -= pconstr->range.lower;

		/* Checking aggregation faisability */
		if ((plat.swms[swm_id].cur_range.lower + delta) >
		    plat.swms[swm_id].info.max) {
			wprintk("constraint exceeding SWM platform range\n");
			return -EINVAL;
		}

		dprintk("additive (LB), cur_lower %d, delta %d\n",
			plat.swms[swm_id].cur_range.lower, delta);

		/* Look for a feasible FSC according to the new range,
		 * which is obtained by adding the cur_range lower bound
		 * to the new one.
		 */
		plat.swms[swm_id].ddp_range.lower += delta;

		/* Updating optimization direction */
		/* NOTE delta can be <0 only if updating a previous asserted
		 * constraint on the same SWM but with a lower value */
		if (delta < 0)
			cpm.relaxed = 1;

		break;

	case CPM_COMPOSITION_RESTRICTIVE:

		/* Checking aggregation faisability */
		if (range->upper < plat.swms[swm_id].info.min) {
			wprintk("constraint exceeding SWM platform range\n");
			return -EINVAL;
		}

		dprintk("restrictive (UB), cur_upper %d, new_upper %d\n",
			plat.swms[swm_id].cur_range.upper, range->upper);

		/* New constraint is more restrictive than current one */
		if (plat.swms[swm_id].cur_range.upper >= range->upper) {

			/* New range replace the old one */
			plat.swms[swm_id].ddp_range.upper = range->upper;
			break;
		}

		/* The new constraint is less restrictive */
		cpm.relaxed = 1;
		plat.swms[swm_id].ddp_range.upper =
		    cpm_aggregate(pconstr, swm_id);

		/* A previous constraint has been relaxed */
		if (pconstr) {
			plat.swms[swm_id].ddp_range.upper =
			    MIN(plat.swms[swm_id].ddp_range.upper,
				range->upper);
		}

		break;

	}

	/* Updating the required ddp_range type */
	plat.swms[swm_id].ddp_range.type = range->type;

	/* Space relaxing - always invalidate current FSC */
	if (cpm.relaxed) {

		nprintk("solution space relaxing - %s %s constraint may "
		     "invalidate current FSC\n",
		     pconstr ? "update" : "new",
		     (plat.swms[swm_id].info.comp ==
		      CPM_COMPOSITION_ADDITIVE) ? "additive" : "restrictive");

		if (likely(fscs))
			fscs->updated = 0;

		return 0;
	}

	/* Space shrinking - verifying if the new ddp_range invalidate
	 * current FSC */
	cpm.ddp.required = 0;
	result = cpm_check_fsc(swm_id, &plat.swms[swm_id].ddp_range);
	if (result) {
		nprintk("solution space shrinking - %s %s constraint "
			"invalidate current FSC\n",
		     pconstr ? "update" : "new",
		     (plat.swms[swm_id].info.comp ==
		      CPM_COMPOSITION_ADDITIVE) ? "additive" : "restrictive");

		if (likely(fscs))
			fscs->updated = 0;

		return 0;
	}

	dprintk("solution space shrinking - %s %s constraint "
		"doesn't invalidate current FSC\n",
	     pconstr ? "update" : "new",
	     (plat.swms[swm_id].info.comp ==
	      CPM_COMPOSITION_ADDITIVE) ? "additive" : "restrictive");

	return 0;

}

/**
 * cpm_verify_fsc() - verify if an FSC is valid
 * @fsc: the FSC to check
 *
 * Given an FSC check if is valid w.r.t. ddp_ranges of its SWM.
 *
 * Return 0 on OK, -EINVAL otherwise
 */
static int cpm_verify_fsc(struct cpm_fsc_core *fsc)
{
	int i, result;
	struct cpm_swm_range *pswm;
	struct cpm_range ddp_range;
#ifdef CONFIG_CPM_DEBUG
	char str[128];
	int count;
#endif

	dprintk("cpm_verify_fsc - FSC%05u...\n", fsc->info.id);

	pswm = fsc->info.swms;
	for (i = 0; i < fsc->info.swms_count; i++) {

		ddp_range = plat.swms[pswm->id].ddp_range;

#ifdef CONFIG_CPM_DEBUG
		count = snprintf(str, 128, "comparing, ddp_range ");
		count +=
		    cpm_sysfs_print_range(str + count, 128 - count, &ddp_range);
		count += snprintf(str + count, 128 - count, ", swm_range ");
		count +=
		    cpm_sysfs_print_range(str + count, 128 - count,
					  &pswm->range);
		dprintk("%s\n", str);
#endif

		result = cpm_merge_range(&ddp_range, &pswm->range);
		if (result) {
			dprintk("cpm_verify_fsc - SWM [%s] not merge\n",
				plat.swms[pswm->id].info.name);
			return -EINVAL;
		}

		dprintk("cpm_verify_fsc - SWM [%s] merge\n",
			plat.swms[pswm->id].info.name);
		pswm++;
	}

	return 0;
}

static int __cpm_notifier_call_chain(struct notifier_block **nl,
				     unsigned long val, struct cpm_fsc *pfsc)
{
	int result = NOTIFY_DONE;
	struct notifier_block *nb, *next_nb;
	struct cpm_dev *pdev;
	struct cpm_fsc_dwr *pfdwr;
	int count;

	nb = rcu_dereference(*nl);
	pfdwr = pfsc->dwrs;
	if (unlikely(!pfdwr->cdev)) {
		eprintk("missing device definition from governor\n");
		return -EINVAL;
	}

	count = 0;

	/* Looping on the registered device's list */
	while (nb) {

		next_nb = rcu_dereference(nb->next);

		/* Current device to notify */
		pdev = container_of(nb, struct cpm_dev, nb);
		dprintk("notifying device [%s]\n", dev_name(pdev->dev));

		/* Looking for the FSC's DWR for current device */
		/* Note that the governor should produce a DWR list */
		while (pfdwr->cdev != pdev) {
			dprintk("cur dev [%s] differ from dwr dev [%s]\n",
				dev_name(pfdwr->cdev->dev),
				dev_name(pdev->dev));
			pfdwr++;
			count++;
		}

		/* NOTE: it could happen that a governor return an FSC's DWR
		 * list not ordered according to the registered devices list.
		 * In this case we could wrap around the DWR list: start
		 * search from start one more time for safety */
		if (count == pfsc->dwrs_count) {
			nprintk("unordered DWR list for this DWR\n");

			/* Restart search from beginning */
			pfdwr = pfsc->dwrs;
			count = 0;
			while (pfdwr->cdev != pdev) {
				dprintk("cur dev [%s] differ from dwr "
					"dev [%s]\n",
					dev_name(pfdwr->cdev->dev),
					dev_name(pdev->dev));
				pfdwr++;
				count++;
			}
			if (count == pfsc->dwrs_count) {
				eprintk("no FSC's DWR defined for this "
					"device [%s]\n",
					dev_name(pdev->dev));
				nb = next_nb;
				continue;
			}
		}

		/* Notify DWR's ID to current device */
		result = nb->notifier_call(nb, val, (void *)&pfdwr->dwr->id);
		if ((result & NOTIFY_STOP_MASK) == NOTIFY_STOP_MASK) {
			dprintk("notification terminated by device [%s]\n",
				dev_name(pdev->dev));
			break;
		}

		pfdwr++;
		count++;
		nb = next_nb;
	}

	return result;

}

/**
 * cpm_notifier_call_chain() - notify reigsterd devices
 * @nh: the notification chain to use
 * @event: the event to notify
 * @pnewfscp: a reference to an FSC
 *
 * Notify all devices about the specified new FSC.
 *
 * Return 0 on agreement, non-null otherwise.
 */
static int cpm_notifier_call_chain(struct srcu_notifier_head *nh,
				   unsigned long event,
				   struct cpm_fsc_pointer *pnewfscp)
{
	int result = 0;
	int idx;

	idx = srcu_read_lock(&nh->srcu);
	result = __cpm_notifier_call_chain(&nh->head, event, pnewfscp->fsc);
	srcu_read_unlock(&nh->srcu, idx);

	if (result & NOTIFY_BAD)
		return result;

	return 0;

}

/**
 * cpm_notify_new_fsc() - notify registerd devices for a new FSC
 * @pnewfscp: a reference to the new FSC to notify
 *
 * All subscribed device will be notified about the new FSC which has been
 * identified.
 *
 * Return 0 on distributed agreement, not null otherwise.
 */
static int cpm_notify_new_fsc(struct cpm_fsc_pointer *pnewfscp)
{
	int result = 0;
	struct cpm_fsc_core *pcfsc;
	struct timespec start, end, delta;

	if (unlikely(!cpm.ddp.required)) {
		dprintk("no need to run a DDP\n");
		return result;
	}

	pcfsc = container_of(pnewfscp->fsc, struct cpm_fsc_core, info);
	dprintk("starting DDP for new FSC [%s]...\n",
		kobject_name(&pcfsc->kobj));

	getrawmonotonic(&start);

	/* Looking for devices distributed agreement */
	result =
	    cpm_notifier_call_chain(&cpm.ddp.notifier_list,
				    CPM_EVENT_PRE_CHANGE, pnewfscp);

	if (result) {
		nprintk("DDP failed, not agreement on selected FSC\n");
		/* Notify not agreement to each device */
		cpm_notifier_call_chain(&cpm.ddp.notifier_list, CPM_EVENT_ABORT,
					fscs->curr);
		return result;
	}

	dprintk("distributed agreement on new FSC [%s]\n",
		kobject_name(&pcfsc->kobj));

	/* Notify distributed agreement to policy */
	if (likely(pol.curr))
		pol.curr->ddp_handler(CPM_EVENT_PRE_CHANGE, (void *)pnewfscp);

	/* Require change to devices */
	result =
	    cpm_notifier_call_chain(&cpm.ddp.notifier_list, CPM_EVENT_DO_CHANGE,
				    pnewfscp);
	if (unlikely(result))
		eprintk("pre-change failed, some device aborted "
				"change request\n");

	/* Synking devices after FSC change */
	result =
	    cpm_notifier_call_chain(&cpm.ddp.notifier_list,
				    CPM_EVENT_POST_CHANGE, pnewfscp);
	if (unlikely(result))
		eprintk("post-change failed, some device aborted "
				"change request\n");

	if (likely(pol.curr))
		pol.curr->ddp_handler(CPM_EVENT_POST_CHANGE, (void *)pnewfscp);

	getrawmonotonic(&end);
	delta = timespec_sub(end, start);
	iprintk("EX-DDP, agreement in: %d %03ld.%09ld\n", devs.count,
		delta.tv_sec, delta.tv_nsec);

	cpm.ddp.required = 0;

	dprintk("DDP completed\n");

	return result;
}

/**
 * __cpm_find_next_valid_fsc() - find the next valid FSC
 * @fscs: the eventually found FSC
 * @pos: the position to start search from
 *
 * Find next valid FSC, starting from that following the specified one.
 * In case of a constraint addition (e.g. shrinking of the solution space) the
 * eventually next valid FSC must follow the current one selected in the
 * ordered FSC list
 */
static struct cpm_fsc_pointer *__cpm_find_next_valid_fsc(struct cpm_fsc_data
							 *fscs,
							 struct list_head *pos)
{
	struct cpm_fsc_pointer *pfscp;
	struct cpm_fsc_core *pfscc;
	int result;

	/* Scan FSC starting from last valid FSC (current FSC) */
	for (pos = pos->next; prefetch(pos->next), pos != &fscs->ordered;
	     pos = pos->next) {
		pfscp = container_of(pos, struct cpm_fsc_pointer, node);
		pfscc = container_of(pfscp->fsc, struct cpm_fsc_core, info);

		/* Check if the FSCs list has been invalidated meanwile */
		if (unlikely(!fscs->valid)) {
			dprintk("invalidate FSCs list while searching "
					"new current FSC, aborting\n");
			return 0;
		}

		result = cpm_verify_fsc(pfscc);
		if (!result) {
			dprintk("next valid FSC @ [%s]\n", pfscc->kobj.name);
			return pfscp;
		}

		dprintk("FSC [%s] not valid, checking next...\n",
			pfscc->kobj.name);
	}

	return 0;

}

/**
 * cpm_find_next_valid_fsc() - find next valid FSC
 * @fscs: the eventually found FSC
 * @pos: the position to start search from
 *
 * Find the next valid FSC, (eventually) authorized by the running policy.
 */
static struct cpm_fsc_pointer *cpm_find_next_valid_fsc(struct cpm_fsc_data
						       *fscs,
						       struct list_head *pnode)
{
	int result = 0;		/* Must be NULL at firt loop run */
	struct cpm_fsc_pointer *pnewfscp = 0;
	struct cpm_fsc_core *pcfsc;
	struct timespec start, end, delta;

	do {

		/* Find next valid FSC */
		getrawmonotonic(&start);
		pnewfscp = __cpm_find_next_valid_fsc(fscs, pnode);
		getrawmonotonic(&end);
		if (unlikely(!pnewfscp))
			break;

		if (unlikely(!pol.curr)) {
			dprintk("policy not registerd, unable to verify "
					"FSC with policy\n");
			break;
		}

		/* Asking FSC authorization to current policy */
		result =
		    pol.curr->ddp_handler(CPM_EVENT_FSC_FOUND,
					  (void *)pnewfscp);
		if (unlikely(result))
			nprintk("FSC found but not authorized by policy\n");

		pnode = &pnewfscp->node;

	} while (pnewfscp && result);

	delta = timespec_sub(end, start);

	if (pnewfscp) {
		pcfsc = container_of(pnewfscp->fsc, struct cpm_fsc_core, info);
		iprintk("new FSC [%s] found\n", kobject_name(&pcfsc->kobj));
		iprintk("EX-FSC, found in: %d %03ld.%09ld\n", fscs->count,
			delta.tv_sec, delta.tv_nsec);
	} else {
		eprintk("no valid FSC found\n");
		iprintk("EX-FSC, not-found in: %d %03ld.%09ld\n", fscs->count,
			delta.tv_sec, delta.tv_nsec);
	}

	delta = timespec_sub(end, start);

	return pnewfscp;

}

/**
 * cpm_update_fsc() - update current FSC
 *
 * If current FSC is marked not-updated, look for another valid FSC and ask
 * agreement to registered devices.
 */
static int cpm_update_fsc(void)
{
	struct kobject *kobj;
	struct cpm_fsc_data *_fscs;
	struct list_head *pnode;
	struct cpm_fsc_pointer *pnewfscp;
	struct cpm_fsc_core *pfscc;
	int result = 0;		/* Must be NULL at first loop run */

	mutex_lock(&fsc_mutex);

	if (unlikely(!fscs)) {
		wprintk("no current FSC defined, perhaps expected\n");
		mutex_unlock(&fsc_mutex);
		return 0;
	}


	/* Get a local reference to current FSCs list */
	_fscs = fscs;

	/* Getting a refcount on FSCs list */
	kobj = kobject_get(&_fscs->kobj);
	if (unlikely(!kobj)) {
		dprintk("invalid FSC list, update current FSC failed\n");
		mutex_unlock(&fsc_mutex);
		return -EINVAL;
	}

	/* Releasing mutex */
	mutex_unlock(&fsc_mutex);

	/* Checking if the FSC list is valid */
	if (unlikely(!_fscs->valid)) {
		dprintk("invalid FSC list\n");

		/* Releasing FSCs list */
		kobject_put(&_fscs->kobj);

		return result;
	}

	if (unlikely(_fscs->updated)) {
		dprintk("no need to update current FSC\n");

		/* Releasing FSCs list */
		kobject_put(&_fscs->kobj);

		return result;
	}

	dprintk("updating current FSC...\n");

	/* Initializing the FSC to start the search from */
	if (cpm.relaxed || !_fscs->curr) {
		/* Search starts from the first FSC */
		dprintk("search for first valid FSC...\n");
		pnode = &_fscs->ordered;
	} else {
		/* Search forward for next valid FSC */
		dprintk("search for next valid FSC...\n");
		pnode = &_fscs->curr->node;

	}

	/* Locking changes on ordered fsc list */
	mutex_lock(&_fscs->omux);

	/* Searching the FSC */
	do {
		/* Find next valid FSC */
		pnewfscp = cpm_find_next_valid_fsc(_fscs, pnode);
		if (unlikely(!pnewfscp))
			break;

		/* Start DDP */
		result = 0;

		cpm.ddp.required = 1;
		result = cpm_notify_new_fsc(pnewfscp);

		if (result)
			eprintk("not-agreement on new FSC activation\n");

	} while (result && pnewfscp);
	/* Exit on DDP agreement or no more FSC availables */

	mutex_unlock(&fscs->omux);

	if (unlikely(!pnewfscp)) {
		nprintk("unable to find a%sfeasible FSC\n",
			result ? " " : "n agreement on a ");

		/* Releasing FSCs list */
		kobject_put(&_fscs->kobj);

		return -EINVAL;
	}

	/* Update new current system FSC */
	mutex_lock(&fsc_mutex);

	/* Update current FSC only if the FSCs list has not been
	 * invalidated meanwhile */
	if (likely(_fscs->valid)) {
		fscs->curr = pnewfscp;
		fscs->updated = 1;

		/* Update sysfs interface */
		pfscc = container_of(pnewfscp->fsc, struct cpm_fsc_core, info);
		cpm_sysfs_fsc_current_export(pfscc);
	}

	mutex_unlock(&fsc_mutex);

	/* Releasing FSCs list */
	kobject_put(&_fscs->kobj);

	return 0;
}

static int cpm_add_constraint(void *entity, u8 type, u8 swm_id,
			      struct cpm_range *range)
{
	struct cpm_constraint *pconstr;
	char entity_name[32];
	char str[64];
	int result;

	__cpm_entity_name(entity, type, entity_name, 32);

	/* Saving the ddp_range into current ones */
	plat.swms[swm_id].cur_range = plat.swms[swm_id].ddp_range;
	cpm_sysfs_print_range(str, 64, &plat.swms[swm_id].cur_range);
	dprintk("updated current SWM [%s] range (%s)\n",
		plat.swms[swm_id].info.name, str);

	cpm_sysfs_print_range(str + 32, 32, range);
	/* Check if the entity has already asserted a constraint for the SWM */
	list_for_each_entry(pconstr, &plat.swms[swm_id].constraints, node) {
		if (entity == pconstr->entity.ptr) {
			cpm_sysfs_print_range(str, 32, &pconstr->range);
			dprintk("replacing constraint for [%s] (%s => %s)\n",
				entity_name, str, str + 32);
			break;
		}
	}

	/* Allocate a new constraint for this device if necessary */
	if (&pconstr->node == &plat.swms[swm_id].constraints) {

		dprintk("allocate a new constraint for [%s] (%s)\n",
			entity_name, str + 32);
		pconstr = kzalloc(sizeof(struct cpm_constraint), GFP_KERNEL);
		if (!pconstr) {
			eprintk("out-of-mem on new constraint allocation\n");
			result = -ENOMEM;
			goto out_constr_nomem;
		}

		/* Add the constraint to the list for this SWM */
		list_add_tail(&pconstr->node, &plat.swms[swm_id].constraints);

	}

	/* Update the constraints values */
	pconstr->entity.type = type;
	pconstr->entity.ptr = entity;
	pconstr->range = *range;

	/* Update sysfs interface for this new entity */
	/* TODO */

	return 0;

out_constr_nomem:

	return result;

}

static int __cpm_update_constraint(void *entity, u8 type, u8 swm_id,
				   struct cpm_range *range)
{
	struct cpm_constraint *pconstr;
	char str[64];
	int result;
	u8 replace_constraint = 0;
	char entity_name[32];
	struct cpm_policy_notify_data cpnd;

	__cpm_entity_name(entity, type, entity_name, 32);

	/* Sanity check the required constraint */
	result = cpm_verify_constraint(swm_id, range);
	if (result) {
		dprintk("constraint assert sanity check failed, aborting\n");
		return result;
	}

	/* Notify policy about new constraint and ask authorization */
	if (likely(pol.curr)) {
		cpnd.entity.ptr = entity;
		cpnd.entity.type = type;
		cpnd.range = *range;

		result =
		    pol.curr->ddp_handler(CPM_EVENT_NEW_CONSTRAINT,
					  (void *)&cpnd);
		if (result) {
			nprintk("constraint assertion denied by policy\n");
			return result;
		}
	}

	/* Check if the entity has already asserted a constraint for the SWM */
	list_for_each_entry(pconstr, &plat.swms[swm_id].constraints, node) {
		if (entity == pconstr->entity.ptr) {
			replace_constraint = 1;
			break;
		}
	}
	cpm_sysfs_print_range(str, 64, range);
	iprintk("%s constraint asserted by [%s], %d:%s %s\n",
		replace_constraint ? "replacing" : "adding",
		entity_name, swm_id, plat.swms[swm_id].info.name, str);
	/* Trying constraints aggregation */
	if (!replace_constraint)
		pconstr = 0;

	result = cpm_aggregate_constraint(pconstr, swm_id, range);
	if (result) {
		/* constraint exceeding SWM ranges: rejected */
		wprintk("rejecting constraint\n");
		return result;
	}

	/* Check if a new FSC must be searched */
	result = cpm_update_fsc();
	if (result) {
		/* Resetting ddp_range to current value for next search */
		plat.swms[swm_id].ddp_range = plat.swms[swm_id].cur_range;
		return result;
	}

	/* Trigger DDP if necessary */

	if (likely(fscs)) {
		/* NOTE This is required to forward notifications on new
		 * constraints that don't invalidate current FSC but shrink
		 * or relax the space */
		result = cpm_notify_new_fsc(fscs->curr);
		if (result) {
			wprintk("DDP failed\n");
			return result;
		}
	}

	/* Keet track of the new constraint */
	result = cpm_add_constraint(entity, type, swm_id, range);
	if (result) {
		eprintk("failed to add a constraint for SWM [%s]\n",
			plat.swms[swm_id].info.name);
	}

	return 0;

}

/**
 * cpm_update_constraint() - update a constraint
 * @dev: the device updating the constraint
 * @swm_id: the SWMto which the constraint refers
 * @range: the new value for the constraint
 *
 */
int cpm_update_constraint(struct device *dev, u8 swm_id,
			  struct cpm_range *range)
{
	struct cpm_dev_core *pcd;
	int result = 0;

	/* Check if the device is already registerd */
	pcd = find_device_block(dev);
	if (!pcd) {
		dprintk("constraint assert failed, device [%s] not registerd\n",
			dev_name(dev));
		return -EEXIST;
	}

	mutex_lock(&plat.mux);
	result =
	    __cpm_update_constraint((void *)dev, CPM_ENTITY_TYPE_DRIVER, swm_id,
				    range);
	mutex_unlock(&plat.mux);

	return result;

}
EXPORT_SYMBOL(cpm_update_constraint);

static int __cpm_remove_constraint(void *entity, u8 type, u8 swm_id)
{
	struct cpm_constraint *pconstr;
	char entity_name[32];
	char str[64];
	int result;

	__cpm_entity_name(entity, type, entity_name, 32);

	/* Check if the required SWM exist */
	if (swm_id >= plat.count) {
		eprintk("constraint assert failed, SWM [%d] not existing\n",
			swm_id);
		return -EINVAL;
	}

	/* Check if the entity has asserted a constraint for the SWM */
	list_for_each_entry(pconstr, &plat.swms[swm_id].constraints, node) {
		if (entity == pconstr->entity.ptr)
			break;
	}

	/* Remove the constraint for this entity only if necessary */
	if (&pconstr->node == &plat.swms[swm_id].constraints) {
		wprintk("entity deasserting un-registered constraint\n");
		return 0;
	}

	cpm_sysfs_print_range(str, 64, &pconstr->range);
	iprintk("[%s] releasing constraint on %d:%s %s\n",
		entity_name, swm_id, plat.swms[swm_id].info.name, str);

	/* Removing the constraint from the list for this SWM */
	list_del(&pconstr->node);

	/* Update ddp_range according to the type of the required new range */
	switch (plat.swms[swm_id].info.comp) {

	case CPM_COMPOSITION_ADDITIVE:

		plat.swms[swm_id].ddp_range.lower = cpm_aggregate(0, swm_id);
		plat.swms[swm_id].ddp_range.type = CPM_SWM_TYPE_LBOUND;

		/* Removing an additive constraint will relax the space for
		 * sure */
		cpm.relaxed = 1;

		break;

	case CPM_COMPOSITION_RESTRICTIVE:

		plat.swms[swm_id].ddp_range.upper = cpm_aggregate(0, swm_id);
		plat.swms[swm_id].ddp_range.type = CPM_SWM_TYPE_UBOUND;

		/* Checking if removing this restrictive constraint relax
		 * the space */
		if (plat.swms[swm_id].ddp_range.upper > pconstr->range.upper)
			cpm.relaxed = 1;

		break;
	}

	/* Space relaxing - always invalidate current FSC */
	if (cpm.relaxed) {
		nprintk("solution space relaxing - constraint removal may "
				"invalidate current FSC\n");
		fscs->updated = 0;
	} else {
		dprintk("constraint removal don't invalidate current FSC\n");
	}

	/* Releasing constraint memory */
	kfree(pconstr);

	/* Update FSC if needed */
	result = cpm_update_fsc();
	if (result) {
		/* A constraint release should always have a feasible FSC */
		eprintk("system corruption: constraint release producing "
				"unsafe state\n");
		/* TODO maybe reset CPM */
	}

	/* Saving the ddp_range into current ones */
	plat.swms[swm_id].cur_range = plat.swms[swm_id].ddp_range;
	cpm_sysfs_print_range(str, 64, &plat.swms[swm_id].cur_range);
	dprintk("updated current SWM [%s] range (%s)\n",
		plat.swms[swm_id].info.name, str);

	return 0;
}


/**
 * cpm_remove_constraint() - release a contraint
 * @dev: the device releasing the constraint
 * @swm_id: the SWMto which the constraint release refers
 *
 */
int cpm_remove_constraint(struct device *dev, u8 swm_id)
{
	struct cpm_dev_core *pcd;
	int result = 0;

	/* Check if the device is already registerd */
	pcd = find_device_block(dev);
	if (!pcd) {
		dprintk("constraint assert failed, device [%s] not registerd\n",
			dev_name(dev));
		return -EEXIST;
	}

	mutex_lock(&plat.mux);
	result =
	    __cpm_remove_constraint((void *)dev, CPM_ENTITY_TYPE_DRIVER,
				    swm_id);
	mutex_unlock(&plat.mux);

	return result;

}
EXPORT_SYMBOL(cpm_remove_constraint);

/************************************************************************
 *	CORE INITIALIZATION						*
 ************************************************************************/

static int __init cpm_sysfs_core_init(void)
{
	int result = 0;

	/* create the "cpm" sysfs' entry under /sys/kernel */
	result =
	    kobject_init_and_add(&cpm.kobj, &cpm_core_ktype, kernel_kobj,
				 "cpm");
	if (result) {
		eprintk("sysfs - core interface creation failed\n");
		goto out_sysfs_core_cpm;
	}

	/* create the "swms" sysfs' entry under /sys/kernel/cpm */
	result =
	    kobject_init_and_add(&plat.kobj, &cpm_plat_ktype, &cpm.kobj,
				 "swms");
	if (result) {
		eprintk("sysfs - platform interface creation failed\n");
		goto out_sysfs_core_swms;
	}
	return result;

out_sysfs_core_swms:
	kobject_put(&cpm.kobj);

out_sysfs_core_cpm:

	return result;

}

static int __init cpm_sysfs_init(void)
{
	int result = 0;

	/* Setting up generic sysfs operations */
	cpm_sysfs_ops.show = cpm_sysfs_show;
	cpm_sysfs_ops.store = cpm_sysfs_store;

	/* Setting up ktype for <kernel>/cpm and its folders */
	cpm_core_ktype.release = cpm_sysfs_core_release;
	cpm_core_ktype.sysfs_ops = &cpm_sysfs_ops;
	cpm_core_ktype.default_attrs = cpm_sysfs_core_attrs;

	/* Setting up ktype for <kernel>/cpm/swms and its folders */
	cpm_plat_ktype.release = cpm_sysfs_plat_release;
	cpm_plat_ktype.sysfs_ops = &cpm_sysfs_ops;
	cpm_plat_ktype.default_attrs = NULL;

	/* Initializing SWMs constraint asserts attributes */
	plat.constr_kattr.attr.name = "constraint";
	plat.constr_kattr.attr.mode = 0222;
	plat.constr_kattr.show = NULL;
	plat.constr_kattr.store = cpm_sysfs_core_swms_constr_store;

	/* Initializing SWMs policy weight asserts attributes */
	plat.weight_kattr.attr.name = "weight";
	plat.weight_kattr.attr.mode = 0220;
	plat.weight_kattr.show = NULL;
	plat.weight_kattr.store = cpm_sysfs_core_swms_weight_store;

	/* Setting up ktype for <kernel>/cpm/fscs and its folders */
	cpm_fsc_ktype.release = cpm_sysfs_fsc_release;
	cpm_fsc_ktype.sysfs_ops = &cpm_sysfs_ops;
	cpm_fsc_ktype.default_attrs = NULL;

	/* Setting up ktype for <devices>/cpm and its folders */
	cpm_dev_ktype.release = cpm_sysfs_dev_release;
	cpm_dev_ktype.sysfs_ops = &cpm_sysfs_ops;
	cpm_dev_ktype.default_attrs = NULL;

	/* Setting-up CPM subsystem */
	result = cpm_sysfs_core_init();
	if (result) {
		eprintk("sysfs - setting-up core sysfs interface failed\n");
		return result;
	}

	dprintk("sysfs - interface initialized\n");

	return result;
}

static int __init cpm_core_setup(void)
{

	/* Setup workqueue */
	cpm.wq = create_singlethread_workqueue("cpm-core");
	dprintk("singlethread workqueue configured\n");

	/* Init device notifier list */
	srcu_init_notifier_head(&cpm.ddp.notifier_list);

	return 0;
}

static int __init cpm_core_init(void)
{

	cpm_core_setup();

	cpm_sysfs_init();

	dprintk("CPM core initialized\n");

	return 0;
}

core_initcall(cpm_core_init);


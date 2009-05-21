/*
 * This module exposes the interface to kernel space for specifying
 * QoS dependencies.  It provides infrastructure for registration of:
 *
 * Dependents on a QoS value : register requirements
 * Watchers of QoS value : get notified when target QoS value changes
 *
 * This QoS design is best effort based.  Dependents register their QoS needs.
 * Watchers register to keep track of the current QoS needs of the system.
 *
 * There are 3 basic classes of QoS parameter: latency, timeout, throughput
 * each have defined units:
 * latency: usec
 * timeout: usec <-- currently not used.
 * throughput: kbs (kilo byte / sec)
 *
 * There are lists of pm_qos_objects each one wrapping requirements, notifiers
 *
 * User mode requirements on a QOS parameter register themselves to the
 * subsystem by opening the device node /dev/... and writing there request to
 * the node.  As long as the process holds a file handle open to the node the
 * client continues to be accounted for.  Upon file release the usermode
 * requirement is removed and a new qos target is computed.  This way when the
 * requirement that the application has is cleaned up when closes the file
 * pointer or exits the pm_qos_object will get an opportunity to clean up.
 *
 * Patrick Bellasi <derkling@gmail.com>
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

struct cpm_asm_map {
	cpm_id id;			/* the ID of the ASM mapped */
	struct list_head node;
};

struct cpm_dev_sysfs {
	struct kobject cpm;		/* the kobj to export device info in sysfs (<device>/cpm) */
	struct kobject dwrs;		/* the kobj to export device DWRS in sysfs (<device>/cpm/dwrs) */
	struct kobject constraints;	/* the kobj to export device constraints in sysfs (<device>/cpm/constraints) */
	/* TODO add entries for statistics */
};

struct cpm_entity {
	union {
		struct task_struct *task;
		struct device *dev;
		void *ptr;
	};
#define CPM_ENTITY_TYPE_DRIVER	0
#define CPM_ENTITY_TYPE_TASK	1
	u8 type:1;			/* The entity type */
};

struct cpm_constraint {
	struct cpm_entity entity;	/* The entity asserting the constraint */
	struct cpm_range range;		/* The asserted contraint */
	struct list_head node;		/* The next constraint */
};

struct cpm_asm_core {
	struct cpm_asm info;		/* the public accessible data */
	cpm_id id;			/* the ASMs unique identifier */
	struct cpm_range cur_range;	/* the current range for this ASM according to asserted constraints */
	struct cpm_range ddp_range;	/* the range to use during a DDP */
	struct cpm_range valid_range;	/* the ASM validity range according to current contraints */
	struct list_head constraints;	/* the list of contraints asserted on this ASM */
#ifdef CONFIG_CPM_SYSFS
	struct kobj_attribute kattr;	/* the sysfs attribute to show ASMs values */
#endif
};

struct cpm_dev_core {
	struct cpm_dev dev_info;	/* the public accessible data */
	struct list_head asm_list;	/* list of cpm_asm_map that belongs to at least one DWR */
#ifdef CONFIG_CPM_SYSFS
	struct cpm_dev_sysfs sysfs;		/* the sysfs interface */
#endif
};

struct cpm_fsc_core {
	struct cpm_fsc info;		/* the public accessible data */
	u8 valid:1;			/* this flasg the FSC as valid */
#ifdef CONFIG_CPM_SYSFS
	struct attribute_group asms_group;  	/* The ASMs of this FSC */
	struct attribute_group dwrs_group;	/* The DWRs of this FSC */
	char name[CPM_NAME_LEN];		/* The name of this FSC */
#endif
};

/* cpm_dev_ktype - the ktype of each device registerd to CPM */
struct kobj_type cpm_dev_ktype;
struct sysfs_ops cpm_dev_ops;

/* Platform defined ASMs */
static struct cpm_asm_core *platform = 0;

/* The number of available ASMs */
static unsigned int cpm_asm_count = 0;

/* The ASMs data mutex */
static DEFINE_MUTEX(asm_mutex);


/* The governor in use */
static struct cpm_governor *governor = 0;

/* The governor's policy in use */
static struct cpm_policy *policy = 0;

/* The list of devices subscribed to some ASM */
static LIST_HEAD(dev_list);

/* The list of active constraints */
static LIST_HEAD(constraint_list);

/* The existing FSC */
static LIST_HEAD(fsc_list);

/* The count of FSC in the fsc_list */
static unsigned int fsc_count = 0;

/* The current selected FSC */
static struct cpm_fsc_pointer *pcfp = 0;

/* The FSC's list mutex */
static DEFINE_MUTEX(fsc_mutex);

/* The ordered FSC list to use for valid FSC selection during a DDP */
static LIST_HEAD(fsc_ordered_list);

/* The ordered FSC list mutex */
static DEFINE_MUTEX(fsc_ordered_mutex);

/* The notifier list used for DDP */
static struct srcu_notifier_head cpm_ddp_notifier_list;

/* The number of devices subscribed (i.e. dev_list entries) */
static u8 cpm_dev_count = 0;

/* The DDP Mutex */
static DEFINE_MUTEX(ddp_mutex);

/* Define if the CPM core is enabled (i.e. governors/policy and DDP will be
 * used */
static short unsigned cpm_enabled = 0;

/* The FSC list is marked outdated when a device register/unregister */
static short unsigned cpm_fsc_outdated = 0;

/* The core workqueue */
struct workqueue_struct *cpm_wq = 0;
EXPORT_SYMBOL(cpm_wq);



/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/
#ifdef CONFIG_CPM_DEBUG

#define dprintk(msg...) cpm_debug_printk(CPM_DEBUG_CORE, \
						"cpm-core", msg)

#define iprintk(msg...) pr_info("cpm-core: " msg)

#define eprintk(msg...) pr_err("cpm-core: " msg)


/* what part(s) of the CPM subsystem are debugged? */
static unsigned int debug = 0xFFFF;

/* is the debug output ratelimit'ed using printk_ratelimit? User can
 * set or modify this value.
 */
static unsigned int debug_ratelimit = 1;

/* is the printk_ratelimit'ing enabled? It's enabled after a successful
 * loading of a CPM governor, temporarily disabled when a new policy
 * is set, and disabled upon CPM governor removal
 */
static unsigned int disable_ratelimit = 1;
static DEFINE_MUTEX(disable_ratelimit_mutex);
static void cpm_debug_enable_ratelimit(void)
{
	mutex_lock(&disable_ratelimit_mutex);
	if (disable_ratelimit)
		disable_ratelimit--;
	mutex_unlock(&disable_ratelimit_mutex);
}

static void cpm_debug_disable_ratelimit(void)
{
	mutex_lock(&disable_ratelimit_mutex);
	disable_ratelimit++;
	mutex_unlock(&disable_ratelimit_mutex);
}

void cpm_debug_printk(unsigned int type, const char *prefix,
			const char *fmt, ...)
{
	char s[256];
	va_list args;
	unsigned int len;

	WARN_ON(!prefix);
	if (type & debug) {
		mutex_lock(&disable_ratelimit_mutex);
		if (!disable_ratelimit && debug_ratelimit
					&& !printk_ratelimit()) {
			mutex_unlock(&disable_ratelimit_mutex);
			return;
		}
		mutex_unlock(&disable_ratelimit_mutex);

		len = snprintf(s, 256, KERN_DEBUG "%s: ", prefix);

		va_start(args, fmt);
		len += vsnprintf(&s[len], (256 - len), fmt, args);
		va_end(args);

		printk(s);

		WARN_ON(len < 5);
	}
}
EXPORT_SYMBOL(cpm_debug_printk);

#endif

/******************************************************************************
 *   CORE                                                                     *
 ******************************************************************************/

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
int cpm_verify_range(struct cpm_range *range, u32 value) {
	switch (range->type) {
	case CPM_ASM_TYPE_UNBOUNDED:
		break;
	case CPM_ASM_TYPE_RANGE:
		if (value>range->upper)
			return CPM_RANGE_ERROR;
		if (value<range->lower)
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
EXPORT_SYMBOL(cpm_verify_range);

/*
 * Merge two cpm_range
 * @first:	first cpm_range, also used to return the result
 * @second:	second cpm_range to be merged with the first
 *
 * Return values: -EINVAL if the two ranges can't be merged, 0 if merge is ok
 */
int cpm_merge_range(struct cpm_range *first, struct cpm_range *second) {
	int ret = 0;

	if ( unlikely(second->type == CPM_ASM_TYPE_UNBOUNDED) )
		return 0;

	switch( first->type ) {
	case CPM_ASM_TYPE_UNBOUNDED:
		(*first) = (*second);
		break;

	case CPM_ASM_TYPE_RANGE:
		if ( first->lower < first->upper) {
		/* first is a range */
			switch( second->type ) {
			case CPM_ASM_TYPE_RANGE:
				if ( second->lower < second->upper) {
				/* second is a range */
					if ( ( first->lower > second->upper ) ||
							( first->upper < second->lower ) ) {
						ret = -EINVAL;
					} else {
						first->lower = MAX(first->lower, second->lower);
						first->upper = MIN(first->upper, second->upper);
					}
				} else {
				/* second is a single value */
					if ( cpm_verify_range(first, second->lower) == CPM_RANGE_ERROR ) {
						ret = -EINVAL;
					} else {
						(*first) = (*second);
					}
				}
				break;
			case CPM_ASM_TYPE_LBOUND:
				if ( cpm_verify_range(first, second->lower) == CPM_RANGE_ERROR ) {
					ret = -EINVAL;
				} else {
					first->lower = second->lower;
				}
				break;
			case CPM_ASM_TYPE_UBOUND:
				if ( cpm_verify_range(first, second->upper) == CPM_RANGE_ERROR ) {
					ret = -EINVAL;
				} else {
					first->upper = second->upper;
				}
				break;
			}
		} else {
		/* first is a single value */
			switch( second->type ) {
			case CPM_ASM_TYPE_RANGE:
				if ( second->lower < second->upper ) {
				/* second is a range */
					if ( cpm_verify_range(second, first->lower) == CPM_RANGE_ERROR ) {
						ret = -EINVAL;
					}
				} else {
				/* second is a single value */
					if ( first->lower != second->lower ) {
						ret = -EINVAL;
					}
				}
				break;
			case CPM_ASM_TYPE_LBOUND:
			case CPM_ASM_TYPE_UBOUND:
				if ( cpm_verify_range(second, first->lower) == CPM_RANGE_ERROR ) {
					ret = -EINVAL;
				}
				break;
			}
		}
		break;

	case CPM_ASM_TYPE_LBOUND:
		switch( second->type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( second->lower < second->upper) {
			/* second is a range */
				if ( cpm_verify_range(first, second->lower) == CPM_RANGE_ERROR ) {
					ret = -EINVAL;
				} else {
					first->lower = MAX(first->lower, second->lower);
					first->upper = second->upper;
					first->type = CPM_ASM_TYPE_RANGE;
				}
				break;

			} else {
			/* first is a single value */
				if ( cpm_verify_range(first, second->lower)
							== CPM_RANGE_ERROR ) {
					ret = -EINVAL;
				} else {
					(*first) = (*second);
				}
			}
			break;
		case CPM_ASM_TYPE_LBOUND:
			first->lower = MAX(first->lower, second->lower);
			break;
		case CPM_ASM_TYPE_UBOUND:
			if ( cpm_verify_range(first, second->upper) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				first->upper = second->upper;
				first->type = CPM_ASM_TYPE_RANGE;
			}
			break;
		}

	case CPM_ASM_TYPE_UBOUND:
		switch( second->type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( second->lower < second->upper ) {
			/* second is a range */
				if ( cpm_verify_range(first, second->lower)
							== CPM_RANGE_ERROR ) {
					ret = -EINVAL;
				} else {
					first->upper = MIN(first->upper, second->upper);
					first->lower = second->lower;
					first->type = CPM_ASM_TYPE_RANGE;
				}
				break;
			} else {
			/* first is a single value */
				if ( cpm_verify_range(first, second->lower)
							== CPM_RANGE_ERROR ) {
					ret = -EINVAL;
				} else {
					(*first) = (*second);
				}
			}
			break;
		case CPM_ASM_TYPE_LBOUND:
			if ( cpm_verify_range(first, second->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				first->lower = second->lower;
				first->type = CPM_ASM_TYPE_RANGE;
			}
			break;
		case CPM_ASM_TYPE_UBOUND:
			first->upper = MAX(first->upper, second->upper);
			break;
		}
	}

	return ret;

}
EXPORT_SYMBOL(cpm_merge_range);


static inline u32 __cpm_get_max(struct cpm_range *range, cpm_id asm_id)
{

	if ( (range->type == CPM_ASM_TYPE_UNBOUNDED) ||
			(range->type == CPM_ASM_TYPE_LBOUND) )
		return platform[asm_id].info.max;

	return range->upper;

}

static inline u32 __cpm_get_min(struct cpm_range *range, cpm_id asm_id)
{

	if ( (range->type == CPM_ASM_TYPE_UNBOUNDED) ||
			(range->type == CPM_ASM_TYPE_UBOUND) )
		return platform[asm_id].info.min;

	return range->lower;

}

int cpm_weight_range(struct cpm_range *range, cpm_id asm_id, u32 *weight)
{
	u32 value;

	if ( asm_id>=cpm_asm_count ) {
		eprintk("weighting unexisting ASM\n");
		return -EINVAL;
	}

	*weight = platform[asm_id].info.weight;

	/* small speed optimization */
	if ( unlikely( ! *weight ) )
		return 0;

	if ( *weight >= 0 )
		value = __cpm_get_max(range, asm_id);
	else
		value = __cpm_get_min(range, asm_id);

	*weight *= value;

	return 0;

}
EXPORT_SYMBOL(cpm_weight_range);

static const char *__cpm_entity_name(void *ptr, u8 type)
{
	if ( type == CPM_ENTITY_TYPE_DRIVER )
		return dev_name((struct device*)ptr);

	return ((struct task_struct*)ptr)->comm;

}

static const char *cpm_entity_name(struct cpm_entity *entity)
{

	if ( entity->type == CPM_ENTITY_TYPE_DRIVER )
		return dev_name(entity->dev);

	return entity->task->comm;
}

/******************************************************************************
 *   CPM PLATFORM                                                             *
 ******************************************************************************/

static int cpm_sysfs_core_asms_init(void);

int cpm_register_platform_asms(struct cpm_platform_data *cpd) {
	int i, result;
	struct cpm_asm *pasm;

	if (!cpd) {
		eprintk("invalid platform data");
		return -EINVAL;
	}

	mutex_lock(&asm_mutex);

	// NOTE platform data can be defined only one time
	if (platform) {
		eprintk("platform data already defined");
		result = -EBUSY;
		goto cpm_plat_asm_exist;
	}

	platform = (struct cpm_asm_core*)kzalloc((cpd->count)*sizeof(struct cpm_asm_core), GFP_KERNEL);
	if ( !platform ) {
		eprintk("out-of-mem on platform data allocation\n");
		result = -ENOMEM;
		goto cpm_platform_asm_nomem;
	}

	pasm = cpd->asms;
	for (i=0; i<cpd->count; i++) {
		platform[i].id = i;
		platform[i].info = (*pasm);
		platform[i].cur_range.lower = pasm->min;
		platform[i].cur_range.upper = pasm->max;
		INIT_LIST_HEAD(&(platform[i].constraints));
		pasm++;
	}
	cpm_asm_count = i;

	mutex_unlock(&asm_mutex);

	dprintk("platform data registered, %u ASM defined\n",
		cpm_asm_count);

	cpm_sysfs_core_asms_init();

	return 0;

cpm_plat_asm_exist:
cpm_platform_asm_nomem:
	mutex_unlock(&asm_mutex);

	return result;

}
EXPORT_SYMBOL(cpm_register_platform_asms);


/******************************************************************************
 *				CPM GOVERNORS                                 *
 ******************************************************************************/

struct cpm_fsc *cpm_get_new_fsc(void) {
	struct cpm_fsc_core *pcfsc;

	dprintk("allocate a new FSC\n");

	pcfsc = (struct cpm_fsc_core*)kzalloc(sizeof(struct cpm_fsc_core), GFP_KERNEL);
	if ( unlikely(!pcfsc) ) {
		eprintk("out-of-memory on cpm_fsc_core alloation\n");
		return 0;
	}

	return &(pcfsc->info);

}
EXPORT_SYMBOL(cpm_get_new_fsc);

static void cpm_work_update_fsc(struct work_struct *work)
{
	int result = 0;

	dprintk("START: FSC update workqueue, governor [%s]\n", governor->name);
	result = governor->build_fsc_list(&dev_list, cpm_dev_count);
	if (result) {
		eprintk("rebuild FSCs failed\n");
	} else {
		cpm_fsc_outdated = 0;
	}

	iprintk("END: FSC update workqueue\n");
}

DECLARE_WORK(cpm_work_governor, cpm_work_update_fsc);

static int cpm_update_governor(void) {
	int result = 0;

	/* return immediatly if CPM is disabled */
	if (!cpm_enabled) {
		dprintk("CPM disabled: governor update disabled\n");
		return 0;
	}

	/* a permission error is returned if the goeverno has not bee
	 * configured */
	if (!governor) {
		eprintk("governor not configured: update failed\n");
		return -EPERM;
	}

	/* let the governor do the job... */
	dprintk("schedling governor call work_queue\n");
	result = queue_work(cpm_wq, &cpm_work_governor);
	if (!result) {
		eprintk("queuing new governor work failed\n");
	}

	return result;

}

int cpm_register_governor(struct cpm_governor *cg) {

	if (!cg) {
		dprintk("governor already registered\n");
		return -EINVAL;
	}

	// TODO use MUTEX

	// TODO add governor list support?!?
	governor = cg;
	
	dprintk("new governor [%s] registered\n",
		(cg->name[0]) ? cg->name : "UNNAMED");

	return 0;
}
EXPORT_SYMBOL(cpm_register_governor);

int cpm_update_policy(void)
{
	int result;

	if ( !policy ) {
		dprintk("policy not defined: unable to obtain ordered_fsc_list\n");
		return -EINVAL;
	}

	dprintk("updating policy...\n");
	result = policy->sort_fsc_list(&fsc_list);
	if ( result ) {
		eprintk("policy update failed\n");
		return -EINVAL;
	}

	return 0;
}

static int cpm_sysfs_fsc_update(void);
int cpm_set_fsc_list(struct list_head *new_fsc_list)
{
	struct list_head old_fsc_list;
	struct list_head *node;

	if ( list_empty(new_fsc_list) ) {
		eprintk("empty FSC list: falling back to best-effort policy\n");
		//TODO disable CPM and fall-back to best-effort policy with
		//only constraints aggretation and notification...
		//Unitl either:
		// - device base change (remove some device)
		return 0;
	}


	mutex_lock(&fsc_mutex);

	/* saving a ref to old FSC list*/
	list_replace_init(&fsc_list, &old_fsc_list);

	/* updating current FSC list */
	list_replace_init(new_fsc_list, &fsc_list);

	/* Update the FSC count */
	fsc_count = 0;
	list_for_each(node, &fsc_list) {
		fsc_count++;
	}

	mutex_unlock(&fsc_mutex);

	dprintk("new FSC list received, [%d] entry\n", fsc_count);

	/* Updating ordered FSC list */
	cpm_update_policy();

#ifdef CONFIG_CPM_SYSFS
	/* release sysfs kobj */
	eprintk("TODO: implement sysfs FSC kobjects release\n");
#else
	struct cpm_fsc *pfsc;

	/* clean-up old list */
	list_for_each(node, old_fsc_list) {
		/* get pointer to contained fsc struct */
		pfsc = list_entry(node, struct cpm_fsc, node);
		/* remove this node from the list */
		list_del(node);
		/* release memory */
		kfree(pfsc);
	}
#endif

	dprintk("Updating FSC sysfs interface\n");
	cpm_sysfs_fsc_update();

	return 0;

}
EXPORT_SYMBOL(cpm_set_fsc_list);


/******************************************************************************
 *				CPM POLICIES				      *
 ******************************************************************************/

int cpm_register_policy(struct cpm_policy *cp) {

	if (!cp)
		return -EINVAL;

	//TODO use mutex
	policy = cp;

	dprintk("new policy [%s] registered\n",
		(cp->name[0]) ? cp->name : "UNNAMED");

	return 0;
}
EXPORT_SYMBOL(cpm_register_policy);


int cpm_set_ordered_fsc_list(struct list_head *fscpl_head) {
	struct list_head old_fsc_ordered_list;
	struct cpm_fsc_pointer *pcfp;

	if (!fscpl_head) {
		eprintk("trying to set empty ordered FSC list\n");
		return -EINVAL;
	}
	
	mutex_lock(&fsc_ordered_mutex);

	/* Update the new ordered FSC list */
	list_replace_init(&fsc_ordered_list, &old_fsc_ordered_list);
	list_replace_init(fscpl_head, &fsc_ordered_list);

	mutex_unlock(&fsc_ordered_mutex);

	/* Clean-up old list */
	list_for_each_entry(pcfp, &old_fsc_ordered_list, node) {
		dprintk("releasing cpm_fsc_pointer\n");
		list_del(&pcfp->node);
		kfree(pcfp);
	}

	dprintk("new FSCs ordered list registered\n");
	
	return 0;
}
EXPORT_SYMBOL(cpm_set_ordered_fsc_list);

/************************************************************************
 *	SYSFS INTERFACE							*
 ************************************************************************/

#ifdef CONFIG_CPM_SYSFS

#define CPM_KATTR_RO(_kattr,_name,_show)	\
	do {					\
		(_kattr)->attr.name = _name;	\
		(_kattr)->attr.mode = 0444;	\
		(_kattr)->show = _show;		\
		(_kattr)->store = NULL;		\
	} while(0);
#define CPM_KATTR_RW(_kattr,_name,_show,_store)	\
	do {					\
		(_kattr)->attr.name = _name;	\
		(_kattr)->attr.mode = 0664;	\
		(_kattr)->show = _show;		\
		(_kattr)->store = _store;	\
	} while(0);
#define	CPM_ADD_ATTR(_pattrs, _kattr)		\
	do {					\
		*(_pattrs) = &((_kattr).attr);	\
		(_pattrs)++;			\
	} while(0);


/* lock protects against cpm_unregister_device() being called while
 * sysfs files are active.
 */
static DEFINE_MUTEX(sysfs_lock);


static int cpm_sysfs_print_range(char *buf, ssize_t size, struct cpm_range *range)
{
	int count = 0;

	switch(range->type) {
	case CPM_ASM_TYPE_UNBOUNDED:
		count = snprintf(buf, size, "UnB");
		break;
	case CPM_ASM_TYPE_RANGE:
		if ( range->lower < range->upper ) {
			count = snprintf(buf, size, "R %u %u",
					range->lower,
					range->upper);
		} else {
			count = snprintf(buf, size, "S %u",
					range->lower);
		}
		break;
	case CPM_ASM_TYPE_LBOUND:
		count = snprintf(buf, size, "LB %u",
			range->lower);
		break;
	case CPM_ASM_TYPE_UBOUND:
		count = snprintf(buf, size, "UB %u",
			range->upper);
		break;
	}

	return count;
}

static void cpm_sysfs_dev_release(struct kobject *kobj)
{
	dprintk("TODO, cpm_sysfs_dev_release [%s]\n", kobj->name);
}

static ssize_t cpm_sysfs_dev_show(struct kobject * kobj, struct attribute *attr,
		char *buf) {
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		ret = kattr->show(kobj, kattr, buf);

	return ret;
}

static ssize_t cpm_sysfs_dev_store(struct kobject *kobj, struct attribute *attr,
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
			&cpm_dev_ktype, &(((pcd->dev_info).dev)->kobj),
			"cpm");

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

static int cpm_sysfs_dev_remove(struct cpm_dev_core *pcd)
{

	kobject_put(&((pcd->sysfs).cpm));
	kobject_put(&((pcd->sysfs).dwrs));
	kobject_put(&((pcd->sysfs).constraints));

	return 0;
}

static ssize_t cpm_asm_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct cpm_asm_range *asm_range;
	ssize_t	count = 0;

	asm_range = container_of(attr, struct cpm_asm_range, kattr);

	mutex_lock(&sysfs_lock);

	count = sprintf(buf, "%d:%s",
			asm_range->id,
			platform[asm_range->id].info.name);

	switch(asm_range->range.type) {
	case CPM_ASM_TYPE_UNBOUNDED:
		count += sprintf(buf+count, " UnB\n");
		break;
	case CPM_ASM_TYPE_RANGE:
		if ( asm_range->range.lower < asm_range->range.upper ) {
			count += sprintf(buf+count, " R %u %u\n",
				asm_range->range.lower,
				asm_range->range.upper);
		} else {
			count += sprintf(buf+count, " S %u\n",
				asm_range->range.lower);
		}
		break;
	case CPM_ASM_TYPE_LBOUND:
		count += sprintf(buf+count, " LB %u\n",
			asm_range->range.lower);
		break;
	case CPM_ASM_TYPE_UBOUND:
		count += sprintf(buf+count, " UB %u\n",
			asm_range->range.upper);
		break;
	}

	mutex_unlock(&sysfs_lock);

	return count;
}

static ssize_t cpm_dwr_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	const struct cpm_dev_dwr *dwr;
	ssize_t	status;

	dwr = container_of(attr, struct cpm_dev_dwr, kattr);

	mutex_lock(&sysfs_lock);

	status = sprintf(buf, "%s\n", dwr->name);

	mutex_unlock(&sysfs_lock);

	return status;
}



/*
 * Initialize and export to sysfs device's DWRs attributes
 */
static int cpm_sysfs_dwr_setup(struct cpm_dev_core *pcd) {
	struct cpm_dev_dwr *pdwr;
	struct cpm_asm_range *pasm;
	struct attribute **pattrs;
	int result = 0;
	u8 i, j;

	/* Init sysfs for this device */
	cpm_sysfs_dev_init(pcd);

	/* create attributes for each DWR */
	for (pdwr = pcd->dev_info.dwrs, i=0; i<pcd->dev_info.dwrs_count; pdwr++, i++) {

		/* build the attribute_group array for the ASM of this DWR */
		pdwr->asms_group.name = pdwr->name;
		pdwr->asms_group.attrs  = pattrs = (struct attribute **)kzalloc(((pdwr->asms_count)+2)*sizeof(struct attribute *), GFP_KERNEL);
		if ( !pattrs ) {
			dprintk("out-of-mem on cpm_dev_block allocation\n");
			result = -ENOMEM;
			goto crd_exit_nomem_dwr;
		}

		/* adding the DWR name attribute */
		CPM_KATTR_RO(&(pdwr->kattr), "name", cpm_dwr_show);
		CPM_ADD_ATTR(pattrs, pdwr->kattr);

		/* scanning a DWR's ASMs and adding attributes */
		pasm = pdwr->asms;
		for (j=0; j<pdwr->asms_count; j++) {
			sprintf(pasm->name, "asm%02d", pasm->id);
			CPM_KATTR_RO(&(pasm->kattr), pasm->name, cpm_asm_show);
			CPM_ADD_ATTR(pattrs, pasm->kattr);
			pasm++;
		}

		/* complete attribute group definition */
		(*pattrs) = NULL;

		/* export this DWR to sysfs */
		result = sysfs_create_group(&(pcd->sysfs.dwrs), &(pdwr->asms_group));
		if ( result ) {
			dprintk("DWR exporting failed\n");
			goto crd_exit_sysfs_dwr_failed;
		}


	}

	return 0;


crd_exit_sysfs_dwr_failed:

crd_exit_nomem_dwr:

	/* TODO add free-up code */
	dprintk("%s:%d memory-leak: free-up allocated structures\n", __FUNCTION__, __LINE__);

	return result;

}

static int cpm_sysfs_dwr_remove(struct cpm_dev_core *pcd)
{
	struct cpm_dev_dwr *pdwr;
	u8 i;

	/* releasing attributes for each DWR */
	for (pdwr = pcd->dev_info.dwrs, i=0; i<pcd->dev_info.dwrs_count; pdwr++, i++) {
		dprintk("1\n");
		sysfs_remove_group(&(pcd->sysfs.dwrs), &(pdwr->asms_group));
		dprintk("2\n");
		kfree(pdwr->asms_group.attrs);
	}
	dprintk("3\n");
	cpm_sysfs_dev_remove(pcd);

	return 0;
}

/*--- CPM Core SYSFS interface ---*/

static ssize_t cpm_sysfs_core_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	ssize_t count = 0;

	if (strcmp(attr->attr.name, "enable") == 0) {
		count = sprintf(buf, "%d\n", cpm_enabled);
	}

	return count;
}

static ssize_t cpm_sysfs_core_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{

	if (strcmp(attr->attr.name, "enable") == 0) {
		sscanf(buf, "%hu", &cpm_enabled);
		if (cpm_enabled) {
			iprintk("CPM enabled\n");
			if (cpm_fsc_outdated) {
				cpm_update_governor();
			}
		} else {
			iprintk("CPM disabled\n");
		}
	}

	return count;
}


static struct kobj_attribute cpm_sysfs_core_enable_attr =
	__ATTR(enable, 0666, cpm_sysfs_core_show, cpm_sysfs_core_store);

static struct attribute *cpm_sysfs_core_attrs[] = {
	&cpm_sysfs_core_enable_attr.attr,
	NULL,   /* need to NULL terminate the list of attributes */
};

static struct attribute_group cpm_sysfs_core_attr_group = {
	.attrs = cpm_sysfs_core_attrs,
};

static struct kobject *cpm_core_kobj;

static struct kobject *cpm_fscs_kobj;

static int __init cpm_sysfs_core_init(void)
{
	int retval;

	/* create the "cpm" sysfs' entry under /sys/kernel */
	cpm_core_kobj = kobject_create_and_add("cpm", kernel_kobj);
	if (!cpm_core_kobj) {
		eprintk("out-of-memory on \"cpm\" sysfs interface creation\n");
		return -ENOMEM;
	}

	retval = sysfs_create_group(cpm_core_kobj, &cpm_sysfs_core_attr_group);
	if (retval) {
		eprintk("\"cpm\" sysfs interface attributes initialization failed\n");
		kobject_put(cpm_core_kobj);
		goto out_sysfs_core_cpm; 
	}

	/* create the "fscs" sysfs' entry under /sys/kernel/cpm */
	cpm_fscs_kobj = kobject_create_and_add("fscs", cpm_core_kobj);
	if (!cpm_fscs_kobj) {
		eprintk("out-of-memory on \"cpm/fscs\" sysfs interface creation\n");
		retval = -ENOMEM;
		goto out_sysfs_core_fscs;
	}

	return 0;

out_sysfs_core_fscs:
	kobject_put(cpm_core_kobj);

out_sysfs_core_cpm:

	return retval;

}

static int __init cpm_sysfs_init(void)
{
	int retval;

	/* Setting-up CPM subsystem */
	retval = cpm_sysfs_core_init();
	if (retval) {
		eprintk("setting-up core sysfs interface FAILED\n");
	}

	/* Setting up ktype for <devices>/cpm and its folders */
	cpm_dev_ops.show = cpm_sysfs_dev_show;
	cpm_dev_ops.store = cpm_sysfs_dev_store;
	cpm_dev_ktype.release = cpm_sysfs_dev_release;
	cpm_dev_ktype.sysfs_ops = &cpm_dev_ops;
	cpm_dev_ktype.default_attrs = NULL;

	dprintk("sysfs interface initialized\n");

	return 0;
}

static ssize_t cpm_sysfs_core_asms_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	ssize_t count = 0;
	struct cpm_asm_core *pcasm;

	pcasm = container_of(attr, struct cpm_asm_core, kattr);

	mutex_lock(&sysfs_lock);

	count = sprintf(buf, "%u:%s %c %c %u %u", pcasm->id, pcasm->info.name,
			(pcasm->info.type == CPM_TYPE_LIB) ? 'L' : 'G',
			(pcasm->info.comp == CPM_COMPOSITION_ADDITIVE) ? 'A' : 'R',
			pcasm->info.min,
			pcasm->info.max);

	switch(pcasm->cur_range.type) {
	case CPM_ASM_TYPE_UNBOUNDED:
		count += sprintf(buf+count, " UnB\n");
		break;
	case CPM_ASM_TYPE_RANGE:
		if ( pcasm->cur_range.lower < pcasm->cur_range.upper ) {
			count += sprintf(buf+count, " R %u %u\n",
				pcasm->cur_range.lower,
				pcasm->cur_range.upper);
		} else {
			count += sprintf(buf+count, " S %u\n",
				pcasm->cur_range.lower);
		}
		break;
	case CPM_ASM_TYPE_LBOUND:
		count += sprintf(buf+count, " LB %u\n",
				pcasm->cur_range.lower);
		break;
	case CPM_ASM_TYPE_UBOUND:
		count += sprintf(buf+count, " UB %u\n",
				pcasm->cur_range.upper);
		break;
	}

	mutex_unlock(&sysfs_lock);
	
	return count;
}

static int __cpm_update_constraint(void *entity, u8 type, cpm_id asm_id, struct cpm_range *range);
static int __cpm_remove_constraint(void *entity, u8 type, cpm_id asm_id);
static ssize_t cpm_sysfs_core_asms_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int value;
	struct cpm_asm_core *pasm;
	struct cpm_range range;
	const char *entity_name;

	pasm = container_of(attr, struct cpm_asm_core, kattr);

	/* Getting user value */
	if ( !sscanf(buf, "%du", &value) ) {
		/* An empty write on sysfs attribute will remove the
		 * the eventually caller asserted constraint on that ASM */
		dprintk("constraint deassertion\n");
		return __cpm_remove_constraint((void*)current, CPM_ENTITY_TYPE_TASK, pasm->id);
	}

	if ( pasm->info.comp == CPM_COMPOSITION_ADDITIVE ) {
		dprintk("additive constraint assertion\n");
		range.lower = value;
		range.upper = pasm->info.max;
		range.type = CPM_ASM_TYPE_LBOUND;
	} else {
		dprintk("restrictive constraint assertion\n");
		range.lower = pasm->info.min;
		range.upper = value;
		range.type = CPM_ASM_TYPE_UBOUND;
	}

	if ( range.lower > range.upper ) {
		entity_name = __cpm_entity_name((void*)current, CPM_ENTITY_TYPE_TASK);
		eprintk("out-of-range constraint asserted by [%s] on ASM [%s]\n",
				entity_name, pasm->info.name);
		return count;
	}

	/* Try to add the contraints */
	__cpm_update_constraint((void*)current, CPM_ENTITY_TYPE_TASK, pasm->id, &range);

	return count;
}

struct attribute_group platform_asms_group;

static int cpm_sysfs_core_asms_init(void)
{
	int result, i;
	struct cpm_asm_core *pcasm;
	struct attribute **pattrs;

	dprintk("exporting platform ASMs...\n");

	platform_asms_group.attrs  = pattrs = (struct attribute **)kzalloc(((cpm_asm_count)+1)*sizeof(struct attribute *), GFP_KERNEL);
	if ( !pattrs ) {
		dprintk("out-of-mem on platform_asms_group.attrs allocation\n");
		result = -ENOMEM;
		goto out_sysfs_asms_init_nomem;
	}
	platform_asms_group.name = "asms";

	pcasm = platform;
	for (i=0; i<cpm_asm_count; i++) {
	
		if ( pcasm->info.userw == CPM_USER_RW) {
			CPM_KATTR_RW(&(pcasm->kattr), pcasm->info.name, cpm_sysfs_core_asms_show, cpm_sysfs_core_asms_store);
		} else {
			CPM_KATTR_RO(&(pcasm->kattr), pcasm->info.name, cpm_sysfs_core_asms_show);
		}
		CPM_ADD_ATTR(pattrs, pcasm->kattr);
		pcasm++;
	}
	/* complete attribute group definition */
	(*pattrs) = NULL;

	/* export platform ASMs to sysfs */
	result = sysfs_create_group(cpm_core_kobj, &(platform_asms_group));
	if ( result ) {
		dprintk("platform ASMs exporting failed\n");
		goto out_sysfs_asms_init_export;
	}

	return 0;


out_sysfs_asms_init_export:


out_sysfs_asms_init_nomem:


	return result;

}

struct attribute_group current_fsc_group;

/*
 * cpm_sysfs_export_asms - export the specified ASMs via sysfs
 * @pasm - the array of ASMs to export
 * @asms_count - the number of elements in the array
 * @name - the name of this group
 * @attr_group - the group of attributes to use for the export
 * @kobj - the parent kobject under witch this group will be exported
 */
static int cpm_sysfs_asms_export(struct cpm_asm_range *pasm, u8 asms_count, char *name, struct attribute_group *attr_group, struct kobject *kobj)
{
	int j, result;
	struct attribute **pattrs;

	attr_group->name = name;
	attr_group->attrs = (struct attribute **)kzalloc((asms_count+1)*sizeof(struct attribute *), GFP_KERNEL);
	if ( !attr_group->attrs  ) {
		dprintk("out-of-mem on ASMs attributes allocation\n");
		result = -ENOMEM;
		goto out_sysfs_asm_nomem;
	}

	/* Loop on ASMs array */
	pattrs = attr_group->attrs;
	for (j=0; pasm && j<asms_count; j++) {
		snprintf(pasm->name, CPM_NAME_LEN, "asm%02d", pasm->id);
		CPM_KATTR_RO(&(pasm->kattr), pasm->name, cpm_asm_show);
		CPM_ADD_ATTR(pattrs, pasm->kattr);
		pasm++;
	}

	/* complete attribute group definition */
	(*pattrs) = NULL;

	/* export this ASMs group via sysfs */
	result = sysfs_create_group(kobj, attr_group);
	if ( result ) {
		dprintk("ASMs exporting failed\n");
		goto out_sysfs_asms_export_failed;
	}

	return 0;


out_sysfs_asms_export_failed:

	kfree(attr_group->attrs);

out_sysfs_asm_nomem:

	return result;

}

static int cpm_sysfs_fsc_export(void)
{
	int result = 0;
	unsigned short i;
	struct cpm_fsc_core *pcfsc;

	dprintk("exporting new FSC list...\n");

	/* Loop on FSCs */
	i = 0;
	list_for_each_entry(pcfsc, &fsc_list, info.node) {

		/* create the asms group for this FSC */
		snprintf(pcfsc->name, CPM_NAME_LEN, "FSC%02u", pcfsc->info.id);

		/* exporting ASMs for this FSC */
		result = cpm_sysfs_asms_export(pcfsc->info.asms,
				pcfsc->info.asms_count,
				pcfsc->name,
				&(pcfsc->asms_group),
				cpm_fscs_kobj);

		/* exporting DWRs for this FSC */
		//TODO

		i++;
	}

	return 0;

}

static int cpm_sysfs_fsc_update(void)
{
	int result;

	/* Cleaning up existing FSC lists */
	//TODO

	/* Export new FSC list */
	result = cpm_sysfs_fsc_export();
	if ( result ) {
		eprintk("exporting FSCs failed\n");
		goto out_sysfs_fsc_export;
	}

	return 0;

out_sysfs_fsc_export:

	return result;
}



#else

static int __init cpm_sysfs_init(void)
{
	return 0;
}

static int cpm_sysfs_dwr_setup(struct cpm_dev_core *pcd)
{
	return 0;
}

static int cpm_sysfs_dwr_release(struct cpm_dev_core *pcd)
{
	return 0;
}

static int cpm_sysfs_core_asms_init(void)
{
	return 0;
}

#endif


/************************************************************************
 *	CPM DRIVER                          				*
 ************************************************************************/

/* Find the cpm_dev_core for the specified device, or null if the device 
 * has never registered an ASM before */
static struct cpm_dev_core *find_device_block(struct device *dev) {
	struct cpm_dev *pdev;
	struct cpm_dev_core *pcdev;

	list_for_each_entry(pdev, &dev_list, node) {
		if ( (pdev->dev) == dev ) {
			pcdev = container_of(pdev, struct cpm_dev_core, dev_info);
			return pcdev;
		}
	}

	return 0;
}

int cpm_register_device(struct device *dev, struct cpm_dev_data *data)
{
	struct cpm_dev_core *pcd;
	struct cpm_dev_dwr *pdwrs;
	struct cpm_dev_dwr *pdwr;
	struct cpm_asm_range *pasms;
	int result = 0;
	u8 i;

	/* Check if the device is already registerd */
	pcd = find_device_block(dev);
	if ( pcd != 0 ) {
		dprintk("device [%s] already registered\n", dev_name(dev));
		return -EEXIST;
	}

	dprintk("registering new device [%s]...\n", dev_name(dev));

	/* add device notifier block */
	pcd = kzalloc(sizeof(struct cpm_dev_core), GFP_KERNEL);
        if (!pcd) {
		dprintk("out-of-mem on cpm_dev_core allocation\n");
		return -ENOMEM;
        }

        // init device reference
        pcd->dev_info.dev = dev;

	// copy DWRs definitions	
	pdwrs = kzalloc(data->dwrs_count*sizeof(struct cpm_dev_dwr), GFP_KERNEL);
        if (!pdwrs) {
		dprintk("out-of-mem on cpm_dev_block allocation\n");
		result = -ENOMEM;
		goto crd_exit_nomem_dwr;
        }
	memcpy((void*)pdwrs, (void*)data->dwrs, data->dwrs_count*sizeof(struct cpm_dev_dwr));

	pcd->dev_info.dwrs = pdwrs;
	pcd->dev_info.dwrs_count = data->dwrs_count;
	
	for (pdwr = pdwrs, i=0; i<pcd->dev_info.dwrs_count; pdwr++, i++) {

		dprintk("ASM array [%p], ASM count [%d]\n", pdwr->asms, pdwr->asms_count);
		if ( ! pdwr->asms || ! pdwr->asms_count ) {
			dprintk("WARNING: uninitialized driver DWR\n");
			continue;
		}

		pasms = kzalloc(pdwr->asms_count*sizeof(struct cpm_asm_range), GFP_KERNEL);
		if (!pasms) {
			dprintk("out-of-mem on cpm_asm_range allocation\n");
			result = -ENOMEM;
			goto crd_exit_nomem_asm;
		}

		memcpy((void*)pasms, (void*)pdwr->asms, pdwr->asms_count*sizeof(struct cpm_asm_range));
		pdwr->asms = pasms;

		dprintk("added DWR [%s:%s] with %d ASMs\n",
				dev_name(dev), pdwr->name, pdwr->asms_count);
	}

	/* Init ASM list for this device */
	INIT_LIST_HEAD(&(pcd->asm_list));

	/* TODO Fill into the ASM list */

	// copy the DDP handler callback reference
	pcd->dev_info.nb.notifier_call = data->notifier_callback;

        // register to DDP notifier chain
        srcu_notifier_chain_register(&cpm_ddp_notifier_list, &((pcd->dev_info).nb));

        // add into device chain
        list_add(&(pcd->dev_info.node), &dev_list);
	cpm_dev_count++;
	cpm_fsc_outdated = 1;

	/* Setting-up sysfs interface */
	cpm_sysfs_dwr_setup(pcd);

	/* Notify governor */
	cpm_update_governor();

        dprintk("new device [%s] successfully registerd\n", dev_name(dev));
	
	return 0;


crd_exit_nomem_asm:
	do {
		kfree(pdwr);
		pdwr--;
	} while (pdwr != pdwrs);

crd_exit_nomem_dwr:
	kfree(pcd);

	return result;

}
EXPORT_SYMBOL(cpm_register_device);


static int __cpm_unregister_device(struct cpm_dev_core *pcd)
{
	struct cpm_dev_dwr *pdwr;
	u8 i;

	dprintk("a\n");
	cpm_sysfs_dwr_remove(pcd);

	dprintk("b\n");
	cpm_dev_count--;
	list_del(&(pcd->dev_info.node));

	dprintk("c\n");
	srcu_notifier_chain_unregister(&cpm_ddp_notifier_list, &((pcd->dev_info).nb));

	dprintk("d\n");
	for (pdwr = pcd->dev_info.dwrs, i=0; i<pcd->dev_info.dwrs_count; pdwr++, i++) {
		kfree(pdwr->asms);
	}

	dprintk("e\n");
	kfree(pcd->dev_info.dwrs);

	dprintk("f\n");
	kfree(pcd);

	/* Notify governor */
	cpm_fsc_outdated = 1;
	cpm_update_governor();


	return 0;

}

int cpm_unregister_device(struct device *dev)
{
	struct cpm_dev_core *pcd;

	// Check if the device is already registerd
	pcd = find_device_block(dev);
	if ( pcd == 0 ) {
		dprintk("device [%s] not registerd\n", dev_name(dev));
		return -ENODEV;
	}

	dprintk("unregistering device [%s]...\n", dev_name(dev));

	__cpm_unregister_device(pcd);

	return 0;
}
EXPORT_SYMBOL(cpm_unregister_device);


/*
 * Gieven an FSC check if is valid w.r.t. the specified ASM's ddp_range
 * return 0 on OK, -EINVAL otherwise
 */
static int cpm_verify_fsc_on_asm(struct cpm_fsc_core *fsc, cpm_id asm_id)
{
	int i, result;
	struct cpm_asm_range *pasm;
	struct cpm_range ddp_range;

	dprintk("cpm_verify_fsc_on_asm - FSC%02d, ASM [%s]...\n",
			fsc->info.id,
			platform[asm_id].info.name);

	/* Check if the specified FSC has a range for the ASM to be verified */
	pasm = fsc->info.asms;
	for (i=0; i<fsc->info.asms_count; i++) {
		if (pasm->id == asm_id) {
			break;
		}
		pasm++;
	}

	if (i==fsc->info.asms_count) {
		/* no ASM ranges for this FSC */
		dprintk("cpm_verify_fsc_on_asm - no ranges for the required ASM\n");
		return 0;
	}

	ddp_range = platform[asm_id].ddp_range;

	result = cpm_merge_range(&ddp_range, &pasm->range);
	if ( result ) {
		dprintk("cpm_verify_fsc_on_asm - ASM [%s] not merge\n",
				platform[pasm->id].info.name);
		result = -EINVAL;
	}

	dprintk("cpm_verify_fsc_on_asm - ASM [%s] merge\n",
			platform[pasm->id].info.name);

	return 0;

}

/*
 * Given an FSC check if is valid w.r.t. ddp_ranges of its ASM
 * return 0 on OK, -EINVAL otherwise
 */
static int cpm_verify_fsc(struct cpm_fsc_core *fsc)
{
	int i, result;
	struct cpm_asm_range *pasm;
	struct cpm_range ddp_range;

	dprintk("cpm_verify_fsc - FSC%02d...\n", fsc->info.id);

	pasm = fsc->info.asms;
	for (i=0; i<fsc->info.asms_count; i++) {

		ddp_range = platform[pasm->id].ddp_range;

		result = cpm_merge_range(&ddp_range, &pasm->range);
		if ( result ) {
			dprintk("cpm_verify_fsc - ASM [%s] not merge\n",
					platform[pasm->id].info.name);
			return -EINVAL;
		}

		dprintk("cpm_verify_fsc - ASM [%s] merge\n",
				platform[pasm->id].info.name);
		pasm++;
	}

	return 0;
}

/*
 * Find next valid FSC, starting from that following the specified one.
 * In case of a constraint addition (e.g. shrinking of the solution space) the
 * eventually next valid FSC must follow the current one selected in the
 * ordered FSC list
 */

static struct cpm_fsc_core *cpm_find_next_valid_fsc(struct cpm_fsc_pointer *pfscp)
{
	struct list_head *pos;
	struct cpm_fsc_core *pfscc;
	int result;

	/* Scan FSC starting from last valid FSC (current FSC) */
	for (pos = pfscp->node.next; prefetch(pos->next), pos != &fsc_ordered_list; pos = pos->next) {
		pfscp = container_of(pos, struct cpm_fsc_pointer, node);
		pfscc = container_of(pfscp->fsc, struct cpm_fsc_core, info);

		result = cpm_verify_fsc(pfscc);
		if ( !result ) {
			dprintk("next valid FSC @ [%s]\n", pfscc->name);
			return pfscc;
		}

		dprintk("FSC [%s] not valid, checking next...\n", pfscc->name);
	}

	return 0;

}

/*
 * Find first valid FSC, starting from the cpm_fsc_ordered list.
 */
static struct cpm_fsc_core *cpm_find_valid_fsc(void) {
	struct cpm_fsc_pointer *pcfp;
	struct cpm_fsc_core *pfscc;
	int result;

	list_for_each_entry(pcfp, &fsc_ordered_list, node) {
		pfscc = container_of(pcfp->fsc, struct cpm_fsc_core, info);

		result = cpm_verify_fsc(pfscc);
		if ( !result ) {
			dprintk("first valid FSC @ [%s]\n", pfscc->name);
			return pfscc;
		}

		dprintk("FSC [%s] not valid, checking next...\n", pfscc->name);
	}

	return 0;
}

static int cpm_aggregate_constraints(cpm_id asm_id, struct cpm_range *range)
{
	struct cpm_fsc_core *pfscc;
	int result;

	dprintk("aggregating constraints for ASM [%s]...\n", platform[asm_id].info.name);

	if ( platform[asm_id].info.comp == CPM_COMPOSITION_ADDITIVE ) {

		dprintk("additive contraint aggregation\n");

		/* Look for a feasible FSC according to the new range,
		 * which is obtained by adding the cur_range lower bound
		 * to the new one.
		 */
		platform[asm_id].ddp_range = platform[asm_id].cur_range;
		platform[asm_id].ddp_range.lower += range->lower;

	} else {

		/* Current constraint is more restrictive than new one */
		if ( platform[asm_id].cur_range.upper <= range->upper ) {
			dprintk("new constraint don't invalidate current FSC\n");
			return 0;
		}

		dprintk("restrictive contraint aggregation\n");

		/* Otherwise look for a feasible FSC according to the new
		 * range, which is obtained by merge of cur_range and the new
		 * one.
		 */
		platform[asm_id].ddp_range = platform[asm_id].cur_range;
		result = cpm_merge_range(&(platform[asm_id].ddp_range), range);
		if (result) {
			eprintk("BUG, unexpected constraints un-merge\n");
			return -EINVAL;
		}

	}

	if ( pcfp ) {
		/* Scan forward for next valid FSC */
		dprintk("search for next valid FSC...\n");
		pfscc = cpm_find_next_valid_fsc(pcfp);
	} else {
		/* Search the first FSC */
		dprintk("search for first valid FSC...\n");
		pfscc = cpm_find_valid_fsc();
	}

	if ( !pfscc ) {
		eprintk("no valid FSC found\n");
		return -EINVAL;
	}

	iprintk("new FSC [%s] found\n", pfscc->name);

	/* Saving the ddp_range into current ones */
	platform[asm_id].cur_range = platform[asm_id].ddp_range;

	return 0;
}

static int cpm_verify_constraint(cpm_id asm_id, struct cpm_range *range)
{

	/* Check if the required ASM exist */
	if ( asm_id>=cpm_asm_count ) {
		eprintk("constraint assert failed, ASM [%d] not existing\n", asm_id);
		return -EEXIST;
	}

	if ( platform[asm_id].info.comp == CPM_COMPOSITION_ADDITIVE ) {

		/* GiB (aka additive) constraints can only be lower bounded */
		if ( likely(range->type == CPM_ASM_TYPE_LBOUND ) )
			return 0;

		if ( range->type == CPM_ASM_TYPE_UBOUND ) {
			eprintk("upper bound asserted on GiB ASM\n");
			return -EINVAL;
		}

		/* Trasform all others in a LB */
		range->type = CPM_ASM_TYPE_LBOUND;

		if ( platform[asm_id].info.max <
			(platform[asm_id].cur_range.lower + range->lower) ) {
			eprintk("constraint exceeding ASM platform's ranges\n");
			return -EINVAL;
		}

	} else {

		/* LiB (aka restrictive) constraints can only be upper bounded */
		if ( likely(range->type == CPM_ASM_TYPE_UBOUND ) )
			return 0;

		if ( range->type == CPM_ASM_TYPE_LBOUND ) {
			eprintk("lower bound asserted on LiB ASM\n");
			return -EINVAL;
		}

		/* Trasform all others in a LB */
		range->type = CPM_ASM_TYPE_UBOUND;

		if ( range->upper < platform[asm_id].info.min ) {
			eprintk("constraint exceeding ASM platform's ranges\n");
			return -EINVAL;
		}

	}

	return 0;
}

static int __cpm_update_constraint(void *entity, u8 type, cpm_id asm_id, struct cpm_range *range)
{
	struct cpm_constraint *pconstr;
	char str[64];
	int result;
	const char *entity_name;

	entity_name = __cpm_entity_name(entity, type);


	/* Sanity check the required constraint */
	result = cpm_verify_constraint(asm_id, range);
	if ( result ) {
		dprintk("verify failed on constraint assert\n");
		return result;
	}

	cpm_sysfs_print_range(str, 64, range);
	iprintk("[%s] constraint assert on %d:%s %s\n",
			entity_name, asm_id,
			platform[asm_id].info.name,
			str);

	/* Trying constraints aggregation */
	result = cpm_aggregate_constraints(asm_id, range);
	if ( result ) {
		/* No valid FSC found: constraint sould be rejected */
		return result;
	}

	/* Check if the entity has already asserted a constraint for the ASM */
	list_for_each_entry(pconstr, &platform[asm_id].constraints, node) {
		if ( entity == pconstr->entity.ptr ) {
			dprintk("already asserted...\n");
			break;
		}
	}

	/* Allocate a new constraint for this device if necessary */
	if ( &pconstr->node == &platform[asm_id].constraints ) {

		dprintk("allocate a new constraint\n");
		pconstr = (struct cpm_constraint*)kzalloc(sizeof(struct cpm_constraint), GFP_KERNEL);
		if ( !pconstr ) {
			eprintk("out-of-mem on new contraint allocation\n");
			result = -ENOMEM;
			goto out_constr_nomem;
		}

	}

	/* Update the constraints values */
	pconstr->entity.type = type;
	pconstr->entity.ptr = entity;
	pconstr->range = *range;

	/* Add the constraint to the list for this ASM */
	list_add(&pconstr->node, &platform[asm_id].constraints);

	/* Start DDP */
	iprintk("New FSC found, starting DDP...\n");
	/* TODO */

	return 0;

out_constr_nomem:

	return result;

}

static int cpm_device_registered(struct device *dev)
{
	struct cpm_dev_core *pcd;

	/* Check if the device is already registerd */
	pcd = find_device_block(dev);
	if ( !pcd ) {
		dprintk("constraint assert failed, device [%s] not registerd\n", dev_name(dev));
		return -EEXIST;
	}

	return 0;
}

int cpm_update_constraint(struct device *dev, cpm_id asm_id, struct cpm_range *range)
{
	int result;

	result = cpm_device_registered(dev);
	if ( result )
		return result;

	return __cpm_update_constraint((void*)dev, CPM_ENTITY_TYPE_DRIVER, asm_id, range);

}
EXPORT_SYMBOL(cpm_update_constraint);

static int __cpm_remove_constraint(void *entity, u8 type, cpm_id asm_id)
{
	struct cpm_constraint *pconstr;
	const char *entity_name;
	char str[64];

	entity_name = __cpm_entity_name(entity, type);

	/* Check if the required ASM exist */
	if ( asm_id>=cpm_asm_count ) {
		eprintk("constraint assert failed, ASM [%d] not existing\n", asm_id);
		return -EEXIST;
	}

	/* Check if the entity has asserted a constraint for the ASM */
	list_for_each_entry(pconstr, &platform[asm_id].constraints, node) {
		if ( entity == pconstr->entity.ptr ) {
			cpm_sysfs_print_range(str, 64, &pconstr->range);
			iprintk("[%s] releasing constraint on %d:%s %s\n",
				entity_name, asm_id,
				platform[asm_id].info.name,
				str);
			break;
		}
	}

	/* Remove the constraint for this entity only if necessary */
	if ( &pconstr->node == &platform[asm_id].constraints ) {
		return 0;
	}

	/* Removing the constraint from the list for this ASM */
	list_del(&pconstr->node);

	/* Releasing object */
	kfree(pconstr);

	/* Looking for new FSC */
	/* TODO */

	/* Start DDP */
	iprintk("New FSC found, starting DDP...\n");
	/* TODO */

	return 0;
}

int cpm_remove_constraint(struct device *dev, cpm_id asm_id)
{
	int result;

	result = cpm_device_registered(dev);
	if ( result )
		return result;

	return __cpm_remove_constraint((void*)dev, CPM_ENTITY_TYPE_DRIVER, asm_id);

}
EXPORT_SYMBOL(cpm_remove_constraint);




static int __init init_cpm_ddp_notifier_list(void)
{
	srcu_init_notifier_head(&cpm_ddp_notifier_list);
	return 0;
}
pure_initcall(init_cpm_ddp_notifier_list);


static int __init cpm_core_setup_workqueue(void)
{

	cpm_wq = create_singlethread_workqueue("cpm-core");
	dprintk("singlethread workqueue configured\n");

	return 0;
}

static int __init cpm_core_init(void)
{

	cpm_core_setup_workqueue();

	cpm_sysfs_init();

	dprintk("CPM Core initialized\n");
	return 0;
}
core_initcall(cpm_core_init);




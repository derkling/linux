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

struct cpm_dev_core {
	struct cpm_dev dev_info;	/* the public accessible data */
	struct list_head asm_list;	/* list of cpm_asm_map that belongs to at least one DWR */
#ifdef CONFIG_CPM_SYSFS
	struct cpm_dev_sysfs sysfs;		/* the sysfs interface */
#endif
};

/* cpm_dev_ktype - the ktype of each device registerd to CPM */
struct kobj_type cpm_dev_ktype;
struct sysfs_ops cpm_dev_ops;

struct cpm_constraint_request {
	struct cpm_range range;
#define CPM_CONSTRAINT_APP	0
#define CPM_CONSTRAINT_DRV	1
	u8 type:1;
	union {
		struct process* app;
		struct device* drv;
	}; 			/* A reference to the request applicant
				   ("current" for APP, device* for DRV) */
	struct list_head node;	/* The next request */
};

struct cpm_constraint {
	cpm_id id;			/* The index of the ASM */
	struct cpm_range cur;		/* The current tighter range for the ASM */
	struct list_head *requests;	/* The requested values for this ASM */
};

/* Platform specific ASMs */
static struct cpm_platform_data *platform = 0;

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

/* The notifier list used for DDP */
static struct srcu_notifier_head cpm_ddp_notifier_list;

/* The number of devices subscribed (i.e. dev_list entries) */
static u8 cpm_dev_count = 0;

/* The DDP Mutex */
DEFINE_MUTEX(ddp_mutex);




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
EXPORT_SYMBOL(cpm_verify_range);

/*
 * Merge two cpm_range
 * @first:	first cpm_range, also used to return the result
 * @second:	second cpm_range to be merged with the first
 *
 * Return values: -EINVAL if the two ranges can't be merged, 0 if merge is ok
 */
int merge_cpm_range(struct cpm_range *first, struct cpm_range *second) {
	int ret = 0;

	switch( first->type ) {

	case CPM_ASM_TYPE_RANGE:
		switch( second->type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( ( first->lower > second->upper ) ||
					( first->upper < second->lower ) ) {
				ret = -EINVAL;
			} else {
				first->lower = MAX(first->lower, second->lower);
				first->upper = MIN(first->upper, second->upper);
			}
			break;
		case CPM_ASM_TYPE_SINGLE:
			if ( cpm_verify_range(first, second->lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				first->lower = second->lower;
				first->type = CPM_ASM_TYPE_SINGLE;
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
		break;

	case CPM_ASM_TYPE_SINGLE:
		switch( second->type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( cpm_verify_range(second, first->lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			}
			break;
		case CPM_ASM_TYPE_SINGLE:
			if ( first->lower != second->lower ) {
				ret = -EINVAL;
			}	
			break;
		case CPM_ASM_TYPE_LBOUND:
		case CPM_ASM_TYPE_UBOUND:
			if ( cpm_verify_range(second, first->lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			}
			break;
		}
		break;

	case CPM_ASM_TYPE_LBOUND:
		switch( second->type ) {
		case CPM_ASM_TYPE_RANGE:
			if ( cpm_verify_range(first, second->lower) == CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				first->lower = MAX(first->lower, second->lower);
				first->upper = second->upper;
				first->type = CPM_ASM_TYPE_RANGE;
			}
			break;

		case CPM_ASM_TYPE_SINGLE:
			if ( cpm_verify_range(first, second->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				first->lower = second->lower;
				first->type = CPM_ASM_TYPE_SINGLE;
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
			if ( cpm_verify_range(first, second->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				first->upper = MIN(first->upper, second->upper);
				first->lower = second->lower;
				first->type = CPM_ASM_TYPE_RANGE;
			}
			break;
		case CPM_ASM_TYPE_SINGLE:
			if ( cpm_verify_range(first, second->lower)
						== CPM_RANGE_ERROR ) {
				ret = -EINVAL;
			} else {
				first->lower = second->lower;
				first->type = CPM_ASM_TYPE_SINGLE;
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
EXPORT_SYMBOL(merge_cpm_range);


/******************************************************************************
 *   CPM PLATFORM                                                             *
 ******************************************************************************/

int cpm_register_platform_asms(struct cpm_platform_data *cpd) {
	
	if (!cpd)
		return -EINVAL;

	// NOTE platform data can be defined onyl one time
	if (platform)
		return -EBUSY;

	// TODO use MUTEX
	platform = cpd;

	// TODO allocate cpm_ranges arrays?!?

	dprintk("platform data registered, %u ASM defined\n",
		platform->count);

	return 0;
}
EXPORT_SYMBOL(cpm_register_platform_asms);


/******************************************************************************
 *				CPM GOVERNORS                                 *
 ******************************************************************************/

int cpm_register_governor(struct cpm_governor *cg) {

	if (!cg)
		return -EINVAL;

	// TODO use MUTEX

	// TODO add governor list support?!?
	governor = cg;
	
	dprintk("new governor [%s] registered\n",
		(cg->name[0]) ? cg->name : "UNNAMED");

	return 0;
}
EXPORT_SYMBOL(cpm_register_governor);


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


int cpm_set_ordered_fsc_list(struct list_head *cfsc){

	if (!cfsc)
		return -EINVAL;
	
	//TODO clean previous fsc list? or made it somewhere else?
	fsc_list = *cfsc;

	dprintk("new FSCs ordered list registered\n");
	
	return 0;
}
EXPORT_SYMBOL(cpm_set_ordered_fsc_list);

/************************************************************************
 *	SYSFS INTERFACE							*
 ************************************************************************/

#ifdef CONFIG_CPM_SYSFS

#define CPM_DATTR_RO(_kattr,_name,_show)	\
	do {					\
		(_kattr)->attr.name = _name;	\
		(_kattr)->attr.mode = 0444;	\
		(_kattr)->show = _show;		\
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
	ssize_t	status = 0;

	asm_range = container_of(attr, struct cpm_asm_range, kattr);

	mutex_lock(&sysfs_lock);

	switch(asm_range->range.type) {
	case CPM_ASM_TYPE_RANGE:
		status = sprintf(buf, "%d:%s (R) %u %u\n",
			asm_range->id,
			platform->asms[asm_range->id].name,
			asm_range->range.lower,
			asm_range->range.upper);
		break;
	case CPM_ASM_TYPE_SINGLE:
		status = sprintf(buf, "%d:%s (S) %u\n",
			asm_range->id,
			platform->asms[asm_range->id].name,
			asm_range->range.lower);
		break;
	case CPM_ASM_TYPE_LBOUND:
		status = sprintf(buf, "%d:%s (L) %u\n",
			asm_range->id,
			platform->asms[asm_range->id].name,
			asm_range->range.lower);
		break;
	case CPM_ASM_TYPE_UBOUND:
		status = sprintf(buf, "%d:%s (U) %u\n",
			asm_range->id,
			platform->asms[asm_range->id].name,
			asm_range->range.upper);
		break;
	}

	mutex_unlock(&sysfs_lock);

	return status;
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
		CPM_DATTR_RO(&(pdwr->kattr), "name", cpm_dwr_show);
		CPM_ADD_ATTR(pattrs, pdwr->kattr);

		/* scanning a DWR's ASMs and adding attributes */
		pasm = pdwr->asms;
		for (j=0; j<pdwr->asms_count; j++) {
			sprintf(pasm->name, "asm%02d", pasm->id);
			CPM_DATTR_RO(&(pasm->kattr), pasm->name, cpm_asm_show);
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



static int __init cpm_sysfs_init(void)
{

	/* Setting-up CPM subsystem */
	/* TODO */

	/* Setting up ktype for <devices>/cpm and its folders */
	cpm_dev_ops.show = cpm_sysfs_dev_show;
	cpm_dev_ops.store = cpm_sysfs_dev_store;
	cpm_dev_ktype.release = cpm_sysfs_dev_release;
	cpm_dev_ktype.sysfs_ops = &cpm_dev_ops;
	cpm_dev_ktype.default_attrs = NULL;

	dprintk("sysfs interface initialized\n");

	return 0;
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

	// Check if the device is already registerd
	pcd = find_device_block(dev);
	if ( pcd != 0 ) {
		dprintk("device [%s] already registered\n", dev_name(dev));
		return -EEXIST;
	}

	dprintk("registering new device [%s]...\n", dev_name(dev));

	// add device notifier block
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
	
	// copy the DDP handler callback reference
	pcd->dev_info.nb.notifier_call = data->notifier_callback;

        // register to DDP notifier chain
        srcu_notifier_chain_register(&cpm_ddp_notifier_list, &((pcd->dev_info).nb));

        // add into device chain
        list_add(&(pcd->dev_info.node), &dev_list);
	cpm_dev_count++;

	/* Setting-up sysfs interface */
	cpm_sysfs_dwr_setup(pcd);

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

int cpm_add_constraint(struct device *dev, cpm_id asm_id, struct cpm_range * range)
{
	return 0;
}
EXPORT_SYMBOL(cpm_add_constraint);

int cpm_update_constraint(struct device *dev, cpm_id asm_id, struct cpm_range * range)
{
	return 0;
}
EXPORT_SYMBOL(cpm_update_constraint);

int cpm_remove_constraint(struct device *dev, cpm_id asm_id)
{
	return 0;
}
EXPORT_SYMBOL(cpm_remove_constraint);




static int __init init_cpm_ddp_notifier_list(void)
{
	srcu_init_notifier_head(&cpm_ddp_notifier_list);
	return 0;
}
pure_initcall(init_cpm_ddp_notifier_list);


static int __init cpm_core_init(void)
{

	cpm_sysfs_init();

	dprintk("CPM Core initialized\n");
	return 0;
}
core_initcall(cpm_core_init);




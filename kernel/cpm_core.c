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

#include <linux/cpm.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/list.h>

#define dprintk(msg...) cpm_debug_printk(CPM_DEBUG_CORE, \
						"cpm-core", msg)

struct cpm_asm_map {
	cpm_id id;			/* the ID of the ASM mapped */
	list_head node;
}

struct cpm_dev_core {
	struct cpm_dev dev;		/* the public accessible data */
	struct notifier_block nb;	/* the notifer chain block */
	list_head asm_list;		/* list of cpm_asm_map that belongs to at least one DWR */
}

struct cpm_fsc_dwr {
	struct cpm_dev_dwr *dwr;	/* a DWR mapping to an FSC */
	struct cpm_dev_core *dev;	/* the device to which this DWR belongs */
	list_head node;			/* the DWR for the next device */
}

struct cpm_fsc_core {
	struct cpm_fsc fsc;	/* public FSC data */
	list_head dwr_list;	/* the list of cpm_fsc_dwr that maps to this FSC */
}

struct cpm_constraint_request {
	cpm_range range;
#define CPM_CONSTRAINT_APP	0
#define CPM_CONSTRAINT_DRV	1
	u8 type:1;
	union {
		struct process* app;
		struct device* drv;
	} 			/* A reference to the request applicant
				   ("current" for APP, device* for DRV) */
	list_head node;		/* The next request */
}

struct cpm_constraint {
	cpm_id id;		/* The index of the ASM */
	cpm_range cur;		/* The current tighter range for the ASM */
	list_head *requests;	/* The requested values for this ASM */
}

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
static u8 cpm_nb_count = 0;

/* The DDP Mutex */
DEFINE_MUTEX(ddp_mutex);




/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/
#ifdef CONFIG_CPM_DEBUG

/* what part(s) of the CPM subsystem are debugged? */
static unsigned int debug;

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
	unsigned long flags;

	mutex_lock(&disable_ratelimit_mutex);
	if (disable_ratelimit)
		disable_ratelimit--;
	mutex_unlock(&disable_ratelimit_mutex);
}

static void cpm_debug_disable_ratelimit(void)
{
	unsigned long flags;

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
	unsigned long flags;

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

/**
 * Update the range for a specified ASM.
 */
int cpm_update_ddp_range(struct cpm_range asms[], cpm_id idx, struct cpm_range *range) {
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


/* Get a reference to the ASM corresponding to the specified id
 * or NULL if it don't exist */
static struct cpm_asm * cpm_get_asm(cpm_id asm_id) {

	WARN_ON();
	if (!platform) {
		WARN_ON
		return 0;
	}

	if (asm_id > platform->count) 
}

static struct cpm_range

/*
 * Verify if a specified device should be notified during a DDP
 * A device has to be notified only if one of the ASM to witch it is
 * subscribed has been updated since last time it has observed its values.
 * To identify last time we seen an ASM value its sequence number is used.
 */
static int __ddp_prehandler(struct notifier_block * nb, unsigned long run, void * ddp_data) {
	struct cpm_dev_core * cdb = 0; 
	struct cpm_asm_map * asm = 0;
	int ret = NOTIFY_JUMP;

	cdb = container_of(nb, struct cpm_dev_core, nb);

	/* check if it exist an ASM that has been updated after last time this
	 * device has checked it (ddp seq number has increased) */
	list_for_each_entry(asm, cdb->asm_list, node) {
		if (asm->seq < (platform->asms)[asm->id].ddp.seq) {
			ret = NOTIFY_OK;
			break;
		}
	}

	if ( ret == NOTIFY_OK ) {
		/* call the (eventually defined) governor pre-handler */
		if (governor->ddp_prehandler)
			governor->ddp_prehandler(nb, run, ddp_data);
	}

	/* all ASMs to witch this device has subscribed has not been updated since
	 * last call: this JUMP this device callback */
	return ret;
}

static int __ddp_posthandler(struct notifier_block * nb, unsigned long run, void * ddp_data) {
	
	if (governor->ddp_posthandler)
		governor->ddp_posthandler(nb, run, ddp_data);
	
	return 0;
}


/******************************************************************************
 *   CPM PLATFORM                                                             *
 ******************************************************************************/

int cpm_register_platform_asms(cpm_platform_data *cpd) {
	
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
 *   CPM GOVERNORS                                                            *
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


/*********************************************************************
 *                      CPM DRIVER                                   *
 *********************************************************************/

/* Find the cpm_dev_core for the specified device, or null if the device 
 * has never registered an ASM before */
static struct cpm_dev_core* find_device_block(struct device *dev) {
	struct cpm_dev_core * nb;

	list_for_each_entry(nb, dev_list, node) {
		if (nb->dev==dev)
			return nb;
	}

	return 0;
}

static struct cpm_asm_map * find_asm_map(struct cpm_dev_core * nb, cpm_id id) {
	struct cpm_asm_map * asm;

	list_for_each_entry(asm, nb->asm_list, node) {
		if (asm->id==id)
			return asm;
	}

	return 0;
}

int cpm_register_dwr(struct device *dev, list_head *dwrs)
{
	return 0;
}
EXPORT_SYMBOL(cpm_register_dwr);

int cpm_release_dwr(struct device *dev)
{
	return 0;
}
EXPORT_SYMBOL(cpm_release_dwr);

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

	dprintk("CPM Core initialized");
	return 0;
}
core_initcall(cpm_core_init);




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
	char name[CPM_NAME_LEN];
	cpm_asm_id id;			/* the ID of the ASM mapped */
	u8 type:1;			/* interest (affect/influenced) */
	u32 seq;			/* last ASM's seq viewed during a DDP */
	struct list_head node;
}

struct cpm_dev_stats {
	u32 ddp_start_count;		/* How many time the device has started a ddp */
	u32 ddp_done_count;
	u32 ddp_ok_count;
	u32 ddp_bad_count;
	struct timespec avg_resp_time;
}

struct cpm_dev_block {
	struct device *dev;		/* the device interested */
	struct notifier_block nb;
	struct list_head asm_list;	/* list of cpm_asm_map */
	struct list_head node;		/* the next node of this type */
	struct cpm_dev_stats stats;	/* statistics on DDP */
	u8 ddp_suspended:1;		/* the device has a DDP pending */ 
}

struct cpm_ddp_failures {
	u32 not_agreement;	/* number of failures due to not-agreement */
	u32 loops;		/* number of failures due to loops detection */
}

struct cpm_ddp_duration {
	struct timespec min;	/* minimum time duration */
	struct timespec max;	/* maximum time duration */
	struct timespec avg;	/* average time duration */
}

struct cpm_ddp_iterations {
	u32 min;	/* minimum number of steps */
	u32 max;	/* maximum number of steps */
	u32 avg;	/* average numner of steps */
}

struct cpm_ddp_stats {
	u32 count;			/* total number of DDP runned */
	u32 success;			/* number of successful ddp */
	struct cpm_ddp_failures failures;
	struct cpm_ddp_duration duration;
	struct cpm_ddp_iterations steps;
}


struct cpm_stats {
	struct cpm_ddp_stats ddp;	/* statistics on DDP */
}


struct cpm_ddp_core_data {
	struct cpm_dev_block *cdb_last_update;  /* device returend a conditional agreement */ 
	int last_dev_resp;			/* the responce of last device notified */
	struct cpm_ddp_data gov_data;		/* governor and drivers DDP data */
}

/* Platform specific ASMs */
static struct cpm_platform_data *platform = 0;

/* The governor in use */
static struct cpm_governor *governor = 0;

/* The governor's policy in use */
static struct cpm_policy *policy = 0;

/* The list of devices subscribed to some ASM */
static LIST_HEAD(dev_list);

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

/* Get a reference to the ASM corresponding to the specified id
 * or NULL if it don't exist */
static struct cpm_asm * cpm_get_asm(cpm_asm_id asm_id) {

	WARN_ON();
	if (!platform) {
		WARN_ON
		return 0;
	}

	if (asm_id > platform->count) 
}

/*
 * Verify if a specified device should be notified during a DDP
 * A device has to be notified only if one of the ASM to witch it is
 * subscribed has been updated since last time it has observed its values.
 * To identify last time we seen an ASM value its sequence number is used.
 */
static int __ddp_prehandler(struct notifier_block * nb, unsigned long run, void * ddp_data) {
	struct cpm_dev_block * cdb = 0; 
	struct cpm_asm_map * asm = 0;
	int ret = NOTIFY_JUMP;

	cdb = container_of(nb, struct cpm_dev_block, nb);

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

/* Find the cpm_dev_block for the specified device, or null if the device 
 * has never registered an ASM before */
static struct cpm_dev_block* find_device_block(struct device *dev) {
	struct cpm_dev_block * nb;

	list_for_each_entry(nb, dev_list, node) {
		if (nb->dev==dev)
			return nb;
	}

	return 0;
}

static struct cpm_asm_map * find_asm_map(struct cpm_dev_block * nb, cpm_asm_id id) {
	struct cpm_asm_map * asm;

	list_for_each_entry(asm, nb->asm_list, node) {
		if (asm->id==id)
			return asm;
	}

	return 0;
}

int cpm_subscribe_asms(struct device *dev, struct cpm_dev_data dev_data[]) {
	struct cpm_dev_block * cdb = 0;
	struct cpm_asm_map * asm = 0;
	int ret = 0;
	u8 idx = 0;

	// Look for the device being already registerd
	cdb = find_device_block(dev);
	if ( !cdb ) { // if dev not registerd
		// add device notifier block
		cdb = kzalloc(sizeof(struct cpm_dev_block), GFP_KERNEL);
		if (!cdb) {
			dprintk("out-of-mem on cpm_dev_block allocation\n");
			ret = -ENOMEM;
			goto nomem_out;
		}

		// init data
		cdb->dev = dev;
		LIST_HEAD_INIT(cdb->asm_list);
		cdb->nb->pre_handler = __ddp_prehandler;
		cdb->nb->post_handler = __ddp_posthandler;

		// register to DDP notifier chain
		srcu_notifier_chain_register(&cpm_ddp_notifier_list, &(cdb->nb));

		// add into device chain
		list_add(cdb, dev_list);

		dprintk("new device [%s] registerd\n", dev->name);

	}
	
	for (idx=0; idx<dev_data->count; idx++) {
	
		// verify that the the ASM exist
		if ( unlikely(dev_data[idx].id > platform.count) ) {
			dprintk("Warning: invalid asm request [%s:%d(%s)]\n",
			dev->name, asm->id, asm->name);
			ret = -EINVAL;
			goto asm_invalid_out;
		}

		// checking if the ASM has already been registerd
		asm = find_asm_map (cdb, dev_data[idx].id);
		if (unlikely(asm)) {
			dprintk("Warning: duplicated asm mapping [%s:%d(%s)]\n",
			dev->name, asm->id, asm->name);
			ret = -EEXIST;
			goto asm_duplicate_out;
		}

		// add ASM notifier block
		asm = kzalloc(sizeof(struct cpm_asm_map), GFP_KERNEL);
		if (!asm) {
			dprintk("out-of-mem on cpm_asm_map allocation\n");
			ret = -ENOMEM;
			goto nomem_out;
		}

		if ( dev_data[idx].name[0] ) {
			strncpy(&(asm->name), dev_data[idx].name, CPM_NAME_LEN);
		} else {
			strncpy(&(asm->name), "Undef", CPM_NAME_LEN);
		}

		asm->id = dev_data[idx].id;
		asm->type = dev_data[idx].type;
		asm->seq = (platform->asms)[asm->id].ddp.seq;

		list_add(asm, cdb->asm_list);

		dprintk("new asm %s association [%s:%d(%s)]\n",
			( asm->id == CPM_INTEREST_AFFECT ) ?
				"AFFECT" : "INFLUENCE",
			dev->name, asm->id, asm->name);
	}

	return 0;

asm_duplicate_out:
asm_invalid_out:
nomem_out:
	return ret;

}
EXPORT_SYMBOL(cpm_subscribe_asms);

int cpm_unsubscribe_asms(struct device *dev, cpm_asm_id asm_id) {
	struct cpm_dev_block * cdb = 0;
	struct cpm_asm_map * asm = 0;
	int ret = 0;

	// Look for the device
	cdb = find_device_block(dev);
	if ( !cdb ) { // if dev not registerd
		dprintk("Warning: unmapping asm for unregistered device [%s:%d(%s)]\n",
			dev->name, asm->id, asm->name);
		return -EINVAL;
	}

	// Update device notifier block
	asm = find_asm_map(cdb, id);
	if (unlikely(!asm)) {
		dprintk("Warning: unmapping unregisterd asm [%s:%d(%s)]\n",
			dev->name, asm->id, asm->name);
		return -EINVAL;
	}

	// Removing ASM from device list
	list_del(asm);

	// Release memory
	kfree(asm);

	dprintk("removed asm %s association [%s:%d(%s)]\n",
			( asm->id == CPM_INTEREST_AFFECT ) ?
				"AFFECT" : "INFLUENCE",
			dev->name, asm->id, asm->name);

	return 0;

}
EXPORT_SYMBOL(cpm_unsubscribe_asms);

int cpm_release_asms(struct device *dev) {
	struct cpm_dev_block * cdb = 0;
	struct cpm_asm_map * asm = 0;
	int ret = 0;

	// Look for the device
	cdb = find_device_block(dev);
	if ( !cdb ) { // if dev not registerd
		dprintk("Warning: unmapping asm for unregistered device [%s]\n",
			dev->name);
		return -EINVAL;
	}

	// Releasing all ASMs
	list_for_each_entry(asm, cdb->asm_list, node) {
		list_del(asm);
		kfree(asm);
	}

	kfree(cdb);

	return 0;

}
EXPORT_SYMBOL(cpm_release_asms);

/*
 * Start a DDP from the specified device.
 *
 * @cdb the first device to query
 */
static int cpm_run_ddp(struct cpm_ddp_core_data * ddp_data) {
	int ret = 0;

	/* Query all devices untill either all agree (return DONE) or one abort */
	/* (return BAD)                                                         */
	do {

		ret = srcu_notifier_call_chain(&cpm_ddp_notifier_list, CPM_RUN_DDP, &(ddp_data->gov_data->drv_data));
		if ( ret == NOTIFY_BAD )
			break;


/*
 *                list_for_each_entry(cdb, dev_list, node) {
 *
 *                        [> checking if this device sould be notified <]
 *                        ret = __ddp_prehandler(nb, CPM_RUN_DDP, ddp_data); 
 *
 *                        if ( ret == NOTIFY_OK ) {
 *                                
 *                        }
 *
 *
 *                }
 */
	} while (cdb_last_update!=0);

}

/*
 * Request an ASM update
 *
 * @dev device that request the update
 * @asm_id index of the ASM to update
 * @range proposed update for the ASM
 */
int cpm_update_asm(struct device *dev, cpm_asm_id asm_id, struct cpm_range * range) {
	struct cpm_dev_block *cdb = 0;
	struct cpm_ddp_core_data ddp_data;
	int ret = 0;

	/* Seek if the requesting device is registerd */
	ret = -ENODEV;
	list_for_each_entry(cdb, dev_list, node) {
		if ( cdb->dev == dev ) {
			/* TODO add info about the update ASM range */
			dprintk("ASM update [%s] from device [%s:%d(%s)]\n",
					(platform->asms[asm_id]).name,
					dev->name, asm->id, asm->name);
			ret = 0;
			break;
		}
	}

	if ( ret != 0 ) {
		dprintk("ASM update requested from unregistered device [%s:%d(%s)]\n",
					(platform->asms[asm_id]).name,
					dev->name, asm->id, asm->name);
	}

	ret = mutex_trylock(ddp_mutex);
	if ( ret == 0 ) {
		dprintk("ASM update overlapping [%s:%d(%s)]\n",
					(platform->asms[asm_id]).name,
					dev->name, asm->id, asm->name);
		cdb->ddp_suspended = TRUE;
		return -EBUSY;
	}

	/* Verify ASM update feasibility */
	// TODO

	/* Request governor authorization for ASM update */
	if (governor->update_validate)
		governor->update_validate(n);

	/* Init DDP data */
	ddp_data.cdb_last_update = 0;
	ddp_data.last_dev_resp = NOTIFY_BAD;
	ddp_data.notify_data.steps = 0;
	
	if (governor->ddp_init) {
		governor->ddp_init(&ddp_data.gov_data);
	}

	/* Update the ASM range as required by the device */
	// TODO 
	
	/* Run a DDP */
	ret = cpm_run_ddp(&ddp_data);

	return 0;
}
EXPORT_SYMBOL(cpm_update_asm);



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




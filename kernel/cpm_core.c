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

#define dprintk(msg...) cpm_debug_printk(CPM_DEBUG_CORE, \
						"cpm-core", msg)

struct cpm_asm_map {
	struct cpm_asm_params asm; /* driver specific data */
	u8 seq; /* last ASM's seq viewed during current DDP */
	struct list_head node;
}

struct cpm_notifier_block {
	struct device *dev;		/* the device interested */
	struct notifier_block nb;
	struct list_head asm_list;	/* list of cpm_asm_map */
	struct list_head node;		/* the next node of this type */
}

struct cpm_ddp_failures {
	u32 not_agreement;
	u32 loops;
}

struct cpm_ddp_length {
	struct timespec min;
	struct timespec max;
	struct timespec avg;
}

struct cpm_ddp_iterations {
	u32 count;
	u32 avg;
}

struct cpm_ddp_stats {
	u32 success;			/* number of successful ddp */
	struct cpm_ddp_failures failures;
	struct cpm_ddp_length lenght;
	struct cpm_ddp_iterations steps;
}

struct cpm_dev_stats {
	u32 ddp_start_count;		/* How many time the device has started a ddp */
	u32 ddp_done_count;
	u32 ddp_ok_count;
	u32 ddp_bad_count;
	struct timespec avg_resp_time;
}

struct cpm_stats {
	struct cpm_ddp_stats;
}


/* Platform specific ASMs */
static struct cpm_platform_data *platform_data;

/* The governor in use */
static struct cpm_governor *governor;

/* The governor's policy in use */
static struct cpm_policy *policy;

/* The list of devices subscribed to some ASM */
static LIST_HEAD(notifier_list);

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

/*********************************************************************
 *                      CPM PLATFORM INTERFACE                       *
 *********************************************************************/

int cpm_register_platform_asms(cpm_platform_data *cpd) {
	
	BUG_ON(platform_data);
	platform_data = cpd;
	
	dprintk("platform data registered, %u ASM defined\n",
		platform_data->count);
	
	return 0;
}
EXPORT_SYMBOL(cpm_register_platform_asms);


/*********************************************************************
 *                      CPM DRIVER INTERFACE                         *
 *********************************************************************/

/* Find the cpm_notifier_block for the specified device, or null if the device 
 * has never registered an ASM before */
static struct cpm_notifier_block* find_notifier_block(struct device *dev) {
	struct cpm_notifier_block * nb;

	list_for_each_entry(nb, notifier_list, node) {
		if (nb->dev==dev)
			return nb;
	}

	return 0;
}

static struct cpm_asm_map * find_asm_map(struct cpm_notifier_block * nb, cpm_asm_id id) {
	struct cpm_asm_map * asm;

	list_for_each_entry(asm, nb->asm_list, node) {
		if (asm->asm.id==id)
			return asm;
	}

	return 0;
}

int cpm_subscribe_asms(struct device *dev, struct cpm_dev_data dev_data[]) {
	struct cpm_notifier_block * nb = 0;
	struct cpm_asm_map * asm = 0;
	int ret = 0;
	u8 idx = 0;

	// Look for the device being already registerd
	nb = find_notifier_block(dev);
	if ( !nb ) { // if dev not registerd
		// add device notifier block
		nb = kzalloc(sizeof(struct cpm_notifier_block), GFP_KERNEL);
		if (!nb) {
			dprintk("out-of-mem on cpm_notifier_block allocation\n");
			ret = -ENOMEM;
			goto nomem_out;
		}

		// init data
		nb->dev = dev;
		LIST_HEAD_INIT(nb->asm_list);

		// add into notifier chain
		list_add(nb, notifier_list);

		dprintk("new device [%s] registerd\n", dev->name);

	}
	
	for (idx=0; idx<dev_data->count; idx++) {
	
		// verify that the the ASM exist
		if ( unlikely(dev_data[idx].id > platform_data.count) ) {
			dprintk("Warning: invalid asm request [%s:%d(%s)]\n",
			dev->name, asm->asm.id, asm->asm.name);
			ret = -EINVAL;
			goto asm_invalid_out;
		}

		// checking if the ASM has already been registerd
		asm = find_asm_map (nb, dev_data[idx].id);
		if (unlikely(asm)) {
			dprintk("Warning: duplicated asm mapping [%s:%d(%s)]\n",
			dev->name, asm->asm.id, asm->asm.name);
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
			strncpy(&(asm->asm.name), dev_data[idx].name, CPM_ASM_NAME_LEN);
		} else {
			strncpy(&(asm->asm.name), "Undef", CPM_ASM_NAME_LEN);
		}

		asm->asm.id = dev_data[idx].id;
		asm->asm.type = dev_data[idx].type;
		asm->seq = 0;
		// add into asm list for this device
		list_add(asm, nb->asm_list);

		dprintk("new asm %s association [%s:%d(%s)]\n",
			( asm->asm.id == CPM_INTEREST_AFFECT ) ?
				"AFFECT" : "INFLUENCE",
			dev->name, asm->asm.id, asm->asm.name);
	}

	return 0;

asm_duplicate_out:
asm_invalid_out:
nomem_out:
	return ret;

}
EXPORT_SYMBOL(cpm_subscribe_asms);

int cpm_unsubscribe_asms(struct device *dev, cpm_size asm_id) {
	struct cpm_notifier_block * nb = 0;
	struct cpm_asm_map * asm = 0;
	int ret = 0;

	// Look for the device
	nb = find_notifier_block(dev);
	if ( !nb ) { // if dev not registerd
		dprintk("Warning: unmapping asm for unregistered device [%s:%d(%s)]\n",
			dev->name, asm->asm.id, asm->asm.name);
		ret = -EINVAL;
		goto asm_nonexisting_out;
	}

	// Update device notifier block
	asm = find_asm_map(nb, id);
	if (unlikely(!asm)) {
		dprintk("Warning: unmapping unregisterd asm [%s:%d(%s)]\n",
			dev->name, asm->asm.id, asm->asm.name);
		ret = -EINVAL;
		goto asm_nonexisting_out;
	}

	// Removing ASM from device list
	list_del(asm);

	// Release memory
	kfree(asm);

	dprintk("removed asm %s association [%s:%d(%s)]\n",
			( asm->asm.id == CPM_INTEREST_AFFECT ) ?
				"AFFECT" : "INFLUENCE",
			dev->name, asm->asm.id, asm->asm.name);

asm_nonexisting_out:
	return ret;

}
EXPORT_SYMBOL(cpm_unsubscribe_asms);

int cpm_release_asms(struct device *dev) {

}
EXPORT_SYMBOL(cpm_release_asms);
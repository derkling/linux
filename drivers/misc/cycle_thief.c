/* #define DEBUG 1*/

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq_work.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/delay.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Steal cycles on cpu to emulate a lower OPP");

#define _DEFINE_SYSREG_READ_FUNC(_name, _reg_name)             \
static inline unsigned long read_ ## _name(void)               \
{                                                              \
       unsigned long v;                                        \
       __asm__ volatile ("mrs %0, " #_reg_name : "=r" (v));    \
       return v;                                               \
}

/* Define read function for system register */
#define DEFINE_SYSREG_READ_FUNC(_name)                         \
       _DEFINE_SYSREG_READ_FUNC(_name, _name)

#define _DEFINE_SYSREG_WRITE_FUNC(_name, _reg_name)            \
static inline void write_ ## _name(unsigned long v)	       \
{                                                              \
       __asm__ volatile ("msr " #_reg_name ", %0" :: "r" (v)); \
}

/* Define write function for system register */
#define DEFINE_SYSREG_WRITE_FUNC(_name)                         \
       _DEFINE_SYSREG_WRITE_FUNC(_name, _name)


#define DEFINE_SYSREG_ACCESS_FUNCS(_name) \
	DEFINE_SYSREG_READ_FUNC(_name)    \
	DEFINE_SYSREG_WRITE_FUNC(_name)

DEFINE_SYSREG_ACCESS_FUNCS(pmccntr_el0);
DEFINE_SYSREG_ACCESS_FUNCS(pmcntenset_el0);
DEFINE_SYSREG_ACCESS_FUNCS(pmovsclr_el0);
DEFINE_SYSREG_ACCESS_FUNCS(pmcr_el0);
DEFINE_SYSREG_ACCESS_FUNCS(pmintenset_el1);
DEFINE_SYSREG_ACCESS_FUNCS(pmevtyper0_el0);
DEFINE_SYSREG_ACCESS_FUNCS(pmevcntr0_el0);

#define CYCLES_EV	0x11
#define BUF_SIZE	30
#define NUM_REAL_OPPS	5

/*
 * TODO: we could generate more frequencies if we want better granularity but we
 * won't do this for the moment; When we do, this will come from DT together
 * with the table for periods.
 */
#define NUM_SIM_OPPS	5

#define NUM_CT_CLUSTERS	2

#define CT_CLUSTER_0_ID	0

/* Cluster 1 would be MEDIUM */
#define CT_CLUSTER_1_ID	1

/* Cluster 2 would be BIG */
#define CT_CLUSTER_2_ID	2


/* Only the big cluster will be split */
static unsigned long default_cpu_mask = 0xF0;
#define NUM_CPUS	8

static u64 default_period = ULONG_MAX-1;

static int CPU_IRQ_MAP[NUM_CPUS] = {7, 8, 9, 10, 11, 12, 13, 14};

static char names[BUF_SIZE * NUM_CPUS];

spinlock_t cycle_thief_lock;

typedef struct {
	u64 period;
	u64 overflow_count;
	u64 saved_cycles;
} cycle_thief_data_t;

static cycle_thief_data_t ct_data[NUM_CPUS];

typedef struct {
	int current_idx;
	int num_cpus;
	int *cpus;
} cycle_thief_cluster_t;

static cycle_thief_cluster_t ct_clusters[NUM_CT_CLUSTERS];

bool enabled = false;

/* TODO: This should be read from DT at some point */
static u64 period_table[NUM_REAL_OPPS][NUM_SIM_OPPS] = {
	/* From 903000 kHz */
	{
	 0,
	 0,
	 0,
	 0,
	 0
	},
	{
	 /* From 1421000 kHz */
	 4106, // to 903 KHz
	 0,
	 0,
	 0,
	 0
	},
	/* From 1805000 kHz */
	{
	  3012, // to 903000 kHz
	  8800, // to 1421000 kHz
	  0,
	  0,
	  0
	},
	/* From 2112000 kHz */
	{
	  2657, // to 903000 kHz
	  5633, // to 1421000 kHz
	  14600, // to 1805000 kHz
	  0,
	  0
	},
	/* From 2362000 kHz */
	{
	  2385,  // to 903000 kHz
	  4450,  // to 1421000 kHz
	  8454,  // to 1805000 kHz
	  20015, // to 2112000 kHz
	  0
	},
};

static struct dentry *root_debugfs_dir;

inline cycle_thief_data_t *get_cycle_thief_data(int cpu) {
	if (cpu > NUM_CPUS || cpu < 0) {
		return NULL;
	}
	return &ct_data[cpu];
}

static inline int get_irq_for_cpu(int cpu) {
	if (cpu > NUM_CPUS) {
		return -EINVAL;
	}
	return CPU_IRQ_MAP[cpu];
}

static void read_cycle_counter(void *info) {
	u64 val;
	val = read_pmccntr_el0();
	*(u64 *)info = val;
}

static irqreturn_t handle_overflow_irq(int irq,  void *dev) {
	unsigned int pmovsclr;
	cycle_thief_data_t *data;
	int cpu;

	cpu = smp_processor_id();

	data = get_cycle_thief_data(cpu);


	data->overflow_count++;

	// Clear the cycle counter overflow bit
	pmovsclr = (unsigned int)read_pmovsclr_el0();
	pmovsclr |= 1 << 31;
	write_pmovsclr_el0(pmovsclr);

	write_pmccntr_el0(ULONG_MAX - data->period);

	return IRQ_HANDLED;
}

static void register_cycle_thief_interrupt(int cpu) {
	char name[BUF_SIZE];
	int res, irq;
	cpumask_t irq_mask;

	irq = get_irq_for_cpu(cpu);
	snprintf(name, BUF_SIZE, "cycle-thief-%d", cpu);

#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
	if ((res = request_irq(irq, handle_overflow_irq,
				IRQF_NOBALANCING | IRQF_NO_THREAD,
				name, (void *)cpu))) {
		pr_debug("ERROR: could not get PMU IRQ %d on cpu %d; got %d\n",
				irq, cpu, res);
	}
#pragma GCC diagnostic pop

	cpumask_clear(&irq_mask);
	cpumask_set_cpu(cpu, &irq_mask);
	irq_set_affinity(irq, &irq_mask);

	pr_debug("Using IRQ %d for PMU on cpu %d\n", irq, cpu);

}

static void setup_cycle_thief(void *info) {
	unsigned int pmcntenset, pmcr, pmintenset;
	int cpu;

	cpu = smp_processor_id();

	// Enable PMUs by setting PMCR_EL0.E
	pmcr = (unsigned int)read_pmcr_el0();
	pmcr |= 1;
	write_pmcr_el0(pmcr);

	// Enable cycle counter by setting PMCNTENSET.C
	// Count enable set register
	pmcntenset = (unsigned int)read_pmcntenset_el0();
	pmcntenset |= 1 << 31;
	write_pmcntenset_el0(pmcntenset);

	// Enable cycle counter overflow interrupt
	// Interrupt enable set register
	pmintenset = (unsigned int)read_pmintenset_el1();
	pmintenset |= 1 << 31;
	write_pmintenset_el1(pmintenset);

	write_pmccntr_el0(0);

	__asm__("dsb ish");
}

static void inline setup_cycle_thief_on_cpu(int cpu) {
	smp_call_function_single(cpu, setup_cycle_thief,
			NULL, 1);
}

static void start_cycle_thief(void *info) {
	int cpu;
	cycle_thief_data_t *data;

	cpu = smp_processor_id();

	data = get_cycle_thief_data(cpu);
	write_pmccntr_el0(ULONG_MAX - data->period);

}

static void inline start_cycle_thief_on_cpu(int cpu) {
	smp_call_function_single(cpu, start_cycle_thief,
			NULL, 1);
}

static void stop_cycle_thief(void *info) {
	write_pmccntr_el0(0);
}

static void stop_cycle_thief_on_cpu(int cpu) {
	smp_call_function_single(cpu, stop_cycle_thief,
			NULL, 1);
}

inline void cycle_thief_save(int index) {
	int cpu;
	cycle_thief_data_t *data;

	if (!index) {
		// do nothing on WFI
		return;
	}

	cpu = smp_processor_id();
	data = get_cycle_thief_data(cpu);

	spin_lock(&cycle_thief_lock);
	data->saved_cycles = read_pmccntr_el0();
	spin_unlock(&cycle_thief_lock);
}

inline void cycle_thief_restore(int index) {
	int cpu;
	cycle_thief_data_t *data;

	if (!index || !enabled) {
		// do nothing on WFI or if disabled
		return;
	}

	cpu = smp_processor_id();
	data = get_cycle_thief_data(cpu);

	setup_cycle_thief(NULL);
	start_cycle_thief(NULL);
}

ssize_t cycles_read(struct file *filp, char *buff,
		size_t count, loff_t *offp) {
	char kbuf[BUF_SIZE];
	int cpu;
	u64 cycles;
	size_t len, ret;
	loff_t zero = 0;

#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
	cpu = (int)filp->f_inode->i_private;
#pragma GCC diagnostic pop
	smp_call_function_single(cpu, read_cycle_counter,
			(void *)&cycles, 1);

	pr_debug("cycle thief cpu %u cycles: %llu\n", cpu, cycles);

	len = snprintf(kbuf, BUF_SIZE-1, "%llu", cycles);
	kbuf[len++] = '\n';
	kbuf[len] = '\0';

	memset(buff, 0, count);
	ret = simple_read_from_buffer(buff, count, &zero, kbuf, len);

	return ret;
}

struct file_operations cycles_fops = {
	.read = cycles_read
};

ssize_t period_read(struct file *filp, char *buff,
		size_t count, loff_t *offp) {
	char kbuf[BUF_SIZE];
	int cpu;
	size_t len, ret;
	u64 period;
	loff_t zero = 0;

#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
	cpu = (int)filp->f_inode->i_private;
#pragma GCC diagnostic pop

	spin_lock(&cycle_thief_lock);
	period = ct_data[cpu].period;
	spin_unlock(&cycle_thief_lock);

	pr_debug("cycle thief cpu %u period: %llu\n", cpu, period);

	len = snprintf(kbuf, BUF_SIZE-1, "%llu", period);
	kbuf[len++] = '\n';
	kbuf[len] = '\0';

	memset(buff, 0, count);
	ret = simple_read_from_buffer(buff, count, &zero, kbuf, len);
	return ret;
}

struct file_operations period_fops = {
	.read = period_read
};

/*
 * Must be called with cycle_thief_lock held.
 */
int adjust_rate(unsigned int sim_index, unsigned int real_index,
		unsigned int cluster_id) {
	int cpu, i;
	u64 new_period;
	cycle_thief_data_t *data;

	pr_debug("cycle thief: sim index %u real index %u\n", sim_index,
							    real_index);
	new_period = period_table[real_index][sim_index];

	if (!new_period) {
		if (sim_index > real_index) {
			/*
			 * You can't obtain higher simulated frequency from
			 * lower real frequency
			 */
			pr_debug("cycle thief: attempt to set invalid frequency\n");
			return -1;
		}
		pr_debug("cycle thief: real index equals sim index\n");
		new_period = default_period;
	}

	if (real_index >= NUM_REAL_OPPS || sim_index >= NUM_SIM_OPPS) {
		pr_debug("cycle_thief: requested index out of range\n");
		dump_stack();
		return -1;
	}

	pr_debug("cycle thief: setting cluster %u period to %llu\n",
		 cluster_id, new_period);

	for (i = 0; i < ct_clusters[cluster_id].num_cpus; i++) {
		cpu = ct_clusters[cluster_id].cpus[i];
		data = get_cycle_thief_data(cpu);
		data->period = new_period;
	}

	return 0;
}

/*
 * returns: 0 if no actual frequency ajustment necessary
 *          1 if frequency adjustment is necessary
 * TODO: the adjustment in period is done before the actual frequency changes
 * which is a problem as you might be throttled or receive higher performance
 * for a short while before the real frequency changes
 */
int cycle_thief_set_rate(u32 id, unsigned int index) {
	unsigned int new_idx_1, new_idx_2;
	int real_new_idx;

	if ((id == CT_CLUSTER_0_ID) || (id > CT_CLUSTER_2_ID))
		/* Change in OPP is permitted*/
		return -1;

	pr_debug("cycle thief set rate id %u, idx %u\n", id, index);

	spin_lock(&cycle_thief_lock);

	switch (id){
	case CT_CLUSTER_1_ID:
		new_idx_1 = index;
		new_idx_2 = (ct_clusters[CT_CLUSTER_2_ID].current_idx < 0)? 0 :
			    ct_clusters[CT_CLUSTER_2_ID].current_idx;
		break;
	case CT_CLUSTER_2_ID:
		new_idx_2 = index;
		new_idx_1 =  (ct_clusters[CT_CLUSTER_1_ID].current_idx < 0)? 0 :
			     ct_clusters[CT_CLUSTER_1_ID].current_idx;
		break;
	}

	real_new_idx = max(new_idx_1, new_idx_2);

	/*
	 * TODO: could be optimised a bit by only calling one adjust_rate if
	 * change is not needed - max has not changed but I'll leave it for
	 * later
	 */
	if (!adjust_rate(new_idx_1, real_new_idx, CT_CLUSTER_1_ID))
		ct_clusters[CT_CLUSTER_1_ID].current_idx = new_idx_1;
	if (!adjust_rate(new_idx_2, real_new_idx, CT_CLUSTER_2_ID))
		ct_clusters[CT_CLUSTER_2_ID].current_idx = new_idx_2;

	spin_unlock(&cycle_thief_lock);

	return real_new_idx;
}

static inline void print_regs(void) {
	unsigned int pmcntenset, pmcr, pmintenset;
	u64 pmccntr, pmevcntr0;
	int cpu;

	preempt_disable();

	cpu = smp_processor_id();

	pmcr = (unsigned int)read_pmcr_el0();
	pmcntenset = (unsigned int)read_pmcntenset_el0();
	pmintenset = (unsigned int)read_pmintenset_el1();
	pmccntr = read_pmccntr_el0();
	pmevcntr0 = read_pmevcntr0_el0();

	trace_printk("regs on cpu %i:\n\tpmcr: 0x%x\n\tpmcntenset: 0x%x\n\t"
		     "pmintenset: 0x%x\n\tpmccntr: %llu\n\tpmevcntr0: %llu\n",
		     cpu, pmcr, pmcntenset, pmintenset, pmccntr, pmevcntr0);

	preempt_enable();
}

static void enable_cycle_thief(void) {
	int cpu;
	cpumask_t *cpu_mask;

	pr_err("Enabling cycle thief\n");

	cpu_mask = to_cpumask(&default_cpu_mask);

	for_each_cpu(cpu, cpu_mask) {
		setup_cycle_thief_on_cpu(cpu);
	}
	__asm__("isb");

	for_each_cpu(cpu, cpu_mask) {
		register_cycle_thief_interrupt(cpu);
	}

	for_each_cpu(cpu, cpu_mask) {
		start_cycle_thief_on_cpu(cpu);
	}
}

static void disable_cycle_thief(void) {
	int irq, cpu;
	cpumask_t *cpu_mask;

	cpu_mask = to_cpumask(&default_cpu_mask);

	for_each_cpu(cpu, cpu_mask) {
		irq = get_irq_for_cpu(cpu);
		free_irq(irq, NULL);
		stop_cycle_thief_on_cpu(cpu);
	}
}

ssize_t enabled_read(struct file *filp, char *buff,
		size_t count, loff_t *offp) {
	char kbuf[BUF_SIZE];
	size_t len, ret;
	loff_t zero = 0;

	len = snprintf(kbuf, BUF_SIZE-1, "%u", enabled);
	kbuf[len++] = '\n';
	kbuf[len] = '\0';

	memset(buff, 0, count);
	ret = simple_read_from_buffer(buff, count, &zero, kbuf, len);

	return ret;
}

static ssize_t enabled_write(struct file *file, const char __user *buf,
			      size_t len, loff_t *ppos) {
	int ret;
	bool new_enabled;
	char *kbuf = kmalloc(len + 1, GFP_KERNEL);

	if (!kbuf)
		return -ENOMEM;

	ret = simple_write_to_buffer(kbuf, len, ppos, buf, len);
	if (ret != len) {
		ret = ret >= 0 ? -EIO : ret;
		goto out;
	}
	kbuf[len] = '\0';

	if ((ret = strtobool(kbuf, &new_enabled))) {
		goto out;
	}

	if (new_enabled == enabled) {
		goto out;
	}

	if (new_enabled) {
		enable_cycle_thief();
	} else {
		disable_cycle_thief();
	}

	enabled = new_enabled;
	ret = len;
out:
	kfree(kbuf);
	return ret;
}

struct file_operations enabled_fops = {
	.read = enabled_read,
	.write = enabled_write
};

int cycle_thief_init(void) {
	int cpu;
	char *name;
	cycle_thief_data_t *data;
	struct dentry *cpu_dir;
	unsigned long data_size_m = 0;
	unsigned long data_size_b = 0;

	root_debugfs_dir = debugfs_create_dir("cycle_thief", NULL);
	debugfs_create_file("enabled", S_IRUGO, root_debugfs_dir,
			NULL, &enabled_fops);

	for (cpu = 0; cpu < NUM_CPUS; cpu++) {
		data = get_cycle_thief_data(cpu);
		data->period = default_period;
		data->overflow_count = 0;
		data->saved_cycles = 0;

		name = &names[BUF_SIZE*cpu];
		snprintf(name, BUF_SIZE, "cpu%i", cpu);
		cpu_dir = debugfs_create_dir(name, root_debugfs_dir);
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
		debugfs_create_file("cycles", S_IRUGO, cpu_dir,
				(void *)cpu, &cycles_fops);
		debugfs_create_file("period", S_IRUGO, cpu_dir,
				(void *)cpu, &period_fops);
#pragma GCC diagnostic pop
	}

	ct_clusters[CT_CLUSTER_1_ID].current_idx = -1;
	ct_clusters[CT_CLUSTER_1_ID].num_cpus = 2;
        data_size_m = sizeof(int) * ct_clusters[CT_CLUSTER_1_ID].num_cpus;
	ct_clusters[CT_CLUSTER_1_ID].cpus = kzalloc(data_size_m, GFP_KERNEL);
	ct_clusters[CT_CLUSTER_1_ID].cpus[0] = 6;
	ct_clusters[CT_CLUSTER_1_ID].cpus[1] = 7;

	ct_clusters[CT_CLUSTER_2_ID].current_idx = -1;
	ct_clusters[CT_CLUSTER_2_ID].num_cpus = 2;
	data_size_b = sizeof(int) * ct_clusters[CT_CLUSTER_2_ID].num_cpus;
	ct_clusters[CT_CLUSTER_2_ID].cpus = kzalloc(data_size_b, GFP_KERNEL);
	ct_clusters[CT_CLUSTER_2_ID].cpus[0] = 4;
	ct_clusters[CT_CLUSTER_2_ID].cpus[1] = 5;

	trace_printk("Cycle thief initialized\n");

	return 0;
}

void cycle_thief_exit(void) {
	disable_cycle_thief();
	debugfs_remove_recursive(root_debugfs_dir);
	trace_printk("Cycle thief exited\n");
}

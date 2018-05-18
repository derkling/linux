/*
 * arch/arm64/kernel/actmon.c
 *
 * Copyright (C) 2018 Arm Limited.
 *
 */

#include <asm/actmon.h>

// This provides us with the interface to the activity monitor
// counters. Not clear if this is where things should be in the long
// run, but this will do for the prototyping we are doing here.
#include <linux/psci.h>

// We use the interfaces defined herein to read the value of the
// generic counter.
#include <asm/sysreg.h>

// Used to schedule the counter reading on speciifc CPUs based on the
// cpus in policy-cpus.
#include <linux/smp.h>
#include <linux/irqflags.h>

#include <trace/events/power.h>

#include <linux/debugfs.h>

/*
 * Switches to enable or disable functionality within debugfs.
 */

static u32 use_actmon = 1;
static u32 get_orig_scale = 0;
static u32 always_use_cached_capacity = 0;

/*
 * Some debugfs specific things. Specifically, this will be used to
 * reference our directory in debugfs.
 */
static struct dentry *dir = 0;

#define BUF_SIZE 30

static u32 debugfs_initialised = 0;
static int init_debugfs(void);


/*
 * We need to store the ratio between the constant frequency counter
 * frequency and the maximum frequench of each CPU. This is used as
 * part of the frequency calculation. In theory, we only need to
 * calculate this once, so it should be relatively low overhead.
 *
 * We need to make sure that this is scaled correctly - we store it as
 * an integer, and therefore we need to make sure that we have enough
 * precision to use this properly. Therefore, we scale this by TODO,
 * and undo that scaling later on in the calculation.
 */
static DEFINE_PER_CPU(u64, const_freq_ratio) = 0;
static DEFINE_PER_CPU(u32, initialised) = 0;

/*
 * Update the const_freq_ratio for a particular CPU.
 */
void
actmon_update_cpu_freq_max(int cpu, u32 max_freq)
{
	u64 const_freq_cntr_rate;

        /*
	 * We need to get the frequency of the generic timer. This
	 * will tell us how to scale the const freq counter. The unit
	 * is Hz.
	 */
	const_freq_cntr_rate = read_cpuid(CNTFRQ_EL0);

	/*
	 * For cpufreq, the frequency unit is kHz, whilst it is Hz for
	 * the generic timer. Therefore, we need to scale this by 1000
	 * (we could use 1024, or 2**10), but as we update this
	 * rarely, we should be able to go for the accurate
	 * calculation. As we want to perform only integer division,
	 * we then further scale it by another 1024.
	 */
	per_cpu(const_freq_ratio, cpu) = (((uint64_t)max_freq * 1000) << 10) / const_freq_cntr_rate;
	per_cpu(initialised, cpu) = 1;

	// Let's now make sure that the debugfs interface has been initialised!
	if (!debugfs_initialised) {
		if (!init_debugfs()) {
			printk("Failed to initialise actmon debugfs!\n");
		} else {
			debugfs_initialised = 1;
		}
	}

	return;
}

/*
 * Small wrapper functions to get the counters and update the
 * references.
 */
inline volatile uint64_t
__get_constant_frequency_counter(uint64_t *ref)
{
	volatile uint64_t const_freq_counter = 0, const_freq_counter_delta = 0;
	int this_cpu = smp_processor_id();

	const_freq_counter = get_constant_frequency_counter();

	/*
	 * get the difference to the reference and update the reference
	 */
	const_freq_counter_delta = const_freq_counter - *ref;
	*ref = const_freq_counter;

	return const_freq_counter_delta;
}

inline volatile uint64_t
__get_core_cycle_counter(uint64_t *ref)
{
	volatile uint64_t core_cycle_counter = 0, core_cycle_counter_delta = 0;
	int this_cpu = smp_processor_id();

	core_cycle_counter = get_core_cycle_counter();

	/*
	 * get the difference to the reference and update the reference
	 */
	core_cycle_counter_delta = core_cycle_counter - *ref;
	*ref = core_cycle_counter;

	return core_cycle_counter_delta;
}

/*
 * Do the actual scale factor calculation.
 */
inline unsigned long
__calc_actmon_scale(int cpu, uint64_t const_freq_counter_delta,
		    uint64_t core_cycle_counter_delta,
		    int on_tick)
{
	unsigned long scale;

	if (const_freq_counter_delta > 0xFFFFFFFF0FFFFFFF ||
	    core_cycle_counter_delta > 0xFFFFFFFF0FFFFFFF) {
		printk("**********************************************************************\n");
		printk("CPU%d: const_freq_counter_delta: %llu, core_cycle_counter_delta: %llu\n",
		       cpu, const_freq_counter_delta, core_cycle_counter_delta);
		printk("**********************************************************************\n");
	}

	scale = core_cycle_counter_delta << SCHED_CAPACITY_SHIFT;
	scale <<= SCHED_CAPACITY_SHIFT;
	scale /= const_freq_counter_delta;
	scale /= per_cpu(const_freq_ratio, cpu);

	trace_cpu_capacity_actmon(scale, cpu, const_freq_counter_delta,
				  core_cycle_counter_delta,
				  per_cpu(const_freq_ratio, cpu), on_tick);

	if (get_orig_scale) {
		cpufreq_scale_freq_capacity(NULL, cpu);
	}

	return scale;
}


/*
 * There are times when we will be required to get the scale factor
 * for a CPU that we are not running on. In those cases, we return a
 * cached version, which is update on each scheduling tick. Therefore,
 * we avoid having to do a cross-call to another CPU to get the values
 * of the monitors.
 */
static DEFINE_PER_CPU(u64, cached_scale) = {1024};
static DEFINE_PER_CPU(u64, local_const_freq_ref) = {0};
static DEFINE_PER_CPU(u64, local_core_cycle_ref) = {0};

/*
 * The periodic update, called on a scheduler tick. We keep local
 * references (for now) as we do not have access to any common data
 * structure.
 */
void
actmon_scale_freq_capacity_tick(int cpu)
{
	uint64_t const_freq_counter_delta = 0, core_cycle_counter_delta = 0;
	unsigned long scale;
	uint64_t const_freq_ref, core_cycle_ref;
	int this_cpu = smp_processor_id();

	if (!use_actmon) {
		scale = cpufreq_scale_freq_capacity(NULL, cpu);
		per_cpu(cached_scale, cpu) = scale;
		return;
	}

	/*
	 * If we are running on the wrong CPU here, something has gone
	 * really wrong.
	 */
	BUG_ON(this_cpu != cpu);

	const_freq_counter_delta = __get_constant_frequency_counter(per_cpu_ptr(&local_const_freq_ref, cpu));
	core_cycle_counter_delta = __get_core_cycle_counter(per_cpu_ptr(&local_core_cycle_ref, cpu));
	scale = __calc_actmon_scale(cpu, const_freq_counter_delta,
				    core_cycle_counter_delta, 1);

	per_cpu(cached_scale, cpu) = scale;

	return;
}


/*
 * Returns the cached capacity of cpu.
 */
unsigned long
actmon_get_cached_capacity(int cpu)
{
	if (!use_actmon) {
		return cpufreq_scale_freq_capacity(NULL, cpu);
	}
	return per_cpu(cached_scale, cpu);
}


/*
 * Returns the current capacity of cpu which has been calculated using
 * the activity monitors. This ensures that we are able to get the
 * average capacity of a CPU, rather than the one that we think we
 * have set. Therefore, this scaling will respond to the frequency
 * changing underneath the kernel.
 */
unsigned long
actmon_scale_freq_capacity(struct sched_avg *sa, int cpu)
{
	uint64_t const_freq_counter_delta = 0, core_cycle_counter_delta = 0;
	unsigned long scale;
	int this_cpu = smp_processor_id();

	if (!use_actmon) {
		return cpufreq_scale_freq_capacity(NULL, cpu);
	}

	/*
	 * It is possible that we will be called without being passed
	 * sched_avg. We need to decide what to do here. For now,
	 * assume we are running at the highest frequency.
	 *
	 * An alternative would be to return the most recently
	 * calculated freq-scale for the CPU.
	 */
	if (!sa) {
		return 1024;
	}

	if (!per_cpu(initialised, cpu)) {
		return 1024;
	}

	if (always_use_cached_capacity) {
		return per_cpu(cached_scale, cpu);
	}

	/*
	 * Get the perf counter values for the CPU in question
	 */
	if (cpu != this_cpu) {
		trace_cpu_capacity_actmon(per_cpu(cached_scale, cpu), cpu, 0, 0, 0, 0);
		return per_cpu(cached_scale, cpu);
	}

	const_freq_counter_delta = __get_constant_frequency_counter(&sa->const_freq_ref);
	core_cycle_counter_delta = __get_core_cycle_counter(&sa->core_cycle_ref);

	/*
	 * TODO: Do need to make sure that we deal with wrap-around of
	 * the counters here too? They are 64-bit, so it should be
	 * really rare!
	 */

	/*
	 * Finally, let's calculate the actual scale factor based on
	 * the average frequency.
	 */
	scale = __calc_actmon_scale(cpu, const_freq_counter_delta,
				    core_cycle_counter_delta, 0);

	return scale;
}


/*
 * DebugFS entries!
 */

/*
 * Function to enable or disable activity monitors based on a debugfs
 * write. Please note that anything non-zero is treated as enabling
 * the activity monitor based scaling.
 */
static ssize_t debugfs_write_actmon_enable(struct file *file,
					  const char __user *buf,
					  size_t size,
					  loff_t *ppos)
{
	int ret;
	u32 temp;
	ret = kstrtou32_from_user(buf, size, 0, &temp);
	if (ret) {
		printk("OH BUGGER!\n");
		return size;
	}

	if (temp == 0) {
		use_actmon = 0;
	} else {
		use_actmon = 1;
	}

	return size;
}

/*
 * Function to check if the activity monitor based scaling is enabled
 * or not. Returns 0 if it is disabled, 1 otherwise.
 */
static ssize_t debugfs_read_actmon_enable(struct file *file,
					  char __user *buf,
					  size_t size,
					  loff_t *ppos)
{
	char kbuf[BUF_SIZE];
	size_t len, ret;

	len = snprintf(kbuf, BUF_SIZE, "%u\n", use_actmon);

	ret = simple_read_from_buffer(buf, size, ppos, kbuf, len);

	return ret;
}

struct file_operations actmon_en_fops = {
	.read = debugfs_read_actmon_enable,
	.write = debugfs_write_actmon_enable,
	.open = simple_open,
	.llseek = default_llseek,
};

/*
 * Setting this to true forces EAS to always use the scaling factors
 * calcualated on a tick.
 */
static ssize_t debugfs_write_actmon_force_cached(struct file *file,
						 const char __user *buf,
						 size_t size,
						 loff_t *ppos)
{
	int ret;
	u32 temp;
	ret = kstrtou32_from_user(buf, size, 0, &temp);
	if (ret) {
		printk("OH BUGGER!\n");
		return size;
	}

	if (temp == 0) {
		always_use_cached_capacity = 0;
	} else {
		always_use_cached_capacity = 1;
	}

	return size;
}

/*
 * Return if we are using the cached values or not. True if we are.
 */
static ssize_t debugfs_read_actmon_force_cached(struct file *file,
						char __user *buf,
						size_t size,
						loff_t *ppos)
{
	char kbuf[BUF_SIZE];
	size_t len, ret;

	len = snprintf(kbuf, BUF_SIZE, "%u\n", always_use_cached_capacity);

	ret = simple_read_from_buffer(buf, size, ppos, kbuf, len);

	return ret;
}

struct file_operations actmon_force_cached_fops = {
	.read = debugfs_read_actmon_force_cached,
	.write = debugfs_write_actmon_force_cached,
	.open = simple_open,
	.llseek = default_llseek,
};


/*
 * This is for debugging purposes. This also gets the old CPUFreq
 * scale values, and reports them using a tracepoint.
 */
static ssize_t debugfs_write_actmon_get_orig_cap(struct file *file,
						 const char __user *buf,
						 size_t size,
						 loff_t *ppos)
{
	int ret;
	u32 temp;
	ret = kstrtou32_from_user(buf, size, 0, &temp);
	if (ret) {
		printk("OH BUGGER!\n");
		return size;
	}

	if (temp == 0) {
		get_orig_scale = 0;
	} else {
		get_orig_scale = 1;
	}

	return size;
}

/*
 * Check if the reading of CPUFreq scale values is enabled or not.
 */
static ssize_t debugfs_read_actmon_get_orig_cap(struct file *file,
						char __user *buf,
						size_t size,
						loff_t *ppos)
{
	char kbuf[BUF_SIZE];
	size_t len, ret;

	len = snprintf(kbuf, BUF_SIZE, "%u\n", get_orig_scale);

	ret = simple_read_from_buffer(buf, size, ppos, kbuf, len);

	return ret;
}

struct file_operations actmon_get_orig_cap_fops = {
	.read = debugfs_read_actmon_get_orig_cap,
	.write = debugfs_write_actmon_get_orig_cap,
	.open = simple_open,
	.llseek = default_llseek,
};


static int init_debugfs(void)
{
    struct dentry *temp;

    dir = debugfs_create_dir("actmon", 0);
    if (!dir) {
        return -1;
    }

    debugfs_create_file("actmon_en", S_IRUGO | S_IWUGO,
			dir, (void *)NULL, &actmon_en_fops);
    debugfs_create_file("actmon_force_cached", S_IRUGO | S_IWUGO,
			dir, (void *)NULL, &actmon_force_cached_fops);
    debugfs_create_file("actmon_get_orig_scale", S_IRUGO | S_IWUGO,
			dir, (void *)NULL, &actmon_get_orig_cap_fops);

    return 0;
}

// This is called when the module is removed.
void cleanup_module(void)
{
    debugfs_remove_recursive(dir);
}

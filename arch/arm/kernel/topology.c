/*
 * arch/arm/kernel/topology.c
 *
 * Copyright (C) 2011 Linaro Limited.
 * Written by: Vincent Guittot
 *
 * based on arch/sh/kernel/topology.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/percpu.h>
#include <linux/node.h>
#include <linux/nodemask.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <asm/cputype.h>
#include <asm/topology.h>

/*
 * cpu capacity scale management
 */

/*
 * cpu capacity table
 * This per cpu data structure describes the relative capacity of each core.
 * On a heteregenous system, cores don't have the same computation capacity
 * and we reflect that difference in the cpu_capacity field so the scheduler
 * can take this difference into account during load balance. A per cpu
 * structure is preferred because each CPU updates its own cpu_capacity field
 * during the load balance except for idle cores. One idle core is selected
 * to run the rebalance_domains for all idle cores and the cpu_capacity can be
 * updated during this sequence.
 */
static DEFINE_PER_CPU(unsigned long, cpu_scale);

unsigned long arch_scale_freq_capacity(struct sched_domain *sd, int cpu)
{
	return per_cpu(cpu_scale, cpu);
}

static void set_capacity_scale(unsigned int cpu, unsigned long capacity)
{
	per_cpu(cpu_scale, cpu) = capacity;
}

#ifdef CONFIG_OF
struct cpu_efficiency {
	const char *compatible;
	unsigned long efficiency;
};

/*
 * Table of relative efficiency of each processors
 * The efficiency value must fit in 20bit and the final
 * cpu_scale value must be in the range
 *   0 < cpu_scale < 3*SCHED_CAPACITY_SCALE/2
 * in order to return at most 1 when DIV_ROUND_CLOSEST
 * is used to compute the capacity of a CPU.
 * Processors that are not defined in the table,
 * use the default SCHED_CAPACITY_SCALE value for cpu_scale.
 */
static const struct cpu_efficiency table_efficiency[] = {
	{"arm,cortex-a15", 3891},
	{"arm,cortex-a7",  2048},
	{NULL, },
};

static unsigned long *__cpu_capacity;
#define cpu_capacity(cpu)	__cpu_capacity[cpu]

static unsigned long middle_capacity = 1;

/*
 * Iterate all CPUs' descriptor in DT and compute the efficiency
 * (as per table_efficiency). Also calculate a middle efficiency
 * as close as possible to  (max{eff_i} - min{eff_i}) / 2
 * This is later used to scale the cpu_capacity field such that an
 * 'average' CPU is of middle capacity. Also see the comments near
 * table_efficiency[] and update_cpu_capacity().
 */
static void __init parse_dt_topology(void)
{
	const struct cpu_efficiency *cpu_eff;
	struct device_node *cn = NULL;
	unsigned long min_capacity = (unsigned long)(-1);
	unsigned long max_capacity = 0;
	unsigned long capacity = 0;
	int alloc_size, cpu = 0;

	alloc_size = nr_cpu_ids * sizeof(*__cpu_capacity);
	__cpu_capacity = kzalloc(alloc_size, GFP_NOWAIT);

	for_each_possible_cpu(cpu) {
		const u32 *rate;
		int len;

		/* too early to use cpu->of_node */
		cn = of_get_cpu_node(cpu, NULL);
		if (!cn) {
			pr_err("missing device node for CPU %d\n", cpu);
			continue;
		}

		for (cpu_eff = table_efficiency; cpu_eff->compatible; cpu_eff++)
			if (of_device_is_compatible(cn, cpu_eff->compatible))
				break;

		if (cpu_eff->compatible == NULL)
			continue;

		rate = of_get_property(cn, "clock-frequency", &len);
		if (!rate || len != 4) {
			pr_err("%s missing clock-frequency property\n",
				cn->full_name);
			continue;
		}

		capacity = ((be32_to_cpup(rate)) >> 20) * cpu_eff->efficiency;

		/* Save min capacity of the system */
		if (capacity < min_capacity)
			min_capacity = capacity;

		/* Save max capacity of the system */
		if (capacity > max_capacity)
			max_capacity = capacity;

		cpu_capacity(cpu) = capacity;
	}

	/* If min and max capacities are equals, we bypass the update of the
	 * cpu_scale because all CPUs have the same capacity. Otherwise, we
	 * compute a middle_capacity factor that will ensure that the capacity
	 * of an 'average' CPU of the system will be as close as possible to
	 * SCHED_CAPACITY_SCALE, which is the default value, but with the
	 * constraint explained near table_efficiency[].
	 */
	if (4*max_capacity < (3*(max_capacity + min_capacity)))
		middle_capacity = (min_capacity + max_capacity)
				>> (SCHED_CAPACITY_SHIFT+1);
	else
		middle_capacity = ((max_capacity / 3)
				>> (SCHED_CAPACITY_SHIFT-1)) + 1;

}

/*
 * Look for a customed capacity of a CPU in the cpu_capacity table during the
 * boot. The update of all CPUs is in O(n^2) for heteregeneous system but the
 * function returns directly for SMP system.
 */
static void update_cpu_capacity(unsigned int cpu)
{
	if (!cpu_capacity(cpu))
		return;

	set_capacity_scale(cpu, cpu_capacity(cpu) / middle_capacity);

	printk(KERN_INFO "CPU%u: update cpu_capacity %lu\n",
		cpu, arch_scale_freq_capacity(NULL, cpu));
}

#else
static inline void parse_dt_topology(void) {}
static inline void update_cpu_capacity(unsigned int cpuid) {}
#endif

 /*
 * cpu topology table
 */
struct cputopo_arm cpu_topology[NR_CPUS];
EXPORT_SYMBOL_GPL(cpu_topology);

const struct cpumask *cpu_coregroup_mask(int cpu)
{
	return &cpu_topology[cpu].core_sibling;
}

/*
 * The current assumption is that we can power gate each core independently.
 * This will be superseded by DT binding once available.
 */
const struct cpumask *cpu_corepower_mask(int cpu)
{
	return &cpu_topology[cpu].thread_sibling;
}

static void update_siblings_masks(unsigned int cpuid)
{
	struct cputopo_arm *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];
	int cpu;

	/* update core and thread sibling masks */
	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		if (cpuid_topo->socket_id != cpu_topo->socket_id)
			continue;

		cpumask_set_cpu(cpuid, &cpu_topo->core_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->core_sibling);

		if (cpuid_topo->core_id != cpu_topo->core_id)
			continue;

		cpumask_set_cpu(cpuid, &cpu_topo->thread_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->thread_sibling);
	}
	smp_wmb();
}

/*
 * store_cpu_topology is called at boot when only one cpu is running
 * and with the mutex cpu_hotplug.lock locked, when several cpus have booted,
 * which prevents simultaneous write access to cpu_topology array
 */
void store_cpu_topology(unsigned int cpuid)
{
	struct cputopo_arm *cpuid_topo = &cpu_topology[cpuid];
	unsigned int mpidr;

	/* If the cpu topology has been already set, just return */
	if (cpuid_topo->core_id != -1)
		return;

	mpidr = read_cpuid_mpidr();

	/* create cpu topology mapping */
	if ((mpidr & MPIDR_SMP_BITMASK) == MPIDR_SMP_VALUE) {
		/*
		 * This is a multiprocessor system
		 * multiprocessor format & multiprocessor mode field are set
		 */

		if (mpidr & MPIDR_MT_BITMASK) {
			/* core performance interdependency */
			cpuid_topo->thread_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);
			cpuid_topo->core_id = MPIDR_AFFINITY_LEVEL(mpidr, 1);
			cpuid_topo->socket_id = MPIDR_AFFINITY_LEVEL(mpidr, 2);
		} else {
			/* largely independent cores */
			cpuid_topo->thread_id = -1;
			cpuid_topo->core_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);
			cpuid_topo->socket_id = MPIDR_AFFINITY_LEVEL(mpidr, 1);
		}
	} else {
		/*
		 * This is an uniprocessor system
		 * we are in multiprocessor format but uniprocessor system
		 * or in the old uniprocessor format
		 */
		cpuid_topo->thread_id = -1;
		cpuid_topo->core_id = 0;
		cpuid_topo->socket_id = -1;
	}

	update_siblings_masks(cpuid);

	update_cpu_capacity(cpuid);

	printk(KERN_INFO "CPU%u: thread %d, cpu %d, socket %d, mpidr %x\n",
		cpuid, cpu_topology[cpuid].thread_id,
		cpu_topology[cpuid].core_id,
		cpu_topology[cpuid].socket_id, mpidr);
}

/*
 * ARM TC2 specific energy cost model data. There are no unit requirements for
 * the data. Data can be normalized to any reference point, but the
 * normalization must be consistent. That is, one bogo-joule/watt must be the
 * same quantity for all data, but we don't care what it is.
 */
static struct idle_state idle_states_cluster_a7[] = {
	 { .power = 10, .wu_energy = 6 /* << 10 */, },
	};

static struct idle_state idle_states_cluster_a15[] = {
	 { .power = 16, .wu_energy = 31 /* << 10 */, },
	};

static struct capacity_state cap_states_cluster_a7[] = {
	/* Cluster only power */
	 { .cap =  182, .power =  434, }, /*  350 MHz */
	 { .cap =  208, .power =  409, }, /*  400 MHz */
	 { .cap =  260, .power =  411, }, /*  500 MHz */
	 { .cap =  312, .power =  412, }, /*  600 MHz */
	 { .cap =  364, .power =  427, }, /*  700 MHz */
	 { .cap =  416, .power =  417, }, /*  800 MHz */
	 { .cap =  468, .power =  573, }, /*  900 MHz */
	 { .cap =  520, .power =  718, }, /* 1000 MHz */
	};

static struct capacity_state cap_states_cluster_a15[] = {
	/* Cluster only power */
	 { .cap =  427, .power = 1159, }, /*  500 MHz */
	 { .cap =  512, .power = 1195, }, /*  600 MHz */
	 { .cap =  597, .power = 1196, }, /*  700 MHz */
	 { .cap =  683, .power = 1199, }, /*  800 MHz */
	 { .cap =  768, .power = 1210, }, /*  900 MHz */
	 { .cap =  853, .power = 1236, }, /* 1000 MHz */
	 { .cap =  939, .power = 1672, }, /* 1100 MHz */
	 { .cap = 1024, .power = 2225, }, /* 1200 MHz */
	};

static struct sched_group_energy energy_cluster_a7 = {
	  .nr_idle_states = ARRAY_SIZE(idle_states_cluster_a7),
	  .idle_states    = idle_states_cluster_a7,
	  .nr_cap_states  = ARRAY_SIZE(cap_states_cluster_a7),
	  .cap_states     = cap_states_cluster_a7,
};

static struct sched_group_energy energy_cluster_a15 = {
	  .nr_idle_states = ARRAY_SIZE(idle_states_cluster_a15),
	  .idle_states    = idle_states_cluster_a15,
	  .nr_cap_states  = ARRAY_SIZE(cap_states_cluster_a15),
	  .cap_states     = cap_states_cluster_a15,
};

static struct idle_state idle_states_core_a7[] = {
	 { .power = 0 /* No power gating */, .wu_energy = 0 /* << 10 */, },
	};

static struct idle_state idle_states_core_a15[] = {
	 { .power = 0 /* No power gating */, .wu_energy = 1 /* << 10 */, },
	};

static struct capacity_state cap_states_core_a7[] = {
	/* Power per cpu */
	 { .cap =  182, .power =   27, }, /*  350 MHz */
	 { .cap =  208, .power =   40, }, /*  400 MHz */
	 { .cap =  260, .power =   49, }, /*  500 MHz */
	 { .cap =  312, .power =   60, }, /*  600 MHz */
	 { .cap =  364, .power =   65, }, /*  700 MHz */
	 { .cap =  416, .power =   80, }, /*  800 MHz */
	 { .cap =  468, .power =  111, }, /*  900 MHz */
	 { .cap =  520, .power =  150, }, /* 1000 MHz */
	};

static struct capacity_state cap_states_core_a15[] = {
	/* Power per cpu */
	 { .cap =  427, .power =  296, }, /*  500 MHz */
	 { .cap =  512, .power =  338, }, /*  600 MHz */
	 { .cap =  597, .power =  403, }, /*  700 MHz */
	 { .cap =  683, .power =  457, }, /*  800 MHz */
	 { .cap =  768, .power =  516, }, /*  900 MHz */
	 { .cap =  853, .power =  563, }, /* 1000 MHz */
	 { .cap =  939, .power =  758, }, /* 1100 MHz */
	 { .cap = 1024, .power = 1024, }, /* 1200 MHz */
	};

static struct sched_group_energy energy_core_a7 = {
	  .nr_idle_states = ARRAY_SIZE(idle_states_core_a7),
	  .idle_states    = idle_states_core_a7,
	  .nr_cap_states  = ARRAY_SIZE(cap_states_core_a7),
	  .cap_states     = cap_states_core_a7,
};

static struct sched_group_energy energy_core_a15 = {
	  .nr_idle_states = ARRAY_SIZE(idle_states_core_a15),
	  .idle_states    = idle_states_core_a15,
	  .nr_cap_states  = ARRAY_SIZE(cap_states_core_a15),
	  .cap_states     = cap_states_core_a15,
};

/* sd energy functions */
static inline const struct sched_group_energy *cpu_cluster_energy(int cpu)
{
	return cpu_topology[cpu].socket_id ? &energy_cluster_a7 :
			&energy_cluster_a15;
}

static inline const struct sched_group_energy *cpu_core_energy(int cpu)
{
	return cpu_topology[cpu].socket_id ? &energy_core_a7 :
			&energy_core_a15;
}

static inline const struct sched_group_energy *cpu_sys_energy(int cpu)
{
	return NULL;
}

static unsigned long freqs_a7[] = {350000, 400000, 500000, 600000, 700000, 800000, 900000, 1000000};
static unsigned long freqs_a15[] = {500000, 600000, 700000, 800000, 900000, 10000000, 1100000, 1200000};

static DEFINE_PER_CPU(atomic_long_t, cpu_curr_capacity);

unsigned long arch_scale_curr_capacity(int cpu)
{
	return atomic_long_read(&per_cpu(cpu_curr_capacity, cpu));
}

unsigned long arch_scale_avg_capacity(struct task_struct *p)
{
	return arch_scale_curr_capacity(task_cpu(p));
}


void arch_scale_set_curr_freq(int cpu, unsigned long freq)
{
	unsigned long *freqs_table;
	unsigned long idx, nr_states;
	struct sched_group_energy *capacity_table;

	if (cpu_topology[cpu].socket_id) {
		freqs_table = freqs_a7;
		nr_states = ARRAY_SIZE(freqs_a7);
		capacity_table = &energy_core_a7;
	} else {
		freqs_table = freqs_a15;
		nr_states = ARRAY_SIZE(freqs_a15);
		capacity_table = &energy_core_a15;
	}

	for (idx = 0; idx < nr_states; idx++)
		if (freqs_table[idx] == freq)
			break;

	if (idx >= nr_states)
		idx = nr_states-1;

	atomic_long_set(&per_cpu(cpu_curr_capacity, cpu), capacity_table->cap_states[idx].cap);
	trace_printk("arm set_curr_capacity cpu=%d cap=%lu", cpu, capacity_table->cap_states[idx].cap);
}


static inline const int cpu_corepower_flags(void)
{
	return SD_SHARE_PKG_RESOURCES  | SD_SHARE_POWERDOMAIN | \
		SD_SHARE_CAP_STATES;
}

static struct sched_domain_topology_level arm_topology[] = {
#ifdef CONFIG_SCHED_MC
	{ cpu_coregroup_mask, cpu_corepower_flags, cpu_core_energy, SD_INIT_NAME(MC) },
#endif
	{ cpu_cpu_mask, 0, cpu_cluster_energy, SD_INIT_NAME(DIE) },
	{ NULL,	0, cpu_sys_energy},
};

/*
 * cluster_to_logical_mask - return cpu logical mask of CPUs in a cluster
 * @socket_id:		cluster HW identifier
 * @cluster_mask:	the cpumask location to be initialized, modified by the
 *			function only if return value == 0
 *
 * Return:
 *
 * 0 on success
 * -EINVAL if cluster_mask is NULL or there is no record matching socket_id
 */
int cluster_to_logical_mask(unsigned int socket_id, cpumask_t *cluster_mask)
{
	int cpu;

	if (!cluster_mask)
		return -EINVAL;

	for_each_online_cpu(cpu)
		if (socket_id == topology_physical_package_id(cpu)) {
			cpumask_copy(cluster_mask, topology_core_cpumask(cpu));
			return 0;
		}

	return -EINVAL;
}

/*
 * init_cpu_topology is called at boot when only one cpu is running
 * which prevent simultaneous write access to cpu_topology array
 */
void __init init_cpu_topology(void)
{
	unsigned int cpu;

	/* init core mask and capacity */
	for_each_possible_cpu(cpu) {
		struct cputopo_arm *cpu_topo = &(cpu_topology[cpu]);

		cpu_topo->thread_id = -1;
		cpu_topo->core_id =  -1;
		cpu_topo->socket_id = -1;
		cpumask_clear(&cpu_topo->core_sibling);
		cpumask_clear(&cpu_topo->thread_sibling);

		set_capacity_scale(cpu, SCHED_CAPACITY_SCALE);
		atomic_long_set(&per_cpu(cpu_curr_capacity, cpu),
						SCHED_CAPACITY_SCALE);
	}
	smp_wmb();

	parse_dt_topology();

	/* Set scheduler topology descriptor */
	set_sched_topology(arm_topology);
}

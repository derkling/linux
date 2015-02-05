/*
 * arch/arm64/kernel/topology.c
 *
 * Copyright (C) 2011,2013,2014 Linaro Limited.
 *
 * Based on the arm32 version written by Vincent Guittot in turn based on
 * arch/sh/kernel/topology.c
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
#include <asm/smp_plat.h>


/*
 * cpu capacity table
 * This per cpu data structure describes the relative capacity of each core.
 * On a heteregenous system, cores don't have the same computation capacity
 * and we reflect that difference in the cpu_capacity field so the scheduler can
 * take this difference into account during load balance. A per cpu structure
 * is preferred because each CPU updates its own cpu_capacity field during the
 * load balance except for idle cores. One idle core is selected to run the
 * rebalance_domains for all idle cores and the cpu_capacity can be updated
 * during this sequence.
 */
static DEFINE_PER_CPU(unsigned long, cpu_scale);

unsigned long arch_scale_cpu_capacity(struct sched_domain *sd, int cpu)
{
	return per_cpu(cpu_scale, cpu);
}

static void set_capacity_scale(unsigned int cpu, unsigned long capacity)
{
	per_cpu(cpu_scale, cpu) = capacity;
}

static int __init get_cpu_for_node(struct device_node *node)
{
	struct device_node *cpu_node;
	int cpu;

	cpu_node = of_parse_phandle(node, "cpu", 0);
	if (!cpu_node)
		return -1;

	for_each_possible_cpu(cpu) {
		if (of_get_cpu_node(cpu, NULL) == cpu_node) {
			of_node_put(cpu_node);
			return cpu;
		}
	}

	pr_crit("Unable to find CPU node for %s\n", cpu_node->full_name);

	of_node_put(cpu_node);
	return -1;
}

static int __init parse_core(struct device_node *core, int cluster_id,
			     int core_id)
{
	char name[10];
	bool leaf = true;
	int i = 0;
	int cpu;
	struct device_node *t;

	do {
		snprintf(name, sizeof(name), "thread%d", i);
		t = of_get_child_by_name(core, name);
		if (t) {
			leaf = false;
			cpu = get_cpu_for_node(t);
			if (cpu >= 0) {
				cpu_topology[cpu].cluster_id = cluster_id;
				cpu_topology[cpu].core_id = core_id;
				cpu_topology[cpu].thread_id = i;
			} else {
				pr_err("%s: Can't get CPU for thread\n",
				       t->full_name);
				of_node_put(t);
				return -EINVAL;
			}
			of_node_put(t);
		}
		i++;
	} while (t);

	cpu = get_cpu_for_node(core);
	if (cpu >= 0) {
		if (!leaf) {
			pr_err("%s: Core has both threads and CPU\n",
			       core->full_name);
			return -EINVAL;
		}

		cpu_topology[cpu].cluster_id = cluster_id;
		cpu_topology[cpu].core_id = core_id;
	} else if (leaf) {
		pr_err("%s: Can't get CPU for leaf core\n", core->full_name);
		return -EINVAL;
	}

	return 0;
}

static int __init parse_cluster(struct device_node *cluster, int depth)
{
	char name[10];
	bool leaf = true;
	bool has_cores = false;
	struct device_node *c;
	static int cluster_id __initdata;
	int core_id = 0;
	int i, ret;

	/*
	 * First check for child clusters; we currently ignore any
	 * information about the nesting of clusters and present the
	 * scheduler with a flat list of them.
	 */
	i = 0;
	do {
		snprintf(name, sizeof(name), "cluster%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			leaf = false;
			ret = parse_cluster(c, depth + 1);
			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	/* Now check for cores */
	i = 0;
	do {
		snprintf(name, sizeof(name), "core%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			has_cores = true;

			if (depth == 0) {
				pr_err("%s: cpu-map children should be clusters\n",
				       c->full_name);
				of_node_put(c);
				return -EINVAL;
			}

			if (leaf) {
				ret = parse_core(c, cluster_id, core_id++);
			} else {
				pr_err("%s: Non-leaf cluster with core %s\n",
				       cluster->full_name, name);
				ret = -EINVAL;
			}

			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	if (leaf && !has_cores)
		pr_warn("%s: empty cluster\n", cluster->full_name);

	if (leaf)
		cluster_id++;

	return 0;
}

struct cpu_efficiency {
	const char *compatible;
	unsigned long efficiency;
};

/*
 * Table of relative efficiency of each processors
 * The efficiency value must fit in 20bit and the final
 * cpu_scale value must be in the range
 *   0 < cpu_scale < SCHED_CAPACITY_SCALE.
 * Processors that are not defined in the table,
 * use the default SCHED_CAPACITY_SCALE value for cpu_scale.
 */
static const struct cpu_efficiency table_efficiency[] = {
	{ "arm,cortex-a57", 3891 },
	{ "arm,cortex-a53", 2048 },
	{ NULL, },
};

static unsigned long *__cpu_capacity;
#define cpu_capacity(cpu)	__cpu_capacity[cpu]

static unsigned long max_cpu_perf;

/*
 * Iterate all CPUs' descriptor in DT and compute the efficiency
 * (as per table_efficiency). Calculate the max cpu performance too.
 */
static int __init parse_dt_topology(void)
{
	struct device_node *cn, *map;
	int ret = 0;
	int cpu;

	cn = of_find_node_by_path("/cpus");
	if (!cn) {
		pr_err("No CPU information found in DT\n");
		return 0;
	}

	/*
	 * When topology is provided cpu-map is essentially a root
	 * cluster with restricted subnodes.
	 */
	map = of_get_child_by_name(cn, "cpu-map");
	if (!map)
		goto out;

	ret = parse_cluster(map, 0);
	if (ret != 0)
		goto out_map;

	/*
	 * Check that all cores are in the topology; the SMP code will
	 * only mark cores described in the DT as possible.
	 */
	for_each_possible_cpu(cpu) {
		if (cpu_topology[cpu].cluster_id == -1) {
			pr_err("CPU%d: No topology information specified\n",
			       cpu);
			ret = -EINVAL;
		}
	}

out_map:
	of_node_put(map);
out:
	of_node_put(cn);
	return ret;
}

static void __init parse_dt_cpu_capacity(void)
{
	const struct cpu_efficiency *cpu_eff;
	struct device_node *cn;
	int cpu;

	__cpu_capacity = kcalloc(nr_cpu_ids, sizeof(*__cpu_capacity),
				 GFP_NOWAIT);

	for_each_possible_cpu(cpu) {
		const u32 *rate;
		int len;
		unsigned long cpu_perf;

		/* Too early to use cpu->of_node */
		cn = of_get_cpu_node(cpu, NULL);
		if (!cn) {
			pr_err("Missing device node for CPU %d\n", cpu);
			continue;
		}

		for (cpu_eff = table_efficiency; cpu_eff->compatible; cpu_eff++)
			if (of_device_is_compatible(cn, cpu_eff->compatible))
				break;

		if (cpu_eff->compatible == NULL) {
			pr_warn("%s: Unknown CPU type\n", cn->full_name);
			continue;
		}

		rate = of_get_property(cn, "clock-frequency", &len);
		if (!rate || len != 4) {
			pr_err("%s: Missing clock-frequency property\n",
				cn->full_name);
			continue;
		}

		cpu_perf = ((be32_to_cpup(rate)) >> 20) * cpu_eff->efficiency;
		cpu_capacity(cpu) = cpu_perf;
		max_cpu_perf = max(max_cpu_perf, cpu_perf);
	}

	if (cpu < num_possible_cpus())
		max_cpu_perf = 0;
}

/*
 * Scheduler load-tracking scale-invariance
 *
 * Provides the scheduler with a scale-invariance correction factor that
 * compensates for frequency scaling.
 */

static DEFINE_PER_CPU(atomic_long_t, cpu_curr_freq);
static DEFINE_PER_CPU(atomic_long_t, cpu_max_freq);

/* cpufreq callback function setting current cpu frequency */
void arch_scale_set_curr_freq(int cpu, unsigned long freq)
{
	atomic_long_set(&per_cpu(cpu_curr_freq, cpu), freq);
}

/* cpufreq callback function setting max cpu frequency */
void arch_scale_set_max_freq(int cpu, unsigned long freq)
{
	atomic_long_set(&per_cpu(cpu_max_freq, cpu), freq);
}

unsigned long arch_scale_freq_capacity(struct sched_domain *sd, int cpu)
{
	unsigned long curr = atomic_long_read(&per_cpu(cpu_curr_freq, cpu));
	unsigned long max = atomic_long_read(&per_cpu(cpu_max_freq, cpu));

	if (!max)
		return SCHED_CAPACITY_SCALE;

	return (curr * SCHED_CAPACITY_SCALE) / max;
}

/*
 * cpu topology table
 */
struct cpu_topology cpu_topology[NR_CPUS];
EXPORT_SYMBOL_GPL(cpu_topology);

/*
 * ARM JUNO specific energy cost model data. There are no unit requirements for
 * the data. Data can be normalized to any reference point, but the
 * normalization must be consistent. That is, one bogo-joule/watt must be the
 * same quantity for all data, but we don't care what it is.
 */

static struct idle_state idle_states_cluster_a53[] = {
	{ .power = 16 },
	{ .power = 39 },
};

static struct idle_state idle_states_cluster_a57[] = {
	{ .power = 28 },
	{ .power = 37 },
};

static struct capacity_state cap_states_cluster_a53[] = {
        /* Power per cluster */
	{ .cap = 236, .power = 25, },
	{ .cap = 302, .power = 30, },
	{ .cap = 368, .power = 37, },
	{ .cap = 407, .power = 44, },
	{ .cap = 447, .power = 75, },
};

static struct capacity_state cap_states_cluster_a57[] = {
        /* Power per cluster */
	{ .cap = 418, .power = 25, },
	{ .cap = 581, .power = 30, },
	{ .cap = 745, .power = 43, },
	{ .cap = 884, .power = 47, },
	{ .cap = 1024, .power = 60, },
};

static struct sched_group_energy energy_cluster_a53 = {
	.nr_idle_states = ARRAY_SIZE(idle_states_cluster_a53),
	.idle_states    = idle_states_cluster_a53,
	.nr_cap_states  = ARRAY_SIZE(cap_states_cluster_a53),
	.cap_states     = cap_states_cluster_a53,
};

static struct sched_group_energy energy_cluster_a57 = {
	.nr_idle_states = ARRAY_SIZE(idle_states_cluster_a57),
	.idle_states    = idle_states_cluster_a57,
	.nr_cap_states  = ARRAY_SIZE(cap_states_cluster_a57),
	.cap_states     = cap_states_cluster_a57,
};

static struct idle_state idle_states_core_a53[] = {
	{ .power = 0 },
	{ .power = 6 },
};

static struct idle_state idle_states_core_a57[] = {
	{ .power = 0 },
	{ .power = 6 },
};

static struct capacity_state cap_states_core_a53[] = {
        /* Power per cpu */
	{ .cap = 236, .power = 33, },
	{ .cap = 302, .power = 46, },
	{ .cap = 368, .power = 62, },
	{ .cap = 407, .power = 77, },
	{ .cap = 447, .power = 83, },
};

static struct capacity_state cap_states_core_a57[] = {
        /* Power per cpu */
	{ .cap = 418, .power = 166, },
	{ .cap = 581, .power = 252, },
	{ .cap = 745, .power = 358, },
	{ .cap = 884, .power = 479, },
	{ .cap = 1024, .power = 615, },
};

static struct sched_group_energy energy_core_a53 = {
	.nr_idle_states = ARRAY_SIZE(idle_states_core_a53),
	.idle_states    = idle_states_core_a53,
	.nr_cap_states  = ARRAY_SIZE(cap_states_core_a53),
	.cap_states     = cap_states_core_a53,
};

static struct sched_group_energy energy_core_a57 = {
	  .nr_idle_states = ARRAY_SIZE(idle_states_core_a57),
	  .idle_states    = idle_states_core_a57,
	  .nr_cap_states  = ARRAY_SIZE(cap_states_core_a57),
	  .cap_states     = cap_states_core_a57,
};

/* sd energy functions */
static inline const struct sched_group_energy *cpu_cluster_energy(int cpu)
{
	return cpu_topology[cpu].cluster_id ? &energy_cluster_a53 :
			&energy_cluster_a57;
}

static inline const struct sched_group_energy *cpu_core_energy(int cpu)
{
	return cpu_topology[cpu].cluster_id ? &energy_core_a53 :
			&energy_core_a57;
}

const struct cpumask *cpu_coregroup_mask(int cpu)
{
	return &cpu_topology[cpu].core_sibling;
}

static inline const int cpu_corepower_flags(void)
{
	return SD_SHARE_PKG_RESOURCES  | SD_SHARE_POWERDOMAIN | \
	       SD_SHARE_CAP_STATES;
}

static struct sched_domain_topology_level arm64_topology[] = {
#ifdef CONFIG_SCHED_MC
	{ cpu_coregroup_mask, cpu_corepower_flags, cpu_core_energy, SD_INIT_NAME(MC) },
#endif
	{ cpu_cpu_mask, 0, cpu_cluster_energy, SD_INIT_NAME(DIE) },
	{ NULL, },
};

/*
 * Look for a customed capacity of a CPU in the cpu_topo_data table during the
 * boot. The update of all CPUs is in O(n^2) for heteregeneous system but the
 * function returns directly for SMP system or if there is no complete set
 * of cpu efficiency, clock frequency data for each cpu.
 */
static void update_cpu_capacity(unsigned int cpu)
{
	unsigned long capacity;
	int max_cap_idx;

	if (!cpu_capacity(cpu))
		return;

	if (!max_cpu_perf)
		cpu_capacity(cpu) = 0;

	max_cap_idx = cpu_core_energy(cpu)->nr_cap_states - 1;
	capacity = cpu_core_energy(cpu)->cap_states[max_cap_idx].cap;
	set_capacity_scale(cpu, capacity);

	pr_info("CPU%d: update cpu_capacity %lu\n",
		cpu, arch_scale_cpu_capacity(NULL, cpu));
}

static void update_siblings_masks(unsigned int cpuid)
{
	struct cpu_topology *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];
	int cpu;

	if (cpuid_topo->cluster_id == -1) {
		/*
		 * DT does not contain topology information for this cpu.
		 */
		pr_debug("CPU%u: No topology information configured\n", cpuid);
		return;
	}

	/* update core and thread sibling masks */
	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		if (cpuid_topo->cluster_id != cpu_topo->cluster_id)
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
}


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

	for_each_online_cpu(cpu) {
		if (socket_id == topology_physical_package_id(cpu)) {
			cpumask_copy(cluster_mask, topology_core_cpumask(cpu));
			return 0;
		}
	}

	return -EINVAL;
}

void store_cpu_topology(unsigned int cpuid)
{
	update_siblings_masks(cpuid);
	update_cpu_capacity(cpuid);
}

static void __init reset_cpu_topology(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct cpu_topology *cpu_topo = &cpu_topology[cpu];

		cpu_topo->thread_id = -1;
		cpu_topo->core_id = 0;
		cpu_topo->cluster_id = -1;

		cpumask_clear(&cpu_topo->core_sibling);
		cpumask_set_cpu(cpu, &cpu_topo->core_sibling);
		cpumask_clear(&cpu_topo->thread_sibling);
		cpumask_set_cpu(cpu, &cpu_topo->thread_sibling);
	}
}

static void __init reset_cpu_capacity(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		set_capacity_scale(cpu, SCHED_CAPACITY_SCALE);
}

void __init init_cpu_topology(void)
{
	reset_cpu_topology();

	/*
	 * Discard anything that was parsed if we hit an error so we
	 * don't use partial information.
	 */
	if (parse_dt_topology())
		reset_cpu_topology();
	else
		set_sched_topology(arm64_topology);

	reset_cpu_capacity();
	parse_dt_cpu_capacity();
}

#ifndef _ASM_ARM_TOPOLOGY_H
#define _ASM_ARM_TOPOLOGY_H

#ifdef CONFIG_ARM_CPU_TOPOLOGY

#include <linux/cpumask.h>
#include <linux/cpufreq_fei.h>

struct cputopo_arm {
	int thread_id;
	int core_id;
	int socket_id;
	cpumask_t thread_sibling;
	cpumask_t core_sibling;
};

extern struct cputopo_arm cpu_topology[NR_CPUS];

#define topology_physical_package_id(cpu)	(cpu_topology[cpu].socket_id)
#define topology_core_id(cpu)		(cpu_topology[cpu].core_id)
#define topology_core_cpumask(cpu)	(&cpu_topology[cpu].core_sibling)
#define topology_sibling_cpumask(cpu)	(&cpu_topology[cpu].thread_sibling)

void init_cpu_topology(void);
void store_cpu_topology(unsigned int cpuid);
const struct cpumask *cpu_coregroup_mask(int cpu);

DECLARE_PER_CPU(unsigned long, cpu_scale);

static inline
unsigned long scale_cpu_capacity(void __always_unused *sd, int cpu)
{
#ifdef CONFIG_CPU_FREQ
	unsigned long max_cap_scale = cpufreq_scale_max_freq_capacity(cpu);

	/*
	 * 10 is SCHED_CAPACITY_SHIFT, unfortunately there is no easy way to
	 * include linux/sched.h here.
	 */
	return per_cpu(cpu_scale, cpu) * max_cap_scale >> 10;
#else
	return per_cpu(cpu_scale, cpu);
#endif
}

#ifdef CONFIG_CPU_FREQ
#define arch_scale_freq_capacity cpufreq_scale_freq_capacity
#endif
#define arch_scale_cpu_capacity scale_cpu_capacity

#else

static inline void init_cpu_topology(void) { }
static inline void store_cpu_topology(unsigned int cpuid) { }

#endif

#include <asm-generic/topology.h>

#endif /* _ASM_ARM_TOPOLOGY_H */

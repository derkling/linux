/*
 * include/linux/arch_topology.h - arch specific cpu topology information
 */
#ifndef _LINUX_ARCH_TOPOLOGY_H_
#define _LINUX_ARCH_TOPOLOGY_H_

void atd_normalize_cpu_capacity(void);

struct device_node;
int atd_parse_cpu_capacity(struct device_node *cpu_node, int cpu);

struct sched_domain;
unsigned long atd_scale_cpu_capacity(struct sched_domain *sd, int cpu);

#ifdef CONFIG_CPU_FREQ
unsigned long atd_scale_freq_capacity(struct sched_domain *sd, int cpu);
#endif

void atd_set_capacity_scale(unsigned int cpu, unsigned long capacity);

int atd_get_mc_sd_flags(void);
int atd_get_die_sd_flags(void);

#endif /* _LINUX_ARCH_TOPOLOGY_H_ */

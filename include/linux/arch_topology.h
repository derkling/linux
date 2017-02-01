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

void atd_set_capacity_scale(unsigned int cpu, unsigned long capacity);

#endif /* _LINUX_ARCH_TOPOLOGY_H_ */

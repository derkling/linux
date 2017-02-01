/*
 * include/linux/arch_topology.h - arch specific cpu topology information
 */
#ifndef _LINUX_ARCH_TOPOLOGY_H_
#define _LINUX_ARCH_TOPOLOGY_H_

#include <linux/device.h>
#include <linux/topology.h>

void normalize_cpu_capacity(void);
int __init parse_cpu_capacity(struct device_node *cpu_node, int cpu);
unsigned long arch_scale_cpu_capacity(struct sched_domain *sd, int cpu);
void set_capacity_scale(unsigned int cpu, unsigned long capacity);

#endif /* _LINUX_ARCH_TOPOLOGY_H_ */

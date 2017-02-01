/*
 * include/linux/arch_topology.h - arch specific cpu topology information
 */
#ifndef _LINUX_ARCH_TOPOLOGY_H_
#define _LINUX_ARCH_TOPOLOGY_H_

#include <linux/device.h>
#include <linux/topology.h>

void drv_normalize_cpu_capacity(void);
int __init drv_parse_cpu_capacity(struct device_node *cpu_node, int cpu);
unsigned long drv_scale_cpu_capacity(struct sched_domain *sd, int cpu);
void drv_set_capacity_scale(unsigned int cpu, unsigned long capacity);

#endif /* _LINUX_ARCH_TOPOLOGY_H_ */

#ifndef ARM_BL_CPUFREQ_HVC_H
#define ARM_BL_CPUFREQ_HVC_H

#ifndef __ASSEMBLY__
int __arm_bl_get_cluster(void);
void __arm_bl_switch_cluster(void);
#endif /* ! __ASSEMBLY__ */

/* Hypervisor call numbers for the ARM big.LITTLE switcher: */
#define ARM_BL_HVC_SWITCH_CLUSTER 1
#define ARM_BL_HVC_GET_MPIDR 2

#endif /* ! ARM_BL_CPUFREQ_HVC_H */

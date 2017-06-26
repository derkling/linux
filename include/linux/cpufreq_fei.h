/*
 * linux/include/linux/cpufreq_fei.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LINUX_CPUFREQ_FEI_H
#define _LINUX_CPUFREQ_FEI_H

DECLARE_PER_CPU(unsigned long, freq_scale);
DECLARE_PER_CPU(unsigned long, max_freq_scale);

static inline
unsigned long cpufreq_scale_freq_capacity(void __always_unused *sd, int cpu)
{
	return per_cpu(freq_scale, cpu);
}

static inline unsigned long cpufreq_scale_max_freq_capacity(int cpu)
{
	return per_cpu(max_freq_scale, cpu);
}

#endif /* _LINUX_CPUFREQ_FEI_H */

/*
 * arch/arm64/kernel/actmon.h
 *
 * Copyright (C) 2018 Arm Limited.
 *
 */

#ifndef _ASM_ACTMON_H
#define _ASM_ACTMON_H

#include <linux/sched.h>

extern unsigned long cpufreq_scale_freq_capacity(struct sched_domain *sd, int cpu);
void actmon_update_cpu_freq_max(int cpu, u32 max_freq);
unsigned long actmon_scale_freq_capacity(struct sched_avg *sa, int cpu);

#endif /*_ASM_ACTMON_H*/

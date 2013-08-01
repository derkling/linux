/*
 * cpuidle_scheduled.h - interface for the scheduled CPU idle governor
 *
 * Copyright 2013 Linaro limited
 *
 * This code is licenced under the GPL version 2 as described
 * in the COPYING file that acompanies the Linux Kernel.
 */

#ifndef _LINUX_CPUIDLE_SCHEDULED_H
#define _LINUX_CPUIDLE_SCHEDULED_H

#ifdef CONFIG_CPU_IDLE_GOV_SCHEDULED

#include <linux/cpuidle.h>

extern void cpuidle_scheduled_result(struct cpuidle_state *, unsigned int);

#endif /* CONFIG_CPU_IDLE_GOV_SCHEDULED */

#endif /* _LINUX_CPUIDLE_SCHEDULED_H */

/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_ENERGY_MODEL_H
#define _LINUX_ENERGY_MODEL_H
#include <linux/types.h>
#include <linux/cpumask.h>
#include <linux/jump_label.h>
#include <linux/rcupdate.h>
#include <linux/kobject.h>
#include <linux/sched/cpufreq.h>

#ifdef CONFIG_ENERGY_MODEL
struct em_cap_state {
	unsigned long capacity;
	unsigned long frequency;
	unsigned long power;
};

struct em_freq_domain {
	struct em_cap_state *cs_table;
	int nr_cap_states;
	cpumask_t cpus;

	/* Private fields used by the EM core. */
	struct kobject kobj;
	struct list_head next;
};

int em_register_freq_domain(cpumask_t *span, int nr_states,
			int (*get_power)(unsigned long*, unsigned long*, int));
void em_rescale_cpu_capacity(void);
struct em_freq_domain *em_cpu_get(int cpu);
void em_cpu_put(struct em_freq_domain *fd);

/**
 * em_fd_energy() - Estimates the energy consumed by the CPUs of a freq. domain
 * @fd		: frequency domain for which energy has to be estimated
 * @max_util	: highest utilization among CPUs of the domain
 * @sum_util	: sum of the utilization of all CPUs in the domain
 *
 * Return: the sum of the energy consumed by the CPUs of the domain assuming
 * an Operating Point satisfying the max utilization of the domain.
 */
static inline unsigned long em_fd_energy(struct em_freq_domain *fd,
				unsigned long max_util, unsigned long sum_util)
{
	struct em_cap_state *cs, *table;
	unsigned long freq;
	int i;

	table = rcu_dereference(fd->cs_table);
	if (!table)
		return 0;

	/* Map the utilization value to a frequency */
	cs = &table[fd->nr_cap_states-1];
	freq = map_util_freq(max_util, cs->frequency, cs->capacity);

	/* Find the lowest capacity state above this frequency */
	for (i = 0; i < fd->nr_cap_states; i++) {
		cs = &fd->cs_table[i];
		if (cs->frequency >= freq)
			break;
	}

	return cs->power * sum_util / cs->capacity;
}
#else
struct em_freq_domain;
int em_register_freq_domain(cpumask_t *span, int nr_states,
			int (*get_power)(unsigned long*, unsigned long*, int))
{
	return -ENOTSUPP;
}
static inline struct em_freq_domain *em_cpu_get(int cpu)
{
	return NULL;
}
static inline unsigned long em_fd_energy(struct em_freq_domain *fd,
			unsigned long max_util, unsigned long sum_util)
{
	return 0;
}
static inline void em_rescale_cpu_capacity(void) { }
static inline void em_cpu_put(struct em_freq_domain *fd) { }
#endif

#endif

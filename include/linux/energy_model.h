// SPDX-License-Identifier: GPL-2.0
#ifndef _LINUX_ENERGY_MODEL_H
#define _LINUX_ENERGY_MODEL_H
#include <linux/types.h>
#include <linux/cpumask.h>
#include <linux/jump_label.h>
#include <linux/sched/cpufreq.h>

#ifdef CONFIG_ENERGY_MODEL
struct em_cap_state {
	unsigned long cap;
	unsigned long freq;
	unsigned long power;
};

struct em_freq_domain {
	struct em_cap_state *cs_table;
	int nr_cap_states;
	cpumask_t span;

	/* Private fields used by the EM core. */
	struct kobject kobj;
	struct list_head next;
};


int em_register_freq_domain(cpumask_t *span, int nr_states,
						long (*get_power)(long*, int));
void em_rescale_cpu_capacity(void);
struct em_freq_domain *em_fd_of(int cpu);
void em_fd_put(struct em_freq_domain *fd);

/* XXX: Documentation here */
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
	freq = map_util_freq(max_util, cs->freq, cs->cap);

	/* Find the lowest capacity state above this frequency */
	for (i = 0; i < fd->nr_cap_states; i++) {
		cs = &fd->cs_table[i];
		if (cs->freq >= freq)
			break;
	}

	return cs->power * sum_util / cs->cap;
}
#else
struct em_freq_domain;
#define freq_domain_span(fd) NULL
static inline int em_register_freq_domain(cpumask_t *span, int nr_states, long (*get_power)(long*, int)) { return -ENOTSUPP; }
static inline void em_rescale_cpu_capacity(void) { }
static inline struct em_freq_domain *em_fd_of(int cpu) { return NULL; }
static inline void em_fd_put(struct em_freq_domain *fd) { }
static inline unsigned long em_fd_energy(struct em_freq_domain *fd, unsigned long max_util, unsigned long sum_util) { return 0; }
#endif

#endif

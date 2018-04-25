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

struct em_cap_state_table {
	struct em_cap_state *cap_states;
	int nr_cap_states;
};

struct em_freq_domain {
	cpumask_t span;
	struct list_head next;
};

struct energy_model {
	struct em_cap_state_table ** __percpu cap_tables;
	struct list_head freq_domains;
};

extern struct energy_model *energy_model;
static inline struct energy_model *dereference_energy_model(void)
{
	return rcu_dereference(energy_model);
}

int em_build_freq_domain(cpumask_t *span, int nr_states, long (*f)(long*, int));
void em_normalize_cpu_capacity(void);

#define freq_domain_span(fd) &((fd)->span)
#define for_each_freq_domain(fd, em) \
			list_for_each_entry(fd, &em->freq_domains, next)

static inline struct em_cap_state *em_find_cap_state_ceil(int cpu,
				unsigned long util, struct energy_model* em)
{
	struct em_cap_state_table *table = *per_cpu_ptr(em->cap_tables, cpu);
	struct em_cap_state *cs = NULL;
	unsigned long freq;
	int i;

	/* Map the utilization value to a frequency */
	cs = &table->cap_states[table->nr_cap_states-1];
	freq = map_util_freq(util, cs->freq, cs->cap);

	/* Find the lowest capacity state above this frequency */
	for (i = 0; i < table->nr_cap_states; i++) {
		cs = &table->cap_states[i];
		if (cs->freq >= freq)
			break;
	}

	return cs;
}

static inline long em_get_cap_state_energy(struct em_cap_state *cs, long util)
{
	return cs->power * util / cs->cap;
}
#else
struct em_freq_domain;
struct em_cap_state;
struct energy_model;

#define freq_domain_span(fd) NULL
#define for_each_freq_domain(fd, em) for (fd=NULL; fd;)

static inline struct energy_model *dereference_energy_model(void)
{
	return NULL;
}

static inline int em_build_freq_domain(cpumask_t *span, int nr_states, long (*f)(long*, int))
{
	return -ENOTSUPP;
}

static inline void em_normalize_cpu_capacity(void) { }

static inline struct em_cap_state *em_find_cap_state_ceil(int cpu,
				unsigned long util, struct energy_model* em)
{
	return NULL;
}
static inline long em_get_cap_state_energy(struct em_cap_state *cs, long util)
{
	return -EINVAL;
}
#endif

#endif

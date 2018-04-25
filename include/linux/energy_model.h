// SPDX-License-Identifier: GPL-2.0
#ifndef _LINUX_ENERGY_MODEL_H
#define _LINUX_ENERGY_MODEL_H
#include <linux/types.h>
#include <linux/cpumask.h>
#include <linux/jump_label.h>

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
extern struct static_key_false _energy_model_ready;
static inline bool energy_model_ready(void)
{
	return static_branch_unlikely(&_energy_model_ready);
}

static inline struct energy_model *dereference_energy_model(void)
{
	return rcu_dereference(energy_model);
}

int em_build_freq_domain(cpumask_t *span, int nr_states, long (*f)(long*, int));
void em_normalize_cpu_capacity(void);

#define freq_domain_span(fd) &((fd)->span)
#define for_each_freq_domain(fd, em) \
			list_for_each_entry(fd, &em->freq_domains, next)
#else

struct em_freq_domain;
struct em_cap_state;
struct energy_model;

#define freq_domain_span(fd) NULL
#define for_each_freq_domain(fd, em) for (fd=NULL; fd;)

static inline bool energy_model_ready(void)
{
	return false;
}

static inline struct energy_model *dereference_energy_model(void)
{
	return NULL;
}

static inline int em_build_freq_domain(cpumask_t *span, int nr_states, long (*f)(long*, int))
{
	return -ENOTSUPP;
}

static inline void em_normalize_cpu_capacity(void) { }
#endif

#endif

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
	struct kobject kobj;
	struct list_head next;
	struct rcu_head rcu;
};

extern struct list_head freq_domain_list;

#define freq_domain_span(fd) &((fd)->span)

/**
 * for_each_freq_domain() - Iterates over all available frequency domains.
 *
 * A client holding references on some frequency domains is guaranteed to see
 * _at least_ those in the list. Other frequency domains referenced by other
 * clients will also be visible in the list.
 *
 * If a client A traverses the list and sees a domain X on which it has no
 * reference, there is a risk of concurrent deletion of that domain if client
 * B releases its reference on X. This problem is avoided using RCU
 * synchronization mechanisms impling that for_each_freq_domain() must be used
 * under rcu_read_lock() protection.
 */
#define for_each_freq_domain(fd) \
			list_for_each_entry_rcu(fd, &freq_domain_list, next)

int em_register_freq_domain(cpumask_t *span, int nr_states,
						long (*get_power)(long*, int));
void em_rescale_cpu_capacity(void);

static inline struct em_freq_domain *em_fd_of_raw(int cpu)
{
	struct em_freq_domain *fd;

	if (list_empty(&freq_domain_list))
		return NULL;

	for_each_freq_domain(fd) {
		if (cpumask_test_cpu(cpu, freq_domain_span(fd)))
			return fd;
	}

	return NULL;
}

static inline struct em_freq_domain *em_fd_of(int cpu)
{
	struct em_freq_domain *fd = em_fd_of_raw(cpu);

	if (fd)
		kobject_get(&fd->kobj);

	return fd;
}

static inline void em_fd_put(struct em_freq_domain *fd)
{
	if (!fd)
		return;
	kobject_put(&fd->kobj);
}

/* XXX: Documentation here */
static inline unsigned long em_fd_energy(struct em_freq_domain *fd,
				unsigned long max_util, unsigned long sum_util)
{
	struct em_cap_state *cs = NULL;
	unsigned long freq;
	int i;

	/* Map the utilization value to a frequency */
	cs = &fd->cs_table[fd->nr_cap_states-1];
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
#define for_each_freq_domain(fd, em) for (fd=NULL; fd;)
static inline struct em_freq_domain *em_fd_of(int cpu) { return NULL; }
static inline int em_register_freq_domain(cpumask_t *span, int nr_states, long (*get_power)(long*, int)) { return -ENOTSUPP; }
static inline void em_rescale_cpu_capacity(void) { }
static inline unsigned long em_fd_energy(struct em_freq_domain *fd, unsigned long max_util, unsigned long sum_util) { return 0; }
#endif

#endif

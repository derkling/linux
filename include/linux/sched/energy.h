// SPDX-License-Identifier: GPL-2.0
#ifndef _LINUX_SCHED_ENERGY_H
#define _LINUX_SCHED_ENERGY_H

struct capacity_state {
	unsigned long cap;	/* compute capacity */
	unsigned long power;	/* power consumption at this compute capacity */
};

struct sched_energy_model {
	int nr_cap_states;
	struct capacity_state *cap_states;
};

struct freq_domain {
	struct list_head next;
	cpumask_t span;
};

#if defined(CONFIG_SMP) && defined(CONFIG_PM_OPP)
extern struct sched_energy_model ** __percpu energy_model;
extern struct static_key_false sched_energy_present;
extern struct list_head sched_freq_domains;

static inline bool sched_energy_enabled(void)
{
	return static_branch_unlikely(&sched_energy_present);
}

static inline
struct capacity_state *find_cap_state(int cpu, unsigned long util)
{
	struct sched_energy_model *em = *per_cpu_ptr(energy_model, cpu);
	struct capacity_state *cs = NULL;
	int i;

	util += util >> 2;

	for (i = 0; i < em->nr_cap_states; i++) {
		cs = &em->cap_states[i];
		if (cs->cap >= util)
			break;
	}

	return cs;
}

static inline struct cpumask *freq_domain_span(struct freq_domain *fd)
{
	return &fd->span;
}

extern void init_sched_energy(void);

#define for_each_freq_domain(fdom) \
	list_for_each_entry(fdom, &sched_freq_domains, next)

#else
struct freq_domain;
static inline bool sched_energy_enabled(void) { return false; }
static inline struct cpumask
*freq_domain_span(struct freq_domain *fd) { return NULL; }
static inline struct capacity_state
*find_cap_state(int cpu, unsigned long util) { return NULL; }
static inline void init_sched_energy(void) { }
#define for_each_freq_domain(fdom) for (; fdom; fdom = NULL)
#endif

#endif /* _LINUX_SCHED_ENERGY_H */

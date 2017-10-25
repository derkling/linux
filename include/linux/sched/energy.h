#ifndef _LINUX_SCHED_ENERGY_H
#define _LINUX_SCHED_ENERGY_H

#ifdef CONFIG_SMP
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

extern struct sched_energy_model __percpu **energy_model;
extern struct static_key_false sched_energy_present;
extern struct list_head freq_domains;
#define for_each_freq_domain(fdom) \
			list_for_each_entry(fdom, &freq_domains, next)

void init_sched_energy(void);
#else
static inline void init_sched_energy(void) { }
#endif

#endif

/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_ENERGY_MODEL_H
#define _LINUX_ENERGY_MODEL_H
#include <linux/cpumask.h>
#include <linux/jump_label.h>
#include <linux/kobject.h>
#include <linux/rcupdate.h>
#include <linux/sched/cpufreq.h>
#include <linux/types.h>

#ifdef CONFIG_ENERGY_MODEL

struct em_cap_state {
	unsigned long capacity;
	unsigned long frequency; /* Kilo-hertz */
	unsigned long power; /* Milli-watts */
	unsigned long power_coeff; /* power * "max_freq" / frequency */
};

struct em_cs_table {
	struct em_cap_state *state; /* Cap. states, in ascending order. */
	int nr_cap_states;
	struct rcu_head rcu;
};

struct em_freq_domain {
	struct em_cs_table *cs_table; /* Cap. state table, RCU-protected */
	cpumask_t cpus; /* CPUs of the frequency domain. */
	struct kobject kobj;
};

#define EM_CPU_MAX_POWER 0xFFFF

struct em_data_callback {
	/**
	 * active_power() - Provide power at the next capacity state of a CPU
	 * @power	: Active power at the capacity state in mW (modified)
	 * @freq	: Frequency at the capacity state in kHz (modified)
	 * @cpu		: CPU for which we do this operation
	 *
	 * active_power() must find the lowest capacity state of 'cpu' above
	 * 'freq' and update 'power' and 'freq' to the matching active power
	 * and frequency.
	 *
	 * The power is the one of a single CPU in the domain, expressed in
	 * milli-watts. It is expected to fit in the [0, EM_CPU_MAX_POWER]
	 * range.
	 *
	 * Return 0 on success.
	 */
	int (*active_power) (unsigned long *power, unsigned long *freq, int cpu);
};
#define EM_DATA_CB(_active_power_cb) { .active_power = &_active_power_cb }

void em_rescale_cpu_capacity(void);
struct em_freq_domain *em_cpu_get(int cpu);
int em_register_freq_domain(cpumask_t *span, unsigned int nr_states,
						struct em_data_callback *cb);

/**
 * em_fd_energy() - Estimates the energy consumed by the CPUs of a freq. domain
 * @fd		: frequency domain for which energy has to be estimated
 * @max_util	: highest utilization among CPUs of the domain
 * @sum_util	: sum of the utilization of all CPUs in the domain
 *
 * Return: the sum of the energy consumed by the CPUs of the domain assuming
 * a capacity state satisfying the max utilization of the domain.
 */
static inline unsigned long em_fd_energy(struct em_freq_domain *fd,
				unsigned long max_util, unsigned long sum_util)
{
	struct em_cs_table *cs_table;
	struct em_cap_state *cs;
	unsigned long freq, scale_cpu;
	int i;

	cs_table = rcu_dereference(fd->cs_table);
	if (!cs_table)
		return 0;

	/* Map the utilization value to a frequency */
	scale_cpu = arch_scale_cpu_capacity(NULL, cpumask_first(&fd->cpus));
	cs = &cs_table->state[cs_table->nr_cap_states - 1];
	freq = map_util_freq(max_util, cs->frequency, scale_cpu);

	/* Find the lowest capacity state above this frequency */
	for (i = 0; i < cs_table->nr_cap_states; i++) {
		cs = &cs_table->state[i];
		if (cs->frequency >= freq)
			break;
	}

	/*
	 * The capacity of a CPU at a specific performance state is defined as:
	 *
	 *     cap = freq * scale_cpu / max_freq
	 *
	 * The energy consumed by this CPU can be estimated as:
	 *
	 *     nrg = power * util / cap
	 *
	 * because (util / cap) represents the percentage of busy time of the
	 * CPU. Based on those definitions, we have:
	 *
	 *     nrg = power * util * max_freq / (scale_cpu * freq)
	 *
	 * which can be re-arranged as a product of two terms:
	 *
	 *     nrg = (power * max_freq / freq) * (util / scale_cpu)
	 *
	 * The first term is static, and is stored in the em_cap_state struct
	 * as "power_coeff". The parameters of the second term change over time
	 * so it is always re-computed.
	 */
	return cs->power_coeff * sum_util / scale_cpu;
}

/**
 * em_fd_nr_cap_states() - Get the number of capacity states of a freq. domain
 * @fd		: frequency domain for which want to do this
 *
 * Return: the number of capacity state in the frequency domain table
 */
static inline int em_fd_nr_cap_states(struct em_freq_domain *fd)
{
	struct em_cs_table *table = rcu_dereference(fd->cs_table);

	return table->nr_cap_states;
}

#else
struct em_freq_domain {};
struct em_data_callback {};
#define EM_DATA_CB(_active_power_cb) { }

static inline int em_register_freq_domain(cpumask_t *span,
			unsigned int nr_states, struct em_data_callback *cb)
{
	return -EINVAL;
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
static inline int em_fd_nr_cap_states(struct em_freq_domain *fd)
{
	return 0;
}
static inline void em_rescale_cpu_capacity(void) { }
#endif

#endif

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
	unsigned long frequency; /* Kilo-hertz */
	unsigned long power; /* Milli-watts */
	unsigned long cost; /* power * max_frequency / frequency */
};

struct em_freq_domain {
	struct em_cap_state *table; /* Capacity states, in ascending order. */
	int nr_cap_states;
	struct kobject kobj;
	unsigned long cpus[0]; /* CPUs of the frequency domain. */
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

struct em_freq_domain *em_cpu_get(int cpu);
int em_register_freq_domain(cpumask_t *span, unsigned int nr_states,
						struct em_data_callback *cb);

/**
 * em_fd_energy() - Estimates the energy consumed by the CPUs of a freq. domain
 * @fd		: frequency domain for which energy has to be estimated
 * @max_util	: highest utilization among CPUs of the domain
 * @sum_util	: sum of the utilization of all CPUs in the domain
 *
 * em_fd_energy() dereferences the capacity state table of the frequency
 * domain, so it must be called under RCU read lock.
 *
 * Return: the sum of the energy consumed by the CPUs of the domain assuming
 * a capacity state satisfying the max utilization of the domain.
 */
static inline unsigned long em_fd_energy(struct em_freq_domain *fd,
				unsigned long max_util, unsigned long sum_util)
{
	unsigned long freq, scale_cpu;
	struct em_cap_state *cs;
	int i, cpu;

	/* Map the utilization value to a frequency */
	cpu = cpumask_first(to_cpumask(fd->cpus));
	scale_cpu = arch_scale_cpu_capacity(NULL, cpu);
	cs = &fd->table[fd->nr_cap_states - 1];
	freq = map_util_freq(max_util, cs->frequency, scale_cpu);

	/* Find the lowest capacity state above this frequency */
	for (i = 0; i < fd->nr_cap_states; i++) {
		cs = &fd->table[i];
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
	 * as 'cost'. The parameters of the second term change at run-time.
	 */
	return cs->cost * sum_util / scale_cpu;
}

/**
 * em_fd_nr_cap_states() - Get the number of capacity states of a freq. domain
 * @fd		: frequency domain for which want to do this
 *
 * Return: the number of capacity state in the frequency domain table
 */
static inline int em_fd_nr_cap_states(struct em_freq_domain *fd)
{
	return fd->nr_cap_states;
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
#endif

#endif

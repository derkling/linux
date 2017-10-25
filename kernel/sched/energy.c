// SPDX-License-Identifier: GPL-2.0
/*
 * Energy-aware scheduling models
 *
 * Copyright (C) 2018, Arm Ltd.
 * Written by: Quentin Perret, Arm Ltd.
 */

#define pr_fmt(fmt) "sched-energy: " fmt

#include <linux/sched/topology.h>
#include <linux/sched/energy.h>
#include <linux/pm_opp.h>

#include "sched.h"

DEFINE_STATIC_KEY_FALSE(sched_energy_present);
struct sched_energy_model ** __percpu energy_model;

/*
 * A copy of the cpumasks representing the frequency domains is kept private
 * to the scheduler. They are stacked in a dynamically allocated linked list
 * as we don't know how many frequency domains the system has.
 */
LIST_HEAD(sched_freq_domains);

static struct sched_energy_model *build_energy_model(int cpu)
{
	unsigned long cap_scale = arch_scale_cpu_capacity(NULL, cpu);
	unsigned long cap, freq, power, max_freq = ULONG_MAX;
	unsigned long opp_eff, prev_opp_eff = ULONG_MAX;
	struct sched_energy_model *em = NULL;
	struct device *cpu_dev;
	struct dev_pm_opp *opp;
	int opp_cnt, i;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("CPU%d: Failed to get device\n", cpu);
		return NULL;
	}

	opp_cnt = dev_pm_opp_get_opp_count(cpu_dev);
	if (opp_cnt <= 0) {
		pr_err("CPU%d: Failed to get # of available OPPs.\n", cpu);
		return NULL;
	}

	opp = dev_pm_opp_find_freq_floor(cpu_dev, &max_freq);
	if (IS_ERR(opp)) {
		pr_err("CPU%d: Failed to get max frequency.\n", cpu);
		return NULL;
	}

	dev_pm_opp_put(opp);
	if (!max_freq) {
		pr_err("CPU%d: Found null max frequency.\n", cpu);
		return NULL;
	}

	em = kzalloc(sizeof(*em), GFP_KERNEL);
	if (!em)
		return NULL;

	em->cap_states = kcalloc(opp_cnt, sizeof(*em->cap_states), GFP_KERNEL);
	if (!em->cap_states)
		goto free_em;

	for (i = 0, freq = 0; i < opp_cnt; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq);
		if (IS_ERR(opp)) {
			pr_err("CPU%d: Failed to get OPP %d.\n", cpu, i+1);
			goto free_cs;
		}

		power = dev_pm_opp_get_power(opp);
		dev_pm_opp_put(opp);
		if (!power || !freq)
			goto free_cs;

		cap = freq * cap_scale / max_freq;
		em->cap_states[i].power = power;
		em->cap_states[i].cap = cap;

		/*
		 * The capacity/watts efficiency ratio should decrease as the
		 * frequency grows on sane platforms. If not, warn the user
		 * that some high OPPs are more power efficient than some
		 * of the lower ones.
		 */
		opp_eff = (cap << 20) / power;
		if (opp_eff >= prev_opp_eff)
			pr_warn("CPU%d: cap/pwr: OPP%d > OPP%d\n", cpu, i, i-1);
		prev_opp_eff = opp_eff;
	}

	em->nr_cap_states = opp_cnt;
	return em;

free_cs:
	kfree(em->cap_states);
free_em:
	kfree(em);
	return NULL;
}

static void free_energy_model(void)
{
	struct sched_energy_model *em;
	struct freq_domain *tmp, *pos;
	int cpu;

	list_for_each_entry_safe(pos, tmp, &sched_freq_domains, next) {
		cpu = cpumask_first(&(pos->span));
		em = *per_cpu_ptr(energy_model, cpu);
		if (em) {
			kfree(em->cap_states);
			kfree(em);
		}

		list_del(&(pos->next));
		kfree(pos);
	}

	free_percpu(energy_model);
}

void init_sched_energy(void)
{
	struct freq_domain *fdom;
	struct sched_energy_model *em;
	struct sched_domain *sd;
	struct device *cpu_dev;
	int cpu, ret, fdom_cpu;

	/* Energy Aware Scheduling is used for asymmetric systems only. */
	rcu_read_lock();
	sd = lowest_flag_domain(smp_processor_id(), SD_ASYM_CPUCAPACITY);
	rcu_read_unlock();
	if (!sd)
		return;

	energy_model = alloc_percpu(struct sched_energy_model *);
	if (!energy_model)
		goto exit_fail;

	for_each_possible_cpu(cpu) {
		if (*per_cpu_ptr(energy_model, cpu))
			continue;

		/* Keep a copy of the sharing_cpus mask */
		fdom = kzalloc(sizeof(struct freq_domain), GFP_KERNEL);
		if (!fdom)
			goto free_em;

		cpu_dev = get_cpu_device(cpu);
		ret = dev_pm_opp_get_sharing_cpus(cpu_dev, &(fdom->span));
		if (ret)
			goto free_em;
		list_add(&(fdom->next), &sched_freq_domains);

		/*
		 * Build the energy model of one CPU, and link it to all CPUs
		 * in its frequency domain. This should be correct as long as
		 * they share the same micro-architecture.
		 */
		fdom_cpu = cpumask_first(&(fdom->span));
		em = build_energy_model(fdom_cpu);
		if (!em)
			goto free_em;

		for_each_cpu(fdom_cpu, &(fdom->span))
			*per_cpu_ptr(energy_model, fdom_cpu) = em;
	}

	static_branch_enable(&sched_energy_present);

	pr_info("Energy Aware Scheduling started.\n");
	return;
free_em:
	free_energy_model();
exit_fail:
	pr_err("Energy Aware Scheduling initialization failed.\n");
}

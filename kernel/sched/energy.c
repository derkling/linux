/*
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 *
 * Energy-aware scheduling models
 *
 * Copyright (C) 2018, Arm Ltd.
 * Written by: Quentin Perret, Arm Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#define pr_fmt(fmt) "sched-energy: " fmt

#include <linux/sched/topology.h>
#include <linux/sched/energy.h>
#include <linux/arch_topology.h>
#include <linux/pm_opp.h>
#include <linux/device.h>

#include "sched.h"

DEFINE_STATIC_KEY_FALSE(sched_energy_present);
struct sched_energy_model __percpu *energy_model;

/*
 * A copy of the cpumasks representing the frequency domains is kept private
 * to the scheduler. They are stacked in a dynamically allocated linked list
 * as we don't know how many frequency domains the system has.
 */
LIST_HEAD(freq_domains);

static ssize_t cpu_energy_model_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, dev);
	struct sched_energy_model *em = per_cpu_ptr(energy_model, cpu->dev.id);
	struct capacity_state *cs;
	ssize_t char_cnt = 0;
	int i;

	for (i=0; i<em->nb_cap_states; i++) {
		cs = &(em->cap_states[i]);
		char_cnt += sprintf(buf + char_cnt, "%lu\t%lu\n", cs->cap, cs->power);
	}

	return char_cnt;
}

static DEVICE_ATTR_RO(cpu_energy_model);

static int build_energy_model(int cpu)
{
	struct sched_energy_model *em = per_cpu_ptr(energy_model, cpu);
	unsigned long cap_scale = arch_scale_cpu_capacity(NULL, cpu);
	unsigned long cap, freq, power, max_freq = ULONG_MAX;
	unsigned long opp_eff, prev_opp_eff = ULONG_MAX;
	struct device *cpu_dev;
	struct dev_pm_opp *opp;
	int opp_cnt, i;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("CPU%d: Failed to get device\n", cpu);
		return -ENODEV;
	}

	opp_cnt = dev_pm_opp_get_opp_count(cpu_dev);
	if (opp_cnt <= 0) {
		pr_err("CPU%d: Failed to get # of available OPPs.\n", cpu);
		return opp_cnt;
	}

	opp = dev_pm_opp_find_freq_floor(cpu_dev, &max_freq);
	if (IS_ERR(opp)) {
		pr_err("CPU%d: Failed to get max frequency.\n", cpu);
		return -EINVAL;
	}
	dev_pm_opp_put(opp);

	em->cap_states = kcalloc(opp_cnt, sizeof(*em->cap_states), GFP_KERNEL);
	if (!em->cap_states)
		return -ENOMEM;

	for (i = 0, freq = 0; i < opp_cnt; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq);
		if (IS_ERR(opp)) {
			pr_err("CPU%d: Failed to get OPP %d.\n", cpu, i+1);
			return -EINVAL;
		}

		power = dev_pm_opp_get_power(opp);
		dev_pm_opp_put(opp);
		if (!power)
			return -EINVAL;

		cap = freq * cap_scale / max_freq;
		em->cap_states[i].power = power;
		em->cap_states[i].cap = cap;

		/*
		 * The capacity/watts efficiency ratio should decrease as the
		 * frequency grows on sane platforms. If not, warn the user
		 * that some high OPPs are more power efficient than some
		 * of the lower ones.
		 */
		opp_eff = (cap << SCHED_CAPACITY_SHIFT) / (power / 1000);
		if (opp_eff >= prev_opp_eff)
			pr_warn("CPU%d: OPP %d inconsistent.", cpu, i+1);
		prev_opp_eff = opp_eff;
	}

	em->nb_cap_states = opp_cnt;
	device_create_file(cpu_dev, &dev_attr_cpu_energy_model);

	return 0;
}

static void free_energy_model(void)
{
	struct freq_domain *tmp, *pos;
	struct device *cpu_dev;
	int cpu;

	for_each_possible_cpu(cpu) {
		kfree(per_cpu_ptr(energy_model, cpu)->cap_states);
		cpu_dev = get_cpu_device(cpu);
		device_remove_file(cpu_dev, &dev_attr_cpu_energy_model);
	}

	free_percpu(energy_model);

	list_for_each_entry_safe(pos, tmp, &freq_domains, next) {
		list_del(&(pos->next));
		kfree(pos);
	}
}

void start_sched_energy(void)
{
	struct freq_domain *tmp;
	struct device *cpu_dev;
	int cpu, ret, fdom_cpu;

	/* Energy Aware Scheduling is used for asymetric systems only */
	if (!lowest_flag_domain(smp_processor_id(), SD_ASYM_CPUCAPACITY))
		return;

	energy_model = alloc_percpu(struct sched_energy_model);
	if (!energy_model) {
		pr_err("Failed to allocate energy model.\n");
		return;
	}

	for_each_possible_cpu(cpu) {
		if (per_cpu_ptr(energy_model, cpu)->cap_states)
			continue;

		/* Keep a copy of the sharing_cpus mask */
		tmp = kzalloc(sizeof(struct freq_domain), GFP_KERNEL);
		if (!tmp) {
			free_energy_model();
			return;
		}
		cpu_dev = get_cpu_device(cpu);
		dev_pm_opp_get_sharing_cpus(cpu_dev, &(tmp->fdom));
		list_add(&(tmp->next), &freq_domains);

		/* Build the energy model for each CPU in this freq. domain */
		for_each_cpu(fdom_cpu, &(tmp->fdom)) {
			ret = build_energy_model(fdom_cpu);
			if (ret) {
				free_energy_model();
				return;
			}
		}
	}

	static_branch_enable(&sched_energy_present);

	pr_info("Energy Aware Scheduling started.\n");
}

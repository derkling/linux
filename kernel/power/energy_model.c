// SPDX-License-Identifier: GPL-2.0
/*
 * Energy Model of CPUs
 *
 * Copyright (c) 2018, Arm ltd.
 * Written by: Quentin Perret, Arm ltd.
 */

#define pr_fmt(fmt) "energy_model: " fmt

#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/energy_model.h>
#include <linux/sched/topology.h>

LIST_HEAD(freq_domain_list);

static ssize_t cpu_energy_model_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, dev);
	struct em_freq_domain *fd;
	struct em_cap_state *cs;
	ssize_t char_cnt = 0;
	int i;

	fd = em_fd_of(cpu->dev.id);
	if (!fd)
		return 0;

	for (i = 0; i < fd->nr_cap_states; i++) {
		cs = &fd->cs_table[i];
		char_cnt += sprintf(buf + char_cnt, "%lu\t%lu\t%lu\n",
						cs->cap, cs->power, cs->freq);
	}

	return char_cnt;
}

static DEVICE_ATTR_RO(cpu_energy_model);

static void fd_show_sysfs(struct em_freq_domain *fd)
{
	int cpu;

	for_each_cpu(cpu, freq_domain_span(fd)) {
		struct device *cpu_dev = get_cpu_device(cpu);
		device_create_file(cpu_dev, &dev_attr_cpu_energy_model);
	}
}

static void fd_update_capacity(struct em_freq_domain *fd)
{
	long fmax = fd->cs_table[fd->nr_cap_states-1].freq;
	long cmax = arch_scale_cpu_capacity(NULL, cpumask_first(&fd->span));
	int i;

	for (i = 0; i < fd->nr_cap_states; i++)
		fd->cs_table[i].cap = cmax * fd->cs_table[i].freq / fmax;
}

/**
 * em_rescale_cpu_capacity() - Re-scale capacity values of the Energy Model
 *
 * This should be used on platforms where the CPU capacities can be updated
 * at run time. This code can update the capacity state tables while they are
 * in use. This can lead to transient inaccuracies in the model.
 */
void em_rescale_cpu_capacity(void)
{
	struct em_freq_domain *fd;

	for_each_freq_domain(fd)
		fd_update_capacity(fd);

	pr_info("Re-scaled CPU capacities\n");
}

/**
 * em_register_freq_domain() - Register the Energy Model of a frequency domain
 * @span	: Mask of CPUs in the frequency domain
 * @nr_states	: Number of capacity states to register
 * @get_power	: Callback returning the power of capacity states
 *
 * Create a capacity state table for a frequency domain, and  extends the
 * em_freq_domain linked list with it.
 *
 * The callback prototype is: long get_power(long *freq, int cpu)
 * It must return the power of the lowest capacity state of "cpu" above "freq"
 * and modify "freq" accordingly.
 */
int em_register_freq_domain(cpumask_t *span, int nr_states,
						long (*get_power)(long*, int))
{
	unsigned long opp_eff, prev_opp_eff = ULONG_MAX;
	struct em_freq_domain *fd;
	int i, ret = -ENOMEM;
	long power, freq;

	if (!span || nr_states <= 0 || !get_power)
		return -EINVAL;

	fd = kzalloc(sizeof(*fd), GFP_KERNEL);
	if (!fd)
		return -ENOMEM;

	fd->cs_table = kcalloc(nr_states, sizeof(*fd->cs_table), GFP_KERNEL);
	if (!fd->cs_table)
		goto free_fd;

	/* Copy the span of the frequency domain */
	cpumask_copy(&fd->span, span);

	/* Build the list of capacity states for this freq domain */
	for (i = 0, freq = 0; i < nr_states; i++, freq++) {
		power = get_power(&freq, cpumask_first(span));
		if (power <= 0 || freq <= 0) {
			ret = -EINVAL;
			goto free_fd;
		}
		fd->cs_table[i].power = power;
		fd->cs_table[i].freq = freq;

		/*
		 * The hertz/watts efficiency ratio should decrease as the
		 * frequency grows on sane platforms. If not, warn the user
		 * that some high OPPs are more power efficient than some
		 * of the lower ones.
		 */
		opp_eff = freq / power;
		if (opp_eff >= prev_opp_eff) {
			pr_warn("%*pbl: hz/watt efficiency: OPP %d >= OPP%d\n",
					cpumask_pr_args(span), i, i-1);
		}
		prev_opp_eff = opp_eff;
	}
	fd->nr_cap_states = nr_states;
	fd_update_capacity(fd);

	list_add(&fd->next, &freq_domain_list);

	pr_info("Added freq domain %*pbl\n", cpumask_pr_args(span));
	fd_show_sysfs(fd);

	return 0;

free_fd:
	kfree(fd->cs_table);
	kfree(fd);
	return ret;
}

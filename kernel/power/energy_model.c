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

struct energy_model *energy_model;

static ssize_t cpu_energy_model_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, dev);
	struct em_cap_state_table *table;
	struct energy_model *em;
	ssize_t char_cnt = 0;
	int i;

	rcu_read_lock();
	em = dereference_energy_model();
	if (!em)
		goto unlock;

	table = *per_cpu_ptr(em->cap_tables, cpu->dev.id);
	if (!table)
		goto unlock;

	for (i = 0; i < table->nr_cap_states; i++) {
		struct em_cap_state *cs = &table->cap_states[i];

		char_cnt += sprintf(buf + char_cnt, "%lu\t%lu\t%lu\n",
				cs->cap, cs->power, cs->freq);
	}

unlock:
	rcu_read_unlock();
	return char_cnt;
}

static DEVICE_ATTR_RO(cpu_energy_model);

static void free_em(struct energy_model *em)
{
	struct em_cap_state_table *cs_tmp;
	struct em_freq_domain *fd_tmp, *pos;
	int cpu;

	if (!em)
		return;

	list_for_each_entry_safe(pos, fd_tmp, &em->freq_domains, next) {
		cpu = cpumask_first(&pos->span);
		cs_tmp = *per_cpu_ptr(em->cap_tables, cpu);
		if (cs_tmp) {
			kfree(cs_tmp->cap_states);
			kfree(cs_tmp);
		}
		list_del(&pos->next);
		kfree(pos);
	}
	free_percpu(em->cap_tables);
	kfree(em);
}

static struct energy_model *create_em(void)
{
	struct energy_model *new_em;

	new_em = kzalloc(sizeof(*new_em), GFP_KERNEL);
	if (!new_em)
		return NULL;

	new_em->cap_tables = alloc_percpu(struct em_cap_state_table *);
	if (!new_em->cap_tables) {
		kfree(new_em);
		return NULL;
	}

	INIT_LIST_HEAD(&new_em->freq_domains);

	return new_em;
}

static struct em_cap_state_table *copy_cs_table(struct em_cap_state_table *old)
{
	struct em_cap_state_table *new = kzalloc(sizeof(*new), GFP_KERNEL);
	size_t cnt;

	if (!new)
		return NULL;
	new->cap_states = kcalloc(old->nr_cap_states,
				  sizeof(*old->cap_states),
				  GFP_KERNEL);
	if (!new->cap_states) {
		kfree(new);
		return NULL;
	}

	cnt = sizeof(*old->cap_states) * old->nr_cap_states;
	memcpy(new->cap_states, old->cap_states,  cnt);
	new->nr_cap_states = old->nr_cap_states;
	return new;
}

static struct energy_model *copy_em(struct energy_model *old_em)
{
	struct em_cap_state_table *cs_tmp;
	struct em_freq_domain *fd_tmp, *pos;
	struct energy_model *new_em;
	int cpu;

	if (!old_em)
		return NULL;

	new_em = create_em();
	if (!new_em)
		return NULL;

	for_each_freq_domain(pos, old_em) {
		/* Copy the frequency domain */
		fd_tmp = kzalloc(sizeof(*fd_tmp), GFP_KERNEL);
		if (!fd_tmp)
			goto free_new_em;
		cpumask_copy(&fd_tmp->span, &pos->span);
		list_add(&fd_tmp->next, &new_em->freq_domains);

		/* Copy the capacity state table */
		cpu = cpumask_first(&fd_tmp->span);
		cs_tmp = copy_cs_table(*per_cpu_ptr(old_em->cap_tables, cpu));
		if (!cs_tmp)
			goto free_new_em;
		for_each_cpu(cpu, &fd_tmp->span)
			*per_cpu_ptr(new_em->cap_tables, cpu) = cs_tmp;
	}

	return new_em;

free_new_em:
	free_em(new_em);
	return NULL;
}

static void cap_state_table_normalize(struct em_cap_state_table *v, int cpu)
{
	long fmax = v->cap_states[v->nr_cap_states-1].freq;
	long cmax = arch_scale_cpu_capacity(NULL, cpu);
	int i;

	for (i = 0; i < v->nr_cap_states; i++)
		v->cap_states[i].cap = cmax * v->cap_states[i].freq / fmax;
}

static DEFINE_MUTEX(em_mutex);

void em_normalize_cpu_capacity(void)
{
	struct energy_model *old_em, *new_em;
	struct em_cap_state_table *table;
	struct em_freq_domain *fd;
	int cpu;

	mutex_lock(&em_mutex);

	/* Copy the old EM */
	old_em = rcu_dereference(energy_model);
	new_em = copy_em(old_em);
	if (!new_em) {
		mutex_unlock(&em_mutex);
		return;
	}

	/* Update the capacity state tables */
	for_each_freq_domain(fd, new_em) {
		cpu = cpumask_first(&fd->span);
		table = *per_cpu_ptr(new_em->cap_tables, cpu);
		cap_state_table_normalize(table, cpu);
	}

	rcu_assign_pointer(energy_model, new_em);

	mutex_unlock(&em_mutex);
	synchronize_rcu();
	free_em(old_em);
	pr_info("Updated CPU capacities\n");
}

/*
 * long fun(long *freq, int cpu)
 *    Returns the power of the lowest OPP >= freq. Modifies freq accordingly.
 */
int em_build_freq_domain(cpumask_t *span, int nr_states, long (*fun)(long*, int))
{
	unsigned long opp_eff, prev_opp_eff = ULONG_MAX;
	struct energy_model *new_em, *old_em;
	struct em_cap_state_table *tmp;
	struct em_freq_domain *fd;
	int i, cpu, ret = -ENOMEM;
	long power, freq;

	if (!span || nr_states <= 0 || !fun)
		return -EINVAL;

	tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	tmp->cap_states = kcalloc(nr_states, sizeof(*tmp->cap_states),
								GFP_KERNEL);
	if (!tmp->cap_states)
		goto free_tmp;

	/* Build the list of capacity states for this freq domain */
	cpu = cpumask_first(span);
	for (i = 0, freq = 0; i < nr_states; i++, freq++) {
		power = fun(&freq, cpu);
		if (power <= 0) {
			ret = power;
			goto free_tmp;
		}
		tmp->cap_states[i].power = power;
		tmp->cap_states[i].freq = freq;

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
	tmp->nr_cap_states = nr_states;
	cap_state_table_normalize(tmp, cpu);

	/* Keep a copy of the span of the frequency domain */
	fd = kzalloc(sizeof(*fd), GFP_KERNEL);
	if (!fd)
		goto free_tmp;
	cpumask_copy(&fd->span, span);

	/* Update the shared energy model structure */
	mutex_lock(&em_mutex);
	old_em = rcu_dereference(energy_model);
	new_em = old_em ? copy_em(old_em) : create_em();
	if (!new_em) {
		mutex_unlock(&em_mutex);
		goto free_fd;
	}

	/* Add the new elements to the model */
	for_each_cpu(cpu, span)
		*per_cpu_ptr(new_em->cap_tables, cpu) = tmp;
	list_add(&fd->next, &new_em->freq_domains);

	rcu_assign_pointer(energy_model, new_em);
	mutex_unlock(&em_mutex);
	synchronize_rcu();
	free_em(old_em);

	pr_info("Added freq domain %*pbl\n", cpumask_pr_args(span));

	for_each_cpu(cpu, span) {
		struct device *cpu_dev = get_cpu_device(cpu);
		device_create_file(cpu_dev, &dev_attr_cpu_energy_model);
	}

	return 0;

free_fd:
	kfree(fd);
free_tmp:
	kfree(tmp->cap_states);
	kfree(tmp);
	return ret;
}

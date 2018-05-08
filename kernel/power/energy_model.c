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

static struct kobject *em_kobject;
static LIST_HEAD(freq_domain_list);
static DEFINE_PER_CPU(struct em_freq_domain *, em_cpu_data);
static DEFINE_RWLOCK(em_lock);

/* Getters for the attributes of em_freq_domain objects */
struct em_fd_attr {
	struct attribute attr;
	ssize_t (*show)(struct em_freq_domain *fd, char *buf);
	ssize_t (*store)(struct em_freq_domain *fd, const char *buf, size_t count);
};

#define show_em_fd_cs_attr(attr) \
static ssize_t show_em_fd_cs_##attr(struct em_freq_domain *fd, char *buf) \
{ \
	ssize_t cnt = 0; \
	int i; \
	for (i = 0; i < fd->nr_cap_states; i++) \
		cnt += sprintf(buf + cnt, "%lu\n", fd->cs_table[i].attr); \
	return cnt; \
}

show_em_fd_cs_attr(power);
show_em_fd_cs_attr(freq);
show_em_fd_cs_attr(cap);

static ssize_t show_em_fd_span(struct em_freq_domain *fd, char *buf) {
	return sprintf(buf, "%*pbl\n", cpumask_pr_args(&fd->span));
}

static struct em_fd_attr em_fd_power_attr = __ATTR(power, 0444, show_em_fd_cs_power, NULL);
static struct em_fd_attr em_fd_freq_attr = __ATTR(frequency, 0444, show_em_fd_cs_freq, NULL);
static struct em_fd_attr em_fd_cap_attr = __ATTR(capacity, 0444, show_em_fd_cs_cap, NULL);
static struct em_fd_attr em_fd_cpus_attr = __ATTR(cpus, 0444, show_em_fd_span, NULL);

static struct attribute *em_fd_default_attrs[] = {
	&em_fd_power_attr.attr,
	&em_fd_freq_attr.attr,
	&em_fd_cap_attr.attr,
	&em_fd_cpus_attr.attr,
	NULL
};

#define to_fd(k) container_of(k, struct em_freq_domain, kobj)
#define to_fd_attr(a) container_of(a, struct em_fd_attr, attr)

static ssize_t show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct em_freq_domain *fd = to_fd(kobj);
	struct em_fd_attr *fd_attr = to_fd_attr(attr);
	ssize_t ret;

	ret = fd_attr->show(fd, buf);

	return ret;
}

static const struct sysfs_ops em_fd_sysfs_ops = {
	.show	= show,
};

static void em_fd_release(struct kobject *kobj)
{
	struct em_freq_domain *fd = to_fd(kobj);

	write_lock_irqsave(&em_lock, flags);
	list_del(&fd->next);
	write_unlock_irqrestore(&em_lock, flags);
	kfree(fd->cs_table);
	kfree(fd);
	pr_info("%s: %*pbl\n", __func__, cpumask_pr_args(&fd->span));
}

static struct kobj_type ktype_em_fd = {
	.sysfs_ops	= &em_fd_sysfs_ops,
	.default_attrs	= em_fd_default_attrs,
	.release	= em_fd_release,
};

static void em_core_init(void)
{
	em_kobject = kobject_create_and_add("energy_model", &cpu_subsys.dev_root->kobj);
	BUG_ON(!em_kobject);
}

static void fd_update_cs_table(struct em_cap_state *table, int nr_cap_states,
									int cpu)
{
	long cmax = arch_scale_cpu_capacity(NULL, cpu);
	long fmax = table[nr_cap_states-1].freq;
	int i;

	for (i = 0; i < nr_cap_states; i++)
		table[i].cap = cmax * table[i].freq / fmax;
}

static struct em_freq_domain *em_create_fd(cpumask_t *span, int nr_states,
						long (*get_power)(long*, int))
{
	unsigned long opp_eff, prev_opp_eff = ULONG_MAX;
	struct em_cap_state *cs_table;
	struct em_freq_domain *fd;
	long power, freq;
	int i;

	if (!span || nr_states <= 0 || !get_power)
		return NULL;

	fd = kzalloc(sizeof(*fd), GFP_KERNEL);
	if (!fd)
		return fd;

	cs_table = kcalloc(nr_states, sizeof(*fd->cs_table), GFP_KERNEL);
	if (!cs_table)
		goto free_fd;


	/* Copy the span of the frequency domain */
	cpumask_copy(&fd->span, span);

	/* Build the list of capacity states for this freq domain */
	for (i = 0, freq = 0; i < nr_states; i++, freq++) {
		power = get_power(&freq, cpumask_first(span));
		if (power <= 0 || freq <= 0) {
			kfree(cs_table);
			goto free_fd;
		}

		cs_table[i].power = power;
		cs_table[i].freq = freq;

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

	fd_update_cs_table(cs_table, nr_states, cpumask_first(span));
	rcu_assign_pointer(fd->cs_table, cs_table);
	fd->nr_cap_states = nr_states;

	return fd;

free_fd:
	kfree(fd);

	return NULL;
}

/**
 * em_rescale_cpu_capacity() - Re-scale capacity values of the Energy Model
 */
void em_rescale_cpu_capacity(void)
{
	struct em_cap_state *old_table, *new_table;
	struct em_freq_domain *fd;
	int nr_states, cpu;

	list_for_each_entry(fd, &freq_domain_list, next) {
		old_table = rcu_dereference(fd->cs_table);
		if (!old_table)
			continue;

		nr_states = fd->nr_cap_states;
		new_table = kcalloc(nr_states, sizeof(*new_table), GFP_KERNEL);
		if (!new_table)
			return;

		memcpy(new_table, old_table, nr_states * sizeof(*new_table));
		cpu = cpumask_first(&fd->span);
		fd_update_cs_table(new_table, nr_states, cpu);

		rcu_assign_pointer(fd->cs_table, new_table);
		synchronize_rcu();
		kfree(old_table);
	}

	pr_info("Re-scaled CPU capacities\n");
}

static struct em_freq_domain *em_fd_of_raw(int cpu)
{
	struct em_freq_domain *fd;

	if (list_empty(&freq_domain_list))
		return NULL;

	list_for_each_entry(fd, &freq_domain_list, next) {
		if (cpumask_test_cpu(cpu, &fd->span))
			return fd;
	}

	return NULL;
}

struct em_freq_domain *em_fd_of(int cpu)
{
	struct em_freq_domain *fd;
	unsigned long flags;

	read_lock_irqsave(&em_lock, flags);
	fd = em_fd_of_raw(cpu);
	if (fd)
		kobject_get(&fd->kobj);
	read_unlock_irqrestore(&em_lock, flags);

	return fd;
}

void em_fd_put(struct em_freq_domain *fd)
{
	kobject_put(&fd->kobj);
}

static int cpuhp_em_online(unsigned int cpu)
{
	struct em_freq_domain *fd;
	unsigned long flags;

	read_lock_irqsave(&em_lock, flags);
	fd = em_fd_of_raw(cpu);
	per_cpu(em_cpu_data, cpu) = fd;
	read_unlock_irqrestore(&em_lock, flags);

	return 0;
}

static int cpuhp_em_offline(unsigned int cpu)
{
	struct em_freq_domain *fd;
	unsigned long flags;

	fd = per_cpu(em_cpu_data, cpu);
	if (fd) {
		write_lock_irqsave(&em_lock, flags);
		//pr_info("CPU%d offline\n", cpu); /* XXX: debug */
		per_cpu(em_cpu_data, cpu) = NULL;

		/* Check if "cpu" is the last in the freq domain. */
		for_each_cpu(cpu, &fd->span) {
			if (per_cpu(em_cpu_data, cpu)) {
				write_unlock_irqrestore(&em_lock, flags);
				return 0;
			}
		}
		write_unlock_irqrestore(&em_lock, flags);
		kobject_put(&fd->kobj);
		pr_info("No CPU online in %*pbl, release kobj\n", cpumask_pr_args(&fd->span)); /* XXX: debug */
	}

	return 0;
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
	struct em_freq_domain *fd;
	unsigned long flags;
	bool new_fd = true;
	int ret, cpu;

	/* Make sure we don't register again an existing freq domain. */
	list_for_each_entry(fd, &freq_domain_list, next) {
		if (!cpumask_equal(&fd->span, span))
			continue;
		new_fd = false;

		/* Take a kobj reference if we don't have one */
		for_each_cpu(cpu, span) {
			if (per_cpu(em_cpu_data, cpu))
				goto add_fd;
		}

		kobject_get(&fd->kobj);
		goto add_fd;
	}

	fd = em_create_fd(span, nr_states, get_power);
	if (!fd)
		return -EINVAL;

	if (!em_kobject)
		em_core_init();

	ret = kobject_init_and_add(&fd->kobj, &ktype_em_fd, em_kobject,
				"fd%u", cpumask_first(&fd->span));
	if (ret) {
		pr_err("%s: Failed to init fd->kobj: %d\n", __func__, ret);
		goto free_fd;
	}

	ret = cpuhp_setup_state_nocalls_cpuslocked(CPUHP_AP_ONLINE_DYN,
						   "energy_model:online",
						   cpuhp_em_online,
						   cpuhp_em_offline);
	if (ret < 0) {
		pr_err("%s: Failed cpuhp registration: %d\n", __func__, ret);
		kobject_put(&fd->kobj);
		goto free_fd;
	}

	pr_info("Created freq domain %*pbl\n", cpumask_pr_args(span));

add_fd:
	write_lock_irqsave(&em_lock, flags);
	for_each_cpu_and(cpu, span, cpu_online_mask)
		per_cpu(em_cpu_data, cpu) = fd;

	if (new_fd)
		list_add(&fd->next, &freq_domain_list);
	write_unlock_irqrestore(&em_lock, flags);

	return 0;

free_fd:
	kfree(fd->cs_table);
	kfree(fd);

	return ret;
}

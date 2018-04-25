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
	ssize_t (*store)(struct em_freq_domain *fd, const char *buf, size_t s);
};

#define EM_ATTR_LEN 13
#define show_table_attr(_attr) \
static ssize_t show_##_attr(struct em_freq_domain *fd, char *buf) \
{ \
	ssize_t cnt = 0; \
	int i; \
	for (i = 0; i < fd->nr_cap_states; i++) { \
		if (cnt >= (ssize_t) (PAGE_SIZE / sizeof(char) \
				      - (EM_ATTR_LEN + 2))) \
			goto out; \
		cnt += scnprintf(&buf[cnt], EM_ATTR_LEN + 1, "%lu ", \
				 fd->cs_table[i]._attr); \
	} \
out: \
	cnt += sprintf(&buf[cnt], "\n"); \
	return cnt; \
}

show_table_attr(power);
show_table_attr(frequency);
show_table_attr(capacity);

static ssize_t show_cpus(struct em_freq_domain *fd, char *buf)
{
	return sprintf(buf, "%*pbl\n", cpumask_pr_args(&fd->cpus));
}

#define fd_attr(_name) em_fd_##_name##_attr
#define define_fd_attr(_name) static struct em_fd_attr fd_attr(_name) = \
		__ATTR(_name, 0444, show_##_name, NULL)

define_fd_attr(power);
define_fd_attr(frequency);
define_fd_attr(capacity);
define_fd_attr(cpus);

static struct attribute *em_fd_default_attrs[] = {
	&fd_attr(power).attr,
	&fd_attr(frequency).attr,
	&fd_attr(capacity).attr,
	&fd_attr(cpus).attr,
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
	unsigned long flags;

	write_lock_irqsave(&em_lock, flags);
	list_del(&fd->next);
	write_unlock_irqrestore(&em_lock, flags);
	kfree(fd->cs_table);
	kfree(fd);
	pr_debug("%s: %*pbl\n", __func__, cpumask_pr_args(&fd->cpus));
}

static struct kobj_type ktype_em_fd = {
	.sysfs_ops	= &em_fd_sysfs_ops,
	.default_attrs	= em_fd_default_attrs,
	.release	= em_fd_release,
};

static void fd_update_cs_table(struct em_cap_state *table, int nr_cap_states,
									int cpu)
{
	unsigned long cmax = arch_scale_cpu_capacity(NULL, cpu);
	unsigned long fmax = table[nr_cap_states-1].frequency;
	int i;

	for (i = 0; i < nr_cap_states; i++)
		table[i].capacity = cmax * table[i].frequency / fmax;
}

static struct em_freq_domain *em_create_fd(cpumask_t *span, int nr_states,
			int (*get_power)(unsigned long*, unsigned long*, int))
{
	unsigned long opp_eff, prev_opp_eff = ULONG_MAX;
	int i, ret, cpu = cpumask_first(span);
	struct em_cap_state *cs_table;
	struct em_freq_domain *fd;
	unsigned long power, freq;

	fd = kzalloc(sizeof(*fd), GFP_NOWAIT);
	if (!fd)
		return fd;

	cs_table = kcalloc(nr_states, sizeof(*fd->cs_table), GFP_NOWAIT);
	if (!cs_table)
		goto free_fd;


	/* Copy the span of the frequency domain */
	cpumask_copy(&fd->cpus, span);

	/* Build the list of capacity states for this freq domain */
	for (i = 0, freq = 0; i < nr_states; i++, freq++) {
		ret = get_power(&power, &freq, cpu);
		if (ret) {
			kfree(cs_table);
			goto free_fd;
		}

		cs_table[i].power = power;
		cs_table[i].frequency = freq;

		/*
		 * The hertz/watts efficiency ratio should decrease as the
		 * frequency grows on sane platforms. If not, warn the user
		 * that some high OPPs are more power efficient than some
		 * of the lower ones.
		 */
		opp_eff = freq / power;
		if (opp_eff >= prev_opp_eff)
			pr_warn("%*pbl: hz/watt efficiency: OPP %d >= OPP%d\n",
					cpumask_pr_args(span), i, i - 1);
		prev_opp_eff = opp_eff;
	}

	fd_update_cs_table(cs_table, nr_states, cpu);
	rcu_assign_pointer(fd->cs_table, cs_table);
	fd->nr_cap_states = nr_states;

	ret = kobject_init_and_add(&fd->kobj, &ktype_em_fd, em_kobject, "fd%u",
									cpu);
	if (ret) {
		kfree(fd->cs_table);
		goto free_fd;
	}

	return fd;

free_fd:
	kfree(fd);

	return NULL;
}

/**
 * em_rescale_cpu_capacity() - Re-scale capacity values of the Energy Model
 *
 * This re-scales the capacity values for all capacity states of all frequency
 * domains of the Energy Model. This should be used when the capacity values
 * of the CPUs are updated at run-time, after the EM was registered.
 *
 * The update occurs even for frequency domains currently marked as busy by
 * readers, thanks to RCU protection on the capacity state table.
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
		new_table = kcalloc(nr_states, sizeof(*new_table), GFP_NOWAIT);
		if (!new_table)
			return;

		memcpy(new_table, old_table, nr_states * sizeof(*new_table));
		cpu = cpumask_first(&fd->cpus);
		fd_update_cs_table(new_table, nr_states, cpu);

		rcu_assign_pointer(fd->cs_table, new_table);
		synchronize_rcu();
		kfree(old_table);
	}

	pr_debug("Re-scaled CPU capacities\n");
}
EXPORT_SYMBOL_GPL(em_rescale_cpu_capacity);

static struct em_freq_domain *em_cpu_get_raw(int cpu)
{
	struct em_freq_domain *fd;

	if (list_empty(&freq_domain_list))
		return NULL;

	list_for_each_entry(fd, &freq_domain_list, next) {
		if (cpumask_test_cpu(cpu, &fd->cpus))
			return fd;
	}

	return NULL;
}

/**
 * em_cpu_get() - Return the frequency domain for a CPU and mark it busy
 * @cpu : CPU to find the frequency domain for
 *
 * This returns the frequency domain to which 'cpu' belongs, or NULL if it
 * doesn't exist. It also increments the kobject reference count to mark it
 * busy. A corresponding call to em_cpu_put() is then required to decrement it
 * back and release the reference. If no such call to em_cpu_put() is made,
 * the frequency domain will never be freed.
 *
 * Holding the kobject reference guarantees to callers that the frequency
 * domain object is accessible, but the actual content of the capacity state
 * table (which is RCU-protected) can be updated asynchronously at run-time.
 */
struct em_freq_domain *em_cpu_get(int cpu)
{
	struct em_freq_domain *fd;
	unsigned long flags;

	read_lock_irqsave(&em_lock, flags);
	fd = em_cpu_get_raw(cpu);
	if (fd)
		kobject_get(&fd->kobj);
	read_unlock_irqrestore(&em_lock, flags);

	return fd;
}
EXPORT_SYMBOL_GPL(em_cpu_get);

/**
 * em_cpu_put() - Decrement the usage count of a frequency domain
 * @fd: frequency domain earlier returned by em_cpu_get()
 *
 * This decrements the kobject reference count incremented earlier by calling
 * em_cpu_get().
 */
void em_cpu_put(struct em_freq_domain *fd)
{
	kobject_put(&fd->kobj);
}
EXPORT_SYMBOL_GPL(em_cpu_put);

static int cpuhp_em_online(unsigned int cpu)
{
	struct em_freq_domain *fd;
	unsigned long flags;
	int fd_cpu;

	write_lock_irqsave(&em_lock, flags);
	fd = em_cpu_get_raw(cpu);
	if (!fd)
		goto unlock;

	/* Take the reference iff "cpu" is the first in the freq. domain. */
	for_each_cpu(fd_cpu, &fd->cpus) {
		if (per_cpu(em_cpu_data, fd_cpu))
			goto unlock;
	}
	kobject_get(&fd->kobj);

unlock:
	per_cpu(em_cpu_data, cpu) = fd;
	write_unlock_irqrestore(&em_lock, flags);

	return 0;
}

static int cpuhp_em_offline(unsigned int cpu)
{
	struct em_freq_domain *fd;
	unsigned long flags;
	int fd_cpu;

	write_lock_irqsave(&em_lock, flags);
	fd = per_cpu(em_cpu_data, cpu);
	if (!fd)
		goto unlock;

	per_cpu(em_cpu_data, cpu) = NULL;

	/* Release the reference iff "cpu" is the last in the freq domain. */
	for_each_cpu(fd_cpu, &fd->cpus) {
		if (per_cpu(em_cpu_data, fd_cpu))
			goto unlock;
	}

	kobject_put(&fd->kobj);
unlock:
	write_unlock_irqrestore(&em_lock, flags);

	return 0;
}

static int em_core_init(void)
{
	int ret;

	em_kobject = kobject_create_and_add("energy_model",
						&cpu_subsys.dev_root->kobj);
	if (!em_kobject)
		return -ENODEV;

	ret = cpuhp_setup_state_nocalls_cpuslocked(CPUHP_AP_ONLINE_DYN,
						   "energy_model:online",
						   cpuhp_em_online,
						   cpuhp_em_offline);
	if (ret < 0) {
		pr_err("%s: Failed cpuhp registration: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

/**
 * em_register_freq_domain() - Register the Energy Model of a frequency domain
 * @span	: Mask of CPUs in the frequency domain
 * @nr_states	: Number of capacity states to register
 * @get_power	: Callback providing the power of capacity states
 *
 * Create a capacity state table for a frequency domain using the get_power()
 * callback provided by the caller. The get_power() callback has the following
 * signature:
 *
 *       int get_power(unsigned long *power, unsigned long *freq, int cpu)
 *
 * get_power() must find the lowest capacity state of 'cpu' above 'freq'
 * and update 'power' and 'freq' to the matching power and frequency values
 * of that capacity state. get_power() should return 0 on success or non-zero
 * values otherwise.
 *
 * If multiple clients register the same frequency domain, all but the first
 * registration will be ignored.
 */
int em_register_freq_domain(cpumask_t *span, int nr_states,
			int (*get_power)(unsigned long*, unsigned long*, int))
{
	struct em_freq_domain *fd;
	unsigned long flags;
	int cpu, ret = 0;

	if (!span || nr_states <= 0 || !get_power)
		return -EINVAL;

	write_lock_irqsave(&em_lock, flags);

	/* Create the core objects the first time this function is called. */
	if (!em_kobject) {
		ret = em_core_init();
		if (ret)
			goto unlock;
	}

	/* Make sure we don't register again an existing domain. */
	fd = em_cpu_get_raw(cpumask_first(span));
	if (fd) {
		ret = -EEXIST;
		goto unlock;
	}

	/* Create the frequency domain & associated kobject. */
	fd = em_create_fd(span, nr_states, get_power);
	if (!fd) {
		ret = -EINVAL;
		goto unlock;
	}

	list_add(&fd->next, &freq_domain_list);
	for_each_cpu_and(cpu, span, cpu_online_mask)
		per_cpu(em_cpu_data, cpu) = fd;

	pr_debug("Created freq domain %*pbl\n", cpumask_pr_args(span));

unlock:
	write_unlock_irqrestore(&em_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(em_register_freq_domain);

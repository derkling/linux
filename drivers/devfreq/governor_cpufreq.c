/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "dev-cpufreq: " fmt

#include <linux/devfreq.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/cpu_pm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "governor.h"

#include <trace/events/dsu_dvfs.h>

#define DEFAULT_POLLING_MS		10

struct cpu_state {
	unsigned int freq;
	unsigned int min_freq;
	unsigned int max_freq;
	unsigned int first_cpu;
};

struct cpu_status {
	bool on;
	struct cpu_state *state;
};

static struct cpu_status status[NR_CPUS];
static int cpufreq_cnt;

struct freq_map {
	unsigned int cpu_khz;
	unsigned int target_freq;
};

struct devfreq_node {
	struct devfreq *df;
	void *orig_data;
	struct device *dev;
	struct device_node *of_node;
	struct list_head list;
	struct freq_map **map;
	struct freq_map *common_map;
	unsigned int timeout;
	struct delayed_work dwork;
	bool drop;
	unsigned long prev_tgt;
	struct workqueue_struct *update_wq;
	struct work_struct update_handle;
	struct hrtimer poll_timer;
	u32 polling_ms;
};
static LIST_HEAD(devfreq_list);
static DEFINE_MUTEX(state_lock);
static DEFINE_MUTEX(cpufreq_reg_lock);

#define show_attr(name) \
static ssize_t show_##name(struct device *dev,				\
			struct device_attribute *attr, char *buf)	\
{									\
	struct devfreq *df = to_devfreq(dev);				\
	struct devfreq_node *n = df->data;				\
	return snprintf(buf, PAGE_SIZE, "%u\n", n->name);		\
}

#define store_attr(name, _min, _max) \
static ssize_t store_##name(struct device *dev,				\
			struct device_attribute *attr, const char *buf,	\
			size_t count)					\
{									\
	struct devfreq *df = to_devfreq(dev);				\
	struct devfreq_node *n = df->data;				\
	int ret;							\
	unsigned int val;						\
	ret = sscanf(buf, "%u", &val);					\
	if (ret != 1)							\
		return -EINVAL;						\
	val = max(val, _min);						\
	val = min(val, _max);						\
	n->name = val;							\
	return count;							\
}

#define gov_attr(__attr, min, max)	\
show_attr(__attr)			\
store_attr(__attr, min, max)		\
static DEVICE_ATTR(__attr, 0644, show_##__attr, store_##__attr)

static int update_node(struct devfreq_node *node)
{
	int ret;
	struct devfreq *df = node->df;

	if (!df)
		return 0;

	cancel_delayed_work_sync(&node->dwork);

	mutex_lock(&df->lock);
	node->drop = false;
	ret = update_devfreq(df);
	if (ret) {
		dev_err(df->dev.parent, "Unable to update frequency\n");
		goto out;
	}

	if (!node->timeout)
		goto out;

	if (df->previous_freq <= df->min_freq)
		goto out;

	schedule_delayed_work(&node->dwork,
			      msecs_to_jiffies(node->timeout));
out:
	mutex_unlock(&df->lock);
	return ret;
}

/*
* static void update_all_devfreqs(void)
* {
*	struct devfreq_node *node;
*
*	list_for_each_entry(node, &devfreq_list, list) {
*		if (node->update_wq)
*			queue_work(node->update_wq, &node->update_handle);
*		else
*			update_node(node);
*	}
* }
*/

static void devfreq_node_handle_update(struct work_struct *work)
{
	struct devfreq_node *node = container_of(work, struct devfreq_node,
					     update_handle);
	update_node(node);
}

static enum hrtimer_restart devfreq_node_polling(struct hrtimer *hrtimer)
{
	struct devfreq_node *node = container_of(hrtimer, struct devfreq_node,
					     poll_timer);

	queue_work(node->update_wq, &node->update_handle);

	hrtimer_forward_now(&node->poll_timer,
			    ms_to_ktime(node->polling_ms));
	return HRTIMER_RESTART;
}

static void do_timeout(struct work_struct *work)
{
	struct devfreq_node *node = container_of(to_delayed_work(work),
						struct devfreq_node, dwork);
	struct devfreq *df = node->df;

	mutex_lock(&df->lock);
	node->drop = true;
	update_devfreq(df);
	mutex_unlock(&df->lock);
}

static struct devfreq_node *find_devfreq_node(struct device *dev)
{
	struct devfreq_node *node;

	list_for_each_entry(node, &devfreq_list, list)
		if (node->dev == dev || node->of_node == dev->of_node)
			return node;

	return NULL;
}

static void add_policy(struct cpufreq_policy *policy)
{
	struct cpu_state *new_state;
	struct cpu_state *state = status[policy->cpu].state;
	unsigned int cpu, first_cpu;

	mutex_lock(&state_lock);

	if (state) {
		new_state = state;
	} else {
		new_state = kzalloc(sizeof(struct cpu_state), GFP_KERNEL);
		if (!new_state)
			goto out;
	}

	first_cpu = cpumask_first(policy->related_cpus);
	new_state->first_cpu = first_cpu;
	new_state->freq = policy->cur;
	new_state->min_freq = policy->cpuinfo.min_freq;
	new_state->max_freq = policy->cpuinfo.max_freq;

	for_each_cpu(cpu, policy->related_cpus) {
		status[cpu].state = new_state;
		status[cpu].on = true;
	}

out:
	mutex_unlock(&state_lock);
}

static void remove_policy(struct cpufreq_policy *policy)
{
	unsigned int cpu, first_cpu;

	mutex_lock(&state_lock);

	first_cpu = cpumask_first(policy->related_cpus);
	if (status[first_cpu].state)
		kfree(status[first_cpu].state);

	for_each_cpu(cpu, policy->related_cpus) {
		status[first_cpu].state = NULL;
		status[cpu].on = false;
	}

	mutex_unlock(&state_lock);
}

static void enable_cpu(unsigned int cpu) {
	status[cpu].on = true;
}

static void disable_cpu(unsigned int cpu) {
	status[cpu].on = false;
}

#ifdef CONFIG_HOTPLUG_CPU
static int hotplug_notifier(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	unsigned int cpu = (long)data;

	/* We are only interested in these two cases */
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_UP_PREPARE:
		enable_cpu(cpu);
		break;
	case CPU_DOWN_PREPARE:
		disable_cpu(cpu);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_hotplug_nb = {
	.notifier_call = hotplug_notifier,
};

static int register_cpu_hotplug(void)
{
	return register_hotcpu_notifier(&cpu_hotplug_nb);
}

static void unregister_cpu_hotplug(void)
{
	unregister_hotcpu_notifier(&cpu_hotplug_nb);
}

#else /* ! CONFIG_HOTPLUG_CPU */
static int register_cpu_hotplug(void)
{
	return -ENODEV;
}

static int unregister_cpu_hotplug(void)
{
	return 0;
}

#endif /* CONFIG_HOTPLUG_CPU */


#ifdef CONFIG_CPU_PM
static int cpu_pm_notifier(struct notifier_block *nb, unsigned long action,
			       void *data)
{
	unsigned int cpu = smp_processor_id();

	switch (action) {
	case CPU_PM_ENTER:
		disable_cpu(cpu);
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		enable_cpu(cpu);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_pm_nb = {
	.notifier_call = cpu_pm_notifier,
};

static int register_cpu_pm(void)
{
	return cpu_pm_register_notifier(&cpu_pm_nb);
}

static void unregister_cpu_pm(void)
{
	cpu_pm_unregister_notifier(&cpu_pm_nb);
}

#else /* ! CONFIG_CPU_PM */
static int register_cpu_pm(void)
{
	return -ENODEV;
}

static int unregister_cpu_pm(void)
{
	return 0;
}

#endif /* CONFIG_CPU_PM */

static int cpufreq_policy_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	switch (event) {
	case CPUFREQ_CREATE_POLICY:
		add_policy(policy);
		break;

	case CPUFREQ_REMOVE_POLICY:
		remove_policy(policy);
		break;
	}

	return 0;
}

static struct notifier_block cpufreq_policy_nb = {
	.notifier_call = cpufreq_policy_notifier
};

static int cpufreq_trans_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_state *state;

	if (event != CPUFREQ_POSTCHANGE)
		return 0;

	mutex_lock(&state_lock);

	state = status[freq->cpu].state;
	if (!state)
		goto out;

	if (state->freq != freq->new) {
		state->freq = freq->new;
	}

out:
	mutex_unlock(&state_lock);
	return 0;
}

static struct notifier_block cpufreq_trans_nb = {
	.notifier_call = cpufreq_trans_notifier
};

static int register_cpufreq(void)
{
	int ret = 0;
	unsigned int cpu;
	struct cpufreq_policy *policy;

	mutex_lock(&cpufreq_reg_lock);

	if (cpufreq_cnt)
		goto cnt_not_zero;

	get_online_cpus();
	ret = cpufreq_register_notifier(&cpufreq_policy_nb,
				CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		goto out;

	ret = cpufreq_register_notifier(&cpufreq_trans_nb,
				CPUFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		cpufreq_unregister_notifier(&cpufreq_policy_nb,
				CPUFREQ_POLICY_NOTIFIER);
		goto out;
	}

	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (policy) {
			add_policy(policy);
			cpufreq_cpu_put(policy);
		}
	}
out:
	put_online_cpus();
cnt_not_zero:
	if (!ret)
		cpufreq_cnt++;
	mutex_unlock(&cpufreq_reg_lock);
	return ret;
}

static int unregister_cpufreq(void)
{
	int ret = 0;
	int cpu;
	struct cpu_state *state;

	mutex_lock(&cpufreq_reg_lock);

	if (cpufreq_cnt > 1)
		goto out;

	cpufreq_unregister_notifier(&cpufreq_policy_nb,
				CPUFREQ_POLICY_NOTIFIER);
	cpufreq_unregister_notifier(&cpufreq_trans_nb,
				CPUFREQ_TRANSITION_NOTIFIER);

	for (cpu = ARRAY_SIZE(status) - 1; cpu >= 0; cpu--) {
		state = status[cpu].state;
		if (!state)
			continue;
		if (state->first_cpu == cpu)
			kfree(state);
		state = NULL;
	}

out:
	cpufreq_cnt--;
	mutex_unlock(&cpufreq_reg_lock);
	return ret;
}

static unsigned int interpolate_freq(struct devfreq *df, unsigned int cpu)
{
	unsigned int *freq_table = df->profile->freq_table;
	struct cpu_state *state = status[cpu].state;
	unsigned int cpu_min = state->min_freq;
	unsigned int cpu_max = state->max_freq;
	unsigned int cpu_freq = state->freq;
	unsigned int dev_min, dev_max, cpu_percent;

	if (freq_table) {
		dev_min = freq_table[0];
		dev_max = freq_table[df->profile->max_state - 1];
	} else {
		if (df->max_freq <= df->min_freq)
			return 0;
		dev_min = df->min_freq;
		dev_max = df->max_freq;
	}

	cpu_percent = ((cpu_freq - cpu_min) * 100) / (cpu_max - cpu_min);
	return dev_min + mult_frac(dev_max - dev_min, cpu_percent, 100);
}

static unsigned int cpu_to_dev_freq(struct devfreq *df, unsigned int cpu)
{
	struct freq_map *map = NULL;
	unsigned int cpu_khz = 0, freq;
	struct devfreq_node *n = df->data;
	struct cpu_state *state = status[cpu].state;

	if (!state || !status[cpu].on) {
		freq = 0;
		goto out;
	}

	if (n->common_map)
		map = n->common_map;
	else if (n->map)
		map = n->map[cpu];

	if (!map)
		map = n->map[state->first_cpu];

	if (!map) {
		freq = interpolate_freq(df, cpu);
		goto out;
	}

	cpu_khz = state->freq;

	while (map->cpu_khz && map->cpu_khz < cpu_khz)
		map++;
	if (!map->cpu_khz)
		map--;
	freq = map->target_freq;

out:
	trace_dsu_dvfs_gov_status_each(cpu, cpu_khz, freq);
	return freq;
}

static int devfreq_cpufreq_get_freq(struct devfreq *df,
					unsigned long *freq)
{
	unsigned int cpu, tgt_freq = 0;
	struct devfreq_node *node;

	node = df->data;
	if (!node) {
		pr_err("Unable to find devfreq node!\n");
		return -ENODEV;
	}

	if (node->drop) {
		*freq = 0;
		return 0;
	}

	for_each_possible_cpu(cpu)
		tgt_freq = max(tgt_freq, cpu_to_dev_freq(df, cpu));

	if (node->timeout && tgt_freq < node->prev_tgt)
		*freq = 0;
	else
		*freq = tgt_freq;

	node->prev_tgt = tgt_freq;

	trace_dsu_dvfs_gov_status(*freq);

	return 0;
}

static unsigned int show_table(char *buf, unsigned int len,
				struct freq_map *map)
{
	unsigned int cnt = 0;

	cnt += snprintf(buf + cnt, len - cnt, "CPU freq\tDevice freq\n");

	while (map->cpu_khz && cnt < len) {
		cnt += snprintf(buf + cnt, len - cnt, "%8u\t%11u\n",
				map->cpu_khz, map->target_freq);
		map++;
	}
	if (cnt < len)
		cnt += snprintf(buf + cnt, len - cnt, "\n");

	return cnt;
}

static ssize_t show_map(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct devfreq *df = to_devfreq(dev);
	struct devfreq_node *n = df->data;
	struct freq_map *map;
	unsigned int cnt = 0, cpu;

	mutex_lock(&state_lock);
	if (n->common_map) {
		map = n->common_map;
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"Common table for all CPUs:\n");
		cnt += show_table(buf + cnt, PAGE_SIZE - cnt, map);
	} else if (n->map) {
		for_each_possible_cpu(cpu) {
			map = n->map[cpu];
			if (!map)
				continue;
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
					"CPU %u:\n", cpu);
			if (cnt >= PAGE_SIZE)
				break;
			cnt += show_table(buf + cnt, PAGE_SIZE - cnt, map);
			if (cnt >= PAGE_SIZE)
				break;
		}
	} else {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"Device freq interpolated based on CPU freq\n");
	}
	mutex_unlock(&state_lock);

	return cnt;
}

static ssize_t show_devfreq_state(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{

	unsigned int cpu, cnt = 0;
	unsigned int len;
	struct cpu_state *state;

	len = PAGE_SIZE;
	cnt += snprintf(buf + cnt, len - cnt,
			"CPU\tFreq\tMin\tMax\tFirst\tOn\n");

	mutex_lock(&state_lock);
	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		state = status[cpu].state;
		if (state)
			cnt += snprintf(buf + cnt, len - cnt,
					"%u\t%u\t%u\t%u\t%d\t%u\n",
					cpu, state->freq, state->min_freq,
					state->max_freq, state->first_cpu,
					status[cpu].on
			);
	}
	mutex_unlock(&state_lock);

	if (cnt < len)
		cnt += snprintf(buf + cnt, len - cnt, "\n");

	return cnt;
}


static DEVICE_ATTR(freq_map, 0444, show_map, NULL);
static DEVICE_ATTR(state, 0444, show_devfreq_state, NULL);
gov_attr(timeout, 0U, 100U);

static struct attribute *dev_attr[] = {
	&dev_attr_freq_map.attr,
	&dev_attr_timeout.attr,
	&dev_attr_state.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.name = "cpufreq",
	.attrs = dev_attr,
};

static int devfreq_cpufreq_gov_start(struct devfreq *devfreq)
{
	int ret = 0;
	struct devfreq_node *node;
	bool alloc = false;
	struct cpufreq_policy *policy;

	ret = sysfs_create_group(&devfreq->dev.kobj, &dev_attr_group);
	if (ret)
		return ret;

	mutex_lock(&state_lock);

	node = find_devfreq_node(devfreq->dev.parent);
	if (node == NULL) {
		node = kzalloc(sizeof(struct devfreq_node), GFP_KERNEL);
		if (!node) {
			pr_err("Out of memory!\n");
			ret = -ENOMEM;
			goto alloc_fail;
		}
		alloc = true;
		node->dev = devfreq->dev.parent;
		list_add_tail(&node->list, &devfreq_list);
	}

	INIT_DELAYED_WORK(&node->dwork, do_timeout);

	node->df = devfreq;
	node->orig_data = devfreq->data;
	devfreq->data = node;

	ret = update_node(node);
	if (ret)
		goto update_fail;

	mutex_unlock(&state_lock);

	ret = register_cpufreq();
	if (ret) {
		pr_warn("Unable to register cpufreq notifications.\n");
		goto update_fail;
	}

	ret = register_cpu_pm();
	if (ret) {
		pr_warn("Unable to register CPU PM notifications.\n");
		goto cpu_pm_fail;
	}

	ret = register_cpu_hotplug();
	if (ret) {
		pr_warn("Unable to register hotplug notifications.\n");
		goto hotplug_fail;
	}

	node->polling_ms = DEFAULT_POLLING_MS;
	node->update_wq = alloc_workqueue("devfreq_cpufreq_wq", WQ_UNBOUND, 0);
	if (IS_ERR(node->update_wq)) {
		pr_err("Cannot create workqueue.\n");
		/* TODO: properly handle error */
		return PTR_ERR(node->update_wq);
	}
	/* TODO: is there a better way */
	policy = cpufreq_cpu_get(0);
	if (policy) {
		if (workqueue_set_unbound_cpumask(policy->related_cpus)) {
			pr_err("Error to bound to LITTLE CPUs\n");
		}
		cpufreq_cpu_put(policy);
	}

	INIT_WORK(&node->update_handle, devfreq_node_handle_update);

	hrtimer_init(&node->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	node->poll_timer.function = devfreq_node_polling;
	hrtimer_start(&node->poll_timer, ms_to_ktime(node->polling_ms),
		      HRTIMER_MODE_REL);

	return 0;

hotplug_fail:
	unregister_cpu_pm();
cpu_pm_fail:
	unregister_cpufreq();
update_fail:
	devfreq->data = node->orig_data;
	if (alloc) {
		list_del(&node->list);
		kfree(node);
	}
alloc_fail:
	mutex_unlock(&state_lock);
	sysfs_remove_group(&devfreq->dev.kobj, &dev_attr_group);
	return ret;
}

static void devfreq_cpufreq_gov_stop(struct devfreq *devfreq)
{
	struct devfreq_node *node = devfreq->data;

	cancel_delayed_work_sync(&node->dwork);

	/* Cancel hrtimer */
	if (hrtimer_active(&node->poll_timer))
		hrtimer_cancel(&node->poll_timer);
	/* Wait for pending work */
	flush_workqueue(node->update_wq);
	/* Destroy workqueue */
	destroy_workqueue(node->update_wq);

	mutex_lock(&state_lock);
	devfreq->data = node->orig_data;
	if (node->map || node->common_map) {
		node->df = NULL;
	} else {
		list_del(&node->list);
		kfree(node);
	}
	mutex_unlock(&state_lock);

	sysfs_remove_group(&devfreq->dev.kobj, &dev_attr_group);
	unregister_cpufreq();
	unregister_cpu_pm();
	unregister_cpu_hotplug();
}

static int devfreq_cpufreq_ev_handler(struct devfreq *devfreq,
					unsigned int event, void *data)
{
	int ret;

	switch (event) {
	case DEVFREQ_GOV_START:

		ret = devfreq_cpufreq_gov_start(devfreq);
		if (ret) {
			pr_err("Governor start failed!\n");
			return ret;
		}
		pr_debug("Enabled dev CPUfreq governor\n");
		break;

	case DEVFREQ_GOV_STOP:

		devfreq_cpufreq_gov_stop(devfreq);
		pr_debug("Disabled dev CPUfreq governor\n");
		break;
	}

	return 0;
}

static struct devfreq_governor devfreq_cpufreq = {
	.name = "cpufreq",
	.get_target_freq = devfreq_cpufreq_get_freq,
	.event_handler = devfreq_cpufreq_ev_handler,
};

#define NUM_COLS	2
static struct freq_map *read_tbl(struct device_node *of_node, char *prop_name)
{
	int len, nf, i, j;
	u32 data;
	struct freq_map *tbl;

	if (!of_find_property(of_node, prop_name, &len))
		return NULL;
	len /= sizeof(data);

	if (len % NUM_COLS || len == 0)
		return NULL;
	nf = len / NUM_COLS;

	tbl = kzalloc((nf + 1) * sizeof(*tbl), GFP_KERNEL);
	if (!tbl)
		return NULL;

	for (i = 0, j = 0; i < nf; i++, j += 2) {
		of_property_read_u32_index(of_node, prop_name, j, &data);
		tbl[i].cpu_khz = data;

		of_property_read_u32_index(of_node, prop_name, j + 1, &data);
		tbl[i].target_freq = data;
	}
	tbl[i].cpu_khz = 0;

	return tbl;
}

#define PROP_TARGET "target-dev"
#define PROP_TABLE "cpu-to-dev-map"
static int add_table_from_of(struct device_node *of_node)
{
	struct device_node *target_of_node;
	struct devfreq_node *node;
	struct freq_map *common_tbl;
	struct freq_map **tbl_list = NULL;
	static char prop_name[] = PROP_TABLE "-999999";
	int cpu, ret, cnt = 0, prop_sz = ARRAY_SIZE(prop_name);

	target_of_node = of_parse_phandle(of_node, PROP_TARGET, 0);
	if (!target_of_node)
		return -EINVAL;

	node = kzalloc(sizeof(struct devfreq_node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	common_tbl = read_tbl(of_node, PROP_TABLE);
	if (!common_tbl) {
		tbl_list = kzalloc(sizeof(*tbl_list) * NR_CPUS, GFP_KERNEL);
		if (!tbl_list)
			return -ENOMEM;

		for_each_possible_cpu(cpu) {
			ret = snprintf(prop_name, prop_sz, "%s-%d",
					PROP_TABLE, cpu);
			if (ret >= prop_sz) {
				pr_warn("More CPUs than I can handle!\n");
				pr_warn("Skipping rest of the tables!\n");
				break;
			}
			tbl_list[cpu] = read_tbl(of_node, prop_name);
			if (tbl_list[cpu])
				cnt++;
		}
	}
	if (!common_tbl && !cnt) {
		kfree(tbl_list);
		return -EINVAL;
	}

	mutex_lock(&state_lock);
	node->of_node = target_of_node;
	node->map = tbl_list;
	node->common_map = common_tbl;
	list_add_tail(&node->list, &devfreq_list);
	mutex_unlock(&state_lock);

	return 0;
}

static int __init devfreq_cpufreq_init(void)
{
	int ret;
	struct device_node *of_par, *of_child;

	of_par = of_find_node_by_name(NULL, "devfreq-cpufreq");
	if (of_par) {
		for_each_child_of_node(of_par, of_child) {
			ret = add_table_from_of(of_child);
			if (ret)
				pr_err("Parsing %s failed!\n", of_child->name);
			else
				pr_debug("Parsed %s.\n", of_child->name);
		}
		of_node_put(of_par);
	} else {
		pr_info("No tables parsed from DT.\n");
	}

	ret = devfreq_add_governor(&devfreq_cpufreq);
	if (ret) {
		pr_err("Governor add failed!\n");
		return ret;
	}

	return 0;
}
subsys_initcall(devfreq_cpufreq_init);

static void __exit devfreq_cpufreq_exit(void)
{
	int ret, cpu;
	struct devfreq_node *node, *tmp;

	ret = devfreq_remove_governor(&devfreq_cpufreq);
	if (ret)
		pr_err("Governor remove failed!\n");

	mutex_lock(&state_lock);
	list_for_each_entry_safe(node, tmp, &devfreq_list, list) {
		kfree(node->common_map);
		for_each_possible_cpu(cpu)
			kfree(node->map[cpu]);
		kfree(node->map);
		list_del(&node->list);
		kfree(node);
	}
	mutex_unlock(&state_lock);

	return;
}
module_exit(devfreq_cpufreq_exit);

MODULE_DESCRIPTION("CPU freq based generic governor for devfreq devices");
MODULE_LICENSE("GPL v2");

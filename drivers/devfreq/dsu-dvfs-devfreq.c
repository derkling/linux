/*
 * A devfreq driver for ARM DynamIQ Shared Unit (DSU) DVFS control
 *
 * Copyright (c) 2017 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 *		Ionela Voinescu <Ionela.Voinescu@arm.com>
 *
 */

#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#define CREATE_TRACE_POINTS
#include <trace/events/dsu_dvfs.h>

#define DSU_DVFS_PLATFORM_DEVICE_NAME			"dsu_dvfs"
#define DSU_DVFS_DEFAULT_GOVERNOR			"cpufreq"
#define DSU_DVFS_DEFAULT_REQUIRED_GOV_POLLING		10000

struct dsu_dvfs {
	int id;
	struct devfreq *devfreq;
	struct platform_device *pdev;
	struct devfreq_dev_profile *devfreq_profile;
	unsigned int follow_cpu;
	u32 cur_freq;
	u32 prev_freq;
	u32 initial_freq;
	unsigned long *freq_table;
	int freq_table_len;
	unsigned int last_update;
};

static atomic_t dsu_dvfs_device_id = ATOMIC_INIT(0);

static int dsu_dvfs_set_frequency(struct device *dev,
				  unsigned long freq)
{
	struct dsu_dvfs *dsu = dev_get_drvdata(dev);
	struct cpufreq_policy *policy;
	unsigned long prev_min = 0;

	dsu->prev_freq = dsu->cur_freq;
	dsu->cur_freq = freq;

	policy = cpufreq_cpu_get(dsu->follow_cpu);
	if (policy) {
		prev_min = policy->min;
		if (policy->min == dsu->cur_freq) {
			cpufreq_cpu_put(policy);
		} else {
			policy->user_policy.min = dsu->cur_freq;
			cpufreq_cpu_put(policy);
			cpufreq_update_policy(dsu->follow_cpu);
		}
	}

	trace_dsu_dvfs_dev_status(dsu->id, dsu->follow_cpu, dsu->cur_freq,
				  dsu->prev_freq, prev_min);

	return 0;
}

static int dsu_dvfs_devfreq_target(struct device *dev, unsigned long *freq,
				   u32 flags)
{
	/* Set requested frequency */
	return dsu_dvfs_set_frequency(dev, *freq);
}

static int dsu_dvfs_devfreq_get_dev_status(struct device *dev,
					    struct devfreq_dev_status *stat)
{
	struct dsu_dvfs *dsu = dev_get_drvdata(dev);
	unsigned int const usec = ktime_to_us(ktime_get());
	unsigned int delta;

	delta = usec - dsu->last_update;
	dsu->last_update = usec;

	stat->current_frequency = dsu->cur_freq;
	stat->total_time = delta;
	stat->busy_time  = delta;

	return 0;
}

static int dsu_dvfs_reinit_device(struct device *dev)
{
	struct dsu_dvfs *dsu = dev_get_drvdata(dev);

	dsu->last_update = ktime_to_us(ktime_get());

	return dsu_dvfs_set_frequency(dev, dsu->initial_freq);
}

static int dsu_dvfs_setup_devfreq_profile(struct platform_device *pdev)
{
	struct dsu_dvfs *dsu = platform_get_drvdata(pdev);
	struct devfreq_dev_profile *df_profile;

	dsu->devfreq_profile = devm_kzalloc(&pdev->dev,
			sizeof(struct devfreq_dev_profile), GFP_KERNEL);
	if (IS_ERR(dsu->devfreq_profile)) {
		dev_dbg(&pdev->dev, "no memory.\n");
		return PTR_ERR(dsu->devfreq_profile);
	}

	df_profile = dsu->devfreq_profile;

	df_profile->target = dsu_dvfs_devfreq_target;
	df_profile->get_dev_status = dsu_dvfs_devfreq_get_dev_status;
	df_profile->freq_table = dsu->freq_table;
	df_profile->max_state = dsu->freq_table_len;
	df_profile->polling_ms = DSU_DVFS_DEFAULT_REQUIRED_GOV_POLLING;
	df_profile->initial_freq = dsu->initial_freq;

	return 0;
}

static int dsu_dvfs_parse_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct dsu_dvfs *dsu = platform_get_drvdata(pdev);
	int ret = 0;

	of_node_get(node);

	ret = of_property_read_u32(node, "cpu",
				   &dsu->follow_cpu);
	of_node_put(node);

	dev_info(&pdev->dev, "DT info captured.\n");

	return ret;
}

static int dsu_pctrl_create_configuration(struct platform_device *pdev)
{
	struct dsu_dvfs *dsu = platform_get_drvdata(pdev);
	int count, i;
	struct cpufreq_frequency_table *table;
	struct cpufreq_policy *policy;

	dev_info(&pdev->dev, "start to create configuration.\n");

	/* TODO: this is just for coupled systems where you control
	 * frequency of DSU by controlling the frequency of the
	 * coupled CPU group
	 */
	policy = cpufreq_cpu_get(dsu->follow_cpu);
	if (!policy)
		return PTR_ERR(policy);

	dsu->initial_freq = policy->min;
	dsu->cur_freq = policy->min;
	dev_info(&pdev->dev, "found min frequency: %d.\n", dsu->cur_freq);

	table = policy->freq_table;
	for (count = 0; table[count].frequency != CPUFREQ_TABLE_END; count++)
		;
	dsu->freq_table_len = count;
	dsu->freq_table = devm_kcalloc(&pdev->dev, dsu->freq_table_len,
				       sizeof(*dsu->freq_table),
				       GFP_KERNEL);
	if (IS_ERR(dsu->freq_table))
		return -ENOMEM;

	dev_info(&pdev->dev, "found %d frequency in frequency table.\n", count);

	for (i = 0; i < count; i++) {
		dsu->freq_table[i] = table[i].frequency;
		dev_info(&pdev->dev, "freq %lu.\n", dsu->freq_table[i]);
	}

	cpufreq_cpu_put(policy);
	dev_info(&pdev->dev, "finish creating configuration.\n");

	return 0;
}

static void dsu_pctrl_remove_opps(struct platform_device *pdev)
{
	struct dev_pm_opp *opp;
	int i, count;
	unsigned long freq;

	count = dev_pm_opp_get_opp_count(&pdev->dev);
	if (count <= 0)
		return;

	rcu_read_lock();
	for (i = 0, freq = 0; i < count; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(&pdev->dev, &freq);
		if (!IS_ERR(opp))
			dev_pm_opp_remove(&pdev->dev, freq);
	}
	rcu_read_unlock();
}

static int dsu_pctrl_enable_opps(struct platform_device *pdev)
{
	struct dsu_dvfs *dsu = platform_get_drvdata(pdev);
	int i, ret;
	int opp_count = 0;

	if (dsu->freq_table_len <= 0)
		return -EINVAL;

	for (i = 0; i < dsu->freq_table_len; i++) {
		ret = dev_pm_opp_add(&pdev->dev, dsu->freq_table[i],
				     dsu->freq_table[i]);
		if (ret)
			dev_warn(&pdev->dev, "cannot add a new OPP.\n");
		else
			opp_count++;
	}

	if (opp_count == 0) {
		dev_err(&pdev->dev, "device has no OPP registered.\n");
		return -ENODEV;
	}

	return 0;
}

static int dsu_dvfs_setup(struct platform_device *pdev)
{
	struct dsu_dvfs *dsu = platform_get_drvdata(pdev);
	int ret = 0;

	ret = dsu_pctrl_create_configuration(pdev);
	if (ret) {
		dev_err(&pdev->dev, "cannot create frequency table.\n");
		return ret;
	}

	ret = dsu_pctrl_enable_opps(pdev);
	if (ret) {
		dev_info(&pdev->dev, "device setup failed.\n");
		return ret;
	}

	ret = dsu_dvfs_setup_devfreq_profile(pdev);
	if (ret) {
		dev_info(&pdev->dev, "device setup failed.\n");
		return ret;
	}
	dsu->last_update = ktime_to_us(ktime_get());

	dsu->devfreq = devm_devfreq_add_device(&pdev->dev,
					       dsu->devfreq_profile,
					       DSU_DVFS_DEFAULT_GOVERNOR, NULL);

	if (IS_ERR(dsu->devfreq)) {
		dev_err(&pdev->dev, "registering to devfreq failed.\n");
		return PTR_ERR(dsu->devfreq);
	}

	dsu->devfreq->min_freq = dsu->freq_table[0];
	dsu->devfreq->max_freq = dsu->freq_table[dsu->freq_table_len - 1];

	return 0;
}


static int dsu_dvfs_init_device(struct platform_device *pdev)
{
	dsu_dvfs_reinit_device(&pdev->dev);

	return 0;
}

static int dsu_dvfs_suspend(struct device *dev)
{
	struct dsu_dvfs *dsu = dev_get_drvdata(dev);
	int ret = 0;

	ret = devfreq_suspend_device(dsu->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to suspend devfreq device.\n");
		return ret;
	}

	/* Set the number of active frequency to minimum during suspend */
	ret = dsu_dvfs_set_frequency(dev, dsu->initial_freq);
	if (ret < 0) {
		dev_err(dev, "failed to set frequency to minimum.\n");
		return ret;
	}

	return ret;
}

static int dsu_dvfs_resume(struct device *dev)
{

	struct dsu_dvfs *dsu = dev_get_drvdata(dev);
	int ret = 0;

	dsu_dvfs_reinit_device(dev);

	ret = devfreq_resume_device(dsu->devfreq);
	if (ret < 0)
		dev_err(dev, "failed to resume devfreq device.\n");

	return ret;
}

static SIMPLE_DEV_PM_OPS(dsu_dvfs_pm, dsu_dvfs_suspend, dsu_dvfs_resume);

static int dsu_dvfs_devfreq_probe(struct platform_device *pdev)
{
	struct dsu_dvfs *dsu;
	int ret = 0;

	dev_info(&pdev->dev, "registering DSU frequency control device.\n");

	dsu = devm_kzalloc(&pdev->dev, sizeof(*dsu), GFP_KERNEL);
	if (!dsu)
		return -ENOMEM;

	platform_set_drvdata(pdev, dsu);
	dsu->pdev = pdev;
	dsu->id = atomic_inc_return(&dsu_dvfs_device_id);

	ret = dsu_dvfs_parse_dt(pdev);
	if (ret)
		goto failed;

	ret = dsu_dvfs_setup(pdev);
	if (ret)
		goto failed;

	ret = dsu_dvfs_init_device(pdev);
	if (ret)
		goto failed;

	dev_info(&pdev->dev, "sucessfull in registering DSU device.\n");

	return 0;
failed:
	dev_err(&pdev->dev, "failed to register driver, err %d.\n", ret);
	kfree(dsu);
	return ret;
}

static int dsu_dvfs_devfreq_remove(struct platform_device *pdev)
{
	struct dsu_dvfs *dsu = platform_get_drvdata(pdev);
	int ret = 0;

	dev_info(&pdev->dev, "unregistering DSU frequency control device.\n");

	ret = dsu_dvfs_set_frequency(&pdev->dev, dsu->initial_freq);

	devm_devfreq_remove_device(&pdev->dev, dsu->devfreq);
	dsu_pctrl_remove_opps(pdev);

	return ret;
}

static const struct of_device_id dsu_dvfs_devfreq_id[] = {
	{.compatible = "arm,dsu_dvfs", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, dsu_dvfs_devfreq_id);

static struct platform_driver dsu_dvfs_devfreq_driver = {
	.probe	= dsu_dvfs_devfreq_probe,
	.remove = dsu_dvfs_devfreq_remove,
	.driver = {
		.name = DSU_DVFS_PLATFORM_DEVICE_NAME,
		.of_match_table = dsu_dvfs_devfreq_id,
		.pm = &dsu_dvfs_pm,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(dsu_dvfs_devfreq_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARM DSU DVFS devfreq driver");
MODULE_AUTHOR("ARM Ltd.");
MODULE_VERSION("1.0");

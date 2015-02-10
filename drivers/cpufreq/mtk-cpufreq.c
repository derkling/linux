/*
* Copyright (c) 2015 Linaro Ltd.
* Author: Pi-Cheng Chen <pi-cheng.chen@linaro.org>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq-dt.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#define VOLT_SHIFT_LOW_LIMIT		100000
#define VOLT_SHIFT_UP_LIMIT		200000
#define MAX_VOLTAGE			1150000

struct cpu_opp_table {
	unsigned int freq;
	int vproc;
	int vsram;
};

/*
 * The struct dvfs_info holds necessary information for doing CPU DVFS of each
 * cluster. For Mediatek SoCs, each CPU cluster in SoC has two voltage inputs,
 * Vproc and Vsram. For some cluster in SoC, the two voltage inputs are supplied
 * by different PMICs. In this case, when scaling up/down Vsram and Vproc, the
 * voltages of two inputs need to be scaled under a hardware limitation:
 * 100mV < Vsram -Vproc < 200mV
 * When scaling up/down the clock frequency of a cluster, the clock source need
 * to be switched to another stable PLL clock temporarily, and switched back to
 * the original PLL after the original PLL becomes stable at target frequency.
 * Hence the voltage inputs of cluster need to be set to an intermediate voltage
 * before the clock frequency being scaled up/down.
 */

struct dvfs_info {
	struct cpufreq_cpu_domain domain;
	struct cpu_opp_table *opp_tbl;
	struct device *cpu_dev;
	struct regulator *proc_reg;
	struct regulator *sram_reg;
	unsigned int intermediate_freq;
	int intermediate_vproc;
};

static struct cpufreq_dt_platform_data cpufreq_dt_pd;

static inline struct cpufreq_cpu_domain *to_cpu_domain(struct list_head *_n)
{
	return container_of(_n, struct cpufreq_cpu_domain, node);
}

static inline struct dvfs_info *to_dvfs_info(struct cpufreq_cpu_domain *_d)
{
	return container_of(_d, struct dvfs_info, domain);
}

static struct cpufreq_cpu_domain *get_cpu_domain(struct list_head *domain_list,
						 int cpu)
{
	struct list_head *node;

	list_for_each(node, domain_list) {
		struct cpufreq_cpu_domain *domain;

		domain = to_cpu_domain(node);
		if (cpumask_test_cpu(cpu, &domain->cpus))
			return domain;
	}

	return NULL;
}

static int cpu_opp_table_get_freq_index(struct cpu_opp_table *opp_tbl,
					unsigned int freq)
{
	int i;

	for (i = 0; opp_tbl[i].freq != 0; i++) {
		if (opp_tbl[i].freq >= freq)
			return i;
	}

	return -EINVAL;
}

static int cpu_opp_table_get_volt_index(struct cpu_opp_table *opp_tbl,
					unsigned int volt)
{
	int i;

	for (i = 0; opp_tbl[i].vproc != -1; i++)
		if (opp_tbl[i].vproc >= volt)
			return i;

	return -EINVAL;
}

static int get_regulator_voltage_ceil(struct regulator *regulator, int voltage)
{
	int cnt, i, volt = -1;

	if (IS_ERR_OR_NULL(regulator))
		return -EINVAL;

	cnt = regulator_count_voltages(regulator);
	for (i = 0; i < cnt && volt < voltage; i++)
		volt = regulator_list_voltage(regulator, i);

	return (i >= cnt) ? -EINVAL : volt;
}

static int get_regulator_voltage_floor(struct regulator *regulator, int voltage)
{
	int cnt, i, volt = -1;

	if (IS_ERR_OR_NULL(regulator))
		return -EINVAL;

	cnt = regulator_count_voltages(regulator);
	/* skip all trailing 0s in the list of supported voltages */
	for (i = cnt - 1; i >= 0 && volt <= 0; i--)
		volt = regulator_list_voltage(regulator, i);

	for (; i >= 0; i--) {
		volt = regulator_list_voltage(regulator, i);
		if (volt <= voltage)
			return volt;
	}

	return -EINVAL;
}

static int mtk_cpufreq_voltage_trace(struct dvfs_info *dvfs, int new_index)
{
	struct cpu_opp_table *opp_tbl = dvfs->opp_tbl;
	int old_vproc, new_vproc, old_vsram, new_vsram, vsram, vproc;

	old_vproc = regulator_get_voltage(dvfs->proc_reg);
	old_vsram = regulator_get_voltage(dvfs->sram_reg);

	new_vproc = opp_tbl[new_index].vproc;
	new_vsram = opp_tbl[new_index].vsram;

	/*
	 * In the case the voltage is going to be scaled up, Vsram and Vproc
	 * need to be scaled up step by step. In each step, Vsram needs to be
	 * set to (Vproc + 200mV) first, then Vproc is set to (Vsram - 100mV).
	 * Repeat the step until Vsram and Vproc are set to target voltage.
	 */
	if (old_vproc < new_vproc) {
		while (1) {
			vsram = (new_vsram - old_vproc < VOLT_SHIFT_UP_LIMIT) ?
				new_vsram : old_vproc + VOLT_SHIFT_UP_LIMIT;
			vsram = get_regulator_voltage_floor(dvfs->sram_reg,
							    vsram);
			regulator_set_voltage_tol(dvfs->sram_reg, vsram, 0);

			vproc = (new_vsram == vsram) ?
				new_vproc : vsram - VOLT_SHIFT_LOW_LIMIT;
			vproc = get_regulator_voltage_ceil(dvfs->proc_reg,
							   vproc);
			regulator_set_voltage_tol(dvfs->proc_reg, vproc, 0);

			if (new_vproc == vproc && new_vsram == vsram)
				break;

			old_vproc = vproc;
		}

	/*
	 * In the case the voltage is going to be scaled down, Vsram and Vproc
	 * need to be scaled down step by step. In each step, Vproc needs to be
	 * set to (Vsram - 200mV) first, then Vproc is set to (Vproc + 100mV).
	 * Repeat the step until Vsram and Vproc are set to target voltage.
	 */
	} else if (old_vproc > new_vproc) {
		while (1) {
			vproc = (old_vsram - new_vproc < VOLT_SHIFT_UP_LIMIT) ?
				new_vproc : old_vsram - VOLT_SHIFT_UP_LIMIT;
			vproc = get_regulator_voltage_ceil(dvfs->proc_reg,
							   vproc);
			regulator_set_voltage_tol(dvfs->proc_reg, vproc, 0);

			vsram = (new_vproc == vproc) ?
				new_vsram : vproc + VOLT_SHIFT_LOW_LIMIT;
			vsram = get_regulator_voltage_floor(dvfs->sram_reg,
							    vsram);
			regulator_set_voltage_tol(dvfs->sram_reg, vsram, 0);

			if (new_vproc == vproc && new_vsram == vsram)
				break;

			old_vsram = vsram;
		}
	}

	return 0;
}

static int mtk_cpufreq_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	struct cpufreq_freqs *freqs = data;
	struct cpufreq_dt_platform_data *pd = &cpufreq_dt_pd;
	struct cpufreq_cpu_domain *domain = get_cpu_domain(&pd->domain_list,
							   freqs->cpu);
	struct dvfs_info *dvfs = to_dvfs_info(domain);
	struct cpu_opp_table *opp_tbl = dvfs->opp_tbl;
	int old_vproc, new_vproc, new_index;

	old_vproc = regulator_get_voltage(dvfs->proc_reg);
	new_index = cpu_opp_table_get_freq_index(opp_tbl, freqs->new * 1000);
	new_vproc = opp_tbl[new_index].vproc;

	/*
	 * If the target Vproc is lower than intermediate voltage, set Vproc
	 * to intermediate voltage before setting clock frequency of CPU.
	 */
	if (action == CPUFREQ_PRECHANGE &&
	    new_vproc < dvfs->intermediate_vproc) {
		new_vproc = dvfs->intermediate_vproc;
		new_index = cpu_opp_table_get_volt_index(opp_tbl, new_vproc);
		BUG_ON(new_index < 0);
	}

	if (old_vproc == new_vproc)
		return NOTIFY_DONE;

	if ((action == CPUFREQ_PRECHANGE && old_vproc < new_vproc) ||
	    (action == CPUFREQ_POSTCHANGE && old_vproc > new_vproc)) {
		/*
		 * If we have regulators for both Vproc and Vsram of a cluster
		 * we need to do 'voltage trace' to scale up/down the voltage.
		 * Otherwise just set Vproc to target voltage.
		 */
		if (!IS_ERR_OR_NULL(dvfs->proc_reg) &&
		    !IS_ERR_OR_NULL(dvfs->sram_reg))
			mtk_cpufreq_voltage_trace(dvfs, new_index);
		else if (!IS_ERR_OR_NULL(dvfs->proc_reg))
			regulator_set_voltage_tol(dvfs->proc_reg, new_vproc, 0);
	}

	return NOTIFY_OK;
}

static struct notifier_block mtk_cpufreq_nb = {
	.notifier_call = mtk_cpufreq_notify,
};

static int cpu_opp_table_init(struct device *dev, struct dvfs_info *dvfs)
{
	struct device *cpu_dev = dvfs->cpu_dev;
	struct cpu_opp_table *opp_tbl;
	struct dev_pm_opp *opp;
	int ret, cnt, i;
	unsigned long rate, vproc, vsram;

	ret = of_init_opp_table(cpu_dev);
	if (ret) {
		dev_err(dev, "Failed to init mtk_opp_table: %d\n", ret);
		return ret;
	}

	rcu_read_lock();

	cnt = dev_pm_opp_get_opp_count(cpu_dev);
	if (cnt < 0) {
		dev_err(cpu_dev, "No OPP table is found: %d", cnt);
		ret = cnt;
		goto out_free_opp_tbl;
	}

	opp_tbl = devm_kcalloc(dev, (cnt + 1), sizeof(struct cpu_opp_table),
			       GFP_ATOMIC);
	if (!opp_tbl) {
		ret = -ENOMEM;
		goto out_free_opp_tbl;
	}

	for (i = 0, rate = 0; i < cnt; i++, rate++) {
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
		if (IS_ERR(opp)) {
			ret = PTR_ERR(opp);
			goto out_free_opp_tbl;
		}

		vproc = dev_pm_opp_get_voltage(opp);
		vproc = get_regulator_voltage_ceil(dvfs->proc_reg, vproc);

		if (!IS_ERR_OR_NULL(dvfs->sram_reg)) {
			vsram = vproc + VOLT_SHIFT_LOW_LIMIT;
			vsram = get_regulator_voltage_ceil(dvfs->sram_reg,
							   vsram);
			if (vsram > MAX_VOLTAGE)
				vsram = MAX_VOLTAGE;
		} else {
			vsram = 0;
		}

		if (vproc < 0 ||
		    (!IS_ERR_OR_NULL(dvfs->sram_reg) && vsram < 0)) {
			dev_err(dev,
				"Failed to get voltage setting of OPPs\n");
			ret = -EINVAL;
			goto out_free_opp_tbl;
		}

		opp_tbl[i].freq = rate;
		opp_tbl[i].vproc = vproc;
		opp_tbl[i].vsram = vsram;
	}

	opp_tbl[i].freq = 0;
	opp_tbl[i].vproc = -1;
	opp_tbl[i].vsram = -1;
	dvfs->opp_tbl = opp_tbl;

	/* Get intermediate voltage setting for each cluster */
	rate = dvfs->intermediate_freq;
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
	vproc = dev_pm_opp_get_voltage(opp);
	vproc = get_regulator_voltage_ceil(dvfs->proc_reg, vproc);
	if (vproc < 0) {
		dev_err(dev, "Failed to get intermediate voltage\n");
		ret = -EINVAL;
	}
	dvfs->intermediate_vproc = vproc;

out_free_opp_tbl:
	rcu_read_unlock();
	of_free_opp_table(cpu_dev);

	return ret;
}

static int mtk_cpufreq_dvfs_info_init(struct dvfs_info *dvfs)
{
	struct cpufreq_cpu_domain *domain = &dvfs->domain;
	struct device *cpu_dev;
	struct clk *inter_clk;
	struct regulator *proc_reg, *sram_reg;
	int cpu;

	/*
	 * For each cluster, the intermediate clock source and Vproc regualtor
	 * are mandatory. If Vsram regulator was provided, we need to do
	 * 'voltage trace' for this cluster when scaling up/down th voltage.
	 */
	for_each_cpu(cpu, &domain->cpus) {
		if (!dvfs->cpu_dev) {
			cpu_dev = get_cpu_device(cpu);
			proc_reg = regulator_get_exclusive(cpu_dev, "proc");
			sram_reg = regulator_get_exclusive(cpu_dev, "sram");
			inter_clk = clk_get(cpu_dev, "intermediate");

			if (PTR_ERR(proc_reg) == -EPROBE_DEFER ||
			    PTR_ERR(sram_reg) == -EPROBE_DEFER ||
			    PTR_ERR(inter_clk) == -EPROBE_DEFER) {
				dev_warn(cpu_dev,
					 "Regulator/clk is not ready yet.\n");
				return -EPROBE_DEFER;
			}

			if (IS_ERR_OR_NULL(proc_reg) ||
			    IS_ERR_OR_NULL(inter_clk))
				continue;

			dvfs->cpu_dev = cpu_dev;
			dvfs->proc_reg = proc_reg;
			dvfs->sram_reg = sram_reg;
			dvfs->intermediate_freq = clk_get_rate(inter_clk);

			return 0;
		}
	}

	return -ENODEV;
}

static int mtk_cpufreq_probe(struct platform_device *pdev)
{
	struct cpufreq_dt_platform_data *pd;
	struct platform_device *dev;
	int cpu, ret;

	pd = &cpufreq_dt_pd;
	pd->independent_clocks = 1,
	INIT_LIST_HEAD(&pd->domain_list);

	for_each_possible_cpu(cpu) {
		struct dvfs_info *dvfs;
		struct cpufreq_cpu_domain *domain;

		if (get_cpu_domain(&pd->domain_list, cpu))
			continue;

		dvfs = devm_kzalloc(&pdev->dev, sizeof(*dvfs), GFP_KERNEL);
		if (!dvfs)
			return -ENOMEM;

		domain = &dvfs->domain;
		cpumask_copy(&domain->cpus, &cpu_topology[cpu].core_sibling);

		ret = mtk_cpufreq_dvfs_info_init(dvfs);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to initialize dvfs_info: %d\n,", ret);
			return ret;
		}

		ret = cpu_opp_table_init(&pdev->dev, dvfs);
		if (ret) {
			dev_err(&pdev->dev, "Failed to setup cpu_opp_table: %d\n",
				ret);
			return ret;
		}

		list_add(&domain->node, &pd->domain_list);
	}

	ret = cpufreq_register_notifier(&mtk_cpufreq_nb,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register cpufreq notifier\n");
		return ret;
	}

	dev = platform_device_register_data(NULL, "cpufreq-dt", -1, pd,
					    sizeof(*pd));
	if (IS_ERR(dev)) {
		dev_err(&pdev->dev,
			"Failed to register cpufreq-dt platform device\n");
		return PTR_ERR(dev);
	}

	return 0;
}

static struct platform_driver mtk_cpufreq_platdrv = {
	.driver	= {
		.name	= "mtk-cpufreq",
	},
	.probe	= mtk_cpufreq_probe,
};

static int mtk_cpufreq_driver_init(void)
{
	struct platform_device *pdev;
	int ret;

	if (!of_machine_is_compatible("mediatek,mt8173"))
		return -ENODEV;

	ret = platform_driver_register(&mtk_cpufreq_platdrv);
	if (ret) {
		pr_err("%s: Failed to register mtk-cpufreq driver\n",
		       __func__);
		return ret;
	}

	pdev = platform_device_register_simple("mtk-cpufreq", 0, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("%s: Failed to register mtk-cpufreq device\n",
		       __func__);
		return PTR_ERR(pdev);
	}

	return 0;
}
module_init(mtk_cpufreq_driver_init);

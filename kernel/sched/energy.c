/*
 * Obtain energy cost data from DT and populate relevant scheduler data
 * structures.
 *
 * Copyright (C) 2015 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define pr_fmt(fmt) "sched-energy: " fmt

#define DEBUG

#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched_energy.h>
#include <linux/stddef.h>
#include <linux/cpuset.h>
#include <linux/cpufreq.h>

struct sched_group_energy *sge_array[NR_CPUS][NR_SD_LEVELS];
unsigned long long elapsed[NR_CPUS];

static void free_resources(void)
{
	int cpu, sd_level;
	struct sched_group_energy *sge;

	for_each_possible_cpu(cpu) {
		for_each_possible_sd_level(sd_level) {
			sge = sge_array[cpu][sd_level];
			if (sge) {
				kfree(sge->cap_states);
				kfree(sge->idle_states);
				kfree(sge);
			}
		}
	}
}

void init_sched_energy_costs_from_dt(void)
{
	struct device_node *cn, *cp;
	struct capacity_state *cap_states;
	struct idle_state *idle_states;
	struct sched_group_energy *sge;
	const struct property *prop;
	int sd_level, i, nstates, cpu;
	const __be32 *val;

	for_each_possible_cpu(cpu) {
		cn = of_get_cpu_node(cpu, NULL);
		if (!cn) {
			pr_warn("CPU device node missing for CPU %d\n", cpu);
			return;
		}

		if (!of_find_property(cn, "sched-energy-costs", NULL)) {
			pr_warn("CPU device node has no sched-energy-costs\n");
			return;
		}

		for_each_possible_sd_level(sd_level) {
			cp = of_parse_phandle(cn, "sched-energy-costs", sd_level);
			if (!cp)
				break;

			prop = of_find_property(cp, "busy-cost-data", NULL);
			if (!prop || !prop->value) {
				pr_warn("No busy-cost data, skipping sched_energy init\n");
				goto out;
			}

			sge = kcalloc(1, sizeof(struct sched_group_energy),
				      GFP_NOWAIT);

			nstates = (prop->length / sizeof(u32)) / 2;
			cap_states = kcalloc(nstates,
					     sizeof(struct capacity_state),
					     GFP_NOWAIT);

			for (i = 0, val = prop->value; i < nstates; i++) {
				cap_states[i].cap = be32_to_cpup(val++);
				cap_states[i].power = be32_to_cpup(val++);
			}

			sge->nr_cap_states = nstates;
			sge->cap_states = cap_states;

			prop = of_find_property(cp, "idle-cost-data", NULL);
			if (!prop || !prop->value) {
				pr_warn("No idle-cost data, skipping sched_energy init\n");
				goto out;
			}

			nstates = (prop->length / sizeof(u32));
			idle_states = kcalloc(nstates,
					      sizeof(struct idle_state),
					      GFP_NOWAIT);

			for (i = 0, val = prop->value; i < nstates; i++)
				idle_states[i].power = be32_to_cpup(val++);

			sge->nr_idle_states = nstates;
			sge->idle_states = idle_states;

			sge_array[cpu][sd_level] = sge;
		}
	}

	pr_info("Sched-energy-costs installed from DT\n");
	return;

out:
	free_resources();
}

static int run_bogus_benchmark(int cpu)
{
	int i, ret, trials = 100;
	unsigned long res;
	unsigned long long begin, diff;
	long long diff_avg = 0;

	ret = set_cpus_allowed_ptr(current, cpumask_of(cpu));
	if (ret) {
		pr_warn("%s: failed to set allowed ptr\n", __func__);
		return -EINVAL;
	}

	while (trials--) {
		begin = sched_clock();
		for (i = 0; i < 100000; i++)
			res = int_sqrt(i);
		diff = sched_clock() - begin;
		diff_avg = (diff_avg + diff) >> 1;
		pr_info("%s: cpu=%d diff=%llu diff_avg=%lld\n", __func__, cpu, diff, diff_avg);
		if ((abs(diff - diff_avg) << 5) < diff_avg)
			break;
	}
	elapsed[cpu] = diff_avg;
	pr_info("%s: cpu=%d elapsed=%llu trials=%d\n", __func__, cpu, elapsed[cpu], trials);

	ret = set_cpus_allowed_ptr(current, cpu_active_mask);
	if (ret) {
		pr_warn("%s: failed to set allowed ptr\n", __func__);
		return -EINVAL;
	}

	return 0;
}

void init_sched_energy_costs_default(void)
{
	struct capacity_state *cap_states;
	struct idle_state *idle_states;
	struct sched_group_energy *sge;
	int sd_level, i, nstates, cpu, ret;
	unsigned long long elapsed_min = ULLONG_MAX;
	struct cpufreq_policy curr_policy, bench_policy, *policy;

	if (sge_array[0][0]) {
		pr_info("Sched-energy-costs already installed: skipping\n");
		return;
	}

	for_each_possible_cpu(cpu) {
		ret = cpufreq_get_policy(&curr_policy, cpu);
		if (ret)
			return;

		pr_info("related_cpus=%*pbl\n", cpumask_pr_args(curr_policy.related_cpus));
		if (cpu != cpumask_first(curr_policy.related_cpus)) {
			pr_info("freq domain already visited\n");
			elapsed[cpu] = elapsed[cpumask_first(curr_policy.related_cpus)];
			continue;
		}

		ret = cpufreq_get_policy(&bench_policy, cpu);
		if (ret)
			return;

		if (cpufreq_parse_governor("performance",
					   &bench_policy.policy,
					   &bench_policy.governor))
			return;

		policy = cpufreq_cpu_get(cpu);
		down_write(&policy->rwsem);
		ret = cpufreq_set_policy(policy, &bench_policy);

		policy->user_policy.policy = policy->policy;
		policy->user_policy.governor = policy->governor;
		up_write(&policy->rwsem);
		cpufreq_cpu_put(policy);

		if (ret)
			return;

		run_bogus_benchmark(cpu);
		if (elapsed[cpu] < elapsed_min)
			elapsed_min = elapsed[cpu];
		pr_info("cpu=%d elapsed=%llu (min=%llu)\n", cpu, elapsed[cpu], elapsed_min);

		policy = cpufreq_cpu_get(cpu);
		down_write(&policy->rwsem);
		ret = cpufreq_set_policy(policy, &curr_policy);

		policy->user_policy.policy = policy->policy;
		policy->user_policy.governor = policy->governor;
		up_write(&policy->rwsem);
		cpufreq_cpu_put(policy);

		if (ret)
			return;
	}

	for_each_possible_cpu(cpu) {
		for_each_possible_sd_level(sd_level) {
			sge = kcalloc(1, sizeof(struct sched_group_energy),
				      GFP_NOWAIT);

			nstates = 1;
			cap_states = kcalloc(nstates,
					     sizeof(struct capacity_state),
					     GFP_NOWAIT);

			cap_states[0].cap = (elapsed_min << 10) / elapsed[cpu];
			cap_states[0].power = 0;

			sge->nr_cap_states = nstates;
			sge->cap_states = cap_states;

			nstates = 1;
			idle_states = kcalloc(nstates,
					      sizeof(struct idle_state),
					      GFP_NOWAIT);

			for (i = 0; i < nstates; i++)
				idle_states[i].power = 0;

			sge->nr_idle_states = nstates;
			sge->idle_states = idle_states;

			sge_array[cpu][sd_level] = sge;
		}
	}

	pr_info("Sched-energy-costs installed from default\n");
	request_energy_costs_update();
	rebuild_sched_domains();

	return;
}

void init_cpu_capacity_default(void)
{
	int cpu;
	unsigned long long elapsed_min = ULLONG_MAX;
	unsigned int curr_min, curr_max;
	struct cpufreq_policy *policy;

	if (sge_array[0][0]) {
		pr_info("Sched-energy-costs already installed: skipping\n");
		return;
	}

	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (IS_ERR_OR_NULL(policy))
			return;

		pr_info("related_cpus=%*pbl\n", cpumask_pr_args(policy->related_cpus));
		if (cpu != cpumask_first(policy->related_cpus)) {
			pr_info("freq domain already visited\n");
			elapsed[cpu] = elapsed[cpumask_first(policy->related_cpus)];
			cpufreq_cpu_put(policy);
			continue;
		}

		down_write(&policy->rwsem);
		curr_min = policy->user_policy.min;
		curr_max = policy->user_policy.max;
		policy->user_policy.min = policy->cpuinfo.max_freq;
		policy->user_policy.max = policy->cpuinfo.max_freq;
		up_write(&policy->rwsem);
		cpufreq_cpu_put(policy);
		cpufreq_update_policy(cpu);

		run_bogus_benchmark(cpu);
		if (elapsed[cpu] < elapsed_min)
			elapsed_min = elapsed[cpu];
		pr_info("cpu=%d elapsed=%llu (min=%llu)\n", cpu, elapsed[cpu], elapsed_min);

		policy = cpufreq_cpu_get(cpu);
		down_write(&policy->rwsem);
		policy->user_policy.min = curr_min;
		policy->user_policy.max = curr_max;
		up_write(&policy->rwsem);
		cpufreq_cpu_put(policy);
		cpufreq_update_policy(cpu);
	}

	for_each_possible_cpu(cpu)
		set_capacity_scale(cpu, (elapsed_min << 10) / elapsed[cpu]);

	pr_info("CPUs capacity installed from default\n");
	request_energy_costs_update();
	rebuild_sched_domains();
}

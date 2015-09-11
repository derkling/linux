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

unsigned long __attribute__((optimize("O0")))
my_int_sqrt(unsigned long x)
{
	unsigned long b, m, y = 0;

	if (x <= 1)
		return x;

	m = 1UL << (BITS_PER_LONG - 2);
	while (m != 0) {
		b = y + m;
		y >>= 1;

		if (x >= b) {
			x -= b;
			y += m;
		}
		m >>= 2;
	}

	return y;
}

unsigned long __attribute__((optimize("O0")))
bogus1(void)
{
	unsigned long i, res;

	for (i = 0; i < 100000; i++)
		res = my_int_sqrt(i);

	return res;
}

static int __attribute__((optimize("O0")))
run_bogus_benchmark(int cpu)
{
	struct sched_param param;
	int ret, trials = 25;
	u64 begin, end, diff, diff_avg = 0, count = 0;
	unsigned long res;

	param.sched_priority = 50;
	ret = sched_setscheduler_nocheck(current, SCHED_FIFO, &param);
	if (ret) {
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		do_exit(-EINVAL);
	}

	ret = set_cpus_allowed_ptr(current, cpumask_of(cpu));
	if (ret) {
		pr_warn("%s: failed to set allowed ptr\n", __func__);
		return -EINVAL;
	}

	while (trials--) {
		begin = local_clock();
		res = bogus1();
		end = local_clock();
		diff = end - begin;
		diff_avg = diff_avg * count + diff;
		diff_avg = div64_u64(diff_avg, ++count);
		pr_info("%s: cpu=%d begin=%llu end=%llu diff=%llu diff_avg=%llu count=%llu res=%lu\n",
			__func__, cpu, begin, end, diff, diff_avg, count, res);
	}
	elapsed[cpu] = diff_avg;
	pr_info("%s: cpu=%d elapsed=%llu\n", __func__, cpu, elapsed[cpu]);

	ret = set_cpus_allowed_ptr(current, cpu_active_mask);
	if (ret) {
		pr_warn("%s: failed to set allowed ptr\n", __func__);
		return -EINVAL;
	}

	param.sched_priority = 0;
	ret = sched_setscheduler_nocheck(current, SCHED_NORMAL, &param);
	if (ret) {
		pr_warn("%s: failed to set SCHED_NORMAL\n", __func__);
		do_exit(-EINVAL);
	}

	return 0;
}

void __weak request_energy_costs_update(void) {}

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

			cap_states[0].cap = div64_u64((elapsed_min << 10),
						      elapsed[cpu]);
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
			pr_info("cpu=%d elapsed=%llu (min=%llu)\n", cpu, elapsed[cpu], elapsed_min);
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

	for_each_possible_cpu(cpu) {
		unsigned long capacity;

		capacity = div64_u64((elapsed_min << 10), elapsed[cpu]);
		set_capacity_scale(cpu, capacity);
	}

	pr_info("CPUs capacity installed from default\n");
	request_energy_costs_update();
	rebuild_sched_domains();
}

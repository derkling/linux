#define pr_fmt(fmt) "em_debugtest: " fmt

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/energy_model.h>

#define NR_STATES 5

unsigned long freq[] = { 100000, 200000, 300000, 400000, 500000 };
unsigned long power[] = { 100, 400, 800, 1300, 1900 };

int callback(unsigned long *p, unsigned long *f, int cpu)
{
	int i = 0;

	while ((i < NR_STATES - 1) && (freq[i] < *f))
		i++;

	*f = freq[i];
	*p = power[i];

	return 0;
}

cpumask_t visited_cpus;

static int __init em_debugtest_init(void) {
	struct em_data_callback em_cb = EM_DATA_CB(callback);
	struct cpufreq_policy *policy;
	int cpu, ret;

	pr_info("%s: Registering data in the EM Framework\n", __func__);

	cpumask_copy(&visited_cpus, cpu_possible_mask);

	while(!cpumask_empty(&visited_cpus)) {
		cpu = cpumask_first(&visited_cpus);
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			pr_info("%s: CPU%d: no policy\n", __func__, cpu);
			continue;
		}

		pr_info("%s: policy of CPU%d: %*pbl\n", __func__, cpu,
						cpumask_pr_args(policy->cpus));

		ret = em_register_freq_domain(policy->cpus, NR_STATES, &em_cb, 1);
		pr_info("%s: EM registration exited with output: %d\n",
				__func__, ret);

		cpumask_xor(&visited_cpus, &visited_cpus, policy->cpus);
	}

	return 0;
}

late_initcall(em_debugtest_init);

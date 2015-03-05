#ifndef __DT_IDLE_STATES
#define __DT_IDLE_STATES

void dt_probe_idle_affinity(cpumask_t *possible_mask);

int dt_init_idle_driver(struct cpuidle_driver *drv,
			const struct of_device_id *matches,
			unsigned int start_idx);
#endif

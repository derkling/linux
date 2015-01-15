/*
 * irq.c - the irq governor
 *
 * Copyright (C) 2014 Daniel Lezcano <daniel.lezcano@linaro.org>
 *
*/
#include <linux/ktime.h>
#include <linux/irq.h>
#include <linux/cpuidle.h>

static int select(struct cpuidle_driver *drv, struct cpuidle_device *dev,
		  int latency_req, s64 next_timer_event)
{
	s64 next_irq_event = irqt_get_next_prediction(dev->cpu);
	s64 next_event = next_irq_event ? 
		min(next_irq_event, next_timer_event) : next_timer_event; 

	return cpuidle_find_state(drv, dev, next_event, latency_req);
}

static struct cpuidle_governor irq_governor = {
	.name   = "irq",
	.rating = 30,
	.select = select,
	.owner  = THIS_MODULE,
};

static int __init irq_init(void)
{
	return cpuidle_register_governor(&irq_governor);
}

postcore_initcall(irq_init);

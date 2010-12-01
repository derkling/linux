/*
 *  linux/arch/arm/common/timer-sp.c
 *
 *  Copyright (C) 1999 - 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/hardware/arm_timer.h>

struct sp804_clocksource {
	struct clocksource	cs;
	void __iomem		*base;
};

struct sp804_clock_event_device {
	struct clock_event_device dev;
	void __iomem		*base;
};

#define to_sp804_clocksource(x) container_of((x), struct sp804_clocksource, cs)
#define to_sp804_clock_event_device(x) container_of((x), struct sp804_clock_event_device, dev);

/*
 * These timers are currently always setup to be clocked at 1MHz.
 */
#define TIMER_FREQ_HZ	(1000000)

static unsigned long sp804_clksrc_rate = TIMER_FREQ_HZ;

static cycle_t sp804_read(struct clocksource *cs)
{
	struct sp804_clocksource* sp804_cs = to_sp804_clocksource(cs);
	return ~readl(sp804_cs->base + TIMER_VALUE);
}

void __init sp804_clocksource_init(void __iomem *base, char *clk_id)
{
	struct clk *clk;
	struct sp804_clocksource *cs = kzalloc(sizeof(struct sp804_clocksource), GFP_KERNEL);

	if (!cs)
		return;

	cs->cs.name = clk_id ? clk_id : "timer-sp";
	cs->cs.rating = 200;
	cs->cs.read = sp804_read;
	cs->cs.mask = CLOCKSOURCE_MASK(32);
	cs->cs.shift = 20;
	cs->cs.flags = CLOCK_SOURCE_IS_CONTINUOUS;
	cs->base = base;

	clk = clk_get_sys("sp804", clk_id);
	if (!IS_ERR(clk)) {
		sp804_clksrc_rate = clk_get_rate(clk);
	}
	printk(KERN_INFO "SP804: initialising %s clock with rate %ld\n", cs->cs.name, sp804_clksrc_rate);

	/* setup timer 0 as free-running clocksource */
	writel(0, base + TIMER_CTRL);
	writel(0xffffffff, base + TIMER_LOAD);
	writel(0xffffffff, base + TIMER_VALUE);
	writel(TIMER_CTRL_32BIT | TIMER_CTRL_ENABLE | TIMER_CTRL_PERIODIC,
		base + TIMER_CTRL);

	clocksource_register_khz(&cs->cs, sp804_clksrc_rate / 1000);
}


static unsigned long sp804_clkevt_rate = TIMER_FREQ_HZ;

/*
 * IRQ handler for the timer
 */
static irqreturn_t sp804_timer_interrupt(int irq, void *dev_id)
{
	unsigned long sp804mis;
	struct clock_event_device *evt = dev_id;
	struct sp804_clock_event_device *sp804_evt = to_sp804_clock_event_device(evt);

	sp804mis = readl(sp804_evt->base + TIMER_MIS);

	if (sp804mis) {
		/* clear the interrupt */
		writel(1, sp804_evt->base + TIMER_INTCLR);

		evt->event_handler(evt);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void sp804_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evt)
{
	unsigned long ctrl = TIMER_CTRL_32BIT | TIMER_CTRL_IE;
	struct sp804_clock_event_device *sp804_evt = to_sp804_clock_event_device(evt);

	writel(ctrl, sp804_evt->base + TIMER_CTRL);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(sp804_clkevt_rate / HZ, sp804_evt->base + TIMER_LOAD);
		ctrl |= TIMER_CTRL_PERIODIC | TIMER_CTRL_ENABLE;
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl |= TIMER_CTRL_ONESHOT;
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		break;
	}

	writel(ctrl, sp804_evt->base + TIMER_CTRL);
}

static int sp804_set_next_event(unsigned long next,
	struct clock_event_device *evt)
{
	unsigned long ctrl;
	struct sp804_clock_event_device *sp804_evt = to_sp804_clock_event_device(evt);

	ctrl = readl(sp804_evt->base + TIMER_CTRL);
	writel(next, sp804_evt->base + TIMER_LOAD);
	writel(ctrl | TIMER_CTRL_ENABLE, sp804_evt->base + TIMER_CTRL);

	return 0;
}

void __init sp804_clockevents_init(void __iomem *base, unsigned int timer_irq,
				   char *clk_id)
{
	struct clk *clk;
	struct sp804_clock_event_device *evt;
	struct irqaction *sp804_timer_irq;

	evt = kzalloc(sizeof(struct sp804_clock_event_device), GFP_KERNEL);
	if (!evt)
		return;

	sp804_timer_irq = kzalloc(sizeof(struct irqaction), GFP_KERNEL);
	if (!sp804_timer_irq) {
		kfree(evt);
		return;
	}

	evt->dev.name = clk_id ? clk_id : "timer-sp";
	evt->dev.shift = 32;
	evt->dev.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	evt->dev.set_mode = sp804_set_mode;
	evt->dev.set_next_event = sp804_set_next_event;
	evt->dev.rating = 300;
	evt->dev.cpumask = cpu_all_mask;
	evt->base = base;

	clk = clk_get_sys("sp804", clk_id);
	if (!IS_ERR(clk))
		sp804_clkevt_rate = clk_get_rate(clk);

	evt->dev.irq = timer_irq;
	evt->dev.mult = div_sc(sp804_clkevt_rate / 1000, NSEC_PER_MSEC, evt->dev.shift);
	evt->dev.max_delta_ns = clockevent_delta2ns(0xffffffff, &evt->dev);
	evt->dev.min_delta_ns = clockevent_delta2ns(0xf, &evt->dev);

	sp804_timer_irq->name = "timer";
	sp804_timer_irq->flags = IRQF_TIMER | IRQF_IRQPOLL | IRQF_SHARED | IRQF_PROBE_SHARED;
	sp804_timer_irq->handler = sp804_timer_interrupt;
	sp804_timer_irq->dev_id = evt;

	setup_irq(timer_irq, sp804_timer_irq);
	clockevents_register_device(&evt->dev);
}

#undef DEBUG

/*
 * ARM bandwidth monitors support.
 *
 * Copyright (C) 2010 ARM Ltd.
 */
#define pr_fmt(fmt) "hw perfevents: " fmt

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <asm/pmu.h>
#include <asm/io.h>

#define HW_OP_UNSUPPORTED		0xFFFF
static struct platform_device *pmu_device;

/* Regs definitions.  */
#define ARMBWL_PHYBASE                  0xE001A000
/* MTU regs.  */
#define MTU(idx, reg) ((idx * 0x100) + reg)
#define MTVER      0x100 /* Trace unit version number register.  */
#define MTCTRL     0x104 /* Trace unit control register.  */
#define MTFILT     0x108 /* Filter configuration register.  */
#define MTFILTM    0x10C /* Filter mask register.  */
#define MTADDRH    0x110 /* Address range high limit filter register.  */
#define MTADDRL    0x114 /* Address range low limit filter register.  */
#define MTLATT     0x118 /* Latency threshold register.  */
#define MTLATM     0x11C /* Min/Max latency register.  */
#define MTLATS     0x120 /* Above latency threshold latency sum shadow register.  */
#define MTLATC     0x124 /* Above latency threshold transaction count shadow register.  */
#define MTXACTC    0x128 /* Total transaction count shadow register.  */
#define MTBWC      0x12C /* Clock cycles with active data transfer count shadow register.  */
/* MTCTRL regs.  */
#define MTCVER     0xE00 /* Control unit version number register.  */
#define MTCCTRL    0xE04 /* Control unit control register.  */
#define MTCWIND    0xE08 /* Sampling window duration register.  */
#define MTTCFG     0xE0C /* Trace interface (ATB) configuration register.  */
#define MTPMC00    0xE10 /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC01    0xE14 /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC02    0xE18 /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC03    0xE1C /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC04    0xE20 /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC05    0xE24 /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC06    0xE28 /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC07    0xE2C /* PMUEVENT bus signals counters (1st core).  */
#define MTPMC10    0xE30 /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMC11    0xE34 /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMC12    0xE38 /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMC13    0xE3C /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMC14    0xE40 /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMC15    0xE44 /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMC16    0xE48 /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMC17    0xE4C /* PMUEVENT bus signals counters (2nd core).  */
#define MTPMSEL0   0xE50 /* PMUEVENT signals configuration register (1st core).  */
#define MTPMSEL1   0xE54 /* PMUEVENT signals configuration register (2nd core).  */
#define ITATBCTR0  0xEF8 /* Integration test ATB control register.  */
#define ITCTRL     0xF00 /* Integration mode control register.  */
#define AUTHSTATUS 0xFB8 /* Authentication status register.  */
#define DEVTYPE    0xFCC /* Device type identifier register.  */
#define PID4       0xFD0 /* Peripheral ID4 Register.  */
#define PID5       0xFD4 /* Peripheral ID4 Register.  */
#define PID6       0xFD8 /* Peripheral ID5 Register.  */
#define PID7       0xFDC /* Peripheral ID6 Register.  */
#define PID0       0xFE0 /* Peripheral ID0 Register.  */
#define PID1       0xFE4 /* Peripheral ID1 Register.  */
#define PID2       0xFE8 /* Peripheral ID2 Register.  */
#define PID3       0xFEC /* Peripheral ID3 Register.  */
#define ID0        0xFF0 /* Component ID0 Register.  */
#define ID1        0xFF4 /* Component ID1 Register.  */
#define ID2        0xFF8 /* Component ID2 Register.  */
#define ID3        0xFFC /* Component ID3 Register.  */
/* ---- ---- ----  */

enum bwl_perf_types {
	BWL_PERFCTR_BR_EXEC		    = 0x5,
	BWL_PERFCTR_BR_MISPREDICT	    = 0x6,
	BWL_PERFCTR_INSTR_EXEC	            = 0x7,
	BWL_PERFCTR_CPU_CYCLES	            = 0xFF,
};

struct arm_pmu {
	enum arm_perf_pmu_ids id;
	irqreturn_t	(*handle_irq)(int irq_num, void *dev);
	void		(*enable)(struct hw_perf_event *evt, int idx);
	void		(*disable)(struct hw_perf_event *evt, int idx);
	int		(*event_map)(int evt);
	u64		(*raw_event)(u64);
	u32		(*read_counter)(int idx);
	void		(*write_counter)(int idx, u32 val);
	void		(*start)(void);
	void		(*stop)(void);
	int		num_events;
	u64		max_period;
};

static struct bwl_info {
	const struct pmu *pmu;
	const struct arm_pmu *armpmu;
	void __iomem *base;
} *info;

static const unsigned bwl_perf_map[PERF_COUNT_HW_MAX] = {
	[PERF_COUNT_HW_CPU_CYCLES]	    = BWL_PERFCTR_CPU_CYCLES,
	[PERF_COUNT_HW_INSTRUCTIONS]	    = BWL_PERFCTR_INSTR_EXEC,
	[PERF_COUNT_HW_CACHE_REFERENCES]    = HW_OP_UNSUPPORTED,
	[PERF_COUNT_HW_CACHE_MISSES]	    = HW_OP_UNSUPPORTED,
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = BWL_PERFCTR_BR_EXEC,
	[PERF_COUNT_HW_BRANCH_MISSES]	    = BWL_PERFCTR_BR_MISPREDICT,
	[PERF_COUNT_HW_BUS_CYCLES]	    = HW_OP_UNSUPPORTED,
};


static irqreturn_t bwl_handle_irq(int irq_num, void *dev)
{
	printk(KERN_ERR "%s\n", __func__);

	return IRQ_HANDLED;
}

void bwl_enable_event(struct hw_perf_event *hwc, int idx)
{
	printk(KERN_ERR "%s\n", __func__);
}

static void bwl_disable_event(struct hw_perf_event *hwc, int idx)
{
	printk(KERN_ERR "%s\n", __func__);
}

static inline int bwl_event_map(int config)
{
	int mapping = bwl_perf_map[config];

	printk(KERN_ERR "%s\n", __func__);

	if (HW_OP_UNSUPPORTED == mapping)
		mapping = -EOPNOTSUPP;
	return mapping;
}

static u64 bwl_raw_event(u64 config)
{
	printk(KERN_ERR "%s\n", __func__);
	return config & 0xff;
}

static inline u32 bwl_read_counter(int idx)
{
	unsigned long value = 0;

	/* Read counter.  */
	printk(KERN_ERR "%s\n", __func__);

	return value;
}

static inline void bwl_write_counter(int idx, u32 value)
{
	printk(KERN_ERR "%s\n", __func__);
	/* Write counter.  */
}

static void bwl_start(void)
{
	printk(KERN_ERR "%s\n", __func__);
	/* Start.  */
}

static void bwl_stop(void)
{
	printk(KERN_ERR "%s\n", __func__);
	/* Stop.  */
}

static const struct arm_pmu bwl = {
	.id			= ARM_PERF_PMU_ID_BWL,
	.handle_irq		= bwl_handle_irq,
	.enable			= bwl_enable_event,
	.disable		= bwl_disable_event,
	.event_map		= bwl_event_map,
	.raw_event		= bwl_raw_event,
	.read_counter		= bwl_read_counter,
	.write_counter		= bwl_write_counter,
	.start			= bwl_start,
	.stop			= bwl_stop,
	.num_events		= 3,
	.max_period		= (1LLU << 32) - 1,
};

enum arm_perf_pmu_ids armpmu_get_pmu_id(void)
{
	return bwl.id;
}
EXPORT_SYMBOL_GPL(armpmu_get_pmu_id);

int armpmu_get_max_events(void)
{
	return bwl.num_events;
}
EXPORT_SYMBOL_GPL(armpmu_get_max_events);

static int __init init_hw_perf_events(void)
{
	printk(KERN_ERR "%s\n", __func__);

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* Init armpmu_info struct.  */
	info->base = ioremap_nocache(ARMBWL_PHYBASE, SZ_4K);
	if (!info->base) {
		kfree(info);
		printk(KERN_ERR "armbwl: can't ioremap registers.\n");
		perf_max_events = -1;
		goto out;
	}

	info->armpmu = &bwl;
	perf_max_events = bwl.num_events;

	if (info->armpmu) {
		pr_info("bwl enabled, %d counters available\n", perf_max_events);
	} else {
		pr_info("no hardware support available\n");
		perf_max_events = -1;
	}

out:
	return 0;
}
arch_initcall(init_hw_perf_events);

static void armpmu_disable(struct perf_event *event)
{
	printk(KERN_ERR "%s\n", __func__);
	/* Disable.  */
}

static void armpmu_read(struct perf_event *event)
{
	printk(KERN_ERR "%s\n", __func__);
	/* Read.  */
}

static void armpmu_unthrottle(struct perf_event *event)
{
	printk(KERN_ERR "%s\n", __func__);
	/* Unthrottle.  */
}

static int armpmu_enable(struct perf_event *event)
{
	int err = 0;

	printk(KERN_ERR "%s\n", __func__);

	/* Enable.  */

	return err;
}

static struct pmu pmu = {
	.enable	    = armpmu_enable,
	.disable    = armpmu_disable,
	.unthrottle = armpmu_unthrottle,
	.read	    = armpmu_read,
};
static atomic_t active_events = ATOMIC_INIT(0);
static DEFINE_MUTEX(pmu_reserve_mutex);

static int armpmu_reserve_hardware(void)
{
	const struct arm_pmu *armpmu = info->armpmu;
	int i, err = -ENODEV, irq;

	printk(KERN_ERR "%s\n", __func__);

	pmu_device = reserve_pmu(ARM_PMU_DEVICE_BWL);
	if (IS_ERR(pmu_device)) {
		pr_warning("unable to reserve pmu\n");
		return PTR_ERR(pmu_device);
	}

	init_pmu(ARM_PMU_DEVICE_BWL);

	if (pmu_device->num_resources < 1) {
		pr_err("no irqs for PMUs defined\n");
		return -ENODEV;
	}

	for (i = 0; i < pmu_device->num_resources; ++i) {
		irq = platform_get_irq(pmu_device, i);
		if (irq < 0)
			continue;

		err = request_irq(irq, armpmu->handle_irq,
				  IRQF_DISABLED | IRQF_NOBALANCING,
				  "armpmu", NULL);
		if (err) {
			pr_warning("unable to request IRQ%d for ARM perf "
				"counters\n", irq);
			break;
		}
	}

	if (err) {
		for (i = i - 1; i >= 0; --i) {
			irq = platform_get_irq(pmu_device, i);
			if (irq >= 0)
				free_irq(irq, NULL);
		}
		release_pmu(pmu_device);
		pmu_device = NULL;
	}

	return err;
}

static void armpmu_release_hardware(void)
{
	const struct arm_pmu *armpmu = info->armpmu;
	int i, irq;

	printk(KERN_ERR "%s\n", __func__);

	for (i = pmu_device->num_resources - 1; i >= 0; --i) {
		irq = platform_get_irq(pmu_device, i);
		if (irq >= 0)
			free_irq(irq, NULL);
	}
	armpmu->stop();

	release_pmu(pmu_device);
	pmu_device = NULL;
}


static void hw_perf_event_destroy(struct perf_event *event)
{
	printk(KERN_ERR "%s\n", __func__);

	if (atomic_dec_and_mutex_lock(&active_events, &pmu_reserve_mutex)) {
		armpmu_release_hardware();
		mutex_unlock(&pmu_reserve_mutex);
	}
}

static int __hw_perf_event_init(struct perf_event *event)
{
	const struct arm_pmu *armpmu = info->armpmu;
	struct hw_perf_event *hwc = &event->hw;
	int mapping, err;

	printk(KERN_ERR "%s\n", __func__);

	/* Decode the generic type into an ARM event identifier. */
	if (PERF_TYPE_HARDWARE == event->attr.type) {
		mapping = armpmu->event_map(event->attr.config);
	} else {
		pr_debug("event type %x not supported\n", event->attr.type);
		return -EOPNOTSUPP;
	}

	if (mapping < 0) {
		pr_debug("event %x:%llx not supported\n", event->attr.type,
			 event->attr.config);
		return mapping;
	}

	/*
	 * Check whether we need to exclude the counter from certain modes.
	 * The ARM performance counters are on all of the time so if someone
	 * has asked us for some excludes then we have to fail.
	 */
	if (event->attr.exclude_kernel || event->attr.exclude_user ||
	    event->attr.exclude_hv || event->attr.exclude_idle) {
		pr_debug("ARM performance counters do not support "
			 "mode exclusion\n");
		return -EPERM;
	}

	/*
	 * We don't assign an index until we actually place the event onto
	 * hardware. Use -1 to signify that we haven't decided where to put it
	 * yet. For SMP systems, each core has it's own PMU so we can't do any
	 * clever allocation or constraints checking at this point.
	 */
	hwc->idx = -1;

	/*
	 * Store the event encoding into the config_base field. config and
	 * event_base are unused as the only 2 things we need to know are
	 * the event mapping and the counter to use. The counter to use is
	 * also the indx and the config_base is the event type.
	 */
	hwc->config_base	    = (unsigned long)mapping;
	hwc->config		    = 0;
	hwc->event_base		    = 0;

	if (!hwc->sample_period) {
		hwc->sample_period  = armpmu->max_period;
		hwc->last_period    = hwc->sample_period;
		// local64_set(&hwc->period_left, hwc->sample_period);
	}

	err = 0;
	if (event->group_leader != event) // Sure about this?!?
		return -EINVAL;

	return err;
}

const struct pmu *hw_perf_event_init(struct perf_event *event)
{
	const struct arm_pmu *armpmu = info->armpmu;
	int err = 0;

	printk(KERN_ERR "%s\n", __func__);

	if (!armpmu)
		return ERR_PTR(-ENODEV);

	if (!atomic_inc_not_zero(&active_events)) {
		if (atomic_read(&active_events) > perf_max_events) {
			atomic_dec(&active_events);
			return ERR_PTR(-ENOSPC);
		}

		mutex_lock(&pmu_reserve_mutex);
		if (atomic_read(&active_events) == 0) {
			err = armpmu_reserve_hardware();
		}

		if (!err)
			atomic_inc(&active_events);
		mutex_unlock(&pmu_reserve_mutex);
	}

	if (err)
		return ERR_PTR(err);

	err = __hw_perf_event_init(event);
	if (err) {
		hw_perf_event_destroy(event);
		return ERR_PTR(err);
	}
	info->pmu = &pmu;

	return info->pmu;
}

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2015 ARM Limited
 */

#define pr_fmt(fmt) "psci: " fmt

#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/cpuidle.h>
#include <linux/errno.h>
#include <linux/linkage.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/printk.h>
#include <linux/psci.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include <linux/memblock.h>
#include <linux/mman.h>
#include <linux/io.h>

#include <linux/mm.h>
#include <uapi/linux/psci.h>

#include <asm/cpuidle.h>
#include <asm/cputype.h>
#include <asm/sysreg.h>
#include <asm/system_misc.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

/*
 * While a 64-bit OS can make calls with SMC32 calling conventions, for some
 * calls it is necessary to use SMC64 to pass or return 64-bit values.
 * For such calls PSCI_FN_NATIVE(version, name) will choose the appropriate
 * (native-width) function ID.
 */
#ifdef CONFIG_64BIT
#define PSCI_FN_NATIVE(version, name)	PSCI_##version##_FN64_##name
#else
#define PSCI_FN_NATIVE(version, name)	PSCI_##version##_FN_##name
#endif

/*
 * The CPU any Trusted OS is resident on. The trusted OS may reject CPU_OFF
 * calls to its resident CPU, so we must avoid issuing those. We never migrate
 * a Trusted OS even if it claims to be capable of migration -- doing so will
 * require cooperation with a Trusted OS driver.
 */
static int resident_cpu = -1;

bool psci_tos_resident_on(int cpu)
{
	return cpu == resident_cpu;
}

struct psci_operations psci_ops;

typedef unsigned long (psci_fn)(unsigned long, unsigned long,
				unsigned long, unsigned long);
static psci_fn *invoke_psci_fn;

enum psci_function {
	PSCI_FN_CPU_SUSPEND,
	PSCI_FN_CPU_ON,
	PSCI_FN_CPU_OFF,
	PSCI_FN_MIGRATE,
	PSCI_FN_MAX,
};

static u32 psci_function_id[PSCI_FN_MAX];

#define PSCI_0_2_POWER_STATE_MASK		\
				(PSCI_0_2_POWER_STATE_ID_MASK | \
				PSCI_0_2_POWER_STATE_TYPE_MASK | \
				PSCI_0_2_POWER_STATE_AFFL_MASK)

#define PSCI_1_0_EXT_POWER_STATE_MASK		\
				(PSCI_1_0_EXT_POWER_STATE_ID_MASK | \
				PSCI_1_0_EXT_POWER_STATE_TYPE_MASK)

static u32 psci_cpu_suspend_feature;

static inline bool psci_has_ext_power_state(void)
{
	return psci_cpu_suspend_feature &
				PSCI_1_0_FEATURES_CPU_SUSPEND_PF_MASK;
}

static inline bool psci_power_state_loses_context(u32 state)
{
	const u32 mask = psci_has_ext_power_state() ?
					PSCI_1_0_EXT_POWER_STATE_TYPE_MASK :
					PSCI_0_2_POWER_STATE_TYPE_MASK;

	return state & mask;
}

static inline bool psci_power_state_is_valid(u32 state)
{
	const u32 valid_mask = psci_has_ext_power_state() ?
			       PSCI_1_0_EXT_POWER_STATE_MASK :
			       PSCI_0_2_POWER_STATE_MASK;

	return !(state & ~valid_mask);
}

static unsigned long __invoke_psci_fn_hvc(unsigned long function_id,
			unsigned long arg0, unsigned long arg1,
			unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_hvc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return res.a0;
}

static unsigned long __invoke_psci_fn_smc(unsigned long function_id,
			unsigned long arg0, unsigned long arg1,
			unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return res.a0;
}

static int psci_to_linux_errno(int errno)
{
	switch (errno) {
	case PSCI_RET_SUCCESS:
		return 0;
	case PSCI_RET_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	case PSCI_RET_INVALID_PARAMS:
	case PSCI_RET_INVALID_ADDRESS:
		return -EINVAL;
	case PSCI_RET_DENIED:
		return -EPERM;
	};

	return -EINVAL;
}

/* physical address, hardcoded value
 * XXX VALUE CHANGES MUST BE SYNCHRONIZED WITH ATF
 */
static uint32_t memPointer = 0xf0000000;
/* shared memory size */
static uint32_t reserveSize =  0x1000;

#define MPIDR_CLUSTER_MASK 0xff00
#define MPIDR_CPU_MASK 0xff

inline static uint64_t read_mpidr(void)
{
    return read_sysreg(mpidr_el1);
}

/*
 * return linear sequencial index as a function of the mpidr parameter
 * Verbatim copy of the same code in ATG XXX changes must be synchronized with AFT
 */
static inline uint32_t getLinearCpuId(uint64_t mpidr)
{
    uint32_t clusterId = (mpidr & MPIDR_CLUSTER_MASK)>>8;
    uint32_t coreId = (mpidr & MPIDR_CPU_MASK);

    uint32_t clusterOffset = clusterId ? 2:0;

    return coreId + clusterOffset;
}

/*
 * return linear sequencial index as a function of the mpidr
 * Verbatim copy of the same code in ATG XXX changes must be synchronized with AFT
 */
static inline uint32_t getCpuId(void)
{
    uint64_t mpidr = read_mpidr();

    return getLinearCpuId(mpidr);
}

/*
 * structure of the data stored in the shared memory page.
 * XXX changes must be synchronized with ATF
 * XXX pay attention to alignment/padding rules in order not to fall into inconsistent states
 */
struct CounterData
{
    uint64_t mpidrValue;

    uint64_t constantFrequencyBeforeSleep;
    uint64_t constantFrequencyOffset;

    uint64_t cycleCounterBeforeSleep;

    uint64_t retiredInstructionsBeforeSleepH;
    uint64_t retiredInstructionsBeforeSleepL;
};

/*
 * virtual address where the shared memory page is mapped onto
 */
static struct CounterData *virtAddress = NULL;

#define PSCI_DEBUG 0
/* debug related enum */
enum DebugCallee {
    SUSPEND,
    CPU_ON,
    CPU_OFF,
    WAKE_UP,
    SYS_SUSPEND,
    SYS_OFF,
    BLANK
};

/* debug related array of "strings" */
static char *calleeNames[] = {"S", "ON", "OFF", "W", "SYS_SUSP", "SYS_OFF", " "};

uint64_t get_constant_frequency_counter_debug(enum DebugCallee debug_callee, uint32_t state )
{
    uint64_t mpidr = read_mpidr();
    uint64_t linearCpuId = getLinearCpuId(mpidr);
    uint64_t offset = virtAddress[linearCpuId ].constantFrequencyOffset;

    volatile uint64_t counterValue =  read_sysreg(cntpct_el0) - offset;

#if PSCI_DEBUG
    /* "debug infrastructure" */
    if(linearCpuId == 0)
    {
        /*uint64_t freq = read_sysreg(cntfrq_el0);*/
        uint64_t cycleCounter = read_sysreg(pmccntr_el0);
        static uint64_t lastOffset = 0;

        if(state != 0xffffffff)
        {
            static uint64_t lastCycleValue = 0;
            static uint64_t lastConstValue = 0;
            pr_info("id %llx %s s %x const=%llx cycle=%llx  ratio %lld part %lld\n", mpidr, calleeNames[debug_callee], state, counterValue, cycleCounter, (cycleCounter*1000)/counterValue, ((cycleCounter-lastCycleValue)*1000)/(counterValue - lastConstValue)  );
            lastCycleValue= cycleCounter;
            lastConstValue= counterValue;
        }
        else
        {
            pr_info("id %llx %s const=%llx cycle=%llx  ratio %lld\n", mpidr, calleeNames[debug_callee], counterValue, cycleCounter, (cycleCounter*1000)/counterValue );
        }

        lastOffset = offset;
    }
#endif

    return counterValue;
}

/* sample the constant frequency counter */
volatile uint64_t get_constant_frequency_counter(void)
{
 #if CONFIG_GEM5_ACTMON
    return read_sysreg(pmevcntr0_el0);
#elif CONFIG_JUNO_ACTMON
    return get_constant_frequency_counter_debug(BLANK, 0);
#else
    panic("ACTMON not set to gem5 nor Juno");
#endif
}

/* sample the cpu cycle counter */
uint64_t get_core_cycle_counter(void)
{
#if CONFIG_GEM5_ACTMON
    return read_sysreg(pmccntr_el0);
#elif CONFIG_JUNO_ACTMON
    return read_sysreg(pmccntr_el0);
#else
    panic("ACTMON not set to gem5 nor Juno");
#endif
}

/* sample numer of instructions retired counter */
uint64_t get_instructions_retired_counter(void)
{
 #if CONFIG_GEM5_ACTMON
    return read_sysreg(pmevcntr1_el0);
#elif CONFIG_JUNO_ACTMON
    return (((uint64_t)read_sysreg(pmevcntr1_el0))<<32) | read_sysreg(pmevcntr0_el0);
#else
    panic("ACTMON not set to gem5 nor Juno");
#endif
}

/* sample memory stall cycles - only available on gem5 */
uint64_t get_memory_stall_cycle_counter(void)
{
 #if CONFIG_GEM5_ACTMON
    return read_sysreg(pmevcntr2_el0);
#elif CONFIG_JUNO_ACTMON
    panic("backend mem stall not present on Juno");
#else
    panic("ACTMON not set to gem5 nor Juno");
#endif
}

void debugCounters(void)
{
    uint32_t cpuId = getCpuId();
    if (cpuId == 0)
    {   
        uint64_t cycleCounter     = get_core_cycle_counter();
        uint64_t constFreqCounter = get_constant_frequency_counter();
        uint64_t instRetired      = get_instructions_retired_counter();
        
        static uint64_t lastCycleValue;
        static uint64_t lastConstValue;
        
        pr_info("id %llx   const=%llx cycle=%llx  ratio %lld part %lld\n", cpuId, constFreqCounter, cycleCounter, (cycleCounter*1000)/constFreqCounter, ((cycleCounter-lastCycleValue)*1000)/(constFreqCounter - lastConstValue)  );

        lastConstValue = constFreqCounter;
        lastCycleValue = cycleCounter;
    }
}

static u32 psci_get_version(void)
{

#if CONFIG_GEM5_ACTMON
    /* Initialize PMU configuration
     * enable bit in pmcr
     * enable pmcntenset_el0 
     * The PMEVTYPER0 is set to the constant freq event 0x4004
     */
    uint32_t pmcrVal;
    write_sysreg(0x80000007, pmcntenset_el0);
    pmcrVal = read_sysreg(pmcr_el0);
    write_sysreg(pmcrVal | 1, pmcr_el0);
    
    write_sysreg(0x4004, pmevtyper0_el0);
    write_sysreg(0x8, pmevtyper1_el0);
    write_sysreg(0x4005, pmevtyper2_el0);
#endif

    pr_info("attempt reserve %X", memPointer);

    virtAddress = memremap(memPointer, reserveSize ,MEMREMAP_WB);
    pr_info("allocated address 0x%llX\n", (uint64_t)virtAddress);
    if(!virtAddress)
    {
        panic("failed to allocate the intended physical address\n");
    }

	return invoke_psci_fn(PSCI_0_2_FN_PSCI_VERSION, 0, 0, 0);
}

static int psci_cpu_suspend(u32 state, unsigned long entry_point)
{
	int err;
	u32 fn;

    /*debugCounters();*/
	fn = psci_function_id[PSCI_FN_CPU_SUSPEND];
	err = invoke_psci_fn(fn, state, entry_point, 0);

    /*debugCounters();*/

	return psci_to_linux_errno(err);
}

static int psci_cpu_off(u32 state)
{
	int err;
	u32 fn;


	fn = psci_function_id[PSCI_FN_CPU_OFF];
	err = invoke_psci_fn(fn, state, 0, 0);
	return psci_to_linux_errno(err);
}

static int psci_cpu_on(unsigned long cpuid, unsigned long entry_point)
{
	int err;
	u32 fn;

	fn = psci_function_id[PSCI_FN_CPU_ON];
	err = invoke_psci_fn(fn, cpuid, entry_point, 0);

	return psci_to_linux_errno(err);
}

static int psci_migrate(unsigned long cpuid)
{
	int err;
	u32 fn;

	fn = psci_function_id[PSCI_FN_MIGRATE];
	err = invoke_psci_fn(fn, cpuid, 0, 0);
	return psci_to_linux_errno(err);
}

static int psci_affinity_info(unsigned long target_affinity,
		unsigned long lowest_affinity_level)
{
	return invoke_psci_fn(PSCI_FN_NATIVE(0_2, AFFINITY_INFO),
			      target_affinity, lowest_affinity_level, 0);
}

static int psci_migrate_info_type(void)
{
	return invoke_psci_fn(PSCI_0_2_FN_MIGRATE_INFO_TYPE, 0, 0, 0);
}

static unsigned long psci_migrate_info_up_cpu(void)
{
	return invoke_psci_fn(PSCI_FN_NATIVE(0_2, MIGRATE_INFO_UP_CPU),
			      0, 0, 0);
}

static int get_set_conduit_method(struct device_node *np)
{
	const char *method;

	pr_info("probing for conduit method from DT.\n");

	if (of_property_read_string(np, "method", &method)) {
		pr_warn("missing \"method\" property\n");
		return -ENXIO;
	}

	if (!strcmp("hvc", method)) {
		invoke_psci_fn = __invoke_psci_fn_hvc;
	} else if (!strcmp("smc", method)) {
		invoke_psci_fn = __invoke_psci_fn_smc;
	} else {
		pr_warn("invalid \"method\" property: %s\n", method);
		return -EINVAL;
	}
	return 0;
}

static void psci_sys_reset(enum reboot_mode reboot_mode, const char *cmd)
{
    pr_info("SYS RESET\n");
	invoke_psci_fn(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0);
}

static void psci_sys_poweroff(void)
{
    pr_info("SYS OFF\n");
	invoke_psci_fn(PSCI_0_2_FN_SYSTEM_OFF, 0, 0, 0);
}

static int __init psci_features(u32 psci_func_id)
{
	return invoke_psci_fn(PSCI_1_0_FN_PSCI_FEATURES,
			      psci_func_id, 0, 0);
}

#ifdef CONFIG_CPU_IDLE
static DEFINE_PER_CPU_READ_MOSTLY(u32 *, psci_power_state);

static int psci_dt_cpu_init_idle(struct device_node *cpu_node, int cpu)
{
	int i, ret, count = 0;
	u32 *psci_states;
	struct device_node *state_node;

	/* Count idle states */
	while ((state_node = of_parse_phandle(cpu_node, "cpu-idle-states",
					      count))) {
		count++;
		of_node_put(state_node);
	}

	if (!count)
		return -ENODEV;

	psci_states = kcalloc(count, sizeof(*psci_states), GFP_KERNEL);
	if (!psci_states)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		u32 state;

		state_node = of_parse_phandle(cpu_node, "cpu-idle-states", i);

		ret = of_property_read_u32(state_node,
					   "arm,psci-suspend-param",
					   &state);
		if (ret) {
			pr_warn(" * %s missing arm,psci-suspend-param property\n",
				state_node->full_name);
			of_node_put(state_node);
			goto free_mem;
		}

		of_node_put(state_node);
		pr_debug("psci-power-state %#x index %d\n", state, i);
		if (!psci_power_state_is_valid(state)) {
			pr_warn("Invalid PSCI power state %#x\n", state);
			ret = -EINVAL;
			goto free_mem;
		}
		psci_states[i] = state;
	}
	/* Idle states parsed correctly, initialize per-cpu pointer */
	per_cpu(psci_power_state, cpu) = psci_states;
	return 0;

free_mem:
	kfree(psci_states);
	return ret;
}

#ifdef CONFIG_ACPI
#include <acpi/processor.h>

static int __maybe_unused psci_acpi_cpu_init_idle(unsigned int cpu)
{
	int i, count;
	u32 *psci_states;
	struct acpi_lpi_state *lpi;
	struct acpi_processor *pr = per_cpu(processors, cpu);

	if (unlikely(!pr || !pr->flags.has_lpi))
		return -EINVAL;

	count = pr->power.count - 1;
	if (count <= 0)
		return -ENODEV;

	psci_states = kcalloc(count, sizeof(*psci_states), GFP_KERNEL);
	if (!psci_states)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		u32 state;

		lpi = &pr->power.lpi_states[i + 1];
		/*
		 * Only bits[31:0] represent a PSCI power_state while
		 * bits[63:32] must be 0x0 as per ARM ACPI FFH Specification
		 */
		state = lpi->address;
		if (!psci_power_state_is_valid(state)) {
			pr_warn("Invalid PSCI power state %#x\n", state);
			kfree(psci_states);
			return -EINVAL;
		}
		psci_states[i] = state;
	}
	/* Idle states parsed correctly, initialize per-cpu pointer */
	per_cpu(psci_power_state, cpu) = psci_states;
	return 0;
}
#else
static int __maybe_unused psci_acpi_cpu_init_idle(unsigned int cpu)
{
	return -EINVAL;
}
#endif

int psci_cpu_init_idle(unsigned int cpu)
{
	struct device_node *cpu_node;
	int ret;

	/*
	 * If the PSCI cpu_suspend function hook has not been initialized
	 * idle states must not be enabled, so bail out
	 */
	if (!psci_ops.cpu_suspend)
		return -EOPNOTSUPP;

	if (!acpi_disabled)
		return psci_acpi_cpu_init_idle(cpu);

	cpu_node = of_get_cpu_node(cpu, NULL);
	if (!cpu_node)
		return -ENODEV;

	ret = psci_dt_cpu_init_idle(cpu_node, cpu);

	of_node_put(cpu_node);

	return ret;
}

static int psci_suspend_finisher(unsigned long index)
{
	u32 *state = __this_cpu_read(psci_power_state);

	return psci_ops.cpu_suspend(state[index - 1],
				    virt_to_phys(cpu_resume));
}

int psci_cpu_suspend_enter(unsigned long index)
{
	int ret;
	u32 *state = __this_cpu_read(psci_power_state);
	/*
	 * idle state index 0 corresponds to wfi, should never be called
	 * from the cpu_suspend operations
	 */
	if (WARN_ON_ONCE(!index))
		return -EINVAL;

	if (!psci_power_state_loses_context(state[index - 1]))
		ret = psci_ops.cpu_suspend(state[index - 1], 0);
	else
		ret = cpu_suspend(index, psci_suspend_finisher);

	return ret;
}

/* ARM specific CPU idle operations */
#ifdef CONFIG_ARM
static const struct cpuidle_ops psci_cpuidle_ops __initconst = {
	.suspend = psci_cpu_suspend_enter,
	.init = psci_dt_cpu_init_idle,
};

CPUIDLE_METHOD_OF_DECLARE(psci, "psci", &psci_cpuidle_ops);
#endif
#endif

static int psci_system_suspend(unsigned long unused)
{

    pr_info("SYS SUSPEND\n");
	return invoke_psci_fn(PSCI_FN_NATIVE(1_0, SYSTEM_SUSPEND),
			      virt_to_phys(cpu_resume), 0, 0);

}

static int psci_system_suspend_enter(suspend_state_t state)
{
	return cpu_suspend(0, psci_system_suspend);
}

static const struct platform_suspend_ops psci_suspend_ops = {
	.valid          = suspend_valid_only_mem,
	.enter          = psci_system_suspend_enter,
};

static void __init psci_init_system_suspend(void)
{
	int ret;

	if (!IS_ENABLED(CONFIG_SUSPEND))
		return;

	ret = psci_features(PSCI_FN_NATIVE(1_0, SYSTEM_SUSPEND));

	if (ret != PSCI_RET_NOT_SUPPORTED)
		suspend_set_ops(&psci_suspend_ops);
}

static void __init psci_init_cpu_suspend(void)
{
	int feature = psci_features(psci_function_id[PSCI_FN_CPU_SUSPEND]);

	if (feature != PSCI_RET_NOT_SUPPORTED)
		psci_cpu_suspend_feature = feature;
}

/*
 * Detect the presence of a resident Trusted OS which may cause CPU_OFF to
 * return DENIED (which would be fatal).
 */
static void __init psci_init_migrate(void)
{
	unsigned long cpuid;
	int type, cpu = -1;

	type = psci_ops.migrate_info_type();

	if (type == PSCI_0_2_TOS_MP) {
		pr_info("Trusted OS migration not required\n");
		return;
	}

	if (type == PSCI_RET_NOT_SUPPORTED) {
		pr_info("MIGRATE_INFO_TYPE not supported.\n");
		return;
	}

	if (type != PSCI_0_2_TOS_UP_MIGRATE &&
	    type != PSCI_0_2_TOS_UP_NO_MIGRATE) {
		pr_err("MIGRATE_INFO_TYPE returned unknown type (%d)\n", type);
		return;
	}

	cpuid = psci_migrate_info_up_cpu();
	if (cpuid & ~MPIDR_HWID_BITMASK) {
		pr_warn("MIGRATE_INFO_UP_CPU reported invalid physical ID (0x%lx)\n",
			cpuid);
		return;
	}

	cpu = get_logical_index(cpuid);
	resident_cpu = cpu >= 0 ? cpu : -1;

	pr_info("Trusted OS resident on physical CPU 0x%lx\n", cpuid);
}

static void __init psci_0_2_set_functions(void)
{
	pr_info("Using standard PSCI v0.2 function IDs\n");
	psci_function_id[PSCI_FN_CPU_SUSPEND] =
					PSCI_FN_NATIVE(0_2, CPU_SUSPEND);
	psci_ops.cpu_suspend = psci_cpu_suspend;

	psci_function_id[PSCI_FN_CPU_OFF] = PSCI_0_2_FN_CPU_OFF;
	psci_ops.cpu_off = psci_cpu_off;

	psci_function_id[PSCI_FN_CPU_ON] = PSCI_FN_NATIVE(0_2, CPU_ON);
	psci_ops.cpu_on = psci_cpu_on;

	psci_function_id[PSCI_FN_MIGRATE] = PSCI_FN_NATIVE(0_2, MIGRATE);
	psci_ops.migrate = psci_migrate;

	psci_ops.affinity_info = psci_affinity_info;

	psci_ops.migrate_info_type = psci_migrate_info_type;

	arm_pm_restart = psci_sys_reset;

	pm_power_off = psci_sys_poweroff;
}

/*
 * Probe function for PSCI firmware versions >= 0.2
 */
static int __init psci_probe(void)
{
	u32 ver = psci_get_version();

	pr_info("PSCIv%d.%d detected in firmware.\n",
			PSCI_VERSION_MAJOR(ver),
			PSCI_VERSION_MINOR(ver));

	if (PSCI_VERSION_MAJOR(ver) == 0 && PSCI_VERSION_MINOR(ver) < 2) {
		pr_err("Conflicting PSCI version detected.\n");
		return -EINVAL;
	}

	psci_0_2_set_functions();

	psci_init_migrate();

	if (PSCI_VERSION_MAJOR(ver) >= 1) {
		psci_init_cpu_suspend();
		psci_init_system_suspend();
	}

	return 0;
}

typedef int (*psci_initcall_t)(const struct device_node *);

/*
 * PSCI init function for PSCI versions >=0.2
 *
 * Probe based on PSCI PSCI_VERSION function
 */
static int __init psci_0_2_init(struct device_node *np)
{
	int err;

	err = get_set_conduit_method(np);

	if (err)
		goto out_put_node;
	/*
	 * Starting with v0.2, the PSCI specification introduced a call
	 * (PSCI_VERSION) that allows probing the firmware version, so
	 * that PSCI function IDs and version specific initialization
	 * can be carried out according to the specific version reported
	 * by firmware
	 */
	err = psci_probe();

out_put_node:
	of_node_put(np);
	return err;
}

/*
 * PSCI < v0.2 get PSCI Function IDs via DT.
 */
static int __init psci_0_1_init(struct device_node *np)
{
	u32 id;
	int err;

	err = get_set_conduit_method(np);

	if (err)
		goto out_put_node;

	pr_info("Using PSCI v0.1 Function IDs from DT\n");

	if (!of_property_read_u32(np, "cpu_suspend", &id)) {
		psci_function_id[PSCI_FN_CPU_SUSPEND] = id;
		psci_ops.cpu_suspend = psci_cpu_suspend;
	}

	if (!of_property_read_u32(np, "cpu_off", &id)) {
		psci_function_id[PSCI_FN_CPU_OFF] = id;
		psci_ops.cpu_off = psci_cpu_off;
	}

	if (!of_property_read_u32(np, "cpu_on", &id)) {
		psci_function_id[PSCI_FN_CPU_ON] = id;
		psci_ops.cpu_on = psci_cpu_on;
	}

	if (!of_property_read_u32(np, "migrate", &id)) {
		psci_function_id[PSCI_FN_MIGRATE] = id;
		psci_ops.migrate = psci_migrate;
	}

out_put_node:
	of_node_put(np);
	return err;
}

static const struct of_device_id psci_of_match[] __initconst = {
	{ .compatible = "arm,psci",	.data = psci_0_1_init},
	{ .compatible = "arm,psci-0.2",	.data = psci_0_2_init},
	{ .compatible = "arm,psci-1.0",	.data = psci_0_2_init},
	{},
};

int __init psci_dt_init(void)
{
	struct device_node *np;
	const struct of_device_id *matched_np;
	psci_initcall_t init_fn;

	np = of_find_matching_node_and_match(NULL, psci_of_match, &matched_np);

	if (!np)
		return -ENODEV;

	init_fn = (psci_initcall_t)matched_np->data;
	return init_fn(np);
}

#ifdef CONFIG_ACPI
/*
 * We use PSCI 0.2+ when ACPI is deployed on ARM64 and it's
 * explicitly clarified in SBBR
 */
int __init psci_acpi_init(void)
{
	if (!acpi_psci_present()) {
		pr_info("is not implemented in ACPI.\n");
		return -EOPNOTSUPP;
	}

	pr_info("probing for conduit method from ACPI.\n");

	if (acpi_psci_use_hvc())
		invoke_psci_fn = __invoke_psci_fn_hvc;
	else
		invoke_psci_fn = __invoke_psci_fn_smc;

	return psci_probe();
}
#endif

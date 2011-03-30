/*
 * Copyright (C) 2008-2010 ARM Limited
 *
 * This software is provided 'as-is', without any express or implied
 * warranties including the implied warranties of satisfactory quality,
 * fitness for purpose or non infringement.  In no event will  ARM be
 * liable for any damages arising from the use of this software.
 *
 * Permission is granted to anyone to use, copy and modify this software for
 * any purpose, and to redistribute the software, subject to the following
 * restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 */
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <asm/thread_info.h>

#define TRUE  1
#define FALSE 0

#define A9_SMP_BIT     (1<<6)
#define MPIDR_U_BIT    (1<<30)

/*
 * Structures we hide from the OS API
 */

DECLARE_PER_CPU(u8, cluster_down);
extern spinlock_t sr_lock;

struct appf_cpu_context {
    u32 flags;
    u32 saved_items;
    u32 *control_data;
    u32 *pmu_data;
    u32 *timer_data;
    u32 *global_timer_data;
    u32 *vfp_data;
    u32 *gic_interface_data;
    u32 *gic_dist_private_data;
    u32 *banked_registers;
    u32 *cp15_data;
    u32 *debug_data;
    u32 *mmu_data;
    u32 *other_data;
};

struct appf_cluster_context {
    u32 saved_items;
    u32 *gic_dist_shared_data;
    u32 *l2_data;
    u32 *scu_data;
};

struct appf_main_table {
    u32 os_mmu_context[4];
    u32 fw_mmu_context[4];
    u32 num_clusters;
    struct appf_cluster            *cluster_table;
};


/*
 * A cluster is a container for CPUs, typically either a single CPU or a
 * coherent cluster.
 * We assume the CPUs in the cluster can be switched off independently.
 */
struct appf_cluster {
    u32 cpu_type;                /* A9mpcore, A5mpcore, etc                  */
    s32 num_cpus;
    s32 active_cpus;    /* Initialized to number of cpus present    */
    u32 scu_address;             /*  0 => no SCU                             */
    u32 ic_address;              /*  0 => no Interrupt Controller            */
    u32 l2_address;              /*  0 => no L2CC                            */
    struct appf_cluster_context *s_context;
    struct appf_cluster_context *ns_context;
    struct appf_cpu *cpu_table;
    u32 power_state;
    void *lock;
    u8  cluster_down;
};

struct appf_cpu {
    u32 ic_address;              /*  0 => no Interrupt Controller            */
    struct appf_cpu_context *s_context;
    struct appf_cpu_context *ns_context;
    struct cpu_context_save *stack;
    volatile u32 power_state;
    volatile u32 cluster_down;
};

struct appf_arch {
	unsigned int		cpu_val;
	unsigned int		cpu_mask;

	int (*init)(void);

	int  (*save_context)(struct appf_cluster *, struct appf_cpu *,
					   unsigned, int);
	int  (*restore_context)(struct appf_cluster *, struct appf_cpu *, int);
	int  (*enter_cstate)(unsigned cpu_index,
					struct appf_cpu *cpu,
					struct appf_cluster *cluster);
	int  (*leave_cstate)(unsigned, struct appf_cpu *,
					struct appf_cluster *);
	int  (*power_up_cpu)(struct appf_cpu *, struct appf_cluster *,
					unsigned int);
	void (*reset)(void);

};
extern struct appf_arch *arch;
void lookup_arch(void);
struct appf_cluster_context;
struct appf_cpu_context;
/*
 * Global variables
 */
extern int is_smp;
extern struct appf_main_table main_table;
extern unsigned int appf_main_table;
extern u32 appf_translation_table1[];
extern u32 appf_translation_table2[];
extern unsigned appf_runtime_call_flat_mapped;
extern unsigned appf_save_flat_mapped;
extern unsigned appf_restore_flat_mapped;
extern unsigned appf_device_memory_flat_mapped;
extern void *context_memory;
extern void *context_memory_uncached;

/*
 * Entry points
 */
typedef u32 (appf_save_context_t)
	(struct appf_cluster *,
	struct appf_cpu*, u32, u32);
typedef u32 (appf_restore_context_t)
	(struct appf_cluster *,
	struct appf_cpu*, u32);

extern appf_save_context_t appf_save_flat_call;
extern appf_restore_context_t appf_restore_flat_call;
extern int appf_warm_reset(struct appf_main_table *);

extern int power_down_cpu(unsigned cstate, unsigned rstate, unsigned flags);
extern int power_up_cpu(unsigned cluster_index, unsigned cpu);
extern struct appf_arch *get_arch(void);
/*
 * Context save/restore
 */
extern int save_cpu_context(struct appf_cluster *cluster, struct appf_cpu *cpu,
			unsigned flags);

extern int linux_appf_setup_translation_tables(void);
extern void *linux_appf_setup_io(void*, unsigned int);
extern int appf_setup_translation_tables(void);
extern int appf_setup_secure_translation_tables(void);

extern int appf_platform_enter_cstate(unsigned cpu_index, struct appf_cpu *cpu,
		struct appf_cluster *cluster);
extern int appf_platform_leave_cstate(unsigned cpu_index, struct appf_cpu *cpu,
		struct appf_cluster *cluster);
extern int appf_platform_a8_power_up_cpu(struct appf_cpu *cpup,
		struct appf_cluster *cluster, unsigned int cpu);
extern int appf_platform_a8_enter_cstate(unsigned cpu_index,
		struct appf_cpu *cpu, struct appf_cluster *cluster);
extern int appf_platform_a8_leave_cstate(unsigned cpu_index,
		struct appf_cpu *cpu, struct appf_cluster *cluster);
extern int appf_platform_a9_power_up_cpu(struct appf_cpu *cpup,
		struct appf_cluster *cluster, unsigned int cpu);
extern int appf_platform_a9_enter_cstate(unsigned cpu_index,
		struct appf_cpu *cpu, struct appf_cluster *cluster);
extern int appf_platform_a9_leave_cstate(unsigned cpu_index,
		struct appf_cpu *cpu, struct appf_cluster *cluster);
extern int appf_platform_enter_cstate1(unsigned cpu_index,
		struct appf_cpu *cpu, struct appf_cluster *cluster);
extern int appf_platform_leave_cstate1(unsigned cpu_index,
		struct appf_cpu *cpu, struct appf_cluster *cluster);

/* Functions in context.c */
extern int appf_platform_a9_restore_context(struct appf_cluster *cluster,
		struct appf_cpu *cpu, int is_secure);
extern int appf_platform_a9_save_context(struct appf_cluster *cluster,
		struct appf_cpu *cpu, unsigned flags, int is_secure);
extern int appf_platform_a8_restore_context(struct appf_cluster *cluster,
		struct appf_cpu *cpu, int is_secure);
extern int appf_platform_a8_save_context(struct appf_cluster *cluster,
		struct appf_cpu *cpu, unsigned flags, int is_secure);

/* Functions in reset.S */
extern void platform_a8_reset_handler(void);
extern void platform_a9_reset_handler(void);

extern struct appf_main_table main_table;

/* Functions in platform.c */
extern int appf_platform_boottime_init(void);
extern int appf_platform_runtime_init(void);
extern int appf_platform_a8_init(void);
extern int appf_platform_a9_init(void);
extern int appf_platform_init(void);
extern int appf_platform_get_cpu_index(void);
extern int appf_platform_get_cluster_index(void);

/* Functions in power.c */
extern int appf_platform_power_up_cpu(struct appf_cpu *cpup,
				struct appf_cluster *cluster,
				unsigned int cpu);

/*
 * This function returns a suitable stack pointer for this CPU.
 *
 * The address returned must be the initial stack pointer - that
 * is, the address of the first byte AFTER the allocated region.
 *
 * The memory must be 8-byte aligned.
 */
extern unsigned appf_platform_get_stack_pointer(void);
extern int appf_platform_get_cpu_index(void);
extern int appf_platform_get_cluster_index(void);
extern unsigned int srvector;


/*
 * A 4KB page that will be mapped as DEVICE memory
 * Locks must be allocated here.
 */
extern unsigned *appf_device_memory;

extern unsigned platform_cpu_stacks[NR_CPUS];
extern unsigned platform_cpu_context[NR_CPUS];
/*
 * Power statuses
 */
#define STATUS_RUN      0
#define STATUS_STANDBY  1
#define STATUS_DORMANT  2
#define STATUS_SHUTDOWN 3

/* TODO: check these sizes */
#define CONTEXT_SPACE 32768
#define CONTEXT_SPACE_UNCACHED PAGE_SIZE
#define STACK_SIZE 1024

#define CPU_A5             0x4100c050
#define CPU_A8             0x4100c080
#define CPU_A9             0x410fc090

/*
 * Maximum size of each item of context, in bytes
 * We round these up to 32 bytes to preserve cache-line alignment
 */

#define PMU_DATA_SIZE               128
#define TIMER_DATA_SIZE             128
#define VFP_DATA_SIZE               288
#define GIC_INTERFACE_DATA_SIZE      64
#define GIC_DIST_PRIVATE_DATA_SIZE   96
#define BANKED_REGISTERS_SIZE       128
#define CP15_DATA_SIZE               64
#define DEBUG_DATA_SIZE            1024
#define MMU_DATA_SIZE                64
#define OTHER_DATA_SIZE              32
#define CONTROL_DATA_SIZE            32

#define GIC_DIST_SHARED_DATA_SIZE  2592
#define SCU_DATA_SIZE                32
#define L2_DATA_SIZE                256
#define GLOBAL_TIMER_DATA_SIZE      128

#define TTB_S		(1 << 1)
#define TTB_RGN_NC	(0 << 3)
#define TTB_RGN_OC_WBWA	(1 << 3)
#define TTB_RGN_OC_WT	(2 << 3)
#define TTB_RGN_OC_WB	(3 << 3)
#define TTB_NOS		(1 << 5)
#define TTB_IRGN_NC	((0 << 0) | (0 << 6))
#define TTB_IRGN_WBWA	((0 << 0) | (1 << 6))
#define TTB_IRGN_WT	((1 << 0) | (0 << 6))
#define TTB_IRGN_WB	((1 << 0) | (1 << 6))

/* PTWs cacheable, inner WB not shareable, outer WB not shareable */
#define TTB_FLAGS_UP	(TTB_IRGN_WB|TTB_RGN_OC_WB)

/* PTWs cacheable, inner WBWA shareable, outer WBWA not shareable */
#define TTB_FLAGS_SMP	(TTB_IRGN_WBWA|TTB_S|TTB_NOS|TTB_RGN_OC_WBWA)

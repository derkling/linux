/*
 * include/linux/vexpress.h
 *
 * Copyright (C) 2012 ARM Limited
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 * Versatile Express register definitions and exported functions
 */

#define	VEXPRESS_SPC_WAKE_INTR_IRQ(cluster, cpu) \
			(1 << (4 * (cluster) + (cpu)))
#define	VEXPRESS_SPC_WAKE_INTR_FIQ(cluster, cpu) \
			(1 << (7 * (cluster) + (cpu)))
#define	VEXPRESS_SPC_WAKE_INTR_SWDOG			(1 << 10)
#define	VEXPRESS_SPC_WAKE_INTR_GTIMER		(1 << 11)
#define	VEXPRESS_SPC_WAKE_INTR_MASK			0xFFF

#ifndef __ASSEMBLY__
extern void plat_safe_barrier(unsigned int *);
#endif

#ifdef CONFIG_ARM_SPC

/*
 * TODO:
 * There should not be a need to export these constants if the
 * driver implements what the outside world desires. We do this
 * to be able to turn on ACE/ACP snoops in 'bLiks-tc2-asm.S'
 */
#define SPC_PHYS_BASE           0x7FFF0000

#define A15_SNOOP_MASK          (0x3 << 7)
#define A7_SNOOP_MASK           (0x1 << 13)

#define A15_PART_NO             0xF
#define A7_PART_NO              0x7

#define SNOOP_CTL_A15		0x404
#define SNOOP_CTL_A7		0x504
#define A15_BX_ADDR0            0xB68
#define A7_BX_ADDR0             0xB78

#ifndef __ASSEMBLY__
extern u32 vexpress_spc_get_clusterid(int cpu_part_no);
extern u32 vexpress_spc_read_rsthold_reg(int cluster);
extern u32 vexpress_spc_read_rststat_reg(int cluster);
extern u32 vexpress_scc_read_rststat(int cluster);
extern u32 vexpress_spc_get_wake_intr(int raw);
extern int vexpress_spc_standbywfi_status(int cluster, int cpu);
extern int vexpress_spc_standbywfil2_status(int cluster);
extern int vexpress_spc_set_cpu_wakeup_irq(u32 cpu, u32 cluster, u32 set);
extern int vexpress_spc_set_global_wakeup_intr(u32 set);
extern int vexpress_spc_get_performance(int cluster, int *perf);
extern int vexpress_spc_set_performance(int cluster, int perf);
extern int vexpress_spc_wfi_cpustat(int cluster);
extern void vexpress_spc_set_wake_intr(u32 mask);
extern void vexpress_spc_write_bxaddr_reg(int cluster, int cpu, u32 val);
extern void vexpress_spc_write_rsthold_reg(int cluster, u32 value);
extern void vexpress_spc_powerdown_enable(int cluster, int enable);
extern void vexpress_spc_adb400_pd_enable(int cluster, int enable);
extern void vexpress_spc_wfi_cpureset(int cluster, int cpu, int enable);
extern void vexpress_spc_wfi_cluster_reset(int cluster, int enable);
extern void vexpress_scc_ctl_snoops(int cluster, int enable);
extern bool vexpress_spc_check_loaded(void);
#endif				/* ! __ASSEMBLY__ */
#else
#ifndef __ASSEMBLY__
static inline int vexpress_spc_set_cpu_wakeup_irq(u32 cpu, u32 cluster, u32 set)
{
	return  0;
}

static inline int vexpress_spc_set_global_wakeup_intr(u32 set)
{
	return 0;
}

static inline int vexpress_spc_standbywfi_status(int cluster, int cpu)
{
	return 0;
}

static inline int vexpress_spc_standbywfil2_status(int cluster)
{
	return 0;
}

static inline u32 vexpress_spc_get_clusterid(int cpu_part_no)
{
	return 0;
}

static inline u32 vexpress_spc_read_rsthold_reg(int cluster)
{
	return 0;
}

static inline u32 vexpress_spc_read_rststat_reg(int cluster)
{
	return 0;
}

static inline void vexpress_spc_write_bxaddr_reg(int cluster, int cpu, u32 val)
{
}

static inline void vexpress_spc_write_rsthold_reg(int cluster, u32 value)
{
}

static inline u32 vexpress_scc_read_rststat(int cluster)
{
	return 0;
}

static inline int vexpress_spc_get_performance(int cluster, int *perf)
{
	return -EINVAL;
}
static inline int vexpress_spc_set_performance(int cluster, int perf)
{
	return -EINVAL;
}
static inline void vexpress_spc_set_wake_intr(u32 mask) { }
static inline u32 vexpress_spc_get_wake_intr(int raw) { return 0; }
static inline void vexpress_spc_powerdown_enable(int cluster, int enable) { }
static inline void vexpress_spc_adb400_pd_enable(int cluster, int enable) { }
static inline void vexpress_spc_wfi_cpureset(int cluster, int cpu, int enable)
{ }
static inline int vexpress_spc_wfi_cpustat(int cluster) { return 0; }
static inline void vexpress_spc_wfi_cluster_reset(int cluster, int enable) { }
static inline bool vexpress_spc_check_loaded(void)
{
	return false;
}
static inline void vexpress_scc_ctl_snoops(int cluster, int enable) { }
#endif				/* ! __ASSEMBLY__ */
#endif

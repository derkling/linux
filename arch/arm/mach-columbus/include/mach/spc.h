/*
 * Serial Power Controller (SPC) support
 *
 * Copyright (C) 2012 ARM Ltd.
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

#ifndef __ASM_ARCH_SPC_H
#define __ASM_ARCH_SPC_H

#define	WAKE_INTR_IRQ(cluster, cpu)	(1 << (4 * (cluster) + (cpu)))
#define	WAKE_INTR_FIQ(cluster, cpu)	(1 << (7 * (cluster) + (cpu)))
#define	WAKE_INTR_SWDOG			(1 << 10)
#define	WAKE_INTR_GTIMER		(1 << 11)
#define	WAKE_INTR_MASK			0xFFF

#ifdef CONFIG_ARM_SPC
extern int spc_set_performance(int cluster, int perf);
extern void spc_set_wake_intr(u32 mask);
extern u32 spc_get_wake_intr(int raw);
extern void spc_powerdown_enable(int cluster, int enable);
extern void spc_adb400_pd_enable(int cluster, int enable);
extern void spc_wfi_cpureset(int cluster, int cpu, int enable);
extern int spc_wfi_cpustat(int cluster);
extern void spc_wfi_cluster_reset(int cluster, int enable);
extern void scc_ctl_snoops(int cluster, int enable);
#else
static inline int spc_set_performance(int cluster, int perf)
{
	return -EINVAL;
}
static inline void spc_set_wake_intr(u32 mask) { }
static inline u32 spc_get_wake_intr(int raw) { return 0; }
static inline void spc_powerdown_enable(int cluster, int enable) { }
static inline void spc_adb400_pd_enable(int cluster, int enable) { }
static inline void spc_wfi_cpureset(int cluster, int cpu, int enable) { }
static inline int spc_wfi_cpustat(int cluster) { return 0; }
static inline void spc_wfi_cluster_reset(int cluster, int enable) { }
static inline void scc_ctl_snoops(int cluster, int enable) { }
#endif

#endif

#ifndef __ELBA_SCPC_H
#define __ELBA_SCPC_H

#include <linux/compiler.h>
#include <linux/spinlock_types.h>

#define DRIVER_NAME "SCPC"
#define RES_SIZE    "4096"
#define PWR_STATE(dev, dev_state, cluster_state, time_unit, time)	\
	((0 << 28) | \
	(dev << 20) | \
	(dev_state << 16) | \
	(cluster_state << 12) | \
	(time_unit << 10) | \
	time)

#define PERF(performance) \
	((1 << 28) | \
	perf)

enum spc_regs {
	spc_hip_control   = 0x000,
	spc_mid_control   = 0x010,
	spc_wake_int_mask = 0x020,
	spc_hip_auto_shutdown = 0x030,
	spc_hip_force_wakeup = 0x034,
};

enum spc_pwr_devs {
	HIP_Cpu0       = 0x00,
	HIP_Cpu1       = 0x01,
	HIP_Cpu0_Neon  = 0x02,
	HIP_Cpu1_Neon  = 0x03,
	HIP_PTM        = 0x04,
	HIP_MBIST      = 0x05,
	HIP_Debug_Vsoc = 0x06,
	MID_Cpu0       = 0x07,
	MID_Cpu1       = 0x08,
	MID_Cpu0_Neon  = 0x09,
	MID_Cpu1_Neon  = 0x0A,
	MID_PTM        = 0x0B,
	MID_MBIST      = 0x0C,
	MID_Debug_Vsoc = 0x0D,
	VCC_Vgraph     = 0x0E,
	System         = 0x0F,
	DeviceNum      = System + 1,
};

enum spc_pwr_states {
	Run        = 0x00,
	Clock_Stop = 0x01,
	Dormant    = 0x02,
	Off        = 0x03,
	StateNum   = Off + 1,
};

enum spc_time_unit {
	ns    = 0x00,
	us    = 0x01,
	ms    = 0x02,
	s     = 0x03,
	Units = s + 1,
};

enum spc_time {
	Unknown = 0x00,
};

struct scpc_info {
	void __iomem *base;
	spinlock_t lock;
};

extern int hip_set_performance(int perf);
extern int mid_set_performance(int perf);
extern int set_wakeup_int(int devs);
extern int hip_set_auto_shutdown(int cpu);
extern int hip_force_wakeup(int cpu);
extern int hip_set_power_state(int dev, int dev_state,
		int cluster_state, int time_unit, int time);
extern int mid_set_power_state(int dev, int dev_state,
		int cluster_state, int time_unit, int time);
#endif /* __ELBA_SCPC_H  */

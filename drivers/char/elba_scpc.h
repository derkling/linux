#ifndef __ELBA_SCPC_H
#define __ELBA_SCPC_H

#define DRIVER_NAME "SCPC"
#define RES_SIZE    "4096"
#define PWR_STATE(dev, dev_state, cluster_state, time_unit, time)	\
	(0 << 28) | \
	(dev << 20) | \
	(dev_state << 16) | \
	(cluster_state << 12) | \
	(time_unit << 10) | \
	time

#define PERF(performance) \
	(1 << 28) | \
	perf

enum {
	spc_hip_control   = 0x000,
	spc_mid_control   = 0x010,
	spc_wake_int_mask = 0x020,
} spc_regs;

enum {
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
} spc_pwr_devs;

enum {
	Run        = 0x00,
	Clock_Stop = 0x01,
	Dormant    = 0x02,
	Off        = 0x03,
	StateNum   = Off + 1,

} spc_pwr_states;

enum {
	ns    = 0x00,
	us    = 0x01,
	ms    = 0x02,
	s     = 0x03,
	Units = s + 1,
} spc_time_unit;

enum {
	Unknown = 0x00,
} spc_time;

struct scpc_info {
	volatile void __iomem *base;
	spinlock_t lock;
};

#endif /* __ELBA_SCPC_H  */

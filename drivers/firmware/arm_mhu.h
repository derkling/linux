#ifndef _ARM_MHU_H
#define _ARM_MHU_H

/* Registers.  */
#define SCP_INTR_L_STAT  0x000
#define SCP_INTR_L_SET   0x008
#define SCP_INTR_L_CLEAR 0x010
#define SCP_INTR_H_STAT  0x020
#define SCP_INTR_H_SET   0x028
#define SCP_INTR_H_CLEAR 0x030
#define CPU_INTR_L_STAT  0x100
#define CPU_INTR_L_SET   0x108
#define CPU_INTR_L_CLEAR 0x110
#define CPU_INTR_H_STAT  0x120
#define CPU_INTR_H_SET   0x128
#define CPU_INTR_H_CLEAR 0x130
#define PID4             0xFD0
#define PID0             0xFE0
#define PID1             0xFE4
#define PID2             0xFE8
#define PID3             0xFEC
#define COMPID0          0xFF0
#define COMPID1          0xFF4
#define COMPID2          0xFF8
#define COMPID3          0xFFC

/* Memory offsets.  */
#define SCP_LOW      0x000
#define CPU_LOW      0x200
#define SCP_HIGH     0x400
#define CPU_HIGH     0x600

struct arm_mhu_data {
	void *regs;
	void *mem;
	int hi_irq;
	int lo_irq;
	struct completion *sync;
};

int get_dvfs_size(int cluster, int cpu, int *size);
int get_dvfs_capabilities(int cluster, int cpu, int *freqs);
int get_performance(int cluster, int cpu, int *perf);
int set_performance(int cluster, int cpu, int index);

static inline unsigned int mhu_reg_readl(struct arm_mhu_data *data, unsigned int reg, unsigned int value)
{
	return readl(data->regs + reg);
}

static inline void mhu_reg_writel(struct arm_mhu_data *data, unsigned int reg, unsigned int value)
{
	writel(value, data->regs + reg);
}

#endif /* _ARM_MHU_H  */

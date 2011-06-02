#ifndef __ELBA_MTU_H
#define __ELBA_MTU_H

#define MTC_BASE            0xE001A000
#define MTCVER              0xE00
#define MTCCTRL             0xE04
#define MTCWIND             0xE08
#define MTTCFG              0xE0C
/* MTU1: DRAM controller 1  */
#define MTCTRL1             0x104
#define MTLATS1             0x120
#define MTXACTC1            0x128
#define MTBWC1              0x12C
/* MTU2: DRAM controller 2  */
#define MTCTRL2             0x204
#define MTLATS2             0x220
#define MTXACTC2            0x228
#define MTBWC2              0x22C
/* MTU3: DMA  */
#define MTCTRL3             0x304
#define MTLATS3             0x320
#define MTXACTC3            0x328
#define MTBWC3              0x32C
/* MTU4: Level2 Cache controller 1  */
#define MTCTRL4             0x404
#define MTLATS4             0x420
#define MTXACTC4            0x428
#define MTBWC4              0x42C
/* MTU5: Level2 Cache controller 2  */
#define MTCTRL5             0x504
#define MTLATS5             0x520
#define MTXACTC5            0x528
#define MTBWC5              0x52C
/* MTU6: HDLCD  */
#define MTCTRL6             0x604
#define MTLATS6             0x620
#define MTXACTC6            0x628
#define MTBWC6              0x62C
/* MTU7: Mali400  */
#define MTCTRL7             0x704
#define MTLATS7             0x720
#define MTXACTC7            0x728
#define MTBWC7              0x72C
/* MTU8: Mali AVE  */
#define MTCTRL8             0x804
#define MTLATS8             0x820
#define MTXACTC8            0x828
#define MTBWC8              0x82C
/* MTU9: PCI Express Root Complex  */
#define MTCTRL9             0x904
#define MTFILT9             0x908
#define MTFILTM9             0x90C
#define MTLATS9             0x920
#define MTXACTC9            0x928
#define MTBWC9              0x92C

#define MTC_IRQ             (32 + 11)
#define MTC_SAMPLES         32
/* Number of clock cycles in one sampling interval */
#define MTC_SAMPLE_WINDOW   (0x0000FFFF)  

#define SAMPLES(samples) \
	unsigned int *dmc1_bw = samples->dmc1_bw;		\
	unsigned int *latency1 = samples->latency1;		\
	unsigned int *dmc2_bw = samples->dmc2_bw;		\
	unsigned int *latency2 = samples->latency2;		\
	unsigned int *dma_bw = samples->dma_bw;			\
	unsigned int *latency3 = samples->latency3;		\
	unsigned int *l2cc1_bw = samples->l2cc1_bw;		\
	unsigned int *latency4 = samples->latency4;		\
	unsigned int *l2cc2_bw = samples->l2cc2_bw;		\
	unsigned int *latency5 = samples->latency5;		\
	unsigned int *hdlcd_bw = samples->hdlcd_bw;		\
	unsigned int *latency6 = samples->latency6;		\
	unsigned int *mali400_bw = samples->mali400_bw;		\
	unsigned int *latency7 = samples->latency7;		\
	unsigned int *maliave_bw = samples->maliave_bw;		\
	unsigned int *latency8 = samples->latency8;		\
	unsigned int *pcie_bw = samples->pcie_bw;		\
	unsigned int *latency9 = samples->latency9;		\
	
/* Samples  */
struct samples {
	unsigned int dmc1_bw[MTC_SAMPLES];
	unsigned int dmc2_bw[MTC_SAMPLES];
	unsigned int dma_bw[MTC_SAMPLES];
	unsigned int l2cc1_bw[MTC_SAMPLES];
	unsigned int l2cc2_bw[MTC_SAMPLES];
	unsigned int hdlcd_bw[MTC_SAMPLES];
	unsigned int mali400_bw[MTC_SAMPLES];
	unsigned int maliave_bw[MTC_SAMPLES];
	unsigned int pcie_bw[MTC_SAMPLES];

	unsigned int latency1[MTC_SAMPLES];
	unsigned int latency2[MTC_SAMPLES];
	unsigned int latency3[MTC_SAMPLES];	
	unsigned int latency4[MTC_SAMPLES];
	unsigned int latency5[MTC_SAMPLES];
	unsigned int latency6[MTC_SAMPLES];
	unsigned int latency7[MTC_SAMPLES];
	unsigned int latency8[MTC_SAMPLES];
	unsigned int latency9[MTC_SAMPLES];
};

struct elba_mtc_info {
	void __iomem *regs;
	unsigned long mem;
	struct samples *memlimit;
	int memsize;
	struct samples *samples;
	struct samples *current_sample;
	int blocks;
	int major;
};

extern struct elba_mtc_info *mtc_info;
#endif /* __ELBA_MTU_H  */

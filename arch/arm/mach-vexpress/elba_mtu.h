#ifndef __ELBA_MTU_H
#define __ELBA_MTU_H

#define MTC_BASE            0xE001A000
#define MTCVER              0xE00
#define MTCCTRL             0xE04
#define MTCWIND             0xE08
#define MTTCFG              0xE0C

/* Generic MTU macros.  */
#define MTVER               0x000
#define MTCTRL              0x004
#define MTFILT              0x008
#define MTFILTM             0x00C
#define MTLATM              0x01C
#define MTLATS              0x020
#define MTLATC              0x024
#define MTXACTC             0x028
#define MTBWC               0x02C

/* MTUS  */
#define DRAM1               1
#define DRAM2               2
#define DMA                 3
#define L2CC1               4
#define L2CC2               5
#define HDLCD               6
#define MALI400             7
#define MALIAVE             8
#define PCIE                9

#define MTU_OFF(mtu, reg) \
	({ (reg) + (0x100 * (mtu)); })

#define MTC_IRQ             (32 + 11)

/* Number of clock cycles in one sampling interval */
#define MTC_SAMPLE_WINDOW   (0x0000FFFF)  

#define SAMPLES_BW(samples) \
	unsigned int *dmc1_bw = &samples->dmc1_bw;		\
	unsigned int *dmc2_bw = &samples->dmc2_bw;		\
	unsigned int *dma_bw = &samples->dma_bw;		\
	unsigned int *l2cc1_bw = &samples->l2cc1_bw;		\
	unsigned int *l2cc2_bw = &samples->l2cc2_bw;		\
	unsigned int *hdlcd_bw = &samples->hdlcd_bw;		\
	unsigned int *mali400_bw = &samples->mali400_bw;	\
	unsigned int *maliave_bw = &samples->maliave_bw;	\
	unsigned int *pcie_bw = &samples->pcie_bw;		\

#define SAMPLES_LAT(samples) \
	unsigned int *latency1 = &samples->latency1;		\
	unsigned int *transactions1 = &samples->transactions1;  \
	unsigned int *maxlat1 = &samples->maxlat1;		\
	unsigned int *latency2 = &samples->latency2;		\
	unsigned int *transactions2 = &samples->transactions2;  \
	unsigned int *maxlat2 = &samples->maxlat2;		\
	unsigned int *latency3 = &samples->latency3;		\
	unsigned int *transactions3 = &samples->transactions3;  \
	unsigned int *maxlat3 = &samples->maxlat3;		\
	unsigned int *latency4 = &samples->latency4;		\
	unsigned int *transactions4 = &samples->transactions4;  \
	unsigned int *maxlat4 = &samples->maxlat4;		\
	unsigned int *latency5 = &samples->latency5;		\
	unsigned int *transactions5 = &samples->transactions5;  \
	unsigned int *maxlat5 = &samples->maxlat5;		\
	unsigned int *latency6 = &samples->latency6;		\
	unsigned int *transactions6 = &samples->transactions6;  \
	unsigned int *maxlat6 = &samples->maxlat6;		\
	unsigned int *latency7 = &samples->latency7;		\
	unsigned int *transactions7 = &samples->transactions7;  \
	unsigned int *maxlat7 = &samples->maxlat7;		\
	unsigned int *latency8 = &samples->latency8;		\
	unsigned int *transactions8 = &samples->transactions8;  \
	unsigned int *maxlat8 = &samples->maxlat8;		\
	unsigned int *latency9 = &samples->latency9;		\
	unsigned int *transactions9 = &samples->transactions9;  \
	unsigned int *maxlat9 = &samples->maxlat9;		\

/* Samples  */
struct samples {
	unsigned int dmc1_bw;
	unsigned int dmc2_bw;
	unsigned int dma_bw;
	unsigned int l2cc1_bw;
	unsigned int l2cc2_bw;
	unsigned int hdlcd_bw;
	unsigned int mali400_bw;
	unsigned int maliave_bw;
	unsigned int pcie_bw;

	unsigned int latency1;
	unsigned int transactions1;
	unsigned int maxlat1;
	unsigned int latency2;
	unsigned int transactions2;
	unsigned int maxlat2;
	unsigned int latency3;
	unsigned int transactions3;
	unsigned int maxlat3;
	unsigned int latency4;
	unsigned int transactions4;
	unsigned int maxlat4;
	unsigned int latency5;
	unsigned int transactions5;
	unsigned int maxlat5;
	unsigned int latency6;
	unsigned int transactions6;
	unsigned int maxlat6;
	unsigned int latency7;
	unsigned int transactions7;
	unsigned int maxlat7;
	unsigned int latency8;
	unsigned int transactions8;
	unsigned int maxlat8;
	unsigned int latency9;
	unsigned int transactions9;
	unsigned int maxlat9;
};

enum {
	ide = 0x001,
	mxe = 0x040,
	bwe = 0x100,
	sme = 0x200,
	we  = 0x400,
};

#define MTUS 9
struct mtu_setup {
	int mtu_id;
	int enabled;
};

struct elba_mtc_info {
	void __iomem *regs;
	struct samples *samples;
	spinlock_t lock;
};

extern struct elba_mtc_info *mtc_info;
#endif /* __ELBA_MTU_H  */

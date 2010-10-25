#ifndef __MACH_LT_ELBA_H
#define __MACH_LT_ELBA_H


/*
 * Physical base addresses
 */
#define LT_ELBA_SMC		(0xEC000000)
#define LT_ELBA_SCC		(0xE3000000)
#define LT_ELBA_L2CC		(0xE0202000)
#define LT_ELBA_MPIC		(0xE0200000)
#define LT_ELBA_SP804_TIMER	(0xE0011000)

#define ELBA_A9_MPCORE_GIC_CPU	(LT_ELBA_MPIC + 0x0100)
#define ELBA_A9_MPCORE_GIC_DIST	(LT_ELBA_MPIC + 0x1000)

#define ELBA_A9_MPCORE_SCU	(LT_ELBA_MPIC + 0x0000)
#define ELBA_A9_MPCORE_TWD	(LT_ELBA_MPIC + 0x0600)

extern struct vexpress_tile_desc lt_elba_desc;

#endif

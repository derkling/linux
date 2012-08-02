/*
 * arch/arm/include/asm/bL_iks_tc2.h
 *
 * Created by:  Achin Gupta, 2012-07-25
 * Copyright:   (C) 2012  ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef BL_IKS_TC2_H
#define BL_IKS_TC2_H

#define TC2_MAILBOX_BASE    0x7fff0000
#define TC2_MAILBOX_OFFSET  0xb68

/*
 * TODO:
 * There should not be a need to export these constants if the
 * driver implements what the outside world desires. We do this
 * to be able to turn on CCI snoops in 'bLiks-tc2-asm.S'
 */
#define CCI_SLAVE_OFFSET(n)     (0x1000 + 0x1000 * (n))
#define SLAVE_SNOOPCTL_OFFSET   0
#define PENDING_STATUS_OFFSET   0xc
#define SNOOPCTL_SNOOP_ENABLE   (1 << 0)
#define SNOOPCTL_DVM_ENABLE     (1 << 1)
#define STATUS_CHANGE_PENDING   (1 << 0)

#define CCI_PHYS_BASE   0x2c090000
#define CCI_SLAVE_A15   3
#define CCI_SLAVE_A7    4

#define CCI_A15_OFFSET  CCI_SLAVE_OFFSET(CCI_SLAVE_A15)
#define CCI_A7_OFFSET   CCI_SLAVE_OFFSET(CCI_SLAVE_A7)

#ifndef __ASSEMBLY__

#ifdef CONFIG_ARCH_VEXPRESS_TC2_IKS
extern void bLiks_power_up_setup(void);
extern void bLiks_reserve(void);
#else
static inline void bLiks_power_up_setup(void)
{
}

static inline void bLiks_reserve(void)
{
}
#endif

#endif				/* ! __ASSEMBLY__ */

#endif				/* ! BL_IKS_TC2_H */

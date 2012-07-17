/*
 * arch/arm/include/asm/bL_switcher.h
 *
 * Created by:  Dave Martin, 2012-06-22
 * Copyright:   (C) 2012  Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef ARCH_VEXPRESS_KINGFISHER_H
#define ARCH_VEXPRESS_KINGFISHER_H

/* CCI-related stuff should move to a separate header */
#define CCI_SLAVE_OFFSET(n)	(0x1000 + 0x1000 * (n))
#define SLAVE_SNOOPCTL_OFFSET	0
#define SNOOPCTL_SNOOP_ENABLE	(1 << 0)
#define SNOOPCTL_DVM_ENABLE	(1 << 1)

#define CCI_STATUS_OFFSET	0xc
#define STATUS_CHANGE_PENDING	(1 << 0)

#define RTSM_CCI_PHYS_BASE	0x2c090000
#define RTSM_CCI_SLAVE_A15	3
#define RTSM_CCI_SLAVE_A7	4

#define RTSM_CCI_A15_OFFSET	CCI_SLAVE_OFFSET(RTSM_CCI_SLAVE_A15)
#define RTSM_CCI_A7_OFFSET	CCI_SLAVE_OFFSET(RTSM_CCI_SLAVE_A7)

#ifndef __ASSEMBLY__

#ifdef CONFIG_ARCH_VEXPRESS_KINGFISHER
extern void kfs_reserve(void);
#else
static void kfs_reserve(void) { }
#endif

#endif /* ! __ASSEMBLY__ */

#endif /* ! ARCH_VEXPRESS_KINGFISHER_H */

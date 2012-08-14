/*
 * arch/arm/include/asm/kingfisher_setup.S
 *
 * Created by:  Dave Martin, 2012-06-22
 * Copyright:   (C) 2012  Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/linkage.h>
#include <asm/asm-offsets.h>
#include <asm/bL_entry.h>
#include <mach/kingfisher.h>

/*
 * Enable cluster-level coherency, in preparation for turning on the MMU.
 * The ACTLR SMP bit does not need to be set here, because cpu_resume()
 * already restores that.
 */
ENTRY(bL_kfs_power_up_setup)
	mrc	p15, 0, r0, c0, c0, 5	@ MPIDR
	ubfx	r1, r0, #0, #4		@ cpu
	ubfx	r0, r0, #8, #4		@ cluster

	__bL_sync_get_cluster_base r2, r0, r3
	__bL_inbound_enter_critical r2, r1, 2f, r3, r12

	@ Set up the cluster if/when appropriate (in other cases,
	@ __bL_inbound_enter_critical will branch to 2f.)

	@ A15/A7 may not require explicit L2 invalidation on reset, dependent
	@ on hardware integration desicions.
	@ For now, this code assumes that L2 is either already invalidated, or
	@ invalidation is not required.

	ldr	r3, =RTSM_CCI_PHYS_BASE + RTSM_CCI_A15_OFFSET
	cmp	r0, #0		@ A15 cluster?
	addne	r3, r3, #RTSM_CCI_A7_OFFSET - RTSM_CCI_A15_OFFSET

	@ r3 now points to the correct CCI slave register block

	ldr	r0, [r3, #SLAVE_SNOOPCTL_OFFSET]
	orr	r0, r0, #SNOOPCTL_SNOOP_ENABLE | SNOOPCTL_DVM_ENABLE
	str	r0, [r3, #SLAVE_SNOOPCTL_OFFSET]	@ enable CCI snoops

	@ Wait for snoop control change to complete:

	ldr	r3, =RTSM_CCI_PHYS_BASE

	b	1f
0:	dsb
1:	ldr	r0, [r3, #CCI_STATUS_OFFSET]
	tst	r0, #STATUS_CHANGE_PENDING
	bne	0b

	__bL_inbound_leave_critical r2, r1, r0

	@ Implementation-specific local CPU setup operations should go here,
	@ if any.  In this case, there is nothing to do.

2:	__bL_inbound_cpu_up r2, r1, r0
	bx	lr
ENDPROC(bL_kfs_power_up_setup)

ENTRY(plat_safe_barrier)
	dsb
	bx	lr
ENDPROC(plat_safe_barrier)
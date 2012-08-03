/*
 * arch/arm/include/asm/bL_iks_tc2-asm.S
 *
 * Created by:  Achin Gupta, 2012-07-25
 * Copyright:   (C) 2012  ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/linkage.h>
#include <asm/asm-offsets.h>
#include <asm/bL_entry.h>
#include <mach/bLiks-tc2.h>
#include <linux/vexpress.h>

/*
 * Enable cluster-level coherency, in preparation for turning on the MMU.
 * The ACTLR SMP bit does not need to be set here, because cpu_resume()
 * already restores that.
 */
ENTRY(bLiks_power_up_setup)
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

	@ Enable ACE and ACP snoops as applicable

	ldr	r3, =SPC_PHYS_BASE + SNOOP_CTL_A15
	mrc	p15, 0, r0, c0, c0, 0	@ MIDR
	ubfx    r0, r0, #4, #4		@ Primary part number
	cmp     r0, #A15_PART_NO
	addne   r3, r3, #SNOOP_CTL_A7 - SNOOP_CTL_A15
	ldr     r0, [r3]
	bicne   r0, #A7_SNOOP_MASK
	biceq   r0, #A15_SNOOP_MASK
	str     r0, [r3]

	@ Enable CCI snoops

	ldr	r3, =CCI_PHYS_BASE + CCI_A15_OFFSET
	mrc	p15, 0, r0, c0, c0, 5	@ MPIDR
	ubfx	r0, r0, #8, #4		@ cluster
	cmp	r0, #0		@ A15 cluster?
	addne	r3, r3, #CCI_A7_OFFSET - CCI_A15_OFFSET

	@ r3 now points to the correct CCI slave register block

	ldr	r0, [r3, #SLAVE_SNOOPCTL_OFFSET]
	orr	r0, r0, #SNOOPCTL_SNOOP_ENABLE | SNOOPCTL_DVM_ENABLE
	str	r0, [r3, #SLAVE_SNOOPCTL_OFFSET]	@ enable CCI snoops

	@ Wait for snoop control change to complete:

	ldr	r3, =CCI_PHYS_BASE

	b	1f
1:
	ldr	r0, [r3, #PENDING_STATUS_OFFSET]
	tst	r0, #STATUS_CHANGE_PENDING
	bne	1b

	@ Restore the cpuid

	mrc	p15, 0, r0, c0, c0, 5	@ MPIDR
	ubfx	r1, r0, #0, #4		@ cpu
	__bL_inbound_leave_critical r2, r1, r0

	@ Implementation-specific local CPU setup operations should go here,
	@ if any.  In this case, there is nothing to do.

2:	__bL_inbound_cpu_up r2, r1, r0

	@ Clear the BX addr register

	ldr	r3, =SPC_PHYS_BASE + A15_BX_ADDR0
	mrc	p15, 0, r0, c0, c0, 5	@ MPIDR
	ubfx	r1, r0, #0, #4		@ cpu
	ubfx	r0, r0, #8, #4		@ cluster
        lsl     r1, r1, #2
        lsl     r0, r0, #4
        add     r1, r0, r1
        mov     r0, #0
	str	r0, [r3, r1]

	bx	lr
ENDPROC(bLiks_power_up_setup)

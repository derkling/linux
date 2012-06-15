/*
 * Columbus secure APIs file.
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>
 *
 * This program is free software,you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>

	.syntax unified
	.arch_extension sec

/*
 * This function implements the SMC call stub for powerdown API.
 *
 * Currently a stubbed out version just executing wfi for testing purposes.
 *
 * Function C-prototype is:
 * shutdown_smc(u32 unused, u32 cstate, u32 wakeup_h, u32 wakeup_l)
 * r1: target C-state
 * r2: wake-up timestamp (63:32)
 * r3: wake-up timestamp (31:0)
 *
 * Return value:
 * 	does not return on success
 * 	-1 on failure
 */

ENTRY(shutdown_smc)
	ldr 	r0, =#0x80100001
	smc #0
	mov	pc, lr
ENDPROC(shutdown_smc)

ENTRY(_smc_down)
	ldr 	r0, =#0x80100001
	smc #0
	mov	pc, lr
ENDPROC(_smc_down)

ENTRY(_smc_up)
	ldr 	r0, =0x80100002
	smc #0
	mov	pc, lr
ENDPROC(_smc_up)

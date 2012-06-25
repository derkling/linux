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
 * _smc_down(u32 unused, u32 pwr_exp, u32 resume_addr, u32 context_id)
 * _smc_up(u32 unused, u32 pwr_exp, u32 resume_addr, u32 context_id)
 * r1: [0:15] platform specific state ID (e.g. mpidr)
 *     [16:17] target C-state
 * r2: resume code/vector address where the core(s) must resume execution
 * r3: contextID value that will be placed in R0 when resumed @r2
 *
 * Return value:
 * 	 0 on success (only for _smc_up)
 *	-1 invalid parameters
 *	-2 retry/already up
 */

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

@ Copyright (C) 2010 ARM Limited

@ This software is provided 'as-is', without any express or implied
@ warranties including the implied warranties of satisfactory quality,
@ fitness for purpose or non infringement.  In no event will  ARM be
@ liable for any damages arising from the use of this software.

@ Permission is granted to anyone to use, copy and modify this software for
@ any purpose, and to redistribute the software, subject to the following
@ restrictions:

@ 1. The origin of this software must not be misrepresented; you must not
@    claim that you wrote the original software. If you use this software
@    in a product, an acknowledgment in the product documentation would be
@    appreciated but is not required.
@ 2. Altered source versions must be plainly marked as such, and must not be
@    misrepresented as being the original software.
@ 3. This notice may not be removed or altered from any source distribution.

#include <linux/linkage.h>
#include <generated/asm-offsets.h>
#include <asm/thread_info.h>
#include <asm/memory.h>
#include <asm/ptrace.h>
#include <mach/entry-macro.S>

#define	APPF_FUNCTION_INITIALIZE 0
#define	SCTLR_RR (1<<14)
#define	SCTLR_I (1<<12)
#define	SCTLR_Z (1<<11)
#define	OFFSET (PAGE_OFFSET - PHYS_OFFSET)
#define	DELAY	100

	.syntax unified
	.text

ENTRY(switch_page_tables)
	PUSH	{r4, r5, r6, r7}
	mov 	r2, 0
	@ Get APPF values for TTBR0, TTBCR, DACR
	@ Get current TTBR0, TTBCR, DACR
	mrc	p15, 0, r4, c3, c0, 0	  @ read DACR
	mrc	p15, 0, r5, c2, c0, 0	  @ read TTBR0
	mrc	p15, 0, r6, c2, c0, 1	  @ read TTBR1
	mrc	p15, 0, r7, c2, c0, 2	  @ read TTBCR
	stmia   r1, {r4-r7}

	@ Switch translation tables to APPF
	ldmia   r0, {r4-r7}
	mcr	p15, 0, r5, c2, c0, 0	     @ write TTBR0
	dsb
	mcr	p15, 0, r2, c8, c7, 0	     @ Inv both TLBs
	mcr	p15, 0, r2, c7, c5, 6	     @ Inv branch predictor array
	dsb		   @ Wait for the preceding operations to finish
	isb		   @ Now re-fetch following instructions
	pop	{r4, r5, r6, r7}
	bx	lr
ENDPROC(switch_page_tables)

ENTRY(restore_page_tables)
	push	{r4, r5, r6, r7}
	mov 	r2, #0
	@ Switch translation tables to APPF
	ldmia   r0, {r4-r7}
	mcr	p15, 0,  r4, c3, c0, 0	     @ write DACR
	mcr	p15, 0,  r5, c2, c0, 0	     @ write TTBR0
	mcr	p15, 0,  r6, c2, c0, 1	     @ write TTBR0
	mcr	p15, 0,  r7, c2, c0, 2	     @ write TTBCR
	dsb
	mcr	p15, 0, r2, c8, c7, 0	     @ Inv both TLBs
	mcr	p15, 0, r2, c7, c5, 6	     @ Inv branch predictor array
	dsb		   @ Wait for the preceding operations to finish
	isb		   @ Now re-fetch following instructions
	pop	{r4, r5, r6, r7}
	bx	lr

ENDPROC(restore_page_tables)

ENTRY(appf_sleep)
	mov	r12, r0
	mov	r2, lr @ we do know r2 is not corrupted across call
	bl	appf_platform_get_context
	mov 	lr, r2
	stmia	r0!, {r4 - sl, fp, lr} 	@ Store most regs on stack
	bl	appf_platform_set_stack_pointer
	dsb
	wfi
	mov	r1, #DELAY
idle:
	subs	r1, r1, #1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bne	idle
ENDPROC(appf_sleep)

ENTRY(appf_reset_entry_point)
	@ This is the entry point from the platform warm start code
	@ Single argument: r0 is non-zero if caches need invalidating
	mov	r4, r0
	msr     cpsr_c, #PSR_I_BIT | PSR_F_BIT | SVC_MODE @ set SVC, irqs off
	@ Get stack pointer
	bl	appf_platform_get_stack_pointer
	mov	sp, r0

	cmp	r4, #0
	beq	0f

	@ Clear I cache levels to PoU (also clears BTBs)
	bl	invalidate_icache_v7_pou

	@ Turn I cache and branch prediction on
0:                  bl	read_sctlr
	orr	r0, r0, #(SCTLR_RR | SCTLR_I  |  SCTLR_Z)
	bl	write_sctlr

	cmp	r4, #0
	beq	1f

	@ Clear all data cache levels visible to CPU
	bl	invalidate_dcache_v7_all
1:
	bl	appf_platform_get_context
	ldmia	r0, {r4 - sl, fp, pc}
ENDPROC(appf_reset_entry_point)

	.data
	.align
	.global appf_save_flat_mapped
	.global appf_restore_flat_mapped

ENTRY(appf_save_flat_call)
	adr	r12, appf_save_flat_mapped
	ldr	r12, [r12]
	sub	lr, lr ,#OFFSET
	bx	r12
ENDPROC(appf_save_flat_call)

ENTRY(appf_restore_flat_call)
	adr	r12, appf_restore_flat_mapped
	ldr	r12, [r12]
	add	lr, lr ,#OFFSET
	bx	r12
ENDPROC(appf_restore_flat_call)

appf_save_flat_mapped:
	.word 0
appf_restore_flat_mapped:
	.word 0
	.end

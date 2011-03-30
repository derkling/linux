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
#include <mach/memory.h>
#include <asm/memory.h>
#include "appf_defs.h"

	.global platform_cpu_stacks
	.global platform_cpu_context
	.global srvector
	.global srdata

	.syntax unified

#define CPU_A5 0x4100c050
#define OFFSET (PAGE_OFFSET - PHYS_OFFSET)

ENTRY(platform_a8_reset_handler)
	mov r0, #1
	b	appf_reset_entry_point
ENDPROC(platform_a8_reset_handler)

ENTRY(platform_a9_reset_handler)
	@ TODO: Restore Endianness bit(s) in CPSR
	@ TODO: Check for a warm start - either here or in boot monitor

	@ Work out whether caches need to be invalidated: A9 - yes, A5 - no
	@ TODO: If the cpu has been in an L1-retention state, don't invalidate
	mrc	p15, 0, r0, c0, c0, 0
	ldr	r1, =CPU_A5
	cmp	r0, r1
	moveq	r0, #0
	movne	r0, #1
	mov	r2, #RESET_TAG
	ldr	r1, [r2]
	add	r1, r1, #1
	str 	r1, [r2]
	mrc	p15, 0, r1, c0, c0, 5	  @ Read MPIDR
	and	r1, r1, #0xff		     @ extract CPU number
	b	appf_reset_entry_point
ENDPROC(platform_a9_reset_handler)


ENTRY(appf_platform_get_stack_pointer)
	adr	r1, platform_cpu_stacks
	mrc	p15, 0, r0, c0, c0, 5	  @ Read MPIDR
	and	r0, r0, #0xff		  @ extract CPU number
	ldr	r0, [r1, r0, lsl #2]
	bx	lr
ENDPROC(appf_platform_get_stack_pointer)

ENTRY(appf_patch_reset)
	adr r2, 1f
	ldr r3, [r2], #4
	str r3, [r0], #4
	ldr r3, [r2]
	str r3, [r0], #4
	str r1, [r0]
	bx lr
1:
	ldr r0, 2f
	ldr pc, [r0]
2:
ENDPROC(appf_patch_reset)

ENTRY(appf_platform_get_context)
	adr	r1, platform_cpu_context
	mrc	p15, 0, r0, c0, c0, 5	  @ Read MPIDR
	and	r0, r0, #0xff		  @ extract CPU number
	ldr	r0, [r1, r0, lsl #2]
	@sub	r0, r0, #OFFSET
	bx	lr
ENDPROC(appf_platform_get_stack_pointer)

ENTRY(appf_platform_set_stack_pointer)
	adr	r1, platform_cpu_stacks
	mrc	p15, 0, r2, c0, c0, 5	  @ Read MPIDR
	and	r2, r2, #0xff		  @ extract CPU number
	str 	sp, [r1, r2, lsl #2]
	bx	lr
ENDPROC(appf_platform_set_stack_pointer)

srdata:
.LCsraddress:
	.word 0

srvector:
	ldr pc,.LCsraddress + 0x200

platform_cpu_stacks:
	.space 16,0

platform_cpu_context:
	.space 16,0

	.end

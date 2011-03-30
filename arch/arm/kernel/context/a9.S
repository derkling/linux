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

	.syntax unified
	.text

ENTRY(save_a9_debug)
	@ TODO
	bx	lr
ENDPROC(save_a9_debug)

ENTRY(restore_a9_debug)
	@ TODO
	bx	lr
ENDPROC(restore_a9_debug)

ENTRY(save_a9_other)
	mrc	p15,0,r12,c15,c0,0	  @ Read Power Control Register
	str	r12, [r0], #4
	mrc     p15, 4, r12, c15, c0, 0   @ Read Configuration Base Address
					  @ Register
	str	r12, [r0], #4

	mrc	p15,0,r3,c0,c0,0	  @ Read Main ID Register
	ubfX	r3, r3, #20, #4		  @ Extract major version number
	cmp	r3, #2
	blt	1f			  @ PLE only possible in r2p0 onwards
	mrc	p15,0,r3,c11,c0,0	  @ Read PLE IDR
	cmp	r3, #0
	beq	1f			  @ No PLE present

	mrc	p15,0,r3,c11,c1,0	  @ Read PLE UAR
	mrc	p15,0,r12,c11,c1,1	  @ Read PLE PCR
	stm	r0!, {r3, r12}

1:      bx	lr
ENDPROC(save_a9_other)

ENTRY(restore_a9_other)
	cmp	r1, #0			  @ Check we are secure
	ldr	r12, [r0], #4
	andne	r12, r12, #0x01		  @ We only restore the Dynamic Clock
					  @ gating bit
	mcrne	p15,0,r12,c15,c0,0	  @ Write Power Control Register
					  @ (if secure)
	ldr	r12, [r0], #4
	mcrne   p15, 4, r12, c15, c0, 0   @ Write Configuration Base Address
					  @ Register (if Secure)

	mrc	p15,0,r3,c0,c0,0	  @ Read Main ID Register
	ubfx	r3, r3, #20, #4		  @ Extract major version number
	cmp	r3, #2
	blt	1f			  @ PLE only possible in r2p0 onwards
	mrc	p15,0,r3,c11,c0,0	  @ Read PLE IDR
	cmp	r3, #0
	beq	1f			  @ No PLE present

	ldm	r0!, {r3, r12}
	mcr	p15,0,r3,c11,c1,0	  @ Write PLE UAR
	mcr	p15,0,r12,c11,c1,1	  @ Write PLE PCR

1:      bx	lr
ENDPROC(restore_a9_other)

ENTRY(save_a5_other)
	mrc	p15, 5, r12, c15, c0, 0    @ Read TLB Hitmap Register
	str	r12, [r0], #4
	mrc     p15, 4, r12, c15, c0, 0     @ read configuration base address
					    @ register
	str	r12, [r0], #4
	bx	lr
ENDPROC(save_a5_other)

ENTRY(restore_a5_other)
	ldr	r12, [r0], #4
	mcr	p15, 5, r12, c15, c0, 0    @ Write TLB Hitmap Register
	ldr	r12, [r0], #4
	cmp	r1, #0			   @ Check we are secure
	mcrne   p15, 4, r12, c15, c0, 0    @ Write Configuration Base Address
					   @ Register (if Secure)
	bx	lr
ENDPROC(restore_a5_other)
	.end

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

ENTRY(save_a8_debug)
	@ TODO
	bx	lr
ENDPROC(save_a8_debug)

ENTRY(restore_a8_debug)
	@ TODO
	bx	lr
ENDPROC(restore_a8_debug)

ENTRY(save_a8_other)
	@ First wait for any running Preloads to finish
0:      mrc	p15,0,r3,c11,c0,2	  @ read ple identification and
					  @ status register running
	cmp	r3, #0
	bne	0b

	@ Now check which PLE channels are present
	mrc	p15,0,r12,c11,c0,0	  @ read ple idr
	cmp	r12, #0
	beq	20f			  @ no ple present

	@ Read the channel number register to store later
	mrc	p15,0,r3,c11,c2,0	  @ read ple channel number register

	@ Save channel 0 (if present)
	tst	r12, #1
	beq	10f

	mov	r1, #0
	mcr	p15,0,r1,c11,c2,0	  @ write ple channel number register
	mrc	p15,0,r1,c11,c4,0	  @ read ple control register
	mrc	p15,0,r2,c11,c5,0	  @ read ple internal start address
					  @ register
	stm	r0!, {r1, r2}
	mrc	p15,0,r1,c11,c7,0	  @ read ple internal end address
					  @ register
	mrc	p15,0,r2,c11,c15,0	  @ read ple context id register
	stm	r0!, {r1, r2}

	@ Save channel 1 (if present)
10:                 TST	r12, #2
	beq	15f

	mov	r1, #2
	mcr	p15,0,r1,c11,c2,0	  @ write ple channel number register
	mrc	p15,0,r1,c11,c4,0	  @ read ple control register
	mrc	p15,0,r2,c11,c5,0	  @ read ple internal start address
					  @ register
	stm	r0!, {r1, r2}
	mrc	p15,0,r1,c11,c7,0	  @ read ple internal end address
					  @ register
	mrc	p15,0,r2,c11,c15,0	  @ read ple context id register
	stm	r0!, {r1, r2}

	@ Save channel number register
15:     str	r3, [r0], #4

20:     bx	lr
ENDPROC(save_a8_other)

ENTRY(restore_a8_other)
	@ First check which PLE channels are present
	mrc	p15,0,r12,c11,c0,0	  @ read ple idr
	cmp	r12, #0
	beq	20f			  @ no ple present

	@ Restore channel 0 (if present)
	tst	r12, #1
	beq	10f

	mov	r1, #0
	mcr	p15,0,r1,c11,c2,0	  @ write ple channel number register
	ldm	r0!, {r1, r2}
	mcr	p15,0,r1,c11,c4,0	  @ write ple control register
	mcr	p15,0,r2,c11,c5,0	  @ write ple internal start address
					  @ register
	ldm	r0!, {r1, r2}
	mcr	p15,0,r1,c11,c7,0	  @ write ple internal end address
					  @ register
	mcr	p15,0,r2,c11,c15,0	  @ write ple context id register

10:                     @ Restore channel 1 (if present)
	tst	r12, #2
	beq	15f

	mov	r1, #1
	mcr	p15,0,r1,c11,c2,0	  @ write ple channel number register
	ldm	r0!, {r1, r2}
	mcr	p15,0,r1,c11,c4,0	  @ write ple control register
	mcr	p15,0,r2,c11,c5,0	  @ write ple internal start address
					  @ register
	ldm	r0!, {r1, r2}
	mcr	p15,0,r1,c11,c7,0	  @ write ple internal end address
					  @ register
	mcr	p15,0,r2,c11,c15,0	  @ write ple context id register

	@ Restore channel number register
15:     ldr	r3, [r0], #4
	mcr	p15,0,r3,c11,c2,0	  @ write ple channel number register

20:     bx	lr
ENDPROC(restore_a8_other)

	.end




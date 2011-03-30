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

#define MIDR_CPU_MASK 0xff00fff0

	.syntax unified


	@ This function takes three arguments
	@ r0: Destination start address (must be word aligned)
	@ r1: Source start address (must be word aligned)
	@ r2: Number of words to copy
	@ Return value is updated destination pointer (first unwritten word)
ENTRY(copy_words)
	cmp	r2, #0
	beq	1f
0:      ldr	r3, [r1], #4
	str	r3, [r0], #4
	subs	r2, r2, #1
	bne	0b
1:      bx	lr
ENDPROC(copy_word)

ENTRY(read_mvbar)
	mrc	p15, 0, r0, c12, c0, 1
	bx	lr
ENDPROC(read_mvbar)

ENTRY(read_cbar)
	mrc	p15, 4, r0, c15, c0, 0	     @ Read Configuration Base Address
					     @ Register
	bx	lr
ENDPROC(read_cbar)

	@ Functions we need in the runtime entry point, i.e. before we
	@ switch pagetables, are placed in this area.

ENTRY(read_mpidr)
	mrc	p15, 0, r0, c0, c0, 5
	bx	lr
ENDPROC(read_mpidr)

ENTRY(read_sctlr)
	mrc     p15, 0, r0, c1, c0, 0
	bx      lr
ENDPROC(read_sctlr)

ENTRY(write_sctlr)
	mcr     p15, 0, r0, c1, c0, 0
	bx      lr
ENDPROC(write_sctlr)

	.end

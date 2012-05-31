#include <linux/linkage.h>
#include <asm/asm-offsets.h>

	.text
ENTRY(disable_clean_inv_dcache)
 ARM(	stmfd	sp!, {r4-r5, r7, r9-r11, lr}	)
 THUMB(	stmfd	sp!, {r4-r7, r9-r11, lr}	)

	mrc	p15, 0, r3, c1, c0, 0
	bic	r3, #4			@ clear C bit
	mcr	p15, 0, r3, c1, c0, 0
	isb
	mrc	p15, 1, r0, c0, c0, 1
	ands	r0, r0, #0xe00000
	mov	r0, r0, lsr #21
	blne	v7_flush_dcache_level
	mrc	p15, 0, r3, c1, c0, 1
	bic	r3, #0x40		@ clear SMP bit
	mcr	p15, 0, r3, c1, c0, 1
	dsb
	isb
 ARM(	ldmfd	sp!, {r4-r5, r7, r9-r11, lr}	)
 THUMB(	ldmfd	sp!, {r4-r7, r9-r11, lr}	)
	mov	pc, lr
ENDPROC(disable_clean_inv_dcache)

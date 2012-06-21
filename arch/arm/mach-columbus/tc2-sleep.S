#include <linux/linkage.h>
ENTRY(tc2_cpu_resume)
	mrc	p15, 0, r0, c0, c0, 5
	ands	r0, r0, #0xff00
	adr	r0, value
	addne	r0, r0, #12
	ldmia	r0, {r1, r2, r3} 	@ CCI address, SCC snoop control & val
	mvn	r3, r3			@ undo actions done at shutdown
	ldr	r0, [r2]
	and	r4, r0, r3
	str	r4, [r2]
	mov	r0, #1			@ enable CCI for the cluster
	str	r0, [r1]
	b 	cpu_resume
ENDPROC(tc2_cpu_resume)

value:
	.long 	0x2c094000
	.long 	0x7fff0404
	.long 	0x180
	.long 	0x2c095000
	.long 	0x7fff0504
	.long 	0x2000

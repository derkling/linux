@ copyright (c) 2008-2010 arm limited

@ this software is provided 'as-is', without any express or implied
@ warranties including the implied warranties of satisfactory quality,
@ fitness for purpose or non infringement.  in no event will  arm be
@ liable for any damages arising from the use of this software.

@ permission is granted to anyone to use, copy and modify this software for
@ any purpose, and to redistribute the software, subject to the following
@ restrictions:

@ 1. the origin of this software must not be misrepresented; you must not
@    claim that you wrote the original software. if you use this software
@    in a product, an acknowledgment in the product documentation would be
@    appreciated but is not required.
@ 2. altered source versions must be plainly marked as such, and must not be
@    misrepresented as being the original software.
@ 3. this notice may not be removed or altered from any source distribution.

#include <linux/linkage.h>
#include <generated/asm-offsets.h>
#include <asm/memory.h>
#include <asm/thread_info.h>
#include <mach/memory.h>

#include "appf_defs.h"

@ aliases for mode encodings - do not change
#define MODE_USR 0x10
#define MODE_FIQ 0x11
#define MODE_IRQ 0x12
#define MODE_SVC 0x13
#define MODE_ABT 0x17
#define MODE_UND 0x1b
#define MODE_SYS 0x1f
#define OFFSET (PAGE_OFFSET - PLAT_PHYS_OFFSET)

#define MODE_MON 0x16 @@ a-profile (security extensions) only
#define SCR_NS 0x01 @@ a-profile (security extensions) only

#define CACHE_LINE_SIZE 32 @@ todo: remove this
#define CACHE_LINE_MASK 31 @@ todo: remove this
#define OFFSET (PAGE_OFFSET - PLAT_PHYS_OFFSET)

        .syntax unified
ENTRY(save_performance_monitors)
	push	{r4, r8, r9, r10}
	@ ignore:
	@        count enable clear register
	@        software increment register
	@        interrupt enable clear register
	mrc	p15,0,r8,c9,c12,0	  @ pmon: control register
	bic	r1,r8,#1
	mcr	p15,0,r1,c9,c12,0	  @ disable counter updates from here
	isb				  @ 0b0 => pmcr<0>
	mrc	p15,0,r9,c9,c12,3	  @ pmon: overflow flag status reg
	mrc	p15,0,r10,c9,c12,5	  @ pmon: event counter selection reg
	stm	r0!, {r8-r10}
	ubfx	r9,r8,#11,#5		  @ extract # of event counters, n
	tst	r9, r9
	beq	1f
0:	subs	r9,r9,#1		  @ decrement n
	mcr	p15,0,r9,c9,c12,5	  @ pmon: select countern
	isb
	mrc	p15,0,r3,c9,c13,1	  @ pmon: save event type register
	mrc	p15,0,r4,c9,c13,2	  @ pmon: save event counter register
	stm	r0!, {r3,r4}
	bne	0b
1:	mrc	p15,0,r1,c9,c13,0	  @ pmon: cycle count register
	mrc	p15,0,r2,c9,c14,0	  @ pmon: user enable register
	mrc	p15,0,r3,c9,c14,1	  @ pmon: interrupt enable set reg
	mrc	p15,0,r4,c9,c12,1	  @ pmon: count enable set register
	stm	r0!, {r1-r4}
	pop	{r4, r8, r9, r10}
	bx	lr
ENDPROC(save_performance_monitors)

ENTRY(restore_performance_monitors)
	push	{r4-r5, r8-r10, lr}
	@ note: all counters disabled by pmcr<0> == 0 on reset

	@ restore performance counters
	ldm	r0!,{r8-r10}		@ recover first block of pmon context
	@ (pmcr, pmovsr, pmselr)
	mov	r1, #0		    	@ generate register of all 0's
	mvn	r2, #0		    	@ generate register of all 1's
	mcr	p15,0,r2,c9,c14,2	@ disable all counter related interrupts
	mcr	p15,0,r2,c9,c12,3	@ clear all overflow flags
	isb

	ubfx	r12,r8,#11,#5	     	@ extract # of event counters, n (0-31)
	tst	r12, r12
	beq	20f
	mov	r3, r12		   	@ for n >0, generate a 2nd copy of n
	mov	r4, #1
	lsl	r4, r4, r3
	sub	r4, r4, #1	     	@ set bits<n-1:0> to all 1's

0:      subs    r3,r3,#1                @ decrement n
	mcr	p15,0,r3,c9,c12,5       @ select event countern
	isb
	mrc	p15,0,r5,c9,c13,1       @ read event type register
	bfc	r5,#0,#8
	mcr	p15,0,r5,c9,c13,1       @ set event type to 0x0
	mcr	p15,0,r2,c9,c13,2       @ set event counter to all 1's
	isb
	bne	0b

	mov	r3, #1
	bic	r5, r9, #1<<31
	mcr	p15,0,r5,c9,c12,1	@ enable event counters
	@ (pmovsr bits set)
	mcr	p15,0,r3,c9,c12,0	 @ set the pmcr global enable bit
	isb
	mcr	p15,0,r9,c9,c12,4       @ set event count overflow bits
	isb
	mcr	p15,0,r4,c9,c12,2       @ disable event counters

	@ restore the event counters
10:     subs	r12,r12,#1              @ decrement n
	mcr	p15,0,r12,c9,c12,5      @ select event countern
	isb
	ldm	r0!,{r3-r4}
	mcr	p15,0,r3,c9,c13,1       @ restore event type
	mcr	p15,0,r4,c9,c13,2       @ restore event counter
	isb
	bne	10b

20:     tst	r9, #0x80000000		   @ check for cycle count overflow flag
	beq	40f
	mcr	p15,0,r2,c9,c13,0	  @ set cycle counter to all 1's
	isb
	mov	r3, #0x80000000
	mcr	p15,0,r3,c9,c12,1	  @ enable the cycle counter
	isb
30:
	mrc	p15,0,r4,c9,c12,3	  @ check cycle count overflow now set
	movs	r4,r4			  @ test bit<31>
	bpl	30b
	mcr	p15,0,r3,c9,c12,2	  @ disable the cycle counter
40:
	mcr	p15,0,r1,c9,c12,0	  @ clear the pmcr global enable bit
	isb
	@ restore the remaining pmon registers
	ldm	r0!,{r1-r4}
	mcr	p15,0,r1,c9,c13,0	  @ restore cycle count register
	mcr	p15,0,r2,c9,c14,0	  @ restore user enable register
	mcr	p15,0,r3,c9,c14,1	  @ restore interrupt enable set reg
	mcr	p15,0,r4,c9,c12,1	  @ restore count enable set register
	mcr	p15,0,r10,c9,c12,5	  @ restore event counter selection
	isb
	mcr	p15,0,r8,c9,c12,0	  @ restore the pm control register
	isb

	pop	{r4-r5, r8-r10, pc}
ENDPROC(restore_performance_monitors)


ENTRY(save_banked_registers)
	mrs	r2, cpsr		  @ save current mode
	cps	#MODE_SYS		  @ switch to system mode
	str	sp,[r0], #4		  @ save the user sp
	str	lr,[r0], #4		  @ save the user lr
	cps	#MODE_ABT		  @ switch to abort mode
	str	sp,[r0], #4		  @ save the current sp
	mrs	r3,spsr
	stm	r0!,{r3,lr}		  @ save the current spsr, lr
	cps	#MODE_UND		  @ switch to undefined mode
	str	sp,[r0], #4		  @ save the current sp
	mrs	r3,spsr
	stm	r0!,{r3,lr}		  @ save the current spsr, lr
	cps	#MODE_IRQ		  @ switch to irq mode
	str	sp,[r0], #4		  @ save the current sp
	mrs	r3,spsr
	stm	r0!,{r3,lr}		  @ save the current spsr, lr
	cps	#MODE_FIQ		  @ switch to fiq mode
	str	sp,[r0], #4		  @ save the current sp
	mrs	r3,spsr
	stm	r0!,{r8-r12,lr}		  @ save the current spsr,r8-r12,lr
	msr	cpsr_cxsf, r2		  @ switch back to original mode
	bx	lr
ENDPROC(save_banked_registers)

ENTRY(restore_banked_registers)
	mrs	r2, cpsr		  @ save current mode
	cps	#MODE_SYS		  @ switch to system mode
	ldr	sp,[r0],#4		  @ restore the user sp
	ldr	lr,[r0],#4		  @ restore the user lr
	cps	#MODE_ABT		  @ switch to abort mode
	ldr	sp,[r0],#4		  @ restore the current sp
	ldm	r0!,{r3,lr}		  @ restore the current lr
	msr	spsr_fsxc,r3		  @ restore the current spsr
	cps	#MODE_UND		  @ switch to undefined mode
	ldr	sp,[r0],#4		  @ restore the current sp
	ldm	r0!,{r3,lr}		  @ restore the current lr
	msr	spsr_fsxc,r3		  @ restore the current spsr
	cps	#MODE_IRQ		  @ switch to irq mode
	ldr	sp,[r0],#4		  @ restore the current sp
	ldm	r0!,{r3,lr}		  @ restore the current lr
	msr	spsr_fsxc,r3		  @ restore the current spsr
	cps	#MODE_FIQ		  @ switch to fiq mode
	ldr	sp,[r0],#4		  @ restore the current sp
	ldm	r0!,{r8-r12,lr}		  @ restore the current r8-r12,lr
	msr	spsr_fsxc,r4		  @ restore the current spsr
	msr	cpsr_cxsf, r2		  @ switch back to original mode
0:
	 bx	lr
ENDPROC(restore_banked_registers)
ENTRY(save_cp15)
	@ csselr cache size selection register
	mrc	p15,2,r3,c0,c0,0
	str	r3,[r0], #4

	@ implementation defined - proprietary features:
	@ (cp15 register 15, tcm support, lockdown support, etc.)

	@ note: imp def registers might have save and restore order that relate
	@ to other cp15 registers or logical grouping requirements and can
	@ therefore occur at any point in this sequence.
	bx	lr
ENDPROC(save_cp15)

ENTRY(restore_cp15)
	@ csselr cache size selection register
	ldr	r3,[r0], #4
	mcr	p15,2,r3,c0,c0,0
	bx	lr
ENDPROC(restore_cp15)

	@ function called with two arguments:
	@	r0 contains address to store control registers
	@	r1 is non-zero if we are secure
ENTRY(save_control_registers)
	mrc	p15,0,r2,c1,c0,1	  @ actlr - auxiliary control register
	mrc	p15,0,r3,c1,c0,0	  @ sctlr - system control register
	mrc	p15,0,r12,c1,c0,2	  @ cpacr - coprocessor access control
					  @ register
	stm	r0!, {r2-r3, r12}
	cmp	r1, #0			  @ are we secure?
	beq	0f
	mrc	p15,0,r1,c12,c0,1	  @ mvbar - monitor vector base
					  @ address register
	mrc	p15,0,r2,c1,c1,0	  @ secure configuration register
	mrc	p15,0,r3,c1,c1,1	  @ secure debug enable register
	mrc	p15,0,r12,c1,c1,2	  @ non-secure access control register
	stm	r0!, {r1-r3,r12}
0:                  bx	lr
ENDPROC(save_control_registers)


	@ function called with two arguments:
	@	r0 contains address to read control registers
	@	r1 is non-zero if we are secure
ENTRY(restore_control_registers)
	ldm	r0!, {r2-r3, r12}
	mcr	p15,0,r2,c1,c0,1	  @ actlr - auxiliary control register
	mcr	p15,0,r3,c1,c0,0	  @ sctlr - system control register
	mcr	p15,0,r12,c1,c0,2	  @ cpacr - coprocessor access control
					  @ register
	cmp	r1, #0			  @ are we secure?
	beq	0f
	ldm	r0!, {r1-r3,r12}
	mcr	p15,0,r1,c12,c0,1	  @ mvbar - monitor vector base
					  @ address register
	mcr	p15,0,r2,c1,c1,0	  @ secure configuration register
	mcr	p15,0,r3,c1,c1,1	  @ secure debug enable register
	mcr	p15,0,r12,c1,c1,2	  @ non-secure access control register
0:                  isb
	add	sp, sp, #OFFSET 	@ running virtual then
	add	lr, lr, #OFFSET 	@ going back to kernel virtual address
	push	{r0, lr}
	bl 	appf_platform_get_stack_pointer
	sub	r2, sp, #OFFSET
	cmp	r2, r0
	movlt 	r1, r0
	movlt	r0, r2
	ldrlt	r2, =outer_cache
	ldrlt	r2, [r2, #OUTER_CACHE_INV_RANGE]
	blxlt	r2
	pop	{r0, pc}
ENDPROC(restore_control_registers)

ENTRY(save_mmu)
	push	{r4, r5, r6, r7}
	@ assumption: no useful fault address / fault status information

	mrc	p15,0,r4,c3,c0,0	   @ dacr
	mrc	p15,0,r5,c2,c0,0	   @ ttbr0
	mrc	p15,0,r6,c2,c0,1	   @ ttbr1
	mrc	p15,0,r7,c2,c0,2	   @ ttbcr
	stm	r0!, {r4-r7}

	mrc	p15,0,r4,c12,c0,0	  @ vbar
	mrc	p15,0,r5,c7,c4,0	  @ par
	mrc	p15,0,r6,c10,c2,0	  @ prrr
	mrc	p15,0,r7,c10,c2,1	  @ nmrr
	stm	r0!, {r4-r7}

	@ todo: implementation defined - tcm, lockdown and performance
	@ monitor support
	@ cp15 registers 9 and 11

	mrc	p15,0,r4,c13,c0,1	  @ contextidr
	mrc	p15,0,r5,c13,c0,2	  @ tpidrurw
	mrc	p15,0,r6,c13,c0,3	  @ tpidruro
	mrc	p15,0,r7,c13,c0,4	  @ tpidrprw
	stm	r0!, {r4-r7}

	pop	{r4, r5, r6, r7}
	bx	lr
ENDPROC(save_mmu)


ENTRY(restore_mmu)

	push	{r4, r5, r6, r7}
	ldm	r0!, {r4-r7}
	mcr	p15,0,r4,c3,c0,0	   @ dacr
	mcr	p15,0,r5,c2,c0,0	   @ ttbr0
	mcr	p15,0,r6,c2,c0,1	   @ ttbr1
	mcr	p15,0,r7,c2,c0,2	   @ ttbcr

	ldm	r0!, {r4-r7}
	mcr	p15,0,r4,c12,c0,0	  @ vbar
	mcr	p15,0,r5,c7,c4,0	  @ par
	mcr	p15,0,r6,c10,c2,0	  @ prrr
	mcr	p15,0,r7,c10,c2,1	  @ nmrr

	@ todo: implementation defined - tcm, lockdown and performance
	@ monitor support
	@ cp15 registers 9 and 11

	ldm	r0!, {r4-r7}
	mcr	p15,0,r4,c13,c0,1	  @ contextidr
	mcr	p15,0,r5,c13,c0,2	  @ tpidrurw
	mcr	p15,0,r6,c13,c0,3	  @ tpidruro
	mcr	p15,0,r7,c13,c0,4	  @ tpidrprw

	pop	{r4, r5, r6, r7}
	bx	lr
ENDPROC(restore_mmu)


ENTRY(save_vfp)
	@ fpu state save/restore.
	@ fpsid,mvfr0 and mvfr1 don't get serialized/saved (read only).
	mrc	p15,0,r3,c1,c0,2	   @ cpacr allows cp10 and cp11 access
	orr	r2,r3,#0xf00000
	mcr	p15,0,r2,c1,c0,2
	isb
	mrc	p15,0,r2,c1,c0,2
	and	r2,r2,#0xf00000
	cmp	r2,#0xf00000
	beq	0f
	movs	r2, #0
	@ override to 0 to indicate that no fpu is present
	@	str     r2,[r11,#dm_vfp] @ todo: autodetect vfp in c!!
	b	2f

0:                      @	save configuration registers and enable.
	mrs	r12,fpexc
	str	r12,[r0],#4		   @ save the fpexc
	@ enable fpu access to save/restore the other registers.
	ldr	r2,=0x40000000
	msr	fpexc,r2
	mrs	r2,fpscr
	str	r2,[r0],#4		   @ save the fpscr
	@ store the vfp-d16 registers.
	vstm	r0!, {d0-d15}
	@ check for advanced simd/vfp-d32 support
	mrs	r2,mvfr0
	and	r2,r2,#0xf		   @ extract the a_simd bitfield
	cmp	r2, #0x2
	blt	1f
	@ store the advanced simd/vfp-d32 additional registers.
	vstm	r0!, {d16-d31}

	@ implementation defined: save any subarchitecture defined state
	@ note: don't change the order of the fpexc and cpacr restores
1:
	msr	fpexc,r12           @ restore the original en bit of fpu.
2:
	mcr	p15,0,r3,c1,c0,2    @ restore the original cpacr value.
	bx	lr
ENDPROC(save_vfp)


ENTRY(restore_vfp)
	@ fpu state save/restore. obviously fpsid,mvfr0 and mvfr1 don't get
	@ serialized (ro).
	@ modify cpacr to allow cp10 and cp11 access
	mrc	p15,0,r1,c1,c0,2
	orr	r2,r1,#0x00f00000
	mcr	p15,0,r2,c1,c0,2
	@ enable fpu access to save/restore the rest of registers.
	ldr	r2,=0x40000000
	msr	fpexc, r2
	@ recover fpexc and fpscr. these will be restored later.
	ldm	r0!,{r3,r12}
	@ restore the vfp-d16 registers.
	vldm	r0!, {d0-d15}
	@ check for advanced simd/vfp-d32 support
	mrs	r2, mvfr0
	and	r2,r2,#0xf		    @ extract the a_simd bitfield
	cmp	r2, #0x2
	blt	0f

	@ store the advanced simd/vfp-d32 additional registers.
	vldm	r0!, {d16-d31}

	@ implementation defined: restore any subarchitecture defined state

0:                      @ restore configuration registers and enable.
	@ restore fpscr _before_ fpexc since fpexc could disable fpu
	@ and make setting fpscr unpredictable.
	msr	fpscr,r12
	msr	fpexc,r3		  @ restore fpexc after fpscr
	@ cpacr (c1,c0,2) is restored later.
	mcr	p15,0,r1,c1,c0,2
	bx	lr
ENDPROC(restore_vfp)


	@ this function disables l2 data caching, then cleans and invalidates
	@ the stack in l2.
	@ r0 contains the pl310 address
	@ r1 contains the stack start addres
	@ r2 contains the stack size
	@ r3 contains 1 if the l2 is to be disabled, 0 if it is to be left on.

ENTRY(disable_clean_inv_cache_pl310)
	@ should we disable the l2 cache?
	cmp	r1, #0
	beq	2f

	@ sync and disable l2 cache
	mov	r3, #0
	str	r3, [r0, #0x730]	   @ pl310: cache sync register
	dsb
	ldr	r3, [r0, #0x100]	   @ pl310: control register
	bic	r3, #1
	str	r3, [r0, #0x100]

2:                      @ disable mmu
	adr	r3, 3f
	sub	r3, r3, #OFFSET
	bx 	r3
3:
	dsb
	mrc	p15, 0, r3, c1, c0, 0
	bic	r3, #1			   @ clear m bit
	mcr	p15, 0, r3, c1, c0, 0
	dsb

	sub	sp, sp, #OFFSET		   @ sp running physical from now on
	sub	lr, lr, #OFFSET		   @ pc running physical on return

	mov	r1, sp
	lsr	r2, r1, #13
	lsl     r2, r2, #13
	add 	r2, r2, #THREAD_SIZE
	sub	r2, r2, r1
	@ clean+invalidate stack in l2
	bic	r1, #CACHE_LINE_MASK
5:                  cmp	r2, #0
	blt	10f
	str	r1, [r0, #0x7f0]	   @ pl310: clean and invalidate by pa
	add	r1, #CACHE_LINE_SIZE
	subs	r2, #CACHE_LINE_SIZE
	b	5b
10:     bx	lr        		   @ stack invalidated, we can return
					   @ to C

ENDPROC(disable_clean_inv_cache_pl310)

	@ this function disables l1 data caching, then cleans and invalidates
	@ the whole l1 data cache.

ENTRY(disable_clean_inv_stack)
	push	{r4, lr}

	@ disable l1 cache
	dsb
	mrc	p15, 0, r3, c1, c0, 0
	bic	r3, #4			   @ clear c bit
	mcr	p15, 0, r3, c1, c0, 0
	dsb


	@ no more data cache allocations can happen at l1.
	@ until we finish cleaning the inner cache, any accesses to dirty data
	@ (e.g. by translation table walks) may get the wrong (outer) data, so
	@ we have to be sure everything that might be accessed is clean.
	@ we already know that the translation tables are clean (see late_init).

	mov	r0, #0			  @ select l1 data/unified cache
	mcr	p15,2,r0,c0,c0,0
	mrc	p15,1,r0,c0,c0,0	  @ read size
	ubfx	r3, r0, #13, #15	  @ sets - 1
	add	r3, r3, #1		  @ sets
	ubfx	r4, r0, #0, #3		  @ log2(words per line) - 2
	add	r4, r4, #4		  @ set shift = log2(bytes per line)
	ubfx	r2, r0, #3, #10		  @ ways - 1
	clz	r12, r2			  @ way shift
	add	r2, r2, #1		  @ ways

	@ r2,r3 inner, outer loop targets, r1 inner loop counter, r0 zero
5:                  cmp	r3, #0
	beq	20f
	sub	r3, r3, #1
	mov	r1, r2

10:                 cmp	r1, #0
	beq	5b
	sub	r1, r1, #1
	mov	r0, r1, lsl r12		  @ fill in way field
	orr	r0, r0, r3, lsl r4	  @ fill in set field
	mcr	p15,0,r0,c7,c14,2	  @ dccisw
	b	10b
20:
	mrc     p15, 0, r3, c1, c0, 1
	bic     r3, #64
	mcr     p15, 0, r3, c1, c0, 1
	dsb
	pop	{r4, lr}
	bx	lr

ENDPROC(disable_clean_inv_stack)

ENTRY(disable_clean_inv_dcache_v7_l1)
	push	{r4, lr}

	@ disable l1 cache
	dsb
	mrc	p15, 0, r3, c1, c0, 0
	bic	r3, #4			   @ clear c bit
	mcr	p15, 0, r3, c1, c0, 0
	dsb

	@ no more data cache allocations can happen at l1.
	@ until we finish cleaning the inner cache, any accesses to dirty data
	@ (e.g. by translation table walks) may get the wrong (outer) data, so
	@ we have to be sure everything that might be accessed is clean.
	@ we already know that the translation tables are clean (see late_init).

	mov	r0, #0			  @ select l1 data/unified cache
	mcr	p15,2,r0,c0,c0,0
	mrc	p15,1,r0,c0,c0,0	  @ read size
	ubfx	r3, r0, #13, #15	  @ sets - 1
	add	r3, r3, #1		  @ sets
	ubfx	r4, r0, #0, #3		  @ log2(words per line) - 2
	add	r4, r4, #4		  @ set shift = log2(bytes per line)
	ubfx	r2, r0, #3, #10		  @ ways - 1
	clz	r12, r2			  @ way shift
	add	r2, r2, #1		  @ ways

	@ r2,r3 inner, outer loop targets, r1 inner loop counter, r0 zero
5:      cmp	r3, #0
	beq	20f
	sub	r3, r3, #1
	mov	r1, r2

10:     cmp	r1, #0
	beq	5b
	sub	r1, r1, #1
	mov	r0, r1, lsl r12		  @ fill in way field
	orr	r0, r0, r3, lsl r4	  @ fill in set field
	mcr	p15,0,r0,c7,c14,2	  @ dccisw
	b	10b

20:     dsb
	pop	{r4, lr}
	bx	lr
ENDPROC(disable_clean_inv_dcache_v7_l1)


ENTRY(invalidate_icache_v7_pou)
	mov     r0, #0
	mcr     p15, 0, r0, c7, c5, 0           @ iciallu
	bx	lr
ENDPROC(invalidate_icache_v7_pou)

ENTRY(invalidate_dcache_v7_all)
	@ must iterate over the caches in order to synthesise a complete
	@ invalidation of data/unified cache
	push    {r4-r11}
	mrc     p15, 1, r0, c0, c0, 1           @ read clidr
	ands    r3, r0, #0x7000000              @ extract loc from clidr
	mov     r3, r3, lsr #23                 @ left align loc bit field
	beq     finished                        @ if loc is 0, then no need to
						@ clean
	mov     r10, #0                         @ start clean at cache level 0
						@ (in r10)
loop1:
	add     r2, r10, r10, lsr #1            @ work out 3x current cache
						@ level
	mov     r12, r0, lsr r2                 @ extract cache type bits from
						@ clidr
	and     r12, r12, #7                    @ mask of bits for current
						@ cache only
	cmp     r12, #2                         @ see what cache we have at
						@ this level
	blt     skip                            @ skip if no cache, or just
						@ i-cache
	mcr     p15, 2, r10, c0, c0, 0          @ select current cache level
						@ in cssr
	mov     r12, #0
	mcr     p15, 0, r12, c7, c5, 4          @ prefetchflush to sync new
						@ cssr&csidr
	mrc     p15, 1, r12, c0, c0, 0          @ read the new csidr
	and     r2, r12, #7                     @ extract the length of the
						@cache lines
	add     r2, r2, #4                      @ add 4 (line length offset)
	ldr     r6, =0x3ff
	ands    r6, r6, r12, lsr #3             @ find maximum number on the
						@ way size
	clz     r5, r6                          @ find bit pos of way size
						@ increment
	ldr     r7, =0x7fff
	ands    r7, r7, r12, lsr #13            @ extract max number of the
						@ index size
loop2:
	mov     r8, r6                          @ create working copy of max
						@ way size
loop3:
	orr     r11, r10, r8, lsl r5            @ factor way and cache number
						@ into r11
	orr     r11, r11, r7, lsl r2            @ factor index number into r11
	mcr     p15, 0, r11, c7, c6, 2          @ invalidate by set/way
	subs    r8, r8, #1                      @ decrement the way
	bge     loop3
	subs    r7, r7, #1                      @ decrement the index
	bge     loop2
skip:
	add     r10, r10, #2                    @ increment cache number
	cmp     r3, r10
	bgt     loop1
finished:
	mov     r10, #0

	mcr     p15, 0, r10, c7, c10, 4         @ drain write buffer
	mcr     p15, 0, r10, c8, c7, 0          @ invalidate i + d tlbs
	mcr     p15, 0, r10, c2, c0, 2          @ ttb control register
	pop     {r4-r11}
	bx      lr
ENDPROC(invalidate_dcache_v7_all)


ENTRY(disable_clean_inv_dcache_v7_all)
	@ must iterate over the caches in order to synthesise a complete clean
	@ of data/unified cache
	push    {r4-r11}

	@ disable integrated data/unified cache
	dsb
	mrc	p15, 0, r3, c1, c0, 0
	bic	r3, #4			   @ clear c bit
	mcr	p15, 0, r3, c1, c0, 0
	dsb

	@ no more data cache allocations can happen.
	@ until we finish cleaning the cache, any accesses to dirty data
	@ (e.g. by translation table walks) may get the wrong (outer) data, so
	@ we have to be sure everything that might be accessed is clean.
	@ we already know that the translation tables are clean (see late_init).


	mrc     p15, 1, r0, c0, c0, 1           @ read clidr
	ands    r3, r0, #0x7000000              @ extract loc from clidr
	mov     r3, r3, lsr #23                 @ left align loc bit field
	beq     50f                             @ if loc is 0, then no need to
						@ clean
	mov     r10, #0                         @ start clean at cache level 0
						@ (in r10)
10:
	add     r2, r10, r10, lsr #1            @ work out 3x current cache
						@ level
	mov     r12, r0, lsr r2                 @ extract cache type bits from
						@ clidr
	and     r12, r12, #7                    @ mask of bits for current
						@ cache only
	cmp     r12, #2                         @ see what cache we have at
						@ this level
	blt     40f                             @ skip if no cache, or just
						@ i-cache
	mcr     p15, 2, r10, c0, c0, 0          @ select current cache level
						@ in cssr
	mov     r12, #0
	mcr     p15, 0, r12, c7, c5, 4          @ prefetchflush to sync new
						@ cssr&csidr
	mrc     p15, 1, r12, c0, c0, 0          @ read the new csidr
	and     r2, r12, #7                     @ extract the length of the
						@ cache lines
	add     r2, r2, #4                      @ add 4 (line length offset)
	ldr     r6, =0x3ff
	ands    r6, r6, r12, lsr #3             @ find maximum number on the
						@ way size
	clz     r5, r6                          @ find bit pos of way size
						@ increment
	ldr     r7, =0x7fff
	ands    r7, r7, r12, lsr #13            @ extract max number of the
						@ index size
20:
	mov     r8, r6                          @ create working copy of max
						@ way size
30:
	orr     r11, r10, r8, lsl r5            @ factor way and cache number
						@ into r11
	orr     r11, r11, r7, lsl r2            @ factor index number into r11
	mcr     p15, 0, r11, c7, c14, 2         @ clean & invalidate by set/way
	subs    r8, r8, #1                      @ decrement the way
	bge     30b
	subs    r7, r7, #1                      @ decrement the index
	bge     20b
40:
	add     r10, r10, #2                    @ increment cache number
	cmp     r3, r10
	bgt     10b
50:
	mov     r10, #0
	mcr     p15, 0, r10, c7, c10, 4         @ drain write buffer
	pop     {r4-r11}
	bx	lr
ENDPROC(disable_clean_inv_dcache_v7_all)


	@ this function cleans the whole l1 data cache
ENTRY(clean_dcache_v7_l1)
	push	{r4, lr}

	mov	r0, #0			  @ select l1 data/unified cache
	mcr	p15,2,r0,c0,c0,0
	mrc	p15,1,r0,c0,c0,0	  @ read size (ccsidr)
	ubfx	r3, r0, #13, #15	  @ sets - 1
	add	r3, r3, #1		  @ sets
	ubfx	r4, r0, #0, #3		  @ log2(words per line) - 2
	add	r4, r4, #4		  @ set shift = log2(bytes per line)
	ubfx	r2, r0, #3, #10		  @ ways - 1
	clz	r12, r2			  @ way shift
	add	r2, r2, #1		  @ ways

	@ r2,r3 inner, outer loop targets, r1 inner loop counter, r0 zero
0:      cmp	r3, #0
	beq	20f
	sub	r3, r3, #1
	mov	r1, r2

10:     cmp	r1, #0
	beq	0b
	sub	r1, r1, #1
	mov	r0, r1, lsl r12		  @ fill in way field
	orr	r0, r0, r3, lsl r4	  @ fill in set field
	mcr	p15,0,r0,c7,c10,2	  @ dccsw
	b	10b

20:     dsb
	pop	{r4, lr}
	bx	lr
ENDPROC(clean_dcache_v7_l1)

	@ this function cleans a single line from the l1 dcache
ENTRY(clean_mva_dcache_v7_l1)
	mcr	p15,0,r0,c7,c10,1	  @ dccmvac
	bx	lr
ENDPROC(clean_mva_dcache_v7_l1)

ENTRY(enter_secure_monitor_mode)
	mov	r0, lr
	mov	r1, sp
	smc	#0
appf_smc_handler:
	@ we are now in monitor mode, make sure we're secure
	mrc	p15, 0, r12, c1, c1, 0
	bic	r12, #SCR_NS
	mcr	p15, 0, r12, c1, c1, 0
	@ restore sp and return - stack must be uncached or in ns memory!
	mov	sp, r1
	bx	r0
ENDPROC(enter_secure_monitor_mode)

ENTRY(enter_nonsecure_svc_mode)
	@ copy the monitor mode sp and lr values
	mov	r2, lr
	mov	r3, sp
	mrc	p15, 0, r1, c1, c1, 0
	orr	r1, #SCR_NS
	mcr	p15, 0, r1, c1, c1, 0
	adr	lr, non_secure
	movs	pc, lr
non_secure:
	@ we are now in non-secure state
	@ restore sp and return
	mov	sp, r3
	bx	r2
ENDPROC(enter_nonsecure_svc_mode)

	.end
.data


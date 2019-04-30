/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _ASM_X86_ACRN_HYPERCALL_H
#define _ASM_X86_ACRN_HYPERCALL_H

#include <linux/errno.h>

#ifdef CONFIG_ACRN_GUEST

/*
 * Hypercalls for ACRN guest
 *
 * Hypercall number is passed in R8 register.
 * Up to 2 arguments are passed in RDI, RSI.
 * Return value will be placed in RAX.
 */

static inline long acrn_hypercall0(unsigned long hcall_id)
{
	long result;

	/* the hypercall is implemented with the VMCALL instruction.
	 * volatile qualifier is added to avoid that it is dropped
	 * because of compiler optimization.
	 */
	asm volatile("movq %[hcall_id], %%r8\n\t"
		     "vmcall\n\t"
		     : "=a" (result)
		     : [hcall_id] "g" (hcall_id)
		     : "r8");

	return result;
}

static inline long acrn_hypercall1(unsigned long hcall_id,
				   unsigned long param1)
{
	long result;

	asm volatile("movq %[hcall_id], %%r8\n\t"
		     "vmcall\n\t"
		     : "=a" (result)
		     : [hcall_id] "g" (hcall_id), "D" (param1)
		     : "r8");

	return result;
}

static inline long acrn_hypercall2(unsigned long hcall_id,
				   unsigned long param1,
				   unsigned long param2)
{
	long result;

	asm volatile("movq %[hcall_id], %%r8\n\t"
		     "vmcall\n\t"
		     : "=a" (result)
		     : [hcall_id] "g" (hcall_id), "D" (param1), "S" (param2)
		     : "r8");

	return result;
}

#else

static inline long acrn_hypercall0(unsigned long hcall_id)
{
	return -ENOTSUPP;
}

static inline long acrn_hypercall1(unsigned long hcall_id,
				   unsigned long param1)
{
	return -ENOTSUPP;
}

static inline long acrn_hypercall2(unsigned long hcall_id,
				   unsigned long param1,
				   unsigned long param2)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_ACRN_GUEST */
#endif /* _ASM_X86_ACRN_HYPERCALL_H */

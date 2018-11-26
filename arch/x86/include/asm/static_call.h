/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_STATIC_CALL_H
#define _ASM_STATIC_CALL_H

#include <asm/asm-offsets.h>

#ifdef CONFIG_HAVE_STATIC_CALL_INLINE

/*
 * This trampoline is only used during boot / module init, so it's safe to use
 * the indirect branch without a retpoline.
 */
#define __ARCH_STATIC_CALL_TRAMP_JMP(key, func)				\
	ANNOTATE_RETPOLINE_SAFE						\
	"jmpq *" __stringify(key) "+" __stringify(SC_KEY_func) "(%rip) \n"

#else /* !CONFIG_HAVE_STATIC_CALL_INLINE */

/*
 * Manually construct a 5-byte direct JMP to prevent the assembler from
 * optimizing it into a 2-byte JMP.
 */
#define __ARCH_STATIC_CALL_JMP_LABEL(key) ".L" __stringify(key ## _after_jmp)
#define __ARCH_STATIC_CALL_TRAMP_JMP(key, func)				\
	".byte 0xe9						\n"	\
	".long " #func " - " __ARCH_STATIC_CALL_JMP_LABEL(key) "\n"	\
	__ARCH_STATIC_CALL_JMP_LABEL(key) ":"

#endif /* !CONFIG_HAVE_STATIC_CALL_INLINE */

/*
 * For CONFIG_HAVE_STATIC_CALL_INLINE, this is a temporary trampoline which
 * uses the current value of the key->func pointer to do an indirect jump to
 * the function.  This trampoline is only used during boot, before the call
 * sites get patched by static_call_update().  The name of this trampoline has
 * a magical aspect: objtool uses it to find static call sites so it can create
 * the .static_call_sites section.
 *
 * For CONFIG_HAVE_STATIC_CALL, this is a permanent trampoline which
 * does a direct jump to the function.  The direct jump gets patched by
 * static_call_update().
 */
#define ARCH_DEFINE_STATIC_CALL_TRAMP(key, func)			\
	asm(".pushsection .text, \"ax\"				\n"	\
	    ".align 4						\n"	\
	    ".globl " STATIC_CALL_TRAMP_STR(key) "		\n"	\
	    ".type " STATIC_CALL_TRAMP_STR(key) ", @function	\n"	\
	    STATIC_CALL_TRAMP_STR(key) ":			\n"	\
	    __ARCH_STATIC_CALL_TRAMP_JMP(key, func) "           \n"	\
	    ".popsection					\n")

#endif /* _ASM_STATIC_CALL_H */

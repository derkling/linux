/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_STATIC_CALL_H
#define _ASM_STATIC_CALL_H

/*
 * Manually construct a 5-byte direct JMP to prevent the assembler from
 * optimizing it into a 2-byte JMP.
 */
#define __ARCH_STATIC_CALL_JMP_LABEL(key) ".L" __stringify(key ## _after_jmp)
#define __ARCH_STATIC_CALL_TRAMP_JMP(key, func)				\
	".byte 0xe9						\n"	\
	".long " #func " - " __ARCH_STATIC_CALL_JMP_LABEL(key) "\n"	\
	__ARCH_STATIC_CALL_JMP_LABEL(key) ":"

/*
 * This is a permanent trampoline which does a direct jump to the function.
 * The direct jump get patched by static_call_update().
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

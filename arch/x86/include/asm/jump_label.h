/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_JUMP_LABEL_H
#define _ASM_X86_JUMP_LABEL_H

#define HAVE_JUMP_LABEL_BATCH

#ifdef CONFIG_X86_64
# define STATIC_KEY_NOP2 P6_NOP2
# define STATIC_KEY_NOP5 P6_NOP5_ATOMIC
#else
# define STATIC_KEY_NOP2 GENERIC_NOP2
# define STATIC_KEY_NOP5 GENERIC_NOP5_ATOMIC
#endif

#include <asm/asm.h>
#include <asm/nops.h>

/* XXX AS_HAS_NOPS_DIRECTIVE */
#define USE_VARIABLE_JMP 1

#ifndef __ASSEMBLY__

#include <linux/stringify.h>
#include <linux/types.h>

#define JUMP_TABLE_ENTRY				\
	".pushsection __jump_table,  \"aw\" \n\t"	\
	_ASM_ALIGN "\n\t"				\
	".long 1b - . \n\t"				\
	".long %l[l_yes] - . \n\t"			\
	_ASM_PTR "%c0 + %c1 - .\n\t"			\
	".popsection \n\t"

static __always_inline bool arch_static_branch(struct static_key *key, bool branch)
{
	asm_volatile_goto("1:"
#ifdef USE_VARIABLE_JMP
		/*
		 * The below depends on GNU as UB, as: '%l[l_yes] - (1b + 2)'
		 * could be crossing sections (.text and .text.unlikely), in
		 * which case the result is undefined.
		 *
		 * Any attempt to emit the result of either 'disp', 'res',
		 * 'is_byte' or 'is_long' will result in an assembler error,
		 * but somehow it works as an argument to .nops (and .skip)
		 * directives.
		 *
		 * In the case it does cross sections it results in: is_byte=0
		 * and is_long=1 which is exactly what is required and
		 * consistent is with what GNU as would've done for a jmp.
		 */
		".set disp, %l[l_yes] - (1b + 2) \n\t"
		".set res, (disp >> 31) == (disp >> 7) \n\t"
		".set is_byte, -res \n\t"
		".set is_long, -(~res) \n\t"

		/*
		 * The above STATIC_KEY_NOP* MUST be the same as emitted by
		 * .nops.
		 *
		 * This relies on .nops emitting single instruction nops for 2
		 * and 5 bytes, per experiment it does so but AFAICT that is not
		 * guaranteed behaviour.
		 */
		".nops (2*is_byte) + (5*is_long)\n\t"
#else
		".byte " __stringify(STATIC_KEY_NOP5) "\n\t"
#endif
		JUMP_TABLE_ENTRY
		: :  "i" (key), "i" (branch) : : l_yes);

	return false;
l_yes:
	return true;
}

static __always_inline bool arch_static_branch_jump(struct static_key *key, bool branch)
{
	asm_volatile_goto("1:"
#ifdef USE_VARIABLE_JMP
		"jmp %l[l_yes] \n\t"
#else
		".byte 0xe9 \n\t"
		".long %l[l_yes] - (. + 4) \n\t"
#endif
		JUMP_TABLE_ENTRY
		: :  "i" (key), "i" (branch) : : l_yes);

	return false;
l_yes:
	return true;
}

extern int arch_jump_entry_size(struct jump_entry *entry);

#else	/* __ASSEMBLY__ */

.macro STATIC_BRANCH_FALSE_LIKELY target, key
.Lstatic_jump_\@:
#ifdef USE_VARIABLE_JMP
	jmp \target
#else
	.byte		0xe9
	.long		\target - (. + 4)
#endif

	.pushsection __jump_table, "aw"
	_ASM_ALIGN
	.long		.Lstatic_jump_\@ - .
	.long		\target - .
	_ASM_PTR	\key + 1 - .
	.popsection
.endm

#endif	/* __ASSEMBLY__ */

#endif

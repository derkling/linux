// SPDX-License-Identifier: GPL-2.0
#include <linux/static_call.h>
#include <linux/memory.h>
#include <linux/bug.h>
#include <asm/text-patching.h>
#include <asm/nospec-branch.h>

#define CALL_INSN_SIZE 5

void arch_static_call_transform(void *site, void *tramp, void *func)
{
	unsigned char opcodes[CALL_INSN_SIZE];
	unsigned char insn_opcode;
	unsigned long insn;
	s32 dest_relative;

	mutex_lock(&text_mutex);

	insn = (unsigned long)tramp;

	insn_opcode = *(unsigned char *)insn;
	if (insn_opcode != 0xE9) {
		WARN_ONCE(1, "unexpected static call insn opcode 0x%x at %pS",
			  insn_opcode, (void *)insn);
		goto unlock;
	}

	dest_relative = (long)(func) - (long)(insn + CALL_INSN_SIZE);

	opcodes[0] = insn_opcode;
	memcpy(&opcodes[1], &dest_relative, CALL_INSN_SIZE - 1);

	text_poke_bp((void *)insn, opcodes, CALL_INSN_SIZE, NULL);

unlock:
	mutex_unlock(&text_mutex);
}
EXPORT_SYMBOL_GPL(arch_static_call_transform);

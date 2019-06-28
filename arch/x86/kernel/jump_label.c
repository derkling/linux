// SPDX-License-Identifier: GPL-2.0
/*
 * jump label x86 support
 *
 * Copyright (C) 2009 Jason Baron <jbaron@redhat.com>
 *
 */
#include <linux/jump_label.h>
#include <linux/memory.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/jhash.h>
#include <linux/cpu.h>
#include <asm/kprobes.h>
#include <asm/alternative.h>
#include <asm/text-patching.h>

union jump_code_union {
	char code[JMP32_INSN_SIZE];
	struct {
		char opcode;
		union {
			s8	d8;
			s32	d32;
		};
	} __attribute__((packed));
};

int arch_jump_entry_size(struct jump_entry *entry)
{
	return JMP32_INSN_SIZE;
}

static inline bool __jump_disp_is_byte(s32 disp)
{
	return (disp >> 31) == (disp >> 7);
}

static int __jump_label_set_jump_code(struct jump_entry *entry,
				      enum jump_label_type type,
				      union jump_code_union *code,
				      int init)
{
	static unsigned char default_nop2[] = { STATIC_KEY_NOP2 };
	static unsigned char default_nop5[] = { STATIC_KEY_NOP5 };
	s32 disp = jump_entry_target(entry) - jump_entry_code(entry);
	void *ip = (void *)jump_entry_code(entry);
	const unsigned char *nop;
	const void *expect;
	int line, size;

	size = arch_jump_entry_size(entry);
	disp -= size;
	if (size == JMP8_INSN_SIZE) {
		BUG_ON(!__jump_disp_is_byte(disp));
		code->opcode = JMP8_INSN_OPCODE;
		code->d8 = disp;
		nop = init ? default_nop2 : ideal_nops[2];
	} else {
		code->opcode = JMP32_INSN_OPCODE;
		code->d32 = disp;
		nop = init ? default_nop5 : ideal_nops[NOP_ATOMIC5];
	}

	if (init || type == JUMP_LABEL_JMP) {
		expect = nop; line = __LINE__;
	} else {
		expect = code->code; line = __LINE__;
	}

	if (memcmp(ip, expect, size)) {
		/*
		 * The location is not an op that we were expecting.
		 * Something went wrong. Crash the box, as something could be
		 * corrupting the kernel.
		 */
		pr_crit("jump_label: Fatal kernel bug, unexpected op at %pS [%p] (%5ph != %5ph)) line:%d init:%d size:%d type:%d\n",
			ip, ip, ip, expect, line, init, size, type);
		BUG();
	}

	if (type == JUMP_LABEL_NOP)
		memcpy(code, nop, size);

	return size;
}

static void __ref __jump_label_transform(struct jump_entry *entry,
					 enum jump_label_type type,
					 int init)
{
	union jump_code_union code;
	int size;

	size = __jump_label_set_jump_code(entry, type, &code, init);

	/*
	 * As long as only a single processor is running and the code is still
	 * not marked as RO, text_poke_early() can be used; Checking that
	 * system_state is SYSTEM_BOOTING guarantees it. It will be set to
	 * SYSTEM_SCHEDULING before other cores are awaken and before the
	 * code is write-protected.
	 *
	 * At the time the change is being done, just ignore whether we
	 * are doing nop -> jump or jump -> nop transition, and assume
	 * always nop being the 'currently valid' instruction
	 */
	if (init || system_state == SYSTEM_BOOTING) {
		text_poke_early((void *)jump_entry_code(entry), &code, size);
		return;
	}

	text_poke_bp((void *)jump_entry_code(entry), &code, size, NULL);
}

void arch_jump_label_transform(struct jump_entry *entry,
			       enum jump_label_type type)
{
	mutex_lock(&text_mutex);
	__jump_label_transform(entry, type, 0);
	mutex_unlock(&text_mutex);
}

#define TP_VEC_MAX (PAGE_SIZE / sizeof(struct text_poke_loc))
static struct text_poke_loc tp_vec[TP_VEC_MAX];
static int tp_vec_nr;

bool arch_jump_label_transform_queue(struct jump_entry *entry,
				     enum jump_label_type type)
{
	struct text_poke_loc *tp;
	void *entry_code;
	int size;

	if (system_state == SYSTEM_BOOTING) {
		/*
		 * Fallback to the non-batching mode.
		 */
		arch_jump_label_transform(entry, type);
		return true;
	}

	/*
	 * No more space in the vector, tell upper layer to apply
	 * the queue before continuing.
	 */
	if (tp_vec_nr == TP_VEC_MAX)
		return false;

	tp = &tp_vec[tp_vec_nr];

	entry_code = (void *)jump_entry_code(entry);

	/*
	 * The INT3 handler will do a bsearch in the queue, so we need entries
	 * to be sorted. We can survive an unsorted list by rejecting the entry,
	 * forcing the generic jump_label code to apply the queue. Warning once,
	 * to raise the attention to the case of an unsorted entry that is
	 * better not happen, because, in the worst case we will perform in the
	 * same way as we do without batching - with some more overhead.
	 */
	if (tp_vec_nr > 0) {
		int prev = tp_vec_nr - 1;
		struct text_poke_loc *prev_tp = &tp_vec[prev];

		if (WARN_ON_ONCE(prev_tp->addr > entry_code))
			return false;
	}

	size = __jump_label_set_jump_code(entry, type,
				   (union jump_code_union *)&tp->text, 0);

	text_poke_loc_init(tp, entry_code, NULL, size, NULL);

	tp_vec_nr++;

	return true;
}

void arch_jump_label_transform_apply(void)
{
	if (!tp_vec_nr)
		return;

	mutex_lock(&text_mutex);
	text_poke_bp_batch(tp_vec, tp_vec_nr);
	mutex_unlock(&text_mutex);

	tp_vec_nr = 0;
}

static enum {
	JL_STATE_START,
	JL_STATE_NO_UPDATE,
	JL_STATE_UPDATE,
} jlstate __initdata_or_module = JL_STATE_START;

__init_or_module void arch_jump_label_transform_static(struct jump_entry *entry,
				      enum jump_label_type type)
{
	/*
	 * Rewrite the NOP on init / module-load to ensure we got the ideal
	 * nop.  Don't bother with trying to figure out what size and what nop
	 * it should be for now, simply do an unconditional rewrite.
	 */
	if (jlstate == JL_STATE_UPDATE || jlstate == JL_STATE_START)
		__jump_label_transform(entry, type, 1);
}

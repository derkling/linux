/*
 * Copyright (C) 2008-2010 ARM Limited
 *
 * This software is provided 'as-is', without any express or implied
 * warranties including the implied warranties of satisfactory quality,
 * fitness for purpose or non infringement.  In no event will  ARM be
 * liable for any damages arising from the use of this software.
 *
 * Permission is granted to anyone to use, copy and modify this software for
 * any purpose, and to redistribute the software, subject to the following
 * restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 */

#include "appf_internals.h"
#include "appf_helpers.h"

#define GIC_DIST_ENABLE      0x00000001

struct set_and_clear_regs {
    volatile unsigned int set[32], clear[32];
};

struct interrupt_distributor {
	/* 0x000 */	volatile unsigned int control;
			const unsigned int controller_type;
			const unsigned int implementer;
			const char padding1[116];
	/* 0x080 */	volatile unsigned int security[32];
	/* 0x100 */	struct set_and_clear_regs enable;
	/* 0x200 */	struct set_and_clear_regs pending;
	/* 0x300 */	volatile const unsigned int active[32];
			const char padding2[128];
	/* 0x400 */	volatile unsigned int priority[256];
	/* 0x800 */	volatile unsigned int target[256];
	/* 0xC00 */	volatile unsigned int configuration[64];
	/* 0xD00 */	const char padding3[512];
	/* 0xF00 */	volatile unsigned int software_interrupt;
			const char padding4[220];
	/* 0xFE0 */	unsigned const int peripheral_id[4];
	/* 0xFF0 */	unsigned const int primecell_id[4];
};

struct cpu_interface {
    /* 0x00 */	volatile unsigned int control;
    /* 0x04 */	volatile unsigned int priority_mask;
    /* 0x08 */	volatile unsigned int binary_point;
    /* 0x0c */	volatile unsigned const int interrupt_ack;
    /* 0x10 */	volatile unsigned int end_of_interrupt;
    /* 0x14 */	volatile unsigned const int running_priority;
    /* 0x18 */	volatile unsigned const int highest_pending;
    /* 0x1c */	volatile unsigned int aliased_binary_point;
};

/*
 * Saves the GIC CPU interface context
 * Requires 3 or 4 words of memory
 */
void save_gic_interface(u32 *pointer, unsigned gic_interface_address,
			int is_secure)
{
	struct cpu_interface *ci = (struct cpu_interface *)
				gic_interface_address;

	pointer[0] = ci->control;
	pointer[1] = ci->priority_mask;
	pointer[2] = ci->binary_point;

	if (is_secure)
		pointer[3] = ci->aliased_binary_point;
}

/*
 * Enables or disables the GIC distributor (for the current security state)
 * Parameter 'enabled' is boolean.
 * Return value is boolean, and reports whether GIC was previously enabled.
 */
int gic_distributor_set_enabled(int enabled,
				unsigned gic_distributor_address)
{
	unsigned tmp;
	struct interrupt_distributor *id =
		(struct interrupt_distributor *) gic_distributor_address;

	tmp = id->control;

	if (enabled)
		id->control = tmp | GIC_DIST_ENABLE;
	else
		id->control = tmp & ~GIC_DIST_ENABLE;

	return (tmp & GIC_DIST_ENABLE) != 0;
}

/*
 * Saves this CPU's banked parts of the distributor
 * Returns non-zero if an SGI/PPI interrupt is pending
 * (after saving all required context)
 * Requires 19 words of memory
 */
int save_gic_distributor_private(u32 *pointer,
				unsigned gic_distributor_address,
				int is_secure)
{
	struct interrupt_distributor *id =
	    (struct interrupt_distributor *) gic_distributor_address;

	*pointer = id->enable.set[0];
	++pointer;
	pointer = copy_words(pointer, id->priority, 8);
	pointer = copy_words(pointer, id->target, 8);
	if (is_secure) {
		*pointer = id->security[0];
		++pointer;
	}
	/* Save just the PPI configurations (SGIs are not configurable) */
	*pointer = id->configuration[1];
	++pointer;
	*pointer = id->pending.set[0];
	if (*pointer)
		return -1;
	else
		return 0;
}

/*
 * Saves the shared parts of the distributor. GIC distributor should be
 * disabled first.
 * Requires 1 word of memory, plus 20 words for each block of 32 SPIs
 * (max 641 words)
 * Returns non-zero if an SPI interrupt is pending (after saving all required
 * context)
 */
int save_gic_distributor_shared(u32 *pointer,
				unsigned gic_distributor_address,
				int is_secure)
{
	struct interrupt_distributor *id =
	    (struct interrupt_distributor *) gic_distributor_address;
	unsigned num_spis, *saved_pending;
	int i, retval = 0;


	/* Calculate how many SPIs the GIC supports */
	num_spis = 32 * (id->controller_type & 0x1f);

	/* TODO: add nonsecure stuff */

	/* Save rest of GIC configuration */
	if (num_spis) {
		pointer =
		    copy_words(pointer, id->enable.set + 1, num_spis / 32);
		pointer =
		    copy_words(pointer, id->priority + 8, num_spis / 4);
		pointer =
		    copy_words(pointer, id->target + 8, num_spis / 4);
		pointer =
		    copy_words(pointer, id->configuration + 2,
			       num_spis / 16);
		if (is_secure) {
			pointer =
			    copy_words(pointer, id->security + 1,
				       num_spis / 32);
		}
		saved_pending = pointer;
		pointer =
		    copy_words(pointer, id->pending.set + 1,
			       num_spis / 32);

		/* Check interrupt pending bits */
		for (i = 0; i < num_spis / 32; ++i) {
			if (saved_pending[i]) {
				retval = -1;
				break;
			}
		}
	}

	/* Save control register */
	*pointer = id->control;
	++pointer;

	return retval;
}

void restore_gic_interface(u32 *pointer, unsigned gic_interface_address,
			   int is_secure)
{
	struct cpu_interface *ci = (struct cpu_interface *)
				gic_interface_address;

	ci->priority_mask = pointer[1];
	ci->binary_point = pointer[2];

	if (is_secure)
		ci->aliased_binary_point = pointer[3];

	/* Restore control register last */
	ci->control = pointer[0];
}

void restore_gic_distributor_private(u32 *pointer,
				     unsigned gic_distributor_address,
				     int is_secure)
{
	struct interrupt_distributor *id =
	    (struct interrupt_distributor *) gic_distributor_address;

	/*
	 * We assume the distributor is disabled so we can write to its config
	 */
	id->enable.set[0] = *pointer;
	++pointer;
	copy_words(id->priority, pointer, 8);
	pointer += 8;
	copy_words(id->target, pointer, 8);
	pointer += 8;
	if (is_secure) {
		id->security[0] = *pointer;
		++pointer;
	}
	/* Restore just the PPI configurations (SGIs are not configurable) */
	id->configuration[1] = *pointer;
	++pointer;
	id->pending.set[0] = *pointer;
}

void restore_gic_distributor_shared(u32 *pointer,
				    unsigned gic_distributor_address,
				    int is_secure)
{
	struct interrupt_distributor *id =
	    (struct interrupt_distributor *) gic_distributor_address;
	unsigned num_spis;

	/*
	 * We assume the distributor is disabled so we can write to its config
	 * registers
	 */

	/* Calculate how many SPIs the GIC supports */
	num_spis = 32 * ((id->controller_type) & 0x1f);

	/* TODO: add nonsecure stuff */

	/* Restore rest of GIC configuration */
	if (num_spis) {
		copy_words(id->enable.set + 1, pointer, num_spis / 32);
		pointer += num_spis / 32;
		copy_words(id->priority + 8, pointer, num_spis / 4);
		pointer += num_spis / 4;
		copy_words(id->target + 8, pointer, num_spis / 4);
		pointer += num_spis / 4;
		copy_words(id->configuration + 2, pointer, num_spis / 16);
		pointer += num_spis / 16;
		if (is_secure) {
			copy_words(id->security + 1, pointer,
				   num_spis / 32);
			pointer += num_spis / 32;
		}
		copy_words(id->pending.set + 1, pointer, num_spis / 32);
		pointer += num_spis / 32;
	}

	/*
	 * Restore control register - if the GIC was disabled during save, it
	 * will be restored as disabled.
	 * We are just restoring the Reserved bits really.
	 */
	id->control = *pointer;

	return;
}

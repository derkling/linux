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

#include <linux/compiler.h>
#include <asm/cputype.h>
#include "appf_internals.h"
#include "appf_helpers.h"
#include "appf_defs.h"

struct appf_arch *arch;

struct appf_arch archs[] = {
	{
	 .cpu_val = CPU_A8,
	 .cpu_mask = 0xff0ffff0,
	 .init = appf_platform_a8_init,
	 .save_context = appf_platform_a8_save_context,
	 .restore_context = appf_platform_a8_restore_context,
	 .enter_cstate = appf_platform_a8_enter_cstate,
	 .leave_cstate = appf_platform_a8_leave_cstate,
	 .power_up_cpu = appf_platform_a8_power_up_cpu,
	 .reset = platform_a8_reset_handler},
	{
	 .cpu_val = CPU_A9,
	 .cpu_mask = 0xff0ffff0,
	 .init = appf_platform_a9_init,
	 .save_context = appf_platform_a9_save_context,
	 .restore_context = appf_platform_a9_restore_context,
	 .enter_cstate = appf_platform_a9_enter_cstate,
	 .leave_cstate = appf_platform_a9_leave_cstate,
	 .power_up_cpu = appf_platform_a9_power_up_cpu,
	 .reset = platform_a9_reset_handler},
	{
	 .cpu_val = CPU_A5,
	 .cpu_mask = 0xff0ffff0,
	 .init = appf_platform_a9_init,
	 .save_context = appf_platform_a9_save_context,
	 .restore_context = appf_platform_a9_restore_context,
	 .enter_cstate = appf_platform_a9_enter_cstate,
	 .leave_cstate = appf_platform_a9_leave_cstate,
	 .power_up_cpu = appf_platform_a9_power_up_cpu,
	 .reset = platform_a9_reset_handler},
	{
	 .cpu_val = 0x0}
};

void lookup_arch(void)
{
	struct appf_arch *ap;
	for (ap = archs; ap->cpu_val; ap++)
		if ((read_cpuid_id() & ap->cpu_mask) == ap->cpu_val) {
			arch = ap;
			break;
		}
}

void *arch_reset_handler(void)
{
	return (void *) arch->reset;
}

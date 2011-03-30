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
#define APPF_SAVE_PMU          (1 << 0)
#define APPF_SAVE_TIMERS       (1 << 1)
#define APPF_SAVE_GLOBAL_TIMER (1 << 2)
#define APPF_SAVE_VFP          (1 << 3)
#define APPF_SAVE_DEBUG        (1 << 4)
#define APPF_SAVE_L2           (1 << 31)
#define APPF_SAVE_SCU          (1 << 30)
#define APPF_SAVE_OTHER        (1 << 29)
#define APPF_SAVE_ALL (APPF_SAVE_PMU | APPF_SAVE_TIMERS | \
		APPF_SAVE_GLOBAL_TIMER | APPF_SAVE_VFP | APPF_SAVE_DEBUG | \
		APPF_SAVE_L2 | APPF_SAVE_SCU | APPF_SAVE_OTHER)

extern void *arch_reset_handler(void);
extern int power_down_cpu(unsigned cstate, unsigned rstate, unsigned flags);
extern int power_up_cpus(unsigned int cluster_index, unsigned int cpu);
extern void appf_patch_reset(void *, void*);


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

#include <linux/smp.h>
#include <asm/hardware/elba_scpc.h>
#include <asm/system.h>
#include <asm/appf_platform_api.h>
#include "appf_internals.h"
#include "appf_helpers.h"
#include "power_v7.c"

int appf_platform_power_up_cpu(struct appf_cpu *cpup,
			       struct appf_cluster *cluster,
			       unsigned int cpu)
{
	return arch->power_up_cpu(cpup, cluster, cpu);
}


/*
 * This function puts the executing CPU, and possibly the whole cluster,
 * into STANDBY
 */
int appf_platform_enter_cstate1(unsigned cpu_index,
				struct appf_cpu *cpu,
				struct appf_cluster *cluster)
{
	if (cluster->power_state == 1 && cluster->l2_address)
		set_status_pl310(STATUS_STANDBY, cluster->l2_address);

	return 0;
}

/*
 * This function removes the executing CPU, and possibly the whole cluster,
 * from STANDBY
 */
int appf_platform_leave_cstate1(unsigned cpu_index,
				struct appf_cpu *cpu,
				struct appf_cluster *cluster)
{
	return 0;
}

int appf_platform_enter_cstate(unsigned cpu_index,
			       struct appf_cpu *cpu,
			       struct appf_cluster *cluster)
{
	return arch->enter_cstate(cpu_index, cpu, cluster);
}

int appf_platform_leave_cstate(unsigned cpu_index,
			       struct appf_cpu *cpu,
			       struct appf_cluster *cluster)
{
	return arch->leave_cstate(cpu_index, cpu, cluster);
}

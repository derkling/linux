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

int appf_platform_a8_power_up_cpu(struct appf_cpu *cpup,
				struct appf_cluster *cluster,
				unsigned int cpu)
{
    return 0;
}

/*
 * This function powers up a CPU
 */
int  appf_platform_a9_power_up_cpu(struct appf_cpu *cpup,
					   struct appf_cluster *cluster,
					   unsigned int cpu)
{
	if (cpup->power_state >= 1) {
		cpup->power_state = 0;
		dsb();
		smp_cross_call(cpumask_of(cpu), 1);
		/* TODO: cluster->power_state = rstate; */
	}

	return 0;
}

int appf_platform_a8_enter_cstate(unsigned cpu_index, struct appf_cpu *cpu,
		struct appf_cluster *cluster)
{
    return 0;
}

int appf_platform_a8_leave_cstate(unsigned cpu_index, struct appf_cpu *cpu,
		struct appf_cluster *cluster)
{
    /*
     * Housekeeping...
     */
    ++cluster->active_cpus;
    cluster->power_state = 0;
    cpu->power_state = 0;

    return 0;
}

int  appf_platform_a9_enter_cstate(unsigned cpu_index,
					   struct appf_cpu *cpu,
					   struct appf_cluster *cluster)
{
	hip_set_power_state(cpu_index, cpu->power_state,
					cluster->power_state, 0, 1);
	return 0;
}

/*
 * This function tells the PCU this CPU has finished powering up.
 * It is entered with cluster->lock held.
 */
int  appf_platform_a9_leave_cstate(unsigned cpu_index,
					struct appf_cpu *cpu,
					struct appf_cluster *cluster)
{
	/*
	 * Housekeeping...
	 */

	if (cluster->power_state == 1 && cluster->l2_address)
		set_status_pl310(STATUS_RUN, cluster->l2_address);

	return 0;
}

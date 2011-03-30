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
int appf_platform_a9_save_context(struct appf_cluster *cluster,
				  struct appf_cpu *cpu,
				  unsigned flags, int is_secure)
{
	u32 saved_items = 0;
	u32 cluster_saved_items = 0;
	struct appf_cpu_context *context;
	struct appf_cluster_context *cluster_context;
	unsigned int cpu_index = appf_platform_get_cpu_index();

	if (is_secure) {
		context = cpu->s_context;
		cluster_context = cluster->s_context;
	} else {
		context = cpu->ns_context;
		cluster_context = cluster->ns_context;
	}

	/* add flags as required by hardware (e.g. APPF_SAVE_L2 if L2 is on) */
	flags |= context->flags;

	/*
	 * Save perf. monitors first, so we don't interfere too much with
	 * counts
	 */
	if (flags & APPF_SAVE_PMU) {
		save_performance_monitors(context->pmu_data);
		saved_items |= APPF_SAVE_PMU;
	}

	if (flags & APPF_SAVE_TIMERS) {
		save_a9_timers(context->timer_data, cluster->scu_address);
		saved_items |= APPF_SAVE_TIMERS;
	}

	if (flags & APPF_SAVE_GLOBAL_TIMER) {
		save_a9_global_timer(context->global_timer_data,
				     cluster->scu_address);
		saved_items |= APPF_SAVE_GLOBAL_TIMER;
	}
	if (flags & APPF_SAVE_VFP) {
		save_vfp(context->vfp_data);
		saved_items |= APPF_SAVE_VFP;
	}
	save_gic_interface(context->gic_interface_data, cpu->ic_address,
			   is_secure);
	save_gic_distributor_private(context->gic_dist_private_data,
				     cluster->ic_address, is_secure);

	/* TODO: check return value and re-enable GIC and quit if nonzero! */

	save_banked_registers(context->banked_registers);
	save_cp15(context->cp15_data);


	if (flags & APPF_SAVE_OTHER) {
		if (cluster->cpu_type == CPU_A9)
			save_a9_other(context->other_data, is_secure);
		else
			save_a5_other(context->other_data, is_secure);

		saved_items |= APPF_SAVE_OTHER;
	}

	if (flags & APPF_SAVE_DEBUG) {
		save_a9_debug(context->debug_data);
		saved_items |= APPF_SAVE_DEBUG;
	}

	if (cpu->cluster_down) {
		save_gic_distributor_shared
		    (cluster_context->gic_dist_shared_data,
		     cluster->ic_address, is_secure);
	}
	save_control_registers(context->control_data, is_secure);
	save_mmu(context->mmu_data, main_table.os_mmu_context);
	context->saved_items = saved_items;

	if (cpu->cluster_down) {
		if (flags & APPF_SAVE_SCU) {
			save_a9_scu(cluster_context->scu_data,
				    cluster->scu_address);
			cluster_saved_items |= APPF_SAVE_SCU;
		}

		if (flags & APPF_SAVE_L2) {
			save_pl310(cluster_context->l2_data,
				   cluster->l2_address);
			cluster_saved_items |= APPF_SAVE_L2;
		}
		cluster_context->saved_items = cluster_saved_items;
	}

	/*
	 * DISABLE DATA CACHES
	 *
	 * First, disable, then clean+invalidate the L1 cache.
	 *
	 * Note that if L1 was to be dormant and we were the last CPU,
	 * we would only need to clean some key data
	 * out of L1 and clean+invalidate the stack.
	 */

	appf_platform_enter_cstate(cpu_index, cpu, cluster);

	disable_clean_inv_stack();

#ifdef CONFIG_CACHE_L2X0
	if (cluster->l2_address && is_enabled_pl310(cluster->l2_address)) {

		disable_clean_inv_cache_pl310(cluster->l2_address,
					      cpu->cluster_down
					      && is_secure);

		/*
		 * We need to partially or fully clean the L2, because we
		 * will enter reset with cacheing disabled
		 */
		if (cluster->power_state > 2)
			/* Clean the whole thing
			 * TODO: Could optimize this by doing it once in S i
			 * (if we are saving context in S+NS)
			 *
			 */
			clean_pl310(cluster->l2_address);
	}
#endif
	if (cluster->scu_address && cpu->power_state >= 2)
		set_status_a9_scu(cpu_index, STATUS_SHUTDOWN,
				cluster->scu_address);

	if (cluster->power_state == 1 && cluster->l2_address)
		set_status_pl310(STATUS_STANDBY, cluster->l2_address);

	return 0;
}

/*
 * This function restores all the context that was lost
 * when a CPU and cluster entered a low power state. It is called shortly after
 * reset, with the MMU and data cache off.
 *
 * This function is called with cluster->lock held
 */
int appf_platform_a9_restore_context(struct appf_cluster *cluster,
				struct appf_cpu *cpu, int is_secure)
{
	struct appf_cpu_context *context;
	struct appf_cluster_context *cluster_context;
	u32 saved_items, cluster_saved_items = 0;
	int cluster_init = cluster->cluster_down;

	/*
	 * At this point we may not write to any static data, and we may
	 * only read the data that we explicitly cleaned from the L2 above.
	 */
	cluster->cluster_down = 0;
	++cluster->active_cpus;

	if (is_secure) {
		context = cpu->s_context;
		cluster_context = cluster->s_context;
	} else {
		context = cpu->ns_context;
		cluster_context = cluster->ns_context;
	}

	saved_items = context->saved_items;

	/* First set up the SCU & L2, if necessary */
	if (cluster_init) {
		cluster_saved_items = cluster_context->saved_items;
		if (cluster_saved_items & APPF_SAVE_SCU) {
			restore_a9_scu(cluster_context->scu_data,
				       cluster->scu_address);
		}
		if (cluster_saved_items & APPF_SAVE_L2) {
			restore_pl310(cluster_context->l2_data,
				      cluster->l2_address,
				      cluster->power_state == 2);
		}
	}

	if (cpu->power_state >= 2 && cluster->scu_address)
		set_status_a9_scu(cpu - cluster->cpu_table, STATUS_RUN,
				  cluster->scu_address);
	cpu->power_state = 0;

	/* Next get the MMU back on */
	restore_mmu(context->mmu_data, main_table.fw_mmu_context);
	restore_control_registers(context->control_data, is_secure);
	/*
	 * MMU and L1 and L2 caches are on, we may now read/write any data.
	 * Now we need to restore the rest of this CPU's context
	 */

	/*
	 * Get the debug registers restored, so we can debug most of the APPF
	 * code sensibly!
	 */
	if (saved_items & APPF_SAVE_DEBUG)
		restore_a9_debug(context->debug_data);

	/* Restore shared items if necessary */
	if (cluster_init) {
		restore_gic_distributor_shared
			(cluster_context->gic_dist_shared_data,
			cluster->ic_address, is_secure);
	}

	restore_gic_distributor_private(context->gic_dist_private_data,
					cluster->ic_address, is_secure);
	restore_gic_interface(context->gic_interface_data, cpu->ic_address,
					is_secure);

	if (saved_items & APPF_SAVE_OTHER) {
		if (read_cpuid_id() == CPU_A9)
			restore_a9_other(context->other_data, is_secure);
		else
			restore_a5_other(context->other_data, is_secure);

	}

	restore_cp15(context->cp15_data);
	restore_banked_registers(context->banked_registers);
	if (saved_items & APPF_SAVE_VFP)
		restore_vfp(context->vfp_data);

	if (saved_items & APPF_SAVE_TIMERS)
		restore_a9_timers(context->timer_data,
				  cluster->scu_address);

	if (saved_items & APPF_SAVE_GLOBAL_TIMER)
		restore_a9_global_timer(context->global_timer_data,
					cluster->scu_address);

	if (saved_items & APPF_SAVE_PMU)
		restore_performance_monitors(context->pmu_data);

	/* Return to OS */
	return 0;
}


/*
 * This function saves all a8 the context that will be lost
 * when a CPU and cluster enter a low power state.
 *
 * This function is called with cluster->lock held
 */
int appf_platform_a8_save_context(struct appf_cluster *cluster,
				  struct appf_cpu *cpu, unsigned flags,
				  int is_secure)
{
	u32 saved_items = 0;
	struct appf_cpu_context *context;

	if (is_secure)
		context = cpu->s_context;
	else
		context = cpu->ns_context;

	/* add flags as required by hardware */
	flags |= context->flags;

	/*
	 * Save perf. monitors first, so we don't interfere too much
	 * with counts
	 */
	if (flags & APPF_SAVE_PMU) {
		save_performance_monitors(context->pmu_data);
		saved_items |= APPF_SAVE_PMU;
	}

	if (flags & APPF_SAVE_VFP) {
		save_vfp(context->vfp_data);
		saved_items |= APPF_SAVE_VFP;
	}

	/* TODO: disable on-board GIC somehow? */

	save_banked_registers(context->banked_registers);
	save_cp15(context->cp15_data);

	if (flags & APPF_SAVE_OTHER) {
		save_a8_other(context->other_data);
		saved_items |= APPF_SAVE_OTHER;
	}

	if (flags & APPF_SAVE_DEBUG) {
		save_a8_debug(context->debug_data);
		saved_items |= APPF_SAVE_DEBUG;
	}

	save_control_registers(context->control_data, is_secure);
	save_mmu(context->mmu_data, main_table.os_mmu_context);
	context->saved_items = saved_items;

	/*
	 * Disable, then clean+invalidate the L1 (data) & L2 caches.
	 *
	 * Note that if L1 or L2 was to be dormant we would only need to i
	 * clean some key data out,
	 * and clean+invalidate the stack.
	 */
	disable_clean_inv_dcache_v7_all();

	return 0;
}

/*
 * This function restores all the a8 context that was lost
 * when a CPU and cluster entered a low power state. It is called shortly after
 * reset, with the MMU and data cache off.
 */
int appf_platform_a8_restore_context(struct appf_cluster *cluster,
				     struct appf_cpu *cpu, int is_secure)
{
	struct appf_cpu_context *context;
	u32 saved_items;

	/*
	 * If the L1 or L2 is dormat, there are special precautions:
	 * At this point we may not write to any static data, and we may
	 * only read the data that we explicitly cleaned from the caches above.
	 */
	if (is_secure)
		context = cpu->s_context;
	else
		context = cpu->ns_context;

	saved_items = context->saved_items;

	/* First get the MMU back on */
	restore_mmu(context->mmu_data, main_table.fw_mmu_context);
	restore_control_registers(context->control_data, is_secure);

	/*
	 * MMU and L1 and L2 caches are on, we may now read/write any data.
	 * Now we need to restore the rest of this CPU's context
	 */

	/*
	 * Get the debug registers restored, so we can debug most of the
	 * APPF code sensibly!
	 */
	if (saved_items & APPF_SAVE_DEBUG)
		restore_a8_debug(context->debug_data);


	if (saved_items & APPF_SAVE_OTHER)
		restore_a8_other(context->other_data);

	restore_cp15(context->cp15_data);
	restore_banked_registers(context->banked_registers);

	if (saved_items & APPF_SAVE_VFP)
		restore_vfp(context->vfp_data);

	if (saved_items & APPF_SAVE_PMU)
		restore_performance_monitors(context->pmu_data);

	/* Return to OS */
	return 0;
}

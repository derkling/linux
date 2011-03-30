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

int appf_platform_a8_init(void)
{
	int i;

#if !SECURE_ONLY && !NONSECURE_ONLY
	/* Also need to ensure that the SMC instruction reaches APPF code */
	*(unsigned *) (read_mvbar() + 0x20) = (unsigned) &appf_smc_handler;
#endif

	/*
	 * Setup tables - Note that pointers are flat-mapped/physical
	 * addresses
	 */

	aem_cluster[0].cpu_type = read_cpuid_id() & 0xff0ffff0;

	if (aem_cluster[0].cpu_type != CPU_A8)
		return -ENODEV;

	is_smp = FALSE;

	aem_cluster[0].num_cpus = 1;
	aem_cluster[0].cpu_table = &aem_cpu[0];
	aem_cluster[0].lock = (void *) appf_device_memory_flat_mapped;
	initialize_spinlock(aem_cluster[0].lock);

	/*
	 * Note that we wastefully allocate memory for every item in both
	 * the S and NS worlds.
	 * This could be done more efficiently, but we are keeping it
	 * simple here.
	 *
	 * Also, the Secure context perhaps might need to be allocated
	 * in Secure memory.
	 */

	/* (No cluster context for A8) */

	for (i = 0; i < aem_cluster[0].num_cpus; ++i) {
		aem_cpu[i].s_context =
		    (void *) get_memory(sizeof(struct appf_cpu_context),
					APPF_UNCACHED);
		aem_cpu[i].s_context->control_data =
		    (void *) get_memory(CONTROL_DATA_SIZE, APPF_UNCACHED);
		aem_cpu[i].s_context->pmu_data =
		    (void *) get_memory(PMU_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->vfp_data =
		    (void *) get_memory(VFP_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->banked_registers =
		    (void *) get_memory(BANKED_REGISTERS_SIZE,
					APPF_CACHED);
		aem_cpu[i].s_context->cp15_data =
		    (void *) get_memory(CP15_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->debug_data =
		    (void *) get_memory(DEBUG_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->mmu_data =
		    (void *) get_memory(MMU_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->other_data =
		    (void *) get_memory(OTHER_DATA_SIZE, APPF_CACHED);

		aem_cpu[i].ns_context =
		    (void *) get_memory(sizeof(struct appf_cpu_context),
					APPF_CACHED);
		aem_cpu[i].ns_context->control_data =
		    (void *) get_memory(CONTROL_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->pmu_data =
		    (void *) get_memory(PMU_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->vfp_data =
		    (void *) get_memory(VFP_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->banked_registers =
		    (void *) get_memory(BANKED_REGISTERS_SIZE,
					APPF_CACHED);
		aem_cpu[i].ns_context->cp15_data =
		    (void *) get_memory(CP15_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->debug_data =
		    (void *) get_memory(DEBUG_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->mmu_data =
		    (void *) get_memory(MMU_DATA_SIZE, APPF_UNCACHED);
		aem_cpu[i].ns_context->other_data =
		    (void *) get_memory(OTHER_DATA_SIZE, APPF_CACHED);

#if SECURE_ONLY
		aem_cpu[i].s_context->flags = APPF_SAVE_PMU |
		    APPF_SAVE_DEBUG;
#elif NONSECURE_ONLY
		aem_cpu[i].ns_context->flags = APPF_SAVE_PMU |
		    APPF_SAVE_DEBUG;
#else
		aem_cpu[i].ns_context->flags = APPF_SAVE_PMU |
		    APPF_SAVE_DEBUG;
#endif

	}

	main_table.cluster_table = (struct appf_cluster *)
	    get_memory(sizeof(struct appf_cluster), APPF_UNCACHED);
	main_table.num_clusters =
	    sizeof(aem_cluster) / sizeof(aem_cluster[0]);
	memcpy((void *) main_table.cluster_table,
	       &aem_cluster[0], sizeof(struct appf_cluster));
	return 0;
}

/*
 * This function is called at the end of runtime initialization.
 *
 * It is called using APPF's translation tables and stack, by the same CPU that
 * did the early initialization.
 */
int appf_platform_a9_init(void)
{
	int i;
	unsigned int cbar = 0;

#if !SECURE_ONLY && !NONSECURE_ONLY
	/* Also need to ensure that the SMC instruction reaches APPF code */
	*(unsigned *) (read_mvbar() + 0x20) = (unsigned) &appf_smc_handler;
#endif

	aem_cluster[0].cpu_type = read_cpuid_id() & 0xff0ffff0;
	aem_cluster[0].cluster_down = 0;
	if ((aem_cluster[0].cpu_type != CPU_A9) &&
	    (aem_cluster[0].cpu_type != CPU_A5))
		return -ENODEV;

	if (read_mpidr() & MPIDR_U_BIT) {
		is_smp = FALSE;
		aem_cluster[0].num_cpus = 1;
	} else {
		is_smp = TRUE;
		cbar = read_cbar();
		if (!cbar) {
			early_printk(KERN_DEBUG "memory mapping failed\n");
			return -ENOMEM;
		}
		aem_cluster[0].scu_address = cbar;
		aem_cluster[0].ic_address = cbar + 0x1000;
#ifdef CONFIG_ARCH_VEXPRESS_LT_ELBA
		/* TODO: is this actually standardized? */
		aem_cluster[0].l2_address = cbar + 0x2000;
#else
		/* TODO: is this actually standardized? */
		aem_cluster[0].l2_address = cbar + 0xa000;
#endif
		aem_cluster[0].num_cpus = num_online_cpus();
	}

	aem_cluster[0].active_cpus = aem_cluster[0].num_cpus;
	aem_cluster[0].cpu_table = (struct appf_cpu *)
	    get_memory(sizeof(struct appf_cpu) * MAX_CPUS, APPF_UNCACHED);
	aem_cluster[0].lock = (void *) (appf_device_memory_flat_mapped);

	initialize_spinlock(aem_cluster[0].lock);


	/*
	 * Note that we wastefully allocate memory for every item in both
	 * the S and NS worlds.
	 * This could be done more efficiently, but we are keeping it
	 * simple here.
	 *
	 * Also, the Secure context perhaps might need to be allocated
	 * in Secure memory.
	 */

	aem_cluster[0].s_context =
	    (void *) get_memory(sizeof(struct appf_cluster_context),
				APPF_UNCACHED);

	aem_cluster[0].s_context->gic_dist_shared_data =
	    (void *) get_memory(GIC_DIST_SHARED_DATA_SIZE, APPF_CACHED);

	aem_cluster[0].s_context->l2_data =
	    (void *) get_memory(L2_DATA_SIZE, APPF_UNCACHED);
	aem_cluster[0].s_context->scu_data =
	    (void *) get_memory(SCU_DATA_SIZE, APPF_UNCACHED);

	aem_cluster[0].ns_context =
	    (void *) get_memory(sizeof(struct appf_cluster_context),
				APPF_UNCACHED);
	aem_cluster[0].ns_context->gic_dist_shared_data =
	    (void *) get_memory(GIC_DIST_SHARED_DATA_SIZE, APPF_CACHED);

	aem_cluster[0].ns_context->l2_data =
	    (void *) get_memory(L2_DATA_SIZE, APPF_UNCACHED);
	aem_cluster[0].ns_context->scu_data =
	    (void *) get_memory(SCU_DATA_SIZE, APPF_UNCACHED);

	for (i = 0, aem_cpu = aem_cluster[0].cpu_table;
	     i < aem_cluster[0].num_cpus; ++i) {

		if (is_smp)
			aem_cpu[i].ic_address = cbar + 0x100;

		platform_cpu_context[i] = (unsigned long)
		    get_memory(sizeof(struct cpu_context_save),
			       APPF_UNCACHED);

		aem_cpu[i].s_context =
		    (void *) get_memory(sizeof(struct appf_cpu_context),
					APPF_UNCACHED);
		aem_cpu[i].s_context->control_data =
		    (void *) get_memory(CONTROL_DATA_SIZE, APPF_UNCACHED);
		aem_cpu[i].s_context->pmu_data =
		    (void *) get_memory(PMU_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->timer_data =
		    (void *) get_memory(TIMER_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->global_timer_data =
		    (void *) get_memory(GLOBAL_TIMER_DATA_SIZE,
					APPF_CACHED);
		aem_cpu[i].s_context->vfp_data =
		    (void *) get_memory(VFP_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->gic_interface_data =
		    (void *) get_memory(GIC_INTERFACE_DATA_SIZE,
					APPF_CACHED);
		aem_cpu[i].s_context->gic_dist_private_data =
		    (void *) get_memory(GIC_DIST_PRIVATE_DATA_SIZE,
					APPF_CACHED);
		aem_cpu[i].s_context->banked_registers =
		    (void *) get_memory(BANKED_REGISTERS_SIZE,
					APPF_CACHED);
		aem_cpu[i].s_context->cp15_data =
		    (void *) get_memory(CP15_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->debug_data =
		    (void *) get_memory(DEBUG_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].s_context->mmu_data =
		    (void *) get_memory(MMU_DATA_SIZE, APPF_UNCACHED);
		aem_cpu[i].s_context->other_data =
		    (void *) get_memory(OTHER_DATA_SIZE, APPF_CACHED);

		aem_cpu[i].ns_context =
		    (void *) get_memory(sizeof(struct appf_cpu_context),
					APPF_UNCACHED);
		aem_cpu[i].ns_context->control_data =
		    (void *) get_memory(CONTROL_DATA_SIZE, APPF_UNCACHED);
		aem_cpu[i].ns_context->pmu_data =
		    (void *) get_memory(PMU_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->timer_data =
		    (void *) get_memory(TIMER_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->global_timer_data =
		    (void *) get_memory(GLOBAL_TIMER_DATA_SIZE,
					APPF_CACHED);
		aem_cpu[i].ns_context->vfp_data =
		    (void *) get_memory(VFP_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->gic_interface_data =
		    (void *) get_memory(GIC_INTERFACE_DATA_SIZE,
					APPF_CACHED);
		aem_cpu[i].ns_context->gic_dist_private_data =
		    (void *) get_memory(GIC_DIST_PRIVATE_DATA_SIZE,
					APPF_CACHED);
		aem_cpu[i].ns_context->banked_registers =
		    (void *) get_memory(BANKED_REGISTERS_SIZE,
					APPF_CACHED);
		aem_cpu[i].ns_context->cp15_data =
		    (void *) get_memory(CP15_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->debug_data =
		    (void *) get_memory(DEBUG_DATA_SIZE, APPF_CACHED);
		aem_cpu[i].ns_context->mmu_data =
		    (void *) get_memory(MMU_DATA_SIZE, APPF_UNCACHED);
		aem_cpu[i].ns_context->other_data =
		    (void *) get_memory(OTHER_DATA_SIZE, APPF_CACHED);
	}

	main_table.cluster_table = (struct appf_cluster *)
	    get_memory(sizeof(struct appf_cluster), APPF_UNCACHED);
	main_table.num_clusters =
	    sizeof(aem_cluster) / sizeof(aem_cluster[0]);
	memcpy((void *) main_table.cluster_table,
	       &aem_cluster[0], sizeof(struct appf_cluster));

	return 0;
}

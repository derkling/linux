#ifndef __CPUHOTPLUG_H
#define __CPUHOTPLUG_H

enum cpuhp_state {
	/*
	 * P: Priority of the callback
	 * S: Startup function or NULL for none
	 * T: Teardown function or NULL for NONE
	 * C: kind of callback Configuration:
	 *	C: Compile time
	 *	I: runtime and Invoked
	 *	P: runtime and not invoked (Passive)
	 */
	CPUHP_OFFLINE,
	CPUHP_CPUFREQ_POSTDEAD,		/* P: 0 S: NULL T: cpufreq_offline_finish C: P */
	CPUHP_CREATE_THREADS,		/* P: _cpu_up S: smpboot_create_threads T: NULL C: C */
	CPUHP_PERF_X86_UNCORE_PREP,	/* P: 21 S: uncore_prepare_cpu T: uncore_dead_cpu C: I */
	CPUHP_PERF_X86_AMD_UNCORE_PREP,	/* P: 21 S: amd_uncore_cpu_up_prepare T: amd_uncore_cpu_dead C: I */
	CPUHP_PERF_X86_RAPL_PREP,	/* P: 20 S: rapl_cpu_prepare T: rapl_cpu_kfree C: I */
	CPUHP_PERF_X86_PREPARE,		/* P: 20 S: x86_pmu_prepare_cpu T: x86_pmu_dead_cpu C: I */
	CPUHP_PERF_BFIN,		/* P: 20 S: bfin_pmu_prepare_cpu T: NULL C: I */
	CPUHP_PERF_POWER,		/* P: 20 S: power_pmu_prepare_cpu T: NULL C: I */
	CPUHP_PERF_SUPERH,		/* P: 20 S: sh_pmu_prepare_cpu T: NULL C: I */
	CPUHP_PERF_PREPARE,		/* P: 20 S: perf_event_init_cpu T: perf_event_exit_cpu C: C */
	CPUHP_SCHED_MIGRATE_PREP,	/* P: 10 S: sched_migration_prepare_cpu T: sched_migration_dead_cpu C: C */
	CPUHP_WORKQUEUE_PREP,		/* P: 5 S: workqueue_prepare_cpu T: NULL C: C */
	CPUHP_POWER_NUMA_PREPARE,	/* P: 1 S: ppc_numa_cpu_prepare T: ppc_numa_cpu_dead C: I */
	CPUHP_RCUTREE_PREPARE,		/* P: 0 S: rcutree_prepare_cpu T: rcutree_dead_cpu C: C */
	CPUHP_HRTIMERS_PREPARE,		/* P: 0 S: hrtimers_prepare_cpu T: hrtimers_dead_cpu C: C */
	CPUHP_PROFILE_PREPARE,		/* P: 0 S: profile_prepare_cpu T: profile_dead_cpu C: I */
	CPUHP_X2APIC_PREPARE,		/* P: 0 S: x2apic_prepare_cpu T: x2apic_dead_cpu C: I */
	CPUHP_SMPCFD_PREPARE,		/* P: 0 S: smpcfd_prepare_cpu T: smpcfd_dead_cpu C: C */
	CPUHP_ARM_BL_PREPARE,		/* P: 0 S: bL_switcher_cpu_pre T: NULL C: P */
	CPUHP_RELAY_PREPARE,		/* P: 0 S: relay_prepare_cpu T: NULL C: C */
	CPUHP_SLAB_PREPARE,		/* P: 0 S: slab_prepare_cpu T: slab_dead_cpu C: C */
	CPUHP_XEN_EV_PREPEARE,		/* P: 0 S: xen_evtchn_cpu_prepare T: NULL C: P */
	CPUHP_MD_RAID5_PREPARE,		/* P: 0 S: raid456_cpu_up_prepare T: raid456_cpu_dead C: P */
	CPUHP_CPUIDLE_COUPLED_PREPARE,	/* P: 0 S: coupled_cpu_up_prepare T: coupled_cpu_online C: P */
	CPUHP_XEN_HVM_GUEST_PREPARE,	/* P: 0 S: xen_hvm_cpu_up_prepare T: NULL C: P */
	CPUHP_S390_HWS_PREPARE,		/* P: 0 S: s390_hws_cpu_prepare T: s390_hws_cpu_prepare C: P */
	CPUHP_POWER_PMAC_PREPARE,	/* P: 0 S: smp_core99_cpu_prepare T: NULL C: P */
	CPUHP_POWER_MMU_CTX_PREPARE,	/* P: 0 S: mmu_ctx_cpu_prepare T: mmu_ctx_cpu_dead C: P */
	CPUHP_ARM_SHMOBILE_SCU_PREPARE,	/* P: 0 S: shmobile_scu_cpu_prepare T: NULL C: P */
	CPUHP_SH_SH3X_PREPARE,		/* P: 0 S: shx3_cpu_prepare T: NULL C: P */
	CPUHP_X86_MICRCODE_PREPARE,	/* P: 0 S: nmc_cpu_prepare T: mc_cpu_dead C: P */
	CPUHP_NOTF_ERR_INJ_PREPARE,	/* P: 0 S: notf_err_inj_up_prepare T: notf_err_inj_dead C: P */
	CPUHP_MIPS_CAVIUM_PREPARE,	/* P: 0 S: octeon_update_boot_vector T: NULL C: P */
	CPUHP_MIPS_LOONGSON_PREPARE,	/* P: 0 S: loongson3_enable_clock T: loongson3_disable_clock C: P */
	CPUHP_BLK_MQ_PREPARE,		/* P: 0 S: blk_mq_queue_reinit_prepare T: blk_mq_queue_reinit_dead C: P */
	CPUHP_NET_FLOW_PREPARE,		/* P: 0 S: flow_cache_cpu_up_prep T: flow_cache_cpu_dead C: P */
	CPUHP_TOPOLOGY_PREPARE,		/* P: 0 S: topology_add_dev T: topology_remove_dev C: I */
	CPUHP_X86_THERM_PREPARE,	/* P: 0 S: thermal_throttle_prepare T: thermal_throttle_dead C: I */
	CPUHP_X86_CPUID_PREPARE,	/* P: 0 S: cpuid_device_create T: cpuid_device_destroy C: I */
	CPUHP_X86_MSR_PREPARE,		/* P: 0 S: msr_device_create T: msr_device_destroy C: I */
	CPUHP_NET_IUCV_PREPARE,		/* P: 0 S: iucv_cpu_prepare T: iucv_cpu_dead C: I */
	CPUHP_CPUFREQ_ACPI_PREPARE,	/* P: 0 S: cpufreq_boost_prepare T: NULL C: P */
	CPUHP_TRACE_RB_PREPARE,		/* P: 0 S: trace_rb_cpu_prepare T: NULL C: P */
	CPUHP_NOTIFY_PREPARE,		/* P: CPU_UP_PREPARE S: notify_prepare: T: NULL C: C */
	CPUHP_NOTIFY_DEAD,		/* P: CPU_DEAD S: NULL: T: notify_dead C: C */
	CPUHP_X86_APB_DEAD,		/* P: -20 S: NULL T: apbt_cpu_dead C: I */
	CPUHP_X86_HPET_DEAD,		/* P: -20 S: NULL T: hpet_cpuhp_dead C: I */
	CPUHP_SLUB_DEAD,		/* P: 0 S: NULL T: slub_cpu_dead C: P */
	CPUHP_TIMERS_DEAD,		/* P: 0 S: NULL T: timers_dead_cpu C: C */
	CPUHP_MM_WRITEBACK_DEAD,	/* P: 0 S: NULL T: page_writeback_cpu_online C: I */
	CPUHP_SOFTIRQ_DEAD,		/* P: 0 S: NULL T: takeover_tasklets C: P */
	CPUHP_NET_MVNETA_DEAD,		/* P: 0 S: NULL T: mvneta_cpu_dead C: P */
	CPUHP_CPUIDLE_PSERIES_DEAD,	/* P: 0 S: NULL T: pseries_cpuidle_cpu_dead C: P */
	CPUHP_CPUIDLE_POWERNV_DEAD,	/* P: 0 S: NULL T: powernv_cpuidle_cpu_dead C: P */
	CPUHP_ARM64_FPSIMD_DEAD,	/* P: 0 S: NULL T: fpsimd_cpu_dead C: P */
	CPUHP_ARM_OMAP_WAKE_DEAD,	/* P: 0 S: NULL T: omap_wakegen_cpu_dead C: P */
	CPUHP_BLOCK_IOPOLL_DEAD,	/* P: 0 S: NULL T: blk_iopoll_cpu_dead C: P */
	CPUHP_BLOCK_SOFTIRQ_DEAD,	/* P: 0 S: NULL T: blk_softirq_cpu_dead C: P */
	CPUHP_VIRT_NET_DEAD,		/* P: 0 S: NULL T: virtnet_cpu_online C: P */
	CPUHP_VIRT_SCSI_DEAD,		/* P: 0 S: NULL T: virtscsi_cpu_online C: P */
	CPUHP_ACPI_CPUDRV_DEAD,		/* P: 0 S: NULL T: acpi_soft_cpu_dead C: P */
	CPUHP_THERMAL_POWERCLMP_DEAD,	/* P: 0 S: NULL T: powerclamp_cpu_dead C: P */
	CPUHP_S390_PFAULT_DEAD,		/* P: 0 S: NULL T: pfault_cpu_dead C: P */
	CPUHP_BLKMQ_DEAD,		/* P: 0 S: NULL T: blk_mq_main_cpu_dead C: P */
	CPUHP_FS_BUFF_DEAD,		/* P: 0 S: NULL T: buffer_exit_cpu_dead C: P */
	CPUHP_PRINTK_DEAD,		/* P: 0 S: NULL T: console_cpu_notify C: P */
	CPUHP_SCHED_HRTICK_DEAD,	/* P: 0 S: NULL T: hotplug_hrtick_dead C: P */
	CPUHP_MM_MEMCQ_DEAD,		/* P: 0 S: NULL T: memcg_hotplug_cpu_dead C: P */
	CPUHP_PERCPU_CNT_DEAD,		/* P: 0 S: NULL T: percpu_counter_cpu_dead C: P */
	CPUHP_RADIX_DEAD,		/* P: 0 S: NULL T: radix_tree_cpu_dead C: P */
	CPUHP_PAGE_ALLOC_DEAD,		/* P: 0 S: NULL T: page_alloc_cpu_dead C: P */
	CPUHP_NET_DEV_DEAD,		/* P: 0 S: NULL T: dev_cpu_dead C: P */
	CPUHP_X86_MCE_DEAD,		/* P: 0 S: NULL T: mce_cpu_dead C: I */
	CPUHP_PCI_XGENE_DEAD,		/* P: 0 S: NULL T: xgene_msi_hwirq_free C: I */
	CPUHP_SCSI_BNX2FC_DEAD,		/* P: 0 S: NULL T: bnx2fc_cpu_dead C: I */
	CPUHP_SCSI_BNX2I_DEAD,		/* P: 0 S: NULL T: bnx2i_cpu_dead C: I */
	CPUHP_SCSI_FCOE_DEAD,		/* P: 0 S: NULL T: fcoe_cpu_dead C: I */
	CPUHP_MM_VMSTAT_DEAD,		/* P: 0 S: NULL T: vmstat_cpu_dead C: P */
	CPUHP_SCHED_DEAD,		/* P: INT_MAX S: NULL T: sched_dead_numa_cpu C: P */
	CPUHP_BRINGUP_CPU,		/* P: __cpu_up S: bringup_cpu T: NULL C: C */
	CPUHP_AP_OFFLINE,
	CPUHP_X86_MCE_STARTING,			/* P: MCE S: mcheck_cpu_starting T: mce_cpu_down_dying C: I */
	CPUHP_AP_SCHED_STARTING,		/* P: INT_MAX S: sched_cpu_active_starting T: NULL C: C */
	CPUHP_AP_IRQ_GIC_STARTING,		/* P: 100 S: gic_starting_cpu T: NULL C: P */
	CPUHP_AP_IRQ_GICV3_STARTING,		/* P: 100 S: gic_starting_cpu T: NULL C: P */
	CPUHP_AP_IRQ_HIP04_STARTING,		/* P: 100 S: hip04_irq_starting_cpu T: NULL C: I */
	CPUHP_AP_IRQ_ARMADA_XP_STARTING,	/* P: 100 S: armada_xp_mpic_starting_cpu T: NULL C: I */
	CPUHP_AP_ARM_MVEBU_COHERENCY,		/* P: 100 S: armada_xp_clear_l2_starting T: NULL C: P */
	CPUHP_AP_IRQ_ARMADA_CASC_STARTING,	/* P: 100 S: mpic_cascaded_starting_cpu T: NULL C: I */
	CPUHP_AP_IRQ_BCM2836_STARTING,		/* P: 100 S: bcm2836_cpu_starting T: bcm2836_cpu_dying C: I */
	CPUHP_AP_PERF_X86_UNCORE_STARTING,	/* P: 21 S: uncore_starting_cpu T: NULL C: I */
	CPUHP_AP_PERF_X86_AMD_UNCORE_STARTING,	/* P: 21 S: amd_uncore_cpu_starting T: NULL C: I */
	CPUHP_AP_PERF_X86_RAPL_STARTING,	/* P: 20 S: rapl_cpu_init T: rapl_cpu_dying C: I */
	CPUHP_AP_PERF_X86_CSTATE_STARTING,	/* P: 20 S: cstate_cpu_init T: NULL C: I */
	CPUHP_AP_PERF_X86_CQM_STARTING,		/* P: 20 S: intel_cqm_cpu_startingT: NULL C: I */
	CPUHP_AP_PERF_X86_AMD_IBS_STARTING,	/* P: 20 S: x86_pmu_amd_ibs_starting_cpu T: x86_pmu_amd_ibs_dying_cpu C: I */
	CPUHP_AP_PERF_X86_STARTING,		/* P: 20 S: x86_pmu_starting_cpu T: x86_pmu_dying_cpu C: I */
	CPUHP_AP_PERF_XTENSA_STARTING,		/* P: 20 S: xtensa_pmu_setup T: NULL C: I */
	CPUHP_AP_PERF_METAG_STARTING,		/* P: 0 S: metag_pmu_starting_cpu T: NULL C: I */
	CPUHP_AP_PERF_ARM_STARTING,		/* P: 0 S: arm_perf_starting_cpu T: NULL C: P */
	CPUHP_AP_MIPS_OP_LOONGSON3_STARTING,	/* P: 0 S: loongson3_starting_cpu T: loongson3_dying_cpu C: P */
	CPUHP_AP_ARM_VFP_STARTING,		/* P: 0 S: vfp_starting_cpu T: vfp_dying_cpu C: P */
	CPUHP_AP_ARM_L2X0_STARTING,		/* P: 0 S: l2c310_starting_cpu T: l2c310_dying_cpu C: I */
	CPUHP_AP_ARM_TWD_STARTING,		/* P: 0 S: twd_timer_starting_cpu T: twd_timer_dying_cpu C: I */
	CPUHP_AP_ARM_ARCH_TIMER_STARTING,	/* P: 0 S: arch_timer_starting_cpu T: arch_timer_dying_cpu C: I */
	CPUHP_AP_ARM_GLOBAL_TIMER_STARTING,	/* P: 0 S: gt_starting_cpu T: gt_dying_cpu C: I */
	CPUHP_AP_ARM_DUMMY_TIMER_STARTING,	/* P: 0 S: dummy_timer_starting_cpu T: NULL C: I */
	CPUHP_AP_EXYNOS4_MCT_TIMER_STARTING,	/* P: 0 S: exynos4_mct_starting_cpu T: exynos4_mct_dying_cpu C: I */
	CPUHP_AP_METAG_TIMER_STARTING,		/* P: 0 S: arch_timer_starting_cpu T: NULL C: I */
	CPUHP_AP_QCOM_TIMER_STARTING,		/* P: 0 S: msm_local_timer_starting_cpu T: msm_local_timer_dying_cpu C: I */
	CPUHP_AP_ARMADA_TIMER_STARTING,		/* P: 0 S: armada_370_xp_timer_starting_cpu T: armada_370_xp_timer_dying_cpu C: I */
	CPUHP_AP_MARCO_TIMER_STARTING,		/* P: 0 S: sirfsoc_local_timer_starting_cpu T: sirfsoc_local_timer_dying_cpu C: I */
	CPUHP_AP_MIPS_GIC_TIMER_STARTING,	/* P: 0 S: gic_starting_cpu T: gic_dying_cpu C: I */
	CPUHP_AP_KVM_STARTING,			/* P: 0 S: kvm_starting_cpu T: kvm_dying_cpu C: P */
	CPUHP_AP_KVM_ARM_VGIC_STARTING,		/* P: 0 S: vgic_starting_cpu T: vgic_dying_cpu C: I */
	CPUHP_AP_KVM_ARM_TIMER_STARTING,	/* P: 0 S: kvm_timer_starting_cpu T: kvm_timer_dying_cpu C: I */
	CPUHP_AP_ARM_KVM_STARTING,		/* P: 0 S: hyp_init_starting_cpu T: NULL C: I */
	CPUHP_AP_ARM_XEN_STARTING,		/* P: 0 S: xen_starting_cpu T: xen_dyning_cpu C: I */
	CPUHP_AP_ARM_CORESIGHT_STARTING,	/* P: 0 S: etm_starting_cpu T: etm_dying_cpu C: P */
	CPUHP_AP_ARM_CORESIGHT4_STARTING,	/* P: 0 S: etm4_starting_cpu T: etm4_dying_cpu C: P */
	CPUHP_AP_ARM64_ISNDEP_STARTING,		/* P: 0 S: run_all_insn_set_hw_mode T: NULL C: P */
	CPUHP_AP_LEDTRIG_STARTING,		/* P: 0 S: ledtrig_starting_cpu T: ledtrig_dying_cpu C: I */
	CPUHP_AP_RCUTREE_DYING,		/* P: 0 S: NULL T: rcutree_dying_cpu C: C */
	CPUHP_AP_SMPCFD_DYING,		/* P: 0 S: NULL T: smpcfd_dying_cpu C: C */
	CPUHP_AP_X86_TBOOT_DYING,	/* P: 0 S: NULL T: tboot_dying_cpu C: I */
	CPUHP_AP_SCHED_NOHZ_DYING,	/* P: 0 S: NULL T: nohz_balance_exit_idle C: C */
	CPUHP_AP_SCHED_MIGRATE_DYING,	/* P: 10 S: NULL T: sched_migration_dying_cpu C: C */
	CPUHP_AP_MAX,
	CPUHP_TEARDOWN_CPU,		/* P: __cpu_die S: NULL T: takedown_cpu C: C */
	CPUHP_PERCPU_THREADS,		/* P: 9 S: smpboot_unpark_threads T: smpboot_park_threads: C: C */
	CPUHP_SCHED_ONLINE,		/* P: INT_MAX S: sched_online_cpu T: NULL C: I */
	CPUHP_SCHED_ONLINE_NUMA,	/* P: INT_MAX S: sched_online_numa_cpu T: NULL C: I */
	CPUHP_SCHED_CPUSET_ONLINE,	/* P: INT_MAX-1 S: cpuset_cpu_active T: NULL C: I */
	CPUHP_SCHED_CPUSET_OFFLINE,	/* P: INT_MIN S: NULL T: cpuset_cpu_inactive C: I */
	CPUHP_SCHED_OFFLINE,		/* P: INT_MIN+1 S: NULL T: sched_offline_cpu C: I */
	CPUHP_X86_KVM_CLK_OFFLINE,	/* P: INT_MIN+1 S: NULL T: kvmclock_cpu_down_prep C: I */
	CPUHP_WORKQUEUE_OFFLINE,	/* P: -5 S: NULL T: workqueue_offline_cpu C: C */
	CPUHP_X86_VDSO_VMA_ONLINE,	/* P: 30 S: vgetcpu_online T: NULL C: I */
	CPUHP_PERF_X86_UNCORE_ONLINE,	/* P: 21 S: uncore_online_cpu T: uncore_offline_cpu C: I */
	CPUHP_PERF_X86_AMD_UNCORE_ONLINE, /* P: 21 S: amd_uncore_cpu_online T: amd_uncore_cpu_down_prepare C: I */
	CPUHP_PERF_ONLINE,		/* P: 20 S: perf_event_init_cpu T: perf_event_exit_cpu C: C */
	CPUHP_PERF_ARM_CCI_ONLINE,	/* P: 21 S: NULL T: cci_pmu_offline_cpu C: I */
	CPUHP_PERF_ARM_CCN_ONLINE,	/* P: 21 S: NULL T: arm_ccn_pmu_offline_cpu C: I */
	CPUHP_PERF_X86_ONLINE,		/* P: 20 S: x86_pmu_online_cpu T: NULL C: I */
	CPUHP_PERF_X86_RAPL_ONLINE,	/* P: 20 S: rapl_cpu_kfree T: rapl_cpu_exit C: I */
	CPUHP_PERF_X86_CQM_ONLINE,	/* P: 20 S: NULL T: intel_cqm_cpu_exit C: I */
	CPUHP_PERF_X86_CSTATE_ONLINE,	/* P: 20 S: NULL T: cstate_cpu_exit C: I */
	CPUHP_PERF_S390_CF_ONLINE,	/* P: 20 S: s390_pmu_online_cpu T: s390_pmu_offline_cpu C: I */
	CPUHP_PERF_S390_SF_ONLINE,	/* P: 20 S: s390_pmu_sf_online_cpu T: s390_pmu_sf_offline_cpu C: I */
	CPUHP_SCHED_MIGRATE_ONLINE,	/* P: 10 S: sched_migration_online_cpu T: NULL C: C */
	CPUHP_WORKQUEUE_ONLINE,		/* P: 5 S: workqueue_online_cpu T: NULL C: C */
	CPUHP_ARM_CORESIGHT_ONLINE,	/* P: 0 S: etm_online_cpu T: NULL C: P */
	CPUHP_ARM_CORESIGHT4_ONLINE,	/* P: 0 S: etm4_online_cpu T: NULL C: P */
	CPUHP_RCUTREE_ONLINE,           /* P: 0 S: rcutree_online_cpu T: rcutree_offline_cpu C: C */
	CPUHP_PROFILE_ONLINE,		/* P: 0 S: profile_online_cpu T: NULL C: I */
	CPUHP_SLAB_ONLINE,		/* P: 0 S: slab_online_cpu T: slab_offline_cpu C: C */
	CPUHP_MM_WRITEBACK_ONLINE,	/* P: 0 S: page_writeback_cpu_online T: NULL C: I */
	CPUHP_RCU_TORTURE,		/* P: 0 S: rcutorture_booster_init T: rcutorture_booster_cleanup C: I */
	CPUHP_NET_MVNETA_ONLINE,	/* P: 0 S: mvneta_cpu_online T: mvneta_cpu_down_prepare C: P */
	CPUHP_CPUIDLE_PSERIES_ONLINE,	/* P: 0 S: pseries_cpuidle_cpu_online T: NULL C: P */
	CPUHP_CPUIDLE_POWERNV_ONLINE,	/* P: 0 S: powernv_cpuidle_cpu_online T: NULL C: P */
	CPUHP_CPUIDLE_COUPLED_ONLINE,	/* P: 0 S: coupled_cpu_online T: coupled_cpu_up_prepare C: P */
	CPUHP_BUS_CDMM_ONLINE,		/* P: 0 S: mips_cdmm_cpu_online T: mips_cdmm_cpu_down_prep C: I */
	CPUHP_X86_KVM_ONLINE,		/* P: 0 S: kvm_cpu_online T: kvm_cpu_down_prepare C: P */
	CPUHP_POWER_PMAC_ONLINE,	/* P: 0 S: smp_core99_cpu_online T: NULL C: P */
	CPUHP_ARM_OMAP_WAKE_ONLINE,	/* P: 0 S: omap_wakegen_cpu_online T: NULL C: P */
	CPUHP_IA64_MCA_ONLINE,		/* P: 0 S: mca_cpu_online T: NULL C: I */
	CPUHP_X86_MICRCODE_ONLINE,	/* P: 0 S: mc_cpu_online T: mc_cpu_down_prep C: P */
	CPUHP_OPROFILE_TIMER_ONLINE,	/* P: 0 S: oprofile_timer_online T: oprofile_timer_prep_down C: P */
	CPUHP_VIRT_NET_ONLINE,		/* P: 0 S: virtnet_cpu_online T: virtnet_cpu_down_prep C: P */
	CPUHP_VIRT_SCSI_ONLINE,		/* P: 0 S: virtscsi_cpu_online T: NULL C: P */
	CPUHP_ACPI_CPUDRV_ONLINE,	/* P: 0 S: acpi_soft_cpu_online T: NULL C: P */
	CPUHP_CPUFREQ_ONLINE,		/* P: 0 S: cpufreq_online T: cpufreq_offline_prepare C: P */
	CPUHP_THERMAL_POWERCLMP_ONLINE,	/* P: 0 S: powerclamp_cpu_online T: NULL C: P */
	CPUHP_PADATA_ONLINE,		/* P: 0 S: padata_cpu_online T: padata_cpu_prep_down C: P */
	CPUHP_X86_X2APIC_NV_ONLINE,	/* P: 0 S: uv_heartbeat_enable T: uv_heartbeat_disable C: P */
	CPUHP_PRINTK_ONLINE,		/* P: 0 S: console_cpu_notify T: NULL C: P */
	CPUHP_SCHED_HRTICK_DOWN_PREP,	/* P: 0 S: NULL T: hotplug_hrtick_dead C: P */
	CPUHP_PERCPU_CNT_ONLINE,	/* P: 0 S: compute_batch_value T: NULL C: I */
	CPUHP_MM_VMSCAN_ONLINE,		/* P: 0 S: kswapd_cpu_online T: NULL C: P */
	CPUHP_S390_SMP_ONLINE,		/* P: 0 S: smp_cpu_online T: smp_cpu_pre_down C: I */
	CPUHP_CACHEINFO_ONLINE,		/* P: 0 S: cacheinfo_cpu_online T: cacheinfo_cpu_pre_down C: I */
	CPUHP_IA64_ERRINJ_ONLINE,	/* P: 0 S: err_inject_add_dev T: err_inject_remove_dev C: I */
	CPUHP_IA64_PALINFO_ONLINE,	/* P: 0 S: palinfo_add_proc T: palinfo_del_proc C: I */
	CPUHP_IA64_SALINFO_ONLINE,	/* P: 0 S: salinfo_cpu_online T: salinfo_cpu_pre_down C: I */
	CPUHP_IA64_TOPOLOGY_ONLINE,	/* P: 0 S: cache_cpu_online T: cache_cpu_pre_down C: I */
	CPUHP_X86_MCE_ONLINE,		/* P: 0 S: mce_cpu_online T: NULL C: I */
	CPUHP_HWMON_TEMP_ONLINE,	/* P: 0 S: coretemp_cpu_online T: coretemp_cpu_offline C: I */
	CPUHP_HWMON_VIA_ONLINE,		/* P: 0 S: via_cputemp_online T: via_cputemp_down_prep C: I */
	CPUHP_PCI_XGENE_ONLINE,		/* P: 0 S: xgene_msi_hwirq_alloc T: NULL C: I */
	CPUHP_POWERCAP_RAPL,		/* P: 0 S: rapl_cpu_online T: repl_cpu_prep_down C: P */
	CPUHP_SCSI_BNX2FC_ONLINE,	/* P: 0 S: bnx2fc_cpu_online T: NULL C: I */
	CPUHP_SCSI_BNX2I_ONLINE,	/* P: 0 S: bnx2i_cpu_online T: NULL C: I */
	CPUHP_SCSI_FCOE_ONLINE,		/* P: 0 S: fcoe_cpu_online T: NULL C: I */
	CPUHP_X86_PKG_THERM_ONLINE,	/* P: 0 S: pkg_thermal_cpu_online T: pkg_thermal_cpu_pre_down C: I */
	CPUHP_WDT_OCTEON_ONLINE,	/* P: 0 S: octeon_wdt_cpu_online T: octeon_wdt_cpu_pre_down C: I */
	CPUHP_NET_IUCV_ONLINE,		/* P: 0 S: iucv_cpu_online T: iucv_cpu_down_prep C: I */
	CPUHP_ARM_HWBREAK_ONLINE,	/* P: 0 S: dbg_reset_online T: NULL C: I */
	CPUHP_ARM64_DMON_ONLINE,	/* P: 0 S: os_lock_online T: NULL C: I */
	CPUHP_ARM64_HWBREAK_ONLINE,	/* P: 0 S: hw_breakpoint_online T: NULL C: I */
	CPUHP_POWERPC_TOPOLOGY_ONLINE,	/* P: 0 S: register_cpu_online T: unregister_cpu_online C: I */
	CPUHP_SPARC_TOPOLOGY_ONLINE,	/* P: 0 S: register_cpu_online T: unregister_cpu_online C: I */
	CPUHP_X86_OPRO_NMI_ONLINE,	/* P: 0 S: oprofile_cpu_online T: oprofile_cpu_predown C: I */
	CPUHP_PCI_AMDBUS_ONLINE,	/* P: 0 S: amd_bus_cpu_online T: NULL C: I */
	CPUHP_CPUFREQ_ACPI_PRE_DOWN,	/* P: 0 S: NULL T: cpufreq_boost_predown C: I */
	CPUHP_IDLE_INTEL_ONLINE,	/* P: 0 S: intel_idle_cpu_online T: NULL C: I */
	CPUHP_OPROFILE_NMI_ONLINE,	/* P: 0 S: nmi_timer_cpu_online T: nmi_timer_cpu_predown C: I */
	CPUHP_MM_VMSTAT_ONLINE,		/* P: 0 S: vmstat_cpu_online T: vmstat_cpu_down_prep C: P */
	CPUHP_TICK_NOHZ_PREDOWN,	/* P: 0 S: NULL T: tick_nohz_cpu_down C: P */
	CPUHP_ARM_BL_PREDOWN,		/* P: 0 S: NULL T: bL_switcher_cpu_pre C: P */
	CPUHP_X86_HPET_ONLINE,		/* P: -20 S: hpet_cpuhp_online T: NULL C: I */
	CPUHP_X86_KVM_CLK_ONLINE,	/* P: INT_MIN+1 S: kvmclock_cpu_online T: NULL C: I */
	CPUHP_NOTIFY_ONLINE,		/* P: CPU_ONLINE S: notify_online T: NULL, C: C */
	CPUHP_NOTIFY_DOWN_PREPARE,	/* P: CPU_DOWN_PREPARE S: NULL T: notify_down_prepare C: C */
	CPUHP_MAX,
};

#define CPUHP_ONLINE CPUHP_MAX

int __cpuhp_setup_state(enum cpuhp_state state, bool invoke,
			int (*startup)(unsigned int cpu),
			int (*teardown)(unsigned int cpu));

/**
 * cpuhp_setup_state - Setup hotplug state callbacks with calling the callbacks
 * @state:	The state for which the calls are installed
 * @startup:	startup callback function
 * @teardown:	teardown callback function
 *
 * Installs the callback functions and invokes the startup callback on
 * the present cpus which have already reached the @state.
 */
static inline int
cpuhp_setup_state(enum cpuhp_state state, int (*startup)(unsigned int cpu),
		  int (*teardown)(unsigned int cpu))
{
	return __cpuhp_setup_state(state, true, startup, teardown);
}

/**
 * cpuhp_setup_state_nocalls - Setup hotplug state callbacks without calling the
 *			       callbacks
 * @state:	The state for which the calls are installed
 * @startup:	startup callback function
 * @teardown:	teardown callback function
 *
 * No calls are executed. NOP if SMP=n or HOTPLUG_CPU=n
 */
#if defined(CONFIG_SMP) && defined(CONFIG_HOTPLUG_CPU)
static inline int
cpuhp_setup_state_nocalls(enum cpuhp_state state,
			 int (*startup)(unsigned int cpu),
			 int (*teardown)(unsigned int cpu))
{
	return __cpuhp_setup_state(state, false, startup, teardown);
}
#else
static inline int
cpuhp_setup_state_nocalls(enum cpuhp_state state,
			 int (*startup)(unsigned int cpu),
			 int (*teardown)(unsigned int cpu))
{
	return 0;
}
#endif

void __cpuhp_remove_state(enum cpuhp_state state, bool invoke);

/**
 * cpuhp_remove_state - Remove hotplug state callbacks and invoke the teardown
 * @state:	The state for which the calls are removed
 *
 * Removes the callback functions and invokes the teardown callback on
 * the present cpus which have already reached the @state.
 */
static inline void cpuhp_remove_state(enum cpuhp_state state)
{
	__cpuhp_remove_state(state, true);
}

/**
 * cpuhp_remove_state_nocalls - Remove hotplug state callbacks without invoking
 *				teardown
 * @state:	The state for which the calls are removed
 */
static inline void cpuhp_remove_state_nocalls(enum cpuhp_state state)
{
	__cpuhp_remove_state(state, false);
}

#if defined(CONFIG_NO_HZ_COMMON)
int nohz_balance_exit_idle(unsigned int cpu);
#else
#define nohz_balance_exit_idle	NULL
#endif

 /* Performance counter hotplug functions */
#ifdef CONFIG_PERF_EVENTS
int perf_event_init_cpu(unsigned int cpu);
int perf_event_exit_cpu(unsigned int cpu);
#else
#define perf_event_init_cpu	NULL
#define perf_event_exit_cpu	NULL
#endif

/* Workqueue related hotplug events */
int workqueue_prepare_cpu(unsigned int cpu);
int workqueue_online_cpu(unsigned int cpu);
int workqueue_offline_cpu(unsigned int cpu);

/* RCUtree hotplug events */
#if defined(CONFIG_TREE_RCU) || defined(CONFIG_PREEMPT_RCU)
int rcutree_prepare_cpu(unsigned int cpu);
int rcutree_online_cpu(unsigned int cpu);
int rcutree_offline_cpu(unsigned int cpu);
int rcutree_dead_cpu(unsigned int cpu);
int rcutree_dying_cpu(unsigned int cpu);
int rcutree_dying_idle_cpu(unsigned int cpu);
#else
#define rcutree_prepare_cpu	NULL
#define rcutree_online_cpu	NULL
#define rcutree_offline_cpu	NULL
#define rcutree_dead_cpu	NULL
#define rcutree_dying_cpu	NULL
#endif

int hrtimers_prepare_cpu(unsigned int cpu);
#ifdef CONFIG_HOTPLUG_CPU
int hrtimers_dead_cpu(unsigned int cpu);
int timers_dead_cpu(unsigned int cpu);
#else
#define hrtimers_dead_cpu	NULL
#define timers_dead_cpu		NULL
#endif

/* SMP core functions */
int smpcfd_prepare_cpu(unsigned int cpu);
int smpcfd_dead_cpu(unsigned int cpu);
int smpcfd_dying_cpu(unsigned int cpu);

#ifdef CONFIG_RELAY
int relay_prepare_cpu(unsigned int cpu);
#else
#define relay_prepare_cpu	NULL
#endif

/* slab hotplug events */
#if defined(CONFIG_SLAB) && defined(CONFIG_SMP)
int slab_prepare_cpu(unsigned int cpu);
int slab_online_cpu(unsigned int cpu);
int slab_offline_cpu(unsigned int cpu);
int slab_dead_cpu(unsigned int cpu);
#else
#define slab_prepare_cpu	NULL
#define slab_online_cpu		NULL
#define slab_offline_cpu	NULL
#define slab_dead_cpu		NULL
#endif

#ifdef CONFIG_RING_BUFFER
int trace_rb_cpu_prepare(unsigned int cpu);
#else
#define trace_rb_cpu_prepare	NULL
#endif

#endif

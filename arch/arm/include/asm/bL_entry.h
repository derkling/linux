/*
 * arch/arm/include/asm/bL_switcher.h
 *
 * Created by:  Nicolas Pitre, April 2012
 * Copyright:   (C) 2012  Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef BL_ENTRY_H
#define BL_ENTRY_H

#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
#define BL_CPUS_PER_CLUSTER	2
#else
#define BL_CPUS_PER_CLUSTER	4
#endif
#define BL_NR_CLUSTERS		2

/* Definitions for bL_cluster_sync_struct */
#define CPU_DOWN		0
#define CPU_COMING_UP		1
#define CPU_UP			2
#define CPU_GOING_DOWN		3

#define CLUSTER_DOWN		0
#define CLUSTER_UP		1
#define CLUSTER_GOING_DOWN	2

#define INBOUND_NOT_COMING_UP	0
#define INBOUND_COMING_UP	1

#define FIRST_MAN_ANY -1	/* any CPUs can init cluster simultaneously */
#define FIRST_MAN_NONE -2	/* no first man selected */


#ifndef __ASSEMBLY__

#include <linux/mm.h>
#include <linux/types.h>

/* Synchronisation structures for coordinating safe cluster setup/teardown: */

struct bL_cluster_sync_struct {
	s8	cpus[BL_CPUS_PER_CLUSTER];	/* individual CPU states */
	s8	cluster;	/* cluster state */
	s8	inbound;	/* inbound-side state */
	s8	first_man;	/* CPU index of elected first man */
};

struct bL_sync_struct {
	struct bL_cluster_sync_struct clusters[BL_NR_CLUSTERS];
};

/* How much physical memory to reserve for the synchronisation structure: */
#define BL_SYNC_MEM_RESERVE ALIGN(sizeof(struct bL_cluster_sync_struct), (1 << 21))

extern unsigned long bL_sync_phys;	/* physical address of *bL_sync */


/* 
 * Platform specific code should use this symbol to set up secondary
 * entry location after processors are released from reset.
 */
extern void bl_entry_point(void);

/*
 * This is used to indicate where the given CPU from given cluster should
 * branch once it is released from reset using ptr, or NULL if it should
 * be gated.  A gated CPU is held in a WFE loop until its vector becomes
 * non NULL.
 */
void bL_set_entry_vector(unsigned cpu, unsigned cluster, void *ptr);

#endif /* ! __ASSEMBLY__ */

#ifdef __ASSEMBLY__

#include <asm/asm-offsets.h>

.macro __bL_sync_get_cluster_base cluster_base:req, cluster:req, temp:req
	adr	\temp, .L__sync_phys_offset
	ldr	\cluster_base, [\temp]
	add	\cluster_base, \cluster_base, \temp
	ldr	\cluster_base, [\cluster_base]
	mov	\temp, #BL_SYNC_CLUSTER_SIZE
	mla	\cluster_base, \temp, \cluster, \cluster_base

	.ifndef .L__sync_phys_offset
	.text	1
	.align	2
.L__sync_phys_offset:
	.word	bL_sync_phys - .
	.previous
	.endif
.endm

.macro __bL_sync_offset_check
	.if BL_SYNC_CLUSTER_CPUS
		.error "cpus must be the first member of struct bL_cluster_sync_struct"
	.endif
.endm

/*
 * __bL_inbound_enter_critical: enter the cluster setup critical section.
 *
 * The code following the call to this macro until
 * __bL_inbound_leave_critical is protected by the critical section.
 *
 * If the calling CPU was not elected as the first man or if the cluster
 * is already set up (for example, cluster teardown was abandoned before
 * it began) this macro waits for cluster setup to complete if pending,
 * then jumps straight to \not_first_label, after
 *
 * Otherwise, any cluster teardown operations are allowed to get into a
 * safe state, and execution resumes after the macro call, allowing the
 * caller to perform cluster setup.
 */
.macro __bL_inbound_enter_critical cluster_base:req, cpu:req, not_first_label:req, temp1:req, temp2:req
	__bL_sync_offset_check

	@ Signal that this CPU is coming UP:
	mov	\temp1, #CPU_COMING_UP
	strb	\temp1, [\cluster_base, \cpu]

#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	dmb
	ldrb	\temp1, [\cluster_base, \cpu]
	isb
#else
	dsb
#endif

	@ At this point, the cluster cannot unexpectedly enter the GOING_DOWN
	@ state, because there is at least one active CPU (this CPU).

#if !defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	ldrb	\temp2, [\cluster_base, #BL_SYNC_CLUSTER_FIRST_MAN]
#endif

	@ Check if the cluster has been set up yet:
2180:	ldrb	\temp1, [\cluster_base, #BL_SYNC_CLUSTER_CLUSTER]
	cmp	\temp1, #CLUSTER_UP
	beq	\not_first_label

#if !defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	@ Otherwise, wait unless it is our job to set it up:
	cmp	\temp2, \cpu
	beq	2181f
	wfe
	b	2180b
#endif

2181:	@ This CPU is definitely the first man.

	@ Signal that the cluster is being brought up:
	mov	\temp1, #INBOUND_COMING_UP
	strb	\temp1, [\cluster_base, #BL_SYNC_CLUSTER_INBOUND]

#if !defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	@ Invalidate the first_man field:
	mov	\temp1, #FIRST_MAN_NONE
	strb	\temp1, [\cluster_base, #BL_SYNC_CLUSTER_FIRST_MAN]
#endif

#if defined(CONFIG_ARCH_VEXPRESS_TC2_IKS)
	dmb
	ldrb	\temp1, [\cluster_base, #BL_SYNC_CLUSTER_INBOUND]
	isb
#else
	dsb
#endif

	@ Any CPU trying to take the cluster into CLUSTER_GOING_DOWN from this
	@ point onwards will observe INBOUND_COMING_UP and abort.

	@ Wait for any previously-pending cluster teardown operations to abort
	@ or complete:
2182:	ldrb	\temp1, [\cluster_base, #BL_SYNC_CLUSTER_CLUSTER]
	cmp	\temp1, #CLUSTER_GOING_DOWN
	bne	2183f
	wfe
	b	2182b

	@ As an optimisation, if the outbound gave up before teardown started,
	@ skip cluster setup:
	cmp	\temp1, #CLUSTER_UP
	beq	\not_first_label

2183:	@ Fall through to the first-man code following the macro call.
.endm

/*
 * __bL_inbound_leave_critical: leave the cluster setup critical section.
 * signalling completion.
 *
 * This macro must not be called except from the end of the critical section.
 */
.macro __bL_inbound_leave_critical cluster_base:req, cpu:req, temp1:req
	dsb
	mov	\temp1, #INBOUND_NOT_COMING_UP
	strb	\temp1, [\cluster_base, #BL_SYNC_CLUSTER_INBOUND]
	mov	\temp1, #CLUSTER_UP
	strb	\temp1, [\cluster_base, #BL_SYNC_CLUSTER_CLUSTER]
	dsb
	sev
.endm

/*
 * __bL_inbound_cpu_up: mark the CPU as up.  Call this after local setup is
 * complete in bL_power_ops.power_up_setup().
 */
.macro __bL_inbound_cpu_up cluster_base:req, cpu:req, temp1:req
	__bL_sync_offset_check
	dsb
	mov	\temp1, #CPU_UP
	strb	\temp1, [\cluster_base, \cpu]
	dsb
	sev
.endm
#endif /* __ASSEMBLY__ */

#endif

/*
 *  arch/arm/include/asm/lb_lock.h
 *
 *  Copyright (C) 2004-2005 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARM_LB_LOCK_H
#define __ASM_ARM_LB_LOCK_H

/*
 * Lamport's Bakery algorithm for spinlock handling
 *
 * Note that the algorithm requires the bakery struct
 * to be in Strongly-Ordered memory.
 */

/*
 * Bakery structure - declare/allocate one of these for each lock.
 * A pointer to this struct is passed to the lock/unlock functions.
 */
typedef struct {
    volatile char entering[CONFIG_NR_CPUS];
    volatile unsigned number[CONFIG_NR_CPUS];
} bakery_t;

extern void initialize_spinlock(bakery_t *bakery);
extern void get_spinlock(unsigned cpuid, bakery_t *bakery);
extern void release_spinlock(unsigned cpuid, bakery_t *bakery);
#endif

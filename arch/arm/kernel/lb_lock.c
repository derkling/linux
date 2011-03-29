/*
 * Copyright (C) 2008-2010 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ----------------------------------------------------------------
 * File:     lb_lock.c
 * ----------------------------------------------------------------
 */

/**
 * Lamport's Bakery algorithm for spinlock handling
 *
 * Note that the algorithm requires the stack and the bakery struct
 * to be in Strongly-Ordered memory.
 */

#include <asm/system.h>
#include <asm/string.h>
#include <asm/lb_lock.h>

/*
 * Initialize a bakery - only required if the bakery_t is
 * on the stack or heap, as static data is zeroed anyway.
 */
void  initialize_spinlock(bakery_t *bakery)
{
	memset(bakery, 0, sizeof(bakery_t));
}

/*
 * Claim a bakery lock. Function does not return until
 * lock has been obtained.
 */
void  get_spinlock(unsigned cpuid, bakery_t *bakery)
{
	unsigned i, max = 0, my_full_number, his_full_number;

	/* Get a ticket */
	bakery->entering[cpuid] = 1;
	dmb();
	for (i = 0; i < CONFIG_NR_CPUS; ++i) {
		if (bakery->number[i] > max)
			max = bakery->number[i];
	}
	++max;
	bakery->number[cpuid] = max;
	dmb();
	bakery->entering[cpuid] = 0;

	/* Wait for our turn */
	my_full_number = (max << 8) + cpuid;
	for (i = 0; i < CONFIG_NR_CPUS; ++i) {
		while (bakery->entering[i])
			;	/* Wait */
		do {
			his_full_number = bakery->number[i];
			if (his_full_number)
				his_full_number =
				    (his_full_number << 8) + i;

		} while (his_full_number && (his_full_number < my_full_number));
	}
	dmb();
}

/*
 * Release a bakery lock.
 */
void  release_spinlock(unsigned cpuid, bakery_t *bakery)
{
	dmb();
	bakery->number[cpuid] = 0;
}

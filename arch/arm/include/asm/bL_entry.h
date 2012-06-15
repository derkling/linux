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

#define BL_CPUS_PER_CLUSTER	2
#define BL_NR_CLUSTERS		2

#ifndef __ASSEMBLY__

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

#endif

#endif

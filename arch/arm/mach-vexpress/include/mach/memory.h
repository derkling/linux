/*
 *  arch/arm/mach-vexpress/include/mach/memory.h
 *
 *  Copyright (C) 2003 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#if defined(CONFIG_VEXPRESS_ORIGINAL_MEMORY_MAP) || defined(CONFIG_VEXPRESS_ELBA_MEMORY_MAP)
#define PHYS_OFFSET		UL(0x60000000)
#elif defined(CONFIG_VEXPRESS_EXTENDED_MEMORY_MAP)
#define PHYS_OFFSET		UL(0x80000000)

#ifdef CONFIG_SPARSEMEM
#define MAX_PHYSMEM_BITS	36
#define SECTION_SIZE_BITS	31
#endif	/* CONFIG_SPARSEMEM */

#endif	/* MEMORY_MAP */

#define CONSISTENT_DMA_SIZE     (SZ_16M - SZ_2M)

#endif	/* __ASM_ARCH_MEMORY_H */

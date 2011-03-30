/*
 *  arch/arm/mach-vexpress/include/mach/io.h
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
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <mach/hardware.h>

#define IO_SPACE_LIMIT 0xffffffff

#if defined(CONFIG_PCI)
static inline void __iomem *fpga__io(unsigned long addr)
{
	/* check for PCI I/O space */
	if (addr >= VEXPRESS_PCI_IO_BASE && addr <= VEXPRESS_PCI_IO_LIMIT)
		return (void __iomem *)((addr - VEXPRESS_PCI_IO_BASE) + VEXPRESS_PCI_IO_VBASE);
	else
		return (void __iomem *)addr;
}
#else
static inline void __iomem *fpga__io(unsigned long addr) { return NULL; }
#endif

// #define __io(a)		__typesafe_io(a)
#define __io(a)                 fpga__io(a)
#define __mem_pci(a)	(a)

#endif

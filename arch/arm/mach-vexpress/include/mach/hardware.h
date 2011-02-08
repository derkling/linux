/*
 *  arch/arm/mach-vexpress/include/mach/hardware.h
 *
 *  This file contains the hardware definitions for the Versatile Express boards.
 *
 *  Copyright (C) 2009 ARM Limited.
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/motherboard.h>

/* PCI stuff */
#ifndef CONFIG_VEXPRESS_PCIE_RC_IN_FPGA
#define CFGRW2			VEXPRESS_SYSREG_BASE + 0x08
#define CFGRW3			VEXPRESS_SYSREG_BASE + 0x0c
#define SLV_ARMISC_INFO	CFGRW3
#define CFGRW4			VEXPRESS_SYSREG_BASE + 0x10
#define SLV_AWMISC_INFO	CFGRW4
#endif

/* offsets in config space */
#define PCI_PORT_LINK_CONTROL	0x710
#define PCI_RC_DEBUGREG1	0x72c

#ifdef CONFIG_VEXPRESS_PCIE_RC_IN_FPGA

/* Base, limit & mask values for setting up the AXI and OB translation units.
 * The offset regs will be set up dynamically (see pcie.c).
 */
#define VEXPRESS_PCI_DBI_BASE	VEXPRESS_PCI_BASE
#define VEXPRESS_PCI_DBI_SIZE	SZ_4K
#define VEXPRESS_PCI_DBI_LIMIT	(VEXPRESS_PCI_DBI_BASE + VEXPRESS_PCI_DBI_SIZE - 1)
#define VEXPRESS_PCI_DBI_MASK	(VEXPRESS_PCI_DBI_SIZE - 1)

#define VEXPRESS_PCI_CFG0_BASE	VEXPRESS_PCI_DBI_BASE + VEXPRESS_PCI_DBI_SIZE
#define VEXPRESS_PCI_CFG0_SIZE	SZ_4K
#define VEXPRESS_PCI_CFG0_LIMIT	(VEXPRESS_PCI_CFG0_BASE + VEXPRESS_PCI_CFG0_SIZE - 1)
#define VEXPRESS_PCI_CFG0_MASK	(VEXPRESS_PCI_CFG0_SIZE - 1)

#define VEXPRESS_PCI_CFG1_BASE	VEXPRESS_PCI_CFG0_BASE + VEXPRESS_PCI_CFG0_SIZE
#define VEXPRESS_PCI_CFG1_SIZE	SZ_4K
#define VEXPRESS_PCI_CFG1_LIMIT	(VEXPRESS_PCI_CFG1_BASE + VEXPRESS_PCI_CFG1_SIZE - 1)
#define VEXPRESS_PCI_CFG1_MASK	(VEXPRESS_PCI_CFG1_SIZE - 1)

#define VEXPRESS_PCI_IO_BASE	(VEXPRESS_PCI_BASE + SZ_64K)
#define VEXPRESS_PCI_IO_SIZE	SZ_64K
#define VEXPRESS_PCI_IO_LIMIT	(VEXPRESS_PCI_IO_BASE + VEXPRESS_PCI_IO_SIZE - 1)
#define VEXPRESS_PCI_IO_MASK	0xFFFFFFFF

#define VEXPRESS_PCI_MEM_BASE	(VEXPRESS_PCI_IO_BASE + VEXPRESS_PCI_IO_SIZE)
#define VEXPRESS_PCI_MEM_LIMIT	0xFFFFFFFC
#define VEXPRESS_PCI_MEM_MASK	0xFFFFFFFF

#define VEXPRESS_PCI_VBASE	0xD0000000	/* virt addr of config space */
#define VEXPRESS_PCI_DBI_VBASE	(VEXPRESS_PCI_VBASE + VEXPRESS_PCI_DBI_BASE - VEXPRESS_PCI_BASE)
#define VEXPRESS_PCI_CFG0_VBASE	(VEXPRESS_PCI_VBASE + VEXPRESS_PCI_CFG0_BASE - VEXPRESS_PCI_BASE)
#define VEXPRESS_PCI_CFG1_VBASE	(VEXPRESS_PCI_VBASE + VEXPRESS_PCI_CFG1_BASE - VEXPRESS_PCI_BASE)
#define VEXPRESS_PCI_IO_VBASE	(VEXPRESS_PCI_VBASE + VEXPRESS_PCI_IO_BASE - VEXPRESS_PCI_BASE)

#define PCIBIOS_MIN_MEM		VEXPRESS_PCI_MEM_BASE	/* bus = phys addr */
#define PCIBIOS_MIN_IO		VEXPRESS_PCI_IO_BASE	/* bus = phys address */

#else

#define VEXPRESS_PCI_MEM_BASE	VEXPRESS_PCI_BASE	/* mem at same address */
#define VEXPRESS_PCI_MEM_SIZE	SZ_512M		/* 512 MB */
#define VEXPRESS_PCI_IO_BASE	VEXPRESS_PCI_BASE	/* I/O at same address */
#define VEXPRESS_PCI_IO_SIZE	SZ_1M		/* 1 MB */

#define VEXPRESS_PCI_VBASE	0xD0000000	/* virt addr of config space */
#define VEXPRESS_PCI_MEM_VBASE	VEXPRESS_PCI_VBASE	/* mem at same address */
#define VEXPRESS_PCI_IO_VBASE	VEXPRESS_PCI_VBASE	/* I/O at same address */


/* Addresses used by PCI IO/MEM window allocator
 * Note that we don't want I/O allocated at all but passing (-1) as the min
 * when it uses (-1) as the max doesn't seem to do it.
 *
 * These should be PHYSICAL addresses. The virtual ones are still here because
 * it took a while to find out exactly what was going on.
 */

#define  VEXPRESS_ALLOCATE_PCI_MEM_SPACE
#undef   VEXPRESS_ALLOCATE_PCI_IO_SPACE

#if defined(VEXPRESS_ALLOCATE_PCI_MEM_SPACE) && defined(VEXPRESS_ALLOCATE_PCI_IO_SPACE)
#warning can support only one of VEXPRESS_ALLOCATE_PCI_MEM_SPACE and VEXPRESS_ALLOCATE_PCI_IO_SPACE
#endif

#ifdef VEXPRESS_ALLOCATE_PCI_MEM_SPACE
#define PCIBIOS_MIN_MEM		VEXPRESS_PCI_MEM_BASE		/* bus = phys addr */
#else
#define PCIBIOS_MIN_MEM		(-1)
#endif


#ifdef VEXPRESS_ALLOCATE_PCI_IO_SPACE
#define PCIBIOS_MIN_IO		VEXPRESS_PCI_IO_BASE		/* bus = phys address */
#else
#define PCIBIOS_MIN_IO	(-1)
#endif

#endif	/* CONFIG_VEXPRESS_PCIE_RC_IN_FPGA */

#define pcibios_assign_all_busses()     0

#endif

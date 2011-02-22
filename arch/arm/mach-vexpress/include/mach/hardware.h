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

/* offsets in config space */
#define PCI_PORT_LINK_CONTROL	0x710
#define PCI_RC_DEBUGREG1	0x72c


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

#define pcibios_assign_all_busses()     0

#endif

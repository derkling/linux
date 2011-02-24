/* arch/arm/mach-realview/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/uart.h>
#include <mach/core.h>

		.macro	addruart,rp,rv
		ldr	\rp, =VEXPRESS_UART0			@ physical base
		ldr	\rv, =__MMIO_P2V(VEXPRESS_UART0)	@ virtual base
		.endm

#include <asm/hardware/debug-pl01x.S>

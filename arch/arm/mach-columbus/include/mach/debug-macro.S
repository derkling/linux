/* arch/arm/mach-columbus/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/columbus.h>
	
		.macro	addruart,rp,rv,tmp
		mov	\rp, #DEBUG_LL_UART_OFFSET
		orr	\rv, \rp, #DEBUG_LL_UART_VIRT_BASE
		orr	\rp, \rp, #DEBUG_LL_UART_PHYS_BASE
		.endm

#include <asm/hardware/debug-pl01x.S>

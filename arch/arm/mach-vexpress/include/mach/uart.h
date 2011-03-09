#ifndef __MACH_UART_H
#define __MACH_UART_H

#include <mach/motherboard.h>

#ifdef CONFIG_VEXPRESS_USE_TILE_UART0

#ifdef CONFIG_ARCH_VEXPRESS_LT_ELBA
#include <mach/lt-elba.h>
#define VEXPRESS_UART0		LT_ELBA_UART0
#endif

#else
#define VEXPRESS_UART0		V2M_UART0
#endif

#endif

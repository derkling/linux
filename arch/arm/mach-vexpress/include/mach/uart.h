#ifndef __MACH_UART_H
#define __MACH_UART_H

#include <mach/motherboard.h>

#if defined(CONFIG_VEXPRESS_USE_TILE_UART0) || defined(CONFIG_ARCH_TUSCAN)
#include <mach/lt-elba.h>
#define VEXPRESS_UART0		LT_ELBA_UART0
#else
#define VEXPRESS_UART0		V2M_UART0
#endif

#endif

// SPDX-License-Identifier: GPL-2.0
#ifndef _INCLUDE_KPTI_H
#define _INCLUDE_KPTI_H

#ifdef CONFIG_KERNEL_PAGE_TABLE_ISOLATION
#include <asm/kpti.h>
#else
static inline void kpti_init(void) { }
#endif

#endif

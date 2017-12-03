// SPDX-License-Identifier: GPL-2.0
#ifndef _ASM_X86_KPTI_H
#define _ASM_X86_KPTI_H
#ifndef __ASSEMBLY__

#ifdef CONFIG_KERNEL_PAGE_TABLE_ISOLATION
extern void kpti_init(void);
extern void kpti_check_boottime_disable(void);
#else
static inline void kpti_check_boottime_disable(void) { }
#endif

#endif /* __ASSEMBLY__ */
#endif /* _ASM_X86_KPTI_H */

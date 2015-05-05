/*
 * x86 FPU performance measurement methods:
 */
#ifndef _ASM_X86_FPU_MEASURE_H
#define _ASM_X86_FPU_MEASURE_H

#ifdef CONFIG_X86_DEBUG_FPU_PERFORMANCE
extern void fpu__measure(void);
#else
static inline void fpu__measure(void) { }
#endif

#endif /* _ASM_X86_FPU_MEASURE_H */

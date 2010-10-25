#define IRQ_LOCALTIMER		29
#define IRQ_LOCALWDOG		30

#ifndef CONFIG_ARCH_VEXPRESS_LT_ELBA
#define NR_IRQS	128
#else
#define NR_IRQS 160
#endif

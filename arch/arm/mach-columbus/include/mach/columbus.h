#ifdef CONFIG_COLUMBUS_MODEL
#define DEBUG_LL_UART_OFFSET		0x00090000

#define DEBUG_LL_UART_VIRT_BASE		0xf8000000
#define DEBUG_LL_UART_PHYS_BASE		0x1c000000

#define COLUMBUS_SYS_FLAGS_VIRT_BASE	0xf8010000
#define COLUMBUS_SYS_FLAGS_PHYS_BASE	0x1c010000

#define COLUMBUS_SYS_FLAGS_SET_OFFSET	0x30
#define COLUMBUS_SYS_FLAGS_CLR_OFFSET	0x34
#endif /* CONFIG_COLUMBUS_MODEL */

#define COLUMBUS_MMCI_VIRT_BASE		0xf8050000
#define COLUMBUS_MMCI_PHYS_BASE		0x1c050000

#define COLUMBUS_PERIPH_PHYS_BASE	0x2c000000
#define COLUMBUS_PERIPH_VIRT_BASE	0xf9000000

#ifndef __ASSEMBLY__

/* Shared functions between all boards */
void columbus_map_common_io(void);
void columbus_init_irq(void);

/* Shared structures between all boards */
extern struct sys_timer columbus_local_timer;

#ifdef CONFIG_SMP
/* Map the cpu-release-addr address */
void columbus_smp_map_io(void);
#endif /* CONFIG_SMP */

#endif /* __ASSEMBLY */

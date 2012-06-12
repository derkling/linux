#if defined(CONFIG_COLUMBUS_MODEL) || defined(CONFIG_COLUMBUS_TC2)
#define DEBUG_LL_UART_OFFSET		0x00090000

#define DEBUG_LL_UART_VIRT_BASE		0xf8000000
#define DEBUG_LL_UART_PHYS_BASE		0x1c000000

#define COLUMBUS_SYS_FLAGS_VIRT_BASE	0xf8010000
#define COLUMBUS_SYS_FLAGS_PHYS_BASE	0x1c010000

#define COLUMBUS_SYS_FLAGS_SET_OFFSET	0x30
#define COLUMBUS_SYS_FLAGS_CLR_OFFSET	0x34
#define COLUMBUS_SYS_FLAGS_24MHZ_OFFSET	0x5c

#define COLUMBUS_CCI400_VIRT_BASE	0xf8020000
#define COLUMBUS_CCI400_PHYS_BASE	0x2c090000
#define COLUMBUS_CCI400_EAG_OFFSET	0x4000
#define COLUMBUS_CCI400_KF_OFFSET	0x5000
#endif /* CONFIG_COLUMBUS_MODEL */

#define COLUMBUS_MMCI_VIRT_BASE		0xf8050000
#define COLUMBUS_MMCI_PHYS_BASE		0x1c050000

#define COLUMBUS_PERIPH_PHYS_BASE	0x2c000000
#define COLUMBUS_PERIPH_VIRT_BASE	0xf9000000

/*
 * Miscellaneous
 */
#define SYS_MISC_MASTERSITE	(1 << 14)
#define SYS_PROCIDx_HBI_MASK	0xfff

/*
 * Configuration
 */
#define SYS_CFG_START		(1 << 31)
#define SYS_CFG_WRITE		(1 << 30)
#define SYS_CFG_OSC		(1 << 20)
#define SYS_CFG_VOLT		(2 << 20)
#define SYS_CFG_AMP		(3 << 20)
#define SYS_CFG_TEMP		(4 << 20)
#define SYS_CFG_RESET		(5 << 20)
#define SYS_CFG_SCC		(6 << 20)
#define SYS_CFG_MUXFPGA		(7 << 20)
#define SYS_CFG_SHUTDOWN	(8 << 20)
#define SYS_CFG_REBOOT		(9 << 20)
#define SYS_CFG_DVIMODE		(11 << 20)
#define SYS_CFG_POWER		(12 << 20)
#define SYS_CFG_SITE_MB		(0 << 16)
#define SYS_CFG_SITE_DB1	(1 << 16)
#define SYS_CFG_SITE_DB2	(2 << 16)
#define SYS_CFG_STACK(n)	((n) << 12)

#define SYS_CFG_ERR		(1 << 1)
#define SYS_CFG_COMPLETE	(1 << 0)

#define V2M_SYS_MISC		0x060
#define V2M_SYS_CFGDATA		0x0a0
#define V2M_SYS_CFGCTRL		0x0a4
#define V2M_SYS_CFGSTAT		0x0a8


#ifndef __ASSEMBLY__

/* Shared functions between all boards */
void columbus_map_common_io(void);
void columbus_init_irq(void);
void columbus_init_timer(void);

/* Shared structures between all boards */
extern struct sys_timer columbus_local_timer;

#ifdef CONFIG_SMP
/* Map the cpu-release-addr address */
void columbus_smp_map_io(void);
#endif /* CONFIG_SMP */

#endif /* __ASSEMBLY */

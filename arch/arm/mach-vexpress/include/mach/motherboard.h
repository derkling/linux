#ifndef __MACH_MOTHERBOARD_H
#define __MACH_MOTHERBOARD_H

#include <mach/hardware.h>

/*
 * Physical addresses, offset from V2M_PA_CS0-3
 */
#define V2M_NOR0		(V2M_PA_CS0)
#define V2M_NOR1		(V2M_PA_CS1)
#define V2M_SRAM		(V2M_PA_CS2)
#define V2M_VIDEO_SRAM		(V2M_PA_CS3 + 0x00000000)
#define V2M_LAN9118		(V2M_PA_CS3 + 0x02000000)
#define V2M_ISP1761		(V2M_PA_CS3 + 0x03000000)

/*
 * Physical addresses, offset from V2M_PA_CS7
 */
#ifdef CONFIG_VEXPRESS_ORIGINAL_MEMORY_MAP
/* CS register bases for the original memory map. */
#define V2M_PA_CS0		0x40000000
#define V2M_PA_CS1		0x44000000
#define V2M_PA_CS2		0x48000000
#define V2M_PA_CS3		0x4c000000
#define V2M_PA_CS7		0x10000000

#define V2M_PERIPH_OFFSET(x)	(x << 12)
#define V2M_SYSREGS		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(0))
#define V2M_SYSCTL		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(1))
#define V2M_SERIAL_BUS_PCI	(V2M_PA_CS7 + V2M_PERIPH_OFFSET(2))

#elif defined(CONFIG_VEXPRESS_EXTENDED_MEMORY_MAP)
/* CS register bases for the extended memory map. */
#define V2M_PA_CS0		0x08000000
#define V2M_PA_CS1		0x0c000000
#define V2M_PA_CS2		0x14000000
#define V2M_PA_CS3		0x18000000
#define V2M_PA_CS7		0x1c000000

#define V2M_PERIPH_OFFSET(x)	(x << 16)
#define V2M_SYSREGS		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(1))
#define V2M_SYSCTL		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(2))
#define V2M_SERIAL_BUS_PCI	(V2M_PA_CS7 + V2M_PERIPH_OFFSET(3))

#elif defined(CONFIG_VEXPRESS_ELBA_MEMORY_MAP)
/* CS register bases for the ELBA memory map */
#define V2M_PA_CS0		0xEC000000
#define V2M_PA_CS1		0xF0000000
#define V2M_PA_CS2		0xF4000000
#define V2M_PA_CS3		0xF8000000
#define V2M_PA_CS4		0xFC000000
#define V2M_PA_CS5		0xFD000000
#define V2M_PA_CS6		0xFE000000
#define V2M_PA_CS7		0xFF000000

#define V2M_PERIPH_OFFSET(x)	(x << 12)
#define V2M_SYSREGS		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(0))
#define V2M_SYSCTL		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(1))
#define V2M_SERIAL_BUS_PCI	(V2M_PA_CS7 + V2M_PERIPH_OFFSET(2))
#endif

/* Common peripherals relative to CS7. */
#define V2M_AACI		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(4))
#define V2M_MMCI		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(5))
#define V2M_KMI0		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(6))
#define V2M_KMI1		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(7))

#define V2M_UART0		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(9))
#define V2M_UART1		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(10))
#define V2M_UART2		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(11))
#define V2M_UART3		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(12))

#define V2M_WDT			(V2M_PA_CS7 + V2M_PERIPH_OFFSET(15))

#define V2M_TIMER01		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(17))
#define V2M_TIMER23		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(18))

#define V2M_RTC			(V2M_PA_CS7 + V2M_PERIPH_OFFSET(23))

#define V2M_SERIAL_BUS_DVI	(V2M_PA_CS7 + V2M_PERIPH_OFFSET(22))

#define V2M_CF			(V2M_PA_CS7 + V2M_PERIPH_OFFSET(26))

#define V2M_CLCD		(V2M_PA_CS7 + V2M_PERIPH_OFFSET(31))
#define V2M_SIZE_CS7		V2M_PERIPH_OFFSET(32)

/* System register offsets. */
#define V2M_SYS_ID		(V2M_SYSREGS + 0x000)
#define V2M_SYS_SW		(V2M_SYSREGS + 0x004)
#define V2M_SYS_LED		(V2M_SYSREGS + 0x008)
#define V2M_SYS_100HZ		(V2M_SYSREGS + 0x024)
#define V2M_SYS_FLAGS		(V2M_SYSREGS + 0x030)
#define V2M_SYS_FLAGSSET	(V2M_SYSREGS + 0x030)
#define V2M_SYS_FLAGSCLR	(V2M_SYSREGS + 0x034)
#define V2M_SYS_NVFLAGS		(V2M_SYSREGS + 0x038)
#define V2M_SYS_NVFLAGSSET	(V2M_SYSREGS + 0x038)
#define V2M_SYS_NVFLAGSCLR	(V2M_SYSREGS + 0x03c)
#define V2M_SYS_MCI		(V2M_SYSREGS + 0x048)
#define V2M_SYS_FLASH		(V2M_SYSREGS + 0x03c)
#define V2M_SYS_CFGSW		(V2M_SYSREGS + 0x058)
#define V2M_SYS_24MHZ		(V2M_SYSREGS + 0x05c)
#define V2M_SYS_MISC		(V2M_SYSREGS + 0x060)
#define V2M_SYS_DMA		(V2M_SYSREGS + 0x064)
#define V2M_SYS_PROCID0		(V2M_SYSREGS + 0x084)
#define V2M_SYS_PROCID1		(V2M_SYSREGS + 0x088)
#define V2M_SYS_CFGDATA		(V2M_SYSREGS + 0x0a0)
#define V2M_SYS_CFGCTRL		(V2M_SYSREGS + 0x0a4)
#define V2M_SYS_CFGSTAT		(V2M_SYSREGS + 0x0a8)

#define V2M_TIMER0		(V2M_TIMER01 + 0x000)
#define V2M_TIMER1		(V2M_TIMER01 + 0x020)

#define V2M_TIMER2		(V2M_TIMER23 + 0x000)
#define V2M_TIMER3		(V2M_TIMER23 + 0x020)


/*
 * Interrupts.  Those in {} are for AMBA devices
 */
#ifndef CONFIG_ARCH_VEXPRESS_LT_ELBA
#define IRQ_V2M_WDT		{ (32 + 0) }
#define IRQ_V2M_TIMER0		(32 + 2)
#define IRQ_V2M_TIMER1		(32 + 2)
#define IRQ_V2M_TIMER2		(32 + 3)
#define IRQ_V2M_TIMER3		(32 + 3)
#define IRQ_V2M_RTC		{ (32 + 4) }
#define IRQ_V2M_UART0		{ (32 + 5) }
#define IRQ_V2M_UART1		{ (32 + 6) }
#define IRQ_V2M_UART2		{ (32 + 7) }
#define IRQ_V2M_UART3		{ (32 + 8) }
#define IRQ_V2M_MMCI		{ (32 + 9), (32 + 10) }
#define IRQ_V2M_AACI		{ (32 + 11) }
#define IRQ_V2M_KMI0		{ (32 + 12) }
#define IRQ_V2M_KMI1		{ (32 + 13) }
#define IRQ_V2M_CLCD		{ (32 + 14) }
#define IRQ_V2M_LAN9118		(32 + 15)
#define IRQ_V2M_ISP1761		(32 + 16)
#define IRQ_V2M_PCIE		(32 + 17)
#else
#define IRQ_V2M_WDT		{ (32 + 91) } /* on motherboard, mapped to extint[3] */
#define IRQ_V2M_TIMER0		(32 + 88)     /* on motherboard, mapped to extint[0] */
#define IRQ_V2M_TIMER1		(32 + 88)     /* on motherboard, mapped to extint[0] */
#define IRQ_V2M_TIMER2		(32 + 89)     /* on motherboard, mapped to extint[1] */
#define IRQ_V2M_TIMER3		(32 + 89)     /* on motherboard, mapped to extint[1] */
#define IRQ_V2M_RTC		{ (32 + 88) } /* on motherboard, mapped to extint[0] */
#define IRQ_V2M_UART0		{ (32 + 89) } /* on motherboard, mapped to extint[1] */
/*#define IRQ_V2M_UART1		{ (32 + 6) }
#define IRQ_V2M_UART2		{ (32 + 7) }
#define IRQ_V2M_UART3		{ (32 + 8) } */
#define IRQ_V2M_MMCI		{ (32 + 90),  NO_IRQ }  /* on motherboard, mapped to extint[2] */
#define IRQ_V2M_AACI		{ (32 + 91) } /* on motherboard, mapped to extint[3] */
#define IRQ_V2M_KMI0		{ (32 + 92) } /* on motherboard, mapped to extint[4] */
#define IRQ_V2M_KMI1		{ (32 + 93) } /* on motherboard, mapped to extint[5] */
#define IRQ_V2M_CLCD		(32 + 95)     /* on motherboard, mapped to extint[7] */
#define IRQ_V2M_LAN9118		(32 + 94)     /* on motherboard, mapped to extint[6] */
#define IRQ_V2M_ISP1761		(32 + 95)     /* on motherboard, mapped to extint[7] */
#define IRQ_V2M_PCIE		(32 + 26)
#endif


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

#ifndef __ASSEMBLY__
int v2m_cfg_write(u32 devfn, u32 data);
int v2m_cfg_read(u32 devfn, u32 *data);


/*
 * Memory definitions
 */
#define VEXPRESS_BOOT_ROM_LO		0x30000000		/* DoC Base (64Mb)...*/
#define VEXPRESS_BOOT_ROM_HI		0x30000000
#define VEXPRESS_BOOT_ROM_BASE	VEXPRESS_BOOT_ROM_HI		/* Normal position */
#define VEXPRESS_BOOT_ROM_SIZE	SZ_64M

#define VEXPRESS_SSRAM_BASE		/* VEXPRESS_SSMC_BASE ? */
#define VEXPRESS_SSRAM_SIZE		SZ_2M

/*
 *  SDRAM
 */
#define VEXPRESS_SDRAM_BASE		0x00000000

/*
 *  Logic expansion modules
 *
 */


/* ------------------------------------------------------------------------
 *  Versatile Express Registers
 * ------------------------------------------------------------------------
 *
 */
#define VEXPRESS_SYS_ID_OFFSET		0x00
#define VEXPRESS_SYS_SW_OFFSET		0x04
#define VEXPRESS_SYS_LED_OFFSET		0x08

#define VEXPRESS_SYS_100HZ_OFFSET	0x24
#define VEXPRESS_SYS_FLAGS_OFFSET	0x30
#define VEXPRESS_SYS_FLAGSSET_OFFSET	0x30
#define VEXPRESS_SYS_FLAGSCLR_OFFSET	0x34
#define VEXPRESS_SYS_NVFLAGS_OFFSET	0x38
#define VEXPRESS_SYS_NVFLAGSSET_OFFSET	0x38
#define VEXPRESS_SYS_NVFLAGSCLR_OFFSET	0x3C
#define VEXPRESS_SYS_MCI_OFFSET		0x48
#define VEXPRESS_SYS_FLASH_OFFSET	0x4C
#define VEXPRESS_SYS_CLCD_OFFSET	0x50
#define VEXPRESS_SYS_BOOTCS_OFFSET	0x58
#define VEXPRESS_SYS_24MHz_OFFSET	0x5C
#define VEXPRESS_SYS_MISC_OFFSET	0x60
#define VEXPRESS_SYS_PROCID_OFFSET	0x84

#define VEXPRESS_SYS_BASE		0x10000000
#define VEXPRESS_SYS_ID			(VEXPRESS_SYS_BASE + VEXPRESS_SYS_ID_OFFSET)
#define VEXPRESS_SYS_SW			(VEXPRESS_SYS_BASE + VEXPRESS_SYS_SW_OFFSET)
#define VEXPRESS_SYS_LED		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_LED_OFFSET)

#define VEXPRESS_SYS_100HZ		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_100HZ_OFFSET)
#define VEXPRESS_SYS_FLAGS		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_FLAGS_OFFSET)
#define VEXPRESS_SYS_FLAGSSET		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_FLAGSSET_OFFSET)
#define VEXPRESS_SYS_FLAGSCLR		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_FLAGSCLR_OFFSET)
#define VEXPRESS_SYS_NVFLAGS		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_NVFLAGS_OFFSET)
#define VEXPRESS_SYS_NVFLAGSSET		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_NVFLAGSSET_OFFSET)
#define VEXPRESS_SYS_NVFLAGSCLR		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_NVFLAGSCLR_OFFSET)
#define VEXPRESS_SYS_MCI		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_MCI_OFFSET)
#define VEXPRESS_SYS_FLASH		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_FLASH_OFFSET)
#define VEXPRESS_SYS_CLCD		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_CLCD_OFFSET)
#define VEXPRESS_SYS_BOOTCS		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_BOOTCS_OFFSET)
#define VEXPRESS_SYS_24MHz		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_24MHz_OFFSET)
#define VEXPRESS_SYS_MISC		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_MISC_OFFSET)
#define VEXPRESS_SYS_PROCID		(VEXPRESS_SYS_BASE + VEXPRESS_SYS_PROCID_OFFSET)

#define VEXPRESS_SYS_CTRL_LED		(1 << 0)

/* ------------------------------------------------------------------------
 *  Versatile Express control registers
 * ------------------------------------------------------------------------
 */

/*
 * VEXPRESS_IDFIELD
 *
 * 31:24 = manufacturer (0x41 = ARM)
 * 23:16 = architecture (0x08 = AHB system bus, ASB processor bus)
 * 15:12 = FPGA (0x3 = XVC600 or XVC600E)
 * 11:4  = build value
 * 3:0	 = revision number (0x1 = rev B (AHB))
 */

/*
 * VEXPRESS_SYS_FLASH
 */
#define VEXPRESS_FLASHPROG_FLVPPEN	(1 << 0)	/* Enable writing to flash */

/*
 * VEXPRESS_INTREG
 *     - used to acknowledge and control MMCI and UART interrupts
 */
#define VEXPRESS_INTREG_WPROT		0x00	/* MMC protection status (no interrupt generated) */
#define VEXPRESS_INTREG_RI0		0x01	/* Ring indicator UART0 is asserted,		  */
#define VEXPRESS_INTREG_CARDIN		0x08	/* MMCI card in detect				  */
						/* write 1 to acknowledge and clear		  */
#define VEXPRESS_INTREG_RI1		0x02	/* Ring indicator UART1 is asserted,		  */
#define VEXPRESS_INTREG_CARDINSERT	0x03	/* Signal insertion of MMC card			  */

/*
 * Versatile Express common peripheral addresses
 */
#define VEXPRESS_SCTL_BASE		0x10001000	/* System controller */
#define VEXPRESS_I2C_BASE		0x10002000	/* I2C control */
#define VEXPRESS_AACI_BASE		0x10004000	/* Audio */
#define VEXPRESS_MMCI0_BASE		0x10005000	/* MMC interface */
#define VEXPRESS_KMI0_BASE		0x10006000	/* KMI interface */
#define VEXPRESS_KMI1_BASE		0x10007000	/* KMI 2nd interface */
#define VEXPRESS_CHAR_LCD_BASE		0x10008000	/* Character LCD */
#define VEXPRESS_SCI_BASE		0x1000E000	/* Smart card controller */
#define VEXPRESS_GPIO1_BASE		0x10014000	/* GPIO port 1 */
#define VEXPRESS_GPIO2_BASE		0x10015000	/* GPIO port 2 */
#define VEXPRESS_DMAC_BASE		0x10030000	/* DMA controller */

#define VEXPRESS_SDRAM67_BASE		0x70000000	/* SDRAM banks 6 and 7 */

/*
 *  LED settings, bits [7:0]
 */
#define VEXPRESS_SYS_LED0	(1 << 0)
#define VEXPRESS_SYS_LED1	(1 << 1)
#define VEXPRESS_SYS_LED2	(1 << 2)
#define VEXPRESS_SYS_LED3	(1 << 3)
#define VEXPRESS_SYS_LED4	(1 << 4)
#define VEXPRESS_SYS_LED5	(1 << 5)
#define VEXPRESS_SYS_LED6	(1 << 6)
#define VEXPRESS_SYS_LED7	(1 << 7)

#define ALL_LEDS		0xFF

#define LED_BANK		VEXPRESS_SYS_LED

/*
 * Control registers
 */
#define VEXPRESS_IDFIELD_OFFSET		0x0	/* Build information */
#define VEXPRESS_FLASHPROG_OFFSET	0x4	/* Flash devices */
#define VEXPRESS_INTREG_OFFSET		0x8	/* Interrupt control */
#define VEXPRESS_DECODE_OFFSET		0xC	/* Fitted logic modules */

/*
 * System controller bit assignment
 */
#define VEXPRESS_REFCLK		0
#define VEXPRESS_TIMCLK		1

#define VEXPRESS_TIMER1_EnSel	15
#define VEXPRESS_TIMER2_EnSel	17
#define VEXPRESS_TIMER3_EnSel	19
#define VEXPRESS_TIMER4_EnSel	21


#define MAX_TIMER		2
#define MAX_PERIOD		699050
#define TICKS_PER_uSEC		1

/*
 *  These are useconds NOT ticks.
 *
 */
#define mSEC_1			1000
#define mSEC_5			(mSEC_1 * 5)
#define mSEC_10			(mSEC_1 * 10)
#define mSEC_25			(mSEC_1 * 25)
#define SEC_1			(mSEC_1 * 1000)

#define VEXPRESS_CSR_BASE	0x10000000
#define VEXPRESS_CSR_SIZE	0x10000000

/*
 * VEXPRESS_SELECTED_CLCD 1 = Tile, 0 = Motherboard
 */

#if defined(CONFIG_VEXPRESS_USE_TILE_CLCD)
  #define VEXPRESS_SELECTED_CLCD 1
#else
  #define VEXPRESS_SELECTED_CLCD 0
#endif

/*
 * Core tile IDs
 */
#define V2M_CT_ID_CA9		0x0c000191
#define V2M_CT_ID_CA15		0x0c000000 /* FIXME: this is almost certainly
					      wrong, but is what the model
					      currently provides. */
#define V2M_CT_ID_ELBA		0x0c000222
#define V2M_LT_ID_ELBA_FPGA	0x0c000217
#define V2M_CT_ID_UNSUPPORTED	0xff000191
#define V2M_CT_ID_MASK		0xff000fff

struct vexpress_tile_desc {
	u32			id;
	const char		*name;
	void			(*map_io)(void);
	void			(*init_irq)(void);
	void			(*init_timers)(void);
	void			(*init_tile)(void);
#ifdef CONFIG_SMP
	unsigned int		(*get_core_count)(void);
	void			(*smp_enable)(void);
#endif
};

/*
 * Peripheral addresses
 */
#define VEXPRESS_UART0_BASE		0x10009000	/* UART 0 */
#define VEXPRESS_UART1_BASE		0x1000A000	/* UART 1 */
#define VEXPRESS_UART2_BASE		0x1000B000	/* UART 2 */
#define VEXPRESS_UART3_BASE		0x1000C000	/* UART 3 */
#define VEXPRESS_SSP_BASE		0x1000D000	/* Synchronous Serial Port */
#define VEXPRESS_WATCHDOG0_BASE		0x1000F000	/* Watchdog 0 */
#define VEXPRESS_WATCHDOG_BASE		0x10010000	/* watchdog interface */
#define VEXPRESS_TIMER0_1_BASE		0x10011000	/* Timer 0 and 1 */
#define VEXPRESS_TIMER2_3_BASE		0x10012000	/* Timer 2 and 3 */
#define VEXPRESS_RTC_BASE		0x10017000	/* Real Time Clock */
#define VEXPRESS_TIMER4_5_BASE		0x10018000	/* Timer 4/5 */
#define VEXPRESS_TIMER6_7_BASE		0x10019000	/* Timer 6/7 */
#define VEXPRESS_CF_BASE		0x1001A000	/* CF Controller */
#define VEXPRESS_ONB_SRAM_BASE		0x10060000	/* On-board SRAM */
#define VEXPRESS_DMC_BASE		0x100E0000	/* DMC configuration */
#define VEXPRESS_SMC_BASE		0x100E1000	/* SMC configuration */
#define VEXPRESS_SYSREG_BASE		0x100E2000	/* System registers */

#define VEXPRESS_PCIE_TRN_CTRL_BASE	0xE0186000	/* PCI Express Addr Translation regs */
#define VEXPRESS_PCI_BASE		0xD0000000	/* PCI Express */
#define VEXPRESS_PCI_SIZE		SZ_256M		/* D0000000-DFFFFFFF */

#define VEXPRESS_FLASH0_BASE		0x40000000
#define VEXPRESS_FLASH0_SIZE		SZ_64M
#define VEXPRESS_FLASH1_BASE		0x44000000
#define VEXPRESS_FLASH1_SIZE		SZ_64M
#define VEXPRESS_ETH_BASE		0x4E000000	/* Ethernet */
#define VEXPRESS_USB_BASE		0x4F000000	/* USB */
#define VEXPRESS_SDRAM5_BASE		0x60000000	/* SDRAM bank 5 256MB */
#define VEXPRESS_SDRAM6_BASE		0x70000000	/* SDRAM bank 6 256MB */

#define VEXPRESS_SCU_BASE		0x1E000000      /* SCU registers */
#define VEXPRESS_CA9_GIC_CPU_BASE	0x1E000100      /* CA9 Private Generic interrupt controller CPU interface */
#define VEXPRESS_CA5_GIC_CPU_BASE	0x1E000000      /* CA5 Private Generic interrupt controller CPU interface */
#define VEXPRESS_TWD_BASE		0x1E000600
#define VEXPRESS_TWD_PERCPU_BASE	0x1E000700
#define VEXPRESS_TWD_SIZE		0x00000100
#define VEXPRESS_GIC_DIST_BASE		0x1E001000      /* Private Generic interrupt controller distributor */
#define VEXPRESS_L220_BASE		0x1E00A000      /* L220 registers */

#define VEXPRESS_SYS_PLD_CTRL1		0x74

/* PCI Express Address Translation registers
 * All are offsets from VEXPRESS_PCIE_TRN_CTRL_BASE
 */

#define VEXPRESS_TRN_TRANSLATION_ENABLE_RD_EN		0x000	/* Enable OB and IB address translation interface in PCIe RC core */

/* OB translation registers */
#define VEXPRESS_TRN_POM0_MEM_ADDR_OFFSET_RD_EN		0x004
#define VEXPRESS_TRN_POM1_MEM_ADDR_OFFSET_RD_EN		0x008
#define VEXPRESS_TRN_POM2_MEM_ADDR_OFFSET_RD_EN		0x00c
#define VEXPRESS_TRN_POM3_MEM_ADDR_OFFSET_RD_EN		0x010
#define VEXPRESS_TRN_POM4_MEM_ADDR_OFFSET_RD_EN		0x014
#define VEXPRESS_TRN_POM5_MEM_ADDR_OFFSET_RD_EN		0x018
#define VEXPRESS_TRN_POM6_MEM_ADDR_OFFSET_RD_EN		0x01c
#define VEXPRESS_TRN_POM7_MEM_ADDR_OFFSET_RD_EN		0x020
#define VEXPRESS_TRN_IN0_MEM_ADDR_START_RD_EN		0x024
#define VEXPRESS_TRN_IN1_MEM_ADDR_START_RD_EN		0x028
#define VEXPRESS_TRN_IN2_MEM_ADDR_START_RD_EN		0x02c
#define VEXPRESS_TRN_IN3_MEM_ADDR_START_RD_EN		0x030
#define VEXPRESS_TRN_IN4_MEM_ADDR_START_RD_EN		0x034
#define VEXPRESS_TRN_IN5_MEM_ADDR_START_RD_EN		0x038
#define VEXPRESS_TRN_IN6_MEM_ADDR_START_RD_EN		0x03c
#define VEXPRESS_TRN_IN7_MEM_ADDR_START_RD_EN		0x040
#define VEXPRESS_TRN_IN0_MEM_ADDR_LIMIT_RD_EN		0x044
#define VEXPRESS_TRN_IN1_MEM_ADDR_LIMIT_RD_EN		0x048
#define VEXPRESS_TRN_IN2_MEM_ADDR_LIMIT_RD_EN		0x04c
#define VEXPRESS_TRN_IN3_MEM_ADDR_LIMIT_RD_EN		0x050
#define VEXPRESS_TRN_IN4_MEM_ADDR_LIMIT_RD_EN		0x054
#define VEXPRESS_TRN_IN5_MEM_ADDR_LIMIT_RD_EN		0x058
#define VEXPRESS_TRN_IN6_MEM_ADDR_LIMIT_RD_EN		0x05c
#define VEXPRESS_TRN_IN7_MEM_ADDR_LIMIT_RD_EN		0x060
#define VEXPRESS_TRN_IN0_MEM_ADDR_MASK_RD_EN		0x064
#define VEXPRESS_TRN_IN1_MEM_ADDR_MASK_RD_EN		0x068
#define VEXPRESS_TRN_IN2_MEM_ADDR_MASK_RD_EN		0x06c
#define VEXPRESS_TRN_IN3_MEM_ADDR_MASK_RD_EN		0x070
#define VEXPRESS_TRN_IN4_MEM_ADDR_MASK_RD_EN		0x074
#define VEXPRESS_TRN_IN5_MEM_ADDR_MASK_RD_EN		0x078
#define VEXPRESS_TRN_IN6_MEM_ADDR_MASK_RD_EN		0x07c
#define VEXPRESS_TRN_IN7_MEM_ADDR_MASK_RD_EN		0x080
#define VEXPRESS_TRN_POM_IO_ADDR_OFFSET_RD_EN		0x084
#define VEXPRESS_TRN_IN_IO_ADDR_START_RD_EN		0x088
#define VEXPRESS_TRN_IN_IO_ADDR_LIMIT_RD_EN		0x090
#define VEXPRESS_TRN_IN_IO_ADDR_MASK_RD_EN		0x094
#define VEXPRESS_TRN_POM_CFG0_ADDR_OFFSET_RD_EN		0x098
#define VEXPRESS_TRN_IN_CFG0_ADDR_START_RD_EN		0x09c
#define VEXPRESS_TRN_IN_CFG0_ADDR_LIMIT_RD_EN		0x0a0
#define VEXPRESS_TRN_IN_CFG0_ADDR_MASK_RD_EN		0x0a4
#define VEXPRESS_TRN_POM_CFG1_ADDR_OFFSET_RD_EN		0x0a8
#define VEXPRESS_TRN_IN_CFG1_ADDR_START_RD_EN		0x0ac
#define VEXPRESS_TRN_IN_CFG1_ADDR_LIMIT_RD_EN		0x0b0
#define VEXPRESS_TRN_IN_CFG1_ADDR_MASK_RD_EN		0x0b4
#define VEXPRESS_TRN_IN_MSG_ADDR_START_RD_EN		0x0b8
#define VEXPRESS_TRN_IN_MSG_ADDR_LIMIT_RD_EN		0x0bc
#define VEXPRESS_TRN_IN_MEM0_TRANSLATED_FUNC_RD_EN	0x0c0
#define VEXPRESS_TRN_IN_MEM1_TRANSLATED_FUNC_RD_EN	0x0c4
#define VEXPRESS_TRN_IN_MEM2_TRANSLATED_FUNC_RD_EN	0x0c8
#define VEXPRESS_TRN_IN_MEM3_TRANSLATED_FUNC_RD_EN	0x0cc
#define VEXPRESS_TRN_IN_MEM4_TRANSLATED_FUNC_RD_EN	0x0d0
#define VEXPRESS_TRN_IN_MEM5_TRANSLATED_FUNC_RD_EN	0x0d4
#define VEXPRESS_TRN_IN_MEM6_TRANSLATED_FUNC_RD_EN	0x0d8
#define VEXPRESS_TRN_IN_MEM7_TRANSLATED_FUNC_RD_EN	0x0dc
#define VEXPRESS_TRN_PIM0_MEM_ADDR_START_RD_EN		0x0e0
#define VEXPRESS_TRN_PIM1_MEM_ADDR_START_RD_EN		0x0e4
#define VEXPRESS_TRN_PIM_IO_ADDR_START_RD_EN		0x0e8
#define VEXPRESS_TRN_PIM_ROM_ADDR_START_RD_EN		0x0ec
#define VEXPRESS_TRN_PIM0_MEM_ADDR_LIMIT_RD_EN		0x0f0
#define VEXPRESS_TRN_PIM1_MEM_ADDR_LIMIT_RD_EN		0x0f4


#define VEXPRESS_TRN_AMISCPCIE_SLV_CNTRL_RD_EN		0x100	/* [2:0],Enable AXI SLAVE sideband register address decoding (Only bit 0 used) */
#define VEXPRESS_TRN_APP_LTSSM_ENABLE_RD_EN		0x108	/* 1'b0,Enable the LTSSM */
#define VEXPRESS_TRN_BMISCPCIE_SLV_RD_EN		0x10c	/* [10:0],incoming PCIe AXI SLAVE sideband registers */
#define VEXPRESS_TRN_RMISCPCIE_SLV_RD_EN		0x110	/* [10:0],incoming PCIe AXI SLAVE sideband registers */
#define VEXPRESS_TRN_BMISCPCIE_MSTR_RD_EN		0x114	/* [12:0], PCIe AXI MASTER sideband registers */
#define VEXPRESS_TRN_RMISCPCIE_MSTR_RD_EN		0x118	/* [12:0], PCIe AXI MASTER sideband registers */
#define VEXPRESS_TRN_AWMISCPCIE_MSTR_RD_EN		0x11c	/* [23:0], PCIe AXI MASTER sideband registers */
#define VEXPRESS_TRN_ARMISCPCIE_MSTR_RD_EN		0x120	/* [23:0], PCIe AXI MASTER sideband registers */
#define VEXPRESS_TRN_SLV_RESP_ERR_MAP_RD_EN		0x124	/* [5:0],incoming Error response registers */
#define VEXPRESS_TRN_MSTR_RESP_ERR_MAP_RD_EN		0x128	/* [1:0],incoming Error response registers */

/* Registers written by PCIe devices to signal interrupts (MSIs) */
#define VEXPRESS_TRN_PCIE_MSI0_RD_EN			0x200	/* [31:0],MSI register read/write */
#define VEXPRESS_TRN_PCIE_MSI1_RD_EN			0x204	/* [31:0],MSI register read/write */
#define VEXPRESS_TRN_PCIE_MSI2_RD_EN			0x208	/* [31:0],MSI register read/write */
#define VEXPRESS_TRN_PCIE_MSI3_RD_EN			0x20c	/* [31:0],MSI register read/write */
#define VEXPRESS_TRN_PCIE_MSI0_WR_EN_CLR		0x210	/* [31:0],MSI register clear ( or with MSI register contents) */
#define VEXPRESS_TRN_PCIE_MSI1_WR_EN_CLR		0x214	/* [31:0],MSI register clear ( or with MSI register contents) */
#define VEXPRESS_TRN_PCIE_MSI2_WR_EN_CLR		0x218	/* [31:0],MSI register clear ( or with MSI register contents) */
#define VEXPRESS_TRN_PCIE_MSI3_WR_EN_CLR		0x21c	/* [31:0],MSI register clear ( or with MSI register contents) */

#define VEXPRESS_TRN_AWCACHEUSERPCIE_MSTR_RD_EN		0x300	/* [4:0],ACP AW channel cache bit drive and user bit drive */
#define VEXPRESS_TRN_ARCACHEUSERPCIE_MSTR_RD_EN		0x304	/* [4:0],ACP AR channel cache bit drive and user bit drive */

/* PCIe AXI SLAVE sideband registers */
#define VEXPRESS_TRN_AWMISCPCIE_SLV_0_RD_EN		0x400
#define VEXPRESS_TRN_AWMISCPCIE_SLV_1_RD_EN		0x404
#define VEXPRESS_TRN_AWMISCPCIE_SLV_2_RD_EN		0x408
#define VEXPRESS_TRN_AWMISCPCIE_SLV_3_RD_EN		0x40c
#define VEXPRESS_TRN_AWMISCPCIE_SLV_4_RD_EN		0x410
#define VEXPRESS_TRN_AWMISCPCIE_SLV_5_RD_EN		0x414
#define VEXPRESS_TRN_AWMISCPCIE_SLV_6_RD_EN		0x418

#define VEXPRESS_TRN_ARMISCPCIE_SLV_0_RD_EN		0x41c
#define VEXPRESS_TRN_ARMISCPCIE_SLV_1_RD_EN		0x420
#define VEXPRESS_TRN_ARMISCPCIE_SLV_2_RD_EN		0x424
#define VEXPRESS_TRN_ARMISCPCIE_SLV_3_RD_EN		0x428
#define VEXPRESS_TRN_ARMISCPCIE_SLV_4_RD_EN		0x42c
#define VEXPRESS_TRN_ARMISCPCIE_SLV_5_RD_EN		0x430
#define VEXPRESS_TRN_ARMISCPCIE_SLV_6_RD_EN		0x434

#define VEXPRESS_TRN_AMISCPCIE_SLV_BASE_0_RD_EN		0x440
#define VEXPRESS_TRN_AMISCPCIE_SLV_BASE_1_RD_EN		0x444
#define VEXPRESS_TRN_AMISCPCIE_SLV_BASE_2_RD_EN		0x448
#define VEXPRESS_TRN_AMISCPCIE_SLV_BASE_3_RD_EN		0x44c
#define VEXPRESS_TRN_AMISCPCIE_SLV_BASE_4_RD_EN		0x450
#define VEXPRESS_TRN_AMISCPCIE_SLV_BASE_5_RD_EN		0x454
#define VEXPRESS_TRN_AMISCPCIE_SLV_BASE_6_RD_EN		0x458

#define VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_0_RD_EN	0x460
#define VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_1_RD_EN	0x464
#define VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_2_RD_EN	0x468
#define VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_3_RD_EN	0x46c
#define VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_4_RD_EN	0x470
#define VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_5_RD_EN	0x474
#define VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_6_RD_EN	0x478

#define VEXPRESS_PCIE_MSI_INT_SET	\
	(VEXPRESS_PCIE_TRN_CTRL_BASE + VEXPRESS_TRN_PCIE_MSI0_RD_EN)	/* PCI MSI write to set bit and interrupt */
#define VEXPRESS_PCIE_MSI_INT_CLEAR	\
	(VEXPRESS_PCIE_TRN_CTRL_BASE + VEXPRESS_TRN_PCIE_MSI0_WR_EN_CLR)	/* PCI MSI write to clear bit */


#define VEXPRESS_MAX_TILES_SUPPORTED	2

extern struct vexpress_tile_desc *vexpress_tile_desc[VEXPRESS_MAX_TILES_SUPPORTED];

#endif	/* !__ASSEMBLY__ */
#endif

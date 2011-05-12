/*
 *  linux/arch/arm/mach-vexpress/pcie.c
 *
 *  Copyright (C) 2009 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#include <mach/core.h>

/* Driver configuration */
#define VEXPRESS_PCIE_DO_SWIZZLE	/* PCI bus interrupt swizzling */
#define VEXPRESS_PCIE_CHECK_SWITCH	/* define to setup bus regs & read switch id */
#define VEXPRESS_PCIE_CHECK_LINK_UP	/* polls for link up between RC and switch */

#ifdef VEXPRESS_PCIE_CHECK_LINK_UP
#define VEXPRESS_PCIE_CHECK_SWITCH	/* need bus numbers configured */
#endif


#define VEXPRESS_PCIE_MAX_READ_REQUEST_SIZE 128

/* PCIe bus numbers */
#define ROOT_BUS	0
#define LAST_BUS	255
#define SWITCH_BUS	(ROOT_BUS + 1)

#define ROOT_COMPLEX_ID 	0x072013b5
#define IDT_SWITCH_ID		0x8035111d


#define MAKE_BUS_NUMBER(pri,sec,sub) \
    (((pri) & 0xff) | (((sec) & 0xff) << 8) | (((sub) & 0xff) << 16))

#define TWRITE(v,reg) 	writel((v), __MMIO_P2V(VEXPRESS_PCIE_TRN_CTRL_BASE + (reg)))
#define TREAD(reg)	readl(__MMIO_P2V(VEXPRESS_PCIE_TRN_CTRL_BASE + (reg)))

#define PCI_VIRT_ADDR(a) ((a)-VEXPRESS_PCI_BASE+VEXPRESS_PCI_VBASE)
//#define PCI_VIRT_ADDR(addr) (__MMIO_P2V(addr))
#define IOWRITEL(val,addr) writel((val),__MMIO_P2V(addr))
#define IOREADL(addr) readl(__MMIO_P2V(addr))


/* PCIe space mapping - base and limit are obvious
 *	output_address = (input_address & mask) + offset
 */
struct s_trn {
	u32	base;
	u32	limit;
	u32	mask;
	u32	offset;

	u32	axi_base_reg;
	u32	axi_limit_reg;
	u32	axi_arslv_reg;
	u32	axi_awslv_reg;

	u32	ob_base_reg;
	u32	ob_limit_reg;
	u32	ob_mask_reg;
	u32	ob_offset_reg;
};

static const struct s_trn trn_map[5] = {
	{
		/* DBI */
		VEXPRESS_PCI_DBI_BASE, VEXPRESS_PCI_DBI_LIMIT, VEXPRESS_PCI_DBI_MASK, 0,
		VEXPRESS_TRN_AMISCPCIE_SLV_BASE_0_RD_EN, VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_0_RD_EN,
		    VEXPRESS_TRN_ARMISCPCIE_SLV_0_RD_EN, VEXPRESS_TRN_AWMISCPCIE_SLV_0_RD_EN,
		0,0,0,0		/* no OB regs */
	},
	{
		/* CFG0 - ob_offset_reg will be updated as needed */
		VEXPRESS_PCI_CFG0_BASE, VEXPRESS_PCI_CFG0_LIMIT, VEXPRESS_PCI_CFG0_MASK, 0,
		VEXPRESS_TRN_AMISCPCIE_SLV_BASE_1_RD_EN, VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_1_RD_EN,
			VEXPRESS_TRN_ARMISCPCIE_SLV_1_RD_EN, VEXPRESS_TRN_AWMISCPCIE_SLV_1_RD_EN,
		VEXPRESS_TRN_IN_CFG0_ADDR_START_RD_EN, VEXPRESS_TRN_IN_CFG0_ADDR_LIMIT_RD_EN,
		    VEXPRESS_TRN_IN_CFG0_ADDR_MASK_RD_EN, VEXPRESS_TRN_POM_CFG0_ADDR_OFFSET_RD_EN
	},
	{
		/* CFG1 - ob_offset_reg will be updated as needed */
		VEXPRESS_PCI_CFG1_BASE, VEXPRESS_PCI_CFG1_LIMIT, VEXPRESS_PCI_CFG1_MASK, 0,
		VEXPRESS_TRN_AMISCPCIE_SLV_BASE_2_RD_EN, VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_2_RD_EN,
		    VEXPRESS_TRN_ARMISCPCIE_SLV_2_RD_EN, VEXPRESS_TRN_AWMISCPCIE_SLV_2_RD_EN,
		VEXPRESS_TRN_IN_CFG1_ADDR_START_RD_EN, VEXPRESS_TRN_IN_CFG1_ADDR_LIMIT_RD_EN,
		    VEXPRESS_TRN_IN_CFG1_ADDR_MASK_RD_EN, VEXPRESS_TRN_POM_CFG1_ADDR_OFFSET_RD_EN
	},
	{
		/* I/O */
		VEXPRESS_PCI_IO_BASE, VEXPRESS_PCI_IO_LIMIT, VEXPRESS_PCI_IO_MASK, 0,
		VEXPRESS_TRN_AMISCPCIE_SLV_BASE_3_RD_EN, VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_3_RD_EN,
		    VEXPRESS_TRN_ARMISCPCIE_SLV_3_RD_EN, VEXPRESS_TRN_AWMISCPCIE_SLV_3_RD_EN,
		VEXPRESS_TRN_IN_IO_ADDR_START_RD_EN, VEXPRESS_TRN_IN_IO_ADDR_LIMIT_RD_EN,
		    VEXPRESS_TRN_IN_IO_ADDR_MASK_RD_EN, VEXPRESS_TRN_POM_IO_ADDR_OFFSET_RD_EN
	},
	{
		/* Memory */
		VEXPRESS_PCI_MEM_BASE, VEXPRESS_PCI_MEM_LIMIT, VEXPRESS_PCI_MEM_MASK, 0,
		VEXPRESS_TRN_AMISCPCIE_SLV_BASE_4_RD_EN, VEXPRESS_TRN_AMISCPCIE_SLV_LIMIT_4_RD_EN,
		    VEXPRESS_TRN_ARMISCPCIE_SLV_4_RD_EN, VEXPRESS_TRN_AWMISCPCIE_SLV_4_RD_EN,
		VEXPRESS_TRN_IN0_MEM_ADDR_START_RD_EN, VEXPRESS_TRN_IN0_MEM_ADDR_LIMIT_RD_EN,
		    VEXPRESS_TRN_IN0_MEM_ADDR_MASK_RD_EN, VEXPRESS_TRN_POM0_MEM_ADDR_OFFSET_RD_EN
	},
};


enum {
	TI_DBI = 0,
	TI_CFG0,
	TI_CFG1,
	TI_IO,
	TI_MEM
};


static volatile int pci_abort;


/* 
 * Maps a legacy interrupt pin to an interrupt number.
 *
 * Returns interrupt number or -1 for error.
 */
static int vexpress_pci_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq = -1;
	int devslot = PCI_SLOT(dev->devfn);

	/* check pin in range INTA..INTD */
	if (pin >= 1 && pin <= 4) {
		/* 
		 * Pins 1..4 are mapped from IRQ_VEXPRESS_PCIE_A to IRQ_VEXPRESS_PCIE_D
		 */
		irq = IRQ_VEXPRESS_PCIE_A + pin - 1;
	}

	printk(KERN_INFO
	       "PCIe map irq: device: 0x%x, slot %d, pin %d, devslot %d, "
	       "irq: %d\n", dev->device, slot, pin, devslot, irq);

	return irq;
}


/* Calculate address for required access and program the OB translation block,
 * as necessary. There is no need to protect this with a spinlock since the
 * high-level PCI code ensures that we are never reentered.
 *
 * Returns 0, if successful.
 */
static int setup_config(u32 bus, u32 slot, u32 function,
			    int word_offset, u32 *paddr)
{
	u32 addr;
	u32 offset_reg;

	offset_reg = (bus << 24) | ((slot & 0x1f) << 19)
					| ((function & 7) << 16);

	//printk("setup_config for %u %u %u %#x\n", bus, slot, function, word_offset);
	switch (bus) {
	case ROOT_BUS:
		addr = trn_map[TI_DBI].base;
		break;
	case SWITCH_BUS:
		addr = trn_map[TI_CFG0].base;
		TWRITE(offset_reg, trn_map[TI_CFG0].ob_offset_reg);
		break;
	default:
		if (bus < ROOT_BUS || bus > LAST_BUS)
		    return -1;
		addr = trn_map[TI_CFG1].base;
		TWRITE(offset_reg, trn_map[TI_CFG1].ob_offset_reg);
		break;
	}

	addr |= (word_offset & 0xffc);
	addr = PCI_VIRT_ADDR(addr);
	*paddr = addr;
	//printk("setup_config returns %#x\n", addr);

	return 0;
}


/* Reads PCI configuration space and may fake a return value where necessary.
 * Configuration space accesses will be performed with interrupts disabled
 * because the PCIe space mapping has to change.
 *
 * Returns PCIBIOS_SUCCESSFUL if successful, data returned via pointer passed
 */
int vexpress_pci_read_config(struct pci_bus *pbus, u32 devfn,
			int offset, int size, u32 *pdata)
{
	u32 addr, bus, slot, function;

	bus = pbus->number;
	slot = PCI_SLOT(devfn);
	function = PCI_FUNC(devfn);

	/* ignore accesses to non-existent ports */
	if ((bus == SWITCH_BUS && (((slot > 0) && (slot < 4)) || (slot > 7)))
			|| (bus == ROOT_BUS && slot > 0)) {
		/* only slots 0..7 on the switch are connected (and not all of
		 * those)
		 */

		*pdata = ((size == 1) ? 0xFF
				      : ((size == 2) ? 0xFFFF : 0xFFFFFFFF));
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* trap access to the class code reg because the RC has junk */
	if (bus == ROOT_BUS && offset == PCI_CLASS_REVISION && size == 4) {

		*pdata = 0x06040001;	/* Bridge/PCI-PCI/rev 1 */
		return PCIBIOS_SUCCESSFUL;
	}

	if (setup_config(bus, slot, function, offset, &addr) < 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

		pci_abort = 0;
		switch (size) {
		case 1:
			*pdata = (readl(addr) >> (8 * (offset & 3))) & 0xff;
			break;
		case 2:
			*pdata = (u32)readw(addr + (offset & 3));
			break;
		default:
			*pdata = readl(addr);
			break;
		}

		//printk("read_config %u, %u, %u, %#x (%#x) -> %#x\n", bus, slot, function, offset, addr, *pdata);
		if (!pci_abort)
			return PCIBIOS_SUCCESSFUL;

	*pdata = ((size == 1) ? 0xFF : ((size == 2) ? 0xFFFF : 0xFFFFFFFF));
	return PCIBIOS_DEVICE_NOT_FOUND;
}


/* Writes PCI configuration space. Configuration space accesses will be
 * performed with interrupts disabled because the PCIe space mapping has to
 * change.
 *
 * Returns PCIBIOS_SUCCESSFUL if successful.
 */
int vexpress_pci_write_config(struct pci_bus *pbus, u32 devfn,
			int offset, int size, u32 data)
{
	u32 addr, bus, slot, function;

	bus = pbus->number;
	slot = PCI_SLOT(devfn);
	function = PCI_FUNC(devfn);

	if ((bus == SWITCH_BUS && (((slot > 0) && (slot < 4)) || (slot > 7)))
			|| (bus == ROOT_BUS && slot > 0)) {
		/* only slots 0..7 on switch are connected (and not all of
		 * those)
		 */
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (setup_config(bus, slot, function, offset, &addr) < 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	//printk("write_config %u, %u, %u, %#x (%#x) = %#x\n", bus, slot, function, offset, addr, data);
	pci_abort = 0;
	switch (size) {
	case 1:
		writeb(data, addr + (offset & 3));
		break;
	case 2:
		writew(data, addr + (offset & 3));
		break;
	default:
		writel(data, addr);
		break;
	}

	if (!pci_abort)
		return PCIBIOS_SUCCESSFUL;

	return PCIBIOS_DEVICE_NOT_FOUND;
}


static struct pci_ops vexpress_pci_ops =
{
	.read	= vexpress_pci_read_config,
	.write	= vexpress_pci_write_config,
};


/* Walk the PCIe/PCI bus structure
 * Although the Root Complex returns the wrong class code, we can still scan it
 * because read_config traps accesses to the class code reg to return something
 * useful. Otherwise, we'd have to start at the switch.
 */
struct pci_bus *vexpress_pci_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *root_bus;

	sys->busnr = ROOT_BUS;

	root_bus = pci_scan_bus(sys->busnr, &vexpress_pci_ops, sys);
	pci_assign_unassigned_resources();

	return root_bus;
}


/* Handle aborts caused while we were probing the PCIe space.
 */
int vexpress_pci_abort_handler(unsigned long addr, unsigned int fsr,
				struct pt_regs *regs)
{
	/* check abort type - this can be removed if we're trapping only one
	 * type of abort
	 */
	if ((fsr & 0x40f) == 0x406) {
		/* async external abort - too late to adjust PC */
		;
	} else {
		/* step over faulting instruction */
		regs->ARM_pc += 4;

		/* flag the abort */
		pci_abort = 1;
	}

	return 0;
}


/* Pre-initialisation
 */
void __init vexpress_pci_preinit(void)
{
	/* hardware raises abort on access to non-existent device, instead of
	 * returning all Fs. We trap that abort, even if we can't do anything
	 * with it, because the kernel will complain otherwise.
	 */

	/* sync external abort - this is the one that is in DFSR when no MMU */
	hook_fault_code(8, vexpress_pci_abort_handler, SIGBUS, 0,
				"PCI config cycle to non-existent device");

	/* pcibios_setup("debug"); XXX */
}


/* Main setup function.
 * Initialises the PCIe hardware and looks for the Root Complex.
 *
 * Returns:
 *    < 0 for error
 *    > 0 for good exit
 *      0 no error but nothing found?
 */
int __init vexpress_pci_setup(int nr, struct pci_sys_data *sys)
{
	u32 data;
	struct pci_bus bus;
#ifdef VEXPRESS_PCIE_CHECK_LINK_UP
	int i;
#endif

	/* Make sure the AXI and Outbound Translation blocks are disabled */
	/* Disable AXI translation block */
	TWRITE(0, VEXPRESS_TRN_AMISCPCIE_SLV_CNTRL_RD_EN);
	/* Disable OB translation block */
	TWRITE(0, VEXPRESS_TRN_TRANSLATION_ENABLE_RD_EN);


	/* Configure DBI - note, configure AXI translation block only since it
	 * never gets through to the OB translation block
	 */
	TWRITE(trn_map[TI_DBI].base, trn_map[TI_DBI].axi_base_reg);
	TWRITE(trn_map[TI_DBI].limit, trn_map[TI_DBI].axi_limit_reg);
	TWRITE(0x00200000, trn_map[TI_DBI].axi_arslv_reg);
	TWRITE(0x00200000, trn_map[TI_DBI].axi_awslv_reg);


	/* Configure configuration 0 space - AXI translation block */
	TWRITE(trn_map[TI_CFG0].base, trn_map[TI_CFG0].axi_base_reg);
	TWRITE(trn_map[TI_CFG0].limit, trn_map[TI_CFG0].axi_limit_reg);
	TWRITE(4, trn_map[TI_CFG0].axi_arslv_reg);
	TWRITE(4, trn_map[TI_CFG0].axi_awslv_reg);

	/* Configure configuration 0 space - OB translation block */
	TWRITE(trn_map[TI_CFG0].base, trn_map[TI_CFG0].ob_base_reg);
	TWRITE(trn_map[TI_CFG0].limit, trn_map[TI_CFG0].ob_limit_reg);
	TWRITE(trn_map[TI_CFG0].mask, trn_map[TI_CFG0].ob_mask_reg);
	TWRITE(trn_map[TI_CFG0].offset, trn_map[TI_CFG0].ob_offset_reg);


	/* Configure configuration 1 space - AXI translation block */
	TWRITE(trn_map[TI_CFG1].base, trn_map[TI_CFG1].axi_base_reg);
	TWRITE(trn_map[TI_CFG1].limit, trn_map[TI_CFG1].axi_limit_reg);
	TWRITE(5, trn_map[TI_CFG1].axi_arslv_reg);
	TWRITE(5, trn_map[TI_CFG1].axi_awslv_reg);

	/* Configure configuration 1 space - OB translation block */
	TWRITE(trn_map[TI_CFG1].base, trn_map[TI_CFG1].ob_base_reg);
	TWRITE(trn_map[TI_CFG1].limit, trn_map[TI_CFG1].ob_limit_reg);
	TWRITE(trn_map[TI_CFG1].mask, trn_map[TI_CFG1].ob_mask_reg);
	TWRITE(trn_map[TI_CFG1].offset, trn_map[TI_CFG1].ob_offset_reg);


	/* Configure I/O space - AXI translation block */
	TWRITE(trn_map[TI_IO].base, trn_map[TI_IO].axi_base_reg);
	TWRITE(trn_map[TI_IO].limit, trn_map[TI_IO].axi_limit_reg);
	TWRITE(2, trn_map[TI_IO].axi_arslv_reg);
	TWRITE(2, trn_map[TI_IO].axi_awslv_reg);

	/* Configure I/O space - OB translation block */
	TWRITE(trn_map[TI_IO].base, trn_map[TI_IO].ob_base_reg);
	TWRITE(trn_map[TI_IO].limit, trn_map[TI_IO].ob_limit_reg);
	TWRITE(trn_map[TI_IO].mask, trn_map[TI_IO].ob_mask_reg);
	TWRITE(trn_map[TI_IO].offset, trn_map[TI_IO].ob_offset_reg);


	/* Configure memory space - AXI translation block */
	TWRITE(trn_map[TI_MEM].base, trn_map[TI_MEM].axi_base_reg);
	TWRITE(trn_map[TI_MEM].limit, trn_map[TI_MEM].axi_limit_reg);
	TWRITE(0, trn_map[TI_MEM].axi_arslv_reg);
	TWRITE(0, trn_map[TI_MEM].axi_awslv_reg);

	/* Configure memory space - OB translation block */
	TWRITE(trn_map[TI_MEM].base, trn_map[TI_MEM].ob_base_reg);
	TWRITE(trn_map[TI_MEM].limit, trn_map[TI_MEM].ob_limit_reg);
	TWRITE(trn_map[TI_MEM].mask, trn_map[TI_MEM].ob_mask_reg);
	TWRITE(trn_map[TI_MEM].offset, trn_map[TI_MEM].ob_offset_reg);


	/* Enable AXI translation block */
	TWRITE(1, VEXPRESS_TRN_AMISCPCIE_SLV_CNTRL_RD_EN);

	/* Enable OB translation block */
	TWRITE(1, VEXPRESS_TRN_TRANSLATION_ENABLE_RD_EN);

	bus.number = ROOT_BUS;	/* so can use read/write_config */

	/* look for the Root Complex */
	if ((vexpress_pci_read_config(&bus, 0, PCI_VENDOR_ID, 4, &data)
		    != PCIBIOS_SUCCESSFUL)
		    || (data != ROOT_COMPLEX_ID)) {
		printk(KERN_ERR "PCIe can't find root complex (%#x)\n", data);
		return -1;
	}

	/* set bus numbers - only one RC so all buses below this */
	if (vexpress_pci_read_config(&bus, 0, PCI_PRIMARY_BUS, 4, &data)
		    != PCIBIOS_SUCCESSFUL) {
		printk(KERN_ERR "PCIe can't read bus numbers\n");
		return -1;
	}

#ifdef VEXPRESS_PCIE_CHECK_SWITCH
	/* write bus numbers to Root Complex */
	data = (data & 0xff000000)
		    | MAKE_BUS_NUMBER(ROOT_BUS, SWITCH_BUS, 255);
	if (vexpress_pci_write_config(&bus, 0, PCI_PRIMARY_BUS, 4, data)
		    != PCIBIOS_SUCCESSFUL) {
		printk(KERN_ERR "PCIe can't set RC bus numbers\n");
		return -1;
	}
#endif

	/* Only enable the Link Training & Status State Machine (LTSSM) if not  */
	/* already done by the boot firmware */
	data = TREAD(VEXPRESS_TRN_APP_LTSSM_ENABLE_RD_EN);
	if (data == 0) {
		/* set port link control register
		 * Disable fast link and enable scramble
		 * b1==0 scramble enabled
		 * b5==1 link enabled
		 * b7==0 fast link disabled (reserved for simulation)
		 * b8==1 required
		 * 21:16 = number of lanes (encoded)
		 */
		if (vexpress_pci_write_config(&bus, 0, PCI_PORT_LINK_CONTROL, 4, 0x00070120)
			    != PCIBIOS_SUCCESSFUL) {
			printk(KERN_ERR "PCIe can't write link control register\n");
			return -1;
		}

		/* Enable the Link Training & Status State Machine (LTSSM) */
		TWRITE(1, VEXPRESS_TRN_APP_LTSSM_ENABLE_RD_EN);

#ifdef VEXPRESS_PCIE_CHECK_LINK_UP
		/* poll for link up */
		for (i = 0; i < 100; i++) {
			if (vexpress_pci_read_config(&bus, 0, PCI_RC_DEBUGREG1, 4, &data)
				    != PCIBIOS_SUCCESSFUL) {
				printk(KERN_ERR "PCIe: can't read DEBUGREG1\n");
				return -1;
			}
			if (data & 0x10)
				break;
			msleep(1);
		}

		/* check for link up */
		if ((data & 0x10) == 0) {
			printk(KERN_ERR "PCIe: RC->switch link not up\n");
			return -1;
		}
#endif
	}


#ifdef VEXPRESS_PCIE_CHECK_SWITCH
	/* switch should now be up - look for it */
	bus.number = SWITCH_BUS;
	if (vexpress_pci_read_config(&bus, 0, PCI_VENDOR_ID, 4, &data)
		    != PCIBIOS_SUCCESSFUL) {
		printk(KERN_ERR "PCIe can't read switch id\n");
		return -1;
	}
	if (data != IDT_SWITCH_ID) {
		printk(KERN_ERR "PCIe can't find IDT switch\n");
		return -1;
	}
#endif


	/* set bus->cpu mapping */
	sys->mem_offset = 0;
	sys->io_offset = 0;

	/* Note: we don't reserve the addresses for PCI space here because the
	 * window allocator won't be able to get any if we do.
	 */

	return 1;
}


static struct hw_pci vexpress_pci __initdata =
{
	.nr_controllers = 1,
	.setup		= vexpress_pci_setup,
	.preinit	= vexpress_pci_preinit,
	.scan		= vexpress_pci_scan_bus,
	.map_irq	= vexpress_pci_map_irq,
#ifdef VEXPRESS_PCIE_DO_SWIZZLE
	.swizzle	= pci_std_swizzle,
#else
	.swizzle	= NULL,
#endif
};

static void pcie_aer_enable(struct pci_dev *pdev)
{
	u16 config;
	u32 dconfig;

	/*
	 * Enable the PCIe normal error reporting
	 */
	pci_read_config_word(pdev, pdev->pcie_cap + PCI_EXP_DEVCTL, &config);
	config = config |
		PCI_EXP_DEVCTL_CERE |
		PCI_EXP_DEVCTL_NFERE |
		PCI_EXP_DEVCTL_FERE |
		PCI_EXP_DEVCTL_URRE;
	pci_write_config_word(pdev, pdev->pcie_cap + PCI_EXP_DEVCTL, config);

	/* Find the Advanced Error Reporting capability */
	/* Clear Uncorrectable Error Status */
	pci_read_config_dword(pdev, pdev->pcie_cap + PCI_ERR_UNCOR_STATUS,
			&dconfig);
	pci_write_config_dword(pdev, pdev->pcie_cap + PCI_ERR_UNCOR_STATUS,
			dconfig);
	/* Enable reporting of all uncorrectable errors */
	/* Uncorrectable Error Mask - turned on bits disable errors */
	pci_write_config_dword(pdev, pdev->pcie_cap + PCI_ERR_UNCOR_MASK, 0);
	/*
	 * Leave severity at HW default. This only controls if
	 * errors are reported as uncorrectable or
	 * correctable, not if the error is reported.
	 */
	/* PCI_ERR_UNCOR_SEVER - Uncorrectable Error Severity */
	/* Clear Correctable Error Status */
	pci_read_config_dword(pdev, pdev->pcie_cap + PCI_ERR_COR_STATUS, &dconfig);
	pci_write_config_dword(pdev, pdev->pcie_cap + PCI_ERR_COR_STATUS, dconfig);
	/* Enable reporting of all correctable errors */
	/* Correctable Error Mask - turned on bits disable errors */
	pci_write_config_dword(pdev, pdev->pcie_cap + PCI_ERR_COR_MASK, 0);
	/* Advanced Error Capabilities */
	pci_read_config_dword(pdev, pdev->pcie_cap + PCI_ERR_CAP, &dconfig);
	/* ECRC Generation Enable */
	if (config & PCI_ERR_CAP_ECRC_GENC)
		config |= PCI_ERR_CAP_ECRC_GENE;
	/* ECRC Check Enable */
	if (config & PCI_ERR_CAP_ECRC_CHKC)
		config |= PCI_ERR_CAP_ECRC_CHKE;
	pci_write_config_dword(pdev, pdev->pcie_cap + PCI_ERR_CAP, dconfig);
	/* PCI_ERR_HEADER_LOG - Header Log Register (16 bytes) */
	/* Report all errors to the root complex */
	pci_write_config_dword(pdev, pdev->pcie_cap + PCI_ERR_ROOT_COMMAND,
			PCI_ERR_ROOT_CMD_COR_EN |
			PCI_ERR_ROOT_CMD_NONFATAL_EN |
			PCI_ERR_ROOT_CMD_FATAL_EN);
	/* Clear the Root status register */
	pci_read_config_dword(pdev, pdev->pcie_cap + PCI_ERR_ROOT_STATUS, &dconfig);
	pci_write_config_dword(pdev, pdev->pcie_cap + PCI_ERR_ROOT_STATUS, dconfig);
}

static void pcie_fix_sizes(void)
{
	struct pci_dev *pdev = NULL;
	u16 val16, v;

	while ((pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pdev)) != NULL) {
		pci_read_config_word(pdev, pdev->pcie_cap + PCI_EXP_DEVCTL, &val16);
		v = pcie_get_readrq(pdev);
		printk(KERN_ERR "%s: %02x%02x\t%04x : (MPSZ, RRQSZ): (%d, %d) -> (%d, %d)\n",
				__FUNCTION__,
				pdev->bus->number,
				pdev->devfn,
				pdev->vendor,
				((((val16 & PCI_EXP_DEVCTL_PAYLOAD) >> 5) + 1) << 7),
				v, VEXPRESS_PCIE_MAX_READ_REQUEST_SIZE,
				VEXPRESS_PCIE_MAX_READ_REQUEST_SIZE);

		val16 &= ~PCI_EXP_DEVCTL_PAYLOAD;
		pci_write_config_word(pdev, pdev->pcie_cap + PCI_EXP_DEVCTL, val16);
		pcie_set_readrq(pdev, VEXPRESS_PCIE_MAX_READ_REQUEST_SIZE);
		pcie_aer_enable(pdev);

		/* Enable PME feature.  */
		device_set_run_wake(&pdev->dev, true);
		pdev->pme_interrupt = true;
	}
}

#ifdef CONFIG_VEXPRESS_ELBA_ACP
void pcie_acp_init(void __iomem *trn_base)
{
	writel(0x01, trn_base + 0x300);
	writel(0x01, trn_base + 0x304);
}
#else
void pcie_acp_init(void __iomem *trn_base) {}
#endif

int __init vexpress_pcie_fixups(void)
{
	pcie_fix_sizes();
	return 0;
}
late_initcall(vexpress_pcie_fixups);

int __init vexpress_pci_init(void)
{
	pci_common_init(&vexpress_pci);
	return 0;
}
subsys_initcall(vexpress_pci_init);

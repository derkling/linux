/*
 * Copyright (C) 2008-2010 ARM Limited
 *
 * This software is provided 'as-is', without any express or implied
 * warranties including the implied warranties of satisfactory quality,
 * fitness for purpose or non infringement.  In no event will  ARM be
 * liable for any damages arising from the use of this software.
 *
 * Permission is granted to anyone to use, copy and modify this software for
 * any purpose, and to redistribute the software, subject to the following
 * restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/percpu.h>
#include <linux/mm_types.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>
#include <asm/sections.h>
#include <asm/processor.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include "appf_internals.h"
#include "appf_helpers.h"

#define PROT_PTE_DEVICE		(L_PTE_PRESENT|L_PTE_YOUNG|L_PTE_DIRTY|L_PTE_XN)

static pgd_t *pgd;

static void *linux_appf_map_page(void *addr, unsigned int size,
				 pgprot_t prot)
{
	pmd_t *pmd;
	pte_t *pte;
	u64 pfn;
	unsigned long end = (unsigned long) (addr) + size;
	unsigned long vaddr = (unsigned long) (addr);

	pmd = pmd_offset(pgd + pgd_index(vaddr), vaddr);
	pfn = vaddr >> PAGE_SHIFT;
	pte = pte_alloc_kernel(pmd, vaddr);

	do {
		if (!pte)
			return NULL;
		set_pte_ext(pte, pfn_pte(pfn, prot), 0);
		pfn++;
	} while (pte++, vaddr += PAGE_SIZE, vaddr != end);

	return addr;
}

int linux_appf_setup_translation_tables(void)
{
	pgd = pgd_alloc(&init_mm);

	if (!pgd)
		return -ENOMEM;

	identity_mapping_add(pgd, __pa(_stext), __pa(_etext));
	/*
	 * data 1:1 identity table does not seem to be working
	 * when Linux kernel calls are made within 1:1 flat area
	 */
	identity_mapping_add(pgd, __pa(_sdata), __pa(_edata));
	identity_mapping_add(pgd, __pa(__bss_start), __pa(__bss_stop));

	linux_appf_map_page((void *) appf_device_memory_flat_mapped,
			    PAGE_SIZE, pgprot_dmacoherent(pgprot_kernel));
	linux_appf_map_page((void *) read_cbar(), 2 * PAGE_SIZE,
			    __pgprot(PROT_PTE_DEVICE | L_PTE_MT_DEV_SHARED
				     | L_PTE_SHARED));
	linux_appf_map_page(context_memory_uncached,
			    CONTEXT_SPACE_UNCACHED,
			    pgprot_dmacoherent(pgprot_kernel));
#ifdef CONFIG_ARCH_VEXPRESS_LT_ELBA
	linux_appf_map_page((void *) read_cbar() + 0x2000, PAGE_SIZE,
			    __pgprot(PROT_PTE_DEVICE | L_PTE_MT_DEV_SHARED
				     | L_PTE_SHARED));
#else
	linux_appf_map_page((void *) read_cbar() + 0xa000, PAGE_SIZE,
			    __pgprot(PROT_PTE_DEVICE | L_PTE_MT_DEV_SHARED
				     | L_PTE_SHARED));
#endif
	/* Setup values for TTBR0 and TTBCR */
	/* TODO: Add suitable values for non-A9 CPUs */

	/* Calculate physical address of translation tables */
	main_table.fw_mmu_context[1] = virt_to_phys(pgd);

	/* Add appropriate cacheable/shareable attributes */
	/* Table walks are Inner/Outer cacheable WBWA */
	if (num_online_cpus())
		main_table.fw_mmu_context[1] |= TTB_FLAGS_SMP;
	else
		main_table.fw_mmu_context[1] |= TTB_FLAGS_UP;

	main_table.fw_mmu_context[0] = 0xffffffff;	/* DACR = 0xffffffff */
	main_table.fw_mmu_context[2] = 0x0;	/* TTBR1 = 0 */
	main_table.fw_mmu_context[3] = 0x0;	/* Always use TTBR0 */

	return 0;
}

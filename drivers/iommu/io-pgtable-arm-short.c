/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Yong Wu <yong.wu@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"arm-short-desc io-pgtable: "fmt

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/errno.h>

#include "io-pgtable.h"

typedef u32 arm_short_iopte;

struct arm_short_io_pgtable {
	struct io_pgtable	iop;
	struct kmem_cache	*ptekmem;
	size_t			pgd_size;
	void			*pgd;
};

#define io_pgtable_short_to_data(x)				\
	container_of((x), struct arm_short_io_pgtable, iop)

#define io_pgtable_ops_to_pgtable(x)				\
	container_of((x), struct io_pgtable, ops)

#define io_pgtable_short_ops_to_data(x)				\
	io_pgtable_short_to_data(io_pgtable_ops_to_pgtable(x))

#define ARM_SHORT_PGDIR_SHIFT			20
#define ARM_SHORT_PAGE_SHIFT			12
#define ARM_SHORT_PTRS_PER_PTE			256
#define ARM_SHORT_BYTES_PER_PTE			\
	(ARM_SHORT_PTRS_PER_PTE * sizeof(arm_short_iopte))

/* level 1 pagetable */
#define ARM_SHORT_PGD_TYPE_PGTABLE		BIT(0)
#define ARM_SHORT_PGD_TYPE_SECTION		BIT(1)
#define ARM_SHORT_PGD_B_BIT			BIT(2)
#define ARM_SHORT_PGD_C_BIT			BIT(3)
#define ARM_SHORT_PGD_NS_BIT_PAGE		BIT(3)
#define ARM_SHORT_PGD_IMPLE_BIT			BIT(9)
#define ARM_SHORT_PGD_TEX0_BIT			BIT(12)
#define ARM_SHORT_PGD_S_BIT			BIT(16)
#define ARM_SHORT_PGD_NG_BIT			BIT(17)
#define ARM_SHORT_PGD_SUPERSECTION_BIT		BIT(18)
#define ARM_SHORT_PGD_NS_BIT_SECTION		BIT(19)

#define ARM_SHORT_PGD_TYPE_SUPERSECTION		\
	(ARM_SHORT_PGD_TYPE_SECTION | ARM_SHORT_PGD_SUPERSECTION_BIT)
#define ARM_SHORT_PGD_PGTABLE_MSK		(0x3)
#define ARM_SHORT_PGD_SECTION_MSK		\
	(ARM_SHORT_PGD_PGTABLE_MSK | ARM_SHORT_PGD_SUPERSECTION_BIT)
#define ARM_SHORT_PGD_TYPE_IS_PGTABLE(pgd)	\
	(((pgd) & ARM_SHORT_PGD_PGTABLE_MSK) == ARM_SHORT_PGD_TYPE_PGTABLE)
#define ARM_SHORT_PGD_TYPE_IS_SECTION(pgd)	\
	(((pgd) & ARM_SHORT_PGD_SECTION_MSK) == ARM_SHORT_PGD_TYPE_SECTION)
#define ARM_SHORT_PGD_TYPE_IS_SUPERSECTION(pgd)	\
	(((pgd) & ARM_SHORT_PGD_SECTION_MSK) == ARM_SHORT_PGD_TYPE_SUPERSECTION)
#define ARM_SHORT_PGD_PA_PGTABLE_MSK		0xfffffc00
#define ARM_SHORT_PGD_PA_SECTION_MSK		0xfff00000
#define ARM_SHORT_PGD_PA_SUPERSECTION_MSK	0xff000000

/* level 2 pagetable */
#define ARM_SHORT_PTE_TYPE_LARGE		BIT(0)
#define ARM_SHORT_PTE_TYPE_SMALL		BIT(1)
#define ARM_SHORT_PTE_B_BIT			BIT(2)
#define ARM_SHORT_PTE_C_BIT			BIT(3)
#define ARM_SHORT_PTE_TEX0_BIT			BIT(6)
#define ARM_SHORT_PTE_IMPLE_BIT			BIT(9)
#define ARM_SHORT_PTE_S_BIT			BIT(10)
#define ARM_SHORT_PTE_PA_LARGE_MSK		0xffff0000
#define ARM_SHORT_PTE_PA_SMALL_MSK		0xfffff000
#define ARM_SHORT_PTE_TYPE_MSK			(0x3)
#define ARM_SHORT_PTE_TYPE_IS_SMALLPAGE(pte)	\
	(((pte) & ARM_SHORT_PTE_TYPE_MSK) == ARM_SHORT_PTE_TYPE_SMALL)
#define ARM_SHORT_PTE_TYPE_IS_LARGEPAGE(pte)	\
	(((pte) & ARM_SHORT_PTE_TYPE_MSK) == ARM_SHORT_PTE_TYPE_LARGE)

#define ARM_SHORT_PGD_IDX(a)			((a) >> ARM_SHORT_PGDIR_SHIFT)
#define ARM_SHORT_PTE_IDX(a)			\
	(((a) >> ARM_SHORT_PAGE_SHIFT) & (ARM_SHORT_PTRS_PER_PTE - 1))
#define ARM_SHORT_GET_PTE_VA(pgd)		\
	(phys_to_virt((unsigned long)pgd & ARM_SHORT_PGD_PA_PGTABLE_MSK))

#define ARM_SHORT_PTE_GET_PROT(pte)		((arm_short_iopte)pte & 0x7ff)
#define ARM_SHORT_PGD_GET_PROT(pte)		((arm_short_iopte)pte & 0xfffff)

static arm_short_iopte *
arm_short_get_pte_in_pgd(arm_short_iopte pgd, unsigned int iova)
{
	arm_short_iopte *pte;

	pte = ARM_SHORT_GET_PTE_VA(pgd);
	pte += ARM_SHORT_PTE_IDX(iova);
	return pte;
}

static bool arm_short_free_wholepte(struct arm_short_io_pgtable *data,
				    arm_short_iopte *pgd)
{
	arm_short_iopte *pte;
	int i;

	pte = ARM_SHORT_GET_PTE_VA(*pgd);

	for (i = 0; i < ARM_SHORT_PTRS_PER_PTE; i++) {
		if (pte[i] != 0)
			return false;
	}

	/* Free whole pte while all pte is unmap */
	kmem_cache_free(data->ptekmem, pte);
	*pgd = 0;

	return true;
}

static arm_short_iopte __arm_short_pte_prot(unsigned int prot, bool large)
{
	arm_short_iopte pteprot;

	pteprot = ARM_SHORT_PTE_S_BIT;
	pteprot |= large ? ARM_SHORT_PTE_TYPE_LARGE :
				ARM_SHORT_PTE_TYPE_SMALL;
	if (prot & IOMMU_CACHE)
		pteprot |=  ARM_SHORT_PTE_B_BIT | ARM_SHORT_PTE_C_BIT;
	if (prot & IOMMU_WRITE)
		pteprot |= ARM_SHORT_PTE_TEX0_BIT;
	return pteprot;
}

static arm_short_iopte __arm_short_pgd_prot(int prot, bool super)
{
	arm_short_iopte pgdprot;

	pgdprot = ARM_SHORT_PGD_S_BIT;
	pgdprot |= super ? ARM_SHORT_PGD_TYPE_SUPERSECTION :
				ARM_SHORT_PGD_TYPE_SECTION;
	if (prot & IOMMU_CACHE)
		pgdprot |= ARM_SHORT_PGD_C_BIT | ARM_SHORT_PGD_B_BIT;
	if (prot & IOMMU_WRITE)
		pgdprot |= ARM_SHORT_PGD_TEX0_BIT;
	return pgdprot;
}

static arm_short_iopte __arm_short_pgd_to_pte_prot(int pgdprot, bool large)
{
	arm_short_iopte pteprot = 0;

	if (pgdprot & ARM_SHORT_PGD_C_BIT)
		pteprot |= ARM_SHORT_PTE_C_BIT;
	if (pgdprot & ARM_SHORT_PGD_B_BIT)
		pteprot |= ARM_SHORT_PTE_B_BIT;
	if (pgdprot & ARM_SHORT_PGD_TEX0_BIT)
		pteprot |= ARM_SHORT_PTE_TEX0_BIT;
	return pteprot;
}

static int
_arm_short_map(struct arm_short_io_pgtable *data,
	       unsigned int iova, phys_addr_t paddr,
	       arm_short_iopte pgdprot, arm_short_iopte pteprot,
	       bool large)
{
	arm_short_iopte *pgd = data->pgd, *pte;
	const struct iommu_gather_ops *tlb = data->iop.cfg.tlb;
	void *cookie = data->iop.cookie;
	void *pte_va;
	int i, ptenr = large ? 16 : 1;
	bool ptenew = false;

	pgd += ARM_SHORT_PGD_IDX(iova);

	pr_debug("%s iova:0x%x, pa %pa pgd 0x%x pte 0x%x %s %s\n", __func__,
		 iova, &paddr, pgdprot, pteprot, large ? "large" : "small",
		 pteprot ? "page" : "section");

	if (!pteprot) { /* section or supersection */
		if (data->iop.cfg.quirks & IO_PGTABLE_QUIRK_ARM_NS)
			pgdprot |= ARM_SHORT_PGD_NS_BIT_SECTION;
		pte = pgd;
		pteprot = pgdprot;
	} else {        /* page or largepage */
		pgdprot = ARM_SHORT_PGD_TYPE_PGTABLE;
		if (data->iop.cfg.quirks & IO_PGTABLE_QUIRK_ARM_NS)
			pgdprot |= ARM_SHORT_PGD_NS_BIT_PAGE;

		if (!(*pgd)) {
			pte_va = kmem_cache_zalloc(data->ptekmem, GFP_ATOMIC);
			if (unlikely(!pte_va))
				return -ENOMEM;
			ptenew = true;
			*pgd = virt_to_phys(pte_va) | pgdprot;
			kmemleak_ignore(pte_va);
			tlb->flush_pgtable(pgd, sizeof(*pgd), cookie);
		} else {/* Someone else may have allocated for this pgd */
			if (((*pgd) & (~ARM_SHORT_PGD_PA_PGTABLE_MSK)) !=
			     pgdprot) {
				pr_err("The prot of old pgd is not Right!iova=0x%x pgd=0x%x pgdprot=0x%x\n",
				       iova, (*pgd), pgdprot);
				return -EEXIST;
			}
		}
		pte = arm_short_get_pte_in_pgd(*pgd, iova);
	}

	pteprot |= (arm_short_iopte)paddr;
	for (i = 0; i < ptenr; i++) {
		if (pte[i]) {
			pr_err("The To-Map pte exists!(iova=0x%x pte=0x%x i=%d)\n",
			       iova, pte[i], i);
			goto err_exist_pte;
		}
		pte[i] = pteprot;
	}
	tlb->flush_pgtable(pte, ptenr * sizeof(*pte), cookie);

	return 0;

err_exist_pte:
	while (i--)
		pte[i] = 0;
	if (ptenew)
		kmem_cache_free(data->ptekmem, pte_va);
	return -EEXIST;
}

static int arm_short_map(struct io_pgtable_ops *ops, unsigned long iova,
			 phys_addr_t paddr, size_t size, int prot)
{
	struct arm_short_io_pgtable *data = io_pgtable_short_ops_to_data(ops);
	const struct iommu_gather_ops *tlb = data->iop.cfg.tlb;
	int ret;
	arm_short_iopte pgdprot = 0, pteprot = 0;
	bool large;

	if (!(prot & (IOMMU_READ | IOMMU_WRITE)))
		return -EINVAL;

	switch (size) {
	case SZ_4K:
	case SZ_64K:
		large = (size == SZ_64K) ? true : false;
		pteprot = __arm_short_pte_prot(prot, large);
		break;

	case SZ_1M:
	case SZ_16M:
		large = (size == SZ_16M) ? true : false;
		pgdprot = __arm_short_pgd_prot(prot, large);
		break;
	default:
		pr_err("Error Size to map (%zu)\n", size);
		return -EINVAL;
	}

	if ((iova | paddr) & (size - 1)) {
		pr_err("IOVA|PA Not Aligned(iova=0x%lx pa=%pa size=%zu)\n",
		       iova, &paddr, size);
		return -EINVAL;
	}

	ret = _arm_short_map(data, iova, paddr, pgdprot, pteprot, large);

	tlb->tlb_add_flush(iova, size, true, data->iop.cookie);
	tlb->tlb_sync(data->iop.cookie);
	return ret;
}

static phys_addr_t arm_short_iova_to_phys(struct io_pgtable_ops *ops,
					  unsigned long iova)
{
	struct arm_short_io_pgtable *data = io_pgtable_short_ops_to_data(ops);
	arm_short_iopte *pte, *pgd = data->pgd;
	phys_addr_t pa = 0;

	pgd += ARM_SHORT_PGD_IDX(iova);

	if (ARM_SHORT_PGD_TYPE_IS_PGTABLE(*pgd)) {
		pte = arm_short_get_pte_in_pgd(*pgd, iova);

		if (ARM_SHORT_PTE_TYPE_IS_LARGEPAGE(*pte)) {
			pa = (*pte) & ARM_SHORT_PTE_PA_LARGE_MSK;
			pa |= iova & (~ARM_SHORT_PTE_PA_LARGE_MSK);
		} else if (ARM_SHORT_PTE_TYPE_IS_SMALLPAGE(*pte)) {
			pa = (*pte) & ARM_SHORT_PTE_PA_SMALL_MSK;
			pa |= iova & (~ARM_SHORT_PTE_PA_SMALL_MSK);
		}
	} else if (ARM_SHORT_PGD_TYPE_IS_SECTION(*pgd)) {
		pa = (*pgd) & ARM_SHORT_PGD_PA_SECTION_MSK;
		pa |= iova & (~ARM_SHORT_PGD_PA_SECTION_MSK);
	} else if (ARM_SHORT_PGD_TYPE_IS_SUPERSECTION(*pgd)) {
		pa = (*pgd) & ARM_SHORT_PGD_PA_SUPERSECTION_MSK;
		pa |= iova & (~ARM_SHORT_PGD_PA_SUPERSECTION_MSK);
	} else {
		pr_err("Failed while iova to phys(pgd=0x%x)\n", *pgd);
	}

	return pa;
}

/*
 * The split is not full function in this function.
 * Take a example, there are a section map(iova: 0x12300000, pa:0x4300000)
 * If unmap start iova is 0x12300000,and len is 4K, we support it now.
 * If unmap start iova is 0x12350000, we don't support it right now.
 * we need check LPAE further.
 */
static int
arm_short_split_blk_unmap(struct io_pgtable_ops *ops, unsigned int iova,
			  phys_addr_t paddr, size_t size,
			  arm_short_iopte pgdprot, arm_short_iopte pteprot,
			  size_t blk_size)
{
	struct arm_short_io_pgtable *data = io_pgtable_short_ops_to_data(ops);
	struct io_pgtable_cfg *cfg = &data->iop.cfg;
	unsigned int blk_base, blk_start, blk_end;
	phys_addr_t blk_paddr;
	size_t mapsize = 0, nextmapsize;
	int ret;
	unsigned int i;

	/* find the nearest mapsize */
	for (i = find_first_bit(&cfg->pgsize_bitmap, BITS_PER_LONG);
	     i < BITS_PER_LONG && ((1 << i) < blk_size) &&
	     IS_ALIGNED(size, 1 << i);
	     i = find_next_bit(&cfg->pgsize_bitmap, BITS_PER_LONG, i + 1))
		mapsize = 1 << i;

	if (!mapsize) {
		pr_err("Failed to get remap size(%zu) iova=0x%x\n", size, iova);
		return -EINVAL;
	}
	nextmapsize = 1 << i;

	blk_base = iova & ~(blk_size - 1);
	blk_start = blk_base;
	blk_end = blk_start + blk_size;
	blk_paddr = paddr;

	pr_debug("split this map 0x%x,size %zu blksize %zu, mapsize %zu end %zu 0x%x\n",
		 iova, size, blk_size, mapsize, blk_start + blk_size, blk_end);

	for (; blk_start < blk_end;
	     blk_start += mapsize, blk_paddr += mapsize) {
		/* Unmap! */
		if (blk_start < blk_base + size)
			continue;

		/* try to upper map */
		if (IS_ALIGNED(blk_start | blk_paddr, nextmapsize) &&
		    mapsize != nextmapsize) {
			mapsize = nextmapsize;
			i = find_next_bit(&cfg->pgsize_bitmap, BITS_PER_LONG,
					  i + 1);
			if (i < BITS_PER_LONG)
				nextmapsize = 1 << i;
		}

		if (mapsize == SZ_1M) { /* prot here is 0 */
			pgdprot |= __arm_short_pgd_prot(0, mapsize == SZ_16M);
			pteprot = 0;
		} else {
			pteprot |= __arm_short_pte_prot(0, mapsize == SZ_64K);
		}

		ret = _arm_short_map(data, blk_start, blk_paddr,
				     pgdprot, pteprot, mapsize == SZ_64K);
		if (ret < 0)
			break;
	}
	return size;
}

static int arm_short_unmap(struct io_pgtable_ops *ops,
			   unsigned long iova,
			   size_t size)
{
	struct arm_short_io_pgtable *data = io_pgtable_short_ops_to_data(ops);
	const struct iommu_gather_ops *tlb = data->iop.cfg.tlb;
	void *cookie = data->iop.cookie;
	arm_short_iopte *pgd, *pte = NULL;
	unsigned int blk_size = 0;
	int ret;
	unsigned int nrtoclean;
	unsigned int iova_end = iova + size - 1;
	phys_addr_t paddr;

	do {
		pgd = (arm_short_iopte *)data->pgd + ARM_SHORT_PGD_IDX(iova);

		/* get block size */
		if (ARM_SHORT_PGD_TYPE_IS_PGTABLE(*pgd)) {
			pte = arm_short_get_pte_in_pgd(*pgd, iova);

			if (ARM_SHORT_PTE_TYPE_IS_SMALLPAGE(*pte))
				blk_size = SZ_4K;
			else if (ARM_SHORT_PTE_TYPE_IS_LARGEPAGE(*pte))
				blk_size = SZ_64K;
		} else if (ARM_SHORT_PGD_TYPE_IS_SECTION(*pgd)) {
			blk_size = SZ_1M;
		} else if (ARM_SHORT_PGD_TYPE_IS_SUPERSECTION(*pgd)) {
			blk_size = SZ_16M;
		} else {
			break;
		}

		paddr = arm_short_iova_to_phys(ops, iova & ~(blk_size - 1));

		 /* the start iova is not aligned, we will split it.
		  * current we don't add this split.
		  */
		if (!IS_ALIGNED(iova, blk_size)) {
			pr_debug("%s iova(0x%lx) blksize(0x%x) isn't aligned\n",
				 __func__, iova, blk_size);
		}

		if (blk_size == SZ_4K || blk_size == SZ_64K) {
			bool wholeptefreed;

			nrtoclean = blk_size / SZ_4K;
			memset(pte, 0, nrtoclean * sizeof(*pte));

			wholeptefreed = arm_short_free_wholepte(data, pgd);
			if (wholeptefreed)
				tlb->flush_pgtable(pgd, sizeof(*pgd), cookie);
			else
				tlb->flush_pgtable(pte,
						   nrtoclean * sizeof(*pte),
						   cookie);
		} else if (blk_size == SZ_1M || blk_size == SZ_16M) {
			nrtoclean = blk_size / SZ_1M;
			memset(pgd, 0, nrtoclean * sizeof(*pgd));
			tlb->flush_pgtable(pgd, nrtoclean * sizeof(*pgd),
					   cookie);
		}

		if (size < blk_size) { /* split */
			arm_short_iopte pgdprot = 0, pteprot = 0;

			if (blk_size == SZ_64K && pte) {
				pteprot = ARM_SHORT_PTE_GET_PROT(*pte);
			} else if (blk_size == SZ_1M) {
				pteprot = __arm_short_pgd_to_pte_prot(
							*pgd, size == SZ_64K);
			} else if (blk_size == SZ_16M) {
				pgdprot = ARM_SHORT_PGD_GET_PROT(*pgd);
				pteprot = __arm_short_pgd_to_pte_prot(
							*pgd, size == SZ_64K);
			}
			ret = arm_short_split_blk_unmap(
						ops, iova, paddr, size,
						pgdprot, pteprot, blk_size);
			size = size & ~(blk_size - 1);
		} else {
			size -= blk_size;
		}
		iova += blk_size;
		tlb->tlb_add_flush(iova, size, true, cookie);
		tlb->tlb_sync(cookie);
	} while (iova <= iova_end && size);

	return size;
}

static struct io_pgtable *
arm_short_alloc_pgtable(struct io_pgtable_cfg *cfg, void *cookie)
{
	struct arm_short_io_pgtable *data;

	if (cfg->ias > 32)
		return NULL;

	if (cfg->oas > 32)
		return NULL;

	cfg->pgsize_bitmap &= SZ_4K | SZ_64K | SZ_1M;
	if (cfg->quirks & IO_PGTABLE_QUIRK_SHORT_SUPERSECTION)
		cfg->pgsize_bitmap &= SZ_16M;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	data->pgd_size = SZ_16K;
	data->pgd = alloc_pages_exact(data->pgd_size,
				      GFP_KERNEL | __GFP_ZERO | __GFP_DMA);
	if (!data->pgd)
		goto out_free_data;

	cfg->tlb->flush_pgtable(data->pgd, data->pgd_size, cookie);

	/* kmem for pte */
	data->ptekmem = kmem_cache_create("io-pgtable-arm-short",
					   ARM_SHORT_BYTES_PER_PTE,
					   ARM_SHORT_BYTES_PER_PTE,
					   0, NULL);
	if (!data->ptekmem)
		goto out_free_pte;

	/* TTBRs */
	cfg->arm_short_cfg.ttbr[0] = virt_to_phys(data->pgd);
	cfg->arm_short_cfg.ttbr[1] = 0;

	cfg->arm_short_cfg.tcr = 0;
	cfg->arm_short_cfg.nmrr = 0;
	cfg->arm_short_cfg.prrr = 0;

	data->iop.ops = (struct io_pgtable_ops) {
		.map		= arm_short_map,
		.unmap		= arm_short_unmap,
		.iova_to_phys	= arm_short_iova_to_phys,
	};

	return &data->iop;

out_free_pte:
	free_pages_exact(data->pgd, data->pgd_size);
out_free_data:
	kfree(data);
	return NULL;
}

static void arm_short_free_pgtable(struct io_pgtable *iop)
{
	struct arm_short_io_pgtable *data = io_pgtable_short_to_data(iop);

	kmem_cache_destroy(data->ptekmem);
	free_pages_exact(data->pgd, data->pgd_size);
	kfree(data);
}

struct io_pgtable_init_fns io_pgtable_arm_short_init_fns = {
	.alloc	= arm_short_alloc_pgtable,
	.free	= arm_short_free_pgtable,
};

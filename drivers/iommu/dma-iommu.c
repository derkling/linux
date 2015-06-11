/*
 * A fairly generic DMA-API to IOMMU-API glue layer.
 *
 * Copyright (C) 2014-2015 ARM Ltd.
 *
 * based in part on arch/arm/mm/dma-mapping.c:
 * Copyright (C) 2000-2004 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/device.h>
#include <linux/dma-iommu.h>
#include <linux/huge_mm.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/mm.h>

int iommu_dma_init(void)
{
	return iommu_iova_cache_init();
}

struct iommu_dma_domain {
	struct iommu_domain *domain;
	struct iova_domain *iovad;
	struct kref kref;
};

/**
 * iommu_dma_create_domain - Create a DMA mapping domain
 * @ops: iommu_ops representing the IOMMU backing this domain. It is down to
 *       the IOMMU driver whether a domain may span multiple IOMMU instances
 * @base: IOVA at which the mappable address space starts
 * @size: Size of IOVA space
 *
 * @base and @size should be exact multiples of IOMMU page granularity to
 * avoid rounding surprises. If necessary, we reserve the page at address 0
 * to ensure it is an invalid IOVA.
 *
 * Return: Pointer to a domain initialised with the given IOVA range,
 *         or NULL on failure. If successful, the caller holds an initial
 *         reference, which may be released with iommu_dma_release_domain()
 *         once a device is attached.
 */
struct iommu_dma_domain *iommu_dma_create_domain(const struct iommu_ops *ops,
		dma_addr_t base, u64 size)
{
	struct iommu_dma_domain *dom;
	struct iommu_domain *domain;
	struct iova_domain *iovad;
	unsigned long order, base_pfn, end_pfn;

	dom = kzalloc(sizeof(*dom), GFP_KERNEL);
	if (!dom)
		return NULL;
	/*
	 * HACK(sort of): These domains currently belong to this layer and are
	 * opaque from outside it, so they are "unmanaged" by the IOMMU API
	 * itself. Once we have default domain support worked out, then we can
	 * turn things inside out and put these inside managed IOMMU domains...
	 */
	domain = ops->domain_alloc(IOMMU_DOMAIN_UNMANAGED);
	if (!domain)
		goto out_free_dma_domain;

	domain->ops = ops;
	domain->type = IOMMU_DOMAIN_UNMANAGED;

	/* Use the smallest supported page size for IOVA granularity */
	order = __ffs(ops->pgsize_bitmap);
	base_pfn = max_t(unsigned long, 1, base >> order);
	end_pfn = (base + size - 1) >> order;

	/* Check the domain allows at least some access to the device... */
	if (domain->geometry.force_aperture) {
		if (base > domain->geometry.aperture_end ||
		    base + size <= domain->geometry.aperture_start) {
			pr_warn("specified DMA range outside IOMMU capability\n");
			goto out_free_iommu_domain;
		}
		/* ...then finally give it a kicking to make sure it fits */
		base_pfn = max_t(unsigned long, base_pfn,
				domain->geometry.aperture_start >> order);
		end_pfn = min_t(unsigned long, end_pfn,
				domain->geometry.aperture_end >> order);
	}
	/*
	 * Note that this almost certainly breaks the case where multiple
	 * devices with different DMA capabilities need to share a domain,
	 * but we don't have the necessary information to handle that here
	 * anyway - "proper" group and domain allocation needs to involve
	 * the IOMMU driver and a complete view of the bus.
	 */

	iovad = kzalloc(sizeof(*iovad), GFP_KERNEL);
	if (!iovad)
		goto out_free_iommu_domain;

	init_iova_domain(iovad, 1UL << order, base_pfn, end_pfn);

	dom->domain = domain;
	dom->iovad = iovad;
	kref_init(&dom->kref);
	return dom;

out_free_iommu_domain:
	ops->domain_free(domain);
out_free_dma_domain:
	kfree(dom);
	return NULL;
}

static void __iommu_dma_free_domain(struct kref *kref)
{
	struct iommu_dma_domain *dom;

	dom = container_of(kref, struct iommu_dma_domain, kref);
	put_iova_domain(dom->iovad);
	iommu_domain_free(dom->domain);
	kfree(dom);
}

void iommu_dma_release_domain(struct iommu_dma_domain *dom)
{
	kref_put(&dom->kref, __iommu_dma_free_domain);
}

struct iommu_domain *iommu_dma_raw_domain(struct iommu_dma_domain *dom)
{
	return dom->domain;
}

int iommu_dma_attach_device(struct device *dev, struct iommu_dma_domain *dom)
{
	int ret;

	kref_get(&dom->kref);
	ret = iommu_attach_device(dom->domain, dev);
	if (!ret)
		arch_set_dma_domain(dev, dom);
	return ret;
}

void iommu_dma_detach_device(struct device *dev)
{
	struct iommu_dma_domain *dom = arch_get_dma_domain(dev);

	arch_set_dma_domain(dev, NULL);
	iommu_detach_device(dom->domain, dev);
	iommu_dma_release_domain(dom);
}

/*
 * IOVAs are IOMMU _input_ addresses, so there still exists the possibility
 * for static bus translation between device output and IOMMU input (yuck).
 */
static inline dma_addr_t dev_dma_addr(struct device *dev, dma_addr_t addr)
{
	dma_addr_t offset = (dma_addr_t)dev->dma_pfn_offset << PAGE_SHIFT;

	BUG_ON(addr < offset);
	return addr - offset;
}

/**
 * dma_direction_to_prot - Translate DMA API directions to IOMMU API page flags
 * @dir: Direction of DMA transfer
 * @coherent: Is the DMA master cache-coherent?
 *
 * Return: corresponding IOMMU API page protection flags
 */
int dma_direction_to_prot(enum dma_data_direction dir, bool coherent)
{
	int prot = coherent ? IOMMU_CACHE : 0;

	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

static struct iova *__alloc_iova(struct device *dev, size_t size, bool coherent)
{
	struct iommu_dma_domain *dom = arch_get_dma_domain(dev);
	struct iova_domain *iovad = dom->iovad;
	unsigned long shift = iova_shift(iovad);
	unsigned long length = iova_align(iovad, size) >> shift;
	u64 dma_limit = coherent ? dev->coherent_dma_mask : dma_get_mask(dev);

	/*
	 * Enforce size-alignment to be safe - there should probably be
	 * an attribute to control this per-device, or at least per-domain...
	 */
	return alloc_iova(iovad, length, dma_limit >> shift, true);
}

/* The IOVA allocator knows what we mapped, so just unmap whatever that was */
static void __iommu_dma_unmap(struct iommu_dma_domain *dom, dma_addr_t dma_addr)
{
	struct iova_domain *iovad = dom->iovad;
	unsigned long shift = iova_shift(iovad);
	unsigned long pfn = dma_addr >> shift;
	struct iova *iova = find_iova(iovad, pfn);
	size_t size = iova_size(iova) << shift;

	/* ...and if we can't, then something is horribly, horribly wrong */
	BUG_ON(iommu_unmap(dom->domain, pfn << shift, size) < size);
	__free_iova(iovad, iova);
}

static void __iommu_dma_free_pages(struct page **pages, int count)
{
	while (count--)
		__free_page(pages[count]);
	kvfree(pages);
}

static struct page **__iommu_dma_alloc_pages(unsigned int count, gfp_t gfp)
{
	struct page **pages;
	unsigned int i = 0, array_size = count * sizeof(*pages);

	if (array_size <= PAGE_SIZE)
		pages = kzalloc(array_size, GFP_KERNEL);
	else
		pages = vzalloc(array_size);
	if (!pages)
		return NULL;

	while (count) {
		struct page *page = NULL;
		int j, order = __fls(count);

		/*
		 * Higher-order allocations are a convenience rather
		 * than a necessity, hence using __GFP_NORETRY until
		 * falling back to single-page allocations.
		 */
		for (order = min(order, MAX_ORDER); order > 0; order--) {
			page = alloc_pages(gfp | __GFP_NORETRY, order);
			if (!page)
				continue;
			if (PageCompound(page)) {
				if (!split_huge_page(page))
					break;
				__free_pages(page, order);
			} else {
				split_page(page, order);
				break;
			}
		}
		if (!page)
			page = alloc_page(gfp);
		if (!page) {
			__iommu_dma_free_pages(pages, i);
			return NULL;
		}
		j = 1 << order;
		count -= j;
		while (j--)
			pages[i++] = page++;
	}
	return pages;
}

/**
 * iommu_dma_free - Free a buffer allocated by iommu_dma_alloc()
 * @dev: Device which owns this buffer
 * @pages: Array of buffer pages as returned by iommu_dma_alloc()
 * @size: Size of buffer in bytes
 * @handle: DMA address of buffer
 *
 * Frees both the pages associated with the buffer, and the array
 * describing them
 */
void iommu_dma_free(struct device *dev, struct page **pages, size_t size,
		dma_addr_t *handle)
{
	__iommu_dma_unmap(arch_get_dma_domain(dev), *handle);
	__iommu_dma_free_pages(pages, PAGE_ALIGN(size) >> PAGE_SHIFT);
	*handle = DMA_ERROR_CODE;
}

/**
 * iommu_dma_alloc - Allocate and map a buffer contiguous in IOVA space
 * @dev: Device to allocate memory for. Must be a real device
 *	 attached to an iommu_dma_domain
 * @size: Size of buffer in bytes
 * @gfp: Allocation flags
 * @prot: IOMMU mapping flags
 * @coherent: Which dma_mask to base IOVA allocation on
 * @handle: Out argument for allocated DMA handle
 * @flush_page: Arch callback to flush a single page from caches as
 *		necessary. May be NULL for coherent allocations
 *
 * If @size is less than PAGE_SIZE, then a full CPU page will be allocated,
 * but an IOMMU which supports smaller pages might not map the whole thing.
 * For now, the buffer is unconditionally zeroed for compatibility
 *
 * Return: Array of struct page pointers describing the buffer,
 *	   or NULL on failure.
 */
struct page **iommu_dma_alloc(struct device *dev, size_t size, gfp_t gfp,
		int prot, bool coherent, dma_addr_t *handle,
		void (*flush_page)(const void *, phys_addr_t))
{
	struct iommu_dma_domain *dom = arch_get_dma_domain(dev);
	struct iova_domain *iovad = dom->iovad;
	struct iova *iova;
	struct page **pages;
	struct sg_table sgt;
	struct sg_mapping_iter miter;
	dma_addr_t dma_addr;
	unsigned int count = PAGE_ALIGN(size) >> PAGE_SHIFT;

	*handle = DMA_ERROR_CODE;

	/* IOMMU can map any pages, so himem can also be used here */
	gfp |= __GFP_NOWARN | __GFP_HIGHMEM;
	pages = __iommu_dma_alloc_pages(count, gfp);
	if (!pages)
		return NULL;

	iova = __alloc_iova(dev, size, coherent);
	if (!iova)
		goto out_free_pages;

	if (sg_alloc_table_from_pages(&sgt, pages, count, 0, size, GFP_KERNEL))
		goto out_free_iova;

	dma_addr = iova_dma_addr(iovad, iova);
	if (iommu_map_sg(dom->domain, dma_addr, sgt.sgl, sgt.orig_nents, prot)
			< size)
		goto out_free_sg;

	/* Using the non-flushing flag since we're doing our own */
	sg_miter_start(&miter, sgt.sgl, sgt.orig_nents, SG_MITER_FROM_SG);
	while (sg_miter_next(&miter)) {
		memset(miter.addr, 0, PAGE_SIZE);
		if (flush_page)
			flush_page(miter.addr, page_to_phys(miter.page));
	}
	sg_miter_stop(&miter);
	sg_free_table(&sgt);

	*handle = dma_addr;
	return pages;

out_free_sg:
	sg_free_table(&sgt);
out_free_iova:
	__free_iova(iovad, iova);
out_free_pages:
	__iommu_dma_free_pages(pages, count);
	return NULL;
}

/**
 * iommu_dma_mmap - Map a buffer into provided user VMA
 * @pages: Array representing buffer from iommu_dma_alloc()
 * @size: Size of buffer in bytes
 * @vma: VMA describing requested userspace mapping
 *
 * Maps the pages of the buffer in @pages into @vma. The caller is responsible
 * for verifying the correct size and protection of @vma beforehand.
 */

int iommu_dma_mmap(struct page **pages, size_t size, struct vm_area_struct *vma)
{
	unsigned long uaddr = vma->vm_start;
	unsigned int i, count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	int ret = -ENXIO;

	for (i = vma->vm_pgoff; i < count && uaddr < vma->vm_end; i++) {
		ret = vm_insert_page(vma, uaddr, pages[i]);
		if (ret)
			break;
		uaddr += PAGE_SIZE;
	}
	return ret;
}

dma_addr_t iommu_dma_map_page(struct device *dev, struct page *page,
		unsigned long offset, size_t size, int prot, bool coherent)
{
	dma_addr_t dma_addr;
	struct iommu_dma_domain *dom = arch_get_dma_domain(dev);
	struct iova_domain *iovad = dom->iovad;
	phys_addr_t phys = page_to_phys(page) + offset;
	size_t iova_off = iova_offset(iovad, phys);
	size_t len = iova_align(iovad, size + iova_off);
	struct iova *iova = __alloc_iova(dev, len, coherent);

	if (!iova)
		return DMA_ERROR_CODE;

	dma_addr = iova_dma_addr(iovad, iova);
	if (!iommu_map(dom->domain, dma_addr, phys - iova_off, len, prot))
		return dev_dma_addr(dev, dma_addr + iova_off);

	__free_iova(iovad, iova);
	return DMA_ERROR_CODE;
}

void iommu_dma_unmap_page(struct device *dev, dma_addr_t handle, size_t size,
		enum dma_data_direction dir, struct dma_attrs *attrs)
{
	__iommu_dma_unmap(arch_get_dma_domain(dev), handle);
}

static int __finalise_sg(struct device *dev, struct scatterlist *sg, int nents,
		dma_addr_t dma_addr)
{
	struct scatterlist *s, *seg = sg;
	unsigned long seg_mask = dma_get_seg_boundary(dev);
	unsigned int max_len = dma_get_max_seg_size(dev);
	unsigned int seg_len = 0, seg_dma = 0;
	int i, count = 1;

	for_each_sg(sg, s, nents, i) {
		/* Un-swizzling the fields here, hence the naming mismatch */
		unsigned int s_offset = sg_dma_address(s);
		unsigned int s_length = sg_dma_len(s);
		unsigned int s_dma_len = s->length;

		s->offset = s_offset;
		s->length = s_length;
		sg_dma_address(s) = DMA_ERROR_CODE;
		sg_dma_len(s) = 0;

		if (seg_len && (seg_dma + seg_len == dma_addr + s_offset) &&
		    (seg_len + s_dma_len <= max_len) &&
		    ((seg_dma & seg_mask) <= seg_mask - (seg_len + s_length))
		   ) {
			sg_dma_len(seg) += s_dma_len;
		} else {
			if (seg_len) {
				seg = sg_next(seg);
				count++;
			}
			sg_dma_len(seg) = s_dma_len - s_offset;
			sg_dma_address(seg) = dma_addr + s_offset;

			seg_len = s_offset;
			seg_dma = dma_addr + s_offset;
		}
		seg_len += s_length;
		dma_addr += s_dma_len;
	}
	return count;
}

static void __invalidate_sg(struct scatterlist *sg, int nents)
{
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, nents, i) {
		if (sg_dma_address(s) != DMA_ERROR_CODE)
			s->offset = sg_dma_address(s);
		if (sg_dma_len(s))
			s->length = sg_dma_len(s);
		sg_dma_address(s) = DMA_ERROR_CODE;
		sg_dma_len(s) = 0;
	}
}

int iommu_dma_map_sg(struct device *dev, struct scatterlist *sg,
		int nents, int prot, bool coherent)
{
	struct iommu_dma_domain *dom = arch_get_dma_domain(dev);
	struct iova_domain *iovad = dom->iovad;
	struct iova *iova;
	struct scatterlist *s;
	dma_addr_t dma_addr;
	size_t iova_len = 0;
	int i;

	/*
	 * Work out how much IOVA space we need, and align the segments to
	 * IOVA granules for the IOMMU driver to handle. With some clever
	 * trickery we can modify the list in a reversible manner.
	 */
	for_each_sg(sg, s, nents, i) {
		size_t s_offset = iova_offset(iovad, s->offset);
		size_t s_length = s->length;

		sg_dma_address(s) = s->offset;
		sg_dma_len(s) = s_length;
		s->offset -= s_offset;
		s_length = iova_align(iovad, s_length + s_offset);
		s->length = s_length;

		iova_len += s_length;
	}

	iova = __alloc_iova(dev, iova_len, coherent);
	if (!iova)
		goto out_restore_sg;

	/*
	 * We'll leave any physical concatenation to the IOMMU driver's
	 * implementation - it knows better than we do.
	 */
	dma_addr = iova_dma_addr(iovad, iova);
	if (iommu_map_sg(dom->domain, dma_addr, sg, nents, prot) < iova_len)
		goto out_free_iova;

	return __finalise_sg(dev, sg, nents, dev_dma_addr(dev, dma_addr));

out_free_iova:
	__free_iova(iovad, iova);
out_restore_sg:
	__invalidate_sg(sg, nents);
	return 0;
}

void iommu_dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nents,
		enum dma_data_direction dir, struct dma_attrs *attrs)
{
	/*
	 * The scatterlist segments are mapped contiguously
	 * in IOVA space, so this is incredibly easy.
	 */
	__iommu_dma_unmap(arch_get_dma_domain(dev), sg_dma_address(sg));
}

int iommu_dma_supported(struct device *dev, u64 mask)
{
	/*
	 * 'Special' IOMMUs which don't have the same addressing capability
	 * as the CPU will have to wait until we have some way to query that
	 * before they'll be able to use this framework.
	 */
	return 1;
}

int iommu_dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return dma_addr == DMA_ERROR_CODE;
}

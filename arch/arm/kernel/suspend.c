#include <linux/init.h>

#include <asm/idmap.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/memory.h>
#include <asm/suspend.h>
#include <asm/tlbflush.h>

extern int __cpu_suspend(unsigned long, int (*)(unsigned long));
extern void cpu_resume_mmu(void);

/*
 * This is called by __cpu_suspend() to save the state, and do whatever
 * flushing is required to ensure that when the CPU goes to sleep we have
 * the necessary data available when the caches are not searched.
 */
void __cpu_suspend_save(u32 *ptr, u32 ptrsz, u32 sp, u32 *save_ptr)
{
	u32 *ctx = ptr;

	*save_ptr = virt_to_phys(ptr);

	/* This must correspond to the LDM in cpu_resume() assembly */
	*ctx++ = virt_to_phys(idmap_pgd);
	*ctx++ = sp;
	*ctx++ = virt_to_phys(cpu_do_resume);

	cpu_do_suspend(ctx);

	flush_dcache_level(flush_cache_level_cpu());
	/*
	 * flush_dcache_level does not guarantee that
	 * save_ptr and ptr are cleaned to main memory,
	 * just up to the required cache level.
	 * Since the context pointer and context itself
	 * are to be retrieved with the MMU off that
	 * data must be cleaned from all cache levels
	 * to main memory using "area" cache primitives.
	 */
	__cpuc_flush_dcache_area(ptr, ptrsz);
	__cpuc_flush_dcache_area(save_ptr, sizeof(*save_ptr));
	outer_clean_range(*save_ptr, *save_ptr + ptrsz);
	outer_clean_range(virt_to_phys(save_ptr),
			  virt_to_phys(save_ptr) + sizeof(*save_ptr));
}

/*
 * Hide the first two arguments to __cpu_suspend - these are an implementation
 * detail which platform code shouldn't have to know about.
 */
int cpu_suspend(unsigned long arg, int (*fn)(unsigned long))
{
	struct mm_struct mm;
	unsigned long ttb0, ctx_id;
	int ret;

	if (!idmap_pgd)
		return -EINVAL;

	/*
	 * We can't rely on current->active_mm because the installed
	 * mm and the current task pointer are not always in sync.
	 * There is a small window in context_switch() between
	 * switch_mm() and switch_to() where this is definitely the case.
	 * It is even possible for current->active_mm to be NULL at that
	 * point.  Let's populate a dummy mm_struct with data from the
	 * hardware instead, in order to satisfy cpu_switch_mm()'s needs.
	 */
#ifdef CONFIG_ARM_LPAE
	asm volatile ("mrrc p15, 0, %0, %1, c2" : "=r" (ttb0), "=r" (ctx_id));
	mm.pgd = phys_to_virt(ttb0);
	mm.context.id = ctx_id >> (48 - 32);
#else
	asm volatile ("mrc p15, 0, %0, c2, c0, 0" : "=r" (ttb0));
	asm volatile ("mrc p15, 0, %0, c13, c0, 1" : "=r" (ctx_id));
	mm.pgd = phys_to_virt(ttb0);
	mm.context.id = ctx_id;
#endif

	/*
	 * Provide a temporary page table with an identity mapping for
	 * the MMU-enable code, required for resuming.  On successful
	 * resume (indicated by a zero return code), we need to switch
	 * back to the correct page tables.
	 */
	ret = __cpu_suspend(arg, fn);
	if (ret == 0) {
		cpu_switch_mm(mm.pgd, &mm);
		local_flush_tlb_all();
	}

	return ret;
}

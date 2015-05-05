/*
 * FPU performance measurement routines
 */
#include <asm/fpu/internal.h>
#include <asm/tlbflush.h>

#include <linux/kernel.h>

/*
 * Number of repeated measurements we do. We pick the fastest one:
 */
static int loops = 1000;

/*
 * Various small functions, whose overhead we measure:
 */

typedef void (*bench_fn_t)(void) __aligned(32);

static void fn_empty(void)
{
}

/* Basic instructions: */

static void fn_nop(void)
{
	asm volatile ("nop");
}

static void fn_rdtsc(void)
{
	u32 low, high;

	asm volatile ("rdtsc": "=a"(low), "=d"(high));
}

static void fn_rdmsr(void)
{
	u64 efer;

	rdmsrl_safe(MSR_EFER, &efer);
}

static void fn_wrmsr(void)
{
	u64 efer;

	if (!rdmsrl_safe(MSR_EFER, &efer))
		wrmsrl_safe(MSR_EFER, efer);
}

static void fn_cli_same(void)
{
	asm volatile ("cli");
}

static void fn_cli_flip(void)
{
	asm volatile ("sti");
	asm volatile ("cli");
}

static void fn_sti_same(void)
{
	asm volatile ("sti");
}

static void fn_sti_flip(void)
{
	asm volatile ("cli");
	asm volatile ("sti");
}

static void fn_pushf(void)
{
	arch_local_save_flags();
}

static void fn_popf_baseline(void)
{
	arch_local_save_flags();
	asm volatile ("cli");
}

static void fn_popf_flip(void)
{
	unsigned long flags = arch_local_save_flags();
	asm volatile ("cli");

	arch_local_irq_restore(flags);
}

static void fn_popf_same(void)
{
	unsigned long flags = arch_local_save_flags();

	arch_local_irq_restore(flags);
}

/* Basic IRQ save/restore APIs: */

static void fn_irq_save_baseline(void)
{
	local_irq_enable();
}

static void fn_irq_save(void)
{
	unsigned long flags;

	local_irq_enable();
	local_irq_save(flags);
}

static void fn_irq_restore_flip(void)
{
	unsigned long flags;

	local_irq_enable();
	local_irq_save(flags);
	local_irq_restore(flags);
}

static void fn_irq_restore_same(void)
{
	unsigned long flags;

	local_irq_disable();
	local_irq_save(flags);
	local_irq_restore(flags);
}

static void fn_irq_save_restore_flip(void)
{
	unsigned long flags;

	local_irq_enable();

	local_irq_save(flags);
	local_irq_restore(flags);
}

static void fn_irq_save_restore_same(void)
{
	unsigned long flags;

	local_irq_enable();

	local_irq_save(flags);
	local_irq_restore(flags);
}

/* Basic locking primitives: */

static void fn_smp_mb(void)
{
	smp_mb();
}

static void fn_cpu_relax(void)
{
	cpu_relax();
}

static DEFINE_SPINLOCK(test_spinlock);

static void fn_spin_lock_unlock(void)
{
	spin_lock(&test_spinlock);
	spin_unlock(&test_spinlock);
}

static DEFINE_RWLOCK(test_rwlock);

static void fn_read_lock_unlock(void)
{
	read_lock(&test_rwlock);
	read_unlock(&test_rwlock);
}

static void fn_write_lock_unlock(void)
{
	write_lock(&test_rwlock);
	write_unlock(&test_rwlock);
}

static void fn_rcu_read_lock_unlock(void)
{
	rcu_read_lock();
	rcu_read_unlock();
}

static void fn_preempt_disable_enable(void)
{
	preempt_disable();
	preempt_enable();
}

static DEFINE_MUTEX(test_mutex);

static void fn_mutex_lock_unlock(void)
{
	local_irq_enable();

	mutex_lock(&test_mutex);
	mutex_unlock(&test_mutex);
}

/* MM instructions: */

static void fn_flush_tlb(void)
{
	__flush_tlb();
}

static void fn_flush_tlb_global(void)
{
	__flush_tlb_global();
}

static char tlb_flush_target[PAGE_SIZE] __aligned(4096);

static void fn_flush_tlb_one(void)
{
	unsigned long addr = (unsigned long)&tlb_flush_target;

	tlb_flush_target[0]++;
	__flush_tlb_one(addr);
}

static void fn_flush_tlb_range(void)
{
	unsigned long start = (unsigned long)&tlb_flush_target;
	unsigned long end = start+PAGE_SIZE;
	struct mm_struct *mm_saved;

	tlb_flush_target[0]++;

	mm_saved = current->mm;
	current->mm = current->active_mm;

	flush_tlb_mm_range(current->active_mm, start, end, 0);

	current->mm = mm_saved;
}

/* FPU instructions: */
/* FPU instructions: */

static void fn_read_cr0(void)
{
	read_cr0();
}

static void fn_rw_cr0(void)
{
	write_cr0(read_cr0());
}

static void fn_cr0_fault(void)
{
	struct fpu *fpu = &current->thread.fpu;
	u32 cr0 = read_cr0();

	write_cr0(cr0 | X86_CR0_TS);

	asm volatile("fwait");

	/* Zap the FP state we created via the fault: */
	fpu->fpregs_active = 0;
	fpu->fpstate_active = 0;

	write_cr0(cr0);
}

static void fn_fninit(void)
{
	asm volatile ("fninit");
}

static void fn_fwait(void)
{
	asm volatile("fwait");
}

static void fn_fsave(void)
{
	static struct fregs_state fstate __aligned(32);

	copy_fregs_to_user(&fstate);
}

static void fn_frstor(void)
{
	static struct fregs_state fstate __aligned(32);

	copy_fregs_to_user(&fstate);
	copy_user_to_fregs(&fstate);
}

static void fn_fxsave(void)
{
	struct fxregs_state fxstate __aligned(32);

	copy_fxregs_to_user(&fxstate);
}

static void fn_fxrstor(void)
{
	static struct fxregs_state fxstate __aligned(32);

	copy_fxregs_to_user(&fxstate);
	copy_user_to_fxregs(&fxstate);
}

/*
 * Provoke #GP on invalid FXRSTOR:
 */
static void fn_fxrstor_fault(void)
{
	static struct fxregs_state fxstate __aligned(32);
	struct fpu *fpu = &current->thread.fpu;

	copy_fxregs_to_user(&fxstate);

	/* Set invalid MXCSR value, this will generate a #GP: */
	fxstate.mxcsr = -1;

	copy_user_to_fxregs(&fxstate);

	/* Zap any FP state we created via the fault: */
	fpu->fpregs_active = 0;
	fpu->fpstate_active = 0;
}

static void fn_xsave(void)
{
	static struct xregs_state x __aligned(32);

	copy_xregs_to_kernel_booting(&x);
}

static void fn_xrstor(void)
{
	static struct xregs_state x __aligned(32);

	copy_xregs_to_kernel_booting(&x);
	copy_kernel_to_xregs_booting(&x, -1);
}

/*
 * Provoke #GP on invalid XRSTOR:
 */
static void fn_xrstor_fault(void)
{
	static struct xregs_state x __aligned(32);

	copy_xregs_to_kernel_booting(&x);

	/* Set invalid MXCSR value, this will generate a #GP: */
	x.i387.mxcsr = -1;

	copy_kernel_to_xregs_booting(&x, -1);
}

static s64
measure(s64 null_overhead, bench_fn_t bench_fn,
	const char *txt_1, const char *txt_2, const char *txt_3)
{
	unsigned long flags;
	u32 cr0_saved;
	int eager_saved;
	u64 t0, t1;
	s64 delta, delta_min;
	int i;

	delta_min = LONG_MAX;

	/* Disable eagerfpu, so that we can provoke CR0::TS faults: */
	eager_saved = boot_cpu_has(X86_FEATURE_EAGER_FPU);
	setup_clear_cpu_cap(X86_FEATURE_EAGER_FPU);

	/* Save CR0 so that we can freely set it to any value during measurement: */
	cr0_saved = read_cr0();
	/* Clear TS, so that we can measure FPU ops by default: */
	write_cr0(cr0_saved & ~X86_CR0_TS);

	local_irq_save(flags);

	asm volatile (".align 32\n");

	for (i = 0; i < loops; i++) {
		rdtscll(t0);
		mb();

		bench_fn();

		mb();
		rdtscll(t1);
		delta = t1-t0;
		if (delta <= 0)
			continue;

		delta_min = min(delta_min, delta);
	}

	local_irq_restore(flags);
	write_cr0(cr0_saved);

	if (eager_saved)
		setup_force_cpu_cap(X86_FEATURE_EAGER_FPU);

	delta_min = max(0LL, delta_min-null_overhead);

	if (txt_1) {
		if (!txt_2)
			txt_2 = "";
		if (!txt_3)
			txt_3 = "";
		pr_info("x86/fpu: Cost of: %-27s %-5s %-8s: %5Ld cycles\n", txt_1, txt_2, txt_3, delta_min);
	}

	return delta_min;
}

/*
 * Measure all the above primitives:
 */
void __init fpu__measure(void)
{
	s64 cost;
	s64 rdmsr_cost;
	s64 cli_cost, sti_cost, popf_cost, irq_save_cost;
	s64 cr0_read_cost, cr0_write_cost;
	s64 save_cost;

	pr_info("x86/fpu:##################################################################\n");
	pr_info("x86/fpu: Running FPU performance measurement suite (cache hot):\n");

	cost = measure(0, fn_empty, "null", NULL, NULL);

	pr_info("x86/fpu:########  CPU instructions:           ############################\n");
	measure(cost, fn_nop, "NOP", "insn", NULL);
	measure(cost, fn_rdtsc, "RDTSC", "insn", NULL);

	rdmsr_cost = measure(cost, fn_rdmsr, "RDMSR", "insn", NULL);
	measure(cost+rdmsr_cost, fn_wrmsr,"WRMSR", "insn", NULL);

	cli_cost = measure(cost, fn_cli_same, "CLI", "insn", "same-IF");
	measure(cost+cli_cost, fn_cli_flip, "CLI", "insn", "flip-IF");

	sti_cost = measure(cost, fn_sti_same, "STI", "insn", "same-IF");
	measure(cost+sti_cost, fn_sti_flip, "STI", "insn", "flip-IF");

	measure(cost, fn_pushf,	"PUSHF", "insn", NULL);

	popf_cost = measure(cost, fn_popf_baseline, NULL, NULL, NULL);
	measure(cost+popf_cost, fn_popf_same, "POPF", "insn", "same-IF");
	measure(cost+popf_cost, fn_popf_flip, "POPF", "insn", "flip-IF");

	pr_info("x86/fpu:########  IRQ save/restore APIs:      ############################\n");
	irq_save_cost = measure(cost, fn_irq_save_baseline, NULL, NULL, NULL);
	irq_save_cost += measure(cost+irq_save_cost, fn_irq_save, "local_irq_save()", "fn", NULL);
	measure(cost+irq_save_cost, fn_irq_restore_same, "local_irq_restore()", "fn", "same-IF");
	measure(cost+irq_save_cost, fn_irq_restore_flip, "local_irq_restore()", "fn", "flip-IF");
	measure(cost+sti_cost, fn_irq_save_restore_same, "irq_save()+restore()", "fn", "same-IF");
	measure(cost+sti_cost, fn_irq_save_restore_flip, "irq_save()+restore()", "fn", "flip-IF");

	pr_info("x86/fpu:########  locking APIs:               ############################\n");
	measure(cost, fn_smp_mb, "smp_mb()", "fn", NULL);
	measure(cost, fn_cpu_relax, "cpu_relax()", "fn", NULL);
	measure(cost, fn_spin_lock_unlock, "spin_lock()+unlock()", "fn", NULL);
	measure(cost, fn_read_lock_unlock, "read_lock()+unlock()", "fn", NULL);
	measure(cost, fn_write_lock_unlock, "write_lock()+unlock()", "fn", NULL);
	measure(cost, fn_rcu_read_lock_unlock, "rcu_read_lock()+unlock()", "fn", NULL);
	measure(cost, fn_preempt_disable_enable, "preempt_disable()+enable()", "fn", NULL);
	measure(cost+sti_cost, fn_mutex_lock_unlock, "mutex_lock()+unlock()", "fn", NULL);

	pr_info("x86/fpu:########  MM instructions:            ############################\n");
	measure(cost, fn_flush_tlb, "__flush_tlb()", "fn", NULL);
	measure(cost, fn_flush_tlb_global, "__flush_tlb_global()", "fn", NULL);
	measure(cost, fn_flush_tlb_one, "__flush_tlb_one()", "fn", NULL);
	measure(cost, fn_flush_tlb_range, "__flush_tlb_range()", "fn", NULL);

	pr_info("x86/fpu:########  FPU instructions:           ############################\n");
	cr0_read_cost = measure(cost, fn_read_cr0, "CR0", "read", NULL);
	cr0_write_cost = measure(cost+cr0_read_cost, fn_rw_cr0,	"CR0", "write", NULL);

	measure(cost+cr0_read_cost+cr0_write_cost, fn_cr0_fault, "CR0::TS", "fault", NULL);

	measure(cost, fn_fninit, "FNINIT", "insn", NULL);
	measure(cost, fn_fwait,	"FWAIT", "insn", NULL);

	save_cost = measure(cost, fn_fsave, "FSAVE", "insn", NULL);
	measure(cost+save_cost, fn_frstor, "FRSTOR", "insn", NULL);

	if (cpu_has_fxsr) {
		save_cost = measure(cost, fn_fxsave, "FXSAVE", "insn", NULL);
		measure(cost+save_cost, fn_fxrstor, "FXRSTOR", "insn", NULL);
		measure(cost+save_cost, fn_fxrstor_fault,"FXRSTOR", "fault", NULL);
	}
	if (cpu_has_xsaveopt) {
		save_cost = measure(cost, fn_xsave, "XSAVE", "insn", NULL);
		measure(cost+save_cost, fn_xrstor, "XRSTOR", "insn", NULL);
		measure(cost+save_cost, fn_xrstor_fault, "XRSTOR", "fault", NULL);
	}
	pr_info("x86/fpu:##################################################################\n");
}

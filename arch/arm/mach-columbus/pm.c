#include <linux/io.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <asm/system_misc.h>
#include <asm/suspend.h>
#include <mach/columbus.h>
#include <mach/spc.h>

static int tc2_finisher(unsigned long arg)
{
	flush_cache_all();
	spc_powerdown_enable(0, 1);
	spc_powerdown_enable(1, 1);
	scc_ctl_snoops(0, 0);
	scc_ctl_snoops(1, 0);
	writel(0x0, COLUMBUS_CCI400_VIRT_BASE + COLUMBUS_CCI400_EAG_OFFSET);
	writel(0x0, COLUMBUS_CCI400_VIRT_BASE + COLUMBUS_CCI400_KF_OFFSET);
	dsb();
	wfi();
	return 1;
}

static int tc2_pm_begin(suspend_state_t state)
{
	disable_hlt();
	return 0;
}

static void tc2_pm_end(void)
{
	enable_hlt();
	return;
}

static void tc2_pm_finish(void)
{
}

static int tc2_pm_enter(suspend_state_t suspend_state)
{
	int ret;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = cpu_suspend(0, tc2_finisher);
		break;
	default:
		ret = -EINVAL;
	}

	spc_powerdown_enable(0, 0);
	spc_powerdown_enable(1, 0);
	scc_ctl_snoops(0, 1);
	scc_ctl_snoops(1, 1);
	writel(0x1, COLUMBUS_CCI400_VIRT_BASE + COLUMBUS_CCI400_EAG_OFFSET);
	writel(0x1, COLUMBUS_CCI400_VIRT_BASE + COLUMBUS_CCI400_KF_OFFSET);

	return ret;
}
static const struct platform_suspend_ops tc2_pm_ops = {
	.begin		= tc2_pm_begin,
	.end		= tc2_pm_end,
	.enter		= tc2_pm_enter,
	.finish		= tc2_pm_finish,
	.valid		= suspend_valid_only_mem,
};
static int __init tc2_common_pm_late_init(void)
{

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&tc2_pm_ops);
#endif

	return 0;
}
late_initcall(tc2_common_pm_late_init);

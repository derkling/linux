#include <linux/module.h>
#include <linux/suspend.h>

static int vexpress_pm_prepare(void)
{
	printk(KERN_CRIT "--- %s\n", __func__);

	disable_hlt();

	return 0;
}

static int vexpress_pm_suspend(void)
{
	printk(KERN_CRIT "--- %s\n", __func__);

	return 0;
}

static int vexpress_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		printk(KERN_CRIT "--- %s: state %d\n", __func__, state);
		ret = vexpress_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void vexpress_pm_finish(void)
{
	printk(KERN_CRIT "--- %s\n", __func__);

	enable_hlt();
}

static struct platform_suspend_ops vexpress_pm_ops = {
	.prepare = vexpress_pm_prepare,
	.enter = vexpress_pm_enter,
	.finish = vexpress_pm_finish,
	.valid = suspend_valid_only_mem,
};

static int __init vexpress_pm_init(void)
{
	suspend_set_ops(&vexpress_pm_ops);

	return 0;
}
late_initcall(vexpress_pm_init);

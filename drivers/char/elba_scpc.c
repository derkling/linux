/*
 *  linux/drivers/char/elba_scpc.c
 *
 *  Copyright (C) 2010  ARM Ltd
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <asm/hardware/elba_scpc.h>

static struct scpc_info *info;

int hip_set_power_state(int dev, int dev_state, int cluster_state, int time_unit, int time)
{
	/* Parameters checking.  */
	if (dev >= DeviceNum) {
		printk(KERN_ERR "Selected wrong DEV: %x\n", dev);
		return -ENXIO;
	}

	if (dev_state >= StateNum) {
		printk(KERN_ERR "Selected wrong DEV_STATE: %x\n", dev_state);
		return -ENXIO;
	}

	if (cluster_state >= StateNum) {
		printk(KERN_ERR "Selected wrong CLUSTER_STATE: %x\n", cluster_state);
		return -ENXIO;
	}

	if (time_unit >= Units) {
		printk(KERN_ERR "Selected wrong TIME_UNIT: %x\n", time_unit);
		return -ENXIO;
	}

	if (time == Unknown) {
		printk(KERN_ERR "Selected wrong TIME: %x\n", time);
		return -ENXIO;
	}

	spin_lock(&info->lock);
	iowrite32(PWR_STATE(dev, dev_state, cluster_state, time_unit, time), info->base + spc_hip_control);
	spin_unlock(&info->lock);

	return 0;
}
EXPORT_SYMBOL(hip_set_power_state);

int mid_set_power_state(int dev, int dev_state, int cluster_state, int time_unit, int time)
{
	/* Parameters checking.  */
	if (dev >= DeviceNum) {
		printk(KERN_ERR "Selected wrong DEV: %x\n", dev);
		return -ENXIO;
	}

	if (dev_state >= StateNum) {
		printk(KERN_ERR "Selected wrong DEV_STATE: %x\n", dev_state);
		return -ENXIO;
	}

	if (cluster_state >= StateNum) {
		printk(KERN_ERR "Selected wrong CLUSTER_STATE: %x\n", cluster_state);
		return -ENXIO;
	}

	if (time_unit >= Units) {
		printk(KERN_ERR "Selected wrong TIME_UNIT: %x\n", time_unit);
		return -ENXIO;
	}

	if (time == Unknown) {
		printk(KERN_ERR "Selected wrong TIME: %x\n", time);
		return -ENXIO;
	}

	spin_lock(&info->lock);
	iowrite32(PWR_STATE(dev, dev_state, cluster_state, time_unit, time), info->base + spc_mid_control);
	spin_unlock(&info->lock);

	return 0;
}
EXPORT_SYMBOL(mid_set_power_state);

int hip_set_performance(int perf)
{
	/* Parameters checking.  */
	if ((perf < 0) || (perf > 200)) {
		printk(KERN_ERR "performance is not in the correct range (0-200)\n");
		return -ENXIO;
	}

	spin_lock(&info->lock);
	iowrite32(PERF(perf), info->base + spc_hip_control);
	spin_unlock(&info->lock);

	return 0;
}
EXPORT_SYMBOL(hip_set_performance);

int mid_set_performance(int perf)
{
	if ((perf < 0) || (perf > 200)) {
		printk(KERN_ERR "performance is not in the correct range (0-200)\n");
		return -ENXIO;
	}

	spin_lock(&info->lock);
	iowrite32(PERF(perf), info->base + spc_mid_control);
	spin_unlock(&info->lock);

	return 0;
}
EXPORT_SYMBOL(mid_set_performance);

int set_wakeup_int(int devs)
{
	if (devs & ~0x7F) {
		printk(KERN_ERR "Selected wrong BITMASK: %x\n", devs);
		return -ENXIO;
	}

	spin_lock(&info->lock);
	iowrite32(devs, info->base + spc_wake_int_mask);
	spin_unlock(&info->lock);

	return 0;
}
EXPORT_SYMBOL(set_wakeup_int);

int hip_set_auto_shutdown(int cpu)
{
	if (cpu & ~0x3) {
		printk(KERN_ERR "Wrong set-up: %x\n", cpu);
		return -EINVAL;
	}

	spin_lock(&info->lock);
	iowrite32(1 << cpu, info->base + spc_hip_auto_shutdown);
	spin_unlock(&info->lock);

	return 0;
}
EXPORT_SYMBOL(hip_set_auto_shutdown);

int hip_force_wakeup(int cpu)
{
	unsigned long flags;
	if (cpu & ~0x3) {
		printk(KERN_ERR "Wrong set-up: %x\n", cpu);
		return -EINVAL;
	}

	spin_lock_irqsave(&info->lock, flags);
	iowrite32(0x3 << (cpu*2), info->base + spc_hip_force_wakeup);
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}
EXPORT_SYMBOL(hip_force_wakeup);

static int __devinit scpc_driver_init_one(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	int ret = 0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "unable to allocate mem\n");
		ret = -ENOMEM;
		goto out;
	}

	if (!request_region(res->start, resource_size(res), DRIVER_NAME)) {
		printk(KERN_ERR DRIVER_NAME ": address 0x%04x already in use\n", res->start);
		ret = -EBUSY;
		goto reqres_err;
	}

	info->base = ioremap(res->start, resource_size(res));
	if (!info->base) {
		ret = -ENXIO;
		goto ioremap_err;
	}

	/* Init lock.  */
	spin_lock_init(&info->lock);

	platform_set_drvdata(pdev, info);
	printk(KERN_INFO "elba-scpc: loaded at %p (size %d)\n", info->base, resource_size(res));

out:
	return ret;

ioremap_err:
	release_region(res->start, resource_size(res));

reqres_err:
	kfree(info);
	info = NULL;
	goto out;
}

static int __devexit scpc_driver_remove(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;

	iounmap(info->base);
	release_region(res->start, resource_size(res));
	kfree(info);
	info = NULL;

	return 0;
}

static struct platform_driver scpc_platform_driver ={
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
	},
	.probe  = scpc_driver_init_one,
	.remove = scpc_driver_remove,
};

#if defined (CONFIG_DEBUG_FS)
/* Root dentry.  */
static struct dentry *scpc_root;
/* HIP set power  */
static struct dentry *scpc_hip_power_file;
static long scpc_hip_power_value;
/* MID set power  */
static struct dentry *scpc_mid_power_file;
static long scpc_mid_power_value;
/* HIP set perforamnce.  */
static struct dentry *scpc_hip_perf_file;
static long scpc_hip_perf_value;
/* MID set perforamnce.  */
static struct dentry *scpc_mid_perf_file;
static long scpc_mid_perf_value;
/* wakeup interrupt mask  */
static struct dentry *scpc_wakeup_int_file;
static long scpc_wakeup_int_value;

static ssize_t hip_power_read(struct file *file, char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "HIP Power: 0x%08lX\n", scpc_hip_power_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);
}

static ssize_t hip_power_write(struct file *file, const char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	scpc_hip_power_value = simple_strtol(buffer, NULL, 16);

	// hip_set_power_state(scpc_hip_power_value);

	return len;
}

static ssize_t mid_power_read(struct file *file, char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "MID Power: 0x%08lX\n", scpc_mid_power_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);
}

static ssize_t mid_power_write(struct file *file, const char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	scpc_mid_power_value = simple_strtol(buffer, NULL, 16);

	// mid_set_power_state(scpc_mid_power_value);

	return len;
}

static ssize_t hip_performance_read(struct file *file, char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "HIP Performance reg: 0x%08lX\n", scpc_hip_perf_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);
}

static ssize_t hip_performance_write(struct file *file, const char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	scpc_hip_perf_value = simple_strtol(buffer, NULL, 10);

	// hip_set_performance(scpc_hip_perf_value);

	return len;
}

static ssize_t mid_performance_read(struct file *file, char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "MID Performance reg: 0x%08lX\n", scpc_mid_perf_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);
}

static ssize_t mid_performance_write(struct file *file, const char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	scpc_mid_perf_value = simple_strtol(buffer, NULL, 10);

	// mid_set_performance(scpc_mid_perf_value);

	return len;
}

static ssize_t wakeup_int_read(struct file *file, char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "Wakeup Interrupt mask: 0x%08lX\n", scpc_wakeup_int_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);
}

static ssize_t wakeup_int_write(struct file *file, const char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	scpc_wakeup_int_value = simple_strtol(buffer, NULL, 16);

	// set_wakeup_int(scpc_wakeup_int_value);

	return len;
}

static const struct file_operations hip_power_fops = {
	.owner   = THIS_MODULE,
	.read    = hip_power_read,
	.write   = hip_power_write,
};

static const struct file_operations mid_power_fops = {
	.owner   = THIS_MODULE,
	.read    = mid_power_read,
	.write   = mid_power_write,
};

static const struct file_operations hip_perf_fops = {
	.owner   = THIS_MODULE,
	.read    = hip_performance_read,
	.write   = hip_performance_write,
};

static const struct file_operations mid_perf_fops = {
	.owner   = THIS_MODULE,
	.read    = mid_performance_read,
	.write   = mid_performance_write,
};

static const struct file_operations wakeup_int_fops = {
	.owner   = THIS_MODULE,
	.read    = wakeup_int_read,
	.write   = wakeup_int_write,
};

void scpc_create_debugfs(void)
{
	scpc_root = debugfs_create_dir("scpc", NULL);

	scpc_hip_power_file  = debugfs_create_file("hip_power",
						   S_IWUGO,
						   scpc_root,
						   &scpc_hip_power_value,
						   &hip_power_fops);

	scpc_mid_power_file  = debugfs_create_file("mid_power",
						   S_IWUGO,
						   scpc_root,
						   &scpc_mid_power_value,
						   &mid_power_fops);

	scpc_hip_perf_file   = debugfs_create_file("hip_performance",
						   S_IWUGO,
						   scpc_root,
						   &scpc_hip_perf_value,
						   &hip_perf_fops);

	scpc_mid_perf_file   = debugfs_create_file("mid_performance",
						   S_IWUGO,
						   scpc_root,
						   &scpc_mid_perf_value,
						   &mid_perf_fops);

	scpc_wakeup_int_file = debugfs_create_file("wakeup_int_mask",
						   S_IWUGO,
						   scpc_root,
						   &scpc_wakeup_int_value,
						   &wakeup_int_fops);
}

void scpc_remove_debugfs(void)
{
	if (scpc_root) {
		debugfs_remove(scpc_hip_power_file);
		debugfs_remove(scpc_mid_power_file);
		debugfs_remove(scpc_hip_perf_file);
		debugfs_remove(scpc_mid_perf_file);
		debugfs_remove(scpc_wakeup_int_file);
		debugfs_remove(scpc_root);
		scpc_root = 0;
	}
}
#endif


static int __init scpc_init(void)
{
	int ret;

	ret = platform_driver_register(&scpc_platform_driver);

#if defined (CONFIG_DEBUG_FS)
	scpc_create_debugfs();
#endif

	return ret;
}

static void __exit scpc_exit(void)
{
#if defined (CONFIG_DEBUG_FS)
	scpc_remove_debugfs();
#endif

	platform_driver_unregister(&scpc_platform_driver);
}

module_init(scpc_init);
module_exit(scpc_exit);
MODULE_AUTHOR("Giuseppe Calderaro <giuseppe.calderaro@arm.com>");
MODULE_LICENSE("GPL");

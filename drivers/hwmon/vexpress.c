#define DRVNAME "vexpress-hwmon"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#if 0	/* HACK begin */
#include <linux/vexpress.h>
#else
#include <linux/init.h>
#include <linux/slab.h>
//#include <../arch/arm/mach-vexpress/include/mach/motherboard.h>

#define VEXPRESS_CONFIG_FUNC_AMP       3
#define VEXPRESS_CONFIG_FUNC_TEMP      4
#define VEXPRESS_CONFIG_FUNC_POWER     12
#define VEXPRESS_CONFIG_FUNC_ENERGY    13

extern int v2m_cfg_read(u32 devfn, u32 *data);

struct vexpress_config_device {
	u8 func;
	u8 site;
	u16 device;
	const char *label;
};

int vexpress_config_read(struct vexpress_config_device *vecdev,
		int offset, u32 *data)
{
	u32 devfn = (vecdev->func << 20) | (vecdev->site << 16) |
			((vecdev->device + offset) << 0);
	return v2m_cfg_read(devfn, data);
}
#endif	/* HACK end */

static struct device *vexpress_hwmon_dev;
static int vexpress_hwmon_dev_refcount;
static DEFINE_SPINLOCK(vexpress_hwmon_dev_lock);

static ssize_t vexpress_hwmon_name_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	return sprintf(buffer, "%s\n", DRVNAME);
}

static struct device_attribute vexpress_hwmon_name_attr =
		__ATTR(name, 0444, vexpress_hwmon_name_show, NULL);

struct vexpress_hwmon_attrs {
	struct vexpress_config_device *vecdev;
	const char *label;
	struct device_attribute label_attr;
	char label_name[16];
	struct device_attribute input_attr;
	char input_name[16];
};

static ssize_t vexpress_hwmon_label_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct vexpress_hwmon_attrs *attrs = container_of(dev_attr,
			struct vexpress_hwmon_attrs, label_attr);

	return snprintf(buffer, PAGE_SIZE, "%s\n", attrs->label);
}

static ssize_t vexpress_hwmon_u32_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct vexpress_hwmon_attrs *attrs = container_of(dev_attr,
			struct vexpress_hwmon_attrs, input_attr);
	int err;
	u32 value;

	err = vexpress_config_read(attrs->vecdev, 0, &value);
	if (err)
		return err;

	return snprintf(buffer, PAGE_SIZE, "%u\n", value);
}

static ssize_t vexpress_hwmon_u64_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct vexpress_hwmon_attrs *attrs = container_of(dev_attr,
			struct vexpress_hwmon_attrs, input_attr);
	int err;
	u32 value_hi, value_lo;

	err = vexpress_config_read(attrs->vecdev, 0, &value_lo);
	if (err)
		return err;

	err = vexpress_config_read(attrs->vecdev, 1, &value_hi);
	if (err)
		return err;

	return snprintf(buffer, PAGE_SIZE, "%llu\n",
			((u64)value_hi << 32) | value_lo);
}

static struct device *vexpress_hwmon_dev_get(void)
{
	struct device *result;

	spin_lock(&vexpress_hwmon_dev_lock);

	if (vexpress_hwmon_dev) {
		result = vexpress_hwmon_dev;
	} else {
		int err;

		result = hwmon_device_register(NULL);
		if (IS_ERR(result))
			goto out;

		err = device_create_file(result, &vexpress_hwmon_name_attr);
		if (err) {
			result = ERR_PTR(err);
			hwmon_device_unregister(result);
			goto out;
		}

		vexpress_hwmon_dev = result;
	}

	vexpress_hwmon_dev_refcount++;

out:
	spin_unlock(&vexpress_hwmon_dev_lock);

	return result;
}

static void vexpress_hwmon_dev_put(void)
{
	spin_lock(&vexpress_hwmon_dev_lock);

	if (--vexpress_hwmon_dev_refcount == 0) {
		vexpress_hwmon_dev = NULL;
		hwmon_device_unregister(vexpress_hwmon_dev);
	}

	WARN_ON(vexpress_hwmon_dev_refcount < 0);

	spin_unlock(&vexpress_hwmon_dev_lock);
}

static struct {
	const char *name;
	atomic_t index;
} vexpress_hwmon_func[] = {
	[VEXPRESS_CONFIG_FUNC_AMP] = { "curr" },
	[VEXPRESS_CONFIG_FUNC_TEMP] = { "temp" },
	[VEXPRESS_CONFIG_FUNC_POWER] = { "power" },
	[VEXPRESS_CONFIG_FUNC_ENERGY] = { "energy" },
};

static int vexpress_hwmon_probe(struct vexpress_config_device *vecdev)
{
	int err;
	struct device *hwmon_dev;
	struct vexpress_hwmon_attrs *attrs;
	const char *attr_name;
	int attr_index;

	hwmon_dev = vexpress_hwmon_dev_get();
	if (IS_ERR(hwmon_dev)) {
		err = PTR_ERR(hwmon_dev);
		goto error_hwmon_dev_get;
	}

#if 0	/* HACK begin */
	attrs = devm_kzalloc(&vecdev->dev, sizeof(*attrs), GFP_KERNEL);
#else
	attrs = kzalloc(sizeof(*attrs), GFP_KERNEL);
#endif	/* HACK end */
	if (!attrs) {
		err = -ENOMEM;
		goto error_kzalloc;
	}
	attrs->vecdev = vecdev;

#if 0	/* HACK begin */
	err = sysfs_create_link(&vecdev->dev.kobj, &hwmon_dev->kobj, "hwmon");
	if (err)
		goto error_create_link;
#endif	/* HACK end */

	attr_index = atomic_inc_return(&vexpress_hwmon_func[vecdev->func].index);
	attr_name = vexpress_hwmon_func[vecdev->func].name;

	snprintf(attrs->input_name, sizeof(attrs->input_name),
			"%s%d_input", attr_name, attr_index);
	attrs->input_attr.attr.name = attrs->input_name;
	attrs->input_attr.attr.mode = 0444;
	if (vecdev->func == VEXPRESS_CONFIG_FUNC_ENERGY)
		attrs->input_attr.show = vexpress_hwmon_u64_show;
	else
		attrs->input_attr.show = vexpress_hwmon_u32_show;
	sysfs_attr_init(&attrs->input_attr.attr);
	err = device_create_file(hwmon_dev, &attrs->input_attr);
	if (err)
		goto error_create_input;

#if 0	/* HACK begin */
	attrs->label = of_get_property(vecdev->dev.of_node, "label", NULL);
#else
	attrs->label = vecdev->label;
#endif	/* HACK end */
	if (attrs->label) {
		snprintf(attrs->label_name, sizeof(attrs->label_name),
				"%s%d_label", attr_name, attr_index);
		attrs->label_attr.attr.name = attrs->label_name;
		attrs->label_attr.attr.mode = 0444;
		attrs->label_attr.show = vexpress_hwmon_label_show;
		sysfs_attr_init(&attrs->label_attr.attr);
		err = device_create_file(hwmon_dev, &attrs->label_attr);
		if (err)
			goto error_create_label;
	}

#if 0	/* HACK begin */
	vexpress_config_set_drvdata(vecdev, attrs);
#endif	/* HACK end */

	return 0;

error_create_label:
	device_remove_file(hwmon_dev, &attrs->input_attr);
error_create_input:
#if 0	/* HACK begin */
	sysfs_remove_link(&vecdev->dev.kobj, "hwmon");
error_create_link:
#endif	/* HACK end */
error_kzalloc:
	vexpress_hwmon_dev_put();
error_hwmon_dev_get:
	return err;
}

#if 0	/* HACK begin */
static int __devexit vexpress_hwmon_remove(struct vexpress_config_device
		*vecdev)
{
	struct vexpress_hwmon_attrs *attrs =
			vexpress_config_get_drvdata(vecdev);

	if (attrs->label)
		device_remove_file(vexpress_hwmon_dev, &attrs->label_attr);
	device_remove_file(vexpress_hwmon_dev, &attrs->input_attr);
	atomic_dec(&vexpress_hwmon_func[vecdev->func].index);
	sysfs_remove_link(&vecdev->dev.kobj, "hwmon");
	vexpress_hwmon_dev_put();

	return 0;
}

static const unsigned vexpress_hwmon_funcs[] = {
	VEXPRESS_CONFIG_FUNC_AMP,
	VEXPRESS_CONFIG_FUNC_TEMP,
	VEXPRESS_CONFIG_FUNC_POWER,
	VEXPRESS_CONFIG_FUNC_ENERGY,
	0,
};

static struct vexpress_config_driver vexpress_hwmon_driver = {
	.funcs = vexpress_hwmon_funcs,
	.probe = vexpress_hwmon_probe,
	.remove = __devexit_p(vexpress_hwmon_remove),
	.driver	= {
		.name = DRVNAME,
		.owner = THIS_MODULE,
	},
};

static int __init vexpress_hwmon_init(void)
{
	return vexpress_config_driver_register(&vexpress_hwmon_driver);
}

static void __exit vexpress_hwmon_exit(void)
{
	vexpress_config_driver_unregister(&vexpress_hwmon_driver);
}

MODULE_AUTHOR("Pawel Moll <pawel.moll@arm.com>");
MODULE_DESCRIPTION("Versatile Express hwmon");
MODULE_LICENSE("GPL");

module_init(vexpress_hwmon_init);
module_exit(vexpress_hwmon_exit);
#else
static struct vexpress_config_device vexpress_hwmon_devices[] = {
	{
		.func = VEXPRESS_CONFIG_FUNC_POWER,
		.site = 1,
		.device = 0,
		.label = "A15",
	}, {
		.func = VEXPRESS_CONFIG_FUNC_POWER,
		.site = 1,
		.device = 1,
		.label = "A7",
	}, {
		.func = VEXPRESS_CONFIG_FUNC_ENERGY,
		.site = 1,
		.device = 0,
		.label = "A15",
	}, {
		.func = VEXPRESS_CONFIG_FUNC_ENERGY,
		.site = 1,
		.device = 2,
		.label = "A7",
	},
};

static int __init vexpress_hwmon_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vexpress_hwmon_devices); i++)
		vexpress_hwmon_probe(&vexpress_hwmon_devices[i]);

	return 0;
}
device_initcall(vexpress_hwmon_init);
#endif	/* HACK end */

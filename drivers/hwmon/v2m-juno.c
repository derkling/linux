/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2014 ARM Limited
 */

#define DRVNAME "v2m-juno-hwmon"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>

struct v2m_juno_hwmon_data {
	struct device *hwmon_dev;
	struct regmap *reg;
	u32 offset;
	u32 mult;
	u32 div;
};

static ssize_t v2m_juno_hwmon_label_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	const char *label = of_get_property(dev->of_node, "label", NULL);

	return snprintf(buffer, PAGE_SIZE, "%s\n", label);
}

static ssize_t v2m_juno_hwmon_u32_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct v2m_juno_hwmon_data *data = dev_get_drvdata(dev);
	int err;
	u32 value;

	err = regmap_read(data->reg, data->offset, &value);
	if (err)
		return err;

	value *= data->mult;
	value /= data->div;

	return snprintf(buffer, PAGE_SIZE, "%u\n", value);
}

static ssize_t v2m_juno_hwmon_u64_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct v2m_juno_hwmon_data *data = dev_get_drvdata(dev);
	int err;
	u32 value_hi, value_lo;
	u64 value;

	err = regmap_read(data->reg, data->offset, &value_lo);
	if (err)
		return err;

	err = regmap_read(data->reg, data->offset + 4, &value_hi);
	if (err)
		return err;

	value = ((u64)value_hi << 32) | value_lo;
	value *= data->mult;
	value = div_u64(value, data->div);

	return snprintf(buffer, PAGE_SIZE, "%llu\n", value);
}

static umode_t v2m_juno_hwmon_attr_is_visible(struct kobject *kobj,
		struct attribute *attr, int index)
{
	struct device *dev = kobj_to_dev(kobj);
	struct device_attribute *dev_attr = container_of(attr,
				struct device_attribute, attr);

	if (dev_attr->show == v2m_juno_hwmon_label_show &&
			!of_get_property(dev->of_node, "label", NULL))
		return 0;

	return attr->mode;
}

struct v2m_juno_hwmon_type {
	const char *name;
	const struct attribute_group **attr_groups;
};

#define V2M_JUNO_HWMON(_name, _attr_name, _is_u64)		\
static inline ssize_t 						\
_attr_name##_label_show(struct device *dev,			\
		struct device_attribute *attr, char *buf)	\
{								\
	return v2m_juno_hwmon_label_show(dev, attr, buf);	\
}								\
static inline ssize_t 						\
_attr_name##_input_show(struct device *dev,			\
		struct device_attribute *attr, char *buf)	\
{								\
	if (_is_u64)						\
		return v2m_juno_hwmon_u64_show(dev, attr, buf);	\
	else							\
		return v2m_juno_hwmon_u32_show(dev, attr, buf);	\
}								\
static DEVICE_ATTR_RO(_attr_name##_label);			\
static DEVICE_ATTR_RO(_attr_name##_input);			\
static struct attribute *v2m_juno_hwmon_attrs_##_name[] = {	\
	&dev_attr_##_attr_name##_label.attr,			\
	&dev_attr_##_attr_name##_input.attr,			\
	NULL							\
};								\
static struct attribute_group v2m_juno_hwmon_group_##_name = {	\
	.is_visible = v2m_juno_hwmon_attr_is_visible,		\
	.attrs = v2m_juno_hwmon_attrs_##_name,			\
};								\
static const							\
struct attribute_group *v2m_juno_hwmon_groups_##_name[] = { 	\
	&v2m_juno_hwmon_group_##_name,				\
	NULL,							\
};								\
static struct v2m_juno_hwmon_type v2m_juno_hwmon_##_name = {	\
	.name = "v2m_juno_"__stringify(_name),			\
	.attr_groups = v2m_juno_hwmon_groups_##_name,		\
};

V2M_JUNO_HWMON(volt, in1, false);
V2M_JUNO_HWMON(amp, curr1, false);
V2M_JUNO_HWMON(power, power1, false);
V2M_JUNO_HWMON(energy, energy1, true);

static const struct of_device_id v2m_juno_hwmon_of_match[] = {
	{
		.compatible = "arm,vexpress-volt",
		.data = &v2m_juno_hwmon_volt,
	}, {
		.compatible = "arm,vexpress-amp",
		.data = &v2m_juno_hwmon_amp,
	}, {
		.compatible = "arm,vexpress-power",
		.data = &v2m_juno_hwmon_power,
	}, {
		.compatible = "arm,vexpress-energy",
		.data = &v2m_juno_hwmon_energy,
	},
	{}
};
MODULE_DEVICE_TABLE(of, v2m_juno_hwmon_of_match);

static int v2m_juno_hwmon_probe(struct platform_device *pdev)
{
	struct device *parent, *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct v2m_juno_hwmon_data *data;
	const struct of_device_id *match;
	const struct v2m_juno_hwmon_type *type;

	parent = dev->parent;
	if (!parent)
		return -ENODEV;

	match = of_match_device(v2m_juno_hwmon_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;
	type = match->data;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	platform_set_drvdata(pdev, data);

	if (!strcmp(match->compatible, "arm, vexpress-energy"))
		data->mult = 100;
	else
		data->mult = 1000;
	if (of_property_read_u32(np, "offset", &data->offset))
		return -EINVAL;
	if (of_property_read_u32(np, "divisor", &data->div))
		return -EINVAL;

	data->reg = syscon_node_to_regmap(parent->of_node);
	if (IS_ERR(data->reg))
		return PTR_ERR(data->reg);

	data->hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
			type->name, data, type->attr_groups);

	return PTR_ERR_OR_ZERO(data->hwmon_dev);
}

static struct platform_driver v2m_juno_hwmon_driver = {
	.probe = v2m_juno_hwmon_probe,
	.driver	= {
		.name = DRVNAME,
		.of_match_table = v2m_juno_hwmon_of_match,
	},
};

module_platform_driver(v2m_juno_hwmon_driver);

MODULE_AUTHOR("Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>");
MODULE_DESCRIPTION("V2M Juno hwmon sensors driver");
MODULE_LICENSE("GPL");

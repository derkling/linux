/*
 *  Copyright (c) 2012 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpumask.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#define PVT_CFGREG01 0x00
#define PVT_CFGREG02 0x04
#define PVT_CFGREG03 0x08
#define PVT_CFGREG04 0x0C
#define PVT_CFGREG05 0x10
#define PVT_CFGREG06 0x14
#define PVT_CFGREG07 0x18

/* PVT_CFGREG01 */
#define SLEEP_EN_09N	0x00001
#define ADC_EN_09	0x00010
#define ATB_EN_09	0x00040
#define PM_VBGR_EN_09	0x10000

/* PVT_CFGREG02 */
#define ATB_V_CFG_09	0x001
#define ATB_SEL_09_BG	0x300

/* PVT_CFGREG05 */
#define ADC_VIN_SEL_TEMP 0x00001
#define ADC_START	 0x10000

/* PVT_CFGREG06 */
#define ADC_DOUT_09_MASK 0x3FF00

/* PVT_CFGREG07 */
#define PM_VBGR_CTRIM_09 0x002100
#define PM_VBGR_PTRIM_09 0x1D0000

#define IDLE_INTERVAL 10000

#define COOLING_DEV_MAX 8
#define CPUFREQ_CDEV "thermal-cpufreq-"

struct trip_point {
	unsigned long temp;
	enum thermal_trip_type type;
	int num_cdevs;
	struct cpumask cdev_clip_cpus[COOLING_DEV_MAX];
};

struct trip_data {
	int num_trips;
	struct trip_point *trip_points;
};

struct arm_pvt_thermal_dev {
	void __iomem *baseaddr;
	struct thermal_zone_device *tz;
	struct mutex lock;
	struct trip_data *trip_data;
	int num_cdevs;
	struct thermal_cooling_device *cdev[COOLING_DEV_MAX];
	enum thermal_device_mode mode;
};

static struct arm_pvt_thermal_dev *sensor_data;

static inline u32 arm_pvt_read_reg(u32 offset)
{
	return readl_relaxed(sensor_data->baseaddr + offset);
}

static inline void arm_pvt_write_reg(u32 val, u32 offset)
{
	writel(val, sensor_data->baseaddr + offset);
}

static inline void sensor_init(void)
{
	/*
	 * Disable sleep mode(0), Enable ADC(4), Analog Test Bus (ATB) Test
	 * Point(6), BANDGAP(16)
	 */
	arm_pvt_write_reg(SLEEP_EN_09N | ADC_EN_09 | ATB_EN_09 | PM_VBGR_EN_09,
			PVT_CFGREG01);

	/* Mux Select: Diff BANDGAP */
	arm_pvt_write_reg(ATB_V_CFG_09 | ATB_SEL_09_BG, PVT_CFGREG02);

	/* Temperature */
	arm_pvt_write_reg(ADC_VIN_SEL_TEMP | ADC_START, PVT_CFGREG05);

	/* BANDGAP trim defaults */
	arm_pvt_write_reg(PM_VBGR_CTRIM_09 | PM_VBGR_PTRIM_09, PVT_CFGREG07);
}

static bool arm_pvt_thermal_match_dev(struct thermal_cooling_device *cdev,
				struct trip_point *trip_point)
{
	return trip_point->type == THERMAL_TRIP_ACTIVE	&&
		!strncmp(cdev->type, CPUFREQ_CDEV, strlen(CPUFREQ_CDEV)) ;
}

static inline int cluster_to_cpumask(struct device_node *node,
				struct cpumask *cpusmask)
{
	int ret, cluster;

	ret = of_property_read_u32(node, "reg", &cluster);
	if (ret)
		return ret;

	return cluster_to_logical_mask(cluster, cpusmask);
}

static int arm_pvt_bind(struct thermal_zone_device *tz,
			struct thermal_cooling_device *cdev)
{
	struct arm_pvt_thermal_dev *th_sensor = tz->devdata;
	struct trip_data *trip_data = th_sensor->trip_data;
	int i, ret = -EINVAL;

	for (i = 0; i < trip_data->num_trips; i++) {
		if (!arm_pvt_thermal_match_dev(cdev, &trip_data->trip_points[i]))
			continue;

		ret = thermal_zone_bind_cooling_device(tz, i, cdev,
						THERMAL_NO_LIMIT, THERMAL_NO_LIMIT);

		dev_info(&cdev->device, "%s bind to thermal zone %d: %s\n",
			cdev->type, i, ret ? "failed" : "succeeded");
	}

	return ret;
}

static int arm_pvt_unbind(struct thermal_zone_device *tz,
			struct thermal_cooling_device *cdev)
{
	struct arm_pvt_thermal_dev *th_sensor = tz->devdata;
	struct trip_data *trip_data = th_sensor->trip_data;
	int i, ret = -EINVAL;

	for (i = 0; i < trip_data->num_trips; i++) {
		if (!arm_pvt_thermal_match_dev(cdev, &trip_data->trip_points[i]))
			continue;

		ret = thermal_zone_unbind_cooling_device(tz, i, cdev);

		dev_info(&cdev->device, "%s unbind from thermal zone %d: %s\n",
			cdev->type, i, ret ? "failed" : "succeeded");
	}

	return ret;
}

static int arm_pvt_get_temp(struct thermal_zone_device *tz,
			unsigned long *temp)
{
	u32 val;

	val = arm_pvt_read_reg(PVT_CFGREG06);

	/*
	 *  The following formula was arrived at by calibrating the sensor as
	 *  described in the sensor datasheet.
	 */
	*temp = (((val & ADC_DOUT_09_MASK) >> 8) - 0x114) * 1155 + 25000;

	return 0;
}

static int arm_pvt_get_mode(struct thermal_zone_device *tz,
			enum thermal_device_mode *mode)
{
	struct arm_pvt_thermal_dev *th_sensor = tz->devdata;

	*mode = th_sensor->mode;
	return 0;
}

static int arm_pvt_set_mode(struct thermal_zone_device *tz,
			enum thermal_device_mode mode)
{
	struct arm_pvt_thermal_dev *th_sensor = tz->devdata;

	mutex_lock(&tz->lock);

	if (mode == THERMAL_DEVICE_ENABLED)
		tz->polling_delay = IDLE_INTERVAL;
	else
		tz->polling_delay = 0;

	mutex_unlock(&tz->lock);

	if (mode != th_sensor->mode) {
		th_sensor->mode = mode;
		thermal_zone_device_update(tz);
		pr_debug("thermal polling set for duration=%d msec\n",
			tz->polling_delay);
	}

	return 0;
}

static int arm_pvt_get_trip_type(struct thermal_zone_device *tz, int trip,
				enum thermal_trip_type *type)
{
	struct arm_pvt_thermal_dev *th_sensor = tz->devdata;
	struct trip_data *trip_data = th_sensor->trip_data;

	if (trip >= trip_data->num_trips)
		return -EINVAL;

	*type = trip_data->trip_points[trip].type;

	return 0;
}

static int arm_pvt_get_trip_temp(struct thermal_zone_device *tz, int trip,
				unsigned long *temp)
{
	struct arm_pvt_thermal_dev *th_sensor = tz->devdata;
	struct trip_data *trip_data = th_sensor->trip_data;

	if (trip >= trip_data->num_trips)
		return -EINVAL;

	*temp = trip_data->trip_points[trip].temp;

	return 0;
}

static struct thermal_zone_device_ops tz_ops = {
	.bind = arm_pvt_bind,
	.unbind = arm_pvt_unbind,
	.get_temp = arm_pvt_get_temp,
	.get_mode = arm_pvt_get_mode,
	.set_mode = arm_pvt_set_mode,
	.get_trip_type = arm_pvt_get_trip_type,
	.get_trip_temp = arm_pvt_get_trip_temp,
};

static struct trip_data *arm_pvt_parse_dt(struct platform_device *pdev)
{
	struct trip_data *trips;
	struct trip_point *trip_point;
	struct device_node *np = pdev->dev.of_node, *trip_np, *cdev_np;
	const char *type;
	u32 temp;
	int i = 0, j;

	if (!np)
		return NULL;

	trips = devm_kzalloc(&pdev->dev, sizeof(*trips), GFP_KERNEL);
	if (!trips)
		return NULL;

	trips->num_trips = of_get_child_count(np);
	trips->trip_points = devm_kzalloc(&pdev->dev,
					sizeof(*trip_point) * trips->num_trips,	GFP_KERNEL);

	if (!trips->trip_points)
		return NULL;

	for_each_child_of_node(np, trip_np) {
		trip_point = &trips->trip_points[i++];

		if (of_property_read_u32(trip_np, "temp", &temp))
			goto err_parse_dt;
		if (temp < 0 || temp > 125000) {
			dev_err(&pdev->dev, "Sensor doesn't support temp=%d\n", temp);
			goto err_parse_dt;
		}

		trip_point->temp = temp;

		if (of_property_read_string(trip_np, "type", &type))
			goto err_parse_dt;

		if (!strcmp(type, "active"))
			trip_point->type = THERMAL_TRIP_ACTIVE;
		else if (!strcmp(type, "passive"))
			trip_point->type = THERMAL_TRIP_PASSIVE;
		else if (!strcmp(type, "hot"))
			trip_point->type = THERMAL_TRIP_HOT;
		else if (!strcmp(type, "critical"))
			trip_point->type = THERMAL_TRIP_CRITICAL;
		else {
			dev_warn(&pdev->dev, "Un-recognized trip type.\n");
			goto err_parse_dt;
		}

		trip_point->num_cdevs = of_get_child_count(trip_np);

		if (trip_point->num_cdevs > COOLING_DEV_MAX) {
			dev_warn(&pdev->dev, "Too many cooling devices.\n");
			goto err_parse_dt;
		}

		j = 0;
		for_each_child_of_node(trip_np, cdev_np) {
			struct device_node *ncluster;

			ncluster = of_parse_phandle(cdev_np, "cluster", 0);
			if (!ncluster)
				goto err_parse_dt;

			if (cluster_to_cpumask(ncluster, &trip_point->cdev_clip_cpus[j]))
				goto err_parse_dt;

			j++;
		}
	}
	return trips;
err_parse_dt:
	dev_err(&pdev->dev, "Failed to parse DT.\n");
	return NULL;
}

static int arm_pvt_probe(struct platform_device *pdev)
{
	struct arm_pvt_thermal_dev *th_sensor = NULL;
	struct trip_data *trip_data = NULL;
	struct device_node *node = pdev->dev.of_node;
	int i, j;

	if (!cpufreq_frequency_get_table(0)) {
		dev_info(&pdev->dev,
			 "Frequency table not initialized. Deferring probe...\n");
		return -EPROBE_DEFER;
	}

	trip_data = arm_pvt_parse_dt(pdev);
	if (!trip_data)
		return -EINVAL;

	sensor_data = th_sensor = devm_kzalloc(&pdev->dev, sizeof(*th_sensor), GFP_KERNEL);
	if (!th_sensor) {
		dev_err(&pdev->dev, "Unable to allocate sensor data.\n");
		return -ENOMEM;
	}

	th_sensor->baseaddr = of_iomap(node, 0);
	if (!th_sensor->baseaddr) {
		dev_err(&pdev->dev, "Unable to map sensor address.\n");
		return -ENOMEM;
	}

	mutex_init(&th_sensor->lock);
	th_sensor->num_cdevs = 0;
	th_sensor->mode = THERMAL_DEVICE_DISABLED;
	th_sensor->trip_data = trip_data;

	sensor_init();

	for (i = 0; i < trip_data->num_trips; i++) {
		struct trip_point *trip_point = &trip_data->trip_points[i];

		for (j = 0; j < trip_point->num_cdevs; j++) {
			struct thermal_cooling_device *cdev =
				cpufreq_cooling_register(&trip_point->cdev_clip_cpus[j]);

			if (IS_ERR_OR_NULL(cdev))
				continue;

			if (th_sensor->num_cdevs >= COOLING_DEV_MAX) {
				dev_err(&pdev->dev, "Too many cooling devies.\n");
				return -ENOMEM;
			}

			th_sensor->cdev[th_sensor->num_cdevs++] = cdev;
		}
	}

	th_sensor->tz = thermal_zone_device_register("arm_pvt_thermal",
						trip_data->num_trips, 0,
						th_sensor, &tz_ops, NULL, 0,
						IDLE_INTERVAL);

	if (IS_ERR_OR_NULL(th_sensor->tz)) {
		dev_err(&pdev->dev, "Error while registering thermal zone device.\n");
		return th_sensor->tz ? PTR_ERR(th_sensor->tz) : !th_sensor->tz;
	}

	platform_set_drvdata(pdev, th_sensor);
	th_sensor->mode = THERMAL_DEVICE_ENABLED;

	dev_info(&pdev->dev, "ARM PVT Sensor driver loaded.\n");

	return 0;
}

static int arm_pvt_remove(struct platform_device *pdev)
{
	struct arm_pvt_thermal_dev *th_sensor = platform_get_drvdata(pdev);
	int i;

	thermal_zone_device_unregister(th_sensor->tz);
	for (i = 0; i < th_sensor->num_cdevs; i++)
		cpufreq_cooling_unregister(th_sensor->cdev[i]);

	mutex_destroy(&th_sensor->lock);

	return 0;
}

static const struct of_device_id pvt_thermal_id_table[] = {
	{ .compatible = "arm,arm-pvt" },
	{},
};

static struct platform_driver pvt_thermal_driver = {
	.probe = arm_pvt_probe,
	.remove = arm_pvt_remove,
	.driver = {
		.name = "arm_pvt",
		.owner = THIS_MODULE,
		.of_match_table = pvt_thermal_id_table,
	},
};

module_platform_driver(pvt_thermal_driver);

MODULE_DESCRIPTION("ARM PVT Driver");
MODULE_AUTHOR("Punit Agrawal <punit.agrawal@arm.com>");
MODULE_LICENSE("GPL v2");

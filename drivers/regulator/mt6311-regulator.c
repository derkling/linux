/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Henry Chen <henryc.chen@mediatek.com>
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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/mt6311.h>
#include <linux/slab.h>
#include "mt6311-regulator.h"

struct mt6311 {
	struct device *dev;
	struct regmap *regmap;
	struct mt6311_pdata *pdata;
	struct regulator_dev *rdev[MT6311_MAX_REGULATORS];
	int num_regulator;
	int chip_cid;
};

static const struct regmap_config mt6311_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MT6311_FQMTR_CON4,
};

/* Default limits measured in millivolts and milliamps */
#define MT6311_MIN_UV		600000
#define MT6311_MAX_UV		1400000
#define MT6311_STEP_UV		6250

static const struct regulator_linear_range buck_volt_range[] = {
	REGULATOR_LINEAR_RANGE(MT6311_MIN_UV, 0, 0x7f, MT6311_STEP_UV),
};

static struct regulator_ops mt6311_buck_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static struct regulator_ops mt6311_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

#define MT6311_BUCK(_id) \
{\
	.name = #_id,\
	.ops = &mt6311_buck_ops,\
	.type = REGULATOR_VOLTAGE,\
	.id = MT6311_ID_##_id,\
	.n_voltages = (MT6311_MAX_UV - MT6311_MIN_UV) / MT6311_STEP_UV + 1,\
	.min_uV = MT6311_MIN_UV,\
	.uV_step = MT6311_STEP_UV,\
	.owner = THIS_MODULE,\
	.linear_ranges = buck_volt_range, \
	.n_linear_ranges = ARRAY_SIZE(buck_volt_range), \
	.enable_reg = MT6311_VDVFS11_CON9,\
	.enable_mask = MT6311_PMIC_VDVFS11_EN_MASK,\
	.vsel_reg = MT6311_VDVFS11_CON12,\
	.vsel_mask = MT6311_PMIC_VDVFS11_VOSEL_MASK,\
}

#define MT6311_LDO(_id) \
{\
	.name = #_id,\
	.ops = &mt6311_ldo_ops,\
	.type = REGULATOR_VOLTAGE,\
	.id = MT6311_ID_##_id,\
	.owner = THIS_MODULE,\
	.enable_reg = MT6311_LDO_CON3,\
	.enable_mask = MT6311_PMIC_RG_VBIASN_EN_MASK,\
}

static struct regulator_desc mt6311_regulators[] = {
	MT6311_BUCK(VDVFS),
	MT6311_LDO(VBIASN),
};

#ifdef CONFIG_OF
static struct of_regulator_match mt6311_matches[] = {
	[MT6311_ID_VDVFS] = { .name = "VDVFS" },
	[MT6311_ID_VBIASN] = { .name = "VBIASN" },
};

static struct mt6311_pdata *mt6311_parse_regulators_dt(
		struct device *dev)
{
	struct mt6311_pdata *pdata;
	struct device_node *node;
	int i, num, n;

	node = of_get_child_by_name(dev->of_node, "regulators");
	if (!node) {
		dev_err(dev, "regulators node not found\n");
		return ERR_PTR(-ENODEV);
	}

	num = of_regulator_match(dev, node, mt6311_matches,
				 ARRAY_SIZE(mt6311_matches));
	of_node_put(node);
	if (num < 0) {
		dev_err(dev, "Failed to match regulators\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->num_buck = num;
	n = 0;
	for (i = 0; i < ARRAY_SIZE(mt6311_matches); i++) {
		if (!mt6311_matches[i].init_data)
			continue;

		pdata->init_data[n] = mt6311_matches[i].init_data;
		pdata->reg_node[n] = mt6311_matches[i].of_node;

		n++;
	}

	return pdata;
}
#else
static struct mt6311_pdata *mt6311_parse_regulators_dt(
		struct device *dev)
{
	return ERR_PTR(-ENODEV);
}
#endif

static int mt6311_regulator_init(struct mt6311 *chip)
{
	struct regulator_config config = { };
	int i;

	chip->num_regulator = chip->pdata->num_buck;

	for (i = 0; i < chip->num_regulator; i++) {
		config.init_data = chip->pdata->init_data[i];
		config.dev = chip->dev;
		config.driver_data = chip;
		config.regmap = chip->regmap;
		config.of_node = chip->pdata->reg_node[i];

		chip->rdev[i] = devm_regulator_register(chip->dev,
			&mt6311_regulators[i], &config);
		if (IS_ERR(chip->rdev[i])) {
			dev_err(chip->dev,
				"Failed to register MT6311 regulator\n");
			return PTR_ERR(chip->rdev[i]);
		}
	}

	return 0;
}

/*
 * I2C driver interface functions
 */
static int mt6311_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct mt6311 *chip;
	int error, ret;
	unsigned int data;

	chip = devm_kzalloc(&i2c->dev, sizeof(struct mt6311), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &i2c->dev;
	chip->regmap = devm_regmap_init_i2c(i2c, &mt6311_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(chip->dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	i2c_set_clientdata(i2c, chip);

	chip->pdata = i2c->dev.platform_data;

	ret = regmap_read(chip->regmap, MT6311_SWCID, &data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read DEVICE_ID reg: %d\n", ret);
		return ret;
	}

	switch (data) {
	case MT6311_E1_CID_CODE:
	case MT6311_E2_CID_CODE:
	case MT6311_E3_CID_CODE:
		chip->chip_cid = data;
		break;
	default:
		dev_err(chip->dev, "Unsupported device id = 0x%x.\n", data);
		return -ENODEV;
	}

	if (!chip->pdata)
		chip->pdata = mt6311_parse_regulators_dt(chip->dev);

	if (IS_ERR(chip->pdata)) {
		dev_err(chip->dev, "No regulators defined for the platform\n");
		return PTR_ERR(chip->pdata);
	}

	ret = mt6311_regulator_init(chip);

	if (ret < 0)
		dev_err(chip->dev, "Failed to initialize regulator: %d\n", ret);

	return ret;
}

static const struct i2c_device_id mt6311_i2c_id[] = {
	{"mt6311", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mt6311_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id mt6311_dt_ids[] = {
	{ .compatible = "mediatek,mt6311-regulator",
	  .data = &mt6311_i2c_id[0] },
	{},
};
MODULE_DEVICE_TABLE(of, mt6311_dt_ids);
#endif

static struct i2c_driver mt6311_regulator_driver = {
	.driver = {
		.name = "mt6311",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mt6311_dt_ids),
	},
	.probe = mt6311_i2c_probe,
	.id_table = mt6311_i2c_id,
};

module_i2c_driver(mt6311_regulator_driver);

MODULE_AUTHOR("Henry Chen <henryc.chen@mediatek.com>");
MODULE_DESCRIPTION("Regulator device driver for Mediatek MT6311");
MODULE_LICENSE("GPL v2");

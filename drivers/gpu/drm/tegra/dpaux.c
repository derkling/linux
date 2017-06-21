/*
 * Copyright (C) 2013 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/tegra-powergate.h>

#include <drm/drm_dp_helper.h>
#include <drm/drm_panel.h>

#include "dpaux.h"
#include "drm.h"

static DEFINE_MUTEX(dpaux_lock);
static LIST_HEAD(dpaux_list);

struct tegra_dpaux {
	struct drm_dp_aux aux;
	struct host1x_client client;
	struct device *dev;

	void __iomem *regs;
	int irq;

	struct tegra_output *output;

	struct reset_control *rst;
	struct clk *clk_parent;
	struct clk *clk;

	struct completion complete;
	struct list_head list;

	bool enabled;

	struct drm_minor *minor;
	struct drm_info_list *debugfs_files;
	struct dentry *debugfs;
};

static inline struct tegra_dpaux *
host1x_client_to_dpaux(struct host1x_client *client)
{
	return container_of(client, struct tegra_dpaux, client);
}

static inline struct tegra_dpaux *to_dpaux(struct drm_dp_aux *aux)
{
	return container_of(aux, struct tegra_dpaux, aux);
}

static inline unsigned long tegra_dpaux_readl(struct tegra_dpaux *dpaux,
					      unsigned long offset)
{
	return readl(dpaux->regs + (offset << 2));
}

static inline void tegra_dpaux_writel(struct tegra_dpaux *dpaux,
				      unsigned long value,
				      unsigned long offset)
{
	writel(value, dpaux->regs + (offset << 2));
}

static void tegra_dpaux_write_fifo(struct tegra_dpaux *dpaux, const u8 *buffer,
				   size_t size)
{
	unsigned long offset = DPAUX_DP_AUXDATA_WRITE(0);
	size_t i, j;

	for (i = 0; i < size; i += 4) {
		size_t num = min_t(size_t, size - i, 4);
		unsigned long value = 0;

		for (j = 0; j < num; j++)
			value |= buffer[i + j] << (j * 8);

		tegra_dpaux_writel(dpaux, value, offset++);
	}
}

static void tegra_dpaux_read_fifo(struct tegra_dpaux *dpaux, u8 *buffer,
				  size_t size)
{
	unsigned long offset = DPAUX_DP_AUXDATA_READ(0);
	size_t i, j;

	for (i = 0; i < size; i += 4) {
		size_t num = min_t(size_t, size - i, 4);
		unsigned long value;

		value = tegra_dpaux_readl(dpaux, offset++);

		for (j = 0; j < num; j++)
			buffer[i + j] = value >> (j * 8);
	}
}

static int tegra_dpaux_wait_plugged(struct tegra_dpaux *dpaux)
{
	unsigned long value;
	unsigned long timeout;

	/*
	 * Typical plugged latency is 250us, but we use usleep below which
	 * involves kernel schedule, so set a somewhat big timeout here.
	 */
	timeout = jiffies + msecs_to_jiffies(250);
	while (time_before(jiffies, timeout)) {
		value = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);
		if (value & DPAUX_DP_AUXSTAT_HPD_STATUS)
			return 0;

		usleep_range(1000, 2000);
	}

	/* Recheck in case the polling is skipped due to process schedule */
	value = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);
	if (value & DPAUX_DP_AUXSTAT_HPD_STATUS) {
		return 0;
	} else {
		WARN(1, "dpaux waits for plugged timeout.\n");
		return -ETIMEDOUT;
	}
}

static ssize_t tegra_dpaux_transfer(struct drm_dp_aux *aux,
				    struct drm_dp_aux_msg *msg)
{
	unsigned long timeout = msecs_to_jiffies(250);
	struct tegra_dpaux *dpaux = to_dpaux(aux);
	unsigned long status;
	ssize_t ret = 0;
	u32 value;

	/* Tegra has 4x4 byte DP AUX transmit and receive FIFOs. */
	if (msg->size > 16)
		return -EINVAL;

	/*
	 * Allow zero-sized messages only for I2C, in which case they specify
	 * address-only transactions.
	 */
	if (msg->size < 1) {
		switch (msg->request & ~DP_AUX_I2C_MOT) {
		case DP_AUX_I2C_WRITE:
		case DP_AUX_I2C_READ:
			value = DPAUX_DP_AUXCTL_CMD_ADDRESS_ONLY;
			break;

		default:
			return -EINVAL;
		}
	} else {
		/* For non-zero-sized messages, set the CMDLEN field. */
		value = DPAUX_DP_AUXCTL_CMDLEN(msg->size - 1);
	}

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_I2C_WRITE:
		if (msg->request & DP_AUX_I2C_MOT)
			value |= DPAUX_DP_AUXCTL_CMD_MOT_WR;
		else
			value |= DPAUX_DP_AUXCTL_CMD_I2C_WR;

		break;

	case DP_AUX_I2C_READ:
		if (msg->request & DP_AUX_I2C_MOT)
			value |= DPAUX_DP_AUXCTL_CMD_MOT_RD;
		else
			value |= DPAUX_DP_AUXCTL_CMD_I2C_RD;

		break;

	case DP_AUX_I2C_STATUS:
		if (msg->request & DP_AUX_I2C_MOT)
			value |= DPAUX_DP_AUXCTL_CMD_MOT_RQ;
		else
			value |= DPAUX_DP_AUXCTL_CMD_I2C_RQ;

		break;

	case DP_AUX_NATIVE_WRITE:
		value |= DPAUX_DP_AUXCTL_CMD_AUX_WR;
		break;

	case DP_AUX_NATIVE_READ:
		value |= DPAUX_DP_AUXCTL_CMD_AUX_RD;
		break;

	default:
		return -EINVAL;
	}

	if (!dpaux->enabled)
		tegra_dpaux_enable(dpaux);

	if (tegra_dpaux_wait_plugged(dpaux) < 0) {
		WARN(1, "wait HPD failed in dpaux transfer.\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	tegra_dpaux_writel(dpaux, msg->address, DPAUX_DP_AUXADDR);
	tegra_dpaux_writel(dpaux, value, DPAUX_DP_AUXCTL);

	if ((msg->request & DP_AUX_I2C_READ) == 0) {
		tegra_dpaux_write_fifo(dpaux, msg->buffer, msg->size);
		ret = msg->size;
	}

	/* start transaction */
	value = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXCTL);
	value |= DPAUX_DP_AUXCTL_TRANSACTREQ;
	tegra_dpaux_writel(dpaux, value, DPAUX_DP_AUXCTL);

	status = wait_for_completion_timeout(&dpaux->complete, timeout);
	if (!status) {
		WARN(1, "wait dpaux transfer complete timeout.\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	/* read status and clear errors */
	value = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);
	tegra_dpaux_writel(dpaux, 0xf00, DPAUX_DP_AUXSTAT);

	if (value & DPAUX_DP_AUXSTAT_TIMEOUT_ERROR) {
		WARN(1, "dpaux transfer error: timedout.\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	if ((value & DPAUX_DP_AUXSTAT_RX_ERROR) ||
	    (value & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR) ||
	    (value & DPAUX_DP_AUXSTAT_NO_STOP_ERROR)) {
		ret = -EIO;
		goto out;
	}

	switch ((value & DPAUX_DP_AUXSTAT_REPLY_TYPE_MASK) >> 16) {
	case 0x00:
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		break;

	case 0x01:
		msg->reply = DP_AUX_NATIVE_REPLY_NACK;
		break;

	case 0x02:
		msg->reply = DP_AUX_NATIVE_REPLY_DEFER;
		break;

	case 0x04:
		msg->reply = DP_AUX_I2C_REPLY_NACK;
		break;

	case 0x08:
		msg->reply = DP_AUX_I2C_REPLY_DEFER;
		break;
	}

	if ((msg->size > 0) && (msg->reply == DP_AUX_NATIVE_REPLY_ACK)) {
		if (msg->request & DP_AUX_I2C_READ) {
			size_t count = value & DPAUX_DP_AUXSTAT_REPLY_MASK;

			if (WARN_ON(count != msg->size))
				count = min_t(size_t, count, msg->size);

			tegra_dpaux_read_fifo(dpaux, msg->buffer, count);
			ret = count;
		}
	} else if (msg->reply &
			(DP_AUX_NATIVE_REPLY_DEFER | DP_AUX_I2C_REPLY_DEFER)) {
		/* return the requested size for retry */
		ret = msg->size;
	}

out:
	return ret;
}

static irqreturn_t tegra_dpaux_irq(int irq, void *data)
{
	struct tegra_dpaux *dpaux = data;
	irqreturn_t ret = IRQ_HANDLED;
	unsigned long value;

	/* clear interrupts */
	value = tegra_dpaux_readl(dpaux, DPAUX_INTR_AUX);
	tegra_dpaux_writel(dpaux, value, DPAUX_INTR_AUX);

	if (value & DPAUX_INTR_IRQ_EVENT) {
		/* TODO: handle this */
	}

	if (value & DPAUX_INTR_AUX_DONE)
		complete(&dpaux->complete);

	return ret;
}

static int tegra_dpaux_show_regs(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct tegra_dpaux *dpaux = node->info_ent->data;

#define DUMP_REG(name)						\
	seq_printf(s, "%-40s %#05x %08lx\n", #name, name,	\
		   tegra_dpaux_readl(dpaux, name))

	DUMP_REG(DPAUX_CTXSW);
	DUMP_REG(DPAUX_INTR_EN_AUX);
	DUMP_REG(DPAUX_INTR_AUX);
	DUMP_REG(DPAUX_DP_AUXADDR);
	DUMP_REG(DPAUX_DP_AUXCTL);
	DUMP_REG(DPAUX_DP_AUXSTAT);
	DUMP_REG(DPAUX_DP_AUX_SINKSTAT_LO);
	DUMP_REG(DPAUX_DP_AUX_SINKSTAT_HI);
	DUMP_REG(DPAUX_HPD_CONFIG);
	DUMP_REG(DPAUX_HPD_IRQ_CONFIG);
	DUMP_REG(DPAUX_DP_AUX_CONFIG);
	DUMP_REG(DPAUX_HYBRID_PADCTL);
	DUMP_REG(DPAUX_HYBRID_SPARE);
	DUMP_REG(DPAUX_SCRATCH_REG0);
	DUMP_REG(DPAUX_SCRATCH_REG1);
	DUMP_REG(DPAUX_SCRATCH_REG2);

#undef DUMP_REG

	return 0;
}

static struct drm_info_list debugfs_files[] = {
	{ "regs", tegra_dpaux_show_regs, 0, NULL },
};

static int tegra_dpaux_debugfs_init(struct tegra_dpaux *dpaux,
					struct drm_minor *minor)
{
	unsigned int i;
	int err;

	dpaux->debugfs = debugfs_create_dir("dpaux", minor->debugfs_root);
	if (!dpaux->debugfs)
		return -ENOMEM;

	dpaux->debugfs_files = kmemdup(debugfs_files, sizeof(debugfs_files),
				    GFP_KERNEL);
	if (!dpaux->debugfs_files) {
		err = -ENOMEM;
		goto remove;
	}

	for (i = 0; i < ARRAY_SIZE(debugfs_files); i++)
		dpaux->debugfs_files[i].data = dpaux;

	err = drm_debugfs_create_files(dpaux->debugfs_files,
				       ARRAY_SIZE(debugfs_files),
				       dpaux->debugfs, minor);
	if (err < 0)
		goto free;

	dpaux->minor = minor;

	return 0;

free:
	kfree(dpaux->debugfs_files);
	dpaux->debugfs_files = NULL;
remove:
	debugfs_remove(dpaux->debugfs);
	dpaux->debugfs = NULL;

	return err;
}

static int tegra_dpaux_debugfs_exit(struct tegra_dpaux *dpaux)
{
	drm_debugfs_remove_files(dpaux->debugfs_files, ARRAY_SIZE(debugfs_files),
				 dpaux->minor);
	dpaux->minor = NULL;

	kfree(dpaux->debugfs_files);
	dpaux->debugfs_files = NULL;

	debugfs_remove(dpaux->debugfs);
	dpaux->debugfs = NULL;

	return 0;
}

static int tegra_dpaux_init(struct host1x_client *client)
{
	struct tegra_drm *tegra = dev_get_drvdata(client->parent);
	struct tegra_dpaux *dpaux = host1x_client_to_dpaux(client);
	int err;

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_dpaux_debugfs_init(dpaux, tegra->drm->primary);
		if (err < 0)
			dev_err(dpaux->dev, "debugfs setup failed: %d\n", err);
		return err;
	}

	return 0;
}

static int tegra_dpaux_exit(struct host1x_client *client)
{
	struct tegra_dpaux *dpaux = host1x_client_to_dpaux(client);
	int err;

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_dpaux_debugfs_exit(dpaux);
		if (err < 0)
			dev_err(dpaux->dev,
				"debugfs cleanup failed: %d\n", err);
		return err;
	}

	return 0;
}

static const struct host1x_client_ops dpaux_client_ops = {
	.init = tegra_dpaux_init,
	.exit = tegra_dpaux_exit,
};

static int tegra_dpaux_probe(struct platform_device *pdev)
{
	struct tegra_dpaux *dpaux;
	struct host1x_client *client;
	struct resource *regs;
	unsigned long value;
	int err;

	client = drm_host1x_get_client(&pdev->dev);
	if (client) {
		dpaux = host1x_client_to_dpaux(client);
	} else {
		dpaux = kzalloc(sizeof(*dpaux), GFP_KERNEL);
		if (!dpaux)
			return -ENOMEM;

		INIT_LIST_HEAD(&dpaux->client.list);
		dpaux->client.ops = &dpaux_client_ops;
		dpaux->client.dev = &pdev->dev;
	}

	err = drm_host1x_register(&dpaux->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register host1x client: %d\n",
			err);
		return err;
	}

	init_completion(&dpaux->complete);
	INIT_LIST_HEAD(&dpaux->list);
	dpaux->dev = &pdev->dev;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dpaux->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(dpaux->regs))
		return PTR_ERR(dpaux->regs);

	dpaux->irq = platform_get_irq(pdev, 0);
	if (dpaux->irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return -ENXIO;
	}

	dpaux->rst = devm_reset_control_get(&pdev->dev, "dpaux");
	if (IS_ERR(dpaux->rst))
		return PTR_ERR(dpaux->rst);

	dpaux->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dpaux->clk))
		return PTR_ERR(dpaux->clk);

	dpaux->clk_parent = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(dpaux->clk_parent))
		return PTR_ERR(dpaux->clk_parent);

	err = clk_prepare_enable(dpaux->clk_parent);
	if (err < 0)
		return err;

	err = clk_set_rate(dpaux->clk_parent, 270000000);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to set clock to 270 MHz: %d\n",
			err);
		return err;
	}

	err = devm_request_irq(dpaux->dev, dpaux->irq, tegra_dpaux_irq, 0,
			       dev_name(dpaux->dev), dpaux);
	if (err < 0) {
		dev_err(dpaux->dev, "failed to request IRQ#%u: %d\n",
			dpaux->irq, err);
		return err;
	}

	disable_irq(dpaux->irq);

	dpaux->aux.transfer = tegra_dpaux_transfer;
	dpaux->aux.dev = &pdev->dev;

	err = drm_dp_aux_register_i2c_bus(&dpaux->aux);
	if (err < 0)
		return err;

	/* enable and clear all interrupts */
	value = DPAUX_INTR_AUX_DONE | DPAUX_INTR_IRQ_EVENT;
	tegra_dpaux_writel(dpaux, value, DPAUX_INTR_EN_AUX);
	tegra_dpaux_writel(dpaux, 0xffffffff, DPAUX_INTR_AUX);

	mutex_lock(&dpaux_lock);
	list_add_tail(&dpaux->list, &dpaux_list);
	mutex_unlock(&dpaux_lock);

	platform_set_drvdata(pdev, dpaux);
	dpaux->client.driver_probed = 1;
	dev_info(&pdev->dev, "initialized\n");
	return 0;
}

static int tegra_dpaux_remove(struct platform_device *pdev)
{
	struct tegra_dpaux *dpaux = platform_get_drvdata(pdev);

	drm_dp_aux_unregister_i2c_bus(&dpaux->aux);

	mutex_lock(&dpaux_lock);
	list_del(&dpaux->list);
	mutex_unlock(&dpaux_lock);

	clk_disable_unprepare(dpaux->clk_parent);
	reset_control_assert(dpaux->rst);
	clk_disable_unprepare(dpaux->clk);

	kfree(dpaux);
	return 0;
}

static const struct of_device_id tegra_dpaux_of_match[] = {
	{ .compatible = "nvidia,tegra124-dpaux", },
	{ },
};

struct platform_driver tegra_dpaux_driver = {
	.driver = {
		.name = "tegra-dpaux",
		.of_match_table = tegra_dpaux_of_match,
	},
	.probe = tegra_dpaux_probe,
	.remove = tegra_dpaux_remove,
};

struct tegra_dpaux *tegra_dpaux_find_by_of_node(struct device_node *np)
{
	struct tegra_dpaux *dpaux;

	mutex_lock(&dpaux_lock);

	list_for_each_entry(dpaux, &dpaux_list, list)
		if (np == dpaux->dev->of_node) {
			mutex_unlock(&dpaux_lock);
			return dpaux;
		}

	mutex_unlock(&dpaux_lock);

	return NULL;
}

int tegra_dpaux_attach(struct tegra_dpaux *dpaux, struct tegra_output *output)
{
	unsigned long timeout;

	output->connector.polled = DRM_CONNECTOR_POLL_HPD;
	output->ddc = &dpaux->aux.ddc;
	dpaux->output = output;

	timeout = jiffies + msecs_to_jiffies(250);
	while (time_before(jiffies, timeout)) {
		enum drm_connector_status status;

		status = tegra_dpaux_detect(dpaux);
		if (status == connector_status_connected) {
			enable_irq(dpaux->irq);
			return 0;
		}

		usleep_range(1000, 2000);
	}

	return -ETIMEDOUT;
}

int tegra_dpaux_detach(struct tegra_dpaux *dpaux)
{
	unsigned long timeout;

	disable_irq(dpaux->irq);

	timeout = jiffies + msecs_to_jiffies(250);
	while (time_before(jiffies, timeout)) {
		enum drm_connector_status status;

		status = tegra_dpaux_detect(dpaux);
		if (status == connector_status_disconnected) {
			dpaux->output = NULL;
			return 0;
		}

		usleep_range(1000, 2000);
	}

	return -ETIMEDOUT;
}

enum drm_connector_status tegra_dpaux_detect(struct tegra_dpaux *dpaux)
{
	/* nyan doesn't support DP, so for eDP panel, it's always connected. */
	return connector_status_connected;
}

int tegra_dpaux_enable(struct tegra_dpaux *dpaux)
{
	unsigned long value;
	int err;

	if (dpaux->enabled)
		return 0;

	tegra_output_panel_prepare(dpaux->output);
	tegra_unpowergate_partition(TEGRA_POWERGATE_SOR);
	err = clk_prepare_enable(dpaux->clk);
	if (err < 0) {
		dev_info(dpaux->dev, "enable dpaux clock failed: %d\n", err);
		tegra_powergate_partition(TEGRA_POWERGATE_SOR);
		return err;
	}
	reset_control_deassert(dpaux->rst);
	udelay(10);

	/* enable and clear all interrupts */
	value = DPAUX_INTR_AUX_DONE | DPAUX_INTR_IRQ_EVENT;
	tegra_dpaux_writel(dpaux, value, DPAUX_INTR_EN_AUX);
	tegra_dpaux_writel(dpaux, 0xffffffff, DPAUX_INTR_AUX);

	value = DPAUX_HYBRID_PADCTL_AUX_CMH(2) |
		DPAUX_HYBRID_PADCTL_AUX_DRVZ(4) |
		DPAUX_HYBRID_PADCTL_AUX_DRVI(0x18) |
		DPAUX_HYBRID_PADCTL_AUX_INPUT_RCV |
		DPAUX_HYBRID_PADCTL_MODE_AUX;
	tegra_dpaux_writel(dpaux, value, DPAUX_HYBRID_PADCTL);

	value = tegra_dpaux_readl(dpaux, DPAUX_HYBRID_SPARE);
	value &= ~DPAUX_HYBRID_SPARE_PAD_POWER_DOWN;
	tegra_dpaux_writel(dpaux, value, DPAUX_HYBRID_SPARE);

	dpaux->enabled = true;
	return 0;
}

int tegra_dpaux_disable(struct tegra_dpaux *dpaux)
{
	unsigned long value;

	if (!dpaux->enabled)
		return 0;

	/* disable and clear all interrupts */
	tegra_dpaux_writel(dpaux, 0, DPAUX_INTR_EN_AUX);
	tegra_dpaux_writel(dpaux, 0xffffffff, DPAUX_INTR_AUX);

	value = tegra_dpaux_readl(dpaux, DPAUX_HYBRID_SPARE);
	value |= DPAUX_HYBRID_SPARE_PAD_POWER_DOWN;
	tegra_dpaux_writel(dpaux, value, DPAUX_HYBRID_SPARE);

	tegra_powergate_partition(TEGRA_POWERGATE_SOR);
	reset_control_assert(dpaux->rst);
	udelay(10);
	clk_disable_unprepare(dpaux->clk);

	tegra_output_panel_unprepare(dpaux->output);
	dpaux->enabled = false;
	return 0;
}

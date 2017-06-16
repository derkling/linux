/*
 * Copyright (C) 2012 Avionic Design GmbH
 * Copyright (C) 2012 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of_i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>

#include <drm/drm_panel.h>
#include "drm.h"

static int tegra_connector_get_modes(struct drm_connector *connector)
{
	struct tegra_output *output = connector_to_output(connector);
	struct edid *edid = NULL;
	int err = 0;


	if (output->edid) {
		edid = kmemdup(output->edid, sizeof(*edid), GFP_KERNEL);
	} else if (output->ddc) {
		edid = drm_get_edid(connector, output->ddc);
	} else if (output->panel) {
		err = output->panel->funcs->get_modes(output->panel);
		if (err > 0)
			return err;
	}

	drm_mode_connector_update_edid_property(connector, edid);

	if (edid) {
		err = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	return err;
}

static int tegra_connector_mode_valid(struct drm_connector *connector,
				      struct drm_display_mode *mode)
{
	struct tegra_output *output = connector_to_output(connector);
	enum drm_mode_status status = MODE_OK;
	int err;

	err = tegra_output_check_mode(output, mode, &status);
	if (err < 0)
		return MODE_ERROR;

	return status;
}

static struct drm_encoder *
tegra_connector_best_encoder(struct drm_connector *connector)
{
	struct tegra_output *output = connector_to_output(connector);

	return &output->encoder;
}

static const struct drm_connector_helper_funcs connector_helper_funcs = {
	.get_modes = tegra_connector_get_modes,
	.mode_valid = tegra_connector_mode_valid,
	.best_encoder = tegra_connector_best_encoder,
};

static enum drm_connector_status
tegra_connector_detect(struct drm_connector *connector, bool force)
{
	struct tegra_output *output = connector_to_output(connector);
	enum drm_connector_status status = connector_status_unknown;

	if (output->ops->detect)
		return output->ops->detect(output);

	if (gpio_is_valid(output->hpd_gpio)) {
		if (gpio_get_value(output->hpd_gpio) == 0)
			status = connector_status_disconnected;
		else
			status = connector_status_connected;
	} else {
		if (!output->panel)
			status = connector_status_disconnected;
		else
			status = connector_status_connected;

		if (connector->connector_type == DRM_MODE_CONNECTOR_LVDS)
			status = connector_status_connected;
	}

	return status;
}

static void drm_connector_clear(struct drm_connector *connector)
{
	memset(connector, 0, sizeof(*connector));
}

static void tegra_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	drm_connector_clear(connector);
}

static void tegra_connector_dpms(struct drm_connector *connector, int mode)
{
	/*
	 * We don't make use of connector->dpms.
	 * But we need this function otherwise kernel crashes in
	 * drm_crtc_helper_set_config because it calls connector->dpms
	 * without checking whether it is defined.
	 */
}

static void tegra_connector_save(struct drm_connector *connector)
{
	struct tegra_output *output = connector_to_output(connector);

	output->suspended = true;
}

static void tegra_connector_restore(struct drm_connector *connector)
{
	struct tegra_output *output = connector_to_output(connector);

	output->suspended = false;
}

static const struct drm_connector_funcs connector_funcs = {
	.dpms = tegra_connector_dpms,
	.detect = tegra_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = tegra_connector_destroy,
	.save = tegra_connector_save,
	.restore = tegra_connector_restore,
};

int tegra_output_panel_enable(struct tegra_output *output)
{
	int err;

	if (!output->panel)
		return 0;

	if (output->panel_enabled)
		return 0;

	err = drm_panel_enable(output->panel);
	if (err < 0) {
		dev_err(output->dev, "panel enable failed: %d\n", err);
		return err;
	}

	output->panel_enabled = true;
	return 0;
}

int tegra_output_panel_disable(struct tegra_output *output)
{
	int err;

	if (!output->panel)
		return 0;

	if (!output->panel_enabled)
		return 0;

	err = drm_panel_disable(output->panel);
	if (err < 0) {
		dev_err(output->dev, "panel disable failed: %d\n", err);
		return err;
	}

	output->panel_enabled = false;
	return 0;
}

int tegra_output_panel_prepare(struct tegra_output *output)
{
	int err;

	if (!output->panel)
		return 0;

	if (output->panel_prepared)
		return 0;

	err = drm_panel_prepare(output->panel);
	if (err < 0) {
		dev_err(output->dev, "panel prepare failed: %d\n", err);
		return err;
	}

	output->panel_prepared = true;
	return 0;
}

int tegra_output_panel_unprepare(struct tegra_output *output)
{
	int err;

	if (!output->panel)
		return 0;

	if (!output->panel_prepared)
		return 0;

	err = drm_panel_unprepare(output->panel);
	if (err < 0) {
		dev_err(output->dev, "panel unprepare failed: %d\n", err);
		return err;
	}

	output->panel_prepared = false;
	return 0;
}

static void drm_encoder_clear(struct drm_encoder *encoder)
{
	memset(encoder, 0, sizeof(*encoder));
}

static void tegra_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
	drm_encoder_clear(encoder);
}

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = tegra_encoder_destroy,
};

static void tegra_encoder_disable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);

	if (!output->enabled)
		return;

	tegra_output_panel_disable(output);
	tegra_output_disable(output);

	output->enabled = false;
}

static void tegra_encoder_enable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);

	if (output->enabled)
		return;

	tegra_output_enable(output);
	tegra_output_panel_enable(output);

	output->enabled = true;
}

static bool tegra_encoder_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted)
{
	return true;
}

static void tegra_encoder_prepare(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);
	bool mode_changed = false;

	if (encoder->crtc)
		mode_changed = !drm_mode_equal(&encoder->crtc->mode,
						&output->saved_mode);
	if (mode_changed) {
		output->saved_mode = encoder->crtc->mode;
		/*
		 * For some outputs(like HDMI), userspace may change resolutions
		 * without disable output first. So we check mode change here.
		 * In addition, in order to not break the output enable/disable
		 * & panel enable/disable refcounting balance, we disable
		 * encoder manually.
		 */
		tegra_encoder_disable(encoder);
	}
}

static void tegra_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted)
{
}

static const struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.disable = tegra_encoder_disable,
	.mode_fixup = tegra_encoder_mode_fixup,
	.prepare = tegra_encoder_prepare,
	.commit = tegra_encoder_enable,
	.mode_set = tegra_encoder_mode_set,
};

static irqreturn_t hpd_irq(int irq, void *data)
{
	struct tegra_output *output = data;

	if (output->connector.dev && !output->suspended)
		drm_helper_hpd_irq_event(output->connector.dev);

	return IRQ_HANDLED;
}

int tegra_output_probe(struct tegra_output *output)
{
	struct device_node *ddc, *panel;
	enum of_gpio_flags flags;
	int err, size;

	if (!output->of_node)
		output->of_node = output->dev->of_node;

	panel = of_parse_phandle(output->of_node, "nvidia,panel", 0);
	if (panel) {
		output->panel = of_drm_find_panel(panel);
		if (!output->panel)
			return -EPROBE_DEFER;

		of_node_put(panel);
	}

	output->edid = of_get_property(output->of_node, "nvidia,edid", &size);

	ddc = of_parse_phandle(output->of_node, "nvidia,ddc-i2c-bus", 0);
	if (ddc) {
		output->ddc = of_find_i2c_adapter_by_node(ddc);
		if (!output->ddc) {
			err = -EPROBE_DEFER;
			of_node_put(ddc);
			return err;
		}

		of_node_put(ddc);
	}

	output->hpd_gpio = of_get_named_gpio_flags(output->of_node,
						   "nvidia,hpd-gpio", 0,
						   &flags);
	if (gpio_is_valid(output->hpd_gpio)) {
		unsigned long flags;

		err = gpio_request_one(output->hpd_gpio, GPIOF_DIR_IN,
				       "HDMI hotplug detect");
		if (err < 0) {
			dev_err(output->dev, "gpio_request_one(): %d\n", err);
			return err;
		}

		err = gpio_to_irq(output->hpd_gpio);
		if (err < 0) {
			dev_err(output->dev, "gpio_to_irq(): %d\n", err);
			gpio_free(output->hpd_gpio);
			return err;
		}

		output->hpd_irq = err;

		flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT;

		err = request_threaded_irq(output->hpd_irq, NULL, hpd_irq,
					   flags, "hpd", output);
		if (err < 0) {
			dev_err(output->dev, "failed to request IRQ#%u: %d\n",
				output->hpd_irq, err);
			gpio_free(output->hpd_gpio);
			return err;
		}

		output->connector.polled = DRM_CONNECTOR_POLL_HPD;

		/*
		 * Disable the interrupt until the connector has been
		 * initialized to avoid a race in the hotplug interrupt
		 * handler.
		 */
		disable_irq(output->hpd_irq);
	}

	return 0;
}

int tegra_output_remove(struct tegra_output *output)
{
	if (gpio_is_valid(output->hpd_gpio)) {
		free_irq(output->hpd_irq, output);
		gpio_free(output->hpd_gpio);
	}

	if (output->ddc)
		put_device(&output->ddc->dev);

	return 0;
}

int tegra_output_init(struct drm_device *drm, struct tegra_output *output)
{
	int connector, encoder;

	switch (output->type) {
	case TEGRA_OUTPUT_RGB:
		connector = DRM_MODE_CONNECTOR_LVDS;
		encoder = DRM_MODE_ENCODER_LVDS;
		break;

	case TEGRA_OUTPUT_HDMI:
		connector = DRM_MODE_CONNECTOR_HDMIA;
		encoder = DRM_MODE_ENCODER_TMDS;
		break;

	case TEGRA_OUTPUT_DSI:
		connector = DRM_MODE_CONNECTOR_DSI;
		encoder = DRM_MODE_ENCODER_DSI;
		break;

	case TEGRA_OUTPUT_EDP:
		connector = DRM_MODE_CONNECTOR_eDP;
		encoder = DRM_MODE_ENCODER_TMDS;
		break;

	default:
		connector = DRM_MODE_CONNECTOR_Unknown;
		encoder = DRM_MODE_ENCODER_NONE;
		break;
	}

	drm_connector_init(drm, &output->connector, &connector_funcs,
			   connector);
	drm_connector_helper_add(&output->connector, &connector_helper_funcs);
	output->connector.dpms = DRM_MODE_DPMS_OFF;

	if (output->panel)
		drm_panel_attach(output->panel, &output->connector);

	drm_encoder_init(drm, &output->encoder, &encoder_funcs, encoder);
	drm_encoder_helper_add(&output->encoder, &encoder_helper_funcs);

	drm_mode_connector_attach_encoder(&output->connector, &output->encoder);
	drm_sysfs_connector_add(&output->connector);

	switch (output->type) {
	case TEGRA_OUTPUT_EDP:
		output->encoder.possible_crtcs = 0x1;
		break;
	case TEGRA_OUTPUT_HDMI:
		output->encoder.possible_crtcs = 0x2;
		break;
	default:
		output->encoder.possible_crtcs = 0x3;
		break;
	}

	/*
	 * The connector is now registered and ready to receive hotplug events
	 * so the hotplug interrupt can be enabled.
	 */
	if (gpio_is_valid(output->hpd_gpio))
		enable_irq(output->hpd_irq);

	return 0;
}

int tegra_output_exit(struct tegra_output *output)
{
	/*
	 * The connector is going away, so the interrupt must be disabled to
	 * prevent the hotplug interrupt handler from potentially crashing.
	 */
	if (gpio_is_valid(output->hpd_gpio))
		disable_irq(output->hpd_irq);

	return 0;
}

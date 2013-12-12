/*
 * Copyright (C) 2013 ARM Limited
 * Author: Liviu Dudau <Liviu.Dudau@arm.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 *  Silicon Image Sil9022 driver for DRM I2C encoder slave
 */

#include <linux/component.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#define SIL9022_VIDEO_DATA_BASE_REG	0x00
#define SIL9022_IN_FORMAT_REG		0x09
#define SIL9022_OUT_FORMAT_REG		0x0a
#define SIL9022_AVI_INFOFRAME_BASE_REG	0x0c
#define SIL9022_SYS_CONTROL_REG		0x1a
#  define SIL9022_DDC_OUT_MODE		(1 << 0)
#  define SIL9022_DDC_BUS_STAT		(1 << 1)
#  define SIL9022_DDC_BUS_GRANT		(1 << 2)
#  define SIL9022_AV_MUTE		(1 << 3)
#  define SIL9022_TMDS_EN		(1 << 4)
#define SIL9022_ID_REG			0x1b
#define SIL9022_POWER_REG		0x1e
#  define SIL9022_POWER_STATE_D0	0x00
#  define SIL9022_POWER_STATE_D2	0x02
#  define SIL9022_POWER_STATE_D3	0x03
#define SIL9022_SEC_CTRL_REG		0x2a
#define SIL9022_SEC_STATUS_REG		0x29
#define SIL9022_SEC_VERSION_REG		0x30
#define SIL9022_INT_ENABLE_REG		0x3c
#define SIL9022_INT_STATUS_REG		0x3d
#  define SIL9022_HOTPLUG_EVENT		(1 << 0)
#  define SIL9022_RECEIVER_EVENT	(1 << 1)
#  define SIL9022_HOTPLUG_STATE		(1 << 2)
#  define SIL9022_RECEIVER_STATE	(1 << 3)
#define SIL9022_VENDOR_ID		0xb0
#define SIL9022_INTERNAL_PAGE		0xbc
#define SIL9022_INTERNAL_INDEX		0xbd
#define SIL9022_INTERNAL_REG		0xbe
#define SIL9022_CTRL_REG		0xc7


struct sil9022_video_regs {
	uint16_t	pixel_clock;
	uint16_t	vrefresh;
	uint16_t	cols;
	uint16_t	lines;
	uint8_t		pixel_data;
};

struct sil9022_priv {
	struct device *dev;
	struct regmap *regmap;
	struct sil9022_video_regs regs;
	struct drm_encoder_slave slave;
	struct drm_connector connector;
};

static const struct regmap_config sil9022_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#define connector_to_sil9022_priv(x) \
	container_of(x, struct sil9022_priv, connector)

#define encoder_to_sil9022_priv(x) \
	container_of(to_encoder_slave(x), struct sil9022_priv, slave)

static void sil9022_encoder_set_config(struct drm_encoder *encoder, void *params)
{
}

static void sil9022_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s: mode = %d\n", __func__, mode);

	switch (mode) {
	case DRM_MODE_DPMS_OFF:
		/* use D2 for OFF as we cannot control the reset pin */
		regmap_write(priv->regmap, SIL9022_POWER_REG, SIL9022_POWER_STATE_D2);
		break;
	case DRM_MODE_DPMS_ON:
		regmap_write(priv->regmap, SIL9022_POWER_REG, SIL9022_POWER_STATE_D0);
		break;
	}
}

static enum drm_connector_status
sil9022_encoder_detect(struct drm_encoder *encoder,
			struct drm_connector *connector)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	enum drm_connector_status con_status = connector_status_unknown;
	unsigned int status;
	int err;

	if (!priv)
		return con_status;

	err = regmap_read(priv->regmap, SIL9022_INT_STATUS_REG, &status);
	if (!err) {
		if (status & SIL9022_HOTPLUG_STATE)
			con_status = connector_status_connected;
		else
			con_status = connector_status_disconnected;

		/* clear the event status bits */
		regmap_write(priv->regmap, SIL9022_INT_STATUS_REG,
			SIL9022_HOTPLUG_EVENT | SIL9022_RECEIVER_EVENT);
	}

	return con_status;
}

static int sil9022_encoder_get_modes(struct drm_encoder *encoder,
				struct drm_connector *connector)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	struct i2c_client *client = drm_i2c_encoder_get_client(encoder);
	struct edid *edid = NULL;
	unsigned int status;
	int err = 0, timeout = 10;

	/* Disable HDCP link security */
	/* do {
		regmap_write(priv->regmap, SIL9022_SEC_CTRL_REG, 0);
		err = regmap_read(priv->regmap, SIL9022_SEC_CTRL_REG, &status);
	} while (status && !err);
	err = regmap_read(priv->regmap, SIL9022_SEC_STATUS_REG, &status); */

	sil9022_encoder_detect(encoder, connector); // test
	err = regmap_read(priv->regmap, SIL9022_POWER_REG, &status);
	dev_info(priv->dev, "power state = 0x%08x\n", status);

	/* first, request the pass-through mode in order to read the edid */
	err = regmap_read(priv->regmap, SIL9022_SYS_CONTROL_REG, &status);
	if (err) {
		dev_err(priv->dev, "failed to read DDC bus status register\n");
		return err;
	}

	status |= SIL9022_DDC_BUS_GRANT;
	err = regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, status);
	if (err) {
		dev_err(priv->dev, "failed to request DDC bus ownership\n");
		return err;
	}
	do {
		/* wait for state change */
		err = regmap_read(priv->regmap, SIL9022_SYS_CONTROL_REG, &status);
		--timeout;
	} while (((status & SIL9022_DDC_BUS_STAT) != SIL9022_DDC_BUS_STAT) && timeout);

	if (!timeout) {
		dev_warn(priv->dev, "timeout waiting for DDC bus grant\n");
		goto release_ddc;
	}

	/* write back the value read in order to close the i2c switch */
	err = regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, status);
	if (err) {
		dev_err(priv->dev, "failed to switch DDC bus\n");
		goto release_ddc;
	}

	edid = drm_get_edid(connector, client->adapter);
	if (!edid) {
		dev_err(priv->dev, "failed to get EDID data\n");
		err = -1;
	}

release_ddc:
	timeout = 10;
	do {
		status &= ~(SIL9022_DDC_BUS_STAT | SIL9022_DDC_BUS_GRANT);
		regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, status);
		err = regmap_read(priv->regmap, SIL9022_SYS_CONTROL_REG, &status);
		--timeout;
	} while ((status & (SIL9022_DDC_BUS_STAT | SIL9022_DDC_BUS_GRANT)) && timeout);

	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		err = drm_add_edid_modes(connector, edid);
		if (drm_detect_hdmi_monitor(edid))
			regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, status | 1);
		else
			regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, status & 0xfe);
		kfree(edid);
	} else {
		/* add standard modes up to 1080p and set the preferred mode */
		err = drm_add_modes_noedid(connector, 2640, 2720);
		drm_set_preferred_mode(connector, 1600, 1200);
		/* switch to DVI mode */
		regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, status & 0xfe);
	}

	dev_info(priv->dev, "%s returned %d\n", __func__, err);
	return err;
}

static bool sil9022_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s\n", __func__);
	return true;
}

static int sil9022_encoder_mode_valid(struct drm_encoder *encoder,
				struct drm_display_mode *mode)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s\n", __func__);
	return MODE_OK;
}

static void sil9022_encoder_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	/* Sil9022 clock is pixclock / 10000 Hz */
	int clk = adjusted_mode->crtc_clock / 10;
	int err, vrefresh = adjusted_mode->vrefresh * 100;
	struct hdmi_avi_infoframe frame;
	uint8_t avidata[HDMI_INFOFRAME_SIZE(AVI)];
	uint8_t buf[] = {
		clk & 0xff,
		(clk & 0xff00) >> 8,
		vrefresh & 0xff,
		(vrefresh & 0xff00) >> 8,
		adjusted_mode->crtc_hdisplay & 0xff,
		(adjusted_mode->crtc_hdisplay & 0xff00) >> 8,
		adjusted_mode->crtc_vdisplay & 0xff,
		(adjusted_mode->crtc_vdisplay & 0xff00) >> 8,
	};

	dev_info(priv->dev, "%s\n", __func__);

	regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, ~(SIL9022_TMDS_EN | SIL9022_DDC_OUT_MODE));
	err = regmap_bulk_write(priv->regmap, SIL9022_VIDEO_DATA_BASE_REG,
				buf, ARRAY_SIZE(buf));
	if (err) {
		dev_err(priv->dev, "Could not write video mode data\n");
		return;
	}

	regmap_write(priv->regmap, SIL9022_POWER_REG, SIL9022_POWER_STATE_D0);
	regmap_write(priv->regmap, 0x26, 0x50);
	regmap_write(priv->regmap, 0x25, 0x3);
	regmap_write(priv->regmap, 0x27, 0x0);
	regmap_write(priv->regmap, 0x26, 0x40);
	regmap_write(priv->regmap, SIL9022_SYS_CONTROL_REG, SIL9022_DDC_OUT_MODE);
	memset(avidata, 0, sizeof(avidata));
#if 0
	err = drm_hdmi_avi_infoframe_from_display_mode(&frame, adjusted_mode);
	if (err) {
		dev_err(priv->dev, "Unable to generate AVI infoframe from mode\n");
		return;
	}
	err = hdmi_avi_infoframe_pack(&frame, avidata, sizeof(avidata));
	if (err < 0) {
		dev_err(priv->dev, "Packing of AVI infoframe failed: %d\n", err);
		return;
	}
#endif

	/* input is full range RGB */
	regmap_write(priv->regmap, SIL9022_IN_FORMAT_REG, 0x00);
	/* output is full range digital RGB */
	regmap_write(priv->regmap, SIL9022_OUT_FORMAT_REG, 0x00);

	err = regmap_bulk_write(priv->regmap, SIL9022_AVI_INFOFRAME_BASE_REG,
				&avidata[HDMI_INFOFRAME_HEADER_SIZE-1],  /* skip header */
				HDMI_AVI_INFOFRAME_SIZE + 1);
	if (err)
		dev_err(priv->dev, "Failed to write AVI infoframe\n");
}

static void sil9022_handle_hotplug(struct sil9022_priv *priv)
{
	enum drm_connector_status status;

	status = sil9022_encoder_detect(&priv->slave.base, &priv->connector);
}

static irqreturn_t sil9022_irq_thread(int irq, void *data)
{
	struct sil9022_priv *priv = data;

	if (!priv)
		return IRQ_NONE;

	sil9022_handle_hotplug(priv);

	return IRQ_HANDLED;
}

static int sil9022_setup(struct sil9022_priv *priv)
{
	int err;
	unsigned int dev_id, dev_rev;
	struct i2c_client *client = to_i2c_client(priv->dev);

	/* first step is to enable the TPI mode */
	err = regmap_write(priv->regmap, SIL9022_CTRL_REG, 0x00);
	if (err) {
		dev_err(priv->dev, "failed to enable TPI commands\n");
		return err;
	}
	/* read the revision number */
	err = regmap_read(priv->regmap, SIL9022_ID_REG, &dev_id);
	err = err ? err : regmap_read(priv->regmap, SIL9022_ID_REG+1, &dev_rev);
	if (err || dev_id != SIL9022_VENDOR_ID) {
		dev_err(priv->dev, "device not found\n");
		return -ENODEV;
	}
	err = regmap_read(priv->regmap, SIL9022_SEC_VERSION_REG, &dev_id);
	if (err)
		return err;

	dev_info(priv->dev, "found %s chip (rev %01u.%01u)\n",
		dev_id ? "SiI9024" : "SiI9022",
		(dev_rev >> 4) & 0xf, dev_rev & 0xf);

	if (client->irq) {
		int irqf_trigger;

		/* clear pending interrupts */
		regmap_write(priv->regmap, SIL9022_INT_STATUS_REG, 0xff);
		irqf_trigger = irqd_get_trigger_type(irq_get_irq_data(client->irq));
		err = devm_request_threaded_irq(priv->dev, client->irq, NULL,
					sil9022_irq_thread,
					irqf_trigger | IRQF_ONESHOT,
					"sil9022", priv);
		if (err) {
			dev_err(priv->dev, "failed to request IRQ#%u: %d\n",
				client->irq, err);
			return err;
		}

		/* enable only connection detection for now */
		regmap_write(priv->regmap, SIL9022_INT_ENABLE_REG,
			SIL9022_HOTPLUG_EVENT | SIL9022_RECEIVER_EVENT);
	}

	return err;
}

static void sil9022_encoder_helper_dpms(struct drm_encoder *encoder, int mode)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s\n", __func__);
	return sil9022_encoder_dpms(encoder, mode);
}

static void sil9022_encoder_helper_save(struct drm_encoder *encoder)
{
}

static void sil9022_encoder_helper_restore(struct drm_encoder *encoder)
{
}

static bool sil9022_encoder_helper_mode_fixup(struct drm_encoder *encoder,
					const struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s\n", __func__);
	return true;
}

static void sil9022_encoder_helper_prepare(struct drm_encoder *encoder)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s\n", __func__);
	sil9022_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void sil9022_encoder_helper_commit(struct drm_encoder *encoder)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s\n", __func__);
	sil9022_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void sil9022_encoder_helper_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
	struct sil9022_priv *priv = encoder_to_sil9022_priv(encoder);
	dev_info(priv->dev, "%s\n", __func__);
	sil9022_encoder_mode_set(encoder, mode, adjusted_mode);
}

static struct drm_encoder_helper_funcs sil9022_encoder_helper_funcs = {
	.dpms		= sil9022_encoder_helper_dpms,
	.save		= sil9022_encoder_helper_save,
	.restore	= sil9022_encoder_helper_restore,
	.mode_fixup	= sil9022_encoder_helper_mode_fixup,
	.prepare	= sil9022_encoder_helper_prepare,
	.commit		= sil9022_encoder_helper_commit,
	.mode_set	= sil9022_encoder_helper_mode_set,
};

static struct drm_encoder_slave_funcs sil9022_encoder_slave_funcs = {
	.set_config	= sil9022_encoder_set_config,
	.dpms		= sil9022_encoder_dpms,
	.detect		= sil9022_encoder_detect,
	.get_modes	= sil9022_encoder_get_modes,
	.mode_fixup	= sil9022_encoder_mode_fixup,
	.mode_valid	= sil9022_encoder_mode_valid,
	.mode_set	= sil9022_encoder_mode_set,
};

static int sil9022_connector_get_modes(struct drm_connector *connector)
{
	struct sil9022_priv *priv = connector_to_sil9022_priv(connector);
	dev_info(priv->dev, "%s\n", __func__);

	return sil9022_encoder_get_modes(&priv->slave.base, connector);
}

static int sil9022_connector_mode_valid(struct drm_connector *connector,
					struct drm_display_mode *mode)
{
	struct sil9022_priv *priv = connector_to_sil9022_priv(connector);
	dev_info(priv->dev, "%s\n", __func__);

	return sil9022_encoder_mode_valid(&priv->slave.base, mode);
}

static struct drm_encoder *sil9022_connector_best_encoder(struct drm_connector *connector)
{
	struct sil9022_priv *priv = connector_to_sil9022_priv(connector);
	dev_info(priv->dev, "%s\n", __func__);

	return &priv->slave.base;
}

static struct drm_connector_helper_funcs sil9022_connector_helper_funcs = {
	.get_modes	= sil9022_connector_get_modes,
	.mode_valid	= sil9022_connector_mode_valid,
	.best_encoder	= sil9022_connector_best_encoder,
};

static int sil9022_encoder_init(struct i2c_client *client,
				struct drm_device *dev,
				struct drm_encoder_slave *slave)
{
	printk(KERN_INFO "%s\n", __func__);
	slave->slave_funcs = &sil9022_encoder_slave_funcs;
	slave->bus_priv = client;

	return 0;
}

static void sil9022_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs sil9022_encoder_funcs = {
	.destroy = sil9022_encoder_destroy,\
};

static enum drm_connector_status
sil9022_connector_detect(struct drm_connector *connector, bool force)
{
	struct sil9022_priv *priv = connector_to_sil9022_priv(connector);

	return sil9022_encoder_detect(&priv->slave.base, connector);
}

static void sil9022_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs sil9022_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sil9022_connector_detect,
	.destroy = sil9022_connector_destroy,
};

static int sil9022_bind(struct device *dev, struct device *master, void *data)
{
	struct sil9022_priv *priv;
	struct i2c_client *client = to_i2c_client(dev);
	struct drm_device *drm = data;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init_i2c(client, &sil9022_regmap_config);
	if (IS_ERR(priv->regmap)) {
		err = PTR_ERR(priv->regmap);
		dev_err(dev, "Failed to init regmap: %d\n", err);
		return err;
	}

	dev_set_drvdata(dev, priv);
	priv->dev = get_device(dev);

	priv->connector.interlace_allowed = 1;
	priv->slave.base.possible_crtcs = 1;

	err = sil9022_setup(priv);
	if (err) {
		put_device(priv->dev);
		return err;
	}

	priv->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
				 DRM_CONNECTOR_POLL_DISCONNECT;

	drm_encoder_helper_add(&priv->slave.base, &sil9022_encoder_helper_funcs);
	err = drm_encoder_init(drm, &priv->slave.base, &sil9022_encoder_funcs,
				DRM_MODE_ENCODER_TMDS);
	if (err)
		goto err_encoder_init;

	priv->slave.bus_priv = client;

	drm_connector_helper_add(&priv->connector, &sil9022_connector_helper_funcs);
	err = drm_connector_init(drm, &priv->connector,	&sil9022_connector_funcs,
				DRM_MODE_CONNECTOR_DVID);
	if (err)
		goto err_connector_init;

	err = drm_connector_register(&priv->connector);
	if (err)
		goto err_connector_register;

	priv->connector.encoder = &priv->slave.base;
	drm_mode_connector_attach_encoder(&priv->connector, &priv->slave.base);

	dev_info(priv->dev, "%s\n", __func__);
	return 0;

err_connector_register:
	drm_connector_cleanup(&priv->connector);
err_connector_init:
	drm_encoder_cleanup(&priv->slave.base);
err_encoder_init:
	put_device(priv->dev);
	return err;
}

static void sil9022_unbind(struct device *dev, struct device *master, void *data)
{
	struct sil9022_priv *priv = dev_get_drvdata(dev);

	drm_connector_cleanup(&priv->connector);
	drm_encoder_cleanup(&priv->slave.base);
	put_device(priv->dev);
}

static const struct component_ops sil9022_ops = {
	.bind = sil9022_bind,
	.unbind = sil9022_unbind,
};

static int sil9022_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	return component_add(&client->dev, &sil9022_ops);
}

static int sil9022_remove(struct i2c_client *client)
{
	component_del(&client->dev, &sil9022_ops);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sil9022_dt_ids[] = {
	{ .compatible = "sil,sii9022-tpi", },
	{ }
};
MODULE_DEVICE_TABLE(of, sil9022_dt_ids);
#endif

static struct i2c_device_id sil9022_ids[] = {
	{ "sil9022-tpi", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sil9022_ids);

static struct drm_i2c_encoder_driver sil9022_driver = {
	.i2c_driver = {
		.probe	= sil9022_probe,
		.remove	= sil9022_remove,
		.driver	= {
			.name = "sil9022-tpi",
			.of_match_table = of_match_ptr(sil9022_dt_ids),
		},
		.id_table = sil9022_ids,
		.class	= I2C_CLASS_DDC,
	},
	.encoder_init = sil9022_encoder_init,
};

static int __init sil9022_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &sil9022_driver);
}

static void __exit sil9022_exit(void)
{
	drm_i2c_encoder_unregister(&sil9022_driver);
}

module_init(sil9022_init);
module_exit(sil9022_exit);

MODULE_ALIAS(I2C_MODULE_PREFIX "sil9022-tpi");
MODULE_AUTHOR("Liviu Dudau <Liviu.Dudau@arm.com>");
MODULE_DESCRIPTION("Silicon Image SiI9022 HDMI transmitter driver");
MODULE_LICENSE("GPL v2");

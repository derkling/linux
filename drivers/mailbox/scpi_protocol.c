/*
 * System Control and Power Interface (SCPI) Message Protocol driver
 *
 * SCPI Message Protocol is used between the System Control Processor(SCP)
 * and the Application Processors(AP). The Message Handling Unit(MHU)
 * provides a mechanism for inter-processor communication between SCP's
 * Cortex M3 and AP.
 *
 * SCP offers control and management of the core/cluster power states,
 * various power domain DVFS including the core/cluster, certain system
 * clocks configuration, thermal sensors and many others.
 *
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitmap.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/printk.h>
#include <linux/scpi_protocol.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define CMD_ID_SHIFT		0
#define CMD_ID_MASK		0xff
#define CMD_TOKEN_ID_SHIFT	8
#define CMD_TOKEN_ID_MASK	0xff
#define CMD_DATA_SIZE_SHIFT	20
#define CMD_DATA_SIZE_MASK	0x1ff
#define PACK_SCPI_CMD(cmd_id, token, tx_sz)				\
	((((cmd_id) & CMD_ID_MASK) << CMD_ID_SHIFT) |			\
	(((token) & CMD_TOKEN_ID_MASK) << CMD_TOKEN_ID_SHIFT) |	\
	(((tx_sz) & CMD_DATA_SIZE_MASK) << CMD_DATA_SIZE_SHIFT))

#define CMD_SIZE(cmd)	(((cmd) >> CMD_DATA_SIZE_SHIFT) & CMD_DATA_SIZE_MASK)
#define CMD_UNIQ_MASK	(CMD_TOKEN_ID_MASK << CMD_TOKEN_ID_SHIFT | CMD_ID_MASK)
#define CMD_XTRACT_UNIQ(cmd)	((cmd) & CMD_UNIQ_MASK)

#define MAX_DVFS_DOMAINS	3
#define MAX_DVFS_OPPS		8
#define DVFS_LATENCY(hdr)	(le32_to_cpu(hdr) >> 16)
#define DVFS_OPP_COUNT(hdr)	((le32_to_cpu(hdr) >> 8) & 0xff)

#define PROTOCOL_REV_MINOR_BITS	16
#define PROTOCOL_REV_MINOR_MASK	((1U << PROTOCOL_REV_MINOR_BITS) - 1)
#define PROTOCOL_REV_MAJOR(x)	((x) >> PROTOCOL_REV_MINOR_BITS)
#define PROTOCOL_REV_MINOR(x)	((x) & PROTOCOL_REV_MINOR_MASK)

#define FW_REV_MINOR_BITS	24
#define FW_REV_PATCH_BITS	16
#define FW_REV_PATCH_MASK	((1U << FW_REV_PATCH_BITS) - 1)
#define FW_REV_MINOR_MASK	((1U << FW_REV_MINOR_BITS) - 1)
#define FW_REV_MAJOR(x)		((x) >> FW_REV_MINOR_BITS)
#define FW_REV_MINOR(x)		(((x) & FW_REV_MINOR_MASK) >> FW_REV_PATCH_BITS)
#define FW_REV_PATCH(x)		((x) & FW_REV_PATCH_MASK)

enum scpi_error_codes {
	SCPI_SUCCESS = 0, /* Success */
	SCPI_ERR_PARAM = 1, /* Invalid parameter(s) */
	SCPI_ERR_ALIGN = 2, /* Invalid alignment */
	SCPI_ERR_SIZE = 3, /* Invalid size */
	SCPI_ERR_HANDLER = 4, /* Invalid handler/callback */
	SCPI_ERR_ACCESS = 5, /* Invalid access/permission denied */
	SCPI_ERR_RANGE = 6, /* Value out of range */
	SCPI_ERR_TIMEOUT = 7, /* Timeout has occurred */
	SCPI_ERR_NOMEM = 8, /* Invalid memory area or pointer */
	SCPI_ERR_PWRSTATE = 9, /* Invalid power state */
	SCPI_ERR_SUPPORT = 10, /* Not supported or disabled */
	SCPI_ERR_DEVICE = 11, /* Device error */
	SCPI_ERR_MAX
};

enum scpi_std_cmd {
	SCPI_CMD_INVALID		= 0x00,
	SCPI_CMD_SCPI_READY		= 0x01,
	SCPI_CMD_SCPI_CAPABILITIES	= 0x02,
	SCPI_CMD_EVENT			= 0x03,
	SCPI_CMD_SET_CSS_PWR_STATE	= 0x04,
	SCPI_CMD_GET_CSS_PWR_STATE	= 0x05,
	SCPI_CMD_CFG_PWR_STATE_STAT	= 0x06,
	SCPI_CMD_GET_PWR_STATE_STAT	= 0x07,
	SCPI_CMD_SYS_PWR_STATE		= 0x08,
	SCPI_CMD_L2_READY		= 0x09,
	SCPI_CMD_SET_AP_TIMER		= 0x0a,
	SCPI_CMD_CANCEL_AP_TIME		= 0x0b,
	SCPI_CMD_DVFS_CAPABILITIES	= 0x0c,
	SCPI_CMD_GET_DVFS_INFO		= 0x0d,
	SCPI_CMD_SET_DVFS		= 0x0e,
	SCPI_CMD_GET_DVFS		= 0x0f,
	SCPI_CMD_GET_DVFS_STAT		= 0x10,
	SCPI_CMD_SET_RTC		= 0x11,
	SCPI_CMD_GET_RTC		= 0x12,
	SCPI_CMD_CLOCK_CAPABILITIES	= 0x13,
	SCPI_CMD_GET_CLOCK_RANGE	= 0x14,
	SCPI_CMD_SET_CLOCK_VALUE	= 0x15,
	SCPI_CMD_GET_CLOCK_VALUE	= 0x16,
	SCPI_CMD_PSU_CAPABILITIES	= 0x17,
	SCPI_CMD_SET_PSU		= 0x18,
	SCPI_CMD_GET_PSU		= 0x19,
	SCPI_CMD_SENSOR_CAPABILITIES	= 0x1a,
	SCPI_CMD_SENSOR_INFO		= 0x1b,
	SCPI_CMD_SENSOR_VALUE		= 0x1c,
	SCPI_CMD_SENSOR_CFG_PERIODIC	= 0x1d,
	SCPI_CMD_SENSOR_CFG_BOUNDS	= 0x1e,
	SCPI_CMD_SENSOR_ASYNC_VALUE	= 0x1f,
	SCPI_CMD_SET_DEVICE_PWR_STATE	= 0x20,
	SCPI_CMD_GET_DEVICE_PWR_STATE	= 0x21,
	SCPI_CMD_COUNT
};

/*
 * List of commands that can be sent on high priority channel
 * This will be saved as bitmap in scpi_info->cmd_priority
 */
static int hpriority_cmds[] = {
	SCPI_CMD_GET_CSS_PWR_STATE,
	SCPI_CMD_CFG_PWR_STATE_STAT,
	SCPI_CMD_GET_PWR_STATE_STAT,
	SCPI_CMD_SET_DVFS,
	SCPI_CMD_GET_DVFS,
	SCPI_CMD_SET_RTC,
	SCPI_CMD_GET_RTC,
	SCPI_CMD_SET_CLOCK_VALUE,
	SCPI_CMD_GET_CLOCK_VALUE,
	SCPI_CMD_SET_PSU,
	SCPI_CMD_GET_PSU,
	SCPI_CMD_SENSOR_VALUE,
	SCPI_CMD_SENSOR_CFG_PERIODIC,
	SCPI_CMD_SENSOR_CFG_BOUNDS,
	SCPI_CMD_SET_DEVICE_PWR_STATE,
	SCPI_CMD_GET_DEVICE_PWR_STATE,
};

/*
 * +---------------+-------+----------------+
 * |    Payload    | Offset|  Driver View   |
 * +---------------+-------+----------------+
 * |  SCP->AP Low  | 0x000 |  RX_PAYLOAD(L) |
 * |  SCP->AP High | 0x400 |  RX_PAYLOAD(H) |
 * +---------------+-------+----------------+
 * |  AP->SCP Low  | 0x200 |  TX_PAYLOAD(H) |
 * |  AP->SCP High | 0x600 |  TX_PAYLOAD(H) |
 * +---------------+-------+----------------+
*/
#define RX_PAYLOAD(chan)	((chan) * 0x400)
#define TX_PAYLOAD(chan)	((chan) * 0x400 + 0x200)
#define MAX_RX_TIMEOUT		(msecs_to_jiffies(20))

struct scpi_xfer {
	u32 cmd; /* this has to be first element */
	bool high_pri;
	const void *tx_buf;
	void *rx_buf;
	unsigned int tx_len;
	struct list_head node;
	struct completion done;
};

struct scpi_chan {
	struct mbox_client cl;
	struct mbox_chan *chan;
	void __iomem *tx_payload;
	void __iomem *rx_payload;
	struct list_head rx_pending;
	struct list_head xfers_list;
	struct scpi_xfer *xfers;
	spinlock_t rx_lock; /* locking for the rx pending list */
	struct mutex xfers_lock;
	atomic_t token;
};

struct scpi_drvinfo {
	u32 protocol_version;
	u32 firmware_version;
	int chan_count;
	void __iomem *payload_base;
	struct scpi_ops *scpi_ops;
	struct scpi_chan *channels;
	struct scpi_dvfs_info *dvfs[MAX_DVFS_DOMAINS];
	DECLARE_BITMAP(cmd_priority, SCPI_CMD_COUNT);
};

/*
 * The SCP firmware only executes in little-endian mode, so any buffers
 * shared through SCPI should have their contents converted to little-endian
 */
struct scp_capabilities {
	__le32 status;
	__le32 protocol_version;
	__le32 event_version;
	__le32 platform_version;
	__le32 commands[4];
} __packed;

struct clk_get_range {
	__le32 status;
	__le32 min_rate;
	__le32 max_rate;
} __packed;

struct clk_get_value {
	__le32 status;
	__le32 rate;
} __packed;

struct clk_set_value {
	__le32 rate;
	__le16 id;
} __packed;

struct dvfs_info {
	__le32 status;
	__le32 header;
	struct {
		__le32 freq;
		__le32 m_volt;
	} opps[MAX_DVFS_OPPS];
} __packed;

struct dvfs_get {
	__le32 status;
	u8 index;
} __packed;

struct dvfs_set {
	u8 domain;
	u8 index;
} __packed;

static struct scpi_drvinfo *scpi_info;

static int scpi_linux_errmap[SCPI_ERR_MAX] = {
	/* better than switch case as long as return value is continuous */
	0, /* SCPI_SUCCESS */
	-EINVAL, /* SCPI_ERR_PARAM */
	-ENOEXEC, /* SCPI_ERR_ALIGN */
	-EMSGSIZE, /* SCPI_ERR_SIZE */
	-EINVAL, /* SCPI_ERR_HANDLER */
	-EACCES, /* SCPI_ERR_ACCESS */
	-ERANGE, /* SCPI_ERR_RANGE */
	-ETIMEDOUT, /* SCPI_ERR_TIMEOUT */
	-ENOMEM, /* SCPI_ERR_NOMEM */
	-EINVAL, /* SCPI_ERR_PWRSTATE */
	-EOPNOTSUPP, /* SCPI_ERR_SUPPORT */
	-EIO, /* SCPI_ERR_DEVICE */
};

static inline int scpi_to_linux_errno(int errno)
{
	if (errno >= SCPI_SUCCESS && errno < SCPI_ERR_MAX)
		return scpi_linux_errmap[errno];
	return -EIO;
}

static void scpi_process_cmd(struct scpi_chan *ch, u32 c)
{
	struct scpi_xfer *t = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ch->rx_lock, flags);
	if (!list_empty(&ch->rx_pending)) {
		list_for_each_entry(t, &ch->rx_pending, node)
			if (CMD_XTRACT_UNIQ(t->cmd) == CMD_XTRACT_UNIQ(c)) {
				list_del(&t->node);
				break;
			}
	}
	/* check if wait_for_completion is in progress or timed-out */
	if (t && !completion_done(&t->done)) {
		memcpy_fromio(t->rx_buf, ch->rx_payload, CMD_SIZE(c));
		complete(&t->done);
	}
	spin_unlock_irqrestore(&ch->rx_lock, flags);
}

static void scpi_handle_remote_msg(struct mbox_client *c, void *msg)
{
	struct scpi_chan *ch = container_of(c, struct scpi_chan, cl);

	scpi_process_cmd(ch, *(u32 *)msg);
}

static void scpi_tx_prepare(struct mbox_client *c, void *msg)
{
	unsigned long flags;
	struct scpi_xfer *t = msg;
	struct scpi_chan *ch = container_of(c, struct scpi_chan, cl);

	if (t->tx_buf)
		memcpy_toio(ch->tx_payload, t->tx_buf, t->tx_len);
	if (t->rx_buf) {
		spin_lock_irqsave(&ch->rx_lock, flags);
		list_add_tail(&t->node, &ch->rx_pending);
		spin_unlock_irqrestore(&ch->rx_lock, flags);
	}
}

static struct scpi_xfer *get_scpi_xfer(struct scpi_chan *ch)
{
	struct scpi_xfer *t;

	mutex_lock(&ch->xfers_lock);
	if (list_empty(&ch->xfers_list)) {
		mutex_unlock(&ch->xfers_lock);
		return ERR_PTR(-ENOMEM);
	}
	t = list_first_entry(&ch->xfers_list, struct scpi_xfer, node);
	list_del(&t->node);
	mutex_unlock(&ch->xfers_lock);
	return t;
}

static void put_scpi_xfer(struct scpi_xfer *t, struct scpi_chan *ch)
{
	mutex_lock(&ch->xfers_lock);
	list_add_tail(&t->node, &ch->xfers_list);
	mutex_unlock(&ch->xfers_lock);
}

static int
scpi_send_message(u8 cmd, void *tx_buf, unsigned int len, void *rx_buf)
{
	int ret;
	u8 token;
	struct scpi_xfer *msg;
	struct scpi_chan *scpi_chan;

	scpi_chan = test_bit(cmd, scpi_info->cmd_priority) ?
		    scpi_info->channels + 1 : scpi_info->channels;

	msg = get_scpi_xfer(scpi_chan);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	token = atomic_inc_return(&scpi_chan->token) & CMD_TOKEN_ID_MASK;

	msg->cmd = PACK_SCPI_CMD(cmd, token, len);
	msg->tx_buf = tx_buf;
	msg->tx_len = len;
	msg->rx_buf = rx_buf;
	init_completion(&msg->done);

	ret = mbox_send_message(scpi_chan->chan, msg);
	if (ret < 0 || !rx_buf)
		goto out;

	if (!wait_for_completion_timeout(&msg->done, MAX_RX_TIMEOUT))
		ret = -ETIMEDOUT;
	else
		/* first status word */
		ret = le32_to_cpu(*(__le32 *)(msg->rx_buf));
out:
	if (ret < 0 && rx_buf) /* remove entry from the list if timed-out */
		scpi_process_cmd(scpi_chan, msg->cmd);
	put_scpi_xfer(msg, scpi_chan);
	/* SCPI error codes > 0, translate them to Linux scale*/
	return ret > 0 ? scpi_to_linux_errno(ret) : ret;
}

static u32 scpi_get_version(void)
{
	return scpi_info->protocol_version;
}

static int
scpi_clk_get_range(u16 clk_id, unsigned long *min, unsigned long *max)
{
	int ret;
	struct clk_get_range clk;
	__le16 le_clk_id = cpu_to_le16(clk_id);

	ret = scpi_send_message(SCPI_CMD_GET_CLOCK_RANGE,
				&le_clk_id, sizeof(le_clk_id), &clk);
	if (!ret) {
		*min = le32_to_cpu(clk.min_rate);
		*max = le32_to_cpu(clk.max_rate);
	}
	return ret;
}

static unsigned long scpi_clk_get_val(u16 clk_id)
{
	int ret;
	struct clk_get_value clk;
	__le16 le_clk_id = cpu_to_le16(clk_id);

	ret = scpi_send_message(SCPI_CMD_GET_CLOCK_VALUE,
				&le_clk_id, sizeof(le_clk_id), &clk);
	return ret ? ret : le32_to_cpu(clk.rate);
}

static int scpi_clk_set_val(u16 clk_id, unsigned long rate)
{
	int stat;
	struct clk_set_value clk = { cpu_to_le16(clk_id), cpu_to_le32(rate) };

	return scpi_send_message(SCPI_CMD_SET_CLOCK_VALUE,
				 &clk, sizeof(clk), &stat);
}

static int scpi_dvfs_get_idx(u8 domain)
{
	int ret;
	struct dvfs_get dvfs;

	ret = scpi_send_message(SCPI_CMD_GET_DVFS,
				&domain, sizeof(domain), &dvfs);
	return ret ? ret : dvfs.index;
}

static int scpi_dvfs_set_idx(u8 domain, u8 index)
{
	int stat;
	struct dvfs_set dvfs = {domain, index};

	return scpi_send_message(SCPI_CMD_SET_DVFS,
				 &dvfs, sizeof(dvfs), &stat);
}

static struct scpi_dvfs_info *scpi_dvfs_get_info(u8 domain)
{
	struct scpi_dvfs_info *info;
	struct scpi_opp *opp;
	struct dvfs_info buf;
	int ret, i;

	if (domain >= MAX_DVFS_DOMAINS)
		return ERR_PTR(-EINVAL);

	if (scpi_info->dvfs[domain])	/* data already populated */
		return scpi_info->dvfs[domain];

	ret = scpi_send_message(SCPI_CMD_GET_DVFS_INFO, &domain,
				sizeof(domain), &buf);

	if (ret)
		return ERR_PTR(ret);

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	info->count = DVFS_OPP_COUNT(buf.header);
	info->latency = DVFS_LATENCY(buf.header) * 1000; /* uS to nS */

	info->opps = kcalloc(info->count, sizeof(*opp), GFP_KERNEL);
	if (!info->opps) {
		kfree(info);
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0, opp = info->opps; i < info->count; i++, opp++) {
		opp->freq = le32_to_cpu(buf.opps[i].freq);
		opp->m_volt = le32_to_cpu(buf.opps[i].m_volt);
	}

	scpi_info->dvfs[domain] = info;
	return info;
}

static struct scpi_ops scpi_ops = {
	.get_version = scpi_get_version,
	.clk_get_range = scpi_clk_get_range,
	.clk_get_val = scpi_clk_get_val,
	.clk_set_val = scpi_clk_set_val,
	.dvfs_get_idx = scpi_dvfs_get_idx,
	.dvfs_set_idx = scpi_dvfs_set_idx,
	.dvfs_get_info = scpi_dvfs_get_info,
};

struct scpi_ops *get_scpi_ops(void)
{
	return scpi_info ? scpi_info->scpi_ops : NULL;
}
EXPORT_SYMBOL_GPL(get_scpi_ops);

static int scpi_init_versions(struct scpi_drvinfo *info)
{
	int ret;
	struct scp_capabilities caps;

	ret = scpi_send_message(SCPI_CMD_SCPI_CAPABILITIES, NULL, 0, &caps);
	if (!ret) {
		info->protocol_version = le32_to_cpu(caps.protocol_version);
		info->firmware_version = le32_to_cpu(caps.platform_version);
	}
	return ret;
}

static void scpi_free_channels(struct scpi_chan *pchan, int count)
{
	int i;

	for (i = 0; i < count && pchan->chan; i++, pchan++) {
		mbox_free_channel(pchan->chan);
		kfree(pchan->xfers);
	}
}

static ssize_t protocol_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct scpi_drvinfo *scpi_info = dev_get_drvdata(dev);

	return sprintf(buf, "%d.%d\n",
		       PROTOCOL_REV_MAJOR(scpi_info->protocol_version),
		       PROTOCOL_REV_MINOR(scpi_info->protocol_version));
}
static DEVICE_ATTR_RO(protocol_version);

static ssize_t firmware_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct scpi_drvinfo *scpi_info = dev_get_drvdata(dev);

	return sprintf(buf, "%d.%d.%d\n",
		       FW_REV_MAJOR(scpi_info->firmware_version),
		       FW_REV_MINOR(scpi_info->firmware_version),
		       FW_REV_PATCH(scpi_info->firmware_version));
}
static DEVICE_ATTR_RO(firmware_version);

static int scpi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scpi_drvinfo *scpi_info = platform_get_drvdata(pdev);

	of_platform_depopulate(dev);
	device_remove_file(dev, &dev_attr_protocol_version);
	device_remove_file(dev, &dev_attr_firmware_version);
	scpi_free_channels(scpi_info->channels, scpi_info->chan_count);
	platform_set_drvdata(pdev, NULL);

	devm_iounmap(dev, scpi_info->payload_base);
	devm_kfree(dev, scpi_info->channels);
	devm_kfree(dev, scpi_info);
	scpi_info = NULL;

	return 0;
}

#define MAX_SCPI_XFERS		10
static int scpi_alloc_xfer_list(struct scpi_chan *ch)
{
	int i;
	struct scpi_xfer *xfers;

	xfers = kcalloc(MAX_SCPI_XFERS, sizeof(*xfers), GFP_KERNEL);
	if (!xfers)
		return -ENOMEM;

	ch->xfers = xfers;
	for (i = 0; i < MAX_SCPI_XFERS; i++, xfers++)
		list_add_tail(&xfers->node, &ch->xfers_list);
	return 0;
}

static int scpi_probe(struct platform_device *pdev)
{
	int count, idx, ret;
	struct resource res;
	struct scpi_chan *scpi_chan;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	scpi_info = devm_kzalloc(dev, sizeof(*scpi_info), GFP_KERNEL);
	if (!scpi_info) {
		dev_err(dev, "failed to allocate memory for scpi drvinfo\n");
		return -ENOMEM;
	}

	count = of_count_phandle_with_args(np, "mboxes", "#mbox-cells");
	if (count < 0) {
		dev_err(dev, "no mboxes property in '%s'\n", np->full_name);
		return -ENODEV;
	}

	scpi_chan = devm_kcalloc(dev, count, sizeof(*scpi_chan), GFP_KERNEL);
	if (!scpi_chan) {
		dev_err(dev, "failed to allocate memory scpi chaninfo\n");
		return -ENOMEM;
	}

	if (of_address_to_resource(of_parse_phandle(np, "shmem", 0), 0, &res)) {
		dev_err(dev, "failed to get SCPI payload memory resource\n");
		return -EINVAL;
	}

	scpi_info->payload_base = devm_ioremap(dev, res.start,
					       resource_size(&res));
	if (!scpi_info->payload_base) {
		dev_err(dev, "failed to request or ioremap SCPI payload\n");
		return -EADDRNOTAVAIL;
	}

	scpi_info->channels = scpi_chan;
	scpi_info->chan_count = count;
	platform_set_drvdata(pdev, scpi_info);

	for (idx = 0; idx < count; idx++) {
		struct scpi_chan *pchan = scpi_chan + idx;
		struct mbox_client *cl = &pchan->cl;

		cl->dev = dev;
		cl->rx_callback = scpi_handle_remote_msg;
		cl->tx_prepare = scpi_tx_prepare;
		cl->tx_block = true;
		cl->tx_tout = 50;
		cl->knows_txdone = false; /* controller can ack */

		INIT_LIST_HEAD(&pchan->rx_pending);
		INIT_LIST_HEAD(&pchan->xfers_list);
		spin_lock_init(&pchan->rx_lock);
		mutex_init(&pchan->xfers_lock);
		pchan->tx_payload = scpi_info->payload_base + TX_PAYLOAD(idx);
		pchan->rx_payload = scpi_info->payload_base + RX_PAYLOAD(idx);

		ret = scpi_alloc_xfer_list(pchan);
		if (!ret) {
			pchan->chan = mbox_request_channel(cl, idx);
			if (!IS_ERR(pchan->chan))
				continue;
			ret = -EPROBE_DEFER;
			dev_err(dev, "failed to acquire channel#%d\n", idx);
		}
		scpi_free_channels(scpi_chan, idx);
		scpi_info = NULL;
		return ret;
	}

	for (idx = 0; idx < ARRAY_SIZE(hpriority_cmds); idx++)
		set_bit(hpriority_cmds[idx], scpi_info->cmd_priority);

	ret = scpi_init_versions(scpi_info);
	if (ret) {
		dev_err(dev, "incorrect or no SCP firmware found\n");
		scpi_remove(pdev);
		return ret;
	}

	_dev_info(dev, "SCP Protocol %d.%d Firmware %d.%d.%d version\n",
		  PROTOCOL_REV_MAJOR(scpi_info->protocol_version),
		  PROTOCOL_REV_MINOR(scpi_info->protocol_version),
		  FW_REV_MAJOR(scpi_info->firmware_version),
		  FW_REV_MINOR(scpi_info->firmware_version),
		  FW_REV_PATCH(scpi_info->firmware_version));
	scpi_info->scpi_ops = &scpi_ops;
	device_create_file(dev, &dev_attr_protocol_version);
	device_create_file(dev, &dev_attr_firmware_version);
	return of_platform_populate(dev->of_node, NULL, NULL, dev);
}

static struct of_device_id scpi_of_match[] = {
	{.compatible = "arm,scpi"},
	{},
};

MODULE_DEVICE_TABLE(of, scpi_of_match);

static struct platform_driver scpi_driver = {
	.driver = {
		.name = "scpi_protocol",
		.of_match_table = scpi_of_match,
	},
	.probe = scpi_probe,
	.remove = scpi_remove,
};
module_platform_driver(scpi_driver);

MODULE_AUTHOR("Sudeep Holla <sudeep.holla@arm.com>");
MODULE_DESCRIPTION("ARM SCPI mailbox protocol driver");
MODULE_LICENSE("GPL");

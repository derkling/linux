// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013-2015 Fujitsu Semiconductor Ltd.
 * Copyright (C) 2015 Linaro Ltd.
 * Author: Jassi Brar <jaswinder.singh@linaro.org>
 */

#include <linux/amba/bus.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#define MHU_NUM_PCHANS	3	/* Secure, Non-Secure High and Low Priority */
#define MHU_CHAN_MAX	20	/* Max channels to save on unused RAM */

#define MHU_V2_COMPAT_STR	"arm,mhuv2-doorbell"

enum mhu_regs {
	MHU_REG_STAT,
	MHU_REG_SET,
	MHU_REG_CLR,
	MHU_REG_END
};

enum mhu_access_regs {
	MHU_REG_ACC_REQ,
	MHU_REG_ACC_RDY,
	MHU_REG_ACC_END
};

enum mhu_channels {
	MHU_CHAN_LOW,
	MHU_CHAN_HIGH,
	MHU_CHAN_SEC,
	MHU_CHAN_END
};

/**
 * ARM MHU Mailbox device specific data
 *
 * @regs: MHU version specific array of register offset for STAT,
 *        SET & CLEAR registers.
 * @chans: MHU version specific array of channel offset for Low
 *         Priority, High Priority & Secure channels.
 * @acc_regs: An array of access register offsets.
 * @tx_reg_off: Offset for TX register.
 */
struct mhu_data {
	int regs[MHU_REG_END]; /* STAT, SET, CLEAR */
	int chans[MHU_CHAN_END]; /* LP, HP, Sec */
	int acc_regs[MHU_REG_ACC_END];  /* ACCESS_REQUEST, ACCESS_READY */
	long int tx_reg_off;
};

struct mhu_link {
	unsigned irq;
	void __iomem *tx_reg;
	void __iomem *rx_reg;
};

struct arm_mhu {
	void __iomem *base;
	struct mhu_link mlink[MHU_NUM_PCHANS];
	struct mbox_controller mbox;
	struct mhu_data *drvdata;
	struct device *dev;
	const char *name;
};

/**
 * ARM MHU Mailbox platform specific configuration
 *
 * @num_pchans: Maximum number of physical channels
 * @num_doorbells: Maximum number of doorbells per physical channel
 */
struct mhu_mbox_pdata {
	unsigned int num_pchans;
	unsigned int num_doorbells;
	bool support_doorbells;
};

/**
 * ARM MHU Mailbox allocated channel information
 *
 * @mhu: Pointer to parent mailbox device
 * @pchan: Physical channel within which this doorbell resides in
 * @doorbell: doorbell number pertaining to this channel
 */
struct mhu_channel {
	struct arm_mhu *mhu;
	unsigned int pchan;
	unsigned int doorbell;
};

static inline struct mbox_chan *
mhu_mbox_to_channel(struct mbox_controller *mbox,
		    unsigned int pchan, unsigned int doorbell)
{
	int i;
	struct mhu_channel *chan_info;

	for (i = 0; i < mbox->num_chans; i++) {
		chan_info = mbox->chans[i].con_priv;
		if (chan_info && chan_info->pchan == pchan &&
		    chan_info->doorbell == doorbell)
			return &mbox->chans[i];
	}

	dev_err(mbox->dev,
		"Channel not registered: physical channel: %d doorbell: %d\n",
		pchan, doorbell);

	return NULL;
}

static void mhu_mbox_clear_irq(struct mbox_chan *chan)
{
	struct mhu_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].rx_reg;
	struct mhu_data *mdata = chan_info->mhu->drvdata;

	writel_relaxed(BIT(chan_info->doorbell),
			base + mdata->regs[MHU_REG_CLR]);
}

static unsigned int mhu_mbox_irq_to_pchan_num(struct arm_mhu *mhu, int irq)
{
	unsigned int pchan;
	struct mhu_mbox_pdata *pdata = dev_get_platdata(mhu->dev);

	for (pchan = 0; pchan < pdata->num_pchans; pchan++)
		if (mhu->mlink[pchan].irq == irq)
			break;
	return pchan;
}

static struct mbox_chan *mhu_mbox_irq_to_channel(struct arm_mhu *mhu,
						 unsigned int pchan)
{
	unsigned long bits;
	unsigned int doorbell;
	struct mbox_chan *chan = NULL;
	struct mbox_controller *mbox = &mhu->mbox;
	void __iomem *base = mhu->mlink[pchan].rx_reg;

	bits = readl_relaxed(base + mhu->drvdata->regs[MHU_REG_STAT]);
	if (!bits)
		/* No IRQs fired in specified physical channel */
		return NULL;

	/* An IRQ has fired, find the associated channel */
	for (doorbell = 0; bits; doorbell++) {
		if (!test_and_clear_bit(doorbell, &bits))
			continue;

		chan = mhu_mbox_to_channel(mbox, pchan, doorbell);
		if (chan)
			break;
	}

	return chan;
}

static irqreturn_t mhu_mbox_thread_handler(int irq, void *data)
{
	struct mbox_chan *chan;
	struct arm_mhu *mhu = data;
	unsigned int pchan = mhu_mbox_irq_to_pchan_num(mhu, irq);

	while (NULL != (chan = mhu_mbox_irq_to_channel(mhu, pchan))) {
		mbox_chan_received_data(chan, NULL);
		mhu_mbox_clear_irq(chan);
	}

	return IRQ_HANDLED;
}

static bool mhu_doorbell_last_tx_done(struct mbox_chan *chan)
{
	struct mhu_channel *chan_info = chan->con_priv;
	struct mhu_data *mdata = chan_info->mhu->drvdata;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;

	if (readl_relaxed(base + mdata->regs[MHU_REG_STAT])
			& BIT(chan_info->doorbell))
		return false;

	return true;
}

static int mhu_doorbell_send_data(struct mbox_chan *chan, void *data)
{
	struct mhu_channel *chan_info = chan->con_priv;
	struct mhu_data *mdata = chan_info->mhu->drvdata;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;

	/* Send event to co-processor */
	writel_relaxed(BIT(chan_info->doorbell),
			base + mdata->regs[MHU_REG_SET]);

	return 0;
}

static int mhu_doorbell_startup(struct mbox_chan *chan)
{
	struct mhu_channel *chan_info = chan->con_priv;
	struct mhu_data *mdata = chan_info->mhu->drvdata;
	struct device_node *np = chan_info->mhu->dev->of_node;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;

	if (of_device_is_compatible(np, MHU_V2_COMPAT_STR))
		writel_relaxed(0x1, base
			+ (mdata->acc_regs[MHU_REG_ACC_REQ]
			 - (mdata->chans[chan_info->pchan])));

	mhu_mbox_clear_irq(chan);
	return 0;
}

static void mhu_doorbell_shutdown(struct mbox_chan *chan)
{
	struct mhu_channel *chan_info = chan->con_priv;
	struct mbox_controller *mbox = &chan_info->mhu->mbox;
	struct mhu_data *mdata = chan_info->mhu->drvdata;
	struct device_node *np = chan_info->mhu->dev->of_node;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;
	int i;

	for (i = 0; i < mbox->num_chans; i++)
		if (chan == &mbox->chans[i])
			break;

	if (mbox->num_chans == i) {
		dev_warn(mbox->dev, "Request to free non-existent channel\n");
		return;
	}

	if (of_device_is_compatible(np, MHU_V2_COMPAT_STR))
		writel_relaxed(0x0, base
			+ (mdata->acc_regs[MHU_REG_ACC_REQ]
			 - (mdata->chans[chan_info->pchan])));

	/* Reset channel */
	mhu_mbox_clear_irq(chan);
	chan->con_priv = NULL;
}

static struct mbox_chan *mhu_mbox_xlate(struct mbox_controller *mbox,
					const struct of_phandle_args *spec)
{
	struct arm_mhu *mhu = dev_get_drvdata(mbox->dev);
	struct mhu_mbox_pdata *pdata = dev_get_platdata(mhu->dev);
	struct mhu_channel *chan_info;
	struct mbox_chan *chan = NULL;
	unsigned int pchan = spec->args[0];
	unsigned int doorbell = pdata->support_doorbells ? spec->args[1] : 0;
	int i;

	/* Bounds checking */
	if (pchan >= pdata->num_pchans || doorbell >= pdata->num_doorbells) {
		dev_err(mbox->dev,
			"Invalid channel requested pchan: %d doorbell: %d\n",
			pchan, doorbell);
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < mbox->num_chans; i++) {
		chan_info = mbox->chans[i].con_priv;

		/* Is requested channel free? */
		if (chan_info &&
		    mbox->dev == chan_info->mhu->dev &&
		    pchan == chan_info->pchan &&
		    doorbell == chan_info->doorbell) {
			dev_err(mbox->dev, "Channel in use\n");
			return ERR_PTR(-EBUSY);
		}

		/*
		 * Find the first free slot, then continue checking
		 * to see if requested channel is in use
		 */
		if (!chan && !chan_info)
			chan = &mbox->chans[i];
	}

	if (!chan) {
		dev_err(mbox->dev, "No free channels left\n");
		return ERR_PTR(-EBUSY);
	}

	chan_info = devm_kzalloc(mbox->dev, sizeof(*chan_info), GFP_KERNEL);
	if (!chan_info)
		return ERR_PTR(-ENOMEM);

	chan_info->mhu = mhu;
	chan_info->pchan = pchan;
	chan_info->doorbell = doorbell;

	chan->con_priv = chan_info;

	dev_dbg(mbox->dev, "mbox: %s, created channel phys: %d doorbell: %d\n",
		mhu->name, pchan, doorbell);

	return chan;
}

static irqreturn_t mhu_rx_interrupt(int irq, void *p)
{
	struct arm_mhu *mhu = p;
	unsigned int pchan = mhu_mbox_irq_to_pchan_num(mhu, irq);
	struct mbox_chan *chan = mhu_mbox_to_channel(&mhu->mbox, pchan, 0);
	struct mhu_data *mdata = mhu->drvdata;
	void __iomem *base = mhu->mlink[pchan].rx_reg;
	u32 val;

	val = readl_relaxed(base + mdata->regs[MHU_REG_STAT]);
	if (!val)
		return IRQ_NONE;

	mbox_chan_received_data(chan, (void *)&val);

	writel_relaxed(val, base + mdata->regs[MHU_REG_CLR]);

	return IRQ_HANDLED;
}

static bool mhu_last_tx_done(struct mbox_chan *chan)
{
	struct mhu_channel *chan_info = chan->con_priv;
	struct mhu_data *mdata = chan_info->mhu->drvdata;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;
	u32 val = readl_relaxed(base + mdata->regs[MHU_REG_STAT]);

	return (val == 0);
}

static int mhu_send_data(struct mbox_chan *chan, void *data)
{
	struct mhu_channel *chan_info = chan->con_priv;
	struct mhu_data *mdata = chan_info->mhu->drvdata;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;
	u32 *arg = data;

	writel_relaxed(*arg, base + mdata->regs[MHU_REG_SET]);

	return 0;
}

static int mhu_startup(struct mbox_chan *chan)
{
	struct mhu_channel *chan_info = chan->con_priv;
	struct device_node *np = chan_info->mhu->dev->of_node;
	struct mhu_data *mdata = chan_info->mhu->drvdata;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;
	u32 val;

	if (of_device_is_compatible(np, MHU_V2_COMPAT_STR))
		writel_relaxed(0x1, base
			+ (mdata->acc_regs[MHU_REG_ACC_REQ]
			- (mdata->chans[chan_info->pchan])));

	val = readl_relaxed(base + mdata->regs[MHU_REG_STAT]);
	writel_relaxed(val, base + mdata->regs[MHU_REG_CLR]);

	return 0;
}

static const struct mbox_chan_ops mhu_ops = {
	.send_data = mhu_send_data,
	.startup = mhu_startup,
	.last_tx_done = mhu_last_tx_done,
};

static const struct mbox_chan_ops mhu_doorbell_ops = {
	.send_data = mhu_doorbell_send_data,
	.startup = mhu_doorbell_startup,
	.shutdown = mhu_doorbell_shutdown,
	.last_tx_done = mhu_doorbell_last_tx_done,
};

static const struct mhu_mbox_pdata arm_mhu_pdata = {
	.num_pchans = 3,
	.num_doorbells = 1,
	.support_doorbells = false,
};

static const struct mhu_mbox_pdata arm_mhu_doorbell_pdata = {
	.num_pchans = 2,	/* Secure can't be used */
	.num_doorbells = 32,
	.support_doorbells = true,
};

static int mhu_probe(struct amba_device *adev, const struct amba_id *id)
{
	u32 cell_count;
	int i, err, max_chans;
	irq_handler_t handler;
	struct arm_mhu *mhu;
	struct mbox_chan *chans;
	struct mhu_mbox_pdata *pdata;
	struct device *dev = &adev->dev;
	void __iomem *tx_base;
	struct device_node *np = dev->of_node;
	struct mhu_data *mdata = id->data;

	if (!mdata) {
		dev_err(dev, "device data not found\n");
		return -EINVAL;
	}

	err = of_property_read_u32(np, "#mbox-cells", &cell_count);
	if (err) {
		dev_err(dev, "failed to read #mbox-cells in %s\n",
			np->full_name);
		return err;
	}

	if (cell_count == 1) {
		max_chans = MHU_NUM_PCHANS;
		pdata = (struct mhu_mbox_pdata *)&arm_mhu_pdata;
	} else if (cell_count == 2) {
		max_chans = MHU_CHAN_MAX;
		pdata = (struct mhu_mbox_pdata *)&arm_mhu_doorbell_pdata;
	} else {
		dev_err(dev, "incorrect value of #mbox-cells in %s\n",
			np->full_name);
		return -EINVAL;
	}

	if (pdata->num_pchans > MHU_NUM_PCHANS) {
		dev_err(dev, "Number of physical channel can't exceed %d\n",
			MHU_NUM_PCHANS);
		return -EINVAL;
	}

	mhu = devm_kzalloc(dev, sizeof(*mhu), GFP_KERNEL);
	if (!mhu)
		return -ENOMEM;

	mhu->base = devm_ioremap_resource(dev, &adev->res);
	if (IS_ERR(mhu->base)) {
		dev_err(dev, "ioremap failed\n");
		return PTR_ERR(mhu->base);
	}

	if (of_device_is_compatible(np, MHU_V2_COMPAT_STR)) {
		tx_base = of_iomap(np, 1);
		if (!tx_base) {
			dev_err(dev, "failed to map tx registers\n");
			return -ENOMEM;
		}
	}

	err = of_property_read_string(np, "mbox-name", &mhu->name);
	if (err)
		mhu->name = np->full_name;

	chans = devm_kcalloc(dev, max_chans, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	dev->platform_data = pdata;

	mhu->dev = dev;
	mhu->mbox.dev = dev;
	mhu->mbox.chans = chans;
	mhu->mbox.num_chans = max_chans;
	mhu->mbox.txdone_irq = false;
	mhu->mbox.txdone_poll = true;
	mhu->mbox.txpoll_period = 1;
	mhu->drvdata = mdata;
	if (of_device_is_compatible(np, MHU_V2_COMPAT_STR))
		mhu->drvdata->tx_reg_off = tx_base - mhu->base;

	mhu->mbox.of_xlate = mhu_mbox_xlate;
	amba_set_drvdata(adev, mhu);

	if (pdata->support_doorbells) {
		mhu->mbox.ops = &mhu_doorbell_ops;
		handler = mhu_mbox_thread_handler;
	} else {
		mhu->mbox.ops = &mhu_ops;
		handler = mhu_rx_interrupt;
	}

	err = devm_mbox_controller_register(dev, &mhu->mbox);
	if (err) {
		dev_err(dev, "Failed to register mailboxes %d\n", err);
		return err;
	}

	for (i = 0; i < pdata->num_pchans; i++) {
		int irq = mhu->mlink[i].irq = adev->irq[i];

		if (irq <= 0) {
			dev_dbg(dev, "No IRQ found for Channel %d\n", i);
			continue;
		}

		mhu->mlink[i].rx_reg = mhu->base + mhu->drvdata->chans[i];
		mhu->mlink[i].tx_reg = mhu->mlink[i].rx_reg
			+ mhu->drvdata->tx_reg_off;

		err = devm_request_threaded_irq(dev, irq, NULL, handler,
						IRQF_ONESHOT, "mhu_link", mhu);
		if (err) {
			dev_err(dev, "Can't claim IRQ %d\n", irq);
			mbox_controller_unregister(&mhu->mbox);
			return err;
		}
	}

	dev_info(dev, "%s mailbox registered\n", mhu->name);
	return 0;
}

static struct mhu_data arm_mhuv2_data = {
	.regs = { 0x0, 0xC, 0x8 }, /* STAT, SET, CLR */
	.chans = { 0x20, 0x0 }, /* LP, HP */
	.acc_regs = { 0xF88, 0xF8C }, /* ACCESS_REQUEST, ACCESS_READY */
};

static struct mhu_data arm_mhuv1_data = {
	.regs = { 0x0, 0x8, 0x10 }, /* STAT, SET, CLR */
	.chans = { 0x0, 0x20, 0x200 }, /* LP, HP, SEC */
	.tx_reg_off = 0x100,
};

static struct amba_id mhu_ids[] = {
	{
		.id     = 0x4b0d1,
		.mask   = 0xfffff,
		.data	= (void *)&arm_mhuv2_data,
	},
	{
		.id	= 0x1bb098,
		.mask	= 0xffffff,
		.data	= (void *)&arm_mhuv1_data,
	},
	{ 0, 0 },
};
MODULE_DEVICE_TABLE(amba, mhu_ids);

static struct amba_driver arm_mhu_driver = {
	.drv = {
		.name	= "mhu",
	},
	.id_table	= mhu_ids,
	.probe		= mhu_probe,
};
module_amba_driver(arm_mhu_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARM MHU Driver");
MODULE_AUTHOR("Jassi Brar <jassisinghbrar@gmail.com>");

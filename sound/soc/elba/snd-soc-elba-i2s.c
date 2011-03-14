#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <asm/io.h>
#include <mach/lt-elba.h>

/*
 * I2S Controller Register and Bit Definitions
 */
#define IER   0x000
#define IRER  0x004
#define ITER  0x008
#define CER   0x00C
#define CCR   0x010
#define RXFFR 0x014
#define TXFFR 0x018
#define LRBR(channel)    (0x020 + 0x040 * channel)
#define LTHR(channel)    (0x020 + 0x040 * channel)
#define RRBR(channel)    (0x024 + 0x040 * channel)
#define RTHR(channel)    (0x024 + 0x040 * channel)
#define RER(channel)     (0x028 + 0x040 * channel)
#define TER(channel)     (0x02C + 0x040 * channel)
#define RCR(channel)     (0x030 + 0x040 * channel)
#define TCR(channel)     (0x034 + 0x040 * channel)
#define ISR(channel)     (0x038 + 0x040 * channel)
#define IMR(channel)     (0x03C + 0x040 * channel)
#define ROR(channel)     (0x040 + 0x040 * channel)
#define TOR(channel)     (0x044 + 0x040 * channel)
#define RFCR(channel)    (0x048 + 0x040 * channel)
#define TFCR(channel)    (0x04C + 0x040 * channel)
#define RFF(channel)     (0x050 + 0x040 * channel)
#define TFF(channel)     (0x054 + 0x040 * channel)
#define RXDMA            0x1C0
#define RRXDMA           0x1C4
#define TXDMA            0x1C8
#define RTXDMA           0x1CC
#define I2S_COMP_PARAM_2 0x1F0
#define I2S_COMP_PARAM_1 0x1F4
#define I2S_COMP_VERSION 0x1F8
#define I2S_COMP_TYPE    0x1FC

/* ISR_X regs defines.  */
#define TXFE             (1 << 4)
#define TXFO             (1 << 5)

/* Elba I2S system clock.  */
#define ELBA_I2S_SYSCLK 0

/* Audio channels.  */
#define CHANNELS 4

static struct elba_i2s_port {
	unsigned int sadiv;
	unsigned int sacr0;
	unsigned int sacr1;
	unsigned int saimr;
	unsigned int master;
	unsigned int fmt;
} elba_i2s;

static struct elba_i2s_info {
	void __iomem *base;
	int irq;
} *info;
static struct clk *clk_i2s;
static int clk_enabled = 0;

static int elba_i2s_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	if (IS_ERR(clk_i2s))
		return PTR_ERR(clk_i2s);

	if (!cpu_dai->active) {
		/* Disable global bit.  */
		writel(0, info->base + IER);
		/* Disable receiver block enable.  */
		writel(0, info->base + IRER);
		/* Disable transmitter block enable.  */
		writel(0, info->base + ITER);
	}

	return 0;
}

static void elba_i2s_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	/* Disable global bit.  */
	writel(0, info->base + IER);
	/* Disable receiver block enable.  */
	writel(0, info->base + IRER);
	/* Disable transmitter block enable.  */
	writel(0, info->base + ITER);

	/* Disable clock.  */
	if (clk_enabled) {
		clk_disable(clk_i2s);
		clk_enabled = 0;
	}
}

static int elba_i2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

static int elba_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* Disable global bit.  */
		writel(0, info->base + IER);
		/* Disable receiver block enable.  */
		writel(0, info->base + IRER);
		/* Disable transmitter block enable.  */
		writel(0, info->base + ITER);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int elba_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	// struct snd_soc_pcm_runtime *rtd = substream->private_data;
	// struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	// struct pxa2xx_pcm_dma_params *dma_data;

	BUG_ON(IS_ERR(clk_i2s));
	clk_enable(clk_i2s);
	clk_enabled = 1;
	// pxa_i2s_wait();

/* 	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) */
/* 		dma_data = &pxa2xx_i2s_pcm_stereo_out; */
/* 	else */
/* 		dma_data = &pxa2xx_i2s_pcm_stereo_in; */

/* 	snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data); */

	/* is port used by another stream */
	if (!readl(info->base + IER)) {
		writel(0, info->base + IER);
	}

/* 		if (pxa_i2s.master) */
/* 			SACR0 |= SACR0_BCKD; */

/* 		SACR0 |= SACR0_RFTH(14) | SACR0_TFTH(1); */
/* 		SACR1 |= pxa_i2s.fmt; */
/* 	} */
/* 	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) */
/* 		SAIMR |= SAIMR_TFS; */
/* 	else */
/* 		SAIMR |= SAIMR_RFS; */

/* 	switch (params_rate(params)) { */
/* 	case 8000: */
/* 		SADIV = 0x48; */
/* 		break; */
/* 	case 11025: */
/* 		SADIV = 0x34; */
/* 		break; */
/* 	case 16000: */
/* 		SADIV = 0x24; */
/* 		break; */
/* 	case 22050: */
/* 		SADIV = 0x1a; */
/* 		break; */
/* 	case 44100: */
/* 		SADIV = 0xd; */
/* 		break; */
/* 	case 48000: */
/* 		SADIV = 0xc; */
/* 		break; */
/* 	case 96000: /\* not in manual and possibly slightly inaccurate *\/ */
/* 		SADIV = 0x6; */
/* 		break; */
/* 	} */

	return 0;
}

static int elba_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		elba_i2s.fmt = 0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		elba_i2s.fmt = 0; // SACR1_AMSL;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		elba_i2s.master = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		elba_i2s.master = 0;
		break;
	default:
		break;
	}

	return 0;
}

static int elba_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	if (clk_id != ELBA_I2S_SYSCLK)
		return -ENODEV;

	return 0;
}

static int elba_i2s_probe(struct snd_soc_dai *dai)
{
	clk_i2s = clk_get(dai->dev, "I2SCLK");
	if (IS_ERR(clk_i2s))
		return PTR_ERR(clk_i2s);

	return 0;
}

static int elba_i2s_remove(struct snd_soc_dai *dai)
{
	clk_put(clk_i2s);
	clk_i2s = ERR_PTR(-ENOENT);
	return 0;
}


#ifdef CONFIG_PM
static int elba_i2s_suspend(struct snd_soc_dai *dai)
{
	// save registers.

	writel(0, info->base + IER);
	// wait???

	return 0;
}

static int elba_i2s_resume(struct snd_soc_dai *dai)
{
	// wait???

	// restore registers

	return 0;
}
#else
#define elba_i2s_suspend	NULL
#define elba_i2s_resume		NULL
#endif

static irqreturn_t elba_i2s_interrupt(int irq, void *dev_id)
{
	unsigned long isr;
	int channel;

	for (channel = 0; channel < CHANNELS; channel++) {
		isr = ioread32(info->base + ISR(channel));

		switch (isr & (TXFE | TXFO)) {
		case TXFE:
			printk(KERN_ERR "TX Fifo is empty.\n");
			// fill_tx_fifo(channel);
			break;
		case TXFO:
			printk(KERN_ERR "TX Fifo is full.\n");
			// drain_tx_fifo(channel);
			break;
		default:
			/* The lion sleeps tonight.  */
			break;
		}
	}

	return IRQ_HANDLED;
}

#define ELBA_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

static struct snd_soc_dai_ops elba_i2s_dai_ops = {
	.startup	= elba_i2s_startup,
	.shutdown	= elba_i2s_shutdown,
	.prepare        = elba_i2s_prepare, // do we need this?!?
	.trigger	= elba_i2s_trigger,
	.hw_params	= elba_i2s_hw_params,
	.set_fmt	= elba_i2s_set_dai_fmt,
	.set_sysclk	= elba_i2s_set_dai_sysclk,
};

struct snd_soc_dai_driver elba_i2s_dai = {
	.probe = elba_i2s_probe,
	.remove = elba_i2s_remove,
	.suspend = elba_i2s_suspend,
	.resume  = elba_i2s_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = ELBA_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = ELBA_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &elba_i2s_dai_ops,
	.symmetric_rates = 1,
};

static __devinit int elba_i2s_drv_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	int ret;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "unable to allocate mem\n");
		ret = -ENOMEM;
		goto out;
	}

	if (!request_region(res->start, resource_size(res), "elba-i2s")) {
		printk(KERN_ERR "elba-i2s: address 0x%04x already in use\n", res->start);
		ret = -EBUSY;
		goto reqreg_err;
	}

	info->base = ioremap(res->start, resource_size(res));
	if (!info->base) {
		ret = -ENXIO;
		goto ioremap_err;
	}

	info->irq = IRQ_LT_ELBA_I2S;
	ret = request_irq(info->irq, elba_i2s_interrupt, IRQF_SHARED, dev_name(&pdev->dev), NULL);
	if (ret) {
		printk(KERN_ERR "elba-i2s: IRQ %d is not free.\n", info->irq);
		goto irq_err;
	}

	return snd_soc_register_dai(&pdev->dev, &elba_i2s_dai);

irq_err:
	iounmap(info->base);

ioremap_err:
	release_region(res->start, resource_size(res));

reqreg_err:
	kfree(info);
	info = NULL;

out:
	return ret;
}

static __devexit int elba_i2s_drv_remove(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;

	snd_soc_unregister_dai(&pdev->dev);

	iounmap(info->base);
	release_region(res->start, resource_size(res));
	kfree(info);

	return 0;
}

static struct platform_driver elba_i2s_driver = {
	.probe		= elba_i2s_drv_probe,
	.remove		= __devexit_p(elba_i2s_drv_remove),
	.driver		= {
		.name	= "elba-i2s",
		.owner	= THIS_MODULE,
	},
};

static int __init elba_i2s_init(void)
{
	clk_i2s = ERR_PTR(-ENOENT);
	return platform_driver_register(&elba_i2s_driver);
}
module_init(elba_i2s_init);

static void __exit elba_i2s_exit(void)
{
	platform_driver_unregister(&elba_i2s_driver);
}
module_exit(elba_i2s_exit);

MODULE_AUTHOR("Giuseppe Calderaro - <giuseppe.calderaro@arm.com>");
MODULE_DESCRIPTION("ELBA I2S SoC Interface");
MODULE_LICENSE("GPL");

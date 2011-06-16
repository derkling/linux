#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "elba_mtu.h"

#define DRIVER_NAME "elba_mtc"
#define PAGES_ORDER 11
#define MTU_MAJOR   0 /* Dynamic Major  */

static int ratio;
struct elba_mtc_info *mtc_info;
static int enable;

static irqreturn_t mtc_irq_lat_worker(int irq, void *dev_id)
{
	struct elba_mtc_info *mtc_info = (struct elba_mtc_info *)dev_id;
	unsigned long flags;
	unsigned long lat1_data, trans1_data, maxlat1_data;
	unsigned long lat2_data, trans2_data, maxlat2_data;
	unsigned long lat3_data, trans3_data, maxlat3_data;
	unsigned long lat4_data, trans4_data, maxlat4_data;
	unsigned long lat5_data, trans5_data, maxlat5_data;
	unsigned long lat6_data, trans6_data, maxlat6_data;
	unsigned long lat7_data, trans7_data, maxlat7_data;
	unsigned long lat8_data, trans8_data, maxlat8_data;
	unsigned long lat9_data, trans9_data, maxlat9_data;
	
	SAMPLES_LAT(mtc_info->samples);
	
	spin_lock_irqsave(&mtc_info->lock, flags);
	lat1_data = *latency1;
	trans1_data = *transactions1;
	maxlat1_data = *maxlat1;
	lat2_data = *latency2;
	trans2_data = *transactions2;
	maxlat2_data = *maxlat2;
	lat3_data = *latency3;
	trans3_data = *transactions3;
	maxlat3_data = *maxlat3;
	lat4_data = *latency4;
	trans4_data = *transactions4;
	maxlat4_data = *maxlat4;
	lat5_data = *latency5;
	trans5_data = *transactions5;
	maxlat5_data = *maxlat5;
	lat6_data = *latency6;
	trans6_data = *transactions6;
	maxlat6_data = *maxlat6;
	lat7_data = *latency7;
	trans7_data = *transactions7;
	maxlat7_data = *maxlat7;
	lat8_data = *latency8;
	trans8_data = *transactions8;
	maxlat8_data = *maxlat8;
	lat9_data = *latency9;
	trans9_data = *transactions9;
	maxlat9_data = *maxlat9;
	
	*latency1 = *transactions1 = *maxlat1 = 0;
	*latency2 = *transactions2 = *maxlat2 = 0;
	*latency3 = *transactions3 = *maxlat3 = 0;
	*latency4 = *transactions4 = *maxlat4 = 0;
	*latency5 = *transactions5 = *maxlat5 = 0;
	*latency6 = *transactions6 = *maxlat6 = 0;
	*latency7 = *transactions7 = *maxlat7 = 0;
	*latency8 = *transactions8 = *maxlat8 = 0;
	*latency9 = *transactions9 = *maxlat9 = 0;
	enable = 0;
	spin_unlock_irqrestore(&mtc_info->lock, flags);
	
	trace_printk("MTU DRAM 1: avg %u - max %u\n", *latency1 / *transactions1, *maxlat1);
	trace_printk("MTU DRAM 2: avg %u - max %u\n", *latency2 / *transactions2, *maxlat2);
	trace_printk("MTU L2CC 1: avg %u - max %u\n", *latency4 / *transactions4, *maxlat4);
	trace_printk("MTU L2CC 2: avg %u - max %u\n", *latency5 / *transactions5, *maxlat5);
	trace_printk("MTU HDLCD: avg %u - max %u\n", *latency6 / *transactions6, *maxlat6);
	trace_printk("MTU MALI400: avg %u - max %u\n", *latency7 / *transactions7, *maxlat7);
	trace_printk("MTU PCIE: avg %u - max %u\n", *latency9 / *transactions9, *maxlat9);
	
	return IRQ_HANDLED;
}

static irqreturn_t mtc_irq_lat(int irq, void *dev_id)
{
	struct elba_mtc_info *mtc_info = (struct elba_mtc_info *)dev_id;
	void __iomem *regs = mtc_info->regs;
	unsigned long val;
	static int enable;
	SAMPLES_LAT(mtc_info->samples);
	
	val = readl(regs + MTCCTRL);
	writel(val | 0x10, regs + MTCCTRL);

	spin_lock(&mtc_info->lock);
	*latency1 += readl(regs + MTU_OFF(DRAM1, MTLATS));
	*transactions1 += readl(regs + MTU_OFF(DRAM1, MTXACTC));
	val = readl(regs + MTU_OFF(DRAM1, MTLATM));
	if (*maxlat1 < val)
		*maxlat1 = val;

	*latency2 += readl(regs + MTU_OFF(DRAM2, MTLATS));
	*transactions2 += readl(regs + MTU_OFF(DRAM2, MTXACTC));
	val = readl(regs + MTU_OFF(DRAM2, MTLATM));
	if (*maxlat2 < val)
		*maxlat2 = val;

	*latency3 += readl(regs + MTU_OFF(DMA, MTLATS));
	*transactions3 += readl(regs + MTU_OFF(DMA, MTXACTC));
	val = readl(regs + MTU_OFF(DMA, MTLATM));
	if (*maxlat3 < val)
		*maxlat3 = val;

	*latency4 += readl(regs + MTU_OFF(L2CC1, MTLATS));
	*transactions4 += readl(regs + MTU_OFF(L2CC1, MTXACTC));
	val = readl(regs + MTU_OFF(L2CC1, MTLATM));	
	if (*maxlat4 < val)
		*maxlat4 = val;

	*latency5 += readl(regs + MTU_OFF(L2CC2, MTLATS));
	*transactions5 += readl(regs + MTU_OFF(L2CC2, MTXACTC));
	val = readl(regs + MTU_OFF(L2CC2, MTLATM));
	if (*maxlat5 < val)
		*maxlat5 = val;

	*latency6 += readl(regs + MTU_OFF(HDLCD, MTLATS));
	*transactions6 += readl(regs + MTU_OFF(HDLCD, MTXACTC));
	val = readl(regs + MTU_OFF(HDLCD, MTLATM));
	if (*maxlat6 < val)
		*maxlat6 = val;

	*latency7 += readl(regs + MTU_OFF(MALI400, MTLATS));
	*transactions7 += readl(regs + MTU_OFF(MALI400, MTXACTC));
	val = readl(regs + MTU_OFF(MALI400, MTLATM));
	if (*maxlat7 < val)
		*maxlat7 = val;

	*latency8 += readl(regs + MTU_OFF(MALIAVE, MTLATS));
	*transactions8 += readl(regs + MTU_OFF(MALIAVE, MTXACTC));
	val = readl(regs + MTU_OFF(MALIAVE, MTLATM));
	if (*maxlat8 < val)
		*maxlat8 = val;

	*latency9 += readl(regs + MTU_OFF(PCIE, MTLATS));
	*transactions9 += readl(regs + MTU_OFF(PCIE, MTXACTC));
	val = readl(regs + MTU_OFF(PCIE, MTLATM));
	if (*maxlat9 < val)
		*maxlat9 = val;

	/* Use ftrace to output data  */
	if (enable == ratio) {
		spin_unlock(&mtc_info->lock);
		return IRQ_WAKE_THREAD;
	} else
		enable++;
	spin_unlock(&mtc_info->lock);

	return IRQ_HANDLED;
}

static irqreturn_t mtc_irq_bw_worker(int irq, void *dev_id)
{
	struct elba_mtc_info *mtc_info = (struct elba_mtc_info *)dev_id;
	unsigned long flags;
	unsigned int dmc1_data;
	unsigned int dmc2_data;
	unsigned int l2cc1_data;
	unsigned int l2cc2_data;
	unsigned int hdlcd_data;
	unsigned int mali400_data;
	unsigned int pcie_data;

	SAMPLES_BW(mtc_info->samples);

	spin_lock_irqsave(&mtc_info->lock, flags);
	dmc1_data = *dmc1_bw;
	dmc2_data = *dmc2_bw;
	l2cc1_data = *l2cc1_bw;
	l2cc2_data = *l2cc2_bw;
	hdlcd_data = *hdlcd_bw;
	mali400_data = *mali400_bw;
	pcie_data = *pcie_bw;
	
	*dmc1_bw = 0; 
	*dmc2_bw = 0;
	*dma_bw = 0;
	*l2cc1_bw = 0;
	*l2cc2_bw = 0;
	*hdlcd_bw = 0;
	*mali400_bw = 0;
	*maliave_bw = 0; 
	*pcie_bw = 0;
	enable = 0;
	spin_unlock_irqrestore(&mtc_info->lock, flags);

	/* Use ftrace to output data  */
	trace_printk("Multi: %u %u %u %u %u %u %u\n",
		     dmc1_data,
		     dmc2_data,
		     l2cc1_data,
		     l2cc2_data,
		     hdlcd_data,
		     mali400_data,
		     pcie_data);
	
	return IRQ_HANDLED;
}

static irqreturn_t mtc_irq_bw(int irq, void *dev_id)
{
	struct elba_mtc_info *mtc_info = (struct elba_mtc_info *)dev_id;
	void __iomem *regs = mtc_info->regs;
	unsigned long val;
	SAMPLES_BW(mtc_info->samples);

	val = readl(regs + MTCCTRL);
	writel(val | 0x10, regs + MTCCTRL);

	spin_lock(&mtc_info->lock);
	*dmc1_bw += readl(regs + MTU_OFF(DRAM1, MTBWC));
	*dmc2_bw += readl(regs + MTU_OFF(DRAM2, MTBWC));
	*dma_bw += readl(regs + MTU_OFF(DMA, MTBWC));
	*l2cc1_bw += readl(regs + MTU_OFF(L2CC1, MTBWC));
	*l2cc2_bw += readl(regs + MTU_OFF(L2CC2, MTBWC));
	*hdlcd_bw += readl(regs + MTU_OFF(HDLCD, MTBWC));
	*mali400_bw += readl(regs + MTU_OFF(MALI400, MTBWC));
	*maliave_bw += readl(regs + MTU_OFF(MALIAVE, MTBWC));
	*pcie_bw += readl(regs + MTU_OFF(PCIE, MTBWC));
	if (enable == ratio) {
		spin_unlock(&mtc_info->lock);
		return IRQ_WAKE_THREAD;
	} else
		enable++;
	spin_unlock(&mtc_info->lock);
	
	return IRQ_HANDLED;
}

static void mtc_start(void)
{
	void __iomem *regs = mtc_info->regs;
	unsigned long val;

	val = readl(regs + MTCCTRL);
	writel(val | 0x6, regs + MTCCTRL);
}

static void mtc_stop(void)
{
	void __iomem *regs = mtc_info->regs;
	unsigned long val;	

	val = readl(regs + MTCCTRL);
	writel(val & ~0x6, regs + MTCCTRL);
}

static void mtc_set_ctrl(int mtu, int bit)
{
	void __iomem *regs = mtc_info->regs;

	writel(bit, regs + MTCTRL + mtu * 0x100);	
}

static void mtc_set_axiid(int axi_id, int mtu)
{
	void __iomem *regs = mtc_info->regs;

	if (mtu < 1 || mtu > 9)
		return;

	if (axi_id == -1)
		return;

	/* Set AXI ID to filter.  */
	writel(axi_id & 0xFF, regs + MTFILT + mtu * 0x100);
	writel(0xFF, regs + MTFILTM + mtu * 0x100);
}

#if defined (CONFIG_DEBUG_FS)
/* Root dentry.  */
static struct dentry *mtc_root;

/* MTU AXI ID select.  */
static struct dentry *mtc_axiid;
static long mtc_axiid_value;
static ssize_t mtc_axiid_read(struct file *file,
			       char __user *user_buf,
			       size_t len,
			       loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "AXI ID currently selected: %ld\n",
			  mtc_axiid_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);	
}

static ssize_t mtc_axiid_write(struct file *file,
			 const char __user *user_buf,
			 size_t len,
			 loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	mtc_axiid_value = simple_strtol(buffer, NULL, 10);

	mtc_set_axiid(mtc_axiid_value, DRAM1);
	mtc_set_axiid(mtc_axiid_value, DRAM2);
	mtc_set_axiid(mtc_axiid_value, DMA);
	mtc_set_axiid(mtc_axiid_value, L2CC1);
	mtc_set_axiid(mtc_axiid_value, L2CC2);
	mtc_set_axiid(mtc_axiid_value, HDLCD);
	mtc_set_axiid(mtc_axiid_value, MALI400);
	mtc_set_axiid(mtc_axiid_value, MALIAVE);
	mtc_set_axiid(mtc_axiid_value, PCIE);

	return len;	
}

static const struct file_operations mtc_axiid_fops = {
	.owner = THIS_MODULE,
	.read  = mtc_axiid_read,
	.write = mtc_axiid_write,
};

/* MTU enable/disable features.  */
static struct dentry *mtc_ctrl;
static long mtc_ctrl_value;
static ssize_t mtc_ctrl_read(struct file *file,
			       char __user *user_buf,
			       size_t len,
			       loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "ctrl reg: %lx\n",
			  mtc_ctrl_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);	
}

static ssize_t mtc_ctrl_write(struct file *file,
			 const char __user *user_buf,
			 size_t len,
			 loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	mtc_ctrl_value = simple_strtol(buffer, NULL, 10);

	switch (mtc_ctrl_value) {
	case 1: /* read bandwidth.  */
		mtc_set_ctrl(DRAM1, bwe);
		mtc_set_ctrl(DRAM2, bwe);
		mtc_set_ctrl(DMA, bwe);
		mtc_set_ctrl(L2CC1, bwe);
		mtc_set_ctrl(L2CC2, bwe);
		mtc_set_ctrl(HDLCD, bwe);
		mtc_set_ctrl(MALI400, bwe);
		mtc_set_ctrl(MALIAVE, bwe);
		mtc_set_ctrl(PCIE, bwe);
		trace_printk("MTUs: read bandwidth\n");
		break;
	case 2: /* write bandwidth.  */
		mtc_set_ctrl(DRAM1, bwe | we);
		mtc_set_ctrl(DRAM2, bwe | we);
		mtc_set_ctrl(DMA, bwe | we);
		mtc_set_ctrl(L2CC1, bwe | we);
		mtc_set_ctrl(L2CC2, bwe | we);
		mtc_set_ctrl(HDLCD, bwe | we);
		mtc_set_ctrl(MALI400, bwe | we);
		mtc_set_ctrl(MALIAVE, bwe | we);
		mtc_set_ctrl(PCIE, bwe | we);
		trace_printk("MTUs: write bandwidth\n");		
		break;
	case 3: /* avg and max read latency.  */
		mtc_set_ctrl(DRAM1, sme | mxe);
		mtc_set_ctrl(DRAM2, sme | mxe);
		mtc_set_ctrl(DMA, sme | mxe);
		mtc_set_ctrl(L2CC1, sme | mxe);
		mtc_set_ctrl(L2CC2, sme | mxe);
		mtc_set_ctrl(HDLCD, sme | mxe);
		mtc_set_ctrl(MALI400, sme | mxe);
		mtc_set_ctrl(MALIAVE, sme | mxe);
		mtc_set_ctrl(PCIE, sme | mxe);
		trace_printk("MTUs: read latency\n");		
		break;
	case 4: /* avg and max write latency.  */
		mtc_set_ctrl(DRAM1, sme | mxe | we);
		mtc_set_ctrl(DRAM2, sme | mxe | we);
		mtc_set_ctrl(DMA, sme | mxe | we);
		mtc_set_ctrl(L2CC1, sme | mxe | we);
		mtc_set_ctrl(L2CC2, sme | mxe | we);
		mtc_set_ctrl(HDLCD, sme | mxe | we);
		mtc_set_ctrl(MALI400, sme | mxe | we);
		mtc_set_ctrl(MALIAVE, sme | mxe | we);
		mtc_set_ctrl(PCIE, sme | mxe | we);
		trace_printk("MTUs: write latency\n");
	default:
		break;
	}

	return len;	
}

static const struct file_operations mtc_ctrl_fops = {
	.owner = THIS_MODULE,
	.read  = mtc_ctrl_read,
	.write = mtc_ctrl_write,
};


/* MTU enable.  */
static struct dentry *mtc_enable;
static long mtc_enable_value;
static ssize_t mtc_enable_read(struct file *file,
			       char __user *user_buf,
			       size_t len,
			       loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "MTUs are %s\n",
			  mtc_enable_value ? "enabled" : "disabled");

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);
} 

static ssize_t mtc_enable_write(struct file *file,
			 const char __user *user_buf,
			 size_t len,
			 loff_t *ppos)
{
	static int enabled;
	char buffer[20];
	irqreturn_t (*irq_handler)(int irq, void *dev_id);
	irqreturn_t (*irq_handler_worker)(int irq, void *dev_id);
	int err;

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	mtc_enable_value = simple_strtol(buffer, NULL, 10);

	switch (mtc_enable_value) {
	case 0:
		if (enabled) {
			free_irq(MTC_IRQ, mtc_info);
			/* Stop MTUs  */
			mtc_stop();
			enabled = 0;
		}
		break;
	case 1:
		if (!enabled) {
			switch (mtc_ctrl_value) {
			case 1:
			case 2:
				irq_handler = mtc_irq_bw;
				irq_handler_worker = mtc_irq_bw_worker;
				break;
			case 3:
			case 4:
				irq_handler = mtc_irq_lat;
				irq_handler_worker = mtc_irq_lat_worker;
				break;
			default:
				printk(KERN_ERR "%s: wrong mtc control value\n", DRIVER_NAME);
				return 0;
			}

			err = request_threaded_irq(MTC_IRQ, irq_handler,  irq_handler_worker, 0, DRIVER_NAME, mtc_info);
			if (err) {
				printk(KERN_ERR "%s: cannot assign irq %d\n", DRIVER_NAME, MTC_IRQ);
				return 0;
			} else {
				/* Start MTUs  */
				mtc_start();
				enabled = 1;
			}
		}
		break;
	default:
		printk(KERN_ERR "Accepted values: 0 (OFF) - 1 (ON)\n");
		break;
	}

	return len;
}

static const struct file_operations mtc_enable_fops = {
	.owner   = THIS_MODULE,
	.read    = mtc_enable_read,
	.write   = mtc_enable_write,
};

static void mtc_create_debugfs(void)
{
	mtc_root = debugfs_create_dir("mtc", NULL);

	mtc_enable = debugfs_create_file("enable",
					 S_IWUGO,
					 mtc_root,
					 &mtc_enable_value,
					 &mtc_enable_fops);

	mtc_axiid = debugfs_create_file("axiid",
					S_IWUGO,
					mtc_root,
					&mtc_axiid_value,
					&mtc_axiid_fops);

	mtc_ctrl = debugfs_create_file("ctrl",
				       S_IWUGO,
				       mtc_root,
				       &mtc_ctrl_value,
				       &mtc_ctrl_fops);
}

static void mtc_remove_debugfs(void)
{
	if (mtc_root) {
		debugfs_remove(mtc_ctrl);
		debugfs_remove(mtc_axiid);
		debugfs_remove(mtc_enable);
		debugfs_remove(mtc_root);		
		mtc_root = NULL;
	}
}
#else
static void mtc_create_debugfs(void) {}
static void mtc_remove_debugfs(void) {}
#endif

static int mtc_setup(void)
{
	void __iomem *regs = mtc_info->regs;

	mtc_info->samples = kzalloc(sizeof(*mtc_info->samples), GFP_KERNEL);
	if (!mtc_info->samples)
		return -ENOMEM;

	/* Configure MTC  */
	writel(MTC_SAMPLE_WINDOW, regs + MTCWIND);
	writel(0x00007FE0, regs + MTCCTRL);
	/* Enable ATB data upload.  */
	writel(0x00000084, regs + MTTCFG);

	/* Setup MTUs (default setup)  */
	writel(0, regs + MTU_OFF(DRAM1, DRAM1));
	writel(0, regs + MTU_OFF(DRAM1, DRAM2));
	writel(0, regs + MTU_OFF(DRAM1, DMA));
	writel(0, regs + MTU_OFF(DRAM1, L2CC1));
	writel(0, regs + MTU_OFF(DRAM1, L2CC2));
	writel(0, regs + MTU_OFF(DRAM1, HDLCD));
	writel(0, regs + MTU_OFF(DRAM1, MALI400));
	writel(0, regs + MTU_OFF(DRAM1, MALIAVE));
	writel(0, regs + MTU_OFF(DRAM1, PCIE));

	return 0;
}

static int __init elba_mtc_init(void)
{
	void __iomem *regs;
	int err = 0;

	mtc_info = kzalloc(sizeof(*mtc_info), GFP_KERNEL);
	if (!mtc_info)
		return -ENOMEM;

	regs = ioremap(MTC_BASE, SZ_4K);
	if (!regs) {
		printk(KERN_ERR "%s: cannot map MTC\n", DRIVER_NAME);
		goto release_mem;
	}
	mtc_info->regs = regs;
	
	/* Init spinlock.  */
	spin_lock_init(&mtc_info->lock);

	err = mtc_setup();
	if (err) {
		printk(KERN_ERR "%s: cannot alloc samples\n", DRIVER_NAME);
		goto release_map;		
	}

	mtc_create_debugfs();       
out:
	return err;

release_map:
	iounmap(mtc_info->regs);
release_mem:
	kfree(mtc_info);
	goto out;
}

static void __exit elba_mtc_exit(void)
{
	mtc_stop();
	mtc_remove_debugfs();
	free_irq(MTC_IRQ, mtc_info);
	iounmap(mtc_info->regs);
	kfree(mtc_info);
}

module_init(elba_mtc_init);
module_exit(elba_mtc_exit);

/* Module cmdline params  */
module_param(ratio, int, S_IRUGO);
MODULE_PARM_DESC(ratio, "Interrupts tracing ratio");

MODULE_AUTHOR("Giuseppe Calderaro");
MODULE_DESCRIPTION("Elba MTCs driver");
MODULE_LICENSE("GPL");

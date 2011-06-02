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

static int elba_mtc;
static int elba_mali_bypass;
struct elba_mtc_info *mtc_info;

static irqreturn_t mtc_irq_lat(int irq, void *dev_id)
{
	struct elba_mtc_info *mtc_info = (struct elba_mtc_info *)dev_id;
	void __iomem *regs = mtc_info->regs;
	unsigned long val;
	unsigned int latencysum = 0;
	unsigned int transactions = 0;
	SAMPLES_LAT(mtc_info->samples);
	
	val = readl(regs + MTCCTRL);
	writel(val | 0x10, regs + MTCCTRL);

	latencysum = readl(regs + MTU_OFF(DRAM1, MTLATS));
	transactions = readl(regs + MTU_OFF(DRAM1, MTXACTC));
	if (transactions)
		*latency1 = (latencysum / transactions);
	else
		*latency1 = 0;
	*maxlat1 = readl(regs + MTU_OFF(DRAM1, MTLATM));

	latencysum = readl(regs + MTU_OFF(DRAM2, MTLATS));
	transactions = readl(regs + MTU_OFF(DRAM2, MTXACTC));
	if (transactions)
		*latency2 = (latencysum / transactions);
	else
		*latency2 = 0;
	*maxlat2 = readl(regs + MTU_OFF(DRAM2, MTLATM));

	latencysum = readl(regs + MTU_OFF(DMA, MTLATS));
	transactions = readl(regs + MTU_OFF(DMA, MTXACTC));
	if (transactions)
		*latency3 = (latencysum / transactions);
	else
		*latency3 = 0;		
	*maxlat3 = readl(regs + MTU_OFF(DMA, MTLATM));

	latencysum = readl(regs + MTU_OFF(L2CC1, MTLATS));
	transactions = readl(regs + MTU_OFF(L2CC1, MTXACTC));
	if (transactions)
		*latency4 = (latencysum / transactions);
	else
		*latency4 = 0;
	*maxlat4 = readl(regs + MTU_OFF(L2CC1, MTLATM));

	latencysum = readl(regs + MTU_OFF(L2CC2, MTLATS));
	transactions = readl(regs + MTU_OFF(L2CC2, MTXACTC));
	if (transactions)
		*latency5 = (latencysum / transactions);
	else
		*latency5 = 0;
	*maxlat5 = readl(regs + MTU_OFF(L2CC2, MTLATM));

	latencysum = readl(regs + MTU_OFF(HDLCD, MTLATS));
	transactions = readl(regs + MTU_OFF(HDLCD, MTXACTC));
	if (transactions)
		*latency6 = (latencysum / transactions);
	else
		*latency6 = 0;
	*maxlat6 = readl(regs + MTU_OFF(HDLCD, MTLATM));

	latencysum = readl(regs + MTU_OFF(MALI400, MTLATS));
	transactions = readl(regs + MTU_OFF(MALI400, MTXACTC));
	if (transactions)
		*latency7 = (latencysum / transactions);
	else
		*latency7 = 0;
	*maxlat7 = readl(regs + MTU_OFF(MALI400, MTLATM));

	latencysum = readl(regs + MTU_OFF(MALIAVE, MTLATS));
	transactions = readl(regs + MTU_OFF(MALIAVE, MTXACTC));
	if (transactions)
		*latency8 = (latencysum / transactions);
	else
		*latency8 = 0;
	*maxlat8 = readl(regs + MTU_OFF(MALIAVE, MTLATM));

	latencysum = readl(regs + MTU_OFF(PCIE, MTLATS));
	transactions = readl(regs + MTU_OFF(PCIE, MTXACTC));
	if (transactions)
		*latency9 = (latencysum / transactions);
	else
		*latency9 = 0;	       
	*maxlat9 = readl(regs + MTU_OFF(PCIE, MTLATM));

	/* Use ftrace to output data  */
	trace_printk("MTU DRAM 1: avg %d - max %d\n", *latency1, *maxlat1);
	trace_printk("MTU DRAM 2: avg %d - max %d\n", *latency2, *maxlat2);
	trace_printk("MTU L2CC 1: avg %d - max %d\n", *latency4, *maxlat4);
	trace_printk("MTU L2CC 2: avg %d - max %d\n", *latency5, *maxlat5);
	trace_printk("MTU HDLCD: avg %d - max %d\n", *latency6, *maxlat6);
	trace_printk("MTU MALI400: avg %d - max %d\n", *latency7, *maxlat7);
	trace_printk("MTU PCIE: avg %d - max %d\n", *latency9, *maxlat9);

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

	*dmc1_bw = readl(regs + MTU_OFF(DRAM1, MTBWC));
	*dmc2_bw = readl(regs + MTU_OFF(DRAM2, MTBWC));
	*dma_bw = readl(regs + MTU_OFF(DMA, MTBWC));
	*l2cc1_bw = readl(regs + MTU_OFF(L2CC1, MTBWC));
	*l2cc2_bw = readl(regs + MTU_OFF(L2CC2, MTBWC));
	*hdlcd_bw = readl(regs + MTU_OFF(HDLCD, MTBWC));
	*mali400_bw = readl(regs + MTU_OFF(MALI400, MTBWC));
	*maliave_bw = readl(regs + MTU_OFF(MALIAVE, MTBWC));
	*pcie_bw = readl(regs + MTU_OFF(PCIE, MTBWC));
	
	/* Use ftrace to output data  */
	trace_printk("MTU DRAM 1: %d\n", *dmc1_bw);
	trace_printk("MTU DRAM 2: %d\n", *dmc2_bw);
	trace_printk("MTU L2CC 1: %d\n", *l2cc1_bw);
	trace_printk("MTU L2CC 2: %d\n", *l2cc2_bw);
	trace_printk("MTU HDLCD: %d\n", *hdlcd_bw);
	trace_printk("MTU MALI400: %d\n", *mali400_bw);
	trace_printk("MTU PCIE: %d\n", *pcie_bw);

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
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	mtc_enable_value = simple_strtol(buffer, NULL, 10);

	switch (mtc_enable_value) {
	case 0:
		mtc_stop();
		break;
	case 1:
		mtc_start();
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
	case 4: /* avg and max read latency.  */
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
	if(elba_mali_bypass)
		writel(0x00004FE0, regs + MTCCTRL);
	else
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

static irqreturn_t mtc_irq(int irq, void *dev_id)
{
	irqreturn_t ret;

	switch (mtc_ctrl_value) {
	case 1:
	case 2:
		ret = mtc_irq_bw(irq, dev_id);
		break;
	case 3:
	case 4:
		ret = mtc_irq_lat(irq, dev_id);
		break;
	default:
		break;
	}

	return ret;
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

	err = request_irq(MTC_IRQ, mtc_irq,  0, DRIVER_NAME, mtc_info);
	if (err) {
		printk(KERN_ERR "%s: cannot assign irq %d\n", DRIVER_NAME, MTC_IRQ);
		goto release_map;
	}

	err = mtc_setup();
	if (err) {
		printk(KERN_ERR "%s: cannot alloc samples\n", DRIVER_NAME);
		goto release_irq;		
	}

	mtc_create_debugfs();       
out:
	return err;

release_irq:
	free_irq(MTC_IRQ, mtc_info);
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
module_param_named(mtc, elba_mtc, int, S_IRUGO);
MODULE_PARM_DESC(mtc, "Select MTC interface to monitor");

module_param_named(mali_bypass, elba_mali_bypass, int, S_IRUGO);
MODULE_PARM_DESC(mali_bypass, "Select to bypass Mali400 and MaliAVE MTUs");

MODULE_AUTHOR("Giuseppe Calderaro");
MODULE_DESCRIPTION("Elba MTCs driver");
MODULE_LICENSE("GPL");

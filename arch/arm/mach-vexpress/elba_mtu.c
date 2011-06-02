#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/elba_mtu.h>

#define DRIVER_NAME "elba_mtc"
#define PAGES_ORDER 11
#define MTU_MAJOR   0 /* Dynamic Major  */

static int elba_mtc;
static int elba_mali_bypass;
static int elba_axi_id;
static int elba_enable_printk;
static struct samples *samples;
static unsigned int sample_count;
struct elba_mtc_info *mtc_info;
EXPORT_SYMBOL(mtc_info);

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

static void start_axi_id_scan(void)
{
	void __iomem *regs = mtc_info->regs;

	for (elba_axi_id = 0; elba_axi_id < 16; elba_axi_id++) {
		mtc_stop();
		/* Setup AXI-ID for PCIe MTU  */
		writel(elba_axi_id & 0xFF, regs + MTFILT9);
		writel(0xFF, regs + MTFILTM9);
		elba_enable_printk = 1;
		mtc_start();
		udelay(1000);
	}
}

#if defined (CONFIG_DEBUG_FS)
/* Root dentry.  */
static struct dentry *mtc_root;
/* MTU enable.  */
static struct dentry *mtc_enable;
static long mtc_enable_value;
/* PCIe MTU  */
static struct dentry *mtus;
static long mtus_value;

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
	case 2:
		start_axi_id_scan();
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

static ssize_t mtus_debugfs_read(struct file *file,
			       char __user *user_buf,
			       size_t len, 
			       loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;
	SAMPLES(samples);

	count += snprintf(buffer,
			  sizeof(buffer),
			  "Samples: %p\n"
			  "Sample Count: %d\n"
#if 0
			  "DDR Controller 1 samples:\n"
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d\n"
			  
			  "DDR Controller 2 samples:\n"
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d\n"
#endif
			  "PCIe samples:\n"
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d "
			  "%d %d %d %d %d %d %d %d\n",
			  samples,
			  sample_count,
#if 0
			  dmc1_bw[0], dmc1_bw[1], dmc1_bw[2], dmc1_bw[3],
			  dmc1_bw[4], dmc1_bw[5], dmc1_bw[6], dmc1_bw[7],
			  dmc1_bw[8], dmc1_bw[9], dmc1_bw[10], dmc1_bw[11],
			  dmc1_bw[12], dmc1_bw[13], dmc1_bw[14], dmc1_bw[15],
			  dmc1_bw[16], dmc1_bw[17], dmc1_bw[18], dmc1_bw[19],
			  dmc1_bw[20], dmc1_bw[21], dmc1_bw[22], dmc1_bw[23],
			  dmc1_bw[24], dmc1_bw[25], dmc1_bw[26], dmc1_bw[27],
			  dmc1_bw[28], dmc1_bw[29], dmc1_bw[30], dmc1_bw[31],

			  dmc2_bw[0], dmc2_bw[1], dmc2_bw[2], dmc2_bw[3],
			  dmc2_bw[4], dmc2_bw[5], dmc2_bw[6], dmc2_bw[7],
			  dmc2_bw[8], dmc2_bw[9], dmc2_bw[10], dmc2_bw[11],
			  dmc2_bw[12], dmc2_bw[13], dmc2_bw[14], dmc2_bw[15],
			  dmc2_bw[16], dmc2_bw[17], dmc2_bw[18], dmc2_bw[19],
			  dmc2_bw[20], dmc2_bw[21], dmc2_bw[22], dmc2_bw[23],
			  dmc2_bw[24], dmc2_bw[25], dmc2_bw[26], dmc2_bw[27],
			  dmc2_bw[28], dmc2_bw[29], dmc2_bw[30], dmc2_bw[31],
#endif		  
			  pcie_bw[0], pcie_bw[1], pcie_bw[2], pcie_bw[3],
			  pcie_bw[4], pcie_bw[5], pcie_bw[6], pcie_bw[7],
			  pcie_bw[8], pcie_bw[9], pcie_bw[10], pcie_bw[11],
			  pcie_bw[12], pcie_bw[13], pcie_bw[14], pcie_bw[15],
			  pcie_bw[16], pcie_bw[17], pcie_bw[18], pcie_bw[19],
			  pcie_bw[20], pcie_bw[21], pcie_bw[22], pcie_bw[23],
			  pcie_bw[24], pcie_bw[25], pcie_bw[26], pcie_bw[27],
			  pcie_bw[28], pcie_bw[29], pcie_bw[30], pcie_bw[31]
		);
	
	return simple_read_from_buffer(user_buf,
				       len,
				       ppos,
				       buffer,
				       count);

/* 	ssize_t read = copy_to_user(user_buf, (const void *)mtc_info->mem, (4096 << PAGES_ORDER)); */

/* 	return read; */
}

static const struct file_operations mtus_debugfs_fops = {
	.owner           = THIS_MODULE,
	.read            = mtus_debugfs_read,
};

static ssize_t mtus_read(struct file *file,
			       char __user *user_buf,
			       size_t len, 
			       loff_t *ppos)
{
	ssize_t read = copy_to_user(user_buf, (const void *)mtc_info->mem, (4096 << PAGES_ORDER));
	
	return read;
}

static const struct file_operations mtus_fops = {
	.owner           = THIS_MODULE,
	.read            = mtus_read,
};

static void mtc_create_debugfs(void)
{
	mtc_root = debugfs_create_dir("mtc", NULL);

	mtc_enable = debugfs_create_file("enable",
					 S_IWUGO,
					 mtc_root,
					 &mtc_enable_value,
					 &mtc_enable_fops);
	
	mtus = debugfs_create_file("mtus",
				   S_IRUGO,
				   mtc_root,
				   &mtus_value,
				   &mtus_debugfs_fops);
}

static void mtc_remove_debugfs(void)
{
	if (mtc_root) {
		debugfs_remove(mtus);
		debugfs_remove(mtc_enable);
		debugfs_remove(mtc_root);		
		mtc_root = NULL;
	}
}
#else
static void mtc_create_debugfs(void) {}
static void mtc_remove_debugfs(void) {}
#endif

static irqreturn_t mtc_irq(int irq, void *dev_id)
{
	struct elba_mtc_info *mtc_info = (struct elba_mtc_info *)dev_id;
	void __iomem *regs = mtc_info->regs;
	unsigned int val;
	unsigned int latencysum = 0;
	unsigned int transactions = 0;
	int count = sample_count;
	SAMPLES(samples);

	val = readl(regs + MTCCTRL);
	writel(val | 0x10, regs + MTCCTRL);

	if (samples < mtc_info->memlimit) {
		mtc_info->current_sample = samples;
		if (count < MTC_SAMPLES) {		
			/* MTU 1  */
			dmc1_bw[count]  = readl(regs + MTBWC1);
			latencysum = readl(regs + MTLATS1);
			transactions = readl(regs + MTXACTC1);
			if (transactions)
				latency1[count] = (latencysum / transactions);
			else
				latency1[count] = 0;
			/* MTU 2  */
			dmc2_bw[count]  = readl(regs + MTBWC2);
			latencysum = readl(regs + MTLATS2);
			transactions = readl(regs + MTXACTC2);
			if (transactions)
				latency2[count] = (latencysum / transactions);
			else
				latency2[count] = 0;
			/* MTU 3  */
			dma_bw[count]  = readl(regs + MTBWC3);
			latencysum = readl(regs + MTLATS3);
			transactions = readl(regs + MTXACTC3);
			if (transactions)
				latency3[count] = (latencysum / transactions);
			else
				latency3[count] = 0;		
			/* MTU 4  */
			l2cc1_bw[count]  = readl(regs + MTBWC4);
			latencysum = readl(regs + MTLATS4);
			transactions = readl(regs + MTXACTC4);
			if (transactions)
				latency4[count] = (latencysum / transactions);
			else
				latency4[count] = 0;
			/* MTU 5  */
			l2cc2_bw[count]  = readl(regs + MTBWC5);
			latencysum = readl(regs + MTLATS5);
			transactions = readl(regs + MTXACTC5);
			if (transactions)
				latency5[count] = (latencysum / transactions);
			else
				latency5[count] = 0;
			/* MTU 6  */
			hdlcd_bw[count]  = readl(regs + MTBWC6);
			latencysum = readl(regs + MTLATS6);
			transactions = readl(regs + MTXACTC6);
			if (transactions)
				latency6[count] = (latencysum / transactions);
			else
				latency6[count] = 0;
			/* MTU 7  */
			mali400_bw[count]  = readl(regs + MTBWC7);
			latencysum = readl(regs + MTLATS7);
			transactions = readl(regs + MTXACTC7);
			if (transactions)
				latency7[count] = (latencysum / transactions);
			else
				latency7[count] = 0;
			/* MTU 8  */
			maliave_bw[count]  = readl(regs + MTBWC8);
			latencysum = readl(regs + MTLATS8);
			transactions = readl(regs + MTXACTC8);
			if (transactions)
				latency8[count] = (latencysum / transactions);
			else
				latency8[count] = 0;
			/* MTU 9  */
			pcie_bw[count]  = readl(regs + MTBWC9);

			if (elba_enable_printk) {
				if(pcie_bw[count])
					printk(KERN_CRIT "--- There we go: %d - id %d\n", pcie_bw[count], elba_axi_id);
				elba_enable_printk = 0;
			}

			latencysum = readl(regs + MTLATS9);
			transactions = readl(regs + MTXACTC9);
			if (transactions)
				latency9[count] = (latencysum / transactions);
			else
				latency9[count] = 0;	       

			if (count == MTC_SAMPLES - 1) {
				samples++;
				sample_count = 0;				
			} else {
				sample_count = count + 1;
			}
		}
	} else {
		samples = (struct samples *)mtc_info->mem;
	}	
	return IRQ_HANDLED;
}

static int mtc_setup(void)
{
	void __iomem *regs = mtc_info->regs;
	unsigned long mem = __get_free_pages(GFP_KERNEL, PAGES_ORDER);
	if (!mem)
		return -ENOMEM;
	mtc_info->mem = mem;
	mtc_info->memsize = (4096 << PAGES_ORDER);
	mtc_info->memlimit = (struct samples *)(mtc_info->mem + mtc_info->memsize);
	mtc_info->samples = (struct samples *)mem;
	samples = mtc_info->samples;
	sample_count = 0;
	mtc_info->blocks = mtc_info->memsize / sizeof(struct samples);
	memset((void *)mem, 0, mtc_info->memsize);
	printk(KERN_INFO "%s: Got %dMb of memory at 0x%lx\n"
	       "%d blocks, %d bytes each.\n",
	       __func__,
	       mtc_info->memsize >> 20,
	       mtc_info->mem,
	       mtc_info->blocks,
	       sizeof(struct samples));
	
	/* Configure MTC  */
	writel(MTC_SAMPLE_WINDOW, regs + MTCWIND);
	if(elba_mali_bypass)
		writel(0x00004FE0, regs + MTCCTRL);
	else
		writel(0x00007FE0, regs + MTCCTRL);
	/* Enable ATB data upload.  */
	writel(0x00000084, regs + MTTCFG);

	/* Setup MTUs  */
	writel(0x00000100, regs + MTCTRL1);
	writel(0x00000100, regs + MTCTRL2);
	writel(0x00000100, regs + MTCTRL3);
	writel(0x00000100, regs + MTCTRL4);
	writel(0x00000100, regs + MTCTRL5);
	writel(0x00000100, regs + MTCTRL6);
	writel(0x00000100, regs + MTCTRL7);
	writel(0x00000100, regs + MTCTRL8);
	writel(0x00000101, regs + MTCTRL9);

	return 0;
}

static void mtc_dealloc(void)
{
	free_pages(mtc_info->mem, PAGES_ORDER);
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

	err = request_irq(MTC_IRQ, mtc_irq, 0, DRIVER_NAME, mtc_info);
	if (err) {
		printk(KERN_ERR "%s: cannot assign irq %d\n", DRIVER_NAME, MTC_IRQ);
		goto release_map;
	}

#ifdef ENABLE_CHAR
	mtc_info->major = register_chrdev(MTU_MAJOR, DRIVER_NAME, &mtus_fops);
	if (err) {
		printk("unable to get major for mtu devs\n");
		goto release_irq;
	}
#endif 

	err = mtc_setup();
	if (err)
		goto release_char;

	mtc_create_debugfs();
	
	mtc_start();

out:
	return err;
release_char:
#ifdef ENABLE_CHAR
	unregister_chrdev(mtc_info->major, DRIVER_NAME);
release_irq:
#endif
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
	mtc_dealloc();
#ifdef ENABLE_CHAR
	unregister_chrdev(mtc_info->major, DRIVER_NAME);
#endif
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
module_param_named(axi_id, elba_axi_id, int, S_IRUGO);
MODULE_PARM_DESC(mtc, "Select AXI ID to filter");
module_param_named(enable_printk, elba_enable_printk, int, S_IRUGO);
MODULE_PARM_DESC(enable_printk, "enable irqs prints");

MODULE_AUTHOR("Giuseppe Calderaro");
MODULE_DESCRIPTION("Elba MTCs driver");
MODULE_LICENSE("GPL");

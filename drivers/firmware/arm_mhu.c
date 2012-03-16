#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/io.h>
#include "arm_mhu.h"

static struct arm_mhu_data *gdata;
static struct list_head lo_head;
static struct mutex lo_mutex;
static struct list_head hi_head;
static struct mutex hi_mutex;

static inline void init_req(struct arm_mhu_request *req, u32 cmd)
{
	req->cmd = cmd;
	init_completion(&req->sync);
}

static inline void append_req(struct arm_mhu_request *req, struct mutex *mutex, struct list_head *head)
{
	mutex_lock(mutex);
	list_add_tail(&req->list, head);
	mutex_unlock(mutex);
}

static inline void wait_for_mhu(void)
{
	volatile u32 status;

	do {
		status = mhu_reg_readl(gdata, SCP_INTR_L_STAT);
		cpu_relax();
	} while (status);	          
}

int get_dvfs_size(int cluster, int cpu, u32 *size)
{
	struct arm_mhu_request *req;

	if (!size)
		return -EFAULT;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	init_req(req, GET_CAPABILITIES);
	append_req(req, &lo_mutex, &lo_head);
	
	/* Check if SPC can deal with us.  */
	wait_for_mhu();

	/* Fill payload.  */
	mhu_mem_writel(gdata, SCP_LOW, 1);
	/* Write set register.  */
	mhu_reg_writel(gdata, SCP_INTR_L_SET, 
		       (1 << 20) | 
		       (0 /* flags  */ << 16) | 
		       (cluster << 12) | 
		       (cpu << 8) |
		       GET_CAPABILITIES);

	/* Wait for response.  */
	wait_for_completion(&req->sync);

	*size = req->payload_size;

	kfree(req);

	return 0;
}

int get_dvfs_capabilities(int cluster, int cpu, u32 *freqs, u32 size)
{
	struct arm_mhu_request *req;
	int i;

	if (!freqs)
		return -EFAULT;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	init_req(req, GET_CAPABILITIES);
	append_req(req, &lo_mutex, &lo_head);
	
	/* Check if SPC can deal with us.  */
	wait_for_mhu();

	/* Fill payload.  */
	mhu_mem_writel(gdata, SCP_LOW, 1);
	/* Write set register.  */
	mhu_reg_writel(gdata, SCP_INTR_L_SET, 
		       (1 << 20) | 
		       (0 /* flags  */ << 16) | 
		       (cluster << 12) | 
		       (cpu << 8) |
		       GET_CAPABILITIES);

	/* Wait for response.  */
	wait_for_completion(&req->sync);

	for (i = 0; i < size; i++) {
		freqs[i] = req->payload[i];
	}

	kfree(req->payload);
	kfree(req);

	return 0;
}

int get_performance(int cluster, int cpu, u32 *perf)
{
	struct arm_mhu_request *req;

	if (!perf)
		return -EFAULT;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	init_req(req, GET_PERFORMANCE);
	append_req(req, &lo_mutex, &lo_head);
	      
	/* Check if SPC can deal with us.  */
	wait_for_mhu();

	/* Fill payload.  */
	mhu_mem_writel(gdata, SCP_LOW, cluster & 0xFF);
	wmb();
	mhu_reg_writel(gdata, SCP_INTR_L_SET, (1 << 20) | 
		       (0 /* flags  */ << 16) | 
		       (cluster << 12) | 
		       (cpu << 8) | 
		       GET_PERFORMANCE);

	/* Wait for response.  */
	wait_for_completion(&req->sync);

	*perf = req->payload[0];
	if (*perf & 0xFF) {
		printk(KERN_ERR "MHU Cmd failed\n");
		return -EFAULT;
	}
	*perf = (*perf >> 16) & 0xFF;

	kfree(req->payload);
	kfree(req);

	return 0;
}

int set_performance(int cluster, int cpu, u32 index)
{
	struct arm_mhu_request *req;
	int ret;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	init_req(req, SET_PERFORMANCE);
	append_req(req, &hi_mutex, &hi_head);
	      
	/* Check if SPC can deal with us.  */
	wait_for_mhu();

	/* Fill payload.  */
	mhu_mem_writel(gdata, SCP_HIGH, (cluster & 0xFF) | ((index << 8) && 0xFF00));
	wmb();
	mhu_reg_writel(gdata, SCP_INTR_H_SET, (2 << 20) | 
		       (0 /* flags  */ << 16) | 
		       (cluster << 12) | 
		       (cpu << 8) | 
		       SET_PERFORMANCE);

	/* Wait for response.  */
	wait_for_completion(&req->sync);

	if (req->payload[0] && 0xFF)
		ret = req->payload[0] & 0xFF;
	else
		ret = 0;

	kfree(req->payload);
	kfree(req);	

	return ret;
}

static irqreturn_t arm_mhu_hi_irq_handler(int irq, void *dev_id)
{
	struct arm_mhu_data *data = dev_id;
	u32 status = mhu_reg_readl(dev_id, SCP_INTR_H_STAT);	
	u32 cmd = status & 0xFF;
	struct list_head *pos;
	struct arm_mhu_request *req = 0;
	int i;

	mutex_lock(&hi_mutex);
	list_for_each(pos, &hi_head) {
		req = list_entry(pos, struct arm_mhu_request, list);
		if (req->cmd == cmd) {
			list_del(&req->list);
			break;
		}
	}
	mutex_unlock(&hi_mutex);	

	req->payload_size = (status & 0x7ff00000) >> 20;
	req->payload = kzalloc(sizeof(u8) * req->payload_size, GFP_KERNEL);
	if (!req->payload) {
		printk(KERN_ERR "MHU: can't allocate memory for payload.\n");
		goto out;
	}

	for (i = 0; i < req->payload_size; i++)
		req->payload[i] = mhu_mem_readl(gdata, SCP_LOW + 4 * i);

	/* Clear SPC interrupt.  */
	mhu_reg_writel(data, SCP_INTR_H_CLEAR, status);

	complete(&req->sync);

out:
	return IRQ_HANDLED;
}

static irqreturn_t arm_mhu_lo_irq_handler(int irq, void *dev_id)
{
	struct arm_mhu_data *data = dev_id;
	u32 status = mhu_reg_readl(dev_id, SCP_INTR_L_STAT);	
	u32 cmd = status & 0xFF;
	struct list_head *pos;
	struct arm_mhu_request *req = 0;
	int i;

	mutex_lock(&lo_mutex);
	list_for_each(pos, &lo_head) {
		req = list_entry(pos, struct arm_mhu_request, list);
		if (req->cmd == cmd) {
			list_del(&req->list);
			break;
		}
	}
	mutex_unlock(&lo_mutex);	

	req->payload_size = (status & 0x7ff00000) >> 20;
	req->payload = kzalloc(sizeof(u32) * req->payload_size, GFP_KERNEL);
	if (!req->payload) {
		printk(KERN_ERR "MHU: can't allocate memory for payload.\n");
		goto out;
	}

	for (i = 0; i < req->payload_size; i++)
		req->payload[i] = mhu_mem_readl(gdata, SCP_LOW + 4 * i);

	/* Clear SPC interrupt.  */
	mhu_reg_writel(data, SCP_INTR_L_CLEAR, status);

	complete(&req->sync);

out:
	return IRQ_HANDLED;
}

static __devinit int arm_mhu_probe(struct platform_device *pdev)
{
	struct arm_mhu_data *data;
	struct resource *res;
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Unable to allocate private data\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No Registers resource\n");
		ret = -EINVAL;
		goto data_free;
	}

	data->regs = ioremap(res->start, res->end - res->start);
	if (!data->regs) {
		dev_err(&pdev->dev, "ioremap regs failed\n");
		ret = -ENODEV;
		goto data_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "No Memory resource\n");
		ret = -EINVAL;
		goto remap_free;
	}

	data->mem = ioremap(res->start, res->end - res->start);
	if (!data->mem) {
		dev_err(&pdev->dev, "ioremap mem failed\n");
		ret = -ENODEV;
		goto remap_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "No Memory resource\n");
		ret = -EINVAL;
		goto remap_free;
	}
	data->hi_irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		dev_err(&pdev->dev, "No Memory resource\n");
		ret = -EINVAL;
		goto remap_free;
	}
	data->lo_irq = res->start;     

	ret = request_threaded_irq(data->hi_irq, 0, arm_mhu_hi_irq_handler, 0, "arm_mhu_hi_irq", data);
	if (ret) {
		dev_err(&pdev->dev, "hi priority irq request failed\n");
		ret = -ENODEV;
		goto remap_free;
	}

	ret = request_threaded_irq(data->lo_irq, 0, arm_mhu_lo_irq_handler, 0, "arm_mhu_hi_irq", data);
	if (ret) {
		dev_err(&pdev->dev, "lo priority irq request failed\n");
		ret = -ENODEV;
		goto irq_free;		
	}

	platform_set_drvdata(pdev, data);

	mutex_init(&lo_mutex);
	mutex_init(&hi_mutex);
	INIT_LIST_HEAD(&lo_head);
	INIT_LIST_HEAD(&hi_head);
	gdata = data;

out:
	return ret;

irq_free:
	free_irq(data->hi_irq, data);

remap_free:
	iounmap(data->regs);

data_free:
	kfree(data);
	goto out;
}

static __devexit int arm_mhu_remove(struct platform_device *pdev)
{
	struct arm_mhu_data *data = platform_get_drvdata(pdev);

	free_irq(data->lo_irq, data);
	free_irq(data->hi_irq, data);
	iounmap(data->mem);
	iounmap(data->regs);
	kfree(data);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id arm_mhu_matches[] = {
	{ .compatible = "arm,mhu" },
	{},
}
#endif

static struct platform_driver arm_mhu_driver = {
	.probe  = arm_mhu_probe,
	.remove = __devexit_p(arm_mhu_remove),
	.driver = {
		.name  = "arm_mhu",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = arm_mhu_matches,
#endif
	},
};

int arm_mhu_init(void)
{
	return platform_driver_register(&arm_mhu_driver);
}

void arm_mhu_exit(void)
{
	platform_driver_unregister(&arm_mhu_driver);
}

module_init(arm_mhu_init);
module_exit(arm_mhu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ARM");
MODULE_DESCRIPTION("MHU");

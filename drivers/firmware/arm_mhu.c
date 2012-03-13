#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/io.h>
#include "arm_mhu.h"

int get_dvfs_size(int cluster, int cpu, int *size)
{
	return 0;
}

int get_dvfs_capabilities(int cluster, int cpu, int *freqs)
{
	return 0;
}

int get_performance(int cluster, int cpu, int *perf)
{
	return 0;
}

int set_performance(int cluster, int cpu, int index)
{
	

	return 0;
}

static irqreturn_t arm_mhu_hi_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static irqreturn_t arm_mhu_lo_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static __devinit int arm_mhu_probe(struct platform_device *pdev)
{
	struct arm_mhu_data *data;
	struct resource *res;
	int ret = 0;

	data= kzalloc(sizeof(*data), GFP_KERNEL);
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

	ret = request_irq(data->hi_irq, arm_mhu_hi_irq_handler, 0, "arm_mhu_hi_irq", data);
	if (ret) {
		dev_err(&pdev->dev, "hi priority irq request failed\n");
		ret = -ENODEV;
		goto remap_free;
	}

	ret = request_irq(data->lo_irq, arm_mhu_lo_irq_handler, 0, "arm_mhu_hi_irq", data);
	if (ret) {
		dev_err(&pdev->dev, "lo priority irq request failed\n");
		ret = -ENODEV;
		goto irq_free;		
	}

	platform_set_drvdata(pdev, data);

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

static struct platform_driver arm_mhu_driver = {
	.probe  = arm_mhu_probe,
	.remove = __devexit_p(arm_mhu_remove),
	.driver = {
		.name  = "arm_mhu",
		.owner = THIS_MODULE,
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

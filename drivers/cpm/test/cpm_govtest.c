/*
 *  linux/drivers/cpm/test/got_test.c
 *
 *  Copyright (C) 2009 STMicroelectronics
 *
 *  Author: Patrick Bellasi <derkling@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/cpm.h>
#include <linux/device.h>
#include <linux/notifier.h>

static unsigned short dc = 2;
module_param(dc, ushort, 0444);
MODULE_PARM_DESC(dc, "dev_count - number of platform device to register (should be >=2 )");

static unsigned short tw = 1;
module_param(tw, ushort, 0444);
MODULE_PARM_DESC(tw, "tree_width - number of DWR for top devices");

static unsigned short fmdc = 1;
module_param(fmdc, ushort, 0444);
MODULE_PARM_DESC(fmdc, "full-merge_dwrs_count - number of full-merging DWRs for last device");

static unsigned short cmdc = 1;
module_param(cmdc, ushort, 0444);
MODULE_PARM_DESC(cmdc, "cross-merge_dwrs_count - number of not-merging DWRs for last device");

static unsigned short nmdc = 1;
module_param(nmdc, ushort, 0444);
MODULE_PARM_DESC(nmdc, "not-merge_dwrs_count - number of not-merging DWRs for last device");

/* define a platform ASM */
#define CPM_PLATFORM_ASM(_name, _type, _mode, _comp, _min, _max)	\
	{								\
		.name = _name,						\
		.type = _type,						\
		.userw = _mode,						\
		.comp = _comp,						\
		.min = _min,						\
		.max = _max,						\
	}

/* initialize ASMs of a DWR */
#define DEV_DWR_ASM(_ASM_ID_, _LOWER_, _UPPER_, _TYPE_) \
{							\
	.id = _ASM_ID_,					\
	.range = {					\
		.lower = _LOWER_,			\
		.upper = _UPPER_,			\
		.type = _TYPE_,				\
	},						\
}

static void pdev_release(struct device *dev);


/*--- Platform ASMs ---*/
struct cpm_asm cpm_platform_asm[] =
{
	[0] = CPM_PLATFORM_ASM("ASM0", CPM_TYPE_LIB, CPM_USER_RW, CPM_COMPOSITION_RESTRICTIVE, 0, 100),
	[1] = CPM_PLATFORM_ASM("ASM1", CPM_TYPE_LIB, CPM_USER_RW, CPM_COMPOSITION_RESTRICTIVE, 0, 50),
	[2] = CPM_PLATFORM_ASM("ASM2", CPM_TYPE_GIB, CPM_USER_RW, CPM_COMPOSITION_ADDITIVE, 0, 40),
	[3] = CPM_PLATFORM_ASM("ASM3", CPM_TYPE_GIB, CPM_USER_RO, CPM_COMPOSITION_ADDITIVE, 0, 50),
};

struct cpm_platform_data cpm_platform_data =
{
	.asms = cpm_platform_asm,
	.count = ARRAY_SIZE(cpm_platform_asm),
};


/*--- Platform devices ---*/
static char pdev_name[] = "cpm_govtest";
static struct platform_device *pdevs;

int govtest_callback(struct notifier_block *nb, unsigned long step, void *data);

/*--- Upper-devices DWRs definitions ---*/
struct cpm_asm_range upperdev_dwr_ranges[] =
{
	DEV_DWR_ASM(0,  5, 30, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(1,  5, 25, CPM_ASM_TYPE_RANGE),
};
/* The list of upper-devs DWRs */
static struct cpm_dev_dwr *upperdev_dwrs_list;
static struct cpm_dev_data upperdev_data =
{
	.notifier_callback = govtest_callback,
};

/*--- Lower-device DWRs definitions ---*/
struct cpm_asm_range lowerdev_dwr0_ranges[] =
{
	DEV_DWR_ASM(0, 10, 20, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(1, 10, 15, CPM_ASM_TYPE_RANGE),
};
struct cpm_asm_range lowerdev_dwr1_ranges[] =
{
	DEV_DWR_ASM(0, 25, 35, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(1, 15, 20, CPM_ASM_TYPE_RANGE),
};
struct cpm_asm_range lowerdev_dwr2_ranges[] =
{
	DEV_DWR_ASM(0, 40, 50, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(1, 20, 25, CPM_ASM_TYPE_RANGE),
};
/* The list of lower-devs DWRs */
struct cpm_dev_dwr *lowerdev_dwrs_list;
static struct cpm_dev_data lowerdev_data =
{
	.notifier_callback = govtest_callback,
};

int govtest_callback(struct notifier_block *nb, unsigned long step, void *data)
{
	struct cpm_dev *pcdev = (struct cpm_dev*)container_of(nb, struct cpm_dev, nb);
	int result = NOTIFY_OK;

	switch(step) {
	case CPM_EVENT_NEW_CONSTRAINT:
		dev_info(pcdev->dev, "new constraint notification\n");
		result = NOTIFY_DONE;
		break;
	case CPM_EVENT_DO_CHANGE:
		dev_info(pcdev->dev, "DDP do-change: new DWR [%d] authorized\n", *(cpm_id*)data);
		break;
	case CPM_EVENT_ABORT:
		dev_info(pcdev->dev, "DDP abort\n");
		break;
	case CPM_EVENT_PRE_CHANGE:
		dev_info(pcdev->dev, "DDP pre-change, going to switch to DWR [%d]...\n", *(cpm_id*)data);
		break;
	case CPM_EVENT_POST_CHANGE:
		dev_info(pcdev->dev, "DDP post-change, switch to DWR [%d] complite\n", *(cpm_id*)data);
		break;
	default:
		dev_warn(pcdev->dev, "unexpected notification, returning OK (0)\n");
		result = NOTIFY_BAD;
	}

	return result;
}

static void release_cpm(void)
{
	struct platform_device *pd;
	int i;

	pd = pdevs;
	for (i=0; i<dc; pd++, i++) {
		if ( cpm_unregister_device(&pd->dev) ) {
			dev_err(&pd->dev, "cpm device data unregister failed\n");
			continue;
		}
		dev_info(&pd->dev, "cpm device data unregistered\n");
	}
	kfree(upperdev_dwrs_list);
	kfree(upperdev_dwrs_list);
}

static void release_platform(void)
{
	struct platform_device *pd;
	int i;

	pd = pdevs;
	for (i=0; i<dc; pd++, i++) {
		platform_device_unregister(pd);
		dev_info(&pd->dev, "platform device unregistered\n");
	}
	kfree(pdevs);
}


/*--- Platform device driver ---*/
static struct platform_driver govtest_driver =
{
	.driver	= {
		.name = "cpm_govtest",
		.owner = THIS_MODULE,
	},
};

static void pdev_release(struct device *dev)
{
	dev_info(dev, "platform device release\n");
}

static int __init govtest_probe(struct platform_device * pdev)
{
	dev_info(&(pdev->dev), "device probed: %s\n", dev_name(&(pdev->dev)));

	if ( strncmp(pdev->name, pdev_name, 11) ) {
		dev_err(&pdev->dev, "failed to probe cpm_govtest device\n");
		return -ENODEV;
	}

	return 0;
}

static int __init init_platform(void)
{
	int i;
	int ret;
	struct platform_device *pd;

	pr_debug("cpm_govtest: registering platform data...\n");

	/* allocate memory for the required platform device */
	pdevs = kzalloc(sizeof(struct platform_device)*dc, GFP_KERNEL);
	if ( unlikely(!pdevs) ) {
		pr_err("cpm_govtest: unable to allocate device, out-of-mem\n");
		return -ENOMEM;
	}
	
	/* initialize upper platform devices */
	pd = pdevs;
	for (i=1; i<dc; pd++, i++) {
		pd->name = pdev_name;
		pd->id = i;
		pd->dev.release = pdev_release;
	}
	/* lets make bottom device to have ID=0 */
	pd->name = pdev_name;
	pd->id = 0;
	pd->dev.release = pdev_release;

	/* register platform ASMs */
	cpm_register_platform_asms(&cpm_platform_data);
	pr_info("cpm_govtest: %d platform ASMs registered\n", cpm_platform_data.count);

	/* register platform devices */
	pd = pdevs;
	for (i=0; i<dc; pd++, i++) {
		ret = platform_device_register(pd);
		if ( unlikely(ret) ) {
			dev_err(&pd->dev, "platform device register failed\n");
			goto out_unregister;
		}
		dev_info(&pd->dev, "platform device registered\n");
	}
	pr_info("cpm_govtest: %d platform device registered\n", i);

	return 0;

out_unregister:

	pd--;
	for ( ; i; pd--, i--) {
		platform_device_unregister(pd);
		dev_info(&pd->dev, "platform device unregistered\n");
	}
	kfree(pdevs);

	return ret;

}

static int __init init_cpm(void)
{
	int i;
	int ret;
	struct cpm_dev_dwr *udl;
	struct cpm_dev_dwr *ldl;
	struct platform_device *pd;

	pr_debug("cpm_govtest: registering CPM data...\n");

	/* allocate memory for the required cpm's upper-device data */
	upperdev_dwrs_list = kzalloc(sizeof(struct cpm_dev_dwr)*tw, GFP_KERNEL);
	if ( unlikely(!upperdev_dwrs_list) ) {
		pr_err("cpm_govtest: unable to allocate upper-devices cpm data, out-of-mem\n");
		return -ENOMEM;
	}

	/* initializing upper-devices data */
	udl = upperdev_dwrs_list;
	for (i=0; i<tw; udl++, i++) {
		udl->id = i;
		snprintf(udl->name, CPM_NAME_LEN, "uDWR%02d", i);
		udl->asms = upperdev_dwr_ranges;
		udl->asms_count = 2;
	}
	upperdev_data.dwrs = upperdev_dwrs_list;
	upperdev_data.dwrs_count = i;

	/* registering upper-devices (first (cd-1) pdevs) */
	pd = pdevs;
	for (i=0; i<dc-1; pd++, i++) {
		ret = cpm_register_device(&pd->dev, &upperdev_data);
		if ( unlikely(ret) ) {
			dev_err(&pd->dev, "cpm upper-device register failed\n");
			goto out_unregister_upperdev;
		}
		dev_info(&pd->dev, "cpm upper-device register done\n");
	}

	pr_info("upper-devices registered\n");

	/* allocate memory for the required cpm's lover-device data */
	i = fmdc + cmdc + nmdc;
	lowerdev_dwrs_list = kzalloc(sizeof(struct cpm_dev_dwr)*i, GFP_KERNEL);
	if ( unlikely(!lowerdev_dwrs_list) ) {
		pr_err("cpm_govtest: unable to allocate lower-device cpm data, out-of-mem\n");
		ret = -ENOMEM;
		i = dc;
		goto out_unregister_upperdev;
	}

	/* initializing lower-devices data */
	ldl = lowerdev_dwrs_list;
	for (i=0; i<(fmdc+cmdc+nmdc); ldl++, i++) {
		ldl->id = i;
		snprintf(ldl->name, CPM_NAME_LEN, "lDWR%02d", i);
		ldl->asms = 	i <  fmdc 	? lowerdev_dwr0_ranges :
				i < (fmdc+cmdc)	? lowerdev_dwr1_ranges :
				lowerdev_dwr2_ranges;
		ldl->asms_count = 2;
	}
	lowerdev_data.dwrs = lowerdev_dwrs_list;
	lowerdev_data.dwrs_count = i;


	/* registering the lower-device */
	ret = cpm_register_device(&pd->dev, &lowerdev_data);
	if ( unlikely(ret) ) {
		dev_err(&pd->dev, "cpm lower-device register failed\n");
		goto out_unregister_lowerdev;
	}
	dev_info(&pd->dev, "cpm lower-device register done\n");

	pr_info("lower-device registered\n");

	return 0;

out_unregister_lowerdev:

	kfree(lowerdev_dwrs_list);

out_unregister_upperdev:

	pd--;
	for ( ; i; pd--, i--) {
		if ( cpm_unregister_device(&pd->dev) ) {
			dev_err(&pd->dev, "cpm device data unregister failed\n");
			continue;
		}
		dev_info(&pd->dev, "cpm device data unregistered\n");
	}
	kfree(upperdev_dwrs_list);

	return ret;


}

static int __init sanity_check_params(void)
{

	if ( unlikely(dc<2) ) {
		pr_err("device count cannot be less than 2\n");
		return -EINVAL;
	}

	if ( unlikely(!tw) ) {
		pr_err("top devices should have at least one DWR\n");
		return -EINVAL;
	}

	if ( unlikely( !(fmdc+cmdc+nmdc) ) ) {
		pr_err("bottom device should have at least ont DWR\n");
		return -EINVAL;
	}

	return 0;
}

static int __init cpm_govtest_init(void)
{

	int ret = 0;

	ret = sanity_check_params();
	if ( unlikely(ret) )
		return ret;

	ret = init_platform();
	if (unlikely(ret)) {
		printk(KERN_ERR "cpm_govtest: platform data register failed\n");
		goto out_platform;
	}
	pr_info("cpm_govtest: platform data initialized\n");

	ret = init_cpm();
	if (unlikely(ret)) {
		printk(KERN_ERR "cpm_govtest: cpm data register failed\n");
		goto out_platform;
	}
	pr_info("cpm_govtest: cpm data initialized\n");

	ret = platform_driver_probe(&govtest_driver, govtest_probe);
	if ( unlikely(ret) ) {
		pr_err("cpm_govtest: platform_driver_probe failed\n");
		goto out_all;
	}

	return 0;

out_all:

	release_cpm();

out_platform:

	release_platform();

	return ret;		
			
}

static void __exit cpm_govtest_exit(void)
{

	platform_driver_unregister(&govtest_driver);
	release_cpm();
	release_platform();

	pr_info("cpm_govtest: driver unloaded.\n");	
	
}
 

MODULE_AUTHOR("Patrick Bellasi <derkling@gmail.com>");
MODULE_DESCRIPTION("cpm_govtest: a governor performances test driver for the CPM framework");
MODULE_LICENSE("GPL");

module_init(cpm_govtest_init);
module_exit(cpm_govtest_exit);


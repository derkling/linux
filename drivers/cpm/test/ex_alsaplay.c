/*
 *  linux/drivers/cpm/test/alsaplay.c
 *
 *  Copyright (C) 2009 STMicroelectronics
 *
 *  Author: Matteo Carnevali <rekstorm@gmail.com>
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


#if 1
#define cpm_register_dwr 0
#endif

#define CPM_BUDDY_DATA_PORT	NULL //0x400
#define CPM_BUDDY_NR_PORTS	32
#define NUMBEROF_CPM_BUDDIES	1

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

/* initialize a DWR (name and id) */
#define DEV_DWR(_dev_, _id_, _name_, _asms_, _count_)		\
{					\
	.dev = _dev_,			\
	.id = _id_,			\
	.name = _name_,			\
	.asms = _asms_,			\
	.asms_count = _count_,		\
}

static void cpm_buddy_dev_release(struct device *dev);


/*--- Platform ASMs ---*/
struct cpm_asm cpm_platform_asm[] = {
	[0] = CPM_PLATFORM_ASM("LATENCY", CPM_TYPE_LIB, CPM_USER_RW, CPM_COMPOSITION_RESTRICTIVE, 0, 100),
	[1] = CPM_PLATFORM_ASM("THROUGHPUT", CPM_TYPE_GIB, CPM_USER_RW, CPM_COMPOSITION_ADDITIVE, 0, 100),
};

struct cpm_platform_data cpm_platform_data = {
	.asms = cpm_platform_asm,
	.count = ARRAY_SIZE(cpm_platform_asm),
};


/*--- Platform devices ---*/
static struct platform_device buddy1 = {
	.name = "cpm_buddy",
	.id = 1,
	.dev = {
		.release = cpm_buddy_dev_release,
	},
};

int cpm_buddy_callback(struct notifier_block *nb, unsigned long step, void *data);

/*--- DWRs definitions ---*/
struct cpm_asm_range buddy1_dwr0_ranges[] = {
	DEV_DWR_ASM(0, 0, 3, CPM_ASM_TYPE_UBOUND),
	DEV_DWR_ASM(1, 25, 100, CPM_ASM_TYPE_LBOUND),
};

struct cpm_asm_range buddy1_dwr1_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 6, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 12, 100, CPM_ASM_TYPE_LBOUND),
};

struct cpm_asm_range buddy1_dwr2_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 3, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 50, 100, CPM_ASM_TYPE_LBOUND),
};  
  
struct cpm_asm_range buddy1_dwr3_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 6, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 25, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr4_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 3, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 100, 100, CPM_ASM_TYPE_LBOUND),
};

struct cpm_asm_range buddy1_dwr5_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 6, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 50, 100, CPM_ASM_TYPE_LBOUND),
};    

struct cpm_asm_range buddy1_dwr6_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 12, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 6, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr7_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 24, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 3, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr8_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 12, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 12, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr9_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 24, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 6, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr10_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 12, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 25, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr11_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 24, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 12, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr12_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 50, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 2, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr13_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 100, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 1, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr14_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 50, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 4, 100, CPM_ASM_TYPE_LBOUND),
};  

struct cpm_asm_range buddy1_dwr15_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 100, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 2, 100, CPM_ASM_TYPE_LBOUND),
};

struct cpm_asm_range buddy1_dwr16_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 50, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 8, 100, CPM_ASM_TYPE_LBOUND),
};

struct cpm_asm_range buddy1_dwr17_ranges[] = {                         
        DEV_DWR_ASM(0, 0, 100, CPM_ASM_TYPE_UBOUND),
        DEV_DWR_ASM(1, 4, 100, CPM_ASM_TYPE_LBOUND),
};      

struct cpm_dev_dwr buddy1_dwrs_list[] = {
	DEV_DWR(&buddy1.dev, 0, "dwr0", buddy1_dwr0_ranges, ARRAY_SIZE(buddy1_dwr0_ranges)),
	DEV_DWR(&buddy1.dev, 1, "dwr1", buddy1_dwr1_ranges, ARRAY_SIZE(buddy1_dwr1_ranges)),
	DEV_DWR(&buddy1.dev, 2, "dwr2", buddy1_dwr2_ranges, ARRAY_SIZE(buddy1_dwr2_ranges)),
	DEV_DWR(&buddy1.dev, 3, "dwr3", buddy1_dwr3_ranges, ARRAY_SIZE(buddy1_dwr3_ranges)),
        DEV_DWR(&buddy1.dev, 4, "dwr4", buddy1_dwr4_ranges, ARRAY_SIZE(buddy1_dwr4_ranges)),
        DEV_DWR(&buddy1.dev, 5, "dwr5", buddy1_dwr5_ranges, ARRAY_SIZE(buddy1_dwr5_ranges)),
        DEV_DWR(&buddy1.dev, 6, "dwr6", buddy1_dwr6_ranges, ARRAY_SIZE(buddy1_dwr6_ranges)),
        DEV_DWR(&buddy1.dev, 7, "dwr7", buddy1_dwr7_ranges, ARRAY_SIZE(buddy1_dwr7_ranges)),
        DEV_DWR(&buddy1.dev, 8, "dwr8", buddy1_dwr8_ranges, ARRAY_SIZE(buddy1_dwr8_ranges)),
        DEV_DWR(&buddy1.dev, 9, "dwr9", buddy1_dwr9_ranges, ARRAY_SIZE(buddy1_dwr9_ranges)),
        DEV_DWR(&buddy1.dev, 10, "dwr10", buddy1_dwr10_ranges, ARRAY_SIZE(buddy1_dwr10_ranges)),
        DEV_DWR(&buddy1.dev, 11, "dwr11", buddy1_dwr11_ranges, ARRAY_SIZE(buddy1_dwr11_ranges)),
        DEV_DWR(&buddy1.dev, 12, "dwr12", buddy1_dwr12_ranges, ARRAY_SIZE(buddy1_dwr12_ranges)),
        DEV_DWR(&buddy1.dev, 13, "dwr13", buddy1_dwr13_ranges, ARRAY_SIZE(buddy1_dwr13_ranges)),
        DEV_DWR(&buddy1.dev, 14, "dwr14", buddy1_dwr14_ranges, ARRAY_SIZE(buddy1_dwr14_ranges)),
        DEV_DWR(&buddy1.dev, 15, "dwr15", buddy1_dwr15_ranges, ARRAY_SIZE(buddy1_dwr15_ranges)),
        DEV_DWR(&buddy1.dev, 16, "dwr16", buddy1_dwr16_ranges, ARRAY_SIZE(buddy1_dwr16_ranges)),
        DEV_DWR(&buddy1.dev, 17, "dwr17", buddy1_dwr17_ranges, ARRAY_SIZE(buddy1_dwr17_ranges)),
};
static struct cpm_dev_data buddy1_data = {
	.notifier_callback = cpm_buddy_callback,
	.dwrs = buddy1_dwrs_list,
	.dwrs_count = ARRAY_SIZE(buddy1_dwrs_list),
};

int cpm_buddy_callback(struct notifier_block *nb, unsigned long step, void *data)
{
	struct cpm_dev *pcdev = (struct cpm_dev*)container_of(nb, struct cpm_dev, nb);
	int result = NOTIFY_OK;

	switch(step) {
	case CPM_EVENT_NEW_CONSTRAINT:
		dev_info(pcdev->dev, "new constraint notification\n");
		result = NOTIFY_DONE;
		break;
	case CPM_EVENT_DO_CHANGE:
		dev_info(pcdev->dev, "DDP do-change: new DWR [%d] authorized\n", *(u16*)data);
		break;
	case CPM_EVENT_ABORT:
		dev_info(pcdev->dev, "DDP abort\n");
		break;
	case CPM_EVENT_PRE_CHANGE:
		dev_info(pcdev->dev, "DDP pre-change, going to switch to DWR [%d]...\n", *(u16*)data);
		break;
	case CPM_EVENT_POST_CHANGE:
		dev_info(pcdev->dev, "DDP post-change, switch to DWR [%d] complite\n", *(u16*)data);
		break;
	default:
		dev_warn(pcdev->dev, "unexpected notification, returning OK (0)\n");
		result = NOTIFY_BAD;
	}

	return result;
}


/*--- Platform device driver ---*/
static struct platform_driver cpm_buddy_driver = {
	.driver	= {
		.name = "cpm_buddy",
		.owner = THIS_MODULE,
	},
};

static void cpm_buddy_dev_release(struct device *dev)
{

}

static int __init cpm_buddy_probe(struct platform_device * pdev) {
	dev_info(&(pdev->dev), "device probed: %s\n", dev_name(&(pdev->dev)));
	return 0;
}

static int __init cpm_buddy_init_platform(void)
{
	int ret;

	printk(KERN_INFO "cpm_buddy: registering platform data...\n");

	cpm_register_platform_asms(&cpm_platform_data);

	ret = platform_device_register(&buddy1);
	if ( unlikely(ret<0) ) {
		dev_err(&(buddy1.dev), "device register failed\n");
		goto out_device1;
	}
	dev_info(&(buddy1.dev), "device register done\n");

out_device1:

	return ret;

}

static int __init cpm_buddy_init(void) {

	int ret = 0;

	ret = cpm_buddy_init_platform();
	if (unlikely(ret)) {
		printk(KERN_ERR "cpm_buddy: platform data register failed\n");
		goto out_platform;
	}

	printk(KERN_INFO "cpm_buddy: registering testing devices to cpm...\n");

	ret = cpm_register_device(&(buddy1.dev), &buddy1_data);
	if ( unlikely(ret<0) ) {
		dev_err(&(buddy1.dev), "cpm_register_device failed\n");
		goto out_cpm1;
	}
	dev_info(&(buddy1.dev), "cpm_register_device done\n");
	
	printk(KERN_INFO "cpm_buddy: drivers successfully loaded.\n");

	ret = platform_driver_probe(&cpm_buddy_driver, cpm_buddy_probe);
	if ( unlikely(ret<0) ) {
		printk(KERN_ERR "cpm_buddy: platform_driver_probe failed.\n");
		goto out_region;
	}
			
	return 0;

out_region:

out_cpm1:

out_platform:

	return ret;		
			
}


static void __exit cpm_buddy_exit(void) {


	cpm_unregister_device(&(buddy1.dev));

	platform_driver_unregister(&cpm_buddy_driver);
	platform_device_unregister(&buddy1);

	printk(KERN_INFO "cpm_buddy: driver unloaded.\n");	
	
}
 

MODULE_AUTHOR("Matteo Carnevali <rekstorm@gmail.com>");
MODULE_DESCRIPTION("cpm buddy: a test driver for CPM framework");
MODULE_LICENSE("GPL");

module_init(cpm_buddy_init);
module_exit(cpm_buddy_exit);


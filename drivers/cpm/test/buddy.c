/*
 *  linux/drivers/cpm/cpm_buddy.c
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


#if 1
#define cpm_register_dwr 0
#endif

#define CPM_BUDDY_DATA_PORT	NULL //0x400
#define CPM_BUDDY_NR_PORTS	32


/* macro to initialize ASMs of a DWR */
#define DEV_DWR_ASM(_ASM_ID_, _LOWER_, _UPPER_, _TYPE_) \
{							\
	.id = _ASM_ID_,					\
	.range = {					\
		.lower = _LOWER_,			\
		.upper = _UPPER_,			\
		.type = _TYPE_,				\
	},						\
}

/* dwr ranges (array) for dwr0 of buddy1 for each asm which belongs to dwr */
struct cpm_asm_range buddy1_dwr0_ranges[] = {
	DEV_DWR_ASM(0, 20, 50, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(1, 45, 70, CPM_ASM_TYPE_RANGE)	
};

/* dwr ranges (array) for dwr1 of buddy1 for each asm which belongs to dwr */
struct cpm_asm_range buddy1_dwr1_ranges[] = {
	DEV_DWR_ASM(0, 50, 70, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(2, 0, 30, CPM_ASM_TYPE_RANGE)
};

/* dwr ranges (array) for dwr2 of buddy1 for each asm which belongs to dwr */
struct cpm_asm_range buddy1_dwr2_ranges[] = {
	DEV_DWR_ASM(1, 0, 45, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(2, 30, 100, CPM_ASM_TYPE_RANGE)
};

/*macro to initialize a DWR (name and id) */
#define DEV_DWR(dwr_id, _id_)		\
{					\
	.region = {			\
		.name = #dwr_id,	\
		.id = _id_,		\
		},			\
}
 
 /* the list (array) of dwrs of buddy1, buddy 1 has 3 dwrs */
struct cpm_dev_dwr buddy1_dwrs_list[] = {
	DEV_DWR(dwr0, 0),
	DEV_DWR(dwr1, 1),
	DEV_DWR(dwr2, 2),
};

#define BUDDY1_DWR0_NUM_ASM \
sizeof(buddy1_dwr0_ranges)/sizeof(struct cpm_asm_range)

#define BUDDY1_DWR1_NUM_ASM \
sizeof(buddy1_dwr1_ranges)/sizeof(struct cpm_asm_range)

#define BUDDY1_DWR2_NUM_ASM \
sizeof(buddy1_dwr2_ranges)/sizeof(struct cpm_asm_range)


//buddy2

/* dwr ranges (array) for dwr0 of buddy1 for each asm which belongs to dwr */
struct cpm_asm_range buddy2_dwr0_ranges[] = {
	DEV_DWR_ASM(0, 22, 52, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(2, 47, 72, CPM_ASM_TYPE_RANGE)	
};

/* dwr ranges (array) for dwr1 of buddy1 for each asm which belongs to dwr */
struct cpm_asm_range buddy2_dwr1_ranges[] = {
	DEV_DWR_ASM(1, 52, 72, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(2, 2, 32, CPM_ASM_TYPE_RANGE)
};

/* dwr ranges (array) for dwr2 of buddy1 for each asm which belongs to dwr */
struct cpm_asm_range buddy2_dwr2_ranges[] = {
	DEV_DWR_ASM(1, 2, 47, CPM_ASM_TYPE_RANGE),
	DEV_DWR_ASM(3, 32, 102, CPM_ASM_TYPE_RANGE)
};

/*macro to initialize a DWR (name and id) */
#define DEV_DWR(dwr_id, _id_)		\
{					\
	.region = {			\
		.name = #dwr_id,	\
		.id = _id_,		\
		},			\
}
 
 /* the list (array) of dwrs of buddy1, buddy 1 has 3 dwrs */
struct cpm_dev_dwr buddy2_dwrs_list[] = {
	DEV_DWR(dwr0, 0),
	DEV_DWR(dwr1, 1),
	DEV_DWR(dwr2, 2),
};


/* SPECIALIZED macro to get the number of asm for a certain dwr of a device*/
#define BUDDY2_DWR0_NUM_ASM \
sizeof(buddy2_dwr0_ranges)/sizeof(struct cpm_asm_range)

#define BUDDY2_DWR1_NUM_ASM \
sizeof(buddy2_dwr1_ranges)/sizeof(struct cpm_asm_range)

#define BUDDY2_DWR2_NUM_ASM \
sizeof(buddy2_dwr2_ranges)/sizeof(struct cpm_asm_range)


/*macro to get the number of DWRs for a device*/
#define BUDDY_DWRS_NUM(_name_)	\
sizeof(_name_)/sizeof(struct cpm_dev_dwr)


/* GENERALIZED macro to get the number of asm for a certain dwr of a device
*  the _name_ is something like buddy1_dwr0_ranges
*/
#define BUDDY_DWR_NUM_ASM(_name_)	\
(sizeof(_name_)/sizeof(struct cpm_asm_range))
 

/* definition of the LIST_HEAD for dwrs list of a buddy analogous of 
						the array buddy1_dwrs_list[]*/
LIST_HEAD(buddy1_dwr_list);
LIST_HEAD(buddy2_dwr_list);

/*todo for each buddyX*/
static struct platform_device buddy1 = {
	.name = "cpm_buddy",
	.id = 1
};

static struct platform_device buddy2 = {
	.name = "cpm_buddy",
	.id = 2
};

/*todo for each buddyX_data*/
static struct cpm_dev_data *buddy1_data;
static struct cpm_dev_data *buddy2_data;

static struct platform_driver cpm_buddy_driver = {
//	.probe = cpm_buddy_probe,
//	.resume = cpm_buddy_resume,
	.driver	= {
		.name = "cpm_buddy",
		.owner = THIS_MODULE,
	},
};

// to be removed
int asm_array[4] = {20, 30, 40, 50};
int asm_array1[4] = {22, 32, 42, 52};


/* SYSFS interface - show and store*/

/*Show NAME of the device*/
static ssize_t cpm_buddy_name_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	return snprintf(buf, PAGE_SIZE, "cpm_buddy.%d\n", pdev->id);
}

static DEVICE_ATTR(name, 0444, cpm_buddy_name_show, NULL);


/* set constraint: SETCONSTRAINT*/
static ssize_t cpm_buddy_setconstraint_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	return snprintf(buf, PAGE_SIZE, "cpm_buddy.%d\n", pdev->id);
}

static ssize_t cpm_buddy_setconstraint_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	
	asm_array[3] = simple_strtoul(buf, NULL, 10);
	
	if(asm_array[3] > 1000){
		printk(KERN_INFO "cpm_buddy: asm3 > 1000.\n");
	}
	return count;
}

static DEVICE_ATTR(setconstraint, 0644, cpm_buddy_setconstraint_show, cpm_buddy_setconstraint_store);


/* remove constraint: RMCONSTRAINT*/
static ssize_t cpm_buddy_rmconstraint_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
//	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	return snprintf(buf, PAGE_SIZE, "remove\n");//, pdev->id);
}

static ssize_t cpm_buddy_rmconstraint_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	
	asm_array[3] = simple_strtoul(buf, NULL, 10);
	
	if(asm_array[3] > 1000){
		printk(KERN_INFO "cpm_buddy: asm3 > 1000.\n");
	}
	return count;
}

static DEVICE_ATTR(rmconstraint, 0644, cpm_buddy_rmconstraint_show, cpm_buddy_rmconstraint_store);


/* ASM0 */
static ssize_t cpm_buddy_asm0_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	
	if((pdev->id) == 0){
		return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[0]);
		}
	else if((pdev->id) == 1) {
		return snprintf(buf, PAGE_SIZE, "%d\n", asm_array1[0]);
		}
	else return snprintf(buf, PAGE_SIZE, "Error\n");
	
}				   


/* ASM1 */
static ssize_t cpm_buddy_asm1_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[1]);
}

/* ASM2 */
static ssize_t cpm_buddy_asm2_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[2]);
}

/* ASM3 */
static ssize_t cpm_buddy_asm3_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[3]);
}

static ssize_t cpm_buddy_asm3_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	
	asm_array[3] = simple_strtoul(buf, NULL, 10);
	if(asm_array[3] > 1000){
		printk(KERN_INFO "cpm_buddy: asm3 > 1000.\n");
	}
	return count;
}

static DEVICE_ATTR(asm0, 0444, cpm_buddy_asm0_show, NULL);
static DEVICE_ATTR(asm1, 0444, cpm_buddy_asm1_show, NULL);
static DEVICE_ATTR(asm2, 0444, cpm_buddy_asm2_show, NULL);
static DEVICE_ATTR(asm3, 0644, cpm_buddy_asm3_show, cpm_buddy_asm3_store);


static struct attribute *dwr1_attributes[] = {
	&dev_attr_asm0.attr,
	&dev_attr_asm1.attr,
	&dev_attr_asm2.attr,
	&dev_attr_asm3.attr,
	NULL
};

static const struct attribute_group dwr1_group =  { 
	.name = "dwr1",		//if name is present a new directory is created
	.attrs = dwr1_attributes 
};


static struct attribute *dwr2_attributes[] = {
	&dev_attr_asm0.attr,
	&dev_attr_asm1.attr,
	&dev_attr_asm2.attr,
	&dev_attr_asm3.attr,
	NULL
};

static const struct attribute_group dwr2_group =  { 
	.name = "dwr2",		//if name is present a new directory is created
	.attrs = dwr2_attributes 
};

/* /constraints/cons_asmX: CONS_ASM0*/
static ssize_t cpm_buddy_cons_asm0_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	
	if((pdev->id) == 0){
		return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[0]);
		}
	else if((pdev->id) == 1) {
		return snprintf(buf, PAGE_SIZE, "%d\n", asm_array1[0]);
		}
	else return snprintf(buf, PAGE_SIZE, "Error\n");
	
}				   

/* /constraints/cons_asmX: CONS_ASM1*/
static ssize_t cpm_buddy_cons_asm1_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[1]);
}

/* /constraints/cons_asmX: CONS_ASM2*/
static ssize_t cpm_buddy_cons_asm2_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[2]);
}

/* /constraints/cons_asmX: CONS_ASM3*/
static ssize_t cpm_buddy_cons_asm3_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", asm_array[3]);
}

static DEVICE_ATTR(cons_asm0, 0644, cpm_buddy_cons_asm0_show, NULL);
static DEVICE_ATTR(cons_asm1, 0644, cpm_buddy_cons_asm1_show, NULL);
static DEVICE_ATTR(cons_asm2, 0644, cpm_buddy_cons_asm2_show, NULL);
static DEVICE_ATTR(cons_asm3, 0644, cpm_buddy_cons_asm3_show, NULL);


static struct attribute *constraints_attributes[] = {
	&dev_attr_cons_asm0.attr,
	&dev_attr_cons_asm1.attr,
	&dev_attr_cons_asm2.attr,
	&dev_attr_cons_asm3.attr,
	NULL
};

static const struct attribute_group constraint_group =  { 
	.name = "constraints",
	.attrs = constraints_attributes 
};

/****** END SYSFS ******/


static int __init cpm_buddy_probe(struct platform_device * pdev) {
	printk(KERN_INFO "device probed: %s.%d\n", pdev->name, pdev->id);
	return 0;
}

static int __init cpm_buddy_init(void) {

	int ret = 0, i = 0;	

	printk(KERN_INFO "cpm_buddy: init started.\n");
/* macro to initialize a dwr x with its list of asms and associated ranges*/

#define INIT_DWR(_buddy, _dwr_id, _dwr) 					\
	INIT_LIST_HEAD(&(_buddy##_dwrs_list[_dwr_id].region.asm_range));	\
	for(i = 0; i < BUDDY_DWR_NUM_ASM(_buddy##_dwr##_ranges); i++){		\
		list_add(&(_buddy##_dwr##_ranges[i].node),			\
			&(_buddy##_dwrs_list[_dwr_id].region.asm_range));	\
	}
	
	INIT_DWR(buddy1, 0, _dwr0)
	INIT_DWR(buddy1, 1, _dwr1)
	INIT_DWR(buddy1, 2, _dwr2)
	printk(KERN_INFO "cpm_buddy1: INIT_DWR done.\n");
	
	INIT_DWR(buddy2, 0, _dwr0)
	INIT_DWR(buddy2, 1, _dwr1)
	INIT_DWR(buddy2, 2, _dwr2)
	printk(KERN_INFO "cpm_buddy2: INIT_DWR done.\n");
/*debug*/
	/*Init the asm_range list for dwr X */
/*	INIT_LIST_HEAD(&(buddy1_dwrs_list[0].region.asm_range));
	
	for(i = 0; i < BUDDY_DWR_NUM_ASM(buddy1_dwr0_ranges); i++){		
		list_add(&(buddy1_dwr0_ranges[i].node),	&(buddy1_dwrs_list[0].region.asm_range));	
	}*/


/* macro to initialize the list of dwrs for device (buddy) x*/
#define INIT_DEV_DWRLIST(_buddy)						\
	for(i = 0; i < BUDDY_DWRS_NUM(_buddy##_dwrs_list); i++){		\
		list_add(&(_buddy##_dwrs_list[i].node), &_buddy##_dwr_list);	\
	}

	/*to be done for each buddy controlled by this driver*/
	INIT_DEV_DWRLIST(buddy1)

	printk(KERN_INFO "cpm_buddy1: INIT_DEV_DWRLIST done.\n");
	
	INIT_DEV_DWRLIST(buddy2)

	printk(KERN_INFO "cpm_buddy2: INIT_DEV_DWRLIST done.\n");	
/*	for(i = 0; i < BUDDY_DWRS_NUM(buddy1_dwrs_list); i++){
		list_add(&(buddy1_dwrs_list[i].node), &buddy1_dwr_list);
	}*/

	/* NOT USED 'cause I do not have a real device now*/
	/*if (!request_region(CPM_BUDDY_DATA_PORT, CPM_BUDDY_NR_PORTS,
								"cpm_buddy")) {
		ret = -ENXIO;
		goto out;
	}*/
	

	ret = platform_device_register(&buddy1);	
	
	if (IS_ERR(&buddy1)) {
		ret = PTR_ERR(&buddy1);
		goto out_driver;
	}
	printk(KERN_INFO "cpm_buddy: platform_device_register done.\n");

	
	ret = platform_device_register(&buddy2);	
	
	if (IS_ERR(&buddy2)) {
		ret = PTR_ERR(&buddy2);
		goto out_driver;
	}
	printk(KERN_INFO "cpm_buddy: platform_device_register done.\n");
	

	buddy1_data = kmalloc(sizeof(struct cpm_dev_data), GFP_KERNEL);
	
	buddy1_data->dwrs = &buddy1_dwr_list;
	printk(KERN_INFO "cpm_buddy: buddy1_data->dwrs = &buddy1_dwr_list; done.\n");
	
	buddy2_data = kmalloc(sizeof(struct cpm_dev_data), GFP_KERNEL);
	
	buddy2_data->dwrs = &buddy2_dwr_list;
	printk(KERN_INFO "cpm_buddy: buddy2_data->dwrs = &buddy2_dwr_list; done.\n");	
	
	ret = cpm_register_device(&(buddy1.dev), buddy1_data);
	printk(KERN_INFO "cpm_buddy: cpm_register_device done.\n");
		
	ret = sysfs_create_file(&buddy1.dev.kobj, &dev_attr_name.attr);
	ret = sysfs_create_group(&buddy1.dev.kobj, &dwr1_group);
	ret = sysfs_create_group(&buddy1.dev.kobj, &dwr2_group);
	ret = sysfs_create_group(&buddy1.dev.kobj, &constraint_group);
	ret = sysfs_create_file(&buddy1.dev.kobj, &dev_attr_setconstraint.attr);
	ret = sysfs_create_file(&buddy1.dev.kobj, &dev_attr_rmconstraint.attr);
	printk(KERN_INFO "cpm_buddy: sysfs_create_files done.\n");
	
	if (ret)
		goto out_device1;	
	
	ret = cpm_register_device(&(buddy2.dev), buddy2_data);
	printk(KERN_INFO "cpm_buddy: cpm_register_device done.\n");
		
	ret = sysfs_create_file(&buddy2.dev.kobj, &dev_attr_name.attr);
	ret = sysfs_create_group(&buddy2.dev.kobj, &dwr1_group);
	ret = sysfs_create_group(&buddy2.dev.kobj, &dwr2_group);
	ret = sysfs_create_group(&buddy2.dev.kobj, &constraint_group);
	ret = sysfs_create_file(&buddy2.dev.kobj, &dev_attr_setconstraint.attr);
	ret = sysfs_create_file(&buddy2.dev.kobj, &dev_attr_rmconstraint.attr);
	printk(KERN_INFO "cpm_buddy: sysfs_create_file name done.\n");		
	if (ret)
		goto out_device2;

/* REFUSO??? esiste nel cpm_core.c cpm_register_dwr: cpm_register_dwr(struct device *dev, list_head *dwrs);  
	err = cpm_register_dwr(&dev, &buddy1_dwr_list);*/


	printk(KERN_INFO "cpm_buddy: driver successfully loaded.\n");
	
	ret = platform_driver_probe(&cpm_buddy_driver, cpm_buddy_probe);
	if (ret)
		goto out_region;
			
	return 0;
	
out_device1:
	platform_device_unregister(&buddy1);
	printk(KERN_INFO "cpm_buddy: sysfs_create_file FAILED.\n");
out_device2:
	platform_device_unregister(&buddy2);
	printk(KERN_INFO "cpm_buddy: sysfs_create_file FAILED.\n");
out_driver:
	platform_driver_unregister(&cpm_buddy_driver);
	printk(KERN_INFO "cpm_buddy: platform_device_register_simple FAILED.\n");
out_region:
	//release_region(CPM_BUDDY_DATA_PORT, CPM_BUDDY_NR_PORTS);
	printk(KERN_INFO "cpm_buddy: platform_driver_register FAILED.\n");
/*out:
	printk(KERN_WARNING "cpm_buddy: driver init failed (ret=%d)!\n", ret); */
	return ret;		
			
	
}


static void __exit cpm_buddy_exit(void) {


/*cpm_release_dwr*/

/*
#define SYSFS_DEL_PUT(cpm_buddy)	\
	kobject_del(cpm_buddy##_kobj);	\
	kobject_put(cpm_buddy##_kobj);
	
	SYSFS_DEL_PUT(cpm_buddy1)
	SYSFS_DEL_PUT(cpm_buddy2) */

	platform_driver_unregister(&cpm_buddy_driver);
	sysfs_remove_file(&buddy2.dev.kobj, &dev_attr_rmconstraint.attr);
	sysfs_remove_file(&buddy1.dev.kobj, &dev_attr_rmconstraint.attr);
	sysfs_remove_file(&buddy2.dev.kobj, &dev_attr_setconstraint.attr);
	sysfs_remove_file(&buddy1.dev.kobj, &dev_attr_setconstraint.attr);
	sysfs_remove_group(&buddy2.dev.kobj, &constraint_group);
	sysfs_remove_group(&buddy1.dev.kobj, &constraint_group);
	sysfs_remove_group(&buddy2.dev.kobj, &dwr2_group);
	sysfs_remove_group(&buddy1.dev.kobj, &dwr2_group);
	sysfs_remove_group(&buddy2.dev.kobj, &dwr1_group);
	sysfs_remove_group(&buddy1.dev.kobj, &dwr1_group);
	sysfs_remove_file(&buddy2.dev.kobj, &dev_attr_name.attr);
	sysfs_remove_file(&buddy1.dev.kobj, &dev_attr_name.attr);
	platform_device_unregister(&buddy2);
	platform_device_unregister(&buddy1);
	//release_region(APPLESMC_DATA_PORT, APPLESMC_NR_PORTS);

	printk(KERN_INFO "cpm_buddy: driver unloaded.\n");	
	
}
 

MODULE_AUTHOR("Matteo Carnevali <rekstorm@gmail.com>");
MODULE_DESCRIPTION("cpm buddy: a test driver for CPM framework");
MODULE_LICENSE("GPL");


module_init(cpm_buddy_init);
module_exit(cpm_buddy_exit);

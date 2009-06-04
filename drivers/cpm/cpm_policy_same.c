/*
 * This policy dummy manages fsc ordering and returns to cpm_core a pointer to 
 * a list of cpm_fsc_pointers. Elements of the list point to registered FSCs in
 * the same order as the original list.
 *  
 * It also dummy supervises ddp process returning always CPM_DDP_OK
 *
 * Stefano Bosisio <stebosisio@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/cpm.h>


struct list_head *original_fscs = 0;


/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/

#define dprintk(msg...) cpm_debug_printk(CPM_DEBUG_CORE, \
						"cpm-policy-same", msg)

#define iprintk(msg...) pr_info("cpm-policy-same" msg)

#define eprintk(msg...) pr_err("cpm-policy-same" msg)


static void dummy_ordering(struct work_struct *work)
{
	LIST_HEAD(ordered_fscs);
	struct cpm_fsc_pointer *newp = 0; 
	struct cpm_fsc *fscs = 0;

	dprintk("starting list creation\n");

	list_for_each_entry(fscs, original_fscs, node){
		newp = kzalloc(sizeof(struct cpm_fsc_pointer),GFP_KERNEL);
		if (!newp){
			eprintk("out-of-mem on cpm_fsc_pointer\n");
			return;
		}
		newp->fsc = fscs;
		INIT_LIST_HEAD(&newp->node);
		list_add(&newp->node,&ordered_fscs);
	}
	dprintk("calling cpm_set_ordered_fsc_list\n");
	cpm_set_ordered_fsc_list(&ordered_fscs);
	original_fscs =  0;

}


static DECLARE_WORK(cpm_work_policy,dummy_ordering);


int sort_fsc_list_same(struct list_head *fsc_list)
{
	int result = 0;	

	original_fscs = fsc_list;

	dprintk("queuing work\n");
	result = queue_work(cpm_wq,&cpm_work_policy);
	if (!result)
		return -EAGAIN;

	return 0;

} 


int ddp_handler_same(unsigned long phase, void *data)
{
	dprintk("ddp_handler returning CPM_DDP_OK");
	return CPM_DDP_OK;
}


struct cpm_policy cpm_policy_same = {
	.name		= "same policy",
	.sort_fsc_list	= sort_fsc_list_same,
	.ddp_handler	= ddp_handler_same,
};


static int __init cpm_policy_same_init(void)
{
	int err;
	err = cpm_register_policy(&cpm_policy_same);
	return err;
}


static void __exit cpm_policy_same_exit(void)
{
}


MODULE_AUTHOR("Stefano Bosisio <stebosisio@gmail.com>");
MODULE_DESCRIPTION("'cpm_same_policy'-cpm module that dummy manage ddp process"
		"and FSCs ordering");
MODULE_LICENSE("GPL");

module_init(cpm_policy_same_init);
module_exit(cpm_policy_same_exit);

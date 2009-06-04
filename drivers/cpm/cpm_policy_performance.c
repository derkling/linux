/*
 * This policy manages fsc ordering and returns to cpm_core a pointer to 
 * a list of cpm_fsc_pointers. Elements of the list point to registered FSCs and
 * order them according to he weight computed through... 
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
						"cpm-policy-perf", msg)

#define iprintk(msg...) pr_info("cpm-policy-perf" msg)

#define eprintk(msg...) pr_err("cpm-policy-perf" msg)


/*debug print an fsc list*/
void print_ord_fscs(struct list_head *list){
	struct cpm_fsc_pointer *p = 0;
	list_for_each_entry(p,list,node){
		dprintk("FSC:%2hu weight:%3hu\n",p->fsc->id,\
					*(u8*)(p->fsc->pol_data));
	}
}


static void performance_ordering(struct work_struct *work)
{
	LIST_HEAD(ordered_fscs);
	struct cpm_fsc_pointer *newp = 0, *p = 0; 
	struct cpm_fsc *fscs = 0;
	u32 tot_weight = 0, weight = 0, i = 0;
	int err = 0;

	dprintk("starting ordered list creation\n");

	/*iterate for each fsc*/
	list_for_each_entry(fscs, original_fscs, node){
		tot_weight = 0;
		/*evaluate weight of each asms*/
		dprintk("evaluating FSC:%2hu\n",fscs->id);
		dprintk("#asm:%3hu\n",fscs->asms_count);
		for (i = 0; i < fscs->asms_count;i++){
			err = cpm_weight_range(&(fscs->asms[i].range),\
						fscs->asms[i].id,&weight);
			if (err){
				eprintk("error on FSC weight evaluation\n");
				return;
			}
			else{
				tot_weight += weight;
				dprintk("fsc:%2hu id:%2hu weight:%3hu\n",\
					fscs->id,fscs->asms[i].id,weight);
			}
		}
		dprintk("fsc:%2hu TOT weight:%3hu\n",fscs->id,tot_weight);
		
		/*store computed weight into fsc's policy data*/
		if (fscs->pol_data){
			kfree(fscs->pol_data);
		}
		fscs->pol_data = kzalloc(sizeof(u8),GFP_KERNEL);
		if (!fscs->pol_data){
			eprintk("out-of-mem on u8\n");
			return;
		}else{
			dprintk("copying weight\n");
			*((u8*)(fscs->pol_data)) = tot_weight;
		}

		/*allocate and initialize element for pointer list*/
		newp = kzalloc(sizeof(struct cpm_fsc_pointer),GFP_KERNEL);
		if (!newp){
			eprintk("out-of-mem on cpm_fsc_pointer\n");
			return;
		}
		newp->fsc = fscs;
		INIT_LIST_HEAD(&newp->node);

		/*list empty: simply add*/
		if (list_empty(&ordered_fscs)){
			list_add(&newp->node,&ordered_fscs);
		}
		else{
			/*position search*/
			list_for_each_entry(p,&ordered_fscs,node){
				if ((*((u8*)(p->fsc->pol_data)) > tot_weight)\
				||(list_is_last((&p->node),&ordered_fscs)))
					break;
			}
			if (*((u8*)(p->fsc->pol_data)) > tot_weight){
				/*add in found position*/
				list_add_tail(&newp->node,&(p->node));
			}else{
				/*add at the end of the list*/
				list_add(&newp->node,&(p->node));
			}
		}

	}
	
	dprintk("ordered fsc list\n");
	print_ord_fscs(&ordered_fscs);
	dprintk("calling cpm_set_ordered_fsc_list\n");
	
	cpm_set_ordered_fsc_list(&ordered_fscs);
	original_fscs =  0;
	return;
}


static DECLARE_WORK(cpm_work_policy,performance_ordering);


int sort_fsc_list_performance(struct list_head *fsc_list)
{
	int result;	

	original_fscs = fsc_list;

	dprintk("queuing work\n");
	result = queue_work(cpm_wq,&cpm_work_policy);
	if (!result)
		return -EAGAIN;

	return 0;
} 


int ddp_handler_performance(unsigned long phase, void *data)
{
	dprintk("ddp_handler returning OK (0)\n");
	return 0;
}


struct cpm_policy cpm_policy_same = {
	.name		= "performance",
	.sort_fsc_list	= sort_fsc_list_performance,
	.ddp_handler	= ddp_handler_performance,
};


static int __init cpm_policy_performance_init(void)
{
	int err;
	err = cpm_register_policy(&cpm_policy_same);
	return err;
}


static void __exit cpm_policy_performance_exit(void)
{
	return; 
}


MODULE_AUTHOR("Stefano Bosisio <stebosisio@gmail.com>");
MODULE_DESCRIPTION("'cpm_policy_performance'-cpm module that order FSCs "
			"according to the weight computed through...");
MODULE_LICENSE("GPL");

module_init(cpm_policy_performance_init);
module_exit(cpm_policy_performance_exit);

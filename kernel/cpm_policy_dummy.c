/*
 * This module manages fsc ordering and returns to cpm_core a pointer to the 
 * current fsc ordered list to be used in ddp decision process.
 *
 * It also supervises ddp process returning a CPM_DDP_OK or a CPM_DDP_BAD
 * answer to the core according to proper OPPH considerations
 *
 * Stefano Bosisio <stebosisio@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/cpm.h>

typedef struct {
	struct timespec last_started_ddp;
	struct timespec last_ended_ddp;
	struct timespec delta_ddp;
} opph_data;


/************************** sysfs interface ************************/


/******************************sysfs end****************************/

static inline void compute_fsc_weight(struct cpm_fsc * fsc)
{
 return;
}

static inline int is_weighter(struct list_head *analized, struct list_head *ref)
{
	if (.../*is weighter*/)
		return TRUE;
	else
		return FALSE; 

}


int sort_fsc_list_dummy(struct list_head *fsc_list)
{
	struct cpm_fsc_core *fscs = 0, *ord_fscs = 0;
	LIST_HEAD(new_fsc);

	list_for_each_entry_safe (fscs, fsc_list, fsc.node){
		compute_fsc_weight(&fscs.fsc);

		list_for_each_entry(ord_fscs, new_fsc, fsc_node){
			if (is_weighter(fscs,ord_fscs))
				break;
		}	

		if (list_is_last(ord_fscs,new_fsc)&&(!is_weighter(fscs,ord_fscs)))
			list_move(fscs.node,ord_fscs.node);
		else
			list_move_tail(fscs.node,ord_fscs.node);
	}
	
	cpm_set_ordered_fsc_list(new_fsc);
	
	return TRUE;
} 


int ddp_handler_dummy (unsigned long phase, void *data)
{
	static opph_data opph_times; 
  

	switch (phase){
		case CPM_EVENT_NEW_CONSTRAINT:
			opph_times.last_start_ddp = current_kernel_time();
			break;

		case CPM_EVENT_FSC_FOUND:
			if (timespec_compare(opph_times.delta_ddp, timespec_sub(current_kernel_time-opph_times.last_ended_ddp))<0)
				return CPM_DDP_OK;
			else
				return CPM_DDP_BAD;

		case CPM_EVENT_PRE_CHANGE:
			break;

		case CPM_EVENT_DO_CHANGE:
			break;

		case CPM_EVENT_POST_CHANGE:
			opph_times.last_ended_ddp = current_kernel_time();
			opph_times.ddp_delta = timespec_sub(times.last_ended_ddp, times.last_started_ddp);
			break;

		default:
			break;
	} 
	return CPM_DDP_DONE;
}






struct cpm_policy cpm_policy_dummy {
	.name		= "dummy policy",
	.sort_fsc_list	= sort_fsc_list_dummy,
	.ddp_handler	= ddp_handler_dummy,
};


static int __init cpm_policy_dummy_init(void)
{
	int err;
 
	err = cpm_register_policy(&cpm_policy_dummy);

	return err;
}


static void __exit cpm_policy_dummy_exit(void)
{
	return; 
}


MODULE_AUTHOR("Stefano Bosisio <stebosisio@gmail.com>");
MODULE_DESCRIPTION("'cpm_policy' - cpm module that manage ddp process "
		"and fsc ordering");
MODULE_LICENSE("GPL");

module_init(cpm_policy_dummy_init);
module_exit(cpm_policy_dummy_exit);

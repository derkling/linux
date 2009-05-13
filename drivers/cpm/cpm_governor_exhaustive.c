/* 
 * This governor manages FSC generation through an exhaustive search process that
 * ideally builds and explores a complete DWRS COMBINATION TREE.

 * DWRS COMBINATION TREE is composed by a dummy root and by a level for each
 * device that is registered to cpm_core. Each level includes a node for each
 * DWR of the corresponding device. Edges between nodes allows to build paths
 * that starts from tree dummy root and that are composed by a number of steps
 * equal to the number of registered device. Each step corresponds to a dwr of a
 * different device. Tree's edges allows all possible DWR combination.
 *
 * Governor is based on a recursive mechanism: each recursive call explores DWRS 
 * of a different device. Each DWR is merged with the candidate FSC generated
 * by the merging of DWRS of the devices analized in less deeper recursive
 * level. If merge reach a successful conclusion and the current analyzed device
 * is not the last core registered device a new recursive call is made, passing 
 * the new candidate fsc. Othewise the next device's DWR is analyzed or the
 * control return to the upper recursive level if no others DWRs exists for
 * current device.
 *
 * Stefano Bosisio <stebosisio@gmail.com>
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cpm.h>


/*struct for tracing candidate fsc among the various recursive level*/
struct cpm_gov_asm_range{
	struct cpm_asm_range normal_range;	/*candidate range for an asm*/
	struct list_head node;		/*next range of this candidate fsc*/
	struct list_head old;		/*previous candidate range for asm*/
	struct cpm_dev *dev;	/*refer to device from which range derives*/
};

/*global data*/
struct list_head *h_dev = 0;		/*head of cpm_dev list*/
struct list_head *l_ranges = 0;		/*head of cpm_gov_asm_list*/
struct cpm_fsc_dwr *curr_dwr = 0;	/*dwr that maps with current candate fsc*/
struct list_head *__fsc_list = 0;	/*internal list of founded FSCs*/
u8 tot_dev = 0;				/*total number of cpm_dev*/
int tot_grgs = 0;			/*total ranges of candidate FSC*/

/*
 * Recursive function that explorates DWRS COMBINATION TREE 
 * @l_dev:	pointer to cpm_dev that interests the current recursive level
 * @ndev:	number of device analyzed in the current recursive level
 */
int __build_fsc_list_exhaustive(struct list_head *l_dev, u8 ndev)
{
	struct cpm_dev_dwr dwrs;
	struct cpm_asm_range d_rgs;
	struct cpm_gov_asm_range *g_rgs = 0, *g_rgs_next = 0, *new_g_rg = 0;
	struct cpm_fsc *new_fsc = 0;
	struct cpm_asm_range new_cand_rg;
	int merge_res = 0, idwr = 0, ndwr = 0, irgs = 0, nrgs = 0, i = 0;

	ndwr = list_entry(l_dev->next,struct cpm_dev,node)->dwrs_count;

	/*iterate on dwrs of the current level device*/
	for (idwr = 0 ; idwr < ndwr ; idwr++){
		merge_res = 0;
		dwrs = list_entry(l_dev->next,struct cpm_dev,node)->dwrs[idwr];
		if (list_empty(l_ranges)){
			/*no governor ranges are present yet: this is the first*/
			/*device level. DWR's ranges can be copied directly */
			nrgs = dwrs.asms_count;
			for (irgs = 0 ; irgs < nrgs ; irgs++){
				new_g_rg = kzalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
				if (!new_g_rg)
					return -ENOMEM;
				new_g_rg->normal_range = dwrs.asms[irgs];
				INIT_LIST_HEAD(&(new_g_rg->old));				
				INIT_LIST_HEAD(&(new_g_rg->node));
				new_g_rg->dev = list_entry(l_dev->next,struct cpm_dev,node);
				list_add(&(new_g_rg->node),l_ranges);
				tot_grgs++;
			}
		}else{
			/*governor ranges are present: this is not the first*/
			/*device. DWR's ranges must by compared with current*/
			/*governor ranges looking for a merge*/
			nrgs = dwrs.asms_count;
			for (irgs = 0 ; irgs < nrgs ; irgs++){
				d_rgs = dwrs.asms[irgs]; 
				list_for_each_entry(g_rgs,l_ranges,node){
					if (d_rgs.id == g_rgs->normal_range.id)
						break;	
				}
				if (d_rgs.id == g_rgs->normal_range.id){
					/*a candidate range for this asm exist*/
					/*in governor ranges. must be merged*/
					/* with the currend dwr range*/
					new_cand_rg = g_rgs->normal_range;
					merge_res = merge_cpm_range(&(new_cand_rg.range),&(dwrs.asms[irgs].range));
					if (merge_res == -EINVAL){
						/*no merging is possible: stop current dwr analysis*/
						break;
					}else{
						//merging is possible: new candidate range for this asm must be added
						new_g_rg = kzalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
						if (!new_g_rg)
							return -ENOMEM;
						new_g_rg->normal_range = new_cand_rg;
	                                	INIT_LIST_HEAD(&(new_g_rg->node));
	        	                        INIT_LIST_HEAD(&(new_g_rg->old));			
						new_g_rg->dev = list_entry(l_dev->next,struct cpm_dev,node);	
						list_add_tail(&new_g_rg->old,&g_rgs->old);
						list_replace(&g_rgs->node,&new_g_rg->node);
					}
				} else {
					/*a candidate range for this asm doesn't exist yet. it must be simply added*/
					new_g_rg = 0;
					new_g_rg = kzalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
					if (! new_g_rg)
						return -ENOMEM;
					new_g_rg->normal_range = dwrs.asms[irgs];
                                        INIT_LIST_HEAD(&(new_g_rg->node));
                                        INIT_LIST_HEAD(&(new_g_rg->old));
                                        new_g_rg->dev = list_entry(l_dev->next,struct cpm_dev,node);
					list_add(&new_g_rg->node,&g_rgs->node);
					tot_grgs++;
				}
			}
		}
		if (merge_res != -EINVAL){
			/*current DWR must be added to DWRs that map on current FSC*/
			curr_dwr[ndev].dwr = &(list_entry(l_dev->next,struct cpm_dev,node)->dwrs[idwr]);
			/*if this is the last device a NEW FSC has been found*/
			if (list_is_last(l_dev,h_dev)){
				/*last device has been query: new fsc can be added to fsc_list*/
				new_fsc	= (struct cpm_fsc *)(kzalloc(sizeof(struct cpm_fsc),GFP_KERNEL));
				if (!new_fsc)
					return -ENOMEM;
				new_fsc->dwrs = (struct cpm_fsc_dwr *)(kzalloc(sizeof(struct cpm_fsc_dwr)*tot_dev,GFP_KERNEL));
				if (!new_fsc->dwrs)
					return -ENOMEM;
				new_fsc->asms = (struct cpm_asm_range *)(kzalloc(sizeof(struct cpm_asm_range)*tot_grgs,GFP_KERNEL));
				new_fsc->gov_data = 0;
				new_fsc->pol_data = 0;
				INIT_LIST_HEAD(&(new_fsc->node));	
				i = 0;
				list_for_each_entry(g_rgs,l_ranges,node){
					new_fsc->asms[i] = g_rgs->normal_range;
					i++;
				}
				new_fsc->asms_count = tot_grgs;
				for (i=0; i < tot_dev; i++){
					new_fsc->dwrs[i] = curr_dwr[i];
				}
				new_fsc->dwrs_count = tot_dev;
				list_add_tail(&new_fsc->node,__fsc_list);
			}else{
				/*if this isn't the last device a new recursive call on the next device must be done*/
				__build_fsc_list_exhaustive(l_dev->next,ndev+1);
			}
		}
		/*remove all modification to governor asms ranges made by current device-dwr*/
		list_for_each_entry_safe(g_rgs, g_rgs_next, l_ranges,node){
			if (g_rgs->dev == container_of(l_dev, struct cpm_dev,node)){
				list_replace(&(g_rgs->node), &(container_of(&g_rgs->old, struct cpm_gov_asm_range, old))->node);
				list_del(&g_rgs->old);
				kfree(g_rgs);
				tot_grgs--;
			}
		}	
	}
	return 0;
}

/*
 * FSCs building function
 * @ndev:	number of devices registered to cpm_core
 *
 * Return value:	-EINVAL if dev_list is empty or if an empty fsc list is
 * 			 generated, TRUE if fsc generation is successful
 */
int build_fsc_list_exhaustive(struct list_head *dev_list, u8 ndev, struct list_head *fsc_list){
	
	/*Device list can't be empty*/
	if (!list_empty(dev_list)){
		/*Number of devices can't be equal to 0*/
		if (ndev == 0)
			return -EINVAL;
   		/*global data setup*/            
 		h_dev = dev_list;
		tot_dev = ndev;
		tot_grgs = 0;
		/*Array allocation for DWRs-FSC mapping*/
		curr_dwr = (struct cpm_fsc_dwr *)kzalloc(sizeof(struct cpm_fsc_dwr)*ndev,GFP_KERNEL);			
		if (!curr_dwr)
			return -ENOMEM;
		l_ranges = (struct list_head *)kzalloc(sizeof(struct list_head),GFP_KERNEL);
		INIT_LIST_HEAD(l_ranges);
	
		/*Call to recursive function for DWRS COMBINATION TREE exploration*/
		__build_fsc_list_exhaustive(dev_list, 0);	
	
		kfree(curr_dwr);
		kfree(l_ranges);
		if (list_empty(__fsc_list))
			return -EINVAL;
		else{
			fsc_list = __fsc_list;
			return TRUE;
		}
	}else
		return -EINVAL;	
}

/*
 * Governor data struct declaration
 */
struct cpm_governor cpm_governor_exhaustive = {
	.name = "exhaustive",
	.build_fsc_list = build_fsc_list_exhaustive,
};

/*
 * Module initialization function
 */
static int __init cpm_governor_exhaustive_init(void)
{
	int err = FALSE;
	err = cpm_register_governor(&cpm_governor_exhaustive);
	return err;
}

/*
 * Module unregistration function
 */ 
static void __exit cpm_governor_exhaustive_exit(void)
{
	return;
}

MODULE_AUTHOR("Stefano Bosisio <stebosisio@gmail.com>");
MODULE_DESCRIPTION("'cpm_governor_exhaustive' - cpm module that manage "
			"fscs generation throught exhaustive search");
MODULE_LICENSE("GPL");

module_init(cpm_governor_exhaustive_init);
module_exit(cpm_governor_exhaustive_exit);

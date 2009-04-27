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
	struct list_head old;		/*previous candidate range for asm*/
	struct cpm_dev *dev;	/*refer to device from which range derives*/
};

/*
 * Recursive function that explorates DWRS COMBINATION TREE 
 * @h_dev: 	pointer to the head of cpm_devs list registered to cpm_core
 * @l_dev:	pointer to cpm_dev that interests the current recursive level
 * @l_ranges:	pointer to the head cpm_gov_asm_ranges candidate list
 * @fsc_list:	pointer to current founded fscs
 */
void __build_fsc_list_exhaustive(struct list_head *h_dev, struct list_head *l_dev, struct list_head *l_ranges, struct list_head *fsc_list)
{
	struct cpm_dev_dwr *dwrs = 0;
	struct cpm_asm_range *d_rgs = 0;
	struct cpm_gov_asm_range *g_rgs = 0, *g_rgs_next = 0, *new_g_rg = 0;
	struct cpm_fsc *new_fsc = 0;
	struct cpm_asm_range new_cand_rg,*new_fsc_range = 0;
	int merge_res = 0;

	/*iterate on dwrs of the current level device*/
	merge_res = 0;
	list_entry(l_dev->next,struct cpm_dev,node);
	list_for_each_entry(dwrs,&(((list_entry(l_dev->next,struct cpm_dev,node))->dwr_list)),node){
		if (list_empty(l_ranges)){
			/*no governor ranges are present: this is the first*/
			/*device level. DWR's ranges can be copied directly */
			list_for_each_entry(d_rgs,&(dwrs->region.asm_range),node){
				new_g_rg = 0;
				while (!new_g_rg){
					new_g_rg = kmalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
				}
				new_g_rg->normal_range = *d_rgs;
				INIT_LIST_HEAD(&(new_g_rg->old));				
				INIT_LIST_HEAD(&(new_g_rg->normal_range.node));
				list_add(&(new_g_rg->normal_range.node),l_ranges);
			}
		}else{
			/*governor ranges are present: this is not the first*/
			/*device. DWR's ranges must by compared with current*/
			/*governor ranges looking for a merge*/
			list_for_each_entry(d_rgs,&(dwrs->region.asm_range),node){
				list_for_each_entry(g_rgs,l_ranges,normal_range.node){
					if (d_rgs->id == g_rgs->normal_range.id)
						break;	
				}
				if (d_rgs->id == g_rgs->normal_range.id){
					/*a candidate range for this asm exist*/
					/*in governor ranges. must be merged*/
					/* with the currend dwr range*/
					new_cand_rg = g_rgs->normal_range;
					merge_res = merge_cpm_range(&(new_cand_rg.range),&(d_rgs->range));
					if (merge_res == -EINVAL){
						/*no merging is possible: stop current dwr analysis*/
						break;
					}else{
						//merging is possible: new candidate range for this asm must be added
						new_g_rg = 0;
						while(!new_g_rg){
							new_g_rg = kmalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
						}
						new_g_rg->normal_range = new_cand_rg;
						new_g_rg->dev = list_entry(l_dev->next,struct cpm_dev,node);					
	                                	INIT_LIST_HEAD(&(new_g_rg->normal_range.node));
	        	                        INIT_LIST_HEAD(&(new_g_rg->old));			
						list_add_tail(&new_g_rg->old,&g_rgs->old);
						list_replace(&g_rgs->normal_range.node,&new_g_rg->normal_range.node);
					}
				} else {
					/*a candidate range for this asm doesn't exist yet. it must be simply added*/
					new_g_rg = 0;
					while(!new_g_rg){	
						new_g_rg = kmalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
					}
					new_g_rg->normal_range = *d_rgs;
                                        INIT_LIST_HEAD(&(new_g_rg->normal_range.node));
                                        INIT_LIST_HEAD(&(new_g_rg->old));
                                        new_g_rg->dev = list_entry(l_dev->next,struct cpm_dev,node);
					list_add(&new_g_rg->normal_range.node,&g_rgs->normal_range.node);
				}
			}
		}
		if (merge_res != -EINVAL){
			/*if this is the last device a NEW FSC has been found*/
			if (list_is_last(l_dev,h_dev)){
				/*last device has been query: new fsc can be added to fsc_list*/
				new_fsc = 0;
				while(!new_fsc){	
					new_fsc	= (struct cpm_fsc *)(kmalloc(sizeof(struct cpm_fsc),GFP_KERNEL));
				}
				new_fsc->gov_data = 0;
				new_fsc->pol_data = 0;
				INIT_LIST_HEAD(&(new_fsc->node));
				INIT_LIST_HEAD(&(new_fsc->region.asm_range));
				list_for_each_entry(g_rgs,l_ranges,normal_range.node){
					new_fsc_range = 0;
					while (!new_fsc_range){
						new_fsc_range = (struct cpm_asm_range*)(kmalloc(sizeof(struct cpm_asm_range),GFP_KERNEL));
					}
					*new_fsc_range = g_rgs->normal_range;
					INIT_LIST_HEAD(&(new_fsc_range->node));
					list_add_tail(&new_fsc_range->node,&new_fsc->region.asm_range);
				}
				list_add_tail(&new_fsc->node,fsc_list);
			}else{
				/*if this isn't the last device a new recursive call on the next device must be done*/
				__build_fsc_list_exhaustive(h_dev,l_dev->next,l_ranges,fsc_list);
			}
		}
		/*remove all modification to governor asms ranges made by current device-dwr*/
		list_for_each_entry_safe(g_rgs, g_rgs_next, l_ranges,normal_range.node){
			if (g_rgs->dev == container_of(l_dev, struct cpm_dev,node)){
				list_replace(&(g_rgs->normal_range.node), &(container_of(&g_rgs->old, struct cpm_gov_asm_range, old))->normal_range.node);
				list_del(&g_rgs->old);
				kfree(g_rgs);
			}
		}	
	}
	return;
}

/*
 * FSCs building function
 * @dev_list:	pointer to cpm_dev that interests the current recursive level
 * @fsc_list:	pointer to current founded fscs
 *
 * Return value:	-EINVAL if dev_list is empty or if an empty fsc list is
 * 			 generated, TRUE if fsc generation is successful
 */
int build_fsc_list_exhaustive(struct list_head *dev_list, struct list_head *fsc_list){
	/*The list for tracking FSC candidate among recursive calls*/
	LIST_HEAD(level_ranges);
	/*device list can't be empty*/
	if (!list_empty(dev_list)){
		/*Call to recursive function for DWRS COMBINATION TREE exploration*/
		__build_fsc_list_exhaustive(dev_list, dev_list, &level_ranges, fsc_list);	
		if (list_empty(fsc_list))
			return -EINVAL;
		else
			return TRUE;
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

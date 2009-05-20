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


/* cpm_gov_asm_range - the type for ranges of candidate FSCs during search*/
struct cpm_gov_asm_range{
	struct cpm_asm_range normal_range;	/*candidate range for an asm*/
	struct list_head node;		/*next range of this candidate fsc*/
	struct list_head old;		/*previous candidate range for asm*/
	struct cpm_dev *dev;	/*refer to device from which range derives*/
};


/*The head of passed cpm_dev list*/
struct list_head *h_dev = 0;

/*total number of cpm_dev*/
u8 tot_dev = 0;

/*The head of cpm_gov_asm_list*/
struct list_head *l_ranges = 0;

/*total ranges of candidate FSC*/
int tot_grgs = 0;

/*The DWRs that maps with current candidate FSC*/
struct cpm_fsc_dwr *curr_dwr = 0;

/*The internal list of founded FSCs*/
LIST_HEAD(__fsc_list);

/*The number of FSCs found*/
u8 tot_fsc = 0;




/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/

#define dprintk(msg...) cpm_debug_printk(CPM_DEBUG_CORE, \
						"cpm-gov-exh ", msg)

#define iprintk(msg...) pr_info("cpm-gov-exh: " msg)

#define eprintk(msg...) pr_err("cpm-gov-exh " msg)

/*debug print ranges strored during fsc search*/
void print_g_rgs(){
        struct cpm_gov_asm_range *p = 0, *pold = 0;
	struct cpm_range pr;
	list_for_each_entry(p,l_ranges,node){
		pr = p->normal_range.range;
		dprintk("dev:%2p id:%2hu t:%1hu l:%3hu u:%3hu\n",\
			p->dev,p->normal_range.id,pr.type,pr.lower,pr.upper);
		list_for_each_entry(pold,&p->old,old){
			pr = pold->normal_range.range;
			dprintk("dev:%2p id:%2hu t:%1hu l:%3hu u:%3hu\n",\
				pold->dev,pold->normal_range.id,pr.type,pr.lower,pr.upper);
		}
	}
}

/*debug print an fsc list*/
void print_fscs(){
	struct cpm_fsc *p = 0;
	int i = 0;
	struct cpm_asm_range r;
	list_for_each_entry(p,&__fsc_list,node){
		dprintk("FSC:\n");
		for (i=0;i<p->asms_count;i++){
			r = p->asms[i];

			dprintk("id:%2hu t:%1u l:%3hu u:%3hu\n",\
				r.id,r.range.type, r.range.lower,r.range.upper); 
		}
	}
}

/*
 * Recursive function that explorates DWRS COMBINATION TREE 
 * @l_dev:	pointer to cpm_dev that interests the current recursive level
 * @ndev:	number of device analyzed in the current recursive level
 */
int __build_fsc_list_exhaustive(struct list_head *l_dev, u8 ndev)
{
	struct cpm_dev *dev = 0;
	struct cpm_dev_dwr dwrs;
	struct cpm_asm_range d_rgs;
	struct cpm_gov_asm_range *g_rgs = 0, *g_rgs_next = 0, *new_g_rg = 0;
	struct cpm_fsc *new_fsc = 0;
	struct cpm_asm_range new_cand_rg;
	u8 idwr = 0, ndwr = 0, irgs = 0, i = 0;
	int merge_res = 0;
	
	dprintk("recursive search call: list_head 0x%p ndev:%2hu\n", l_dev, ndev);	

	dev = (list_entry(l_dev->next,struct cpm_dev,node));
	ndwr = dev->dwrs_count;
	dprintk("dev:%2hu total dwrs:%2hu\n", ndev, dev->dwrs_count);

	/*iterate on dwrs of the current level device*/
	for (idwr = 0 ; idwr < ndwr ; idwr++){
		dprintk("analizyng dwr:%2hu of dev:%2hu\n", idwr, ndev);
		merge_res = 0;
		dwrs = dev->dwrs[idwr];
		if (list_empty(l_ranges)){
			/*no governor ranges are present yet: this is the first*/
			/*device level. DWR's ranges can be copied directly */
			//nrgs = dwrs.asms_count;
			dprintk("l_ranges empty: adding DWR ranges\n");
			for (irgs = 0 ; irgs < dwrs.asms_count; irgs++){
				new_g_rg = kzalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
				if (!new_g_rg){
					eprintk("out-of-mem on cpm_gov_asm_range\n");
					return -ENOMEM;
				}
				new_g_rg->normal_range = dwrs.asms[irgs];
				INIT_LIST_HEAD(&(new_g_rg->old));				
				INIT_LIST_HEAD(&(new_g_rg->node));
				new_g_rg->dev = dev;
				dprintk("adding range t:%1hu l:%3hu u:%3hu\n",\
					 new_g_rg->normal_range.range.type,new_g_rg->normal_range.range.lower,\
						new_g_rg->normal_range.range.upper);
				list_add(&(new_g_rg->node),l_ranges);
				tot_grgs++;
			}
			dprintk("ranges added total ranges %2hu\n", tot_grgs);
		}else{
			/*governor ranges are present: this is not the first*/
			/*device. DWR's ranges must by compared with current*/
			/*governor ranges looking for a merge*/
			//nrgs = dwrs.asms_count 
			dprintk("l_ranges NOT empty: search if asm is  present\n");
			for (irgs = 0 ; irgs < dwrs.asms_count ; irgs++){
				dprintk("analizyng asm:%2hu of dwr:%2hu\n", irgs, idwr);		
				d_rgs = dwrs.asms[irgs]; 
				list_for_each_entry(g_rgs,l_ranges,node){
					if (d_rgs.id == g_rgs->normal_range.id)
						break;	
				}
				if (d_rgs.id == g_rgs->normal_range.id){
					/*a candidate range for this asm exist*/
					/*in governor ranges. must be merged*/
					/* with the currend dwr range*/
					dprintk("asm exist in local governor ranges\n");
					new_cand_rg = g_rgs->normal_range;
					merge_res = cpm_merge_range(&(new_cand_rg.range),&(dwrs.asms[irgs].range));
					dprintk("return merge:%2hu\n",merge_res);
					if (merge_res == -EINVAL){
						/*no merging is possible: stop current dwr analysis*/
						dprintk("no merge possible\n");
						break;
					}else{
						//merging is possible: new candidate range for this asm must be added
						dprintk("merging possible\n");
						new_g_rg = kzalloc(sizeof(struct cpm_gov_asm_range),GFP_KERNEL);
						if (!new_g_rg)
							return -ENOMEM;
						new_g_rg->normal_range = new_cand_rg;
	                                	INIT_LIST_HEAD(&(new_g_rg->node));
	        	                        INIT_LIST_HEAD(&(new_g_rg->old));			
						new_g_rg->dev = dev;	
						list_add_tail(&new_g_rg->old,&g_rgs->old);
						list_replace(&g_rgs->node,&new_g_rg->node);
						dprintk("adding range t:%1hu l:%3hu u:%3hu\n",\
							new_g_rg->normal_range.range.type,new_g_rg->normal_range.range.lower,\
								new_g_rg->normal_range.range.upper);
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
                                        new_g_rg->dev = dev;
					list_add(&new_g_rg->node,&g_rgs->node);
					tot_grgs++;
					dprintk("adding range t:%1hu l:%3hu u:%3hu\n",\
						new_g_rg->normal_range.range.type,new_g_rg->normal_range.range.lower,\
							new_g_rg->normal_range.range.upper);
					dprintk("ranges added total ranges %hu\n", tot_grgs);
				}
			}
		}
		
		/*print current governor ranges status*/
		print_g_rgs();
	
		if (merge_res != -EINVAL){
			/*current DWR must be added to DWRs that map on current FSC*/
			curr_dwr[ndev].dwr = &dev->dwrs[idwr];
			/*if this is the last device a NEW FSC has been found*/
			dprintk("next:%p head:%p\n",l_dev->next,h_dev);
			if (ndev == tot_dev-1){
				/*last device has been query: new fsc can be added to fsc_list*/
				dprintk("building the new FSC found\n");
				new_fsc = cpm_get_new_fsc(); 
				if (!new_fsc)
					return -ENOMEM;
				new_fsc->dwrs = (struct cpm_fsc_dwr *)(kzalloc(sizeof(struct cpm_fsc_dwr)*tot_dev,GFP_KERNEL));
				if (!new_fsc->dwrs)
					return -ENOMEM;
				new_fsc->asms = (struct cpm_asm_range *)(kzalloc(sizeof(struct cpm_asm_range)*tot_grgs,GFP_KERNEL));
				new_fsc->id = tot_fsc++;
				new_fsc->gov_data = 0;
				new_fsc->pol_data = 0;
				INIT_LIST_HEAD(&(new_fsc->node));	
				i = 0;
				dprintk("copy range into FSC\n");
				list_for_each_entry(g_rgs,l_ranges,node){
					new_fsc->asms[i] = g_rgs->normal_range;
					i++;
				}
				dprintk("ranges copied into fsc:%hu",i);
				dprintk("copy dwr that bind to the new FSC\n");
				new_fsc->asms_count = tot_grgs;
				for (i=0; i < tot_dev; i++){
					new_fsc->dwrs[i] = curr_dwr[i];
				}
				new_fsc->dwrs_count = tot_dev;
				dprintk("add new FSC to FSC LIST\n");
				list_add_tail(&new_fsc->node,&__fsc_list);
				dprintk ("added! NOT DONE\n");
			}else{
				/*if this isn't the last device a new recursive call on the next device must be done*/
				__build_fsc_list_exhaustive(l_dev->next,ndev+1);
			}
		}
		/*remove all modification to governor asms ranges made by current device-dwr*/
		dprintk("removing ranges of this device-dwr\n");
		dprintk("l_ranges BEFORE remove\n");
		print_g_rgs();		
		dprintk("dev: %p\n",dev);
		list_for_each_entry_safe(g_rgs, g_rgs_next, l_ranges,node){
			if (g_rgs->dev == dev){
				dprintk("%hu match!\n",ndev);
				if (list_empty(&g_rgs->old)){
					list_del(&g_rgs->node);
					tot_grgs--;
				}
				else{
					list_replace(&(g_rgs->node), &(container_of(g_rgs->old.next, struct cpm_gov_asm_range, old))->node);
					list_del(&g_rgs->old);
				}
				kfree(g_rgs);
			}
		}
		dprintk("l_ranges AFTER remove\n");
		print_g_rgs();		
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
int build_fsc_list_exhaustive(struct list_head *dev_list, u8 ndev)
{
	dprintk("entered in build_fsc_list\n"); 	
	dprintk("devlist 0x%p count %hu fsclist\n", dev_list, ndev);
	
	/*Device list can't be empty*/
	if (list_empty(dev_list)){
		eprintk("error on build_fsc_list_exhaustive dev_list==0\n");
		return -EINVAL;	
	}else{
		/*Number of devices can't be equal to 0*/
		if (ndev == 0){
			eprintk("error on build_fsc_list_exhaustive ndev==0");
			return -EINVAL;
		}

   		/*Global data setup*/            
 		h_dev = dev_list;
		tot_dev = ndev;
		tot_grgs = 0;
		tot_fsc = 0;	
	
		/*Array allocation for DWRs-FSC mapping*/
		curr_dwr = (struct cpm_fsc_dwr *)kzalloc(sizeof(struct cpm_fsc_dwr *)*ndev,GFP_KERNEL);
		if (!curr_dwr){
			eprintk("out-of-mem on cpm_fsc_dwr allocation\n");
			return -ENOMEM;
		}
		
		l_ranges = (struct list_head *)kzalloc(sizeof(struct list_head),GFP_KERNEL);
		if(!l_ranges){
			eprintk("out-of-mem on list_head allocation\n");
			return -ENOMEM;
		}
		INIT_LIST_HEAD(l_ranges);
	
		/*Call to recursive function for DWRS COMBINATION TREE exploration*/
		dprintk("starting recursive searching\n");
		__build_fsc_list_exhaustive(dev_list, 0);	
	
		if (list_empty(&__fsc_list)){
			dprintk("empty fsc list returned\n");
			return -EINVAL;
		}
		else{
			dprintk("fsc list returned\n");
			print_fscs();			
			cpm_set_fsc_list(&__fsc_list);
			INIT_LIST_HEAD(&__fsc_list);			
			return TRUE;
		}
		
		h_dev = 0;
		tot_dev = 0;		
		kfree(curr_dwr);
		kfree(l_ranges);
	}
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

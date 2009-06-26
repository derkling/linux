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

static struct list_head *original_fscs;

struct performance_data {
	s32 weight;
};

#define get_pol_data(_fsc)((struct performance_data *)_fsc->pol_data)

/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/

#ifdef CONFIG_CPM_POLICY_DEBUG
#define dprintk(msg...) pr_debug("cpm-policy-perf" msg)
#else
#define dprintk(msg...) do {} while (0);
#endif

#define iprintk(msg...) pr_info("cpm-policy-perf" msg)

#define eprintk(msg...) pr_err("cpm-policy-perf" msg)

/*debug print an fsc list*/
void print_ord_fscs(struct list_head *list)
{
	struct cpm_fsc_pointer *p = 0;
	list_for_each_entry(p, list, node) {
		dprintk("FSC:%2hu weight:%3d\n", p->fsc->id,
			*(u32 *) (p->fsc->pol_data));
	}
}

static int performance_ordering(void)
{
	LIST_HEAD(ordered_fscs);
	struct cpm_fsc_pointer *pnode = 0;
	struct cpm_fsc_pointer *p = 0;
	struct cpm_fsc *fscs = 0;
	s32 tot_weight = 0, weight = 0;
	u32 i = 0;
	int err = 0;
	int result = 0;

	dprintk("starting ordered list creation\n");

	/*iterate for each fsc */
	list_for_each_entry(fscs, original_fscs, node) {
		tot_weight = 0;
		/*evaluate weight of each asms */
		dprintk("evaluating FSC:%2hu\n", fscs->id);
		dprintk("#asm:%3hu\n", fscs->asms_count);
		for (i = 0; i < fscs->asms_count; i++) {
			err = cpm_weight_range(&(fscs->asms[i].range),
					       fscs->asms[i].id, &weight);
			if (err) {
				eprintk("error on FSC weight evaluation\n");
				result = -EINVAL;
				goto out_sort_nomem;
			} else {
				tot_weight += weight;
				dprintk("fsc:%2hu id:%2hu weight:%3d\n",
					fscs->id, fscs->asms[i].id, weight);
			}
		}
		dprintk("fsc:%2hu TOT weight:%3d\n", fscs->id, tot_weight);

		/*store computed weight into fsc's policy data */
		kfree(fscs->pol_data);

		fscs->pol_data =
		    kzalloc(sizeof(struct performance_data), GFP_KERNEL);
		if (!fscs->pol_data) {
			eprintk("out-of-mem on u8\n");
			result = -ENOMEM;
			goto out_sort_nomem;
		} else {
			dprintk("copying weight\n");
			get_pol_data(fscs)->weight = tot_weight;
		}

		/*allocate and initialize element for pointer list */
		pnode = kzalloc(sizeof(struct cpm_fsc_pointer), GFP_KERNEL);
		if (!pnode) {
			eprintk("out-of-mem on cpm_fsc_pointer\n");
			result = -ENOMEM;
			goto out_sort_nomem;
		}
		pnode->fsc = fscs;
		INIT_LIST_HEAD(&pnode->node);

		/*list empty: simply add */
		if (list_empty(&ordered_fscs)) {
			list_add(&pnode->node, &ordered_fscs);
		} else {
			/*position search */
			list_for_each_entry(p, &ordered_fscs, node) {
				if ((get_pol_data(p->fsc)->weight < tot_weight)
				    || list_is_last((&p->node), &ordered_fscs))
					break;
			}
			if (get_pol_data(p->fsc)->weight < tot_weight) {
				/*add in found position */
				list_add_tail(&pnode->node, &(p->node));
			} else {
				/*add at the end of the list */
				list_add(&pnode->node, &(p->node));
			}
		}

	}

	dprintk("ordered fsc list\n");
	print_ord_fscs(&ordered_fscs);
	dprintk("calling cpm_set_ordered_fsc_list\n");

	cpm_set_ordered_fsc_list(&ordered_fscs);
	original_fscs = 0;
	return 0;

out_sort_nomem:

	list_for_each_entry_safe(pnode, p, &ordered_fscs, node) {
		list_del(&pnode->node);
		kfree(pnode->fsc->pol_data);
		kfree(pnode);
	}

	return result;

}

int sort_fsc_list_performance(struct list_head *fsc_list)
{
	int result = 0;

	original_fscs = fsc_list;

	result = performance_ordering();

	return result;
}

int ddp_handler_performance(unsigned long phase, void *data)
{
	dprintk("ddp_handler returning OK (0)\n");
	return 0;
}

struct cpm_policy cpm_policy_same = {
	.name = "performance",
	.sort_fsc_list = sort_fsc_list_performance,
	.ddp_handler = ddp_handler_performance,
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

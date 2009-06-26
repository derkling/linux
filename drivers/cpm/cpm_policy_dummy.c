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

/*********************************************************************
 *                     UNIFIED DEBUG HELPERS                         *
 *********************************************************************/

#ifdef CONFIG_CPM_POLICY_DEBUG
#define dprintk(msg...) pr_debug("cpm-policy-same" msg)
#else
#define dprintk(msg...) do {} while (0);
#endif

#define iprintk(msg...) pr_info("cpm-policy-same" msg)

#define eprintk(msg...) pr_err("cpm-policy-same" msg)

int sort_fsc_list_same(struct list_head *fsc_list)
{
	struct cpm_fsc_pointer *pnode = 0;
	struct cpm_fsc_pointer *pnext = 0;
	struct cpm_fsc *fscs = 0;
	LIST_HEAD(ordered_fscs);
	int result = 0;

	dprintk("starting list creation\n");

	list_for_each_entry(fscs, fsc_list, node) {
		pnode = kzalloc(sizeof(struct cpm_fsc_pointer), GFP_KERNEL);
		if (unlikely(!pnode)) {
			eprintk("out-of-mem on cpm_fsc_pointer\n");
			result = -ENOMEM;
			goto out_sort_nomem;
		}
		pnode->fsc = fscs;
		list_add(&pnode->node, &ordered_fscs);
	}

	dprintk("calling cpm_set_ordered_fsc_list\n");
	cpm_set_ordered_fsc_list(&ordered_fscs);

	return 0;

out_sort_nomem:

	list_for_each_entry_safe(pnode, pnext, &ordered_fscs, node) {
		list_del(&pnode->node);
		kfree(pnode);
	}

	return result;

}

int ddp_handler_same(unsigned long phase, void *data)
{

	switch (phase) {
	case CPM_EVENT_NEW_CONSTRAINT:
		dprintk("new constraint notification: authorized\n");
		break;
	case CPM_EVENT_FSC_FOUND:
		dprintk("next FSC notification: authorized\n");
		break;
	case CPM_EVENT_PRE_CHANGE:
		dprintk("DDP pre-change notification\n");
		break;
	case CPM_EVENT_POST_CHANGE:
		dprintk("DDP post-change notification\n");
		break;
	default:
		dprintk("unexpected notification, returning OK (0)\n");
	}
	return 0;
}

struct cpm_policy cpm_policy_same = {
	.name = "same policy",
	.sort_fsc_list = sort_fsc_list_same,
	.ddp_handler = ddp_handler_same,
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

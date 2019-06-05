/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/module.h>
#include <linux/static_call.h>
#include <asm/bug.h>

static int foo_a(int x)
{
	return x+1;
}

static int foo_b(int x)
{
	return x*2;
}

DEFINE_STATIC_CALL(foo, foo_a);

static int __init test_static_call_init(void)
{
	WARN_ON(static_call(foo, 2) != 3);

	static_call_update(foo, foo_b);

	WARN_ON(static_call(foo, 2) != 4);

	static_call_update(foo, foo_a);

	WARN_ON(static_call(foo, 2) != 3);

	return 0;
}
module_init(test_static_call_init);

static void __exit test_static_call_exit(void)
{
}
module_exit(test_static_call_exit);

MODULE_AUTHOR("Peter Zijlstra <peterz@infradead.org>");
MODULE_LICENSE("GPL");

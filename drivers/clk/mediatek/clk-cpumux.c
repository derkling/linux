/*
 * Copyright (c) 2015 Linaro Ltd.
 * Author: Pi-Cheng Chen <pi-cheng.chen@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "clk-mtk.h"
#include "clk-cpumux.h"

#define ARMPLL_INDEX	1
#define MAINPLL_INDEX	2

static inline struct mtk_clk_cpumux *to_clk_mux(struct clk_hw *_hw)
{
	return container_of(_hw, struct mtk_clk_cpumux, hw);
}

static u8 clk_cpumux_get_parent(struct clk_hw *hw)
{
	struct mtk_clk_cpumux *mux = to_clk_mux(hw);
	int num_parents = __clk_get_num_parents(hw->clk);
	u32 val;

	val = clk_readl(mux->reg) >> mux->shift;
	val &= mux->mask;

	if (val >= num_parents)
		return -EINVAL;

	return val;
}

static int clk_cpumux_set_parent(struct clk_hw *hw, u8 index)
{
	struct mtk_clk_cpumux *mux = to_clk_mux(hw);
	unsigned long flags = 0;
	u32 val;

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);

	val = clk_readl(mux->reg);
	val &= ~(mux->mask << mux->shift);
	val |= index << mux->shift;
	clk_writel(val, mux->reg);

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);

	return 0;
}

static int clk_cpumux_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk *armpll = clk_get_parent_by_index(hw->clk, ARMPLL_INDEX);
	struct clk *mainpll = clk_get_parent_by_index(hw->clk, MAINPLL_INDEX);
	struct clk *cur_parent = clk_get_parent(hw->clk);
	int ret1, ret2;

	if (cur_parent == armpll) {
		ret1 = clk_set_parent(hw->clk, mainpll);
		BUG_ON(ret1);
	}

	ret2 = clk_set_rate(armpll, rate);

	ret1 = clk_set_parent(hw->clk, armpll);

	return ret2 ? ret2 : ret1;
}

static long clk_cpumux_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	struct clk *armpll = clk_get_parent_by_index(hw->clk, ARMPLL_INDEX);

	return clk_round_rate(armpll, rate);
}

static unsigned long clk_cpumux_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk *armpll = clk_get_parent_by_index(hw->clk, ARMPLL_INDEX);

	return clk_get_rate(armpll);
}

static struct clk_ops clk_cpumux_ops = {
	.get_parent = clk_cpumux_get_parent,
	.set_parent = clk_cpumux_set_parent,
	.set_rate = clk_cpumux_set_rate,
	.round_rate = clk_cpumux_round_rate,
	.recalc_rate = clk_cpumux_recalc_rate,
};

static struct clk *mtk_clk_register_cpumux(struct mtk_mux *mux,
					   void __iomem *base, spinlock_t *lock)
{
	struct mtk_clk_cpumux *cpumux;
	struct clk *clk;
	struct clk_init_data init;

	cpumux = kzalloc(sizeof(*cpumux), GFP_KERNEL);
	if (!cpumux)
		return ERR_PTR(-ENOMEM);

	init.name = mux->name;
	init.ops = &clk_cpumux_ops;
	init.parent_names = mux->parent_names;
	init.num_parents = mux->num_parents;

	cpumux->reg = base + mux->reg;
	cpumux->lock = lock;
	cpumux->shift = mux->shift;
	cpumux->mask = (BIT(mux->width) - 1);
	cpumux->hw.init = &init;

	clk = clk_register(NULL, &cpumux->hw);
	if (IS_ERR(clk))
		kfree(cpumux);

	return clk;
}

int mtk_clk_register_cpumuxes(void __iomem *base, struct mtk_mux *clks, int num,
			      struct clk_onecell_data *clk_data,
			      spinlock_t *lock)
{
	int i;
	struct clk *clk;

	if (!clk_data)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		struct mtk_mux *mux = &clks[i];

		clk = mtk_clk_register_cpumux(mux, base, lock);
		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
			       mux->name, PTR_ERR(clk));
			continue;
		}

		clk_data->clks[mux->id] = clk;
	}

	return 0;
}

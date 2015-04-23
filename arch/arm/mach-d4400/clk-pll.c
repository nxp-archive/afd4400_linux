/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gcr.h>

#include "clk.h"

/**
 * struct d4400_hw_clk_pll - D4400 PLL clock
 * @clk_hw:	 clock source
 * @ccm_base:	 base address of CCM registers
 * @type:	 pll type
 * @lock:	register lock
 */
struct d4400_hw_clk_pll {
	struct clk_hw	hw;
	void __iomem	*ccm_base;
	enum d4400_pll_type type;
	spinlock_t	*lock;
};

#define PLL_TIMEOUT_MS  100

#define CCDR2_REG_OFFSET    0x0C
#define CMCR2_REG_OFFSET    0x98
#define SPLLGSR_REG_OFFSET  0x120
#define SPLLLKSR_REG_OFFSET 0x130
#define DPLLGSR_REG_OFFSET  0x160
#define DPLLLKSR_REG_OFFSET 0x170
#define TPLLGSR_REG_OFFSET  0x1A0
#define TPLLLKSR_REG_OFFSET 0x1B0
#define TPLLGDCR_REG_OFFSET 0x2A0

#define CCDR2_DEVCLK_DIV_OFFSET	24
#define CCDR2_DEVCLK_DIV_MASK	(0x7 << CCDR2_DEVCLK_DIV_OFFSET)

#define CMCR2_PLL_SYS_HRESET (1<<24)
#define CMCR2_PLL_DDR_HRESET (1<<25)
#define CMCR2_PLL_TBGEN_HRESET (1<<26)
#define CMCR2_PLL_SYS_HRESET_STAT (1<<20)
#define CMCR2_PLL_DDR_HRESET_STAT (1<<21)
#define CMCR2_PLL_TBGEN_HRESET_STAT (1<<22)

#define SPLLGSR_KILL_OFFSET  31
#define SPLLGSR_KILL_MASK    (0x1<<SPLLGSR_KILL_OFFSET)
#define SPLLGSR_CFG_OFFSET   1
#define SPLLGSR_CFG_MASK    (0x3F<<SPLLGSR_CFG_OFFSET)
#define DPLLGSR_KILL_OFFSET  31
#define DPLLGSR_KILL_MASK    (0x1<<DPLLGSR_KILL_OFFSET)
#define DPLLGSR_CFG_OFFSET   1
#define DPLLGSR_CFG_MASK     (0x3F<<DPLLGSR_CFG_OFFSET)
#define TPLLGSR_KILL_OFFSET  31
#define TPLLGSR_KILL_MASK    (0x1<<TPLLGSR_KILL_OFFSET)
#define TPLLGSR_CFG_OFFSET   1
#define TPLLGSR_CFG_MASK     (0x3F<<TPLLGSR_CFG_OFFSET)

#define SPLLLKSR_PLL_LKD_OFFSET  22
#define SPLLLKSR_PLL_LKD_MASK    (0x1<<SPLLLKSR_PLL_LKD_OFFSET)
#define SPLLLKSR_LFM_LKD_OFFSET   23
#define SPLLLKSR_LFM_LKD_MASK    (0x1<<SPLLLKSR_LFM_LKD_OFFSET)
#define DPLLLKSR_PLL_LKD_OFFSET  22
#define DPLLLKSR_PLL_LKD_MASK    (0x1<<DPLLLKSR_PLL_LKD_OFFSET)
#define DPLLLKSR_LFM_LKD_OFFSET   23
#define DPLLLKSR_LFM_LKD_MASK    (0x1<<DPLLLKSR_LFM_LKD_OFFSET)
#define TPLLLKSR_PLL_LKD_OFFSET  22
#define TPLLLKSR_PLL_LKD_MASK    (0x1<<TPLLLKSR_PLL_LKD_OFFSET)
#define TPLLLKSR_LFM_LKD_OFFSET   23
#define TPLLLKSR_LFM_LKD_MASK    (0x1<<TPLLLKSR_LFM_LKD_OFFSET)

#define SPLL_MASK (SPLLLKSR_PLL_LKD_MASK | SPLLLKSR_LFM_LKD_MASK)
#define SPLL_ACTIVE SPLL_MASK
#define DPLL_MASK (DPLLLKSR_PLL_LKD_MASK | DPLLLKSR_LFM_LKD_MASK)
#define DPLL_ACTIVE DPLL_MASK
#define TPLL_MASK (TPLLLKSR_PLL_LKD_MASK | TPLLLKSR_LFM_LKD_MASK)
#define TPLL_ACTIVE TPLL_MASK

#define nPLLGDCR_OCLKVCOD_OFFSET  17
#define nPLLGDCR_OCLKVCOD_MASK   (0x3<<nPLLGDCR_OCLKVCOD_OFFSET)
#define nPLLGDCR_OCLKVCOD_DIV2   (0<<nPLLGDCR_OCLKVCOD_OFFSET)
#define nPLLGDCR_OCLKVCOD_DIV4   (1<<nPLLGDCR_OCLKVCOD_OFFSET)
#define nPLLGDCR_OCLKVCOD_DIV6   (2<<nPLLGDCR_OCLKVCOD_OFFSET)
#define nPLLGDCR_OCLKVCOD_DIV8   (3<<nPLLGDCR_OCLKVCOD_OFFSET)
#define nPLLGDCR_CFG_OFFSET  1
#define nPLLGDCR_CFG_MASK    (0xFF<<nPLLGDCR_CFG_OFFSET)
#define nPLLGDCR_OVERIDE_EN  (1<<0)

#define to_clk_pll(_hw) container_of(_hw, struct d4400_hw_clk_pll, hw)

static int clk_pll_enable(struct clk_hw *hw)
{
	struct d4400_hw_clk_pll *pll = to_clk_pll(hw);
	unsigned long timeout = jiffies + msecs_to_jiffies(10);

	switch (pll->type) {
	case D4400_PLL_SYS:
		/* check if LFM_LKD and HFM_LKD bits in
		*  system PLLLKSR reg are ON
		*/
		while ((readl_relaxed(pll->ccm_base + SPLLLKSR_REG_OFFSET)
			& SPLL_MASK) != SPLL_ACTIVE)
			if (time_after(jiffies, timeout))
				return -ETIMEDOUT;
		break;
	case D4400_PLL_DDR:
		/* check if LFM_LKD and HFM_LKD bits in
		*  DDR PLLLKSR reg are ON
		*/
		while ((readl_relaxed(pll->ccm_base + DPLLLKSR_REG_OFFSET)
			& DPLL_MASK) != DPLL_ACTIVE)
			if (time_after(jiffies, timeout))
				return -ETIMEDOUT;
		break;
	case D4400_PLL_TBGEN:
		/* check if LFM_LKD and HFM_LKD bits in
		* TBGEN PLLLKSR reg are ON
		*/
		while ((readl_relaxed(pll->ccm_base + TPLLLKSR_REG_OFFSET)
			& TPLL_MASK) != TPLL_ACTIVE)
			if (time_after(jiffies, timeout))
				return -ETIMEDOUT;
		break;
	default:
		return -EBADRQC;
	}
	return 0;
}

static void clk_pll_disable(struct clk_hw *hw)
{
	struct d4400_hw_clk_pll *pll = to_clk_pll(hw);
	u32 val;
	unsigned long flags = 0;

	switch (pll->type) {
	case D4400_PLL_SYS:
		if (pll->lock)
			spin_lock_irqsave(pll->lock, flags);
		val = readl_relaxed(pll->ccm_base + SPLLGSR_REG_OFFSET);
		val |= SPLLGSR_KILL_MASK;
		writel_relaxed(val, pll->ccm_base + SPLLGSR_REG_OFFSET);
		if (pll->lock)
			spin_unlock_irqrestore(pll->lock, flags);
		break;
	case D4400_PLL_DDR:
		if (pll->lock)
			spin_lock_irqsave(pll->lock, flags);
		val = readl_relaxed(pll->ccm_base + DPLLGSR_REG_OFFSET);
		val |= DPLLGSR_KILL_MASK;
		writel_relaxed(val, pll->ccm_base + DPLLGSR_REG_OFFSET);
		if (pll->lock)
			spin_unlock_irqrestore(pll->lock, flags);
		break;
	case D4400_PLL_TBGEN:
		if (pll->lock)
			spin_lock_irqsave(pll->lock, flags);
		val = readl_relaxed(pll->ccm_base + TPLLGSR_REG_OFFSET);
		val |= TPLLGSR_KILL_MASK;
		writel_relaxed(val, pll->ccm_base + TPLLGSR_REG_OFFSET);
		if (pll->lock)
			spin_unlock_irqrestore(pll->lock, flags);
		break;
	default:
		break;
	}
}

unsigned long clk_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct d4400_hw_clk_pll *pll = to_clk_pll(hw);
	unsigned long val = 0;

	switch (pll->type) {
	case D4400_PLL_SYS:
		val = readl_relaxed(pll->ccm_base + SPLLGSR_REG_OFFSET);
		val &= SPLLGSR_CFG_MASK;
		val >>= SPLLGSR_CFG_OFFSET;
		break;
	case D4400_PLL_DDR:
		val = readl_relaxed(pll->ccm_base + DPLLGSR_REG_OFFSET);
		val &= DPLLGSR_CFG_MASK;
		val >>= SPLLGSR_CFG_OFFSET;
		break;
	case D4400_PLL_TBGEN:
		val = readl_relaxed(pll->ccm_base + TPLLGSR_REG_OFFSET);
		val &= TPLLGSR_CFG_MASK;
		val >>= SPLLGSR_CFG_OFFSET;
		break;
	default:
		return -EBADRQC;
	}
	return val * parent_rate;
}

static int clk_pll_set_parent(struct clk_hw *hw, u8 index)
{
	struct d4400_hw_clk_pll *pll = to_clk_pll(hw);
	unsigned long flags = 0;

	switch (pll->type) {
	case D4400_PLL_SYS:
		if (pll->lock)
			spin_lock_irqsave(pll->lock, flags);

		if (index)
			gcr_set_pll_parent(SET_SYS_PLL_PARENT,
						PARENT_SRC_SGMIICLK);
		else
			gcr_set_pll_parent(SET_SYS_PLL_PARENT,
						PARENT_SRC_DEVCLK);

		if (pll->lock)
			spin_unlock_irqrestore(pll->lock, flags);
		break;
	case D4400_PLL_DDR:
		if (pll->lock)
			spin_lock_irqsave(pll->lock, flags);
		if (index)
			gcr_set_pll_parent(SET_DDR_PLL_PARENT,
						PARENT_SRC_SGMIICLK);
		else
			gcr_set_pll_parent(SET_DDR_PLL_PARENT,
						PARENT_SRC_DEVCLK);

		if (pll->lock)
			spin_unlock_irqrestore(pll->lock, flags);
		break;
	case D4400_PLL_TBGEN:
		break;
	default:
		return -EBADRQC;
	}

	return 0;
}

static u8 clk_pll_get_parent(struct clk_hw *hw)
{
	struct d4400_hw_clk_pll *pll = to_clk_pll(hw);
	u32 val;

	switch (pll->type) {
	case D4400_PLL_SYS:
		val = gcr_get_pll_parent(GET_SYS_PLL_PARENT);
		if (val == SYS_PARENT_CLK_SRC)
			return 1;
		else if (!val)
			return 0;
		break;
	case D4400_PLL_DDR:
		val = gcr_get_pll_parent(GET_DDR_PLL_PARENT);
		if (val == DDR_PARENT_CLK_SRC)
			return 1;
		else if (!val)
			return 0;
		break;
	default:
		break;
	}
	return -EBADRQC;
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	unsigned long min_rate = parent_rate * 1;
	unsigned long max_rate = parent_rate * 64;
	u32 div;
	u32 mfn, mfd = 1000000;
	s64 temp64;

	if (rate > max_rate)
		rate = max_rate;
	else if (rate < min_rate)
		rate = min_rate;

	div = rate / parent_rate;
	temp64 = (u64) (rate - div * parent_rate);
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	return parent_rate * div + parent_rate / mfd * mfn;
}

static int clk_pll_tbgen_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct d4400_hw_clk_pll *pll = to_clk_pll(hw);
	u32 multiplier, val;
	unsigned long timeout;
	unsigned long error;

	/* Parent rate should be 122.88 MHz */
	if (parent_rate != 122880000)
		return -EBADRQC;

	multiplier = (rate + parent_rate/2) / parent_rate;
	error = rate - multiplier * parent_rate;
	if (multiplier < 4 || multiplier > 8 || error != 0)
		return -EBADRQC;

	/* Reset TBGEN PLL */
	val = readl(pll->ccm_base + CMCR2_REG_OFFSET);
	val |= CMCR2_PLL_TBGEN_HRESET;
	writel(val, pll->ccm_base + CMCR2_REG_OFFSET);

	timeout = jiffies + msecs_to_jiffies(PLL_TIMEOUT_MS);
	val = readl(pll->ccm_base + CMCR2_REG_OFFSET);
	while (val & CMCR2_PLL_TBGEN_HRESET_STAT) {
		if (time_is_before_jiffies(timeout))
			return -EBUSY;
		else
			schedule_timeout_interruptible(1);
		val = readl(pll->ccm_base + CMCR2_REG_OFFSET);
	}

	/* Reconfigure */
	val = (multiplier << nPLLGDCR_CFG_OFFSET) | nPLLGDCR_OVERIDE_EN;
	if (rate >= 600100000UL)
		val |= nPLLGDCR_OCLKVCOD_DIV4;
	else
		val |= nPLLGDCR_OCLKVCOD_DIV6;
	writel(val, pll->ccm_base + TPLLGDCR_REG_OFFSET);

	val = readl(pll->ccm_base + CCDR2_REG_OFFSET);
	val = (val & (~CCDR2_DEVCLK_DIV_MASK)) |
	      ((multiplier - 1) << CCDR2_DEVCLK_DIV_OFFSET);
	writel(val, pll->ccm_base + CCDR2_REG_OFFSET);

	udelay(200);

	/* Restart TBGEN PLL */
	val = readl(pll->ccm_base + CMCR2_REG_OFFSET);
	val &= ~CMCR2_PLL_TBGEN_HRESET;
	writel(val, pll->ccm_base + CMCR2_REG_OFFSET);

	timeout = jiffies + msecs_to_jiffies(PLL_TIMEOUT_MS);
	val = readl(pll->ccm_base + TPLLLKSR_REG_OFFSET);
	while (!(val & TPLLLKSR_PLL_LKD_MASK)) {
		if (time_is_before_jiffies(timeout))
			return -EBUSY;
		else
			schedule_timeout_interruptible(1);
		val = readl(pll->ccm_base + TPLLLKSR_REG_OFFSET);
	};

	return 0;
}

static const struct clk_ops clk_pll_ops = {
	.enable		= clk_pll_enable,
	.disable	= clk_pll_disable,
	.recalc_rate	= clk_pll_recalc_rate,
	.round_rate	= clk_pll_round_rate,
	.set_parent	= clk_pll_set_parent,
	.get_parent	= clk_pll_get_parent,
};

static const struct clk_ops clk_pll_tbgen_ops = {
	.enable		= clk_pll_enable,
	.disable	= clk_pll_disable,
	.recalc_rate	= clk_pll_recalc_rate,
	.round_rate	= clk_pll_round_rate,
	.set_rate	= clk_pll_tbgen_set_rate,
};

struct clk *d4400_clk_pll(enum d4400_pll_type type, const char *name,
			 void __iomem *ccm_base,
			  const char **parent_names, int num_parents)
{
	struct d4400_hw_clk_pll *pll;
	const struct clk_ops *ops;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);
	if (type != D4400_PLL_TBGEN)
		ops = &clk_pll_ops;
	else
		ops = &clk_pll_tbgen_ops;
	pll->ccm_base = ccm_base;
	pll->type = type;

	init.name = name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}

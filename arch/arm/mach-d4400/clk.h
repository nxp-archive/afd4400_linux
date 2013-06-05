#ifndef __MACH_D4400_CLK_H
#define __MACH_D4400_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>

extern spinlock_t d4400_ccm_lock;

enum d4400_pll_type {
	D4400_PLL_SYS,
	D4400_PLL_DDR,
	D4400_PLL_TBGEN,
	D4400_PLL_TBGEN_HALF,
};

struct clk *d4400_clk_pll(enum d4400_pll_type type, const char *name,
			void __iomem *scm_base, void __iomem *ccm_base,
			const char **parent_name, int num_parents);

struct clk *d4400_clk_gate(struct device *dev, const char *name,
	                const char *parent_name, unsigned long flags,
	                void __iomem *reg, u8 bit_idx,
		        u8 clk_gate_flags, spinlock_t *lock);
#endif


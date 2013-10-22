#ifndef __MACH_D4400_CLK_H
#define __MACH_D4400_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>

#define CCM_VPGCSR_OFFSET 0x9C
#define CCM_CCDR2_OFFSET 0xC
#define CCM_REV_CLK_SEL  0x4000
#define CCM_REV_CLK_DEV_MASK  0x3

extern spinlock_t d4400_ccm_lock;

enum d4400_pll_type {
	D4400_PLL_SYS,
	D4400_PLL_DDR,
	D4400_PLL_TBGEN,
	D4400_PLL_TBGEN_HALF,
};


struct clk *d4400_clk_pll(enum d4400_pll_type type, const char *name,
			void __iomem *ccm_base,
			const char **parent_name, int num_parents);

struct clk *d4400_clk_gate(struct device *dev, const char *name,
	                const char *parent_name, unsigned long flags,
	                void __iomem *reg, u8 bit_idx,
		        u8 clk_gate_flags, spinlock_t *lock);

extern void d4400_rev_clk_select(u8 cpri_id, u8 clk_dev);
extern int d4400_ccm_vspa_full_pow_gate(u8 vspa_id);
extern int d4400_ccm_vspa_full_pow_up(u8 vspa_id);

#endif


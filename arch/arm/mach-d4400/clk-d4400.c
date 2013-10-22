/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include "clk.h"
#include "common.h"


DEFINE_SPINLOCK(d4400_ccm_lock);
static void __iomem *ccm_base;

void __init d4400_clock_map_io(void) { }

enum d4400_clks {
	dummy, osc1_dev, osc2_sgmii, sys_pll, ddr_pll, tbgen_pll,
	tbgen_pll_half,	ref, sys, arm, per, vsp_clk, ram, sys_bus,
	ahb, ip, gpc, uart1_serial, uart1_per, uart2_serial, uart2_per,
	uart3_serial, uart3_per, uart4_serial, uart4_per, weim_sel,
	weim_clk, div_pll_sys, etsec_rtc, sync_ref, async_ckil, sync_ckil,
	ecspi1_sel, ecspi1_clk, ecspi2_sel, ecspi2_clk, ecspi3_sel, ecspi3_clk,
	ecspi4_sel, ecspi4_clk, ecspi5_sel, ecspi5_clk, ecspi6_sel, ecspi6_clk,
	ecspi7_sel, ecspi7_clk, ecspi8_sel, ecspi8_clk, vspa_dp_sel,
	vspa_dp_clk, ccm_at, trace_clk, ddr_pll_by, mmdc_sel, mmdc_clk,
	epit_ipg, epit_per, uart1_ipg, uart1_pen,  uart2_ipg, uart2_pen,
        uart3_ipg, uart3_pen, uart4_ipg, uart4_pen, i2c1_per,
	i2c2_per, i2c3_per, i2c4_per, i2c5_per, i2c6_per,
	i2c7_per, i2c8_per, i2c9_per, i2c10_per, i2c11_per, sync_ref_src_1,
	sync_ref_src_2, clk_max
};

static const char *pll_sys_sels[]  = { "osc1_dev", "osc2_sgmii" };
static const char *pll_ddr_sels[]  = { "osc1_dev", "osc2_sgmii" };
static const char *pll_tbgen_sels[]  = { "osc1_dev" };
static const char *ref_sels[]  = { "osc1_dev", "osc2_sgmii" };
static const char *sys_sels[]  = { "ref", "sys_pll" };
static const char *uart1_sels[] = {"ref", "sys_bus"};
static const char *uart2_sels[] = {"ref", "sys_bus"};
static const char *uart3_sels[] = {"ref", "sys_bus"};
static const char *uart4_sels[] = {"ref", "sys_bus"};
static const char *weim_sels[] = {"ref", "sys_pll"};
static const char *etsec_rtc_sels[] = {"osc1_dev", "osc2_sgmii"};
static const char *ecspi1_sels[] = {"ref", "sys_bus"};
static const char *ecspi2_sels[] = {"ref", "sys_bus"};
static const char *ecspi3_sels[] = {"ref", "sys_bus"};
static const char *ecspi4_sels[] = {"ref", "sys_bus"};
static const char *ecspi5_sels[] = {"ref", "sys_bus"};
static const char *ecspi6_sels[] = {"ref", "sys_bus"};
static const char *ecspi7_sels[] = {"ref", "sys_bus"};
static const char *ecspi8_sels[] = {"ref", "sys_bus"};
static const char *trace_sels[] = {"mmdc_clk", "sys_pll", "tbgen_pll"};
static const char *mmdc_sels[] = {"ref", "sys_pll", "ddr_pll", "ddr_pll_by"};
static const char *vspa_dp_sels[] = {"sys_bus", "sys_pll", "ddr_pll",
					"ddr_pll_by"};
static const char *sync_ref_sels[] = {"sync_ref_src_1", "sync_ref_src_2"};

static struct clk *clk[clk_max];
static struct clk_onecell_data clk_data;

void d4400_rev_clk_select(u8 cpri_id, u8 clk_dev)
{
	u32 val;

	val = readl(ccm_base + CCM_CCDR2_OFFSET);
	val &= (~CCM_REV_CLK_DEV_MASK) << 16;
	val |= clk_dev << 16;
	if (cpri_id == 1) /* select recover clock from cpri1 else 2 */
		val &= ~CCM_REV_CLK_SEL;
	else
		val |= CCM_REV_CLK_SEL;
	writel(val, ccm_base + CCM_CCDR2_OFFSET);
}
EXPORT_SYMBOL(d4400_rev_clk_select);

int d4400_ccm_vspa_full_pow_gate(u8 vspa_id)
{
	u32 val;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);

	val = readl(ccm_base + CCM_VPGCSR_OFFSET);
	val |= BITS_MASK(1, (vspa_id - 1));
	writel(val, ccm_base + CCM_VPGCSR_OFFSET);

	while (!(readl(ccm_base + CCM_VPGCSR_OFFSET)
		& BITS_MASK(1, (vspa_id + 0x10 - 1))))
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	return 0;
}

int d4400_ccm_vspa_full_pow_up(u8 vspa_id)
{
	u32 val;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);

	val = readl(ccm_base + CCM_VPGCSR_OFFSET);
	val &= ~BITS_MASK(1, (vspa_id - 1));
	writel(val, ccm_base + CCM_VPGCSR_OFFSET);

	while (readl(ccm_base + CCM_VPGCSR_OFFSET)
		& BITS_MASK(1, (vspa_id + 0x10 - 1)))
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	return 0;
}

int __init d4400_clocks_init(void)
{
	struct device_node *np;
	void __iomem *base;
	int i, irq;

	clk[dummy] = clk_register_fixed_rate(NULL, "dummy", NULL,
			CLK_IS_ROOT, 0);

	/* retrieve the freqency of fixed clocks from device tree */
	for_each_compatible_node(np, NULL, "fixed-clock") {
				u32 rate;
	if (of_property_read_u32(np, "clock-frequency", &rate))
		continue;

	if (of_device_is_compatible(np, "fsl,d4400-osc1-dev-clk"))
		clk[osc1_dev] = clk_register_fixed_rate(NULL, "osc1_dev", NULL,
			CLK_IS_ROOT, rate);
	else if (of_device_is_compatible(np, "fsl,d4400-osc2-sgmii-clk"))
		clk[osc2_sgmii] = clk_register_fixed_rate(NULL, "osc2_sgmii", NULL,
					CLK_IS_ROOT, rate);
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,d4400-ccm");
	/* no device tree device */
	if (!np)
		return -ENODEV;

	ccm_base = of_iomap(np, 0);
	WARN_ON(!ccm_base);

	clk[sys_pll] = d4400_clk_pll(D4400_PLL_SYS, "sys_pll",
				ccm_base, pll_sys_sels,
				ARRAY_SIZE(pll_sys_sels));
	clk[ddr_pll] = d4400_clk_pll(D4400_PLL_DDR, "ddr_pll",
				ccm_base, pll_ddr_sels,
				ARRAY_SIZE(pll_ddr_sels));
	clk[tbgen_pll] = d4400_clk_pll(D4400_PLL_TBGEN, "tbgen_pll",
				ccm_base, pll_tbgen_sels,
				ARRAY_SIZE(pll_tbgen_sels));

	clk[tbgen_pll_half] = clk_register_fixed_factor(NULL, "tbgen_pll_half",
				"tbgen_pll", CLK_SET_RATE_PARENT, 1, 2);

	clk[ref] = clk_register_mux(NULL, "ref", ref_sels, ARRAY_SIZE(ref_sels),
				0, ccm_base + 0x4, 4, 1, 0, &d4400_ccm_lock);
	clk[sys] = clk_register_mux(NULL, "sys", sys_sels, ARRAY_SIZE(sys_sels),
				0, ccm_base + 0x4, 0, 1, 0, &d4400_ccm_lock);
	clk[arm] = clk_register_divider(NULL, "arm", "sys", CLK_SET_RATE_PARENT,
				ccm_base + 0x8, 0, 2, 0, &d4400_ccm_lock);
	clk[per] = clk_register_fixed_factor(NULL, "per", "arm",
				CLK_SET_RATE_PARENT, 1, 2);
	clk[vsp_clk] = clk_register_divider(NULL, "vsp_clk", "sys",
				CLK_SET_RATE_PARENT, ccm_base + 0x8,
				4, 2, 0, &d4400_ccm_lock);
	clk[ram] = clk_register_divider(NULL, "ram", "vsp_clk",
				CLK_SET_RATE_PARENT, ccm_base + 0x8, 8, 2,
				0, &d4400_ccm_lock);
	clk[sys_bus] = clk_register_fixed_factor(NULL, "sys_bus", "ram",
				CLK_SET_RATE_PARENT, 1, 2);
	clk[ahb] = clk_register_divider(NULL, "ahb", "sys_bus",
				CLK_SET_RATE_PARENT, ccm_base + 0x8, 12, 2,
				0, &d4400_ccm_lock);
	clk[ip] = clk_register_divider(NULL, "ip", "ahb",
				CLK_SET_RATE_PARENT, ccm_base + 0x8, 14, 2,
				0, &d4400_ccm_lock);
	clk[gpc] = clk_register_divider(NULL, "gpc", "ip",
				CLK_SET_RATE_PARENT, ccm_base + 0x8, 18, 2,
				0, &d4400_ccm_lock);
	clk[uart1_serial] = clk_register_mux(NULL, "uart1_serial", uart1_sels,
				ARRAY_SIZE(uart1_sels), 0, ccm_base + 0x0, 0, 1,
				0, &d4400_ccm_lock);
	clk[uart1_ipg] = d4400_clk_gate(NULL, "uart1_ipg", "ip",
					CLK_SET_RATE_PARENT, ccm_base + 0x30,
					20, 0, &d4400_ccm_lock);
	clk[uart1_pen] = clk_register_gate(NULL, "uart1_pen", "uart1_serial",
					CLK_SET_RATE_PARENT, ccm_base + 0x14,
					28, 0, &d4400_ccm_lock);
	clk[uart1_per] = clk_register_divider(NULL, "uart1_per", "uart1_pen",
				CLK_SET_RATE_PARENT, ccm_base + 0xC, 0, 4,
				0, &d4400_ccm_lock);
	clk[uart2_serial] = clk_register_mux(NULL, "uart2_serial", uart2_sels,
				ARRAY_SIZE(uart2_sels), 0, ccm_base + 0x0, 1,
				1, 0, &d4400_ccm_lock);
	clk[uart2_ipg] = d4400_clk_gate(NULL, "uart2_ipg", "ip",
					CLK_SET_RATE_PARENT, ccm_base + 0x30,
					22, 0, &d4400_ccm_lock);
	clk[uart2_pen] = clk_register_gate(NULL, "uart2_pen", "uart2_serial",
					CLK_SET_RATE_PARENT, ccm_base + 0x14,
					29, 0, &d4400_ccm_lock);
	clk[uart2_per] = clk_register_divider(NULL, "uart2_per", "uart2_pen",
				CLK_SET_RATE_PARENT, ccm_base + 0xC, 4, 4,
				0, &d4400_ccm_lock);
	clk[uart3_serial] = clk_register_mux(NULL, "uart3_serial", uart3_sels,
				ARRAY_SIZE(uart3_sels), 0, ccm_base + 0x0, 2, 1,
				0, &d4400_ccm_lock);
	clk[uart3_ipg] = d4400_clk_gate(NULL, "uart3_ipg", "ip",
					CLK_SET_RATE_PARENT, ccm_base + 0x30,
					24, 0, &d4400_ccm_lock);
	clk[uart3_pen] = clk_register_gate(NULL, "uart3_pen", "uart3_serial",
					CLK_SET_RATE_PARENT, ccm_base + 0x14,
					30, 0, &d4400_ccm_lock);
	clk[uart3_per] = clk_register_divider(NULL, "uart3_per", "uart3_pen",
				CLK_SET_RATE_PARENT, ccm_base + 0xC, 8, 4,
				0, &d4400_ccm_lock);
	clk[uart4_serial] = clk_register_mux(NULL, "uart4_serial", uart4_sels,
				ARRAY_SIZE(uart4_sels), 0, ccm_base + 0x0, 3, 1,
				0, &d4400_ccm_lock);
	clk[uart4_ipg] = d4400_clk_gate(NULL, "uart4_ipg", "ip",
					CLK_SET_RATE_PARENT, ccm_base + 0x34,
					0, 0, &d4400_ccm_lock);
	clk[uart4_pen] = clk_register_gate(NULL, "uart4_pen", "uart4_serial",
					CLK_SET_RATE_PARENT, ccm_base + 0x14,
					31, 0, &d4400_ccm_lock);
	clk[uart4_per] = clk_register_divider(NULL, "uart4_per", "uart4_pen",
				CLK_SET_RATE_PARENT, ccm_base + 0xC, 12, 4,
				0, &d4400_ccm_lock);
	clk[weim_sel] = clk_register_mux(NULL, "weim_sel", weim_sels,
				ARRAY_SIZE(weim_sels), 0, ccm_base + 0x0,
				31, 1, 0, &d4400_ccm_lock);
	clk[weim_clk] = clk_register_divider(NULL, "weim_clk", "weim_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x8, 20, 4,
				0, &d4400_ccm_lock);
	clk[div_pll_sys] = clk_register_divider(NULL, "div_pll_sys", "sys_pll",
				CLK_SET_RATE_PARENT, ccm_base + 0x8, 24, 4,
				0, &d4400_ccm_lock);
	clk[etsec_rtc] = clk_register_mux(NULL, "etsec_rtc", etsec_rtc_sels,
				ARRAY_SIZE(etsec_rtc_sels), 0, ccm_base + 0x0,
				27, 1, 0, &d4400_ccm_lock);
	clk[sync_ref_src_1] = clk_register_fixed_factor(NULL, "sync_ref_src_1", "ref",
				CLK_SET_RATE_PARENT, 1, 6);
	clk[sync_ref_src_2] = clk_register_fixed_factor(NULL, "sync_ref_src_2", "ref",
				CLK_SET_RATE_PARENT, 1, 3);
	clk[sync_ref] = clk_register_mux(NULL, "sync_ref", sync_ref_sels,
				ARRAY_SIZE(sync_ref_sels), 0, ccm_base + 0x4,
				0, 1, 0, &d4400_ccm_lock);
	clk[async_ckil] = clk_register_fixed_factor(NULL, "async_ckil",
				"osc1_dev", CLK_SET_RATE_PARENT, 1, 4096);
	clk[sync_ckil] = clk_register_fixed_factor(NULL, "sync_ckil",
				"osc1_dev", CLK_SET_RATE_PARENT, 1, 4096);
	clk[ecspi1_sel] = clk_register_mux(NULL, "ecspi1_sel", ecspi1_sels,
				ARRAY_SIZE(ecspi1_sels), 0, ccm_base + 0x0,
				8, 1, 0, &d4400_ccm_lock);
	clk[ecspi1_clk] = clk_register_divider(NULL, "ecspi1_clk", "ecspi1_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 0, 4,
				0, &d4400_ccm_lock);
	clk[ecspi2_sel] = clk_register_mux(NULL, "ecspi2_sel", ecspi2_sels,
				ARRAY_SIZE(ecspi2_sels), 0, ccm_base + 0x0,
				9, 1, 0, &d4400_ccm_lock);
	clk[ecspi2_clk] = clk_register_divider(NULL, "ecspi2_clk", "ecspi2_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 4, 4,
				0, &d4400_ccm_lock);
	clk[ecspi3_sel] = clk_register_mux(NULL, "ecspi3_sel", ecspi3_sels,
				ARRAY_SIZE(ecspi3_sels), 0, ccm_base + 0x0,
				10, 1, 0, &d4400_ccm_lock);
	clk[ecspi3_clk] = clk_register_divider(NULL, "ecspi3_clk", "ecspi3_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 8, 4,
				0, &d4400_ccm_lock);
	clk[ecspi4_sel] = clk_register_mux(NULL, "ecspi4_sel", ecspi4_sels,
				ARRAY_SIZE(ecspi4_sels), 0, ccm_base + 0x0,
				11, 1, 0, &d4400_ccm_lock);
	clk[ecspi4_clk] = clk_register_divider(NULL, "ecspi4_clk", "ecspi4_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 12, 4,
				0, &d4400_ccm_lock);
	clk[ecspi5_sel] = clk_register_mux(NULL, "ecspi5_sel", ecspi5_sels,
				ARRAY_SIZE(ecspi5_sels), 0, ccm_base + 0x0,
				12, 1, 0, &d4400_ccm_lock);
	clk[ecspi5_clk] = clk_register_divider(NULL, "ecspi5_clk", "ecspi5_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 16, 4,
				0, &d4400_ccm_lock);
	clk[ecspi6_sel] = clk_register_mux(NULL, "ecspi6_sel", ecspi6_sels,
				ARRAY_SIZE(ecspi6_sels), 0, ccm_base + 0x0,
				13, 1, 0, &d4400_ccm_lock);
	clk[ecspi6_clk] = clk_register_divider(NULL, "ecspi6_clk", "ecspi6_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 20, 4,
				0, &d4400_ccm_lock);
	clk[ecspi7_sel] = clk_register_mux(NULL, "ecspi7_sel", ecspi7_sels,
				ARRAY_SIZE(ecspi7_sels), 0, ccm_base + 0x0,
				14, 1, 0, &d4400_ccm_lock);
	clk[ecspi7_clk] = clk_register_divider(NULL, "ecspi7_clk", "ecspi7_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 24, 4,
				0, &d4400_ccm_lock);
	clk[ecspi8_sel] = clk_register_mux(NULL, "ecspi8_sel", ecspi8_sels,
				ARRAY_SIZE(ecspi8_sels), 0, ccm_base + 0x0,
				15, 1, 0, &d4400_ccm_lock);
	clk[ecspi8_clk] = clk_register_divider(NULL, "ecspi8_clk", "ecspi8_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x10, 28, 4,
				0, &d4400_ccm_lock);
	clk[ddr_pll_by] = clk_register_fixed_factor(NULL, "ddr_pll_by",
				"ddr_pll", CLK_SET_RATE_PARENT, 1, 2);
	clk[mmdc_sel] = clk_register_mux(NULL, "mmdc_sel", mmdc_sels,
				ARRAY_SIZE(mmdc_sels), 0, ccm_base + 0x0,
				16, 2, 0, &d4400_ccm_lock);
	clk[mmdc_clk] = clk_register_divider(NULL, "mmdc_clk", "mmdc_sel",
				CLK_SET_RATE_PARENT, ccm_base + 0x8, 28, 4,
				0, &d4400_ccm_lock);
	clk[trace_clk] = clk_register_mux(NULL, "trace_clk", trace_sels,
				ARRAY_SIZE(trace_sels), 0, ccm_base + 0x0,
				29, 2, 0, &d4400_ccm_lock);
	clk[ccm_at] = clk_register_divider(NULL, "ccm_at", "trace_clk",
				CLK_SET_RATE_PARENT, ccm_base + 0xC, 28, 4,
				0, &d4400_ccm_lock);
	clk[vspa_dp_sel] = clk_register_mux(NULL, "vspa_dp_sel", vspa_dp_sels,
				ARRAY_SIZE(vspa_dp_sels), 0, ccm_base + 0x0,
				18, 2, 0, &d4400_ccm_lock);
	clk[vspa_dp_clk] = clk_register_divider(NULL, "vspa_dp_clk",
				"vspa_dp_sel", CLK_SET_RATE_PARENT,
				ccm_base + 0x8, 16, 2, 0, &d4400_ccm_lock);
	clk[epit_ipg] = d4400_clk_gate(NULL, "epit_ipg", "ip",
					CLK_SET_RATE_PARENT,ccm_base + 0x30,
					18, 0, &d4400_ccm_lock);
	clk[epit_per] = d4400_clk_gate(NULL, "epit_per", "sync_ref",
					CLK_SET_RATE_PARENT, ccm_base + 0x4c,
					18, 0, &d4400_ccm_lock);
	clk[i2c1_per] = d4400_clk_gate(NULL, "i2c1_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					2, 0, &d4400_ccm_lock);
	clk[i2c2_per] = d4400_clk_gate(NULL, "i2c2_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					4, 0, &d4400_ccm_lock);
	clk[i2c3_per] = d4400_clk_gate(NULL, "i2c3_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					6, 0, &d4400_ccm_lock);
	clk[i2c4_per] = d4400_clk_gate(NULL, "i2c4_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					8, 0, &d4400_ccm_lock);
	clk[i2c5_per] = d4400_clk_gate(NULL, "i2c5_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					16, 0, &d4400_ccm_lock);
	clk[i2c6_per] = d4400_clk_gate(NULL, "i2c6_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					18, 0, &d4400_ccm_lock);
	clk[i2c7_per] = d4400_clk_gate(NULL, "i2c7_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					20, 0, &d4400_ccm_lock);
	clk[i2c8_per] = d4400_clk_gate(NULL, "i2c8_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					22, 0, &d4400_ccm_lock);
	clk[i2c9_per] = d4400_clk_gate(NULL, "i2c9_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x60,
					24, 0, &d4400_ccm_lock);
	clk[i2c10_per] = d4400_clk_gate(NULL, "i2c10_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x64,
					0, 0, &d4400_ccm_lock);
	clk[i2c11_per] = d4400_clk_gate(NULL, "i2c11_per", "sync_ref",
					CLK_SET_RATE_PARENT,ccm_base + 0x58,
					20, 0, &d4400_ccm_lock);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (IS_ERR(clk[i]))
			pr_err("D4400 clk %d: register failed with %ld\n",
				i, PTR_ERR(clk[i]));

	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);

	clk_register_clkdev(clk[epit_ipg], "ref", "d4400-epit");
	clk_register_clkdev(clk[epit_per], "sync_ref", "d4400-epit");

	np = of_find_compatible_node(NULL, NULL, "fsl,d4400-epit");
	base = of_iomap(np, 0);
	WARN_ON(!base);
	irq = irq_of_parse_and_map(np, 0);
	epit_timer_init(base, irq);
	return 0;
}


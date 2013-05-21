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
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "clk.h"
#include "common.h"

static void __iomem *ccm_base;

void __init d4400_clock_map_io(void) { }

enum d4400_clks {
};

static struct clk *clk[clk_max];
static struct clk_onecell_data clk_data;

static enum d4400_clks const clks_init_on[] __initconst = {
};

int __init d4400_clocks_init(void)
{
	struct device_node *np;
	void __iomem *base;
	int i, irq;

	np = of_find_compatible_node(NULL, NULL, "fsl,d4400-epit");
	base = of_iomap(np, 0);
	WARN_ON(!base);
	irq = irq_of_parse_and_map(np, 0);
	epit_timer_init(base, irq);
	return 0;
}


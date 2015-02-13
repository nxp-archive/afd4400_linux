/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/system_misc.h>
#include <mach/simreset.h>

#include "common.h"
#include "hardware.h"

#define D4400_SILICON_REVISION_REG	0x24

/* Reboot method: 0-reboot using hard reset, nonzero-simulated reset */
static int reboot_method;
int d4400_query_reboot_method(void)
{
	return reboot_method;
}
void d4400_set_reboot_method(int method)
{
	reboot_method = method;
}

static int d4400_revision(void)
{
	struct device_node *np;
	void __iomem *base;
	static u32 rev;

	if (!rev) {
		np = of_find_compatible_node(NULL, NULL, "fsl,d4400-iim");
		if (!np)
			return D4400_CHIP_REVISION_UNKNOWN;
		base = of_iomap(np, 0);
		if (!base) {
			of_node_put(np);
			return D4400_CHIP_REVISION_UNKNOWN;
		}
		rev =  readl_relaxed(base + D4400_SILICON_REVISION_REG);
		iounmap(base);
		of_node_put(np);
	}

	switch (rev & 0xff) {
	case 0x10:
		return D4400_CHIP_REVISION_1_0;
	default:
		return D4400_CHIP_REVISION_UNKNOWN;
	}
}

void d4400_restart(char mode, const char *cmd)
{
	if (d4400_query_reboot_method()) {
		/* Simulated reboot */
		pr_info("Executing simulated reset sequence.\n");
		sim_reset(); /* Never returns */
	} else {
		/* Hard reset using WDT */
		struct device_node *np;
		void __iomem *wdog_base;

		/* Clear simreset marker to notify bootup that this is NOT
		 * a simulated reset.
		 */
		clear_ram_simreset_marker();

		np = of_find_compatible_node(NULL, NULL, "fsl,d4400-wdt");
		if (!np) {
			pr_warn("Failed to find compatible DT node, using soft reset");
			goto soft;
		}

		wdog_base = of_iomap(np, 0);
		if (!wdog_base) {
			pr_warn("Unable to remap watchdog address, using soft reset");
			goto soft;
		}
		pr_info("Executing cpu reset sequence.\n");

		/* enable wdog */
		writew_relaxed(1 << 2, wdog_base);
		/* write twice to ensure the request will not get ignored */
		writew_relaxed(1 << 2, wdog_base);

		/* wait for reset to assert ... */
		mdelay(500);

		pr_err("Watchdog reset failed to assert reset\n");

		/* delay to allow the serial port to show the message */
		mdelay(50);

soft:
		/* we'll take a jump through zero as a poor second */
		soft_restart(0);
	}
}

static void __init d4400_init_late(void)
{
/*
 *FIXME - Add power management register function here
 */
	d4400_gpc_init();
	#if defined(D4400_GIT_LAST_COMMIT_HASH) && \
		defined(D4400_GIT_LAST_COMMIT_DATE)
	pr_err("KERNEL: git commit: Hash: %.7s; Date: %s)\n",
		D4400_GIT_LAST_COMMIT_HASH,
		D4400_GIT_LAST_COMMIT_DATE);

	pr_err("KERNEL: Built %s %s\n", __DATE__, __TIME__);
	#endif

	/* System initialization is ending, reset marker. */
	clear_simreset_marker();
	/* By default, reset method is cpu reset. */
	d4400_set_reboot_method(0);
}

static void __init d4400_map_io(void)
{
	d4400_lluart_map_io();
}

static const struct of_device_id d4400_irq_match[] __initconst = {
	{ .compatible = "arm,cortex-a9-gic", .data = gic_of_init, },
	{ /* sentinel */ }
};

static void __init d4400_init_irq(void)
{
	/*
	 * Reducing cache ways to 8
	 */
	l2x0_of_init(0, 0xFFFEFFFF);
	of_irq_init(d4400_irq_match);
	d4400_clock_map_io();
}

static void __init d4400_timer_init(void)
{
	d4400_scm_init();
	d4400_clocks_init();
	d4400_print_silicon_rev("D4400", d4400_revision());
}
static void __init d4400_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	check_ram_simreset_marker();
}

static struct sys_timer d4400_timer = {
	.init = d4400_timer_init,
};

static const char *d4400_dt_compat[] __initdata = {
	"fsl,d4400",
	NULL,
};

DT_MACHINE_START(D4400, "Freescale D4400 (Device Tree)")
	.map_io		= d4400_map_io,
	.init_irq	= d4400_init_irq,
	.handle_irq	= d4400_handle_irq,
	.timer		= &d4400_timer,
	.init_machine	= d4400_init_machine,
	.init_late      = d4400_init_late,
	.dt_compat	= d4400_dt_compat,
	.restart	= d4400_restart,
MACHINE_END


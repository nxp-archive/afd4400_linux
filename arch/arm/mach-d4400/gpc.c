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

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include "common.h"

#define GPC_PGC_PGCR_PCR_OFFSET	0x00
#define GPC_PGC_PGCR_PCR_MASK	BITS_MASK(1, GPC_PGC_PGCR_PCR_OFFSET)

/* controller offset addresses */
#define FULL_VSPA1_PGC_OFFSET	0x2E0
#define FULL_VSPA11_PGC_OFFSET 0x420

/* pgc_register addresses */
#define GPC_PGC_PGCR_REG_OFFSET 0x00
static void __iomem *gpc_base;

/* vspa_id max = 11, start ID = 1 */
int d4400_gpc_vspa_full_pow_gate(u8 vspa_id)
{
	u32 val;
	u32 offset = FULL_VSPA1_PGC_OFFSET + (vspa_id - 1) * 0x20;
	if (offset > FULL_VSPA11_PGC_OFFSET)
		return -EINVAL;
	val = readl(gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET);
	val |= GPC_PGC_PGCR_PCR_MASK;
	writel(val, (gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET));
	return d4400_ccm_vspa_full_pow_gate(vspa_id);
}

/* vspa_id max = 11 , start ID = 1 */
int d4400_gpc_vspa_full_pow_up(u8 vspa_id)
{
	u32 offset = FULL_VSPA1_PGC_OFFSET + (vspa_id - 1) * 0x20;
	if (offset > FULL_VSPA11_PGC_OFFSET)
		return -EINVAL;
	return d4400_ccm_vspa_full_pow_up(vspa_id);
}

void __init d4400_gpc_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,d4400-gpc");
	gpc_base = of_iomap(np, 0);
	WARN_ON(!gpc_base);
}

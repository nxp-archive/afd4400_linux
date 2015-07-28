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

#define NUM_VSPAS		11

#define GPC_PGC_PGCR_PCR_OFFSET	0x00
#define GPC_PGC_PGCR_PCR_MASK	BITS_MASK(1, GPC_PGC_PGCR_PCR_OFFSET)

/* controller offset addresses */
#define CORE_VSPA0_PGC_OFFSET	0x0A0
#define FULL_VSPA0_PGC_OFFSET	0x2E0

/* pgc_register addresses */
#define GPC_PGC_PGCR_REG_OFFSET   0x00
#define GPC_PGC_PUPSCR_REG_OFFSET 0x04
#define GPC_PGC_PDNSCR_REG_OFFSET 0x08
#define GPC_PGC_PGSR_REG_OFFSET   0x0C

static void __iomem *gpc_base = NULL;

/* Enables core power gating during VSPA IDLE */
/* vspa_id max = 10, start ID = 0 */
int d4400_gpc_vspa_core_pow_gate(u8 vspa_id)
{
	u32 val;
	u32 offset;

	if (vspa_id >= NUM_VSPAS)
		return -EINVAL;

	offset = CORE_VSPA0_PGC_OFFSET + vspa_id * 0x20;
	val = readl(gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET);
	val |= GPC_PGC_PGCR_PCR_MASK;
	writel(val, (gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET));
	return 0;
}

/* Disables core power gating during VSPA IDLE */
/* vspa_id max = 10 , start ID = 0 */
int d4400_gpc_vspa_core_pow_up(u8 vspa_id)
{
	u32 val;
	u32 offset;

	offset = CORE_VSPA0_PGC_OFFSET + vspa_id * 0x20;
	val = readl(gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET);
	val &= ~GPC_PGC_PGCR_PCR_MASK;
	writel(val, (gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET));
	return 0;
}

/* returns 1 if VSPA core power is always on (not gated) */
/* vspa_id max = 10 , start ID = 0 */
int d4400_gpc_vspa_core_pow(u8 vspa_id)
{
	u32 val;
	u32 offset;

	if (vspa_id >= NUM_VSPAS)
		return -EINVAL;

	offset = CORE_VSPA0_PGC_OFFSET + vspa_id * 0x20;
	val = readl(gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET);
	return (val & GPC_PGC_PGCR_PCR_MASK) ? 0 : 1;
}

/* vspa_id max = 10, start ID = 0 */
int d4400_gpc_vspa_full_pow_gate(u8 vspa_id)
{
	u32 val;
	u32 offset;

	if (vspa_id >= NUM_VSPAS)
		return -EINVAL;

	offset = FULL_VSPA0_PGC_OFFSET + vspa_id * 0x20;
	val = readl(gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET);
	val |= GPC_PGC_PGCR_PCR_MASK;
	writel(val, (gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET));
	return d4400_ccm_vspa_full_pow_gate(vspa_id);
}

/* vspa_id max = 10 , start ID = 0 */
int d4400_gpc_vspa_full_pow_up(u8 vspa_id)
{
	u32 val;
	u32 ret;
	u32 offset;

	if (vspa_id >= NUM_VSPAS)
		return -EINVAL;

	ret = d4400_ccm_vspa_full_pow_up(vspa_id);
	offset = FULL_VSPA0_PGC_OFFSET + vspa_id * 0x20;
	val = readl(gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET);
	val &= ~GPC_PGC_PGCR_PCR_MASK;
	writel(val, (gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET));
	return ret;
}

/* returns 1 if VSPA power is on */
/* vspa_id max = 10 , start ID = 0 */
int d4400_gpc_vspa_full_pow(u8 vspa_id)
{
	u32 val;
	u32 offset;

	if (vspa_id >= NUM_VSPAS)
		return -EINVAL;

	offset = FULL_VSPA0_PGC_OFFSET + vspa_id * 0x20;
	val = readl(gpc_base + offset + GPC_PGC_PGCR_REG_OFFSET);
	return (val & GPC_PGC_PGCR_PCR_MASK) ?
					d4400_ccm_vspa_full_pow(vspa_id) : 1;
}

void __init d4400_gpc_init(void)
{
	struct device_node *np;
	u32 val;
	u32 offset;
	int i;

	np = of_find_compatible_node(NULL, NULL, "fsl,d4400-gpc");
	gpc_base = of_iomap(np, 0);
	WARN_ON(!gpc_base);

	/* Setup VSPA Power up / down synchronization clocks */
	val = (0xF << 8) | (0xF << 0);
	for (i = 0; i < NUM_VSPAS; i++) {
		offset = FULL_VSPA0_PGC_OFFSET + i * 0x20;
		writel(val, (gpc_base + offset + GPC_PGC_PUPSCR_REG_OFFSET));
		writel(val, (gpc_base + offset + GPC_PGC_PDNSCR_REG_OFFSET));
	}
	val = (0x2 << 8) | (0x2 << 0);
	for (i = 0; i < NUM_VSPAS; i++) {
		offset = CORE_VSPA0_PGC_OFFSET + i * 0x20;
		writel(val, (gpc_base + offset + GPC_PGC_PUPSCR_REG_OFFSET));
		writel(val, (gpc_base + offset + GPC_PGC_PDNSCR_REG_OFFSET));
	}
}

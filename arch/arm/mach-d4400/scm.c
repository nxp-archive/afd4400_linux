/*
 * linux/arch/arm/mach-d4400/scm.c
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/gcr.h>
#include <linux/cpri.h>

#include "common.h"

static struct gcr_priv *priv;
u32 gcr_get_pll_parent(enum gcr_get_pll_parent_param_t gcr_param)
{
	struct gcr_reg_map *gcr;
	u32 val = 0;
	gcr = priv->gcr_reg;

	switch (gcr_param) {
	case GET_SYS_PLL_PARENT:
		val = readl(&gcr->gcr[0]);
		val &= SYS_PARENT_CLK_SRC;
		return val;
	case GET_DDR_PLL_PARENT:
		val = readl(&gcr->gcr[0]);
		val &= DDR_PARENT_CLK_SRC;
		return val;
	default:
		return -EBADRQC;
	}
}

u32 gcr_set_pll_parent(enum gcr_set_pll_parent_param_t gcr_param,
		enum parent_src_clk_t parent_src_clk)
{
	struct gcr_reg_map *gcr;
	u32 val = 0;
	gcr = priv->gcr_reg;

	switch (gcr_param) {
	case SET_SYS_PLL_PARENT:
		val = readl(&gcr->gcr[0]);
		if (parent_src_clk == PARENT_SRC_SGMIICLK) {
			val |= GCR0_PLL_SYS_DEV_CLK_MASK;
			writel(val, &gcr->gcr[0]);
			val |= GCR0_PLL_SYS_RGMII_CLK_MASK;
			writel(val, &gcr->gcr[0]);
		} else {
			val &= ~GCR0_PLL_SYS_RGMII_CLK_MASK;
			writel(val, &gcr->gcr[0]);
			val &= ~GCR0_PLL_SYS_DEV_CLK_MASK;
			writel(val, &gcr->gcr[0]);
		}
		break;
	case SET_DDR_PLL_PARENT:
		val = readl(&gcr->gcr[0]);
		if (parent_src_clk == PARENT_SRC_SGMIICLK) {
			val |= GCR0_PLL_DDR_DEV_CLK_MASK;
			writel(val, &gcr->gcr[0]);
			val |= GCR0_PLL_DDR_RGMII_CLK_MASK;
			writel(val, &gcr->gcr[0]);
		} else {
			val &= ~GCR0_PLL_DDR_RGMII_CLK_MASK;
			writel(val, &gcr->gcr[0]);
			val &= ~GCR0_PLL_DDR_DEV_CLK_MASK;
			writel(val, &gcr->gcr[0]);
		}
		break;
	default:
		return -EBADRQC;
	}
	return 0;
}

static inline void scm_reg_write(u32 data, u32 gcr_id)
{
	struct gcr_reg_map *gcr = NULL;
	gcr = priv->gcr_reg;
	if (gcr_id > 36) {
		/* 7 is added to skip GSR registers */
		gcr_id = gcr_id + CFG_NUM_GSR_REG;
	}
	writel(data, &gcr->gcr[gcr_id]);
	return;
}

static inline u32 scm_reg_read(u32 gcr_id)
{
	struct gcr_reg_map *gcr = NULL;
	u32 data = 0;

	gcr = priv->gcr_reg;
	if (gcr_id > 36) {
		/* 7 is added to skip GSR registers */
		gcr_id = gcr_id + CFG_NUM_GSR_REG;
	}
	data = readl(&gcr->gcr[gcr_id]);
	return data;
}

void gcr_set_cpri_line_rate(unsigned char cpri_id, enum cpri_link_rate linerate)
{
	struct gcr_reg_map *gcr;
	u32 value = 0;
	unsigned char gcr_id;
	char line_rate[8] = {0, 2, 4, 5, 8, 10, 16};
	u32 rate;

	if (cpri_id == 1)
		gcr_id = GCR4_CPRI_CTRL;
	else
		gcr_id = GCR6_CPRI_CTRL;

	gcr = priv->gcr_reg;
	rate = line_rate[linerate];

	value = scm_reg_read(gcr_id);
	value |= rate << 1;
	scm_reg_write(value, gcr_id);
	value = scm_reg_read(gcr_id);
}
EXPORT_SYMBOL_GPL(gcr_set_cpri_line_rate);

void gcr_linkrate_autoneg_reset(unsigned char cpri_id)
{
	struct gcr_reg_map *gcr;
	u32 value = 0;
	unsigned char gcr_id;

	if (cpri_id == 1)
		gcr_id = GCR4_CPRI_CTRL;
	else
		gcr_id = GCR6_CPRI_CTRL;

	gcr = priv->gcr_reg;
	value = scm_reg_read(gcr_id);
	value |= 1;
	scm_reg_write(value, gcr_id);
}
EXPORT_SYMBOL_GPL(gcr_linkrate_autoneg_reset);

static int jesd_conf_val_set1(unsigned char jesd_chan)
{
	int val = 0;
	switch (jesd_chan) {
	case 1:
		val = JESDTX_DMA_REQ(1);
		break;
	case 2:
		val = JESDTX_DMA_REQ(2);
		break;
	case 3:
		val = JESDTX_DMA_REQ(3);
		break;
	case 4:
		val = JESDTX_DMA_REQ(4);
		break;
	case 5:
		val = JESDTX_DMA_REQ(5);
		break;
	case 6:
		val = JESDTX_DMA_REQ(6);
		break;
	case 7:
		val = JESDTX_DMA_REQ(7);
		break;
	case 8:
		val = JESDTX_DMA_REQ(8);
		break;
	case 9:
		val = JESDTX_DMA_REQ(9);
		break;
	case 10:
		val = JESDTX_DMA_REQ(10);
		break;
	case 13:
		val = JESDRX_DMA_REQ(13);
		break;
	}

	return val;
}

static int jesd_conf_val_set2(unsigned char jesd_chan, unsigned char gcr_id)
{
	int val = 0;
	switch (jesd_chan) {

	case 1:
		val = JESDRX_DMA_REQ(1);
		break;
	case 2:
		val = JESDRX_DMA_REQ(2);
		break;
	case 3:
		val = JESDRX_DMA_REQ(3);
		break;
	case 4:
		val = JESDRX_DMA_REQ(4);
		break;
	case 5:
		if (gcr_id == 41)
			val = JESDRX_DMA_REQ(41_5);
		else
			val = JESDRX_DMA_REQ(5);
		break;
	case 6:
		if (gcr_id == 41)
			val = JESDRX_DMA_REQ(41_6);
		else
			val = JESDRX_DMA_REQ(6);
		break;
	case 7:
		if (gcr_id == 41)
			val = JESDRX_DMA_REQ(41_7);
		else
			val = JESDRX_DMA_REQ(7);
		break;
	case 8:
		if (gcr_id == 5)
			val = JESDRX_DMA_REQ(5_8);
		else
			val = JESDRX_DMA_REQ(8);
		break;
	case 9:
		if (gcr_id == 5)
			val = JESDTX_DMA_REQ(9);
		else
			val = JESDRX_DMA_REQ(9);
		break;
	case 10:
		if (gcr_id == 5)
			val = JESDTX_DMA_REQ(10);
		else
			val = JESDRX_DMA_REQ(10);
		break;
	case 11:
		val = JESDRX_DMA_REQ(11);
		break;
	case 12:
		val = JESDRX_DMA_REQ(12);
		break;
	}
	return val;
}

static int jesd_conf_val_set3(unsigned char jesd_chan)
{
	int val = 0;

	switch (jesd_chan) {

	case 1:
		val = JESDTX_DMA_REQ(1);
		break;
	case 2:
		val = JESDTX_DMA_REQ(2);
		break;
	case 3:
		val = JESDTX_DMA_REQ(3);
		break;
	case 4:
		val = JESDTX_DMA_REQ(4);
		break;
	case 5:
		val = JESDTX_DMA_REQ(5);
		break;
	case 6:
		val = JESDTX_DMA_REQ(6);
		break;
	case 7:
		val = JESDTX_DMA_REQ(7);
		break;
	case 8:
		val = JESDTX_DMA_REQ(8);
		break;
	}
	return val;
}

void gcr_jesd_init(void)
{
	u32 val;

	val = TPD_BGR_EN;
	scm_reg_write(val, 72);

	val = OHM_TERM_EN;
	scm_reg_write(val, 75);
	return;
}
EXPORT_SYMBOL_GPL(gcr_jesd_init);

static int cpri_conf_val_set1(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan)
{
	int val = 0;

	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX1_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(2JESD_18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(2JESD_19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(2JESD_21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(2JESD_22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(2JESD_23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(24);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX2_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX2_DMA_REQ(2JESD_18);
			break;
		case 19:
			val = CPRI_RX2_DMA_REQ(2JESD_19);
			break;
		case 20:
			val = CPRI_RX2_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX2_DMA_REQ(2JESD_21);
			break;
		case 22:
			val = CPRI_RX2_DMA_REQ(2JESD_22);
			break;
		case 23:
			val = CPRI_RX2_DMA_REQ(2JESD_23);
			break;
		case 24:
			val = CPRI_RX2_DMA_REQ(24);
			break;
		}
	}
	return val;
}

static int cpri_conf_val_set2(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan, unsigned char gcr_id)
{
	int val = 0;
	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 19:
			val = CPRI_RX1_DMA_REQ(2JESD_19);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(2JESD_22);
			break;
		case 18:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_18);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_18);
			break;
		case 23:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_23);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_23);
			break;
		case 17:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_17);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_17);
			break;
		case 24:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_24);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_24);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(2JESD_21);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 19:
			val = CPRI_RX2_DMA_REQ(2JESD_19);
			break;
		case 22:
			val = CPRI_RX2_DMA_REQ(2JESD_22);
			break;
		case 18:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_18);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_18);
			break;
		case 23:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_23);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_23);
			break;
		case 17:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_17);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_17);
			break;
		case 24:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_24);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_24);
			break;

		case 20:
			val = CPRI_RX2_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX2_DMA_REQ(2JESD_21);
			break;
		}
	}
	return val;
}

static int cpri_conf_val_set3(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan)
{
	int val = 0;

	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 1:
			val = CPRI_RX1_DMA_REQ(TX_1);
			break;
		case 2:
			val = CPRI_RX1_DMA_REQ(TX_2);
			break;
		case 3:
			val = CPRI_RX1_DMA_REQ(TX_3);
			break;
		case 4:
			val = CPRI_RX1_DMA_REQ(TX_4);
			break;
		case 5:
			val = CPRI_RX1_DMA_REQ(TX_5);
			break;
		case 6:
			val = CPRI_RX1_DMA_REQ(TX_6);
			break;
		case 7:
			val = CPRI_RX1_DMA_REQ(TX_7);
			break;
		case 8:
			val = CPRI_RX1_DMA_REQ(TX_8);
			break;
		case 9:
			val = CPRI_RX1_DMA_REQ(TX_9);
			break;
		case 11:
			val = CPRI_RX1_DMA_REQ(TX_11);
			break;
		case 12:
			val = CPRI_RX1_DMA_REQ(TX_12);
			break;
		case 13:
			val = CPRI_RX1_DMA_REQ(TX_13);
			break;
		case 14:
			val = CPRI_RX1_DMA_REQ(TX_14);
			break;
		case 15:
			val = CPRI_RX1_DMA_REQ(TX_15);
			break;
		case 16:
			val = CPRI_RX1_DMA_REQ(TX_16);
			break;
		case 17:
			val = CPRI_RX1_DMA_REQ(TX_17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(TX_18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(TX_19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(TX_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(TX_21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(TX_22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(TX_23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(TX_24);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 1:
			val = CPRI_RX1_DMA_REQ(TX_1);
			break;
		case 2:
			val = CPRI_RX1_DMA_REQ(TX_2);
			break;
		case 3:
			val = CPRI_RX1_DMA_REQ(TX_3);
			break;
		case 4:
			val = CPRI_RX1_DMA_REQ(TX_4);
			break;
		case 5:
			val = CPRI_RX1_DMA_REQ(TX_5);
			break;
		case 6:
			val = CPRI_RX1_DMA_REQ(TX_6);
			break;
		case 7:
			val = CPRI_RX1_DMA_REQ(TX_7);
			break;
		case 8:
			val = CPRI_RX1_DMA_REQ(TX_8);
			break;
		case 9:
			val = CPRI_RX1_DMA_REQ(TX_9);
			break;
		case 11:
			val = CPRI_RX1_DMA_REQ(TX_11);
			break;
		case 12:
			val = CPRI_RX1_DMA_REQ(TX_12);
			break;
		case 13:
			val = CPRI_RX1_DMA_REQ(TX_13);
			break;
		case 14:
			val = CPRI_RX1_DMA_REQ(TX_14);
			break;
		case 15:
			val = CPRI_RX1_DMA_REQ(TX_15);
			break;
		case 16:
			val = CPRI_RX1_DMA_REQ(TX_16);
			break;
		case 17:
			val = CPRI_RX1_DMA_REQ(TX_17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(TX_18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(TX_19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(TX_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(TX_21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(TX_22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(TX_23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(TX_24);
			break;
		}
	}
	return val;
}
static int cpri_conf_val_set4(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan)
{
	int val = 0;

	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX1_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(24);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX2_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX2_DMA_REQ(18);
			break;
		case 19:
			val = CPRI_RX2_DMA_REQ(19);
			break;
		case 20:
			val = CPRI_RX2_DMA_REQ(20);
			break;
		case 21:
			val = CPRI_RX2_DMA_REQ(21);
			break;
		case 22:
			val = CPRI_RX2_DMA_REQ(22);
			break;
		case 23:
			val = CPRI_RX2_DMA_REQ(23);
			break;
		case 24:
			val = CPRI_RX2_DMA_REQ(24);
			break;
		}
	}
	return val;
}

static inline int get_tx_module(enum vsp_id_t vsp)
{
	if (((vsp >= M2VSP_MIN) && (vsp <= M2VSP_MAX)))
		return 2;
	else if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
		return 1;
	return -EINVAL;
}

static u32 vsp_intf_gcr_reg_cfg(unsigned char gcr_id,
		struct dma_intf_switch_parm_t *chan_parm)
{
	unsigned int ret = 0;
	int val = 0;
	enum vsp_id_t vsp = chan_parm->vsp_id;
	enum cpri_core_info cpri_framert_id = chan_parm->cpri_framert_id;
	unsigned char cpri_chan = chan_parm->dma_request_id;
	enum dma_channel_id_t chan_id = chan_parm->chan_id;
	enum dma_req_dev_type_t dev_type = chan_parm->dev_type;
	unsigned char jesd_chan = chan_parm->dma_request_id;
	u32 reg;

	reg = scm_reg_read(gcr_id);
	if ((gcr_id >= 46) && (gcr_id <= 50)) {
		if (dev_type == DEV_CPRI)
			val = cpri_conf_val_set1(cpri_framert_id, cpri_chan);
		else
			val = jesd_conf_val_set1(jesd_chan);
		if (!val)
			return 0;
		/* to check even and odd value of chan id */
		if ((chan_id & 0x1) && (get_tx_module(vsp) == 1)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << ((vsp - 1) * 3 + 20));
			ret = (reg | (val << ((vsp - 1) * 3 + 20)));
		} else if ((chan_id & 0x1) && (get_tx_module(vsp) == 2)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << ((vsp-M2VSP_MIN-1) * 3 + 20));
			ret = (reg | (val<<((vsp-M2VSP_MIN - 1) * 3 + 20)));
		} else if (~(chan_id & 0x1) && (get_tx_module(vsp) == 1)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << ((vsp - 1) * 3));
			ret = (reg | (val << ((vsp - 1) * 3)));
		} else if (~(chan_id & 0x1) && (get_tx_module(vsp) == 2)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << (((vsp - M2VSP_MIN) - 1) * 3));
			ret = (reg | (val << (((vsp - M2VSP_MIN) - 1) *	3)));
		}
	} else if ((gcr_id == 41) || (gcr_id == 42) || (gcr_id == 5)) {
		if (dev_type == DEV_CPRI)
			val = cpri_conf_val_set2(cpri_framert_id, cpri_chan,
					gcr_id);
		else
			val = jesd_conf_val_set2(jesd_chan, gcr_id);
		if (!val)
			return 0;
		switch (chan_id) {
		case 10:
		case 13:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << (((vsp - 5) * 3) + 20)));
			ret = (reg | (val << (((vsp - 5) * 3) + 20)));
			break;
		case 9:
		case 12:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << (((vsp - 5) * 3) + 10)));
			ret = (reg | (val << (((vsp - 5) * 3) + 10)));
			break;
		case 8:
		case 11:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << ((vsp - 5) * 3)));
			ret = (reg | (val << (((vsp - 5) * 3))));
			break;
		case 14:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << ((vsp - 5) * 3)));
			ret = (reg | (val << (((vsp - 5) * 3))));
			break;
		case 15:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << (((vsp - 5) * 3) + 10)));
			ret = (reg | (val << (((vsp - 5) * 3) + 10)));
			break;
		default:
			ret = 0;
			break;
		}
	} else if ((gcr_id >= 35) && (gcr_id <= 40)) {
		if (dev_type == DEV_CPRI)
			val = cpri_conf_val_set3(cpri_framert_id, cpri_chan);
		if (!val)
			return 0;
		/* reg mask to clear the bits for value */
		reg &= (~(GCR_MSK << (((vsp - 5) * 3) + (((chan_id - 16)
							 % 3) * 10))));
		ret = (reg | (val << (((vsp - 5) * 3) +	(((chan_id - 16) % 3)
						 * 10))));

	} else {
		if (dev_type == DEV_CPRI) {
			val = cpri_conf_val_set4(cpri_framert_id, cpri_chan);

		} else { /* device type is JESD */
			val = jesd_conf_val_set3(jesd_chan);
		}
		if (!val)
			return 0;

		if (chan_id & 0x1) {/* to check even and odd value of chan id */
			if (vsp < M2VSP_MIN) {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << ((vsp + 3) * 3)));
				ret = (reg | (val << ((vsp + 3) * 3)));
			} else {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << (((vsp - M2VSP_MIN)
							     + 3) * 3)));
				ret = (reg | (val << (((vsp - M2VSP_MIN)
								+ 3) * 3)));
			}
		} else {
			if (vsp < M2VSP_MIN) {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << (((vsp + 3) * 3) - 12)));
				ret = (reg | (val << (((vsp + 3) * 3) - 12)));
			} else {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << ((((vsp - M2VSP_MIN)
							   + 3) * 3) - 12)));
				ret = (reg  | (val <<
							((((vsp - M2VSP_MIN)
							   + 3) * 3) - 12)));
			}
		}
	}
	return ret;
}

static u32 vsp_intf_gcr_reg_cfg_2bit(unsigned char gcr_id,
		struct dma_intf_switch_parm_t *chan_parm)
{
	enum cpri_core_info cpri_framert_id = chan_parm->cpri_framert_id;
	unsigned char cpri_chan = chan_parm->dma_request_id;
	enum dma_channel_id_t chan_id = chan_parm->chan_id;
	enum vsp_id_t vsp_id = chan_parm->vsp_id;
	u8 mask_bits = 0x00;
	u8 shift_mask_value = 0x00;
	int val = 0;
	u32 reg;

	reg = scm_reg_read(gcr_id);

	/* for RX */
	if ((vsp_id >= M1VSP_MIN && vsp_id <= M1VSP_MAX) ||
			(vsp_id >= M2VSP_MIN && vsp_id <= M2VSP_MAX)) {
		mask_bits = GCR_MSK1;
		shift_mask_value = (chan_id - 16) * 2;

		if (cpri_framert_id == CPRI_FRAMER_1) {
			if (cpri_chan >= 1 && cpri_chan <= 16)
				val = CPRI_RX1_DMA_REQ_2BIT(1);
			else
				val = CPRI_RX1_DMA_REQ_2BIT(2);
		} else if (cpri_framert_id == CPRI_FRAMER_2) {
			if (cpri_chan >= 1 && cpri_chan <= 16)
				val = CPRI_RX2_DMA_REQ_2BIT(1);
			else
				val = CPRI_RX2_DMA_REQ_2BIT(2);
		}
	} else {
		/* for TX*/
		mask_bits = GCR_MSK;

		if (cpri_framert_id == CPRI_FRAMER_1) {
			if (cpri_chan >= 1 && cpri_chan <= 16)
				val = CPRI_TX1_DMA_REQ_2BIT(1);
			else
				val = CPRI_TX1_DMA_REQ_2BIT(2);
		} else if (cpri_framert_id == CPRI_FRAMER_2) {
			if (cpri_chan >= 1 && cpri_chan <= 16)
				val = CPRI_TX2_DMA_REQ_2BIT(1);
			else
				val = CPRI_TX2_DMA_REQ_2BIT(2);
		}
		/* calculate shift_mask_value */
		switch (chan_id) {
		case DMA_CHAN_18:
		case DMA_CHAN_21:
		case DMA_CHAN_24:
		case DMA_CHAN_27:
		case DMA_CHAN_30:
			shift_mask_value = (vsp_id * 4) - (vsp_id - 5);
			break;
		case DMA_CHAN_17:
		case DMA_CHAN_20:
		case DMA_CHAN_23:
		case DMA_CHAN_26:
		case DMA_CHAN_29:
			shift_mask_value = (vsp_id * 3) - 5;
			break;
		case DMA_CHAN_16:
		case DMA_CHAN_19:
		case DMA_CHAN_22:
		case DMA_CHAN_25:
		case DMA_CHAN_28:
		case DMA_CHAN_31:
			shift_mask_value = (vsp_id - 5) * 3;
			break;
		default:
			return 0;
		}
	}

	return ((reg & (~(mask_bits << shift_mask_value)))
			| (val << shift_mask_value));
}

static unsigned int get_up_link_gcr_reg(enum vsp_id_t vsp,
					enum dma_comm_type_t chan)
{
	unsigned char gcr_id = 0; /*invalid id*/

		if ((vsp > 7) || (vsp < 5))
			goto out;
		if (chan < 16) {
			if ((chan >= 8) && (chan <= 10))
				gcr_id = 41;
			else if ((chan >= 11) && (chan <= 13))
				gcr_id = 42;
			else if ((chan >= 14) && (chan <= 15))
				gcr_id = 5;
		} else if (chan >= 16) {
			if ((chan >= 16) && (chan <= 18))
				gcr_id = 35;
			else if ((chan >= 19) && (chan <= 21))
				gcr_id = 36;
			else if ((chan >= 22) && (chan <= 24))
				gcr_id = 37;
			else if ((chan >= 25) && (chan <= 27))
				gcr_id = 38;
			else if ((chan >= 28) && (chan <= 30))
				gcr_id = 39;
			else if (chan == 31)
				gcr_id = 40;
		}
out:
	return gcr_id;
}

static unsigned char get_down_link_gcr_reg(enum vsp_id_t vsp,
		enum dma_channel_id_t chan)
{
	unsigned char gcr_id = 0; /*invalid id*/

	if (chan < 16) {
		if ((chan == 8) || (chan == 9)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 22;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 2;
		} else if ((chan == 10) || (chan == 11)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 23;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 3;
		} else if ((chan == 12) || (chan == 13)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 46;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 49;
		} else if ((chan == 14) || (chan == 15)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 47;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 50;

		}

	} else if (chan >= 16) {

		if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
			gcr_id = GCR_OFFSET_56 + (vsp-M1VSP_MIN);
		else if ((vsp >= M2VSP_MIN) &&
				(vsp <= M2VSP_MAX))
			gcr_id = GCR_OFFSET_60 + (vsp-M2VSP_MIN);
	}
	return gcr_id;
}


int gcr_vsp_intf_dma_cfg(struct dma_intf_switch_parm_t *chan_parm,
		unsigned char count)
{
	unsigned int tmp = 0;
	unsigned char gcr_id = 0; /*invalid id*/
	u32 value = 0;
	enum dma_channel_id_t chan;
	enum dma_comm_type_t cmd;
	enum vsp_id_t vsp;

	for (tmp = 0; tmp < count; tmp++) {

		chan = (chan_parm + tmp)->chan_id;
		cmd = (chan_parm + tmp)->comm_type;
		vsp = (chan_parm + tmp)->vsp_id;
		switch (cmd) {
		case SETUP_DL:
			gcr_id = get_down_link_gcr_reg(vsp, chan);
			break;
		case SETUP_UL:
			gcr_id = get_up_link_gcr_reg(vsp, chan);
			break;
		}
		if (gcr_id != 0) {
			if (chan < 16)
				value = vsp_intf_gcr_reg_cfg(gcr_id,
						(chan_parm + tmp));
			else
				value = vsp_intf_gcr_reg_cfg_2bit(gcr_id,
						(chan_parm + tmp));
			if (value != 0)
				scm_reg_write(value, gcr_id);
			else
				return -EINVAL;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}


int gcr_cpri_dma_mux(struct cpri_dma_mux_config *cpri_mux_parm,
		unsigned count)
{
	int tmp = 0;
	unsigned char gcr_id = 0; /*invalid id*/
	unsigned char cpri_dma_out = 0;
	unsigned char cpri_core_id = 0;
	u32 value = 0;
	enum cpri_rxtx_id rxtx_id;

	for (tmp = 0; tmp < count; tmp++) {
		rxtx_id = (cpri_mux_parm + tmp)->rxtx_id;
		switch (rxtx_id) {
		case CPRI_RX1:
			gcr_id = 20;
			break;
		case CPRI_RX2:
			gcr_id = 21;
			break;
		case CPRI_TX1:
			gcr_id = 29;
			break;
		case CPRI_TX2:
			gcr_id = 30;
			break;


		}
		value = scm_reg_read(gcr_id);
		cpri_dma_out = (cpri_mux_parm + tmp)->cpri_dma_req_out;
		cpri_core_id = (cpri_mux_parm + tmp)->src_cpri_complex;

		if (cpri_core_id == CPRI_CORE_CMPLX_1) {
			value = ((value & (~(GCR_MSK2<<cpri_dma_out)))|
					((((rxtx_id == CPRI_TX1) ||
					(rxtx_id == CPRI_RX1))
					? 0 : 1)<<cpri_dma_out));
		} else {
			value = ((value & (~(GCR_MSK2<<cpri_dma_out)))|
					((((rxtx_id == CPRI_TX1) ||
					(rxtx_id == CPRI_RX1))
					? 1 : 0)<<cpri_dma_out));
		}
		scm_reg_write(value, gcr_id);
	}
	return 0;
}


static inline int ivsp_gcr_reg_cfg(unsigned int reg_val,
			enum dma_channel_id_t chan,
			enum vsp_id_t vsp1, enum vsp_id_t vsp2)
{
	if ((vsp1 % 2) == 0)
		return ((reg_val&(~(GCR_MSK3 << (chan * 4)))) | ((vsp2 - 1) <<
					(chan*4)));
	else
		return ((reg_val&(~(GCR_MSK3 << ((chan-4) * 4))))|
				((vsp2 - 1) << ((chan - 4) * 4)));
}

static inline int ivsp_parm_validate(enum vsp_id_t vsp1, enum vsp_id_t vsp2,
		enum dma_channel_id_t chan)
{
	return ((vsp1 == vsp2) | ((chan < INTER_VSP_CHAN_MIN) |
				(chan > INTER_VSP_CHAN_MAX)));
}


static inline int ivsp_get_gcr_id(enum vsp_id_t src_vsp)
{
	switch (src_vsp) {
	case 1:
	case 2:
		return 8;
	case 3:
	case 4:
		return 9;
	case 5:
	case 6:
		return 10;
	case 7:
	case 8:
		return 11;
	case 9:
	case 10:
		return 12;
	case 11:
		return 13;
	default:
		return 0;
	}
	return 0;
}


int gcr_inter_vsp_dma_cfg(struct inter_vsp_dma_config_t *vsp_parm,
		unsigned char count)
{
	int tmp = 0;
	int ret = 1;
	u32 val = 0;
	unsigned char gcr_id = 0; /*invalid id*/

	for (tmp = 0; tmp < count; tmp++) {
		ret = ivsp_parm_validate((vsp_parm + tmp)->src_vsp_id,
				(vsp_parm + tmp)->dst_vsp_id,
				(vsp_parm + tmp)->chan_id);
		if (ret)
			return -EINVAL;

		gcr_id = ivsp_get_gcr_id((vsp_parm + tmp)->src_vsp_id);
		ret = 0;
		ret = scm_reg_read(gcr_id);
		val = ivsp_gcr_reg_cfg(ret,
			 (vsp_parm + tmp)->chan_id,
			 (vsp_parm + tmp)->src_vsp_id,
			 (vsp_parm + tmp)->dst_vsp_id);

		scm_reg_write(val, gcr_id);
	}
	return 0;
}

int gcr_jesd_dma_ptr_rst_req(struct jesd_dma_ptr_rst_parm *ptr_rst_parm,
		unsigned count)
{
	return 0;
}

void gcr_write_set(struct gcr_ctl_parm *param,
		unsigned char count)
{
	unsigned int m_addr;
	int tmp = 0;
	u32 reg = 0;

	for (tmp = 0; tmp < count; tmp++) {
		m_addr = (param + tmp)->reg_offset;
		reg = (param+tmp)->param;
		scm_reg_write(reg, m_addr);
	}
	return;
}

u32 gcr_read_set(u32 gcr_id)
{
	u32 data = 0;
	data = scm_reg_read(gcr_id);
	return data;
}

void *get_scm_priv(void)
{
	return (void *)priv;
}

int __init d4400_scm_init(void)
{
	int ret = 0;
	struct device_node *of_np;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		ret = -ENOMEM;

	of_np = of_find_compatible_node(NULL, NULL, "fsl,d4400-scm");
	priv->gcr_reg = of_iomap(of_np, 0);
	WARN_ON(!priv->gcr_reg);
	if (!priv->gcr_reg)
		goto err_out;

	return ret;
err_out:
	kfree(priv);
	priv = NULL;
	return ret;
}

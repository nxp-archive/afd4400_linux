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

void gcr_config_cpri_line_rate(unsigned char cpri_id,
		enum cpri_link_rate linerate, unsigned char cmd)
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
	value = scm_reg_read(gcr_id);
	if (cmd == SET_LINE_RATE) {
		rate = line_rate[linerate];
		value |= rate << 1;
	} else if (cmd == CLEAR_LINE_RATE)
		value &= CPRI_PHY_LINK_RESET;

	scm_reg_write(value, gcr_id);
}
EXPORT_SYMBOL_GPL(gcr_config_cpri_line_rate);

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

void gcr_sync_update(u32 mask, u32 val)
{
	struct gcr_reg_map *gcr;
	u32 value = 0;
	unsigned char gcr_id;

	gcr_id = GCR72_SYNC_CTRL;
	gcr = priv->gcr_reg;
	value = scm_reg_read(gcr_id);
	value &= ~mask;
	value |= val;
	scm_reg_write(value, gcr_id);
}
EXPORT_SYMBOL_GPL(gcr_sync_update);


void gcr_jesd_init(void)
{
	u32 val;

	val = TPD_BGR_EN;
	scm_reg_write(val, 72);
	val = TBGEN_E0_E1_EN;
	scm_reg_write(val, 7);

	return;
}
EXPORT_SYMBOL_GPL(gcr_jesd_init);

static u32 cpri_conf_val(enum dma_channel_id_t chan_id, u8 cpri_chan,
			enum cpri_core_info cpri_framert_id,
			enum dma_comm_type_t comm_type)
{
	u32 val = 0;

	if (comm_type == SETUP_DL) {
		if ((chan_id >= DMA_CHAN_8 && chan_id <= DMA_CHAN_11) ||
					(chan_id == DMA_CHAN_15)) {
			if (cpri_chan >= 17 && cpri_chan <= 20) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_RX1_DMA_REQ_2;
				else
					val = CPRI_RX2_DMA_REQ_4;
			} else if (cpri_chan >= 21 && cpri_chan <= 24) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_RX1_DMA_REQ_3;
				else
					val = CPRI_RX2_DMA_REQ_5;
			}
		} else if (chan_id >= DMA_CHAN_12 && chan_id <= DMA_CHAN_14) {
			if (cpri_chan >= 18 && cpri_chan <= 20) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_RX1_DMA_REQ_3;
				else
					val = CPRI_RX2_DMA_REQ_5;
			} else if (cpri_chan >= 21 && cpri_chan <= 23) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_RX1_DMA_REQ_4;
				else
					val = CPRI_RX2_DMA_REQ_6;
			}
		}
	} else if (comm_type == SETUP_UL) {
		if (chan_id >= DMA_CHAN_8 && chan_id <= DMA_CHAN_13) {
			if (cpri_chan >= 17 && cpri_chan <= 20) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_TX1_DMA_REQ_3;
				else
					val = CPRI_TX2_DMA_REQ_5;
			} else if (cpri_chan >= 21 && cpri_chan <= 24) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_TX1_DMA_REQ_4;
				else
					val = CPRI_TX2_DMA_REQ_6;
			}
		} else if (chan_id >= DMA_CHAN_14 && chan_id <= DMA_CHAN_15) {
			if (cpri_chan >= 17 && cpri_chan <= 18) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_TX1_DMA_REQ_4;
				else
					val = CPRI_TX2_DMA_REQ_6;
			} else if (cpri_chan >= 23 && cpri_chan <= 24) {
				if (cpri_framert_id == CPRI_FRAMER_1)
					val = CPRI_TX1_DMA_REQ_5;
				else
					val = CPRI_TX2_DMA_REQ_7;
			}
		}
	}
	return val;

}

static u32 jesd_conf_val(u8 jesd_chan, u8 gcr_id,
		enum dma_comm_type_t comm_type)
{
	u32 val = 0;

	if (comm_type == SETUP_DL) {
		if (jesd_chan >= 1 && jesd_chan <= 4)
			val = JESD_RX_DMA_REQ_1;
		else if (jesd_chan == 9 || jesd_chan == 10 ||  jesd_chan == 13)
			val = JESD_RX_DMA_REQ_2;
		else if (jesd_chan == 11 || jesd_chan == 12)
			val = JESD_RX_DMA_REQ_3;
		else if (jesd_chan >= 5 && jesd_chan <= 7) {
			if (gcr_id == 41)
				val = JESD_RX_DMA_REQ_2;
			else
				val = JESD_RX_DMA_REQ_1;
		} else if (jesd_chan == 8) {
			if (gcr_id == 42)
				val = JESD_RX_DMA_REQ_2;
			else
				val = JESD_RX_DMA_REQ_1;
		}
	} else if (comm_type == SETUP_UL) {
		if (jesd_chan >= 1 && jesd_chan <= 8)
			val = JESD_TX_DMA_REQ_1;
		else if (jesd_chan == 9 || jesd_chan == 10)
			val = JESD_TX_DMA_REQ_2;
	}
	return val;
}


static u32 vsp_intf_gcr_reg_cfg(unsigned char gcr_id,
		struct dma_intf_switch_parm_t *chan_parm)
{
	u32 val = 0;
	enum vsp_id_t vsp = chan_parm->vsp_id;
	enum cpri_core_info cpri_framert_id = chan_parm->cpri_framert_id;
	unsigned char dma_request_chan = chan_parm->dma_request_id;
	enum dma_channel_id_t vsp_chan_id = chan_parm->chan_id;
	enum dma_req_dev_type_t dev_type = chan_parm->dev_type;
	enum dma_comm_type_t comm_type = chan_parm->comm_type;
	u32 reg_val = 0;
	u8 shift_mask_value = 0x00;

	reg_val = scm_reg_read(gcr_id);

	if (dev_type == DEV_CPRI) {
		val = cpri_conf_val(vsp_chan_id, dma_request_chan,
				cpri_framert_id, comm_type);
	} else if (dev_type == DEV_JESD) {
		val = jesd_conf_val(dma_request_chan, gcr_id, comm_type);
	}
	/* get bit value needed to shift mask and data to be set */
	if (gcr_id == 22) {
		if (vsp_chan_id == DMA_CHAN_9)
			shift_mask_value = (3 * vsp + vsp_chan_id);
		else if (vsp_chan_id == DMA_CHAN_8)
			shift_mask_value = ((vsp - 1) * 3);
	} else if (gcr_id == 23) {
		if (vsp_chan_id == DMA_CHAN_10)
			shift_mask_value = ((vsp - 1) * 3);
		else if (vsp_chan_id == DMA_CHAN_11)
			shift_mask_value = (3 * vsp + (vsp_chan_id - 2));
	} else if (gcr_id == 46) {
		if (vsp_chan_id == DMA_CHAN_12)
			shift_mask_value = ((vsp - 1) * 3);
		else if (vsp_chan_id == DMA_CHAN_13)
			shift_mask_value = ((vsp - 1) * 3 + 20);
	} else if (gcr_id == 47) {
		if (vsp_chan_id == DMA_CHAN_14)
			shift_mask_value = ((vsp - 1) * 3);
		else if (vsp_chan_id == DMA_CHAN_15)
			shift_mask_value = ((vsp - 1) * 3 + 20);
	} else if (gcr_id == 49) {
		if (vsp_chan_id == DMA_CHAN_12)
			shift_mask_value = ((vsp - 8) * 3);
		else if (vsp_chan_id == DMA_CHAN_13)
			shift_mask_value = ((vsp - 8) * 3 + 20);
	} else if (gcr_id == 50) {
		if (vsp_chan_id == DMA_CHAN_14)
			shift_mask_value = ((vsp - 8) * 3);
		else if (vsp_chan_id == DMA_CHAN_15)
			shift_mask_value = ((vsp - 8) * 3 + 20);
	} else if (gcr_id == 2) {
		if (vsp_chan_id == DMA_CHAN_8)
			shift_mask_value = ((vsp - vsp_chan_id) * 3);
		else if (vsp_chan_id == DMA_CHAN_9)
			shift_mask_value = ((vsp + 3) * 3 - 21);
	} else if (gcr_id == 3) {
		if (vsp_chan_id == DMA_CHAN_10)
			shift_mask_value = ((vsp - (vsp_chan_id - 2)) * 3);
		else if (vsp_chan_id == DMA_CHAN_11)
			shift_mask_value = ((vsp + 3) * 3 - 21);
	} else if (gcr_id == 41) {
		if (vsp_chan_id == DMA_CHAN_8)
			shift_mask_value = ((vsp - 5) * 3);
		else if (vsp_chan_id == DMA_CHAN_9)
			shift_mask_value = ((3 * vsp) - 5);
		else if (vsp_chan_id == DMA_CHAN_10)
			shift_mask_value = ((3 * vsp) + 5);
	} else if (gcr_id == 42) {
		if (vsp_chan_id == DMA_CHAN_11)
			shift_mask_value = ((vsp - 5) * 3);
		else if (vsp_chan_id == DMA_CHAN_12)
			shift_mask_value = ((3 * vsp) - 5);
		else if (vsp_chan_id == DMA_CHAN_13)
			shift_mask_value = ((3 * vsp) + 5);
	} else if (gcr_id == 5) {
		if (vsp_chan_id == DMA_CHAN_14)
			shift_mask_value = ((vsp - 5) * 3);
		else if (vsp_chan_id == DMA_CHAN_15)
			shift_mask_value = ((3 * vsp) - 5);
	}


	return ((reg_val & (~(GCR_MSK << shift_mask_value)))
			| (val << shift_mask_value));
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

static unsigned int get_gcr_reg(enum vsp_id_t vsp,
					enum dma_channel_id_t chan)
{
	unsigned char gcr_id = 0; /*invalid id*/

	if (vsp >= M1VSP_MIN && vsp <= M1VSP_MAX) {
		if ((chan >= 8) && (chan <= 9))
			gcr_id = 22;
		if ((chan >= 10) && (chan <= 11))
			gcr_id = 23;
		if ((chan >= 12) && (chan <= 13))
			gcr_id = 46;
		if ((chan >= 14) && (chan <= 15))
			gcr_id = 47;
		if ((chan >= 16) && (chan <= 31)) {
			switch (vsp) {
			case VSP1:
				gcr_id = 56;
				break;
			case VSP2:
				gcr_id = 57;
				break;
			case VSP3:
				gcr_id = 58;
				break;
			case VSP4:
				gcr_id = 59;
				break;
			default:
				gcr_id = 0;
			}
		}
	} else if (vsp >= M2VSP_MIN && vsp <= M2VSP_MAX) {
		if ((chan >= 8) && (chan <= 9))
			gcr_id = 2;
		if ((chan >= 10) && (chan <= 11))
			gcr_id = 3;
		if ((chan >= 12) && (chan <= 13))
			gcr_id = 49;
		if ((chan >= 14) && (chan <= 15))
			gcr_id = 50;
		if ((chan >= 16) && (chan <= 31)) {
			switch (vsp) {
			case VSP8:
				gcr_id = 60;
				break;
			case VSP9:
				gcr_id = 61;
				break;
			case VSP10:
				gcr_id = 62;
				break;
			case VSP11:
				gcr_id = 63;
				break;
			default:
				gcr_id = 0;
			}
		}
	} else if (vsp >= M3VSP_MIN && vsp <= M3VSP_MAX) {
		if ((chan >= 8) && (chan <= 10))
			gcr_id = 41;
		else if ((chan >= 11) && (chan <= 13))
			gcr_id = 42;
		else if ((chan >= 14) && (chan <= 15))
			gcr_id = 5;
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

	return gcr_id;
}


int gcr_vsp_intf_dma_cfg(struct dma_intf_switch_parm_t *chan_parm,
		unsigned char count)
{
	unsigned int tmp = 0;
	unsigned char gcr_id = 0; /*invalid id*/
	u32 value = 0;
	enum dma_channel_id_t chan;
	enum vsp_id_t vsp;

	for (tmp = 0; tmp < count; tmp++) {

		chan = (chan_parm + tmp)->chan_id;
		vsp = (chan_parm + tmp)->vsp_id;
		gcr_id = get_gcr_reg(vsp, chan);
		if (gcr_id != 0) {
			if (chan < 16)
				value = vsp_intf_gcr_reg_cfg(gcr_id,
						(chan_parm + tmp));
			else
				value = vsp_intf_gcr_reg_cfg_2bit(gcr_id,
						(chan_parm + tmp));

			scm_reg_write(value, gcr_id);
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

static u32 get_prr_gcr_id(enum vsp_id_t vsp_id, enum dma_channel_id_t chan_id)
{
	u32 gcr_id = 0;
	if (vsp_id >= M3VSP_MIN && vsp_id <= M3VSP_MAX) {
		if (chan_id >=  DMA_CHAN_8 && chan_id <= DMA_CHAN_13)
			gcr_id = 45;
		else if (chan_id >=  DMA_CHAN_14 && chan_id <= DMA_CHAN_15)
			gcr_id = 5;
	} else if (vsp_id >= M1VSP_MIN && vsp_id <= M1VSP_MAX) {
		if (chan_id >=  DMA_CHAN_8 && chan_id <= DMA_CHAN_11)
			gcr_id = 43;
		else if (chan_id >=  DMA_CHAN_12 && chan_id <= DMA_CHAN_15)
			gcr_id = 48;
	} else if (vsp_id >= M2VSP_MIN && vsp_id <= M2VSP_MAX) {
		if (chan_id >=  DMA_CHAN_8 && chan_id <= DMA_CHAN_11)
			gcr_id = 44;
		else if (chan_id >=  DMA_CHAN_12 && chan_id <= DMA_CHAN_15)
			gcr_id = 51;
	}

	return gcr_id;
}

static u8 jesd_prr_val(enum vsp_id_t vsp_id)
{
	u8 val = 0;

	if (vsp_id >= M3VSP_MIN && vsp_id <= M3VSP_MAX)
		val = vsp_id - 0x04;
	else if (vsp_id >= M1VSP_MIN && vsp_id <= M1VSP_MAX)
		val = vsp_id;
	else if (vsp_id >= M2VSP_MIN && vsp_id <= M2VSP_MAX)
		val = vsp_id - 0x07;

	return val;
}

static s8 get_prr_shift_val(u32 gcr_id, enum dma_channel_id_t chan_id,
		enum jesd_id_t jesd_id, enum dma_comm_type_t comm_type)
{
	s8 bit_shift_val = -1;

	switch (gcr_id) {
	case 5:
		if (comm_type == SETUP_UL) {
			switch (jesd_id) {
			case JESD10:
				bit_shift_val = (chan_id + jesd_id) + 2;
				break;
			case JESD9:
				bit_shift_val = (chan_id + jesd_id) - 2;
				break;
			default:
				break;
			}
		} else if (comm_type == SETUP_DL) {
			switch (jesd_id) {
			case JESD7:
				bit_shift_val = JESD_PRR_SHIFT_VAL_19;
				break;
			case JESD11:
				bit_shift_val = JESD_PRR_SHIFT_VAL_23;
				break;
			case JESD8:
				bit_shift_val = JESD_PRR_SHIFT_VAL_25;
				break;
			case JESD12:
				bit_shift_val = JESD_PRR_SHIFT_VAL_29;
				break;
			default:
				break;
			}
		}
		break;
	case 43:
	case 44:
		bit_shift_val = (chan_id - 8) * 3;
		break;
	case 45:
		if ((jesd_id >= JESD7 && jesd_id <= JESD10) ||
				(chan_id == DMA_CHAN_9 && jesd_id == JESD6) ||
				(chan_id == DMA_CHAN_8 && jesd_id == JESD5))
			bit_shift_val = (chan_id * 2) - 2 * (12 - jesd_id);
		else if ((jesd_id >= JESD1 && jesd_id <= JESD4) ||
				(chan_id == DMA_CHAN_13 && jesd_id == JESD6) ||
				(chan_id == DMA_CHAN_12 && jesd_id == JESD5))
			bit_shift_val = (chan_id * 2) - 2 * (9 - jesd_id);
		break;
	case 48:
	case 51:
		switch (jesd_id) {
		case JESD5:
		case JESD1:
			bit_shift_val = JESD_PRR_SHIFT_VAL_00;
			break;
		case JESD6:
		case JESD2:
			bit_shift_val = JESD_PRR_SHIFT_VAL_06;
			break;
		case JESD7:
		case JESD3:
			bit_shift_val = JESD_PRR_SHIFT_VAL_14;
			break;
		case JESD8:
		case JESD4:
			bit_shift_val = JESD_PRR_SHIFT_VAL_20;
			break;
		case JESD9:
			bit_shift_val = JESD_PRR_SHIFT_VAL_03;
			break;
		case JESD10:
			bit_shift_val = JESD_PRR_SHIFT_VAL_11;
			break;
		case JESD13:
			bit_shift_val = JESD_PRR_SHIFT_VAL_17;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return bit_shift_val;
}

static u8 jesd_prr_val_reg52(enum vsp_id_t vsp_id,
				enum dma_channel_id_t chan_id,
				enum dma_comm_type_t comm_type)
{
	u8 val = 0;

	if (vsp_id >= M1VSP_MIN && vsp_id <= M1VSP_MAX) {
		val = 0x01;
	} else if (vsp_id >= M2VSP_MIN && vsp_id <= M2VSP_MAX) {
		val = 0x02;
	} else if (vsp_id >= M3VSP_MIN && vsp_id <= M3VSP_MAX) {
		switch (comm_type) {
		case SETUP_UL:
			if (chan_id == DMA_CHAN_14 || chan_id == DMA_CHAN_15)
				val = 0x03;
			break;
		case SETUP_DL:
			if (chan_id >= DMA_CHAN_8 && chan_id <= DMA_CHAN_11)
				val = 0x01;
			else if (chan_id >= DMA_CHAN_2
					&& chan_id <= DMA_CHAN_15)
				val = 0x02;
			break;
		}
	}

	return val;
}

static u8 get_prr_shift_val_reg52(enum jesd_id_t jesd_id,
				enum dma_comm_type_t comm_type)
{
	u8 bit_shift_val = 0;

	switch (comm_type) {
	case SETUP_UL:
		bit_shift_val = jesd_id * 2 + 8;
		break;
	case SETUP_DL:
		if (jesd_id == JESD13)
			bit_shift_val = 0;
		else
			bit_shift_val = jesd_id * 2 - 8;
		break;
	}

	return bit_shift_val;
}

static u8 validate_req(enum vsp_id_t vsp_id, enum dma_channel_id_t chan_id,
			enum jesd_id_t jesd_id, enum dma_comm_type_t comm_type)
{
	u8 ret = 0;
	if (vsp_id >= M1VSP_MIN && vsp_id <= M1VSP_MAX) {
		switch (comm_type) {
		case SETUP_DL:
			if (jesd_id == JESD13 && chan_id == DMA_CHAN_14)
				ret = 1;
			break;
		case SETUP_UL:
			if ((jesd_id == JESD10 && chan_id == DMA_CHAN_13) ||
				(jesd_id == JESD9 && chan_id == DMA_CHAN_12) ||
				(jesd_id == JESD8 && chan_id == DMA_CHAN_15) ||
				(jesd_id == JESD7 && chan_id == DMA_CHAN_14) ||
				(jesd_id == JESD6 && chan_id == DMA_CHAN_13) ||
				(jesd_id == JESD5 && chan_id == DMA_CHAN_12) ||
				(jesd_id == JESD4 && chan_id == DMA_CHAN_11) ||
				(jesd_id == JESD3 && chan_id == DMA_CHAN_10) ||
				(jesd_id == JESD2 && chan_id == DMA_CHAN_9) ||
				(jesd_id == JESD1 && chan_id == DMA_CHAN_8))
				ret = 1;
			break;
		}
	} else if (vsp_id >= M2VSP_MIN && vsp_id <= M2VSP_MAX) {
		switch (comm_type) {
		case SETUP_DL:
			if (jesd_id == JESD13 && chan_id == DMA_CHAN_14)
				ret = 1;
			break;
		case SETUP_UL:
			if ((jesd_id == JESD10 && chan_id == DMA_CHAN_13) ||
				(jesd_id == JESD9 && chan_id == DMA_CHAN_12) ||
				(jesd_id == JESD8 && chan_id == DMA_CHAN_11) ||
				(jesd_id == JESD7 && chan_id == DMA_CHAN_10) ||
				(jesd_id == JESD6 && chan_id == DMA_CHAN_9) ||
				(jesd_id == JESD5 && chan_id == DMA_CHAN_8) ||
				(jesd_id == JESD4 && chan_id == DMA_CHAN_15) ||
				(jesd_id == JESD3 && chan_id == DMA_CHAN_15) ||
				(jesd_id == JESD2 && chan_id == DMA_CHAN_13) ||
				(jesd_id == JESD1 && chan_id == DMA_CHAN_12))
				ret = 1;
			break;
		}
	} else if (vsp_id >= M3VSP_MIN && vsp_id <= M3VSP_MAX) {
		switch (comm_type) {
		case SETUP_DL:
			if ((jesd_id == JESD8 &&
					(chan_id == DMA_CHAN_11 ||
						chan_id == DMA_CHAN_15)) ||
					(jesd_id == JESD7 &&
					(chan_id == DMA_CHAN_10 ||
						chan_id == DMA_CHAN_14)) ||
					(jesd_id == JESD6 &&
					(chan_id == DMA_CHAN_9 ||
						chan_id == DMA_CHAN_13)) ||
					(jesd_id == JESD5 &&
					(chan_id == DMA_CHAN_8 ||
						chan_id == DMA_CHAN_12)))
				ret = 1;
			break;
		case SETUP_UL:
			if ((jesd_id == JESD10 && chan_id == DMA_CHAN_15) ||
				(jesd_id == JESD9 && chan_id == DMA_CHAN_14))
				ret = 1;
			break;
		}
	}

	return ret;
}

int gcr_jesd_dma_ptr_rst_req(struct jesd_dma_ptr_rst_parm *ptr_rst_parm,
		u8 count)
{
	u32 gcr_id = 0;
	u32 tmp = 0;
	u32 reg_val = 0;
	u8 mask = 0;
	s8 bit_shift_val = 0;
	u8 val = 0;
	enum vsp_id_t vsp_id;
	enum jesd_id_t jesd_id;
	enum dma_channel_id_t chan_id;
	enum dma_comm_type_t comm_type;

	for (tmp = 0; tmp < count; tmp++) {
		vsp_id = (ptr_rst_parm + tmp)->vsp_id;
		jesd_id = (ptr_rst_parm + tmp)->jesd_id;
		chan_id = (ptr_rst_parm + tmp)->chan_id;
		comm_type = (ptr_rst_parm + tmp)->comm_type;
		if (vsp_id < MIN_VSP_ID || vsp_id > MAX_VSP_ID)
			return -EINVAL;

		if (comm_type != SETUP_DL && comm_type != SETUP_UL)
			return -EINVAL;

		if (comm_type == SETUP_UL &&
				jesd_id == JESD3 &&
				chan_id == DMA_CHAN_15)
			if (vsp_id >= M2VSP_MIN && vsp_id <= M2VSP_MAX)
				goto REG52;

		gcr_id = get_prr_gcr_id(vsp_id, chan_id);
		if (gcr_id == 0)
			return -EINVAL;
		/* get mask value */
		if (vsp_id >= M3VSP_MIN && vsp_id <= M3VSP_MAX)
			mask = GCR_MSK1;
		else
			mask = GCR_MSK;

		val = jesd_prr_val(vsp_id);
		bit_shift_val = get_prr_shift_val(gcr_id, chan_id,
						jesd_id, comm_type);
		if (bit_shift_val < 0)
			return -EINVAL;

		reg_val = scm_reg_read(gcr_id);
		reg_val = ((reg_val & (~(mask << bit_shift_val)))
			| (val << bit_shift_val));
		scm_reg_write(reg_val, gcr_id);

REG52:
		/* logic to set GCR reg 52 */
		if (((comm_type == SETUP_UL) &&
				(jesd_id >= JESD1 && jesd_id <= JESD10)) ||
				((comm_type == SETUP_DL) &&
				((jesd_id >= JESD5 && jesd_id <= JESD8) ||
				(jesd_id == JESD13)))) {
			if (!validate_req(vsp_id, chan_id, jesd_id, comm_type))
				return 0;
			mask = GCR_MSK1;
			gcr_id = 52;
			reg_val = 0;
			val = jesd_prr_val_reg52(vsp_id, chan_id, comm_type);
			if (val == 0x00)
				return -EINVAL;

			bit_shift_val = get_prr_shift_val_reg52(
						jesd_id, comm_type);

			reg_val = scm_reg_read(gcr_id);
			reg_val = ((reg_val & ~(mask << bit_shift_val)) |
					(val << bit_shift_val));
			scm_reg_write(reg_val, gcr_id);
		}
	}

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

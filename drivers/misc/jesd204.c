/*
 * drivers/misc/jesd204.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/tbgen.h>
#include <linux/jesd204.h>
#include <linux/jesd_phygasket.h>
#include <mach/serdes-d4400.h>

#define JESD_DBG_ERR	(1 << 0)
#define JESD_DBG_INFO	(1 << 1)

#define jesd_err(...)	do {							\
				if (jdev->debug & JESD_DBG_ERR)			\
					dev_err(jdev->dev, __VA_ARGS__);	\
			} while (0)

#define jesd_info(...)	do {							\
				if (jdev->debug & JESD_DBG_INFO)		\
					dev_info(jdev->dev, __VA_ARGS__);	\
			} while (0)

static struct class *jesd204_class;
static struct list_head jesd_list;
static u8 pll_blk0_is_on;
static u8 pll_blk1_is_on;
static struct mutex jesd_mutex;

static struct of_device_id jesd204_match_dts_id[] = {
	{.compatible = "fsl,d4400-jesd204-tx", .data = (const void *)JESD_TX},
	{.compatible = "fsl,d4400-jesd204-rx", .data = (const void *)JESD_RX},
	{},
};

static int jesd_config_phygasket(const struct jesd_complex *jdev, int flag, int lane)
{
	enum phygasket_data_src phy_data_src;
	int ret;

	switch (jdev->type) {
	case JESD_TX:
		phy_data_src = PHY_DATA_JESDTX;
		break;
	case JESD_RX:
		if (flag & PHY_GASKET_LOOPBACK)
			phy_data_src = PHY_DATA_RX_LOOPBACK;
		else
			phy_data_src = PHY_DATA_JESDRX;
		break;
	default:
		return -EINVAL;
	}

	phy_gasket_swap_lanes(jdev->phy, lane,
			flag & PHY_GASKET_SWAP, jdev->type);
	ret = phy_gasket_lane_ctrl(jdev->phy, phy_data_src, lane);
	if (ret)
		jesd_err("Phy gasket init failed\n");

	return ret;
}

static u32 jesd_calculate_chksum(const struct transport_params *params)
{
	u32 sum = 0;
	sum += params->DID;
	sum += params->BID;
	sum += params->SCR;
	sum += params->F - 1;
	sum += params->K - 1;
	sum += params->M - 1;
	sum += params->L - 1;
	sum += params->CS;
	sum += params->CF;
	sum += params->N - 1;
	sum += params->NP - 1;
	sum += params->S - 1;
	sum += params->HD;
	sum += params->SUBCLASSV;
	sum += params->JESDV;

	return sum;
}

static void jesd_config_tx_regs(const struct transport_params *params,
		__iomem struct config_registers_tx *reg, int loopback)
{
#define REG_CLK_ENABLE (1 << 0)
#define REG_L2SIDES (1 << 0)
#define REG_SYSREF_MASK (1 << 20)
	u32 val;

	writel(REG_CLK_ENABLE | REG_SYSREF_MASK, &reg->tx_transcontrol);
	wmb();
	writel(params->DID, &reg->tx_did);
	writel(params->BID, &reg->tx_bid);
	writel(params->L0ID, &reg->tx_lid_0);

	val = params->L - 1;
	val |= (params->SCR << 7);
	writel(val, &reg->tx_scr);

	writel(params->F - 1, &reg->tx_opf);
	writel(params->K - 1, &reg->tx_fpmf);
	writel(1, &reg->tx_cpd);
	writel(0xF, &reg->tx_cbps);
	writel(0xF | (params->SUBCLASSV << 5),
		&reg->tx_nbcw);
	writel(params->JESDV << 5,
		&reg->tx_spcp);

	if (params->jesd_transport_flag & L2SIDES)
		writel(REG_L2SIDES, &reg->tx_frm_ctrl);
	else
		writel(0, &reg->tx_frm_ctrl);

	writel(0x3, &reg->tx_m_en);

	if (params->L == 2)
		writel(0x3, &reg->tx_l_en);
	else
		writel(0x1, &reg->tx_l_en);

	writel(params->sync_delay, &reg->tx_sync_delay);

#define REG_PIPELINE_TX	(6 << 23)
#define REG_SYNC_TBGEN	(1 << 21)
#define REG_IQ_SWAP	(1 << 7)
#define REG_SYNC_REQ_BYPASS	(1 << 22)
#define REG_SYNC_TBGEN		(1 << 21)
	val = ((params->clk_div - 1) << 4) | REG_CLK_ENABLE | REG_PIPELINE_TX;

	if (params->jesd_transport_flag & IQ_SWAP)
		val |= REG_IQ_SWAP;
	if (params->jesd_transport_flag & SYNCN_TO_REF)
		val |= REG_SYNC_REQ_BYPASS;
	if (params->jesd_transport_flag & SYNC_FROM_TBGEN)
		val |= REG_SYNC_TBGEN;
	if (loopback)
		val |= REG_SYNC_TBGEN;

	writel(val | REG_SYSREF_MASK, &reg->tx_transcontrol);
}


static void jesd_config_tx(struct jesd_complex *jdev,
	const struct jesd_complex_params *params)
{
	__iomem struct config_registers_tx *reg0, *reg1;
	u8 checksum;
	int loopback;

	reg0 = (__iomem struct config_registers_tx *)(jdev->tdev[0]->regs);
	reg1 = (__iomem struct config_registers_tx *)(jdev->tdev[1]->regs);

	loopback = params->jesd_complex_flag &
		(PHY_GASKET_LOOPBACK | SERDES_LOOPBACK);

	if (params->trans0_params.L) {
		jesd_config_tx_regs(&params->trans0_params, reg0, loopback);
		atomic_set(&jdev->tdev[0]->state, JESD_STATE_READY);

		checksum =
			(jesd_calculate_chksum(&params->trans0_params) +
			params->trans0_params.L0ID) & 0xFF;
		writel(checksum, &reg0->tx_csum_0);

		if (params->trans0_params.L == 2) {
			checksum = (jesd_calculate_chksum(&params->trans0_params) +
				params->trans0_params.L1ID) & 0xFF;
			writel(checksum, &reg0->tx_csum_1);
			writel(params->trans0_params.L1ID, &reg0->tx_lid_1);
		}
	}
	if (params->trans1_params.L) {
		jesd_config_tx_regs(&params->trans1_params, reg1, loopback);
		atomic_set(&jdev->tdev[1]->state, JESD_STATE_READY);

		checksum =
			(jesd_calculate_chksum(&params->trans1_params) +
			params->trans1_params.L0ID) & 0xFF;
		writel(checksum, &reg1->tx_csum_0);
	}
}

static void jesd_config_rx_regs(const struct transport_params *params,
		__iomem struct config_registers_rx *reg)
{
	u32 val;

	writel(REG_CLK_ENABLE | REG_SYSREF_MASK, &reg->rx_transcontrol);
	wmb();
#define REG_RX_DIS	(1 << 7)
#define REG_ILD_RST	(1 << 4)
#define REG_SRST	(1 << 3)
	writel(REG_RX_DIS | REG_ILD_RST | REG_SRST, &reg->rx_ctrl_0);
	udelay(200);
	writel(REG_RX_DIS, &reg->rx_ctrl_0);
	writel(params->DID, &reg->rx_did);
	writel(params->BID, &reg->rx_bid);
	writel(params->L0ID, &reg->rx_lid_0);

	val = params->L - 1;
	val |= (params->SCR << 7);
	writel(val, &reg->rx_scr);

	writel(params->F - 1, &reg->rx_opf);
	writel(params->K - 1, &reg->rx_fpmf);
	writel(1, &reg->rx_cpd);
	writel(0xF, &reg->rx_cbps);
	writel(0xF | (params->SUBCLASSV << 5),
		&reg->rx_nbcw);
	writel(params->JESDV << 5,
		&reg->rx_spcp);
	writel(params->F, &reg->rx_ctrl_1);

	if (params->L == 2)
		writel(0x3, &reg->rx_lane_en);
	else
		writel(0x1, &reg->rx_lane_en);

	writel(params->sync_delay, &reg->rx_rcv_delay);

#define REG_PIPELINE_RX	(3 << 23)
#define REG_WCBUF_OVERFLOW_PROTECT	(1 << 18)
	val = ((params->clk_div - 1) << 4) | REG_CLK_ENABLE | REG_PIPELINE_RX;

	if (params->jesd_transport_flag & RCBUF_UNDERFLOW_PROTECT)
		val |= REG_WCBUF_OVERFLOW_PROTECT;

	if (params->jesd_transport_flag & IQ_SWAP)
		val |= REG_IQ_SWAP;

	writel(val | REG_SYSREF_MASK, &reg->rx_transcontrol);
	writel(0, &reg->rx_lane_dskew);
	writel(0, &reg->rx_sync_ass_msk);
	writel(0, &reg->rx_irq_ve_msk);
}

static void jesd_config_rx(struct jesd_complex *jdev,
	const struct jesd_complex_params *params)
{
	__iomem struct config_registers_rx *reg0, *reg1;
	u8 checksum;

	reg0 = (__iomem struct config_registers_rx *)(jdev->tdev[0]->regs);
	reg1 = (__iomem struct config_registers_rx *)(jdev->tdev[1]->regs);

	if (params->trans0_params.L) {
		jesd_config_rx_regs(&params->trans0_params, reg0);
		atomic_set(&jdev->tdev[0]->state, JESD_STATE_READY);
		checksum =
			(jesd_calculate_chksum(&params->trans0_params) +
			params->trans0_params.L0ID) & 0xFF;
		writel(checksum, &reg0->rx_csum_0);
	}
	if (params->trans1_params.L) {
		jesd_config_rx_regs(&params->trans1_params, reg1);
		atomic_set(&jdev->tdev[1]->state, JESD_STATE_READY);
		checksum =
			(jesd_calculate_chksum(&params->trans1_params) +
			params->trans1_params.L0ID) & 0xFF;
		writel(checksum, &reg1->rx_csum_0);
	}
}

static int jesd_serdes_lane_init(int lane_id, int rate_kbps,
	const struct jesd_complex *jdev, int pair_lanes)
{
	struct serdes_lane_params lane_param;
	int ret = 0;

	memset(&lane_param, 0 , sizeof(struct serdes_lane_params));

	lane_param.lane_id = lane_id;
	lane_param.gen_conf.lane_prot = SERDES_LANE_PROTS_JESD204;
	lane_param.gen_conf.bit_rate_kbps = rate_kbps;

	if (jdev->pll_id == SERDES_PLL_1) {
		lane_param.gen_conf.cflag =
		(SERDES_20BIT_EN |  SERDES_TPLL_LES |
			SERDES_RPLL_LES);
	} else
		lane_param.gen_conf.cflag = SERDES_20BIT_EN;

	if (jdev->type == JESD_TX)
		lane_param.gen_conf.cflag |= SERDES_JESD_TX;
	else
		lane_param.gen_conf.cflag |= SERDES_JESD_RX;

	if (pair_lanes)
		lane_param.grp_prot = SERDES_PROT_JESD_2_LANE;
	else
		lane_param.grp_prot = SERDES_PROT_JESD_1_LANE;

	if ((pair_lanes && lane_id % 2) || !pair_lanes)
		lane_param.gen_conf.cflag |= SERDES_FIRST_LANE;

	if (jdev->params.jesd_complex_flag & SERDES_LOOPBACK)
		lane_param.gen_conf.cflag |= SERDES_LOOPBACK_EN;

	if (serdes_init_lane(jdev->serdes_handle, &lane_param))
		return -EINVAL;

	ret = serdes_lane_power_up(jdev->serdes_handle, lane_param.lane_id, jdev->type);
	if (ret)
		jesd_err("Failed to power up lane id=%d\n",
			lane_param.lane_id);

	return ret;
}

static int jesd_init_serdes_pll(const struct jesd_complex *jdev)
{
	struct serdes_pll_params pll_param;
	int ret = 0, i;
	unsigned int tbgen_pll_freq;

	if (jdev->complex_id < 8 && pll_blk0_is_on)
		return 0;
	if (jdev->complex_id >= 8 && pll_blk1_is_on)
		return 0;

	memset(&pll_param, 0, sizeof(struct serdes_pll_params));
	pll_param.pll_id = jdev->pll_id;
	pll_param.rfclk_sel = REF_CLK_FREQ_122_88_MHZ;
	tbgen_pll_freq = tbgen_get_pll_freq();

	if (tbgen_pll_freq == REF_CLK_614400KHZ) {
		pll_param.frate_sel = PLL_FREQ_3_072_GHZ;
		pll_param.vco_type = SERDES_RING_VCO;
	} else if (tbgen_pll_freq == REF_CLK_983040KHZ) {
		pll_param.frate_sel = PLL_FREQ_4_9152_GHZ;
		pll_param.vco_type = SERDES_LC_VCO;
	} else {
		/* 737Mhz tbgen pll */
		pll_param.frate_sel = PLL_FREQ_3_6864_GHZ;
		pll_param.vco_type = SERDES_RING_VCO;
	}

	ret = serdes_init_pll(jdev->serdes_handle, &pll_param);
	if (ret) {
		jesd_err("%s fail\n", __func__);
		return ret;
	}

	/* Power down serdes lanes in the block */
	if (jdev->complex_id < 8) {
		pll_blk0_is_on = 1;
		for (i = 0; i < 8; i++) {
			serdes_lane_power_down(jdev->serdes_handle, i, JESD_TX);
			serdes_lane_power_down(jdev->serdes_handle, i, JESD_RX);
		}
	} else {
		pll_blk1_is_on = 1;
		for (i = 2; i < 6; i++) {
			serdes_lane_power_down(jdev->serdes_handle, i, JESD_TX);
			serdes_lane_power_down(jdev->serdes_handle, i, JESD_RX);
		}
	}

	return 0;
}

static int jesd_init_phy(const struct jesd_complex *jdev,
	const struct jesd_complex_params *params)
{
	int t0_lanes, t1_lane;
	int serdes[2] = {-1, -1};
	int phy[2] = {-1, -1};
	int pair_lane = 0;
	unsigned int lane_rate[2], temp;
	unsigned tbgen_pll_freq = tbgen_get_pll_freq();
	int ret, loopback, i;

	t0_lanes = params->trans0_params.L;
	t1_lane = params->trans1_params.L;
	loopback = params->jesd_complex_flag &
		(PHY_GASKET_LOOPBACK | SERDES_LOOPBACK);

	if (t0_lanes == 2 && t1_lane == 0) {
		serdes[0] = jdev->first_serdes;
		serdes[1] = jdev->first_serdes + 1;
		phy[0] = jdev->first_serdes;
		phy[1] = jdev->first_serdes + 1;
		pair_lane = 1;
		lane_rate[0] = tbgen_pll_freq * 10 / params->trans0_params.clk_div;
		lane_rate[1] = lane_rate[0];
	} else if (t0_lanes == 1 && t1_lane == 1) {
		serdes[0] = jdev->first_serdes;
		serdes[1] = jdev->first_serdes + 1;
		phy[0] = jdev->first_serdes;
		phy[1] = jdev->first_serdes + 1;
		lane_rate[0] = tbgen_pll_freq * 10 / params->trans0_params.clk_div;
		lane_rate[1] = tbgen_pll_freq * 10 / params->trans1_params.clk_div;
		if (params->jesd_complex_flag & PHY_GASKET_SWAP) {
			temp = lane_rate[0];
			lane_rate[0] = lane_rate[1];
			lane_rate[1] = temp;
		}
	} else if (t0_lanes == 1 && t1_lane == 0) {
		serdes[0] = (params->jesd_complex_flag & PHY_GASKET_SWAP) ?
			(jdev->first_serdes + 1) : jdev->first_serdes;
		phy[0] = jdev->first_serdes;
		lane_rate[0] = tbgen_pll_freq * 10 / params->trans0_params.clk_div;
	} else if (t0_lanes == 0 && t1_lane == 1) {
		serdes[0] = (params->jesd_complex_flag & PHY_GASKET_SWAP) ?
			jdev->first_serdes : (jdev->first_serdes + 1);
		phy[0] = jdev->first_serdes + 1;
		lane_rate[0] = tbgen_pll_freq * 10 / params->trans1_params.clk_div;
	} else {
		jesd_err("Invaid jesd L config\n");
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		if (serdes[i] >= 0) {
			ret = jesd_serdes_lane_init(serdes[i], lane_rate[i], jdev, pair_lane);
			if (ret)
				return ret;
		}
		if (phy[i] >= 0) {
			tbgen_set_sync_loopback(phy[i], loopback);
			jesd_config_phygasket(jdev,
			jdev->params.jesd_complex_flag, phy[i]);
		}
	}

	return ret;
}

static void jesd_flush(const struct jesd_complex *jdev,
	const struct jesd_complex_params *params, unsigned int flush_mask)

{
	__iomem struct config_registers_rx *reg_rx[2];
	u32 i, val;

	if (jdev->type == JESD_TX)
		return;
	else {
		reg_rx[0] = (__iomem struct config_registers_rx *)(jdev->tdev[0]->regs);
		reg_rx[1] = (__iomem struct config_registers_rx *)(jdev->tdev[1]->regs);
	}

	if (params->trans0_params.L == 2)
		reg_rx[1] = NULL;
	else if (params->trans0_params.L == 1 && params->trans1_params.L == 1) {
		if (flush_mask == 0x1)
			reg_rx[1] = NULL;
		else if (flush_mask == 0x2) {
			reg_rx[0] = NULL;
		}
	} else if (params->trans0_params.L == 1 && params->trans1_params.L == 0)
		reg_rx[1] = NULL;
	else if (params->trans1_params.L == 1 && params->trans0_params.L == 0)
		reg_rx[0] = NULL;

#define REG_SW_DMA_ENABLE (1 << 17)
	for (i = 0; i < 2; i++) {
		if (reg_rx[i]) {
			val = readl(&reg_rx[i]->rx_transcontrol);
			writel(val & ~REG_SW_DMA_ENABLE,
				&reg_rx[i]->rx_transcontrol);
			udelay(200);
			writel(val, &reg_rx[i]->rx_transcontrol);
		}
	}
}

static void jesd_change_sync(struct jesd_complex *jdev, unsigned int mask)
{
	__iomem struct config_registers_tx *reg_tx[2];
	u32 val, i;

	if (jdev->type == JESD_TX) {
		reg_tx[0] = (__iomem struct config_registers_tx *)(jdev->tdev[0]->regs);
		reg_tx[1] = (__iomem struct config_registers_tx *)(jdev->tdev[1]->regs);
	} else
		return;

	for (i = 0; i < 2; i++) {
		val = readl(&reg_tx[i]->tx_transcontrol);
		if (mask & (1 << i))
			writel(val | REG_SYNC_TBGEN, &reg_tx[i]->tx_transcontrol);
		else
			writel(val & (~REG_SYNC_TBGEN), &reg_tx[i]->tx_transcontrol);
	}
}

static void jesd_stop(const struct jesd_complex *jdev,
	const struct jesd_complex_params *params, unsigned int disable_mask)

{
	__iomem struct config_registers_tx *reg_tx[2];
	__iomem struct config_registers_rx *reg_rx[2];
	u32 val;
	int lane0_id = -1, lane1_id = -1, i;

	if (jdev->type == JESD_TX) {
		reg_tx[0] = (__iomem struct config_registers_tx *)(jdev->tdev[0]->regs);
		reg_tx[1] = (__iomem struct config_registers_tx *)(jdev->tdev[1]->regs);
	} else {
		reg_rx[0] = (__iomem struct config_registers_rx *)(jdev->tdev[0]->regs);
		reg_rx[1] = (__iomem struct config_registers_rx *)(jdev->tdev[1]->regs);
	}

#define REG_TX_EN	(1 << 1)
	if (params->trans0_params.L == 2) {
		lane0_id = jdev->first_serdes;
		lane1_id = jdev->first_serdes + 1;
		reg_tx[1] = NULL;
		reg_rx[1] = NULL;
	} else if (params->trans0_params.L == 1 && params->trans1_params.L == 1) {
		if ((disable_mask & 0x3) ==
			(JESD_POWER_OFF_TRANSPORT0 | JESD_POWER_OFF_TRANSPORT0)) {
			lane0_id = jdev->first_serdes;
			lane1_id = jdev->first_serdes + 1;
		} else if ((disable_mask & 0x3) == JESD_POWER_OFF_TRANSPORT0) {
			lane0_id = (params->jesd_complex_flag & PHY_GASKET_SWAP) ?
				(jdev->first_serdes + 1) : jdev->first_serdes;
			reg_tx[1] = NULL;
			reg_rx[1] = NULL;
		} else if ((disable_mask & 0x3) == JESD_POWER_OFF_TRANSPORT1) {
			lane0_id = (params->jesd_complex_flag & PHY_GASKET_SWAP) ?
				jdev->first_serdes : (jdev->first_serdes + 1);
			reg_tx[0] = NULL;
			reg_rx[0] = NULL;
		}
	} else if (params->trans0_params.L == 1 && params->trans1_params.L == 0) {
		lane0_id = (params->jesd_complex_flag & PHY_GASKET_SWAP) ?
			(jdev->first_serdes + 1) : jdev->first_serdes;
		reg_tx[1] = NULL;
		reg_rx[1] = NULL;
	} else if (params->trans1_params.L == 1 && params->trans0_params.L == 0) {
		lane0_id = (params->jesd_complex_flag & PHY_GASKET_SWAP) ?
			jdev->first_serdes : (jdev->first_serdes + 1);
		reg_tx[0] = NULL;
		reg_rx[0] = NULL;
	}

	if (jdev->type == JESD_TX) {
		for (i = 0; i < 2; i++) {
			if (reg_tx[i]) {
				val = readl(&reg_tx[i]->tx_frm_ctrl);
				val &= ~REG_TX_EN;
				writel(val, &reg_tx[i]->tx_frm_ctrl);
				writel(0, &reg_tx[i]->tx_transcontrol);
				atomic_set(&jdev->tdev[i]->state, JESD_STATE_IDLE);
			}
		}
	} else {
		for (i = 0; i < 2; i++) {
			if (reg_rx[i]) {
				val = readl(&reg_rx[i]->rx_transcontrol);
				val &= ~REG_SW_DMA_ENABLE;
				writel(val, &reg_rx[i]->rx_transcontrol);
				udelay(200);
				val = readl(&reg_rx[i]->rx_ctrl_0);
				val |= REG_RX_DIS;
				writel(val, &reg_rx[i]->rx_ctrl_0);
				writel(0, &reg_rx[i]->rx_transcontrol);
				atomic_set(&jdev->tdev[i]->state, JESD_STATE_IDLE);
			}
		}
	}

	if (lane0_id >= 0 && (disable_mask & JESD_POWER_OFF_SERDES))
		serdes_lane_power_down(jdev->serdes_handle, lane0_id, jdev->type);

	if (lane1_id >= 0 && (disable_mask & JESD_POWER_OFF_SERDES))
		serdes_lane_power_down(jdev->serdes_handle, lane1_id, jdev->type);
}

static int jesd_check_tx_sync(struct jesd_complex *jdev,
		struct jesd_transport *tdev, int entry)
{

	struct config_registers_tx *reg_tx;
	u32 val, val1;
	int i, fail_user_mode = 1;
	reg_tx = (struct config_registers_tx *)tdev->regs;

	if (!entry) {
		if (atomic_read(&tdev->state) != JESD_STATE_SYNC_RETRY &&
			atomic_read(&tdev->state) != JESD_STATE_SYSREF_CAPTURE)
			return 0;
	} else {
		if (atomic_read(&tdev->state) != JESD_STATE_RUNNING)
			return 0;
	}

	writel(0x2, &reg_tx->tx_diag_sel);
	val = readl(&reg_tx->tx_diag_data) & 0x3;
#define REG_TX_SYNC	(1 << 9)
	writel(REG_TX_SYNC, &reg_tx->tx_irq_status);

	switch (val) {
	case 0:
		jesd_info("in cgs state\n");
		break;
	case 1:
		jesd_info("in ilas state\n");
		break;
	case 2:
		jesd_info("in user data state\n");
		fail_user_mode = 0;
		for (i = 0; i < 1000; i++) {
			val = readl(&reg_tx->tx_diag_data) & 0x3;
			val1 = readl(&reg_tx->tx_irq_status) & REG_TX_SYNC;
			if (val != 0x2 || val1) {
				fail_user_mode = 1;
				break;
			}
		}
		break;
	default:
		jesd_info("jesd tx in invalid state\n");
	}

	if (!entry) {
		if (!fail_user_mode) {
			atomic_set(&tdev->state, JESD_STATE_RUNNING);
			jesd_info("synced\n");
		} else {
			jesd_info("not synced\n");
			atomic_set(&tdev->state, JESD_STATE_SYNC_RETRY);
			return -EAGAIN;
		}
	} else {
		if (fail_user_mode) {
			atomic_set(&tdev->state, JESD_STATE_SYNC_FAILURE);
			jesd_info(" from running to failure mode\n");
			return -1;
		}
	}
	return 0;
}

static int jesd_check_rx_sync(struct jesd_complex *jdev,
		struct jesd_transport *tdev, int entry)
{

	struct config_registers_rx *reg_rx;
	u32 lane_mask;
	int i, fail_user_mode = 0;
	reg_rx = (struct config_registers_rx *)tdev->regs;

	if (!entry) {
		if (atomic_read(&tdev->state) != JESD_STATE_SYNC_RETRY &&
			atomic_read(&tdev->state) != JESD_STATE_SYSREF_CAPTURE)
			return 0;
	} else {
		if (atomic_read(&tdev->state) != JESD_STATE_RUNNING)
			return 0;
	}

	lane_mask = readl(&reg_rx->rx_lane_en);
	for (i = 0; i < 1000; i++) {
		if (readl(&reg_rx->rx_cgs) != lane_mask ||
			readl(&reg_rx->rx_fsf) != lane_mask ||
			readl(&reg_rx->rx_ilsf) != lane_mask) {
				fail_user_mode = 1;
				break;
			}

	}
	if (readl(&reg_rx->rx_cgs) == lane_mask)
		jesd_info("cgs sync\n");
	if (readl(&reg_rx->rx_fsf) == lane_mask)
		jesd_info("frame sync\n");
	if (readl(&reg_rx->rx_ilsf) == lane_mask)
		jesd_info("initial lane sync\n");

	if (!entry) {
		if (!fail_user_mode) {
			atomic_set(&tdev->state, JESD_STATE_RUNNING);
			jesd_info("synced\n");
		} else {
			jesd_info("not synced\n");
			atomic_set(&tdev->state, JESD_STATE_SYNC_RETRY);
			return -EAGAIN;
		}
	} else {
		if (fail_user_mode) {
			atomic_set(&tdev->state, JESD_STATE_SYNC_FAILURE);
			jesd_info(" from running to failure mode\n");
			return -1;
		}
	}
	return 0;
}

static void jesd_power_off_pll(int en)
{
	struct jesd_complex *jdev, *jdev_blk0 = NULL, *jdev_blk1 = NULL;
	struct jesd_transport *tdev;
	int i;
	int pll_blk0_usage = 0, pll_blk1_usage = 0;
	if (!en)
		return;

	list_for_each_entry(jdev, &jesd_list, list) {
		if (jdev->complex_id == 0)
			jdev_blk0 = jdev;
		else if (jdev->complex_id == 8)
			jdev_blk1 = jdev;

		for (i = 0; i < 2; i++) {
			tdev = jdev->tdev[i];
			if (atomic_read(&tdev->state) > JESD_STATE_IDLE) {
				if (jdev->complex_id < 8)
					pll_blk0_usage = 1;
				else
					pll_blk1_usage = 1;
			}
		}
	}
	if (!pll_blk0_usage) {
		serdes_pll_power_down(jdev_blk0->serdes_handle, jdev_blk0->pll_id);
		pll_blk0_is_on = 0;
	}
	if (!pll_blk1_usage) {
		serdes_pll_power_down(jdev_blk1->serdes_handle, jdev_blk1->pll_id);
		pll_blk1_is_on = 0;
	}
}


static int jesd_start_ready(unsigned int timeout)
{
	struct jesd_complex *jdev;
	struct jesd_transport *tdev = NULL;
	struct config_registers_tx *reg_tx;
	struct config_registers_rx *reg_rx;
	u32 val;
	unsigned long timeout_jiffies;
	int ret, all_sync_fail, i, find_ready = 0;

	list_for_each_entry(jdev, &jesd_list, list) {
		for (i = 0; i < 2; i++) {
			tdev = jdev->tdev[i];
			if (atomic_read(&tdev->state) == JESD_STATE_READY) {
				find_ready = 1;
				break;
			}
		}
		if (find_ready)
			break;
	}
	if (!find_ready)
		return -ENODEV;
#define REG_SYSREF_ROSE (1 << 8)
	/* Enable sysref interrupt */
	if (jdev->type == JESD_TX) {
		reg_tx = tdev->regs;
		writel(REG_SYSREF_ROSE, &reg_tx->tx_irq_status);
		writel(REG_SYSREF_ROSE, &reg_tx->tx_irq_enable);
	} else {
		reg_rx = tdev->regs;
		writel(REG_SYSREF_ROSE, &reg_rx->rx_irq_status);
		writel(REG_SYSREF_ROSE, &reg_rx->rx_irq_enable);
	}
	ret = wait_for_completion_timeout(&tdev->sysref_completion,
			msecs_to_jiffies(50));
	if (!ret) {
		jesd_err("Fail to get JESD sysref");
		return -EAGAIN;
	}

	list_for_each_entry(jdev, &jesd_list, list) {
		for (i = 0; i < 2; i++) {
			tdev = jdev->tdev[i];
			if (atomic_read(&tdev->state) == JESD_STATE_SYSREF_CAPTURE) {
				if (jdev->type == JESD_TX) {
					reg_tx = tdev->regs;
#define REG_DFIFO_RESET	(1 << 3)
					val = readl(&reg_tx->tx_transcontrol);
					writel(REG_DFIFO_RESET | val,
							&reg_tx->tx_transcontrol);
					udelay(20);
					writel(val, &reg_tx->tx_transcontrol);
				} else {
					reg_rx = tdev->regs;
					val = readl(&reg_rx->rx_transcontrol);
					writel(REG_DFIFO_RESET | val,
							&reg_rx->rx_transcontrol);
					udelay(20);
					writel(val | REG_SW_DMA_ENABLE, &reg_rx->rx_transcontrol);
				}
			}
		}
	}

	list_for_each_entry(jdev, &jesd_list, list) {
		for (i = 0; i < 2; i++) {
			tdev = jdev->tdev[i];
			if (atomic_read(&tdev->state) == JESD_STATE_SYSREF_CAPTURE) {
				if (jdev->type == JESD_TX) {
					reg_tx = tdev->regs;
					val = readl(&reg_tx->tx_frm_ctrl);
					writel(val | REG_TX_EN, &reg_tx->tx_frm_ctrl);
				} else {
					reg_rx = tdev->regs;
					val = readl(&reg_rx->rx_ctrl_0);
					writel(val & (~REG_RX_DIS), &reg_rx->rx_ctrl_0);
				}
			}
		}
	}

	timeout_jiffies = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout_jiffies)) {
		all_sync_fail = 0;
		list_for_each_entry(jdev, &jesd_list, list) {
			for (i = 0; i < 2; i++) {
				tdev = jdev->tdev[i];
				if (jdev->type == JESD_TX) {
					if (jesd_check_tx_sync(jdev, tdev, 0))
						all_sync_fail = 1;
				}
				if (jdev->type == JESD_RX) {
					if (jesd_check_rx_sync(jdev, tdev, 0))
						all_sync_fail = 1;
				}
			}
		}
		if (!all_sync_fail)
			return 0;
		else
			schedule_timeout_interruptible(1);
	}

	list_for_each_entry(jdev, &jesd_list, list) {
		for (i = 0; i < 2; i++) {
			tdev = jdev->tdev[i];
			if (atomic_read(&tdev->state) == JESD_STATE_SYNC_RETRY)
				atomic_set(&tdev->state, JESD_STATE_SYNC_FAILURE);
		}
	}

	return -ETIME;
}

static long jesd204_ioctl(struct file *pfile, unsigned int cmd,
						unsigned long arg)
{
	struct jesd_complex *jdev = (struct jesd_complex *)pfile->private_data;
	struct jesd_complex_state state;
	int ret = 0;

	switch (cmd) {
	case JESD_INIT_PARAM:
		if (copy_from_user(&jdev->params,
				(struct jesd_complex_params *)arg,
				 sizeof(struct jesd_complex_params))) {
			ret = -EFAULT;
			break;
		}
		break;

	case JESD_INIT_HW:
		if (arg & JESD_INIT_SERDES_PLL) {
			mutex_lock(&jesd_mutex);
			ret = jesd_init_serdes_pll(jdev);
			if (ret) {
				jesd_err("Fail to init jesd pll\n");
				mutex_unlock(&jesd_mutex);
				break;
			}
			mutex_unlock(&jesd_mutex);
		}

		if (arg & JESD_INIT_SERDES) {
			ret = jesd_init_phy(jdev, &jdev->params);
			if (ret) {
				jesd_err("Fail to init jesd serdes\n");
				break;
			}
		}

		if (arg & JESD_INIT_REGS) {
			if (jdev->type == JESD_TX)
				jesd_config_tx(jdev, &jdev->params);
			else
				jesd_config_rx(jdev, &jdev->params);
		}

		break;

	case JESD_START:
		ret = jesd_start_ready((unsigned int)arg);
		break;

	case JESD_POWER_DOWN:
		jesd_stop(jdev, &jdev->params, (unsigned int)arg);
		mutex_lock(&jesd_mutex);
		jesd_power_off_pll(arg & JESD_POWER_OFF_SERDES_PLL);
		mutex_unlock(&jesd_mutex);
		break;

	case JESD_FLUSH:
		jesd_flush(jdev, &jdev->params, (unsigned int)arg);
		break;

	case JESD_CHANGE_SYNC:
		jesd_change_sync(jdev, (unsigned int)arg);
		break;

	case JESD_CHECK_STATE:
		if (jdev->type == JESD_TX) {
			jesd_check_tx_sync(jdev, jdev->tdev[0], 1);
			jesd_check_tx_sync(jdev, jdev->tdev[1], 1);
		} else {
			jesd_check_rx_sync(jdev, jdev->tdev[0], 1);
			jesd_check_rx_sync(jdev, jdev->tdev[1], 1);
		}
		state.transport0_state = atomic_read(&jdev->tdev[0]->state);
		state.transport1_state = atomic_read(&jdev->tdev[1]->state);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			return -EFAULT;
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static int jesd204_open(struct inode *inode, struct file *pfile)
{
	struct jesd_complex *jdev;

	jdev = container_of(inode->i_cdev, struct jesd_complex, cdev);
	pfile->private_data = jdev;

	return 0;
}

int jesd204_release(struct inode *inode, struct file *pfile)
{
	return 0;
}

static const struct file_operations jesd204_fops = {
	.owner = THIS_MODULE,
	.open = jesd204_open,
	.unlocked_ioctl = jesd204_ioctl,
	.release = jesd204_release
};

static ssize_t set_debug(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	int err;
	unsigned int val;
	struct jesd_complex *jdev = dev_get_drvdata(dev);

	err = kstrtouint(buf, 0, &val);
	if (err)
		return err;

	jdev->debug = val;
	return count;
}

static ssize_t show_status(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct jesd_complex *jdev = dev_get_drvdata(dev);
	int len;
	const char *jesd_state_name[] = {"idle", "ready", "sysref_capture",
				"running", "sync_retry", "sync_fail"};

	len = sprintf(buf, "current debug flag:0x%02x\n", jdev->debug);
	len += sprintf(&buf[len], "%s transport0 state:%s, transport1 state:%s\n",
			jdev->name,
			jesd_state_name[atomic_read(&jdev->tdev[0]->state)],
			jesd_state_name[atomic_read(&jdev->tdev[1]->state)]);

	return len;
}

static irqreturn_t jesd_isr(int irq, void *_tdev)
{
	struct jesd_transport *tdev_in = _tdev, *tdev;
	struct jesd_complex *jdev;
	struct config_registers_tx *reg_tx;
	struct config_registers_rx *reg_rx;
	u32 val, i, sysref_unmask = 0;

	if (tdev_in->jdev->type == JESD_TX) {
		reg_tx = tdev_in->regs;
		val = readl(&reg_tx->tx_irq_status);
		if (val)
			writel(val, &reg_tx->tx_irq_status);
		else
			return IRQ_NONE;
	} else {
		reg_rx = tdev_in->regs;
		val = readl(&reg_rx->rx_irq_status);
		if (val)
			writel(val, &reg_rx->rx_irq_status);
		else
			return IRQ_NONE;
	}
	if (atomic_read(&tdev_in->state) == JESD_STATE_SYSREF_CAPTURE) {
		if (tdev_in->jdev->type == JESD_TX) {
			reg_tx = tdev_in->regs;
			writel(0, &reg_tx->tx_irq_enable);
		} else {
			reg_rx = tdev_in->regs;
			writel(0, &reg_rx->rx_irq_enable);
		}
	}

	list_for_each_entry(jdev, &jesd_list, list) {
		for (i = 0; i < 2; i++) {
			tdev = jdev->tdev[i];
			if (atomic_read(&tdev->state) == JESD_STATE_SYSREF_CAPTURE) {
				sysref_unmask = 1;
				if (jdev->type == JESD_TX) {
					reg_tx = tdev->regs;
					val = readl(&reg_tx->tx_transcontrol);
					writel(val | REG_SYSREF_MASK,
						&reg_tx->tx_transcontrol);
				} else {
					reg_rx = tdev->regs;
					val = readl(&reg_rx->rx_transcontrol);
					writel(val | REG_SYSREF_MASK,
						&reg_rx->rx_transcontrol);
				}
			}
			if (atomic_read(&tdev->state) == JESD_STATE_READY) {
				if (jdev->type == JESD_TX) {
					reg_tx = tdev->regs;
					val = readl(&reg_tx->tx_transcontrol);
					writel(val & (~REG_SYSREF_MASK),
						&reg_tx->tx_transcontrol);
				} else {
					reg_rx = tdev->regs;
					val = readl(&reg_rx->rx_transcontrol);
					writel(val & (~REG_SYSREF_MASK),
						&reg_rx->rx_transcontrol);
				}
				atomic_set(&tdev->state,
					JESD_STATE_SYSREF_CAPTURE);
			}
		}
	}

	if (sysref_unmask)
		complete(&tdev_in->sysref_completion);

	return IRQ_HANDLED;
}

static DEVICE_ATTR(debug,	S_IWUSR | S_IRUGO, show_status, set_debug);
static struct attribute *attributes[] = {
	&dev_attr_debug.attr,
	NULL
};
static const struct attribute_group attr_group = {
	.attrs = attributes,
};

static int jesd204_of_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct jesd_complex *jdev;
	struct device_node *child;
	struct device_node *node;
	struct of_phandle_args serdesspec;
	const struct of_device_id *match;

	jdev = devm_kzalloc(&pdev->dev,
		sizeof(struct jesd_complex), GFP_KERNEL);
	if (!jdev)
		return -ENOMEM;

	match = of_match_device(jesd204_match_dts_id, &pdev->dev);
	if (match)
		jdev->type = (enum jesd_complex_type)match->data;
	else
		return -ENODEV;

	jdev->dev = &pdev->dev;
	for (i = 0; i < 2; i++) {
		jdev->tdev[i] = devm_kzalloc(&pdev->dev,
			sizeof(struct jesd_transport), GFP_KERNEL);
		if (!jdev->tdev[i])
			return -ENOMEM;
	}

	jdev->node = pdev->dev.of_node;
	jdev->debug = JESD_DBG_ERR;

	if (of_property_read_u32(jdev->node, "complex-id",
		&jdev->complex_id)) {
		dev_err(jdev->dev, "Missing complex-id in DT\n")	;
		return -EINVAL;
	}

	if (jdev->type == JESD_TX)
		sprintf(jdev->name, "jesd_tx%d", jdev->complex_id);
	else
		sprintf(jdev->name, "jesd_rx%d", jdev->complex_id);

	if (of_property_read_u32(jdev->node, "pll-id",
		&jdev->pll_id)) {
		dev_err(jdev->dev, "Missing pll-id in DT\n")	;
		return -EINVAL;
	}

	node = of_parse_phandle(jdev->node, "phy-gasket", 0);
	if (!node) {
		dev_err(jdev->dev, "Missing phy-gasket in DT\n");
		return -EINVAL;
	}

	jdev->phy = map_phygasket(node);
	if (!jdev->phy) {
		dev_err(jdev->dev, "phy gasket is not mapped\n");
		return -EPROBE_DEFER;
	}

	if (of_get_named_serdes(jdev->node, &serdesspec, "first-serdes", 0)) {
		dev_err(jdev->dev,
			"Failed to get serdes lane handle\n");
		return -EINVAL;
	} else {
		jdev->first_serdes = serdesspec.args[0];
		jdev->serdes_handle = get_attached_serdes_dev(serdesspec.np);
		if (!jdev->serdes_handle)
			return -EPROBE_DEFER;
	}
	/* Power down serdes pll */
	serdes_pll_power_down(jdev->serdes_handle, jdev->pll_id);
	if (jdev->complex_id < 8) {
		serdes_pll_power_down(jdev->serdes_handle, 0);
		serdes_pll_power_down(jdev->serdes_handle, 1);
	}

	/* Power down serdes lanes */
	serdes_lane_power_down(jdev->serdes_handle, jdev->first_serdes, JESD_RX);
	serdes_lane_power_down(jdev->serdes_handle, jdev->first_serdes, JESD_TX);
	serdes_lane_power_down(jdev->serdes_handle, jdev->first_serdes + 1, JESD_RX);
	serdes_lane_power_down(jdev->serdes_handle, jdev->first_serdes + 1, JESD_TX);

	i = 0;

	for_each_child_of_node(jdev->node, child) {
		jdev->tdev[i]->irq = irq_of_parse_and_map(child, 0);
		if (jdev->tdev[i]->irq) {
			if (request_irq(jdev->tdev[i]->irq, jesd_isr,
				0, jdev->name, jdev->tdev[i])) {
				jesd_err("request_irq failed\n");
				return -EINVAL;
			}
		} else {
			jesd_err("No IRQ# found in DT\n");
			return -EINVAL;
		}
		jdev->tdev[i]->regs = of_iomap(child, 0);
		init_completion(&jdev->tdev[i]->sysref_completion);
		jdev->tdev[i]->jdev = jdev;
		i++;
	}

	ret = alloc_chrdev_region(&jdev->devt, 0, 1, "jesd");
	if (ret) {
		dev_err(jdev->dev,
			"jesd char region allocation failed\n");
		goto out0;
	}

	cdev_init(&jdev->cdev, &jesd204_fops);
	jdev->cdev.owner = THIS_MODULE;
	cdev_add(&jdev->cdev, jdev->devt, 1);
	device_create(jesd204_class, jdev->dev, jdev->devt,
			NULL, jdev->name);

	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Error while creating group\n");
		goto out1;
	}

	list_add(&jdev->list, &jesd_list);
	dev_set_drvdata(&pdev->dev, jdev);
	return 0;

out1:
	cdev_del(&jdev->cdev);
	device_destroy(jesd204_class, jdev->devt);
	unregister_chrdev_region(jdev->devt, 1);
out0:
	for (i = 0; i < 2; i++) {
		if (jdev->tdev[i]->regs)
			iounmap(jdev->tdev[i]->regs);
	}
	return ret;
}

static int jesd204_of_remove(struct platform_device *pdev)
{
	struct jesd_complex *jdev = NULL;
	int i;

	jdev = (struct jesd_complex *)dev_get_drvdata(&pdev->dev);
	sysfs_remove_group(&pdev->dev.kobj, &attr_group);
	cdev_del(&jdev->cdev);
	device_destroy(jesd204_class, jdev->devt);
	unregister_chrdev_region(jdev->devt, 1);
	for (i = 0; i < 2; i++) {
		iounmap(jdev->tdev[i]->regs);
		free_irq(jdev->tdev[i]->irq, jdev->tdev[i]);
	}
	return 0;
}

static struct platform_driver jesd204_driver = {
	.driver = {
		.name = "fsl-jesd204",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(jesd204_match_dts_id),
	},
	.probe = jesd204_of_probe,
	.remove = jesd204_of_remove,
};


static int __init jesd_init(void)
{
	jesd204_class = class_create(THIS_MODULE, "jesd204");

	if (IS_ERR(jesd204_class))
		return PTR_ERR(jesd204_class);

	INIT_LIST_HEAD(&jesd_list);
	mutex_init(&jesd_mutex);
	return platform_driver_register(&jesd204_driver);
}

static void __exit jesd_exit(void)
{
	platform_driver_unregister(&jesd204_driver);
	class_destroy(jesd204_class);
}

module_init(jesd_init);
module_exit(jesd_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("jesd204 driver");

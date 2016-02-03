/*
 * drivers/net/cpri/cpri_an.c
 * CPRI device driver - autoneg
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/cpri.h>
#include <linux/qixis.h>
#include <mach/serdes-d4400.h>
#include "cpri.h"


signed int set_sfp_input_amp_limit(struct cpri_framer *framer,
		u32 max_volt, u8 flag)
{
	s32 ret;
	u32 lane_id;
	struct device_node *child = NULL;
	unsigned int framer_id;

	for_each_child_of_node(framer->cpri_node, child) {

		of_property_read_u32(child, "framer-id", &framer_id);
		if (framer_id < 0) {
			ERR("Failed to get framer id\n");
			return -EINVAL;
		}
		if (framer_id == framer->id)
			break;
	}
	if (of_get_named_serdes(child, &framer->serdesspec,
			"serdes-handle", 0)) {
		ERR("Failed to get serdes-handle\n");
		return -EINVAL;
	}
	framer->serdes_handle = get_attached_serdes_dev(framer->serdesspec.np);
	if (framer->serdes_handle == NULL) {
		ERR("Failed to get serdes handle\n");
		return -EINVAL;
	}
	lane_id = framer->serdesspec.args[0];
	ret = serdes_sfp_amp_set(framer->serdes_handle, lane_id,
			max_volt, flag);
	return ret;
}


static int linkrate_autoneg_reset(struct cpri_framer *framer,
		enum cpri_link_rate linerate)
{
	struct cpri_dev *cpri_dev_pair = NULL;
	struct serdes_pll_params pll_param;
	struct serdes_lane_params lane_param;
	int serdes_init;
	int i;
	struct device *dev = framer->cpri_dev->dev;
	u32 line_rate[7] = {0, 1228800, 2457600, 3072000, 4915200,
		6144000, 9830400};
	int grp_prot_sel[4] = {SERDES_PROT_CPRI_1,
				SERDES_PROT_CPRI_1,
				SERDES_PROT_CPRI_2,
				SERDES_PROT_CPRI_2,
				};
	int lane_sel[4] = {LANE_C,
				LANE_D,
				LANE_E,
				LANE_F,
				};

	if (!(framer->autoneg_params.mode & RESET_LINK))
		return 0;

	cpri_dev_pair = get_pair_cpri_dev(framer->cpri_dev);

	pll_param.pll_id = SERDES_PLL_1;
	pll_param.rfclk_sel = REF_CLK_FREQ_122_88_MHZ;
	if ((linerate == RATE4_3072_0M) || (linerate == RATE6_6144_0M)) {
		pll_param.frate_sel = PLL_FREQ_3_072_GHZ;
		pll_param.vco_type = SERDES_RING_VCO;
	} else {
		pll_param.frate_sel = PLL_FREQ_4_9152_GHZ;
		pll_param.vco_type = SERDES_LC_VCO;
	}

	/* disable all clks */
	cpri_reg_clear(&framer->cpri_dev->regs->cpri_ctrlclk, C2_CLK_MASK);
	cpri_reg_clear(&framer->cpri_dev->regs->cpri_ctrlclk, C1_CLK_MASK);
	cpri_reg_clear(&cpri_dev_pair->regs->cpri_ctrlclk, C2_CLK_MASK);
	cpri_reg_clear(&cpri_dev_pair->regs->cpri_ctrlclk, C1_CLK_MASK);

	/* gcr setting for complex1 and 2 */
	gcr_config_cpri_line_rate(1, 0, CLEAR_LINE_RATE);
	gcr_config_cpri_line_rate(1, linerate, SET_LINE_RATE);
	gcr_linkrate_autoneg_reset(1);
	gcr_config_cpri_line_rate(2, 0, CLEAR_LINE_RATE);
	gcr_config_cpri_line_rate(2, linerate, SET_LINE_RATE);
	gcr_linkrate_autoneg_reset(2);

	serdes_init = serdes_init_pll(framer->serdes_handle, &pll_param);
	if ((serdes_init != -EALREADY) && (serdes_init != 0)) {
		dev_err(dev, "CPRI init pll fail!");
		return -EINVAL;
	}

	lane_param.gen_conf.lane_prot = SERDES_LANE_PROTS_CPRI;
	lane_param.gen_conf.bit_rate_kbps = line_rate[linerate];
	lane_param.gen_conf.cflag = (SERDES_20BIT_EN |  SERDES_TPLL_LES |
			SERDES_RPLL_LES | SERDES_FIRST_LANE);
	if (framer->autoneg_params.mode & CPRI_SERDES_LOOPBACK)
		lane_param.gen_conf.cflag |= SERDES_LOOPBACK_EN;

	for (i = 0; i < 4; i++) {
		lane_param.grp_prot = grp_prot_sel[i];
		lane_param.lane_id = lane_sel[i];
		if (serdes_init_lane(framer->serdes_handle, &lane_param)) {
			dev_err(dev, "CPRI lane init fail!");
			return -EINVAL;
		}
	}
	/* This lane outputs the CDR clk in slave mode.
	 * Jcpll will lock to this clk.
	 */
	if ((framer->cpri_dev->dev_id == 0)
			&& (framer->id == 0)) {
		lane_param.grp_prot = SERDES_PROT_CPRI_1;
		lane_param.lane_id = LANE_C;
	} else if ((framer->cpri_dev->dev_id == 0) && (framer->id == 1)) {
		lane_param.grp_prot = SERDES_PROT_CPRI_1;
		lane_param.lane_id = LANE_D;
	} else if ((framer->cpri_dev->dev_id == 1) && (framer->id == 0)) {
		lane_param.grp_prot = SERDES_PROT_CPRI_2;
		lane_param.lane_id = LANE_E;
	} else if ((framer->cpri_dev->dev_id == 1) && (framer->id == 1)) {
		lane_param.grp_prot = SERDES_PROT_CPRI_2;
		lane_param.lane_id = LANE_F;
	} else {
		ERR("CPRI lane init - no matching lane id!");
		return -EINVAL;
	}

	if (!(framer->autoneg_params.mode & REC_MODE)) {
		serdes_jcpll_enable(framer->serdes_handle, &lane_param,
			&pll_param);
		d4400_rev_clk_select(SERDES_PLL_1, REV_CLK_DIV_1);
	}

	/* Enable all framers clock */
	cpri_reg_set(&framer->cpri_dev->regs->cpri_ctrlclk, C2_CLK_MASK);
	cpri_reg_set(&framer->cpri_dev->regs->cpri_ctrlclk, C1_CLK_MASK);
	cpri_reg_set(&cpri_dev_pair->regs->cpri_ctrlclk, C2_CLK_MASK);
	cpri_reg_set(&cpri_dev_pair->regs->cpri_ctrlclk, C1_CLK_MASK);
	return 0;
}

/* Init CPRI framer before auto-negotiating */
static void cpri_init_framer(struct cpri_framer *framer,
				enum cpri_link_rate rate)
{
	struct cpri_autoneg_params *autoneg_params = &(framer->autoneg_params);
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_framer *re_framer = framer->cpri_dev->framer[0];
	enum cpri_prot_ver tproto_ver;
	u32 tx_seed;

	/* The CPRI complex is in daisy chain mode
	 * Some of the slave regs nees to be set
	 */
	cpri_reg_clear(&regs->cpri_auxctrl,
			AUX_MODE_MASK);

	if (framer->autoneg_params.mode & RE_MODE_MASTER) {
		cpri_reg_set_val(&regs->cpri_timer_cfg,
			CPRI_SYNC_ESA_MASK, CPRI_PAIRED_SYNC);
		cpri_reg_set(&re_framer->regs->cpri_auxctrl, AUX_MODE_MASK);
		cpri_reg_set(&regs->cpri_auxctrl, AUX_MODE_MASK);
		cpri_reg_set(&re_framer->regs->cpri_cwddelay, CW_DELAY_EN);
	} else if (framer->autoneg_params.mode & RE_MODE_SLAVE)
		cpri_reg_set_val(&framer->regs->cpri_timer_cfg,
				CPRI_SYNC_ESA_MASK, CPRI_SELF_SYNC);

	/* Due to the silicon bug, we are using master mode
	 * althrough is's in slave mode. The slave mode will
	 * set the SYNC_PULSE_MODE bit to get around this
	 */
	cpri_reg_clear(&regs->cpri_config, SLAVE_MODE_MASK);

	if (framer->autoneg_params.mode & REC_MODE)
		cpri_reg_set(&regs->cpri_config, CONF_SYNC_PULSE_MODE_MASK);
	else
		cpri_reg_clear(&regs->cpri_config, CONF_SYNC_PULSE_MODE_MASK);

	cpri_reg_set(&framer->regs->cpri_timeren, CPRI_TMR_EN);

	/* Disable Tx and Rx AxCs */
	cpri_reg_set_val(&regs->cpri_raccr, MASK_ALL, 0);
	cpri_reg_set_val(&regs->cpri_taccr, MASK_ALL, 0);

	/* Use user's eth pointer as default */
	cpri_reg_set_val(&regs->cpri_cmconfig,
				TX_FAST_CM_PTR_MASK,
				(u32)autoneg_params->eth_ptr);

	/* Setup init proto version */
	if ((rate < RATE5_4915_2M) || (autoneg_params->tx_prot_ver == VER_1)) {
		tproto_ver = VER_1;
		tx_seed = 0;
	} else {
		tproto_ver = VER_2;
		tx_seed = autoneg_params->tx_scr_seed;
	}
	cpri_reg_set_val(&regs->cpri_tprotver, PROTO_VER_MASK, tproto_ver);
	cpri_reg_set_val(&regs->cpri_tscrseed, SCR_SEED_MASK, tx_seed);

	cpri_reg_clear(&regs->cpri_rcr, MASK_ALL);
	cpri_reg_clear(&regs->cpri_tcr, MASK_ALL);

	/* Reset CPRIn_MAP_CONFIG and CPRIn_MAP_TBL_CONFIG */
	cpri_reg_clear(&regs->cpri_map_config, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_tbl_config, MASK_ALL);

	clear_control_tx_table(framer);
	/* Set the Max number of AxCs */
	cpri_write(framer->max_axcs, &regs->cpri_rgenmode);
	cpri_write(framer->max_axcs, &regs->cpri_tgenmode);
}

static void link_monitor_handler(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;
	struct cpri_framer *pair_framer;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	u8 cw_data[16];
	u32 reset_status;
	int lcv_cnt;


	reset_status = cpri_read(&framer->regs->cpri_hwreset)
			& (RESET_DETECT_HOLD_MASK | RESET_DETECT_MASK);

	if (reset_status) {
		if ((framer->cpri_enabled_monitor & RRE) &&
			(framer->autoneg_params.mode & RE_MODE_SLAVE))
				atomic_inc(&framer->err_cnt[RRE_BITPOS]);

		if ((framer->cpri_enabled_monitor & RRA) &&
			!(framer->autoneg_params.mode & RE_MODE_SLAVE))
				atomic_inc(&framer->err_cnt[RRA_BITPOS]);
	}

	/* Sends back reset ack in slave mode only
	 * If in daisy chain mode, the ack should be
	 * coming from node connected downlink
	 */
	if (reset_status) {
		if (framer->autoneg_params.mode & RE_MODE_SLAVE) {
			pair_framer = framer->cpri_dev->framer[1];
			if (!(pair_framer->autoneg_params.mode &
					RE_MODE_MASTER)) {
				spin_lock(&framer->tx_cw_lock);
				rdwr_tx_cw(framer, 130, TX_CW_READ, cw_data);
				cw_data[0] |= CW130_RST;
				rdwr_tx_cw(framer, 130, TX_CW_WRITE, cw_data);
				spin_unlock(&framer->tx_cw_lock);
			}
		}
	}

	/* Check control word 130, just in case we missed the interrupts */
	spin_lock(&framer->rx_cw_lock);
	read_rx_cw(framer, 130, cw_data);
	spin_unlock(&framer->rx_cw_lock);

	if (cw_data[0] & CW130_RST) {
		if ((framer->cpri_enabled_monitor & RRE) &&
			(framer->autoneg_params.mode & RE_MODE_SLAVE))
				atomic_inc(&framer->err_cnt[RRE_BITPOS]);

		if ((framer->cpri_enabled_monitor & RRA) &&
			!(framer->autoneg_params.mode & RE_MODE_SLAVE))
				atomic_inc(&framer->err_cnt[RRA_BITPOS]);
	}

	if ((cw_data[0] & CW130_RAI) && (framer->cpri_enabled_monitor & RAI))
		atomic_inc(&framer->err_cnt[RAI_BITPOS]);

	if ((cw_data[0] & CW130_SDI) && (framer->cpri_enabled_monitor & RSDI))
		atomic_inc(&framer->err_cnt[RSDI_BITPOS]);

	if ((cw_data[0] & CW130_LOS) && (framer->cpri_enabled_monitor & RLOS))
		atomic_inc(&framer->err_cnt[RLOS_BITPOS]);

	if ((cw_data[0] & CW130_LOF) && (framer->cpri_enabled_monitor & RLOF))
		atomic_inc(&framer->err_cnt[RLOF_BITPOS]);

	/* Check proto version */
	if (framer->cpri_enabled_monitor & PROTO_VER_MISMATCH) {
		spin_lock(&framer->rx_cw_lock);
		read_rx_cw(framer, 2, cw_data);
		spin_unlock(&framer->rx_cw_lock);
		if ((cw_data[0] & CW2_MASK) != result->common_prot_ver)
			atomic_inc(&framer->err_cnt[PROTO_VER_MISMATCH_BITPOS]);
	}

	/* Check eth pointer */
	if (framer->cpri_enabled_monitor & ETH_PTR_MISMATCH) {
		spin_lock(&framer->rx_cw_lock);
		read_rx_cw(framer, 194, cw_data);
		spin_unlock(&framer->rx_cw_lock);
		if ((cw_data[0] & CW194_MASK) != result->common_eth_ptr)
			atomic_inc(&framer->err_cnt[ETH_PTR_MISMATCH_BITPOS]);
	}

	/* Check JCPLL status, if in RE mode */
	if (framer->cpri_enabled_monitor & JCPLL_LOCK_LOSS) {
		if (!(framer->autoneg_params.mode & REC_MODE) &&
			!(qixis_jcpll_locked()))
			atomic_inc(&framer->err_cnt[JCPLL_LOCK_LOSS_BITPOS]);
	}

	/* Check the lcv reg */
	if (framer->cpri_enabled_monitor & RX_LINE_RATE_CODING_VIOLATION) {
			lcv_cnt = cpri_read(&framer->regs->cpri_lcv) & 0xFF;
			atomic_add(lcv_cnt,
			&framer->err_cnt[RX_LINE_RATE_CODING_VIOLATION_BITPOS]);
	}

	/* We check this for possible link failure,
	 * But user needs to check other stats for link too.
	 */
	if (framer->cpri_enabled_monitor & CPRI_RATE_SYNC) {
		if (cpri_reg_get_val(&framer->regs->cpri_status,
			(RX_HFN_STATE_MASK | RX_BFN_STATE_MASK |
			RX_STATE_MASK | RX_LOS_MASK)) != 0xE)
			atomic_inc(&framer->err_cnt[CPRI_RATE_SYNC_BITPOS]);
	}

	mod_timer(&framer->link_monitor_timer, jiffies + HZ);
}

static void cpri_start_monitor(struct cpri_framer *framer,
	int timer_interval)
{
	init_timer(&framer->link_monitor_timer);
	framer->link_monitor_timer.function = link_monitor_handler;
	framer->link_monitor_timer.data = (unsigned long) framer;

	framer->link_monitor_timer.expires = jiffies + timer_interval * HZ;
	add_timer(&framer->link_monitor_timer);
}

/* This function will enable the err interrupt and link monitor
 * if corresponding bits are set
 */
void cpri_set_monitor(struct cpri_framer *framer,
		const struct monitor_config_en *config)
{
	int i;
	u32 mask;

	spin_lock(&framer->err_en_lock);

	framer->cpri_enabled_monitor |= config->enable_mask;

	/* Read it once to clear the sticky remote request bit */
	if (framer->cpri_enabled_monitor & (RRE | RRA))
		cpri_read(&framer->regs->cpri_hwreset);

	/* Read clear the lcv reg */
	if (framer->cpri_enabled_monitor &
			RX_LINE_RATE_CODING_VIOLATION)
		cpri_read(&framer->regs->cpri_lcv);


	/* Enable Err interrupts */
	mask = CPRI_ERR_EVT_ALL;
	if (!(framer->eth_priv->ndev->flags & IFF_UP))
		mask &= ((~RX_ETH_DMA_OVERRUN) &
			(~ETH_FORWARD_REM_FIFO_FULL));

	/* Clear error reg */
	cpri_write(mask & framer->cpri_enabled_monitor,
		&framer->regs->cpri_errevent);

	cpri_write(mask & framer->cpri_enabled_monitor,
		&framer->regs->cpri_errinten);

	/* Clear the corresponding stats */
	for (i = 0; i < SFP_MONITOR_BITPOS; i++) {
		if (framer->cpri_enabled_monitor & (1 << i))
			atomic_set(&framer->err_cnt[i], 0);
	}

	if (framer->cpri_enabled_monitor & SFP_MONITOR) {
		atomic_set(&framer->err_cnt[SFP_PRESENCE_BITPOS], 0);
		atomic_set(&framer->err_cnt[SFP_RXLOS_BITPOS], 0);
		atomic_set(&framer->err_cnt[SFP_TXFAULT_BITPOS], 0);
	}

	/* Enable other err monitor */
	if (config->timer_enable) {
		if (!test_and_set_bit(CPRI_MONITOR_STARTED_BITPOS,
				&framer->cpri_state))
			cpri_start_monitor(framer, config->timer_interval);
	}
	spin_unlock(&framer->err_en_lock);
}
/* This function will disable the monitor functions
 * if corresponding bits are set.
 */
void cpri_clear_monitor(struct cpri_framer *framer,
			const struct monitor_config_disable *config)
{
	spin_lock(&framer->err_en_lock);

	framer->cpri_enabled_monitor &= (~config->disable_mask);

	if (config->timer_disable) {
		if (test_and_clear_bit(CPRI_MONITOR_STARTED_BITPOS,
					&framer->cpri_state))
			del_timer(&framer->link_monitor_timer);
	}

	cpri_write(framer->cpri_enabled_monitor & CPRI_ERR_EVT_ALL,
			&framer->regs->cpri_errinten);

	spin_unlock(&framer->err_en_lock);
}

static void eth_setup_timer_expiry_hndlr(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	set_bit(ETHPTR_TIMEREXP_BITPOS,
		&framer->timer_expiry_events);
}

static int cpri_cm_autoneg(struct cpri_framer *framer)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_params);
	struct timer_list eth_setup_timer;
	struct cpri_framer_regs *regs = framer->regs;
	/* struct device *dev = framer->cpri_dev->dev; */
	u8 tx_eth_rate;
	u32 cmconfig;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);

	init_eth(framer);
	/* Init eth rate expiry timer */
	init_timer(&eth_setup_timer);
	eth_setup_timer.function = eth_setup_timer_expiry_hndlr;
	eth_setup_timer.data = (unsigned long) framer;

	clear_bit(ETHPTR_TIMEREXP_BITPOS, &framer->timer_expiry_events);

	/* Setup eth rate expiry timer */
	eth_setup_timer.expires = jiffies + (param->ethptr_neg_timeout) * HZ / 1000;
	add_timer(&eth_setup_timer);

	tx_eth_rate = param->eth_ptr;
	cpri_reg_set_val(&regs->cpri_cmconfig,
			TX_FAST_CM_PTR_MASK, (u32)tx_eth_rate);

	while (!test_bit(ETHPTR_TIMEREXP_BITPOS,
		&framer->timer_expiry_events)) {
		cmconfig = cpri_read(&regs->cpri_cmstatus);

		if (cmconfig & RX_FAST_CM_PTR_VALID_MASK) {
			if (!(param->mode & STICK_TO_ETHPTR)) {
				tx_eth_rate = cmconfig & RX_FAST_CM_PTR_MASK;
				cpri_reg_set_val(&regs->cpri_cmconfig,
					TX_FAST_CM_PTR_MASK, tx_eth_rate);
				break;
			} else if (tx_eth_rate ==
				(cmconfig & RX_FAST_CM_PTR_MASK))
				break;
		}
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}

	del_timer(&eth_setup_timer);
	if (test_bit(ETHPTR_TIMEREXP_BITPOS,
		&framer->timer_expiry_events)) {
		DEBUG(DEBUG_AUTONEG, "C&M autoneg failed\n");
		return -ETIME;
	} else {
		result->common_eth_ptr = tx_eth_rate;
		set_bit(CPRI_ETHPTR_BITPOS,
			&framer->cpri_state);
		DEBUG(DEBUG_AUTONEG, "C&M autoneg success, eth ptr = %d\n",
			result->common_eth_ptr);
		return 0;
	}
}

static void proto_setup_timer_expiry_hndlr(unsigned long ptr)
{

	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	set_bit(PROTVER_TIMEREXP_BITPOS,
		&framer->timer_expiry_events);
}

static int cpri_proto_autoneg(struct cpri_framer *framer)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_params);
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	struct cpri_framer_regs  __iomem *regs = framer->regs;
	struct timer_list proto_setup_timer;
	/* struct device *dev = framer->cpri_dev->dev; */
	enum cpri_prot_ver tproto_ver, rproto_ver;
	unsigned long timer_dur;
	u8 rx_scr_en;

	/* Setup protocol ver expiry timer */
	init_timer(&proto_setup_timer);
	proto_setup_timer.function = proto_setup_timer_expiry_hndlr;
	timer_dur = jiffies + param->ethptr_neg_timeout * HZ / 1000;
	proto_setup_timer.expires = timer_dur;
	proto_setup_timer.data = (unsigned long) framer;
	clear_bit(PROTVER_TIMEREXP_BITPOS,
		&framer->timer_expiry_events);

	add_timer(&proto_setup_timer);

	rx_scr_en = (u8) cpri_reg_get_val(&regs->cpri_rscrseed,
				RX_SCR_EN_MASK);
	tproto_ver = (u8) cpri_read(&regs->cpri_tprotver);

	rproto_ver = (rx_scr_en == 0) ? VER_1 : VER_2;

	if (tproto_ver != rproto_ver) {
		if (param->mode & STICK_TO_PROTO) {
			while (!test_bit(PROTVER_TIMEREXP_BITPOS,
				&framer->timer_expiry_events)) {
				rx_scr_en = (u8) cpri_reg_get_val(
						&regs->cpri_rscrseed,
						RX_SCR_EN_MASK);
				rproto_ver = (rx_scr_en == 0) ? VER_1 : VER_2;
				if (rproto_ver == tproto_ver)
					break;
				else
					schedule_timeout_interruptible(
						msecs_to_jiffies(1));
			}
		} else {
			tproto_ver = rproto_ver;
			if (tproto_ver == VER_1)
				cpri_reg_set_val(&regs->cpri_tscrseed,
					SCR_SEED_MASK, 0);
			cpri_reg_set_val(&regs->cpri_tprotver,
				PROTO_VER_MASK, tproto_ver);
		}
	}
	del_timer(&proto_setup_timer);
	if (test_bit(PROTVER_TIMEREXP_BITPOS,
		&framer->timer_expiry_events)) {
		DEBUG(DEBUG_AUTONEG, "protocol autoneg failed");
		return -ETIME;
	} else {
		result->common_prot_ver = tproto_ver;
		result->rx_scramble_seed =
			cpri_read(&regs->cpri_rscrseed) & SCR_SEED_MASK;
		set_bit(CPRI_PROTVER_BITPOS,
			&framer->cpri_state);
		DEBUG(DEBUG_AUTONEG, "protocol autoneg success, ver = %d\n",
			result->common_prot_ver);
		return 0;
	}
}

static int check_linesync(struct cpri_framer *framer,
			enum cpri_link_rate rate)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	/* struct device *dev = framer->cpri_dev->dev; */
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	int err = 0;
	u32 mask;
	u32 line_sync_acheived = 0;
	const char *rate_name[] = {"1228.8", "2457.6",
				"3072.0", "4915.2",
				"6144.0", "9830.4"};

	cpri_reg_set(&regs->cpri_config,
			(CONF_RX_EN_MASK | CONF_TX_EN_MASK));

	mask = (RX_HFN_STATE_MASK | RX_BFN_STATE_MASK |
		RX_LOS_MASK | RX_STATE_MASK);

	sfp_set_tx_enable(framer->sfp_dev, 1);

	while (!test_bit(RATE_TIMEREXP_BITPOS, &framer->timer_expiry_events)) {
		line_sync_acheived = cpri_reg_get_val(
					&regs->cpri_status, mask);

		#if 0 // DJH - Might need this later for different BBUs.
		if (line_sync_acheived == 0xE) {
			spin_lock(&framer->rx_cw_lock);
			read_rx_cw(framer, 130, cw_buf);
			spin_unlock(&framer->rx_cw_lock);
			if (!(cw_buf[0] & CW130_RAI)) {
				printk("Passed CW130_RAI test.\n");
				break;
			}
		}
		#else
		if (line_sync_acheived == 0xE)
				break;
		#endif
		schedule_timeout_interruptible(msecs_to_jiffies(2));
	}

	if (line_sync_acheived != 0xE)
		if (test_bit(RATE_TIMEREXP_BITPOS,
					&framer->timer_expiry_events)) {
			err = -ETIME;
		}

	if (!err) {
		result->common_rate = rate;
		DEBUG(DEBUG_AUTONEG, "rate autoneg success, %s Mbps\n",
			rate_name[rate - 1]);
	}
	return err;
}

static void linerate_setup_timer_expiry_hndlr(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	set_bit(RATE_TIMEREXP_BITPOS, &framer->timer_expiry_events);
}

static int cpri_rate_autoneg(struct cpri_framer *framer)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_params);
	struct device *dev = framer->cpri_dev->dev;
	struct timer_list linerate_timer;
	enum cpri_link_rate rate;
	int err = 0;
	unsigned long timer_dur;
	struct device_node *child = NULL;
	unsigned int framer_id;
	int assign_once = 0;

	for_each_child_of_node(framer->cpri_node, child) {

		of_property_read_u32(child, "framer-id", &framer_id);
		if (framer_id < 0) {
			ERR("Failed to get framer id\n");
			return -ENODEV;
		}
		if (framer_id == framer->id)
			break;
	}

	if (of_get_named_serdes(child, &framer->serdesspec,
			"serdes-handle", 0)) {
			ERR("Failed to get serdes-handle\n");
			return -ENODEV;
	}

	framer->serdes_handle = get_attached_serdes_dev(framer->serdesspec.np);
	if (framer->serdes_handle == NULL) {
		ERR("Failed to get serdes handle\n");
		return -ENODEV;
	}

	/* Setup the linerate timer */
	init_timer(&linerate_timer);
	linerate_timer.function = linerate_setup_timer_expiry_hndlr;
	linerate_timer.data = (unsigned long) framer;

	clear_bit(RATE_TIMEREXP_BITPOS, &framer->timer_expiry_events);
	if (param->rate_preferred)
		rate = param->rate_preferred;
	else
		rate = param->rate_high;

	while (1) {
		if (linkrate_autoneg_reset(framer, rate)) {
			dev_err(dev, "Line rate reset failed");
			return -EFAULT;
		}
		mdelay(1);

		cpri_init_framer(framer, rate);
		/* Start timer */
		timer_dur = jiffies + (param->rate_neg_timeout) * HZ / 1000;
		linerate_timer.expires = timer_dur;
		add_timer(&linerate_timer);
		/* check for sync achieved */
		err = check_linesync(framer, rate);
		if (err == 0)
			break;

		/* timer will be destroyed for every new line rate */
		del_timer(&linerate_timer);
		clear_bit(RATE_TIMEREXP_BITPOS, &framer->timer_expiry_events);

		if (param->rate_preferred && (param->mode & STICK_TO_RATE))
			break;
		else if (param->rate_preferred && !assign_once) {
			assign_once = 1;
			rate = param->rate_high;
			continue;
		}
		if (--rate < param->rate_low)
			break;
	}

	if (err != 0) {
		DEBUG(DEBUG_AUTONEG, "rate autoneg failed\n");
	} else {
		del_timer(&linerate_timer);
		set_bit(CPRI_RATE_BITPOS,
			&framer->cpri_state);
		if (framer->autoneg_params.mode & RADIO_FRAME_OUTPUT_SEL) {
			err = cpri_mux_10ms_output(framer->id, framer->cpri_dev->dev_id);
			if (err)
				pr_warn("cpri fail to mux 10ms output\n");
		}
	}
	return err;
}

static int process_autoneg_cmd(struct cpri_framer *framer)
{
	int retval = -EINVAL;
	int cmd = framer->autoneg_params.autoneg_steps;

	if (cmd & RATE_AUTONEG)
		clear_bit(CPRI_RATE_BITPOS,
			&framer->cpri_state);

	if (cmd & PROTO_AUTONEG)
		clear_bit(CPRI_PROTVER_BITPOS,
			&framer->cpri_state);

	if (cmd & ETHPTR_AUTONEG)
		clear_bit(CPRI_ETHPTR_BITPOS,
			&framer->cpri_state);

	if (cmd & RATE_AUTONEG) {
		retval = cpri_rate_autoneg(framer);
		if (retval)
			goto out;
	}

	if (cmd & PROTO_AUTONEG) {
		retval = cpri_proto_autoneg(framer);
		if (retval)
			goto out;
	}

	if (cmd & ETHPTR_AUTONEG) {
		retval = cpri_cm_autoneg(framer);
		if (retval)
			goto out;
	}

out:
	return retval;
}

/* Function to process autoneg related commands from user */
int cpri_autoneg_ioctl(struct cpri_framer *framer, unsigned int cmd,
				unsigned long arg)
{
	void __user *ioargp = (void __user *)arg;
	int err;

	switch (cmd) {
	case CPRI_START_AUTONEG:
		if (copy_from_user(&framer->autoneg_params,
				(struct cpri_autoneg_params *) ioargp,
				sizeof(struct cpri_autoneg_params)) != 0) {
			err = -EFAULT;
			goto out;
		}
		err = process_autoneg_cmd(framer);
		if (err != 0) {
			DEBUG(DEBUG_AUTONEG, "CPRI autoneg failure\n");
			goto out;
		}
		break;

	case CPRI_GET_AUTONEG_OUTPUT:
		if (copy_to_user((struct cpri_autoneg_output *) ioargp,
			&framer->autoneg_output,
			sizeof(struct cpri_autoneg_output))) {
			err = -EFAULT;
			goto out;
		}
		break;

	default:
		ERR("got invalid autoneg IOCTL cmd 0x%x\n", cmd);
		return -EINVAL;
	}
	return 0;

out:
	return err;
}

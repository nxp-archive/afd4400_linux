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

#include <linux/cpri.h>
#include <linux/qixis.h>
#include <mach/serdes-d4400.h>


signed int set_sfp_input_amp_limit(struct cpri_framer *framer,
		u32 max_volt, u8 flag)
{
	s32 ret;
	u32 lane_id;
	struct device_node *child = NULL;
	unsigned int framer_id;
	struct device *dev = framer->cpri_dev->dev;

	for_each_child_of_node(framer->cpri_node, child) {

		of_property_read_u32(child, "framer-id", &framer_id);
		if (framer_id < 0) {
			dev_err(dev, "Failed to get framer id\n");
			return -EINVAL;
		}
		if (framer_id == framer->id)
			break;
	}
	if (of_get_named_serdes(child, &framer->serdesspec,
			"serdes-handle", 0)) {
			dev_err(dev, "Failed to get serdes-handle\n");
			return -EINVAL;
	}
	framer->serdes_handle = get_attached_serdes_dev(framer->serdesspec.np);
	if (framer->serdes_handle == NULL) {
		dev_err(dev, "Failed to get serdes handle\n");
		return -EINVAL;
	}
	lane_id = framer->serdesspec.args[0];
	ret = serdes_sfp_amp_set(framer->serdes_handle, lane_id,
			max_volt, flag);
	return ret;
}

	/* check for all other framers
	* if any of the other framers
	* are configured, we do nothing
	* unless force reset
	*/

static int chk_framers_state(struct cpri_framer *framer)
{
	struct cpri_framer *pair_framer;
	struct cpri_dev *cpri_dev_pair = NULL;

	if (framer->id == 1)
		pair_framer = framer->cpri_dev->framer[1];
	else
		pair_framer = framer->cpri_dev->framer[0];

	cpri_dev_pair = get_pair_cpri_dev(framer->cpri_dev);

	if (pair_framer->framer_state >=
			CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS)
		return -1;
	else if (cpri_dev_pair->framer[0]->framer_state >=
			CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS)
		return -1;
	else if (cpri_dev_pair->framer[1]->framer_state >=
			CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS)
		return -1;
	else
		return 0;
}



int linkrate_autoneg_reset(struct cpri_framer *framer,
		enum cpri_link_rate linerate)
{
	struct cpri_framer *pair_framer;
	struct cpri_dev *cpri_dev_pair = NULL;

	struct serdes_pll_params pll_param;
	struct serdes_lane_params lane_param;
	int serdes_init;
	int count = 0;
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

	if (chk_framers_state(framer)) {
		if (!(framer->autoneg_param.flags & CPRI_LINK_FORCE_RST))
			return 0;
	}

	if (framer->id == 1)
		pair_framer = framer->cpri_dev->framer[1];
	else
		pair_framer = framer->cpri_dev->framer[0];

	cpri_dev_pair = get_pair_cpri_dev(framer->cpri_dev);

	pll_param.pll_id = SERDES_PLL_1;
	pll_param.rfclk_sel = REF_CLK_FREQ_122_88_MHZ;
	if ((linerate ==  RATE4_3072_0M) || (linerate == RATE6_6144_0M)) {
		pll_param.frate_sel = PLL_FREQ_3_072_GHZ;
		pll_param.vco_type = SERDES_RING_VCO;
	} else {
		pll_param.frate_sel = PLL_FREQ_4_9152_GHZ;
		pll_param.vco_type = SERDES_LC_VCO;
	}

	/* disable all clks */
	cpri_reg_clear(&framer->cpri_dev->regs->cpri_ctrlclk,
					C2_CLK_MASK);
	cpri_reg_clear(&framer->cpri_dev->regs->cpri_ctrlclk,
					C1_CLK_MASK);
	cpri_reg_clear(&cpri_dev_pair->regs->cpri_ctrlclk,
					C2_CLK_MASK);
	cpri_reg_clear(&cpri_dev_pair->regs->cpri_ctrlclk,
					C1_CLK_MASK);

	/* gcr setting for complex1 and 2 */
	gcr_config_cpri_line_rate(1, 0, CLEAR_LINE_RATE);
	gcr_config_cpri_line_rate(1, linerate, SET_LINE_RATE);
	gcr_linkrate_autoneg_reset(1);
	gcr_config_cpri_line_rate(2, 0, CLEAR_LINE_RATE);
	gcr_config_cpri_line_rate(2, linerate, SET_LINE_RATE);
	gcr_linkrate_autoneg_reset(2);

retry:  serdes_init = serdes_init_pll(framer->serdes_handle, &pll_param);
	if ((serdes_init != -EALREADY) && (serdes_init != 0)) {
		dev_err(dev, "CPRI init pll fail!");
		return -EINVAL;
	}

	lane_param.gen_conf.lane_prot = SERDES_LANE_PROTS_CPRI;
	lane_param.gen_conf.bit_rate_kbps = line_rate[linerate];
	lane_param.gen_conf.cflag = (SERDES_20BIT_EN |  SERDES_TPLL_LES |
			SERDES_RPLL_LES | SERDES_FIRST_LANE);
	if (framer->autoneg_param.flags & CPRI_SERDES_LOOPBACK)
		lane_param.gen_conf.cflag |= SERDES_LOOPBACK_EN;

	for (i = 0; i < 4; i++) {
		lane_param.grp_prot = grp_prot_sel[i];
		lane_param.lane_id = lane_sel[i];
		if (serdes_init_lane(framer->serdes_handle, &lane_param)) {
			dev_err(dev, "CPRI lane init fail!");
			return -EINVAL;
		}
	}
	/* this lane output the cdr clk */
	/* and jcpll will lock to this clk */
	if ((framer->cpri_dev->dev_id == 1) && (framer->id == 1)) {
		lane_param.grp_prot = SERDES_PROT_CPRI_1;
		lane_param.lane_id = LANE_C;
	} else if ((framer->cpri_dev->dev_id == 1) && (framer->id == 2)) {
		lane_param.grp_prot = SERDES_PROT_CPRI_1;
		lane_param.lane_id = LANE_D;
	} else if ((framer->cpri_dev->dev_id == 2) && (framer->id == 1)) {
		lane_param.grp_prot = SERDES_PROT_CPRI_2;
		lane_param.lane_id = LANE_E;
	} else {
		lane_param.grp_prot = SERDES_PROT_CPRI_2;
		lane_param.lane_id = LANE_F;
	}

	if ((framer->framer_param.ctrl_flags & CPRI_DEV_SLAVE) ||
	(framer->autoneg_param.flags & CPRI_FRMR_SELF_SYNC_MODE)) {
		serdes_jcpll_enable(framer->serdes_handle, &lane_param,
			&pll_param);
		gcr_sync_update(BGR_EN_TX10_SYNC, BGR_EN_TX10_SYNC);
		d4400_rev_clk_select(SERDES_PLL_1, REV_CLK_DIV_1);
		qixis_unlock_jcpll();

		if (!(qixis_read(QIXIS_CLK_JCPLL_STATUS) &
					APPLY_STATUS)) {
			count++;
			if (count == SERDES_PLL_LOCK_RETRY_CNT) {
				dev_err(dev, "Failed to unlock jcpll");
				return -EINVAL;
			}
			mdelay(1);
			goto retry;
		}
	} else if (!(framer->autoneg_param.flags &
				CPRI_FRMR_PAIR_SYNC_MODE))
		qixis_lock_jcpll();

	/* Enable all framers clock */
	cpri_reg_set(&framer->cpri_dev->regs->cpri_ctrlclk,
					C2_CLK_MASK);
	cpri_reg_set(&framer->cpri_dev->regs->cpri_ctrlclk,
					C1_CLK_MASK);
	cpri_reg_set(&cpri_dev_pair->regs->cpri_ctrlclk,
					C2_CLK_MASK);
	cpri_reg_set(&cpri_dev_pair->regs->cpri_ctrlclk,
					C1_CLK_MASK);
	return 0;
}

static void cpri_configure_irq_events(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	/* Enable timing interrupt events here - control events are enabled
	 * after their respective initialisation
	 */
	cpri_reg_set(&regs->cpri_rctrltiminginten,
			BFN_TIMING_EVENT_EN_MASK
			| HFN_TIMING_EVENT_EN_MASK |
			TIMING_INT_LEVEL_MASK);

	cpri_reg_set(&regs->cpri_tctrltiminginten,
			BFN_TIMING_EVENT_EN_MASK
			| HFN_TIMING_EVENT_EN_MASK|
			TIMING_INT_LEVEL_MASK);

	cpri_reg_set(&regs->cpri_errinten, CPRI_ERR_EVT_ALL);
}



static void cpri_init_framer(struct cpri_framer *framer,
				enum cpri_link_rate rate)
{
	struct cpri_dev_init_params *param = &framer->framer_param;
	struct cpri_autoneg_params *autoneg_param = &(framer->autoneg_param);
	struct cpri_framer_regs __iomem *regs = framer->regs;
	enum cpri_prot_ver tproto_ver;
	u32 tx_seed;

	clear_axc_buff(framer);
	cpri_reg_set_val(&regs->cpri_rdelay_ctrl,
				 MASK_ALL, 0x20);
	if (framer->autoneg_param.flags & CPRI_FRMR_PAIR_SYNC_MODE) {
		cpri_reg_set_val(&framer->regs->cpri_timer_cfg,
			CPRI_SYNC_ESA_MASK, CPRI_PAIRED_SYNC);
		cpri_reg_set(&framer->regs->cpri_timeren, CPRI_TMR_EN);
	} else if (framer->autoneg_param.flags & CPRI_FRMR_SELF_SYNC_MODE) {
		cpri_reg_set_val(&framer->regs->cpri_timer_cfg,
				CPRI_SYNC_ESA_MASK, CPRI_SELF_SYNC);
		cpri_reg_set(&framer->regs->cpri_timeren, CPRI_TMR_EN);
	}

	init_eth(framer);
	/* Disable Tx and Rx AxCs */
	cpri_reg_set_val(&regs->cpri_raccr,
		MASK_ALL, 0);

	cpri_reg_set_val(&regs->cpri_taccr,
		MASK_ALL, 0);
	/* Use user's first eth pointer */
	cpri_reg_set_val(&regs->cpri_cmconfig,
				TX_FAST_CM_PTR_MASK,
				(u32)autoneg_param->eth_rate);

	/* Setup init ptoto version */
	if ((rate < RATE5_4915_2M) ||
			(autoneg_param->tx_prot_ver == VER_1)) {
		tproto_ver = VER_1;
		tx_seed = 0;
	} else {
		tproto_ver = VER_2;
		tx_seed = autoneg_param->tx_scr_seed;
	}
	cpri_reg_set_val(&regs->cpri_tprotver,
				PROTO_VER_MASK, tproto_ver);
	cpri_reg_set_val(&regs->cpri_tscrseed,
				SCR_SEED_MASK, tx_seed);

	/* Clear all control interrupt events */
	cpri_reg_clear(&regs->cpri_rcr,
		MASK_ALL);

	cpri_reg_clear(&regs->cpri_tcr,
		MASK_ALL);

	if (param->ctrl_flags & CPRI_DAISY_CHAINED) {
		cpri_reg_set(&regs->cpri_auxctrl,
				AUX_MODE_MASK);
		if (framer->autoneg_param.flags & CPRI_FRMR_SELF_SYNC_MODE)
			cpri_reg_set(&regs->cpri_cwddelay, CW_DELAY_EN);
	} else
		cpri_reg_clear(&regs->cpri_auxctrl,
			AUX_MODE_MASK);

	if (param->ctrl_flags & CPRI_DEV_SLAVE)
		cpri_reg_set(&regs->cpri_config,
			SLAVE_MODE_MASK);

	if (param->ctrl_flags & CPRI_DEV_MASTER)
		cpri_reg_clear(&regs->cpri_config,
				SLAVE_MODE_MASK);

	/* CPRI config params */
	if (param->ctrl_flags & CPRI_SET_10_ACKS)
		cpri_reg_set(&regs->cpri_config,
			CONF_SET_10_ACKS_MASK);
	else
		cpri_reg_clear(&regs->cpri_config,
			CONF_SET_10_ACKS_MASK);

	if (param->ctrl_flags & CPRI_CNT_6_RESET)
		cpri_reg_set(&regs->cpri_config,
			CONF_CNT_6_RESET_MASK);
	else
		cpri_reg_clear(&regs->cpri_config,
			CONF_CNT_6_RESET_MASK);

	if (param->ctrl_flags & CPRI_SYNC_PULSE_MODE)
		cpri_reg_set(&regs->cpri_config,
			CONF_SYNC_PULSE_MODE_MASK);
	else
		cpri_reg_clear(&regs->cpri_config,
			CONF_SYNC_PULSE_MODE_MASK);

	if (param->ctrl_flags & CPRI_TX_CW_INSERT)
		cpri_reg_set(&regs->cpri_config,
			TX_CW_INSERT_EN_MASK);
	else
		cpri_reg_clear(&regs->cpri_config,
				TX_CW_INSERT_EN_MASK);

	/* Control word params */
	if (param->ctrl_flags & CPRI_CW)
		cpri_reg_set(&regs->cpri_auxcwdmasken,
			CW_EN_MASK);
	else
		cpri_reg_clear(&regs->cpri_auxcwdmasken,
			CW_EN_MASK);

	if (param->ctrl_flags & CPRI_CW130)
		cpri_reg_set(&regs->cpri_auxcwdmasken,
			CW130_EN_MASK);
	else
		cpri_reg_clear(&regs->cpri_auxcwdmasken,
				CW130_EN_MASK);

	/* Rx control param */
	if (param->ctrl_flags & CPRI_RX_IQ_SYNC)
		cpri_reg_set(&regs->cpri_rcr,
			IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(
				&regs->cpri_rcr,
				IQ_SYNC_EN_MASK);

	/* Tx control param */
	if (param->ctrl_flags & CPRI_TX_IQ_SYNC)
		cpri_reg_set(&regs->cpri_tcr,
				IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(&regs->cpri_tcr,
				IQ_SYNC_EN_MASK);

	/* ECC error indication config param */
	if (param->ctrl_flags & CPRI_SINGLE_BIT_ECC_ERROR_OUTPUT)
		cpri_reg_set(&regs->cpri_eccerrindicateen,
				SINGLE_BIT_ECC_ERROR_OUTPUT_EN_MASK);
	else
		cpri_reg_clear(&regs->cpri_eccerrindicateen,
				SINGLE_BIT_ECC_ERROR_OUTPUT_EN_MASK);

	if (param->ctrl_flags & CPRI_MULTI_BIT_ECC_ERROR_OUTPUT)
		cpri_reg_set(&regs->cpri_eccerrindicateen,
			MULTI_BIT_ECC_ERROR_OUTPUT_EN_MASK);
	else
		cpri_reg_clear(&regs->cpri_eccerrindicateen,
				MULTI_BIT_ECC_ERROR_OUTPUT_EN_MASK);

	/* Reset CPRIn_MAP_CONFIG and CPRIn_MAP_TBL_CONFIG */
	cpri_reg_clear(&regs->cpri_map_config, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_tbl_config, MASK_ALL);

	/* Tx framer size setting */
	cpri_reg_set_val(&regs->cpri_tbufsize,
			FR_BUF_SIZE_MASK,
			param->tx_framer_buffer_size);

	clear_control_tx_table(framer);
	/* Configure framer events */
	cpri_configure_irq_events(framer);

	return;
}


static void set_delay_config(struct cpri_framer *framer)
{
	struct cpri_delays_raw_cfg *cfg = &framer->delay_cfg;
	struct cpri_framer_regs __iomem *regs = framer->regs;

	cpri_reg_set_val(&regs->cpri_exdelaycfg,
				TX_EX_DELAY_MASK,
				cfg->tx_ex_delay);
	cpri_reg_set_val(&regs->cpri_exdelaycfg,
				RX_EX_DELAY_PERIOD_MASK,
				cfg->rx_ex_delay_period);
}

static void vss_reconfig(struct interface_reconf_param *recnf_parm,
			struct cpri_framer *framer)
{
	return;
}
static s32 axc_reconfig(struct interface_reconf_param *recnf_parm,
			struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	u32 rsr = 1, tsr = 1;
	u8 i = 0;

	/*Change state to CPRI_STATE_AUTONEG_COMPLETE*/
	cpri_state_machine(framer, CPRI_STATE_AUTONEG_COMPLETE);

	if (cpri_axc_map_tbl_flush(framer, UL_AXCS) < 0)
		return -EFAULT;

	if (cpri_axc_map_tbl_flush(framer, DL_AXCS) < 0)
		return -EFAULT;

	/* Clear IQ data path in CPRIn_RCR & CPRIn_TCR */
	cpri_reg_clear(&regs->cpri_rcr, IQ_EN_MASK);
	cpri_reg_clear(&regs->cpri_tcr, IQ_EN_MASK);
	/* wait for IQ status bit in CPRInRSR and CPRInTSR registers to clear */
	for (i = 0; i < 10; i++) {
		rsr = cpri_reg_get_val(&regs->cpri_rstatus, IQ_EN_MASK);
		tsr = cpri_reg_get_val(&regs->cpri_tstatus, IQ_EN_MASK);
		if ((rsr && tsr) == 0)
			break;
		schedule_timeout_interruptible(msecs_to_jiffies(100));
		dev_info(dev, "waiting for IQ status bit to clear\n");
	}
	/* Reset CPRIn_RACCR, CPRIn_TACCR */
	cpri_reg_clear(&regs->cpri_raccr, MASK_ALL);
	cpri_reg_clear(&regs->cpri_taccr, MASK_ALL);
	/* Reset CPRIn_MAP_CONFIG and CPRIn_MAP_TBL_CONFIG */
	cpri_reg_clear(&regs->cpri_map_config, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_tbl_config, MASK_ALL);

	return 0;
}

static int eth_reconfig(struct interface_reconf_param *recnf_parm,
			struct cpri_framer *framer)
{
	
	init_eth(framer);

	return 0;
}

static void link_reconfig(struct interface_reconf_param *recnf_parm,
		struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	struct cpri_autoneg_params *param = &framer->autoneg_param;
	struct cpri_autoneg_output *output = &(framer->autoneg_output);
	u32 rsr = 1, tsr = 1;
	u32 i = 0;
	u32 mask = 0;

	/* Clear bf data */
	output->cpri_bf_word_size = 0;
	output->cpri_bf_iq_datablock_size = 0;

	param->linerate_timeout = recnf_parm->params.linerate_timeout;
	param->proto_timeout = recnf_parm->params.proto_timeout;
	param->link_rate_low = recnf_parm->params.link_rate_low;
	param->link_rate_high = recnf_parm->params.link_rate_high;
	param->cnm_timeout = recnf_parm->params.cnm_timeout;

	memset(&framer->serdesspec, 0, sizeof(struct of_phandle_args));
	framer->serdes_handle = NULL;

	/* disable timing interrupt events */
	cpri_reg_clear(&regs->cpri_rctrltiminginten,
			BFN_TIMING_EVENT_EN_MASK
			| HFN_TIMING_EVENT_EN_MASK);

	cpri_reg_clear(&regs->cpri_tctrltiminginten,
			BFN_TIMING_EVENT_EN_MASK
			| HFN_TIMING_EVENT_EN_MASK);

	/* Disable framer interrupt */
	mask = ETH_EVENT_EN_MASK  | CONTROL_INT_LEVEL_MASK;

	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_rctrltiminginten,
			mask, 0);
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tctrltiminginten,
			mask, 0);

	mask = 0;
	mask = (RX_IQ_OVERRUN | TX_IQ_UNDERRUN |
		TX_VSS_UNDERRUN | RX_VSS_OVERRUN |
		ECC_CONFIG_MEM | ECC_DATA_MEM | RX_ETH_MEM_OVERRUN |
		TX_ETH_UNDERRUN | RX_ETH_BD_UNDERRUN | RX_ETH_DMA_OVERRUN |
		ETH_FORWARD_REM_FIFO_FULL);
	cpri_reg_write(&framer->regs_lock,
		 &framer->regs->cpri_errinten, mask, 0);

	/* clear protocol version and scr seed value */
	cpri_reg_clear(&regs->cpri_tprotver, PROTO_VER_MASK);
	cpri_reg_clear(&regs->cpri_tscrseed, SCR_SEED_MASK);

	/* Disable Tx and Rx*/
	cpri_reg_clear(&regs->cpri_config, CONF_RX_EN_MASK);
	cpri_reg_clear(&regs->cpri_config, CONF_TX_EN_MASK);
	/* Reset CPRIn_RACCR, CPRIn_TACCR, CPRIn_RCR & CPRIn_TCR */
	cpri_reg_clear(&regs->cpri_raccr, MASK_ALL);
	cpri_reg_clear(&regs->cpri_taccr, MASK_ALL);
	cpri_reg_clear(&regs->cpri_rcr, MASK_ALL);
	cpri_reg_clear(&regs->cpri_tcr, MASK_ALL);
	/* max 1 sec wait for CPRInRSR and CPRInTSR registers to clear */
	for (i = 0; i < 10; i++) {
		rsr = cpri_read(&regs->cpri_rstatus);
		tsr = cpri_read(&regs->cpri_tstatus);
		if ((rsr && tsr) == 0)
			break;
		schedule_timeout_interruptible(msecs_to_jiffies(100));
		dev_info(dev, "waiting for RSR and TSR to clear\n");
	}

	/* change state to configured*/
	cpri_state_machine(framer, CPRI_STATE_CONFIGURED);
}

void link_monitor(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	u8 cw_data[16];
	u32 reset_status;
	
	/* check proto version */
	read_rx_cw(framer, 2, cw_data);
	if(cw_data[0] != result->common_prot_ver) {
		framer->stats.current_event |= PROTO_VER_MISMATCH;
		wake_up_interruptible(&framer->event_queue);
	}

	/* check eth pointer */
	read_rx_cw(framer, 194, cw_data);
	if (cw_data[0] != result->common_eth_link_rate) {
		framer->stats.current_event |= ETH_PTR_MISMATCH;
		wake_up_interruptible(&framer->event_queue);
	}

	reset_status = cpri_read(&framer->regs->cpri_hwreset)
			& (RESET_DETECT_HOLD_MASK | RESET_DETECT_MASK);
	if (reset_status) {
		framer->stats.current_event |= RRE;
		wake_up_interruptible(&framer->event_queue);
	}
	
	mod_timer(&framer->link_poller, jiffies + HZ);
}

static void cpri_init_autoneg_timers(struct cpri_framer *framer)
{
	/* Initialise poller after operational mode */
	init_timer(&framer->link_poller);
	framer->link_poller.function = link_monitor;
	framer->link_poller.data = (unsigned long) framer;
	
	framer->link_poller.expires = jiffies + HZ;
	add_timer(&framer->link_poller);
}



static void set_autoneg_param(struct cpri_framer *framer,
		struct cpri_autoneg_params *param)
{
	cpri_state_machine(framer,
			CPRI_STATE_CONFIGURED);
}

void update_bf_data(struct cpri_framer *framer)
{
	struct cpri_autoneg_output *output = &(framer->autoneg_output);
	struct device *dev = framer->cpri_dev->dev;

	switch (output->common_link_rate) {
	case RATE2_1228_8M:
		output->cpri_bf_word_size = BF_W_SIZE_16;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_240;
		break;
	case RATE3_2457_6M:
		output->cpri_bf_word_size = BF_W_SIZE_32;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_480;
		break;
	case RATE4_3072_0M:
		output->cpri_bf_word_size = BF_W_SIZE_40;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_600;
		break;
	case RATE5_4915_2M:
		output->cpri_bf_word_size = BF_W_SIZE_64;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_960;
		break;
	case RATE6_6144_0M:
		output->cpri_bf_word_size = BF_W_SIZE_80;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_1200;
		break;
	case RATE7_9830_4M:
		output->cpri_bf_word_size = BF_W_SIZE_128;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_1920;
		break;
	default:
		dev_err(dev, "Invalid common link rate: %d\n",
				output->common_link_rate);
	break;
	}

}

void cpri_setup_vendor_autoneg(struct cpri_framer *framer)
{
	
	int buffer_size;
	int threshold;
	int axi_trans_size;
	int vss_int_en;
	int bytes_per_hf;
	struct cpri_autoneg_output *output = &(framer->autoneg_output);
	
	if (output->common_eth_link_rate < 20 &&
		output->common_eth_link_rate > 63)
		return;

	/* clean up vss first */
	vss_deconfig(framer);

	bytes_per_hf = output->cpri_bf_word_size / 8 *
			(output->common_eth_link_rate - 16) * 4;
	buffer_size = framer->autoneg_param.rx_vss_params.vss_buf_size * bytes_per_hf;
	threshold = framer->autoneg_param.rx_vss_params.vss_buf_thresh * bytes_per_hf;
	/* axi_trans_size = framer->autoneg_param.rx_vss_params.vss_axi_trans_size; */
	/* The driver selects a proper axi transaction size */
	if ((buffer_size % 128) == 0)
		axi_trans_size = 128;
	else if ((buffer_size % 64) == 0)
		axi_trans_size = 64;
	else if((buffer_size % 32) == 0)
		axi_trans_size = 32;
	else
		axi_trans_size = 16;

	vss_int_en = framer->autoneg_param.flags & CPRI_RX_VSS_INT_EN;
	/* Setup rx vss DMA */
	rx_vss_config(framer, buffer_size, threshold,
			axi_trans_size, vss_int_en);

	buffer_size = framer->autoneg_param.tx_vss_params.vss_buf_size * bytes_per_hf;
	threshold = framer->autoneg_param.tx_vss_params.vss_buf_thresh * bytes_per_hf;
	/* axi_trans_size = framer->autoneg_param.tx_vss_params.vss_axi_trans_size; */
	
	if ((buffer_size % 128) == 0)
		axi_trans_size = 128;
	else if ((buffer_size % 64) == 0)
		axi_trans_size = 64;
	else if((buffer_size % 32) == 0)
		axi_trans_size = 32;
	else
		axi_trans_size = 16;

	vss_int_en = framer->autoneg_param.flags & CPRI_TX_VSS_INT_EN;
	/* Setup tx vss DMA */
	tx_vss_config(framer, buffer_size, threshold,
			axi_trans_size, vss_int_en);

	cpri_state_machine(framer,
		CPRI_STATE_AUTONEG_COMPLETE);
}


static void eth_setup_timer_expiry_hndlr(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	framer->timer_expiry_events = TEVENT_ETHRATE_SETUP_TEXPIRED;
	 /* TBD: Notify user */
}

void cpri_eth_autoneg(struct work_struct *work)
{
	struct cpri_framer *framer = container_of(work, struct cpri_framer,
							ethautoneg_task);
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct timer_list eth_setup_timer;
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	u8 tx_eth_rate;
	u32 cmconfig;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);


	/* Init eth rate expiry timer */
	init_timer(&eth_setup_timer);
	eth_setup_timer.function = eth_setup_timer_expiry_hndlr;
	eth_setup_timer.data = (unsigned long) framer;

	/* Turn ON Tx */
	cpri_reg_set(&regs->cpri_config, CONF_TX_EN_MASK);
	framer->timer_expiry_events = 0;

	/* Setup eth rate expiry timer */
	eth_setup_timer.expires = jiffies + (param->cnm_timeout) * HZ;
	add_timer(&eth_setup_timer);

	tx_eth_rate = param->eth_rate;
	cpri_reg_set_val(&regs->cpri_cmconfig,
			TX_FAST_CM_PTR_MASK, (u32)tx_eth_rate);

	while (framer->timer_expiry_events 
			!= TEVENT_ETHRATE_SETUP_TEXPIRED) {
		cmconfig = cpri_reg_get_val(
			&regs->cpri_cmstatus,
			MASK_ALL);
		if (cmconfig & RX_FAST_CM_PTR_VALID_MASK) {
			tx_eth_rate = cmconfig & RX_FAST_CM_PTR_MASK;
			cpri_reg_set_val(&regs->cpri_cmconfig,
				TX_FAST_CM_PTR_MASK, tx_eth_rate);
			break;
		}
		schedule_timeout_interruptible(msecs_to_jiffies(1));
		
	}
	if (framer->timer_expiry_events 
			== TEVENT_ETHRATE_SETUP_TEXPIRED) {
		dev_info(dev, "C&M autoneg fail\n");
		framer->stats.cnm_auto_neg_failures++;
		framer->dev_flags |= CPRI_PASSIVE_LINK;
		framer->timer_expiry_events = 0;
	} else {
		result->common_eth_link_rate = tx_eth_rate;
		dev_info(dev, "C&M autoneg success, eth ptr:%d\n",
			result->common_eth_link_rate);
	}
	del_timer_sync(&eth_setup_timer);
	/* do vendor config autoneg */
	if ((framer->framer_state == CPRI_STATE_PROT_VER_AUTONEG)) {
		cpri_state_machine(framer,
			CPRI_STATE_ETH_RATE_AUTONEG);

		/* Setup ethernet DMA here */
		if (framer->frmr_ethflag & CPRI_ETH_SUPPORTED)
			cpri_eth_enable(framer);

		cpri_init_autoneg_timers(framer);
		cpri_setup_vendor_autoneg(framer);
	}
}

static void proto_setup_timer_expiry_hndlr(unsigned long ptr)
{

	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	framer->timer_expiry_events = TEVENT_PROTVER_SETUP_TEXPIRED;
	/* TBD: Notify user */
}

void cpri_proto_ver_autoneg(struct work_struct *work)
{
	struct cpri_framer *framer = container_of(work, struct cpri_framer,
						protoautoneg_task);
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	struct cpri_framer_regs  __iomem *regs = framer->regs;
	struct timer_list proto_setup_timer;
	struct device *dev = framer->cpri_dev->dev;
	enum cpri_prot_ver tproto_ver, rproto_ver;
	unsigned long timer_dur;
	u8 rx_scr_en;
	u32 tx_seed;

	/* Setup protocol ver expiry timer */
	init_timer(&proto_setup_timer);
	proto_setup_timer.function = proto_setup_timer_expiry_hndlr;
	timer_dur = jiffies + param->proto_timeout * HZ;
	proto_setup_timer.expires = timer_dur;
	proto_setup_timer.data = (unsigned long) framer;
	framer->timer_expiry_events = 0;
	add_timer(&proto_setup_timer);

	/* Turn ON Tx */
	cpri_reg_set(&regs->cpri_config, CONF_TX_EN_MASK);

	/* Lower lane rate starts on VER_1 */
	/* or the user specify to use VER_1 */
	if ((result->common_link_rate < RATE5_4915_2M) ||
			(param->tx_prot_ver == VER_1)) {
		tproto_ver = VER_1;
		tx_seed = 0;
	} else {
		tproto_ver = VER_2;
		tx_seed = param->tx_scr_seed;
	}

	cpri_reg_set_val(&regs->cpri_tprotver,
				PROTO_VER_MASK, tproto_ver);
	cpri_reg_set_val(&regs->cpri_tscrseed,
				SCR_SEED_MASK, tx_seed);

	rx_scr_en = (u8) cpri_reg_get_val(&regs->cpri_rscrseed,
				RX_SCR_EN_MASK);
	rproto_ver = (rx_scr_en == 0) ? VER_1 : VER_2;

	if ((rproto_ver == VER_1) && (tproto_ver == VER_2)) {
		tproto_ver = VER_1;
		tx_seed = 0;
		cpri_reg_set_val(&regs->cpri_tprotver,
			PROTO_VER_MASK, tproto_ver);
		cpri_reg_set_val(&regs->cpri_tscrseed,
			SCR_SEED_MASK, tx_seed);
	}
	while (framer->timer_expiry_events 
			!= TEVENT_PROTVER_SETUP_TEXPIRED) {
		rx_scr_en = (u8) cpri_reg_get_val(&regs->cpri_rscrseed,
				RX_SCR_EN_MASK);
		rproto_ver = (rx_scr_en == 0) ? VER_1 : VER_2;
		if (rproto_ver == tproto_ver)
			break;
		else
			schedule_timeout_interruptible(msecs_to_jiffies(1));
		
	}
	del_timer_sync(&proto_setup_timer);
	if (framer->timer_expiry_events 
			== TEVENT_PROTVER_SETUP_TEXPIRED) {
		dev_err(dev, "protocal autoneg failure!");
		framer->timer_expiry_events = 0;
	} else {
		result->common_prot_ver = tproto_ver;
		dev_info(dev, "protocol autoneg success,prot ver:%d\n",
				result->common_prot_ver);
	}
	cpri_state_machine(framer,
			CPRI_STATE_PROT_VER_AUTONEG);
}



static int check_linesync(struct cpri_framer *framer, enum cpri_link_rate rate)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	int err = 0;
	u32 mask;
	u32 line_sync_acheived;
	u8 cw_buf[16];
	char *rate_name[] = {"1228.8", "2457.6",
				"3072.0", "4915.2",
				"6144.0", "9830.4"};

	cpri_reg_set(&regs->cpri_config,
				CONF_RX_EN_MASK);

	cpri_reg_set(&regs->cpri_config,
				CONF_TX_EN_MASK);
	set_sfp_txdisable(framer->sfp_dev, 0);
	
	mask = (RX_HFN_STATE_MASK | RX_BFN_STATE_MASK);
	/* Wait for RAI to be cleared */
	while (framer->timer_expiry_events != TEVENT_LINERATE_SETUP_TEXPIRED) {
		line_sync_acheived = cpri_reg_get_val(
					&regs->cpri_status, mask);
		if (line_sync_acheived == 3) {
			read_rx_cw(framer, 130, cw_buf);
			if (!(cw_buf[0] & 0x2))
				break;
		}
		schedule_timeout_interruptible(msecs_to_jiffies(2));
	}
	if (framer->timer_expiry_events == TEVENT_LINERATE_SETUP_TEXPIRED)
		err = -ETIME;
	if (!err) {
		result->common_link_rate = rate;
		dev_info(dev, "line rate autoneg success at %s mbps",
				rate_name[rate - 1]);
	}

	return err;
}

static void linerate_setup_timer_expiry_hndlr(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	framer->timer_expiry_events = TEVENT_LINERATE_SETUP_TEXPIRED;
}

void cpri_linkrate_autoneg(struct work_struct *work)
{
	struct cpri_framer *framer = container_of(work, struct cpri_framer,
						lineautoneg_task);
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct device *dev = framer->cpri_dev->dev;
	struct timer_list linerate_timer;
	enum cpri_link_rate rate;
	enum cpri_link_rate low = param->link_rate_low;
	enum cpri_link_rate high = param->link_rate_high;
	int err = 0;
	unsigned long timer_dur;
	struct device_node *child = NULL;
	unsigned int framer_id;

	for_each_child_of_node(framer->cpri_node, child) {

		of_property_read_u32(child, "framer-id", &framer_id);
		if (framer_id < 0) {
			dev_err(dev, "Failed to get framer id\n");
			return;
		}
		if (framer_id == framer->id)
			break;
	}
	if (of_get_named_serdes(child, &framer->serdesspec,
			"serdes-handle", 0)) {
			dev_err(dev, "Failed to get serdes-handle\n");
			return;
	}
	framer->serdes_handle = get_attached_serdes_dev(framer->serdesspec.np);
	if (framer->serdes_handle == NULL) {
		dev_err(dev, "Failed to get serdes handle\n");
		return;
	}

	del_timer(&framer->link_poller);
	/* Setup line rate timer */
	init_timer(&linerate_timer);
	linerate_timer.function = linerate_setup_timer_expiry_hndlr;
	linerate_timer.data = (unsigned long) framer;

	for (rate = high; rate >= low; rate--) {
		dev_dbg(dev, "trying new link rate %d\n", rate);
		cpri_state_machine(framer,
				CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS);
		framer->cpri_dev->intr_cpri_frmr_state =
				CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS;

		if (linkrate_autoneg_reset(framer, rate)) {
			dev_err(dev, "line autoneg reset failed\n");
			cpri_state_machine(framer,
					CPRI_STATE_LINK_ERROR);
			return;
		} else
			dev_dbg(dev, "Pass line autoneg reset\n");

		mdelay(1);

		/* Enable cpri complex interrupt here */
		/* cpri_interrupt_enable(framer->cpri_dev); */

		/* enable framer interrupt */
		cpri_init_framer(framer, rate);
		/* Start timer */
		timer_dur = jiffies + (param->linerate_timeout) * HZ;
		linerate_timer.expires = timer_dur;
		add_timer(&linerate_timer);

		/* check for sync acheived */
		if ((err = check_linesync(framer, rate)) == 0)
			break;

		/* timer will be destroyed for every new line rate */
		del_timer_sync(&linerate_timer);
		framer->timer_expiry_events = 0;
	}
	del_timer_sync(&linerate_timer);

	if (err != 0) {
		framer->stats.l1_auto_neg_failures++;
		dev_err(dev, "line rate autoneg failed\n");
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
	} else {
		cpri_eth_parm_init(framer);
		update_bf_data(framer);
		err = init_framer_axc_param(framer);
		if (err != 0)
			dev_err(dev, "init axc param failed !\n");
		cpri_state_machine(framer, CPRI_STATE_LINE_RATE_AUTONEG);
	}
}

void cpri_autoneg_all(struct work_struct *work)
{
	struct cpri_framer *framer = container_of(work, struct cpri_framer,
						allautoneg_task);
	enum cpri_state *state = &(framer->framer_state);


	framer->timer_expiry_events = 0;

	while (*state != CPRI_STATE_AUTONEG_COMPLETE) {
		/* do line rate autoneg */
		cpri_linkrate_autoneg(&framer->lineautoneg_task);
		if (*state != CPRI_STATE_LINE_RATE_AUTONEG)
			break;
		/* do proto ver autoneg */
		cpri_proto_ver_autoneg(&framer->protoautoneg_task);
		if (*state != CPRI_STATE_PROT_VER_AUTONEG)
			break;

		/* do eth rate autoneg */
		cpri_eth_autoneg(&framer->ethautoneg_task);
		if (*state != CPRI_STATE_ETH_RATE_AUTONEG)
			break;
		else {
			cpri_state_machine(framer,
				CPRI_STATE_AUTONEG_COMPLETE);
			break;
		}
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}
	
}

static s32 process_recnfg(struct interface_reconf_param *recnf_parm,
			struct cpri_framer *framer)
{
	s32 retval = 0;
	struct device *dev = framer->cpri_dev->dev;
	enum interface_recfg_cmd recnfg_cmd = recnf_parm->recnfg_cmd;

	if (recnfg_cmd == CPRI_LINK_RECONFIG_INIT_REQ ||
			recnfg_cmd == CPRI_ETH_INTERFACE_RECONFIG_INIT_REQ ||
			recnfg_cmd == CPRI_AXC_INTERFACE_RECONFIG_INIT_REQ) {
		clear_axc_buff(framer);
	}

	switch (recnfg_cmd) {
	case CPRI_LINK_RECONFIG_INIT_REQ:
		link_reconfig(recnf_parm, framer);
		/* TODO: Notify user-link is now ready for link reconfig.
		 * This will be updated once the new notification mechanism
		 * implemented
		 */
		break;
	case CPRI_ETH_INTERFACE_RECONFIG_INIT_REQ:
		retval = eth_reconfig(recnf_parm, framer);
		/* TODO: Notify user-link is now ready for interface reconfig.
		 * This will be updated once the new notification mechanism
		 * implemented
		 */
		break;
	case CPRI_VSS_INTERFACE_RECONFIG_INIT_REQ:
		vss_reconfig(recnf_parm, framer);
		/* TODO: Notify user-link is now ready for vendor specific
		 * reconfig. this will be updated once the new notification
		 * mechanism implemented
		 */
		break;
	case CPRI_AXC_INTERFACE_RECONFIG_INIT_REQ:
		retval = axc_reconfig(recnf_parm, framer);
		/* TODO: Notify user-link is now ready for axc reconfig,
		 * this will be updated once the new notification mechanism
		 * implemented
		 */
		break;
	default:
		retval = -EINVAL;
		dev_info(dev, "got invalid reconfig cmd: %d\n", recnfg_cmd);
	}

	return retval;
}

static int process_autoneg_cmd(enum autoneg_cmd cmd, struct cpri_framer *framer)
{
	int retval = 0;
	struct device *dev = framer->cpri_dev->dev;

	switch (cmd) {
	case CPRI_DO_LINE_RATE_AUTONEG:
		schedule_work(&framer->lineautoneg_task);
		break;
	case CPRI_DO_PROTOCOL_AUTONEG:
		schedule_work(&framer->protoautoneg_task);
		break;
	case CPRI_DO_CNM_AUTONEG:
		schedule_work(&framer->ethautoneg_task);
		break;
	case CPRI_DO_AUTONEG_ALL:
		schedule_work(&framer->allautoneg_task);
		break;
	default:
		retval = -EINVAL;
		dev_info(dev, "got invalid autoneg cmd: %d\n", cmd);
	}

	return retval;
}

/* Function to process autoneg related commands from user */
int cpri_autoneg_ioctl(struct cpri_framer *framer, unsigned int cmd,
				unsigned long arg)
{
	struct cpri_autoneg_params *param = &framer->autoneg_param;
	struct cpri_autoneg_params autoneg_param;
	struct cpri_autoneg_output *output = &framer->autoneg_output;
	enum autoneg_cmd an_cmd;
	struct interface_reconf_param interface_recfg_param;
	struct device *dev = framer->cpri_dev->dev;
	void __user *ioargp = (void __user *)arg;
	int err;

	switch (cmd) {
	case CPRI_START_AUTONEG:
		if (copy_from_user(&an_cmd, (enum autoneg_cmd *)ioargp,
				sizeof(enum autoneg_cmd)) != 0) {
			err = -EFAULT;
			goto out;
		}

		err = process_autoneg_cmd(an_cmd, framer);
		if (err != 0)
			goto out;

		break;

	case CPRI_START_RECONFIG:
		if (copy_from_user(&interface_recfg_param,
				(struct interface_reconf_param *)ioargp,
				sizeof(struct interface_reconf_param)) != 0) {
			err = -EFAULT;
			goto out;
		}

		err = process_recnfg(&interface_recfg_param, framer);
		if (err != 0)
			goto out;

		break;

	case CPRI_INIT_AUTONEG_PARAM:
		if (copy_from_user(param, (struct cpri_autoneg_params *)ioargp,
				sizeof(struct cpri_autoneg_params)) != 0) {
			err = -EFAULT;
			goto out;
		}
		set_autoneg_param(framer, param);
		break;

	case CPRI_GET_AUTONEG_PARAM:
		if (copy_from_user(&autoneg_param,
				(struct cpri_autoneg_params *)ioargp,
				sizeof(struct cpri_autoneg_params))) {
			err = -EFAULT;
			goto out;
		}
		autoneg_param.flags = param->flags;
		autoneg_param.linerate_timeout = param->linerate_timeout;
		autoneg_param.link_rate_low = param->link_rate_low;
		autoneg_param.link_rate_high = param->link_rate_high;
		autoneg_param.cnm_timeout = param->cnm_timeout;
		autoneg_param.proto_timeout = param->proto_timeout;
		autoneg_param.tx_prot_ver = param->tx_prot_ver;
		autoneg_param.tx_scr_seed = param->tx_scr_seed;

		if (copy_to_user((struct cpri_autoneg_params *)ioargp,
			&autoneg_param, sizeof(struct cpri_autoneg_params))) {
			err = -EFAULT;
			goto out;
		}

		break;

	case CPRI_GET_AUTONEG_OUTPUT:
		if (copy_to_user(ioargp,
			(struct cpri_autoneg_output *)output,
			sizeof(struct cpri_autoneg_output))) {
			err = -EFAULT;
			goto out;
		}

		break;

	case CPRI_SET_RAWDELAY:
		if (copy_from_user(&framer->delay_cfg,
			(struct cpri_delays_raw_cfg *)ioargp,
			sizeof(struct cpri_delays_raw_cfg)) != 0) {
			err = -EFAULT;
			goto out;
		}
		set_delay_config(framer);

		break;

	case CPRI_GET_RAWDELAY:
		if (copy_to_user(ioargp,
			(struct cpri_delays_raw *)&framer->delay_out,
			sizeof(struct cpri_delays_raw))) {
			err = -EFAULT;
			goto out;
		}

		break;

	default:
		err = -EINVAL;
		dev_info(dev, "got invalid autoneg cmd: %d\n", cmd);
		goto out;
	}

	return 0;

out:
	dev_err(dev, "autoneg ioctl error\n");
	return err;
}

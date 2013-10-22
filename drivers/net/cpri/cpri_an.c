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
#include <mach/serdes-d4400.h>

int linkrate_autoneg_reset(struct cpri_framer *framer,
		enum cpri_link_rate linerate)
{
	struct serdes_pll_params pll_param;
	struct serdes_lane_params lane_param;
	struct cpri_framer *pair_framer;
	int serdes_init;
	u32 line_rate[7] = {0, 1228800, 2457600, 3072000, 4915200,
		6144000, 9830400};

	if (framer->id == 1)
		pair_framer = framer->cpri_dev->framer[1];
	else
		pair_framer = framer->cpri_dev->framer[0];

	pll_param.pll_id = SERDES_PLL_1;
	pll_param.rfclk_sel = REF_CLK_FREQ_122_88_MHZ;
	if ((linerate ==  RATE4_3072_0M) || (linerate == RATE6_6144_0M)) {
		pll_param.frate_sel = PLL_FREQ_3_072_GHZ;
		pll_param.vco_type = SERDES_RING_VCO;
	} else {
		pll_param.frate_sel = PLL_FREQ_4_9152_GHZ;
		pll_param.vco_type = SERDES_LC_VCO;
	}
	/* cpri hardware support common link rate for both framers
	 * below logic will take care of if either of cpri framer is
	 * up than next will follow first framer
	 */
	if (pair_framer->framer_state <
			CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS) {
		gcr_clear_cpri_line_rate(framer->cpri_dev->dev_id);
		gcr_set_cpri_line_rate(framer->cpri_dev->dev_id, linerate);
		gcr_linkrate_autoneg_reset(framer->cpri_dev->dev_id);
		serdes_init = serdes_init_pll(framer->serdes_handle,
				&pll_param);
		if ((serdes_init != -EALREADY) && (serdes_init != 0))
			return -EINVAL;
	}
	lane_param.lane_id = framer->serdesspec.args[0];
	lane_param.grp_prot = SERDES_PROT_CPRI;
	lane_param.gen_conf.lane_prot = SERDES_LANE_PROTS_CPRI;
	lane_param.gen_conf.bit_rate_kbps = line_rate[linerate];
	lane_param.gen_conf.cflag = (SERDES_20BIT_EN |  SERDES_TPLL_LES |
			SERDES_RPLL_LES | SERDES_FIRST_LANE);
	if (framer->autoneg_param.flags & CPRI_SERDES_LOOPBACK)
		lane_param.gen_conf.cflag |= SERDES_LOOPBACK_EN;

	if (serdes_init_lane(framer->serdes_handle, &lane_param))
		return -EINVAL;


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
			| HFN_TIMING_EVENT_EN_MASK);

	cpri_reg_set(&regs->cpri_tctrltiminginten,
			BFN_TIMING_EVENT_EN_MASK
			| HFN_TIMING_EVENT_EN_MASK);

	/* TBD: CPRIICR is not set in this driver. It is not clear
	 * why we have this physical interrupt line and the similar
	 * configuration like the above
	 */
	/* Enable all error events by default */
#if 0 /* temporarily commented on medusa bcz of clock error hang issue */
	cpri_reg_set(&regs->cpri_errinten,
			CPRI_ERR_EVT_ALL);
#endif
	framer_int_enable(framer);
}


static void cpri_init_framer(struct cpri_framer *framer)
{
	struct cpri_dev_init_params *param = &framer->framer_param;
	struct cpri_framer_regs __iomem *regs = framer->regs;

	/* Rx scrambler setting */
	if (framer->autoneg_param.flags & CPRI_RX_SCRAMBLER_EN)
		cpri_reg_set(&regs->cpri_rscrseed,
				RX_SCR_EN_MASK);

	/* Disable Tx and Rx AxCs */
	cpri_reg_set_val(&regs->cpri_raccr,
		MASK_ALL, 0);

	cpri_reg_set_val(&regs->cpri_taccr,
			MASK_ALL, 0);

	cpri_reg_clear(&regs->cpri_cmconfig, MASK_ALL);

	/* Clear all control interrupt events */
	cpri_reg_clear(&regs->cpri_rcr,
		ETH_EN_MASK | VSS_EN_MASK | IQ_EN_MASK);

	cpri_reg_clear(&regs->cpri_tcr,
		ETH_EN_MASK | VSS_EN_MASK | IQ_EN_MASK);

	if (param->ctrl_flags & CPRI_DAISY_CHAINED)
		cpri_reg_set(&regs->cpri_auxctrl,
				AUX_MODE_MASK);
	else
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

#if 0
	/* SRC config param */
	if (param->ctrl_flags & RST_REQ_BYP)
		src_reset_request_bypass(framer->src_bypass_status,
				framer->src_bypass_dur_sec);
#endif

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
		cpri_reg_clear(&regs->cpri_rcr,
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

	/* Tx framer size setting */
	cpri_reg_set_val(&regs->cpri_tbufsize,
			FR_BUF_SIZE_MASK,
			param->tx_framer_buffer_size);

	/* VSS AXI transaction size setting */
	cpri_reg_set_val(&regs->cpri_rvssaxisize,
			AXI_TRANSAC_SIZE_MASK,
			param->axi_vss_rx_trans_size);
	cpri_reg_set_val(&regs->cpri_tvssaxisize,
			AXI_TRANSAC_SIZE_MASK,
			param->axi_vss_tx_trans_size);

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

static void interface_reconfig(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;

	/* Disable Tx and Rx */
	cpri_reg_clear(&regs->cpri_config,
			CONF_TX_EN_MASK|CONF_RX_EN_MASK);

	/* Reset CPRIn_RACCR, CPRIn_TACCR, CPRIn_RCR & CPRIn_TCR */
	cpri_reg_clear(&regs->cpri_raccr, 0xFFFFFFFF);
	cpri_reg_clear(&regs->cpri_taccr, 0xFFFFFFFF);
	cpri_reg_clear(&regs->cpri_rcr, 0xFFFFFFFF);
	cpri_reg_clear(&regs->cpri_tcr, 0xFFFFFFFF);

	/* Reset CPRIn_MAP_CONFIG and CPRIn_MAP_TBL_CONFIG */
	cpri_reg_clear(&regs->cpri_map_config, 0xFFFFFFFF);
	cpri_reg_clear(&regs->cpri_map_tbl_config, 0xFFFFFFFF);
}

static void link_reconfig(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	u32 rsr = 1, tsr = 1;
	int i;

	/* Disable Tx and Rx*/
	cpri_reg_clear(&regs->cpri_config, CONF_RX_EN_MASK);
	cpri_reg_clear(&regs->cpri_config, CONF_TX_EN_MASK);
	/* Reset CPRIn_RACCR, CPRIn_TACCR, CPRIn_RCR & CPRIn_TCR */
	cpri_reg_clear(&regs->cpri_raccr, 0xFFFFFFFF);
	cpri_reg_clear(&regs->cpri_taccr, 0xFFFFFFFF);
	cpri_reg_clear(&regs->cpri_rcr, 0xFFFFFFFF);
	cpri_reg_clear(&regs->cpri_tcr, 0xFFFFFFFF);
	/* max 1 sec wait for CPRInRSR and CPRInTSR registers to clear */
	for (i = 0; i < 10; i++) {
		rsr = cpri_read(&regs->cpri_rstatus);
		tsr = cpri_read(&regs->cpri_tstatus);
		if ((rsr && tsr) == 0)
			break;
		schedule_timeout_interruptible(msecs_to_jiffies(100));
		dev_info(dev, "waiting for RSR and TSR to clear\n");
	}
}

void link_monitor(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;
	u8 tx_eth_rate, rx_eth_rate;
	enum cpri_prot_ver rx_prot_ver, tx_prot_ver;
	int err = 0;

	get_txprotver(framer, &tx_prot_ver);
	get_rxprotver(framer, &rx_prot_ver);
	if (rx_prot_ver == tx_prot_ver) {
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
		 /* TBD: Notify user */
		err = -ENOLINK;
	}

	get_txethrate(framer, &tx_eth_rate);
	get_rxethrate(framer, &rx_eth_rate);
	if (tx_eth_rate == rx_eth_rate) {
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
		 /* TBD: Notify user */
		err = -ENOLINK;
	}

	if (err == -ENOLINK) {
		del_timer_sync(&framer->link_poller);
	} else {
		mod_timer(&framer->link_poller,
			jiffies + framer->link_poll_dur_sec * HZ);
	}
}

void ethptr_monitor(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	enum cpri_prot_ver rx_prot_ver, tx_prot_ver;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	u32 valid_ptr;

	get_txprotver(framer, &tx_prot_ver);
	get_rxprotver(framer, &rx_prot_ver);
	if (rx_prot_ver != tx_prot_ver) {
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
		del_timer_sync(&framer->ethptr_poller);
		/* TBD: Notify  user */
	}

	if (framer->framer_param.ctrl_flags & CPRI_DEV_SLAVE) {
		set_txethrate(param->eth_rates[param->eth_rates_count],
			framer);
		valid_ptr = cpri_reg_get_val(
			&regs->cpri_cmstatus, RX_FAST_CM_PTR_VALID_MASK);
		if (valid_ptr) {
			cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
			del_timer_sync(&framer->ethptr_poller);
			/* TBD: Notify  user */
		}
	} else {
		set_txethrate(0, framer);
	}

	mod_timer(&framer->ethptr_poller,
		jiffies + framer->passive_poll_dur_sec * HZ);
}

void l1_timer_expiry_hndlr(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	framer->timer_expiry_events = TEVENT_LITIMER_EXPIRED;
	/* TBD: Notify user */
}

static void cpri_init_autoneg_timers(struct cpri_framer *framer)
{
	/* Initialise L1 timer */
	init_timer(&framer->l1_timer);
	framer->l1_timer.function = l1_timer_expiry_hndlr;
	framer->l1_timer.data = (unsigned long) framer;

	/* Initialise poller for operational mode */
	init_timer(&framer->link_poller);
	framer->link_poller.function = link_monitor;
	framer->link_poller.data = (unsigned long) framer;

	/* Initialise poller for passive link mode */
	init_timer(&framer->ethptr_poller);
	framer->ethptr_poller.function = ethptr_monitor;
	framer->ethptr_poller.data = (unsigned long) framer;
}

static void set_autoneg_param(struct cpri_framer *framer,
		struct cpri_autoneg_params *param)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;

	/* Rx scrambler setting */
	if (param->flags & CPRI_RX_SCRAMBLER_EN)
		cpri_reg_set(&regs->cpri_rscrseed,
				RX_SCR_EN_MASK);
	framer->l1_expiry_dur_sec = param->l1_setup_timeout;
	cpri_init_autoneg_timers(framer);

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
#if 0	/* need to test with control word testing */
	enum cpri_prot_ver tx_prot_ver, rx_prot_ver;
	struct device *dev = framer->cpri_dev->dev;

	/* Check Tx/Rx protocol version */
	get_txprotver(framer, &tx_prot_ver);
	get_rxprotver(framer, &rx_prot_ver);
	if (tx_prot_ver != rx_prot_ver) {
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
		/* TBD: Notify  user */
		framer->stats.vendor_config_failures++;
		dev_err(dev, "cpri prot version mismatch\n");
		return;
	}
#endif


/* end here */
	/* Setup ethernet DMA here */
	if(framer->frmr_ethflag == CPRI_ETH_SUPPORTED)
		cpri_eth_enable(framer);
	cpri_state_machine(framer,
			CPRI_STATE_AUTONEG_COMPLETE);
}

static int check_ethrate(struct cpri_framer *framer)
{
	struct cpri_framer_regs *regs = framer->regs;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	struct device *dev = framer->cpri_dev->dev;
	u32 tx_eth_rate, rx_eth_rate;
	int status, err = 0;
	u8 hf_cnt;
	static u8 txhfcnt;


	txhfcnt = (u8) cpri_reg_get_val(
			&regs->cpri_thfnctr, TX_HFN_COUNTER_MASK);

	/* if c&m ptr invalid */
	while (1) {
		hf_cnt = (u8) cpri_reg_get_val(
			&regs->cpri_thfnctr, TX_HFN_COUNTER_MASK);
		/* Check whether two HF passed */
		hf_cnt -= txhfcnt;
		if (hf_cnt >= 5) {
			status = cpri_reg_get_val(
				&regs->cpri_cmstatus,
				RX_FAST_CM_PTR_VALID_MASK);
			rx_eth_rate = cpri_reg_get_val(
					&regs->cpri_cmstatus,
					RX_FAST_CM_PTR_MASK);
			if ((status) && (rx_eth_rate >= CPRI_ETH_PTR_MIN))
				break;
			txhfcnt = (u8) cpri_reg_get_val(
				&regs->cpri_thfnctr, TX_HFN_COUNTER_MASK);
		}
		if (framer->timer_expiry_events ==
				TEVENT_ETHRATE_SETUP_TEXPIRED) {
			err = -ETIME;
			goto out;
		}
		dev_dbg(dev, "waiting for atleast 2 HF\n");
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}


	if (status) {
		tx_eth_rate = cpri_reg_get_val(
					&regs->cpri_cmconfig,
					TX_FAST_CM_PTR_MASK);
		mdelay(2);

		rx_eth_rate = cpri_reg_get_val(
					&regs->cpri_cmstatus,
					RX_FAST_CM_PTR_MASK);

		if (tx_eth_rate == rx_eth_rate) {
			result->common_eth_link_rate = rx_eth_rate;
			dev_info(dev, "eth autoneg success\n");
			err = 0;
		} else {
			err = -ENOLINK;
			dev_info(dev, "eth autoneg fail:txPtr: %d, txPtr: %d\n",
				tx_eth_rate, rx_eth_rate);
		}
	}

out:
	return err;
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
	struct cpri_dev *cpdev = framer->cpri_dev;
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct timer_list eth_setup_timer;
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	u8 tx_eth_rate;
	int i, err = 0;

	/* Init eth rate expiry timer */
	init_timer(&eth_setup_timer);
	eth_setup_timer.function = eth_setup_timer_expiry_hndlr;
	eth_setup_timer.data = (unsigned long) framer;

	/* Turn ON Tx */
	cpri_reg_set(&regs->cpri_config, CONF_TX_EN_MASK);


	framer->timer_expiry_events = 0;

	for (i = param->eth_rates_count; i >= 1 ; i--) {
		tx_eth_rate = param->eth_rates[i - 1];
		cpri_reg_set_val(
			&regs->cpri_cmconfig, TX_FAST_CM_PTR_MASK,
			(u32)tx_eth_rate);
		/* Setup eth rate expiry timer */
		eth_setup_timer.expires = jiffies + (param->cnm_timeout) * HZ;
		add_timer(&eth_setup_timer);
		err = check_ethrate(framer);
		if (err == -ENOLINK) {
			dev_info(dev, "eth err -ENOLINK");
			continue;
		} else if (err == -ETIME) {
			dev_info(dev, "eth autoneg timeout\n");
			goto out_fail;
		} else
			goto out_pass;
	}

out_fail:
	dev_info(dev, "eth autoneg fail\n");
	framer->stats.cnm_auto_neg_failures++;
	framer->dev_flags |= CPRI_PASSIVE_LINK;
	cpri_state_machine(framer,
			CPRI_STATE_LINK_ERROR);

out_pass:
	cpri_reg_set(&cpdev->regs->cpri_intctrl[0],
			IEVENT_ETH_MASK);
	del_timer_sync(&eth_setup_timer);
	/* do vendor config autoneg */
	cpri_state_machine(framer,
			CPRI_STATE_ETH_RATE_AUTONEG);
	cpri_setup_vendor_autoneg(framer);
	/* Set framer state */
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
	enum cpri_prot_ver rproto_ver = VER_1, tproto_ver = VER_2;
	unsigned long timer_dur;
	int err = -ETIME;
	u8 hf_cnt;
	u32 pver;
	static u8 txhfcnt;

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
	cpri_reg_set_val(&regs->cpri_tprotver,
			PROTO_VER_MASK, 2);
	txhfcnt = (u8) cpri_reg_get_val(&regs->cpri_thfnctr,
					TX_HFN_COUNTER_MASK);


	while (framer->timer_expiry_events != TEVENT_PROTVER_SETUP_TEXPIRED) {

		/* Update the received scramble seed value */
		result->rx_scramble_seed_val =
			cpri_reg_get_val(&regs->cpri_rscrseed, SCR_SEED_MASK);


		/* Check whether two HF passed */
		hf_cnt = (u8) cpri_reg_get_val(&regs->cpri_thfnctr,
					TX_HFN_COUNTER_MASK);
		hf_cnt -= txhfcnt;
		if (hf_cnt >= 2) {
			dev_info(dev, "proto ver - 2HF passed\n");
			/* Get last set proto ver */
			pver = cpri_reg_get_val(
					&regs->cpri_tprotver, PROTO_VER_MASK);
			tproto_ver = (pver == 1) ? VER_1 : VER_2;
			get_rxprotver(framer, &rproto_ver);
			if (rproto_ver == tproto_ver) {
				result->common_prot_ver  = tproto_ver;
				err = 0;
				dev_info(dev, "%d:%d protocol autoneg success",
						framer->cpri_dev->dev_id,
						framer->id);
				break;
			} else {
				/* Change Tx proto ver */
				if (tproto_ver == VER_1)
					cpri_reg_set_val(&regs->cpri_tprotver,
							PROTO_VER_MASK, 2);
				else
					cpri_reg_set_val(&regs->cpri_tprotver,
							PROTO_VER_MASK, 1);
				txhfcnt = (u8) cpri_reg_get_val(
						&regs->cpri_thfnctr,
						TX_HFN_COUNTER_MASK);
			}
		}
		dev_dbg(dev, "proto ver autoneg in progress\n");
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}

	if (framer->timer_expiry_events == TEVENT_PROTVER_SETUP_TEXPIRED)
		dev_info(dev, "proto ver autoneg timer expired\n");
	del_timer_sync(&proto_setup_timer);
	if (err != 0) {
		framer->stats.proto_auto_neg_failures++;
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
	} else {
		cpri_state_machine(framer,
				CPRI_STATE_PROT_VER_AUTONEG);
	}

}

static int cpri_stable(struct cpri_framer *framer, enum cpri_link_rate rate)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	struct device *dev = framer->cpri_dev->dev;
	u8 tx_eth_rate;
	int err = 0;
	static u8 txhfcnt;

	/* update the common line rate between REC and RE */
	result->common_link_rate = rate;

	/* Update Tx proto ver and Tx scrambler seed */
	if (rate > RATE4_3072_0M) {
		cpri_reg_set_val(&regs->cpri_tprotver,
				PROTO_VER_MASK, 2);

		cpri_reg_set_val(&regs->cpri_tscrseed,
				SCR_SEED_MASK, param->tx_scr_seed);
	} else
		cpri_reg_set_val(&regs->cpri_tprotver,
				PROTO_VER_MASK, 1);

	/* Set max Tx ethernet rate/ptr */
	tx_eth_rate = param->eth_rates[param->eth_rates_count-1];

	/* Turn on Tx and enable tx for 1 HF */
	cpri_reg_set(&regs->cpri_config, CONF_TX_EN_MASK);
	txhfcnt = 0;
	while (framer->timer_expiry_events != TEVENT_LINERATE_SETUP_TEXPIRED) {
		txhfcnt = (u8) cpri_reg_get_val(
				&regs->cpri_thfnctr, TX_HFN_COUNTER_MASK);
		if (txhfcnt >= 1)
			break;
		dev_dbg(dev, "waiting for 1 HF to sent out\n");
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}

	dev_info(dev, "line rate autoneg successful\n");

	return err;
}


static int check_framesync(struct cpri_framer *framer, enum cpri_link_rate rate)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	u32 line_sync_acheived;
	u32 mask = 0;

	/* Check hfn sync status */
	mask = (RX_HFN_STATE_MASK | RX_BFN_STATE_MASK);
	while (framer->timer_expiry_events != TEVENT_LINERATE_SETUP_TEXPIRED) {
		line_sync_acheived = cpri_reg_get_val(
					&regs->cpri_status, mask);
		if (line_sync_acheived)
			return 0;

		schedule_timeout_interruptible(msecs_to_jiffies(20));
	}

	return -ETIME;
}

static int check_linesync(struct cpri_framer *framer, enum cpri_link_rate rate)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
#if 0
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
#endif
	struct device *dev = framer->cpri_dev->dev;
	u32 rai_cleared;
	int err = 0;

	/* TBD: Setup hw before starting autoneg with REC - Not clear
	 * Set Tx protocol ver ? Set Tx seed ?
	 * Identify channels and what data to send ?
	 */
	cpri_reg_set(&regs->cpri_config,
				CONF_RX_EN_MASK);

	do {
		/* Turn ON Tx for 'tx_on_dur_sec' */
		cpri_reg_set(&regs->cpri_config,
				CONF_TX_EN_MASK);
#if 0
		/* Enable SFP Tx */
		set_sfp_txdisable(framer->sfp_dev, 1);

		schedule_timeout_interruptible(param->tx_on_time * HZ);

		/* Turn OFF Tx for 'tx_off_dur_sec'*/
		cpri_reg_clear(&regs->cpri_config,
				CONF_TX_EN_MASK);

		/* Disable SFP Tx */
		set_sfp_txdisable(framer->sfp_dev, 0);

		schedule_timeout_interruptible(param->tx_off_time * HZ);
#endif
		/* TBD: check whether REC signal received
		 * Need to handle LOS here
		 * LOS will go down when REC signal is recieved
		 */

		err = check_framesync(framer, rate);
		if (err == -ETIME)
			break;

		/* Check for remote alarm indication cleared */
		rai_cleared = (RAI & cpri_read(&regs->cpri_errevent));

		dev_dbg(dev, "waiting for RAI to clear\n");
		schedule_timeout_interruptible(msecs_to_jiffies(1));

	} while (rai_cleared);

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
	struct cpri_common_regs __iomem *common_regs = framer->cpri_dev->regs;
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

	/* Setup line rate timer */
	init_timer(&linerate_timer);
	linerate_timer.function = linerate_setup_timer_expiry_hndlr;
	linerate_timer.data = (unsigned long) framer;

	for (rate = high; rate >= low; rate--) {
		dev_dbg(dev, "trying new link rate\n");
		cpri_state_machine(framer,
				CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS);
#if 0
		/* Disable framer clock */
		if (framer->id == 2)
			cpri_reg_clear(
				&common_regs->cpri_ctrlclk, C2_CLK_MASK);
		else
			cpri_reg_clear(
				&common_regs->cpri_ctrlclk, C1_CLK_MASK);
		/* Init and set highest possible line rate for serdes only when
		 *  the previous state is STANDBY & the framer config in
		 *  SLAVE mode
		 */
		if (framer->framer_param.ctrl_flags & SLAVE_MODE) {
			if (framer->framer_state == STANDBY)
				/* TBD: serdes_init needs to be called */
		}
#endif

		/* TBD: wait for serdes PLL to be locked if not come out
		 * if timeout happen
		 */

		if (linkrate_autoneg_reset(framer, rate)) {
			dev_err(dev, "line autoneg reset failed\n");
			return;
		}

		mdelay(10);
		/* Enable framer clock */
		if (framer->id == 2)
			cpri_reg_set(
				&common_regs->cpri_ctrlclk, C2_CLK_MASK);
		else
			cpri_reg_set(
				&common_regs->cpri_ctrlclk, C1_CLK_MASK);

		mdelay(1);

		cpri_init_framer(framer);
		/* Start timer */
		timer_dur = jiffies + (param->linerate_timeout) * HZ;
		linerate_timer.expires = timer_dur;
		add_timer(&linerate_timer);

		/* check for sync acheived */
		err = check_linesync(framer, rate);
		if (err == 0) {
			err = cpri_stable(framer, rate);
			break;
		}

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
			dev_err(dev, "init axc param failed !!\n");
		cpri_state_machine(framer, CPRI_STATE_LINE_RATE_AUTONEG);
	}
}

void cpri_autoneg_all(struct work_struct *work)
{
	struct cpri_framer *framer = container_of(work, struct cpri_framer,
						allautoneg_task);
	enum cpri_state *state = &(framer->framer_state);
	unsigned long timer_dur;
	int err = 0;


	framer->timer_expiry_events = 0;

	while (*state != CPRI_STATE_AUTONEG_COMPLETE) {
		/* do line rate autoneg */
		cpri_linkrate_autoneg(&framer->lineautoneg_task);
		if (*state != CPRI_STATE_LINE_RATE_AUTONEG) {
			err = -ENOLINK;
			goto out;
		}

		/* Transition 2 */
		timer_dur = jiffies + framer->l1_expiry_dur_sec * HZ;
		framer->l1_timer.expires = timer_dur;
		add_timer(&framer->l1_timer);

		/* do proto ver autoneg */
		cpri_proto_ver_autoneg(&framer->protoautoneg_task);
		if (*state != CPRI_STATE_PROT_VER_AUTONEG) {
			err = -ENOLINK;
			goto out;
		}

		/* do eth rate autoneg */
		cpri_eth_autoneg(&framer->ethautoneg_task);
		if (*state != CPRI_STATE_ETH_RATE_AUTONEG) {
			err = -ENOLINK;
			goto out;
		} else {
			cpri_state_machine(framer,
				CPRI_STATE_AUTONEG_COMPLETE);
			break;
		}

	}


out:
	/* Transition 6 or 14 or error in individual anutoneg phase */
	del_timer_sync(&framer->l1_timer);
}

static int process_reconfig_cmd(enum recfg_cmd cmd, struct cpri_framer *framer)
{
	int retval = 0;
	struct device *dev = framer->cpri_dev->dev;

	switch (cmd) {
	case CPRI_LINK_RECONFIG_INIT_REQ:
		link_reconfig(framer);
		/* TODO: Notify user-link is now ready for link reconfig
		 * this will be updated once the new notification mechanism
		 * implemented
		 */
		break;
	case CPRI_INTERFACE_RECONFIG_INIT_REQ:
		interface_reconfig(framer);
		/* TODO: Notify user-link is now ready for interface &
		 * vendor specific reconfig. this will be updated once
		 * the new notification mechanism implemented
		 */
		break;
	case CPRI_INTERFACE_RECONFIG_SETUP_REQ:
		/* TODO: To be updated after merging AxC code */
		break;
	case CPRI_AXC_MAPPING_RECONFIG_INIT_REQ:
		/* TODO: To be updated after merging AxC code */
		break;
	case CPRI_AXC_MAPPING_RECONFIG_SETUP_REQ:
		/* TODO: To be updated after merging AxC code */
		break;
	default:
		retval = -EINVAL;
		dev_info(dev, "got invalid reconfig cmd: %d\n", cmd);
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
	enum recfg_cmd recfg_cmd;
	struct device *dev = framer->cpri_dev->dev;
	void __user *ioargp = (void __user *)arg;
	unsigned long timer_dur;
	unsigned int count;
	unsigned int *buf;
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
		if (copy_from_user(&recfg_cmd, (enum recfg_cmd *)ioargp,
				sizeof(enum recfg_cmd)) != 0) {
			err = -EFAULT;
			goto out;
		}

		err = process_reconfig_cmd(recfg_cmd, framer);
		if (err != 0)
			goto out;

		break;

	case CPRI_START_L1TIMER:
		/* This command is used when application controls
		 * individual stages of autonegotiation
		 */
		timer_dur = jiffies + framer->l1_expiry_dur_sec * HZ;
		framer->l1_timer.expires = timer_dur;
		add_timer(&framer->l1_timer);

		break;

	case CPRI_STOP_L1TIMER:
		/* This command is used when application controls
		 * individual stages of autonegotiation
		 */
		del_timer_sync(&framer->l1_timer);

		break;

	case CPRI_SET_L1TIMER:
		if (copy_from_user(&framer->l1_expiry_dur_sec,
			(unsigned long *)ioargp, sizeof(unsigned long)) != 0) {
			err = -EFAULT;
			goto out;
		}

		del_timer_sync(&framer->l1_timer);
		timer_dur = jiffies + (framer->l1_expiry_dur_sec) * HZ;
		framer->l1_timer.expires = timer_dur;

		break;

	case CPRI_INIT_AUTONEG_PARAM:
		if (param->eth_rates != NULL) {
			kfree(param->eth_rates);
			param->eth_rates = NULL;
		}
		if (copy_from_user(param, (struct cpri_autoneg_params *)ioargp,
				sizeof(struct cpri_autoneg_params)) != 0) {
			err = -EFAULT;
			goto out;
		}

		count = param->eth_rates_count;
		buf = kzalloc((sizeof(unsigned int) * count), GFP_KERNEL);
		if (!buf) {
			err = -ENOMEM;
			goto out;
		}

		if (copy_from_user(buf, (unsigned int *)param->eth_rates,
				sizeof(unsigned int) * count) != 0) {
			err = -EFAULT;
			kfree(buf);
			goto out;
		}

		param->eth_rates = buf;
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
		autoneg_param.l1_setup_timeout = param->l1_setup_timeout;
		autoneg_param.tx_on_time = param->tx_on_time;
		autoneg_param.tx_off_time = param->tx_off_time;
		autoneg_param.linerate_timeout = param->linerate_timeout;
		autoneg_param.link_rate_low = param->link_rate_low;
		autoneg_param.link_rate_high = param->link_rate_high;
		autoneg_param.cnm_timeout = param->cnm_timeout;
		autoneg_param.cm_mode = param->cm_mode;
		autoneg_param.hdlc_rate_low = param->hdlc_rate_low;
		autoneg_param.hdlc_rate_high = param->hdlc_rate_high;
		autoneg_param.eth_rates_count = param->eth_rates_count;
		autoneg_param.proto_timeout = param->proto_timeout;
		autoneg_param.tx_prot_ver = param->tx_prot_ver;
		autoneg_param.tx_scr_seed = param->tx_scr_seed;

		if (copy_to_user(autoneg_param.eth_rates, param->eth_rates,
			(sizeof(unsigned int) * param->eth_rates_count))) {
			err = -EFAULT;
			goto out;
		}

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

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

static u8 txhfcnt;

static void set_delay_config(struct cpri_framer *framer)
{
	struct cpri_delays_raw_cfg *cfg = &framer->delay_cfg;
	struct cpri_framer_regs __iomem *regs = framer->regs;

	cpri_reg_set_val(&framer->regs_lock,
				&regs->cpri_exdelaycfg,
				TX_EX_DELAY_MASK,
				cfg->tx_ex_delay);
	cpri_reg_set_val(&framer->regs_lock,
				&regs->cpri_exdelaycfg,
				RX_EX_DELAY_PERIOD_MASK,
				cfg->rx_ex_delay_period);
}

static void interface_reconfig(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;

	/* Disable Tx and Rx */
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_config,
			CONF_TX_EN_MASK|CONF_RX_EN_MASK);

	/* Reset CPRIn_RACCR, CPRIn_TACCR, CPRIn_RCR & CPRIn_TCR */
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_raxcctrl, 0xFFFFFFFF);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_taxcctrl, 0xFFFFFFFF);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_rctrl, 0xFFFFFFFF);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_tctrl, 0xFFFFFFFF);

	/* Reset CPRIn_MAP_CONFIG and CPRIn_MAP_TBL_CONFIG */
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_mapcfg, 0xFFFFFFFF);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_maptblcfg, 0xFFFFFFFF);
}

static void link_reconfig(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	u32 rsr = 1, tsr = 1;
	int i;

	/* Disable Tx and Rx*/
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_config,
			CONF_TX_EN_MASK|CONF_RX_EN_MASK);
	/* Reset CPRIn_RACCR, CPRIn_TACCR, CPRIn_RCR & CPRIn_TCR */
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_raxcctrl, 0xFFFFFFFF);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_taxcctrl, 0xFFFFFFFF);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_rctrl, 0xFFFFFFFF);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_tctrl, 0xFFFFFFFF);
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
		framer->framer_state = PROT_VER_MISMATCH;
		 /* TBD: Notify user */
		err = -ENOLINK;
	}

	get_txethrate(framer, &tx_eth_rate);
	get_rxethrate(framer, &rx_eth_rate);
	if (tx_eth_rate == rx_eth_rate) {
		framer->framer_state = ETHLINK_RATE_MISMATCH;
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
		framer->framer_state = PROT_VER_MISMATCH;
		del_timer_sync(&framer->ethptr_poller);
		/* TBD: Notify  user */
	}

	if (framer->framer_param.ctrl_flags & CPRI_DEV_SLAVE) {
		set_txethrate(param->eth_rates[param->eth_rates_count],
			framer);
		valid_ptr = cpri_reg_get_val(&framer->regs_lock,
			&regs->cpri_cmstatus, RX_FAST_CM_PTR_VALID_MASK);
		if (valid_ptr) {
			framer->framer_state = ETH_LINK_PROPOSED;
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

	framer->framer_state = L1TIMER_EXPIRED;
	/* TBD: Notify user */
}

static void cpri_init_autoneg_timers(struct cpri_framer *framer)
{
	struct timer_list l1_timer = framer->l1_timer;
	struct timer_list link_poller = framer->link_poller;
	struct timer_list ethptr_poller = framer->ethptr_poller;

	/* Initialise L1 timer */
	init_timer(&l1_timer);
	l1_timer.function = l1_timer_expiry_hndlr;
	l1_timer.data = (unsigned long) framer;

	/* Initialise poller for operational mode */
	init_timer(&link_poller);
	link_poller.function = link_monitor;
	link_poller.data = (unsigned long) framer;

	/* Initialise poller for passive link mode */
	init_timer(&ethptr_poller);
	ethptr_poller.function = ethptr_monitor;
	ethptr_poller.data = (unsigned long) framer;
}

static void set_autoneg_param(struct cpri_framer *framer,
		struct cpri_autoneg_params *param)
{
	struct cpri_framer_regs __iomem  *regs = framer->regs;

	/* Rx scrambler setting */
	if (param->flags & CPRI_RX_SCRAMBLER_EN)
		cpri_reg_set(&framer->regs_lock, &regs->cpri_rscrseed,
				RX_SCR_EN_MASK);

	cpri_init_autoneg_timers(framer);
}

static void update_delay(struct cpri_framer *framer)
{
	struct cpri_delays_raw *delay_out = &framer->delay_out;
	struct cpri_framer_regs  __iomem *regs = framer->regs;

	delay_out->rx_ext_buf_delay_valid = cpri_reg_get_val(&framer->regs_lock,
			&regs->cpri_exdelaystatus, RX_EX_BUF_DELAY_VALID_MASK);

	delay_out->rx_ext_buf_delay = cpri_reg_get_val(&framer->regs_lock,
			&regs->cpri_exdelaystatus, RX_EX_BUF_DELAY_MASK);


	delay_out->rx_byte_delay = cpri_reg_get_val(&framer->regs_lock,
			&regs->cpri_rdelay, RX_BYTE_DELAY_MASK);

	delay_out->rx_buf_delay = cpri_reg_get_val(&framer->regs_lock,
			&regs->cpri_rdelay, RX_BUF_DELAY_MASK);

	delay_out->rx_align_delay = cpri_reg_get_val(&framer->regs_lock,
			&regs->cpri_rdelay, RX_ALIGN_DELAY_MASK);


	delay_out->rx_roundtrip_delay = cpri_reg_get_val(&framer->regs_lock,
			&regs->cpri_exdelaystatus, RX_ROUND_TRIP_DELAY_MASK);
}

static void update_bf_data(struct cpri_framer *framer)
{
	struct cpri_autoneg_output *output = &(framer->autoneg_output);
	struct device *dev = framer->cpri_dev->dev;

	switch (output->common_link_rate) {
	case RATE2_1228_8M:
		output->cpri_bf_word_size = BF_W_SIZE_16;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_240;
	case RATE3_2457_6M:
		output->cpri_bf_word_size = BF_W_SIZE_32;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_480;
	case RATE4_3072_0M:
		output->cpri_bf_word_size = BF_W_SIZE_40;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_600;
	case RATE5_4915_2M:
		output->cpri_bf_word_size = BF_W_SIZE_64;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_960;
	case RATE6_6144_0M:
		output->cpri_bf_word_size = BF_W_SIZE_80;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_1200;
	case RATE7_9830_4M:
		output->cpri_bf_word_size = BF_W_SIZE_128;
		output->cpri_bf_iq_datablock_size = BF_IQ_BITS_1920;
	default:
		dev_err(dev, "Invalid common link rate\n");
	break;
	}
}

static int cpri_setup_vendor_autoneg(struct cpri_framer *framer)
{
	enum cpri_prot_ver tx_prot_ver, rx_prot_ver;
	u8 tx_eth_rate, rx_eth_rate;

	/* Set framer state */
	framer->framer_state = VENDOR_CONFIG;

	/* Check Tx/Rx protocol version */
	get_txprotver(framer, &tx_prot_ver);
	get_rxprotver(framer, &rx_prot_ver);
	if (tx_prot_ver != rx_prot_ver) {
		framer->framer_state = PROT_VER_MISMATCH;
		/* TBD: Notify  user */
		framer->stats.vendor_config_failures++;
		return -ENOLINK;
	}

	/* Check Tx/Rx eth rate */
	get_txethrate(framer, &tx_eth_rate);
	get_rxethrate(framer, &rx_eth_rate);
	if (tx_eth_rate != rx_eth_rate) {
		framer->framer_state = ETHLINK_RATE_MISMATCH;
		/* TBD: Notify  user */
		framer->stats.vendor_config_failures++;
		return -ENOLINK;
	}

	/* Setup ethernet DMA here */
	cpri_eth_enable(framer);

	return 0;
}

static int check_ethrate(struct cpri_framer *framer, int count)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct cpri_framer_regs *regs = framer->regs;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	struct device *dev = framer->cpri_dev->dev;
	u8 tx_eth_rate, rx_eth_rate;
	int status, err = 0;
	u8 hf_cnt;

	if (framer->timer_expiry_events != TEVENT_ETHRATE_SETUP_TEXPIRED) {
		err = -ETIME;
		goto out;
	}

	status = cpri_reg_get_val(&framer->regs_lock, &regs->cpri_cmstatus,
			RX_FAST_CM_PTR_VALID_MASK);

	/* if c&m ptr invalid */
	while (!status) {
		/* Check whether two HF passed */
		hf_cnt = (u8) cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_thfnctr, TX_HFN_COUNTER_MASK);
		hf_cnt -= txhfcnt;
		if (hf_cnt > 2) {
			tx_eth_rate = param->eth_rates[count-1];
			cpri_reg_set_val(&framer->regs_lock,
				&regs->cpri_cmconfig, TX_FAST_CM_PTR_MASK,
				(u32)tx_eth_rate);
			err = -ENOLINK;
			break;
		}
		dev_info(dev, "waiting for atleast 2 HF\n");
	}

	if (status) {
		tx_eth_rate = (u8) cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_cmconfig,
					TX_FAST_CM_PTR_MASK);

		rx_eth_rate = (u8) cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_cmstatus,
					RX_FAST_CM_PTR_VALID_MASK);

		if (tx_eth_rate == rx_eth_rate) {
			result->common_eth_link_rate = rx_eth_rate;
			dev_info(dev, "eth autoneg success\n");
			err = 0;
		}
	}

out:
	return err;
}

static void eth_setup_timer_expiry_hndlr(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	framer->timer_expiry_events = TEVENT_ETHRATE_SETUP_TEXPIRED;
	framer->framer_state = ETHLINK_SETUP_TIMER_EXPIRED;
	 /* TBD: Notify user */
}

static int cpri_eth_autoneg(struct cpri_framer *framer)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct timer_list eth_setup_timer;
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	int i, err = 0;

	/* Set framer state */
	framer->framer_state = ETHLINK_AUTONEG;

	/* Init eth rate expiry timer */
	init_timer(&eth_setup_timer);
	eth_setup_timer.function = eth_setup_timer_expiry_hndlr;
	eth_setup_timer.data = (unsigned long) framer;

	/* Turn ON Tx */
	cpri_reg_set(&framer->regs_lock, &regs->cpri_config, CONF_TX_EN_MASK);

	/* Setup eth rate expiry timer */
	eth_setup_timer.expires = jiffies + (param->cnm_timeout) * HZ;
	add_timer(&eth_setup_timer);

	for (i = param->eth_rates_count; i >= 1 ; i--) {
		err = check_ethrate(framer, i);
		if (err == -ENOLINK)
			continue;
		else if (err == -ETIME) {
			dev_info(dev, "eth autoneg timeout\n");
			goto out_fail;
		} else
			goto out_pass;
	}

out_fail:
	framer->stats.cnm_auto_neg_failures++;
	framer->framer_state = PASSIVELINK;
	framer->dev_flags |= CPRI_PASSIVE_LINK;

out_pass:
	del_timer_sync(&eth_setup_timer);
	return err;
}

static void proto_setup_timer_expiry_hndlr(unsigned long ptr)
{

	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	framer->timer_expiry_events = TEVENT_PROTVER_SETUP_TEXPIRED;
	framer->framer_state = PROT_VER_SETUP_TIMER_EXPIRED;
	/* TBD: Notify user */
}

static int cpri_proto_ver_autoneg(struct cpri_framer *framer)
{
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

	/* Set framer state */
	framer->framer_state = PROT_VER_AUTONEG;

	/* Setup protocol ver expiry timer */
	init_timer(&proto_setup_timer);
	proto_setup_timer.function = proto_setup_timer_expiry_hndlr;
	timer_dur = jiffies + param->proto_timeout * HZ;
	proto_setup_timer.expires = timer_dur;
	proto_setup_timer.data = (unsigned long) framer;
	add_timer(&proto_setup_timer);

	/* Turn ON Tx */
	cpri_reg_set(&framer->regs_lock, &regs->cpri_config, CONF_TX_EN_MASK);

	while (framer->timer_expiry_events == TEVENT_PROTVER_SETUP_TEXPIRED) {

		/* Update the received scramble seed value */
		result->rx_scramble_seed_val =
			cpri_reg_get_val(&framer->regs_lock,
						&regs->cpri_rscrseed,
						SCR_SEED_MASK);

		/* Get last set proto ver */
		pver = cpri_reg_get_val(&framer->regs_lock,
						&regs->cpri_tprotver,
						PROTO_VER_MASK);
		tproto_ver = (pver == 1) ? VER_1 : VER_2;

		/* Check whether two HF passed */
		hf_cnt = (u8) cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_thfnctr,
					TX_HFN_COUNTER_MASK);
		hf_cnt -= txhfcnt;
		if (hf_cnt >= 2) {
			/* Change Tx proto ver */
			dev_info(dev, "proto ver - 2HF passed\n");
			if (tproto_ver == VER_1)
				cpri_reg_set_val(&framer->regs_lock,
						&regs->cpri_tprotver,
						PROTO_VER_MASK, 2);
			else
				cpri_reg_set_val(&framer->regs_lock,
						&regs->cpri_tprotver,
						PROTO_VER_MASK, 1);
		} else {
			/* No reg so reading from CW */
			get_rxprotver(framer, &rproto_ver);

			if (rproto_ver == tproto_ver) {
				result->common_prot_ver  = tproto_ver;
				err = 0;
				dev_info(dev, "proto ver autoneg successful\n");
				break;
			}
		}
		dev_info(dev, "proto ver autoneg in progress\n");
	}

	if (err != 0)
		framer->stats.proto_auto_neg_failures++;

	del_timer_sync(&proto_setup_timer);

	return err;
}

static int cpri_stable(struct cpri_framer *framer, enum cpri_link_rate rate)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_autoneg_output *result = &(framer->autoneg_output);
	struct device *dev = framer->cpri_dev->dev;
	u8 tx_eth_rate;
	int err = 0, i;

	/* update the common line rate between REC and RE */
	result->common_link_rate = rate;

	/* Update Tx proto ver and Tx scrambler seed */
	if (rate > RATE4_3072_0M) {
		cpri_reg_set_val(&framer->regs_lock, &regs->cpri_tprotver,
				PROTO_VER_MASK, 2);

		cpri_reg_set_val(&framer->regs_lock, &regs->cpri_tscrseed,
				SCR_SEED_MASK, param->tx_scr_seed);
	} else
		cpri_reg_set_val(&framer->regs_lock, &regs->cpri_tprotver,
				PROTO_VER_MASK, 1);

	/* Set max Tx ethernet rate/ptr */
	tx_eth_rate = param->eth_rates[param->eth_rates_count-1];
	cpri_reg_set_val(&framer->regs_lock, &regs->cpri_cmconfig,
			TX_FAST_CM_PTR_MASK, (u32)tx_eth_rate);

	/* Turn on Tx and enable tx for 1 HF */
	cpri_reg_set(&framer->regs_lock, &regs->cpri_config, CONF_TX_EN_MASK);
	txhfcnt = 0;
	for (i = 0; i < 10; i++) {
		txhfcnt = (u8) cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_thfnctr,
					TX_HFN_COUNTER_MASK);
		if (txhfcnt >= 1)
			break;
		dev_info(dev, "waiting for 1 HF to sent out\n");
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}
	/* Turn OFF Tx */
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_config, CONF_TX_EN_MASK);

	dev_info(dev, "line rate autoneg successful\n");

	return err;
}


static int check_hfnsync(struct cpri_framer *framer, enum cpri_link_rate rate)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	u32 hfn_sync_acheived;

	/* Check hfn sync status */
	while (framer->timer_expiry_events == TEVENT_LINERATE_SETUP_TEXPIRED) {
		hfn_sync_acheived = cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_status, RX_HFN_STATE_MASK);
		if (hfn_sync_acheived)
			return 0;

		dev_info(dev, "waiting for HFSYNC\n");
		msleep(20);
	}

	return -ETIME;
}

static int check_linesync(struct cpri_framer *framer, enum cpri_link_rate rate)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct device *dev = framer->cpri_dev->dev;
	u32 rai_cleared;
	int err = 0;

	/* TBD: Setup hw before starting autoneg with REC - Not clear
	 * Set Tx protocol ver ? Set Tx seed ?
	 * Identify channels and what data to send ?
	 */

	do {
		/* Turn ON Tx for 'tx_on_dur_sec' */
		cpri_reg_set(&framer->regs_lock, &regs->cpri_config,
				CONF_TX_EN_MASK);

		/* Enable SFP Tx */
		set_sfp_txdisable(framer->sfp_dev, 1);

		schedule_timeout_interruptible(param->tx_on_time * HZ);

		/* Turn OFF Tx for 'tx_off_dur_sec'*/
		cpri_reg_clear(&framer->regs_lock, &regs->cpri_config,
				CONF_TX_EN_MASK);

		/* Disable SFP Tx */
		set_sfp_txdisable(framer->sfp_dev, 0);

		schedule_timeout_interruptible(param->tx_off_time * HZ);

		/* TBD: check whether REC signal received
		 * Need to handle LOS here
		 * LOS will go down when REC signal is recieved
		 */

		err = check_hfnsync(framer, rate);
		if (err == -ETIME)
			break;

		/* Check for remote alarm indication cleared */
		rai_cleared = (RAI & cpri_read(&regs->cpri_errevent));

		dev_info(dev, "waiting for RAI to clear\n");

	} while (rai_cleared);

	return err;
}

static void linerate_setup_timer_expiry_hndlr(unsigned long ptr)
{
	struct cpri_framer *framer = (struct cpri_framer *)ptr;

	framer->timer_expiry_events = TEVENT_LINERATE_SETUP_TEXPIRED;
	framer->framer_state = LINK_RATE_SETUP_TIMER_EXPIRED;
}

static int cpri_linkrate_autoneg(struct cpri_framer *framer)
{
	struct cpri_autoneg_params *param = &(framer->autoneg_param);
	struct cpri_common_regs __iomem *common_regs = framer->cpri_dev->regs;
	struct device *dev = framer->cpri_dev->dev;
	struct timer_list linerate_timer;
	enum cpri_link_rate rate;
	enum cpri_link_rate low = param->link_rate_low;
	enum cpri_link_rate high = param->link_rate_high;
	int err = 0;
	unsigned long timer_dur;

	/* Set framer state */
	framer->framer_state = LINE_RATE_AUTONEG;

	/* Setup line rate timer */
	init_timer(&linerate_timer);
	linerate_timer.function = linerate_setup_timer_expiry_hndlr;
	linerate_timer.data = (unsigned long) framer;

#if 0
	/* Init and set highest possible line rate for serdes only when
	 *  the previous state is STANDBY & the framer config in SLAVE mode
	 */
	if (framer->framer_param.ctrl_flags & SLAVE_MODE) {
		if (framer->framer_state == STANDBY)
			/* TBD: serdes_init needs to be called */
	}
#endif

	for (rate = high; rate >= low; rate--) {
		/* Disable framer clock */
		if (framer->id)
			cpri_reg_clear(&framer->cpri_dev->lock,
				&common_regs->cpri_ctrlclk, C2_CLK_MASK);
		else
			cpri_reg_clear(&framer->cpri_dev->lock,
				&common_regs->cpri_ctrlclk, C1_CLK_MASK);

		/* TBD: serdes_cfg to be called here - Configure SerDes
		 * for new link rate
		 */

		/* TBD: Reset the CPRI block; REG NOT FOUND IN BG */

		/* Enable framer clock */
		if (framer->id)
			cpri_reg_clear(&framer->cpri_dev->lock,
				&common_regs->cpri_ctrlclk, C2_CLK_MASK);
		else
			cpri_reg_clear(&framer->cpri_dev->lock,
				&common_regs->cpri_ctrlclk, C1_CLK_MASK);

		/* Start timer */
		timer_dur = jiffies + (param->linerate_timeout) * HZ;
		linerate_timer.expires = timer_dur;
		add_timer(&linerate_timer);

		/* check for sync acheived */
		if (check_linesync(framer, rate) == 0) {
			err = cpri_stable(framer, rate);
			break;
		}

		/* timer will be destroyed for every new line rate */
		del_timer_sync(&linerate_timer);
		dev_info(dev, "trying next link rate\n");
	}

	del_timer_sync(&linerate_timer);

	if (err != 0)
		framer->stats.l1_auto_neg_failures++;

	return err;
}

static int cpri_autoneg_all(struct cpri_framer *framer)
{
	enum cpri_state *state = &(framer->framer_state);
	struct device *dev = framer->cpri_dev->dev;
	struct net_device *ndev = framer->eth_priv->ndev;
	unsigned long timer_dur;
	int err = 0;

	*state = STANDBY;

	while (*state != L1TIMER_EXPIRED) {
		/* do line rate autoneg */
		err = cpri_linkrate_autoneg(framer);
		if (err != 0)
			return err;

		/* Transition 2 */
		timer_dur = jiffies + framer->l1_expiry_dur_sec * HZ;
		framer->l1_timer.expires = timer_dur;
		add_timer(&framer->l1_timer);

		/* do proto ver autoneg */
		err = cpri_proto_ver_autoneg(framer);
		if (err != 0) {
			err = -ENOLINK;
			goto out;
		}
		/* do eth rate autoneg */
		err = cpri_eth_autoneg(framer);
		if (err != 0) {
			err = -ENOLINK;
			goto out_passive;
		}
		/* do vendor config autoneg */
		err = cpri_setup_vendor_autoneg(framer);
		if (err != 0) {
			err = -ENOLINK;
			goto out;
		} else {
			*state = AUTONEG_COMPLETED;
			break;
		}
	}

out_passive:

	if (*state == AUTONEG_COMPLETED || *state == PASSIVELINK) {

		dev_info(dev, "autoneg passed\n");
		*state = OPERATIONAL;

		/* Update autoneg output data */
		update_delay(framer);
		update_bf_data(framer);

		/* Setup poller for checking Tx/Rx eth rate and
		 * proto ver mismatch periodically
		 */
		timer_dur = jiffies + framer->link_poll_dur_sec * HZ;
		framer->link_poller.expires = timer_dur;
		add_timer(&framer->link_poller);

		/* Setup poller for checking change in eth ptr */
		if (*state != PASSIVELINK)
			netif_carrier_on(ndev);
		else {
			timer_dur = jiffies + framer->passive_poll_dur_sec * HZ;
			framer->ethptr_poller.expires = timer_dur;
			add_timer(&framer->ethptr_poller);
		}
	}

out:
	/* Transition 6 or 14 or error in individual anutoneg phase */
	del_timer_sync(&framer->l1_timer);

	return err;
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
		if (cpri_linkrate_autoneg(framer) != 0)
			retval = -EFAULT;
		break;
	case CPRI_DO_PROTOCOL_AUTONEG:
		if (cpri_proto_ver_autoneg(framer) != 0)
			retval = -EFAULT;
		break;
	case CPRI_DO_CNM_AUTONEG:
		if (cpri_eth_autoneg(framer) != 0)
			retval = -EFAULT;
		break;
	case CPRI_DO_VENDOR_AUTONEG:
		if (cpri_setup_vendor_autoneg(framer) != 0)
			retval = -EFAULT;
		break;
	case CPRI_DO_AUTONEG_ALL:
		if (cpri_autoneg_all(framer) != 0)
			retval = -EFAULT;
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
	struct cpri_autoneg_output *output = &framer->autoneg_output;
	enum autoneg_cmd an_cmd;
	enum recfg_cmd recfg_cmd;
	struct device *dev = framer->cpri_dev->dev;
	void __user *ioargp = (void __user *)arg;
	unsigned long timer_dur;
	int err = 0;

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
		timer_dur = jiffies + framer->l1_expiry_dur_sec * HZ;
		framer->l1_timer.expires = timer_dur;
		add_timer(&framer->l1_timer);

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
		if (copy_from_user(param,
			(struct cpri_autoneg_params *)ioargp,
			sizeof(struct cpri_autoneg_params)) != 0) {
			err = -EFAULT;
			goto out;
		}

		set_autoneg_param(framer, param);

		break;

	case CPRI_GET_AUTONEG_PARAM:
		if (copy_to_user(ioargp,
			(struct cpri_autoneg_params *)param,
			sizeof(struct cpri_autoneg_params))) {
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

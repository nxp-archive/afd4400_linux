/*
 * drivers/net/cpri/cpri_cw.c
 * CPRI device driver - control word utility functions
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/cpri.h>

static void set_cw_data(u8 *cw_word, u32 data)
{
	*(cw_word + 0) = ((data >> CW_BYTE0) & BYTE_MASK);
	*(cw_word + 1) = ((data >> CW_BYTE1) & BYTE_MASK);
	*(cw_word + 2) = ((data >> CW_BYTE2) & BYTE_MASK);
	*(cw_word + 3) = ((data >> CW_BYTE3) & BYTE_MASK);
	return;
}

static u32 cw_read(u32 *addr, u32 mask)
{
	return cpri_reg_get_val(addr, mask);
}

void clear_control_tx_table(struct cpri_framer *framer)
{
	u32 value = 0, i;
	struct cpri_framer_regs *regs = framer->regs;

	for (i = 0; i <= MAX_TCTA_ADDR; i++) {

		cpri_reg_clear(&regs->cpri_tctrldata0,
			MASK_ALL);
		cpri_reg_clear(&regs->cpri_tctrldata1,
			MASK_ALL);
		cpri_reg_clear(&regs->cpri_tctrldata2,
			MASK_ALL);
		cpri_reg_clear(&regs->cpri_tctrldata3,
			MASK_ALL);
		value = i;
		value = value << TCTA_ADDR_OFFSET;
		value |= TCT_WRITE_MASK;
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
				MASK_ALL, value);
	}
}

/* Write transmit control word */
static int cpri_write_txctrlword(struct cpri_framer *framer)
{
	unsigned int count;
	struct hf_ctrl_chans *hf_channels = &(framer->tx_hf_ctrl_chans);
	struct ctrl_chan *channels;
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int i, j;
	u32 mask = 0, *addr;
	u32 cw, idx;
	u32 data = 0;
	int err = 0;

	count = hf_channels->channel_count;
	channels = hf_channels->channels;

	/* Enable cw insertion */
	cpri_reg_set(&regs->cpri_config,
			TX_CW_INSERT_EN_MASK);

	for (i = 0; i < count; i++) {

		cpri_reg_clear(&regs->cpri_tctrlattrib, MASK_ALL);
		mask = 0;
		/* Find control word and select it in CPRInTCTIE */
		cw = FIND_CW_NUM(channels[i].chan, channels[i].word_bitmap);
		if (cw <= 79) {
			if (cw <= 15)
				mask = 1 << cw;

			if (cw > 15 && cw < 64) {
				dev_err(dev, "Invalid cw request\n");
				err = -ENOMEM;
				goto out;
			}

			if (cw >= 64)
				mask = 1 << (cw-48);

			cpri_reg_set(
				&regs->cpri_tctrlinserttb1, mask);
		}
		if (cw >= 128) {
			if (cw <= 143)
				mask = 1 << (cw-128);

			if (cw > 143 && cw < 192) {
				dev_err(dev, "Invalid cw request\n");
				err = -ENOMEM;
				goto out;
			}

			if (cw >= 192)
				mask = 1 << (cw-176);

			cpri_reg_set(
				&regs->cpri_tctrlinserttb2, mask);
		}

		/* Write data in CPRInTCD0 - CPRInTCD3 */
		addr = (&regs->cpri_tctrldata0);
		idx = channels[i].word_bitmap;
		mask = 24;
		for (j = 1; j <= MAX_CWBYTES; j++) {
			data |= channels[i].words[idx][j-1] << mask;
			mask -= SIZE_BYTE;
			if (j % SIZE_REGBYTES == 0) {
				cpri_reg_set_val(addr, MASK_ALL, data);
				data = 0;
				addr++;
				mask = 24;
			}
		}

		/* Write cw address in CPRInTCA */
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
				MASK_ALL, ((cw << TCTA_ADDR_OFFSET) |
					TCT_WRITE_MASK));

	}

out:

	/* TBD: we need to check with IP team do we have to wait for 1
	 * or 2 HF before disabling sot that all words from table are
	 * transmitted?
	 */

	return err;
}

/* Read transmit control word */
static int cpri_read_txctrlword(struct cpri_framer *framer)
{
	unsigned int count;
	struct hf_ctrl_chans *hf_channels = &(framer->tx_hf_ctrl_chans);
	struct ctrl_chan *channels;
	struct cpri_framer_regs *regs = framer->regs;
	unsigned int i, j;
	u32 mask = 0, *addr;
	u8 cw, idx;
	u32 data;

	count = hf_channels->channel_count;
	channels = hf_channels->channels;

	for (i = 0; i < count; i++) {

		/* Find control word */
		cw = FIND_CW_NUM(channels[i].chan, channels[i].word_bitmap);

		/* Write cw address in CPRInTCA */
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
				TCT_ADDR_MASK, (u32) cw);

		/* Read command in CPRInTCA */
		cpri_reg_clear(&regs->cpri_tctrlattrib,
				TCT_WRITE_MASK);

		/* Read data From CPRInTCD0 - CPRInTCD3 */
		addr = (&regs->cpri_tctrldata0);
		idx = channels[i].word_bitmap;
		for (j = 0; j < MAX_CWBYTES; ) {
			mask = MASK_ALL;
			data = cw_read(addr, mask);
			set_cw_data(&channels[i].words[idx][j], data);
			j = j + SIZE_REGBYTES;
			addr++;
		}
	}

	return 0;
}

/* Read receive control word */
static int cpri_read_rxctrlword(struct cpri_framer *framer)
{
	unsigned int count;
	struct hf_ctrl_chans *hf_channels = &(framer->rx_hf_ctrl_chans);
	struct ctrl_chan *channels;
	struct cpri_framer_regs *regs = framer->regs;
	unsigned int i, j;
	u32 mask = 0, *addr;
	u8 cw, idx;
	u32 data;

	count = hf_channels->channel_count;
	channels = hf_channels->channels;

	for (i = 0; i < count; i++) {
		/* Find control word */
		cw = FIND_CW_NUM(channels[i].chan, channels[i].word_bitmap);

		/* Write cw address in CPRInRCA */
		cpri_reg_set_val(&regs->cpri_rctrlattrib,
				TCT_ADDR_MASK, (u32) cw);

		/* Read data From CPRInRCD0 - CPRInRCD3 */
		addr = (&regs->cpri_rctrldata0);
		idx = channels[i].word_bitmap;
		for (j = 0; j < MAX_CWBYTES; ) {
			mask = MASK_ALL;
			data = cw_read(addr, mask);
			set_cw_data(&channels[i].words[idx][j], data);
			j = j + SIZE_REGBYTES;
			addr++;
		}
	}

	return 0;
}

/* Set Tx Ethernet rate */
int set_txethrate(u8 eth_rate, struct cpri_framer *framer)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *eth_chan;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int count;
	int err = 0;

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channel_count = 1;
	count = hf_channels->channel_count;
	eth_chan = kzalloc(sizeof(struct ctrl_chan) * count, GFP_KERNEL);
	if (!eth_chan) {
		dev_err(dev, "TxCW write failed: ethernet rate not set\n");
		return -ENOMEM;
	}

	hf_channels->channels = eth_chan;
	hf_channels->channels[0].chan = 2;
	hf_channels->channels[0].words[CWIDX_ETHPTR][WDOFF_B0] = eth_rate;
	hf_channels->channels[0].word_bitmap = WORD_IDX3;

	if (cpri_write_txctrlword(framer) < 0) {
		dev_err(dev, "TxCW write failed: ethernet rate not set\n");
		err = -ENOMEM;
	}

	kfree(eth_chan);
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}

/* Get Tx Ethernet rate */
int get_txethrate(struct cpri_framer *framer, u8 *eth_rate)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *eth_chan;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int count;
	int err = 0;

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channel_count = 1;
	count = hf_channels->channel_count;
	eth_chan = kzalloc(sizeof(struct ctrl_chan) * count, GFP_KERNEL);
	if (!eth_chan) {
		dev_err(dev, "TxCW read failed: ethernet rate not read\n");
		return -ENOMEM;
	}

	hf_channels->channels = eth_chan;
	hf_channels->channels[0].chan = 2;
	hf_channels->channels[0].word_bitmap = WORD_IDX3;

	if (cpri_read_txctrlword(framer) < 0) {
		dev_err(dev, "TxCW read failed: ethernet rate not read\n");
		err = -ENOMEM;
		goto out;
	}

	*eth_rate = hf_channels->channels[0].words[CWIDX_ETHPTR][WDOFF_B0];

out:
	kfree(eth_chan);
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}

/* Set Tx protocol version */
int set_txprotver(enum cpri_prot_ver ver, struct cpri_framer *framer)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *prtover_chan;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int count;
	int err = 0;

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channel_count = 1;
	count = hf_channels->channel_count;
	prtover_chan = kzalloc(sizeof(struct ctrl_chan) * count, GFP_KERNEL);
	if (!prtover_chan) {
		dev_err(dev, "TxCW write failed: proto ver not set\n");
		return -ENOMEM;
	}

	hf_channels->channels = prtover_chan;
	hf_channels->channels[0].chan = 2;
	hf_channels->channels[0].words[CWIDX_PROTVER][WDOFF_B0] = (u8)ver;
	hf_channels->channels[0].word_bitmap = WORD_IDX0;

	if (cpri_write_txctrlword(framer) < 0) {
		dev_err(dev, "TxCW write failed: proto ver not set\n");
		err = -ENOMEM;
	}

	kfree(prtover_chan);
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}

/* Get Tx protocol version */
int get_txprotver(struct cpri_framer *framer,
			enum cpri_prot_ver *prot_ver)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *prtover_chan;
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;
	unsigned int count;
	u8 ver;

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channel_count = 1;
	count = hf_channels->channel_count;
	prtover_chan = kzalloc(sizeof(struct ctrl_chan) * count, GFP_KERNEL);
	if (!prtover_chan) {
		dev_err(dev, "TxCW read failed: proto ver not read\n");
		return -ENOMEM;
	}

	hf_channels->channels = prtover_chan;
	hf_channels->channels[0].chan = 2;
	hf_channels->channels[0].word_bitmap = WORD_IDX0;

	if (cpri_read_txctrlword(framer) < 0) {
		dev_err(dev, "TxCW read failed: proto ver not read\n");
		err = -ENOMEM;
		goto out;
	}

	ver = hf_channels->channels[0].words[CWIDX_PROTVER][WDOFF_B0];
	*prot_ver = (ver == 1) ? VER_1 : VER_2;

out:
	kfree(prtover_chan);
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}

/* Get Rx Ethernet rate */
int get_rxethrate(struct cpri_framer *framer, u8 *eth_rate)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *eth_chan;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int count;
	int err = 0;

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->rx_hf_ctrl_chans);
	hf_channels->channel_count = 1;
	count = hf_channels->channel_count;
	eth_chan = kzalloc(sizeof(struct ctrl_chan) * count, GFP_KERNEL);
	if (!eth_chan) {
		dev_err(dev, "RxCW read failed: ethernet rate not read\n");
		return -ENOMEM;
	}

	hf_channels->channels = eth_chan;
	hf_channels->channels[0].chan = 2;
	hf_channels->channels[0].word_bitmap = WORD_IDX3;

	if (cpri_read_rxctrlword(framer) < 0) {
		dev_err(dev, "RxCW read failed: ethernet rate not read\n");
		err = -ENOMEM;
		goto out;
	}

	*eth_rate = hf_channels->channels[0].words[CWIDX_ETHPTR][WDOFF_B0];

out:
	kfree(eth_chan);
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}

/* Get Rx protocol version */
int get_rxprotver(struct cpri_framer *framer, enum cpri_prot_ver *prot_ver)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *prtover_chan;
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;
	unsigned int count;
	u8 ver;

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->rx_hf_ctrl_chans);
	hf_channels->channel_count = 1;
	count = hf_channels->channel_count;
	prtover_chan = kzalloc(sizeof(struct ctrl_chan) * count, GFP_KERNEL);
	if (!prtover_chan) {
		dev_err(dev, "RxCW read failed: proto ver not read\n");
		return -ENOMEM;
	}

	hf_channels->channels = prtover_chan;
	hf_channels->channels[0].chan = 2;
	hf_channels->channels[0].word_bitmap = WORD_IDX0;


	if (cpri_read_rxctrlword(framer) < 0) {
		dev_err(dev, "RxCW read failed: proto ver not read\n");
		err = -ENOMEM;
		goto out;
	}

	ver = hf_channels->channels[0].words[CWIDX_PROTVER][WDOFF_B0];
	if ((ver == 1) || (ver == 2))
		*prot_ver = (ver == 1) ? VER_1 : VER_2;
	else
		 *prot_ver = VER_INVALID;

out:
	kfree(prtover_chan);
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}

/*
 * drivers/rf/cpri/cpri_cw.c
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

#include "cpri.h"

static void cw_write(raw_spinlock_t *lock, u32 *addr, u8 data, u32 mask)
{
	cpri_reg_set_val(lock, addr, (u32) data, mask);
}

static u8 cw_read(raw_spinlock_t *lock, u32 *addr, u32 mask)
{
	return (u8) cpri_reg_get_val(lock, addr, mask);
}

/* Write transmit control word */
static int cpri_write_txctrlword(struct cpri_framer *framer)
{
	unsigned int count;
	struct hf_ctrl_chans *hf_channels = &(framer->tx_hf_ctrl_chans);
	struct ctrl_chan **channels;
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int i, j;
	u32 mask = 0, *addr;
	u8 cw, idx, data;
	int err = 0;

	count = hf_channels->channel_count;
	channels = &(hf_channels->channels[0]);

	/* Enable cw insertion */
	cpri_reg_set(&framer->regs_lock, &regs->cpri_config,
			TX_CW_INSERT_EN_MASK);

	for (i = 0; i < count; i++) {

		if (channels[i] == NULL) {
			dev_err(dev, "Invalid channel container\n");
			err = -ENOMEM;
			goto out;
		}
		/* Find control word and select it in CPRInTCTIE */
		cw = FIND_CW_NUM(channels[i]->chan, channels[i]->word_bitmap);
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

			cpri_reg_set(&framer->regs_lock,
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

			cpri_reg_set(&framer->regs_lock,
				&regs->cpri_tctrlinserttb2, mask);
		}

		/* Write data in CPRInTCD0 - CPRInTCD3 */
		addr = (&regs->cpri_tctrldata0);
		idx = channels[i]->word_bitmap;
		for (j = 1; j <= MAX_CWBYTES; j++) {
			data = channels[i]->words[idx][j-1];
			mask = BYTE_MASK << mask;
			cw_write(&framer->regs_lock, addr, data, mask);
			mask += SIZE_BYTE;
			if (j % SIZE_REGBYTES == 0) {
				addr++;
				mask = 0;
			}
		}

		/* Write cw address in CPRInTCA */
		cpri_reg_set_val(&framer->regs_lock, &regs->cpri_tctrlattrib,
				TCT_ADDR_MASK, (u32) cw);

		/* Write command in CPRInTCA */
		cpri_reg_set(&framer->regs_lock, &regs->cpri_tctrlattrib,
				TCT_WRITE_MASK);
	}

out:
	/* Disable cw insertion */
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_config,
			TX_CW_INSERT_EN_MASK);

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
	struct ctrl_chan **channels;
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int i, j;
	u32 mask = 0, *addr;
	u8 cw, idx, data;

	count = hf_channels->channel_count;
	channels = &(hf_channels->channels[0]);

	for (i = 0; i < count; i++) {

		if (channels[i] == NULL) {
			dev_err(dev, "Invalid channel container\n");
			return -ENOMEM;
		}
		/* Find control word */
		cw = FIND_CW_NUM(channels[i]->chan, channels[i]->word_bitmap);

		/* Write cw address in CPRInTCA */
		cpri_reg_set_val(&framer->regs_lock, &regs->cpri_tctrlattrib,
				TCT_ADDR_MASK, (u32) cw);

		/* Read command in CPRInTCA */
		cpri_reg_clear(&framer->regs_lock, &regs->cpri_tctrlattrib,
				TCT_WRITE_MASK);

		/* Read data From CPRInTCD0 - CPRInTCD3 */
		addr = (&regs->cpri_tctrldata0);
		idx = channels[i]->word_bitmap;
		for (j = 1; j <= MAX_CWBYTES; j++) {
			mask = BYTE_MASK << mask;
			data = cw_read(&framer->regs_lock, addr, mask);
			channels[i]->words[idx][j-1] = data;
			mask += SIZE_BYTE;
			if (j % SIZE_REGBYTES == 0) {
				addr++;
				mask = 0;
			}
		}
	}

	return 0;
}

/* Read receive control word */
static int cpri_read_rxctrlword(struct cpri_framer *framer)
{
	unsigned int count;
	struct hf_ctrl_chans *hf_channels = &(framer->rx_hf_ctrl_chans);
	struct ctrl_chan **channels;
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int i, j;
	u32 mask = 0, *addr;
	u8 cw, idx, data;

	count = hf_channels->channel_count;
	channels = &(hf_channels->channels[0]);

	for (i = 0; i < count; i++) {

		if (channels[i] == NULL) {
			dev_err(dev, "Invalid channel container\n");
			return -ENOMEM;
		}
		/* Find control word */
		cw = FIND_CW_NUM(channels[i]->chan, channels[i]->word_bitmap);

		/* Write cw address in CPRInRCA */
		cpri_reg_set_val(&framer->regs_lock, &regs->cpri_rctrlattrib,
				TCT_ADDR_MASK, (u32) cw);

		/* Read data From CPRInRCD0 - CPRInRCD3 */
		addr = (&regs->cpri_rctrldata0);
		idx = channels[i]->word_bitmap;
		for (j = 1; j <= MAX_CWBYTES; j++) {
			mask = BYTE_MASK << mask;
			data = cw_read(&framer->regs_lock, addr, mask);
			channels[i]->words[idx][j-1] = data;
			mask += SIZE_BYTE;
			if (j % SIZE_REGBYTES == 0) {
				addr++;
				mask = 0;
			}
		}
	}

	return 0;
}

/* Set Tx Ethernet rate */
int set_txethrate(u8 eth_rate, struct cpri_framer *framer)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *eth_chan[1];
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;

	eth_chan[0] = kzalloc(sizeof(struct ctrl_chan), GFP_KERNEL);

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channels = &eth_chan[0];
	hf_channels->channels[0]->chan = 2;
	hf_channels->channels[0]->words[CWIDX_ETHPTR][WDOFF_B0] = eth_rate;
	hf_channels->channels[0]->word_bitmap = WORD_IDX3;
	hf_channels->channel_count = 1;

	if (cpri_write_txctrlword(framer) < 0) {
		dev_err(dev, "MEM error, not able to set control word\n");
		err = -ENOMEM;
	}

	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}
EXPORT_SYMBOL(set_txethrate);

/* Get Tx Ethernet rate */
int get_txethrate(struct cpri_framer *framer, u8 *eth_rate)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *eth_chan[1];
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;

	eth_chan[0] = kzalloc(sizeof(struct ctrl_chan), GFP_KERNEL);

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channels = &eth_chan[0];
	hf_channels->channels[0]->chan = 2;
	hf_channels->channels[0]->word_bitmap = WORD_IDX3;
	hf_channels->channel_count = 1;

	if (cpri_read_txctrlword(framer) < 0) {
		dev_err(dev, "MEM error, not able to get control word\n");
		err = -ENOMEM;
		goto out;
	}

	*eth_rate = hf_channels->channels[0]->words[CWIDX_ETHPTR][WDOFF_B0];

out:
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}
EXPORT_SYMBOL(get_txethrate);

/* Set Tx protocol version */
int set_txprotver(enum cpri_prot_ver ver, struct cpri_framer *framer)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *prtover_chan[1];
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;

	prtover_chan[0] = kzalloc(sizeof(struct ctrl_chan), GFP_KERNEL);

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channels = &prtover_chan[0];
	hf_channels->channels[0]->chan = 2;
	hf_channels->channels[0]->words[CWIDX_PROTVER][WDOFF_B0] = (u8)ver;
	hf_channels->channels[0]->word_bitmap = WORD_IDX0;
	hf_channels->channel_count = 1;

	if (cpri_write_txctrlword(framer) < 0) {
		dev_err(dev, "MEM error, not able to set control word\n");
		err = -ENOMEM;
	}

	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}
EXPORT_SYMBOL(set_txprotver);

/* Get Tx protocol version */
int get_txprotver(struct cpri_framer *framer,
			enum cpri_prot_ver *prot_ver)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *prtover_chan[1];
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;
	u8 ver;

	prtover_chan[0] = kzalloc(sizeof(struct ctrl_chan), GFP_KERNEL);

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->tx_hf_ctrl_chans);
	hf_channels->channels = &prtover_chan[0];
	hf_channels->channels[0]->chan = 2;
	hf_channels->channels[0]->word_bitmap = WORD_IDX0;
	hf_channels->channel_count = 1;

	if (cpri_read_txctrlword(framer) < 0) {
		dev_err(dev, "MEM error, not able to get control word\n");
		err = -ENOMEM;
	}

	ver = hf_channels->channels[0]->words[CWIDX_PROTVER][WDOFF_B0];

	raw_spin_unlock(&framer->tx_cwt_lock);
	*prot_ver = (ver == 1) ? VER_1 : VER_2;

	return err;
}
EXPORT_SYMBOL(get_txprotver);

/* Get Rx Ethernet rate */
int get_rxethrate(struct cpri_framer *framer, u8 *eth_rate)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *eth_chan[1];
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;

	eth_chan[0] = kzalloc(sizeof(struct ctrl_chan), GFP_KERNEL);

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->rx_hf_ctrl_chans);
	hf_channels->channels = &eth_chan[0];
	hf_channels->channels[0]->chan = 2;
	hf_channels->channels[0]->word_bitmap = WORD_IDX3;
	hf_channels->channel_count = 1;

	if (cpri_read_rxctrlword(framer) < 0) {
		dev_err(dev, "MEM error, not able to get control word\n");
		err = -ENOMEM;
		goto out;
	}

	*eth_rate = hf_channels->channels[0]->words[CWIDX_ETHPTR][WDOFF_B0];

out:
	raw_spin_unlock(&framer->tx_cwt_lock);

	return err;
}
EXPORT_SYMBOL(get_rxethrate);

/* Get Rx protocol version */
int get_rxprotver(struct cpri_framer *framer, enum cpri_prot_ver *prot_ver)
{
	struct hf_ctrl_chans *hf_channels;
	struct ctrl_chan *prtover_chan[1];
	struct device *dev = framer->cpri_dev->dev;
	int err = 0;
	u8 ver;

	prtover_chan[0] = kzalloc(sizeof(struct ctrl_chan), GFP_KERNEL);

	raw_spin_lock(&framer->tx_cwt_lock);

	hf_channels = &(framer->rx_hf_ctrl_chans);
	hf_channels->channels = &prtover_chan[0];
	hf_channels->channels[0]->chan = 2;
	hf_channels->channels[0]->word_bitmap = WORD_IDX0;
	hf_channels->channel_count = 1;

	if (cpri_read_rxctrlword(framer) < 0) {
		dev_err(dev, "MEM error, not able to get control word\n");
		err = -ENOMEM;
	}

	ver = hf_channels->channels[0]->words[CWIDX_PROTVER][WDOFF_B0];

	raw_spin_unlock(&framer->tx_cwt_lock);
	*prot_ver = (ver == 1) ? VER_1 : VER_2;

	return err;
}
EXPORT_SYMBOL(get_rxprotver);

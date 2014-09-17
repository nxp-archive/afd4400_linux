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

static u32 cw_read(u32 *addr, u32 mask)
{
	return cpri_reg_get_val(addr, mask);
}

/* clear and disable control table */
void clear_control_tx_table(struct cpri_framer *framer)
{
	u32 value = 0, i;
	struct cpri_framer_regs *regs = framer->regs;

	cpri_reg_clear(&regs->cpri_tctrldata0,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrldata1,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrldata2,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrldata3,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrlinserttb1,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrlinserttb2,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_config,
		TX_CW_INSERT_EN_MASK);

	/* Make sure that the above regs are updated */
	wmb();
	for (i = 0; i <= MAX_TCTA_ADDR; i++) {
		value = i << TCTA_ADDR_OFFSET;
		value |= TCT_WRITE_MASK;
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
				MASK_ALL, value);
	}
}

/* read_rx_cw - reads the rx control word
 * @bf_index: the 0-255 index number in one hyper frame.
 * @buf: pass 16 bytes to buf and data will be placed to it.
 */
void read_rx_cw(struct cpri_framer *framer, int bf_index, u8 *buf)
{
	struct cpri_framer_regs *regs = framer->regs;
	u32 *reg_addr;
	u32 data;
	int i;
	cpri_reg_set_val(&regs->cpri_rctrlattrib,
				TCT_ADDR_MASK, bf_index);
	reg_addr = (&regs->cpri_rctrldata0);
	wmb();
	/* data is in big endian */
	for (i = 0; i < 4; i++) {
		data = cw_read(reg_addr, MASK_ALL);
		*((u32 *)buf) = be32_to_cpu(data);
		buf += 4;
		reg_addr++;
	}
}

/* rdwr_tx_cw - read/write the tx control table.
 * @bf_index : 0-255 number in one hyper frame
 * @operation: use only one of the following:
 *	--TX_CW_READ: reads the tx control table,
 *	data will be put into buf.
 *	--TX_CW_WRITE: write to tx control table,
 *	using the 16 byte data in buf, the cpri control
 *	word will be updated.
 *	--TX_CW_BYPASS: this control word will be
 *	inserted from framer instead
 *	(eg. daisy chained or from framer's own reg).
 */
void rdwr_tx_cw(struct cpri_framer *framer,
		int bf_index, int operation, u8 *buf)
{
	struct cpri_framer_regs *regs = framer->regs;
	int i;
	u32 *reg_tcd0, *reg_tctie;
	int data;
	int mask = 0;

	reg_tcd0 = (&regs->cpri_tctrldata0);
	if (operation & TX_CW_READ) {
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
				MASK_ALL,
				bf_index << TCTA_ADDR_OFFSET);

		for (i = 0; i < 4; i++) {
			data = cw_read(reg_tcd0, MASK_ALL);
			*((u32 *)buf) = be32_to_cpu(data);
			buf += 4;
			reg_tcd0++;
		}
		return;
	}

	if ((bf_index >= 0) && (bf_index <= 15)) {
		reg_tctie = (&regs->cpri_tctrlinserttb1);
		mask = 1 << bf_index;
	} else if ((bf_index >= 64) && (bf_index <= 79)) {
		reg_tctie = (&regs->cpri_tctrlinserttb1);
		mask = 1 << (bf_index - 48);
	} else if ((bf_index >= 128) && (bf_index <= 143)) {
		reg_tctie = (&regs->cpri_tctrlinserttb2);
		mask = 1 << (bf_index - 128);
	} else if ((bf_index >= 192) && (bf_index <= 207)) {
		reg_tctie = (&regs->cpri_tctrlinserttb2);
		mask = 1 << (bf_index - 176);
	} else {
		reg_tctie = (&regs->cpri_tctrlinserttb1);
		mask = 0;
	}
	if (operation & TX_CW_BYPASS) {
		cpri_reg_clear(reg_tctie, mask);
	} else {
		cpri_reg_set(&regs->cpri_config,
				TX_CW_INSERT_EN_MASK);
		for (i = 0; i < 4; i++) {
			data = *((u32 *)buf);
			data = cpu_to_be32(data);
			cpri_reg_set_val(reg_tcd0,
			MASK_ALL, data);
			buf += 4;
			reg_tcd0++;
		}
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
			MASK_ALL,
			(bf_index << TCTA_ADDR_OFFSET) | TCT_WRITE_MASK);
		cpri_reg_set_val(reg_tctie,
			MASK_ALL, mask);
		return;
	}
}


/*
 * drivers/tbgen/tbgen.c
 *
 * Author: Freescale
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>

#include <linux/tbgen.h>

struct tbgen_dev *tbg;

/**@breif support function for driver
*/
static void write_reg(u32 *reg, unsigned int offset,
					unsigned int length, u32 *buf);
static void read_reg(u32 *reg, unsigned int offset,
					unsigned int length, u32 *buf);
static void notify_off_chip_devs(void);
static void force_jesd204_recapture(struct tbgen_dev *tbg);
static void ccm_tbgen_clock_src(u8 source);
static void ccm_tbgen_pll_enable(u8 enable);
/**@brief inline for set, clear and test bits for 32 bit
*/
static inline void
tclear_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) &= ~(1 << (nr & 31));
}

static inline void
tset_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) |= (1 << (nr & 31));
}

struct tbgen_dev *get_tbgen_device(void)
{
	if (tbg != NULL)
		return tbg;
	else
		return NULL;
}
EXPORT_SYMBOL(get_tbgen_device);

u32 get_ref_clock(struct tbgen_dev *tbg)
{
	u32 clock = 0;
	clock = tbg->tbgregs->refclk_per_10ms;
	return clock;
}
EXPORT_SYMBOL(get_ref_clock);

static void notify_off_chip_devs(void)
{
	/*dummy function to be implemented at latter stage*/
}

static void force_jesd204_recapture(struct tbgen_dev *tbg)
{
	/*raise user space event to recapture jesd204*/
}

static void ccm_tbgen_clock_src(u8 source)
{
	/*dummy function to be implemented at latter stage*/
}
static void ccm_tbgen_pll_enable(u8 enable)
{
	/*dummy function to be implemented at latter stage*/
}


static void write_reg(u32 *reg,
			unsigned int offset,
			unsigned int length,
			u32 *buf)
{
	int i;

	for (i = 0; i < offset; i++)
		reg++;

	for (i = 0; i < length; i++) {
		*reg = *buf;
		reg++;
		buf++;
	}
}

static void read_reg(u32 *reg,
			unsigned int offset,
			unsigned int length,
			u32 *buf)
{
	int i;

	for (i = 0; i < offset; i++)
		reg++;

	for (i = 0; i < length; i++) {
		*buf = *reg;
		reg++;
		buf++;
	}
}

static void handle_rfg_interrupt(struct tbgen_dev *tbg)
{
	u64 cnt_10ms = 0;
	spin_lock(&tbg->do_lock);
	/*read 32 bit of ts10mslo and then 32 bit if ts10mshi*/
	cnt_10ms = tbg->tbgregs->ts_10_ms_lo;
	cnt_10ms |= (tbg->tbgregs->ts_10_ms_hi << 31);

	notify_off_chip_devs();
	force_jesd204_recapture(tbg);
	spin_unlock(&tbg->do_lock);
}

int tbgen_pll_conf(struct tbgen_dev *tbg)
{
	u32 clock;
	switch (tbg->tbg_param.tbgen_pll.pll_mode) {
	case TB_REF_614_4:
		clock = 614;
		break;
	case TB_REF_983_04:
		clock = 983;
		break;
	default:
		clock = 614;
		break;
	}
	tbg->tbgregs->refclk_per_10ms = clock;

	ccm_tbgen_clock_src(tbg->tbg_param.tbgen_pll.clock_src);
	ccm_tbgen_pll_enable(tbg->tbg_param.tbgen_pll.tbgen_in_standby);

	return 0;
}

void rfg_enable(struct tbgen_dev *tbg, u8 flag)
{
	if (flag)
		tset_bit(RFGEN_BIT0, &tbg->tbgregs->rfg_cr);
	else
		tclear_bit(RFGEN_BIT0, &tbg->tbgregs->rfg_cr);
}

void tbgen_radio_frame_generation(struct tbgen_dev *tbg)
{
	/*mask for the first 5bits*/
	u32 syncadv_mask = tbg->rfg.sync_adv & 0x1f;

	tset_bit(SYNCOUT_CTRL_BIT18, &tbg->tbgregs->rfg_cr);
/**
 * Program the Initial Advance Timing for SYSREF output via RFGCR->SYNCADV
 * bit field register.
 * 19 to 23 bit of rfg_cr shall have this
 */
	syncadv_mask = (syncadv_mask << 19);
	tbg->tbgregs->rfg_cr |= syncadv_mask;

	tset_bit(FSIE_BIT5, &tbg->tbgregs->cntrl_1);
	rfg_enable(tbg, 1);
/*let isr do the rest*/
}

int tbgen_conf_rfg(struct tbgen_dev *tbg)
{
	int retcode = 0;
	u32 err_tres = tbg->rfg.drift_threshold;
	/*conf the rfg threshold*/
	/**
	 * Program the rfg threshold via RFGCR->THRES
	 * 4 to 15 bit of rfg_cr shall have this
	 */
	if (tbg->rfg.drift_threshold > 0) {
		err_tres = (tbg->rfg.drift_threshold << 4);
		tbg->tbgregs->rfg_cr |= err_tres;
		tset_bit(RFGERRIE_BIT4, &tbg->tbgregs->cntrl_1);
	}

	if (tbg->rfg.ref_syncsel)
		tset_bit(REFSYNCSEL_BIT2, &tbg->tbgregs->rfg_cr);
	else
		tclear_bit(REFSYNCSEL_BIT2, &tbg->tbgregs->rfg_cr);

	tbgen_radio_frame_generation(tbg);
	return retcode;
}
/**
 *
 *	@brief Writing ‘0’ and then ‘1’ to the Radio Frame Generator Enable bit
 *	of the Radio Frame Generator Control Register.
 */
int tbgen_restart_radio_frame_generation(struct tbgen_dev *tbg)
{
	int retcode = 0;

	rfg_enable(tbg, 0);
	retcode = tbgen_conf_rfg(tbg);
	rfg_enable(tbg, 1);

	return retcode;
}

u64 tbgen_get_master_counter(struct tbgen_dev *tbg)
{
	u64 master_counter = 0;
	/*read 32 bit of mstr_cnt_lo and then 32 bit if mstr_cnt_hi*/
	master_counter = tbg->tbgregs->mstr_cnt_lo;
	master_counter |= (tbg->tbgregs->mstr_cnt_hi << 31);

	return master_counter;
}


u64 tbgen_get_l10_mcntr(struct tbgen_dev *tbg)
{
	u64 ts_10ms_cnt = 0;

	/*read 32 bit of ts10mslo and then 32 bit if ts10mshi*/
	ts_10ms_cnt = tbg->tbgregs->ts_10_ms_lo;
	ts_10ms_cnt |= (tbg->tbgregs->ts_10_ms_hi << 31);

	return ts_10ms_cnt;
}

static void tbgen_isr_tasklet(unsigned long data)
{
	struct tbgen_dev *tbg = (struct tbgen_dev *)data;
	u32 ien = tbg->ien;
	u32 istat = 0;

	istat = tbg->tbgregs->int_stat;

	if ((ien & TBG_FISE) && (istat & TBG_FISE)) {
		tbg->mon_rfg_isr = tbg->mon_rfg_isr + 1;

		if (tbg->mon_rfg_isr == FIRST_PASS) {
			tclear_bit(REFSYNCSEL_BIT2, &tbg->tbgregs->rfg_cr);
		} else if (tbg->mon_rfg_isr == SECOND_PASS) {
			tclear_bit(FSIE_BIT5, &tbg->tbgregs->cntrl_1);
			/*int stat is a write to clear*/
			tset_bit(FSIE_BIT5, &tbg->tbgregs->int_stat);
			handle_rfg_interrupt(tbg);
		} else
			tbg->mon_rfg_isr = 0;
	}
}

void clear_interrupts(struct tbgen_dev *tbg)
{
	tclear_bit(FSIE_BIT5, &tbg->tbgregs->cntrl_1);
	tclear_bit(RFGERRIE_BIT4, &tbg->tbgregs->cntrl_1);
}

void enable_interrupts(struct tbgen_dev *tbg)
{
	tset_bit(FSIE_BIT5, &tbg->tbgregs->cntrl_1);
	tset_bit(RFGERRIE_BIT4, &tbg->tbgregs->cntrl_1);
}

static irqreturn_t do_isr(int irq, void *param)
{
	struct tbgen_dev *tbg = param;
	spin_lock(&tbg->isr_lock);
	if (tbg->state != RFG_START) {
		pr_err("Interrupt received for tbgen is in a non running state (%d)\n",
			tbg->state);
	/*
	* It is possible to really be running, i.e. we have re-loaded
	* a running card
	* Clear interrupt source
	*/
		clear_interrupts(tbg);
		return IRQ_HANDLED;
	}

	/* Clear interrupt source */
	clear_interrupts(tbg);
	/* Scehdule the bottom half of the ISR */
	tasklet_hi_schedule(&tbg->do_tasklet);
	spin_unlock(&tbg->isr_lock);
	enable_interrupts(tbg);
	return IRQ_HANDLED;
}

/** @brief register irq
 *  @params 1 tbgen device instance
 *  @return returns success or fail depedning on the kernel api's retrun
 */
static int tbgen_register_irq(struct tbgen_dev *tbg)
{
	int retcode = 0;

	tasklet_init(&tbg->do_tasklet,
			tbgen_isr_tasklet,
			(unsigned long)tbg);

	retcode = request_irq(tbg->irq,
				do_isr,
				0, /*no flag*/
				"tbgen",
				tbg);
	return retcode;
}


void tbgen_jesd_tx_alignment(struct tbgen_dev *tbg,
				struct tx_timer_conf *tmr)
{
	u64 ts_10ms_cnt = 0;
	u64 tx_start = 0;
	u32 timer_id = tmr->timer_id;

	memcpy(&tbg->tbg_txtmr[timer_id].tmr,
			tmr,
			sizeof(struct tx_timer_conf));
	/*read 32 bit of ts10mslo and then 32 bit if ts10mshi*/
	ts_10ms_cnt = tbg->tbgregs->ts_10_ms_lo;
	ts_10ms_cnt |= (tbg->tbgregs->ts_10_ms_hi << 31);

	tx_start = tmr->tx_offset + ts_10ms_cnt;

	/*configure the  osethi followed by osetlo
	do not swap*/
	tbg->tbgregs->tx_tmr[tmr->timer_id].align_oset_hi = (tx_start >> 31);
	tbg->tbgregs->tx_tmr[tmr->timer_id].align_oset_lo =
						(tx_start & 0xFFFFFFFF);

	/*configure ctrl*/
	tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl |=
						(tmr->ref_frm_clk << 8);
	/*polarity to 0*/
	tclear_bit(POLARITY_BIT7,
			&tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl);
	/*enable the sync error*/
	tset_bit(SYNCERR_BIT3,
			&tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl);

	tclear_bit(ISYNCBYP_BIT2,
			&tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl);
	tset_bit(OUTSEL_BIT1,
			&tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl);

	tset_bit(ENBALE_TMR_BIT0,
			&tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl);

	tbg->tbg_txtmr[timer_id].configured = 1;
}

void tbgen_jesd_axrf_alignment(struct tbgen_dev *tbg,
				struct axrf_timer_conf *tmr)
{
	u64 ts_10ms_cnt = 0;
	u64 tx_start = 0;
	u32 timer_id = tmr->timer_id;

	memcpy(&tbg->tbg_tx_axrf[timer_id].tmr,
			tmr,
			sizeof(struct tx_timer_conf));
	/*read 32 bit of ts10mslo and then 32 bit if ts10mshi*/
	ts_10ms_cnt = tbg->tbgregs->ts_10_ms_lo;
	ts_10ms_cnt |= (tbg->tbgregs->ts_10_ms_hi << 31);

	tx_start = tmr->tx_offset + ts_10ms_cnt;

	/*configure the  osethi followed by osetlo
	do not swap*/
	tbg->tbgregs->axrf_tmr[tmr->timer_id].align_oset_hi = (tx_start >> 31);
	tbg->tbgregs->axrf_tmr[tmr->timer_id].align_oset_lo =
						(tx_start & 0xFFFFFFFF);

	/*polarity to 0*/
	tclear_bit(POLARITY_BIT7,
			&tbg->tbgregs->axrf_tmr[tmr->timer_id].alig_ctrl);
	tset_bit(ENBALE_TMR_BIT0,
			&tbg->tbgregs->axrf_tmr[tmr->timer_id].alig_ctrl);

	tbg->tbg_tx_axrf[timer_id].configured = 1;
}

void tbgen_jesd_rx_alignment(struct tbgen_dev *tbg, struct rx_timer_conf *tmr)
{
	u64 ts_10ms_cnt = 0;
	u64 rx_start = 0;
	u32 timer_id = tmr->timer_id;

	memcpy(&tbg->tbg_rxtmr[timer_id].tmr,
			tmr,
			sizeof(struct rx_timer_conf));

	/*disable the timer*/
	tclear_bit(0, &tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl);

	/*read 32 bit of ts10mslo and then 32 bit if ts10mshi*/
	ts_10ms_cnt = tbg->tbgregs->ts_10_ms_lo;
	ts_10ms_cnt |= (tbg->tbgregs->ts_10_ms_hi << 31);

	rx_start = tmr->rx_offset + ts_10ms_cnt;

	tbg->tbgregs->rx_tmr[tmr->timer_id].align_oset_hi = (rx_start >> 31);
	tbg->tbgregs->rx_tmr[tmr->timer_id].align_oset_lo =
						(rx_start & 0xFFFFFFFF);

	/** @brief for the rx aling ctrl register
	 * clear before we write into it
	 * polarity to 0 for 0 - 1 trigger
	 * one shot = 1
	 * strb mode to 00
	 */
	tbg->tbgregs->rx_tmr[tmr->timer_id].alig_ctrl = 0;

	tclear_bit(POLARITY_BIT7,
			&tbg->tbgregs->rx_tmr[tmr->timer_id].alig_ctrl);

	if (tmr->oneshot_mode)
		tset_bit(6, &tbg->tbgregs->rx_tmr[tmr->timer_id].alig_ctrl);
	else
		tclear_bit(6, &tbg->tbgregs->rx_tmr[tmr->timer_id].alig_ctrl);

	/*mask the 4th and 5th bit for strobe*/
	tbg->tbgregs->rx_tmr[tmr->timer_id].alig_ctrl |=
					((tmr->strobe_mode & 0x3) << 4);

	tset_bit(0, &tbg->tbgregs->rx_tmr[tmr->timer_id].alig_ctrl);
	tbg->tbg_rxtmr[timer_id].configured = 1;
}


void tbgen_jesd_srx_alignment(struct tbgen_dev *tbg,
						struct srx_timer_conf *tmr)
{
	u64 ts_10ms_cnt = 0;
	u64 srx_start = 0;
	u32 timer_id = tmr->timer_id;

	memcpy(&tbg->tbg_srxtmr[timer_id].tmr,
			tmr,
			sizeof(struct srx_timer_conf));

	/*disable the timer*/
	tclear_bit(0, &tbg->tbgregs->tx_tmr[tmr->timer_id].alig_ctrl);

	/*read 32 bit of ts10mslo and then 32 bit if ts10mshi*/
	ts_10ms_cnt = tbg->tbgregs->ts_10_ms_lo;
	ts_10ms_cnt |= (tbg->tbgregs->ts_10_ms_hi << 31);

	srx_start = tmr->srx_offset + ts_10ms_cnt;

	tbg->tbgregs->srx_tmr[tmr->timer_id].align_oset_hi = (srx_start >> 31);
	tbg->tbgregs->srx_tmr[tmr->timer_id].align_oset_lo =
						(srx_start & 0xFFFFFFFF);

	if (tmr->oneshot_mode == 0)
		tbg->tbgregs->srx_tmr[tmr->timer_id].align_intrvl =
						tmr->alignment_interval;


	tbg->tbgregs->srx_tmr[tmr->timer_id].alig_ctrl = 0;

	tclear_bit(POLARITY_BIT7,
			&tbg->tbgregs->rx_tmr[tmr->timer_id].alig_ctrl);

	if (tmr->oneshot_mode)
		tset_bit(6, &tbg->tbgregs->srx_tmr[tmr->timer_id].alig_ctrl);
	else
		tclear_bit(6, &tbg->tbgregs->srx_tmr[tmr->timer_id].alig_ctrl);


	/*mask the 4th and 5th bit for strobe*/
	tbg->tbgregs->srx_tmr[tmr->timer_id].alig_ctrl |=
					((tmr->strobe_mode & 0x3) << 4);
	/*mask the 8th - 11th bit for pulsewidth*/
	tbg->tbgregs->srx_tmr[tmr->timer_id].alig_ctrl |=
					((tmr->tmr_pulsewidth & 0xf) << 8);

	tset_bit(0, &tbg->tbgregs->srx_tmr[tmr->timer_id].alig_ctrl);

	tbg->tbg_srxtmr[timer_id].configured = 1;
}

void tbgen_isrmask(struct tbgen_dev *tbg)
{
	if (tbg->c_irq.rfg_err_ie == 1)
		tset_bit(4, &tbg->tbgregs->cntrl_1);
	else
		tclear_bit(4, &tbg->tbgregs->cntrl_1);

	if (tbg->c_irq.frm_sync_ie == 1)
		tset_bit(5, &tbg->tbgregs->cntrl_1);
	else
		tclear_bit(5, &tbg->tbgregs->cntrl_1);

	if (tbg->c_irq.re_sync_ie == 1)
		tset_bit(25, &tbg->tbgregs->cntrl_1);
	else
		tclear_bit(25, &tbg->tbgregs->cntrl_1);
}

int tx_realign(struct tbgen_dev *tbg, u32 timer_id)
{
	int retcode = 0;
	if (timer_id <= MAX_TX_ALIGNMENT_TIMERS &&
		tbg->tbg_txtmr[timer_id].configured == 1) {



		tclear_bit(ENBALE_TMR_BIT0,
			&tbg->tbgregs->tx_tmr[timer_id].alig_ctrl);
		tbg->tbgregs->tx_tmr[timer_id].alig_ctrl = 0;

		tbg->tbgregs->tx_tmr[timer_id].align_oset_hi = 0;
		tbg->tbgregs->tx_tmr[timer_id].align_oset_lo = 0;

		tbgen_jesd_tx_alignment(tbg, &tbg->tbg_txtmr[timer_id].tmr);

		spin_unlock(&tbg->tbg_txtmr[timer_id].lock_tx_tmr);
	} else
		retcode = -EINVAL;
	return retcode;
}

int tx_axrf_realign(struct tbgen_dev *tbg, u32 timer_id)
{
	int retcode = 0;

	if (timer_id <= MAX_AXRF_ALIGNMENT_TIMERS &&
		tbg->tbg_tx_axrf[timer_id].configured == 1) {

		spin_lock(&tbg->tbg_tx_axrf[timer_id].lock_axrf_tmr);

		tclear_bit(ENBALE_TMR_BIT0,
			&tbg->tbgregs->axrf_tmr[timer_id].alig_ctrl);
		tbg->tbgregs->axrf_tmr[timer_id].alig_ctrl = 0;
		tbg->tbgregs->axrf_tmr[timer_id].align_oset_hi = 0;
		tbg->tbgregs->axrf_tmr[timer_id].align_oset_lo = 0;

		tbgen_jesd_axrf_alignment(tbg, &tbg->tbg_tx_axrf[timer_id].tmr);

		spin_unlock
			(&tbg->tbg_tx_axrf[timer_id].lock_axrf_tmr);
	} else
		retcode = -EINVAL;
	return retcode;
}

int rx_realign(struct tbgen_dev *tbg, u32 timer_id)
{
	int retcode = 0;

	if (timer_id <= MAX_RX_ALIGNMENT_TIMERS &&
		tbg->tbg_rxtmr[timer_id].configured == 1) {

		spin_lock(&tbg->tbg_rxtmr[timer_id].lock_rx_tmr);

		tclear_bit(ENBALE_TMR_BIT0,
				&tbg->tbgregs->rx_tmr[timer_id].alig_ctrl);
		tbg->tbgregs->rx_tmr[timer_id].alig_ctrl = 0;
		tbg->tbgregs->rx_tmr[timer_id].align_oset_hi = 0;
		tbg->tbgregs->rx_tmr[timer_id].align_oset_lo = 0;

		tbgen_jesd_rx_alignment(tbg, &tbg->tbg_rxtmr[timer_id].tmr);

		spin_unlock(&tbg->tbg_rxtmr[timer_id].lock_rx_tmr);
	} else
		retcode = -EINVAL;
	return retcode;
}

int srx_realign(struct tbgen_dev *tbg, u32 timer_id)
{
	int retcode = 0;

	if (timer_id <= MAX_SRX_ALIGNMENT_TIMERS &&
		tbg->tbg_srxtmr[timer_id].configured == 1) {

		spin_lock(&tbg->tbg_srxtmr[timer_id].lock_srx_tmr);

		tclear_bit(ENBALE_TMR_BIT0,
			&tbg->tbgregs->srx_tmr[timer_id].alig_ctrl);
		tbg->tbgregs->srx_tmr[timer_id].alig_ctrl = 0;
		tbg->tbgregs->srx_tmr[timer_id].align_oset_hi = 0;
		tbg->tbgregs->srx_tmr[timer_id].align_oset_lo = 0;

		tbgen_jesd_srx_alignment(tbg, &tbg->tbg_srxtmr[timer_id].tmr);

		spin_unlock(&tbg->tbg_srxtmr[timer_id].lock_srx_tmr);
	} else
		retcode = -EINVAL;
	return retcode;
}

int disable_timer(struct tbgen_dev *tbg, uint8_t type, uint8_t timer_id)
{
	int ret = 0;
	switch (type) {
	case JESD_TX_ALIGNMENT:
		if (tbg->tbg_txtmr[timer_id].configured == 1) {
			tclear_bit(ENBALE_TMR_BIT0,
				&tbg->tbgregs->tx_tmr[timer_id].alig_ctrl);
			tbg->tbg_txtmr[timer_id].configured = 0;
		} else
			ret = -EINVAL;
		break;
	case JESD_TX_AXRF:
		if (tbg->tbg_tx_axrf[timer_id].configured == 1) {
			tclear_bit(ENBALE_TMR_BIT0,
				&tbg->tbgregs->axrf_tmr[timer_id].alig_ctrl);
			tbg->tbg_tx_axrf[timer_id].configured = 0;
		} else
			ret = -EINVAL;
		break;
	case JESD_RX_ALIGNMENT:
		if (tbg->tbg_rxtmr[timer_id].configured == 1) {
			tclear_bit(ENBALE_TMR_BIT0,
				&tbg->tbgregs->rx_tmr[timer_id].alig_ctrl);
			tbg->tbg_rxtmr[timer_id].configured = 0;
		} else
			ret = -EINVAL;
	case JESD_SRX_ALIGNMENT:
		if (tbg->tbg_srxtmr[timer_id].configured == 1) {
			tclear_bit(ENBALE_TMR_BIT0,
				&tbg->tbgregs->srx_tmr[timer_id].alig_ctrl);
			tbg->tbg_srxtmr[timer_id].configured = 0;
		} else

			ret = -EINVAL;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

void write_multiple_regs(struct tbgen_dev *tbg,
			u32 offset,
			u32 len,
			u32 value)
{
	write_reg(&tbg->tbgregs->rfg_cr, offset, len, &value);
}

int config_alignment_timers(struct tbgen_dev *tbg,
				struct jesd_align_timer_params *tmr_prams)
{
	int retcode = 0;
	struct tx_timer_conf *tx_tmr;
	struct axrf_timer_conf *axrf_tmr;
	struct rx_timer_conf *rx_tmr;
	struct srx_timer_conf *srx_tmr;

	switch (tmr_prams->type) {
	case JESD_TX_ALIGNMENT:
		spin_lock(&tbg->tbg_txtmr[tmr_prams->id].lock_tx_tmr);
		tx_tmr = kzalloc(sizeof(struct tx_timer_conf), GFP_KERNEL);

		if (tx_tmr == NULL)
			retcode = -ENOMEM;

		tx_tmr->timer_id = tmr_prams->id;
		tx_tmr->tx_offset = tmr_prams->jx_start_tmr;
		tx_tmr->ref_frm_clk = 0; /*TBD is there a need ?*/

		if (tx_tmr->timer_id <= MAX_TX_ALIGNMENT_TIMERS)
			tbgen_jesd_tx_alignment(tbg, tx_tmr);
		else
			retcode = -EINVAL;
		kfree(tx_tmr);
		spin_unlock(&tbg->tbg_txtmr[tmr_prams->id].lock_tx_tmr);
		break;
	case JESD_TX_AXRF:
		spin_lock(&tbg->tbg_tx_axrf[tmr_prams->id].lock_axrf_tmr);
		axrf_tmr = kzalloc(sizeof(struct axrf_timer_conf), GFP_KERNEL);

		if (axrf_tmr == NULL)
			retcode = -ENOMEM;

		axrf_tmr->timer_id = tmr_prams->id;
		axrf_tmr->tx_offset = tmr_prams->jx_start_tmr;

		if (axrf_tmr->timer_id <= MAX_TX_ALIGNMENT_TIMERS)
			tbgen_jesd_axrf_alignment(tbg, axrf_tmr);
		else
			retcode = -EINVAL;

		kfree(axrf_tmr);
		spin_unlock(&tbg->tbg_tx_axrf[tmr_prams->id].lock_axrf_tmr);
		break;
	case JESD_RX_ALIGNMENT:
		spin_lock(&tbg->tbg_rxtmr[tmr_prams->id].lock_rx_tmr);
		rx_tmr = kzalloc(sizeof(struct rx_timer_conf), GFP_KERNEL);
		if (rx_tmr == NULL)
			retcode = -ENOMEM;

		rx_tmr->timer_id = tmr_prams->id;
		rx_tmr->rx_offset = tmr_prams->jx_start_tmr;
		rx_tmr->strobe_mode = tmr_prams->timer.jxstrobe;
		rx_tmr->oneshot_mode = tmr_prams->timer.perodic;

		if (rx_tmr->timer_id <= MAX_TX_ALIGNMENT_TIMERS)
			tbgen_jesd_rx_alignment(tbg, rx_tmr);
		else
			retcode = -EINVAL;

		kfree(rx_tmr);
		spin_unlock(&tbg->tbg_rxtmr[tmr_prams->id].lock_rx_tmr);
		break;
	case JESD_SRX_ALIGNMENT:
		spin_lock(&tbg->tbg_srxtmr[tmr_prams->id].lock_srx_tmr);
		srx_tmr = kzalloc(sizeof(struct srx_timer_conf), GFP_KERNEL);

		if (srx_tmr == NULL)
			retcode = -ENOMEM;

		srx_tmr->timer_id = tmr_prams->id;
		srx_tmr->srx_offset = tmr_prams->jx_start_tmr;
		srx_tmr->strobe_mode = tmr_prams->timer.jxstrobe;
		srx_tmr->oneshot_mode = tmr_prams->timer.perodic;
		srx_tmr->alignment_interval = tmr_prams->timer.perodic_int;

		if (srx_tmr->timer_id <= MAX_TX_ALIGNMENT_TIMERS)
			tbgen_jesd_srx_alignment(tbg, srx_tmr);
		else
			retcode = -EINVAL;
		kfree(srx_tmr);
		spin_unlock(&tbg->tbg_srxtmr[tmr_prams->id].lock_srx_tmr);
		break;
	default:
		pr_err("Ioctl alignment timer not supported %d\n",
						tmr_prams->type);
		retcode = -ENOTTY;
		break;
	}
	return retcode;
}

int reconfig_alignement_timers(struct tbgen_dev *tbg,
				struct jesd_align_timer_params *tmr_params)
{
	int retcode = 0;
	switch (tmr_params->type) {
	case JESD_TX_ALIGNMENT:
		spin_lock(&tbg->tbg_txtmr[tmr_params->id].lock_tx_tmr);
		retcode = tx_realign(tbg, tmr_params->id);
		spin_unlock(&tbg->tbg_txtmr[tmr_params->id].lock_tx_tmr);
		break;
	case JESD_TX_AXRF:
		spin_lock(&tbg->tbg_tx_axrf[tmr_params->id].lock_axrf_tmr);
		retcode = tx_axrf_realign(tbg, tmr_params->id);
		spin_unlock(&tbg->tbg_tx_axrf[tmr_params->id].lock_axrf_tmr);
		break;
	case JESD_RX_ALIGNMENT:
		spin_lock(&tbg->tbg_rxtmr[tmr_params->id].lock_rx_tmr);
		retcode = rx_realign(tbg, tmr_params->id);
		spin_unlock(&tbg->tbg_rxtmr[tmr_params->id].lock_rx_tmr);
		break;
	case JESD_SRX_ALIGNMENT:
		spin_lock(&tbg->tbg_srxtmr[tmr_params->id].lock_srx_tmr);
		retcode = srx_realign(tbg, tmr_params->id);
		spin_unlock(&tbg->tbg_srxtmr[tmr_params->id].lock_srx_tmr);
		break;
	default:
		pr_err("Ioctl realignment timer not supported %d\n",
							tmr_params->type);
	break;
	}
	return retcode;
}

long tbgen_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	int retcode = -ENOSYS;
	u64 master_counter = 0;
	u64 l10_counter = 0;
	int count = 0;
	struct tbgen_dev *tbg;
	struct jesd_align_timer_params *set_params, *reset_params;
	struct tbgen_reg_read_buf *regcnf;
	struct tbgen_device_info dev_info;
	struct timer_disable tmr_disable;
	struct tbgen_reg_write_buf *wreg_buf;
	struct tbgen_reg_write_buf write_reg;
	void __user *argp = (void __user *)arg;

	tbg = pfile->private_data;

	if (tbg != NULL) {
		switch (cmd) {
		case TBGEN_SET_PLL:
			if (copy_from_user(&tbg->tbg_param,
				(struct tbgen_param *)arg,
				sizeof(struct tbgen_param))) {
				retcode = -EFAULT;
				break;
			}
			retcode = tbgen_pll_conf(tbg);
			if (retcode < 0)
				break;
			else
				tbg->state = PLL_LOCKED;
			break;
		case TBGEN_RFG:
			tbg->mon_rfg_isr = 0;

			if (copy_from_user(&tbg->rfg,
				(struct tbg_rfg *)arg,
				sizeof(struct tbg_rfg))) {
				retcode = -EFAULT;
				break;
			}

			retcode = tbgen_conf_rfg(tbg);
			if (retcode < 0)
				break;
			else
				tbg->state = RFG_START;
			break;
		case TBGEN_RFG_RESET:
			tbg->mon_rfg_isr = 0;

			if (copy_from_user(&tbg->rfg,
				(struct tbg_rfg *)arg,
				sizeof(struct tbg_rfg))) {
				retcode = -EFAULT;
				break;
			}

			retcode = tbgen_restart_radio_frame_generation(tbg);
			if (retcode < 0)
				break;
			else
				tbg->state = RFG_RESET;
			break;
		case TBGEN_RFG_ENABLE:
			rfg_enable(tbg, 1);
			break;
		case TBGEN_ALIGNMENT_TIMERS:
			set_params = kzalloc
				(sizeof(struct jesd_align_timer_params),
				GFP_KERNEL);
			if (set_params == NULL)
				retcode = -ENOMEM;

			if (copy_from_user(set_params,
				(struct jesd_align_timer_params *)arg,
				sizeof(struct jesd_align_timer_params))) {
				retcode = -EFAULT;
				break;
			}
			retcode = config_alignment_timers(tbg, set_params);
			kfree(set_params);
			break;
		case TBGEN_REALIGNMENT_TIMERS:
			reset_params = kzalloc
				(sizeof(struct jesd_align_timer_params),
				GFP_KERNEL);
			if (reset_params == NULL)
				retcode = -ENOMEM;

			if (copy_from_user
				(reset_params,
				(struct jesd_align_timer_params *)arg,
				sizeof(struct jesd_align_timer_params))) {
				retcode = -EFAULT;
				break;
			}
			retcode = reconfig_alignement_timers(tbg, reset_params);
			kfree(reset_params);
			break;
		case TBGEN_DISABLE_TIMER_INSTANCE:
			if (copy_from_user(&tmr_disable,
				(struct irq_conf *)arg,
				sizeof(struct timer_disable))) {
				retcode = -EFAULT;
				break;
			}
			retcode = disable_timer(tbg,
					tmr_disable.tmr_type,
					tmr_disable.timer_identifier);
			break;
		case TBGEN_ISR_MASK:
			if (copy_from_user(&tbg->c_irq,
				(struct irq_conf *)arg,
				sizeof(struct irq_conf))) {
				retcode = -EFAULT;
				break;
			}
			tbgen_isrmask(tbg);
			break;
		case TBGEN_WRITE_REG:
			wreg_buf = (struct tbgen_reg_write_buf *) arg;
			count = wreg_buf->count;

			write_reg.regs =
				kzalloc((sizeof(struct tbgen_wreg) * count),
					GFP_KERNEL);

			if (copy_from_user(&write_reg,
				(struct tbgen_reg_write_buf *)argp,
				sizeof(struct tbgen_reg_write_buf)*count)) {
					retcode = -EFAULT;
					break;
			}

			while (count) {
				write_multiple_regs(tbg,
						write_reg.regs->offset,
						1,
						write_reg.regs->value);
				write_reg.regs++;
				count--;
			}

			kfree(write_reg.regs);
			break;
		case TBGEN_READ_REG:
			regcnf = kzalloc(sizeof(struct tbgen_reg_read_buf),
						GFP_KERNEL);
				if (!regcnf) {
					retcode = -ENOMEM;
					break;
				}
			regcnf->buf = kzalloc(sizeof(struct tbg_regs),
								GFP_KERNEL);
			if (!regcnf->buf) {
				retcode = -ENOMEM;
				break;
			}

			if (copy_from_user(&regcnf, (struct reg_conf *)arg,
					sizeof(struct tbgen_reg_read_buf))) {
					retcode = -EFAULT;
					break;
			}

			if (regcnf->len > sizeof(struct tbg_regs)) {
				regcnf->len = sizeof(struct tbg_regs);
				regcnf->offset = 0;
			}
			read_reg(&tbg->tbgregs->rfg_cr, regcnf->offset,
						regcnf->len, regcnf->buf);

			if (copy_to_user((struct reg_conf *)arg, &regcnf,
					sizeof(struct tbgen_reg_read_buf)))
				retcode = -EFAULT;

			kfree(regcnf->buf);
			kfree(regcnf);
			break;
		case TBGEN_GET_DEV_INFO:
			/*append any details for dev info added in here*/
			dev_info.state = tbg->state;

			if (copy_to_user((struct tbgen_device_info *)arg,
					&dev_info,
					sizeof(struct tbgen_device_info)))
				retcode = -EFAULT;
			break;
		case TBGEN_GET_MASTER_COUNTER:
			master_counter = tbgen_get_master_counter(tbg);

			if (copy_to_user((u64 *)arg, &master_counter,
							sizeof(master_counter)))
				retcode = -EFAULT;
			break;
		case TBGEN_GET_L10MCNTR:
			l10_counter = tbgen_get_master_counter(tbg);

			if (copy_to_user((u64 *)arg, &l10_counter,
							sizeof(l10_counter)))
				retcode = -EFAULT;
			break;
		case TBGEN_RESET:
			tset_bit(SWRST_BIT0, &tbg->tbgregs->cntrl_0);
			break;
		default:
			retcode = -ENOTTY;
			break;
		}
	}

	return retcode;
}

/**
 * @brief Called whenever apps performs an open on /dev/tbgen.
 *
 * \param inode.
 * \param file ptr.
 * \return zero on success or negative number on failure.
 */
static int tbgen_open(struct inode *inode, struct file *pfile)
{
	struct tbgen_dev *tbg = NULL;
	int retcode = 0;

	tbg = container_of(inode->i_cdev, struct tbgen_dev, c_dev);
	if (tbg != NULL) {
		pfile->private_data = tbg;
		tbg->state = TBGEN_STANDBY;
	} else
		retcode = -ENODEV;

	return retcode;
}

/**
 * @brief Called whenever apps performs an close on /dev/tbgen.
 *
 * \param inode.
 * \param file ptr.
 * \return zero on success or negative number on failure.
 */
int tbgen_release(struct inode *inode, struct file *pfile)
{
	int retcode = 0;
	struct tbgen_dev *tbg = NULL;

	tbg = pfile->private_data;
	pfile->private_data = NULL;

	if (tbg == NULL)
		retcode = -ENODEV;

	return retcode;
}

/** @brief file ops structure
 */
static const struct file_operations tbg_fops = {
	.owner = THIS_MODULE,
	.open = tbgen_open,
	.unlocked_ioctl = tbgen_ioctl,
	.release = tbgen_release
};

static struct miscdevice tbg_miscdev = {
	MISC_DYNAMIC_MINOR,
	"tbgen",
	&tbg_fops
};
/** @brief probe function call for tbgen, the call is expected to be once (DTS)
 *
 * \params platform device instance given from platform base
 * \return zero on success or negative number on failure.
 */

static int __init tbgen_of_probe(struct platform_device *pdev)
{
	int retcode = 0;
	int id;

	dev_info(&pdev->dev, "fsl,Tbgen: Probe entry\n");

	tbg = kzalloc(sizeof(struct tbgen_dev), GFP_KERNEL);

	if (!tbg) {
		retcode = -ENOMEM;
		goto tbginit_err0;
	}

	tbg->node = pdev->dev.of_node;
	tbg->dev = &pdev->dev;

	retcode = of_address_to_resource(tbg->node, 0, &tbg->res);
	if (retcode < 0)
		goto tbginit_err1;

	tbg->tbgregs = of_iomap(tbg->node, 0);

	if (!tbg->tbgregs)
		goto tbginit_err2;

	tbg->irq = irq_of_parse_and_map(tbg->node, 0);
	retcode = tbgen_register_irq(tbg);

	retcode = misc_register(&tbg_miscdev);

	if (retcode < 0) {
		dev_info(&pdev->dev, "error %d misc register\n", retcode);
		goto tbginit_err3;
	}

	spin_lock_init(&tbg->do_lock);
	spin_lock_init(&tbg->isr_lock);

	for (id = 0; id < MAX_TX_ALIGNMENT_TIMERS; id++)
		spin_lock_init(&tbg->tbg_txtmr[id].lock_tx_tmr);

	for (id = 0; id < MAX_AXRF_ALIGNMENT_TIMERS; id++)
		spin_lock_init(&tbg->tbg_tx_axrf[id].lock_axrf_tmr);

	for (id = 0; id < MAX_RX_ALIGNMENT_TIMERS; id++)
		spin_lock_init(&tbg->tbg_rxtmr[id].lock_rx_tmr);

	for (id = 0; id < MAX_SRX_ALIGNMENT_TIMERS; id++)
		spin_lock_init(&tbg->tbg_srxtmr[id].lock_srx_tmr);

	dev_info(&pdev->dev, "fsl, Tbgen: probe success\n");
	return retcode;

tbginit_err3:
		iounmap(tbg->tbgregs);
		dev_info(&pdev->dev, "fsl, Tbgen: tbginit error 3");
tbginit_err2:
		release_mem_region(tbg->res.start, resource_size(&tbg->res));
		dev_info(&pdev->dev, "fsl, Tbgen: tbginit error 2");
tbginit_err1:
		kfree(tbg);
		dev_info(&pdev->dev, "fsl, Tbgen: tbginit error 1");
tbginit_err0:
		dev_info(&pdev->dev, "fsl, Tbgen: tbginit error 0");
		return retcode;
}

/** @brief remove function call for tbgen
 *
 *  \params platform device instance given from platform base
 *  \return zero on success or negative number on failure.
 */
static int __exit tbgen_of_remove(struct platform_device *pdev)
{
	int retcode = 0;
	struct tbgen_dev *tbg = NULL;
	u32 id;

	if (tbg != NULL) {
		for (id = 0; id < MAX_TX_ALIGNMENT_TIMERS; id++)
			disable_timer(tbg, JESD_TX_ALIGNMENT, id);

		for (id = 0; id < MAX_AXRF_ALIGNMENT_TIMERS; id++)
			disable_timer(tbg, JESD_TX_AXRF, id);

		for (id = 0; id < MAX_RX_ALIGNMENT_TIMERS; id++)
			disable_timer(tbg, JESD_RX_ALIGNMENT, id);

		for (id = 0; id < MAX_SRX_ALIGNMENT_TIMERS; id++)
			disable_timer(tbg, JESD_SRX_ALIGNMENT, id);

		misc_deregister(&tbg_miscdev);

		release_mem_region(tbg->res.start, resource_size(&tbg->res));
		kfree(tbg);
	} else
		retcode = -EINVAL;

/*tbgrm_err:*/
	return retcode;
}

static struct of_device_id tbgen_match[] = {
	{.compatible = "fsl,d4400-tbgen",},
	{},
};

static struct platform_driver tbgen_driver = {
	.driver = {
		.name = "fsl-tbgen",
		.owner = THIS_MODULE,
		.of_match_table = tbgen_match,
	},
	.probe = tbgen_of_probe,
	.remove = tbgen_of_remove,
};


/** @brief entry point and registration platform
 *
 *  \return zero on success or negative number on failure.
 */
static int __init tbgen_module_init(void)
{
	return platform_driver_register(&tbgen_driver);
}

/** @brief exit point and un-registration of platform
 *
 *  \return zero on success or negative number on failure.
 */

static void __exit tbgen_module_clean(void)
{
	platform_driver_unregister(&tbgen_driver);
}

module_init(tbgen_module_init);
module_exit(tbgen_module_clean);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("tbgen driver");

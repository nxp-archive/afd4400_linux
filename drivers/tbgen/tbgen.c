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
#include <linux/jesd204.h>

struct tbgen_dev *tbg;

/**@breif support function for driver
*/
static void write_reg(u32 *reg, unsigned int offset,
					unsigned int length, u32 *buf);
static int tbgen_change_state(struct tbgen_dev *tbg,
	enum tbgen_dev_state new_state);

static int tbgen_timer_ctrl(struct tbgen_dev *tbg, enum timer_type type,
	int timer_id, int enable);
static u64 tbgen_get_l10_mcntr(struct tbgen_dev *tbg);

static void tbgen_write_reg(u32 *reg, u32 val)
{
	iowrite32(val, reg);
}

static u32 tbgen_read_reg(u32 *reg)
{
	return ioread32(reg);
}

static void tbgen_update_reg(u32 *reg, u32 val, u32 mask)
{
	u32 reg_val;

	reg_val = ioread32(reg);
	reg_val &= ~mask;
	reg_val |= (val & mask);
	iowrite32(reg_val, reg);
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
	return tbg->refclk_khz;
}
EXPORT_SYMBOL(get_ref_clock);

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

static int tbgen_handle_rfg_irq(struct tbgen_dev *tbg)
{
	int disable_fs_irq = 0;
	u32 *reg, val;
	unsigned long flags;

	spin_lock_irqsave(&tbg->lock, flags);

	if (tbg->sync_state == SYNC_CPRI_RX_RFP) {
		/* Change SYNC source to SYSREF_IN */
		reg = &tbg->tbgregs->rfg_cr;
		val = tbgen_read_reg(reg);
		val &= ~SYNC_SEL_CPRI_RX_RFP;
		tbgen_write_reg(reg, val);
		tbg->sync_state = SYNC_SYSREF_IN_CONFIGURE;

	} else if (tbg->sync_state == SYNC_SYSREF_IN_CONFIGURE) {

		tbg->last_10ms_counter = tbgen_get_l10_mcntr(tbg);
		disable_fs_irq = 1;
		tbg->sync_state = SYNC_SYSREF_IN_CONFIGURED;
		/*XXX: Notify JESD to recatture SYNC*/
	}

	spin_unlock_irqrestore(&tbg->lock, flags);

	return disable_fs_irq;
}

int validate_refclk(struct tbgen_dev *tbg, u32 ref_clk)
{
	int retcode = -EINVAL;

	switch (ref_clk) {
	case REF_CLK_614MHZ:
	case REF_CLK_983MHZ:
	case REF_CLK_122MHZ:
		retcode = 0;
		break;
	default:
		dev_err(tbg->dev, "Invalid ref clk %d khz\n", ref_clk);
		retcode = -EINVAL;
		break;
	};

	return retcode;
}

int tbgen_pll_init(struct tbgen_dev *tbg, struct tbg_pll *pll_params)
{
	int retcode = 0;
	u32 val, *reg;
	struct tbg_regs *tbgregs = tbg->tbgregs;

	retcode = validate_refclk(tbg, pll_params->refclk_khz);
	if (retcode)
		goto out;

	/*Refclk cycles per 10 ms*/
	val = (pll_params->refclk_khz * 10) & REFCLK_PER_10MS_MASK;
	reg = &tbgregs->refclk_per_10ms;
	tbgen_write_reg(reg, val);

	tbg->refclk_khz = pll_params->refclk_khz;

	/* XXX: TBD - Enable/programe TBGEN PLL using, using clk_get/enable.
	 * For medusa programming TBGEN PLL is not required
	 */
out:
	return retcode;
}

int rfg_enable(struct tbgen_dev *tbg, u8 enable)
{
	int retcode = 0;
	struct tbg_regs *tbgregs = tbg->tbgregs;
	u32 val, *reg, mask;
	unsigned long flags;

	if (enable && (tbg->state != TBG_STATE_CONFIGURED)) {
		dev_err(tbg->dev, "Enable Failed, tbgen is not configured\n");
		retcode = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&tbg->lock, flags);

	if (enable) {
		tbg->sync_state = SYNC_CPRI_RX_RFP;
		/* Enable Frame sync IRQ */
		reg = &tbgregs->cntrl_1;
		val = IRQ_FS;
		mask = IRQ_FS;
		tbgen_update_reg(reg, val, mask);

		reg = &tbgregs->rfg_cr;
		val = RFGEN;
		mask = RFGEN;
		tbgen_update_reg(reg, val, mask);
	} else {
		/* FSIE should be disabled after first frame sync interrupt
		 * but disabling it just in case it was enabled for some
		 * reason
		 */
		reg = &tbgregs->cntrl_1;
		val = ~IRQ_FS;
		mask = IRQ_FS;
		tbgen_update_reg(reg, val, mask);

		reg = &tbgregs->rfg_cr;
		val = ~RFGEN;
		mask = RFGEN;
		tbgen_update_reg(reg, val, mask);
	}

	spin_unlock_irqrestore(&tbg->lock, flags);

	return retcode;
out:
	return retcode;
}

int tbgen_rfg_init(struct tbgen_dev *tbg, struct tbg_rfg *rfg_params)
{
	int retcode = 0;
	u32 *reg, val, mask;
	struct tbg_regs *tbgregs = tbg->tbgregs;

	reg = &tbgregs->rfg_cr;
	val = rfg_params->drift_threshold << RFG_ERR_THRESHOLD_SHIFT;
	mask = RFG_ERR_THRESHOLD_MASK << RFG_ERR_THRESHOLD_SHIFT;
	tbgen_update_reg(reg, val, mask);

	reg = &tbgregs->rfg_cr;
	mask = 1 << SYNC_SEL_SHIFT;
	val =  SYNC_SEL_SYSREF;
	tbgen_update_reg(reg, val, mask);

	reg = &tbgregs->rfg_cr;
	mask = 1 << SYNC_OUT_SHIFT;
	val = SYNC_OUT_INTERNAL_10MS;
	tbgen_update_reg(reg, val, mask);

	reg = &tbgregs->rfg_cr;
	val = (rfg_params->sync_adv & SYNC_ADV_MASK) << SYNC_ADV_SHIFT;
	mask = SYNC_ADV_MASK << SYNC_ADV_SHIFT;
	tbgen_update_reg(reg, val, mask);

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
/*XXX: Fixed RFG reset */
#if 0
	rfg_enable(tbg, 0);
	retcode = tbgen_rfg_init(tbg);
	rfg_enable(tbg, 1);
#endif
	return retcode;
}

static u64 tbgen_get_master_counter(struct tbgen_dev *tbg)
{
	u64 master_counter = 0, tmp;

	master_counter = tbgen_read_reg(&tbg->tbgregs->mstr_cnt_lo);
	tmp = tbgen_read_reg(&tbg->tbgregs->mstr_cnt_hi);
	master_counter |= tmp << 32;

	return master_counter;
}


static u64 tbgen_get_l10_mcntr(struct tbgen_dev *tbg)
{
	u64 ts_10ms_cnt = 0, tmp;

	ts_10ms_cnt = tbgen_read_reg(&tbg->tbgregs->ts_10_ms_lo);
	tmp = tbgen_read_reg(&tbg->tbgregs->ts_10_ms_hi);
	ts_10ms_cnt |= tmp << 32;

	return ts_10ms_cnt;
}

static void tbgen_tasklet(unsigned long data)
{
	struct tbgen_dev *tbg = (struct tbgen_dev *)data;
	struct tbg_regs *tbgregs = tbg->tbgregs;
	u32 int_stat = 0, val;

	int_stat = tbgen_read_reg(&tbgregs->int_stat);
	/* Clear the IRQs which we serviced*/
	tbgen_write_reg(&tbgregs->int_stat, int_stat);

	if (int_stat & IRQ_FS) {
		if (tbgen_handle_rfg_irq(tbg)) {
			/* Disable FS IRQ */
			int_stat &= ~IRQ_FS;
		}
	}

	/* Enable IRQs which we serviced */
	val = tbgen_read_reg(&tbgregs->cntrl_1);
	val |= int_stat;
	tbgen_write_reg(&tbgregs->cntrl_1, val);
}

static irqreturn_t tbgen_isr(int irq, void *dev)
{
	struct tbgen_dev *tbg = (struct tbgen_dev *) dev;
	struct tbg_regs *tbgregs = tbg->tbgregs;
	u32 int_stat, val;

	int_stat = tbgen_read_reg(&tbgregs->int_stat);
	if (!int_stat) {
		dev_info(tbg->dev, "%s: Spurious IRQ\n", __func__);
		return IRQ_NONE;
	}
	spin_lock(&tbg->lock);
	val = tbgen_read_reg(&tbgregs->cntrl_1);
	val &= ~int_stat;
	tbgen_write_reg(&tbgregs->cntrl_1, val);
	spin_unlock(&tbg->lock);

	tasklet_hi_schedule(&tbg->tasklet);

	return IRQ_HANDLED;
}

/** @brief register irq
 *  @params 1 tbgen device instance
 *  @return returns success or fail depedning on the kernel api's retrun
 */
static int tbgen_register_irq(struct tbgen_dev *tbg)
{
	int retcode = 0;

	retcode = request_irq(tbg->irq, tbgen_isr, 0, "tbgen", tbg);
	if (retcode) {
		dev_err(tbg->dev, "Failed to register TBGEN IRQ %d\n, err %d",
			tbg->irq, retcode);
		goto out;
	}
	tasklet_init(&tbg->tasklet, tbgen_tasklet, (unsigned long)tbg);

out:
	return retcode;
}

static int tbgen_config_tx_timer_regs(struct tbgen_timer *timer,
	struct timer_param *timer_param,
	struct txalign_timer_regs *tx_timer_regs)
{
	int retcode = 0;
	u32 *reg, val = 0;

	dev_dbg(tbg->dev, "%s: config_flags %x, offset %llx\n", __func__,
		timer_param->config_flags, timer_param->offset);

	if (timer_param->config_flags & TIMER_CONF_OUTSEL_TX_IDLE_EN)
		val |= TXCTRL_OUTSEL_TXIDLE_EN;

	if (timer_param->config_flags & TIMER_CONF_SYNC_BYPASS)
		val |= TXCTRL_ISYNC_BYP;

	if (timer_param->config_flags & TIMER_CONF_SYNC_ERR_DETECT_EN)
		val |= TXCTRL_SREPEN;

	if (timer_param->config_flags & TIMER_CONF_ACTIVE_LOW)
		val |= TMRCTRL_POL_ACTIVE_LOW;

	spin_lock(&timer->lock);
	reg = &tx_timer_regs->ctrl;
	/*Disable and clean timer*/
	tbgen_write_reg(reg, 0);
	/*Init timer parameters, but not do not enable it here*/
	tbgen_write_reg(reg, val);

	memcpy(&timer->timer_param, timer_param, sizeof(struct timer_param));
	timer->status_flags = STATUS_FLG_CONFIGURED;
	spin_unlock(&timer->lock);

	return retcode;
}

static int tbgen_config_tx_timers(struct tbgen_dev *tbg,
			struct alignment_timer_params *align_timer_params)
{
	int timer_id, retcode = 0;
	struct tbgen_timer *timer;
	struct txalign_timer_regs *tx_timer_regs;

	timer_id = align_timer_params->id;
	switch (align_timer_params->type) {
	case JESD_TX_ALIGNMENT:
		if (timer_id < MAX_TX_ALIGNMENT_TIMERS) {
			timer = &tbg->tbg_txtmr[timer_id];
			tx_timer_regs = &tbg->tbgregs->tx_tmr[timer_id];
		} else {
			dev_err(tbg->dev, "Invalid tx alignment timer id %d\n",
				timer_id);
			retcode = -EINVAL;
			goto out;
		}
		break;
	case JESD_TX_AXRF:
		if (timer_id  < MAX_AXRF_ALIGNMENT_TIMERS) {
			timer = &tbg->tbg_tx_axrf[timer_id];
			tx_timer_regs = &tbg->tbgregs->axrf_tmr[timer_id];
		} else {
			dev_err(tbg->dev, "Invalid tx-axrf timer id %d\n",
				timer_id);
			retcode = -EINVAL;
			goto out;
		}
		break;
	default:
		retcode = -EINVAL;
		dev_err(tbg->dev, "tbgen_config_tx_timers: Tmr typ %d inval",
			align_timer_params->type);
		goto out;
		break;
	}

	retcode = tbgen_config_tx_timer_regs(timer, &align_timer_params->timer,
			tx_timer_regs);
	if (retcode) {
		dev_err(tbg->dev, "Timer [type %d, id %d] init Failed\n",
			align_timer_params->type, timer_id);
		goto out;
	}

	dev_dbg(tbg->dev, "Timer [type %d, id %d] initialized\n",
		align_timer_params->type, timer_id);
out:
	return retcode;
}

static int tbgen_config_generic_timer_regs(struct tbgen_timer *timer,
	struct timer_param *timer_param, struct gen_timer_regs *timer_regs)
{
	int retcode = 0;
	u32 *reg, val = 0, tmp;

	dev_dbg(tbg->dev, "%s: config_flags 0x%x, strobe %d, interval 0x%x\n",
			__func__, timer_param->config_flags,
			timer_param->strobe_mode, timer_param->interval);

	dev_dbg(tbg->dev, "%s: pulse_width 0x%x, offset 0x%llx\n",
			__func__, timer_param->pulse_width,
			timer_param->offset);

	switch (timer_param->strobe_mode) {
	case STROBE_TOGGLE:
		tmp = GEN_CTRL_STRB_TOGGLE;
		break;
	case STROBE_PULSE:
		tmp = GEN_CTRL_STRB_PULSE;
		break;
	case STROBE_CYCLE:
		tmp = GEN_CTRL_STRB_CYCLE;
		break;
	default:
		dev_err(tbg->dev, "genric timer conf: Invalid strobe %d\n",
			timer_param->strobe_mode);
		retcode = -EINVAL;
		goto out;
	}

	val &= ~(GEN_CTRL_STRB_MASK << GEN_CTRL_STRB_SHIFT);
	val |= (tmp << GEN_CTRL_STRB_SHIFT);

	if (timer_param->config_flags & TIMER_CONF_ONE_SHOT)
		val |= GEN_CTRL_ONESHOT;

	if (timer_param->config_flags & TIMER_CONF_ACTIVE_LOW)
		val |= TMRCTRL_POL_ACTIVE_LOW;

	if (timer_param->strobe_mode == STROBE_PULSE) {
		tmp = timer_param->pulse_width & GEN_CTRL_PULSE_WIDTH_MASK;
		val &= ~(GEN_CTRL_PULSE_WIDTH_MASK <<
				GEN_CTRL_PULSE_WIDTH_SHIFT);
		val |= (tmp << GEN_CTRL_PULSE_WIDTH_SHIFT);
	}
	reg = &timer_regs->ctrl;

	spin_lock(&timer->lock);
	tbgen_write_reg(reg, val);
	tbgen_write_reg(&timer_regs->interval, timer_param->interval);
	memcpy(&timer->timer_param, timer_param, sizeof(struct timer_param));
	timer->status_flags = STATUS_FLG_CONFIGURED;
	spin_unlock(&timer->lock);

out:
	return retcode;
}

static int tbgen_config_generic_timers(struct tbgen_dev *tbg,
			struct alignment_timer_params *align_timer_params)
{
	int retcode = 0, timer_id = align_timer_params->id;
	struct gen_timer_regs *timer_regs;
	struct tbgen_timer *timer;

	switch (align_timer_params->type) {
	case JESD_RX_ALIGNMENT:
		if (timer_id < MAX_RX_ALIGNMENT_TIMERS) {
			timer_regs = &tbg->tbgregs->rx_tmr[timer_id];
			timer = &tbg->tbg_rxtmr[timer_id];
		} else {
			retcode = -EINVAL;
			dev_err(tbg->dev, "Rx timer id invalid %d\n", timer_id);
			goto out;
		}
		break;
	case JESD_SRX_ALIGNMENT:
		if (timer_id < MAX_SRX_ALIGNMENT_TIMERS) {
			timer_regs = &tbg->tbgregs->srx_tmr[timer_id];
			timer = &tbg->tbg_srxtmr[timer_id];
		} else {
			retcode = -EINVAL;
			dev_err(tbg->dev, "Rx timer id invalid %d\n", timer_id);
			goto out;
		}
		break;
	default:
		dev_err(tbg->dev, "conf generic_timers Failed timer typ %d\n",
				align_timer_params->type);
		goto out;
	}
	retcode = tbgen_config_generic_timer_regs(timer,
			&align_timer_params->timer, timer_regs);
	if (retcode) {
		dev_err(tbg->dev, "Timer [type %d, id %d] init Failed\n",
			align_timer_params->type, timer_id);
		goto out;
	}

	dev_info(tbg->dev, "Timer [type %d, id %d] initialized\n",
		align_timer_params->type, timer_id);
out:
	return retcode;
}

struct tbgen_timer *tbgen_get_timer(struct tbgen_dev *tbg,
	enum timer_type type, int timer_id)
{
	struct tbgen_timer *timer = NULL;

	switch (type) {

	case JESD_TX_ALIGNMENT:
		if (timer_id > MAX_TX_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			goto out;
		}
		timer = &tbg->tbg_txtmr[timer_id];
		break;
	case JESD_TX_AXRF:
		if (timer_id > MAX_AXRF_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			goto out;
		}
		timer = &tbg->tbg_tx_axrf[timer_id];
		break;
	case JESD_RX_ALIGNMENT:
		if (timer_id > MAX_RX_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			goto out;
		}
		timer = &tbg->tbg_rxtmr[timer_id];
		break;
	case JESD_SRX_ALIGNMENT:
		if (timer_id > MAX_SRX_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			goto out;
		}
		timer = &tbg->tbg_srxtmr[timer_id];
		break;
	default:
		dev_err(tbg->dev, "%s: Invalid timer type %d\n", __func__,
			type);
		goto out;
	}

out:
	return timer;
}

int tbgen_attach_timer(struct tbgen_timer *timer, void *jesd_dev_handle)
{
	timer->jesd_dev_handle = jesd_dev_handle;
	SET_STATUS_FLAG(timer, STATUS_FLG_JESD_ATACHED);
	return 0;
}

int tbgen_timer_set_jesd_ready(struct tbgen_timer *timer, int ready)
{
	struct tbgen_dev *tbg = timer->tbg;

	if (!CHECK_STATUS_FLAG(timer, STATUS_FLG_CONFIGURED)) {
		dev_err(tbg->dev, "%s: Timer [typ %d, id %d] not configured\n",
			__func__, timer->type, timer->id);
		return -EINVAL;
	}
	if (ready)
		SET_STATUS_FLAG(timer, STATUS_FLG_JESD_READY);
	else
		CLEAR_STATUS_FLAG(timer, STATUS_FLG_JESD_READY);

	return 0;
}

int tbgen_set_sync_loopback(struct tbgen_timer *timer, int enabled)
{
	struct tbg_regs *tbgregs = timer->tbg->tbgregs;
	u32 *reg, val, mask;
	int rc = 0;

	if (timer->type != JESD_TX_ALIGNMENT) {
		dev_err(tbg->dev, "syn loopbak valid only for Tx alignment\n");
		rc = -EINVAL;
		goto out;
	}

	val = (1 << timer->id);
	mask = val;
	reg = &tbgregs->debug;

	if (!enabled)
		val = ~val;

	tbgen_update_reg(reg, val, mask);
out:
	return rc;
}

int tbgen_timer_enable(struct tbgen_timer *timer)
{
	return tbgen_timer_ctrl(timer->tbg, timer->type, timer->id, 1);
}

int tbgen_timer_disable(struct tbgen_timer *timer)
{

	return tbgen_timer_ctrl(timer->tbg, timer->type, timer->id, 0);
}

static int tbgen_timer_ctrl(struct tbgen_dev *tbg, enum timer_type type,
	int timer_id, int enable)
{
	int retcode = 0;
	u32 *ctrl_reg, *osethi_reg, *osetlo_reg, val, mask;
	u64 start_timer;
	struct tbgen_timer *timer;
	struct txalign_timer_regs *tx_timer_regs;
	struct gen_timer_regs *generic_timer_regs;

	if (timer_id < 0) {
		dev_err(tbg->dev, "%s: Invalid timer id %d\n", __func__,
			timer_id);
		retcode = -EINVAL;
		goto out;
	}
	switch (type) {

	case JESD_TX_ALIGNMENT:
		if (timer_id > MAX_TX_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			retcode = -EINVAL;
			goto out;
		}
		timer = &tbg->tbg_txtmr[timer_id];
		tx_timer_regs = &tbg->tbgregs->tx_tmr[timer_id];
		ctrl_reg = &tx_timer_regs->ctrl;
		osetlo_reg = &tx_timer_regs->osetlo;
		osethi_reg = &tx_timer_regs->osethi;
		break;
	case JESD_TX_AXRF:
		if (timer_id > MAX_AXRF_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			retcode = -EINVAL;
			goto out;
		}
		timer = &tbg->tbg_tx_axrf[timer_id];
		tx_timer_regs = &tbg->tbgregs->axrf_tmr[timer_id];
		ctrl_reg = &tx_timer_regs->ctrl;
		osetlo_reg = &tx_timer_regs->osetlo;
		osethi_reg = &tx_timer_regs->osethi;
		break;
	case JESD_RX_ALIGNMENT:
		if (timer_id > MAX_RX_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			retcode = -EINVAL;
			goto out;
		}
		timer = &tbg->tbg_rxtmr[timer_id];
		generic_timer_regs = &tbg->tbgregs->rx_tmr[timer_id];
		ctrl_reg = &generic_timer_regs->ctrl;
		osetlo_reg = &generic_timer_regs->osetlo;
		osethi_reg = &generic_timer_regs->osethi;
		break;
	case JESD_SRX_ALIGNMENT:
		if (timer_id > MAX_SRX_ALIGNMENT_TIMERS) {
			dev_err(tbg->dev, "%s: Invalid timer id %d\n",
				__func__, timer_id);
			retcode = -EINVAL;
			goto out;
		}
		timer = &tbg->tbg_srxtmr[timer_id];
		generic_timer_regs = &tbg->tbgregs->srx_tmr[timer_id];
		ctrl_reg = &generic_timer_regs->ctrl;
		osetlo_reg = &generic_timer_regs->osetlo;
		osethi_reg = &generic_timer_regs->osethi;
		break;
	default:
		dev_err(tbg->dev, "%s: Invalid timer type %d\n", __func__,
			type);
		retcode = -EINVAL;
		goto out;
	}

	dev_dbg(tbg->dev, "%s: Timer id %d Enable %d, offset %llx\n",
		__func__, timer_id, enable, timer->timer_param.offset);

	spin_lock(&timer->lock);
	if (enable) {
		if (!(timer->status_flags & STATUS_FLG_CONFIGURED)) {
			dev_err(tbg->dev, "%s:[%d:%d] Failed, uninit timer\n",
				__func__, type, timer_id);
			spin_unlock(&timer->lock);
			retcode = -EINVAL;
			goto out;
		}
		/*Program Timer fire time */
		start_timer = tbg->last_10ms_counter;
		start_timer += timer->timer_param.offset;
		val = start_timer & 0xffffffff;
		tbgen_write_reg(osetlo_reg, val);
		val = (start_timer >> 32) & 0xffffffff;
		tbgen_write_reg(osethi_reg, val);

		/*Enable Timer*/
		val = TMRCTRL_EN;
		mask = TMRCTRL_EN;
		tbgen_update_reg(ctrl_reg, val, mask);
		timer->status_flags |= STATUS_FLG_ENABLED;
	} else {
		val = ~TMRCTRL_EN;
		mask = TMRCTRL_EN;
		tbgen_update_reg(ctrl_reg, val, mask);
		timer->status_flags &= ~STATUS_FLG_ENABLED;
	}
	spin_unlock(&timer->lock);
out:
	return retcode;
}
EXPORT_SYMBOL(tbgen_timer_ctrl);

void write_multiple_regs(struct tbgen_dev *tbg,
			u32 offset,
			u32 len,
			u32 value)
{
	write_reg(&tbg->tbgregs->rfg_cr, offset, len, &value);
}

int config_alignment_timers(struct tbgen_dev *tbg,
				struct alignment_timer_params *timer_params)
{
	int retcode = 0;

	if (timer_params->id < 0) {
		dev_err(tbg->dev, "Invalid timer id %d\n", timer_params->id);
		retcode = -EINVAL;
		goto out;
	}

	dev_dbg(tbg->dev, "%s: Init Timer type %d, id %d\n", __func__,
		timer_params->type, timer_params->id);

	switch (timer_params->type) {
	case JESD_TX_AXRF:
	case JESD_TX_ALIGNMENT:
		retcode = tbgen_config_tx_timers(tbg, timer_params);
		break;
	case JESD_RX_ALIGNMENT:
	case JESD_SRX_ALIGNMENT:
		retcode = tbgen_config_generic_timers(tbg, timer_params);
		break;
	default:
		break;
	}

out:
	return retcode;
}

static int tbgen_change_state(struct tbgen_dev *tbg,
	enum tbgen_dev_state new_state)
{
	enum tbgen_dev_state old_state = tbg->state;
	int retcode = -EINVAL;

	switch (old_state) {
	case TBG_STATE_STANDBY:
		/* Change state only if both PLL and RFG are configured*/
		if (new_state == TBG_STATE_CONFIGURED)
			if (TBG_CHCK_CONFIG_MASK(tbg, TBG_CONFIGURED_MASK))
				retcode = 0;
		break;
	case TBG_STATE_CONFIGURED:
		/* Transition to Ready if PLLS are locked, and RFG ready*/
		if (new_state == TBG_STATE_READY)
			if (TBG_CHCK_CONFIG_MASK(tbg, TBG_READY_MASK))
				retcode = 0;
		/* Failure transitions allowed */
		if ((new_state == TBG_STATE_PLL_FAILED) ||
				(new_state == TBG_STATE_RFG_FAILED))
			retcode = 0;
		break;
	case TBG_STATE_PLL_FAILED:
	case TBG_STATE_RFG_FAILED:
		if ((new_state == TBG_STATE_STANDBY))
			retcode = 0;
	default:
		dev_dbg(tbg->dev, "invalid old state %d\n", tbg->state);
		break;
	}

	if (!retcode) {
		dev_dbg(tbg->dev, "State transition %d -> %d done\n", old_state,
			new_state);
		tbg->state = new_state;
	} else {
		dev_dbg(tbg->dev, "State transition %d -> %d Invalid\n",
			old_state, new_state);
	}

	return retcode;
}

long tbgen_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	int retcode = -ENOSYS, count = 0, size;
	u32 *regs;
	u64 master_counter = 0, l10_counter = 0;
	struct tbgen_dev *tbg;
	struct alignment_timer_params alignment_timer_params;
	struct tbgen_reg_read_buf regcnf;
	struct tbgen_device_info dev_info;
	struct timer_ctrl tmr_ctrl;
	struct tbgen_reg_write_buf *wreg_buf;
	struct tbgen_reg_write_buf write_reg;
	struct tbg_pll pll_params;
	struct tbg_rfg rfg_params;
	void __user *argp = (void __user *)arg;

	tbg = pfile->private_data;
	switch (cmd) {
	case TBGEN_SET_PLL:
		if (copy_from_user(&pll_params, (struct tbg_pll *)arg,
				sizeof(struct tbg_pll))) {
			retcode = -EFAULT;
			break;
		}
		retcode = tbgen_pll_init(tbg, &pll_params);
		if (retcode) {
			dev_err(tbg->dev, "PLL configuration failed\n");
			break;
		}
		TBG_SET_CONFIG_MASK(tbg, TBG_PLL_CONFIGURED);
		tbgen_change_state(tbg, TBG_STATE_CONFIGURED);
		break;

	case TBGEN_RFG_INIT:
		tbg->mon_rfg_isr = 0;

		if (copy_from_user(&rfg_params, (struct tbg_rfg *)arg,
				sizeof(struct tbg_rfg))) {
			retcode = -EFAULT;
			break;
		}
		retcode = tbgen_rfg_init(tbg, &rfg_params);
		if (retcode) {
			dev_err(tbg->dev, "RFG configuration failed\n");
			break;
		}
		TBG_SET_CONFIG_MASK(tbg, TBG_RFG_CONFIGURED);
		tbgen_change_state(tbg, TBG_STATE_CONFIGURED);
		break;
	case TBGEN_RFG_RESET:
		tbg->mon_rfg_isr = 0;

		if (copy_from_user(&rfg_params, (struct tbg_rfg *)arg,
				sizeof(struct tbg_rfg))) {
			retcode = -EFAULT;
			break;
		}

		retcode = tbgen_restart_radio_frame_generation(tbg);
	case TBGEN_RFG_ENABLE:
		retcode = rfg_enable(tbg, 1);
		if (!retcode) {
			TBG_SET_CONFIG_MASK(tbg, TBG_RFG_READY);
			tbgen_change_state(tbg, TBG_STATE_READY);
		}
		break;
	case TBGEN_CONFIG_TIMER:

		if (copy_from_user(&alignment_timer_params,
				(struct alignment_timer_params *)arg,
				sizeof(struct alignment_timer_params))) {
			retcode = -EFAULT;
			break;
		}
		retcode = config_alignment_timers(tbg, &alignment_timer_params);
		break;

	case TBGEN_TIMER_CTRL:
		if (copy_from_user(&tmr_ctrl,
			(struct irq_conf *)arg,
			sizeof(struct timer_ctrl))) {
			retcode = -EFAULT;
			break;
		}
		retcode = tbgen_timer_ctrl(tbg, tmr_ctrl.type, tmr_ctrl.id,
				tmr_ctrl.enable);
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

		if (copy_from_user(&regcnf, (struct reg_conf *)arg,
				sizeof(struct tbgen_reg_read_buf))) {
				retcode = -EFAULT;
				break;
		}

		regs = (u32 *) tbg->tbgregs;
		size = sizeof(struct tbg_regs);
		if ((regcnf.len * 4 + regcnf.len) > size) {
			dev_err(tbg->dev, "invalid len/offset:%d/0x%x",
					regcnf.len, regcnf.offset);
			retcode = -EINVAL;
			break;
		}
		retcode = jesd_reg_dump_to_user(regs, regcnf.offset,
			regcnf.len, regcnf.buf);
		break;
	case TBGEN_GET_DEV_INFO:

		dev_info.state = tbg->state;

		if (copy_to_user((struct tbgen_device_info *)arg, &dev_info,
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
		l10_counter = tbgen_get_l10_mcntr(tbg);

		if (copy_to_user((u64 *)arg, &l10_counter,
						sizeof(l10_counter)))
			retcode = -EFAULT;
		break;
	case TBGEN_RESET:
		/*XXX: Implement TBGEN reset:
		 * 1. Disable all timers
		 * 2. Rset RFG
		 * 3. Reset TBGEN
		 * 4. Reprogram all timers again from strored timer_param
		 */
		break;
	default:
		retcode = -ENOTTY;
		break;
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
	pfile->private_data = tbg;
	atomic_inc(&tbg->ref);

	return 0;
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
	atomic_dec(&tbg->ref);
	return 0;
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
	int retcode = 0, id;

	tbg = kzalloc(sizeof(struct tbgen_dev), GFP_KERNEL);

	if (!tbg) {
		retcode = -ENOMEM;
		goto out;
	}

	tbg->node = pdev->dev.of_node;
	tbg->dev = &pdev->dev;
	atomic_set(&tbg->ref, 0);
	tbg->tbgregs = of_iomap(tbg->node, 0);
	if (!tbg->tbgregs)
		goto out;

	tbg->irq = irq_of_parse_and_map(tbg->node, 0);
	if (tbg->irq) {
		retcode = tbgen_register_irq(tbg);
		if (retcode) {
			dev_err(tbg->dev, "Failed to register IRQs, err %d\n",
				retcode);
			goto out;
		}
	} else {
		tbg->dev_flags |= FLG_NO_INTERRUPTS;
	}

	retcode = misc_register(&tbg_miscdev);
	if (retcode < 0) {
		dev_info(&pdev->dev, "error %d misc register\n", retcode);
		goto out;
	}

	spin_lock_init(&tbg->lock);

	for (id = 0; id < MAX_TX_ALIGNMENT_TIMERS; id++) {
		spin_lock_init(&tbg->tbg_txtmr[id].lock);
		tbg->tbg_txtmr[id].tbg = tbg;
	}

	for (id = 0; id < MAX_AXRF_ALIGNMENT_TIMERS; id++) {
		spin_lock_init(&tbg->tbg_tx_axrf[id].lock);
		tbg->tbg_tx_axrf[id].tbg = tbg;
	}

	for (id = 0; id < MAX_RX_ALIGNMENT_TIMERS; id++) {
		spin_lock_init(&tbg->tbg_rxtmr[id].lock);
		tbg->tbg_rxtmr[id].tbg = tbg;
	}

	for (id = 0; id < MAX_SRX_ALIGNMENT_TIMERS; id++) {
		spin_lock_init(&tbg->tbg_srxtmr[id].lock);
		tbg->tbg_srxtmr[id].tbg = tbg;
	}

	dev_set_drvdata(tbg->dev, tbg);
	tbg->state = TBG_STATE_STANDBY;
	tbg->sync_state = SYNC_INVALID;
	/*Disable all interrupts*/
	tbgen_write_reg(&tbg->tbgregs->cntrl_1, 0);

	return retcode;

out:
	if (tbg->tbgregs)
		iounmap(tbg->tbgregs);
	kfree(tbg);
	return retcode;
}

/** @brief remove function call for tbgen
 *
 *  \params platform device instance given from platform base
 *  \return zero on success or negative number on failure.
 */
static int __exit tbgen_of_remove(struct platform_device *pdev)
{
	int retcode = 0, id;
	struct tbgen_dev *tbg = dev_get_drvdata(&pdev->dev);

	for (id = 0; id < MAX_TX_ALIGNMENT_TIMERS; id++)
		tbgen_timer_ctrl(tbg, JESD_TX_ALIGNMENT, id, 0);

	for (id = 0; id < MAX_AXRF_ALIGNMENT_TIMERS; id++)
		tbgen_timer_ctrl(tbg, JESD_TX_AXRF, id, 0);

	for (id = 0; id < MAX_RX_ALIGNMENT_TIMERS; id++)
		tbgen_timer_ctrl(tbg, JESD_RX_ALIGNMENT, id, 0);

	for (id = 0; id < MAX_SRX_ALIGNMENT_TIMERS; id++)
		tbgen_timer_ctrl(tbg, JESD_SRX_ALIGNMENT, id, 0);

	/*XXX: Disable IRQ and RFG */

	misc_deregister(&tbg_miscdev);

	kfree(tbg);

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

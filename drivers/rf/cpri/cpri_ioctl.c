/*
 * drivers/rf/cpri/cpri_ioctl.c
 * CPRI device driver - ioctl interface
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/pid.h>

#include "cpri.h"

static void set_delay_config(struct cpri_framer *framer)
{
	struct cpri_delay_config *cfg = &framer->delay_cfg;
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

static void cpri_reg_write_bulk(raw_spinlock_t *lock, u32 *base,
		u32 offset, unsigned int length, u32 *buf)
{
	int i;
	u32 mask = MASK_ALL;

	for (i = 0; i < offset; i++)
		base++;

	for (i = 0; i < length; i++) {
		cpri_reg_set_val(lock, base, mask, buf[i]);
		base++;
	}
}

static void cpri_reg_read_bulk(raw_spinlock_t *lock, u32 *base,
		u32 offset, unsigned int length, u32 *buf)
{
	int i;
	u32 mask = MASK_ALL;

	for (i = 0; i < offset; i++)
		base++;

	for (i = 0; i < length; i++) {
		buf[i] = cpri_reg_get_val(lock, base, mask);
		base++;
	}
}

static void cpri_reset_bfn(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	cpri_reg_set(&framer->regs_lock,
			&regs->cpri_tx_control,
			TX_RESET_BFN_MASK);
}

static void cpri_fill_framer_stats(struct cpri_framer *framer)
{
	struct cpri_framer_regs  __iomem  *regs = framer->regs;
	struct cpri_framer_stats *stats = (&framer->stats);

	/* Timing events, error events, vss events are updated
	 * by the irq handlers.Autoneg errors are updtaed in the
	 * autoneg state machine. Rest of the stats are updated here
	 */
	stats->rx_line_coding_violation = cpri_reg_get_val(&framer->regs_lock,
							&regs->cpri_lcv,
							CNT_LCV_MASK);
}

static void cpri_fill_framer_info(struct framer_info *info,
			struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	/* Current test mode of framer */
	info->test_flags = framer->test_flags;

	/* Current recovered BFN and HFN */
	info->cur_bfn = (u16) cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_bfn,
					RECOVERED_BFN_CNT_MASK);
	info->cur_hfn = (u8) cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_hfn,
					RECOVERED_HFN_CNT_MASK);

	/* Current framer hw status */
	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_LOS_MASK))
		info->cpri_status_flags |= RX_LOS_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_STATE_MASK))
		info->cpri_status_flags |= RX_STATE_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_HFN_STATE_MASK))
		info->cpri_status_flags |= RX_HFN_STATE_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_BFN_STATE_MASK))
		info->cpri_status_flags |= RX_BFN_STATE_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_LOS_HOLD_MASK))
		info->cpri_status_flags |= RX_LOS_HOLD_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_STATE_HOLD_MASK))
		info->cpri_status_flags |= RX_STATE_HOLD_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_FREQ_ALARM_HOLD_MASK))
		info->cpri_status_flags |= RX_FREQ_ALARM_HOLD_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_status,
				RX_RFP_HOLD_MASK))
		info->cpri_status_flags |= RX_RFP_HOLD_STATUS;

	/* Current hw reset status */
	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_hwreset,
				RESET_GEN_DONE_HOLD_MASK))
		info->hw_reset_status_flags |= RESET_GEN_DONE_HOLD_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_hwreset,
				RESET_GEN_DONE_MASK))
		info->hw_reset_status_flags |= RESET_GEN_DONE_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_hwreset,
				RESET_DETECT_HOLD_MASK))
		info->hw_reset_status_flags |= RESET_DETECT_HOLD_STATUS;

	if (cpri_reg_get_val(&framer->regs_lock,
				&regs->cpri_hwreset,
				RESET_DETECT_MASK))
		info->hw_reset_status_flags |= RESET_DETECT_STATUS;

	/* Current hfn and bfn counters */
	info->tx_hfn_counter = cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_hwreset,
					TX_HFN_COUNTER_MASK);
	info->tx_bfn_counter = cpri_reg_get_val(&framer->regs_lock,
					&regs->cpri_hwreset,
					TX_BFN_COUNTER_MASK);

	/* framer state */
	info->state = framer->framer_state;

	/* current framer param settings */
	memcpy((struct cpri_framer_param *)&info->framer_param,
		(struct cpri_framer_param *)&framer->framer_param,
		sizeof(struct cpri_framer_param));

	/* current axc count */
	info->max_axc_count = framer->framer_param.max_axc_count;
}

static void cpri_init_framer(struct cpri_framer *framer)
{
	struct cpri_framer_param *param = &framer->framer_param;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_common_regs __iomem *cregs = framer->cpri_dev->regs;

	/* CPRI config params */
	if (param->framer_pram_flags & SET_10_ACKS)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_config,
				CONF_SET_10_ACKS_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_config,
				CONF_SET_10_ACKS_MASK);

	if (param->framer_pram_flags & CNT_6_RESET)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_config,
				CONF_CNT_6_RESET_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_config,
				CONF_CNT_6_RESET_MASK);

	if (param->framer_pram_flags & SYNC_PULSE_MODE)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_config,
				CONF_SYNC_PULSE_MODE_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_config,
				CONF_SYNC_PULSE_MODE_MASK);

	if (param->framer_pram_flags & TX_CW_INSERT)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_config,
				TX_CW_INSERT_EN_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_config,
				TX_CW_INSERT_EN_MASK);

	/* Control word params */
	if (param->framer_pram_flags & CW)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_auxcwdmasken,
				CW_EN_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_auxcwdmasken,
				CW_EN_MASK);

	if (param->framer_pram_flags & CW130)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_auxcwdmasken,
				CW130_EN_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_auxcwdmasken,
				CW130_EN_MASK);

#if 0
	/* TBD: SRC config param */
	if (param->framer_pram_flags & RST_REQ_BYP)
		src_reset_request_bypass(framer->src_bypass_status,
				framer->src_bypass_dur_sec);
#endif

	/* Remote reset params */
	if (param->framer_pram_flags & C1_REM_RES_OP)
		cpri_reg_set(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C1_REM_RES_OP_EN_MASK);
	else
		cpri_reg_clear(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C1_REM_RES_OP_EN_MASK);

	if (param->framer_pram_flags & C1_REM_RES_ACK_OP)
		cpri_reg_set(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C1_REM_RES_ACK_OP_EN_MASK);
	else
		cpri_reg_clear(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C1_REM_RES_ACK_OP_EN_MASK);

	if (param->framer_pram_flags & C2_REM_RES_OP)
		cpri_reg_set(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C2_REM_RES_OP_EN_MASK);
	else
		cpri_reg_clear(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C2_REM_RES_OP_EN_MASK);

	if (param->framer_pram_flags & C2_REM_RES_ACK_OP)
		cpri_reg_set(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C2_REM_RES_ACK_OP_EN_MASK);
	else
		cpri_reg_clear(&framer->cpri_dev->lock,
				&cregs->cpri_remresetoutputctrl,
				C2_REM_RES_ACK_OP_EN_MASK);

	/* Rx control param */
	if (param->framer_pram_flags & RX_IQ_SYNC)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_rctrl,
				IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_rctrl,
				IQ_SYNC_EN_MASK);

	/* Tx control param */
	if (param->framer_pram_flags & TX_IQ_SYNC)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_rctrl,
				IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_rctrl,
				IQ_SYNC_EN_MASK);

	/* General Rx sync param */
	if (param->framer_pram_flags & GEN_RX_IQ_SYNC)
		cpri_reg_set(&framer->cpri_dev->lock,
				&cregs->cpri_rgensync,
				IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(&framer->cpri_dev->lock,
				&cregs->cpri_rgensync,
				IQ_SYNC_EN_MASK);

	/* General Tx sync param */
	if (param->framer_pram_flags & GEN_TX_IQ_SYNC)
		cpri_reg_set(&framer->cpri_dev->lock,
				&cregs->cpri_tgensync,
				IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(&framer->cpri_dev->lock,
				&cregs->cpri_tgensync,
				IQ_SYNC_EN_MASK);

	/* ECC error indication config param */
	if (param->framer_pram_flags & SINGLE_BIT_ECC_ERROR_OUTPUT)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_eccerrindicateen,
				SINGLE_BIT_ECC_ERROR_OUTPUT_EN_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_eccerrindicateen,
				SINGLE_BIT_ECC_ERROR_OUTPUT_EN_MASK);

	if (param->framer_pram_flags & MULTI_BIT_ECC_ERROR_OUTPUT)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_eccerrindicateen,
				MULTI_BIT_ECC_ERROR_OUTPUT_EN_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_eccerrindicateen,
				MULTI_BIT_ECC_ERROR_OUTPUT_EN_MASK);

	if (param->framer_pram_flags & AUX_MODE)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_auxctrl,
				AUX_MODE_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				 &regs->cpri_auxctrl,
				AUX_MODE_MASK);

	if (param->framer_pram_flags & SLAVE_MODE)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_config,
				SLAVE_MODE_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_config,
				SLAVE_MODE_MASK);

	if (param->framer_pram_flags & ADVANCED_MAP_MODE)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_mapcfg,
				MAP_MODE_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_mapcfg,
				MAP_MODE_MASK);

	/* Tx framer size setting */
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_tbufsize,
			FR_BUF_SIZE_MASK,
			param->tx_framer_buf_size);

	/* TODO: max axc count, K0 and K1 setting */

	/* VSS AXI transaction size setting */
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_rvssaxisize,
			AXI_TRANSAC_SIZE_MASK,
			param->rx_vss_axi_transac_size);
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_tvssaxisize,
			AXI_TRANSAC_SIZE_MASK,
			param->rx_vss_axi_transac_size);

	/* Map offset setting */
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_rmapoffset,
			MAP_OFFSET_X_MASK,
			param->map_sync_offset.rx_bfn_offset);
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_rmapoffset,
			MAP_OFFSET_Z_MASK,
			param->map_sync_offset.rx_hfn_offset);
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_tmapoffset,
			MAP_OFFSET_X_MASK,
			param->map_sync_offset.tx_bfn_offset);
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_tmapoffset,
			MAP_OFFSET_Z_MASK,
			param->map_sync_offset.tx_hfn_offset);

	/* Tx Offset setting */
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_tstartoffset,
			MAP_OFFSET_X_MASK,
			param->map_sync_offset.start_tx_bfn_offset);
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_tstartoffset,
			MAP_OFFSET_Z_MASK,
			param->map_sync_offset.start_tx_hfn_offset);
}

static void cpri_set_test_mode(unsigned int mode, struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	if (mode & RX_TRANSPARENT_MODE)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_mapcfg,
				RX_TRANSPARENT_MODE_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_mapcfg,
				RX_TRANSPARENT_MODE_MASK);

	if (mode & TX_TRANSPARENT_MODE)
		cpri_reg_set(&framer->regs_lock,
				&regs->cpri_mapcfg,
				TX_TRANSPARENT_MODE_MASK);
	else
		cpri_reg_clear(&framer->regs_lock,
				&regs->cpri_mapcfg,
				TX_TRANSPARENT_MODE_MASK);
}

long cpri_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct regrw reg;
	struct cpri_framer *framer = fp->private_data;
	struct device *dev = framer->cpri_dev->dev;
	struct framer_info info;
	struct cpri_delay_config *delay_cfg = &framer->delay_cfg;
	struct cpri_delay_output *delay_out = &framer->delay_out;
	int err = 0;
	void __user *ioargp = (void __user *)arg;

	if (_IOC_TYPE(cmd) != CPRI_MAGIC) {
		dev_err(dev, "invalid case, CMD=%d\n", cmd);
		return -EINVAL;
	}

	switch (cmd) {
	case CPRI_SET_STATE:
		if (copy_from_user(&framer->framer_state,
				(enum cpri_state *)ioargp,
				sizeof(enum cpri_state)) != 0) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CPRI_GET_STATE:
		if (copy_to_user((enum cpri_state *)ioargp,
				&(framer->framer_state),
				sizeof(enum cpri_state))) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CPRI_SET_TEST_MODE:
		if (get_user(framer->test_flags,
				(unsigned int *)ioargp) != 0) {
			err = -EFAULT;
			goto out;
		}
		cpri_set_test_mode(framer->test_flags, framer);
		break;
	case CPRI_GET_TEST_MODE:
		if (put_user(framer->test_flags, (unsigned int *)ioargp) != 0) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CPRI_INIT_FRAMER:
		if (copy_from_user(&(framer->framer_param),
				(struct cpri_framer_param *)ioargp,
				sizeof(struct cpri_framer_param)) != 0) {
			err = -EFAULT;
			goto out;
		}
		cpri_init_framer(framer);
		break;
	case CPRI_GET_FRAMER_INFO:
		cpri_fill_framer_info(&info, framer);
		if (copy_to_user((struct framer_info *)ioargp, &info,
				sizeof(struct framer_info))) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CPRI_GET_STATS:
		cpri_fill_framer_stats(framer);
		if (copy_to_user((struct cpri_framer_stats *)ioargp,
				&framer->stats,
				sizeof(struct cpri_framer_stats))) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CPRI_BFN_RESET:
		cpri_reset_bfn(framer);
		break;
	case CPRI_REG_WRITE:
		if (copy_from_user(&reg,
				(struct regrw *)ioargp,
				sizeof(struct regrw)) != 0) {
			err = -EFAULT;
			goto out;
		}

		cpri_reg_write_bulk(&framer->regs_lock, (u32 *)framer->regs,
				reg.offset, reg.length, (u32 *)&reg.buf);

		break;
	case CPRI_REG_READ:
		if (copy_from_user(&reg,
				(struct regrw *)ioargp,
				sizeof(struct regrw)) != 0) {
			err = -EFAULT;
			goto out;
		}

		cpri_reg_read_bulk(&framer->regs_lock, (u32 *)framer->regs,
				reg.offset, reg.length, (u32 *)&reg.buf);

		if (copy_to_user(ioargp, (struct regrw *)&reg,
					sizeof(struct regrw))) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CPRI_EVENT_NOTIFICATION:
		if (get_user(framer->notification_state,
			(unsigned char *)ioargp) != 0) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CPRI_CFG_DELAY_RAW:
		if (copy_from_user(delay_cfg,
			(struct cpri_delay_config *)ioargp,
			sizeof(struct cpri_delay_config)) != 0) {
			err = -EFAULT;
			goto out;
		}
		set_delay_config(framer);
		break;
	case CPRI_GET_DELAY_RAW:
		if (copy_to_user(ioargp,
			(struct cpri_delay_output *)delay_out,
			sizeof(struct cpri_delay_output))) {
			err = -EFAULT;
			goto out;
		}
		break;
	default:
		err = cpri_autoneg_ioctl(framer, cmd, arg);
		if (err < 0)
			goto out;
		break;
	}

	return 0;

out:
	dev_err(dev, "IOCTL error\n");
	return err;
}

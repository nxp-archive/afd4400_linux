/*
 * drivers/net/cpri/cpri_ioctl.c
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

#include <linux/cpri.h>
#include <linux/cpri_axc.h>

static void cpri_reg_write_bulk(void *base,
		u32 offset, unsigned int length, u32 value)
{
	int i;
	u32 mask = MASK_ALL;

	base = (char *)base + offset;

	for (i = 0; i < length; i++) {
		cpri_reg_set_val(base, mask, value);
		base += (sizeof(u32));
	}
}

static void cpri_reg_read_bulk(void *base,
		u32 offset, unsigned int length, u32 *buf)
{
	int i;
	u32 mask = MASK_ALL;

	base = (char *)base + offset;

	for (i = 0; i < length; i++) {
		buf[i] = cpri_reg_get_val(base, mask);
		base += (sizeof(u32));
	}
}

static void cpri_reset_bfn(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	cpri_reg_set(&regs->cpri_tx_control,
			TX_RESET_BFN_MASK);
}

static void cpri_fill_framer_stats(struct cpri_framer *framer)
{
	struct cpri_framer_regs  __iomem  *regs = framer->regs;
	struct cpri_dev_stats *stats = (&framer->stats);

	/* Timing events, error events, vss events are updated
	 * by the irq handlers.Autoneg errors are updtaed in the
	 * autoneg state machine. Rest of the stats are updated here
	 */
	stats->rx_line_coding_violation = cpri_reg_get_val(
						&regs->cpri_lcv, CNT_LCV_MASK);
}

static void fill_sfp_info(struct cpri_framer *framer,
			struct cpri_dev_info *dev_info)
{
	int status;
	struct sfp *info = &(framer->sfp_dev->info);

	status = sfp_raw_read(framer->sfp_dev, (u8 *)&info->type,
			0, sizeof(struct sfp), SFP_MEM_EEPROM);
	if (status < 0) {
		dev_err(framer->cpri_dev->dev, "SFP read error");
		return;
	}

	dev_info->sfp_info.type = info->type;
	dev_info->sfp_info.ext_type = info->ext_type;
	dev_info->sfp_info.connector_type = info->connector_type;

	memcpy(dev_info->sfp_info.compatibility_code,
		info->compatibility_code, sizeof(info->compatibility_code));

	dev_info->sfp_info.encoding = info->encoding;
	dev_info->sfp_info.bitrate = info->bitrate;
	dev_info->sfp_info.link_len_9u_km = info->link_len_9u_km;
	dev_info->sfp_info.link_len_9u_100m = info->link_len_9u_100m;
	dev_info->sfp_info.link_len_50u_10m = info->link_len_50u_10m;
	dev_info->sfp_info.link_len_62_5u_10m = info->link_len_62_5u_10m;
	dev_info->sfp_info.link_len_cu_m = info->link_len_cu_m;

	memcpy(dev_info->sfp_info.vendor_name,
		info->vendor_name, sizeof(info->vendor_name));

	memcpy(dev_info->sfp_info.vendor_oui,
		info->vendor_oui, sizeof(info->vendor_oui));

	memcpy(dev_info->sfp_info.vendor_pn,
		info->vendor_pn, sizeof(info->vendor_pn));

	memcpy(dev_info->sfp_info.vendor_rev,
		info->vendor_rev, sizeof(info->vendor_rev));

	memcpy(dev_info->sfp_info.wavelength,
		info->wavelength, sizeof(info->wavelength));

	dev_info->sfp_info.check_code_b = info->check_code_b;

	memcpy(dev_info->sfp_info.options,
		info->options, sizeof(info->options));

	dev_info->sfp_info.bitrate_max = info->bitrate_max;
	dev_info->sfp_info.bitrate_min = info->bitrate_min;

	memcpy(dev_info->sfp_info.vendor_sn,
		info->vendor_sn, sizeof(info->vendor_sn));

	memcpy(dev_info->sfp_info.manf_date,
		info->manf_date, sizeof(info->manf_date));

	dev_info->sfp_info.diag_type = info->diag_type;
	dev_info->sfp_info.enhanced_options = info->enhanced_options;
	dev_info->sfp_info.sfp_compliance = info->sfp_compliance;
	dev_info->sfp_info.check_code_e = info->check_code_e;
}

static void cpri_fill_framer_info(struct cpri_dev_info *info,
			struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	/* Current framer state */
	info->state = framer->framer_state;

	/* Current test mode of framer */
	info->test_flags = framer->test_flags;

	/* Current device operation modes */
	if ((framer->test_flags | UL_TRANSPARENT) ||
		(framer->test_flags | UL_TRANSPARENT))
		info->dev_flags |= CPRI_TEST_MODE;

	info->dev_flags |= framer->dev_flags;

	/* Current framer hw status */
	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_LOS_MASK))
		info->hw_status |= RX_LOS_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_STATE_MASK))
		info->hw_status |= RX_STATE_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_HFN_STATE_MASK))
		info->hw_status |= RX_HFN_STATE_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_BFN_STATE_MASK))
		info->hw_status |= RX_BFN_STATE_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_LOS_HOLD_MASK))
		info->hw_status |= RX_LOS_HOLD_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_STATE_HOLD_MASK))
		info->hw_status |= RX_STATE_HOLD_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_FREQ_ALARM_HOLD_MASK))
		info->hw_status |= RX_FREQ_ALARM_HOLD_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_RFP_HOLD_MASK))
		info->hw_status |= RX_RFP_HOLD_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_status,
				RX_RFP_HOLD_MASK))
		info->hw_status |= RX_RFP_HOLD_STATUS;


	if (cpri_reg_get_val(
				&regs->cpri_hwreset,
				RESET_GEN_DONE_HOLD_MASK))
		info->hw_status |= RESET_GEN_DONE_HOLD_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_hwreset,
				RESET_GEN_DONE_MASK))
		info->hw_status |= RESET_GEN_DONE_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_hwreset,
				RESET_DETECT_HOLD_MASK))
		info->hw_status |= RESET_DETECT_HOLD_STATUS;

	if (cpri_reg_get_val(
				&regs->cpri_hwreset,
				RESET_DETECT_MASK))
		info->hw_status |= RESET_DETECT_STATUS;

	/* Current recovered BFN and HFN */
	info->current_bfn = (u16) cpri_reg_get_val(
					&regs->cpri_bfn,
					RECOVERED_BFN_CNT_MASK);

	info->current_hfn = (u8) cpri_reg_get_val(
					&regs->cpri_hfn,
					RECOVERED_HFN_CNT_MASK);

	/* SFP info */
	if (framer->sfp_dev != NULL)
		fill_sfp_info(framer, info);

	/* Current framer param settings */
	memcpy((struct cpri_dev_init_params *)&info->init_params,
		(struct cpri_dev_init_params *)&framer->framer_param,
		sizeof(struct cpri_dev_init_params));
}

static void cpri_init_param(struct cpri_framer *framer)
{
	struct cpri_dev_init_params *param = &framer->framer_param;
	struct cpri_common_regs __iomem *cregs = framer->cpri_dev->regs;


	/* Remote reset params */
	if (param->ctrl_flags & CPRI_C1_REM_RES_OP)
		cpri_reg_set(&cregs->cpri_remresetoutputctrl,
				C1_REM_RES_OP_EN_MASK);
	else
		cpri_reg_clear(&cregs->cpri_remresetoutputctrl,
				C1_REM_RES_OP_EN_MASK);

	if (param->ctrl_flags & CPRI_C1_REM_RES_ACK_OP)
		cpri_reg_set(&cregs->cpri_remresetoutputctrl,
			C1_REM_RES_ACK_OP_EN_MASK);
	else
		cpri_reg_clear(&cregs->cpri_remresetoutputctrl,
			C1_REM_RES_ACK_OP_EN_MASK);

	if (param->ctrl_flags & CPRI_C2_REM_RES_OP)
		cpri_reg_set(&cregs->cpri_remresetoutputctrl,
			C2_REM_RES_OP_EN_MASK);
	else
		cpri_reg_clear(&cregs->cpri_remresetoutputctrl,
				C2_REM_RES_OP_EN_MASK);

	if (param->ctrl_flags & CPRI_C2_REM_RES_ACK_OP)
		cpri_reg_set(&cregs->cpri_remresetoutputctrl,
				C2_REM_RES_ACK_OP_EN_MASK);
	else
		cpri_reg_clear(&cregs->cpri_remresetoutputctrl,
				C2_REM_RES_ACK_OP_EN_MASK);

	/* General Rx sync param */
	if (param->ctrl_flags & CPRI_GEN_RX_IQ_SYNC)
		cpri_reg_set(&cregs->cpri_rgensync,
				IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(&cregs->cpri_rgensync,
				IQ_SYNC_EN_MASK);

	/* General Tx sync param */
	if (param->ctrl_flags & CPRI_GEN_TX_IQ_SYNC)
		cpri_reg_set(&cregs->cpri_tgensync,
				IQ_SYNC_EN_MASK);
	else
		cpri_reg_clear(&cregs->cpri_tgensync,
				IQ_SYNC_EN_MASK);


	return;
}

static void cpri_set_test_mode(unsigned int mode, struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	if (mode & UL_TRANSPARENT)
		cpri_reg_set(&regs->cpri_map_config, RX_TRANSPARENT_MODE_MASK);
	else
		cpri_reg_clear(&regs->cpri_map_config,
				RX_TRANSPARENT_MODE_MASK);

	if (mode & DL_TRANSPARENT)
		cpri_reg_set(&regs->cpri_map_config, TX_TRANSPARENT_MODE_MASK);
	else
		cpri_reg_clear(&regs->cpri_map_config,
				TX_TRANSPARENT_MODE_MASK);
}

static int cpri_dev_ctrl(struct cpri_dev_ctrl *ctrl, struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_map_offsets map_sync_offset;
	struct axc **axcs;
	struct axc *axc;
	unsigned int loop;
	u32 mask;
	u32 line_sync_acheived;

	if ((ctrl->ctrl_mask & DEV_START_DL) ||
		(ctrl->ctrl_mask & DEV_START_UL)) {

		if (copy_from_user(&map_sync_offset,
				(struct cpri_map_offsets *)ctrl->ctrl_data,
				sizeof(struct cpri_map_offsets)) != 0)
			return -EFAULT;
	}

	if (ctrl->ctrl_mask & DEV_START_UL) {

		/* Set Tx map offset values */
		cpri_reg_set_val(&regs->cpri_map_offset_tx,
			MAP_OFFSET_X_MASK, map_sync_offset.tx_map_offset_bf);

		cpri_reg_set_val(&regs->cpri_map_offset_tx,
			MAP_OFFSET_Z_MASK, map_sync_offset.tx_map_offset_hf);

		cpri_reg_set_val(&regs->cpri_tstartoffset,
			MAP_OFFSET_X_MASK, map_sync_offset.tx_start_offset_bf);

		cpri_reg_set_val(&regs->cpri_tstartoffset,
			MAP_OFFSET_Z_MASK, map_sync_offset.tx_start_offset_hf);
		cpri_reg_set_val(&regs->cpri_map_config, AXC_MODE_MASK, 0x1);
		/* enable axc transmit control reg */
		cpri_reg_set(&regs->cpri_tcr, AXC_ENABLE_MASK);
		loop = 0;
		axcs = framer->ul_axcs;
		while (loop < framer->framer_param.max_axc_count) {
			if (axcs[loop] == NULL) {
				loop++;
				continue;
			}
			axc = axcs[loop];
			loop++;
			cpri_reg_set(&regs->cpri_taccr,
					(AXC_ENABLE_MASK << axc->id));
			cpri_reg_set(&regs->cpri_taxciqvspthreshinten,
					(AXC_ENABLE_MASK << axc->id));
		}
		/* Enable Tx */
		cpri_reg_set(&regs->cpri_config,
			CONF_TX_EN_MASK);

		/* Check hfn sync status */
		mask = (RX_HFN_STATE_MASK | RX_BFN_STATE_MASK);
		while (1) {
			line_sync_acheived = cpri_reg_get_val(
					&regs->cpri_status, mask);
			if (line_sync_acheived)
				break;

			schedule_timeout_interruptible(msecs_to_jiffies(20));
		}



		/* Set Tx control interrupt events */
		cpri_reg_set(&regs->cpri_tcr, IQ_EN_MASK);


		framer->dev_flags |= CPRI_DATA_MODE;

	} else if (ctrl->ctrl_mask & DEV_START_DL) {
		/* Set Rx map offset values */
		cpri_reg_set_val(&regs->cpri_map_offset_rx,
			MAP_OFFSET_X_MASK, map_sync_offset.rx_map_offset_bf);

		cpri_reg_set_val(&regs->cpri_map_offset_rx,
			MAP_OFFSET_Z_MASK, map_sync_offset.rx_map_offset_hf);

		cpri_reg_set_val(&regs->cpri_map_config, AXC_MODE_MASK, 0x1);
		/* enable axc transmit control reg */
		cpri_reg_set(&regs->cpri_rcr, AXC_ENABLE_MASK);
		loop = 0;
		axcs = framer->dl_axcs;
		while (loop < framer->framer_param.max_axc_count) {
			if (axcs[loop] == NULL) {
				loop++;
				continue;
			}
			axc = axcs[loop];
			loop++;
			cpri_reg_set(&regs->cpri_raccr,
					(AXC_ENABLE_MASK << axc->id));
			cpri_reg_set(&regs->cpri_raxciqvspthreshinten,
					(AXC_ENABLE_MASK << axc->id));
		}

		/* Enable Rx */
		cpri_reg_set(&regs->cpri_config,
			CONF_RX_EN_MASK);
		mdelay(10);
		/* Set Rx control interrupt events */
		cpri_reg_set(&regs->cpri_rcr, IQ_EN_MASK);



		framer->dev_flags |= CPRI_DATA_MODE;
		cpri_state_machine(framer,
					CPRI_STATE_OPERATIONAL);


	} else if (ctrl->ctrl_mask & DEV_STANDBY) {

		cpri_state_machine(framer,
					CPRI_STATE_STANDBY);

		/* Disable Tx and Rx */
		cpri_reg_clear(&regs->cpri_config,
			CONF_TX_EN_MASK|CONF_RX_EN_MASK);

		/* Disable Tx and Rx AxCs */
		cpri_reg_set_val(&regs->cpri_raccr,
			MASK_ALL, 0);

		cpri_reg_set_val(&regs->cpri_taccr,
			MASK_ALL, 0);

		/* Clear all control interrupt events */
		cpri_reg_clear(&regs->cpri_rcr,
			ETH_EN_MASK | VSS_EN_MASK | IQ_EN_MASK);

		cpri_reg_clear(&regs->cpri_tcr,
			ETH_EN_MASK | VSS_EN_MASK | IQ_EN_MASK);

	} else
		return -EINVAL;

	return 0;
}

static int validate_cpri_state(struct cpri_framer *framer, unsigned int cmd)
{
	int ret = 0;

	/* this is commented temporarily need to enable after sfp functionality
	 */
#if 0
	if (framer->framer_state == CPRI_STATE_SFP_DETACHED &&
			(framer->sfp_dev == NULL)) {
		ret = -EINVAL;
	}
#endif

	switch (cmd) {

	case CPRI_SET_AXC_PARAM:
	case CPRI_GET_AXC_PARAM:
	case CPRI_GET_MAP_TABLE:
	case CPRI_CTRL_AXC:
		ret = cpri_state_validation(framer->framer_state,
				CPRI_STATE_AXC_CONFIG);
		break;

	case CPRI_MAP_INIT_AXC:
	case CPRI_MAP_CLEAR_AXC:
		ret = cpri_state_validation(framer->framer_state,
				CPRI_STATE_AXC_MAP_INIT);
		break;
	case CPRI_START_AUTONEG:
	case CPRI_START_RECONFIG:
		ret = cpri_state_validation(framer->framer_state,
				CPRI_STATE_LINE_RATE_AUTONEG);
		break;
	}

	return ret;
}

long cpri_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct cpri_framer *framer = fp->private_data;
	struct device *dev = framer->cpri_dev->dev;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_common_regs __iomem *comm_regs = framer->cpri_dev->regs;
	struct cpri_dev_ctrl devctrl;
	struct cpri_dev_info info;

	struct cpri_reg_write_buf wreg;
	struct cpri_reg *wregset;
	struct cpri_reg_read_buf rreg;
	u32 *buf = NULL;
	unsigned int framer_state;

	struct sfp_reg_write_buf sfp_wreg;
	struct sfp_reg *sfp_wregset;
	struct sfp_reg_read_buf sfp_rreg;
	u8 *sfp_buf = NULL;

	int err = 0, count, i;
	void __user *ioargp = (void __user *)arg;

	if (_IOC_TYPE(cmd) != CPRI_MAGIC) {
		dev_err(dev, "invalid case, CMD=%d\n", cmd);
		return -EINVAL;
	}
	if (validate_cpri_state(framer, cmd)) {
		dev_err(dev, "cpri cmd discarded 'check cpri_state': %d\n",
				framer->framer_state);
		return -EINVAL;
	}

	switch (cmd) {
	case CPRI_INIT_DEV:
		if (copy_from_user(&(framer->framer_param),
				(struct cpri_dev_init_params *)ioargp,
				sizeof(struct cpri_dev_init_params)) != 0) {
			err = -EFAULT;
			goto out;
		}
		cpri_init_param(framer);
		break;

	case CPRI_CTRL_DEV:
		if (copy_from_user(&devctrl,
				(struct cpri_dev_ctrl *)ioargp,
				sizeof(struct cpri_dev_ctrl)) != 0) {
			err = -EFAULT;
			goto out;
		}

		cpri_dev_ctrl(&devctrl, framer);

		break;

	case CPRI_SET_TESTMODE:
		if (get_user(framer->test_flags, (unsigned int *)ioargp) != 0) {
			err = -EFAULT;
			goto out;
		}

		cpri_set_test_mode(framer->test_flags, framer);

		break;

	case CPRI_GET_STATE:
		if (copy_to_user((enum cpri_state *)ioargp,
				&(framer->framer_state),
				sizeof(enum cpri_state))) {
			err = -EFAULT;
			goto out;
		}

		break;

	case CPRI_BD_DUMP:
		bd_dump(framer->eth_priv->ndev);

		break;

	case CPRI_SET_STATE:
		if (copy_from_user(&framer_state,
				(enum cpri_state *)ioargp,
				sizeof(enum cpri_state)) != 0) {
			err = -EFAULT;
			goto out;
		}
		cpri_state_machine(framer,
					framer_state);

		break;

	case CPRI_GET_INFO:
		memset(&info, 0, sizeof(struct cpri_dev_info));

		cpri_fill_framer_info(&info, framer);
		if (copy_to_user((struct cpri_dev_info *)ioargp, &info,
				sizeof(struct cpri_dev_info))) {
			err = -EFAULT;
			goto out;
		}

		break;

	case CPRI_GET_STATS:
		cpri_fill_framer_stats(framer);

		if (copy_to_user((struct cpri_dev_stats *)ioargp,
			&framer->stats, sizeof(struct cpri_dev_stats))) {
			err = -EFAULT;
			goto out;
		}

		break;

	case CPRI_CLEAR_STATS:
		memset(&framer->stats, 0, sizeof(struct cpri_dev_stats));
		break;

	case CPRI_READ_REG:
		if (copy_from_user(&rreg, (struct cpri_reg_read_buf *)ioargp,
				sizeof(struct cpri_reg_read_buf)) != 0) {
			err = -EFAULT;
			goto out;
		}

		buf = kzalloc(sizeof(u32) * rreg.count, GFP_KERNEL);
		if (!buf) {
			err = -ENOMEM;
			goto out;
		}

		cpri_reg_read_bulk((u32 *)regs,
			rreg.start_offset, rreg.count, (u32 *)buf);

		if (copy_to_user((u32 *)rreg.reg_buff, (u32 *)buf,
				sizeof(u32) * rreg.count)) {
			err = -EFAULT;
			kfree(buf);
			goto out;
		}

		kfree(buf);
		break;

	case CPRI_WRITE_REG:
		if (copy_from_user(&wreg, (struct cpri_reg_write_buf *)ioargp,
				sizeof(struct cpri_reg_write_buf)) != 0) {
			err = -EFAULT;
			goto out;
		}

		count = wreg.count;
		wregset = kmalloc((sizeof(struct cpri_reg) * count),
					GFP_KERNEL);
		if (!wregset) {
			err = -ENOMEM;
			goto out;
		}

		if (copy_from_user(wregset, (struct cpri_reg *) wreg.regs,
				(sizeof(struct cpri_reg) * count)) != 0) {
			err = -EFAULT;
			kfree(wregset);
			goto out;
		}

		for (i = 0; i < count; i++) {
			cpri_reg_write_bulk((u32 *)regs,
				(wregset + i)->offset, 1, (wregset + i)->value);
		}

		kfree(wregset);
		break;

	case CPRI_READ_REG_COMMON:
		if (copy_from_user(&rreg, (struct cpri_reg_read_buf *)ioargp,
				sizeof(struct cpri_reg_read_buf)) != 0) {
			err = -EFAULT;
			goto out;
		}

		buf = kzalloc(sizeof(u32) * rreg.count, GFP_KERNEL);
		if (!buf) {
			err = -ENOMEM;
			goto out;
		}

		cpri_reg_read_bulk((u32 *)comm_regs,
			rreg.start_offset, rreg.count, (u32 *)buf);

		if (copy_to_user((u32 *)rreg.reg_buff, (u32 *)buf,
				sizeof(u32) * rreg.count)) {
			err = -EFAULT;
			kfree(buf);
			goto out;
		}

		kfree(buf);

		break;

	case CPRI_WRITE_REG_COMMON:
		if (copy_from_user(&wreg, (struct cpri_reg_write_buf *)ioargp,
				sizeof(struct cpri_reg_write_buf)) != 0) {
			err = -EFAULT;
			goto out;
		}

		count = wreg.count;

		wregset = kmalloc((sizeof(struct cpri_reg) * count),
					GFP_KERNEL);
		if (!wregset) {
			err = -ENOMEM;
			goto out;
		}

		if (copy_from_user(wregset, (struct cpri_reg *) wreg.regs,
				(sizeof(struct cpri_reg) * count)) != 0) {
			err = -EFAULT;
			kfree(wregset);
			goto out;
		}

		for (i = 0; i < count; i++) {
			cpri_reg_write_bulk(
				(u32 *)comm_regs, (wregset + i)->offset,
				1, (wregset + i)->value);
		}

		kfree(wregset);
		break;

	case SFP_READ_REG:
		if (copy_from_user(&sfp_rreg, (struct sfp_reg_read_buf *)ioargp,
				sizeof(struct sfp_reg_read_buf)) != 0) {
			err = -EFAULT;
			goto out;
		}

		sfp_buf = kzalloc(sizeof(u8) * sfp_rreg.count, GFP_KERNEL);
		if (!sfp_buf) {
			err = -ENOMEM;
			goto out;
		}

		sfp_raw_read(framer->sfp_dev, sfp_buf, sfp_rreg.start_offset,
			sfp_rreg.count, SFP_MEM_EEPROM);

		if (copy_to_user(sfp_rreg.reg_buff, buf,
				sizeof(u8) * rreg.count)) {
			err = -EFAULT;
			kfree(sfp_buf);
			goto out;
		}

		kfree(sfp_buf);
		break;

	case SFP_READ_DIAG_REG:
		if (copy_from_user(&sfp_rreg, (struct sfp_reg_read_buf *)ioargp,
				sizeof(struct sfp_reg_read_buf)) != 0) {
			err = -EFAULT;
			goto out;
		}

		sfp_buf = kzalloc(sizeof(u8) * sfp_rreg.count, GFP_KERNEL);
		if (!sfp_buf) {
			err = -ENOMEM;
			goto out;
		}

		sfp_raw_read(framer->sfp_dev, sfp_buf, sfp_rreg.start_offset,
			sfp_rreg.count, SFP_MEM_DIAG);

		if (copy_to_user(sfp_rreg.reg_buff, buf,
				sizeof(u8) * rreg.count)) {
			err = -EFAULT;
			kfree(sfp_buf);
			goto out;
		}

		kfree(sfp_buf);
		break;

	case SFP_WRITE_REG:
		if (copy_from_user(&sfp_wreg,
				(struct sfp_reg_write_buf *) ioargp,
				sizeof(struct sfp_reg_write_buf)) != 0) {
			err = -EFAULT;
			goto out;
		}

		count = sfp_wreg.count;
		sfp_wregset = kmalloc((sizeof(struct sfp_reg) * count),
					GFP_KERNEL);
		if (!sfp_wregset) {
			err = -ENOMEM;
			goto out;
		}

		if (copy_from_user(sfp_wregset,
				(struct sfp_reg *) sfp_wreg.regs,
				(sizeof(struct sfp_reg) * count)) != 0) {
			err = -EFAULT;
			kfree(sfp_wregset);
			goto out;
		}

		for (i = 0; i < count; i++) {
			sfp_raw_write(framer->sfp_dev,
				&(sfp_wregset + i)->value,
				(sfp_wregset + i)->offset, 1, SFP_MEM_DIAG);
		}

		kfree(sfp_wregset);
		break;

	case CPRI_BFN_RESET:
		cpri_reset_bfn(framer);

		break;
	case CPRI_SET_AXC_PARAM:
	case CPRI_GET_AXC_PARAM:
	case CPRI_GET_MAP_TABLE:
	case CPRI_MAP_INIT_AXC:
	case CPRI_CTRL_AXC:
	case CPRI_MAP_CLEAR_AXC:
		err = cpri_axc_ioctl(framer, arg, cmd);
		if (err < 0)
			goto out;
		break;


	default:
		err = cpri_autoneg_ioctl(framer, cmd, arg);
		if (err < 0)
			goto out;

		break;
	}

	return 0;

out:
	dev_err(dev, "IOCTL failure\n");
	return err;
}

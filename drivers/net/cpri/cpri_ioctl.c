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
#include <linux/gpio.h>

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

static int calc_nframe_delay(struct cpri_framer *framer,
		struct frame_diff_buf *buf)
{
	u32 nframer_diff_read = 0;
	int err = 0;
	u32 count = 0;

	cpri_write(CPRI_FRAME_DIFF_STATUS_BIT,
			&framer->regs->cpri_framediffstatus);
	cpri_write(CPRI_FRAME_DIFF_STATUS_BIT,
			&framer->regs->cpri_framediffctrl);
	/* Waiting for nframediff cal to be over */
	while (1) {
		nframer_diff_read = cpri_reg_get_val(
				&framer->regs->cpri_framediffstatus,
				MASK_ALL);
		if (nframer_diff_read & CPRI_FRAME_DIFF_STATUS_BIT) {
			buf->framediff_hfn = (nframer_diff_read >> 8) & 0XFF;
			buf->framediff_x = (nframer_diff_read >> 16) & 0xFF;
			break;
		}

		if (count == NFRAME_DIFF_COUNT_LOOP) {
			err = -ETIME;
			break;
		}

		schedule_timeout_interruptible(msecs_to_jiffies(200));
		count++;
	}

	return err;
}

static void cpri_config_axc_offset(struct cpri_framer *framer,
		const struct cpri_axc_map_offset *offset)
{
	u32 reg_val;
	reg_val = (offset->map_rx_offset_z << 8) | offset->map_rx_offset_x;
	cpri_write(reg_val, &framer->regs->cpri_map_offset_rx);

	reg_val = (offset->map_tx_offset_z << 8) | offset->map_tx_offset_x;
	cpri_write(reg_val, &framer->regs->cpri_map_offset_tx);

	reg_val = (offset->start_tx_offset_z << 8) | offset->start_tx_offset_x;
	cpri_write(reg_val, &framer->regs->cpri_tstartoffset);
}

/* Enable CPRI HW reset the board.
 * Right now single hop reset function is tested.
 */
void cpri_config_hwrst(struct cpri_framer *framer, int enable)
{

	src_cpri_hwrst(enable);

	if (enable) {
		cpri_write(0x303,
			&framer->cpri_dev->regs->cpri_remresetoutputctrl);
		cpri_reg_set(&framer->regs->cpri_hwreset, 0x4);
	} else {
		cpri_write(0,
			&framer->cpri_dev->regs->cpri_remresetoutputctrl);
		cpri_reg_clear(&framer->regs->cpri_hwreset, 0x4);
	}
}

long cpri_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct cpri_framer *framer = fp->private_data;
	struct device *dev = framer->cpri_dev->dev;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_common_regs __iomem *comm_regs = framer->cpri_dev->regs;

	struct cpri_reg_write_buf wreg;
	struct cpri_reg *wregset;
	struct cpri_reg_read_buf rreg;
	u32 *buf = NULL;

	struct sfp_reg_write_buf sfp_wreg;
	struct sfp_reg *sfp_wregset;
	struct sfp_reg_read_buf sfp_rreg;
	struct serdes_amp_data sfp_amp;
	u8 *sfp_buf = NULL;

	struct rx_cw_params rx_cw_param;
	struct tx_cw_params tx_cw_param;
	char cw_data[16];

	struct frame_diff_buf frame_diff;
	struct monitor_config_en monitor_cfg_en;
	struct monitor_config_disable monitor_cfg_disable;

	int err = 0, count, i;
	void __user *ioargp = (void __user *)arg;
	struct cpri_axc_map_offset axc_map_offset;
	struct sfp_dev *sfp = framer->sfp_dev;
	int err_cnt[CPRI_ERR_CNT];

	if (_IOC_TYPE(cmd) != CPRI_MAGIC) {
		dev_err(dev, "invalid case, CMD=%d\n", cmd);
		return -EINVAL;
	}

	switch (cmd) {

	case CPRI_GET_STATE:
		if (gpio_get_value_cansleep(sfp->prs))
			clear_bit(CPRI_SFP_PRESENT_BITPOS,
				&framer->cpri_state);
		else
			set_bit(CPRI_SFP_PRESENT_BITPOS,
				&framer->cpri_state);
		if (copy_to_user((u32 *)ioargp,
				(u32 *)&framer->cpri_state, sizeof(u32))) {
			err = -EFAULT;
			goto out;
		}
		break;

	case CPRI_GET_NFRAME_DIFF:
		err = calc_nframe_delay(framer, &frame_diff);
		if (err)
			return err;
		else if (copy_to_user((u32 *)ioargp, &frame_diff,
				sizeof(struct frame_diff_buf))) {
			err = -EFAULT;
			goto out;
		}
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

	case SERDES_AMP_SET:
		if (copy_from_user(&sfp_amp,
				(struct serdes_amp_data *)ioargp,
				sizeof(struct serdes_amp_data)) != 0) {
			err = -EFAULT;
			goto out;
		}
		err = set_sfp_input_amp_limit(framer, sfp_amp.sfp_max_mvolt,
				sfp_amp.rxequil_boost_en);
		if (err) {
			err = -EINVAL;
			goto out;
		}

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

		if (copy_to_user(sfp_rreg.reg_buff, sfp_buf,
				sizeof(u8) * sfp_rreg.count)) {
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

		if (copy_to_user(sfp_rreg.reg_buff, sfp_buf,
				sizeof(u8) * sfp_rreg.count)) {
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

	case CPRI_AXC_MAP:
	case CPRI_AXC_CTRL:
		err = cpri_axc_ioctl(framer, arg, cmd);
		if (err < 0)
			goto out;
		break;

	case CPRI_RX_CTRL_TABLE:
		memset(cw_data, 0, sizeof(cw_data));
		if (copy_from_user(&rx_cw_param,
				(struct rx_cw_params *) ioargp,
				sizeof(struct rx_cw_params)) != 0) {
			err = -EFAULT;
			goto out;
		}
		spin_lock(&framer->rx_cw_lock);
		read_rx_cw(framer, rx_cw_param.bf_index, cw_data);
		spin_unlock(&framer->rx_cw_lock);

		if (copy_to_user(rx_cw_param.data,
			cw_data,
			rx_cw_param.len < 16 ? rx_cw_param.len : 16)) {
			err = -EFAULT;
			goto out;
		}
		break;

	case CPRI_TX_CTRL_TABLE:
		memset(cw_data, 0, sizeof(cw_data));
		if (copy_from_user(&tx_cw_param,
				(struct tx_cw_params *) ioargp,
				sizeof(struct tx_cw_params)) != 0) {
			err = -EFAULT;
			goto out;
		}

		if (tx_cw_param.operation & TX_CW_WRITE) {
			if (copy_from_user(cw_data, tx_cw_param.data,
					tx_cw_param.len < 16 ?
					tx_cw_param.len : 16) != 0) {
				err = -EFAULT;
				goto out;
			}
		}
		spin_lock(&framer->tx_cw_lock);
		rdwr_tx_cw(framer, tx_cw_param.bf_index,
				tx_cw_param.operation, cw_data);
		spin_unlock(&framer->tx_cw_lock);

		if (tx_cw_param.operation & TX_CW_READ) {
			if (copy_to_user(tx_cw_param.data, cw_data,
				tx_cw_param.len < 16 ? tx_cw_param.len : 16)) {
				err = -EFAULT;
				goto out;
			}
		}
		break;

	case CPRI_READ_STATS:
		for (i = 0; i < CPRI_ERR_CNT; i++)
			err_cnt[i] = atomic_read(&framer->err_cnt[i]);
		if (copy_to_user((struct cpri_dev_stats *) ioargp,
				err_cnt, sizeof(err_cnt))) {
			err = -EFAULT;
			goto out;
		}
	break;

	case CPRI_SET_MONITOR:
		if (copy_from_user(&monitor_cfg_en,
			(struct monitor_config_en *)ioargp,
			sizeof(struct monitor_config_en)) != 0) {
			err = -EFAULT;
			goto out;
		}
		cpri_set_monitor(framer, &monitor_cfg_en);
	break;

	case CPRI_CLEAR_MONITOR:
		if (copy_from_user(&monitor_cfg_disable,
			(struct monitor_config_disable *)ioargp,
			sizeof(struct monitor_config_disable)) != 0) {
			err = -EFAULT;
			goto out;
		}
		cpri_clear_monitor(framer, &monitor_cfg_disable);
	break;

	case CPRI_AXC_OFFSET_REGS:
	if (copy_from_user(&axc_map_offset,
			(struct cpri_axc_map_offset *) ioargp,
			sizeof(struct cpri_axc_map_offset)) != 0) {
		err = -EFAULT;
		goto out;
	}

	cpri_config_axc_offset(framer, &axc_map_offset);

	break;

	case CPRI_HW_RESET:
		cpri_config_hwrst(framer, (int)ioargp);
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

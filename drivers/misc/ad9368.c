/*
 * File: ad9368.c
 *
 * AD9368-1 is RF Reciever, AD9368-2 is RF Transciever from Analog
 * devices. These are used for 3G/4G macro BTS, Active antenna,
 * and small cell applications.
 * This driver supports both Ad9368-1 and AD9368-2 RFICs, it provides
 * interface for progmming ADI script, configuting GPIOs, read/write
 * gain and attenuations etc.
 *
 * This driver is derived work from drivers/rf/phy/ad9361.c
 * Author: pankaj chauhan <pankaj.chauhan@freescale.com>
 * Author: Rohit Thapliyal <rohit.thapliyal@freescale.com>
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/ad9368.h>

#define DEV_NAME "AD9368"

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

struct ad9368_drv_priv {
	bool reset_status;
	struct class *class;
	unsigned int minor;
	unsigned int minor_max;
	dev_t dev_t;
	struct list_head dev_list;
};

static struct ad9368_drv_priv *drv_priv;

MODULE_LICENSE("GPL v2");

static int ad9368_run_cmds(struct rf_phy_dev *phy_dev,
		struct rf_phy_cmd *cmds,
		int count);
static int ad9368_read(struct rf_phy_dev *phy_dev, u32 start, u32 count,
		u32 *buff);
static int ad9368_write(struct rf_phy_dev *phy_dev, u32 reg,
		 u32 data, u8 probe);
static int ad9368_start(struct rf_phy_dev *phy_dev);
static int ad_init(struct rf_phy_dev *phy, struct rf_init_params *params);
static int ad9368_stop(struct rf_phy_dev *phy);
static int ad9368_open(struct inode *inode, struct file *filep);
static int ad9368_release(struct inode *inode, struct file *filep);
static long ad9368_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg);
static int rfdev_message(struct rf_phy_dev *rf_dev,
	struct spi_ioc_transfer *u_xfers, unsigned n_xfers);

static struct rf_phy_ops ad9368_ops = {
	.init = ad_init,
	.run_cmds = ad9368_run_cmds,
	.read_regs = ad9368_read,
	.write_reg = ad9368_write,
	.start = ad9368_start,
	.stop = ad9368_stop,
	.spi_ioc_transfer = rfdev_message,
};

static const struct file_operations ad9368_fops = {
	.owner		= THIS_MODULE,
	.open		= ad9368_open,
	.release	= ad9368_release,
	.unlocked_ioctl	= ad9368_ioctl,
};

static const struct rf_device_id ad9368_spi_id[] = {
	{ "ad9368-1", DEVICE_ID_AD93681 },
	{ "ad9525", DEVICE_ID_AD9525 },
	{ "ad9368-2", DEVICE_ID_AD93682},
};

static int ad9368_open(struct inode *inode, struct file *filep)
{
	int minor, match = 0, size, index = 0;
	struct rf_phy_dev *phy_dev = NULL;
	struct ad_dev_info *phy_info;
	struct device *dev;

	minor = iminor(inode);
	list_for_each_entry(phy_dev, &drv_priv->dev_list, list) {
		if (MINOR(phy_dev->dev_t) == minor) {
			match = 1;
			break;
		}
	}

	phy_info = phy_dev->priv;
	dev = &(phy_info->ad_spi->dev);
	size = sizeof(ad9368_spi_id)/sizeof(ad9368_spi_id[0]);
	while (index < size) {
		if (!strncmp(phy_info->ad_spi->modalias,
			ad9368_spi_id[index].name,
				strlen(phy_info->ad_spi->modalias))) {
			phy_info->device_id = index;
			break;
		}
	index++;
	}
	if (!match) {
		return -ENODEV;
	} else {
		filep->private_data = phy_dev;
		atomic_inc(&phy_dev->ref);
	}
	return 0;
}

int ad9368_release(struct inode *inode, struct file *filep)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = filep->private_data;
	atomic_dec(&phy_dev->ref);

	return 0;
}


static void phy_info_complete(void *arg)
{
	complete(arg);
}

static ssize_t
rfdev_sync(struct ad_dev_info *phy_info, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	message->complete = phy_info_complete;
	message->context = &done;

	spin_lock_irq(&phy_info->spi_lock);
	if (phy_info->ad_spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(phy_info->ad_spi, message);
	spin_unlock_irq(&phy_info->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static int rfdev_message(struct rf_phy_dev *rf_dev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct ad_dev_info *phy_info = rf_dev->priv;
	struct spi_message      msg;
	struct spi_transfer     *k_xfers;
	struct spi_transfer     *k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned                n, total;
	u8                      *buf;
	int                     status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	buf = phy_info->rx_buf;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf)
			k_tmp->rx_buf = buf;
		if (u_tmp->tx_buf)
			k_tmp->tx_buf = (const u8 __user *)
					(uintptr_t)u_tmp->tx_buf;
		buf += k_tmp->len;
		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		dev_dbg(&phy_info->ad_spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? :
			phy_info->ad_spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : phy_info->ad_spi->max_speed_hz);
		spi_message_add_tail(k_tmp, &msg);
	}

	status = rfdev_sync(phy_info, &msg);
	if (status < 0)
		goto done;

	/*for (n=0;n<3;n++)
		dev_info(&phy_info->ad_spi->dev,"RX BUF:%x",
				phy_info->rx_buf[n]);*/
	/* copy any rx data out of bounce buffer */
	buf = phy_info->rx_buf;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
				(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
	buf += u_tmp->len;
	}
	status = total;
done:
	kfree(k_xfers);
	return status;
}


static long ad9368_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	int err = 0;
	struct rf_phy_dev *phy_dev;
	struct rf_init_params init_params;
	struct rf_phy_cmd_set cmd_set;
	struct rf_reg_buf reg_buf;
	struct rf_write_reg_buf write_reg_buf;
	struct rf_tx_buf tx_buf;
	struct rf_tx_en_dis tx_en_dis;
	struct rf_rx_gain rx_gain;
	struct rf_gain_ctrl gain_ctrl;
	struct spi_ioc_transfer ioc_transfer;
	u32	*buf;
	int rc = -ENOSYS, size;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	phy_dev = filep->private_data;
	switch (cmd) {

	case RF_DEV_INIT:

		if (copy_from_user(&init_params,
			(struct rf_init_params *)arg, sizeof(init_params))) {
			rc = -EFAULT;
			goto out;
		}

		if (phy_dev)
			rc = phy_dev->ops->init(phy_dev, &init_params);

		break;

	case RF_RUN_PHY_CMDS:
		rc = -EFAULT;
		if (!copy_from_user(&cmd_set, (struct rf_phy_cmd_set *)arg,
			sizeof(struct rf_phy_cmd_set))) {

			size = sizeof(struct rf_phy_cmd) * cmd_set.count;
			buf = kzalloc(size, GFP_KERNEL);
			if (buf) {
				if (!copy_from_user(buf,
						(u32 *) cmd_set.cmds, size))
					rc = phy_dev->ops->run_cmds(phy_dev,
						(struct rf_phy_cmd *)buf,
						cmd_set.count);
				kfree(buf);
			} else {
				rc = -ENOMEM;
			}
		}
		break;

	case RF_EN_DIS_TX:

		if (!copy_from_user(&tx_en_dis, (struct rf_tx_en_dis *)arg,
				sizeof(struct rf_tx_en_dis))) {
			if (phy_dev->ops->en_dis_tx) {
				rc = phy_dev->ops->en_dis_tx(phy_dev,
						tx_en_dis.tx_if,
						tx_en_dis.tx_cmd);
			}
		} else {
			rc = -EFAULT;
		}
		break;

	case RF_READ_PHY_REGS:

		if (!copy_from_user(&reg_buf, (struct rf_reg_buf *)arg,
			sizeof(struct rf_reg_buf))) {

			size = 4 * reg_buf.count;
			buf = kzalloc(size, GFP_KERNEL);
			if (!buf) {
				rc = -ENOMEM;
				goto out;
			}
			rc = phy_dev->ops->read_regs(phy_dev, reg_buf.addr,
				reg_buf.count, buf);

			if (copy_to_user((u32 *)reg_buf.buf,
					buf, size))
				rc = -EFAULT;
			kfree(buf);
		} else {
			rc = -EFAULT;
		}
		break;

	case RF_WRITE_PHY_REGS:

		if (!copy_from_user(&write_reg_buf,
			(struct rf_write_reg_buf *)arg,
			sizeof(struct rf_write_reg_buf))) {
			rc = phy_dev->ops->write_reg(phy_dev,
						write_reg_buf.addr,
						write_reg_buf.data, 0);
		} else {
			rc = -EFAULT;
		}

		break;

	case RF_START:
		rc = phy_dev->ops->start(phy_dev);
		break;

	case RF_STOP:
		rc = phy_dev->ops->stop(phy_dev);
		break;

	case RF_SET_TX_ATTEN:
		if (!copy_from_user(&tx_buf, (struct rf_tx_buf *)arg,
				sizeof(struct rf_tx_buf))) {
			if (phy_dev->ops->set_tx_atten) {
				rc = phy_dev->ops->set_tx_atten(phy_dev,
						tx_buf.tx_if,
						tx_buf.tx_atten);
			}
		} else {
			rc = -EFAULT;
		}
		break;

	case RF_READ_RX_GAIN:
		if (!copy_from_user(&rx_gain, (struct rf_rx_gain *)arg,
			sizeof(struct rf_rx_gain))) {

			if (phy_dev->ops->get_rx_gain)
				rc = phy_dev->ops->get_rx_gain(phy_dev,
							&rx_gain);

			if (!rc && copy_to_user((struct rf_rx_gain *)arg,
						&rx_gain,
						sizeof(struct rf_rx_gain)))
				rc = -EFAULT;
		} else {
			rc = -EFAULT;
		}
		break;

	case RF_WRITE_RX_GAIN:

		if (!copy_from_user(&rx_gain, (struct rf_rx_gain *)arg,
			sizeof(struct rf_rx_gain))) {

			if (phy_dev->ops->set_rx_gain)
				rc = phy_dev->ops->set_rx_gain(phy_dev,
						&rx_gain);
		} else {
			rc = -EFAULT;
		}
		break;

	case RF_SET_GAIN_CTRL_MODE:

		if (!copy_from_user(&gain_ctrl, (struct rf_gain_ctrl *)arg,
			sizeof(struct rf_gain_ctrl))) {

			if (phy_dev->ops->set_gain_ctrl_mode)
				rc = phy_dev->ops->set_gain_ctrl_mode(phy_dev,
						&gain_ctrl);
		} else {
			rc = -EFAULT;
		}
		break;
	case RF_SPI_IOC_TRANSFER:
		if (!copy_from_user(&ioc_transfer,
			(struct spi_ioc_transfer *)arg,
			sizeof(struct spi_ioc_transfer))) {
			if (phy_dev->ops->spi_ioc_transfer)
				rc = phy_dev->ops->spi_ioc_transfer(phy_dev,
					&ioc_transfer, 1);
		}	else
			rc = -EFAULT;
	break;


	default:
		rc = -ENOSYS;
	}

out:
	return rc;
}

int rf_release(struct inode *inode, struct file *filep)
{
	struct rf_phy_dev *phy_dev;

	phy_dev = filep->private_data;
	atomic_dec(&phy_dev->ref);

	return 0;
}



static int spi_write_transaction(struct rf_phy_dev *phy_dev, u8 *buf, int len)
{
	u8 *tx_buf;
	int status;
	struct spi_message spi_msg;
	struct spi_transfer ad_tx;
	struct ad_dev_info *phy_info = phy_dev->priv;

	spi_message_init(&spi_msg);
	memset(&ad_tx, 0, sizeof(ad_tx));
	tx_buf = buf;
	ad_tx.len = len;
	ad_tx.tx_buf = tx_buf;
	spi_message_add_tail(&ad_tx, &spi_msg);
	status = rfdev_sync(phy_info, &spi_msg);
	return status;
}


static int check_cal_done(struct rf_phy_dev *phy_dev, u32 reg, u32 mask,
		u32 bitval)
{
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev = &phy_info->ad_spi->dev;
	u32 val;
	int rc = 0;
	ad9368_read(phy_dev, reg, 1, &val);
	switch (bitval) {
	case 0:
		if (!(val & mask))
			rc = 1;
		break;
	case 1:
		if (val & mask)
			rc = 1;
		break;
	default:
		dev_err(dev, "Invalid bit value\n");
		return -EINVAL;
	}
	return rc;
}

int ad9368_run_cmds(struct rf_phy_dev *phy_dev,
		struct rf_phy_cmd *cmds, int count)
{
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev = &phy_info->ad_spi->dev;
	int i, elapsed_time;
	u32 val;
	for (i = 0; i < count; i++) {
		switch (cmds[i].cmd) {
		case SPI_WRITE:
			dev_dbg(dev, "SPI_WRITE addr:%x data:%x\n",
				cmds[i].param1, cmds[i].param2);
			/*if (cmds[i].param1 == REG_RX_CP_CONFIG) {
				ad9368_read(phy_dev,
					REG_RX_CAL_STATUS, 1, &val);
				if (val & MASK_RX_CP_CAL_VALID)
					break;
			} else if (cmds[i].param1 == REG_TX_CP_CONFIG) {
				ad9368_read(phy_dev,
					REG_TX_CAL_STATUS, 1, &val);
				if (val & MASK_TX_CP_CAL_VALID)
					break;
			}*/
			ad9368_write(phy_dev, cmds[i].param1,
				cmds[i].param2, 0);
			dev_dbg(dev, "Write: %x %x\n", cmds[i].param1,
							cmds[i].param2);
			break;

		case SPI_READ:

			/* XXX: This READ is not returnning anything from
			 * here so can be removed safely. But not removing
			 * it completely from here because these reads may
			 * impact timing between reg updates up RFIC.
			 *
			 * This needs to be fixed, right way is to return
			 * read values, and let user space decide what to
			 * do this value.
			 */
			dev_dbg(dev, "SPI_READ %x\n", cmds[i].param1);
			ad9368_read(phy_dev, cmds[i].param1, 1, &val);
			dev_info(dev, "Read from reg: %x, val %x\n",
				cmds[i].param1, val);
			break;

		case SPI_WAIT:
			dev_dbg(dev, "SPI_WAIT %u\n", cmds[i].param1);
			msleep_interruptible(cmds[i].param3);
				break;

		case SPI_WAIT_BBPLL_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev, REG_CH1_OVERFLOW,
							MASK_BBPLL_LOCK, 1))
					break;
			}
			break;

		case SPI_WAIT_RXCP_CAL:
			dev_dbg(dev, "SPI_WAIT_RXCMP_CALL %u\n",
			cmds[i].param1);
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev, REG_RX_CAL_STATUS,
						MASK_RX_CP_CAL_VALID, 1))
					break;
			}
			break;

		case SPI_WAIT_TXCP_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_TX_CAL_STATUS,
						MASK_TX_CP_CAL_VALID, 1))
					break;
			}
			break;

		case SPI_WAIT_RXFILTER_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_RX_BB_TUNE, 0))
					break;
			}
			break;

		case SPI_WAIT_TXFILTER_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_TX_BB_TUNE, 0))
					break;
			}
			break;

		case SPI_WAIT_BBDC_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_DC_CAL_BBSTART, 0))
					break;
			}
			break;

		case SPI_WAIT_RFDC_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_DC_CAL_RFSTART, 0))
					break;
			}
			break;

		case SPI_WAIT_TXQUAD_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_TXQUAD_CAL, 0))
					break;
			}
			break;

		case SPI_WAIT_CALDONE:
			dev_dbg(dev, "Waiting for unknown CALDONE.\n");
			msleep_interruptible(cmds[i].param3);
			break;
		case SPI_SET_CHANNEL:
			phy_info->device_id = cmds[i].param3;
			dev_dbg(dev, "SPI_SET_CHANNEL RECEIVED %u.\n",
			 cmds[i].param3);
			break;
		case SPI_WAIT_CLKPLL_CAL:
			dev_dbg(dev, "SPI_WAIT_CLKPLL_CAL RECEIVED.\n");
			break;
		case SPI_WAIT_INITARM_CAL:
			dev_dbg(dev, "SPI_WAIT_INITARM_CAL RECEIVED.\n");
			break;
		case SPI_WAIT_RFPLLLOCK_CAL:
			dev_dbg(dev, "SPI_WAIT_RFPLLLOCK_CAL RECEIVED.\n");
			break;
		case SPI_WAIT_RFPLLCP_CAL:
			dev_dbg(dev, "SPI_WAIT_RFPLLCP_CAL RECEIVED.\n");
			break;
		case SPI_WAIT_ADCTUNER_CAL:
			dev_dbg(dev, "SPI_WAIT_ADCTUNER_CAL RECEIVED.\n");
			break;
		case SPI_WAIT_RXTIA_CAL:
			dev_dbg(dev, "SPI_WAIT_RXTIA_CAL RECEIVED.\n");
			break;
		case SPI_WAIT_RCAL_CAL:
			dev_dbg(dev, "SPI_WAIT_RCAL_CAL RECEIVED.\n");
			break;
		default:
			dev_dbg(dev, "Not a valid AD_PHY command\n");
			return -EINVAL;
		}
	}

	return 0;
}

int ad_init(struct rf_phy_dev *phy_dev,
		struct rf_init_params *params)
{
	struct ad_dev_info *phy_info = phy_dev->priv;
	int rc = 0;

	phy_info->prev_ensm_state = ENSM_STATE_INVALID;

	return rc;
}

int ad9368_stop(struct rf_phy_dev *phy_dev)
{

	return 0;
}

int ad9368_start(struct rf_phy_dev *phy_dev)
{
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev = &phy_info->ad_spi->dev;

	dev_dbg(dev, "Reading BBPLL locked status.\n");

	if (check_cal_done(phy_dev, REG_CH1_OVERFLOW,
			MASK_BBPLL_LOCK, 1))
		return 0;
	else {
		dev_err(dev, "BBPLL not locked.\n");
		return -EBUSY;
	}
}

int ad9368_read(struct rf_phy_dev *phy_dev, u32 start,
		u32 count, u32 *buff)
{
	u32 data;
	u16 read_addr;
	int i = 0, j = 0, rc;
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev = &phy_info->ad_spi->dev;
	uint8_t tx[2];
	uint8_t rx[3] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = TRANSACTION_BYTES,
		.delay_usecs = 0,
		.speed_hz = 500000,
		.bits_per_word = 8,
	};
	/* RFdev f/w provides start address as u32, but
	 * ADI has only 12 bits regs, so downsize it to 16 bits
	 */
	read_addr =  (u16)(start + count - 1);
	j = count - 1;
	if (phy_info->device_id == DEVICE_ID_AD9525)
		tx[0] = OPCODE_READ_AD9525;
	else
		tx[0] = OPCODE_READ;

	tx[0] |=	(read_addr>>8);
	tx[1] = (u8)read_addr;

	rc = rfdev_message(phy_dev, &tr, 1);
	if (rc < 0)
		goto out;

	for (i = COMMAND_LEN; i < (COMMAND_LEN + count); i++) {
		data = phy_info->rx_buf[i];
		buff[j--] = data;
	}
out:
	dev_dbg(dev, "Failed to read %d regs, start addr %x\n",
		count, start);
	return rc;
}


int ad9368_write(struct rf_phy_dev *phy_dev, u32 reg,
		u32 data, u8 probe)
{
	int rc;
	u8 cntrwd[3];
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev = &phy_info->ad_spi->dev;
	dev_dbg(dev, "AD's register write call.\n");

	if (phy_info->device_id == DEVICE_ID_AD9525) {
		if (probe == 1) {
			cntrwd[0] = OPCODE_WRITE_AD9525;
			cntrwd[1] = OPCODE_INIT_REG;
			cntrwd[2] = OPCODE_MIRROR_VAL;
			rc = spi_write_transaction(phy_dev, cntrwd,
				TRANSACTION_BYTES);
			cntrwd[0] = AD9525_PUSH_WRITE1_REG;
			cntrwd[1] = AD9525_PUSH_WRITE2_REG;
			cntrwd[2] = OPCODE_INIT_VAL;
			rc = spi_write_transaction(phy_dev, cntrwd,
				TRANSACTION_BYTES);
			cntrwd[0] = OPCODE_WRITE_AD9525;
			cntrwd[1] = AD9525_READBACK_REG;
			cntrwd[2] = OPCODE_INIT_VAL;
			rc = spi_write_transaction(phy_dev, cntrwd,
				TRANSACTION_BYTES);
		} else if (probe == 0) {
			cntrwd[0] = OPCODE_WRITE_AD9525;
			cntrwd[0] |= (reg>>8);
			cntrwd[1] = (u8)reg;
			cntrwd[2] = (u8)data;
			rc = spi_write_transaction(phy_dev, cntrwd,
				TRANSACTION_BYTES);
		}
		cntrwd[0] = AD9525_PUSH_WRITE1_REG;
		cntrwd[1] = AD9525_PUSH_WRITE2_REG;
		cntrwd[2] = OPCODE_INIT_VAL;
		rc = spi_write_transaction(phy_dev, cntrwd, TRANSACTION_BYTES);
	} else {
		cntrwd[0] = OPCODE_WRITE_AD9368;
		cntrwd[1] = OPCODE_INIT_REG;
		cntrwd[2] = OPCODE_INIT_REG;
		rc = spi_write_transaction(phy_dev, cntrwd, TRANSACTION_BYTES);
		cntrwd[0] = OPCODE_WRITE_AD9368;
		cntrwd[0] |= (reg>>8);
		cntrwd[1] = (u8)reg;
		cntrwd[2] = (u8)data;
		rc = spi_write_transaction(phy_dev, cntrwd, TRANSACTION_BYTES);
	}
	return rc;
}

static int ad9368_remove(struct spi_device *spi)
{
	int ret = 0;
	struct device *dev = &spi->dev;
	struct rf_phy_dev *phy_dev = dev_get_drvdata(&spi->dev);
	struct ad_dev_info *phy_info = phy_dev->priv;

	dev_dbg(dev, "AD9368 PHY module uninstalled\n");

	gpio_free(phy_info->reset_gpio);
	cdev_del(&phy_dev->cdev);
	device_destroy(drv_priv->class, phy_dev->dev_t);
	kfree(phy_dev);

	return ret;
}

static int check_revision(struct rf_phy_dev *phy_dev)
{
	u32 val;
	u8 rev;
	int rc = 0;
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev = &(phy_info->ad_spi->dev);

	if (phy_info->device_id == DEVICE_ID_AD93681) {
		ad9368_read(phy_dev, PRODUCT_CODE_AD93681_REG, 1, &val);
		rev = (u8) val & REV_MASK;
		if (rev != 7) {
			dev_err(dev, "Unsuported AD9368 revision 0x%x\n", rev);
			rc = -ENODEV;
			goto out;
		}
		dev_dbg(dev, "Detected AD9368 Rev 0x%x\n", rev);
	}
	if (phy_info->device_id == DEVICE_ID_AD9525) {
		ad9368_write(phy_dev, PRODUCT_CODE_AD9525_REG, 0, 1);
		ad9368_read(phy_dev, PRODUCT_CODE_AD9525_REG, 1, &val);
		rev = (u8) val;
		if (rev != 1) {
			dev_err(dev, "Unsuported AD9525 revision 0x%x\n", rev);
			rc = -ENODEV;
			goto out;
		}
		dev_dbg(dev, "Detected AD9525 Rev 0x%x\n", rev);
	}

	/*product_id = (u8) val & PRODUCT_ID_MASK;
	if (product_id != PRODUCT_ID_9368) {
		dev_err(dev, "Unsuported product id 0x%x\n", product_id);
		rc = -ENODEV;
		goto out;
	}*/
	return rc;
out:
	dev_err(dev, "AD9368 revision Failed\n");
	return rc;
}


static int ad_init_gpio(struct device_node *child, struct device *dev,
			struct ad_dev_info *phy_info)
{
/*	char *gpio_name;
	int gpio, ret = 0;
	u32 flags;

	gpio_name = (char *) of_get_property(child, "label", NULL);
	if (!gpio_name)
		gpio_name = (char *) child->name;

	gpio = of_get_gpio_flags(child, 0, &flags);
	if (gpio < 0) {
		dev_err(dev, "Could not get gpio for %s, err %d\n",
			gpio_name, gpio);
		ret = gpio;
		goto out;
	}

	if (!strncmp(gpio_name, "reset", RF_NAME_SIZE))
		phy_info->reset_gpio = gpio;
	else if (!strncmp(gpio_name, "pa_en", RF_NAME_SIZE))
		phy_info->pa_en_gpio = gpio;
	else if (!strncmp(gpio_name, "lna_en", RF_NAME_SIZE))
		phy_info->lna_en_gpio = gpio;

	ret = gpio_request(gpio, dev_name(dev));
	if (ret) {
		dev_dbg(dev, "gpio_request failed, gpio %s[%d]",
				gpio_name, gpio);
		goto out;
	}

	ret = gpio_direction_output(gpio, 0);
	if (ret) {
		dev_dbg(dev,
			"gpio_direction_output failed, gpio %s[%d]",
			gpio_name, gpio);
		goto out;
	}

out:
	return ret;
*/
	return 0;
}

static void ad_reset_phy(struct ad_dev_info *phy_info, struct device *dev)
{
	if ((phy_info->reset_gpio == GPIO_INVAL) ||
		(phy_info->lna_en_gpio == GPIO_INVAL) ||
		(phy_info->pa_en_gpio == GPIO_INVAL))
		dev_info(dev, "Control GPIOs not intialized in dtb\n");

	if ((phy_info->reset_gpio != GPIO_INVAL)) {
		/* Toggle reset gpio to reset AD9368 */
		gpio_set_value(phy_info->reset_gpio, 0);
		gpio_set_value(phy_info->reset_gpio, 1);
	}

}


static int ad9368_probe(struct spi_device *spi)
{
	static struct rf_phy_dev *phy_dev;
	struct device *dev = &spi->dev;
	struct ad_dev_info *phy_info;
	struct device_node *np = spi->dev.of_node;
	char dev_name[RF_NAME_SIZE];

	int ret = 0, size;
	memset(dev_name, 0, RF_NAME_SIZE);
	size = sizeof(struct rf_phy_dev) + sizeof(struct ad_dev_info);
	phy_dev = kzalloc(size, GFP_KERNEL);
	if (!phy_dev) {
		dev_dbg(dev, "Failed to allocate rf_phy_dev\n");
		return -ENOMEM;
	}

	phy_dev->priv = (void *) ((u32) phy_dev + sizeof(struct rf_phy_dev));
	phy_dev->ops = &ad9368_ops;
	strncpy(&phy_dev->name[0], DEV_NAME, sizeof(phy_dev->name));
	phy_dev->phy_id = (u32) np;

	phy_info = (struct ad_dev_info *) phy_dev->priv;
	phy_info->ad_spi = spi;

	size = sizeof(ad9368_spi_id)/sizeof(ad9368_spi_id[0]);
	while (ret < size) {
		if (!strncmp(spi->modalias, ad9368_spi_id[ret].name,
			strlen(phy_info->ad_spi->modalias))) {
				phy_info->device_id = ret;
				break;
		}
	ret++;
	}
	ret = 0;

	spi->bits_per_word = 8;

	phy_info->reset_gpio = GPIO_INVAL;

	if (ad_init_gpio(np, dev, phy_info))
		goto out;

	ad_reset_phy(phy_info, dev);
	dev_set_drvdata(&spi->dev, phy_dev);
	dev_set_drvdata(dev, phy_dev);

	ret = check_revision(phy_dev);

	if (ret)
		goto out;

	if (drv_priv->minor > drv_priv->minor_max) {
		dev_err(dev, "AD9368:abort probe, devices more than max [%d]\n",
				drv_priv->minor_max);
		goto out;
	}

	phy_dev->dev_t = MKDEV(MAJOR(drv_priv->dev_t), drv_priv->minor);
	drv_priv->minor++;
	cdev_init(&phy_dev->cdev, &ad9368_fops);
	ret = cdev_add(&phy_dev->cdev, phy_dev->dev_t, 1);
	if (ret) {
		dev_err(dev, "AD9368: Failed to add cdev\n");
		goto out;
	}

	if (phy_info->device_id == DEVICE_ID_AD93681)
		phy_dev->dev = device_create(drv_priv->class, &spi->dev,
			phy_dev->dev_t, phy_dev, "rf_dac%d",
				spi->chip_select);
	else if (phy_info->device_id == DEVICE_ID_AD93682)
		phy_dev->dev = device_create(drv_priv->class, &spi->dev,
			phy_dev->dev_t, phy_dev, "rf_adc%d",
				spi->chip_select);
	else if (phy_info->device_id == DEVICE_ID_AD9525)
		phy_dev->dev = device_create(drv_priv->class, &spi->dev,
			phy_dev->dev_t, phy_dev, "clk_synthizer_%d",
				spi->chip_select);
	ret = IS_ERR(phy_dev->dev) ? PTR_ERR(phy_dev->dev) : 0;

	if (ret == 0) {
		dev_info(dev, "ad9368/ad9525 probe passed\n");
		list_add(&phy_dev->list, &drv_priv->dev_list);
	}
	return ret;
out:
	if (phy_info->reset_gpio != GPIO_INVAL)
		gpio_free(phy_info->reset_gpio);

	kfree(phy_dev);
	return ret;
}

static struct of_device_id ad9368_match[] = {
	{.compatible = "adi,ad9368-1",},
/*	{.compatible = "adi,ad9368-2",},*/
	{.compatible = "adi,ad9525",},
	{},
};


static struct spi_driver ad9368_driver = {
	.driver = {
		.name = "ad9368_driver",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ad9368_match),
		},
	.probe = ad9368_probe,
	.remove = ad9368_remove,
};

static int __init ad9368_init(void)
{
	int ret = 0;

	ret = spi_register_driver(&ad9368_driver);
	if (ret) {
		pr_err("ad9368: spi_register_driver failed with err %x\n",
			ret);
		return ret;
	}

	drv_priv = kzalloc(sizeof(struct ad9368_drv_priv), GFP_KERNEL);
	if (!drv_priv) {
		pr_err("ad9368: Failed to allocate drv_priv\n");
		ret = -ENOMEM;
		goto out;
	}

	INIT_LIST_HEAD(&drv_priv->dev_list);

	drv_priv->class = class_create(THIS_MODULE, "AD9368_RFICs");
	if (IS_ERR(drv_priv->class)) {
		pr_err("%s: Unable to create AD9368_RFICs class\n", DEV_NAME);
		ret = PTR_ERR(drv_priv->class);
		goto out;
	}

	ret = alloc_chrdev_region(&drv_priv->dev_t, 0, MAX_RFICS, DEV_NAME);
	if (ret) {
		pr_err("%s: Failed to allocate chrdev region\n", DEV_NAME);
		goto out;
	}

	drv_priv->minor = MINOR(drv_priv->dev_t);
	drv_priv->minor_max = drv_priv->minor + MAX_RFICS - 1;

	return ret;

out:
	spi_unregister_driver(&ad9368_driver);
	class_destroy(drv_priv->class);
	kfree(drv_priv);

	return ret;
}

static void __exit ad9368_exit(void)
{
	spi_unregister_driver(&ad9368_driver);
	unregister_chrdev_region(drv_priv->dev_t, MAX_RFICS);
	class_destroy(drv_priv->class);
	kfree(drv_priv);
}

module_init(ad9368_init);
module_exit(ad9368_exit);

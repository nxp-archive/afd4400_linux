/*
 * File: roc_card.c
 *
 * ROC-1 is RF Reciever, ROC-2 is RF Transciever from Analog
 * devices. These are used for 3G/4G macro BTS, Active antenna,
 * and small cell applications.
 * This driver supports both Ad9368-1, ROC-2 and AD9525 RFICs by
 * creating a single device and maintaining a handle for the
 * underlying devices.
 *
 * This driver is derived work from drivers/rf/phy/ad9361.c
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
#include <linux/platform_device.h>
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
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/ad9368.h>
#include <linux/gpio.h>

#define DEV_NAME "ROC"

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static struct of_device_id roc_match[] = {
	{.compatible = "adi,roc-card",},
	{.compatible = "adi,dfe-roc-card",},
};

struct roc_drv_priv {
	bool reset_status;
	struct class *class;
	unsigned int minor;
	unsigned int minor_max;
	dev_t dev_t;
	struct list_head dev_list;
};

static struct roc_drv_priv *drv_priv;

MODULE_LICENSE("GPL v2");

static int roc_run_cmds(struct roc_dev *roc_dev,
		struct rf_phy_cmd *cmds,
		int count);
static int roc_start(struct roc_dev *roc_dev);
static int rf_init(struct roc_dev *roc_dev, struct rf_init_params *params);
static int roc_stop(struct roc_dev *roc_dev);
static int roc_open(struct inode *inode, struct file *filep);
static int roc_release(struct inode *inode, struct file *filep);
static long roc_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg);
static int roc_read(struct roc_dev *roc_dev, u32 start, u32 count,
		u32 *buff);
static int roc_write(struct roc_dev *roc_dev, u32 reg,
		u32 data, u8 probe);
static int rocdev_message(struct roc_dev *roc_dev,
	struct spi_ioc_transfer *u_xfers, unsigned n_xfers);

static struct rf_phy_ops roc_ops = {
	.init = rf_init,
	.run_cmds = roc_run_cmds,
	.read_regs = roc_read,
	.write_reg = roc_write,
	.start = roc_start,
	.stop = roc_stop,
	.spi_ioc_transfer = rocdev_message,
};

static const struct file_operations roc_fops = {
	.owner		= THIS_MODULE,
	.open		= roc_open,
	.release	= roc_release,
	.unlocked_ioctl	= roc_ioctl,
};

static int roc_open(struct inode *inode, struct file *filep)
{
	int minor, match = 0, index = 0;
	struct roc_dev *roc_dev = NULL;

	minor = iminor(inode);
	list_for_each_entry(roc_dev, &drv_priv->dev_list, list) {
		if (MINOR(roc_dev->dev_t) == minor) {
			match = 1;
			break;
		}
	}

	if (!match) {
		return -ENODEV;
	} else {
		filep->private_data = roc_dev;
		for (index = 0; index < 3; index++) {
			roc_dev->phy_dev[index] = get_attached_phy_dev
				(roc_dev->rf_dev_node[index]);
			if ((roc_dev->phy_dev[index] == NULL) &&
				(roc_dev->device_flag == DFE_DEV_ID))
				return -ENODEV;
		}
		atomic_inc(&roc_dev->ref);
	}
	return 0;
}

int roc_release(struct inode *inode, struct file *filep)
{
	struct roc_dev *roc_dev;
	roc_dev = filep->private_data;
	atomic_dec(&roc_dev->ref);

	return 0;
}


static int rocdev_message(struct roc_dev *roc_dev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = roc_dev->phy_dev[roc_dev->device_id];
	if (phy_dev != NULL)
		rfdev_message(phy_dev, u_xfers, n_xfers);
	return 0;
}


static long roc_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	int err = 0;
	struct roc_dev *roc_dev;
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
	int device_id;
	int rc = -ENOSYS, size;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	roc_dev = filep->private_data;
	if (roc_dev->device_id == DEVICE_ID_AD93681)
		phy_dev = roc_dev->phy_dev[DEVICE_ID_AD93681];
	else if (roc_dev->device_id == DEVICE_ID_AD93682)
		phy_dev = roc_dev->phy_dev[DEVICE_ID_AD93682];
	else if (roc_dev->device_id == DEVICE_ID_AD9525)
		phy_dev = roc_dev->phy_dev[DEVICE_ID_AD9525];

	switch (cmd) {

	case RF_DEV_INIT:

		if (copy_from_user(&init_params,
			(struct rf_init_params *)arg, sizeof(init_params))) {
			rc = -EFAULT;
			goto out;
		}

		if (roc_dev)
			rc = roc_dev->ops->init(roc_dev, &init_params);

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
					rc = roc_dev->ops->run_cmds(roc_dev,
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
			if (roc_dev->ops->en_dis_tx) {
				rc = roc_dev->ops->en_dis_tx(roc_dev,
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
			rc = roc_dev->ops->read_regs(roc_dev, reg_buf.addr,
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
			rc = roc_dev->ops->write_reg(roc_dev,
						write_reg_buf.addr,
						write_reg_buf.data, 0);
		} else {
			rc = -EFAULT;
		}

		break;

	case RF_START:
		rc = roc_dev->ops->start(roc_dev);
		break;

	case RF_STOP:
		rc = roc_dev->ops->stop(roc_dev);
		break;

	case RF_SET_TX_ATTEN:
		if (!copy_from_user(&tx_buf, (struct rf_tx_buf *)arg,
				sizeof(struct rf_tx_buf))) {
			if (roc_dev->ops->set_tx_atten) {
				rc = roc_dev->ops->set_tx_atten(roc_dev,
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

			if (roc_dev->ops->get_rx_gain)
				rc = roc_dev->ops->get_rx_gain(roc_dev,
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

			if (roc_dev->ops->set_rx_gain)
				rc = roc_dev->ops->set_rx_gain(roc_dev,
						&rx_gain);
		} else {
			rc = -EFAULT;
		}
		break;

	case RF_SET_GAIN_CTRL_MODE:

		if (!copy_from_user(&gain_ctrl, (struct rf_gain_ctrl *)arg,
			sizeof(struct rf_gain_ctrl))) {

			if (roc_dev->ops->set_gain_ctrl_mode)
				rc = roc_dev->ops->set_gain_ctrl_mode(roc_dev,
						&gain_ctrl);
		} else {
			rc = -EFAULT;
		}
		break;
	case RF_SET_SPI_STREAM_ID:
		if (!copy_from_user(&device_id, (int *)arg,
			sizeof(int))) {
			roc_dev->device_id = device_id;
			if (roc_dev->device_id >= 0 && roc_dev->device_id < 3) {
				phy_dev = roc_dev->phy_dev[roc_dev->device_id];
				rc = 0;
			} else
				rc = -EFAULT;
		} else {
			rc = -EFAULT;
		}
		break;
	case RF_SPI_IOC_TRANSFER:
		if (!copy_from_user(&ioc_transfer,
			(struct spi_ioc_transfer *)arg,
			sizeof(struct spi_ioc_transfer))) {
			if (roc_dev->ops->spi_ioc_transfer)
				rc = roc_dev->ops->spi_ioc_transfer(roc_dev,
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
	struct roc_dev *roc_dev;

	roc_dev = filep->private_data;
	atomic_dec(&roc_dev->ref);

	return 0;
}


int roc_run_cmds(struct roc_dev *roc_dev,
		struct rf_phy_cmd *cmds, int count)
{
	struct rf_phy_dev *phy_dev;
	struct device *dev = roc_dev->dev;
	int i, elapsed_time;
	u32 val, rc = 0;
	phy_dev = roc_dev->phy_dev[roc_dev->device_id];
	for (i = 0; i < count; i++) {
		if (phy_dev == NULL && cmds[i].cmd != SPI_SET_CHANNEL)
			continue;
		switch (cmds[i].cmd) {
		case SPI_WRITE:
			dev_dbg(dev, "SPI_WRITE addr:%x data:%x\n",
				cmds[i].param1, cmds[i].param2);
			/*if (cmds[i].param1 == REG_RX_CP_CONFIG) {
				roc_read(phy_dev,
					REG_RX_CAL_STATUS, 1, &val);
				if (val & MASK_RX_CP_CAL_VALID)
					break;
			} else if (cmds[i].param1 == REG_TX_CP_CONFIG) {
				roc_read(phy_dev,
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
			dev_dbg(dev, "Read from reg: %x, val %x\n",
				cmds[i].param1, val);
			break;

		case SPI_WAIT:
			dev_dbg(dev, "SPI_WAIT %u\n", cmds[i].param1);
			msleep_interruptible(cmds[i].param3);
				break;

		case SPI_WAIT_CALPLL_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALPLL_LOCK,
						MASK_CALPLL_LOCK, 1))
					break;
			}
			break;

		case SPI_WAIT_CLKPLLCP_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CLKPLLCP, MASK_CLKPLLCP, 1))
					break;
			}
			break;

		case SPI_WAIT_TXFILTER_CAL:
			dev_dbg(dev, "SPI_WAIT_TXFILTER_CAL\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_TX_BB_TUNE, 0)) {
					rc = 0;
					break;
				} else
					rc = 1;
			}
			break;

		case SPI_WAIT_RFDC_CAL:
			dev_dbg(dev, "SPI_WAIT_RFDC_CALL\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_DC_CAL_RFSTART, 0)) {
					rc = 0;
					break;
				} else
					rc = 1;
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
		/*
		*Kernel handler for WAIT_CALDONE MAILBOX command
		*If cmds[i].param1 != 0 then it's a single mailbox command check
		*cmds[i].param1 is the register address
		*cmds[i].param2 is the mask
		*They are calculated in the user space
		*else it's a mask command checking
		*all four registers starting from 0x41C
		*/
		case SPI_WAIT_MAILBOX:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (cmds[i].param1) {
					if (check_cal_done(phy_dev,
							cmds[i].param1,
							cmds[i].param2, 0))
						break;
				} else {
					if (check_cal_done_4regs(phy_dev,
							REG_ARMSTATUS,
							cmds[i].param2))
						break;
				}
			}
			break;

		case SPI_WAIT_ARMBUSY:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_ARMBUSY, MASK_ARMBUSY, 0))
					break;
			}
			break;

		case SPI_WAIT_CALDONE:
			dev_dbg(dev, "Waiting for unknown CALDONE.\n");
			msleep_interruptible(cmds[i].param3);
			break;
		case SPI_SET_CHANNEL:
			/*roc_dev->device_id = cmds[i].param3;
			phy_dev = roc_dev->phy_dev[roc_dev->device_id];*/
			dev_dbg(dev, "SPI_SET_CHANNEL RECEIVED %u.\n",
			 cmds[i].param3);
			break;
		case SPI_WAIT_CLKPLL_CAL:
			dev_dbg(dev, "SPI_WAIT_CLKPLL_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CLKPLL_LOCK,
						MASK_CLKPLL_SET, 1)) {
					rc = 0;
					break;
				} else
					rc = 1;
			}
			break;
		case SPI_WAIT_INITARM_CAL:
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_INITARM, MASK_INITARM, 1))
					break;
			}
			break;

		case SPI_WAIT_RFPLLLOCK_CAL:
			dev_dbg(dev, "SPI_WAIT_RFPLLLOCK_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < 11;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CP_OVERRANGE_VCO_LOCK,
						MASK_RFPLLLOCK_SET, 1)) {
					rc = 0;
					break;
				} else
					rc = 1;
			}
			break;
		case SPI_WAIT_RFPLLCP_CAL:
			dev_dbg(dev, "SPI_WAIT_RFPLLCP_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_RF_CP_CONFIG,
						MASK_RFPLLCP_CAL, 1)) {
					dev_dbg(dev, "RFPLLCP CAL DONE.\n");
					rc = 0;
					break;
				} else
					rc = 1;
			}
			break;
		case SPI_WAIT_ADCTUNER_CAL:
			dev_dbg(dev, "SPI_WAIT_ADCTUNER_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_ADC_TUNE_CAL_START, 0)) {
					rc = 0;
					break;
				} else
					rc = 1;
			}
			break;
		case SPI_WAIT_RXTIA_CAL:
			dev_dbg(dev, "SPI_WAIT_RXTIA_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_RXTIA_CAL_START, 0)) {
					rc = 0;
					break;
				} else
					rc = 1;
			}
			break;
		case SPI_WAIT_RCAL_CAL:
			dev_dbg(dev, "SPI_WAIT_RCAL_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CALIBRATION_CONTROL,
						MASK_RCAL_START, 0)) {
					rc = 0;
					break;
				} else
					rc = 1;
			}
			break;
		default:
			dev_dbg(dev, "Not a valid AD_PHY command\n");
			return -EINVAL;
		}
	}

	return rc;
}

int rf_init(struct roc_dev *roc_dev,
		struct rf_init_params *params)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = roc_dev->phy_dev[roc_dev->device_id];
	if (phy_dev != NULL)
		ad_init(phy_dev, params);
	return 0;
}

int roc_stop(struct roc_dev *roc_dev)
{

	return 0;
}

int roc_start(struct roc_dev *roc_dev)
{
	int rc = 0;
	struct rf_phy_dev *phy_dev;

	phy_dev = roc_dev->phy_dev[roc_dev->device_id];

	gpio_set_value(roc_dev->gpio_rx_enable, 1);
	gpio_set_value(roc_dev->gpio_tx_enable, 1);
	gpio_set_value(roc_dev->gpio_srx_enable, 1);
	rc = ad9368_start(phy_dev);
	if (rc) {
		dev_err(roc_dev->dev, "Failed to start ad9368, err %d\n", rc);
		goto out;
	}

	return rc;
out:
	gpio_set_value(roc_dev->gpio_rx_enable, 0);
	gpio_set_value(roc_dev->gpio_tx_enable, 0);
	gpio_set_value(roc_dev->gpio_srx_enable, 0);
	return rc;
}

int roc_read(struct roc_dev *roc_dev, u32 start, u32 count, u32 *buff)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = roc_dev->phy_dev[roc_dev->device_id];
	if (phy_dev != NULL)
		ad9368_read(phy_dev, start, count, buff);
	return 0;
}

int roc_write(struct roc_dev *roc_dev, u32 reg, u32 data, u8 probe)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = roc_dev->phy_dev[roc_dev->device_id];
	if (phy_dev != NULL)
		ad9368_write(phy_dev, reg, data, probe);
	return 0;
}

static int roc_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct roc_dev *roc_dev = dev_get_drvdata(&pdev->dev);

	gpio_free(roc_dev->gpio_rx_enable);
	gpio_free(roc_dev->gpio_tx_enable);
	gpio_free(roc_dev->gpio_srx_enable);
	dev_dbg(dev, "ROC PHY module uninstalled\n");

	cdev_del(&roc_dev->cdev);
	device_destroy(drv_priv->class, roc_dev->dev_t);
	kfree(roc_dev);

	return ret;
}


struct roc_dev *get_attached_roc_dev(struct device_node **dev_node)
{
	struct roc_dev *roc_dev = NULL;
	struct rf_phy_dev *phy_dev = NULL;
	int i;

	list_for_each_entry(roc_dev, &drv_priv->dev_list, list) {
		for (i = 0; i < roc_dev->phy_devs; i++) {
			phy_dev = roc_dev->phy_dev[i];
			if (*dev_node == phy_dev->rf_dev_node)
				break;
		}
	}

return roc_dev;
}
EXPORT_SYMBOL(get_attached_roc_dev);

static int roc_setup_gpio(struct roc_dev *roc_dev, int gpio)
{
	int rc = 0;

	dev_info(roc_dev->dev, "Setting up gpio %d\n", gpio);

	rc = gpio_request(gpio, dev_name(roc_dev->dev));
	if (rc) {
		dev_dbg(roc_dev->dev, "gpio_request failed, gpio [%d]", gpio);
		goto out;
	}

	rc = gpio_direction_output(gpio, 0);
	if (rc) {
		dev_dbg(roc_dev->dev,
			"gpio_direction_output failed, gpio [%d]", gpio);
		goto out;
	}

out:
	return rc;
}

static int roc_probe(struct platform_device *pdev)
{
	struct roc_dev *roc_dev;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct rf_phy_dev	*phy_dev;
	struct device_node *rf_dev_node;
	int ret = 0, size;
	const struct of_device_id *id;

	if (!np || !of_device_is_available(np)) {
		dev_err(dev, "No ROC device available\n");
		return -ENODEV;
	}
	size = sizeof(struct roc_dev);
	roc_dev = kzalloc(size, GFP_KERNEL);
	if (!roc_dev) {
		dev_dbg(dev, "Failed to allocate %d bytes roc_dev\n", size);
		return -ENOMEM;
	}
	roc_dev->dev = dev;
	roc_dev->irq_gen1 = irq_of_parse_and_map(np, 0);
/*	if (roc_register_irq(roc_dev) < 0) {
		dev_dbg(dev, "roc dev irq init failure\n");
		goto out;
	}*/
	roc_dev->ops = &roc_ops;
	id = of_match_node(roc_match, np);

	if (id) {
		if (!strcmp(id->compatible, "fsl,roc-card"))
			roc_dev->device_flag = MEDUSA_DEV_ID;
		else
			roc_dev->device_flag = DFE_DEV_ID;
	}

	if (drv_priv->minor > drv_priv->minor_max) {
		dev_dbg(dev, "ROC:abort probe, devices more than max [%d]\n",
				drv_priv->minor_max);
		goto out;
	}

	roc_dev->dev_t = MKDEV(MAJOR(drv_priv->dev_t), drv_priv->minor);
	drv_priv->minor++;
	cdev_init(&roc_dev->cdev, &roc_fops);
	ret = cdev_add(&roc_dev->cdev, roc_dev->dev_t, 1);
	if (ret) {
		dev_err(dev, "ROC: Failed to add cdev\n");
		goto out;
	}


	roc_dev->gpio_rx_enable = of_get_named_gpio(np, "cs-gpios", 0);
	roc_dev->gpio_tx_enable = of_get_named_gpio(np, "cs-gpios", 1);
	roc_dev->gpio_srx_enable = of_get_named_gpio(np, "cs-gpios", 2);

	ret = roc_setup_gpio(roc_dev, roc_dev->gpio_rx_enable);
	if (ret) {
		dev_err(dev, "Failed to setup rx_enable gpio\n");
		goto out;
	}


	ret = roc_setup_gpio(roc_dev, roc_dev->gpio_tx_enable);
	if (ret) {
		dev_err(dev, "Failed to setup tx_enable gpio\n");
		goto out;
	}

	ret = roc_setup_gpio(roc_dev, roc_dev->gpio_srx_enable);
	if (ret) {
		dev_err(dev, "Failed to setup srx_enable gpio\n");
		goto out;
	}

	rf_dev_node = of_parse_phandle(np, "9368-1-handle", 0);
	roc_dev->rf_dev_node[roc_dev->phy_devs] = rf_dev_node;

	phy_dev = get_attached_phy_dev(rf_dev_node);
	roc_dev->phy_dev[roc_dev->phy_devs] = phy_dev;

	roc_dev->phy_devs++;

	rf_dev_node = of_parse_phandle(np, "9525-handle", 0);
	roc_dev->rf_dev_node[roc_dev->phy_devs] = rf_dev_node;
	phy_dev = get_attached_phy_dev(rf_dev_node);
	roc_dev->phy_dev[roc_dev->phy_devs] = phy_dev;

	roc_dev->phy_devs++;

	rf_dev_node = of_parse_phandle(np, "9368-2-handle", 0);
	roc_dev->rf_dev_node[roc_dev->phy_devs] = rf_dev_node;
	phy_dev = get_attached_phy_dev(rf_dev_node);
	roc_dev->phy_dev[roc_dev->phy_devs] = phy_dev;

	roc_dev->phy_devs++;

	roc_dev->dev = device_create(drv_priv->class, &pdev->dev,
			roc_dev->dev_t, roc_dev, "roc_dev%d", drv_priv->minor);
	ret = IS_ERR(roc_dev->dev) ? PTR_ERR(roc_dev->dev) : 0;

	if (ret == 0) {
		dev_info(dev, "roc probe passed\n");
		list_add(&roc_dev->list, &drv_priv->dev_list);
		dev_set_drvdata(dev, roc_dev);
	}
	return ret;
out:
	kfree(roc_dev->phy_dev);
	kfree(roc_dev);
	gpio_free(roc_dev->gpio_tx_enable);
	gpio_free(roc_dev->gpio_srx_enable);
	gpio_free(roc_dev->gpio_rx_enable);
	return ret;
}


static struct platform_driver roc_driver = {
	.driver = {
		.name = "roc_driver",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(roc_match),
		},
	.probe = roc_probe,
	.remove = roc_remove,
};

static int __init roc_init(void)
{
	int ret = 0;

	drv_priv = kzalloc(sizeof(struct roc_drv_priv), GFP_KERNEL);
	if (!drv_priv) {
		pr_err("roc: Failed to allocate drv_priv\n");
		ret = -ENOMEM;
		goto out;
	}
	INIT_LIST_HEAD(&drv_priv->dev_list);

	drv_priv->class = class_create(THIS_MODULE, "ROC_RFICs");
	if (IS_ERR(drv_priv->class)) {
		pr_err("%s: Unable to create ROC_RFICs class\n", DEV_NAME);
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

	ret = platform_driver_register(&roc_driver);
	return ret;

out:
	platform_driver_unregister(&roc_driver);
	class_destroy(drv_priv->class);
	kfree(drv_priv);

	return ret;
}

static void __exit roc_exit(void)
{
	platform_driver_unregister(&roc_driver);
	unregister_chrdev_region(drv_priv->dev_t, MAX_RFICS);
	class_destroy(drv_priv->class);
	kfree(drv_priv);
}

module_init(roc_init);
module_exit(roc_exit);

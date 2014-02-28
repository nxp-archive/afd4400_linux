/*
 * File: xcvr_card.c
 *
 * XCVR-1 is RF Reciever, XCVR-2 is RF Transciever from Analog
 * devices. These are used for 3G/4G macro BTS, Active antenna,
 * and small cell applications.
 * This driver supports both Ad9368-1, XCVR-2 and AD9525 RFICs by
 * creating a single device and maintaining a handle for the
 * underlying devices.
 *
 * This driver is derived work from drivers/rf/phy/ad9361.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
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
#include <linux/qixis.h>
#include <mach/src.h>

#define DEV_NAME "XCVR"

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static struct of_device_id xcvr_match[] = {
	{.compatible = "adi,xcvr-card",},
	{.compatible = "adi,dfe-xcvr-card",},
};

struct xcvr_drv_priv {
	bool reset_status;
	struct class *class;
	dev_t dev_t;
	struct list_head dev_list;
};

static struct xcvr_drv_priv *drv_priv;

MODULE_LICENSE("GPL v2");

static int xcvr_run_cmds(struct xcvr_dev *xcvr_dev,
		struct rf_phy_cmd *cmds,
		int count);
static int xcvr_start(struct xcvr_dev *xcvr_dev);
static int rf_init(struct xcvr_dev *xcvr_dev, struct rf_init_params *params);
static int xcvr_stop(struct xcvr_dev *xcvr_dev);
static int xcvr_open(struct inode *inode, struct file *filep);
static int xcvr_release(struct inode *inode, struct file *filep);
static long xcvr_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg);
static int xcvr_read(struct xcvr_dev *xcvr_dev, u32 start, u32 count,
		u32 *buff);
static int xcvr_write(struct xcvr_dev *xcvr_dev, u32 reg,
		u32 data, u8 probe);
static int xcvrdev_message(struct xcvr_dev *xcvr_dev,
	struct spi_ioc_transfer *u_xfers, unsigned n_xfers);

static struct rf_phy_ops xcvr_ops = {
	.init = rf_init,
	.run_cmds = xcvr_run_cmds,
	.read_regs = xcvr_read,
	.write_reg = xcvr_write,
	.start = xcvr_start,
	.stop = xcvr_stop,
	.spi_ioc_transfer = xcvrdev_message,
};

static const struct file_operations xcvr_fops = {
	.owner		= THIS_MODULE,
	.open		= xcvr_open,
	.release	= xcvr_release,
	.unlocked_ioctl	= xcvr_ioctl,
};

static int xcvr_open(struct inode *inode, struct file *filep)
{
	int minor, match = 0, index = 0;
	struct xcvr_dev *xcvr_dev = NULL;

	minor = iminor(inode);
	list_for_each_entry(xcvr_dev, &drv_priv->dev_list, list) {
		if (MINOR(xcvr_dev->dev_t) == minor) {
			match = 1;
			break;
		}
	}

	if (!match) {
		return -ENODEV;
	} else {
		filep->private_data = xcvr_dev;
		for (index = 0; index < 3; index++) {
			xcvr_dev->phy_dev[index] = get_attached_phy_dev
				(xcvr_dev->rf_dev_node[index]);
			if ((xcvr_dev->phy_dev[index] == NULL) &&
				(xcvr_dev->device_flag == DFE_DEV_ID))
				return -ENODEV;
		}
		atomic_inc(&xcvr_dev->ref);
	}
	return 0;
}

int xcvr_release(struct inode *inode, struct file *filep)
{
	struct xcvr_dev *xcvr_dev;
	xcvr_dev = filep->private_data;
	atomic_dec(&xcvr_dev->ref);

	return 0;
}


static int xcvrdev_message(struct xcvr_dev *xcvr_dev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];
	if (phy_dev != NULL)
		rfdev_message(phy_dev, u_xfers, n_xfers);
	return 0;
}

int rf_assert_reset(struct xcvr_dev *xcvr_dev, int dev_id)
{
	int rc = 0, reset;

	if (dev_id == DEVICE_ID_AD93682) {
		reset = xcvr_dev->src_tx_reset;
	} else if (dev_id == DEVICE_ID_AD93681) {
		reset = xcvr_dev->src_rx_reset;
	} else {
		dev_err(xcvr_dev->dev, "Invalid dev_id %d\n",
			dev_id);
		rc = -EINVAL;
		goto out;
	}

	rc = src_assert_reset(xcvr_dev->src_handle, SW_RST, reset);
	if (rc) {
		dev_err(xcvr_dev->dev, "Reset Failed, dev_id %d, err %d\n",
			dev_id, rc);
	} else {
		dev_info(xcvr_dev->dev, "Reset done, dev_id %d\n", dev_id);
	}

out:
	return rc;
}

static long xcvr_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	int err = 0;
	struct xcvr_dev *xcvr_dev;
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

	xcvr_dev = filep->private_data;
	if (xcvr_dev->device_id == DEVICE_ID_AD93681)
		phy_dev = xcvr_dev->phy_dev[DEVICE_ID_AD93681];
	else if (xcvr_dev->device_id == DEVICE_ID_AD93682)
		phy_dev = xcvr_dev->phy_dev[DEVICE_ID_AD93682];
	else if (xcvr_dev->device_id == DEVICE_ID_AD9525)
		phy_dev = xcvr_dev->phy_dev[DEVICE_ID_AD9525];

	switch (cmd) {

	case RF_DEV_INIT:

		if (copy_from_user(&init_params,
			(struct rf_init_params *)arg, sizeof(init_params))) {
			rc = -EFAULT;
			goto out;
		}

		if (xcvr_dev)
			rc = xcvr_dev->ops->init(xcvr_dev, &init_params);

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
					rc = xcvr_dev->ops->run_cmds(xcvr_dev,
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
			if (xcvr_dev->ops->en_dis_tx) {
				rc = xcvr_dev->ops->en_dis_tx(xcvr_dev,
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
			rc = xcvr_dev->ops->read_regs(xcvr_dev, reg_buf.addr,
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
			rc = xcvr_dev->ops->write_reg(xcvr_dev,
						write_reg_buf.addr,
						write_reg_buf.data, 0);
		} else {
			rc = -EFAULT;
		}

		break;

	case RF_START:
		rc = xcvr_dev->ops->start(xcvr_dev);
		break;

	case RF_STOP:
		rc = xcvr_dev->ops->stop(xcvr_dev);
		break;

	case RF_SET_TX_ATTEN:
		if (!copy_from_user(&tx_buf, (struct rf_tx_buf *)arg,
				sizeof(struct rf_tx_buf))) {
			if (xcvr_dev->ops->set_tx_atten) {
				rc = xcvr_dev->ops->set_tx_atten(xcvr_dev,
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

			if (xcvr_dev->ops->get_rx_gain)
				rc = xcvr_dev->ops->get_rx_gain(xcvr_dev,
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

			if (xcvr_dev->ops->set_rx_gain)
				rc = xcvr_dev->ops->set_rx_gain(xcvr_dev,
						&rx_gain);
		} else {
			rc = -EFAULT;
		}
		break;

	case RF_SET_GAIN_CTRL_MODE:

		if (!copy_from_user(&gain_ctrl, (struct rf_gain_ctrl *)arg,
			sizeof(struct rf_gain_ctrl))) {

			if (xcvr_dev->ops->set_gain_ctrl_mode)
				rc = xcvr_dev->ops->set_gain_ctrl_mode(xcvr_dev,
						&gain_ctrl);
		} else {
			rc = -EFAULT;
		}
		break;
	case RF_SET_SPI_STREAM_ID:
		if (!copy_from_user(&device_id, (int *)arg,
			sizeof(int))) {
			xcvr_dev->device_id = device_id;
			if (xcvr_dev->device_id >= 0 && xcvr_dev->device_id < 3) {
				phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];
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
			if (xcvr_dev->ops->spi_ioc_transfer)
				rc = xcvr_dev->ops->spi_ioc_transfer(xcvr_dev,
					&ioc_transfer, 1);
		}	else
			rc = -EFAULT;
		break;
	case RF_RESET:
		if (get_user(device_id, (int *)arg)) {
			rc = -EFAULT;
			break;
		}
		rc = rf_assert_reset(xcvr_dev, device_id);
		break;
	default:
		rc = -ENOSYS;
	}

out:
	return rc;
}

int rf_release(struct inode *inode, struct file *filep)
{
	struct xcvr_dev *xcvr_dev;

	xcvr_dev = filep->private_data;
	atomic_dec(&xcvr_dev->ref);

	return 0;
}


int xcvr_run_cmds(struct xcvr_dev *xcvr_dev,
		struct rf_phy_cmd *cmds, int count)
{
	struct rf_phy_dev *phy_dev;
	struct device *dev = xcvr_dev->dev;
	int i, elapsed_time;
	u32 val, rc = 0;
	phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];
	for (i = 0; i < count; i++) {
		if (phy_dev == NULL && cmds[i].cmd != SPI_SET_CHANNEL)
			continue;
		elapsed_time = 0;
		switch (cmds[i].cmd) {
		case SPI_WRITE:
			dev_dbg(dev, "SPI_WRITE %03X, %02X\n",
				cmds[i].param1, cmds[i].param2);
			/*if (cmds[i].param1 == REG_RX_CP_CONFIG) {
				xcvr_read(phy_dev,
					REG_RX_CAL_STATUS, 1, &val);
				if (val & MASK_RX_CP_CAL_VALID)
					break;
			} else if (cmds[i].param1 == REG_TX_CP_CONFIG) {
				xcvr_read(phy_dev,
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
			ad9368_read(phy_dev, cmds[i].param1, 1, &val);
			dev_dbg(dev, "SPIRead %03X, %02X\n",
				cmds[i].param1, val);
			break;

		case SPI_WAIT:
			dev_dbg(dev, "SPI_WAIT %u RECEIVED\n", cmds[i].param1);
			msleep_interruptible(cmds[i].param3);
				break;

		case SPI_WAIT_CALPLL_CAL:
			dev_dbg(dev, "SPI_WAIT_CALPLL_CAL RECEIVED.\n");
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
			dev_dbg(dev, "SPI_WAIT_CLKPLLCP_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_CLKPLLCP, MASK_CLKPLLCP, 1))
					break;
			}
			break;

		case SPI_WAIT_TXFILTER_CAL:
			dev_dbg(dev, "SPI_WAIT_TXFILTER_CAL RECEIVED\n");
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
			dev_dbg(dev, "SPI_WAIT_RFDC_CAL RECEIVED\n");
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
			dev_dbg(dev, "SPI_WAIT_TXQUAD_CAL RECEIVED.\n");
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
			dev_dbg(dev, "SPI_WAIT_MAILBOX RECEIVED.\n");
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
			dev_dbg(dev, "SPI_WAIT_ARMBUSY RECEIVED.\n");
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
			/*xcvr_dev->device_id = cmds[i].param3;
			phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];*/
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
			dev_dbg(dev, "SPI_WAIT_INITARM_CAL RECEIVED.\n");
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
		case SPI_WAIT_RXADC_CAL:
			dev_dbg(dev, "SPI_WAIT_RXADC_CAL RECEIVED.\n");
			for (elapsed_time = 1; elapsed_time < cmds[i].param3;
					elapsed_time++) {
				msleep_interruptible(1);
				if (check_cal_done(phy_dev,
						REG_RXADC, MASK_RXADC, 0))
					break;
			}
			break;
		default:
			dev_err(dev, "Unknown AD_PHY command %d\n",
					cmds[i].cmd);
			return -EINVAL;
		}
		if (elapsed_time != 0) {
			if (elapsed_time >= cmds[i].param3)
				dev_warn(dev, "TIMEOUT on cmd %d. Limit was %d\n",
					cmds[i].cmd, cmds[i].param3);
			else
				dev_dbg(dev, "WAIT on cmd %d. Used %d, Limit was %d\n",
					cmds[i].cmd, elapsed_time,
					cmds[i].param3);
		}

	}

	return rc;
}

int rf_init(struct xcvr_dev *xcvr_dev,
		struct rf_init_params *params)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];
	if (phy_dev != NULL)
		ad_init(phy_dev, params);
	return 0;
}

int xcvr_stop(struct xcvr_dev *xcvr_dev)
{
	struct rf_phy_dev *phy_dev;

	phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];

	gpio_set_value(xcvr_dev->gpio_rx_enable, 0);
	gpio_set_value(xcvr_dev->gpio_tx_enable, 0);
	gpio_set_value(xcvr_dev->gpio_srx_enable, 0);
	return 0;
}

int xcvr_start(struct xcvr_dev *xcvr_dev)
{
	int rc = 0;
	struct rf_phy_dev *phy_dev;

	phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];

	gpio_set_value(xcvr_dev->gpio_rx_enable, 1);
	gpio_set_value(xcvr_dev->gpio_tx_enable, 1);
	gpio_set_value(xcvr_dev->gpio_srx_enable, 1);
	rc = ad9368_start(phy_dev);
	if (rc) {
		dev_err(xcvr_dev->dev, "Failed to start ad9368, err %d\n", rc);
		goto out;
	}

	return rc;
out:
	gpio_set_value(xcvr_dev->gpio_rx_enable, 0);
	gpio_set_value(xcvr_dev->gpio_tx_enable, 0);
	gpio_set_value(xcvr_dev->gpio_srx_enable, 0);
	return rc;
}

int xcvr_read(struct xcvr_dev *xcvr_dev, u32 start, u32 count, u32 *buff)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];
	if (phy_dev != NULL)
		ad9368_read(phy_dev, start, count, buff);
	return 0;
}

int xcvr_write(struct xcvr_dev *xcvr_dev, u32 reg, u32 data, u8 probe)
{
	struct rf_phy_dev *phy_dev;
	phy_dev = xcvr_dev->phy_dev[xcvr_dev->device_id];
	if (phy_dev != NULL)
		ad9368_write(phy_dev, reg, data, probe);
	return 0;
}

static int xcvr_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct xcvr_dev *xcvr_dev = dev_get_drvdata(&pdev->dev);

	cancel_work_sync(&xcvr_dev->err_task);
	gpio_free(xcvr_dev->gpio_rx_enable);
	gpio_free(xcvr_dev->gpio_tx_enable);
	gpio_free(xcvr_dev->gpio_srx_enable);
	dev_dbg(dev, "XCVR PHY module uninstalled\n");

	cdev_del(&xcvr_dev->cdev);
	device_destroy(drv_priv->class, xcvr_dev->dev_t);
	kfree(xcvr_dev);

	return ret;
}


struct xcvr_dev *get_attached_xcvr_dev(struct device_node **dev_node,
	struct rf_phy_dev *phy_dev)
{
	struct xcvr_dev *xcvr_dev = NULL;
	struct device_node *rf_dev_node;
	int i, found_node = 0;

	if (list_empty(&drv_priv->dev_list))
		return NULL;

	list_for_each_entry(xcvr_dev, &drv_priv->dev_list, list) {
		for (i = 0; i < xcvr_dev->phy_devs; i++) {
			rf_dev_node = xcvr_dev->rf_dev_node[i];
			if (*dev_node == rf_dev_node) {
				found_node = 1;
				xcvr_dev->phy_dev[i] = phy_dev;
				break;
			}
		}
	}
	if (found_node == 1)
		return xcvr_dev;
	else
		return NULL;
}
EXPORT_SYMBOL(get_attached_xcvr_dev);

static int xcvr_setup_gpio(struct xcvr_dev *xcvr_dev, int gpio)
{
	int rc = 0;

	dev_info(xcvr_dev->dev, "Setting up gpio %d\n", gpio);

	rc = gpio_request(gpio, dev_name(xcvr_dev->dev));
	if (rc) {
		dev_dbg(xcvr_dev->dev, "gpio_request failed, gpio [%d]", gpio);
		goto out;
	}

	rc = gpio_direction_output(gpio, 0);
	if (rc) {
		dev_dbg(xcvr_dev->dev,
			"gpio_direction_output failed, gpio [%d]", gpio);
		goto out;
	}

out:
	return rc;
}

/* Stats update during error events */
static void do_err_stats_update(struct xcvr_dev *xcvrdev,
			unsigned int mask)
{
	struct rf_phy_dev *phy_dev;
	struct device *dev = xcvrdev->dev;
	u32 val;
	phy_dev = xcvrdev->phy_dev[xcvrdev->device_id];
	if (mask & BAD_DIS) {
		ad9368_write(phy_dev, REG_SUB_JESD_ADDR, REG_BAD_DIS_JESD, 0);
		ad9368_read(phy_dev, REG_SUB_JESD_DATA, 1, &val);
		if (val & 1)
			xcvrdev->stats[0].bad_disparity_err_count++;
		if (val & 2)
			xcvrdev->stats[1].bad_disparity_err_count++;
		val |= CLEAR_ERROR_IRQ | RESET_ERROR_COUNTER;
		/* Reset the BAD CS IRQ */
		ad9368_write(phy_dev, REG_SUB_JESD_DATA, val, 0);
		ad9368_write(phy_dev, REG_WRITE_EN_JESD, 1, 0);
	}
	if (mask & NOT_IN_TABLE) {
		ad9368_write(phy_dev, REG_SUB_JESD_ADDR,
			REG_NOT_IN_TABLE_JESD, 0);
		ad9368_read(phy_dev, REG_SUB_JESD_DATA, 1, &val);
		if (val & 1)
			xcvrdev->stats[0].not_in_table_err_count++;
		if (val & 2)
			xcvrdev->stats[1].not_in_table_err_count++;
		val |= CLEAR_ERROR_IRQ | RESET_ERROR_COUNTER;
		/* Reset the BAD CS IRQ */
		ad9368_write(phy_dev, REG_SUB_JESD_DATA, val, 0);
		ad9368_write(phy_dev, REG_WRITE_EN_JESD, 1, 0);
	}
	if (mask & UNEXP_K_CHARS) {
		ad9368_write(phy_dev, REG_SUB_JESD_ADDR,
			REG_UNEXP_K_CHARS_JESD, 0);
		ad9368_read(phy_dev, REG_SUB_JESD_DATA, 1, &val);
		if (val & 1)
			xcvrdev->stats[0].unexpected_k_chars_err_count++;
		if (val & 2)
			xcvrdev->stats[1].unexpected_k_chars_err_count++;
		val |= CLEAR_ERROR_IRQ | RESET_ERROR_COUNTER;
		/* Reset the BAD CS IRQ */
		ad9368_write(phy_dev, REG_SUB_JESD_DATA, val, 0);
		ad9368_write(phy_dev, REG_WRITE_EN_JESD, 1, 0);
	}
	if (mask & BAD_CS) {
		ad9368_write(phy_dev, REG_SUB_JESD_ADDR, REG_BAD_CS_JESD, 0);
		ad9368_read(phy_dev, REG_SUB_JESD_DATA, 1, &val);
		if (val & 1)
			xcvrdev->stats[0].bad_checksum_err_count++;
		if (val & 2)
			xcvrdev->stats[1].bad_checksum_err_count++;
		val |= CLEAR_ERROR_IRQ;
		/* Reset the BAD CS IRQ */
		ad9368_write(phy_dev, REG_SUB_JESD_DATA, val, 0);
		ad9368_write(phy_dev, REG_WRITE_EN_JESD, 1, 0);
	} else
		dev_err(dev, "Invalid mask during stats update\n");
}


void xcvr_jesdtx_error_monitor(struct work_struct *work)
{
	struct rf_phy_dev *phy_dev;
	struct xcvr_dev *xcvrdev;
	u32 err_status;
	u32 temp;

	xcvrdev = container_of(work, struct xcvr_dev, err_task);
	if ((phy_dev = xcvrdev->phy_dev[DEVICE_ID_AD93682]) == NULL)
		return;

	ad9368_write(phy_dev, REG_SUB_JESD_ADDR, REG_INT_EN_JESD, 0);
	ad9368_read(phy_dev, REG_SUB_JESD_DATA, 1, &err_status);
	if (!(err_status & JESD_ERR_EVT_ALL)) {
		dev_err(xcvrdev->dev,
			"Spurious error interrupt:%d\n",
				err_status);
		return;
	}
	xcvrdev->err_status = err_status;
	temp = err_status;
	temp &= ~JESD_ERR_EVT_ALL;
	/* Disable the error interrupt mask */
	ad9368_write(phy_dev, REG_SUB_JESD_DATA, temp, 0);
	ad9368_write(phy_dev, REG_WRITE_EN_JESD, 1, 0);
	/* Update stats */
	do_err_stats_update(xcvrdev, xcvrdev->err_status);

	temp = xcvrdev->err_status;
	temp &= JESD_ERR_EVT_ALL;
	/* Restore the error interrupt mask */
	ad9368_write(phy_dev, REG_SUB_JESD_DATA, temp, 0);
	ad9368_write(phy_dev, REG_WRITE_EN_JESD, 1, 0);
	enable_irq(xcvrdev->irq_tx);
}


static irqreturn_t xcvr_jesdtx_isr(int irq, void *dev_id)
{
	struct rf_phy_dev *phy_dev;
	struct xcvr_dev *xcvrdev = dev_id;
	phy_dev = xcvrdev->phy_dev[DEVICE_ID_AD93682];
	dev_dbg(xcvrdev->dev, "JESD Tx interrupt received");
	if (phy_dev == NULL)
		return IRQ_NONE;

	disable_irq_nosync(xcvrdev->irq_tx);
	schedule_work(&xcvrdev->err_task);
	return IRQ_HANDLED;
}

static irqreturn_t xcvr_jesdrx_isr(int irq, void *dev_id)
{
	struct rf_phy_dev *phy_dev;
	struct xcvr_dev *xcvrdev = dev_id;
	phy_dev = xcvrdev->phy_dev[DEVICE_ID_AD93681];
	dev_info(xcvrdev->dev, "JESD Rx interrupt received");
	if (phy_dev == NULL)
		return IRQ_NONE;

	disable_irq_nosync(xcvrdev->irq_rx);
	schedule_work(&xcvrdev->err_task);
	return IRQ_HANDLED;
}

static int xcvr_probe(struct platform_device *pdev)
{

	struct xcvr_dev *xcvr_dev;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct rf_phy_dev	*phy_dev;
	struct device_node *rf_dev_node;
	int ret = 0, size, irq, xcvr_id;
	const struct of_device_id *id;
	struct of_phandle_args src_phandle;
	int resets[2], i, xcvr_present;

	if (!np || !of_device_is_available(np)) {
		dev_err(dev, "No XCVR device available\n");
		return -ENODEV;
	}

	of_property_read_u32(np, "xcvr_id", &xcvr_id);
	if (xcvr_id < 0) {
		dev_err(dev, "xcvr_id property not found\n");
		return -EINVAL;
	}

	xcvr_present = qixis_get_xcvr_present_status(xcvr_id);
	dev_dbg(dev, "xcvr%d present %d\n", xcvr_id, xcvr_present);

	if (xcvr_present == -EPROBE_DEFER) {
		return xcvr_present;
	} else if (!xcvr_present) {
		dev_err(dev, "xcvr%d not connected\n", xcvr_id);
		return -ENODEV;
	}

	size = sizeof(struct xcvr_dev);
	xcvr_dev = kzalloc(size, GFP_KERNEL);
	if (!xcvr_dev) {
		dev_dbg(dev, "Failed to allocate %d bytes xcvr_dev\n", size);
		return -ENOMEM;
	}
	xcvr_dev->dev = dev;
	rf_dev_node = of_parse_phandle(np, "9368-1-handle", 0);
	xcvr_dev->rf_dev_node[xcvr_dev->phy_devs] = rf_dev_node;

	phy_dev = get_attached_phy_dev(rf_dev_node);
	xcvr_dev->phy_dev[xcvr_dev->phy_devs] = phy_dev;

	xcvr_dev->phy_devs++;

	rf_dev_node = of_parse_phandle(np, "9525-handle", 0);
	xcvr_dev->rf_dev_node[xcvr_dev->phy_devs] = rf_dev_node;
	phy_dev = get_attached_phy_dev(rf_dev_node);
	xcvr_dev->phy_dev[xcvr_dev->phy_devs] = phy_dev;

	xcvr_dev->phy_devs++;

	rf_dev_node = of_parse_phandle(np, "9368-2-handle", 0);
	xcvr_dev->rf_dev_node[xcvr_dev->phy_devs] = rf_dev_node;
	phy_dev = get_attached_phy_dev(rf_dev_node);
	xcvr_dev->phy_dev[xcvr_dev->phy_devs] = phy_dev;

	xcvr_dev->phy_devs++;

	for (i = 0; i < 2; i++) {
		if (of_get_named_src_reset(np, &src_phandle,
			"src-resets", i)) {
			dev_err(xcvr_dev->dev, "Failed to get SRC reset %d\n",
				i);
			ret = -ENODEV;
			goto out;
		}
		resets[i] = src_phandle.args[0];
	}

	xcvr_dev->src_tx_reset = resets[0];
	xcvr_dev->src_rx_reset = resets[1];
	xcvr_dev->src_handle = src_get_handle(src_phandle.np);
	if (!xcvr_dev->src_handle) {
		dev_err(xcvr_dev->dev, "SRC not yet probed, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto out;
	}

	xcvr_dev->ops = &xcvr_ops;
	id = of_match_node(xcvr_match, np);

	if (id) {
		if (!strcmp(id->compatible, "fsl,xcvr-card"))
			xcvr_dev->device_flag = MEDUSA_DEV_ID;
		else
			xcvr_dev->device_flag = DFE_DEV_ID;
	}

	xcvr_dev->gpio_rx_enable = of_get_named_gpio(np, "cs-gpios", 0);
	xcvr_dev->gpio_tx_enable = of_get_named_gpio(np, "cs-gpios", 1);
	xcvr_dev->gpio_srx_enable = of_get_named_gpio(np, "cs-gpios", 2);

	/*Setting up GPIOs*/
	ret = xcvr_setup_gpio(xcvr_dev, xcvr_dev->gpio_rx_enable);
	if (ret) {
		dev_err(dev, "Failed to setup rx_enable gpio\n");
		goto out;
	}

	ret = xcvr_setup_gpio(xcvr_dev, xcvr_dev->gpio_tx_enable);
	if (ret) {
		dev_err(dev, "Failed to setup tx_enable gpio\n");
		goto out;
	}

	ret = xcvr_setup_gpio(xcvr_dev, xcvr_dev->gpio_srx_enable);
	if (ret) {
		dev_err(dev, "Failed to setup srx_enable gpio\n");
		goto out;
	}


	xcvr_dev->dev_t = MKDEV(MAJOR(drv_priv->dev_t), xcvr_id);
	cdev_init(&xcvr_dev->cdev, &xcvr_fops);
	ret = cdev_add(&xcvr_dev->cdev, xcvr_dev->dev_t, 1);
	if (ret) {
		dev_err(dev, "XCVR: Failed to add cdev\n");
		goto out;
	}

	xcvr_dev->dev = device_create(drv_priv->class, &pdev->dev,
			xcvr_dev->dev_t, xcvr_dev, "xcvr%d", xcvr_id);
	ret = IS_ERR(xcvr_dev->dev) ? PTR_ERR(xcvr_dev->dev) : 0;

	INIT_WORK(&xcvr_dev->err_task, xcvr_jesdtx_error_monitor);

	irq = platform_get_irq(pdev, 0);
	xcvr_dev->irq_tx = irq;
	if (irq < 0) {
		dev_err(&pdev->dev, "can't get irq number\n");
		return -ENOENT;
	}
	/* Request IRQ */
	ret = devm_request_irq(&pdev->dev, irq, xcvr_jesdtx_isr, 0,
			pdev->name, xcvr_dev);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n", irq);
		return ret;
	}
	dev_info(dev, "IRQ Tx Read:%d", irq);

	irq = platform_get_irq(pdev, 1);
	xcvr_dev->irq_rx = irq;
	if (irq < 0) {
		dev_err(&pdev->dev, "can't get irq number\n");
		return -ENOENT;
	}
	/* Request IRQ */
	ret = devm_request_irq(&pdev->dev, irq, xcvr_jesdrx_isr, 0,
			pdev->name, xcvr_dev);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n", irq);
		return ret;
	}
	dev_info(dev, "IRQ Rx Read:%d", irq);
	if (ret == 0) {
		list_add(&xcvr_dev->list, &drv_priv->dev_list);
		dev_set_drvdata(dev, xcvr_dev);
		dev_info(dev, "xcvr probe passed\n");
	}
	return ret;
out:
	gpio_free(xcvr_dev->gpio_tx_enable);
	gpio_free(xcvr_dev->gpio_srx_enable);
	gpio_free(xcvr_dev->gpio_rx_enable);
	kfree(xcvr_dev->phy_dev);
	kfree(xcvr_dev);
	return ret;
}


static struct platform_driver xcvr_driver = {
	.driver = {
		.name = "xcvr_driver",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xcvr_match),
		},
	.probe = xcvr_probe,
	.remove = xcvr_remove,
};

static int __init xcvr_init(void)
{
	int ret = 0;

	drv_priv = kzalloc(sizeof(struct xcvr_drv_priv), GFP_KERNEL);
	if (!drv_priv) {
		pr_err("xcvr: Failed to allocate drv_priv\n");
		ret = -ENOMEM;
		goto out;
	}
	INIT_LIST_HEAD(&drv_priv->dev_list);

	drv_priv->class = class_create(THIS_MODULE, "XCVR_RFICs");
	if (IS_ERR(drv_priv->class)) {
		pr_err("%s: Unable to create XCVR_RFICs class\n", DEV_NAME);
		ret = PTR_ERR(drv_priv->class);
		goto out;
	}

	ret = alloc_chrdev_region(&drv_priv->dev_t, 0, MAX_RFICS, DEV_NAME);
	if (ret) {
		pr_err("%s: Failed to allocate chrdev region\n", DEV_NAME);
		goto out;
	}


	ret = platform_driver_register(&xcvr_driver);
	return ret;

out:
	platform_driver_unregister(&xcvr_driver);
	class_destroy(drv_priv->class);
	kfree(drv_priv);

	return ret;
}

static void __exit xcvr_exit(void)
{
	platform_driver_unregister(&xcvr_driver);
	unregister_chrdev_region(drv_priv->dev_t, MAX_RFICS);
	class_destroy(drv_priv->class);
	kfree(drv_priv);
}

module_init(xcvr_init);
module_exit(xcvr_exit);

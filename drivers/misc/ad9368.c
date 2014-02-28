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

static LIST_HEAD(phy_dev_list);

MODULE_LICENSE("GPL v2");

static const struct rf_device_id ad9368_spi_id[] = {
	{ "ad9368-1", DEVICE_ID_AD93681 },
	{ "ad9525", DEVICE_ID_AD9525 },
	{ "ad9368-2", DEVICE_ID_AD93682}
};

struct rf_phy_dev *get_attached_phy_dev(struct device_node *rf_dev_node)
{
	struct rf_phy_dev *phy_dev = NULL;
	int found_node = 0;
	if (list_empty(&phy_dev_list))
		return NULL;

	list_for_each_entry(phy_dev, &phy_dev_list, list) {
		if (rf_dev_node == phy_dev->rf_dev_node) {
			found_node = 1;
			break;
		}
	}
	if (found_node == 1)
		return phy_dev;
	else
		return NULL;
}
EXPORT_SYMBOL(get_attached_phy_dev);

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

int rfdev_message(struct rf_phy_dev *rf_dev,
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


int check_cal_done(struct rf_phy_dev *phy_dev, u32 reg, u32 mask,
		u32 bitval)
{
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev;
	u32 val;
	int rc = 0;
	if (phy_info == NULL)
		return -EINVAL;
	dev = &phy_info->ad_spi->dev;
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
EXPORT_SYMBOL(check_cal_done);

/*
*This function checks 4 registers starting from address reg.
*This function is currently made for
*command like WAIT_CALDONE MAILBOX,55555555,2000.
*/

int check_cal_done_4regs(struct rf_phy_dev *phy_dev, u32 reg, u32 mask)
{
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev;
	u32 val[4];
	u32 temp;

	int rc = 0;
	if (phy_info == NULL)
		return -EINVAL;
	dev = &phy_info->ad_spi->dev;
	ad9368_read(phy_dev, reg, 4, val);

	temp = (val[0] & mask) || (val[1] & (mask >> 8))
			|| (val[2] & (mask >> 16)) || (val[3] & (mask >> 24));
	if (!temp)
		rc = 1;

	return rc;
}
EXPORT_SYMBOL(check_cal_done_4regs);

int ad_init(struct rf_phy_dev *phy_dev,
		struct rf_init_params *params)
{
	struct ad_dev_info *phy_info = phy_dev->priv;
	int rc = 0;

	if (phy_info == NULL)
		return -EINVAL;
	phy_info->prev_ensm_state = ENSM_STATE_INVALID;

	return rc;
}
EXPORT_SYMBOL(ad_init);

int ad9368_stop(struct rf_phy_dev *phy_dev)
{

	return 0;
}
EXPORT_SYMBOL(ad9368_stop);

int ad9368_start(struct rf_phy_dev *phy_dev)
{

	/*XXX: Nothing to do as of now*/
	return 0;
}
EXPORT_SYMBOL(ad9368_start);

int ad9368_read(struct rf_phy_dev *phy_dev, u32 start,
		u32 count, u32 *buff)
{
	u32 data;
	u16 read_addr;
	int trans_size;
	int i = 0, j = 0, rc;
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev;
	uint8_t tx[2];
	uint8_t rx[3] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = COMMAND_LEN + count,
		.delay_usecs = 0,
		.speed_hz = phy_info->ad_spi->max_speed_hz,
		.bits_per_word = 8,
	};
	/* RFdev f/w provides start address as u32, but
	 * ADI has only 12 bits regs, so downsize it to 16 bits
	 */
	if (phy_info == NULL)
		return -EINVAL;
	dev = &phy_info->ad_spi->dev;
	read_addr =  (u16)(start + count - 1);
	j = count - 1;

	if (phy_info->device_id == DEVICE_ID_AD9525)
		trans_size = MAX_CLK_READ_TRANS_SIZE;
	else
		trans_size = MAX_READ_TRANSACTION_SIZE;

	while (count > trans_size) {
		if (phy_info->device_id == DEVICE_ID_AD9525) {
			tx[0] = OPCODE_READ_AD9525
				|(((trans_size - 1)
				<< SHIFT_CLK_BYTES_TRANS)
				& (BYTES_CLK_TRANS_MASK));
			tr.len = RXBUF_CLK_SIZE;
		} else {
			tx[0] = OPCODE_READ
				|(((trans_size - 1)
					<< SHIFT_BYTES_TRANSFER)
					& (BYTES_TRANSFER_MASK));
			tr.len = RXBUF_SIZE;
		}

		tx[0] |=	(read_addr>>8);
		tx[1] = (u8)read_addr;
		rc = rfdev_message(phy_dev, &tr, 1);
		if (rc < 0)
			goto out;

		for (i = COMMAND_LEN; i < tr.len ; i++) {
			data = phy_info->rx_buf[i];
			buff[j--] = data;
		}

		count -= trans_size;
		read_addr = read_addr - trans_size;
	}
	if (phy_info->device_id == DEVICE_ID_AD9525)
		tx[0] = OPCODE_READ_AD9525
			|(((count - 1) << SHIFT_CLK_BYTES_TRANS)
				& (BYTES_CLK_TRANS_MASK));
	else
		tx[0] = OPCODE_READ
			|(((count - 1) << SHIFT_BYTES_TRANSFER)
				& (BYTES_TRANSFER_MASK));

	tx[0] |=	(read_addr>>8);
	tx[1] = (u8)read_addr;
	tr.len = COMMAND_LEN + count;
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
EXPORT_SYMBOL(ad9368_read);

int ad9368_write(struct rf_phy_dev *phy_dev, u32 reg,
		u32 data, u8 probe)
{
	int rc;
	u8 cntrwd[3];
	struct ad_dev_info *phy_info = phy_dev->priv;
	struct device *dev;
	if (phy_info == NULL)
		return -EINVAL;
	dev = &phy_info->ad_spi->dev;
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
		cntrwd[0] |= (reg>>8);
		cntrwd[1] = (u8)reg;
		cntrwd[2] = (u8)data;
		rc = spi_write_transaction(phy_dev, cntrwd, TRANSACTION_BYTES);
	}
	return rc;
}
EXPORT_SYMBOL(ad9368_write);

static int ad9368_remove(struct spi_device *spi)
{
	int ret = 0;
	struct device *dev = &spi->dev;
	struct rf_phy_dev *phy_dev = dev_get_drvdata(&spi->dev);
	struct ad_dev_info *phy_info = phy_dev->priv;
	if (phy_info == NULL)
		return -EINVAL;
	dev_dbg(dev, "AD9368 PHY module uninstalled\n");

	gpio_free(phy_info->reset_gpio);
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
		if (rev != 1) {
			dev_err(dev,
				"Unsupported AD9368-1 revision 0x%x\n", rev);
			rc = -ENODEV;
			goto out;
		}
		dev_dbg(dev, "Detected AD9368 Rev 0x%x\n", rev);
	} else if (phy_info->device_id == DEVICE_ID_AD93682) {
		ad9368_read(phy_dev, PRODUCT_CODE_AD93682_REG, 1, &val);
		rev = (u8) val & REV_MASK;
		if (rev != 1) {
			dev_err(dev,
				"Unsupported AD9368-2 revision 0x%x\n", rev);
			rc = -ENODEV;
			goto out;
		}
		dev_dbg(dev, "Detected AD9368-2 Rev 0x%x\n", rev);
	} else if (phy_info->device_id == DEVICE_ID_AD9525) {
		ad9368_write(phy_dev, PRODUCT_CODE_AD9525_REG, 0, 1);
		ad9368_read(phy_dev, PRODUCT_CODE_AD9525_REG, 1, &val);
		rev = (u8) val;
		if (rev != 1) {
			dev_err(dev,
				"Unsupported AD9525 revision 0x%x\n", rev);
			rc = -ENODEV;
			goto out;
		}
		dev_dbg(dev, "Detected AD9525 Rev 0x%x\n", rev);
	}

	/*product_id = (u8) val & PRODUCT_ID_MASK;
	if (product_id != PRODUCT_ID_9368) {
		dev_err(dev, "Unsupported product id 0x%x\n", product_id);
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
		goto out;	}

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
		dev_dbg(dev, "Control GPIOs not intialized in dtb\n");

	if ((phy_info->reset_gpio != GPIO_INVAL)) {
		/* Toggle reset gpio to reset AD9368 */
		gpio_set_value(phy_info->reset_gpio, 0);
		gpio_set_value(phy_info->reset_gpio, 1);
	}

}


static int ad9368_probe(struct spi_device *spi)
{
	static struct rf_phy_dev *phy_dev = NULL;
	struct device *dev = &spi->dev;
	struct ad_dev_info *phy_info = NULL;
	struct device_node *np = spi->dev.of_node;
	char dev_name[RF_NAME_SIZE];
	struct xcvr_dev *xcvr_dev;
	int ret = 0, size;

	memset(dev_name, 0, RF_NAME_SIZE);
	size = sizeof(struct rf_phy_dev) + sizeof(struct ad_dev_info);
	phy_dev = kzalloc(size, GFP_KERNEL);
	if (!phy_dev) {
		dev_dbg(dev, "Failed to allocate rf_phy_dev\n");
		return -ENOMEM;
	}

	xcvr_dev = get_attached_xcvr_dev(&np, phy_dev);
	dev_err(dev, "node %x, phy %x\n", np, phy_dev);
	if (!xcvr_dev) {
		dev_err(dev, "Failed to attach XCVR\n");
		ret = -EPROBE_DEFER;
		goto out;
	}

	phy_dev->priv = (void *) ((u32) phy_dev + sizeof(struct rf_phy_dev));
/*	phy_dev->ops = &ad9368_ops;*/
	strncpy(&phy_dev->name[0], DEV_NAME, sizeof(phy_dev->name));
	phy_dev->phy_id = (u32) np;
	phy_dev->rf_dev_node = np;
	phy_dev->xcvr_dev = xcvr_dev;
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

	if (ret == 0) {
		dev_info(dev, "%s probe passed\n", spi->modalias);
		list_add(&phy_dev->list, &phy_dev_list);
	}
	return ret;
out:
	if (phy_info && phy_info->reset_gpio != GPIO_INVAL)
		gpio_free(phy_info->reset_gpio);

	kfree(phy_dev);
	return ret;
}

static struct of_device_id ad9368_match[] = {
	{.compatible = "adi,ad9368-1",},
	{.compatible = "adi,ad9368-2",},
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

	return ret;

}

static void __exit ad9368_exit(void)
{
	spi_unregister_driver(&ad9368_driver);
}

module_init(ad9368_init);
module_exit(ad9368_exit);

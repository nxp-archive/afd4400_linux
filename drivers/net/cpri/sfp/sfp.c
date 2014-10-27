/*
 * drivers/net/cpri/sfp/sfp.c
 * SFP device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <trace/events/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/cpri.h>

static LIST_HEAD(sfp_dev_list);
raw_spinlock_t sfp_list_lock;

/* This parameter is to help this driver avoid blocking other drivers out
 * of I2C for potentially troublesome amounts of time. With a 100 kHz I2C
 * clock, one 256 byte read takes about 1/43 second which is excessive;
 * but the 1/170 second it takes at 400 kHz may be quite reasonable; and
 * at 1 MHz (Fm+) a 1/430 second delay could easily be invisible.
 *
 * This value is forced to be a power of two so that writes align on pages.
 */
static unsigned io_limit = SFP_EEPROM_INFO_SIZE;
module_param(io_limit, uint, 0);
MODULE_PARM_DESC(io_limit, "Maximum bytes per I/O (default 128)");

/* Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned write_timeout = 100;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");

static int sfp_eeprom_write(struct sfp_dev *sfp, u8 *buf,
		u8 offset, unsigned int count)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	int status;
	unsigned long timeout, write_time;
	int i = 0;

	client = sfp->client;

	/* Use the DIAG address*/
	client->addr = sfp->addr[1];
	if (count > sfp->write_max)
		count = sfp->write_max;

	/* If we'll use I2C calls for I/O, set up the message */
	if (!sfp->use_smbus) {
		msg.addr = client->addr;
		msg.flags = 0;

		/* msg.buf is u8 and casts will mask the values */
		msg.buf = sfp->writebuf;
		msg.buf[i++] = offset;
		memcpy(&msg.buf[i], buf, count);
		msg.len = i + count;
	}

	/* Writes fail if the previous one didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		write_time = jiffies;
		if (sfp->use_smbus) {
			status = i2c_smbus_write_i2c_block_data(client,
					offset, count, buf);
			if (status == 0)
				status = count;
		} else {
			status = i2c_transfer(client->adapter, &msg, 1);
			if (status == 1)
				status = count;
		}
		dev_dbg(&client->dev, "write %zu@%d --> %zd (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		usleep_range(1000, 2000);
	} while (time_before(write_time, timeout));

	return -ETIME;
}

int sfp_raw_write(struct sfp_dev *sfp,
		u8 *buf,
		u8 offset,
		unsigned int count,
		enum mem_type type)
{
	int ret = 0;

	sfp->type = type;

	if (unlikely(sfp->type != SFP_MEM_DIAG))
		return -ENODEV;

	if (unlikely(!count))
		return count;

	if (sfp->addr_cnt < 2)
		return -ENODEV;

	mutex_lock(&sfp->lock);

	while (count) {
		int status;

		status = sfp_eeprom_write(sfp, buf, offset, count);
		if (status <= 0) {
			if (ret == 0)
				ret = status;
			break;
		}

		buf += status;
		offset += status;
		count -= status;
		ret += status;
	}

	mutex_unlock(&sfp->lock);

	return ret;
}
EXPORT_SYMBOL(sfp_raw_write);

void fill_sfp_detail(struct sfp_dev *sfp, u8 *sfp_detail)
{
	memset(sfp_detail, 0, (sizeof(sfp->info.vendor_name) +
				sizeof(sfp->info.vendor_pn)));
	memcpy(sfp_detail, sfp->info.vendor_name,
			sizeof(sfp->info.vendor_name));
	memcpy(&sfp_detail[sizeof(sfp->info.vendor_name)],
			sfp->info.vendor_pn, sizeof(sfp->info.vendor_pn));

}
EXPORT_SYMBOL(fill_sfp_detail);

static int sfp_eeprom_read(struct sfp_dev *sfp, u8 *buf,
		u8 offset, unsigned int count, enum mem_type type)
{
	struct i2c_client *client = sfp->client;
	u8 msgbuf[2] = {0};
	struct i2c_msg msg[2];
	unsigned long timeout = 0, read_time = 0;
	int status = 0, i = 0;


	memset(msg, 0, sizeof(msg));
	/* Determine the memory (eeprom/diagnostics) to read */
	if (type == SFP_MEM_EEPROM)
		client->addr = sfp->addr[0];
	else if ((type == SFP_MEM_DIAG) && (sfp->addr_cnt >= 2))
		client->addr = sfp->addr[1];
	else
		return -ENODEV;

	if ((offset > io_limit) || (!client))
		return -1;

	if (count > io_limit)
		count = io_limit;

	switch (sfp->use_smbus) {
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (count > I2C_SMBUS_BLOCK_MAX)
			count = I2C_SMBUS_BLOCK_MAX;
		break;
	case I2C_SMBUS_WORD_DATA:
		count = 2;
		break;
	case I2C_SMBUS_BYTE_DATA:
		count = 1;
		break;
	default:
		i = 0;
		msgbuf[i++] = offset;

		msg[0].addr = client->addr;
		msg[0].buf = msgbuf;
		msg[0].len = i;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = count;
	}

	/* Reads fail if the previous write didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		read_time = jiffies;
		switch (sfp->use_smbus) {
		case I2C_SMBUS_I2C_BLOCK_DATA:
			status = i2c_smbus_read_i2c_block_data(client, offset,
					count, buf);
			break;
		case I2C_SMBUS_WORD_DATA:
			status = i2c_smbus_read_word_data(client, offset);
			if (status >= 0) {
				buf[0] = status & 0xff;
				buf[1] = status >> 8;
				status = count;
			}
			break;
		case I2C_SMBUS_BYTE_DATA:
			status = i2c_smbus_read_byte_data(client, offset);
			if (status >= 0) {
				buf[0] = status;
				status = count;
			}
			break;
		default:
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2)
				status = count;
		}
		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
				count, offset, status, jiffies);
		usleep_range(1000, 2000);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is slow */
	} while (time_before(read_time, timeout));

	return -ETIME;
}

int sfp_raw_read(struct sfp_dev *sfp,
		u8 *buf,
		u8 offset,
		unsigned int count,
		enum mem_type type)
{
	int ret = 0;

	sfp->type = type;

	if (unlikely(!count))
		return count;

	mutex_lock(&sfp->lock);

	while (count) {
		int status;

		status = sfp_eeprom_read(sfp, buf, offset, count, type);
		if (status <= 0) {
			if (ret == 0)
				ret = status;
			break;
		} else if (status > count)
			status = count;

		buf += status;
		offset += status;
		count -= status;
		ret += status;
	}

	mutex_unlock(&sfp->lock);

	return ret;
}
EXPORT_SYMBOL(sfp_raw_read);


static int read_sfp_info(struct sfp_dev *sfp)
{
	struct device *dev = &(sfp->client->dev);
	u8 offset = 0;
	int ret = 0;
	unsigned int count = SFP_EEPROM_INFO_SIZE;
	enum mem_type type = SFP_MEM_EEPROM;
	u8 *buf = &(sfp->info.type);

	/* Try reading basic eeprom info */
	ret = sfp_raw_read(sfp, buf, offset, count, type);
	if (ret != count) {
		dev_dbg(dev, "basic data read failure ret: %d, count: %d",
			ret, count);
		goto out;
	}

	return 0;
out:
	return -EINVAL;
}


struct sfp_dev *get_attached_sfp_dev(struct device_node *sfp_dev_node)
{
	struct sfp_dev *sfp_dev = NULL;

	if (list_empty(&sfp_dev_list))
		return NULL;

	raw_spin_lock(&sfp_list_lock);

	list_for_each_entry(sfp_dev, &sfp_dev_list, list) {
		if (sfp_dev_node == sfp_dev->dev_node)
			break;
	}

	raw_spin_unlock(&sfp_list_lock);

	return sfp_dev;
}
EXPORT_SYMBOL(get_attached_sfp_dev);

void set_sfp_txdisable(struct sfp_dev *sfp, unsigned value)
{
	gpio_set_value_cansleep(sfp->tx_disable, value);
}
EXPORT_SYMBOL(set_sfp_txdisable);


static int config_sfp_lines(struct sfp_dev *sfp)
{
	struct device *dev = &(sfp->client->dev);

	/* Get the GPIO pin numbers from device node */
	sfp->prs = of_get_named_gpio(sfp->dev_node,
			"gpio-sfp-prs", 0);
	if (sfp->prs == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!gpio_is_valid(sfp->prs))
		dev_err(dev, "sfp prs GPIO NUMBER = %d is not valid\n",
				sfp->prs);
	else {
		gpio_request(sfp->prs, "gpio-sfp-prs");
		gpio_direction_input(sfp->prs);
	}
	sfp->rxlos = of_get_named_gpio(sfp->dev_node,
			"gpio-sfp-rxlos", 0);
	if (sfp->rxlos == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!gpio_is_valid(sfp->rxlos))
		dev_err(dev, "sfp rxlos GPIO NUMBER = %d is not valid\n",
				sfp->rxlos);
	else {
		gpio_request(sfp->rxlos, "gpio-sfp-rxlos");
		gpio_direction_input(sfp->rxlos);
	}
	sfp->txfault = of_get_named_gpio(sfp->dev_node,
			"gpio-sfp-txfault", 0);
	if (sfp->txfault == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!gpio_is_valid(sfp->txfault))
		dev_err(dev, "sfp rxlos GPIO NUMBER = %d is not valid\n",
				sfp->txfault);
	else {
		gpio_request(sfp->txfault, "gpio-gpio-sfp-txfault");
		gpio_direction_input(sfp->txfault);
	}

	sfp->tx_disable = of_get_named_gpio(sfp->dev_node,
				"gpio-sfp-txdisable", 0);
	if (sfp->tx_disable == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!gpio_is_valid(sfp->tx_disable))
		dev_err(dev, "sfp tx_disable GPIO NUMBER = %d is not valid\n",
				sfp->tx_disable);
	else {
		gpio_request(sfp->tx_disable, "gpio-sfp-tx_disable");
		gpio_direction_output(sfp->tx_disable, 1);
	}
	return 0;
}

static int sfp_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sfp_dev *sfp = NULL;
	struct device_node *node = NULL;
	struct device *dev = &client->dev;
	int use_smbus = 0, err = 0;
	unsigned write_max;
	u8 sfp_detail[33] = {0};

	/* Getting the device node from platform */
	node = client->dev.of_node;
	/* Get the number of eeproms supported by the transceiver */
	sfp = kzalloc(sizeof(struct sfp_dev), GFP_KERNEL);
	if (!sfp) {
		err = -ENOMEM;
		goto err_out;
	}

	if (client->dev.of_node) {
		of_property_read_u32(client->dev.of_node, "max-addr",
				&sfp->addr_cnt);
	} else {
		err = -ENODEV;
		goto err_struct;
	}

	/* Chek for smbus support in adapter */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		if (i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
			use_smbus = I2C_SMBUS_I2C_BLOCK_DATA;
		} else if (i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_WORD_DATA)) {
			use_smbus = I2C_SMBUS_WORD_DATA;
		} else if (i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
			use_smbus = I2C_SMBUS_BYTE_DATA;
		} else {
			err = -EPFNOSUPPORT;
			goto err_struct;
		}
	}
	/* if smbus adapter not found */
	write_max = io_limit;
	/* if smbus adapter found, max bytes to write updated here */
	if (use_smbus && write_max > I2C_SMBUS_BLOCK_MAX)
		write_max = I2C_SMBUS_BLOCK_MAX;

	/* ------------ Start populating sfp_dev ---------- */
	sfp->dev_node = node;
	sfp->write_max = write_max;
	sfp->use_smbus = use_smbus;
	mutex_init(&sfp->lock);

	/* buffer (data + address at the beginning) */
	sfp->writebuf = kmalloc(write_max + 2, GFP_KERNEL);
	if (!sfp->writebuf) {
		err = -ENOMEM;
		goto err_struct;
	}

	of_property_read_u32_array(client->dev.of_node, "reg", sfp->addr,
			sfp->addr_cnt);

	/* Getting mem interface address */
	sfp->client = client;

	/* ------------ End populating sfp_dev ---------- */
	/* Configure transceiver pins */
	err = config_sfp_lines(sfp);
	if (err < 0) {
		dev_err(dev, "sfp pin configuration failed\n");
		if (err == -EPROBE_DEFER)
			goto err_clients;
	} else {
		/* gpio read is must for interrupt to unmask interrupt */
		gpio_get_value_cansleep(sfp->prs);
		gpio_get_value_cansleep(sfp->rxlos);
		gpio_get_value_cansleep(sfp->txfault);
		dev_dbg(dev, "prbs: 0x%x, rxloss: 0x%x, txfault: 0x%x",
			gpio_get_value_cansleep(sfp->prs),
			gpio_get_value_cansleep(sfp->rxlos),
			gpio_get_value_cansleep(sfp->txfault));
	}

	/* Log i2c performance info */
	if (use_smbus == I2C_SMBUS_WORD_DATA ||
		use_smbus == I2C_SMBUS_BYTE_DATA) {
		dev_notice(dev, "Falling back to %s reads, perf will suffer\n",
			use_smbus == I2C_SMBUS_WORD_DATA ? "word" : "byte");
	}

	/* Do SFP presence test */
	if (gpio_get_value_cansleep(sfp->prs))
		dev_err(dev, "sfp not inserted\n");
	else  {
		if (read_sfp_info(sfp) < 0)
			dev_err(dev, "sfp i2c read fail");
		else {
			memcpy(sfp_detail, sfp->info.vendor_name,
				sizeof(sfp->info.vendor_name));
			memcpy(&sfp_detail[sizeof(sfp->info.vendor_name)],
				sfp->info.vendor_pn,
				sizeof(sfp->info.vendor_pn));
			dev_info(dev, "SFP detected: %s\n", sfp_detail);
		}
	}

	raw_spin_lock(&sfp_list_lock);
	list_add_tail(&sfp->list, &sfp_dev_list);
	raw_spin_unlock(&sfp_list_lock);

	/* Set the client data for this i2c transceiver device */
	i2c_set_clientdata(client, sfp);

	return 0;

err_clients:
	i2c_unregister_device(sfp->client);
err_struct:
	kfree(sfp);
err_out:
	dev_err(dev, "probe error %d", err);
	return err;
}

static int sfp_remove(struct i2c_client *client)
{
	struct sfp_dev *sfp, *sfpdev;
	struct list_head *pos, *nx;

	sfp = i2c_get_clientdata(client);
	kfree(sfp->writebuf);
	gpio_free(sfp->txfault);
	gpio_free(sfp->rxlos);
	gpio_free(sfp->prs);
	gpio_free(sfp->tx_disable);

	/* Deleting the sfp device from list */
	raw_spin_lock(&sfp_list_lock);
	list_for_each_safe(pos, nx, &sfp_dev_list) {
		sfpdev = list_entry(pos, struct sfp_dev, list);
		if (sfpdev == sfp) {
			list_del(&sfpdev->list);
			kfree(sfpdev);
			break;
		}
	}
	raw_spin_unlock(&sfp_list_lock);

	return 0;
}

static const struct i2c_device_id sfp_ids[] = {
	{ "sfp-dev", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sfp_ids);

static struct i2c_driver sfp_driver = {
	.driver = {
		.name = "sfp-dev",
		.owner = THIS_MODULE,
	},
	.probe = sfp_probe,
	.remove = sfp_remove,
	.id_table = sfp_ids,
};

static int __init sfp_init(void)
{
	if (!io_limit) {
		pr_err("sfp: io_limit must not be 0!\n");
		return -EINVAL;
	}

	io_limit = rounddown_pow_of_two(io_limit);

	return i2c_add_driver(&sfp_driver);
}
module_init(sfp_init);

static void __exit sfp_exit(void)
{
	struct list_head *pos, *nx;
	struct sfp_dev *sfp = NULL;

	i2c_del_driver(&sfp_driver);

	raw_spin_lock(&sfp_list_lock);
	list_for_each_safe(pos, nx, &sfp_dev_list) {
		sfp = list_entry(pos, struct sfp_dev, list);
		list_del(&sfp->list);
		kfree(sfp);
	}
	raw_spin_unlock(&sfp_list_lock);
}
module_exit(sfp_exit);

MODULE_DESCRIPTION("SFP DRIVER");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_LICENSE("GPL");

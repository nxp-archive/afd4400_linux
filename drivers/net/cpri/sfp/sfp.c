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
static unsigned write_timeout = 25;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");

static int sfp_eeprom_write(struct sfp_dev *sfp, u8 *buf,
		u8 offset, unsigned int count)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	int status;
	unsigned long timeout, write_time;
	enum mem_type type = sfp->type;

	client = ((type == SFP_MEM_EEPROM) ? sfp->client[0] : sfp->client[1]);

	if (count > sfp->write_max)
		count = sfp->write_max;

	/* If we'll use I2C calls for I/O, set up the message */
	if (!sfp->use_smbus) {
		int i = 0;

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

	return -ETIMEDOUT;
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
		return -ENOMEM;

	if (unlikely(!count))
		return count;

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

static int sfp_eeprom_read(struct sfp_dev *sfp, u8 *buf,
		u8 offset, unsigned int count)
{
	struct i2c_client *client;
	u8 msgbuf[2];
	struct i2c_msg msg[2];
	unsigned long timeout, read_time;
	enum mem_type type = sfp->type;
	int status, i;

	if (offset > io_limit)
		return -1;

	memset(msg, 0, sizeof(msg));

	/* Determine the memory (eeprom/diagnostics) to read */
	client = ((type == SFP_MEM_EEPROM) ? sfp->client[0] : sfp->client[1]);

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

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is slow */
		usleep_range(1000, 2000);
	} while (time_before(read_time, timeout));

	return -ETIMEDOUT;
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

		status = sfp_eeprom_read(sfp, buf, offset, count);
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
EXPORT_SYMBOL(sfp_raw_read);

static unsigned long long do_bdata_sanity_check(struct sfp *info)
{
	unsigned long long csum = 0;
	u8 *taddr;
	int i;

	taddr = &info->type;
	for (i = 0; i < SFP_BASIC_DATA_SIZE; i++) {
		csum += *taddr;
		taddr++;
	}
	return csum;
}

static int read_sfp_info(struct sfp_dev *sfp)
{
	struct device *dev = &(sfp->client[0]->dev);
	u8 offset = 0;
	unsigned long long csum;
	int ret = 0;
	unsigned int count = SFP_EEPROM_INFO_SIZE;
	enum mem_type type = SFP_MEM_EEPROM;
	u8 *buf = &(sfp->info.type);

	/* Try reading basic eeprom info */
	ret = sfp_raw_read(sfp, buf, offset, count, type);
	if (ret != count) {
		dev_err(dev, "basic data read failure");
		ret = -1;
	}

	/* Check the read was proper */
	csum = do_bdata_sanity_check(&sfp->info);

	if ((u8)csum != sfp->info.check_code_b) {
		dev_err(dev, "basic data verification failed");
		ret = -1;
		goto out;
	}

out:
	return ret;
}

static irqreturn_t txfault_handler(int irq, void *cookie)
{
	/* TODO: notification function in cpri to be called here */

	return IRQ_HANDLED;
}

static irqreturn_t rxlos_handler(int irq, void *cookie)
{
	/* TODO: notification function in cpri to be called here */

	return IRQ_HANDLED;
}

static irqreturn_t prs_handler(int irq, void *cookie)
{
	/* TODO: notification function in cpri to be called here.
	 * Not much info on this interrupt - currently it is handled just
	 * like other interrupts.
	 */

	return IRQ_HANDLED;
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
	gpio_set_value(sfp->tx_disable, value);
}
EXPORT_SYMBOL(set_sfp_txdisable);

static int config_gpio_out(unsigned int pin, const char *label)
{
	return gpio_request_one(pin, GPIOF_DIR_OUT, label);
}

static int config_gpio_irq(unsigned int pin, const char *label,
		irqreturn_t (*handler) (int, void*),
		struct sfp_dev *sfp)
{
	int ret, irq;

	ret = gpio_request_one(pin, GPIOF_IN, label);
	if (ret)
		goto err_request_gpio_failed;

	ret = irq = gpio_to_irq(pin);
	if (ret < 0)
		goto err_get_irq_num_failed;

	ret = request_irq(irq, handler, 0, label, sfp);
	if (ret)
		goto err_request_irq_failed;

err_request_irq_failed:
err_get_irq_num_failed:
	gpio_free(pin);
err_request_gpio_failed:
	return ret;
}

static int config_sfp_lines(struct sfp_dev *sfp)
{
	int ret = 0;

	return 0; /* need to implement with pca 9565 driver changes */
	/* Get the GPIO pin numbers from device node */
	sfp->irq_txfault = of_get_named_gpio(sfp->dev_node,
				"sfp-int-txfault", 0);
	sfp->irq_rxlos = of_get_named_gpio(sfp->dev_node,
				"sfp-int-rxlos", 0);
	sfp->irq_prs = of_get_named_gpio(sfp->dev_node,
				"sfp-int-prs", 0);
	sfp->tx_disable = of_get_named_gpio(sfp->dev_node,
				"sfp-int-disable", 0);

	/* Check enhanced options supported by transceiver
	 * and configure the pins accordingly
	 */
	if (sfp->info.options[1] & TX_FAULT_BIT3_EN) {
		ret = config_gpio_irq(sfp->irq_txfault, "sfp txfault",
				txfault_handler, sfp);
		if (ret < 0)
			goto out;
	}
	if (sfp->info.options[1] & LOS_BIT1_EN) {
		ret = config_gpio_irq(sfp->irq_rxlos, "sfp los",
				rxlos_handler, sfp);
		if (ret < 0)
			goto out;
	}

	/* TBD: Not sure whether PRS is interrupt */
	ret = config_gpio_irq(sfp->irq_prs, "sfp prs", prs_handler, sfp);
	if (ret < 0)
		goto out;

	if (sfp->info.options[1] & TX_DISABLE_BIT4_EN) {
		ret = config_gpio_out(sfp->tx_disable, "sfp txdisable");
		if (ret < 0)
			goto out;
	}

out:
	return ret;
}

static int sfp_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sfp_dev *sfp = NULL;
	struct device_node *node = NULL;
	struct device *dev = &client->dev;
	unsigned int num_addr = 0;
	int use_smbus = 0, err = 0;
	unsigned addr;
	unsigned write_max;
	int i;
	u32 prop[3] = { 0 };

	/* Getting the device node from platform */
	node = client->dev.of_node;
	/* Get the number of eeproms supported by the transceiver */
	if (client->dev.of_node) {
		of_property_read_u32(client->dev.of_node, "max-addr",
				&num_addr);
		sfp = kzalloc((sizeof(struct sfp_dev) +
				num_addr * sizeof(struct i2c_client *)),
				GFP_KERNEL);
		if (!sfp) {
			err = -ENOMEM;
			dev_err(dev, "probe error\n");
			goto err_out;
		}
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
	sfp->num_addresses = num_addr;
	sfp->use_smbus = use_smbus;
	mutex_init(&sfp->lock);

	/* buffer (data + address at the beginning) */
	sfp->writebuf = kmalloc(write_max + 2, GFP_KERNEL);
	if (!sfp->writebuf) {
		err = -ENOMEM;
		goto err_struct;
	}

	/* TBD: The 'reg' field is used by the multiplexer driver to create
	 * nodes and as well by this driver to send notification
	 * on connected framer
	 */
	of_property_read_u32_array(client->dev.of_node, "reg", prop,
			ARRAY_SIZE(prop));

	sfp->id = prop[0];

	/* Getting eeprom mem interface address */
	sfp->client[0] = client;
	sfp->client[0]->addr = prop[0];

	/* Getting diagnostic mem interface address */
	addr = prop[1];
	sfp->client[1] = i2c_new_dummy(client->adapter, addr);
	if (!sfp->client[1]) {
		dev_err(dev, "addr 0x%02x unavailable\n", client->addr + 1);
		err = -EADDRINUSE;
		goto err_clients;
	}
#if 0
	addr = prop[2];
	sfp->client[2] = i2c_new_dummy(client->adapter, addr);
	if (!sfp->client[2]) {
		dev_err(dev, "addr 0x%02x unavailable\n", addr);
		err = -EADDRINUSE;
		goto err_clients;
	}
#endif

	/* ------------ End populating sfp_dev ---------- */
	/* Configure transceiver pins */
	if (config_sfp_lines(sfp) < 0) {
		dev_err(dev, "sfp pin configuration failed\n");
		goto err_clients;
	}

	dev_notice(dev, "Tx Fault pin: %d, Rx LOS pin: %d ",
			sfp->irq_txfault, sfp->irq_rxlos);
	dev_notice(dev, "PRS pin: %d, Tx Disable pin: %d ",
			sfp->irq_prs, sfp->tx_disable);

	/* Log i2c performance info */
	if (use_smbus == I2C_SMBUS_WORD_DATA ||
		use_smbus == I2C_SMBUS_BYTE_DATA) {
		dev_notice(dev, "Falling back to %s reads, perf will suffer\n",
			use_smbus == I2C_SMBUS_WORD_DATA ? "word" : "byte");
	}

	/* Fill SFP info structure and do sanity checks */
	if (read_sfp_info(sfp) < 0) {
		dev_err(dev, "sfp read failed\n");
		goto err_clients;
	}

	/* Get the pair cpri device. If found, update the sfp device there and
	 * change its state
	 */
	sfp->pair_framer = get_attached_cpri_dev(&sfp->dev_node);

	/* Add the populated sfp_dev to global sfp_dev_list. This is required
	 * since the probe will get called for multiple sfp transceivers
	 */
	raw_spin_lock(&sfp_list_lock);
	list_add_tail(&sfp->list, &sfp_dev_list);
	raw_spin_unlock(&sfp_list_lock);

	/* Set the client data for this i2c transceiver device */
	i2c_set_clientdata(client, sfp);

	dev_info(dev, "probe successfull\n");

	return 0;

err_clients:
	for (i = 1; i <= num_addr; i++)
		if (sfp->client[i])
			i2c_unregister_device(sfp->client[i]);
err_struct:
	kfree(sfp);
err_out:
	dev_dbg(dev, "probe error %d\n", err);

	return err;
}

static int sfp_remove(struct i2c_client *client)
{
	struct sfp_dev *sfp, *sfpdev;
	struct list_head *pos, *nx;
	int i;

	sfp = i2c_get_clientdata(client);

	for (i = 1; i < sfp->num_addresses; i++)
		i2c_unregister_device(sfp->client[i]);

	kfree(sfp->writebuf);

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

	kfree(sfp);

	gpio_free(sfp->irq_txfault);
	gpio_free(sfp->irq_rxlos);
	gpio_free(sfp->irq_prs);
	gpio_free(sfp->tx_disable);

	return 0;
}

static const struct i2c_device_id sfp_ids[] = {
	{ "finisar-FTLF8526P3", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sfp_ids);

static struct i2c_driver sfp_driver = {
	.driver = {
		.name = "finisar-FTLF8526P3",
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

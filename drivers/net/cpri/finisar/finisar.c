/*
 * drivers/net/cpri/finisar/finisar.c
 * FINISAR device driver
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

static LIST_HEAD(finisar_dev_list);
raw_spinlock_t finisar_list_lock;

/* This parameter is to help this driver avoid blocking other drivers out
 * of I2C for potentially troublesome amounts of time. With a 100 kHz I2C
 * clock, one 256 byte read takes about 1/43 second which is excessive;
 * but the 1/170 second it takes at 400 kHz may be quite reasonable; and
 * at 1 MHz (Fm+) a 1/430 second delay could easily be invisible.
 *
 * This value is forced to be a power of two so that writes align on pages.
 */
static unsigned io_limit = FINISAR_EEPROM_INFO_SIZE;
module_param(io_limit, uint, 0);
MODULE_PARM_DESC(io_limit, "Maximum bytes per I/O (default 128)");

/* Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned write_timeout = 25;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");

static int finisar_eeprom_write(struct finisar_dev *finisar, u8 *buf,
		u8 offset, unsigned int count)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	int status;
	unsigned long timeout, write_time;
	enum mem_type type = finisar->type;

	client = ((type == FINISAR_MEM_EEPROM) ?
			finisar->client[0] : finisar->client[1]);

	if (count > finisar->write_max)
		count = finisar->write_max;

	/* If we'll use I2C calls for I/O, set up the message */
	if (!finisar->use_smbus) {
		int i = 0;

		msg.addr = client->addr;
		msg.flags = 0;

		/* msg.buf is u8 and casts will mask the values */
		msg.buf = finisar->writebuf;
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
		if (finisar->use_smbus) {
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

int finisar_raw_write(struct finisar_dev *finisar,
		u8 *buf,
		u8 offset,
		unsigned int count,
		enum mem_type type)
{
	int ret = 0;

	finisar->type = type;

	if (unlikely(finisar->type != FINISAR_MEM_DIAG))
		return -ENOMEM;

	if (unlikely(!count))
		return count;

	mutex_lock(&finisar->lock);

	while (count) {
		int status;

		status = finisar_eeprom_write(finisar, buf, offset, count);
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

	mutex_unlock(&finisar->lock);

	return ret;
}
EXPORT_SYMBOL(finisar_raw_write);

static int finisar_eeprom_read(struct finisar_dev *finisar, u8 *buf,
		u8 offset, unsigned int count)
{
	struct i2c_client *client;
	u8 msgbuf[2];
	struct i2c_msg msg[2];
	unsigned long timeout, read_time;
	enum mem_type type = finisar->type;
	int status, i;

	if (offset > io_limit)
		return -1;

	memset(msg, 0, sizeof(msg));

	/* Determine the memory (eeprom/diagnostics) to read */
	client = ((type == FINISAR_MEM_EEPROM) ?
			finisar->client[0] : finisar->client[1]);

	if (count > io_limit)
		count = io_limit;

	switch (finisar->use_smbus) {
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
		switch (finisar->use_smbus) {
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

int finisar_raw_read(struct finisar_dev *finisar,
		u8 *buf,
		u8 offset,
		unsigned int count,
		enum mem_type type)
{
	int ret = 0;

	finisar->type = type;

	if (unlikely(!count))
		return count;

	mutex_lock(&finisar->lock);

	while (count) {
		int status;

		status = finisar_eeprom_read(finisar, buf, offset, count);
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

	mutex_unlock(&finisar->lock);

	return ret;
}
EXPORT_SYMBOL(finisar_raw_read);

static unsigned long long do_bdata_sanity_check(struct finisar *info)
{
	unsigned long long csum = 0;
	u8 *taddr;
	int i;

	taddr = &info->type;
	for (i = 0; i < FINISAR_BASIC_DATA_SIZE; i++) {
		csum += *taddr;
		taddr++;
	}
	return csum;
}

static int read_finisar_info(struct finisar_dev *finisar)
{
	struct device *dev = &(finisar->client[0]->dev);
	u8 offset = 0;
	unsigned long long csum;
	int ret = 0;
	unsigned int count = FINISAR_EEPROM_INFO_SIZE;
	enum mem_type type = FINISAR_MEM_EEPROM;
	u8 *buf = &(finisar->info.type);

	/* Try reading basic eeprom info */
	ret = finisar_raw_read(finisar, buf, offset, count, type);
	if (ret != count) {
		dev_dbg(dev, "basic data read failure");
		ret = -1;
	}

	/* Check the read was proper */
	csum = do_bdata_sanity_check(&finisar->info);

	if ((u8)csum != finisar->info.check_code_b) {
		dev_dbg(dev, "basic data verification failed");
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

struct finisar_dev *get_attached_finisar_dev(struct device_node
						*finisar_dev_node)
{
	struct finisar_dev *finisar_dev = NULL;

	if (list_empty(&finisar_dev_list))
		return NULL;

	raw_spin_lock(&finisar_list_lock);

	list_for_each_entry(finisar_dev, &finisar_dev_list, list) {
		if (finisar_dev_node == finisar_dev->dev_node)
			break;
	}

	raw_spin_unlock(&finisar_list_lock);

	return finisar_dev;
}
EXPORT_SYMBOL(get_attached_finisar_dev);

void set_finisar_txdisable(struct finisar_dev *finisar, unsigned value)
{
	gpio_set_value(finisar->tx_disable, value);
}
EXPORT_SYMBOL(set_finisar_txdisable);

static int config_gpio_out(unsigned int pin, const char *label)
{
	return gpio_request_one(pin, GPIOF_DIR_OUT, label);
}

static int config_gpio_irq(unsigned int pin, const char *label,
		irqreturn_t (*handler) (int, void*),
		struct finisar_dev *finisar)
{
	int ret, irq;

	ret = gpio_request_one(pin, GPIOF_IN, label);
	if (ret)
		goto err_request_gpio_failed;

	ret = irq = gpio_to_irq(pin);
	if (ret < 0)
		goto err_get_irq_num_failed;

	ret = request_irq(irq, handler, 0, label, finisar);
	if (ret)
		goto err_request_irq_failed;

err_request_irq_failed:
err_get_irq_num_failed:
	gpio_free(pin);
err_request_gpio_failed:
	return ret;
}

static int config_finisar_lines(struct finisar_dev *finisar)
{
	int ret = 0;

	return 0; /* need to implement with pca 9565 driver changes */
	/* Get the GPIO pin numbers from device node */
	finisar->irq_txfault = of_get_named_gpio(finisar->dev_node,
				"finisar-int-txfault", 0);
	finisar->irq_rxlos = of_get_named_gpio(finisar->dev_node,
				"finisar-int-rxlos", 0);
	finisar->irq_prs = of_get_named_gpio(finisar->dev_node,
				"finisar-int-prs", 0);
	finisar->tx_disable = of_get_named_gpio(finisar->dev_node,
				"finisar-int-disable", 0);

	/* Check enhanced options supported by transceiver
	 * and configure the pins accordingly
	 */
	if (finisar->info.options[1] & TX_FAULT_BIT3_EN) {
		ret = config_gpio_irq(finisar->irq_txfault, "finisar txfault",
				txfault_handler, finisar);
		if (ret < 0)
			goto out;
	}
	if (finisar->info.options[1] & LOS_BIT1_EN) {
		ret = config_gpio_irq(finisar->irq_rxlos, "finisar los",
				rxlos_handler, finisar);
		if (ret < 0)
			goto out;
	}

	/* TBD: Not sure whether PRS is interrupt */
	ret = config_gpio_irq(finisar->irq_prs, "finisar prs", prs_handler,
				finisar);
	if (ret < 0)
		goto out;

	if (finisar->info.options[1] & TX_DISABLE_BIT4_EN) {
		ret = config_gpio_out(finisar->tx_disable, "finisar txdisable");
		if (ret < 0)
			goto out;
	}

out:
	return ret;
}

static int finisar_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct finisar_dev *finisar = NULL;
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
		finisar = kzalloc((sizeof(struct finisar_dev) +
				num_addr * sizeof(struct i2c_client *)),
				GFP_KERNEL);
		if (!finisar) {
			err = -ENOMEM;
			dev_dbg(dev, "probe error\n");
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

	/* ------------ Start populating finisar_dev ---------- */
	finisar->dev_node = node;
	finisar->write_max = write_max;
	finisar->num_addresses = num_addr;
	finisar->use_smbus = use_smbus;
	mutex_init(&finisar->lock);

	/* buffer (data + address at the beginning) */
	finisar->writebuf = kmalloc(write_max + 2, GFP_KERNEL);
	if (!finisar->writebuf) {
		err = -ENOMEM;
		goto err_struct;
	}

	/* TBD: The 'reg' field is used by the multiplexer driver to create
	 * nodes and as well by this driver to send notification
	 * on connected framer
	 */
	of_property_read_u32_array(client->dev.of_node, "reg", prop,
			ARRAY_SIZE(prop));

	finisar->id = prop[0];

	/* Getting eeprom mem interface address */
	finisar->client[0] = client;
	finisar->client[0]->addr = prop[0];

	/* Getting diagnostic mem interface address */
	addr = prop[1];
	finisar->client[1] = i2c_new_dummy(client->adapter, addr);
	if (!finisar->client[1]) {
		dev_dbg(dev, "addr 0x%02x unavailable\n", client->addr + 1);
		err = -EADDRINUSE;
		goto err_clients;
	}
#if 0
	addr = prop[2];
	finisar->client[2] = i2c_new_dummy(client->adapter, addr);
	if (!finisar->client[2]) {
		dev_dbg(dev, "addr 0x%02x unavailable\n", addr);
		err = -EADDRINUSE;
		goto err_clients;
	}
#endif

	/* ------------ End populating finisar_dev ---------- */
	/* Configure transceiver pins */
	if (config_finisar_lines(finisar) < 0) {
		dev_dbg(dev, "finisar pin configuration failed\n");
		goto err_clients;
	}

	dev_dbg(dev, "Tx Fault pin: %d, Rx LOS pin: %d ",
			finisar->irq_txfault, finisar->irq_rxlos);
	dev_dbg(dev, "PRS pin: %d, Tx Disable pin: %d ",
			finisar->irq_prs, finisar->tx_disable);

	/* Log i2c performance info */
	if (use_smbus == I2C_SMBUS_WORD_DATA ||
		use_smbus == I2C_SMBUS_BYTE_DATA) {
		dev_dbg(dev, "Falling back to %s reads, perf will suffer\n",
			use_smbus == I2C_SMBUS_WORD_DATA ? "word" : "byte");
	}

	/* Fill FINISAR info structure and do sanity checks */
	if (read_finisar_info(finisar) < 0) {
		dev_dbg(dev, "finisar read failed\n");
		goto err_clients;
	}

	/* Get the pair cpri device. If found, update the finisar there and
	 * change its state
	 */
	finisar->pair_framer = get_attached_cpri_dev(&finisar->dev_node);

	/* Add the finisar_dev to global finisar_dev_list. This is required
	 * since the probe will get called for multiple finisar transceivers
	 */
	raw_spin_lock(&finisar_list_lock);
	list_add_tail(&finisar->list, &finisar_dev_list);
	raw_spin_unlock(&finisar_list_lock);

	/* Set the client data for this i2c transceiver device */
	i2c_set_clientdata(client, finisar);

	dev_info(dev, "finisar dev probe successfull\n");

	return 0;

err_clients:
	for (i = 1; i <= num_addr; i++)
		if (finisar->client[i])
			i2c_unregister_device(finisar->client[i]);
err_struct:
	kfree(finisar);
err_out:
	dev_dbg(dev, "probe error %d\n", err);

	return err;
}

static int finisar_remove(struct i2c_client *client)
{
	struct finisar_dev *finisar, *finisardev;
	struct list_head *pos, *nx;
	int i;

	finisar = i2c_get_clientdata(client);

	for (i = 1; i < finisar->num_addresses; i++)
		i2c_unregister_device(finisar->client[i]);

	kfree(finisar->writebuf);

	/* Deleting the finisar device from list */
	raw_spin_lock(&finisar_list_lock);
	list_for_each_safe(pos, nx, &finisar_dev_list) {
		finisardev = list_entry(pos, struct finisar_dev, list);
		if (finisardev == finisar) {
			list_del(&finisardev->list);
			kfree(finisardev);
			break;
		}
	}
	raw_spin_unlock(&finisar_list_lock);

	kfree(finisar);

	gpio_free(finisar->irq_txfault);
	gpio_free(finisar->irq_rxlos);
	gpio_free(finisar->irq_prs);
	gpio_free(finisar->tx_disable);

	return 0;
}

static const struct i2c_device_id finisar_ids[] = {
	{ "finisar-FTLF8526P3", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, finisar_ids);

static struct i2c_driver finisar_driver = {
	.driver = {
		.name = "finisar-FTLF8526P3",
		.owner = THIS_MODULE,
	},
	.probe = finisar_probe,
	.remove = finisar_remove,
	.id_table = finisar_ids,
};

static int __init finisar_init(void)
{
	if (!io_limit) {
		pr_err("finisar: io_limit must not be 0!\n");
		return -EINVAL;
	}

	io_limit = rounddown_pow_of_two(io_limit);

	return i2c_add_driver(&finisar_driver);
}
module_init(finisar_init);

static void __exit finisar_exit(void)
{
	struct list_head *pos, *nx;
	struct finisar_dev *finisar = NULL;

	i2c_del_driver(&finisar_driver);

	raw_spin_lock(&finisar_list_lock);
	list_for_each_safe(pos, nx, &finisar_dev_list) {
		finisar = list_entry(pos, struct finisar_dev, list);
		list_del(&finisar->list);
		kfree(finisar);
	}
	raw_spin_unlock(&finisar_list_lock);
}
module_exit(finisar_exit);

MODULE_DESCRIPTION("FINISAR DRIVER");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_LICENSE("GPL");

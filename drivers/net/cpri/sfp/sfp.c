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
#include <linux/fs.h>
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
#include <mach/simreset.h>

#define MAX_SFPS	4
#define SFP_DEVICE_NAME "sfp"

/* Device major/minor numbers */
static s32 sfp_major;
static s32 sfp_minor;
static struct class *sfp_class;

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

/* Debug level controls runtime messages from the SFP.
 */
static unsigned debug = 0;
module_param(debug, uint, 0);
MODULE_PARM_DESC(debug, "Debug message setting");

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

void sfp_set_attached_framer(struct sfp_dev *sfp, struct cpri_framer *framer)
{
	if (sfp)
		sfp->attached_framer = framer;
}
EXPORT_SYMBOL(sfp_set_attached_framer);

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

static int sfp_read_info(struct sfp_dev *sfp)
{
	struct device *dev = &(sfp->client->dev);
	int ret = 0;
	unsigned int count = SFP_EEPROM_INFO_SIZE;
	u8 *buf = &(sfp->info.type);
	sfp->desc[0] = 0;

	/* Try reading basic eeprom info */
	ret = sfp_raw_read(sfp, &(sfp->info.type), 0, count, SFP_MEM_EEPROM);
	if (ret != count) {
		dev_dbg(dev, "basic data read failure ret: %d, count: %d",
			ret, count);
		goto out;
	}
	/* Try reading diagnostic info */
	ret = sfp_raw_read(sfp, &(sfp->diag.temp_hi_alarm[0]), 0, count,
								SFP_MEM_DIAG);
	if (ret != count) {
		dev_dbg(dev, "diagnostics read failure ret: %d, count: %d",
			ret, count);
		goto out;
	}
	sfp->valid = 1;
	buf = sfp->desc;
	snprintf(buf, sizeof(sfp->info.vendor_name) + 1, "%s",
							sfp->info.vendor_name);
	buf = &sfp->desc[strlen(sfp->desc)];
	if (sfp->info.vendor_pn[0]) {
		snprintf(buf, sizeof(sfp->info.vendor_pn) + 2, " %s",
							sfp->info.vendor_pn);
		buf = &sfp->desc[strlen(sfp->desc)];
	}
	if (sfp->info.vendor_rev[0]) {
		snprintf(buf, sizeof(sfp->info.vendor_rev) + 6, ",rev %s",
							sfp->info.vendor_rev);
		buf = &sfp->desc[strlen(sfp->desc)];
	}
	if (sfp->info.vendor_sn[0]) {
		snprintf(buf, sizeof(sfp->info.vendor_sn) + 2, ":%s",
							sfp->info.vendor_sn);
	}
	return 0;
out:
	return -EINVAL;
}

int sfp_update_realtime_info(struct sfp_dev *sfp)
{
	struct device *dev = &(sfp->client->dev);
	int len;
	int ret;
	u8 tmp[24];

	len = sizeof(tmp);
	ret = sfp_raw_read(sfp, tmp, 96, len, SFP_MEM_DIAG);
	if (ret != len) {
		dev_err(dev, "diagnostics read failure ret: %d, count: %d",
			ret, len);
		return -EINVAL;
	}
	memcpy((u8*)&sfp->diag.temp_msb, tmp, sizeof(tmp));
	return 0;
}
EXPORT_SYMBOL(sfp_update_realtime_info);

void sfp_set_tx_enable(struct sfp_dev *sfp, unsigned value)
{
	sfp->tx_enable_state = value;
	gpio_set_value_cansleep(sfp->tx_disable, !value);
}
EXPORT_SYMBOL(sfp_set_tx_enable);

int sfp_check_gpios(struct sfp_dev *sfp)
{
	struct device *dev = &(sfp->client->dev);
	int state = 0;
	int changed = 0;
	int val;

	/* Pin is high for Loss of RX signal */
	val = gpio_get_value_cansleep(sfp->rxlos);
	if (val)
		state |= SFP_STATE_RXLOS;
	if (sfp->rxlos_state != val) {
		sfp->rxlos_state = val;
		changed |= SFP_STATE_RXLOS;
		if (val)
			sfp->rxlos_count++;
	}
	/* Pin is high for TX Fault */
	val = gpio_get_value_cansleep(sfp->txfault);
	if (val)
		state |= SFP_STATE_TXFAULT;
	if (sfp->txfault_state != val) {
		sfp->txfault_state = val;
		changed |= SFP_STATE_TXFAULT;
		if (val)
			sfp->txfault_count++;
	}
	/* Pin is low when SFP module is present */
	val = !gpio_get_value_cansleep(sfp->prs);
	if (val)
		state |= SFP_STATE_PRS;
	if (sfp->prs_state != val) {
		sfp->prs_state = val;
		changed |= SFP_STATE_PRS;
		if (val) {
			sfp->prs_count++;
			if (sfp_read_info(sfp) < 0)
				dev_err(dev, "sfp i2c read fail");
		} else {
			sfp->valid = 0;
		}
	}
	if (changed && (sfp->debug & 2))
		dev_info(dev, "sfp%d state = %d\n", sfp->id, state);
	if (changed && (sfp->debug & 1)) {
		if (changed & SFP_STATE_PRS) {
			if (!(state & SFP_STATE_PRS))
				dev_info(dev, "sfp%d module removed\n",
						sfp->id);
			else if (sfp->valid)
				dev_info(dev, "sfp%d %s\n",
						sfp->id, sfp->desc);
			else
				dev_info(dev, "sfp%d unknown module inserted\n",
						sfp->id);
		} else if (state & SFP_STATE_PRS) {
			if (changed & SFP_STATE_RXLOS)
				dev_info(dev, "sfp%d RX signal %s\n", sfp->id,
					(state & SFP_STATE_RXLOS) ? "lost" : "OK");
			if (changed & SFP_STATE_TXFAULT)
				dev_info(dev, "sfp%d TX fault %s\n", sfp->id,
					(state & SFP_STATE_TXFAULT) ? "occured" :
								     "cleared");
		}
	}
	/* Notify framer */
	if (changed && sfp->attached_framer)
		cpri_framer_handle_sfp_pin_changes(sfp->attached_framer,
								changed, state);
	return 0;
}
EXPORT_SYMBOL(sfp_check_gpios);

static int sfp_config_gpios(struct sfp_dev *sfp)
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

		/* If tx_disable is an input (1-input), then set tx_disable
		 * as output.
		 */
		if (gpio_get_direction(sfp->tx_disable) == 1) {
			/* Set as output/high to disable. */
			gpio_direction_output(sfp->tx_disable, 1);
		}
	}
	sfp->prs_state = 0;
	sfp->rxlos_state = 0;
	sfp->txfault_state = 0;
	sfp->tx_enable_state = 0;
	return 0;
}

static ssize_t show_stat(struct device *dev,
			struct device_attribute *devattr, char *buf);

static ssize_t show_info(struct device *dev,
			struct device_attribute *devattr, char *buf);

static ssize_t show_desc(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct sfp_dev *sfp = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", sfp->valid ? sfp->desc : "");
}

static ssize_t set_debug(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct sfp_dev *sfp = dev_get_drvdata(dev);
	int err;
	unsigned int val;

	err = kstrtouint(buf, 10, &val);
	if (err)
		return err;

	sfp->debug = val;
	return count;
}

static DEVICE_ATTR(prs_state,        S_IRUGO, show_stat,   NULL);
static DEVICE_ATTR(prs_count,        S_IRUGO, show_stat,   NULL);
static DEVICE_ATTR(rxlos_state,      S_IRUGO, show_stat,   NULL);
static DEVICE_ATTR(rxlos_count,      S_IRUGO, show_stat,   NULL);
static DEVICE_ATTR(txfault_state,    S_IRUGO, show_stat,   NULL);
static DEVICE_ATTR(txfault_count,    S_IRUGO, show_stat,   NULL);
static DEVICE_ATTR(txenable_state,   S_IRUGO, show_stat,   NULL);
static DEVICE_ATTR(debug,  S_IWUSR | S_IRUGO, show_stat,   set_debug);
static DEVICE_ATTR(info,             S_IRUGO, show_desc,   NULL);
static DEVICE_ATTR(eeprom,           S_IRUGO, show_info,   NULL);
static DEVICE_ATTR(diag,             S_IRUGO, show_info,   NULL);
static DEVICE_ATTR(diag_realtime,    S_IRUGO, show_info,   NULL);

static struct attribute *attributes[] = {
	&dev_attr_prs_state.attr,
	&dev_attr_prs_count.attr,
	&dev_attr_rxlos_state.attr,
	&dev_attr_rxlos_count.attr,
	&dev_attr_txfault_state.attr,
	&dev_attr_txfault_count.attr,
	&dev_attr_txenable_state.attr,
	&dev_attr_debug.attr,
	&dev_attr_info.attr,
	&dev_attr_eeprom.attr,
	&dev_attr_diag.attr,
	&dev_attr_diag_realtime.attr,
	NULL
};

static const struct attribute_group attr_group = {
	.attrs = attributes,
};

static ssize_t show_stat(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	unsigned int val;
	struct sfp_dev *sfp = dev_get_drvdata(dev);
	if      (devattr == &dev_attr_prs_state)     val = sfp->prs_state;
	else if (devattr == &dev_attr_prs_count)     val = sfp->prs_count;
	else if (devattr == &dev_attr_rxlos_state)   val = sfp->rxlos_state;
	else if (devattr == &dev_attr_rxlos_count)   val = sfp->rxlos_count;
	else if (devattr == &dev_attr_txfault_state) val = sfp->txfault_state;
	else if (devattr == &dev_attr_txfault_count) val = sfp->txfault_count;
	else if (devattr == &dev_attr_txenable_state)
						   val = sfp->tx_enable_state;
	else if (devattr == &dev_attr_debug)         val = sfp->debug;
	else val = 0;
	return sprintf(buf, "%d\n", val);
}

static ssize_t show_info(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct sfp_dev *sfp = dev_get_drvdata(dev);
	int i;
	int ret;
	int len;
	char *bp;
	u8 *sp;
	u8 tmp[24];

	if (!sfp->valid)
		goto out;

	if (devattr == &dev_attr_eeprom) {
		len = 128;
		sp = (u8*)&sfp->info;
	} else if (devattr == &dev_attr_diag) {
		len = 128;
		sp = (u8*)&sfp->diag;
	} else if (devattr == &dev_attr_diag_realtime) {
		ret = sfp_update_realtime_info(sfp);
		if (ret)
			goto out;
		len = sizeof(tmp);
		sp = (u8*)&sfp->diag.temp_msb;
	} else
		goto out;

	bp = buf;
	sprintf(bp, "%02X ", *sp++);
	for (i = 0; i < len; i++) {
		bp += 3;
		sprintf(bp, "%02X ", *sp++);
	}
	bp += 2;
	*bp++ = '\n';
	return (bp - buf);

out:
	return sprintf(buf, "\n");
}

static int sfp_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sfp_dev *sfp = NULL;
	struct device_node *node = NULL;
	struct device *dev = &client->dev;
	struct device *sysfs_dev;
	dev_t devno;
	int use_smbus = 0, err = 0;
	unsigned write_max;
	u8 device_name[10];

	/* Getting the device node from i2c client */
	node = client->dev.of_node;
	if (!node) {
		err = -ENODEV;
		goto err_out;
	}
	/* Create the device driver data structure */
	sfp = kzalloc(sizeof(struct sfp_dev), GFP_KERNEL);
	if (!sfp) {
		err = -ENOMEM;
		goto err_out;
	}

	/* Obtain the SFP id number */
	err = of_property_read_u32(node, "id", &sfp->id);
	if (err) {
		dev_err(dev, "Missing 'id' field in DTB\n");
		goto err_struct;
	}
	if (sfp->id >= MAX_SFPS) {
		err = -EINVAL;
		dev_err(dev, "SFP id %d is beyond supported max of %d\n",
			sfp->id, MAX_SFPS-1);
		goto err_struct;
	}
	/* Get the maximum I2C address supported by the client */
	err = of_property_read_u32(node, "max-addr", &sfp->addr_cnt);
	if (err) {
		dev_err(dev, "Missing 'max_addr' field in DTB\n");
		goto err_struct;
	}
	err = of_property_read_u32_array(node, "reg", sfp->addr, sfp->addr_cnt);
	if (err) {
		dev_err(dev, "Missing 'reg' field in DTB\n");
		goto err_struct;
	}
	/* Check for smbus support in adapter */
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
	sfp->debug = debug;
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

	/* Set the client data for this i2c transceiver device */
	sfp->client = client;
	i2c_set_clientdata(client, sfp);

	/* ------------ End populating sfp_dev ---------- */

	/* Configure transceiver pins */
	err = sfp_config_gpios(sfp);
	if (err < 0) {
		dev_err(dev, "sfp pin configuration failed\n");
		if (err == -EPROBE_DEFER)
			goto err_clients;
	} else {
		/* gpio read is must for interrupt to unmask interrupt */
		gpio_get_value_cansleep(sfp->prs);
		gpio_get_value_cansleep(sfp->rxlos);
		gpio_get_value_cansleep(sfp->txfault);
		dev_dbg(dev, "prs: 0x%x, rxloss: 0x%x, txfault: 0x%x",
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

	/* Personalize to the SFP id number */
	devno = MKDEV(sfp_major, sfp_minor + sfp->id);
	sprintf(device_name, SFP_DEVICE_NAME "%d", sfp->id);

	/* Create sysfs device */
	sysfs_dev = device_create(sfp_class, dev, devno, NULL, device_name);
	if (IS_ERR(sysfs_dev)) {
		err = PTR_ERR(sysfs_dev);
		dev_err(dev, "error %d while trying to create sysfs %s",
			err, device_name);
		goto device_fail;
	}

	err = sysfs_create_group(&dev->kobj, &attr_group);
	if (err < 0) {
		dev_err(dev, "error %d while trying to create group %s",
			err, SFP_DEVICE_NAME);
		goto group_fail;
	}

	/* Do SFP presence test */
	if (gpio_get_value_cansleep(sfp->prs))
		dev_err(dev, "sfp%d not inserted\n", sfp->id);
	else  {
		if (sfp_read_info(sfp) < 0)
			dev_err(dev, "sfp%d i2c read fail\n", sfp->id);
		else {
			dev_info(dev, "sfp%d %s\n", sfp->id, sfp->desc);
		}
	}

	raw_spin_lock(&sfp_list_lock);
	list_add_tail(&sfp->list, &sfp_dev_list);
	raw_spin_unlock(&sfp_list_lock);

	return 0;

group_fail:
	device_destroy(sfp_class, devno);
device_fail:
err_clients:
	i2c_unregister_device(sfp->client);
err_struct:
	if (sfp->writebuf)
		kfree(sfp->writebuf);
	kfree(sfp);
err_out:
	dev_err(dev, "probe error %d", err);
	return err;
}

static int sfp_remove(struct i2c_client *client)
{
	struct sfp_dev *sfp, *sfpdev;
	struct list_head *pos, *nx;
	struct device *dev = &client->dev;

	sfp = i2c_get_clientdata(client);
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

	sysfs_remove_group(&dev->kobj, &attr_group);
	device_destroy(sfp_class, MKDEV(sfp_major, sfp_minor + sfp->id));
	kfree(sfp->writebuf);
	kfree(sfp);

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
	int err;
	dev_t devno;

	if (!io_limit) {
		pr_err("sfp: io_limit must not be 0!\n");
		err = -EINVAL;
		goto chrdev_fail;
	}
	io_limit = rounddown_pow_of_two(io_limit);

	/* Dynamically allocate  */
	err = alloc_chrdev_region(&devno, 0, MAX_SFPS, SFP_DEVICE_NAME);
	if (err < 0) {
		pr_err("sfp: can't get major number: %d\n", err);
		goto chrdev_fail;
	}
	sfp_major = MAJOR(devno);
	sfp_minor = MINOR(devno);

	/* Create the device class */
	sfp_class = class_create(THIS_MODULE, SFP_DEVICE_NAME);
	if (IS_ERR(sfp_class)) {
		err = PTR_ERR(sfp_class);
		pr_err("sfp: class_create() failed %d\n", err);
		goto class_fail;
	}

	err = i2c_add_driver(&sfp_driver);
	if (err == 0)
		return 0;

	class_destroy(sfp_class);
class_fail:
	unregister_chrdev_region(sfp_major, MAX_SFPS);
chrdev_fail:
	return err;
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

	class_destroy(sfp_class);
	unregister_chrdev_region(sfp_major, MAX_SFPS);
}
module_exit(sfp_exit);

MODULE_DESCRIPTION("SFP DRIVER");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_LICENSE("GPL");

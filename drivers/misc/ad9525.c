/*
 * File: ad9525.c
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
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
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

#define DEV_NAME "AD9525"

#define GPIO_INVAL              (~0)

/* AD9525 registers */
#define AD9525_SPI_MODE_REG     (0x000)
#define AD9525_SPI_MODE_VAL      (0x81) /* SDO Active, MSB first */
#define AD9525_READBACK_REG     (0x004)
#define AD9525_READBACK_VAL      (0x01) /* Read active registers */
#define AD9525_IO_UPDATE_REG    (0x232)
#define AD9525_IO_UPDATE_VAL     (0x01)

MODULE_LICENSE("GPL v2");

struct ad9525_data {
	struct spi_device *spi;
	spinlock_t      spi_lock;
	int reset_gpio;
	int reset_flags;
	int initialized;
};

static int ad9525_write_update(struct ad9525_data *ad9525, u8 val, u16 offset)
{
	u8 tx1_buf[3];
	u8 tx2_buf[3];
	struct spi_message m;
	struct spi_transfer t[2];
	struct spi_device *spi_dev = ad9525->spi;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));
	/* Write transaction */
	tx1_buf[0] = (u8)(((offset >> 8) & 0x3) | 0x80);
	tx1_buf[1] = (u8)(offset & 0x00FF);
	tx1_buf[2] = val;
	t[0].len = sizeof(tx1_buf);
	t[0].tx_buf = tx1_buf;
	spi_message_add_tail(&t[0], &m);
	/* IO Update instruction */
	tx2_buf[0] = (u8)((AD9525_IO_UPDATE_REG >> 8) | 0x80);
	tx2_buf[1] = (u8)(AD9525_IO_UPDATE_REG & 0xFF);
	tx2_buf[2] = AD9525_IO_UPDATE_VAL;
	t[1].len = sizeof(tx2_buf);
	t[1].tx_buf = tx2_buf;
	spi_message_add_tail(&t[1], &m);
	return spi_sync(spi_dev, &m);
}

static int ad9525_init_gpio(struct ad9525_data *ad9525)
{
	int gpio;
	enum of_gpio_flags flags;
	struct device_node *np = ad9525->spi->dev.of_node;

	ad9525->reset_gpio = GPIO_INVAL;
	gpio = of_get_named_gpio_flags(np, "gpio-reset", 0, &flags);
	if (gpio >= 0) {
		gpio_request(gpio, "ad9525-reset");
		gpio_direction_output(gpio, flags ? 0 : 1);
		ad9525->reset_gpio = gpio;
		ad9525->reset_flags = flags;
	} else if (gpio != -ENOENT) {
		dev_info(&(ad9525->spi->dev), "%s failed to get gpio-reset\n",
			ad9525->spi->modalias);
		return gpio;
	}
	return 0;
}

/* Toggle reset gpio to reset AD9525 */
static int ad9525_reset(struct ad9525_data *ad9525)
{
	int ret = 0;
	struct device *dev = &(ad9525->spi->dev);

	ad9525->initialized = 0;
	if (ad9525->reset_gpio != GPIO_INVAL) {
		if (ad9525->reset_flags) {
			gpio_set_value(ad9525->reset_gpio, 1);
			gpio_set_value(ad9525->reset_gpio, 0);
		} else {
			gpio_set_value(ad9525->reset_gpio, 0);
			gpio_set_value(ad9525->reset_gpio, 1);
		}
	} else {
		dev_dbg(dev, "Reset GPIO not set in dtb\n");
	}
	/* Set the SPI communications mode */
	ret = ad9525_write_update(ad9525, AD9525_SPI_MODE_VAL,
						AD9525_SPI_MODE_REG);
	if (ret)
		return ret;

	/* Set the register readback mode */
	ret = ad9525_write_update(ad9525, AD9525_READBACK_VAL,
						AD9525_READBACK_REG);
	if (ret)
		return ret;

	ad9525->initialized = 1;
	return ret;
}


static int ad9525_probe(struct spi_device *spi)
{
	static struct ad9525_data *ad9525;
	struct device *dev = &spi->dev;
	int ret = 0;

	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0)
		goto out;

	ad9525 = kzalloc(sizeof(*ad9525), GFP_KERNEL);
	if (!ad9525) {
		dev_err(dev, "Failed to allocate ad9525\n");
		return -ENOMEM;
	}

	ad9525->spi = spi;
	ad9525->initialized = 0;
	ad9525->reset_gpio = GPIO_INVAL;
	spin_lock_init(&ad9525->spi_lock);

	dev_set_drvdata(dev, ad9525);

	ret = ad9525_init_gpio(ad9525);
	if (ret)
		goto out;

	ret = ad9525_reset(ad9525);
	if (ret)
		goto out;

	dev_info(dev, "%s probe passed\n", spi->modalias);
	return ret;
out:
	if (ad9525 && ad9525->reset_gpio != GPIO_INVAL)
		gpio_free(ad9525->reset_gpio);
	kfree(ad9525);
	return ret;
}

static int ad9525_remove(struct spi_device *spi)
{
	int ret = 0;
	struct device *dev = &spi->dev;
	struct ad9525_data *ad9525 = dev_get_drvdata(&spi->dev);
	dev_dbg(dev, "AD9525 module uninstalled\n");

	if (ad9525 && ad9525->reset_gpio != GPIO_INVAL)
		gpio_free(ad9525->reset_gpio);
	kfree(ad9525);

	return ret;
}

static struct of_device_id ad9525_match[] = {
	{.compatible = "adi,ad9525",},
	{},
};


static struct spi_driver ad9525_driver = {
	.driver = {
		.name = "ad9525_driver",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ad9525_match),
		},
	.probe = ad9525_probe,
	.remove = ad9525_remove,
};

static int __init ad9525_init(void)
{
	int ret = 0;

	ret = spi_register_driver(&ad9525_driver);
	if (ret) {
		pr_err("ad9525: spi_register_driver failed with err %x\n",
			ret);
		return ret;
	}

	return ret;

}

static void __exit ad9525_exit(void)
{
	spi_unregister_driver(&ad9525_driver);
}

module_init(ad9525_init);
module_exit(ad9525_exit);

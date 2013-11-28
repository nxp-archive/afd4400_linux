/*
 * File: spi-d4400.c
 *
 * This driver is derived work from drivers/spi/spi-imx.c
 * Author: Shaveta Leekha <shaveta@freescale.com>
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

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>

#include <linux/platform_data/spi-d4400.h>

#define DRIVER_NAME "spi_d4400"

static int cs_mux;

static inline unsigned spi_d4400_get_fifosize(struct spi_d4400_data *d)
{
	return (d->devtype_data->devtype == D4400_ECSPI) ? 64 : 8;
}

/* D4400 CSPI */
static unsigned int __maybe_unused spi_d4400_clkdiv_2(unsigned int fin,
		unsigned int fspi)
{
	int i, div = 4;

	for (i = 0; i < 7; i++) {
		if (fspi * div >= fin)
			return i;
		div <<= 1;
	}

	return 7;
}

/* D4400 eCSPI */
static unsigned int d4400_ecspi_clkdiv(unsigned int fin, unsigned int fspi)
{
	/*
	 * there are two 4-bit dividers, the pre-divider divides by
	 * $pre, the post-divider by 2^$post
	 */
	unsigned int pre, post;

	if (unlikely(fspi > fin))
		return 0;

	post = fls(fin) - fls(fspi);
	if (fin > fspi << post)
		post++;

	/* now we have: (fin <= fspi << post) with post being minimal */

	post = max(4U, post) - 4;
	if (unlikely(post > 0xf)) {
		pr_err("%s: cannot set clock freq: %u (base freq: %u)\n",
				__func__, fspi, fin);
		return 0xff;
	}

	pre = DIV_ROUND_UP(fin, fspi << post) - 1;

	pr_debug("%s: fin: %u, fspi: %u, post: %u, pre: %u\n",
			__func__, fin, fspi, post, pre);
	return (pre << D4400_ECSPI_CTRL_PREDIV_OFFSET) |
		(post << D4400_ECSPI_CTRL_POSTDIV_OFFSET);
}

static void __maybe_unused d4400_ecspi_intctrl(struct spi_d4400_data *spi_d4400, int enable)
{
	unsigned val = 0;

	if (enable & D4400_INT_TE)
		val |= D4400_ECSPI_INT_TEEN;

	if (enable & D4400_INT_RR)
		val |= D4400_ECSPI_INT_RREN;

	writel(val, spi_d4400->base + D4400_ECSPI_INT);
}

static void __maybe_unused d4400_ecspi_trigger(struct spi_d4400_data *spi_d4400)
{
	u32 reg;

	reg = readl(spi_d4400->base + D4400_ECSPI_CTRL);
	reg |= D4400_ECSPI_CTRL_XCH;
	writel(reg, spi_d4400->base + D4400_ECSPI_CTRL);
}

static int __maybe_unused d4400_ecspi_config(struct spi_d4400_data *spi_d4400,
		struct spi_d4400_config *config)
{
	u32 ctrl = D4400_ECSPI_CTRL_ENABLE, cfg = 0;

	/*
	 * The hardware seems to have a race condition when changing modes. The
	 * current assumption is that the selection of the channel arrives
	 * earlier in the hardware than the mode bits when they are written at
	 * the same time.
	 * So set master mode for all channels as we do not support slave mode.
	 */
	ctrl |= D4400_ECSPI_CTRL_MODE_MASK;

	/* set clock speed */
	ctrl |= d4400_ecspi_clkdiv(spi_d4400->spi_clk, config->speed_hz);

	/* set chip select to use */
	ctrl |= D4400_ECSPI_CTRL_CS(config->cs);

	ctrl |= (config->bpw - 1) << D4400_ECSPI_CTRL_BL_OFFSET;

	cfg |= D4400_ECSPI_CONFIG_SBBCTRL(config->cs);

	if (config->mode & SPI_CPHA)
		cfg |= D4400_ECSPI_CONFIG_SCLKPHA(config->cs);

	if (config->mode & SPI_CPOL) {
		cfg |= D4400_ECSPI_CONFIG_SCLKPOL(config->cs);
		cfg |= D4400_ECSPI_CONFIG_SCLKCTL(config->cs);
	}
	if (config->mode & SPI_CS_HIGH)
		cfg |= D4400_ECSPI_CONFIG_SSBPOL(config->cs);

	writel(ctrl, spi_d4400->base + D4400_ECSPI_CTRL);
	writel(cfg, spi_d4400->base + D4400_ECSPI_CONFIG);

	return 0;
}

static int __maybe_unused d4400_ecspi_rx_available(struct spi_d4400_data *spi_d4400)
{
	return readl(spi_d4400->base + D4400_ECSPI_STAT) & D4400_ECSPI_STAT_RR;
}

static void __maybe_unused d4400_ecspi_reset(struct spi_d4400_data *spi_d4400)
{
	/* drain receive buffer */
	while (d4400_ecspi_rx_available(spi_d4400))
		readl(spi_d4400->base + D4400_CSPIRXDATA);
}

static struct spi_d4400_devtype_data d4400_ecspi_devtype_data = {
	.intctrl = d4400_ecspi_intctrl,
	.config = d4400_ecspi_config,
	.trigger = d4400_ecspi_trigger,
	.rx_available = d4400_ecspi_rx_available,
	.reset = d4400_ecspi_reset,
	.devtype = D4400_ECSPI,
};

static struct platform_device_id spi_d4400_devtype[] = {
	{
		.name = "d4400-ecspi",
		.driver_data = (kernel_ulong_t) &d4400_ecspi_devtype_data,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id spi_d4400_dt_ids[] = {
	{ .compatible = "fsl,d4400-ecspi", .data = &d4400_ecspi_devtype_data, },
	{ /* sentinel */ }
};

static void spi_d4400_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_d4400_data *spi_d4400 = spi_master_get_devdata(spi->master);
	int gpio;
	int dev_toggle_flag = 1;
	int active = is_active != 0;
	int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);

	gpio = spi_d4400->chipselect[spi->chip_select];

	if (!gpio_is_valid(gpio))
		return;

	gpio_set_value(gpio, dev_is_lowactive ^ active);
}

static void spi_d4400_push(struct spi_d4400_data *spi_d4400)
{
	while (spi_d4400->txfifo < spi_d4400_get_fifosize(spi_d4400)) {
		if (!spi_d4400->count)
			break;
		spi_d4400->tx(spi_d4400);
		spi_d4400->txfifo++;
	}

	spi_d4400->devtype_data->trigger(spi_d4400);
}

static irqreturn_t spi_d4400_isr(int irq, void *dev_id)
{
	struct spi_d4400_data *spi_d4400 = dev_id;

	while (spi_d4400->devtype_data->rx_available(spi_d4400)) {
		spi_d4400->rx(spi_d4400);
		spi_d4400->txfifo--;
	}

	if (spi_d4400->count) {
		spi_d4400_push(spi_d4400);
		return IRQ_HANDLED;
	}

	if (spi_d4400->txfifo) {
		/* No data left to push, but still waiting for rx data,
		 * enable receive data available interrupt.
		 */
		spi_d4400->devtype_data->intctrl(
				spi_d4400, D4400_INT_RR);
		return IRQ_HANDLED;
	}

	spi_d4400->devtype_data->intctrl(spi_d4400, 0);
	complete(&spi_d4400->xfer_done);

	return IRQ_HANDLED;
}

static int spi_d4400_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_d4400_data *spi_d4400 = spi_master_get_devdata(spi->master);
	struct spi_d4400_config config;

	config.bpw = t ? t->bits_per_word : spi->bits_per_word;
	config.speed_hz  = t ? t->speed_hz : spi->max_speed_hz;
	config.mode = spi->mode;

	config.cs = spi->chip_select;

	if (!config.speed_hz)
		config.speed_hz = spi->max_speed_hz;
	if (!config.bpw)
		config.bpw = spi->bits_per_word;

	/* Initialize the functions for transfer */
	if (config.bpw <= 8) {
		spi_d4400->rx = spi_d4400_buf_rx_u8;
		spi_d4400->tx = spi_d4400_buf_tx_u8;
	} else if (config.bpw <= 16) {
		spi_d4400->rx = spi_d4400_buf_rx_u16;
		spi_d4400->tx = spi_d4400_buf_tx_u16;
	} else if (config.bpw <= 32) {
		spi_d4400->rx = spi_d4400_buf_rx_u32;
		spi_d4400->tx = spi_d4400_buf_tx_u32;
	} else
		BUG();

	spi_d4400->devtype_data->config(spi_d4400, &config);

	return 0;
}

static int spi_d4400_transfer(struct spi_device *spi,
				struct spi_message *msg)
{
	struct spi_d4400_data *spi_d4400 = spi_master_get_devdata(spi->master);

	unsigned long           flags;

	unsigned                cs_change;

	struct spi_transfer 	*transfer = NULL;
	int			status = 0;
	int			xfer_setup = -1;

	cs_change = 1;
	status = 0;

	if (unlikely(list_empty(&msg->transfers))) {
		dev_err(&spi->dev, "D4400: message list empty\n");
		return -EINVAL;
	}

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	spin_lock_irqsave(&spi_d4400->lock, flags);

	list_for_each_entry(transfer, &msg->transfers, transfer_list) {

		spin_unlock_irqrestore(&spi_d4400->lock, flags);

		if (transfer->speed_hz || transfer->bits_per_word)
			xfer_setup = 1;

		if (xfer_setup != 0) {
			status = spi_d4400_setupxfer(spi, transfer);
			if (status < 0) {
				dev_err(&spi->dev, "D4400: spi_d4400_setupxfer failed\n");
				break;
			}
			if (xfer_setup == -1)
				xfer_setup = 0;
		}

		if (cs_change) {
			spi_d4400_chipselect(spi, 1);
			ndelay(10);
		}
		cs_change = transfer->cs_change;

		if (!transfer->tx_buf && !transfer->rx_buf && transfer->len) {
			dev_err(&spi->dev, "missing rx or tx buf\n");
			status = -EINVAL;
			break;
		}

		if (transfer->len) {
			spi_d4400->tx_buf = transfer->tx_buf;
			spi_d4400->rx_buf = transfer->rx_buf;
			spi_d4400->count = transfer->len;
			spi_d4400->txfifo = 0;

			init_completion(&spi_d4400->xfer_done);

			spi_d4400_push(spi_d4400);

			spi_d4400->devtype_data->intctrl(spi_d4400, D4400_INT_TE);

			wait_for_completion(&spi_d4400->xfer_done);

			status = transfer->len;
		}

		if (status > 0)
			msg->actual_length += status;

		if (status != transfer->len) {
			/* Report correct kind of error */
			if (status >= 0)
				status = -EREMOTEIO;
			break;
		}
		status = 0;

		if (cs_change && !list_is_last(&transfer->transfer_list, &msg->transfers)) {
			/* sometimes a short mid-message deselect of the chip
			 * may be needed to terminate a mode or command
			 */
			spi_d4400_chipselect(spi, 0);
			ndelay(10);
		}
	}

	msg->status = status;
	msg->complete(msg->context);

	if (!(status == 0 && cs_change)) {
		spi_d4400_chipselect(spi, 0);
		ndelay(100);
	}

	return 0;
}

static int spi_d4400_setup(struct spi_device *spi)
{
	struct spi_d4400_data *spi_d4400 = spi_master_get_devdata(spi->master);
	int gpio;

	gpio = spi_d4400->chipselect[spi->chip_select];

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);

	if (gpio_is_valid(gpio))
		gpio_direction_output(gpio, spi->mode & SPI_CS_HIGH ? 0 : 1);

	spi_d4400_chipselect(spi, 0);

	return 0;
}

static void spi_d4400_cleanup(struct spi_device *spi)
{
}

static int fsl_d4400_spi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(spi_d4400_dt_ids, &pdev->dev);
	struct spi_master *master;
	struct spi_d4400_data *spi_d4400;
	struct resource *res;
	struct pinctrl *pinctrl;
	int i, ret, num_cs, bus_num;

	if (!np) {
		dev_err(&pdev->dev, "can't get the device node\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "fsl,spi-num-chipselects", &num_cs);
	if (ret < 0)
		return ret;

	master = spi_alloc_master(&pdev->dev,
			sizeof(struct spi_d4400_data) + sizeof(int) * num_cs);
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	/* Added for static(from device tree) and dynamic bus num allocation */
	ret = of_property_read_u32(np, "fsl,spi-bus-num", &bus_num);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "can't get bus number from device tree\n");
		master->bus_num = pdev->id;
	} else {
		master->bus_num = bus_num;
	}

	master->num_chipselect = num_cs;
	spi_d4400 = spi_master_get_devdata(master);
	spi_d4400->master = spi_master_get(master);

	for (i = 0; i < master->num_chipselect; i++) {
		int cs_gpio = of_get_named_gpio(np, "cs-gpios", i);

		spi_d4400->chipselect[i] = cs_gpio;

		if (!gpio_is_valid(cs_gpio))
			continue;

		ret = gpio_request(spi_d4400->chipselect[i], DRIVER_NAME);
		if (ret) {
			dev_err(&pdev->dev, "can't get cs gpios\n");
			goto out_gpio_free;
		}
	}

	master->transfer = spi_d4400_transfer;
	master->setup = spi_d4400_setup;
	master->cleanup = spi_d4400_cleanup;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	init_completion(&spi_d4400->xfer_done);

	spi_d4400->devtype_data = of_id ? of_id->data :
		(struct spi_d4400_devtype_data *) pdev->id_entry->driver_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get platform resource\n");
		ret = -ENOMEM;
		goto out_gpio_free;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto out_gpio_free;
	}

	spi_d4400->base = ioremap(res->start, resource_size(res));
	if (!spi_d4400->base) {
		ret = -EINVAL;
		goto out_release_mem;
	}

	spi_d4400->irq = platform_get_irq(pdev, 0);
	if (spi_d4400->irq < 0) {
		ret = -EINVAL;
		goto out_iounmap;
	}

	ret = request_irq(spi_d4400->irq, spi_d4400_isr, 0, DRIVER_NAME, spi_d4400);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", spi_d4400->irq, ret);
		goto out_iounmap;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		goto out_free_irq;
	}

	spi_d4400->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(spi_d4400->clk_ipg)) {
		ret = PTR_ERR(spi_d4400->clk_ipg);
		goto out_free_irq;
	}

	spi_d4400->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(spi_d4400->clk_per)) {
		ret = PTR_ERR(spi_d4400->clk_per);
		goto out_free_irq;
	}

	clk_prepare_enable(spi_d4400->clk_per);
	clk_prepare_enable(spi_d4400->clk_ipg);

	spi_d4400->spi_clk = clk_get_rate(spi_d4400->clk_per);

	spi_d4400->devtype_data->reset(spi_d4400);

	spi_d4400->devtype_data->intctrl(spi_d4400, 0);

	master->dev.of_node = pdev->dev.of_node;

	dev_dbg(&pdev->dev, "Registering DFE_spi_master struct\n");

	ret = spi_register_master(master);
	if (ret)
		goto out_clk_put;

	dev_info(&pdev->dev, "DFE D4400 SPI driver probed\n");

	return ret;

out_clk_put:
	clk_disable_unprepare(spi_d4400->clk_per);
	clk_disable_unprepare(spi_d4400->clk_ipg);
out_free_irq:
	free_irq(spi_d4400->irq, spi_d4400);
out_iounmap:
	iounmap(spi_d4400->base);
out_release_mem:
	release_mem_region(res->start, resource_size(res));
out_gpio_free:
	while (--i >= 0) {
		if (gpio_is_valid(spi_d4400->chipselect[i]))
			gpio_free(spi_d4400->chipselect[i]);
	}
	spi_master_put(master);
	kfree(master);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int fsl_d4400_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct spi_d4400_data *spi_d4400 = spi_master_get_devdata(master);
	int i;

	writel(0, spi_d4400->base + D4400_CSPICTRL);
	clk_disable_unprepare(spi_d4400->clk_per);
	clk_disable_unprepare(spi_d4400->clk_ipg);
	free_irq(spi_d4400->irq, spi_d4400);
	iounmap(spi_d4400->base);

	for (i = 0; i < master->num_chipselect; i++)
		if (gpio_is_valid(spi_d4400->chipselect[i]))
			gpio_free(spi_d4400->chipselect[i]);

	spi_master_put(master);

	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);

	spi_unregister_master(master);

	return 0;
}

static struct platform_driver fsl_d4400_spi_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = spi_d4400_dt_ids,
		   },
	.id_table = spi_d4400_devtype,
	.probe = fsl_d4400_spi_probe,
	.remove = fsl_d4400_spi_remove,
};
module_platform_driver(fsl_d4400_spi_driver);

static int __init fsl_d4400_spi_init(void)
{
        return platform_driver_register(&fsl_d4400_spi_driver);
}
module_init(fsl_d4400_spi_init);

static void __exit fsl_d4400_spi_exit(void)
{
        platform_driver_unregister(&fsl_d4400_spi_driver);
}
module_exit(fsl_d4400_spi_exit);

MODULE_DESCRIPTION("SPI D4400 Master Controller driver");
MODULE_AUTHOR("Shaveta Leekha, Freescale");
MODULE_LICENSE("GPL");

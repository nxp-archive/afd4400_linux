/*
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

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/string.h>

#include <linux/qixis.h>

static void __iomem *qixisbase;

void qixis_write(u8 val, u8 offset)
{
	__raw_writeb(val, (qixisbase + offset));
}

u8 qixis_read(u8 offset)
{
	u8 retval =  __raw_readb(qixisbase + offset);

	return retval;
}

int qixis_get_xcvr_present_status(int xcvr_id)
{
	u8 present_stat, mask = 0x2;
	int ret = 0;

	if (!qixisbase) {
		ret = -EPROBE_DEFER;
		goto out;
	}
	present_stat = qixis_read(QIXIS_STAT_PRESENT);
	mask = (mask >> (xcvr_id - 1));

	pr_info("qixis present detect: present stat 0x%x, mask 0x%x",
		present_stat, mask);

	if (!(present_stat & mask))
		ret = 1;
out:
	return ret;
}

int write_jcpll_reg(u8 val, u16 offset)
{
	/* 12 bits (0 - 11) of offset contains register address
	 *  MAX offset value is 0x3ff
	 */
	u8 lsb = (u8)(offset & 0x00ff);
	u8 nibble0 = (u8) ((offset >> 8) & 0x03);

	u8 msb = WRITE_NIBBLE | nibble0;
	int n = TRY_COUNT;

	/* wait till previous transaction is completed */
	/* try n times only */
	while((qixis_read(QIXIS_CLK_JCPLL_ADDH) & APPLY_STATUS)
			&& --n) {
		schedule_timeout_interruptible(msecs_to_jiffies(MAX_DELAY));
	}

	if(!n)
		return JCPLL_XFER_ERR;

	qixis_write(val, QIXIS_CLK_JCPLL_DATA);
	qixis_write(lsb, QIXIS_CLK_JCPLL_ADDL);
	qixis_write(msb, QIXIS_CLK_JCPLL_ADDH);

	return JCPLL_SUCCESS;
}

u8 read_jcpll_reg(u16 offset)
{
	u8 retval = 0xFF;

	/* 12 bits (0 - 11) of offset contains register address
	 * MAX offset value is 0x3ff
	 */
	u8 lsb = (u8)(offset & 0x00ff);
	u8 nibble0 = (u8) ((offset >> 8) & 0x03);

	u8 msb = READ_NIBBLE | nibble0;
	int n = TRY_COUNT;

	qixis_write(lsb, QIXIS_CLK_JCPLL_ADDL);
	qixis_write(msb, QIXIS_CLK_JCPLL_ADDH);

	/* wait till last bit is 0 */
	while((qixis_read(QIXIS_CLK_JCPLL_ADDH) & APPLY_STATUS)
			&& --n) {
		schedule_timeout_interruptible(msecs_to_jiffies(MAX_DELAY));
	}

	if(!n)
		return retval;

	retval = qixis_read(QIXIS_CLK_JCPLL_DATA);

	return retval;
}

int qixis_lock_jcpll(void)
{
	int ret = JCPLL_SUCCESS;

	ret = write_jcpll_reg(VDD_CP_BY_HALF, JCPLL_STATUS_PIN_REG);
	if(ret != JCPLL_SUCCESS)
		return ret;

	ret = write_jcpll_reg(PWR_DWN_PLL, JCPLL_PWR_DWN_REG);
	if(ret != JCPLL_SUCCESS)
		return ret;

	ret = write_jcpll_reg(UPDATE_IO, JCPLL_IO_UPDATE_REG);
	if(ret != JCPLL_SUCCESS)
		return ret;

	return ret;
}

int qixis_unlock_jcpll(void)
{
	int ret = JCPLL_SUCCESS;

	ret = write_jcpll_reg(VDD_CP_NORMAL, JCPLL_STATUS_PIN_REG);
	if(ret != JCPLL_SUCCESS)
		return ret;

	write_jcpll_reg(PWR_ENB_PLL, JCPLL_PWR_DWN_REG);
	if(ret != JCPLL_SUCCESS)
		return ret;

	write_jcpll_reg(UPDATE_IO, JCPLL_IO_UPDATE_REG);
	if(ret != JCPLL_SUCCESS)
		return ret;

	return ret;
}

static int __exit d4400_fpga_remove(struct platform_device *pdev)
{
	qixisbase = NULL;
	return 0;
}

static int __init d4400_fpga_probe(struct platform_device *pdev)
{
	int ret = 0;

	qixisbase = of_iomap(pdev->dev.of_node, 0);
	if (!qixisbase) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	dev_info(&pdev->dev,
		"QIXIS fpga probe : %02x:%02x - %02x.%02x\n",
		qixis_read(QIXIS_ID),
		qixis_read(QIXIS_EVBBRD_VER),
		qixis_read(QIXIS_FPGA_MAJOR_VER),
		qixis_read(QIXIS_FPGA_MINOR_VER));

	return ret;
}

static struct of_device_id d4400_fpga_ids[] = {
	{.compatible = "fsl,d4400-fpga", },
	{ /* sentinel */ }
};

static struct platform_driver d4400_fpga_driver = {
	.remove = __exit_p(d4400_fpga_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = d4400_fpga_ids,
	},
};

static int __init d4400_fpga_init(void)
{
	return platform_driver_probe(&d4400_fpga_driver, d4400_fpga_probe);
}
module_init(d4400_fpga_init);

static void __exit d4400_fpga_exit(void)
{
	platform_driver_unregister(&d4400_fpga_driver);
}
module_exit(d4400_fpga_exit);

MODULE_DESCRIPTION("QIXIS FPGA register access via WEIM bus");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

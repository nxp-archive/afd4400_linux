/*
 * drivers/jesd/phygasket.c
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jesd204.h>
#include <linux/phygasket.h>

struct phygasket_private *phypriv;
/**@brief inline for set, clear and test bits for 32 bit
*/
static inline void
jclear_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) &= ~(1 << (nr & 31));
}

static inline void
jset_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) |= (1 << (nr & 31));
}

void invert_lanes(u32 *offset, u8 lane, u8 invert)
{
	offset = offset + lane;
	if (invert == 1)
		jset_bit(8, offset);
	else
		jclear_bit(8, offset);
}

void phygasket_config_tx_lane(u32 *lane0_ctrl_reg, u8 lane,
	enum phygasket_data_src data_src)
{
	u32 *reg, val;

	reg = lane0_ctrl_reg + lane;
	val = ioread32(reg);
	val &= ~TX_SRC_MASK;

	if (data_src == PHY_DATA_JESDTX)
		val |= TX_SRC_JESDTX;
	else if (data_src == PHY_DATA_TEST_PATTRN)
		val |= TX_SRC_TEST_PATTRN;

	iowrite32(val, reg);
}

void phygasket_config_rx_lane(u32 *lane0_ctrl_reg, u8 lane,
	enum phygasket_data_src data_src)
{
	u32 *reg, val;

	reg = lane0_ctrl_reg + lane;
	val = ioread32(reg);
	val &= ~RX_SRC_LOOPBACK_EN;

	if (data_src == PHY_DATA_RX_LOOPBACK)
		val |= RX_SRC_LOOPBACK_EN;
	iowrite32(val, reg);
}

struct phygasket *map_phygasket(struct device_node *phy_node)
{
	struct phygasket *phy = NULL, *phy_tmp;

	list_for_each_entry(phy_tmp, &phypriv->phy_list, list) {
		if (phy_node == phy_tmp->dev_node) {
			phy = phy_tmp;
			break;
		}
	}
	return phy;
}


int phy_softreset(struct phygasket *phy, u8 device, u8 reset, u8 lane)
{

	if (device == DEVICE_RX) {
		if (reset == 1)
			jset_bit(lane, &phy->pregs->rx_softreset);
		else
			jclear_bit(lane, &phy->pregs->rx_softreset);
	} else if (device == DEVICE_TX) {
		if (reset == 1)
			jset_bit(lane, &phy->pregs->tx_softreset);
		else
			jclear_bit(lane, &phy->pregs->tx_softreset);
	} else
		return -EINVAL;
	return 0;
}

void phy_gasket_swap_lanes(struct phygasket *phy, u8 lane,
	int enable, enum jesd_dev_type dev_type)
{
	u32 *reg, val, shift;

	if (dev_type == JESD_DEV_TX)
		reg = &phy->pregs->tx_swap;
	else
		reg = &phy->pregs->rx_swap;

	shift = lane / 2;
	val = ioread32(reg);
	val &= ~(1 << shift);
	if (enable)
		val |= (1 << shift);

	iowrite32(val, reg);
}

int phy_inverse_lanes(struct phygasket *phy, u8 lane, u8 invert, u8 device)
{
	if (device == DEVICE_RX)
		invert_lanes(&phy->pregs->rx_lane0_ctrl, lane, invert);
	else if (device == DEVICE_TX)
		invert_lanes(&phy->pregs->tx_lane0_ctrl, lane, invert);
	else
		return -EINVAL;

	return 0;
}

int phy_gasket_lane_ctrl(struct phygasket *phy,
	enum phygasket_data_src data_src, u8 lane)
{
	int retcode = 0;

	switch (data_src) {
	case PHY_DATA_JESDTX:
	case PHY_DATA_TEST_PATTRN:
		phygasket_config_tx_lane(&phy->pregs->tx_lane0_ctrl,
			lane, data_src);
		break;
	case PHY_DATA_JESDRX:
	case PHY_DATA_RX_LOOPBACK:
		phygasket_config_rx_lane(&phy->pregs->rx_lane0_ctrl,
			lane, data_src);
		break;
	default:
		retcode = -EINVAL;
		break;
	}

	return retcode;
}

int do_patter_generator(struct phygasket *phy, struct patgen *pgen)
{
	int retcode = 0;
	unsigned int pattern = 0;

	if (phy != NULL) {
		if (phy->init_flag == PHYGASKET_INIT_SUCCESS) {
			phy->pregs->tx_patgen = 0x00;
			pattern = (pgen->rpt_ila << 1);
			pattern |= (pgen->init_cgs << 6);
			pattern |= (pgen->payload << 8);
			pattern |= (pgen->disp << 15);
			pattern |= (pgen->opmf << 16);

			phy->pregs->tx_patgen = pattern;

		/*start a new pattern generation with GO bit set in patgen*/
			jset_bit(0, &phy->pregs->tx_patgen);

		} else
			retcode = -EINVAL;
	} else
		retcode = -EINVAL;

	return retcode;
}

static int phygasket_probe(struct platform_device *pdev)
{
	int retcode = 0;
	struct device_node *node = pdev->dev.of_node;
	struct phygasket *phy = NULL;


	if (!node || !of_device_is_available(node)) {
		retcode = -ENODEV;
		dev_err(&pdev->dev, "phy dev not avaiable");
		goto out;
	}

	phy = kzalloc(sizeof(struct phygasket), GFP_KERNEL);

	if (!phy) {
		retcode = -ENOMEM;
		dev_info(&pdev->dev, "phy id malloc fail");
		goto out;
	}

	phy->dev = &pdev->dev;
	phy->pregs = of_iomap(node, 0);

	if (!phy->pregs) {
		dev_info(phy->dev, "Failed to iomap regs");
		retcode = -ENOMEM;
		goto out;
	}
	phy->init_flag = PHYGASKET_INIT_SUCCESS;
	phy->dev_node = node;
	dev_set_drvdata(phy->dev, phy);
	spin_lock(&phypriv->lock);
	list_add_tail(&phy->list, &phypriv->phy_list);
	spin_unlock(&phypriv->lock);

	return retcode;
out:
	kfree(phy);
	return retcode;
}


static int phygasket_remove(struct platform_device *pdev)
{
	struct phygasket *phy, *phy_temp;
	struct list_head *pos, *nx;
	int retcode = 0;

	phy = (struct phygasket *) dev_get_drvdata(&pdev->dev);

	iounmap(phy->pregs);
	raw_spin_lock(&phypriv->lock);
	list_for_each_safe(pos, nx, &phypriv->phy_list) {
		phy_temp = list_entry(pos, struct phygasket, list);
		if (phy_temp == phy) {
			list_del(&phy->list);
			kfree(phy);
			break;
		}
	}
	raw_spin_unlock(&phypriv->lock);

	return retcode;
}


static struct of_device_id phygasket_match[] = {
	{.compatible = "fsl,d4400-phygasket",},
	{},
};

static struct platform_driver phygasket_driver = {
	.driver = {
		.name = "fsl-phygasket",
		.owner = THIS_MODULE,
		.of_match_table = phygasket_match,
	},
	.probe = phygasket_probe,
	.remove = phygasket_remove,
};


static int __init phygasket_module_init(void)
{

	phypriv = kzalloc(sizeof(struct phygasket_private), GFP_KERNEL);
	if (phypriv == NULL) {
		pr_err("phygasket: Failed to allocate priv");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&phypriv->phy_list);
	spin_lock_init(&phypriv->lock);

	return platform_driver_register(&phygasket_driver);
}

static void __exit phygasket_module_clean(void)
{
	struct phygasket *phy;
	struct list_head *pos, *nx;

	spin_lock(&phypriv->lock);
	list_for_each_safe(pos, nx, &phypriv->phy_list) {
		phy = list_entry(pos, struct phygasket, list);
		list_del(&phy->list);
		kfree(phy);
	}
	spin_unlock(&phypriv->lock);
	kfree(phypriv);
	platform_driver_unregister(&phygasket_driver);
}

module_init(phygasket_module_init);
module_exit(phygasket_module_clean);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("phygasket driver");

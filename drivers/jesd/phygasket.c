/*
* drivers/jesd/phygasket.h
*
* Author: Freescale semiconductor, Inc.
*
* Copyright 2013 Freescale Semiconductor, Inc.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
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
#include <uapi/linux/jesd204.h> /*user and kernel interface header*/

#include "jesd204.h"
#include "phygasket.h"

struct phygasket_private *phypriv;
static struct list_head phy_list = LIST_HEAD_INIT(phy_list);
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

void set_txlanes_jesd204tx(u32 *offset, u8 lane)
{
	offset = offset + lane;
	jclear_bit(0, offset);
	jclear_bit(1, offset);
}

void set_txlanes_testpattern(u32 *offset, u8 lane)
{
	offset = offset + lane;
	jset_bit(0, offset);
	jset_bit(1, offset);
}

void set_rxlanes_phy(u32 *offset, u8 lane)
{
	offset = offset + lane;
	jclear_bit(0, offset);
}


void set_rxlanes_loopback(u32 *offset, u8 lane)
{
	offset = offset + lane;
	jset_bit(0, offset);
}

struct phygasket *map_phygasket(struct device_node *phy_node)
{
	struct phygasket *phy = NULL;

	list_for_each_entry(phy, &phy_list, list) {
		if (phy_node == phy->node)
			break;
	}
	return phy;
}
EXPORT_SYMBOL(map_phygasket);


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
EXPORT_SYMBOL(phy_softreset);

int phy_swap_lanes(struct phygasket *phy, u8 lane, u8 swap, u8 device)
{
	u32 bit_bang = 0;

	if (lane == 0 || lane == 1)
		bit_bang = 0;
	else if (lane == 2 || lane == 3)
		bit_bang = 1;
	else if (lane == 4 || lane == 5)
		bit_bang = 2;
	else if (lane == 6 || lane == 7)
		bit_bang = 3;
	else
		return -EINVAL;

	if (swap == 0) {
		if (device == DEVICE_RX)
			jclear_bit(bit_bang, &phy->pregs->rx_swap);
		else
			jclear_bit(bit_bang, &phy->pregs->tx_swap);
	} else if (swap == 1) {
		if (device == DEVICE_RX)
			jset_bit(bit_bang, &phy->pregs->rx_swap);
		else
			jset_bit(bit_bang, &phy->pregs->tx_swap);
	} else
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(phy_swap_lanes);



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
EXPORT_SYMBOL(phy_inverse_lanes);


int phy_gasket_lane_ctrl(struct phygasket *phy, u8 mode, u8 lane)
{
	int retcode = 0;
	switch (mode) {
	case JESD204TX:
		set_txlanes_jesd204tx(&phy->pregs->tx_lane0_ctrl, lane);
		break;
	case TEST_PATT:
		set_txlanes_testpattern(&phy->pregs->tx_lane0_ctrl, lane);
		break;

	case RX_PHY:
		set_rxlanes_phy(&phy->pregs->rx_lane0_ctrl, lane);
		break;

	case RX_LOOPBACK:
		set_rxlanes_loopback(&phy->pregs->rx_lane0_ctrl, lane);
		break;
	default:
		retcode = -ENOTTY;
		break;
	}
	return retcode;
}
EXPORT_SYMBOL(phy_gasket_lane_ctrl);

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
EXPORT_SYMBOL(do_patter_generator);

int phygasket_init(struct device_node *node)
{
	int retcode = 0;
	unsigned int id = 0;
	struct device_node *child = NULL;
	void __iomem *base;/* virt. address of the control segment */

	for_each_child_of_node(node, child) {

		retcode = (unsigned int) of_property_read_u32(child,
								"phyid",
								&id);

		if (retcode < 0) {
			dev_info(phypriv->dev, "phy id failed");
			goto failed;
		}

		phypriv->phy[id] =
			kzalloc(sizeof(struct phygasket), GFP_KERNEL);

		if (!phypriv->phy[id]) {
			retcode = -ENOMEM;
			dev_info(phypriv->dev, "phy id malloc fail");
			goto failed;
		}

		base = of_iomap(child, 0);

		if (!base) {
			dev_info(phypriv->dev, "phy id iomap");
			retcode = -ENOMEM;
			goto failed;
		}
		phypriv->phy[id]->pregs =
				(struct phy_gasket_regs *)base;
		phypriv->phy[id]->init_flag = PHYGASKET_INIT_SUCCESS;
		phypriv->phy[id]->node = child;
		list_add_tail(&phypriv->phy[id]->list, &phy_list);
	}

failed:
	return retcode;
}

static int __init phygasket_probe(struct platform_device *pdev)
{
	int retcode = -ENODEV;
	struct device_node *node = pdev->dev.of_node;

	phypriv = kzalloc(sizeof(struct phygasket_private), GFP_KERNEL);
	if (phypriv == NULL) {
		retcode = -ENOMEM;
		goto failed0;
	}

	if (!node || !of_device_is_available(node)) {
		retcode = -ENODEV;
		dev_err(&pdev->dev, "phy dev not avaiable");
		goto failed1;
	}
	phypriv->dev = &pdev->dev;
	retcode = phygasket_init(node);

	if (retcode < 0)
		goto failed1;

	dev_info(&pdev->dev, "phygasket probe pass");
	return retcode;
failed1:
	kfree(phypriv);
failed0:
	return retcode;
}


static int __exit phygasket_remove(struct platform_device *pdev)
{
	struct device_node *child = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct resource res;
	struct list_head *cur, *n;
	unsigned int id = 0;
	int retcode = 0;

	list_for_each_safe(cur, n, &phy_list) {
		list_del(&phypriv->phy[id]->list);
	}

	if (phypriv != NULL) {
		for_each_child_of_node(node, child) {
			retcode = (unsigned int) of_property_read_u32(child,
									"phyid",
									&id);
			of_address_to_resource(node, 0, &res);
			kfree(phypriv->phy[id]);
			iounmap(phypriv->phy[id]->pregs);
			release_mem_region(res.start, resource_size(&res));
		}

		kfree(phypriv);
	}
	return retcode;		/* success */
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
	return platform_driver_register(&phygasket_driver);
}

static void __exit phygasket_module_clean(void)
{
	platform_driver_unregister(&phygasket_driver);
}

module_init(phygasket_module_init);
module_exit(phygasket_module_clean);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("phygasket driver");

/*
* src.c
* System reset controller driver. This driver provides API
* for controlling AFD-4400 reset and boot control.
*
* Author: pankaj chauhan <pankaj.chauhan@freescale.com>
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
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <mach/src.h>
#include <mach/src_priv.h>

struct src_priv *src_priv;

static int src_assert_sw_reset(struct src_priv *src_priv, int idx)
{
	int rc = 0, shift;
	u32 *reg, val, tmp;
	struct src_regs *src_regs = src_priv->regs;

	if ((idx < SW_RST_MIN) || (idx > SW_RST_MAX)) {
		dev_err(src_priv->dev, "Invalid SW RST idx %d\n", idx);
		return -EINVAL;
	}

	reg = &src_regs->sscr;
	/* SW_RST1 is at SSCR[8], calculate generic shift */
	shift = (idx - 1) + 8;
	val = 1 << shift;
	iowrite32(val, reg);

	/* SW_RST should complete and get cleared in 45 uSecs, wait
	 * for 100 usecs to check if reset done
	 */
	udelay(100);
	tmp = ioread32(reg);
	if (tmp & val) {
		dev_err(src_priv->dev, "SW RST %d Failed\n", idx);
		rc = -EBUSY;
		goto out;
	}

	dev_dbg(src_priv->dev, "srsr %x\n", ioread32(&src_regs->srsr));
	dev_dbg(src_priv->dev, "SW RST %d done\n", idx);
out:
	iowrite32(val, &src_regs->srsr);
	return rc;
}

int src_assert_reset(void *src_handle, enum rst_type rst_type, int idx)
{
	int rc = 0;
	struct src_priv *src_priv = src_handle;

	switch (rst_type) {
	case SW_RST:
		rc = src_assert_sw_reset(src_priv, idx);
		break;
	default:
		dev_err(src_priv->dev, "Invalid reset type %d\n", rst_type);
		rc = -EINVAL;
		goto out;
	}

out:
	return rc;
}
EXPORT_SYMBOL(src_assert_reset);

void *src_get_handle(struct device_node *src_dev_node)
{
	return src_priv;
}
EXPORT_SYMBOL(src_get_handle);

int of_get_named_src_reset(struct device_node *np,
		struct of_phandle_args *phandle, const char *propname,
		int index)
{
	if (of_parse_phandle_with_args(np, propname, "#src-cells", index,
					 phandle)) {
		return -EINVAL;
	}
	of_node_put(phandle->np);
	return 0;
}
EXPORT_SYMBOL(of_get_named_src_reset);

static int src_of_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *dev_node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;

	src_priv->regs = of_iomap(dev_node, 0);
	if (!src_priv->regs) {
		dev_err(dev, "Failed to iomap src regs\n");
		rc = -ENOMEM;
		goto out;
	}

	src_priv->dev = dev;
	dev_set_drvdata(dev, src_priv);

out:
	return rc;
}

static int src_of_remove(struct platform_device *pdev)
{
	/*Nothing to be done for remove*/
	dev_dbg(&pdev->dev, "Removing SRC driver\n");
	return 0;
}


static struct of_device_id src_match_dts_id[] = {
	{.compatible = "fsl,src-d4400",},
	{},
};

static struct platform_driver src_driver = {
	.driver = {
		.name = "fsl-src",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(src_match_dts_id),
	},
	.probe = src_of_probe,
	.remove = src_of_remove,
};


static int __init src_module_init(void)
{
	int rc;

	src_priv = kzalloc(sizeof(struct src_priv), GFP_KERNEL);

	if (!src_priv) {
		rc = -ENOMEM;
		goto out;
	}

	spin_lock_init(&src_priv->lock);

	return platform_driver_register(&src_driver);
out:
	return rc;
}

static void __exit src_module_cleanup(void)
{
	kfree(src_priv);
	platform_driver_unregister(&src_driver);
}

module_init(src_module_init);
module_exit(src_module_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("System reset controller driver");

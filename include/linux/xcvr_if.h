/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __LINUX_XCVR_IF_H
#define __LINUX_XCVR_IF_H

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/linkage.h>
#include <linux/ipmi-eeprom-util.h>

#define XCVRIF_MOD_NAME "xcvrif"

/* xcvrif_dev->debug */
#define DEBUG_XCVRIF_MESSAGES		(1<<0)
#define DEBUG_XCVRIF_GENERAL		(1<<1)

struct xcvrif_dev {
	int			xcvr_cnt;
	struct device		*dev;
	struct list_head	headlist;

	/* Sysfs support */
	dev_t			devt;
	struct class		*class;
	struct cdev		dev_cdev;
	int			major;
	int			minor;
	atomic_t		ref;
	u32			debug;
};

struct xcvr_i2c_eeprom {
	u8			addr;
	int			pagesize;
	int			bytesize;
	/* Number of bytes for mem address */
	int			addrlen;
	struct i2c_client	*client;
};

struct xcvr_obj {
	int			index;
	const char		*drv_name;
	/* FMC connector: 0-none, 1-fmc1, 2-fmc2, 3-fmc1 & fmc2 */
	int			connector;
	struct xcvrif_dev	*parent;
	struct device		*dev;
	struct platform_device	*pdev;
	struct device_node	*node;
	struct list_head	list;
	struct ipmi_info	*ipmi;
	struct xcvr_i2c_eeprom	*eeprom;
};

#endif /* __LINUX_XCVR_IF_H */

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

#ifndef FSL_D4400_SYS_H
#define FSL_D4400_SYS_H

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/linkage.h>
#include <linux/ipmi-eeprom-util.h>
#include <linux/util-d4400.h>
#include <uapi/linux/fsl-d4400-sys.h>
#include <linux/ipmi-mrec-d4400.h>

/* d4400_sys_dev->debug */
#define DEBUG_D4400_SYS_DEV_MESSAGES		(1<<0)
#define DEBUG_D4400_SYS_DEV_GENERAL		(1<<1)

struct d4400_sys_dev {
	struct device		*dev;
	struct mutex		lock;
	struct ipmi_info	*ipmi;
	struct d4400_sys_i2c_eeprom *eeprom;
	struct d4400_sys_board_info brd_info;
	int			reboot_method;
	void			*brd; /* Board specific data */

	/* Sysfs support */
	dev_t			devt;
	struct class		*class;
	struct cdev		dev_cdev;
	int			major;
	int			minor;
	atomic_t		ref;
	u32			debug;
};

struct d4400_sys_i2c_eeprom {
	u8			addr;
	int			pagesize;
	int			bytesize;
	/* Number of bytes for mem address */
	int			addrlen;
	struct i2c_client	*client;
};

struct d4400_sys_obj {
	int			index;
	const char		*drv_name;

	struct d4400_sys_dev	*parent;
	struct device		*dev;
	struct platform_device	*pdev;
	struct device_node	*node;
	struct ipmi_info	*ipmi;
	struct d4400_sys_i2c_eeprom *eeprom;
};

int d4400_sys_board_type(void);
int d4400_sys_board_rev(void);
int d4400_sys_xcvr_eeprom_power(void);
int d4400_sys_leds_set_clear(unsigned int set, unsigned int clear);
int d4400_sys_set_reboot_method(unsigned int method);

#endif /* FSL_D4400_SYS_H */

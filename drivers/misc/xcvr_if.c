/*
 * File: xcvr_if.c
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
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <mach/src.h>
#include <linux/of_i2c.h>
#include <linux/xcvr_if.h>
#include <linux/qixis.h>

/* Debug and error reporting macros */
#define IF_DEBUG(x) if (xcvrif_dev_data->debug & (x))
#define ERR(...) {if (xcvrif_dev_data->debug & DEBUG_XCVRIF_MESSAGES) \
				pr_err(XCVRIF_MOD_NAME __VA_ARGS__); }

#define xcvr_remove(pxcvr_dev) platform_device_del(pxcvr_dev);

static struct of_device_id xcvrif_match[] = {
	{.compatible = "fsl,xcvr-interface",},
};

struct xcvrif_dev *xcvrif_dev_data;

static int xcvrif_ipmi_str_copy(struct platform_device *pdev,
	char *node_str, char **ipmi_str)
{
	int ret = 0;
	const char *str;

	if (of_property_read_string_index(pdev->dev.of_node, node_str,
		0, &str))
		return -ENODEV;

	*ipmi_str = kzalloc(strlen(str), GFP_KERNEL);

	if (!*ipmi_str)
		return -ENOMEM;
	strcpy(*ipmi_str, str);

	return ret;
}

static int xcvrif_ipmi_match_str(struct platform_device *pdev,
	char *node_str, char *ipmi_str)
{
	int i = 0;
	int matched = 0;
	const char *str;

	do {
		int len;

		if (of_property_read_string_index(pdev->dev.of_node,
			node_str, i, &str))
			break;
		len = strlen(str);

		/* String lengths must match */
		if (len == strlen(ipmi_str)) {
			/* Match strings, case in sensitive */
			if (!strnicmp(str, ipmi_str, len))
				matched = 1;
		}
		++i;
	} while (matched == 0);

	return matched;
}

static struct ipmi_info *xcvrif_verify_ipmi_info(int id,
	struct platform_device *pdev)
{
	int ret = 0, i;
	struct device_node *eeprom_node;
	struct i2c_client *eeprom_client;
	u32 eeprom_addr;
	u8 ipmi_rawbuf[IPMI_EEPROM_DATA_SIZE];
	int matched;
	struct ipmi_info *ipmi = NULL;

	eeprom_node = of_parse_phandle(pdev->dev.of_node, "eeprom-handle", 0);
	if (!eeprom_node) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: eeprom-handle not found\n",
			id);
		goto out;
	}

	eeprom_client = of_find_i2c_device_by_node(eeprom_node);
	if (!eeprom_client) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to find xcvr eeprom i2c device handle\n",
			id);
		goto out;
	}

	if (of_property_read_u32(pdev->dev.of_node, "eeprom-addr",
		&eeprom_addr)) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: eeprom-addr property not found\n",
			id);
		goto out;
	}
	eeprom_client->addr = eeprom_addr;

	/* Read IPMI raw data */
	memset(ipmi_rawbuf, 0, IPMI_EEPROM_DATA_SIZE);
	for (i = 0; i < IPMI_EEPROM_DATA_SIZE; i += I2C_SMBUS_BLOCK_MAX) {
		int rd;
		rd = i2c_smbus_read_i2c_block_data(eeprom_client, i,
			I2C_SMBUS_BLOCK_MAX, &ipmi_rawbuf[i]);
		if (rd != I2C_SMBUS_BLOCK_MAX) {
			/* pr_err("%i IPMI: Failed to read xcvr eeprom\n", id); */
			goto out;
		}
	}
	ipmi = kzalloc(sizeof(struct ipmi_info), GFP_KERNEL);
	if (!ipmi) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to allocate %d bytes for IPMI info\n",
			id, sizeof(struct ipmi_info));
		goto out;
	}

	/* Create IPMI info */
	ret = ipmi_create(ipmi_rawbuf, ipmi);
	if (ret) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to find valid IPMI information\n",
			id);
		goto out;
	}
	pr_info("IPMI Xcvr%i board mfg   :  %s\n",
		id, ipmi->board.mfg_str);
	pr_info("IPMI Xcvr%i board name  :  %s\n",
		id, ipmi->board.name_str);

	matched = xcvrif_ipmi_match_str(pdev, "ipmi-mfg-str",
		ipmi->board.mfg_str);
	if (!matched) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Info valid but no mfg name match found in dts\n",
			id);
		goto out;
	}
	matched = xcvrif_ipmi_match_str(pdev, "ipmi-name-str",
		ipmi->board.name_str);
	if (!matched) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Info valid but no board name match found in dts\n",
			id);
		goto out;
	}
	return ipmi;

out:
	ipmi_free(ipmi);
	kfree(ipmi);
	return NULL;
}

static struct ipmi_info *xcvrif_manual_detect(int id,
	struct platform_device *pdev)
{
	int ret;
	u32 fmc_conn;
	struct ipmi_info *ipmi = NULL;

	if (of_property_read_u32(pdev->dev.of_node, "connector-fmc",
		&fmc_conn)) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: connector-fmc property not found\n",
			id);
		goto out0;
	}

	/* FMC connector number starts at 1, do -1 for zero based numbering.
	 * Possible values for presents detect is 0 (fmc1) or 1 (fmc2).
	 * For wideband board which uses two fmc connectors and have fmc value
	 * of 3, presents detect will fail.  This is OK because EVB revA does
	 * not support wideband board.
	 */
	if (!qixis_xcvr_present(fmc_conn-1))
		goto out0;

	/* A card is present in connector, manually filled IPMI info */
	ipmi = kzalloc(sizeof(struct ipmi_info), GFP_KERNEL);
	if (!ipmi) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to allocate %d bytes for IPMI info\n",
			id, sizeof(struct ipmi_info));
		goto out0;
	}
	ret = xcvrif_ipmi_str_copy(pdev, "ipmi-mfg-str", &ipmi->board.mfg_str);
	ret |= xcvrif_ipmi_str_copy(pdev, "ipmi-name-str", &ipmi->board.name_str);

	if (ret)
		goto out1;

	return ipmi;
out1:
	ipmi_free(ipmi);
	kfree(ipmi);
out0:
	return NULL;
}

static void xcvrif_list_remove(struct list_head *headlist)
{
	struct xcvr_obj *cur, *nxt;

	list_for_each_entry_safe(cur, nxt, headlist, list) {
		list_del(&cur->list);
		/* Free xcvr dev while we are at it. */
		ipmi_free(cur->ipmi);
		kfree(cur);
	}
}

struct xcvr_obj *xcvrif_add_dev(struct platform_device *pxcvr_dev,
	struct device_node *xcvr_node, struct xcvrif_dev *parent,
	struct ipmi_info *ipmi, int index)
{
	struct xcvr_obj *xcvr = NULL;

	if (pxcvr_dev) {
		xcvr = kzalloc(sizeof(struct xcvr_obj), GFP_KERNEL);
		if (!xcvr)
			goto out1;

		if (of_property_read_u32(xcvr_node, "connector-fmc",
			&xcvr->connector)) {
			pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Can't find connector information\n",
				index);
			goto out0;
		}
		xcvr->index = index;
		xcvr->node = xcvr_node;
		xcvr->pdev = pxcvr_dev;
		xcvr->parent = parent;
		xcvr->ipmi = ipmi;
	}
out1:
	return xcvr;
out0:
	kfree(xcvr);
	return NULL;
}

/* Called when apps perform an open on /sys/class/xcvrif */
static int xcvrif_open(struct inode *inode, struct file *pfile)
{
	pfile->private_data = xcvrif_dev_data;
	atomic_inc(&xcvrif_dev_data->ref);
	return 0;
}

static ssize_t xcvrif_read(struct file *filep, char __user *buf, size_t size,
			loff_t *offset)
{
	int ret = 0;
	ret = put_user(0, (int *)buf);
	return ret;
}

int xcvrif_release(struct inode *inode, struct file *pfile)
{
	atomic_dec(&xcvrif_dev_data->ref);
	return 0;
}

/* File ops for sysfs */
static const struct file_operations xcvrif_fops = {
	.owner = THIS_MODULE,
	.open = xcvrif_open,
	.read = xcvrif_read,
	.release = xcvrif_release,
};

static ssize_t set_debug(struct device *dev,
	struct device_attribute *devattr,
	const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	xcvrif_dev_data->debug = val;
	return count;
}

static ssize_t show_debug(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d / 0x%x\n", xcvrif_dev_data->debug,
		xcvrif_dev_data->debug);
}

static ssize_t show_xcvr_cnt(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", xcvrif_dev_data->xcvr_cnt);
}

static ssize_t show_ipmi_info(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct xcvr_obj *cur, *nxt;
	int i = 0;
	char *p = buf;

	list_for_each_entry_safe(cur, nxt, &xcvrif_dev_data->headlist, list) {
		sprintf(p, "Xcvr %i:\tMfg: %s\n\tName: %s\n\tSerial: %s\n\tPartnum: %s\n",
			cur->index,
			cur->ipmi->board.mfg_str,
			cur->ipmi->board.name_str,
			cur->ipmi->board.serial_str,
			cur->ipmi->board.partnum_str);
		p += strlen(p);
		++i;
	}
	return strlen(buf);
}

static DEVICE_ATTR(debug,	S_IWUSR | S_IRUGO, show_debug, set_debug);
static DEVICE_ATTR(xcvr_cnt,	S_IRUGO, show_xcvr_cnt, NULL);
static DEVICE_ATTR(ipmi_info,	S_IRUGO, show_ipmi_info, NULL);

static struct attribute *attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_xcvr_cnt.attr,
	&dev_attr_ipmi_info.attr,
	NULL
};
static const struct attribute_group attr_group = {
	.attrs = attributes,
};

static int xcvrif_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct xcvrif_dev *xcvrif_dev_data = dev_get_drvdata(&pdev->dev);

	sysfs_remove_group(&pdev->dev.kobj, &attr_group);
	device_destroy(xcvrif_dev_data->class, xcvrif_dev_data->devt);
	cdev_del(&xcvrif_dev_data->dev_cdev);
	class_destroy(xcvrif_dev_data->class);
	unregister_chrdev_region(xcvrif_dev_data->devt, 1);

	xcvrif_list_remove(&xcvrif_dev_data->headlist);
	kfree(xcvrif_dev_data);
	return ret;
}

static int xcvr_add_dev(int id, const char *platform_drv_name,
	struct device_node *child)
{
	int ret = 0;
	int use_ipmi = 0;
	struct platform_device *pxcvr_dev = NULL;
	struct xcvr_obj *xcvr_new;
	struct ipmi_info *ipmi = NULL;

	/* platform_drv_name must contain a string that matches with
	 * the driver's platform_driver.driver.name property.  The
	 * driver's probe function is called by platform_device_add().
	 */
	pxcvr_dev = platform_device_alloc(platform_drv_name, id);
	if (pxcvr_dev) {
		pxcvr_dev->dev.of_node = child;

		/* Verify IPMI info only if xcvr eeprom is powered which is
		 * dependent on board rev.
		 */
		if (qixis_xcvr_eeprom_power()) {
			ipmi = xcvrif_verify_ipmi_info(id, pxcvr_dev);
		} else {
			/* Using fallback method to detect xcvr card by
			 * gpio signal detection.
			 */
			ipmi = xcvrif_manual_detect(id, pxcvr_dev);
			use_ipmi = 1;
		}

		if (!ipmi) {
			/* Do not print error mesg because the xcvr card may not be
			 * installed and thus eeprom read failed.  This is not an
			 * actual error.
			 */
			ret = -ENODEV;
			goto out_pdev;
		}
		if (!platform_device_add(pxcvr_dev)) {
			/* Init the device data */
			xcvr_new = xcvrif_add_dev(pxcvr_dev, child,
				xcvrif_dev_data, ipmi, id);
			if (!xcvr_new) {
				pr_err(XCVRIF_MOD_NAME ": Error adding xcvr device\n");
				ret = -ENODEV;
				goto out_ipmi;
			}
			xcvr_new->drv_name = platform_drv_name;
			list_add_tail(&xcvr_new->list,
				&xcvrif_dev_data->headlist);
			if (!use_ipmi)
				pr_info(XCVRIF_MOD_NAME ": Xcvr%i IPMI data matched, driver loaded\n",
					id);
			else
				pr_info(XCVRIF_MOD_NAME ": Xcvr%i card detected, driver loaded\n",
					id);
		}
	} else {
		ret = -ENOMEM;
		goto out;
	}
	return ret;

out_ipmi:
	ipmi_free(ipmi);
out_pdev:
	platform_device_put(pxcvr_dev);
out:
	return ret;
}

static int xcvrif_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;

	int i = 0;
	int cnt = 0;
	struct device_node *child = NULL;
	const char *platform_drv_name = NULL;
	struct device *sysfs_dev;


	if (!np || !of_device_is_available(np)) {
		pr_err(XCVRIF_MOD_NAME ": No XCVR interface device available\n");
		return -ENODEV;
	}

	xcvrif_dev_data = kzalloc(sizeof(struct xcvrif_dev), GFP_KERNEL);
	if (!xcvrif_dev_data) {
		pr_err(XCVRIF_MOD_NAME ": Failed to allocate device data\n");
		ret = -ENOMEM;
		goto out;
	}
	xcvrif_dev_data->dev = dev;
	INIT_LIST_HEAD(&xcvrif_dev_data->headlist);

	for_each_child_of_node(pdev->dev.of_node, child) {

		if (of_property_read_string(child, "platform-drv-name",
			&platform_drv_name)) {
			/* Not a xcvr node, skip */
			continue;
		}

		if (!xcvr_add_dev(cnt, platform_drv_name, child))
			++cnt;
		++i;
	}

	/* Save count of transceiver nodes detected */
	xcvrif_dev_data->xcvr_cnt = cnt;

	/* Create char device */
	ret = alloc_chrdev_region(&xcvrif_dev_data->devt, 0, 1,
		XCVRIF_MOD_NAME);
	if (ret) {

		ret = -ENOMEM;
		goto out;
	}
	xcvrif_dev_data->major = MAJOR(xcvrif_dev_data->devt);
	xcvrif_dev_data->minor = MINOR(xcvrif_dev_data->devt);

	/* Create device class for sysfs */
	xcvrif_dev_data->class = class_create(THIS_MODULE, XCVRIF_MOD_NAME);
	if (IS_ERR(xcvrif_dev_data->class)) {
		ret = PTR_ERR(xcvrif_dev_data->class);
		goto out_class;
	}

	/* Create character device */
	cdev_init(&xcvrif_dev_data->dev_cdev, &xcvrif_fops);
	xcvrif_dev_data->dev_cdev.owner = THIS_MODULE;
	ret = cdev_add(&xcvrif_dev_data->dev_cdev, xcvrif_dev_data->devt, 1);
	if (ret) {
		pr_err(XCVRIF_MOD_NAME ": Error %d while adding cdev",
			ret);
		goto out_cdev;
	}

	/* Create the device node in /dev */
	sysfs_dev = device_create(xcvrif_dev_data->class, &pdev->dev,
		xcvrif_dev_data->devt,	NULL, XCVRIF_MOD_NAME);
	if (NULL == sysfs_dev) {
		ret = PTR_ERR(sysfs_dev);
		pr_err(XCVRIF_MOD_NAME ": Error %d while creating device",
			 ret);
		goto out_dev;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret < 0) {
		pr_err(XCVRIF_MOD_NAME ": Error %d while creating group",
			ret);
		goto out_group;
	}

	dev_set_drvdata(&pdev->dev, xcvrif_dev_data);

	return ret;

out_group:
	device_destroy(xcvrif_dev_data->class, xcvrif_dev_data->devt);
out_dev:
	cdev_del(&xcvrif_dev_data->dev_cdev);
out_cdev:
	class_destroy(xcvrif_dev_data->class);
out_class:
	unregister_chrdev_region(xcvrif_dev_data->devt, 1);
out:
	if (xcvrif_dev_data->xcvr_cnt)
		xcvrif_list_remove(&xcvrif_dev_data->headlist);
	kfree(xcvrif_dev_data);
	return ret;
}

static struct platform_driver xcvrif_driver = {
	.driver = {
		.name = "xcvr_interface_driver",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xcvrif_match),
		},
	.probe = xcvrif_probe,
	.remove = xcvrif_remove,
};

static int __init xcvrif_init(void)
{
	int ret = 0;
	ret = platform_driver_register(&xcvrif_driver);
	return ret;
}

static void __exit xcvrif_exit(void)
{
	platform_driver_unregister(&xcvrif_driver);
}

module_init(xcvrif_init);
module_exit(xcvrif_exit);

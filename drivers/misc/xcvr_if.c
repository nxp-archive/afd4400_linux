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
#include <linux/qixis.h>
#include <linux/xcvr_if.h>
#include <uapi/linux/xcvr_if.h>

/* Debug and error reporting macros */
#define IF_DEBUG(x) if (xcvrif_dev_data->debug & (x))
#define ERR(...) {if (xcvrif_dev_data->debug & DEBUG_XCVRIF_MESSAGES) \
				pr_err(XCVRIF_MOD_NAME __VA_ARGS__); }

#define xcvr_remove(pxcvr_dev) platform_device_del(pxcvr_dev);

static struct of_device_id xcvrif_match[] = {
	{.compatible = "fsl,xcvr-interface",},
};

struct xcvrif_dev *xcvrif_dev_data;
static int generate_ipmi_info_str(char *buf, int xcvr_index);

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

static struct xcvr_i2c_eeprom *xcvrif_get_eeprom_info(
	int id, struct platform_device *pdev)
{
	struct device_node *eeprom_node;
	struct i2c_client *eeprom_client;
	struct xcvr_i2c_eeprom *eeprom;
	u32 addr;
	u32 pagesize;
	u32 bytesize;
	u32 addrlen;

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
		&addr)) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: eeprom-addr property not found\n",
			id);
		goto out;
	}

	if (of_property_read_u32(pdev->dev.of_node, "eeprom-bytesize",
		&bytesize)) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: eeprom-bytesize property not found\n",
			id);
		goto out;
	}

	if (of_property_read_u32(pdev->dev.of_node, "eeprom-pagesize",
		&pagesize)) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: eeprom-bytesize property not found\n",
			id);
		goto out;
	}

	/* Determine how many address bytes are needed: 1 or 2 bytes.
	 * Note that large eeproms that need more than 16 address bits
	 * use bits in the control byte (i2c slave address byte) to extend
	 * the address bits.  For example, 128K byte eeprom (17 addr bits
	 * needed) uses 2 byte address plus 1 bit in the control reg.
	 */
	if (bytesize > 256)
		/* Two addr bytes needed */
		addrlen = 2;
	else
		addrlen = 1;

	eeprom = kzalloc(sizeof(struct xcvr_i2c_eeprom), GFP_KERNEL);
	if (!eeprom) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to allocate %d bytes for eeprom info\n",
			id, sizeof(struct xcvr_i2c_eeprom));
		goto out;
	}
	eeprom->client = eeprom_client;
	eeprom->addr = addr;
	eeprom->pagesize = pagesize;
	eeprom->bytesize = bytesize;
	eeprom->addrlen = addrlen;

	return eeprom;
out:
	return NULL;
}

static int eeprom_read(struct xcvr_i2c_eeprom *eeprom,
	u8 *buf, int start_addr, int num)
{
	int ret = 0;
	struct i2c_msg msgs[2];
	int addr;
	int eeprom_addr = eeprom->addr;

	/* TODO: Add ability to do page boundary crossing such as
	 * r/w from one 64K page that extends to the next 64K page.
	 */

	if ((!eeprom) || (!buf))
		return ret;

	if (eeprom->addrlen > 1) {
		/* Major mfg of eeproms expect MSB of 16-bit address first
		 * followed by LSB.  Any address bits beyond 16-bit is put
		 * in the slave address byte (bits b[1:0] location).
		 */
		addr  = (start_addr & 0xff00) >> 8;
		addr |= (start_addr & 0x00ff) << 8;

		if (eeprom->bytesize > (64 * 1024)) {
			/* Slave addr b[0] is addr bit A17 */
			eeprom_addr &= ~0x01;
			eeprom_addr |= ((start_addr & 0x10000) >> 16);
		}
		if (eeprom->bytesize > (256 * 1024)) {
			/* Slave addr b[1] is addr bit A18 */
			eeprom_addr &= ~0x02;
			eeprom_addr |= ((start_addr & 0x20000) >> 16);
		}
	} else
		addr = start_addr & 0x00ff;

	/* Write the address to read */
	msgs[0].addr = eeprom_addr;
	msgs[0].flags = 0;
	msgs[0].len = eeprom->addrlen;
	msgs[0].buf = (u8 *)&addr; /* Addr msb sent first */

	/* Read the entire IPMI info */
	msgs[1].addr = eeprom_addr;
	msgs[1].flags = I2C_M_RD,
	msgs[1].len = num,
	msgs[1].buf = buf,

	/* Number of msgs processed is returned, neg if error */
	ret = i2c_transfer(eeprom->client->adapter, msgs, 2);
	if (ret != 2)
		ret = -ENODEV;
	else
		ret = 0;
	return ret;
}

/* Get the eeprom offset address.  Part of this address could
 * reside in the i2c slave address itself if the eeprom size is
 * larger than a 16-bit address can hold.
 */
static void eeprom_get_addr(int bytesize, int start_addr, int *addr,
	int *eeprom_addr)
{
	/* Major mfg of eeproms expect MSB of 16-bit address first
	 * followed by LSB.  Any address bits beyond 16-bit is put
	 * in the slave address byte (bits b[1:0] location).
	 */
	*addr  = (start_addr & 0xff00) >> 8;
	*addr |= (start_addr & 0x00ff) << 8;

	if (bytesize > (64 * 1024)) {
		/* Slave addr b[0] is addr bit A17 */
		*eeprom_addr &= ~0x01;
		*eeprom_addr |= ((start_addr & 0x10000) >> 16);
	}
	if (bytesize > (256 * 1024)) {
		/* Slave addr b[1] is addr bit A18 */
		*eeprom_addr &= ~0x02;
		*eeprom_addr |= ((start_addr & 0x20000) >> 16);
	}
}

static int eeprom_write(struct xcvr_i2c_eeprom *eeprom,
	u8 *buf, int start_addr, int num)
{
	int ret = 0;
	struct i2c_msg msgs[2];
	struct i2c_msg msgschk[1];
	int addr;
	int eeprom_addr = eeprom->addr;
	u8 *tmpbuf, *src, *dest;
	int i, shift;
	int allocsize;
	int left;
	int cur_addr;
	int data_size;
	int nextpageaddr;

	/* TODO: Add ability to do page boundary crossing such as
	 * r/w from one 64K page that extends to the next 64K page.
	 */

	if ((!eeprom) || (!buf))
		return ret;

	/* Temp buffer size is up to max of eeprom page size plus offset
	 * address bytes.
	 */
	allocsize = eeprom->addrlen;
	if (num > eeprom->pagesize)
		allocsize += eeprom->pagesize;
	else
		allocsize += num;

	/* Temp buffer */
	tmpbuf = kzalloc(allocsize, GFP_KERNEL);
	if (!tmpbuf)
		return -ENOMEM;

	/* Used to check for NACK after write to see when eeprom
	 * is done writing. */
	msgschk[0].addr = eeprom->addr;
	msgschk[0].flags = 0;
	msgschk[0].len = 0;
	msgschk[0].buf = tmpbuf;

	/* Write one chunk (up to pagesize) to eeprom */
	src = buf;
	left = num;
	cur_addr = start_addr;
	while (left > 0) {

		/* Calculate address.  Note that the LSB of 'addr' contains
		 * the MSB of the offset address.  For example for 16-bit
		 * offset address:
		 *   addr[7:0]  = cur_addr[15:8]
		 *   addr[15:8] = cur_addr[7:0]
		 */
		eeprom_addr = eeprom->addr;
		eeprom_get_addr(eeprom->bytesize, cur_addr, &addr,
			&eeprom_addr);

		/* Limit the write size to one page */
		if (left > eeprom->pagesize)
			data_size = eeprom->pagesize;
		else
			data_size = left;

		/* Make sure we don't cross page boundary */
		nextpageaddr = (cur_addr & ~(eeprom->pagesize-1))
			+ eeprom->pagesize;
		if ((cur_addr + data_size) > nextpageaddr)
			data_size = (nextpageaddr - cur_addr);

		/* Reset */
		dest = tmpbuf;

		/* Put in the address bytes, MSB first */
		for (i = 0; i < eeprom->addrlen; ++i) {
			shift = (8 * i);
			*dest++ = (u8)((addr >> shift) & 0xff);
		}

		/* Data */
		for (i = 0; i < data_size; ++i)
			*dest++ = *src++;

		/* Write: start with address byte(s) followed by data. */
		msgs[0].addr = eeprom_addr;
		msgs[0].flags = 0;
		msgs[0].len = eeprom->addrlen + data_size;
		msgs[0].buf = tmpbuf;

		/* Number of msgs processed is returned, neg if error */
		ret = i2c_transfer(eeprom->client->adapter, msgs, 1);
		if (ret != 1)
			ret = -ENODEV;
		/* Reset */
		ret = 0;

		/* Do dummy read to see when device is done writing.  We'll get
		 * NACKed until then.  Wait until read succeeds or timed out,
		 */
		i = 0;
		while (i2c_transfer(eeprom->client->adapter, msgschk, 1) != 1) {
			++i;
			if (i > 100) {
				ret = -EIO;
				break;
			}
			mdelay(1);
		}
		if (ret)
			break;

		cur_addr += data_size;
		left -= data_size;
	}

	kfree(tmpbuf);

	return ret;
}

static struct ipmi_info *xcvrif_verify_ipmi_info(int id,
	struct platform_device *pdev,
	struct xcvr_i2c_eeprom *eeprom)
{
	int ret = 0;
	int matched;
	u8 ipmi_rawbuf[IPMI_EEPROM_DATA_SIZE];
	struct ipmi_info *ipmi = NULL;

	memset(ipmi_rawbuf, 0, IPMI_EEPROM_DATA_SIZE);
	ret = eeprom_read(eeprom, ipmi_rawbuf, 0, IPMI_EEPROM_DATA_SIZE);
	if (ret)
		goto out0;

	ipmi = kzalloc(sizeof(struct ipmi_info), GFP_KERNEL);
	if (!ipmi) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to allocate %d bytes for IPMI info\n",
			id, sizeof(struct ipmi_info));
		goto out0;
	}

	/* Create IPMI info */
	ret = ipmi_create(ipmi_rawbuf, ipmi);
	if (ret) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to find valid IPMI information\n",
			id);
		goto out1;
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
		goto out1;
	}
	matched = xcvrif_ipmi_match_str(pdev, "ipmi-name-str",
		ipmi->board.name_str);
	if (!matched) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Info valid but no board name match found in dts\n",
			id);
		goto out1;
	}
	return ipmi;

out1:
	ipmi_free(ipmi);
	kfree(ipmi);
out0:
	return NULL;
}

static struct ipmi_info *xcvrif_manual_detect(int id,
	struct platform_device *pdev)
{
	int ret;
	u32 fmc_conn, fmc_occupied;
	struct ipmi_info *ipmi = NULL;

	if (of_property_read_u32(pdev->dev.of_node, "connector-fmc",
		&fmc_conn)) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: connector-fmc property not found\n",
			id);
		goto out0;
	}

	/* FMC connector number starts at 1, do -1 for zero based numbering.
	 * Possible values for presents detect is 0 (fmc1) or 1 (fmc2).
	 */
	fmc_occupied =  (qixis_xcvr_present(0) << 0);
	fmc_occupied |= (qixis_xcvr_present(1) << 1);

	/* Possible values for connector-fmc is: 1-fmc1, 2-fmc2,
	 * 3-fmc1 and fmc2. If card requires both connectors, then skip since
	 * revA does not support anything that occupies two connectors at the
	 * same time.  This implies two ADI cards are not supported.  Jesd
	 * loopback board occupies two fmc also but no driver is needed.
	 */
	if ((fmc_conn >= 3) || (fmc_occupied == 3))
		goto out0;

	if (!(fmc_conn & fmc_occupied))
		goto out0;

	/* A card is present in connector, manually filled IPMI info */
	ipmi = kzalloc(sizeof(struct ipmi_info), GFP_KERNEL);
	if (!ipmi) {
		pr_err(XCVRIF_MOD_NAME ": IPMI Xcvr%i: Failed to allocate %d bytes for IPMI info\n",
			id, sizeof(struct ipmi_info));
		goto out0;
	}
	ret = xcvrif_ipmi_str_copy(pdev, "ipmi-mfg-str", &ipmi->board.mfg_str);
	ret |= xcvrif_ipmi_str_copy(pdev, "ipmi-name-str",
		&ipmi->board.name_str);

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

struct xcvr_obj *xcvrif_add_dev(int index,
	struct platform_device *pxcvr_dev,
	struct device_node *xcvr_node,
	struct xcvrif_dev *parent,
	struct ipmi_info *ipmi,
	struct xcvr_i2c_eeprom *eeprom)
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
		xcvr->eeprom = eeprom;
	}
out1:
	return xcvr;
out0:
	kfree(xcvr);
	return NULL;
}

struct xcvr_obj *get_xcvr_obj(int index)
{
	struct xcvr_obj *cur = NULL;
	struct xcvr_obj *nxt = NULL;
	int i = 0;

	list_for_each_entry_safe(cur, nxt, &xcvrif_dev_data->headlist, list) {
		if (i == index)
			break;
		++i;
	}
	return cur;
}

int xcvrif_ioctl_rw_eeprom(struct xcvr_obj *xobj, struct xcvr_eeprom *xee)
{
	int ret = 0;
	u8 *buf;

	/* Boundary check */
	if ((xee->offset >= xobj->eeprom->bytesize) ||
		((xee->offset+xee->len) > xobj->eeprom->bytesize) ||
		(xee->len == 0)) {
		ret = -EINVAL;
		goto out;
	}

	/* Need kernel buffer for rw */
	buf = kzalloc(xee->len, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	if (xee->rw_flag == EEPROM_READ_OP) {

		/* Do the read */
		ret = eeprom_read(xobj->eeprom, buf, xee->offset, xee->len);
		if (!ret) {
			/* Copy to user buffer */
			if (copy_to_user((u8 *)xee->user_buf, buf, xee->len))
				ret = -EFAULT;
		}
	} else if (xee->rw_flag == EEPROM_WRITE_OP) {

		/* Copy buffer to kernel space */
		if (copy_from_user(buf, xee->user_buf, xee->len)) {
			ret = -EFAULT;
			goto out0;
		}

		/* Do the write */
		ret = eeprom_write(xobj->eeprom, buf, xee->offset, xee->len);
	} else
		ret = -EINVAL;

out0:
	kfree(buf);
out:
	return ret;
}

static long xcvrif_ioctl(struct file *pfile, unsigned int cmd,
		unsigned long arg)
{
	int ret  = 0;
	void __user *argp = (void __user *)arg;
	struct xcvr_obj *xobj;

	switch (cmd) {
	case XCVRIF_READ_SYS_INFO:
	{
		struct xcvr_sys_info xsi;

		xsi.xcvr_cnt = xcvrif_dev_data->xcvr_cnt;

		if (copy_to_user((struct xcvr_sys_info *)argp,
				&xsi, sizeof(struct xcvr_sys_info)))
			ret = -EFAULT;
		break;
	}
	case  XCVRIF_READ_XCVR_INFO:
	{
		struct xcvr_info xi;
		char *buf;
		int len;

		if (copy_from_user(&xi, (struct xcvr_info *)argp,
				sizeof(struct xcvr_info))) {
			ret = -EFAULT;
			goto out;
		}
		/* Get requested xcvr */
		xobj = get_xcvr_obj(xi.index);
		if (!xobj) {
			ret = -EFAULT;
			goto out;
		}
		/* Need kernel space buffer */
		buf = kzalloc(XCVERIF_IPMI_STR_SIZE, GFP_KERNEL);
		if (!buf) {
			ret = -ENOMEM;
			goto out;
		}
		/* Generate ipmi info in text format */
		len = generate_ipmi_info_str(buf, xi.index);
		if (len == 0) {
			ret = -EFAULT;
			goto out_read_xcvr;
		}
		if (copy_to_user((u8 *)xi.user_ipmi_str, buf, len)) {
			ret = -EFAULT;
			goto out_read_xcvr;
		}
		if (copy_to_user((u8 *)xi.eeprom_size,
			&xobj->eeprom->bytesize, sizeof(int))) {
			ret = -EFAULT;
			goto out_read_xcvr;
		}
		if (copy_to_user((u8 *)xi.eeprom_pagesize,
			&xobj->eeprom->pagesize, sizeof(int))) {
			ret = -EFAULT;
			goto out_read_xcvr;
		}
out_read_xcvr:
		kfree(buf);
		break;
	}
	case XCVRIF_RW_EEPROM:
	{
		struct xcvr_eeprom xee;

		if (copy_from_user(&xee, (struct xcvr_eeprom *)argp,
				sizeof(struct xcvr_eeprom))) {
			ret = -EFAULT;
			goto out;
		}

		/* Get requested xcvr */
		xobj = get_xcvr_obj(xee.index);
		if (!xobj) {
			ret = -EFAULT;
			goto out;
		}

		/* Do the read/write */
		ret = xcvrif_ioctl_rw_eeprom(xobj, &xee);
		break;
	}
	default:
		ret = -ENOTTY;
	}
out:
	return ret;
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
	.unlocked_ioctl = xcvrif_ioctl,
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

/* If xcvr_index set to -1, print info for all transceiver */
static int generate_ipmi_info_str(char *buf, int xcvr_index)
{
	struct xcvr_obj *cur, *nxt;
	int i = 0;
	int j;
	char *p = buf;

	list_for_each_entry_safe(cur, nxt, &xcvrif_dev_data->headlist, list) {
		if ((i == xcvr_index) || (xcvr_index == -1)) {
			sprintf(p, "Xcvr %i:\tMfg: %s\n\tName: %s\n\tSerial: %s\n\tPartnum: %s\n",
				cur->index,
				cur->ipmi->board.mfg_str,
				cur->ipmi->board.name_str,
				cur->ipmi->board.serial_str,
				cur->ipmi->board.partnum_str);
			p += strlen(p);

			sprintf(p, "\tMultirecords: %i  size: %i\n",
				cur->ipmi->multirec.num_rec,
				cur->ipmi->multirec.size);
			p += strlen(p);
			for (j = 0; j < cur->ipmi->multirec.num_rec; ++j) {
				ipmi_printstr_record_hdr(&p,
					cur->ipmi->multirec.rechdr[j]);
			}
			sprintf(p, "\n");
			p += strlen(p);
		}
		++i;
	}
	return strlen(buf);
}

static ssize_t show_ipmi_info(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return generate_ipmi_info_str(buf, -1); /* Print all transceiver */
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
	struct xcvr_i2c_eeprom *eeprom = NULL;

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
			/* Get properties of the eeprom */
			eeprom = xcvrif_get_eeprom_info(id, pxcvr_dev);
			if (!eeprom)
				goto out_pdev;
			ipmi = xcvrif_verify_ipmi_info(id, pxcvr_dev, eeprom);
		} else {
			/* Using fallback method to detect xcvr card by
			 * gpio signal detection.
			 */
			ipmi = xcvrif_manual_detect(id, pxcvr_dev);
			use_ipmi = 1;
		}

		if (!ipmi) {
			/* Do not print error mesg because the xcvr card may
			 * not be installed and thus eeprom read failed.  This
			 * is not an actual error.
			 */
			ret = -ENODEV;
			goto out_pdev;
		}
		if (!platform_device_add(pxcvr_dev)) {
			/* Init the device data */
			xcvr_new = xcvrif_add_dev(id, pxcvr_dev, child,
				xcvrif_dev_data, ipmi, eeprom);
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

		if (*platform_drv_name == '\0')
			/* Platform drv name is empty, skip */
			continue;

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

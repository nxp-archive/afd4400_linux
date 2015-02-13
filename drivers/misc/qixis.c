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
#include <linux/cdev.h>
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
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/spi/spi.h>

#include <linux/qixis.h>
#include <mach/mach-d4400.h>

#define DRIVER_NAME "d4400-fpga"
#define QIXIS_DEVICE_NAME "qixis"

/* QIXIS FPGA register numbers */
#define QIXIS_ID		0x00
#define QIXIS_EVBBRD_VER	0x01
#define QIXIS_FPGA_MAJOR_VER	0x02
#define QIXIS_FPGA_MINOR_VER	0x03
#define QIXIS_LED_CONTROL	0x0E
#define QIXIS_CLK_JCPLL_STATUS	0x33
#define QIXIS_CLK_JCPLL_ADDH	0x34
#define QIXIS_CLK_JCPLL_ADDL	0x35
#define QIXIS_CLK_JCPLL_DATA	0x36
#define QIXIS_STAT_PRESENT	0x0B
/* CLK_JCPLL_STATUS JCPLL status bits */
#define JCPLL_LOCKED	0x80
/* CLK_JCPLL_ADDH upper nibble */
#define READ_NIBBLE     0xC0
#define WRITE_NIBBLE    0x80
#define APPLY_STATUS    0x80


/* Time for the SPI transaction to the JCPLL to complete */
#define MAX_DELAY       2	/* in millisecs */
#define TRY_COUNT       5

/* AD9525 JCPLL registers and bitfields */
#define JCPLL_SPI_MODE_REG      0x000
#define COMMS_LSB_FIRST 0x81
#define JCPLL_READBACK_CTL_REG  0x004
#define READ_ACTV_REGS  0x01
#define JCPLL_N_DIVIDER_REG     0x014
#define BYPASS_DIVIDERS 0x38
#define JCPLL_REFC_REG          0x016
#define DISABLE_REFC    0x00
#define ENABLE_REFC     0x80
#define JCPLL_STATUS_PIN_REG    0x017
#define VDD_CP_NORMAL   0x2D
#define VDD_CP_BY_HALF  0xAD
#define JCPLL_REF_MON_PIN_REG   0x018
#define REF_MON_DLD     0x0D
#define JCPLL_PLL_READBACK_REG  0x01F
#define DIG_LOCK_DETECT 0x01
#define JCPLL_PWR_DWN_REG       0x230
#define PWR_DWN_PLL     0x01
#define PWR_ENB_PLL     0x00
#define JCPLL_IO_UPDATE_REG     0x232
#define UPDATE_IO       0x01

/* Device major/minor numbers */
static s32 qixis_major;
static s32 qixis_minor;

static void __iomem *qixis_base;
static struct cdev qixis_cdev;
static struct class *qixis_class;
static struct qixis_board_info qixis_board_info;
static unsigned int searched;

static unsigned int gpio_prsntb1;
static unsigned int gpio_prsntb2;

static struct device_node *jcpll_node;
static struct spi_device *jcpll_dev;

static struct mutex qixis_lock;

static const char * const board_names[] = {
	"UNKNOWN", "D4400_EVB", "D4400_RDB"
};

static int qixis_jcpll_init(void);

static int match_spi_device_from_np(struct device *dev, void *data)
{
	struct device_node *np = dev->of_node;
	return (np == data) ? 1 : 0;
}

static struct spi_device *find_spi_device_from_np(struct device_node *np)
{
	struct device *dev;

	dev = bus_find_device(&spi_bus_type, NULL, np,
				match_spi_device_from_np);
	return dev ? to_spi_device(dev) : NULL;
}

static void qixis_write(u8 offset, u8 val)
{
	__raw_writeb(val, (qixis_base + offset));
}

static u8 qixis_read(u8 offset)
{
	return __raw_readb(qixis_base + offset);
}

static int qixis_wait_for_jcpll_ready(void)
{
	int n = TRY_COUNT;

	if (!qixis_base)
		return 0;

	/* wait until previous transaction is completed */
	/* try n times only */
	while ((qixis_read(QIXIS_CLK_JCPLL_ADDH) & APPLY_STATUS) && --n)
		schedule_timeout_interruptible(msecs_to_jiffies(MAX_DELAY));

	return n;
}

static int qixis_jcpll_write_spi(u8 val, u16 offset)
{
	int err;
	u8 tx_buf[3];
	struct spi_message m;
	struct spi_transfer t;

	if (!jcpll_dev) {
		err = -ENODEV;
		if (!searched)
			err = qixis_jcpll_init();
		if (err)
			return err;
	}

	tx_buf[0] = (u8)((offset >> 8) & 0x3);
	tx_buf[1] = (u8)(offset & 0xFF);
	tx_buf[2] = val;
	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.len = sizeof(tx_buf);
	t.tx_buf = tx_buf;
	spi_message_add_tail(&t, &m);
	err = spi_sync(jcpll_dev, &m);
	return err;
}

static int qixis_jcpll_read_spi(u8 *val_ptr, u16 offset)
{
	int err;
	u8 tx_buf[3];
	u8 rx_buf[3];
	struct spi_message m;
	struct spi_transfer t;

	if (!jcpll_dev) {
		err = -ENODEV;
		if (!searched)
			err = qixis_jcpll_init();
		if (err)
			return err;
	}

	tx_buf[0] = (u8)(((offset >> 8) & 0x3) | 0x80);
	tx_buf[1] = (u8)(offset & 0x00FF);
	tx_buf[2] = 0;
	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.len = sizeof(tx_buf);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	spi_message_add_tail(&t, &m);
	err = spi_sync(jcpll_dev, &m);
	*val_ptr = rx_buf[2];
	return err;
}

static int qixis_jcpll_init(void)
{
	int err;

	if (!jcpll_node || searched)
		return -ENODEV;

	jcpll_dev = find_spi_device_from_np(jcpll_node);
	if (jcpll_dev == NULL)
		return -EFAULT;

	/* Configure the AD9525 communication mode */
	err = qixis_jcpll_write_spi(COMMS_LSB_FIRST, JCPLL_SPI_MODE_REG);
	if (err)
		return err;
	err = qixis_jcpll_write_spi(UPDATE_IO,       JCPLL_IO_UPDATE_REG);
	if (err)
		return err;
	/* Set read mode to return the active registers */
	err = qixis_jcpll_write_spi(READ_ACTV_REGS,  JCPLL_READBACK_CTL_REG);
	if (err)
		return err;
	/* Set the dividers to bypass mode */
	err = qixis_jcpll_write_spi(BYPASS_DIVIDERS, JCPLL_N_DIVIDER_REG);
	if (err)
		return err;
	/* REF MON pin reports the DLD status */
	err = qixis_jcpll_write_spi(REF_MON_DLD,     JCPLL_REF_MON_PIN_REG);
	if (err)
		return err;
	/* Power down PLL and set charge pump pin to VDD/2 */
	err = qixis_jcpll_write_spi(VDD_CP_BY_HALF,  JCPLL_STATUS_PIN_REG);
	if (err)
		return err;
	err = qixis_jcpll_write_spi(PWR_DWN_PLL,     JCPLL_PWR_DWN_REG);
	if (err)
		return err;
	/* Update the registers */
	err = qixis_jcpll_write_spi(UPDATE_IO,       JCPLL_IO_UPDATE_REG);
	if (err)
		return err;

	searched = 1;

	return 0;
}

int qixis_xcvr_present(int xcvr_id)
{
	int mask;
	int ret = 0;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	switch (qixis_board_info.board_type) {
	case QIXIS_BOARD_TYPE_D4400EVB:
		/* Invert active low PRSNTB bits to get PRSNT */
		ret = (int)((~qixis_read(QIXIS_STAT_PRESENT)) & 3);
		/* Reverse bit order */
		mask = 2 >> xcvr_id;
		ret = (ret & mask) ? 1 : 0;
		break;
	case QIXIS_BOARD_TYPE_D4400RDB:
		switch (xcvr_id) {
		case 0:
			ret = gpio_get_value_cansleep(gpio_prsntb1) ? 0 : 1;
			break;
		case 1:
			ret = gpio_get_value_cansleep(gpio_prsntb2) ? 0 : 1;
			break;
		default:
			ret = 0;
			break;
		}
		break;
	default:
		ret = 0;
		break;
	}
	mutex_unlock(&qixis_lock);
	return ret;
}
EXPORT_SYMBOL(qixis_xcvr_present);

static u8 reverse_byte(u8 byte)
{
	int i;
	u8 tmp = 0;

	for (i = 0; i < 8; i++) {
		tmp = (tmp << 1) | (byte & 1);
		byte >>= 1;
	}
	return tmp;
}

int qixis_leds_set_clear(unsigned int set, unsigned int clear)
{
	u8 leds;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	switch (qixis_board_info.board_type) {
	case QIXIS_BOARD_TYPE_D4400EVB:
		leds = reverse_byte(qixis_read(QIXIS_LED_CONTROL));
		leds = (u8) (((leds & ~clear) | set) & 0xF);
		qixis_write(QIXIS_LED_CONTROL, reverse_byte(leds));
		break;
	case QIXIS_BOARD_TYPE_D4400RDB:
		leds = qixis_read(0);
		leds = (u8) (((leds & ~clear) | set) & 0xF);
		qixis_write(0, leds);
		break;
	default:
		break;
	}
	mutex_unlock(&qixis_lock);
	return 0;
}
EXPORT_SYMBOL(qixis_leds_set_clear);

int qixis_set_reboot_method(unsigned int method)
{
	/* 0-hard/WDT reset, nonzero-simulated reset */
	mutex_lock(&qixis_lock);
	d4400_set_reboot_method(method);
	mutex_unlock(&qixis_lock);
	return 0;
}
EXPORT_SYMBOL(qixis_set_reboot_method);

static int __qixis_jcpll_write_reg(u8 val, u16 offset)
{
	u8 nibble0, msb, lsb;
	int err = 0;

	switch (qixis_board_info.board_type) {
	case QIXIS_BOARD_TYPE_D4400EVB:
		/* 12 bits (0 - 11) of offset contains register address
		 *  MAX offset value is 0x3ff
		 */
		lsb = (u8)(offset & 0x00ff);
		nibble0 = (u8) ((offset >> 8) & 0x03);
		msb = WRITE_NIBBLE | nibble0;
		/* Wait for any previous transaction to complete */
		if (!qixis_wait_for_jcpll_ready())
			err = -EBUSY;
		else {
			qixis_write(QIXIS_CLK_JCPLL_DATA, val);
			qixis_write(QIXIS_CLK_JCPLL_ADDL, lsb);
			qixis_write(QIXIS_CLK_JCPLL_ADDH, msb);
		}
		break;
	case QIXIS_BOARD_TYPE_D4400RDB:
		err = qixis_jcpll_write_spi(val, offset);
		break;
	default:
		err = -ENODEV;
	}

	return err;
}

int qixis_jcpll_write_reg(u8 val, u16 offset)
{
	int err;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	err = __qixis_jcpll_write_reg(val, offset);
	mutex_unlock(&qixis_lock);
	return err;
}
EXPORT_SYMBOL(qixis_jcpll_write_reg);

int qixis_jcpll_read_reg(u16 offset)
{
	int err = 0;
	u8 retval = 0xFF;
	u8 nibble0, msb, lsb;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	switch (qixis_board_info.board_type) {
	case QIXIS_BOARD_TYPE_D4400EVB:
		/* 12 bits (0 - 11) of offset contains register address
		 * MAX offset value is 0x3ff
		 */
		lsb = (u8)(offset & 0x00ff);
		nibble0 = (u8) ((offset >> 8) & 0x03);
		msb = READ_NIBBLE | nibble0;
		/* Wait for any previous transaction to complete */
		if (!qixis_wait_for_jcpll_ready())
			err = -EBUSY;
		else {
			qixis_write(QIXIS_CLK_JCPLL_ADDL, lsb);
			qixis_write(QIXIS_CLK_JCPLL_ADDH, msb);
			/* Wait for read transaction to complete */
			if (!qixis_wait_for_jcpll_ready())
				err = -EBUSY;
			retval = qixis_read(QIXIS_CLK_JCPLL_DATA);
		}
		break;
	case QIXIS_BOARD_TYPE_D4400RDB:
		err = qixis_jcpll_read_spi(&retval, offset);
		break;
	default:
		err = -ENODEV;
	}
	mutex_unlock(&qixis_lock);

	return err ? err : retval;
}
EXPORT_SYMBOL(qixis_jcpll_read_reg);

int qixis_jcpll_freq_fixed(void)
{
	int err;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	err = __qixis_jcpll_write_reg(VDD_CP_BY_HALF, JCPLL_STATUS_PIN_REG);
	if (!err)
		err = __qixis_jcpll_write_reg(PWR_DWN_PLL, JCPLL_PWR_DWN_REG);
	if (!err)
		err = __qixis_jcpll_write_reg(UPDATE_IO, JCPLL_IO_UPDATE_REG);
	mutex_unlock(&qixis_lock);
	return err;
}
EXPORT_SYMBOL(qixis_jcpll_freq_fixed);

int qixis_jcpll_freq_track(void)
{
	int err;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	err = __qixis_jcpll_write_reg(VDD_CP_NORMAL, JCPLL_STATUS_PIN_REG);
	if (!err)
		err = __qixis_jcpll_write_reg(PWR_ENB_PLL, JCPLL_PWR_DWN_REG);
	if (!err)
		err = __qixis_jcpll_write_reg(UPDATE_IO, JCPLL_IO_UPDATE_REG);
	mutex_unlock(&qixis_lock);
	return err;
}
EXPORT_SYMBOL(qixis_jcpll_freq_track);

int qixis_jcpll_locked(void)
{
	int locked = 0;
	int error;
	u8 val;

	if (!qixis_base)
		return 0;

	mutex_lock(&qixis_lock);
	switch (qixis_board_info.board_type) {
	case QIXIS_BOARD_TYPE_D4400EVB:
		locked = (qixis_read(QIXIS_CLK_JCPLL_STATUS) & JCPLL_LOCKED) ?
								1 : 0;
		break;
	case QIXIS_BOARD_TYPE_D4400RDB:
		error = qixis_jcpll_read_spi(&val, JCPLL_PLL_READBACK_REG);
		if (!error)
			locked = (val & DIG_LOCK_DETECT) ? 1 : 0;
		break;
	default:
		locked = 0;
		break;
	}
	mutex_unlock(&qixis_lock);
	return locked;
}
EXPORT_SYMBOL(qixis_jcpll_locked);

int qixis_jcpll_use_refc(int enable)
{
	int err;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	err = __qixis_jcpll_write_reg(enable ? ENABLE_REFC : DISABLE_REFC,
						 JCPLL_REFC_REG);
	if (!err)
		err = __qixis_jcpll_write_reg(UPDATE_IO, JCPLL_IO_UPDATE_REG);
	mutex_unlock(&qixis_lock);
	return err;
}
EXPORT_SYMBOL(qixis_jcpll_use_refc);

static ssize_t show_board_type(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", qixis_board_info.board_type);
}

static ssize_t show_board_rev(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", qixis_board_info.board_rev);
}

static ssize_t show_board_type_name(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int brd = qixis_board_info.board_type + 1;
	if (brd > (sizeof(board_names)/sizeof(char *)))
		brd = 0;
	return sprintf(buf, "%s\n", board_names[brd]);
}

static ssize_t show_board_rev_name(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	char rev = qixis_board_info.board_rev < 0 ? '?' :
					qixis_board_info.board_rev + 'A';
	return sprintf(buf, "Rev %c\n", rev);
}

static ssize_t show_leds(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	u8 leds = 0;

	if (!qixis_base)
		return -EPROBE_DEFER;

	mutex_lock(&qixis_lock);
	switch (qixis_board_info.board_type) {
	case QIXIS_BOARD_TYPE_D4400EVB:
		leds = reverse_byte(qixis_read(QIXIS_LED_CONTROL)) & 0xF;
		break;
	case QIXIS_BOARD_TYPE_D4400RDB:
		leds = qixis_read(0) & 0xF;
		break;
	default:
		break;
	}
	mutex_unlock(&qixis_lock);
	return sprintf(buf, "%d\n", leds);
}

static ssize_t set_leds(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int err;
	unsigned int val;

	err = kstrtouint(buf, 10, &val);
	if (err)
		return err;

	qixis_leds_set_clear(val, ~val);
	return count;
}

static DEVICE_ATTR(board_type,      S_IRUGO, show_board_type,      NULL);
static DEVICE_ATTR(board_type_name, S_IRUGO, show_board_type_name, NULL);
static DEVICE_ATTR(board_rev,       S_IRUGO, show_board_rev,       NULL);
static DEVICE_ATTR(board_rev_name,  S_IRUGO, show_board_rev_name,  NULL);
static DEVICE_ATTR(leds,  S_IWUSR | S_IRUGO, show_leds,            set_leds);

static struct attribute *qixis_attributes[] = {
	&dev_attr_board_type.attr,
	&dev_attr_board_type_name.attr,
	&dev_attr_board_rev.attr,
	&dev_attr_board_rev_name.attr,
	&dev_attr_leds.attr,
	NULL
};

static const struct attribute_group qixis_group = {
	.attrs = qixis_attributes,
};

static int qixis_open(struct inode *i, struct file *f)
{
	int err = 0;

	if (jcpll_node && !searched)
		err = qixis_jcpll_init();
	return err;
}

static int qixis_close(struct inode *i, struct file *f)
{
	return 0;
}

static long qixis_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *ioargp = (void __user *)arg;
	u32 temp;
	u8 temp8;
	struct qixis_jcpll_read_buf jcpllrb;
	struct qixis_jcpll_write_buf jcpllwb;

	switch (cmd) {
	/* Return board type and revision */
	case QIXIS_BOARD_INFO:
		if (copy_to_user((struct qixis_board_info *)ioargp,
				&qixis_board_info,
				sizeof(struct qixis_board_info))) {
			err = -EFAULT;
			goto out;
		}
		break;
	/* Set JCPLL to track the reference frequency or fix the frequency */
	case QIXIS_JCPLL_FREQ_TRACK:
		if (arg)
			err = qixis_jcpll_freq_track();
		else
			err = qixis_jcpll_freq_fixed();
		break;
	/* Return JCPLL locked status */
	case QIXIS_JCPLL_LOCKED:
		temp = qixis_jcpll_locked();
		if (copy_to_user((u32 *)ioargp, &temp, sizeof(u32)))
			err = -EFAULT;
		break;
	/* Read a byte to the JCPLL */
	case QIXIS_JCPLL_WRITE:
		if (copy_from_user(&jcpllwb,
				(struct qixis_jcpll_write_buf *)ioargp,
				sizeof(struct qixis_jcpll_write_buf))) {
			err = -EFAULT;
			goto out;
		}
		err = qixis_jcpll_write_reg(jcpllwb.value, jcpllwb.address);
		break;
	/* Read a byte from the JCPLL */
	case QIXIS_JCPLL_READ:
		if (copy_from_user(&jcpllrb,
				(struct qixis_jcpll_read_buf *)ioargp,
				sizeof(struct qixis_jcpll_read_buf))) {
			err = -EFAULT;
			goto out;
		}
		err = qixis_jcpll_read_reg(jcpllrb.address);
		if (err < 0)
			goto out;
		temp8 = (u8) err;
		err = 0;
		if (copy_to_user((u8 *)jcpllrb.value_ptr, &temp8, sizeof(u8)))
			err = -EFAULT;
		break;
	/* Select the JCPLL reference clock source as RefC or normal (REFA) */
	case QIXIS_JCPLL_USE_REFC:
		err = qixis_jcpll_use_refc(arg);
		break;
	/* Check presence of a transciver */
	case QIXIS_XCVR_PRESENT:
		if (copy_from_user(&temp, (u32 *)ioargp, sizeof(u32))) {
			err = -EFAULT;
			goto out;
		}
		temp = qixis_xcvr_present(temp);
		if (copy_to_user((u32 *)ioargp, &temp, sizeof(u32)))
			err = -EFAULT;
		break;
	/* Set / Clear the software LEDs */
	case QIXIS_LEDS_SET_CLEAR:
		err = qixis_leds_set_clear((arg>>8)&0xFF, arg&0xFF);
		break;
	/* Set reboot method */
	case QIXIS_SET_REBOOT_METHOD:
		err = qixis_set_reboot_method(arg);
		break;
	default:
		err = -EINVAL;
	}

	if (!err)
		return 0;
out:
	pr_err("QIXIS Driver: ioctl() error %d\n", err);
	return err;
}

static const struct file_operations qixis_fops = {
	.owner = THIS_MODULE,
	.open = qixis_open,
	.release = qixis_close,
	.unlocked_ioctl = qixis_ioctl,
};



static int __init d4400_fpga_probe(struct platform_device *pdev)
{
	dev_t devno;
	int err = 0;
	struct device *device = NULL;
	u8 reg0, reg1;
	int brd;
	char rev;

	mutex_init(&qixis_lock);

	/* Get the base address from the DTB */
	qixis_base = of_iomap(pdev->dev.of_node, 0);
	if (!qixis_base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	/* Register major number, and accept a dynamic number */
	if (qixis_major != 0) {
		devno = MKDEV(qixis_major, qixis_minor);
		err = register_chrdev_region(devno, 1, QIXIS_DEVICE_NAME);
	} else {
		err = alloc_chrdev_region(&devno, qixis_minor, 1,
						QIXIS_DEVICE_NAME);
		qixis_major = MAJOR(devno);
		qixis_minor = MINOR(devno);
	}
	if (err < 0) {
		dev_err(&pdev->dev, "QIXIS can't get major number: %d\n",
				err);
		goto chrdev_fail;
	}

	/* Create the device class if required */
	if (qixis_class == NULL) {
		qixis_class = class_create(THIS_MODULE, QIXIS_DEVICE_NAME);
		if (IS_ERR(qixis_class)) {
			err = PTR_ERR(qixis_class);
			dev_err(&pdev->dev, "QIXIS class_create() failed %d\n",
					err);
			goto class_fail;
		}
	}

	/* Create the character device */
	cdev_init(&qixis_cdev, &qixis_fops);
	qixis_cdev.owner = THIS_MODULE;
	err = cdev_add(&qixis_cdev, devno, 1);
	if (err < 0) {
		dev_err(&pdev->dev, "QIXIS error %d while trying to add %s",
			err, QIXIS_DEVICE_NAME);
		goto add_fail;
	}

	/* Create sysfs device */
	device = device_create(qixis_class, &pdev->dev, devno, NULL,
				QIXIS_DEVICE_NAME);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		dev_err(&pdev->dev, "QIXIS error %d while trying to create %s",
			err, QIXIS_DEVICE_NAME);
		goto device_fail;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &qixis_group);
	if (err < 0) {
		dev_err(&pdev->dev, "QIXIS error %d while trying to create group %s",
			err, QIXIS_DEVICE_NAME);
		goto group_fail;
	}

	reg0 = qixis_read(0);
	reg1 = qixis_read(1);
	qixis_board_info.board_type = QIXIS_BOARD_TYPE_UNKNOWN;
	qixis_board_info.board_rev  = QIXIS_BOARD_REV_UNKNOWN;
	if (reg0 == reg1) {
		qixis_board_info.board_type = QIXIS_BOARD_TYPE_D4400RDB;
		qixis_board_info.board_rev  = (reg0 >> 6) & 3;
	} else {
		qixis_board_info.board_type = QIXIS_BOARD_TYPE_D4400EVB;
		switch (reg1) {
		case 0x11:
			qixis_board_info.board_rev = QIXIS_BOARD_REV_A;
			break;
		case 0x22:
			qixis_board_info.board_rev = QIXIS_BOARD_REV_B;
			break;
		default:
			qixis_board_info.board_rev = QIXIS_BOARD_REV_UNKNOWN;
			break;
		}
	}

	brd = qixis_board_info.board_type + 1;
	if (brd > (sizeof(board_names)/sizeof(char *)))
		brd = 0;
	rev = qixis_board_info.board_rev < 0 ? '?' :
		qixis_board_info.board_rev + 'A';
	dev_info(&pdev->dev,
		"QIXIS board type: %s, rev %c\n", board_names[brd], rev);

	if (qixis_board_info.board_type == QIXIS_BOARD_TYPE_D4400EVB) {
		dev_info(&pdev->dev,
			"QIXIS fpga probe : %02x:%02x - %02x.%02x\n",
			qixis_read(QIXIS_ID),
			qixis_read(QIXIS_EVBBRD_VER),
			qixis_read(QIXIS_FPGA_MAJOR_VER),
			qixis_read(QIXIS_FPGA_MINOR_VER));
	}

	if (qixis_board_info.board_type == QIXIS_BOARD_TYPE_D4400RDB) {
		jcpll_node = of_parse_phandle(pdev->dev.of_node,
						"jcpll-handle", 0);
		if (jcpll_node == NULL) {
			dev_err(&pdev->dev, "jcpll-handle not found");
			err = -ENOENT;
			goto defer;
		}
		/* Get the GPIO pin numbers from device node */
		gpio_prsntb1 = of_get_named_gpio(pdev->dev.of_node,
			"gpio-prsntb1", 0);
		gpio_prsntb2 = of_get_named_gpio(pdev->dev.of_node,
			"gpio-prsntb2", 0);
		if (gpio_prsntb1 == -EPROBE_DEFER ||
		    gpio_prsntb2 == -EPROBE_DEFER) {
			dev_info(&pdev->dev, "GPIOs Prsntb1, Prsntb2 deferred\n");
			err = -EPROBE_DEFER;
			goto defer;
		}

		if (!gpio_is_valid(gpio_prsntb1))
			dev_err(&pdev->dev, "gpio_prsntb1 GPIO NUMBER = %d is not valid\n",
				gpio_prsntb1);
		else {
			gpio_request(gpio_prsntb1, "gpio-prsntb1");
			gpio_direction_input(gpio_prsntb1);
		}
		if (!gpio_is_valid(gpio_prsntb2))
			dev_err(&pdev->dev, "gpio_prsntb2 GPIO NUMBER = %d is not valid\n",
				gpio_prsntb2);
		else {
			gpio_request(gpio_prsntb2, "gpio-prsntb2");
			gpio_direction_input(gpio_prsntb2);
		}
	}

	return 0;

defer:
	sysfs_remove_group(&pdev->dev.kobj, &qixis_group);
group_fail:
	device_destroy(qixis_class, devno);
device_fail:
	cdev_del(&qixis_cdev);
add_fail:
	class_destroy(qixis_class);
	qixis_class = NULL;
class_fail:
	unregister_chrdev_region(qixis_major, 1);
	qixis_major = 0;
chrdev_fail:
	iounmap(qixis_base);
	qixis_base = NULL;
	return err;
}

static int __exit d4400_fpga_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &qixis_group);
	device_destroy(qixis_class, MKDEV(qixis_major, qixis_minor));
	class_destroy(qixis_class);
	qixis_class = NULL;
	cdev_del(&qixis_cdev);
	unregister_chrdev_region(qixis_major, 1);
	qixis_major = 0;
	iounmap(qixis_base);
	qixis_base = NULL;
	return 0;
}

static struct of_device_id d4400_fpga_ids[] = {
	{.compatible = "fsl,d4400-fpga", },
	{ /* sentinel */ }
};

static struct platform_driver d4400_fpga_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = d4400_fpga_ids,
	},
	.remove = __exit_p(d4400_fpga_remove),
};

static int __init d4400_fpga_init(void)
{
	return platform_driver_probe(&d4400_fpga_driver, d4400_fpga_probe);
}

static void __exit d4400_fpga_exit(void)
{
	platform_driver_unregister(&d4400_fpga_driver);
}

module_init(d4400_fpga_init);
module_exit(d4400_fpga_exit);

MODULE_DESCRIPTION("QIXIS FPGA register access via WEIM bus");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

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
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
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
#include <linux/slab.h>
#include <linux/of_i2c.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/overlay_ext.h>

#include <mach/src.h>
#include <linux/fsl-d4400-sys.h>
#include <mach/mach-d4400.h>
#include <linux/platform_data/ina2xx.h>

#define DRIVER_NAME "d4400-sys"
#define D4400_SYS_MOD_NAME "d4400-sys"

/* Debug and error reporting macros */
#define IF_DEBUG(x) if (d4400_sys_data->debug & (x))
#define ERR(...) {if (d4400_sys_data->debug & DEBUG_D4400_SYS_DEV_MESSAGES) \
				pr_err(D4400_SYS_MOD_NAME __VA_ARGS__); }

struct d4400_sys_dev *d4400_sys_data;

static struct ipmi_info *d4400_sys_verify_ipmi_info(
	struct d4400_sys_i2c_eeprom *eeprom);

/* Note that the board name index must match the
 * enum board_type in util-d4400.h
 */
static const char * const board_names[] = {
	"UNKNOWN",	/* BOARD_TYPE_UNKNOWN+1 */
	"D4400_EVB",	/* BOARD_TYPE_D4400_EVB+1 */
	"D4400_RDB",	/* BOARD_TYPE_D4400_RDB+1 */
	"D4400_4T4R",	/* BOARD_TYPE_D4400_4T4R+1 */
	"D4400_21RRH",	/* BOARD_TYPE_D4400_21RRH+1 */
	"D4400_4T4RK1",	/* BOARD_TYPE_D4400_4T4RK1+1 */
};

/* Default IPMI data for 4T4R POC:
 *  Mfg str:  "Freescale"
 *  Name str: "FSL D4400 4T4R Reference Board"
 *  Serial:   "1001"
 *  Part num: "REV A"
 *  No multirecord version
 */
static const u8 default_ipmi_4t4r_v1_0[] = {
  0x01, 0x00, 0x00, 0x01, 0x00, 0x0a, 0x00, 0xf4, 0x01, 0x09, 0x19, 0x00,
  0x00, 0x00, 0xc9, 0x46, 0x72, 0x65, 0x65, 0x73, 0x63, 0x61, 0x6c, 0x65,
  0xde, 0x46, 0x53, 0x4c, 0x20, 0x44, 0x34, 0x34, 0x30, 0x30, 0x20, 0x34,
  0x54, 0x34, 0x52, 0x20, 0x52, 0x65, 0x66, 0x65, 0x72, 0x65, 0x6e, 0x63,
  0x65, 0x20, 0x42, 0x6f, 0x61, 0x72, 0x64, 0xc4, 0x31, 0x30, 0x30, 0x31,
  0xc5, 0x52, 0x45, 0x56, 0x20, 0x41, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0xc0, 0x02, 0x04, 0xff,
  0x3b, 0x00, 0x00, 0x01, 0x00, 0xc3, 0x82, 0x08, 0xc4, 0xef, 0x34, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

/* Default IPMI data for 4T4RK1 POC board:
 *  Mfg str:  "NXP"
 *  Name str: "NXP AFD4400-4T4RK1 System"
 *  Serial:   "1001"
 *  Part num: "REV A"
 *  Multirecord version v1.01
 */
static const u8 default_ipmi_4t4rk1_v1_01[] = {
0x01, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0xf6,
0x01, 0x07, 0x19, 0x00, 0x00, 0x00, 0xc3, 0x4e,
0x58, 0x50, 0xd9, 0x4e, 0x58, 0x50, 0x20, 0x41,
0x46, 0x44, 0x34, 0x34, 0x30, 0x30, 0x2d, 0x34,
0x54, 0x34, 0x52, 0x4b, 0x31, 0x20, 0x53, 0x79,
0x73, 0x74, 0x65, 0x6d, 0xc4, 0x31, 0x30, 0x30,
0x31, 0xc5, 0x52, 0x45, 0x56, 0x20, 0x41, 0xc0,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef,
0xc0, 0x02, 0x04, 0xfe, 0x3c, 0x01, 0x00, 0x01,
0x00, 0xc3, 0x82, 0x10, 0xbc, 0xef, 0x34, 0x08,
0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#ifdef CONFIG_BOARD_4T4R
static const u8 *default_ipmi_rawdata = default_ipmi_4t4r_v1_0;
#elif defined CONFIG_BOARD_4T4RK1
static const u8 *default_ipmi_rawdata = default_ipmi_4t4rk1_v1_01;
#else
#error fsl-d4400-sys.c Default IPMI table not defined!
#endif

/* Current/power monitoring ina220 device for 4T4R21 revC */
#define INA220_I2C_BUSNUM	6 /* i2c9 */
static struct ina2xx_platform_data ina220 = {
	.shunt_uohms = 1000, /* 0.001 Ohms */
};
static const unsigned short ina2xx_i2c[] = { 0x41, I2C_CLIENT_END };

/* Spi gpio overlays: Swapped here means that miso/mosi, controled as
 * gpios, are swapped as compared to the spi module.
 */
struct property spigpio_swapped_prop[] = {
	{ .name = "gpio-sck",          .value = "&gpioD \n 1D \n 0",
		.length = 3, ._flags = PROP_STR },
	{ .name = "gpio-miso",         .value = "&gpioE \n 8 \n 0",
		.length = 3, ._flags = PROP_STR },
	{ .name = "gpio-mosi",         .value = "&gpioE \n 9 \n 0",
		.length = 3, ._flags = PROP_STR },
	{ .name = "cs-gpios",          .value = "&gpioD \n 1E \n 0",
		.length = 3, ._flags = PROP_STR },
	{}
};
struct property spigpio_normal_prop[] = {
	{ .name = "gpio-sck",          .value = "&gpioD \n 1D \n 0",
		.length = 3, ._flags = PROP_STR },
	{ .name = "gpio-miso",         .value = "&gpioE \n 9 \n 0",
		.length = 3, ._flags = PROP_STR },
	{ .name = "gpio-mosi",         .value = "&gpioE \n 8 \n 0",
		.length = 3, ._flags = PROP_STR },
	{ .name = "cs-gpios",          .value = "&gpioD \n 1E \n 0",
		.length = 3, ._flags = PROP_STR },
	{}
};
struct fragment_node frag_spigpio[] = {
	{
		.target = "&spi_gpio",
		.np_array = NULL,
		/* Property array is dynamically set based upon
		 * board type.  Default to correct spi signal
		 * configuration.
		*/
		.ov_prop_array = spigpio_normal_prop,
	},
	{}
};

static const char *d4400_sys_board_type_name(void)
{
	int brd = 0;

	if (!d4400_sys_data)
		goto out;

	brd = d4400_sys_data->brd_info.board_type + 1;
	if (brd > (sizeof(board_names)/sizeof(char *)))
		brd = 0;
out:
	return board_names[brd];
}

static char d4400_sys_board_rev_letter(void)
{
	char rev = '?';

	if (!d4400_sys_data)
		goto out;

	if (d4400_sys_data->brd_info.board_rev >= 0)
		rev = d4400_sys_data->brd_info.board_rev + 'A';
out:
	return rev;
}

int d4400_sys_board_type(void)
{
	if (!d4400_sys_data)
		return BOARD_TYPE_UNKNOWN;

	return d4400_sys_data->brd_info.board_type;
}
EXPORT_SYMBOL(d4400_sys_board_type);

int d4400_sys_board_rev(void)
{
	if (!d4400_sys_data)
		return BOARD_REV_UNKNOWN;

	return d4400_sys_data->brd_info.board_rev;
}
EXPORT_SYMBOL(d4400_sys_board_rev);

int d4400_sys_xcvr_eeprom_power(void)
{
	if (!d4400_sys_data)
		return 0;

	/* For now, all boards using this driver has eeprom
	 * for transceiver.  Note that these boards may be using
	 * the system eeprom for this purpose.
	 */
	return 1;
}
EXPORT_SYMBOL(d4400_sys_xcvr_eeprom_power);

int d4400_sys_leds_set_clear(unsigned int set, unsigned int clear)
{
	u8 leds = 0;

	if (!d4400_sys_data)
		return -ENODEV;

	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		{
			struct fsl_4t4r_board *brd_4t4r =
				(struct fsl_4t4r_board *)d4400_sys_data->brd;

			leds = (u8) (((leds & ~clear) | set) & 0x03);
			gpio_direction_output(brd_4t4r->gpio_led1,
				leds & 0x01);
			gpio_direction_output(brd_4t4r->gpio_led2,
				(leds & 0x02)>>1);
		}
		break;
	case BOARD_TYPE_D4400_4T4RK1:
		{
			struct fsl_4t4rk1_board *brd_4t4rk1 =
				(struct fsl_4t4rk1_board *)d4400_sys_data->brd;

			leds = (u8) (((leds & ~clear) | set) & 0x0f);
			gpio_direction_output(brd_4t4rk1->gpio_led1,
				leds & 0x01);
			gpio_direction_output(brd_4t4rk1->gpio_led2,
				(leds & 0x02)>>1);
			gpio_direction_output(brd_4t4rk1->gpio_led3,
				(leds & 0x04)>>2);
			gpio_direction_output(brd_4t4rk1->gpio_led4,
				(leds & 0x08)>>3);
		}
		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(d4400_sys_leds_set_clear);

int d4400_sys_set_reboot_method(unsigned int method)
{
	if (!d4400_sys_data)
		return -ENODEV;

	/* 0-hard/WDT reset, nonzero-simulated reset */
	mutex_lock(&d4400_sys_data->lock);
	d4400_set_reboot_method(method);
	mutex_unlock(&d4400_sys_data->lock);
	return 0;
}
EXPORT_SYMBOL(d4400_sys_set_reboot_method);


static int pa_set_clear_4t4r(int pa, u32 set, u32 clear,
	struct fsl_4t4r_board *brd_4t4r)
{
	int ret = 0;
	int i;
	u32 *pa_sig;
	int *pa_pins;

	if ((pa == PA1) && (pa == PA2)) {
		pa_sig = &brd_4t4r->pac[pa].sig;
		pa_pins = brd_4t4r->pac[pa].pins;
	} else
		return -EINVAL;

	if (set) {
		/* Set some bits */
		for (i = 0; i < PA_PINCNT; ++i) {
			if (set & (1 << i)) {
				gpio_direction_output(pa_pins[i], 1);
				*pa_sig |= (1 << i);
			}
		}
	}
	if (clear) {
		/* Clear some bits */
		for (i = 0; i < PA_PINCNT; ++i) {
			if (clear & (1 << i)) {
				gpio_direction_output(pa_pins[i], 0);
				*pa_sig &= ~(1 << i);
			}
		}
	}
	return ret;
}

static int pa_set_clear_4t4rk1(int pa, u32 set, u32 clear,
	struct fsl_4t4rk1_board *brd_4t4rk1)
{
	int ret = 0;
	int i;
	u32 *pa_sig;
	int *pa_pins;

	if (pa == PA2) {
		pa_sig = &brd_4t4rk1->pac[0].sig;
		pa_pins = brd_4t4rk1->pac[0].pins;
	} else {
		if (pa == PA1) {
		pr_warn(D4400_SYS_MOD_NAME ": PA connector 1 is not availabe on 4T4RK1 board.\n");
		}
		return -EINVAL;
	}

	if (set) {
		/* Set some bits */
		for (i = 0; i < PA_PINCNT; ++i) {
			if (set & (1 << i)) {
				gpio_direction_output(pa_pins[i], 1);
				*pa_sig |= (1 << i);
			}
		}
	}
	if (clear) {
		/* Clear some bits */
		for (i = 0; i < PA_PINCNT; ++i) {
			if (clear & (1 << i)) {
				gpio_direction_output(pa_pins[i], 0);
				*pa_sig &= ~(1 << i);
			}
		}
	}
	return ret;
}

static int d4400_sys_pa_set_clear(int pa, unsigned int set, unsigned int clear)
{
	u8 ret = 0;

	if (!d4400_sys_data)
		return -ENODEV;

	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		{
			struct fsl_4t4r_board *brd_4t4r =
				(struct fsl_4t4r_board *)d4400_sys_data->brd;
			ret = pa_set_clear_4t4r(pa, set, clear, brd_4t4r);
		}
		break;
	case BOARD_TYPE_D4400_4T4RK1:
		{
			struct fsl_4t4rk1_board *brd_4t4rk1 =
				(struct fsl_4t4rk1_board *)d4400_sys_data->brd;
			ret = pa_set_clear_4t4rk1(pa, set, clear, brd_4t4rk1);
		}
		break;
	default:
		break;
	}
	return ret;
}
EXPORT_SYMBOL(d4400_sys_pa_set_clear);

static int d4400_sys_xcvr_present(int xcvr_id)
{
	u8 ret = 0;

	if (!d4400_sys_data)
		return -ENODEV;

	mutex_lock(&d4400_sys_data->lock);

	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_4T4RK1:
	case BOARD_TYPE_D4400_21RRH:

		/* Two transceivers available, each bit position
		 * represents a transceiver.
		 */
		if (xcvr_id <= 0x03)
			ret = xcvr_id;
		break;
	default:
		break;
	}
	mutex_unlock(&d4400_sys_data->lock);
	return ret;
}

static int board_ipmi_4t4r(struct ipmi_info *ipmi,
	struct fsl_4t4r_board *brd_4t4r)
{
	int ret = 0;
	u32 recordver = 0;
	struct ipmi_multirecord *mrec = &ipmi->multirec;

	/* Default values if nothing found */
	brd_4t4r->ipmi_mrec_ver = 0;
	brd_4t4r->freqband_MHz = 2100;	/* 2.1GHz */
	brd_4t4r->features = 0;
	brd_4t4r->max_pa_watts = 0;
	brd_4t4r->max_tx = 4;
	brd_4t4r->max_rx = 4;

	/* Get IPMI mrec board info if available */
	if (mrec->num_rec == 0)
		/* Don't return error, early boards may not have
		 * IPMI multirec burned in eeprom.
		 */
		return 0;

	/* Decode multirecord info if it exists */
	ret = d4400_mrec_decode_4t4r(mrec, &recordver, brd_4t4r);
	if (!ret)
		brd_4t4r->ipmi_mrec_ver = recordver;
	return ret;
}


static void wq_pa_alarm_4t4r(struct work_struct *work)
{
	//struct fsl_4t4r_board *brd_4t4r = container_of(to_delayed_work(work),
	//	struct fsl_4t4r_board, pa_alarm_irq_wq);
	//printk("\n\t%s %i  freqband %i MHz\n",
	//	__func__, __LINE__, brd_4t4r->freqband_MHz);
}

static irqreturn_t pa_alarm_handler_4t4r(int irq, void *brd)
{
	struct fsl_4t4r_board *brd_4t4r = (struct fsl_4t4r_board *)brd;

	//printk("\n\t%s %i  brd %08x pa_alarm_irq_wq %08x\n",
	//	__func__, __LINE__, (u32)brd,
	//	(u32)brd_4t4r->pa_alarm_irq_wq.work.func);

	schedule_delayed_work(&brd_4t4r->pa_alarm_irq_wq, 0);
	return IRQ_HANDLED;
}

static int board_setup_4t4r(struct d4400_sys_dev *d4400_sys)
{
	int ret = 0;
	struct device_node *dev_node = d4400_sys->dev->of_node;
	struct device *dev = d4400_sys->dev;
	struct pinctrl *pinctrl;
	struct fsl_4t4r_board *brd_4t4r = NULL;
	int i;
	int pa;
	const char *pacon_node_str[PA_CON_MAX] = { "pa_con1", "pa_con2" };
	int alarm_irq;
	int pin;
	struct pa_con *pac;

	brd_4t4r = kzalloc(
		sizeof(struct fsl_4t4r_board), GFP_KERNEL);
	if (!brd_4t4r) {
		ret = -ENOMEM;
		goto out;
	}

	/* Pin muxing */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "can't get/select pinctrl\n");
		ret = -EINVAL;
		goto out_err;
	}

	/* Led gpio */
	brd_4t4r->gpio_led1 = of_get_named_gpio(dev_node,
		"gpio-led1", 0);
	if (!gpio_is_valid(brd_4t4r->gpio_led1)) {
		dev_err(dev, "4T4R brd led1 gpio %d is not valid\n",
			brd_4t4r->gpio_led1);
		ret = -EINVAL;
		goto out_err;
	} else {
		gpio_request(brd_4t4r->gpio_led1, "gpio-led1");
		gpio_direction_output(brd_4t4r->gpio_led1, 0);
	}

	/* Led gpio */
	brd_4t4r->gpio_led2 = of_get_named_gpio(dev_node,
		"gpio-led2", 0);
	if (!gpio_is_valid(brd_4t4r->gpio_led2)) {
		dev_err(dev, "4T4R brd led2 gpio %d is not valid\n",
			brd_4t4r->gpio_led2);
		ret = -EINVAL;
		goto out_err;
	} else {
		gpio_request(brd_4t4r->gpio_led2, "gpio-led2");
		gpio_direction_output(brd_4t4r->gpio_led2, 0);
	}

	INIT_DELAYED_WORK(&brd_4t4r->pa_alarm_irq_wq, wq_pa_alarm_4t4r);

	/* PA signals */
	for (pa = 0; pa < PA_CON_MAX; ++pa) {
		pac = &brd_4t4r->pac[pa];
		memset(pac, 0, sizeof(struct pa_con));

		for (i = 0; i < PA_PINCNT; ++i) {
			pin = pac->pins[i] = of_get_named_gpio(
				dev_node, pacon_node_str[pa], i);
			if (!gpio_is_valid(pin)) {
				dev_err(dev, "4T4R brd PA%d control signal %d is not valid %i\n",
					pa, pin, i);
				ret = -EINVAL;
				goto out_err;
			}

			/* Setup gpio signal */
			gpio_request(pin, "pa_con_signal");
			if ((i == PA_ALARM1_B) || (i == PA_ALARM2_B)) {
				/* Interrupt input signals */
				gpio_direction_input(pin);
				alarm_irq = gpio_to_irq(pin);
				if (alarm_irq < 0) {
					dev_err(dev, "gpio_to_irq fail");
					goto out_err;
				}

				pac->alarm_irq[pa] = alarm_irq;
				ret = request_irq(alarm_irq,
					(irq_handler_t)pa_alarm_handler_4t4r,
					IRQF_TRIGGER_FALLING,
					"pa_alarm_handler_4t4r",
					brd_4t4r);
				if (ret) {
					dev_err(dev, "request_irq fail");
					goto out_err;
				}
				/* Start with alarm irq disabled */
				disable_irq(alarm_irq);
				if (i == PA_ALARM1_B)
					pac->pa_alarm_enabled[0] = 0;
				else
					pac->pa_alarm_enabled[1] = 0;

			} else {
				/* Gpio output */
				gpio_direction_output(pin, 0);
			}

		}
	}
	/* Get IPMI multirecord info */
	if (d4400_sys->ipmi)
		ret = board_ipmi_4t4r(d4400_sys->ipmi, brd_4t4r);

	/* Normal reboot method:
	 *   0 - via WDT and cpu reset
	 *   1 - simulated cpu reset
	 */
	d4400_sys->reboot_method = 0;

	/* Save board data */
	d4400_sys->brd = brd_4t4r;

out:
	return 0;
out_err:
	for (pa = 0; pa < PA_CON_MAX; ++pa) {
		/* Release gpio */
		for (i = 0; i < PA_PINCNT; ++i)  {
			pin = brd_4t4r->pac[pa].pins[i];
			if (pin)
				gpio_free(pin);
		}
	}
	kfree(brd_4t4r);
	return ret;
}

static u32 get_pa_sig_4t4rk1(struct fsl_4t4rk1_board *brd_4t4rk1)
{
	u32 pa_sig = 0;
	int pa = 0; /* One PA connector for 4t4rk1 */
	int *pa_pins;

	pa = 0;
	pa_sig = brd_4t4rk1->pac[pa].sig;
	pa_pins = brd_4t4rk1->pac[pa].pins;

	/* Update gpio inputs */
	pa_sig &= ~(1 << PA_ALARM1_B);
	if (gpio_get_value_cansleep(pa_pins[PA_ALARM1_B]))
		pa_sig |= (1 << PA_ALARM1_B);
	pa_sig &= ~(1 << PA_ALARM2_B);
	if (gpio_get_value_cansleep(pa_pins[PA_ALARM2_B]))
		pa_sig |= (1 << PA_ALARM2_B);

	/* Update */
	brd_4t4rk1->pac[pa].sig = pa_sig;

	return pa_sig;
}

static void wq_pa_alarm_4t4rk1(struct work_struct *work)
{
	u32 pa_sig = 0;

	struct fsl_4t4rk1_board *brd_4t4rk1 = container_of(to_delayed_work(work),
		struct fsl_4t4rk1_board, pa_alarm_irq_wq);

	pa_sig = get_pa_sig_4t4rk1(brd_4t4rk1);

	if (pa_sig & (1 << PA_ALARM1_B)) {
		pr_info(D4400_SYS_MOD_NAME ": Alarm1_B interrupt\n");
	}
	if (pa_sig & (1 << PA_ALARM2_B)) {
		pr_info(D4400_SYS_MOD_NAME ": Alarm2_B interrupt\n");
	}
	//printk("\n\t%s %i  freqband %i MHz\n",
	//	__func__, __LINE__, brd_4t4rk1->freqband_MHz);
}

static irqreturn_t pa_alarm_handler_4t4rk1(int irq, void *brd)
{
	struct fsl_4t4rk1_board *brd_4t4rk1 = (struct fsl_4t4rk1_board *)brd;

	//printk("\n\t%s %i  brd %08x pa_alarm_irq_wq %08x\n",
	//	__func__, __LINE__, (u32)brd,
	//	(u32)brd_4t4rk1->pa_alarm_irq_wq.work.func);

	schedule_delayed_work(&brd_4t4rk1->pa_alarm_irq_wq, 0);
	return IRQ_HANDLED;
}

static int board_ipmi_4t4rk1(struct ipmi_info *ipmi,
	struct fsl_4t4rk1_board *brd_4t4rk1)
{
	int ret = 0;
	u32 recordver = 0;
	struct ipmi_multirecord *mrec = &ipmi->multirec;

	/* Default values if nothing found */
	brd_4t4rk1->ipmi_mrec_ver = 0;
	brd_4t4rk1->freqband_MHz = 2100;	/* 2.1GHz */
	brd_4t4rk1->features = 0;
	brd_4t4rk1->max_pa_watts = 0;
	brd_4t4rk1->max_tx = 4;
	brd_4t4rk1->max_rx = 4;

	/* Get IPMI mrec board info if available */
	if (mrec->num_rec == 0)
		/* Don't return error, early boards may not have
		 * IPMI multirec burned in eeprom.
		 */
		return 0;

	/* Decode multirecord info if it exists */
	ret = d4400_mrec_decode_4t4rk1(mrec, &recordver, brd_4t4rk1);
	if (!ret)
		brd_4t4rk1->ipmi_mrec_ver = recordver;
	return ret;
}

static int board_setup_4t4rk1(struct d4400_sys_dev *d4400_sys)
{
	int ret = 0;
	struct device_node *dev_node = d4400_sys->dev->of_node;
	struct device *dev = d4400_sys->dev;
	struct pinctrl *pinctrl;
	struct fsl_4t4rk1_board *brd_4t4rk1 = NULL;
	int i;
	int pa;
	const char *pacon_node_str[1] = { "pa_con2" };
	const char *led_node_str[4] = { "gpio-led1", "gpio-led2",
		"gpio-led3", "gpio-led4"};
	const char rx_8vbias_en_str[] = "rx_8vbias_en";
	const char rx_8vbias_stat_str[] = "rx_8vbias_stat";
	int alarm_irq;
	int pin;
	struct pa_con *pac;

	brd_4t4rk1 = kzalloc(
		sizeof(struct fsl_4t4rk1_board), GFP_KERNEL);
	if (!brd_4t4rk1) {
		ret = -ENOMEM;
		goto out;
	}

	/* Pin muxing */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "4T4RK1 brd can't get/select pinctrl\n");
		ret = -EINVAL;
		goto out_err;
	}

	/* Led gpios */
	for (i = 0; i < 4; ++i) {
		u32 *led;

		switch(i)
		{
			default:
			case 0: led = &brd_4t4rk1->gpio_led1; break;
			case 1: led = &brd_4t4rk1->gpio_led2; break;
			case 2: led = &brd_4t4rk1->gpio_led3; break;
			case 3: led = &brd_4t4rk1->gpio_led4; break;
		}

		*led = of_get_named_gpio(dev_node, led_node_str[i], 0);
		if (!gpio_is_valid(*led)) {
			dev_err(dev, "4T4RK1 brd led%i gpio %d is not valid\n",
				i, *led);
			ret = -EINVAL;
			goto out_err;
		} else {
			gpio_request(*led, led_node_str[i]);
			gpio_direction_output(*led, 0); /* Set low */
		}
	}

	/* Uart3 RS-485 direction control gpio */
	brd_4t4rk1->gpio_rs485_dir = of_get_named_gpio(dev_node,
		"gpio-rs485-dir", 0);
	if (!gpio_is_valid(brd_4t4rk1->gpio_rs485_dir)) {
		dev_err(dev, "4T4R brd uart3 RS-485 direction control gpio %d is not valid\n",
			brd_4t4rk1->gpio_rs485_dir);
		ret = -EINVAL;
		goto out_err;
	} else {
		/* Set low for SN65HVD72D receiver enable/transmitter
		 * disable (REb).
		 */
		gpio_request(brd_4t4rk1->gpio_rs485_dir, "gpio-rs485-dir");
		gpio_direction_output(brd_4t4rk1->gpio_rs485_dir, 0);
	}


	/* Rx LNA 8v bias enable and status, all are active low */
	for (i = 0; i < RX_8VBIAS_EN_MAX; ++i) {
		pin = brd_4t4rk1->rx_8vbias_en_pins[i] = of_get_named_gpio(
			dev_node, rx_8vbias_en_str, i);
		if (!gpio_is_valid(pin)) {
			dev_err(dev, "4T4RK1 brd RX LNA 8v bias enable signal %d is not valid %i\n",
				pin, i);
			ret = -EINVAL;
			goto out_err;
		}
		gpio_request(pin, "rx_8vbias_en");
		/* Bias voltage enable is active low, set high to turn OFF */
		gpio_direction_output(pin, 1);
	}
	for (i = 0; i < RX_8VBIAS_STAT_MAX; ++i) {
		pin = brd_4t4rk1->rx_8vbias_stat_pins[i] = of_get_named_gpio(
			dev_node, rx_8vbias_stat_str, i);
		if (!gpio_is_valid(pin)) {
			dev_err(dev, "4T4RK1 brd RX LNA 8v bias status signal %d is not valid %i\n",
				pin, i);
			ret = -EINVAL;
			goto out_err;
		}
		gpio_request(pin, "rx_8vbias_stat");
		gpio_direction_input(pin);
	}


	INIT_DELAYED_WORK(&brd_4t4rk1->pa_alarm_irq_wq, wq_pa_alarm_4t4rk1);

	/* PA connector 2 control signals */
	pa = 0;
	pac = &brd_4t4rk1->pac[pa];
	memset(pac, 0, sizeof(struct pa_con));

	for (i = 0; i < PA_PINCNT; ++i) {
		pin = pac->pins[i] = of_get_named_gpio(
			dev_node, pacon_node_str[pa], i);
		if (!gpio_is_valid(pin)) {
			dev_err(dev, "4T4RK1 brd PA%d control signal %d is not valid %i\n",
				pa, pin, i);
			ret = -EINVAL;
			goto out_err;
		}

		/* Setup gpio signal */
		gpio_request(pin, "pa_con_signal");
		if ((i == PA_ALARM1_B) || (i == PA_ALARM2_B)) {

			/* Interrupt input signals */
			gpio_direction_input(pin);
			alarm_irq = gpio_to_irq(pin);
			if (alarm_irq < 0) {
				dev_err(dev, "gpio_to_irq fail");
				goto out_err;
			}

			pac->alarm_irq[pa] = alarm_irq;
			ret = request_irq(alarm_irq,
				(irq_handler_t)pa_alarm_handler_4t4rk1,
				IRQF_TRIGGER_FALLING,
				"pa_alarm_handler_4t4rk1",
				brd_4t4rk1);
			if (ret) {
				dev_err(dev, "request_irq fail");
				goto out_err;
			}

		} else {
			gpio_direction_output(pin, 0);
		}
	}

	/* Get IPMI multirecord info */
	if (d4400_sys->ipmi)
		ret = board_ipmi_4t4rk1(d4400_sys->ipmi, brd_4t4rk1);

	/* Normal reboot method:
	 *   0 - via WDT and cpu reset
	 *   1 - simulated cpu reset
	 */
	d4400_sys->reboot_method = 0;

	/* Save board data */
	d4400_sys->brd = brd_4t4rk1;

out:
	return 0;
out_err:
	/* Release gpio */
	pa = 0;
	for (i = 0; i < PA_PINCNT; ++i)  {
		pin = brd_4t4rk1->pac[pa].pins[i];
		if (pin)
			gpio_free(pin);
	}
	kfree(brd_4t4rk1);
	return ret;
}

static int d4400_sys_setup_board(struct d4400_sys_dev *d4400_sys)
{
	int ret = 0;
	struct i2c_adapter *i2c_adap;
	struct i2c_board_info i2c_info;

	mutex_lock(&d4400_sys->lock);

	/* Spi gpio signal overlay */
	frag_spigpio[0].ov_prop_array = spigpio_normal_prop;

	if ((d4400_sys_data->brd_info.board_type == BOARD_TYPE_D4400_4T4R) &&
		(d4400_sys_data->brd_info.board_rev == BOARD_REVA))
		/* 4T4R POC revA have miso/mosi swapped */
		frag_spigpio[0].ov_prop_array = spigpio_swapped_prop;

	ret = ov_load(frag_spigpio);
	if (ret < 0) {
		pr_err(D4400_SYS_MOD_NAME ": failed to create spi overlay with err %x\n",
				ret);
		goto out;
	}

	/* Look for i2c devices */
	i2c_adap = i2c_get_adapter(INA220_I2C_BUSNUM);
	if (i2c_adap) {
		memset(&i2c_info, 0, sizeof(struct i2c_board_info));
		strlcpy(i2c_info.type, "ina220", I2C_NAME_SIZE);
		i2c_info.platform_data  = &ina220;

		i2c_new_probed_device(i2c_adap, &i2c_info,
			   ina2xx_i2c, NULL);
		i2c_put_adapter(i2c_adap);
	}

	/* Setup the board */
	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		ret = board_setup_4t4r(d4400_sys);
		break;
	case BOARD_TYPE_D4400_4T4RK1:
		ret = board_setup_4t4rk1(d4400_sys);
		break;
	default:
		break;
	}
out:
	mutex_unlock(&d4400_sys->lock);
	return ret;
}

static int generate_ipmi_info_str(char *buf)
{
	char *p = buf;
	int j;
	struct ipmi_info *ipmi;

	if (!d4400_sys_data)
		return 0;
	if (!d4400_sys_data->ipmi) {
		sprintf(p, "IPMI information does not exists for this board.\n");
		goto out;
	}
	ipmi = d4400_sys_data->ipmi;

	/* Basic IPMI info */
	sprintf(p, "Mfg: %s\n\tName: %s\n\tSerial: %s\n\tPartnum: %s\n",
		ipmi->board.mfg_str,
		ipmi->board.name_str,
		ipmi->board.serial_str,
		ipmi->board.partnum_str);
	p += strlen(p);

	/* System specific info */
	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		{
			struct fsl_4t4r_board *brd_4t4r;
			brd_4t4r = d4400_sys_data->brd;
			sprintf(p, "\tFreq band: %i MHz\n",
				brd_4t4r->freqband_MHz);
			p += strlen(p);
			sprintf(p, "\tMultirec ver: %i%i.%i%i\n",
				(brd_4t4r->ipmi_mrec_ver>>24) & 0xff,
				(brd_4t4r->ipmi_mrec_ver>>16) & 0xff,
				(brd_4t4r->ipmi_mrec_ver>>8) & 0xff,
				brd_4t4r->ipmi_mrec_ver & 0xff);
			p += strlen(p);
			sprintf(p, "\tMax Tx/Rx: %i  %i\n",
				brd_4t4r->max_tx, brd_4t4r->max_rx);
			p += strlen(p);
			sprintf(p, "\tPA watts: %i.%i\n",
				brd_4t4r->max_pa_watts >> 4 & 0xfff,
				brd_4t4r->max_pa_watts & 0x0f);
			p += strlen(p);
			break;
		}
	case BOARD_TYPE_D4400_4T4RK1:
		{
			struct fsl_4t4rk1_board *brd_4t4rk1;
			brd_4t4rk1 = d4400_sys_data->brd;

			sprintf(p, "\tFreq band: %i MHz\n",
				brd_4t4rk1->freqband_MHz);
			p += strlen(p);
			sprintf(p, "\tMultirec ver: %i%i.%i%i\n",
				(brd_4t4rk1->ipmi_mrec_ver>>24) & 0xff,
				(brd_4t4rk1->ipmi_mrec_ver>>16) & 0xff,
				(brd_4t4rk1->ipmi_mrec_ver>>8) & 0xff,
				brd_4t4rk1->ipmi_mrec_ver & 0xff);
			p += strlen(p);
			sprintf(p, "\tMax Tx/Rx: %i  %i\n",
				brd_4t4rk1->max_tx, brd_4t4rk1->max_rx);
			p += strlen(p);
			sprintf(p, "\tPA watts: %i.%i\n",
				brd_4t4rk1->max_pa_watts >> 4 & 0xfff,
				brd_4t4rk1->max_pa_watts & 0x0f);
			p += strlen(p);
			break;
		}
	default:
		break;
	}

	/* Raw multirecord info */
	sprintf(p, "\tMultirecords: %i  size: %i\n",
		ipmi->multirec.num_rec,
		ipmi->multirec.size);
	p += strlen(p);

	for (j = 0; j < ipmi->multirec.num_rec; ++j) {
		ipmi_printstr_record_hdr(&p,
			ipmi->multirec.rechdr[j]);
	}
	p += strlen(p);

out:
	return strlen(buf);
}

static ssize_t show_ipmi_info(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return generate_ipmi_info_str(buf);
}

static ssize_t show_board_type(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", d4400_sys_board_type());
}

static ssize_t show_board_rev(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", d4400_sys_board_rev());
}

static ssize_t show_board_type_name(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%s\n", d4400_sys_board_type_name());
}

static ssize_t show_board_rev_name(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "Rev %c\n", d4400_sys_board_rev_letter());
}

static ssize_t show_leds(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	u8 leds = 0;

	mutex_lock(&d4400_sys_data->lock);

	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		{
			struct fsl_4t4r_board *brd_4t4r =
				(struct fsl_4t4r_board *)d4400_sys_data->brd;
			if (gpio_get_value(brd_4t4r->gpio_led1) > 0)
				leds |= (1 << 0);
			if (gpio_get_value(brd_4t4r->gpio_led2) > 0)
				leds |= (1 << 1);
		}
		break;
	case BOARD_TYPE_D4400_4T4RK1:
		{
			struct fsl_4t4rk1_board *brd_4t4rk1 =
				(struct fsl_4t4rk1_board *)d4400_sys_data->brd;
			if (gpio_get_value(brd_4t4rk1->gpio_led1) > 0)
				leds |= (1 << 0);
			if (gpio_get_value(brd_4t4rk1->gpio_led2) > 0)
				leds |= (1 << 1);
			if (gpio_get_value(brd_4t4rk1->gpio_led3) > 0)
				leds |= (1 << 2);
			if (gpio_get_value(brd_4t4rk1->gpio_led4) > 0)
				leds |= (1 << 3);
		}
		break;
	default:
		break;
	}
	mutex_unlock(&d4400_sys_data->lock);
	return sprintf(buf, "x%02x\n", leds);
}

static ssize_t set_leds(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int err;
	unsigned int val;

	mutex_lock(&d4400_sys_data->lock);
	err = kstrtouint(buf, 0, &val);
	if (err) {
		mutex_unlock(&d4400_sys_data->lock);
		return err;
	}

	d4400_sys_leds_set_clear(val, ~val);
	mutex_unlock(&d4400_sys_data->lock);

	return count;
}

/* Show PA signals include alarm state */
static ssize_t show_pa(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int ret = 0;
	char *p = buf;

	int pa;
	u32 pa_sig = 0;
	int *pa_pins;

	mutex_lock(&d4400_sys_data->lock);

	/* TODO: Implement optional parameters:
	 *  <#1> <#2> <#3>
	 *   #1: pa #, 1 or 2
	 *   #2: signal #, 0-31
	 * If the required parameters are not present, then
	 * all PA control signals are displayed.
	 */

	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		{
			struct fsl_4t4r_board *brd_4t4r =
				(struct fsl_4t4r_board *)d4400_sys_data->brd;

			for (pa = 0; pa < PA_CON_MAX; ++pa) {
				pa_sig = brd_4t4r->pac[pa].sig;
				pa_pins = brd_4t4r->pac[pa].pins;

				/* Update gpio inputs */
				pa_sig &= ~(1 << PA_ALARM1_B);
				if (gpio_get_value_cansleep(pa_pins[PA_ALARM1_B]))
					pa_sig |= (1 << PA_ALARM1_B);
				pa_sig &= ~(1 << PA_ALARM2_B);
				if (gpio_get_value_cansleep(pa_pins[PA_ALARM2_B]))
					pa_sig |= (1 << PA_ALARM2_B);

				/* Update */
				brd_4t4r->pac[pa].sig = pa_sig;

				sprintf(p, "   PA%d: x%08x   ",
					pa, pa_sig);
				p += strlen(p);
			}
			sprintf(p, "\n");
			p += strlen(p);
		}
		break;
	case BOARD_TYPE_D4400_4T4RK1:
		{
			struct fsl_4t4rk1_board *brd_4t4rk1 =
				(struct fsl_4t4rk1_board *)d4400_sys_data->brd;

			/* Get alarm signal state */
			pa_sig = get_pa_sig_4t4rk1(brd_4t4rk1);

			sprintf(p, "   PA0: Not implemented on this board\n");
			p += strlen(p);
			sprintf(p, "   PA1: x%08x   ", pa_sig);
			p += strlen(p);
			sprintf(p, "\n");
			p += strlen(p);
		}
		break;
	default:
		goto out_err;
		break;
	}
	mutex_unlock(&d4400_sys_data->lock);
	return strlen(buf);

out_err:
	mutex_unlock(&d4400_sys_data->lock);
	printk("\n   Parameters: <#1> <#2> <#3>\n");
	printk("     #1: PA #, 0 or 1\n");
	printk("     #2: 0-clear bits, 1-set bits\n");
	printk("     #3: 32-bit value (hex)\n");
	return ret;
}

/* Set PA signals */
static ssize_t set_pa(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret = 0;
	int pa, clearset;
	u32 val32;

	mutex_lock(&d4400_sys_data->lock);

	/* Required parameters:
	 *  <#1> <#2> <#3>
	 *   #1: pa #, 1 or 2
	 *   #2: set or clear bits, 0-to clear / nonzero-to set
	 *   #3: 32-bit value.  For setting, any bits set will have
	 *      the corresponding gpio set.  For clearing, any bits
	 *      set will have the corresponding gpio cleared.
	 */
	if (sscanf(buf, "%d %d %x", &pa, &clearset, &val32) != 3) {
		ret = -EINVAL;
		goto out_err;
	}
	if ((pa != 0) && (pa != 1)) {
		ret = -EINVAL;
		goto out_err;
	}
	if (clearset)
		ret = d4400_sys_pa_set_clear(pa, val32, 0);
	else
		ret = d4400_sys_pa_set_clear(pa, 0, val32);
	mutex_unlock(&d4400_sys_data->lock);
	goto out;

out_err:
	mutex_unlock(&d4400_sys_data->lock);
	printk("\n   Parameters: <#1> <#2> <#3>\n");
	printk("     #1: PA #, 0 or 1\n");
	printk("     #2: 0-clear bits, 1-set bits\n");
	printk("     #3: 32-bit value (hex)\n");
out:
	return count;
}

static ssize_t set_debug(struct device *dev,
	struct device_attribute *devattr,
	const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	mutex_lock(&d4400_sys_data->lock);
	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		mutex_unlock(&d4400_sys_data->lock);
		return ret;
	}
	d4400_sys_data->debug = val;
	mutex_unlock(&d4400_sys_data->lock);

	return count;
}

static ssize_t show_debug(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	u32 debug;

	mutex_lock(&d4400_sys_data->lock);
	debug = d4400_sys_data->debug;
	mutex_unlock(&d4400_sys_data->lock);

	return sprintf(buf, "%d / 0x%x\n", debug, debug);
}

static ssize_t show_reboot(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int reboot = 0;

	mutex_lock(&d4400_sys_data->lock);
	reboot = d4400_sys_data->reboot_method;
	mutex_unlock(&d4400_sys_data->lock);

	return sprintf(buf, "%d\n", reboot);
}

static ssize_t set_reboot(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int err;
	unsigned int val;

	err = kstrtouint(buf, 0, &val);
	if (err)
		return err;

	mutex_lock(&d4400_sys_data->lock);
	if (val)
		/* Simulated reboot */
		d4400_sys_data->reboot_method = 1;
	else
		/* Normal reboot via WDT, cpu reset */
		d4400_sys_data->reboot_method = 0;
	mutex_unlock(&d4400_sys_data->lock);

	d4400_set_reboot_method(d4400_sys_data->reboot_method);

	return count;
}

#ifdef CONFIG_BOARD_4T4RK1
static ssize_t show_rs485dir(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int pin, val = 0;
	struct fsl_4t4rk1_board *brd_4t4rk1 =
		(struct fsl_4t4rk1_board *)d4400_sys_data->brd;

	mutex_lock(&d4400_sys_data->lock);
	pin = brd_4t4rk1->gpio_rs485_dir;
	if (gpio_get_value_cansleep(pin)) {
		val = 1;
	}
	mutex_unlock(&d4400_sys_data->lock);

	if (val) {
		return sprintf(buf, "Output - %i\n", val);
	} else {
		return sprintf(buf, "Input - %i\n", val);
	}
}

static ssize_t set_rs485dir(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int err;
	unsigned int val;
	int pin;
	struct fsl_4t4rk1_board *brd_4t4rk1 =
		(struct fsl_4t4rk1_board *)d4400_sys_data->brd;

	err = kstrtouint(buf, 0, &val);
	if (err)
		return err;

	mutex_lock(&d4400_sys_data->lock);
	pin = brd_4t4rk1->gpio_rs485_dir;
	/* Bias voltage enable is active low */
	if (val > 0) {
		gpio_direction_output(pin, 1); /* Receiver disabled */
	} else {
		gpio_direction_output(pin, 0); /* Receiver enabled */
	}
	mutex_unlock(&d4400_sys_data->lock);

	return count;
}

static ssize_t show_rx8vbias(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int i, pin, val = 0;
	struct fsl_4t4rk1_board *brd_4t4rk1 =
		(struct fsl_4t4rk1_board *)d4400_sys_data->brd;

	mutex_lock(&d4400_sys_data->lock);
	/* Bias status (power good) is active low, 0-bias ON */
	for (i = 0; i < RX_8VBIAS_STAT_MAX; ++i) {
		pin = brd_4t4rk1->rx_8vbias_stat_pins[i];
		if (gpio_get_value_cansleep(pin)) {
			val |= (1 << i);
		}
	}
	mutex_unlock(&d4400_sys_data->lock);

	return sprintf(buf, "x%02x\n", val);
}

static ssize_t set_rx8vbias(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int err;
	unsigned int val;
	int i, pin;
	struct fsl_4t4rk1_board *brd_4t4rk1 =
		(struct fsl_4t4rk1_board *)d4400_sys_data->brd;

	err = kstrtouint(buf, 0, &val);
	if (err)
		return err;

	mutex_lock(&d4400_sys_data->lock);
	for (i = 0; i < RX_8VBIAS_EN_MAX; ++i) {
		pin = brd_4t4rk1->rx_8vbias_en_pins[i];
		if (val & (1 << i)) {
			gpio_direction_output(pin, 1); /* Turn off bias */
		} else {
			gpio_direction_output(pin, 0); /* Turn on bias */
		}
	}
	mutex_unlock(&d4400_sys_data->lock);

	return count;
}
#endif /* CONFIG_BOARD_4T4RK1 */

static DEVICE_ATTR(board_type,      S_IRUGO, show_board_type,      NULL);
static DEVICE_ATTR(board_type_name, S_IRUGO, show_board_type_name, NULL);
static DEVICE_ATTR(board_rev,       S_IRUGO, show_board_rev,       NULL);
static DEVICE_ATTR(board_rev_name,  S_IRUGO, show_board_rev_name,  NULL);
static DEVICE_ATTR(leds,  S_IWUSR | S_IRUGO, show_leds,            set_leds);
static DEVICE_ATTR(reboot, S_IWUSR | S_IRUGO, show_reboot,         set_reboot);
static DEVICE_ATTR(debug, S_IWUSR | S_IRUGO, show_debug,           set_debug);
static DEVICE_ATTR(ipmi_info, S_IRUGO, show_ipmi_info,             NULL);
static DEVICE_ATTR(pa,    S_IWUSR | S_IRUGO, show_pa,              set_pa);
#ifdef CONFIG_BOARD_4T4RK1
static DEVICE_ATTR(rs485_dir, S_IWUSR | S_IRUGO, show_rs485dir,    set_rs485dir);
static DEVICE_ATTR(rx_8vbias_en, S_IWUSR | S_IRUGO, show_rx8vbias, set_rx8vbias);
#endif

static struct attribute *d4400_sys_attributes[] = {
	&dev_attr_board_type.attr,
	&dev_attr_board_type_name.attr,
	&dev_attr_board_rev.attr,
	&dev_attr_board_rev_name.attr,
	&dev_attr_leds.attr,
	&dev_attr_reboot.attr,
	&dev_attr_debug.attr,
	&dev_attr_ipmi_info.attr,
	&dev_attr_pa.attr,
#ifdef CONFIG_BOARD_4T4RK1
	&dev_attr_rs485_dir.attr,
	&dev_attr_rx_8vbias_en.attr,
#endif
	NULL
};

static const struct attribute_group attr_group = {
	.attrs = d4400_sys_attributes,
};

static int d4400_sys_open(struct inode *i, struct file *f)
{
	int err = 0;

	/* For now, nothing to do. */

	return err;
}

static int d4400_sys_close(struct inode *i, struct file *f)
{
	return 0;
}

static long d4400_sys_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int err = 0;
	void __user *ioargp = (void __user *)arg;
	u32 temp;

	switch (cmd) {

	/* Return board type and revision */
	case NXP_D4400_SYS_BOARD_INFO:
		if (copy_to_user((struct d4400_sys_board_info *)ioargp,
				&d4400_sys_data->brd_info,
				sizeof(struct d4400_sys_board_info))) {
			err = -EFAULT;
			goto out;
		}
		break;

	/* Check presence of a transceiver */
	case NXP_D4400_SYS_XCVR_PRESENT:
		if (copy_from_user(&temp, (u32 *)ioargp, sizeof(u32))) {
			err = -EFAULT;
			goto out;
		}
		temp = d4400_sys_xcvr_present(temp);
		if (copy_to_user((u32 *)ioargp, &temp, sizeof(u32)))
			err = -EFAULT;
		break;

	/* Set / Clear the software LEDs */
	case NXP_D4400_SYS_LEDS_SET_CLEAR:
		err = d4400_sys_leds_set_clear((arg>>8)&0xFF, arg&0xFF);
		break;

	/* Set reboot method */
	case NXP_D4400_SYS_SET_REBOOT_METHOD:
		err = d4400_sys_set_reboot_method(arg);
		break;
	default:
		err = -EINVAL;
	}
	if (!err)
		return 0;
out:
	pr_err(D4400_SYS_MOD_NAME ": ioctl() error %d\n", err);
	return err;
}

static const struct file_operations d4400_sys_fops = {
	.owner = THIS_MODULE,
	.open = d4400_sys_open,
	.release = d4400_sys_close,
	.unlocked_ioctl = d4400_sys_ioctl,
};

static int eeprom_read(struct d4400_sys_i2c_eeprom *eeprom,
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

static struct ipmi_info *d4400_sys_verify_ipmi_info(
	struct d4400_sys_i2c_eeprom *eeprom)
{
	int ret = 0;
	u8 ipmi_rawbuf[IPMI_EEPROM_DATA_SIZE];
	struct ipmi_info *ipmi = NULL;

	memset(ipmi_rawbuf, 0, IPMI_EEPROM_DATA_SIZE);

	/* First try to read IPMI data with eeprom data retrieved from
	 * dts file.
	 */
	ret = eeprom_read(eeprom, ipmi_rawbuf, 0, IPMI_EEPROM_DATA_SIZE);
	if (ret) {
		/* If the addrlen is >1, then try addrlen=1 in case the
		 * eeprom is 256 byte.
		 */
		if (eeprom->addrlen > 1) {
			/* Retry with 1 byte addr, assume 256 byte size */
			eeprom->addrlen = 1;
			eeprom->pagesize = 16;
			eeprom->bytesize = 256;
		} else if (eeprom->addrlen == 1) {
			/* Retry with 2 byte addr, assume 64KB byte size */
			eeprom->addrlen = 2;
			eeprom->pagesize = 128;
			eeprom->bytesize = 65536;
		} else
			goto out0;

		/* Retry reading IPMI */
		ret = eeprom_read(eeprom, ipmi_rawbuf, 0,
			IPMI_EEPROM_DATA_SIZE);
		if (ret)
			goto out0;
		pr_info(D4400_SYS_MOD_NAME ": Eeprom retry success, set to %i byte size\n",
			eeprom->bytesize);
	}
	ipmi = kzalloc(sizeof(struct ipmi_info), GFP_KERNEL);
	if (!ipmi) {
		pr_err(D4400_SYS_MOD_NAME ": IPMI: Failed to allocate %d bytes for IPMI info\n",
			sizeof(struct ipmi_info));
		goto out0;
	}

	/* Create IPMI info */
	ret = ipmi_create(ipmi_rawbuf, ipmi);
	if (ret) {
		pr_warn(D4400_SYS_MOD_NAME ": IPMI decode failure, using default IPMI table\n");
		ret = ipmi_create((u8 *)default_ipmi_rawdata, ipmi);
		if (ret)
			goto out1;
	}
	pr_info(D4400_SYS_MOD_NAME " IPMI board mfg/name: %s / %s\n",
		ipmi->board.mfg_str, ipmi->board.name_str);

	return ipmi;
out1:
	ipmi_free(ipmi);
	kfree(ipmi);
out0:
	return NULL;
}

static struct d4400_sys_i2c_eeprom *get_eeprom_info(
	struct platform_device *pdev)
{
	struct device_node *eeprom_node;
	struct i2c_client *eeprom_client;
	struct d4400_sys_i2c_eeprom *eeprom;
	u32 pagesize;
	u32 bytesize;
	u32 addrlen;

	eeprom_node = of_parse_phandle(pdev->dev.of_node, "eeprom-handle", 0);
	if (!eeprom_node) {
		pr_err(D4400_SYS_MOD_NAME ": eeprom-handle not found\n");
		goto out;
	}

	eeprom_client = of_find_i2c_device_by_node(eeprom_node);
	if (!eeprom_client) {
		pr_err(D4400_SYS_MOD_NAME ": Failed to find eeprom i2c device handle\n");
		goto out;
	}

	if (of_property_read_u32(eeprom_node, "bytesize", &bytesize)) {
		pr_err(D4400_SYS_MOD_NAME ": bytesize property not found\n");
		goto out;
	}

	if (of_property_read_u32(eeprom_node, "pagesize", &pagesize)) {
		pr_err(D4400_SYS_MOD_NAME ": pagesize property not found\n");
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

	eeprom = kzalloc(sizeof(struct d4400_sys_i2c_eeprom), GFP_KERNEL);
	if (!eeprom) {
		pr_err(D4400_SYS_MOD_NAME ": Failed to allocate %d bytes for eeprom info\n",
			sizeof(struct d4400_sys_i2c_eeprom));
		goto out;
	}
	eeprom->client = eeprom_client;
	eeprom->addr = eeprom_client->addr;
	eeprom->pagesize = pagesize;
	eeprom->bytesize = bytesize;
	eeprom->addrlen = addrlen;
	return eeprom;
out:
	return NULL;
}

static int __init d4400_sys_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct device *sysfs_dev;

	if (!np || !of_device_is_available(np)) {
		pr_err(D4400_SYS_MOD_NAME ": No  interface device available\n");
		return -ENODEV;
	}

	d4400_sys_data = kzalloc(sizeof(struct d4400_sys_dev), GFP_KERNEL);
	if (!d4400_sys_data) {
		pr_err(D4400_SYS_MOD_NAME ": Failed to allocate device data\n");
		ret = -ENOMEM;
		goto out;
	}
	d4400_sys_data->dev = dev;
	mutex_init(&d4400_sys_data->lock);

	/* Create char device */
	ret = alloc_chrdev_region(&d4400_sys_data->devt, 0, 1,
		D4400_SYS_MOD_NAME);
	if (ret) {

		ret = -ENOMEM;
		goto out;
	}
	d4400_sys_data->major = MAJOR(d4400_sys_data->devt);
	d4400_sys_data->minor = MINOR(d4400_sys_data->devt);

	/* Create device class for sysfs */
	d4400_sys_data->class = class_create(THIS_MODULE, D4400_SYS_MOD_NAME);
	if (IS_ERR(d4400_sys_data->class)) {
		ret = PTR_ERR(d4400_sys_data->class);
		goto out_class;
	}

	/* Create character device */
	cdev_init(&d4400_sys_data->dev_cdev, &d4400_sys_fops);
	d4400_sys_data->dev_cdev.owner = THIS_MODULE;
	ret = cdev_add(&d4400_sys_data->dev_cdev, d4400_sys_data->devt, 1);
	if (ret) {
		pr_err(D4400_SYS_MOD_NAME ": Error %d while adding cdev",
			ret);
		goto out_cdev;
	}

	/* Create the device node in /dev */
	sysfs_dev = device_create(d4400_sys_data->class, &pdev->dev,
		d4400_sys_data->devt,	NULL, D4400_SYS_MOD_NAME);
	if (NULL == sysfs_dev) {
		ret = PTR_ERR(sysfs_dev);
		pr_err(D4400_SYS_MOD_NAME ": Error %d while creating device",
			 ret);
		goto out_dev;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret < 0) {
		pr_err(D4400_SYS_MOD_NAME ": Error %d while creating group",
			ret);
		goto out_group;
	}
	dev_set_drvdata(&pdev->dev, d4400_sys_data);

	/* Set default in case IPMI data is not found */
	d4400_sys_data->brd_info.board_type = BOARD_TYPE_UNKNOWN;
	d4400_sys_data->brd_info.board_rev = BOARD_REV_UNKNOWN;

	/* Get system eeprom info */
	d4400_sys_data->eeprom = get_eeprom_info(pdev);
	d4400_sys_data->ipmi = d4400_sys_verify_ipmi_info(
		d4400_sys_data->eeprom);
	if (!d4400_sys_data->ipmi)
		pr_err(D4400_SYS_MOD_NAME ": Bad IPMI data in eeprom.  Using defaults.");
	else {
		/* Get board type/rev */
		d4400_sys_data->brd_info.board_type = fsl_get_board_type(
			d4400_sys_data->ipmi->board.name_str);
		d4400_sys_data->brd_info.board_rev = fsl_get_board_rev(
			d4400_sys_data->ipmi->board.partnum_str);
	}
	/* Setup the board */
	ret = d4400_sys_setup_board(d4400_sys_data);
	if (ret)
		goto out_group;

	return 0;

out_group:
	device_destroy(d4400_sys_data->class, d4400_sys_data->devt);
out_dev:
	cdev_del(&d4400_sys_data->dev_cdev);
out_cdev:
	class_destroy(d4400_sys_data->class);
out_class:
	unregister_chrdev_region(d4400_sys_data->devt, 1);
out:
	kfree(d4400_sys_data);
	return ret;
}

static int __exit d4400_sys_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct d4400_sys_dev *d4400_sys_data = dev_get_drvdata(&pdev->dev);
	int pa;

	switch (d4400_sys_data->brd_info.board_type) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		{
			struct fsl_4t4r_board *brd_4t4r =
				(struct fsl_4t4r_board *)d4400_sys_data->brd;
			int i, pin;

			for (pa = 0; pa < PA_CON_MAX; ++pa) {
				/* Release gpio */
				for (i = 0; i < PA_PINCNT; ++i)  {
					pin = brd_4t4r->pac[pa].pins[i];
					if (pin)
						gpio_free(pin);
				}
			}
		}
		break;
	default:
		break;
	}

	sysfs_remove_group(&pdev->dev.kobj, &attr_group);
	device_destroy(d4400_sys_data->class, d4400_sys_data->devt);
	cdev_del(&d4400_sys_data->dev_cdev);
	class_destroy(d4400_sys_data->class);
	unregister_chrdev_region(d4400_sys_data->devt, 1);

	kfree(d4400_sys_data);
	return ret;
}

static struct of_device_id d4400_sys_ids[] = {
	{.compatible = "fsl,d4400-sys", },
	{ /* sentinel */ }
};

static struct platform_driver d4400_sys_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = d4400_sys_ids,
	},
	.remove = __exit_p(d4400_sys_remove),
};

static int __init d4400_sys_init(void)
{
	return platform_driver_probe(&d4400_sys_driver,
		d4400_sys_probe);
}

static void __exit d4400_sys_exit(void)
{
	platform_driver_unregister(&d4400_sys_driver);
}

module_init(d4400_sys_init);
module_exit(d4400_sys_exit);

MODULE_DESCRIPTION("NXP DFE D4400 system driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

/* The following code implements the ability for users to write new
 * IPMI raw data to the eeprom via sysfs file "ipmi_info" in the
 * d4400 system driver class: /sys/class/d4400-sys/d4400-sys/device
 * This code has been tested.
 */
#if 0
static DEVICE_ATTR(ipmi_info, S_IWUSR | S_IRUGO, show_ipmi_info, set_ipmi_info);

/* Get the eeprom offset address.  Part of this address could
 * reside in the i2c slave address itself if the eeprom size is
 * larger than a 16-bit address can hold.
 */
static void eeprom_get_addr(int bytesize, int start_addr, int *addr,
	int *eeprom_addr)
{
	/* For sizes 256 bytes or less, don't need to do anything */
	if (bytesize <= 256) {
		*addr = start_addr;
		return;
	}

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

static int eeprom_write(struct d4400_sys_i2c_eeprom *eeprom,
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
	msgschk[0].addr = eeprom_addr;
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

		if (eeprom->bytesize > 256) {
			/* Put in the address bytes, MSB first */
			for (i = 0; i < eeprom->addrlen; ++i) {
				shift = (8 * i);
				*dest++ = (u8)((addr >> shift) & 0xff);
			}
		} else
			*dest++ = (u8)addr;

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
			mdelay(10);
		}
		if (ret)
			break;

		cur_addr += data_size;
		left -= data_size;
	}

	kfree(tmpbuf);

	return ret;
}

static ssize_t set_ipmi_info(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	u8 ipmi_rawbuf[IPMI_EEPROM_DATA_SIZE];
	struct ipmi_info *ipmi = NULL;

	if (count < IPMI_EEPROM_DATA_SIZE) {
		pr_err(D4400_SYS_MOD_NAME ": Insufficient data for IPMI info, %i bytes (%i needed)\n",
			count, IPMI_EEPROM_DATA_SIZE);
		return count;
	}
	memset(ipmi_rawbuf, 0, IPMI_EEPROM_DATA_SIZE);

	mutex_lock(&d4400_sys_data->lock);
	memcpy(ipmi_rawbuf, buf, IPMI_EEPROM_DATA_SIZE);
	mutex_unlock(&d4400_sys_data->lock);

	/* Test the raw data to make sure it is valid IPMI data. */
	ipmi = kzalloc(sizeof(struct ipmi_info), GFP_KERNEL);
	if (!ipmi) {
		pr_err(D4400_SYS_MOD_NAME ": IPMI: Failed to allocate %d bytes for IPMI info\n",
			sizeof(struct ipmi_info));
		goto out0;
	}

	/* Create IPMI info as a test */
	ret = ipmi_create(ipmi_rawbuf, ipmi);
	if (ret) {
		pr_err(D4400_SYS_MOD_NAME ": IPMI: Failed to find valid IPMI information\n");
		goto out1;
	}

	/* Its good, write to eeprom */
	mutex_lock(&d4400_sys_data->lock);
	ret = eeprom_write(d4400_sys_data->eeprom, ipmi_rawbuf, 0,
		IPMI_EEPROM_DATA_SIZE);
	mutex_unlock(&d4400_sys_data->lock);
	if (ret) {
		pr_err(D4400_SYS_MOD_NAME ": IPMI: Failed program new IPMI data to eeprom\n");
		goto out1;
	}
	pr_info(D4400_SYS_MOD_NAME ": IPMI: new IPMI data programmed to eeprom successfully\n");

	/* Reload new IPMI data */
	mutex_lock(&d4400_sys_data->lock);
	d4400_sys_data->ipmi = d4400_sys_verify_ipmi_info(
		d4400_sys_data->eeprom);
	mutex_unlock(&d4400_sys_data->lock);
out1:
	ipmi_free(ipmi);
	kfree(ipmi);
out0:
	return count;
}
#endif

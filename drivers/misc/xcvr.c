/*
 * File: xcvr.c
 * Driver for the ADI transceiver cards, currently supports ADI ROC card
 * and wideband card.
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
#include <linux/device.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/qixis.h>
#include <linux/xcvr.h>
#include <uapi/linux/xcvr.h>
#include <mach/src.h>
#include <linux/overlay_ext.h>

#define DEV_NAME "xcvr"
#define MAX_SPI_CNT	32
#define MAX_INPUT_GPIO	32
#define MAX_OUTPUT_GPIO	32

static int attached_xcvr;
static struct class *xcvr_class;
static dev_t xcvr_dev_t;
static struct cdev xcvr_cdev;
static struct spi_device *spi_devs[MAX_SPI_CNT];
static int input_pins[MAX_INPUT_GPIO] = {[0 ... (MAX_INPUT_GPIO - 1)] = -1};
static int output_pins[MAX_OUTPUT_GPIO] = {[0 ... (MAX_OUTPUT_GPIO - 1)] = -1};
static int ad9680_pins[3] = {[0 ... 2] = -1};
static int pe43711_ss_pins[3] = {[0 ... 2] = -1}; /* Three pe43711 devices for wb2 */
static int lmx2492_ss_pins[3] = {[0 ... 2] = -1}; /* Three lmx2492 devices for wb2 */
/* Assume that every spi chip might have a reset */
static int reset_src[MAX_SPI_CNT] = {[0 ... (MAX_SPI_CNT - 1)] = -1};

/* spi channels */
struct property xcvr_spi1_0_prop[] = {
	{ .name = "name",              .value = "xcvr_spi1_0", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi1_0", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "0",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi1_1_prop[] = {
	{ .name = "name",              .value = "xcvr_spi1_1", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi1_1", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "1",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi1_2_prop[] = {
	{ .name = "name",              .value = "xcvr_spi1_2", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi1_2", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "2",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct device_node xcvr_ecspi1[] = {
	{ .name = "xcvr_spi1_0", .full_name = "xcvr_spi1_0@0", .properties = xcvr_spi1_0_prop, },
	{ .name = "xcvr_spi1_1", .full_name = "xcvr_spi1_1@1", .properties = xcvr_spi1_1_prop, },
	{ .name = "xcvr_spi1_2", .full_name = "xcvr_spi1_2@2", .properties = xcvr_spi1_2_prop, },
	{}
};
struct property xcvr_spi2_0_prop[] = {
	{ .name = "name",              .value = "xcvr_spi2_0", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi2_0", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "0",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct device_node xcvr_ecspi2[] = {
	{ .name = "xcvr_spi2_0", .full_name = "xcvr_spi2_0@0", .properties = xcvr_spi2_0_prop, },
	{}
};
struct property xcvr_spi3_0_prop[] = {
	{ .name = "name",              .value = "xcvr_spi3_0", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi3_0", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "0",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi3_1_prop[] = {
	{ .name = "name",              .value = "xcvr_spi3_1", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi3_1", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "1",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi3_2_prop[] = {
	{ .name = "name",              .value = "xcvr_spi3_2", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi3_2", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "2",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct device_node xcvr_ecspi3[] = {
	{ .name = "xcvr_spi3_0", .full_name = "xcvr_spi3_0@0", .properties = xcvr_spi3_0_prop, },
	{ .name = "xcvr_spi3_1", .full_name = "xcvr_spi3_1@1", .properties = xcvr_spi3_1_prop, },
	{ .name = "xcvr_spi3_2", .full_name = "xcvr_spi3_2@2", .properties = xcvr_spi3_2_prop, },
	{}
};
struct property xcvr_spi5_0_prop[] = {
	{ .name = "name",              .value = "xcvr_spi5_0", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi5_0", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "0",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct device_node xcvr_ecspi5[] = {
	{ .name = "xcvr_spi5_0", .full_name = "xcvr_spi5_0@0", .properties = xcvr_spi5_0_prop, },
	{}
};
struct property xcvr_spi6_0_prop[] = {
	{ .name = "name",              .value = "xcvr_spi6_0", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi6_0", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "0",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi6_1_prop[] = {
	{ .name = "name",              .value = "xcvr_spi6_1", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi6_1", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "1",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi6_2_prop[] = {
	{ .name = "name",              .value = "xcvr_spi6_2", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi6_2", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "2",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi6_3_prop[] = {
	{ .name = "name",              .value = "xcvr_spi6_3", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi6_3", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "3",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct device_node xcvr_ecspi6[] = {
	{ .name = "xcvr_spi6_0", .full_name = "xcvr_spi6_0@0", .properties = xcvr_spi6_0_prop, },
	{ .name = "xcvr_spi6_1", .full_name = "xcvr_spi6_1@1", .properties = xcvr_spi6_1_prop, },
	{ .name = "xcvr_spi6_2", .full_name = "xcvr_spi6_2@2", .properties = xcvr_spi6_2_prop, },
	{ .name = "xcvr_spi6_3", .full_name = "xcvr_spi6_3@3", .properties = xcvr_spi6_3_prop, },
	{}
};
struct property xcvr_spi7_0_prop[] = {
	{ .name = "name",              .value = "xcvr_spi7_0", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi7_0", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "0",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct device_node xcvr_ecspi7[] = {
	{ .name = "xcvr_spi7_0", .full_name = "xcvr_spi7_0@0", .properties = xcvr_spi7_0_prop, },
	{}
};
struct property xcvr_spi8_0_prop[] = {
	{ .name = "name",              .value = "xcvr_spi8_0", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi8_0", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "0",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi8_1_prop[] = {
	{ .name = "name",              .value = "xcvr_spi8_1", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi8_1", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "1",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct property xcvr_spi8_2_prop[] = {
	{ .name = "name",              .value = "xcvr_spi8_2", .length = 1, ._flags = PROP_STR },
	{ .name = "compatible",        .value = "xcvr_spi8_2", .length = 1, ._flags = PROP_STR },
	{ .name = "spi-max-frequency", .value = "1312D00",     .length = 1, ._flags = PROP_NUM },
	{ .name = "reg",               .value = "2",           .length = 1, ._flags = PROP_NUM },
	{}
};
struct device_node xcvr_ecspi8[] = {
	{ .name = "xcvr_spi8_0", .full_name = "xcvr_spi8_0@0", .properties = xcvr_spi8_0_prop, },
	{ .name = "xcvr_spi8_1", .full_name = "xcvr_spi8_1@1", .properties = xcvr_spi8_1_prop, },
	{ .name = "xcvr_spi8_2", .full_name = "xcvr_spi8_2@2", .properties = xcvr_spi8_2_prop, },
	{}
};

/* wideband spi devices attributes */
static struct spi_device_params wideband_spi_params[] = {
	{WB_SPIADC_SENSOR,		SPI_LEN_AD7689, 16,	0, SPI_MODE_0, 15000000},
	{WB_AD9680,			SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_AD9144,			SPI_LEN_NORMAL, 24,	0, SPI_MODE_3, 5000000},
	{WB_SRX_ATTN,			SPI_LEN_PE4312, 6,	0, SPI_MODE_0, 5000000},
	{WB_DCCLO_LMX2485,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_TXLO_LMX2485,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_SRXLO_LMX2485,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_REFLO_ADF4002,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_TX1_ATTN,			SPI_LEN_PE4312, 6,	0, SPI_MODE_0, 5000000},
	{WB_TX2_ATTN,			SPI_LEN_PE4312, 6,	0, SPI_MODE_0, 5000000},
	{WB_GBM_SRX_OFFSET_DAC,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_1, 5000000},
	{WB_SRX_DIFF_OFFSET_DAC,	SPI_LEN_NORMAL, 24,	0, SPI_MODE_1, 5000000},
};

/* wideband spi devices binding to dtb */
static struct of_device_id spi_match_wb[] = {
	{.compatible = "xcvr_spi1_0", .data = &wideband_spi_params[WB_SPIADC_SENSOR]},
	{.compatible = "xcvr_spi2_0", .data = &wideband_spi_params[WB_AD9680]},
	{.compatible = "xcvr_spi3_0", .data = &wideband_spi_params[WB_AD9144]},
	{.compatible = "xcvr_spi5_0", .data = &wideband_spi_params[WB_SRX_ATTN]},
	{.compatible = "xcvr_spi6_0", .data = &wideband_spi_params[WB_DCCLO_LMX2485]},
	{.compatible = "xcvr_spi6_1", .data = &wideband_spi_params[WB_TXLO_LMX2485]},
	{.compatible = "xcvr_spi6_2", .data = &wideband_spi_params[WB_SRXLO_LMX2485]},
	{.compatible = "xcvr_spi6_3", .data = &wideband_spi_params[WB_REFLO_ADF4002]},
	{.compatible = "xcvr_spi7_0", .data = &wideband_spi_params[WB_TX1_ATTN]},
	{.compatible = "xcvr_spi8_0", .data = &wideband_spi_params[WB_TX2_ATTN]},
	{.compatible = "xcvr_spi8_1", .data = &wideband_spi_params[WB_GBM_SRX_OFFSET_DAC]},
	{.compatible = "xcvr_spi8_2", .data = &wideband_spi_params[WB_SRX_DIFF_OFFSET_DAC]},
	{},
};

/* wideband spi overlay */
struct fragment_node frag_xcvr_spi_wb[] = {
	{
		.target = "&ecspi1",
		.np_array = xcvr_ecspi1,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi2",
		.np_array = xcvr_ecspi2,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi3",
		.np_array = xcvr_ecspi3,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi5",
		.np_array = xcvr_ecspi5,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi6",
		.np_array = xcvr_ecspi6,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi7",
		.np_array = xcvr_ecspi7,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi8",
		.np_array = xcvr_ecspi8,
		.ov_prop_array = NULL,
	},
	{}
};

/* wideband2 spi devices attributes */
static struct spi_device_params wideband2_spi_params[] = {
	{WB_SPIADC_SENSOR,		SPI_LEN_AD7689, 16,	0, SPI_MODE_0, 15000000},
	{WB_AD9680,			SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_AD9144,			SPI_LEN_NORMAL, 24,	0, SPI_MODE_3, 5000000},
	{WB_SRX_ATTN_PE43711,		SPI_LEN_PE4312, 8,	0, SPI_MODE_0, 5000000},
	{WB_DCCLO_LMX2492,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_TXLO_LMX2492,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_SRXLO_LMX2492,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_REFLO_ADF4002,		SPI_LEN_NORMAL, 24,	0, SPI_MODE_0, 10000000},
	{WB_TX1_ATTN_PE43711,		SPI_LEN_PE4312, 8,	0, SPI_MODE_0, 5000000},
	{WB_TX2_ATTN_PE43711,		SPI_LEN_PE4312, 8,	0, SPI_MODE_0, 5000000},
	{WB_GBM_SRX_OFFSET_DAC_AD5457R,	SPI_LEN_NORMAL, 24,	0, SPI_MODE_1, 5000000},
	{WB_SRX_DIFF_OFFSET_DAC_AD5457R, SPI_LEN_NORMAL, 24,	0, SPI_MODE_1, 5000000},
};

/* wideband2 spi devices binding to dtb */
static struct of_device_id spi_match_wb2[] = {
	{.compatible = "xcvr_spi1_0", .data = &wideband2_spi_params[WB_SPIADC_SENSOR]},
	{.compatible = "xcvr_spi2_0", .data = &wideband2_spi_params[WB_AD9680]},
	{.compatible = "xcvr_spi3_0", .data = &wideband2_spi_params[WB_AD9144]},
	{.compatible = "xcvr_spi5_0", .data = &wideband2_spi_params[WB_SRX_ATTN_PE43711]},
	{.compatible = "xcvr_spi6_0", .data = &wideband2_spi_params[WB_DCCLO_LMX2492]},
	{.compatible = "xcvr_spi6_1", .data = &wideband2_spi_params[WB_TXLO_LMX2492]},
	{.compatible = "xcvr_spi6_2", .data = &wideband2_spi_params[WB_SRXLO_LMX2492]},
	{.compatible = "xcvr_spi6_3", .data = &wideband2_spi_params[WB_REFLO_ADF4002]},
	{.compatible = "xcvr_spi7_0", .data = &wideband2_spi_params[WB_TX1_ATTN_PE43711]},
	{.compatible = "xcvr_spi8_0", .data = &wideband2_spi_params[WB_TX2_ATTN_PE43711]},
	{.compatible = "xcvr_spi8_1", .data = &wideband2_spi_params[WB_GBM_SRX_OFFSET_DAC]},		/* AD5724 */
	{.compatible = "xcvr_spi8_2", .data = &wideband2_spi_params[WB_SRX_DIFF_OFFSET_DAC_AD5457R]},	/* AD5754 */
	{},
};

/* wideband2 spi overlay */
struct fragment_node frag_xcvr_spi_wb2[] = {
	{
		.target = "&ecspi1",
		.np_array = xcvr_ecspi1,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi2",
		.np_array = xcvr_ecspi2,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi3",
		.np_array = xcvr_ecspi3,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi5",
		.np_array = xcvr_ecspi5,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi6",
		.np_array = xcvr_ecspi6,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi7",
		.np_array = xcvr_ecspi7,
		.ov_prop_array = NULL,
	},
	{
		.target = "&ecspi8",
		.np_array = xcvr_ecspi8,
		.ov_prop_array = NULL,
	},
	{}
};

/* roc devices attribute */
static struct spi_device_params roc_spi_params[] = {
	{AD93681, SPI_LEN_NORMAL, 24, 0, SPI_MODE_0, 20000000},
	{AD93682, SPI_LEN_NORMAL, 24, 0, SPI_MODE_0, 20000000},
	{AD9525,  SPI_LEN_NORMAL, 24, 0, SPI_MODE_0, 15000000},
	{AD93681 + ROC_SPI_CNT, SPI_LEN_NORMAL, 24, 0, SPI_MODE_0, 20000000},
	{AD93682 + ROC_SPI_CNT, SPI_LEN_NORMAL, 24, 0, SPI_MODE_0, 20000000},
	{AD9525 + ROC_SPI_CNT,  SPI_LEN_NORMAL, 24, 0, SPI_MODE_0, 15000000},
};

/* roc0 devices binding to dtb */
static struct of_device_id spi_match_roc0[] = {
#ifdef CONFIG_BOARD_EVB /* and RDB */
	{.compatible = "xcvr_spi1_0", .data = &roc_spi_params[AD93682]},
	{.compatible = "xcvr_spi5_1", .data = &roc_spi_params[AD9525]},
	{.compatible = "xcvr_spi1_2", .data = &roc_spi_params[AD93681]},
#elif defined CONFIG_BOARD_4T4R || defined CONFIG_BOARD_4T4RK1
	{.compatible = "xcvr_spi1_1", .data = &roc_spi_params[AD93681]}, /* tx */
	{.compatible = "xcvr_spi5_0", .data = &roc_spi_params[AD9525]},
	{.compatible = "xcvr_spi1_0", .data = &roc_spi_params[AD93682]}, /* rx */
#else
#error xcvr.c No board defined for ROC_0 transceiver
#endif
	{},
};

/* roc0 spi overlay */
struct fragment_node frag_xcvr_spi_roc0[] = {
	{
		.target = "&ecspi1",
		.np_array = xcvr_ecspi1,
		.ov_prop_array = NULL,
	},
	{}
};

/* roc1 devices binding to dtb */
static struct of_device_id spi_match_roc1[] = {
#ifdef CONFIG_BOARD_EVB /* and RDB */
	{.compatible = "xcvr_spi3_0",
			.data = &roc_spi_params[AD93682 + ROC_SPI_CNT]},
	{.compatible = "xcvr_spi3_1",
			.data = &roc_spi_params[AD9525 + ROC_SPI_CNT]},
	{.compatible = "xcvr_spi3_2",
			.data = &roc_spi_params[AD93681 + ROC_SPI_CNT]},
#elif defined CONFIG_BOARD_4T4R || defined CONFIG_BOARD_4T4RK1
	{.compatible = "xcvr_spi3_0",
			.data = &roc_spi_params[AD93682 + ROC_SPI_CNT]}, /* tx */
	{.compatible = "xcvr_spi5_0",
			.data = &roc_spi_params[AD9525 + ROC_SPI_CNT]},
	{.compatible = "xcvr_spi3_1",
			.data = &roc_spi_params[AD93681 + ROC_SPI_CNT]}, /* rx */
#else
#error xcvr.c No board defined for ROC_1 transceiver
#endif
	{},

};

/* roc1 spi overlay */
struct fragment_node frag_xcvr_spi_roc1[] = {
	{
		.target = "&ecspi3",
		.np_array = xcvr_ecspi3,
		.ov_prop_array = NULL,
	},
	{}
};

static struct of_device_id wb_misc_match[] = {
	{.compatible = "fsl,xcvr-wb",},
	{},
};

static struct of_device_id wb2_misc_match[] = {
	{.compatible = "fsl,xcvr-wb2",},
	{},
};

static struct of_device_id roc_misc_match[] = {
	{.compatible = "adi,xcvr-roc",},
	{},
};


#define XCVR_SPI_PROBE_FUNC(board)					\
static int spi_probe_##board(struct spi_device *spi)			\
{									\
	const struct of_device_id *match;				\
	const struct spi_device_params *spi_params;			\
									\
	match = of_match_device(spi_match_##board, &spi->dev);		\
	if (!match)							\
		return -ENODEV;						\
									\
	spi_params = (const struct spi_device_params *)(match->data);	\
	if (spi_params->id > MAX_SPI_CNT)				\
		return -EINVAL;						\
									\
	dev_set_drvdata(&spi->dev, (void *)spi_params);			\
	spi_devs[spi_params->id] = spi;					\
									\
	return 0;							\
}

#define XCVR_SPI_DRIVER(board)						\
static struct spi_driver spi_driver_##board = {				\
	.driver = {							\
		.name = #board "spi_driver",				\
		.bus = &spi_bus_type,					\
		.owner = THIS_MODULE,					\
		.of_match_table = of_match_ptr(spi_match_##board),	\
		},							\
	.probe = spi_probe_##board,					\
};

XCVR_SPI_PROBE_FUNC(roc0)	/* spi_probe_roc0 */
XCVR_SPI_PROBE_FUNC(roc1)	/* spi_probe_roc1 */
XCVR_SPI_PROBE_FUNC(wb)		/* spi_probe_wb */
XCVR_SPI_PROBE_FUNC(wb2)	/* spi_probe_wb2 */

XCVR_SPI_DRIVER(roc0)		/* spi_driver_roc0 */
XCVR_SPI_DRIVER(roc1)		/* spi_driver_roc1 */
XCVR_SPI_DRIVER(wb)		/* spi_driver_wb */
XCVR_SPI_DRIVER(wb2)		/* spi_driver_wb2 */

static void wb_attenuator_latch_toggle(struct xcvr_spi_buf *spi_buf)
{
	int ss_pin;	/* spi slave select function */
	int gpio_pin;	/* gpio function */
	int latch_en;

	if (spi_buf->chip_id == WB_SRX_ATTN) {
		/* spi5 SRX ss0-256, gpioD_30-257 */
		ss_pin = 256;
		gpio_pin = 257;
		latch_en = pe43711_ss_pins[0];
	} else if (spi_buf->chip_id == WB_TX1_ATTN) {
		/* spi7 TX1 ss0-295, gpioE_17-296 */
		ss_pin = 295;
		gpio_pin = 296;
		latch_en = pe43711_ss_pins[1];
	} else { /* WB_TX2_ATTN */
		/* spi8 TX2 ss0-316, gpioE_21-317 */
		ss_pin = 316;
		gpio_pin = 317;
		latch_en = pe43711_ss_pins[2];
	}
	/* Switch spi ss pin to gpio */
	d4400_pinmux_hack(gpio_pin);

	/* Do a lo-hi-lo pulse to make new reg value effective.
	 * The latency after the SPI write is completed and this
	 * function is called is about ~50-60 usec.  Each of the
	 * gpio function call below has about 4-8 usec latency
	 * which is enough that there is no need to add delay.
	 */
	gpio_direction_output(latch_en, 0);
	//msleep_interruptible(1);
	gpio_direction_output(latch_en, 1);
	//msleep_interruptible(1);
	gpio_direction_output(latch_en, 0);

	/* Switch back to spi ss pin */
	d4400_pinmux_hack(ss_pin);
}

static void wb_pll_latch_toggle(struct xcvr_spi_buf *spi_buf)
{
	int ss_pin;	/* spi slave select function */
	int gpio_pin;	/* gpio function */
	int latch_en;

	if (spi_buf->chip_id == WB_DCCLO_LMX2492) {
		/* spi6 DCC LO ss0-275, gpioE_13-276 */
		ss_pin = 275;
		gpio_pin = 276;
		latch_en = lmx2492_ss_pins[0];
	} else if (spi_buf->chip_id == WB_TXLO_LMX2492) {
		/* spi6 TX LO ss1-279, gpioE_27-280 */
 		ss_pin = 279;
		gpio_pin = 280;
		latch_en = lmx2492_ss_pins[1];
	} else { /* WB_SRXLO_LMX2492 */
		/* spi6 SRX LO ss2-603, gpioA_26-602 */
		ss_pin = 603;
		gpio_pin = 602;
		latch_en = lmx2492_ss_pins[2];
	}
	/* Switch spi ss pin to gpio */
	d4400_pinmux_hack(gpio_pin);

	/* Do a lo-hi-lo pulse to make new reg value effective.
	 * The latency after the SPI write is completed and this
	 * function is called is about ~50-60 usec.  Each of the
	 * gpio function call below has about 4-8 usec latency
	 * which is enough that there is no need to add delay.
	 */
	gpio_direction_output(latch_en, 0);
	//msleep_interruptible(1);
	gpio_direction_output(latch_en, 1);
	//msleep_interruptible(1);
	gpio_direction_output(latch_en, 0);

	/* Switch back to spi ss pin */
	d4400_pinmux_hack(ss_pin);
}

static void wb_ad9680_bitbang(struct xcvr_spi_buf *spi_buf)
{
	int i;
	int temp;
	u8 output = 0;
	u32 data = 0;

	int sdio = ad9680_pins[0];
	int sclk = ad9680_pins[1];
	int ss0 = ad9680_pins[2];

	data = spi_buf->tx_buf[1] | (u16)(spi_buf->tx_buf[2]) << 8;

	/* gpio mode for ecspi2's MOSI */
	/* gpio mode for ecspi's clk */
	/* gpio mode for ecspi's ss0 */
	d4400_pinmux_hack(199);
	d4400_pinmux_hack(206);
	d4400_pinmux_hack(209);

	gpio_direction_output(sdio, 0);
	gpio_direction_output(sclk, 0);
	gpio_direction_output(ss0, 0);

	for (i = 0; i < 16; i++) {
		if (data & (1 << (15 - i)))
			gpio_set_value(sdio, 1);
		else
			gpio_set_value(sdio, 0);

		ndelay(100);
		gpio_set_value(sclk, 1);
		ndelay(100);
		gpio_set_value(sclk, 0);
		ndelay(100);
	}

	gpio_direction_input(sdio);
	/* Now read 8 bits */
	for (i = 0; i < 8; i++) {
		gpio_set_value(sclk, 1);
		ndelay(100);
		temp = gpio_get_value(sdio) ? 1 : 0;
		output |= (temp << (7 - i));
		gpio_set_value(sclk, 0);
		ndelay(100);
	}

	gpio_set_value(ss0, 1);

	spi_buf->rx_buf[0] = output;
	/* Change back to spi mode */
	d4400_pinmux_hack(198);
	d4400_pinmux_hack(205);
	d4400_pinmux_hack(208);
}

static ssize_t xcvr_spi_operation(struct xcvr_spi_buf *spi_buf)
{
	struct spi_message m;
	struct spi_device *spi_dev;
	struct spi_device_params *spi_params;
	struct spi_transfer t;
	unsigned int chip_id = spi_buf->chip_id;
	enum board_type type = spi_buf->type;

	if (type == ROC1)
		chip_id += ROC_SPI_CNT;

	spi_dev = spi_devs[chip_id];
	spi_params = (struct spi_device_params *)dev_get_drvdata(&spi_dev->dev);

	memset(&t, 0, sizeof(struct spi_transfer));
	t.tx_buf = spi_buf->tx_buf;
	t.rx_buf = spi_buf->rx_buf;
	t.len = spi_params->len;
	t.bits_per_word = spi_params->bits_per_word;
	t.cs_change = spi_params->cs_change;
	t.speed_hz = spi_params->max_speed_hz;

	spi_dev->mode = spi_params->mode;
	spi_dev->bits_per_word = spi_params->bits_per_word;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(spi_dev, &m);
}

static void xcvr_gpio_operation(struct xcvr_gpio_buf *buf)
{
	int operation = buf->operation;
	int gpio_id = buf->gpio_id;
	int val = buf->val;
	enum board_type type = buf->type;

	if (type == ROC1)
		gpio_id += ROC_OUTPUT_PINCNT;

	if (operation == 0)
		buf->val = gpio_get_value(input_pins[gpio_id]) ? 1 : 0;
	else
		gpio_set_value(output_pins[gpio_id], val);
}

static int xcvr_open(struct inode *inode, struct file *filep)
{
	return 0;
}

int xcvr_release(struct inode *inode, struct file *filep)
{
	return 0;
}

static int verify_xcvr_spi_device(const struct xcvr_spi_buf *spi_buf)
{
	unsigned int chip_id = spi_buf->chip_id;
	enum board_type type = spi_buf->type;

	if ((type == ROC0 && (attached_xcvr & ROC0_PRESENT)) ||
		(type == ROC1 && (attached_xcvr & ROC1_PRESENT))) {
		if (chip_id > ROC_SPI_CNT)
			return -1;
	}

	if ((type == WIDEBAND) && (attached_xcvr == WB_PRESENT)) {
		if (chip_id > WB_SPI_CNT)
			return -1;
	}

	if ((type == WIDEBAND) && (attached_xcvr == WB2_PRESENT)) {
		if (chip_id > WB_SPI_CNT)
			return -1;
	}

	if (type == ROC1 && (attached_xcvr & ROC1_PRESENT))
		chip_id += ROC_SPI_CNT;

	if (!spi_devs[chip_id])
		return -1;

	return 0;
}

static int verify_xcvr_gpio(const struct xcvr_gpio_buf *gpio_buf)
{
	enum board_type type = gpio_buf->type;
	int gpio_id = gpio_buf->gpio_id;
	int operation = gpio_buf->operation;

	if ((type == ROC0) && (attached_xcvr & ROC0_PRESENT)) {
		if ((gpio_id <= ROC_OUTPUT_PINCNT) && (operation == 1))
			return output_pins[gpio_buf->gpio_id];
	}

	if ((type == ROC1) && (attached_xcvr & ROC1_PRESENT)) {
		if ((gpio_id <= ROC_OUTPUT_PINCNT) && (operation == 1))
			return output_pins[gpio_id + ROC_OUTPUT_PINCNT];
	}

	if ((type == WIDEBAND) && (attached_xcvr == WB_PRESENT)) {
		if (operation == 0 && gpio_id <= WB_INPUT_PINCNT)
			return input_pins[gpio_id];
		else if (operation == 1 && gpio_id <= WB_OUTPUT_PINCNT)
			return output_pins[gpio_id];
	}

	if ((type == WIDEBAND) && (attached_xcvr == WB2_PRESENT)) {
		if (operation == 0 && gpio_id <= WB_INPUT_PINCNT)
			return input_pins[gpio_id];
		else if (operation == 1 && gpio_id <= WB_OUTPUT_PINCNT)
			return output_pins[gpio_id];
	}

	return -1;
}

static int verify_xcvr_reset(const struct xcvr_reset_buf *buf)
{
	enum board_type type = buf->type;
	int chip_id = buf->chip_id;

	if ((type == ROC0 && (attached_xcvr & ROC0_PRESENT)) ||
		(type == WIDEBAND && (attached_xcvr == WB_PRESENT)) ||
		(type == WIDEBAND && (attached_xcvr == WB2_PRESENT)))
		return reset_src[chip_id];

	if (type == ROC1 && (attached_xcvr & ROC1_PRESENT))
		return reset_src[chip_id + ROC_SPI_CNT];

	return -1;
}

static void xcvr_reset_operation(const struct xcvr_reset_buf *buf)
{
	enum board_type type = buf->type;
	int chip_id = buf->chip_id;

	if (type == ROC1)
		chip_id += ROC_SPI_CNT;

	src_assert_reset(src_get_handle(NULL), SW_RST, reset_src[chip_id]);
	udelay(100);
}

static long xcvr_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	int ret  = 0;
	struct xcvr_spi_buf spi_buf;
	struct xcvr_gpio_buf gpio_buf;
	struct xcvr_reset_buf reset_buf;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case XCVR_READ_INFO:
		if (copy_to_user(argp, &attached_xcvr, sizeof(int)))
			ret = -EFAULT;
		break;

	case XCVR_SPI_OPERATION:
		if (copy_from_user(&spi_buf, argp,
				sizeof(struct xcvr_spi_buf)))
			return -EFAULT;

		if (verify_xcvr_spi_device(&spi_buf) < 0)
			ret = -ENODEV;
		else {
			if (((attached_xcvr == WB_PRESENT) ||
				(attached_xcvr == WB2_PRESENT))  &&
				spi_buf.chip_id == WB_AD9680 &&
				spi_buf.operation == READ_OP)
				wb_ad9680_bitbang(&spi_buf);
			else {
				xcvr_spi_operation(&spi_buf);

				/* PE43711 digital attenuator slave sel or
				 * latch enable is required to pulse after
				 * write to latch in value (wb2 board).
				 */
				if ((attached_xcvr == WB2_PRESENT) &&
				  (spi_buf.operation == WRITE_OP) &&
				  ((spi_buf.chip_id == WB_SRX_ATTN) ||
				  (spi_buf.chip_id == WB_TX1_ATTN) ||
				  (spi_buf.chip_id == WB_TX2_ATTN))) {
					wb_attenuator_latch_toggle(&spi_buf);
				}

				/* LMX2492 PLL LO generator slave sel or
				 * latch enable is required to pulse after
				 * write to latch in value (wb2 board).
				 */
				if ((attached_xcvr == WB2_PRESENT) &&
				  (spi_buf.operation == WRITE_OP) &&
				  ((spi_buf.chip_id == WB_DCCLO_LMX2492) ||
				  (spi_buf.chip_id == WB_TXLO_LMX2492) ||
				  (spi_buf.chip_id == WB_SRXLO_LMX2492))) {
					wb_pll_latch_toggle(&spi_buf);
				}
			}

			if (spi_buf.operation == READ_OP) {
				if (copy_to_user(argp, &spi_buf,
						sizeof(struct xcvr_spi_buf)))
					ret = -EFAULT;
			}
		}
		break;

	case XCVR_GPIO_OPERATION:
		if (copy_from_user(&gpio_buf, argp,
				sizeof(struct xcvr_gpio_buf)))
			return -EFAULT;

		if (verify_xcvr_gpio(&gpio_buf) < 0)
			ret = -ENODEV;
		else
			xcvr_gpio_operation(&gpio_buf);
			if (gpio_buf.operation == READ_OP) {
				if (copy_to_user(argp, &gpio_buf,
					sizeof(struct xcvr_gpio_buf)))
					ret = -EFAULT;
			}
		break;

	case XCVR_RESET_OPERATION:
		if (copy_from_user(&reset_buf, argp,
				sizeof(struct xcvr_reset_buf)))
			return -EFAULT;

		if (verify_xcvr_reset(&reset_buf) < 0)
			ret = -ENODEV;
		else
			xcvr_reset_operation(&reset_buf);
		break;

	default:
		ret = -ENOTTY;
	}

	return ret;
}


static const struct file_operations xcvr_fops = {
	.owner          = THIS_MODULE,
	.open           = xcvr_open,
	.release        = xcvr_release,
	.unlocked_ioctl = xcvr_ioctl,
};


static int wb_misc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int i = 0;
	int ret;
	if (!np || !of_device_is_available(np)) {
		pr_err("%s: No wb entry available in dtb\n",
			DEV_NAME);
		return -ENODEV;
	}

	ret = ov_load(frag_xcvr_spi_wb);
	if (ret < 0) {
		pr_err("%s: failed to create spi overlay with err %x\n",
				DEV_NAME, ret);
		goto out1;
	}

	ret = spi_register_driver(&spi_driver_wb);
	if (ret) {
		pr_err("%s: spi_register_driver failed with err %x\n",
				DEV_NAME, ret);
		goto out1;
	}

	for (i = 0; i < WB_INPUT_PINCNT; i++) {
		input_pins[i] =
			of_get_named_gpio(np, "input-gpios", i);
		ret = gpio_request(input_pins[i], "wb_input_pin");
		if (ret < 0)
			break;
		gpio_direction_input(input_pins[i]);
	}

	for (i = 0; i < WB_OUTPUT_PINCNT; i++) {
		output_pins[i] =
			of_get_named_gpio(np, "output-gpios", i);

		ret = gpio_request(output_pins[i], "wb_output_pin");
		if (ret < 0)
			break;
		gpio_direction_output(output_pins[i], 0);
	}

	/* AD9680 is a special device which uses bi-dir spi so
	 * we use gpio to bitbang it in read mode.
	 */
	for (i = 0; i < 3; i++) {
		ad9680_pins[i] =
			of_get_named_gpio(np, "ad9680-spi-pins", i);

		ret = gpio_request(ad9680_pins[i], "ad9680_pins");
		if (ret < 0)
			break;
		gpio_direction_output(ad9680_pins[i], 0);
	}

	of_property_read_u32(np, "tx-reset", &reset_src[WB_AD9144]);

	if (ret) {
		pr_err("%s fail\n", __func__);
		goto out0;
	}

	/* Mark as attached. */
	attached_xcvr |= WB_PRESENT;
out1:
	return ret;
out0:
	spi_unregister_driver(&spi_driver_wb);
	return ret;
}

static int wb_misc_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < WB_INPUT_PINCNT; i++)
		gpio_free(input_pins[i]);

	for (i = 0; i < WB_OUTPUT_PINCNT; i++)
		gpio_free(output_pins[i]);

	for (i = 0; i < 3; i++)
		gpio_free(ad9680_pins[i]);

	attached_xcvr &= ~WB_PRESENT;

	return 0;
}

static int wb2_misc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int i = 0;
	int ret;
	if (!np || !of_device_is_available(np)) {
		pr_err("%s: No wb entry available in dtb\n",
			DEV_NAME);
		return -ENODEV;
	}

	ret = ov_load(frag_xcvr_spi_wb2);
	if (ret < 0) {
		pr_err("%s: failed to create spi overlay with err %x\n",
				DEV_NAME, ret);
		goto out1;
	}

	ret = spi_register_driver(&spi_driver_wb2);
	if (ret) {
		pr_err("%s: spi_register_driver failed with err %x\n",
				DEV_NAME, ret);
		goto out1;
	}

	for (i = 0; i < WB_INPUT_PINCNT; i++) {
		input_pins[i] =
			of_get_named_gpio(np, "input-gpios", i);
		ret = gpio_request(input_pins[i], "wb_input_pin");
		if (ret < 0)
			break;
		gpio_direction_input(input_pins[i]);
	}

	for (i = 0; i < WB_OUTPUT_PINCNT; i++) {
		output_pins[i] =
			of_get_named_gpio(np, "output-gpios", i);
		ret = gpio_request(output_pins[i], "wb_output_pin");
		if (ret < 0)
			break;
		gpio_direction_output(output_pins[i], 0);
	}

	/* AD9680 is a special device which uses bi-dir spi so
	 * we use gpio to bitbang it in read mode.
	 */
	for (i = 0; i < 3; i++) {
		ad9680_pins[i] =
			of_get_named_gpio(np, "ad9680-spi-pins", i);

		ret = gpio_request(ad9680_pins[i], "ad9680_pins");
		if (ret < 0)
			break;
		gpio_direction_output(ad9680_pins[i], 0);
	}

	of_property_read_u32(np, "tx-reset", &reset_src[WB_AD9144]);

	/* PE43711 dig attenuator slave select / latch enable requires an
	 * extra pulse after data is written in order to make new value
	 * effective. Wb2 has 3 pe43711 devices (spi5 ss0, spi7 ss0, and
	 * spi8 ss0).
	 */
	for (i = 0; i < 3; i++) {
		pe43711_ss_pins[i] =
			of_get_named_gpio(np, "pe43711-ss-spi-pins", i);

		ret = gpio_request(pe43711_ss_pins[i], "pe43711_ss_pins");
		if (ret < 0)
			break;

		//printk("\t pe43711 ss pin %d: %d\n", i, pe43711_ss_pins[i]);
		gpio_direction_output(pe43711_ss_pins[i], 0);
	}

	/* LMX2492 PLL LO slave select / latch enable requires an extra
	 * pulse after data is written in order to make new value effective.
	 * Wb2 has 3 lmx2492 devices (spi6 ss0, spi6 ss1, and spi6 ss2).
	 */
	for (i = 0; i < 3; i++) {
		lmx2492_ss_pins[i] =
			of_get_named_gpio(np, "lmx2492-ss-spi-pins", i);

		ret = gpio_request(lmx2492_ss_pins[i], "lmx2492_ss_pins");
		if (ret < 0)
			break;

		//printk("\t lmx2492 ss pin %d: %d\n", i, lmx2492_ss_pins[i]);
		gpio_direction_output(lmx2492_ss_pins[i], 0);
	}

	if (ret) {
		pr_err("%s fail\n", __func__);
		goto out0;
	}

	/* Mark as attached. */
	attached_xcvr |= WB2_PRESENT;
out1:
	return ret;
out0:
	spi_unregister_driver(&spi_driver_wb2);
	return ret;
}

static int wb2_misc_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < WB_INPUT_PINCNT; i++)
		gpio_free(input_pins[i]);

	for (i = 0; i < WB_OUTPUT_PINCNT; i++)
		gpio_free(output_pins[i]);

	for (i = 0; i < 3; i++)
		gpio_free(ad9680_pins[i]);

	attached_xcvr &= ~WB2_PRESENT;

	return 0;
}

static int roc_misc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int id = 0;
	int i;
	int ret = 0;
	int connector = 0;
	int cur_xcvr = 0;

	if (!np || !of_device_is_available(np)) {
		pr_err("%s: No roc entry available in dtb\n",
			DEV_NAME);
		return -ENODEV;
	}

	if (of_property_read_u32(np, "connector-fmc", &connector)) {
		pr_err("%s: Can't find connector information\n", DEV_NAME);
		return -ENODEV;
	}

	if (connector == 1) {
		cur_xcvr = ROC0_PRESENT;
		ret = ov_load(frag_xcvr_spi_roc0);
		if (ret >= 0)
			ret = spi_register_driver(&spi_driver_roc0);
	} else if (connector == 2) {
		cur_xcvr = ROC1_PRESENT;
		ret = ov_load(frag_xcvr_spi_roc1);
		if (ret >= 0)
			ret = spi_register_driver(&spi_driver_roc1);
	} else {
		pr_err("%s: Invalid fmc connector value: %i\n", DEV_NAME,
			connector);
		return -ENODEV;
	}

	if (ret) {
		pr_err("%s: Failed to load spi driver\n", DEV_NAME);
		goto out;
	}

	of_property_read_u32(np, "id", &id);

	if (cur_xcvr & ROC0_PRESENT) {
		for (i = 0; i < ROC_OUTPUT_PINCNT; i++) {
			output_pins[i] = of_get_named_gpio(np, "cs-gpios", i);
			ret = gpio_request(output_pins[i], "roc_output_pin");
			if (ret < 0)
				break;
			gpio_direction_output(output_pins[i], 0);
		}
		of_property_read_u32(np, "tx-reset", &reset_src[AD93682]);
		of_property_read_u32(np, "rx-reset", &reset_src[AD93681]);
	}

	if (cur_xcvr & ROC1_PRESENT) {
		for (i = 0; i < ROC_OUTPUT_PINCNT; i++) {
			output_pins[i + ROC_OUTPUT_PINCNT] =
				of_get_named_gpio(np, "cs-gpios", i);
			ret = gpio_request(output_pins[i + ROC_OUTPUT_PINCNT],
				"roc_output_pin");
			if (ret < 0)
				break;
			gpio_direction_output(output_pins[i + ROC_OUTPUT_PINCNT], 0);
		}
		of_property_read_u32(np, "tx-reset",
			&reset_src[AD93682 + ROC_SPI_CNT]);
		of_property_read_u32(np, "rx-reset",
			&reset_src[AD93681 + ROC_SPI_CNT]);
	}

	if (ret) {
		pr_err("%s fail\n", __func__);
		goto out1;
	}

	/* Mark as attached. */
	attached_xcvr |= cur_xcvr;
	return 0;
out1:
	if (cur_xcvr == ROC0_PRESENT)
		spi_unregister_driver(&spi_driver_roc0);
	else if (cur_xcvr == ROC1_PRESENT)
		spi_unregister_driver(&spi_driver_roc1);
out:
	return ret;
}

static int roc_misc_remove(struct platform_device *pdev)
{
	int i;

	if (attached_xcvr & ROC0_PRESENT) {
		for (i = 0; i < ROC_OUTPUT_PINCNT; i++)
			gpio_free(output_pins[i]);
	}

	if (attached_xcvr & ROC1_PRESENT) {
		for (i = 0; i < ROC_OUTPUT_PINCNT; i++)
			gpio_free(output_pins[i + ROC_OUTPUT_PINCNT]);
	}
	attached_xcvr &= ~(ROC0_PRESENT | ROC1_PRESENT);

	return 0;
}

static struct platform_driver wb_misc_driver = {
	.probe          = wb_misc_probe,
	.remove         = wb_misc_remove,
	.driver         = {
		.name   = "wb_misc_driver",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(wb_misc_match),
	}
};

static struct platform_driver wb2_misc_driver = {
	.probe          = wb2_misc_probe,
	.remove         = wb2_misc_remove,
	.driver         = {
		.name   = "wb2_misc_driver",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(wb2_misc_match),
	}
};

static struct platform_driver roc_misc_driver = {
	.probe          = roc_misc_probe,
	.remove         = roc_misc_remove,
	.driver         = {
		.name   = "roc_misc_driver",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(roc_misc_match),
	}
};

static int __init xcvr_init(void)
{
	int ret = 0;
	struct device *dev;

	/* Transceiver drivers are registered so that the transceiver
	 * interface driver (xcvr_if) can load the xcvr driver(s) if the
	 * xcvr board(s) are connected.
	 */
	if (platform_driver_register(&roc_misc_driver)) {
		pr_err("%s: Error in loading ROC (ADI) transceiver driver\n",
			DEV_NAME);
		ret = -ENODEV;
		goto out0;
	}
	if (platform_driver_register(&wb_misc_driver)) {
		pr_err("%s: Error in loading wideband transceiver driver\n",
			DEV_NAME);
		ret = -ENODEV;
		goto out0;
	}

	if (platform_driver_register(&wb2_misc_driver)) {
		pr_err("%s: Error in loading wideband2 transceiver driver\n",
			DEV_NAME);
		ret = -ENODEV;
		goto out0;
	}

	xcvr_class = class_create(THIS_MODULE, "xcvr_ic");
	if (IS_ERR(xcvr_class)) {
		pr_err("%s: Unable to create xcvr_ic class\n", DEV_NAME);
		return PTR_ERR(xcvr_class);
	}

	ret = alloc_chrdev_region(&xcvr_dev_t, 0, 1, DEV_NAME);
	if (ret) {
		pr_err("%s: Failed to allocate chrdev region\n", DEV_NAME);
		goto out0;
	}

	cdev_init(&xcvr_cdev, &xcvr_fops);
	ret = cdev_add(&xcvr_cdev, xcvr_dev_t, 1);
	if (ret < 0) {
		pr_err("%s: Fail to add cdev\n", DEV_NAME);
		goto out1;
	}

	dev = device_create(xcvr_class, NULL,
		xcvr_dev_t, NULL, DEV_NAME);
	if (IS_ERR(dev)) {
		pr_err("%s: Fail to create sysfs entry\n", DEV_NAME);
		goto out2;
	}

	return 0;
out2:
	cdev_del(&xcvr_cdev);
out1:
	unregister_chrdev_region(xcvr_dev_t, 1);
out0:
	class_destroy(xcvr_class);
	return ret;
}

static void __exit xcvr_exit(void)
{
	switch (attached_xcvr) {
	case ROC0_PRESENT:
		platform_driver_unregister(&roc_misc_driver);
		spi_unregister_driver(&spi_driver_roc0);
		break;
	case ROC1_PRESENT:
		platform_driver_unregister(&roc_misc_driver);
		spi_unregister_driver(&spi_driver_roc1);
		break;
	case (ROC0_PRESENT | ROC1_PRESENT):
		platform_driver_unregister(&roc_misc_driver);
		spi_unregister_driver(&spi_driver_roc0);
		spi_unregister_driver(&spi_driver_roc1);
		break;
	case WB_PRESENT:
		platform_driver_unregister(&wb_misc_driver);
		spi_unregister_driver(&spi_driver_wb);
		break;
	case WB2_PRESENT:
		platform_driver_unregister(&wb2_misc_driver);
		spi_unregister_driver(&spi_driver_wb2);
		break;
	}

	cdev_del(&xcvr_cdev);
	unregister_chrdev_region(xcvr_dev_t, 1);
	device_destroy(xcvr_class, xcvr_dev_t);
	class_destroy(xcvr_class);
}

module_init(xcvr_init);
module_exit(xcvr_exit);

MODULE_DESCRIPTION("XCVR driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DEV_NAME);

/*
 * drivers/misc/gcr.c
 * AFD4400 GCR device driver
 *
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
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
 */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/gcr.h>

#define DRIVER_NAME "gcr"
#define MOD_VERSION "0.1"

/* Debug and error reporting macros */
#define IF_DEBUG(x)     if (debug & (x))
#define ERR(...)        {if (debug & DEBUG_MESSAGES) \
                                pr_err(DRIVER_NAME ": " __VA_ARGS__);}

struct gcr_field_desc {
	unsigned int gcr_id;
	unsigned int field_bit_from;
	unsigned int field_bit_to;
	unsigned int field_bit_val;
	const char *field_name;
	const char *field_desc;
};

#define	GCR_DESC(a, b, c, d, e, f) { .gcr_id = a,		\
					.field_bit_from = b,	\
					.field_bit_to = c,	\
					.field_bit_val = d,	\
					.field_name = e,	\
					.field_desc = f}
/* Copied from datasheet */
struct gcr_field_desc gcr_descs[] = {
	GCR_DESC(2, 21, 23, 0x1, "VSP11_CH11", "JESDTX_8 dma request"),
	GCR_DESC(2, 21, 23, 0x2, "VSP11_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(2, 21, 23, 0x3, "VSP11_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(2, 21, 23, 0x4, "VSP11_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(2, 21, 23, 0x5, "VSP11_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(2, 18, 20, 0x1, "VSP10_CH11", "JESDTX_8 dma request"),
	GCR_DESC(2, 18, 20, 0x2, "VSP10_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(2, 18, 20, 0x3, "VSP10_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(2, 18, 20, 0x4, "VSP10_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(2, 18, 20, 0x5, "VSP10_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(2, 15, 17, 0x1, "VSP9_CH11", "JESDTX_8 dma request"),
	GCR_DESC(2, 15, 17, 0x2, "VSP9_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(2, 15, 17, 0x3, "VSP9_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(2, 15, 17, 0x4, "VSP9_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(2, 15, 17, 0x5, "VSP9_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(2, 12, 14, 0x1, "VSP8_CH11", "JESDTX_8 dma request"),
	GCR_DESC(2, 12, 14, 0x2, "VSP8_CH11", "cpri_rx1 dma request 19"),
	GCR_DESC(2, 12, 14, 0x3, "VSP8_CH11", "cpri_rx1 dma request 22"),
	GCR_DESC(2, 12, 14, 0x4, "VSP8_CH11", "cpri_rx2 dma request 19"),
	GCR_DESC(2, 12, 14, 0x5, "VSP8_CH11", "cpri_rx2 dma request 22"),
	GCR_DESC(2, 9, 11, 0x1, "VSP11_CH10", "JESDTX_7 dma request"),
	GCR_DESC(2, 9, 11, 0x2, "VSP11_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(2, 9, 11, 0x3, "VSP11_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(2, 9, 11, 0x4, "VSP11_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(2, 9, 11, 0x5, "VSP11_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(2, 6, 8, 0x1, "VSP10_CH10", "JESDTX_7 dma request"),
	GCR_DESC(2, 6, 8, 0x2, "VSP10_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(2, 6, 8, 0x3, "VSP10_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(2, 6, 8, 0x4, "VSP10_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(2, 6, 8, 0x5, "VSP10_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(2, 3, 5, 0x1, "VSP9_CH10", "JESDTX_7 dma request"),
	GCR_DESC(2, 3, 5, 0x2, "VSP9_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(2, 3, 5, 0x3, "VSP9_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(2, 3, 5, 0x4, "VSP9_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(2, 3, 5, 0x5, "VSP9_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(2, 0, 2, 0x1, "VSP8_CH10", "JESDTX_7 dma request"),
	GCR_DESC(2, 0, 2, 0x2, "VSP8_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(2, 0, 2, 0x3, "VSP8_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(2, 0, 2, 0x4, "VSP8_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(2, 0, 2, 0x5, "VSP8_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(3, 21, 23, 0x1, "VSP11_CH11", "JESDTX_8 dma request"),
	GCR_DESC(3, 21, 23, 0x2, "VSP11_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(3, 21, 23, 0x3, "VSP11_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(3, 21, 23, 0x4, "VSP11_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(3, 21, 23, 0x5, "VSP11_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(3, 18, 20, 0x1, "VSP10_CH11", "JESDTX_8 dma request"),
	GCR_DESC(3, 18, 20, 0x2, "VSP10_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(3, 18, 20, 0x3, "VSP10_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(3, 18, 20, 0x4, "VSP10_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(3, 18, 20, 0x5, "VSP10_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(3, 15, 17, 0x1, "VSP9_CH11", "JESDTX_8 dma request"),
	GCR_DESC(3, 15, 17, 0x2, "VSP9_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(3, 15, 17, 0x3, "VSP9_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(3, 15, 17, 0x4, "VSP9_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(3, 15, 17, 0x5, "VSP9_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(3, 12, 14, 0x1, "VSP8_CH11", "JESDTX_8 dma request"),
	GCR_DESC(3, 12, 14, 0x2, "VSP8_CH11", "cpri_rx1 dma request 19"),
	GCR_DESC(3, 12, 14, 0x3, "VSP8_CH11", "cpri_rx1 dma request 22"),
	GCR_DESC(3, 12, 14, 0x4, "VSP8_CH11", "cpri_rx2 dma request 19"),
	GCR_DESC(3, 12, 14, 0x5, "VSP8_CH11", "cpri_rx2 dma request 22"),
	GCR_DESC(3, 9, 11, 0x1, "VSP11_CH10", "JESDTX_7 dma request"),
	GCR_DESC(3, 9, 11, 0x2, "VSP11_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(3, 9, 11, 0x3, "VSP11_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(3, 9, 11, 0x4, "VSP11_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(3, 9, 11, 0x5, "VSP11_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(3, 6, 8, 0x1, "VSP10_CH10", "JESDTX_7 dma request"),
	GCR_DESC(3, 6, 8, 0x2, "VSP10_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(3, 6, 8, 0x3, "VSP10_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(3, 6, 8, 0x4, "VSP10_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(3, 6, 8, 0x5, "VSP10_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(3, 3, 5, 0x1, "VSP9_CH10", "JESDTX_7 dma request"),
	GCR_DESC(3, 3, 5, 0x2, "VSP9_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(3, 3, 5, 0x3, "VSP9_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(3, 3, 5, 0x4, "VSP9_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(3, 3, 5, 0x5, "VSP9_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(3, 0, 2, 0x1, "VSP8_CH10", "JESDTX_7 dma request"),
	GCR_DESC(3, 0, 2, 0x2, "VSP8_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(3, 0, 2, 0x3, "VSP8_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(3, 0, 2, 0x4, "VSP8_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(3, 0, 2, 0x5, "VSP8_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(5, 29, 30, 0x1, "JESDRX_12", "JESD204RX_12 dma pointer reset request = VSP5 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 29, 30, 0x2, "JESDRX_12", "JESD204RX_12 dma pointer reset request = VSP6 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 29, 30, 0x3, "JESDRX_12", "JESD204RX_12 dma pointer reset request = VSP7 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 27, 28, 0x1, "JESDTX_10", "JESD204TX_10 dma pointer reset request = VSP5 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 27, 28, 0x2, "JESDTX_10", "JESD204TX_10 dma pointer reset request = VSP6 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 27, 28, 0x3, "JESDTX_10", "JESD204TX_10 dma pointer reset request = VSP7 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 25, 26, 0x1, "JESDRX_08", "JESD204RX_8 dma pointer reset request = VSP5 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 25, 26, 0x2, "JESDRX_08", "JESD204RX_8 dma pointer reset request = VSP6 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 25, 26, 0x3, "JESDRX_08", "JESD204RX_8 dma pointer reset request = VSP7 dma_ptr_rst_req[15]"),
	GCR_DESC(5, 23, 24, 0x1, "JESDRX_11", "JESD204RX_11 dma pointer reset request = VSP5 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 23, 24, 0x2, "JESDRX_11", "JESD204RX_11 dma pointer reset request = VSP6 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 23, 24, 0x3, "JESDRX_11", "JESD204RX_11 dma pointer reset request = VSP7 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 21, 22, 0x1, "JESDTX_09", "JESD204TX_9 dma pointer reset request = VSP5 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 21, 22, 0x2, "JESDTX_09", "JESD204TX_9 dma pointer reset request = VSP6 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 21, 22, 0x3, "JESDTX_09", "JESD204TX_9 dma pointer reset request = VSP7 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 19, 20, 0x1, "JESDRX_07", "JESD204RX7 dma pointer reset request = VSP5 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 19, 20, 0x2, "JESDRX_07", "JESD204RX7 dma pointer reset request = VSP6 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 19, 20, 0x3, "JESDRX_07", "JESD204RX7 dma pointer reset request = VSP7 dma_ptr_rst_req[14]"),
	GCR_DESC(5, 16, 18, 0x1, "VSP7_CH15", "JESDRX_8 dma request"),
	GCR_DESC(5, 16, 18, 0x2, "VSP7_CH15", "JESDTX10 dma request"),
	GCR_DESC(5, 16, 18, 0x3, "VSP7_CH15", "JESDRX12 dma request"),
	GCR_DESC(5, 16, 18, 0x4, "VSP7_CH15", "cpri_tx1 dma request 17"),
	GCR_DESC(5, 16, 18, 0x5, "VSP7_CH15", "cpri_tx1 dma request 24"),
	GCR_DESC(5, 16, 18, 0x6, "VSP7_CH15", "cpri_tx2 dma request 17"),
	GCR_DESC(5, 16, 18, 0x7, "VSP7_CH15", "cpri_tx2 dma request 24"),
	GCR_DESC(5, 13, 15, 0x1, "VSP6_CH15", "JESDRX_8 dma request"),
	GCR_DESC(5, 13, 15, 0x2, "VSP6_CH15", "JESDTX10 dma request"),
	GCR_DESC(5, 13, 15, 0x3, "VSP6_CH15", "JESDRX12 dma request"),
	GCR_DESC(5, 13, 15, 0x4, "VSP6_CH15", "cpri_tx1 dma request 17"),
	GCR_DESC(5, 13, 15, 0x5, "VSP6_CH15", "cpri_tx1 dma request 24"),
	GCR_DESC(5, 13, 15, 0x6, "VSP6_CH15", "cpri_tx2 dma request 17"),
	GCR_DESC(5, 13, 15, 0x7, "VSP6_CH15", "cpri_tx2 dma request 24"),
	GCR_DESC(5, 10, 12, 0x1, "VSP5_CH15", "JESDRX_8 dma request"),
	GCR_DESC(5, 10, 12, 0x2, "VSP5_CH15", "JESDTX10 dma request"),
	GCR_DESC(5, 10, 12, 0x3, "VSP5_CH15", "JESDRX12 dma request"),
	GCR_DESC(5, 10, 12, 0x4, "VSP5_CH15", "cpri_tx1 dma request 17"),
	GCR_DESC(5, 10, 12, 0x5, "VSP5_CH15", "cpri_tx1 dma request 24"),
	GCR_DESC(5, 10, 12, 0x6, "VSP5_CH15", "cpri_tx2 dma request 17"),
	GCR_DESC(5, 10, 12, 0x7, "VSP5_CH15", "cpri_tx2 dma request 24"),
	GCR_DESC(5, 6, 8, 0x1, "VSP7_CH14", "JESDRX_7 dma request"),
	GCR_DESC(5, 6, 8, 0x2, "VSP7_CH14", "JESDTX_9 dma request"),
	GCR_DESC(5, 6, 8, 0x3, "VSP7_CH14", "JESDRX_11 dma request"),
	GCR_DESC(5, 6, 8, 0x4, "VSP7_CH14", "cpri_tx1 dma request 18"),
	GCR_DESC(5, 6, 8, 0x5, "VSP7_CH14", "cpri_tx1 dma request 23"),
	GCR_DESC(5, 6, 8, 0x6, "VSP7_CH14", "cpri_tx2 dma request 18"),
	GCR_DESC(5, 6, 8, 0x7, "VSP7_CH14", "cpri_tx2 dma request 23"),
	GCR_DESC(5, 3, 5, 0x1, "VSP6_CH14", "JESDRX_7 dma request"),
	GCR_DESC(5, 3, 5, 0x2, "VSP6_CH14", "JESDTX_9 dma request"),
	GCR_DESC(5, 3, 5, 0x3, "VSP6_CH14", "JESDRX_11 dma request"),
	GCR_DESC(5, 3, 5, 0x4, "VSP6_CH14", "cpri_tx1 dma request 18"),
	GCR_DESC(5, 3, 5, 0x5, "VSP6_CH14", "cpri_tx1 dma request 23"),
	GCR_DESC(5, 3, 5, 0x6, "VSP6_CH14", "cpri_tx2 dma request 18"),
	GCR_DESC(5, 3, 5, 0x7, "VSP6_CH14", "cpri_tx2 dma request 23"),
	GCR_DESC(5, 0, 2, 0x1, "VSP5_CH14", "JESDRX_7 dma request"),
	GCR_DESC(5, 0, 2, 0x2, "VSP5_CH14", "JESDTX_9 dma request"),
	GCR_DESC(5, 0, 2, 0x3, "VSP5_CH14", "JESDRX_11 dma request"),
	GCR_DESC(5, 0, 2, 0x4, "VSP5_CH14", "cpri_tx1 dma request 18"),
	GCR_DESC(5, 0, 2, 0x5, "VSP5_CH14", "cpri_tx1 dma request 23"),
	GCR_DESC(5, 0, 2, 0x6, "VSP5_CH14", "cpri_tx2 dma request 18"),
	GCR_DESC(5, 0, 2, 0x7, "VSP5_CH14", "cpri_tx2 dma request 23"),
	GCR_DESC(8, 28, 31, 0xb, "VSP2_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0xa, "VSP2_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x9, "VSP2_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x8, "VSP2_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x7, "VSP2_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x6, "VSP2_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x5, "VSP2_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x4, "VSP2_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x3, "VSP2_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 28, 31, 0x1, "VSP2_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 24, 27, 0xb, "VSP2_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0xa, "VSP2_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x9, "VSP2_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x8, "VSP2_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x7, "VSP2_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x6, "VSP2_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x5, "VSP2_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x4, "VSP2_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x3, "VSP2_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 24, 27, 0x1, "VSP2_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 20, 23, 0xb, "VSP2_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0xa, "VSP2_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x9, "VSP2_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x8, "VSP2_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x7, "VSP2_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x6, "VSP2_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x5, "VSP2_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x4, "VSP2_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x3, "VSP2_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 20, 23, 0x1, "VSP2_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 16, 19, 0xb, "VSP2_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0xa, "VSP2_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x9, "VSP2_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x8, "VSP2_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x7, "VSP2_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x6, "VSP2_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x5, "VSP2_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x4, "VSP2_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x3, "VSP2_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 16, 19, 0x1, "VSP2_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 12, 15, 0xb, "VSP1_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0xa, "VSP1_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x9, "VSP1_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x8, "VSP1_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x7, "VSP1_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x6, "VSP1_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x5, "VSP1_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x4, "VSP1_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x3, "VSP1_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 12, 15, 0x2, "VSP1_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(8, 8, 11, 0xb, "VSP1_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0xa, "VSP1_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x9, "VSP1_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x8, "VSP1_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x7, "VSP1_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x6, "VSP1_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x5, "VSP1_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x4, "VSP1_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x3, "VSP1_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 8, 11, 0x2, "VSP1_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(8, 4, 7, 0xb, "VSP1_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0xa, "VSP1_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x9, "VSP1_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x8, "VSP1_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x7, "VSP1_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x6, "VSP1_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x5, "VSP1_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x4, "VSP1_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x3, "VSP1_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 4, 7, 0x2, "VSP1_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(8, 0, 3, 0xb, "VSP1_CH5", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0xa, "VSP1_CH5", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x9, "VSP1_CH5", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x8, "VSP1_CH5", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x7, "VSP1_CH5", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x6, "VSP1_CH5", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x5, "VSP1_CH5", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x4, "VSP1_CH5", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x3, "VSP1_CH5", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(8, 0, 3, 0x2, "VSP1_CH5", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 28, 31, 0xb, "VSP4_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0xa, "VSP4_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x9, "VSP4_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x8, "VSP4_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x7, "VSP4_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x6, "VSP4_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x5, "VSP4_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x3, "VSP4_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x2, "VSP4_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 28, 31, 0x1, "VSP4_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 24, 27, 0xb, "VSP4_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0xa, "VSP4_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x9, "VSP4_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x8, "VSP4_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x7, "VSP4_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x6, "VSP4_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x5, "VSP4_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x3, "VSP4_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x2, "VSP4_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 24, 27, 0x1, "VSP4_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 20, 23, 0xb, "VSP4_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0xa, "VSP4_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x9, "VSP4_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x8, "VSP4_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x7, "VSP4_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x6, "VSP4_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x5, "VSP4_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x3, "VSP4_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x2, "VSP4_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 20, 23, 0x1, "VSP4_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 16, 19, 0xb, "VSP4_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0xa, "VSP4_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x9, "VSP4_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x8, "VSP4_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x7, "VSP4_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x6, "VSP4_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x5, "VSP4_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x3, "VSP4_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x2, "VSP4_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 16, 19, 0x1, "VSP4_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 12, 15, 0xb, "VSP3_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0xa, "VSP3_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x9, "VSP3_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x8, "VSP3_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x7, "VSP3_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x6, "VSP3_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x5, "VSP3_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x4, "VSP3_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x2, "VSP3_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 12, 15, 0x1, "VSP3_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(9, 8, 11, 0xb, "VSP3_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0xa, "VSP3_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x9, "VSP3_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x8, "VSP3_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x7, "VSP3_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x6, "VSP3_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x5, "VSP3_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x4, "VSP3_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x2, "VSP3_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 8, 11, 0x1, "VSP3_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(9, 4, 7, 0xb, "VSP3_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0xa, "VSP3_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x9, "VSP3_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x8, "VSP3_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x7, "VSP3_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x6, "VSP3_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x5, "VSP3_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x4, "VSP3_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x2, "VSP3_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 4, 7, 0x1, "VSP3_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(9, 0, 3, 0xb, "VSP3_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0xa, "VSP3_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x9, "VSP3_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x8, "VSP3_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x7, "VSP3_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x6, "VSP3_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x5, "VSP3_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x4, "VSP3_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x2, "VSP3_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(9, 0, 3, 0x1, "VSP3_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 28, 31, 0xb, "VSP6_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0xa, "VSP6_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x9, "VSP6_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x8, "VSP6_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x7, "VSP6_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x5, "VSP6_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x4, "VSP6_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x3, "VSP6_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x2, "VSP6_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 28, 31, 0x1, "VSP6_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 24, 27, 0xb, "VSP6_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0xa, "VSP6_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0x9, "VSP6_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0x8, "VSP6_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0x7, "VSP6_CH6", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 24, 27, 0x5, "VSP6_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0x4, "VSP6_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0x3, "VSP6_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0x2, "VSP6_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 24, 27, 0x1, "VSP6_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 20, 23, 0xb, "VSP6_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0xa, "VSP6_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x9, "VSP6_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x8, "VSP6_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x7, "VSP6_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x5, "VSP6_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x4, "VSP6_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x3, "VSP6_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x2, "VSP6_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 20, 23, 0x1, "VSP6_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 16, 19, 0xb, "VSP6_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0xa, "VSP6_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x9, "VSP6_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x8, "VSP6_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x7, "VSP6_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x5, "VSP6_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x4, "VSP6_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x3, "VSP6_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x2, "VSP6_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 16, 19, 0x1, "VSP6_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 12, 15, 0xb, "VSP5_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0xa, "VSP5_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x9, "VSP5_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x8, "VSP5_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x7, "VSP5_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x6, "VSP5_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x4, "VSP5_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x3, "VSP5_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x2, "VSP5_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 12, 15, 0x1, "VSP5_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(10, 8, 11, 0xb, "VSP5_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0xa, "VSP5_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x9, "VSP5_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x8, "VSP5_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x7, "VSP5_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x6, "VSP5_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x4, "VSP5_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x3, "VSP5_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x2, "VSP5_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 8, 11, 0x1, "VSP5_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(10, 4, 7, 0xb, "VSP5_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0xa, "VSP5_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x9, "VSP5_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x8, "VSP5_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x7, "VSP5_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x6, "VSP5_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x4, "VSP5_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x3, "VSP5_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x2, "VSP5_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 4, 7, 0x1, "VSP5_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(10, 0, 3, 0xb, "VSP5_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0xa, "VSP5_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x9, "VSP5_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x8, "VSP5_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x7, "VSP5_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x6, "VSP5_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x4, "VSP5_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x3, "VSP5_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x2, "VSP5_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(10, 0, 3, 0x1, "VSP5_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 28, 31, 0xb, "VSP8_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0xa, "VSP8_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x8, "VSP8_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x7, "VSP8_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x6, "VSP8_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x5, "VSP8_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x4, "VSP8_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x3, "VSP8_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x2, "VSP8_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x1, "VSP8_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 24, 27, 0xb, "VSP8_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0xa, "VSP8_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x9, "VSP8_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x7, "VSP8_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x6, "VSP8_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x5, "VSP8_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x4, "VSP8_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x3, "VSP8_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x2, "VSP8_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x1, "VSP8_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 20, 23, 0xb, "VSP8_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0xa, "VSP8_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x9, "VSP8_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x7, "VSP8_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x6, "VSP8_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x5, "VSP8_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x4, "VSP8_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x3, "VSP8_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x2, "VSP8_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x1, "VSP8_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 16, 19, 0xb, "VSP8_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0xa, "VSP8_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x9, "VSP8_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x7, "VSP8_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x6, "VSP8_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x5, "VSP8_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x4, "VSP8_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x3, "VSP8_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x2, "VSP8_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x1, "VSP8_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 12, 15, 0xb, "VSP7_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0xa, "VSP7_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x9, "VSP7_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x8, "VSP7_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x6, "VSP7_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x5, "VSP7_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x4, "VSP7_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x3, "VSP7_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x2, "VSP7_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x1, "VSP7_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 8, 11, 0xb, "VSP7_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0xa, "VSP7_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x9, "VSP7_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x8, "VSP7_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x6, "VSP7_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x5, "VSP7_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x4, "VSP7_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x3, "VSP7_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x2, "VSP7_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x1, "VSP7_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 4, 7, 0xb, "VSP7_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0xa, "VSP7_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x9, "VSP7_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x8, "VSP7_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x6, "VSP7_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x5, "VSP7_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x4, "VSP7_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x3, "VSP7_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x2, "VSP7_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x1, "VSP7_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 0, 3, 0xb, "VSP7_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0xa, "VSP7_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x9, "VSP7_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x8, "VSP7_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x6, "VSP7_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x5, "VSP7_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x4, "VSP7_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x3, "VSP7_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x2, "VSP7_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x1, "VSP7_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 28, 31, 0xb, "VSP8_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0xa, "VSP8_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x8, "VSP8_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x7, "VSP8_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x6, "VSP8_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x5, "VSP8_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x4, "VSP8_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x3, "VSP8_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x2, "VSP8_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 28, 31, 0x1, "VSP8_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 24, 27, 0xb, "VSP8_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0xa, "VSP8_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x9, "VSP8_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x7, "VSP8_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x6, "VSP8_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x5, "VSP8_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x4, "VSP8_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x3, "VSP8_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x2, "VSP8_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 24, 27, 0x1, "VSP8_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 20, 23, 0xb, "VSP8_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0xa, "VSP8_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x9, "VSP8_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x7, "VSP8_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x6, "VSP8_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x5, "VSP8_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x4, "VSP8_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x3, "VSP8_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x2, "VSP8_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 20, 23, 0x1, "VSP8_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 16, 19, 0xb, "VSP8_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0xa, "VSP8_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x9, "VSP8_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x7, "VSP8_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x6, "VSP8_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x5, "VSP8_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x4, "VSP8_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x3, "VSP8_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x2, "VSP8_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 16, 19, 0x1, "VSP8_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 12, 15, 0xb, "VSP7_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0xa, "VSP7_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x9, "VSP7_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x8, "VSP7_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x6, "VSP7_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x5, "VSP7_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x4, "VSP7_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x3, "VSP7_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x2, "VSP7_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 12, 15, 0x1, "VSP7_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(11, 8, 11, 0xb, "VSP7_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0xa, "VSP7_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x9, "VSP7_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x8, "VSP7_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x6, "VSP7_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x5, "VSP7_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x4, "VSP7_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x3, "VSP7_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x2, "VSP7_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 8, 11, 0x1, "VSP7_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(11, 4, 7, 0xb, "VSP7_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0xa, "VSP7_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x9, "VSP7_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x8, "VSP7_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x6, "VSP7_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x5, "VSP7_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x4, "VSP7_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x3, "VSP7_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x2, "VSP7_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 4, 7, 0x1, "VSP7_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(11, 0, 3, 0xb, "VSP7_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0xa, "VSP7_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x9, "VSP7_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x8, "VSP7_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x6, "VSP7_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x5, "VSP7_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x4, "VSP7_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x3, "VSP7_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x2, "VSP7_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(11, 0, 3, 0x1, "VSP7_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 28, 31, 0xb, "VSP10_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x9, "VSP10_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x8, "VSP10_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x7, "VSP10_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x6, "VSP10_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x5, "VSP10_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x4, "VSP10_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x3, "VSP10_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x2, "VSP10_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 28, 31, 0x1, "VSP10_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 24, 27, 0xb, "VSP10_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x9, "VSP10_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x8, "VSP10_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x7, "VSP10_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x6, "VSP10_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x5, "VSP10_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x4, "VSP10_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x3, "VSP10_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x2, "VSP10_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 24, 27, 0x1, "VSP10_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 20, 23, 0xb, "VSP10_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x9, "VSP10_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x8, "VSP10_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x7, "VSP10_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x6, "VSP10_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x5, "VSP10_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x4, "VSP10_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x3, "VSP10_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x2, "VSP10_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 20, 23, 0x1, "VSP10_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 16, 19, 0xb, "VSP10_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x9, "VSP10_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x8, "VSP10_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x7, "VSP10_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x6, "VSP10_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x5, "VSP10_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x4, "VSP10_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x3, "VSP10_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x2, "VSP10_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 16, 19, 0x1, "VSP10_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 12, 15, 0xb, "VSP9_CH7", "VSP11 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0xa, "VSP9_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x8, "VSP9_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x7, "VSP9_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x6, "VSP9_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x5, "VSP9_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x4, "VSP9_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x3, "VSP9_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x2, "VSP9_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 12, 15, 0x1, "VSP9_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(12, 8, 11, 0xb, "VSP9_CH6", "VSP11 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0xa, "VSP9_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x8, "VSP9_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x7, "VSP9_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x6, "VSP9_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x5, "VSP9_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x4, "VSP9_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x3, "VSP9_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x2, "VSP9_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 8, 11, 0x1, "VSP9_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(12, 4, 7, 0xb, "VSP9_CH5", "VSP11 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0xa, "VSP9_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x8, "VSP9_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x7, "VSP9_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x6, "VSP9_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x5, "VSP9_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x4, "VSP9_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x3, "VSP9_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x2, "VSP9_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 4, 7, 0x1, "VSP9_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(12, 0, 3, 0xb, "VSP9_CH4", "VSP11 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0xa, "VSP9_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x8, "VSP9_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x7, "VSP9_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x6, "VSP9_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x5, "VSP9_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x4, "VSP9_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x3, "VSP9_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x2, "VSP9_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(12, 0, 3, 0x1, "VSP9_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 12, 15, 0xa, "VSP11_CH7", "VSP10 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x9, "VSP11_CH7", "VSP9 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x8, "VSP11_CH7", "VSP8 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x7, "VSP11_CH7", "VSP7 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x6, "VSP11_CH7", "VSP6 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x5, "VSP11_CH7", "VSP5 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x4, "VSP11_CH7", "VSP4 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x3, "VSP11_CH7", "VSP3 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x2, "VSP11_CH7", "VSP2 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 12, 15, 0x1, "VSP11_CH7", "VSP1 dma_ptr_rst_req_req[7]"),
	GCR_DESC(13, 8, 11, 0xa, "VSP11_CH6", "VSP10 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x9, "VSP11_CH6", "VSP9 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x8, "VSP11_CH6", "VSP8 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x7, "VSP11_CH6", "VSP7 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x6, "VSP11_CH6", "VSP6 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x5, "VSP11_CH6", "VSP5 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x4, "VSP11_CH6", "VSP4 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x3, "VSP11_CH6", "VSP3 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x2, "VSP11_CH6", "VSP2 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 8, 11, 0x1, "VSP11_CH6", "VSP1 dma_ptr_rst_req_req[6]"),
	GCR_DESC(13, 4, 7, 0xa, "VSP11_CH5", "VSP10 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x9, "VSP11_CH5", "VSP9 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x8, "VSP11_CH5", "VSP8 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x7, "VSP11_CH5", "VSP7 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x6, "VSP11_CH5", "VSP6 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x5, "VSP11_CH5", "VSP5 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x4, "VSP11_CH5", "VSP4 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x3, "VSP11_CH5", "VSP3 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x2, "VSP11_CH5", "VSP2 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 4, 7, 0x1, "VSP11_CH5", "VSP1 dma_ptr_rst_req_req[5]"),
	GCR_DESC(13, 0, 3, 0xa, "VSP11_CH4", "VSP10 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x9, "VSP11_CH4", "VSP9 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x8, "VSP11_CH4", "VSP8 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x7, "VSP11_CH4", "VSP7 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x6, "VSP11_CH4", "VSP6 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x5, "VSP11_CH4", "VSP5 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x4, "VSP11_CH4", "VSP4 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x3, "VSP11_CH4", "VSP3 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x2, "VSP11_CH4", "VSP2 dma_ptr_rst_req_req[4]"),
	GCR_DESC(13, 0, 3, 0x1, "VSP11_CH4", "VSP1 dma_ptr_rst_req_req[4]"),
	GCR_DESC(20, 23, 23, 0x0, "CPRI_RX1_23", "CPRI1 framer 1 rx dma request 23"),
	GCR_DESC(20, 23, 23, 0x1, "CPRI_RX1_23", "CPRI2 framer 2 rx dma request 0"),
	GCR_DESC(20, 22, 22, 0x0, "CPRI_RX1_22", "CPRI1 framer 1 rx dma request 22"),
	GCR_DESC(20, 22, 22, 0x1, "CPRI_RX1_22", "CPRI2 framer 2 rx dma request 1"),
	GCR_DESC(20, 21, 21, 0x0, "CPRI_RX1_21", "CPRI1 framer 1 rx dma request 21"),
	GCR_DESC(20, 21, 21, 0x1, "CPRI_RX1_21", "CPRI2 framer 2 rx dma request 2"),
	GCR_DESC(20, 20, 20, 0x0, "CPRI_RX1_20", "CPRI1 framer 1 rx dma request 20"),
	GCR_DESC(20, 20, 20, 0x1, "CPRI_RX1_20", "CPRI2 framer 2 rx dma request 3"),
	GCR_DESC(20, 19, 19, 0x0, "CPRI_RX1_19", "CPRI1 framer 1 rx dma request 19"),
	GCR_DESC(20, 19, 19, 0x1, "CPRI_RX1_19", "CPRI2 framer 2 rx dma request 4"),
	GCR_DESC(20, 18, 18, 0x0, "CPRI_RX1_18", "CPRI1 framer 1 rx dma request 18"),
	GCR_DESC(20, 18, 18, 0x1, "CPRI_RX1_18", "CPRI2 framer 2 rx dma request 5"),
	GCR_DESC(20, 17, 17, 0x0, "CPRI_RX1_17", "CPRI1 framer 1 rx dma request 17"),
	GCR_DESC(20, 17, 17, 0x1, "CPRI_RX1_17", "CPRI2 framer 2 rx dma request 6"),
	GCR_DESC(20, 16, 16, 0x0, "CPRI_RX1_16", "CPRI1 framer 1 rx dma request 16"),
	GCR_DESC(20, 16, 16, 0x1, "CPRI_RX1_16", "CPRI2 framer 2 rx dma request 7"),
	GCR_DESC(20, 15, 15, 0x0, "CPRI_RX1_15", "CPRI1 framer 1 rx dma request 15"),
	GCR_DESC(20, 15, 15, 0x1, "CPRI_RX1_15", "CPRI2 framer 2 rx dma request 8"),
	GCR_DESC(20, 14, 14, 0x0, "CPRI_RX1_14", "CPRI1 framer 1 rx dma request 14"),
	GCR_DESC(20, 14, 14, 0x1, "CPRI_RX1_14", "CPRI2 framer 2 rx dma request 9"),
	GCR_DESC(20, 13, 13, 0x0, "CPRI_RX1_13", "CPRI1 framer 1 rx dma request 13"),
	GCR_DESC(20, 13, 13, 0x1, "CPRI_RX1_13", "CPRI2 framer 2 rx dma request 10"),
	GCR_DESC(20, 12, 12, 0x0, "CPRI_RX1_12", "CPRI1 framer 1 rx dma request 12"),
	GCR_DESC(20, 12, 12, 0x1, "CPRI_RX1_12", "CPRI2 framer 2 rx dma request 11"),
	GCR_DESC(20, 11, 11, 0x0, "CPRI_RX1_11", "CPRI1 framer 1 rx dma request 11"),
	GCR_DESC(20, 11, 11, 0x1, "CPRI_RX1_11", "CPRI2 framer 2 rx dma request 12"),
	GCR_DESC(20, 10, 10, 0x0, "CPRI_RX1_10", "CPRI1 framer 1 rx dma request 10"),
	GCR_DESC(20, 10, 10, 0x1, "CPRI_RX1_10", "CPRI2 framer 2 rx dma request 13"),
	GCR_DESC(20, 9, 9, 0x0, "CPRI_RX1_09", "CPRI1 framer 1 rx dma request 9"),
	GCR_DESC(20, 9, 9, 0x1, "CPRI_RX1_09", "CPRI2 framer 2 rx dma request 14"),
	GCR_DESC(20, 8, 8, 0x0, "CPRI_RX1_08", "CPRI1 framer 1 rx dma request 8"),
	GCR_DESC(20, 8, 8, 0x1, "CPRI_RX1_08", "CPRI2 framer 2 rx dma request 15"),
	GCR_DESC(20, 7, 7, 0x0, "CPRI_RX1_07", "CPRI1 framer 1 rx dma request 7"),
	GCR_DESC(20, 7, 7, 0x1, "CPRI_RX1_07", "CPRI2 framer 2 rx dma request 16"),
	GCR_DESC(20, 6, 6, 0x0, "CPRI_RX1_06", "CPRI1 framer 1 rx dma request 6"),
	GCR_DESC(20, 6, 6, 0x1, "CPRI_RX1_06", "CPRI2 framer 2 rx dma request 17"),
	GCR_DESC(20, 5, 5, 0x0, "CPRI_RX1_05", "CPRI1 framer 1 rx dma request 5"),
	GCR_DESC(20, 5, 5, 0x1, "CPRI_RX1_05", "CPRI2 framer 2 rx dma request 18"),
	GCR_DESC(20, 4, 4, 0x0, "CPRI_RX1_04", "CPRI1 framer 1 rx dma request 4"),
	GCR_DESC(20, 4, 4, 0x1, "CPRI_RX1_04", "CPRI2 framer 2 rx dma request 19"),
	GCR_DESC(20, 3, 3, 0x0, "CPRI_RX1_03", "CPRI1 framer 1 rx dma request 3"),
	GCR_DESC(20, 3, 3, 0x1, "CPRI_RX1_03", "CPRI2 framer 2 rx dma request 20"),
	GCR_DESC(20, 2, 2, 0x0, "CPRI_RX1_02", "CPRI1 framer 1 rx dma request 2"),
	GCR_DESC(20, 2, 2, 0x1, "CPRI_RX1_02", "CPRI2 framer 2 rx dma request 21"),
	GCR_DESC(20, 1, 1, 0x0, "CPRI_RX1_01", "CPRI1 framer 1 rx dma request 1"),
	GCR_DESC(20, 1, 1, 0x1, "CPRI_RX1_01", "CPRI2 framer 2 rx dma request 22"),
	GCR_DESC(20, 0, 0, 0x0, "CPRI_RX1_00", "CPRI1 framer 1 rx dma request 0"),
	GCR_DESC(20, 0, 0, 0x1, "CPRI_RX1_00", "CPRI2 framer 2 rx dma request 23"),
	GCR_DESC(21, 23, 23, 0x0, "CPRI_RX2_23", "CPRI2 framer 1 rx dma request 23"),
	GCR_DESC(21, 23, 23, 0x1, "CPRI_RX2_23", "CPRI1 framer 2 rx dma request 0"),
	GCR_DESC(21, 22, 22, 0x0, "CPRI_RX2_22", "CPRI2 framer 1 rx dma request 22"),
	GCR_DESC(21, 22, 22, 0x1, "CPRI_RX2_22", "CPRI1 framer 2 rx dma request 1"),
	GCR_DESC(21, 21, 21, 0x0, "CPRI_RX2_21", "CPRI2 framer 1 rx dma request 21"),
	GCR_DESC(21, 21, 21, 0x1, "CPRI_RX2_21", "CPRI1 framer 2 rx dma request 2"),
	GCR_DESC(21, 20, 20, 0x0, "CPRI_RX2_20", "CPRI2 framer 1 rx dma request 20"),
	GCR_DESC(21, 20, 20, 0x1, "CPRI_RX2_20", "CPRI1 framer 2 rx dma request 3"),
	GCR_DESC(21, 19, 19, 0x0, "CPRI_RX2_19", "CPRI2 framer 1 rx dma request 19"),
	GCR_DESC(21, 19, 19, 0x1, "CPRI_RX2_19", "CPRI1 framer 2 rx dma request 4"),
	GCR_DESC(21, 18, 18, 0x0, "CPRI_RX2_18", "CPRI2 framer 1 rx dma request 18"),
	GCR_DESC(21, 18, 18, 0x1, "CPRI_RX2_18", "CPRI1 framer 2 rx dma request 5"),
	GCR_DESC(21, 17, 17, 0x0, "CPRI_RX2_17", "CPRI2 framer 1 rx dma request 17"),
	GCR_DESC(21, 17, 17, 0x1, "CPRI_RX2_17", "CPRI1 framer 2 rx dma request 6"),
	GCR_DESC(21, 16, 16, 0x0, "CPRI_RX2_16", "CPRI2 framer 1 rx dma request 16"),
	GCR_DESC(21, 16, 16, 0x1, "CPRI_RX2_16", "CPRI1 framer 2 rx dma request 7"),
	GCR_DESC(21, 15, 15, 0x0, "CPRI_RX2_15", "CPRI2 framer 1 rx dma request 15"),
	GCR_DESC(21, 15, 15, 0x1, "CPRI_RX2_15", "CPRI1 framer 2 rx dma request 8"),
	GCR_DESC(21, 14, 14, 0x0, "CPRI_RX2_14", "CPRI2 framer 1 rx dma request 14"),
	GCR_DESC(21, 14, 14, 0x1, "CPRI_RX2_14", "CPRI1 framer 2 rx dma request 9"),
	GCR_DESC(21, 13, 13, 0x0, "CPRI_RX2_13", "CPRI2 framer 1 rx dma request 13"),
	GCR_DESC(21, 13, 13, 0x1, "CPRI_RX2_13", "CPRI1 framer 2 rx dma request 10"),
	GCR_DESC(21, 12, 12, 0x0, "CPRI_RX2_12", "CPRI2 framer 1 rx dma request 12"),
	GCR_DESC(21, 12, 12, 0x1, "CPRI_RX2_12", "CPRI1 framer 2 rx dma request 11"),
	GCR_DESC(21, 11, 11, 0x0, "CPRI_RX2_11", "CPRI2 framer 1 rx dma request 11"),
	GCR_DESC(21, 11, 11, 0x1, "CPRI_RX2_11", "CPRI1 framer 2 rx dma request 12"),
	GCR_DESC(21, 10, 10, 0x0, "CPRI_RX2_10", "CPRI2 framer 1 rx dma request 10"),
	GCR_DESC(21, 10, 10, 0x1, "CPRI_RX2_10", "CPRI1 framer 2 rx dma request 13"),
	GCR_DESC(21, 9, 9, 0x0, "CPRI_RX2_09", "CPRI2 framer 1 rx dma request 9"),
	GCR_DESC(21, 9, 9, 0x1, "CPRI_RX2_09", "CPRI1 framer 2 rx dma request 14"),
	GCR_DESC(21, 8, 8, 0x0, "CPRI_RX2_08", "CPRI2 framer 1 rx dma request 8"),
	GCR_DESC(21, 8, 8, 0x1, "CPRI_RX2_08", "CPRI1 framer 2 rx dma request 15"),
	GCR_DESC(21, 7, 7, 0x0, "CPRI_RX2_07", "CPRI2 framer 1 rx dma request 7"),
	GCR_DESC(21, 7, 7, 0x1, "CPRI_RX2_07", "CPRI1 framer 2 rx dma request 16"),
	GCR_DESC(21, 6, 6, 0x0, "CPRI_RX2_06", "CPRI2 framer 1 rx dma request 6"),
	GCR_DESC(21, 6, 6, 0x1, "CPRI_RX2_06", "CPRI1 framer 2 rx dma request 17"),
	GCR_DESC(21, 5, 5, 0x0, "CPRI_RX2_05", "CPRI2 framer 1 rx dma request 5"),
	GCR_DESC(21, 5, 5, 0x1, "CPRI_RX2_05", "CPRI1 framer 2 rx dma request 18"),
	GCR_DESC(21, 4, 4, 0x0, "CPRI_RX2_04", "CPRI2 framer 1 rx dma request 4"),
	GCR_DESC(21, 4, 4, 0x1, "CPRI_RX2_04", "CPRI1 framer 2 rx dma request 19"),
	GCR_DESC(21, 3, 3, 0x0, "CPRI_RX2_03", "CPRI2 framer 1 rx dma request 3"),
	GCR_DESC(21, 3, 3, 0x1, "CPRI_RX2_03", "CPRI1 framer 2 rx dma request 20"),
	GCR_DESC(21, 2, 2, 0x0, "CPRI_RX2_02", "CPRI2 framer 1 rx dma request 2"),
	GCR_DESC(21, 2, 2, 0x1, "CPRI_RX2_02", "CPRI1 framer 2 rx dma request 21"),
	GCR_DESC(21, 1, 1, 0x0, "CPRI_RX2_01", "CPRI2 framer 1 rx dma request 1"),
	GCR_DESC(21, 1, 1, 0x1, "CPRI_RX2_01", "CPRI1 framer 2 rx dma request 22"),
	GCR_DESC(21, 0, 0, 0x0, "CPRI_RX2_00", "CPRI2 framer 1 rx dma request 0"),
	GCR_DESC(21, 0, 0, 0x1, "CPRI_RX2_00", "CPRI1 framer 2 rx dma request 23"),
	GCR_DESC(22, 21, 23, 0x1, "VSP4_CH9", "JESDTX_2 dma request"),
	GCR_DESC(22, 21, 23, 0x2, "VSP4_CH9", "cpri_rx1 dma request 18"),
	GCR_DESC(22, 21, 23, 0x3, "VSP4_CH9", "cpri_rx1 dma request 23"),
	GCR_DESC(22, 21, 23, 0x4, "VSP4_CH9", "cpri_rx2 dma request 18"),
	GCR_DESC(22, 21, 23, 0x5, "VSP4_CH9", "cpri_rx2 dma request 23"),
	GCR_DESC(22, 18, 20, 0x1, "VSP3_CH9", "JESDTX_2 dma request"),
	GCR_DESC(22, 18, 20, 0x2, "VSP3_CH9", "cpri_rx1 dma request 18"),
	GCR_DESC(22, 18, 20, 0x3, "VSP3_CH9", "cpri_rx1 dma request 23"),
	GCR_DESC(22, 18, 20, 0x4, "VSP3_CH9", "cpri_rx2 dma request 18"),
	GCR_DESC(22, 18, 20, 0x5, "VSP3_CH9", "cpri_rx2 dma request 23"),
	GCR_DESC(22, 15, 17, 0x1, "VSP2_CH9", "JESDTX_2 dma request"),
	GCR_DESC(22, 15, 17, 0x2, "VSP2_CH9", "cpri_rx1 dma request 18"),
	GCR_DESC(22, 15, 17, 0x3, "VSP2_CH9", "cpri_rx1 dma request 23"),
	GCR_DESC(22, 15, 17, 0x4, "VSP2_CH9", "cpri_rx2 dma request 18"),
	GCR_DESC(22, 15, 17, 0x5, "VSP2_CH9", "cpri_rx2 dma request 23"),
	GCR_DESC(22, 12, 14, 0x1, "VSP1_CH9", "JESDTX_2 dma request"),
	GCR_DESC(22, 12, 14, 0x2, "VSP1_CH9", "cpri_rx1 dma request 18"),
	GCR_DESC(22, 12, 14, 0x3, "VSP1_CH9", "cpri_rx1 dma request 23"),
	GCR_DESC(22, 12, 14, 0x4, "VSP1_CH9", "cpri_rx2 dma request 18"),
	GCR_DESC(22, 12, 14, 0x5, "VSP1_CH9", "cpri_rx2 dma request 23"),
	GCR_DESC(22, 9, 11, 0x1, "VSP4_CH8", "JESDTX_1 dma request"),
	GCR_DESC(22, 9, 11, 0x2, "VSP4_CH8", "cpri_rx1 dma request 17"),
	GCR_DESC(22, 9, 11, 0x3, "VSP4_CH8", "cpri_rx1 dma request 24"),
	GCR_DESC(22, 9, 11, 0x4, "VSP4_CH8", "cpri_rx2 dma request 17"),
	GCR_DESC(22, 9, 11, 0x5, "VSP4_CH8", "cpri_rx2 dma request 24"),
	GCR_DESC(22, 6, 8, 0x1, "VSP3_CH8", "JESDTX_1 dma request"),
	GCR_DESC(22, 6, 8, 0x2, "VSP3_CH8", "cpri_rx1 dma request 17"),
	GCR_DESC(22, 6, 8, 0x3, "VSP3_CH8", "cpri_rx1 dma request 24"),
	GCR_DESC(22, 6, 8, 0x4, "VSP3_CH8", "cpri_rx2 dma request 17"),
	GCR_DESC(22, 6, 8, 0x5, "VSP3_CH8", "cpri_rx2 dma request 24"),
	GCR_DESC(22, 3, 5, 0x1, "VSP2_CH8", "JESDTX_1 dma request"),
	GCR_DESC(22, 3, 5, 0x2, "VSP2_CH8", "cpri_rx1 dma request 17"),
	GCR_DESC(22, 3, 5, 0x3, "VSP2_CH8", "cpri_rx1 dma request 24"),
	GCR_DESC(22, 3, 5, 0x4, "VSP2_CH8", "cpri_rx2 dma request 17"),
	GCR_DESC(22, 3, 5, 0x5, "VSP2_CH8", "cpri_rx2 dma request 24"),
	GCR_DESC(22, 0, 2, 0x1, "VSP1_CH8", "JESDTX_1 dma request"),
	GCR_DESC(22, 0, 2, 0x2, "VSP1_CH8", "cpri_rx1 dma request 17"),
	GCR_DESC(22, 0, 2, 0x3, "VSP1_CH8", "cpri_rx1 dma request 24"),
	GCR_DESC(22, 0, 2, 0x4, "VSP1_CH8", "cpri_rx2 dma request 17"),
	GCR_DESC(22, 0, 2, 0x5, "VSP1_CH8", "cpri_rx2 dma request 24"),
	GCR_DESC(23, 21, 23, 0x1, "VSP4_CH11", "JESDTX_4 dma request"),
	GCR_DESC(23, 21, 23, 0x2, "VSP4_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(23, 21, 23, 0x3, "VSP4_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(23, 21, 23, 0x4, "VSP4_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(23, 21, 23, 0x5, "VSP4_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(23, 18, 20, 0x1, "VSP3_CH11", "JESDTX_4 dma request"),
	GCR_DESC(23, 18, 20, 0x2, "VSP3_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(23, 18, 20, 0x3, "VSP3_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(23, 18, 20, 0x4, "VSP3_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(23, 18, 20, 0x5, "VSP3_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(23, 15, 17, 0x1, "VSP2_CH11", "JESDTX_4 dma request"),
	GCR_DESC(23, 15, 17, 0x2, "VSP2_CH11", "cpri_rx1 dma request 20"),
	GCR_DESC(23, 15, 17, 0x3, "VSP2_CH11", "cpri_rx1 dma request 21"),
	GCR_DESC(23, 15, 17, 0x4, "VSP2_CH11", "cpri_rx2 dma request 20"),
	GCR_DESC(23, 15, 17, 0x5, "VSP2_CH11", "cpri_rx2 dma request 21"),
	GCR_DESC(23, 12, 14, 0x1, "VSP1_CH11", "JESDTX_3 dma request"),
	GCR_DESC(23, 12, 14, 0x2, "VSP1_CH11", "cpri_rx1 dma request 19"),
	GCR_DESC(23, 12, 14, 0x3, "VSP1_CH11", "cpri_rx1 dma request 22"),
	GCR_DESC(23, 12, 14, 0x4, "VSP1_CH11", "cpri_rx2 dma request 19"),
	GCR_DESC(23, 12, 14, 0x5, "VSP1_CH11", "cpri_rx2 dma request 22"),
	GCR_DESC(23, 9, 11, 0x1, "VSP4_CH10", "JESDTX_3 dma request"),
	GCR_DESC(23, 9, 11, 0x2, "VSP4_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(23, 9, 11, 0x3, "VSP4_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(23, 9, 11, 0x4, "VSP4_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(23, 9, 11, 0x5, "VSP4_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(23, 6, 8, 0x1, "VSP3_CH10", "JESDTX_3 dma request"),
	GCR_DESC(23, 6, 8, 0x2, "VSP3_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(23, 6, 8, 0x3, "VSP3_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(23, 6, 8, 0x4, "VSP3_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(23, 6, 8, 0x5, "VSP3_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(23, 3, 5, 0x1, "VSP2_CH10", "JESDTX_3 dma request"),
	GCR_DESC(23, 3, 5, 0x2, "VSP2_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(23, 3, 5, 0x3, "VSP2_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(23, 3, 5, 0x4, "VSP2_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(23, 3, 5, 0x5, "VSP2_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(23, 0, 2, 0x1, "VSP1_CH10", "JESDTX_3 dma request"),
	GCR_DESC(23, 0, 2, 0x2, "VSP1_CH10", "cpri_rx1 dma request 19"),
	GCR_DESC(23, 0, 2, 0x3, "VSP1_CH10", "cpri_rx1 dma request 22"),
	GCR_DESC(23, 0, 2, 0x4, "VSP1_CH10", "cpri_rx2 dma request 19"),
	GCR_DESC(23, 0, 2, 0x5, "VSP1_CH10", "cpri_rx2 dma request 22"),
	GCR_DESC(29, 23, 23, 0x0, "CPRI_TX1_23", "CPRI1 framer 1 tx dma request 23"),
	GCR_DESC(29, 23, 23, 0x1, "CPRI_TX1_23", "CPRI2 framer 2 tx dma request 0"),
	GCR_DESC(29, 22, 22, 0x0, "CPRI_TX1_22", "CPRI1 framer 1 tx dma request 22"),
	GCR_DESC(29, 22, 22, 0x1, "CPRI_TX1_22", "CPRI2 framer 2 tx dma request 1"),
	GCR_DESC(29, 21, 21, 0x0, "CPRI_TX1_21", "CPRI1 framer 1 tx dma request 21"),
	GCR_DESC(29, 21, 21, 0x1, "CPRI_TX1_21", "CPRI2 framer 2 tx dma request 2"),
	GCR_DESC(29, 20, 20, 0x0, "CPRI_TX1_20", "CPRI1 framer 1 tx dma request 20"),
	GCR_DESC(29, 20, 20, 0x1, "CPRI_TX1_20", "CPRI2 framer 2 tx dma request 3"),
	GCR_DESC(29, 19, 19, 0x0, "CPRI_TX1_19", "CPRI1 framer 1 tx dma request 19"),
	GCR_DESC(29, 19, 19, 0x1, "CPRI_TX1_19", "CPRI2 framer 2 tx dma request 4"),
	GCR_DESC(29, 18, 18, 0x0, "CPRI_TX1_18", "CPRI1 framer 1 tx dma request 18"),
	GCR_DESC(29, 18, 18, 0x1, "CPRI_TX1_18", "CPRI2 framer 2 tx dma request 5"),
	GCR_DESC(29, 17, 17, 0x0, "CPRI_TX1_17", "CPRI1 framer 1 tx dma request 17"),
	GCR_DESC(29, 17, 17, 0x1, "CPRI_TX1_17", "CPRI2 framer 2 tx dma request 6"),
	GCR_DESC(29, 16, 16, 0x0, "CPRI_TX1_16", "CPRI1 framer 1 tx dma request 16"),
	GCR_DESC(29, 16, 16, 0x1, "CPRI_TX1_16", "CPRI2 framer 2 tx dma request 7"),
	GCR_DESC(29, 15, 15, 0x0, "CPRI_TX1_15", "CPRI1 framer 1 tx dma request 15"),
	GCR_DESC(29, 15, 15, 0x1, "CPRI_TX1_15", "CPRI2 framer 2 tx dma request 8"),
	GCR_DESC(29, 14, 14, 0x0, "CPRI_TX1_14", "CPRI1 framer 1 tx dma request 14"),
	GCR_DESC(29, 14, 14, 0x1, "CPRI_TX1_14", "CPRI2 framer 2 tx dma request 9"),
	GCR_DESC(29, 13, 13, 0x0, "CPRI_TX1_13", "CPRI1 framer 1 tx dma request 13"),
	GCR_DESC(29, 13, 13, 0x1, "CPRI_TX1_13", "CPRI2 framer 2 tx dma request 10"),
	GCR_DESC(29, 12, 12, 0x0, "CPRI_TX1_12", "CPRI1 framer 1 tx dma request 12"),
	GCR_DESC(29, 12, 12, 0x1, "CPRI_TX1_12", "CPRI2 framer 2 tx dma request 11"),
	GCR_DESC(29, 11, 11, 0x0, "CPRI_TX1_11", "CPRI1 framer 1 tx dma request 11"),
	GCR_DESC(29, 11, 11, 0x1, "CPRI_TX1_11", "CPRI2 framer 2 tx dma request 12"),
	GCR_DESC(29, 10, 10, 0x0, "CPRI_TX1_10", "CPRI1 framer 1 tx dma request 10"),
	GCR_DESC(29, 10, 10, 0x1, "CPRI_TX1_10", "CPRI2 framer 2 tx dma request 13"),
	GCR_DESC(29, 9, 9, 0x0, "CPRI_TX1_09", "CPRI1 framer 1 tx dma request 9"),
	GCR_DESC(29, 9, 9, 0x1, "CPRI_TX1_09", "CPRI2 framer 2 tx dma request 14"),
	GCR_DESC(29, 8, 8, 0x0, "CPRI_TX1_08", "CPRI1 framer 1 tx dma request 8"),
	GCR_DESC(29, 8, 8, 0x1, "CPRI_TX1_08", "CPRI2 framer 2 tx dma request 15"),
	GCR_DESC(29, 7, 7, 0x0, "CPRI_TX1_07", "CPRI1 framer 1 tx dma request 7"),
	GCR_DESC(29, 7, 7, 0x1, "CPRI_TX1_07", "CPRI2 framer 2 tx dma request 16"),
	GCR_DESC(29, 6, 6, 0x0, "CPRI_TX1_06", "CPRI1 framer 1 tx dma request 6"),
	GCR_DESC(29, 6, 6, 0x1, "CPRI_TX1_06", "CPRI2 framer 2 tx dma request 17"),
	GCR_DESC(29, 5, 5, 0x0, "CPRI_TX1_05", "CPRI1 framer 1 tx dma request 5"),
	GCR_DESC(29, 5, 5, 0x1, "CPRI_TX1_05", "CPRI2 framer 2 tx dma request 18"),
	GCR_DESC(29, 4, 4, 0x0, "CPRI_TX1_04", "CPRI1 framer 1 tx dma request 4"),
	GCR_DESC(29, 4, 4, 0x1, "CPRI_TX1_04", "CPRI2 framer 2 tx dma request 19"),
	GCR_DESC(29, 3, 3, 0x0, "CPRI_TX1_03", "CPRI1 framer 1 tx dma request 3"),
	GCR_DESC(29, 3, 3, 0x1, "CPRI_TX1_03", "CPRI2 framer 2 tx dma request 20"),
	GCR_DESC(29, 2, 2, 0x0, "CPRI_TX1_02", "CPRI1 framer 1 tx dma request 2"),
	GCR_DESC(29, 2, 2, 0x1, "CPRI_TX1_02", "CPRI2 framer 2 tx dma request 21"),
	GCR_DESC(29, 1, 1, 0x0, "CPRI_TX1_01", "CPRI1 framer 1 tx dma request 1"),
	GCR_DESC(29, 1, 1, 0x1, "CPRI_TX1_01", "CPRI2 framer 2 tx dma request 22"),
	GCR_DESC(29, 0, 0, 0x0, "CPRI_TX1_00", "CPRI1 framer 1 tx dma request 0"),
	GCR_DESC(29, 0, 0, 0x1, "CPRI_TX1_00", "CPRI2 framer 2 tx dma request 23"),
	GCR_DESC(30, 23, 23, 0x0, "CPRI_TX2_23", "CPRI2 framer 1 tx dma request 23"),
	GCR_DESC(30, 23, 23, 0x1, "CPRI_TX2_23", "CPRI1 framer 2 tx dma request 0"),
	GCR_DESC(30, 22, 22, 0x0, "CPRI_TX2_22", "CPRI2 framer 1 tx dma request 22"),
	GCR_DESC(30, 22, 22, 0x1, "CPRI_TX2_22", "CPRI1 framer 2 tx dma request 1"),
	GCR_DESC(30, 21, 21, 0x0, "CPRI_TX2_21", "CPRI2 framer 1 tx dma request 21"),
	GCR_DESC(30, 21, 21, 0x1, "CPRI_TX2_21", "CPRI1 framer 2 tx dma request 2"),
	GCR_DESC(30, 20, 20, 0x0, "CPRI_TX2_20", "CPRI2 framer 1 tx dma request 20"),
	GCR_DESC(30, 20, 20, 0x1, "CPRI_TX2_20", "CPRI1 framer 2 tx dma request 3"),
	GCR_DESC(30, 19, 19, 0x0, "CPRI_TX2_19", "CPRI2 framer 1 tx dma request 19"),
	GCR_DESC(30, 19, 19, 0x1, "CPRI_TX2_19", "CPRI1 framer 2 tx dma request 4"),
	GCR_DESC(30, 18, 18, 0x0, "CPRI_TX2_18", "CPRI2 framer 1 tx dma request 18"),
	GCR_DESC(30, 18, 18, 0x1, "CPRI_TX2_18", "CPRI1 framer 2 tx dma request 5"),
	GCR_DESC(30, 17, 17, 0x0, "CPRI_TX2_17", "CPRI2 framer 1 tx dma request 17"),
	GCR_DESC(30, 17, 17, 0x1, "CPRI_TX2_17", "CPRI1 framer 2 tx dma request 6"),
	GCR_DESC(30, 16, 16, 0x0, "CPRI_TX2_16", "CPRI2 framer 1 tx dma request 16"),
	GCR_DESC(30, 16, 16, 0x1, "CPRI_TX2_16", "CPRI1 framer 2 tx dma request 7"),
	GCR_DESC(30, 15, 15, 0x0, "CPRI_TX2_15", "CPRI2 framer 1 tx dma request 15"),
	GCR_DESC(30, 15, 15, 0x1, "CPRI_TX2_15", "CPRI1 framer 2 tx dma request 8"),
	GCR_DESC(30, 14, 14, 0x0, "CPRI_TX2_14", "CPRI2 framer 1 tx dma request 14"),
	GCR_DESC(30, 14, 14, 0x1, "CPRI_TX2_14", "CPRI1 framer 2 tx dma request 9"),
	GCR_DESC(30, 13, 13, 0x0, "CPRI_TX2_13", "CPRI2 framer 1 tx dma request 13"),
	GCR_DESC(30, 13, 13, 0x1, "CPRI_TX2_13", "CPRI1 framer 2 tx dma request 10"),
	GCR_DESC(30, 12, 12, 0x0, "CPRI_TX2_12", "CPRI2 framer 1 tx dma request 12"),
	GCR_DESC(30, 12, 12, 0x1, "CPRI_TX2_12", "CPRI1 framer 2 tx dma request 11"),
	GCR_DESC(30, 11, 11, 0x0, "CPRI_TX2_11", "CPRI2 framer 1 tx dma request 11"),
	GCR_DESC(30, 11, 11, 0x1, "CPRI_TX2_11", "CPRI1 framer 2 tx dma request 12"),
	GCR_DESC(30, 10, 10, 0x0, "CPRI_TX2_10", "CPRI2 framer 1 tx dma request 10"),
	GCR_DESC(30, 10, 10, 0x1, "CPRI_TX2_10", "CPRI1 framer 2 tx dma request 13"),
	GCR_DESC(30, 9, 9, 0x0, "CPRI_TX2_09", "CPRI2 framer 1 tx dma request 9"),
	GCR_DESC(30, 9, 9, 0x1, "CPRI_TX2_09", "CPRI1 framer 2 tx dma request 14"),
	GCR_DESC(30, 8, 8, 0x0, "CPRI_TX2_08", "CPRI2 framer 1 tx dma request 8"),
	GCR_DESC(30, 8, 8, 0x1, "CPRI_TX2_08", "CPRI1 framer 2 tx dma request 15"),
	GCR_DESC(30, 7, 7, 0x0, "CPRI_TX2_07", "CPRI2 framer 1 tx dma request 7"),
	GCR_DESC(30, 7, 7, 0x1, "CPRI_TX2_07", "CPRI1 framer 2 tx dma request 16"),
	GCR_DESC(30, 6, 6, 0x0, "CPRI_TX2_06", "CPRI2 framer 1 tx dma request 6"),
	GCR_DESC(30, 6, 6, 0x1, "CPRI_TX2_06", "CPRI1 framer 2 tx dma request 17"),
	GCR_DESC(30, 5, 5, 0x0, "CPRI_TX2_05", "CPRI2 framer 1 tx dma request 5"),
	GCR_DESC(30, 5, 5, 0x1, "CPRI_TX2_05", "CPRI1 framer 2 tx dma request 18"),
	GCR_DESC(30, 4, 4, 0x0, "CPRI_TX2_04", "CPRI2 framer 1 tx dma request 4"),
	GCR_DESC(30, 4, 4, 0x1, "CPRI_TX2_04", "CPRI1 framer 2 tx dma request 19"),
	GCR_DESC(30, 3, 3, 0x0, "CPRI_TX2_03", "CPRI2 framer 1 tx dma request 3"),
	GCR_DESC(30, 3, 3, 0x1, "CPRI_TX2_03", "CPRI1 framer 2 tx dma request 20"),
	GCR_DESC(30, 2, 2, 0x0, "CPRI_TX2_02", "CPRI2 framer 1 tx dma request 2"),
	GCR_DESC(30, 2, 2, 0x1, "CPRI_TX2_02", "CPRI1 framer 2 tx dma request 21"),
	GCR_DESC(30, 1, 1, 0x0, "CPRI_TX2_01", "CPRI2 framer 1 tx dma request 1"),
	GCR_DESC(30, 1, 1, 0x1, "CPRI_TX2_01", "CPRI1 framer 2 tx dma request 22"),
	GCR_DESC(30, 0, 0, 0x0, "CPRI_TX2_00", "CPRI2 framer 1 tx dma request 0"),
	GCR_DESC(30, 0, 0, 0x1, "CPRI_TX2_00", "CPRI1 framer 2 tx dma request 23"),
	GCR_DESC(35, 26, 28, 0x1, "VSP7_CH18", "cpri_tx1 dma request 14"),
	GCR_DESC(35, 26, 28, 0x2, "VSP7_CH18", "cpri_tx1 dma request 22"),
	GCR_DESC(35, 26, 28, 0x3, "VSP7_CH18", "cpri_tx2 dma request 14"),
	GCR_DESC(35, 26, 28, 0x4, "VSP7_CH18", "cpri_tx2 dma request 22"),
	GCR_DESC(35, 23, 25, 0x1, "VSP6_CH18", "cpri_tx1 dma request 14"),
	GCR_DESC(35, 23, 25, 0x2, "VSP6_CH18", "cpri_tx1 dma request 22"),
	GCR_DESC(35, 23, 25, 0x3, "VSP6_CH18", "cpri_tx2 dma request 14"),
	GCR_DESC(35, 23, 25, 0x4, "VSP6_CH18", "cpri_tx2 dma request 22"),
	GCR_DESC(35, 20, 22, 0x1, "VSP5_CH18", "cpri_tx1 dma request 14"),
	GCR_DESC(35, 20, 22, 0x2, "VSP5_CH18", "cpri_tx1 dma request 22"),
	GCR_DESC(35, 20, 22, 0x3, "VSP5_CH18", "cpri_tx2 dma request 14"),
	GCR_DESC(35, 20, 22, 0x4, "VSP5_CH18", "cpri_tx2 dma request 22"),
	GCR_DESC(35, 16, 18, 0x1, "VSP7_CH17", "cpri_tx1 dma request 15"),
	GCR_DESC(35, 16, 18, 0x2, "VSP7_CH17", "cpri_tx1 dma request 23"),
	GCR_DESC(35, 16, 18, 0x3, "VSP7_CH17", "cpri_tx2 dma request 15"),
	GCR_DESC(35, 16, 18, 0x4, "VSP7_CH17", "cpri_tx2 dma request 23"),
	GCR_DESC(35, 13, 15, 0x1, "VSP6_CH17", "cpri_tx1 dma request 15"),
	GCR_DESC(35, 13, 15, 0x2, "VSP6_CH17", "cpri_tx1 dma request 23"),
	GCR_DESC(35, 13, 15, 0x3, "VSP6_CH17", "cpri_tx2 dma request 15"),
	GCR_DESC(35, 13, 15, 0x4, "VSP6_CH17", "cpri_tx2 dma request 23"),
	GCR_DESC(35, 10, 12, 0x1, "VSP5_CH17", "cpri_tx1 dma request 15"),
	GCR_DESC(35, 10, 12, 0x2, "VSP5_CH17", "cpri_tx1 dma request 23"),
	GCR_DESC(35, 10, 12, 0x3, "VSP5_CH17", "cpri_tx2 dma request 15"),
	GCR_DESC(35, 10, 12, 0x4, "VSP5_CH17", "cpri_tx2 dma request 23"),
	GCR_DESC(35, 6, 8, 0x1, "VSP7_CH16", "cpri_tx1 dma request 16"),
	GCR_DESC(35, 6, 8, 0x2, "VSP7_CH16", "cpri_tx1 dma request 24"),
	GCR_DESC(35, 6, 8, 0x3, "VSP7_CH16", "cpri_tx2 dma request 16"),
	GCR_DESC(35, 6, 8, 0x4, "VSP7_CH16", "cpri_tx2 dma request 24"),
	GCR_DESC(35, 3, 5, 0x1, "VSP6_CH16", "cpri_tx1 dma request 16"),
	GCR_DESC(35, 3, 5, 0x2, "VSP6_CH16", "cpri_tx1 dma request 24"),
	GCR_DESC(35, 3, 5, 0x3, "VSP6_CH16", "cpri_tx2 dma request 16"),
	GCR_DESC(35, 3, 5, 0x4, "VSP6_CH16", "cpri_tx2 dma request 24"),
	GCR_DESC(35, 0, 2, 0x1, "VSP5_CH16", "cpri_tx1 dma request 16"),
	GCR_DESC(35, 0, 2, 0x2, "VSP5_CH16", "cpri_tx1 dma request 24"),
	GCR_DESC(35, 0, 2, 0x3, "VSP5_CH16", "cpri_tx2 dma request 16"),
	GCR_DESC(35, 0, 2, 0x4, "VSP5_CH16", "cpri_tx2 dma request 24"),
	GCR_DESC(36, 26, 28, 0x1, "VSP7_CH21", "cpri_tx1 dma request 11"),
	GCR_DESC(36, 26, 28, 0x2, "VSP7_CH21", "cpri_tx1 dma request 19"),
	GCR_DESC(36, 26, 28, 0x3, "VSP7_CH21", "cpri_tx2 dma request 11"),
	GCR_DESC(36, 26, 28, 0x4, "VSP7_CH21", "cpri_tx2 dma request 19"),
	GCR_DESC(36, 23, 25, 0x1, "VSP6_CH21", "cpri_tx1 dma request 11"),
	GCR_DESC(36, 23, 25, 0x2, "VSP6_CH21", "cpri_tx1 dma request 19"),
	GCR_DESC(36, 23, 25, 0x3, "VSP6_CH21", "cpri_tx2 dma request 11"),
	GCR_DESC(36, 23, 25, 0x4, "VSP6_CH21", "cpri_tx2 dma request 19"),
	GCR_DESC(36, 20, 22, 0x1, "VSP5_CH21", "cpri_tx1 dma request 11"),
	GCR_DESC(36, 20, 22, 0x2, "VSP5_CH21", "cpri_tx1 dma request 19"),
	GCR_DESC(36, 20, 22, 0x3, "VSP5_CH21", "cpri_tx2 dma request 11"),
	GCR_DESC(36, 20, 22, 0x4, "VSP5_CH21", "cpri_tx2 dma request 19"),
	GCR_DESC(36, 16, 18, 0x1, "VSP7_CH20", "cpri_tx1 dma request 12"),
	GCR_DESC(36, 16, 18, 0x2, "VSP7_CH20", "cpri_tx1 dma request 20"),
	GCR_DESC(36, 16, 18, 0x3, "VSP7_CH20", "cpri_tx2 dma request 12"),
	GCR_DESC(36, 16, 18, 0x4, "VSP7_CH20", "cpri_tx2 dma request 20"),
	GCR_DESC(36, 13, 15, 0x1, "VSP6_CH20", "cpri_tx1 dma request 12"),
	GCR_DESC(36, 13, 15, 0x2, "VSP6_CH20", "cpri_tx1 dma request 20"),
	GCR_DESC(36, 13, 15, 0x3, "VSP6_CH20", "cpri_tx2 dma request 12"),
	GCR_DESC(36, 13, 15, 0x4, "VSP6_CH20", "cpri_tx2 dma request 20"),
	GCR_DESC(36, 10, 12, 0x1, "VSP5_CH20", "cpri_tx1 dma request 12"),
	GCR_DESC(36, 10, 12, 0x2, "VSP5_CH20", "cpri_tx1 dma request 20"),
	GCR_DESC(36, 10, 12, 0x3, "VSP5_CH20", "cpri_tx2 dma request 12"),
	GCR_DESC(36, 10, 12, 0x4, "VSP5_CH20", "cpri_tx2 dma request 20"),
	GCR_DESC(36, 6, 8, 0x1, "VSP7_CH19", "cpri_tx1 dma request 13"),
	GCR_DESC(36, 6, 8, 0x2, "VSP7_CH19", "cpri_tx1 dma request 21"),
	GCR_DESC(36, 6, 8, 0x3, "VSP7_CH19", "cpri_tx2 dma request 13"),
	GCR_DESC(36, 6, 8, 0x4, "VSP7_CH19", "cpri_tx2 dma request 21"),
	GCR_DESC(36, 3, 5, 0x1, "VSP6_CH19", "cpri_tx1 dma request 13"),
	GCR_DESC(36, 3, 5, 0x2, "VSP6_CH19", "cpri_tx1 dma request 21"),
	GCR_DESC(36, 3, 5, 0x3, "VSP6_CH19", "cpri_tx2 dma request 13"),
	GCR_DESC(36, 3, 5, 0x4, "VSP6_CH19", "cpri_tx2 dma request 21"),
	GCR_DESC(36, 0, 2, 0x1, "VSP5_CH19", "cpri_tx1 dma request 13"),
	GCR_DESC(36, 0, 2, 0x2, "VSP5_CH19", "cpri_tx1 dma request 21"),
	GCR_DESC(36, 0, 2, 0x3, "VSP5_CH19", "cpri_tx2 dma request 13"),
	GCR_DESC(36, 0, 2, 0x4, "VSP5_CH19", "cpri_tx2 dma request 21"),
	GCR_DESC(37, 26, 28, 0x1, "VSP7_CH24", "cpri_tx1 dma request 8"),
	GCR_DESC(37, 26, 28, 0x2, "VSP7_CH24", "cpri_tx1 dma request 24"),
	GCR_DESC(37, 26, 28, 0x3, "VSP7_CH24", "cpri_tx2 dma request 8"),
	GCR_DESC(37, 26, 28, 0x4, "VSP7_CH24", "cpri_tx2 dma request 24"),
	GCR_DESC(37, 23, 25, 0x1, "VSP6_CH24", "cpri_tx1 dma request 8"),
	GCR_DESC(37, 23, 25, 0x2, "VSP6_CH24", "cpri_tx1 dma request 24"),
	GCR_DESC(37, 23, 25, 0x3, "VSP6_CH24", "cpri_tx2 dma request 8"),
	GCR_DESC(37, 23, 25, 0x4, "VSP6_CH24", "cpri_tx2 dma request 24"),
	GCR_DESC(37, 20, 22, 0x1, "VSP5_CH24", "cpri_tx1 dma request 8"),
	GCR_DESC(37, 20, 22, 0x2, "VSP5_CH24", "cpri_tx1 dma request 24"),
	GCR_DESC(37, 20, 22, 0x3, "VSP5_CH24", "cpri_tx2 dma request 8"),
	GCR_DESC(37, 20, 22, 0x4, "VSP5_CH24", "cpri_tx2 dma request 24"),
	GCR_DESC(37, 16, 18, 0x1, "VSP7_CH23", "cpri_tx1 dma request 9"),
	GCR_DESC(37, 16, 18, 0x2, "VSP7_CH23", "cpri_tx1 dma request 17"),
	GCR_DESC(37, 16, 18, 0x3, "VSP7_CH23", "cpri_tx2 dma request 9"),
	GCR_DESC(37, 16, 18, 0x4, "VSP7_CH23", "cpri_tx2 dma request 17"),
	GCR_DESC(37, 13, 15, 0x1, "VSP6_CH23", "cpri_tx1 dma request 9"),
	GCR_DESC(37, 13, 15, 0x2, "VSP6_CH23", "cpri_tx1 dma request 17"),
	GCR_DESC(37, 13, 15, 0x3, "VSP6_CH23", "cpri_tx2 dma request 9"),
	GCR_DESC(37, 13, 15, 0x4, "VSP6_CH23", "cpri_tx2 dma request 17"),
	GCR_DESC(37, 10, 12, 0x1, "VSP5_CH23", "cpri_tx1 dma request 9"),
	GCR_DESC(37, 10, 12, 0x2, "VSP5_CH23", "cpri_tx1 dma request 17"),
	GCR_DESC(37, 10, 12, 0x3, "VSP5_CH23", "cpri_tx2 dma request 9"),
	GCR_DESC(37, 10, 12, 0x4, "VSP5_CH23", "cpri_tx2 dma request 17"),
	GCR_DESC(37, 6, 8, 0x1, "VSP7_CH22", "cpri_tx1 dma request 10"),
	GCR_DESC(37, 6, 8, 0x2, "VSP7_CH22", "cpri_tx1 dma request 18"),
	GCR_DESC(37, 6, 8, 0x3, "VSP7_CH22", "cpri_tx2 dma request 10"),
	GCR_DESC(37, 6, 8, 0x4, "VSP7_CH22", "cpri_tx2 dma request 18"),
	GCR_DESC(37, 3, 5, 0x1, "VSP6_CH22", "cpri_tx1 dma request 10"),
	GCR_DESC(37, 3, 5, 0x2, "VSP6_CH22", "cpri_tx1 dma request 18"),
	GCR_DESC(37, 3, 5, 0x3, "VSP6_CH22", "cpri_tx2 dma request 10"),
	GCR_DESC(37, 3, 5, 0x4, "VSP6_CH22", "cpri_tx2 dma request 18"),
	GCR_DESC(37, 0, 2, 0x1, "VSP5_CH22", "cpri_tx1 dma request 10"),
	GCR_DESC(37, 0, 2, 0x2, "VSP5_CH22", "cpri_tx1 dma request 18"),
	GCR_DESC(37, 0, 2, 0x3, "VSP5_CH22", "cpri_tx2 dma request 10"),
	GCR_DESC(37, 0, 2, 0x4, "VSP5_CH22", "cpri_tx2 dma request 18"),
	GCR_DESC(38, 26, 28, 0x1, "VSP7_CH27", "cpri_tx1 dma request 5"),
	GCR_DESC(38, 26, 28, 0x2, "VSP7_CH27", "cpri_tx1 dma request 21"),
	GCR_DESC(38, 26, 28, 0x3, "VSP7_CH27", "cpri_tx2 dma request 5"),
	GCR_DESC(38, 26, 28, 0x4, "VSP7_CH27", "cpri_tx2 dma request 21"),
	GCR_DESC(38, 23, 25, 0x1, "VSP6_CH27", "cpri_tx1 dma request 5"),
	GCR_DESC(38, 23, 25, 0x2, "VSP6_CH27", "cpri_tx1 dma request 21"),
	GCR_DESC(38, 23, 25, 0x3, "VSP6_CH27", "cpri_tx2 dma request 5"),
	GCR_DESC(38, 23, 25, 0x4, "VSP6_CH27", "cpri_tx2 dma request 21"),
	GCR_DESC(38, 20, 22, 0x1, "VSP5_CH27", "cpri_tx1 dma request 5"),
	GCR_DESC(38, 20, 22, 0x2, "VSP5_CH27", "cpri_tx1 dma request 21"),
	GCR_DESC(38, 20, 22, 0x3, "VSP5_CH27", "cpri_tx2 dma request 5"),
	GCR_DESC(38, 20, 22, 0x4, "VSP5_CH27", "cpri_tx2 dma request 21"),
	GCR_DESC(38, 16, 18, 0x1, "VSP7_CH26", "cpri_tx1 dma request 6"),
	GCR_DESC(38, 16, 18, 0x2, "VSP7_CH26", "cpri_tx1 dma request 22"),
	GCR_DESC(38, 16, 18, 0x3, "VSP7_CH26", "cpri_tx2 dma request 6"),
	GCR_DESC(38, 16, 18, 0x4, "VSP7_CH26", "cpri_tx2 dma request 22"),
	GCR_DESC(38, 13, 15, 0x1, "VSP6_CH26", "cpri_tx1 dma request 6"),
	GCR_DESC(38, 13, 15, 0x2, "VSP6_CH26", "cpri_tx1 dma request 22"),
	GCR_DESC(38, 13, 15, 0x3, "VSP6_CH26", "cpri_tx2 dma request 6"),
	GCR_DESC(38, 13, 15, 0x4, "VSP6_CH26", "cpri_tx2 dma request 22"),
	GCR_DESC(38, 10, 12, 0x1, "VSP5_CH26", "cpri_tx1 dma request 6"),
	GCR_DESC(38, 10, 12, 0x2, "VSP5_CH26", "cpri_tx1 dma request 22"),
	GCR_DESC(38, 10, 12, 0x3, "VSP5_CH26", "cpri_tx2 dma request 6"),
	GCR_DESC(38, 10, 12, 0x4, "VSP5_CH26", "cpri_tx2 dma request 22"),
	GCR_DESC(38, 6, 8, 0x1, "VSP7_CH25", "cpri_tx1 dma request 7"),
	GCR_DESC(38, 6, 8, 0x2, "VSP7_CH25", "cpri_tx1 dma request 23"),
	GCR_DESC(38, 6, 8, 0x3, "VSP7_CH25", "cpri_tx2 dma request 7"),
	GCR_DESC(38, 6, 8, 0x4, "VSP7_CH25", "cpri_tx2 dma request 23"),
	GCR_DESC(38, 3, 5, 0x1, "VSP6_CH25", "cpri_tx1 dma request 7"),
	GCR_DESC(38, 3, 5, 0x2, "VSP6_CH25", "cpri_tx1 dma request 23"),
	GCR_DESC(38, 3, 5, 0x3, "VSP6_CH25", "cpri_tx2 dma request 7"),
	GCR_DESC(38, 3, 5, 0x4, "VSP6_CH25", "cpri_tx2 dma request 23"),
	GCR_DESC(38, 0, 2, 0x1, "VSP5_CH25", "cpri_tx1 dma request 7"),
	GCR_DESC(38, 0, 2, 0x2, "VSP5_CH25", "cpri_tx1 dma request 23"),
	GCR_DESC(38, 0, 2, 0x3, "VSP5_CH25", "cpri_tx2 dma request 7"),
	GCR_DESC(38, 0, 2, 0x4, "VSP5_CH25", "cpri_tx2 dma request 23"),
	GCR_DESC(39, 26, 28, 0x1, "VSP7_CH30", "cpri_tx1 dma request 2"),
	GCR_DESC(39, 26, 28, 0x2, "VSP7_CH30", "cpri_tx1 dma request 18"),
	GCR_DESC(39, 26, 28, 0x3, "VSP7_CH30", "cpri_tx2 dma request 2"),
	GCR_DESC(39, 26, 28, 0x4, "VSP7_CH30", "cpri_tx2 dma request 18"),
	GCR_DESC(39, 23, 25, 0x1, "VSP6_CH30", "cpri_tx1 dma request 2"),
	GCR_DESC(39, 23, 25, 0x2, "VSP6_CH30", "cpri_tx1 dma request 18"),
	GCR_DESC(39, 23, 25, 0x3, "VSP6_CH30", "cpri_tx2 dma request 2"),
	GCR_DESC(39, 23, 25, 0x4, "VSP6_CH30", "cpri_tx2 dma request 18"),
	GCR_DESC(39, 20, 22, 0x1, "VSP5_CH30", "cpri_tx1 dma request 2"),
	GCR_DESC(39, 20, 22, 0x2, "VSP5_CH30", "cpri_tx1 dma request 18"),
	GCR_DESC(39, 20, 22, 0x3, "VSP5_CH30", "cpri_tx2 dma request 2"),
	GCR_DESC(39, 20, 22, 0x4, "VSP5_CH30", "cpri_tx2 dma request 18"),
	GCR_DESC(39, 16, 18, 0x1, "VSP7_CH29", "cpri_tx1 dma request 3"),
	GCR_DESC(39, 16, 18, 0x2, "VSP7_CH29", "cpri_tx1 dma request 19"),
	GCR_DESC(39, 16, 18, 0x3, "VSP7_CH29", "cpri_tx2 dma request 3"),
	GCR_DESC(39, 16, 18, 0x4, "VSP7_CH29", "cpri_tx2 dma request 19"),
	GCR_DESC(39, 13, 15, 0x1, "VSP6_CH29", "cpri_tx1 dma request 3"),
	GCR_DESC(39, 13, 15, 0x2, "VSP6_CH29", "cpri_tx1 dma request 19"),
	GCR_DESC(39, 13, 15, 0x3, "VSP6_CH29", "cpri_tx2 dma request 3"),
	GCR_DESC(39, 13, 15, 0x4, "VSP6_CH29", "cpri_tx2 dma request 19"),
	GCR_DESC(39, 10, 12, 0x1, "VSP5_CH29", "cpri_tx1 dma request 3"),
	GCR_DESC(39, 10, 12, 0x2, "VSP5_CH29", "cpri_tx1 dma request 19"),
	GCR_DESC(39, 10, 12, 0x3, "VSP5_CH29", "cpri_tx2 dma request 3"),
	GCR_DESC(39, 10, 12, 0x4, "VSP5_CH29", "cpri_tx2 dma request 19"),
	GCR_DESC(39, 6, 8, 0x1, "VSP7_CH28", "cpri_tx1 dma request 4"),
	GCR_DESC(39, 6, 8, 0x2, "VSP7_CH28", "cpri_tx1 dma request 20"),
	GCR_DESC(39, 6, 8, 0x3, "VSP7_CH28", "cpri_tx2 dma request 4"),
	GCR_DESC(39, 6, 8, 0x4, "VSP7_CH28", "cpri_tx2 dma request 20"),
	GCR_DESC(39, 3, 5, 0x1, "VSP6_CH28", "cpri_tx1 dma request 4"),
	GCR_DESC(39, 3, 5, 0x2, "VSP6_CH28", "cpri_tx1 dma request 20"),
	GCR_DESC(39, 3, 5, 0x3, "VSP6_CH28", "cpri_tx2 dma request 4"),
	GCR_DESC(39, 3, 5, 0x4, "VSP6_CH28", "cpri_tx2 dma request 20"),
	GCR_DESC(39, 0, 2, 0x1, "VSP5_CH28", "cpri_tx1 dma request 4"),
	GCR_DESC(39, 0, 2, 0x2, "VSP5_CH28", "cpri_tx1 dma request 20"),
	GCR_DESC(39, 0, 2, 0x3, "VSP5_CH28", "cpri_tx2 dma request 4"),
	GCR_DESC(39, 0, 2, 0x4, "VSP5_CH28", "cpri_tx2 dma request 20"),
	GCR_DESC(40, 6, 8, 0x1, "VSP7_CH31", "cpri_tx1 dma request 1"),
	GCR_DESC(40, 6, 8, 0x2, "VSP7_CH31", "cpri_tx1 dma request 17"),
	GCR_DESC(40, 6, 8, 0x3, "VSP7_CH31", "cpri_tx2 dma request 1"),
	GCR_DESC(40, 6, 8, 0x4, "VSP7_CH31", "cpri_tx2 dma request 17"),
	GCR_DESC(40, 3, 5, 0x1, "VSP6_CH31", "cpri_tx1 dma request 1"),
	GCR_DESC(40, 3, 5, 0x2, "VSP6_CH31", "cpri_tx1 dma request 17"),
	GCR_DESC(40, 3, 5, 0x3, "VSP6_CH31", "cpri_tx2 dma request 1"),
	GCR_DESC(40, 3, 5, 0x4, "VSP6_CH31", "cpri_tx2 dma request 17"),
	GCR_DESC(40, 0, 2, 0x1, "VSP5_CH31", "cpri_tx1 dma request 1"),
	GCR_DESC(40, 0, 2, 0x2, "VSP5_CH31", "cpri_tx1 dma request 17"),
	GCR_DESC(40, 0, 2, 0x3, "VSP5_CH31", "cpri_tx2 dma request 1"),
	GCR_DESC(40, 0, 2, 0x4, "VSP5_CH31", "cpri_tx2 dma request 17"),
	GCR_DESC(41, 26, 28, 0x1, "VSP7_CH10", "JESDRX_3 dma request"),
	GCR_DESC(41, 26, 28, 0x2, "VSP7_CH10", "JESDRX_7 dma request"),
	GCR_DESC(41, 26, 28, 0x3, "VSP7_CH10", "cpri_tx1 dma request 19"),
	GCR_DESC(41, 26, 28, 0x4, "VSP7_CH10", "cpri_tx1 dma request 22"),
	GCR_DESC(41, 26, 28, 0x5, "VSP7_CH10", "cpri_tx2 dma request 19"),
	GCR_DESC(41, 26, 28, 0x6, "VSP7_CH10", "cpri_tx2 dma request 22"),
	GCR_DESC(41, 23, 25, 0x1, "VSP6_CH10", "JESDRX_3 dma request"),
	GCR_DESC(41, 23, 25, 0x2, "VSP6_CH10", "JESDRX_7 dma request"),
	GCR_DESC(41, 23, 25, 0x3, "VSP6_CH10", "cpri_tx1 dma request 19"),
	GCR_DESC(41, 23, 25, 0x4, "VSP6_CH10", "cpri_tx1 dma request 22"),
	GCR_DESC(41, 23, 25, 0x5, "VSP6_CH10", "cpri_tx2 dma request 19"),
	GCR_DESC(41, 23, 25, 0x6, "VSP6_CH10", "cpri_tx2 dma request 22"),
	GCR_DESC(41, 20, 22, 0x1, "VSP5_CH10", "JESDRX_3 dma request"),
	GCR_DESC(41, 20, 22, 0x2, "VSP5_CH10", "JESDRX_7 dma request"),
	GCR_DESC(41, 20, 22, 0x3, "VSP5_CH10", "cpri_tx1 dma request 19"),
	GCR_DESC(41, 20, 22, 0x4, "VSP5_CH10", "cpri_tx1 dma request 22"),
	GCR_DESC(41, 20, 22, 0x5, "VSP5_CH10", "cpri_tx2 dma request 19"),
	GCR_DESC(41, 20, 22, 0x6, "VSP5_CH10", "cpri_tx2 dma request 22"),
	GCR_DESC(41, 16, 18, 0x1, "VSP7_CH9", "JESDRX_2 dma request"),
	GCR_DESC(41, 16, 18, 0x2, "VSP7_CH9", "JESDRX_6 dma request"),
	GCR_DESC(41, 16, 18, 0x3, "VSP7_CH9", "cpri_tx1 dma request 18"),
	GCR_DESC(41, 16, 18, 0x4, "VSP7_CH9", "cpri_tx1 dma request 23"),
	GCR_DESC(41, 16, 18, 0x5, "VSP7_CH9", "cpri_tx2 dma request 18"),
	GCR_DESC(41, 16, 18, 0x6, "VSP7_CH9", "cpri_tx2 dma request 23"),
	GCR_DESC(41, 13, 15, 0x1, "VSP6_CH9", "JESDRX_2 dma request"),
	GCR_DESC(41, 13, 15, 0x2, "VSP6_CH9", "JESDRX_6 dma request"),
	GCR_DESC(41, 13, 15, 0x3, "VSP6_CH9", "cpri_tx1 dma request 18"),
	GCR_DESC(41, 13, 15, 0x4, "VSP6_CH9", "cpri_tx1 dma request 23"),
	GCR_DESC(41, 13, 15, 0x5, "VSP6_CH9", "cpri_tx2 dma request 18"),
	GCR_DESC(41, 13, 15, 0x6, "VSP6_CH9", "cpri_tx2 dma request 23"),
	GCR_DESC(41, 10, 12, 0x1, "VSP5_CH9", "JESDRX_2 dma request"),
	GCR_DESC(41, 10, 12, 0x2, "VSP5_CH9", "JESDRX_6 dma request"),
	GCR_DESC(41, 10, 12, 0x3, "VSP5_CH9", "cpri_tx1 dma request 18"),
	GCR_DESC(41, 10, 12, 0x4, "VSP5_CH9", "cpri_tx1 dma request 23"),
	GCR_DESC(41, 10, 12, 0x5, "VSP5_CH9", "cpri_tx2 dma request 18"),
	GCR_DESC(41, 10, 12, 0x6, "VSP5_CH9", "cpri_tx2 dma request 23"),
	GCR_DESC(41, 6, 8, 0x1, "VSP7_CH8", "JESDRX_1 dma request"),
	GCR_DESC(41, 6, 8, 0x2, "VSP7_CH8", "JESDRX_5 dma request"),
	GCR_DESC(41, 6, 8, 0x3, "VSP7_CH8", "cpri_tx1 dma request 17"),
	GCR_DESC(41, 6, 8, 0x4, "VSP7_CH8", "cpri_tx1 dma request 24"),
	GCR_DESC(41, 6, 8, 0x5, "VSP7_CH8", "cpri_tx2 dma request 17"),
	GCR_DESC(41, 6, 8, 0x6, "VSP7_CH8", "cpri_tx2 dma request 24"),
	GCR_DESC(41, 3, 5, 0x1, "VSP6_CH8", "JESDRX_1 dma request"),
	GCR_DESC(41, 3, 5, 0x2, "VSP6_CH8", "JESDRX_5 dma request"),
	GCR_DESC(41, 3, 5, 0x3, "VSP6_CH8", "cpri_tx1 dma request 17"),
	GCR_DESC(41, 3, 5, 0x4, "VSP6_CH8", "cpri_tx1 dma request 24"),
	GCR_DESC(41, 3, 5, 0x5, "VSP6_CH8", "cpri_tx2 dma request 17"),
	GCR_DESC(41, 3, 5, 0x6, "VSP6_CH8", "cpri_tx2 dma request 24"),
	GCR_DESC(41, 0, 2, 0x1, "VSP5_CH8", "JESDRX_1 dma request"),
	GCR_DESC(41, 0, 2, 0x2, "VSP5_CH8", "JESDRX_5 dma request"),
	GCR_DESC(41, 0, 2, 0x3, "VSP5_CH8", "cpri_tx1 dma request 17"),
	GCR_DESC(41, 0, 2, 0x4, "VSP5_CH8", "cpri_tx1 dma request 24"),
	GCR_DESC(41, 0, 2, 0x5, "VSP5_CH8", "cpri_tx2 dma request 17"),
	GCR_DESC(41, 0, 2, 0x6, "VSP5_CH8", "cpri_tx2 dma request 24"),
	GCR_DESC(42, 26, 28, 0x1, "VSP7_CH13", "JESDRX_6 dma request"),
	GCR_DESC(42, 26, 28, 0x2, "VSP7_CH13", "JESDRX_10 dma request"),
	GCR_DESC(42, 26, 28, 0x3, "VSP7_CH13", "cpri_tx1 dma request 19"),
	GCR_DESC(42, 26, 28, 0x4, "VSP7_CH13", "cpri_tx1 dma request 22"),
	GCR_DESC(42, 26, 28, 0x5, "VSP7_CH13", "cpri_tx2 dma request 19"),
	GCR_DESC(42, 26, 28, 0x6, "VSP7_CH13", "cpri_tx2 dma request 22"),
	GCR_DESC(42, 23, 25, 0x1, "VSP6_CH13", "JESDRX_6 dma request"),
	GCR_DESC(42, 23, 25, 0x2, "VSP6_CH13", "JESDRX_10 dma request"),
	GCR_DESC(42, 23, 25, 0x3, "VSP6_CH13", "cpri_tx1 dma request 19"),
	GCR_DESC(42, 23, 25, 0x4, "VSP6_CH13", "cpri_tx1 dma request 22"),
	GCR_DESC(42, 23, 25, 0x5, "VSP6_CH13", "cpri_tx2 dma request 19"),
	GCR_DESC(42, 23, 25, 0x6, "VSP6_CH13", "cpri_tx2 dma request 22"),
	GCR_DESC(42, 20, 22, 0x1, "VSP5_CH13", "JESDRX_6 dma request"),
	GCR_DESC(42, 20, 22, 0x2, "VSP5_CH13", "JESDRX_10 dma request"),
	GCR_DESC(42, 20, 22, 0x3, "VSP5_CH13", "cpri_tx1 dma request 19"),
	GCR_DESC(42, 20, 22, 0x4, "VSP5_CH13", "cpri_tx1 dma request 22"),
	GCR_DESC(42, 20, 22, 0x5, "VSP5_CH13", "cpri_tx2 dma request 19"),
	GCR_DESC(42, 20, 22, 0x6, "VSP5_CH13", "cpri_tx2 dma request 22"),
	GCR_DESC(42, 16, 18, 0x1, "VSP7_CH12", "JESDRX_5 dma request"),
	GCR_DESC(42, 16, 18, 0x2, "VSP7_CH12", "JESDRX_9 dma request"),
	GCR_DESC(42, 16, 18, 0x3, "VSP7_CH12", "cpri_tx1 dma request 20"),
	GCR_DESC(42, 16, 18, 0x4, "VSP7_CH12", "cpri_tx1 dma request 21"),
	GCR_DESC(42, 16, 18, 0x5, "VSP7_CH12", "cpri_tx2 dma request 20"),
	GCR_DESC(42, 16, 18, 0x6, "VSP7_CH12", "cpri_tx2 dma request 21"),
	GCR_DESC(42, 13, 15, 0x1, "VSP6_CH12", "JESDRX_5 dma request"),
	GCR_DESC(42, 13, 15, 0x2, "VSP6_CH12", "JESDRX_9 dma request"),
	GCR_DESC(42, 13, 15, 0x3, "VSP6_CH12", "cpri_tx1 dma request 20"),
	GCR_DESC(42, 13, 15, 0x4, "VSP6_CH12", "cpri_tx1 dma request 21"),
	GCR_DESC(42, 13, 15, 0x5, "VSP6_CH12", "cpri_tx2 dma request 20"),
	GCR_DESC(42, 13, 15, 0x6, "VSP6_CH12", "cpri_tx2 dma request 21"),
	GCR_DESC(42, 10, 12, 0x1, "VSP5_CH12", "JESDRX_5 dma request"),
	GCR_DESC(42, 10, 12, 0x2, "VSP5_CH12", "JESDRX_9 dma request"),
	GCR_DESC(42, 10, 12, 0x3, "VSP5_CH12", "cpri_tx1 dma request 20"),
	GCR_DESC(42, 10, 12, 0x4, "VSP5_CH12", "cpri_tx1 dma request 21"),
	GCR_DESC(42, 10, 12, 0x5, "VSP5_CH12", "cpri_tx2 dma request 20"),
	GCR_DESC(42, 10, 12, 0x6, "VSP5_CH12", "cpri_tx2 dma request 21"),
	GCR_DESC(42, 6, 8, 0x1, "VSP7_CH11", "JESDRX_4 dma request"),
	GCR_DESC(42, 6, 8, 0x2, "VSP7_CH11", "JESDRX_8 dma request"),
	GCR_DESC(42, 6, 8, 0x3, "VSP7_CH11", "cpri_tx1 dma request 20"),
	GCR_DESC(42, 6, 8, 0x4, "VSP7_CH11", "cpri_tx1 dma request 21"),
	GCR_DESC(42, 6, 8, 0x5, "VSP7_CH11", "cpri_tx2 dma request 20"),
	GCR_DESC(42, 6, 8, 0x6, "VSP7_CH11", "cpri_tx2 dma request 21"),
	GCR_DESC(42, 3, 5, 0x1, "VSP6_CH11", "JESDRX_4 dma request"),
	GCR_DESC(42, 3, 5, 0x2, "VSP6_CH11", "JESDRX_8 dma request"),
	GCR_DESC(42, 3, 5, 0x3, "VSP6_CH11", "cpri_tx1 dma request 20"),
	GCR_DESC(42, 3, 5, 0x4, "VSP6_CH11", "cpri_tx1 dma request 21"),
	GCR_DESC(42, 3, 5, 0x5, "VSP6_CH11", "cpri_tx2 dma request 20"),
	GCR_DESC(42, 3, 5, 0x6, "VSP6_CH11", "cpri_tx2 dma request 21"),
	GCR_DESC(42, 0, 2, 0x1, "VSP5_CH11", "JESDRX_4 dma request"),
	GCR_DESC(42, 0, 2, 0x2, "VSP5_CH11", "JESDRX_8 dma request"),
	GCR_DESC(42, 0, 2, 0x3, "VSP5_CH11", "cpri_tx1 dma request 20"),
	GCR_DESC(42, 0, 2, 0x4, "VSP5_CH11", "cpri_tx1 dma request 21"),
	GCR_DESC(42, 0, 2, 0x5, "VSP5_CH11", "cpri_tx2 dma request 20"),
	GCR_DESC(42, 0, 2, 0x6, "VSP5_CH11", "cpri_tx2 dma request 21"),
	GCR_DESC(43, 9, 11, 0x1, "VSP_JESDTX4", "Request is connected to VSP1 dma_ptr_rst_req[11]"),
	GCR_DESC(43, 9, 11, 0x2, "VSP_JESDTX4", "Request is connected to VSP2 dma_ptr_rst_req[11]"),
	GCR_DESC(43, 9, 11, 0x3, "VSP_JESDTX4", "Request is connected to VSP3 dma_ptr_rst_req[11]"),
	GCR_DESC(43, 9, 11, 0x4, "VSP_JESDTX4", "Request is connected to VSP4 dma_ptr_rst_req[11]"),
	GCR_DESC(43, 6, 8, 0x1, "VSP_JESDTX3", "Request is connected to VSP1 dma_ptr_rst_req[10]"),
	GCR_DESC(43, 6, 8, 0x2, "VSP_JESDTX3", "Request is connected to VSP2 dma_ptr_rst_req[10]"),
	GCR_DESC(43, 6, 8, 0x3, "VSP_JESDTX3", "Request is connected to VSP3 dma_ptr_rst_req[10]"),
	GCR_DESC(43, 6, 8, 0x4, "VSP_JESDTX3", "Request is connected to VSP4 dma_ptr_rst_req[10]"),
	GCR_DESC(43, 3, 5, 0x1, "VSP_JESDTX2", "Request is connected to VSP1 dma_ptr_rst_req[9]"),
	GCR_DESC(43, 3, 5, 0x2, "VSP_JESDTX2", "Request is connected to VSP2 dma_ptr_rst_req[9]"),
	GCR_DESC(43, 3, 5, 0x3, "VSP_JESDTX2", "Request is connected to VSP3 dma_ptr_rst_req[9]"),
	GCR_DESC(43, 3, 5, 0x4, "VSP_JESDTX2", "Request is connected to VSP4 dma_ptr_rst_req[9]"),
	GCR_DESC(43, 0, 2, 0x1, "VSP_JESDTX1", "Request is connected to VSP1 dma_ptr_rst_req[8]"),
	GCR_DESC(43, 0, 2, 0x2, "VSP_JESDTX1", "Request is connected to VSP2 dma_ptr_rst_req[8]"),
	GCR_DESC(43, 0, 2, 0x3, "VSP_JESDTX1", "Request is connected to VSP3 dma_ptr_rst_req[8]"),
	GCR_DESC(43, 0, 2, 0x4, "VSP_JESDTX1", "Request is connected to VSP4 dma_ptr_rst_req[8]"),
	GCR_DESC(44, 9, 11, 0x1, "VSP_JESDTX8", "Request is connected to VSP8 dma_ptr_rst_req[11]"),
	GCR_DESC(44, 9, 11, 0x2, "VSP_JESDTX8", "Request is connected to VSP9 dma_ptr_rst_req[11]"),
	GCR_DESC(44, 9, 11, 0x3, "VSP_JESDTX8", "Request is connected to VSP10 dma_ptr_rst_req[11]"),
	GCR_DESC(44, 9, 11, 0x4, "VSP_JESDTX8", "Request is connected to VSP11 dma_ptr_rst_req[11]"),
	GCR_DESC(44, 6, 8, 0x1, "VSP_JESDTX7", "Request is connected to VSP8 dma_ptr_rst_req[10]"),
	GCR_DESC(44, 6, 8, 0x2, "VSP_JESDTX7", "Request is connected to VSP9 dma_ptr_rst_req[10]"),
	GCR_DESC(44, 6, 8, 0x3, "VSP_JESDTX7", "Request is connected to VSP10 dma_ptr_rst_req[10]"),
	GCR_DESC(44, 6, 8, 0x4, "VSP_JESDTX7", "Request is connected to VSP11 dma_ptr_rst_req[10]"),
	GCR_DESC(44, 3, 5, 0x1, "VSP_JESDTX6", "Request is connected to VSP8 dma_ptr_rst_req[9]"),
	GCR_DESC(44, 3, 5, 0x2, "VSP_JESDTX6", "Request is connected to VSP9 dma_ptr_rst_req[9]"),
	GCR_DESC(44, 3, 5, 0x3, "VSP_JESDTX6", "Request is connected to VSP10 dma_ptr_rst_req[9]"),
	GCR_DESC(44, 3, 5, 0x4, "VSP_JESDTX6", "Request is connected to VSP11 dma_ptr_rst_req[9]"),
	GCR_DESC(44, 0, 2, 0x1, "VSP_JESDTX5", "Request is connected to VSP8 dma_ptr_rst_req[8]"),
	GCR_DESC(44, 0, 2, 0x2, "VSP_JESDTX5", "Request is connected to VSP9 dma_ptr_rst_req[8]"),
	GCR_DESC(44, 0, 2, 0x3, "VSP_JESDTX5", "Request is connected to VSP10 dma_ptr_rst_req[8]"),
	GCR_DESC(44, 0, 2, 0x4, "VSP_JESDTX5", "Request is connected to VSP11 dma_ptr_rst_req[8]"),
	GCR_DESC(45, 22, 23, 0x1, "VSP_JESDRX10", "Request is connected to VSP5 dma_ptr_rst_req[13]"),
	GCR_DESC(45, 22, 23, 0x2, "VSP_JESDRX10", "Request is connected to VSP6 dma_ptr_rst_req[13]"),
	GCR_DESC(45, 22, 23, 0x3, "VSP_JESDRX10", "Request is connected to VSP7 dma_ptr_rst_req[13]"),
	GCR_DESC(45, 20, 21, 0x1, "VSP_JESDRX6", "Request is connected to VSP5 dma_ptr_rst_req[13]"),
	GCR_DESC(45, 20, 21, 0x2, "VSP_JESDRX6", "Request is connected to VSP6 dma_ptr_rst_req[13]"),
	GCR_DESC(45, 20, 21, 0x3, "VSP_JESDRX6", "Request is connected to VSP7 dma_ptr_rst_req[13]"),
	GCR_DESC(45, 18, 19, 0x1, "VSP_JESDRX09", "Request is connected to VSP5 dma_ptr_rst_req[12]"),
	GCR_DESC(45, 18, 19, 0x2, "VSP_JESDRX09", "Request is connected to VSP6 dma_ptr_rst_req[12]"),
	GCR_DESC(45, 18, 19, 0x3, "VSP_JESDRX09", "Request is connected to VSP7 dma_ptr_rst_req[12]"),
	GCR_DESC(45, 16, 17, 0x1, "VSP_JESDRX5", "Request is connected to VSP5 dma_ptr_rst_req[12]"),
	GCR_DESC(45, 16, 17, 0x2, "VSP_JESDRX5", "Request is connected to VSP6 dma_ptr_rst_req[12]"),
	GCR_DESC(45, 16, 17, 0x3, "VSP_JESDRX5", "Request is connected to VSP7 dma_ptr_rst_req[12]"),
	GCR_DESC(45, 14, 15, 0x1, "VSP_JESDRX08", "Request is connected to VSP5 dma_ptr_rst_req[11]"),
	GCR_DESC(45, 14, 15, 0x2, "VSP_JESDRX08", "Request is connected to VSP6 dma_ptr_rst_req[11]"),
	GCR_DESC(45, 14, 15, 0x3, "VSP_JESDRX08", "Request is connected to VSP7 dma_ptr_rst_req[11]"),
	GCR_DESC(45, 12, 13, 0x1, "VSP_JESDRX4", "Request is connected to VSP5 dma_ptr_rst_req[11]"),
	GCR_DESC(45, 12, 13, 0x2, "VSP_JESDRX4", "Request is connected to VSP6 dma_ptr_rst_req[11]"),
	GCR_DESC(45, 12, 13, 0x3, "VSP_JESDRX4", "Request is connected to VSP7 dma_ptr_rst_req[11]"),
	GCR_DESC(45, 10, 11, 0x1, "VSP_JESDRX07", "Request is connected to VSP5 dma_ptr_rst_req[10]"),
	GCR_DESC(45, 10, 11, 0x2, "VSP_JESDRX07", "Request is connected to VSP6 dma_ptr_rst_req[10]"),
	GCR_DESC(45, 10, 11, 0x3, "VSP_JESDRX07", "Request is connected to VSP7 dma_ptr_rst_req[10]"),
	GCR_DESC(45, 8, 9, 0x1, "VSP_JESDRX3", "Request is connected to VSP5 dma_ptr_rst_req[10]"),
	GCR_DESC(45, 8, 9, 0x2, "VSP_JESDRX3", "Request is connected to VSP6 dma_ptr_rst_req[10]"),
	GCR_DESC(45, 8, 9, 0x3, "VSP_JESDRX3", "Request is connected to VSP7 dma_ptr_rst_req[10]"),
	GCR_DESC(45, 6, 7, 0x1, "VSP_JESDRX06", "Request is connected to VSP5 dma_ptr_rst_req[9]"),
	GCR_DESC(45, 6, 7, 0x2, "VSP_JESDRX06", "Request is connected to VSP6 dma_ptr_rst_req[9]"),
	GCR_DESC(45, 6, 7, 0x3, "VSP_JESDRX06", "Request is connected to VSP7 dma_ptr_rst_req[9]"),
	GCR_DESC(45, 4, 5, 0x1, "VSP_JESDRX2", "Request is connected to VSP5 dma_ptr_rst_req[9]"),
	GCR_DESC(45, 4, 5, 0x2, "VSP_JESDRX2", "Request is connected to VSP6 dma_ptr_rst_req[9]"),
	GCR_DESC(45, 4, 5, 0x3, "VSP_JESDRX2", "Request is connected to VSP7 dma_ptr_rst_req[9]"),
	GCR_DESC(45, 2, 3, 0x1, "VSP_JESDRX05", "Request is connected to VSP5 dma_ptr_rst_req[8]"),
	GCR_DESC(45, 2, 3, 0x2, "VSP_JESDRX05", "Request is connected to VSP6 dma_ptr_rst_req[8]"),
	GCR_DESC(45, 2, 3, 0x3, "VSP_JESDRX05", "Request is connected to VSP7 dma_ptr_rst_req[8]"),
	GCR_DESC(45, 0, 1, 0x1, "VSP_JESDRX1", "Request is connected to VSP5 dma_ptr_rst_req[8]"),
	GCR_DESC(45, 0, 1, 0x2, "VSP_JESDRX1", "Request is connected to VSP6 dma_ptr_rst_req[8]"),
	GCR_DESC(45, 0, 1, 0x3, "VSP_JESDRX1", "Request is connected to VSP7 dma_ptr_rst_req[8]"),
	GCR_DESC(46, 29, 31, 0x1, "VSP4_CH13", "JESDTX_6 dma request"),
	GCR_DESC(46, 29, 31, 0x2, "VSP4_CH13", "JESDTX_10 dma request"),
	GCR_DESC(46, 29, 31, 0x3, "VSP4_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(46, 29, 31, 0x4, "VSP4_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(46, 29, 31, 0x5, "VSP4_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(46, 29, 31, 0x6, "VSP4_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(46, 26, 28, 0x1, "VSP3_CH13", "JESDTX_6 dma request"),
	GCR_DESC(46, 26, 28, 0x2, "VSP3_CH13", "JESDTX_10 dma request"),
	GCR_DESC(46, 26, 28, 0x3, "VSP3_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(46, 26, 28, 0x4, "VSP3_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(46, 26, 28, 0x5, "VSP3_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(46, 26, 28, 0x6, "VSP3_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(46, 23, 25, 0x1, "VSP2_CH13", "JESDTX_6 dma request"),
	GCR_DESC(46, 23, 25, 0x2, "VSP2_CH13", "JESDTX_10 dma request"),
	GCR_DESC(46, 23, 25, 0x3, "VSP2_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(46, 23, 25, 0x4, "VSP2_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(46, 23, 25, 0x5, "VSP2_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(46, 23, 25, 0x6, "VSP2_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(46, 20, 22, 0x1, "VSP1_CH13", "JESDTX_6 dma request"),
	GCR_DESC(46, 20, 22, 0x2, "VSP1_CH13", "JESDTX_10 dma0 buffer ready"),
	GCR_DESC(46, 20, 22, 0x3, "VSP1_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(46, 20, 22, 0x4, "VSP1_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(46, 20, 22, 0x5, "VSP1_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(46, 20, 22, 0x6, "VSP1_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(46, 9, 11, 0x1, "VSP4_CH12", "JESDTX_5 dma request"),
	GCR_DESC(46, 9, 11, 0x2, "VSP4_CH12", "JESDTX_9 dma request"),
	GCR_DESC(46, 9, 11, 0x3, "VSP4_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(46, 9, 11, 0x4, "VSP4_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(46, 9, 11, 0x5, "VSP4_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(46, 9, 11, 0x6, "VSP4_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(46, 6, 8, 0x1, "VSP3_CH12", "JESDTX_5 dma request"),
	GCR_DESC(46, 6, 8, 0x2, "VSP3_CH12", "JESDTX_9 dma request"),
	GCR_DESC(46, 6, 8, 0x3, "VSP3_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(46, 6, 8, 0x4, "VSP3_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(46, 6, 8, 0x5, "VSP3_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(46, 6, 8, 0x6, "VSP3_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(46, 3, 5, 0x1, "VSP2_CH12", "JESDTX_5 dma request"),
	GCR_DESC(46, 3, 5, 0x2, "VSP2_CH12", "JESDTX_9 dma request"),
	GCR_DESC(46, 3, 5, 0x3, "VSP2_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(46, 3, 5, 0x4, "VSP2_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(46, 3, 5, 0x5, "VSP2_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(46, 3, 5, 0x6, "VSP2_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(46, 0, 2, 0x1, "VSP1_CH12", "JESDTX_5 dma request"),
	GCR_DESC(46, 0, 2, 0x2, "VSP1_CH12", "JESDTX_9 dma request"),
	GCR_DESC(46, 0, 2, 0x3, "VSP1_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(46, 0, 2, 0x4, "VSP1_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(46, 0, 2, 0x5, "VSP1_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(46, 0, 2, 0x6, "VSP1_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(47, 29, 31, 0x1, "VSP4_CH15", "JESDTX_8 dma request"),
	GCR_DESC(47, 29, 31, 0x2, "VSP4_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(47, 29, 31, 0x3, "VSP4_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(47, 29, 31, 0x4, "VSP4_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(47, 29, 31, 0x5, "VSP4_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(47, 26, 28, 0x1, "VSP3_CH15", "JESDTX_8 dma request"),
	GCR_DESC(47, 26, 28, 0x2, "VSP3_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(47, 26, 28, 0x3, "VSP3_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(47, 26, 28, 0x4, "VSP3_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(47, 26, 28, 0x5, "VSP3_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(47, 23, 25, 0x1, "VSP2_CH15", "JESDTX_8 dma request"),
	GCR_DESC(47, 23, 25, 0x2, "VSP2_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(47, 23, 25, 0x3, "VSP2_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(47, 23, 25, 0x4, "VSP2_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(47, 23, 25, 0x5, "VSP2_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(47, 20, 22, 0x1, "VSP1_CH15", "JESDTX_8 dma request"),
	GCR_DESC(47, 20, 22, 0x2, "VSP1_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(47, 20, 22, 0x3, "VSP1_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(47, 20, 22, 0x4, "VSP1_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(47, 20, 22, 0x5, "VSP1_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(47, 9, 11, 0x1, "VSP4_CH14", "JESDTX_7 dma request"),
	GCR_DESC(47, 9, 11, 0x2, "VSP4_CH14", "JESDRX_13 dma request"),
	GCR_DESC(47, 9, 11, 0x3, "VSP4_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(47, 9, 11, 0x4, "VSP4_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(47, 9, 11, 0x5, "VSP4_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(47, 9, 11, 0x6, "VSP4_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(47, 6, 8, 0x1, "VSP3_CH14", "JESDTX_7 dma request"),
	GCR_DESC(47, 6, 8, 0x2, "VSP3_CH14", "JESDRX_13 dma request"),
	GCR_DESC(47, 6, 8, 0x3, "VSP3_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(47, 6, 8, 0x4, "VSP3_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(47, 6, 8, 0x5, "VSP3_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(47, 6, 8, 0x6, "VSP3_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(47, 3, 5, 0x1, "VSP2_CH14", "JESDTX_7 dma request"),
	GCR_DESC(47, 3, 5, 0x2, "VSP2_CH14", "JESDRX_13 dma request"),
	GCR_DESC(47, 3, 5, 0x3, "VSP2_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(47, 3, 5, 0x4, "VSP2_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(47, 3, 5, 0x5, "VSP2_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(47, 3, 5, 0x6, "VSP2_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(47, 0, 2, 0x1, "VSP1_CH14", "JESDTX_7 dma request"),
	GCR_DESC(47, 0, 2, 0x2, "VSP1_CH14", "JESDRX_13 dma request"),
	GCR_DESC(47, 0, 2, 0x3, "VSP1_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(47, 0, 2, 0x4, "VSP1_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(47, 0, 2, 0x5, "VSP1_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(47, 0, 2, 0x6, "VSP1_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(48, 20, 22, 0x1, "VSP_JESDTX08", "Request is connected to VSP1 dma_ptr_rst_req[15]"),
	GCR_DESC(48, 20, 22, 0x2, "VSP_JESDTX08", "Request is connected to VSP2 dma_ptr_rst_req[15]"),
	GCR_DESC(48, 20, 22, 0x3, "VSP_JESDTX08", "Request is connected to VSP3 dma_ptr_rst_req[15]"),
	GCR_DESC(48, 20, 22, 0x4, "VSP_JESDTX08", "Request is connected to VSP4 dma_ptr_rst_req[15]"),
	GCR_DESC(48, 17, 19, 0x1, "VSP_JESDRX13", "Request is connected to VSP1 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 17, 19, 0x2, "VSP_JESDRX13", "Request is connected to VSP2 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 17, 19, 0x3, "VSP_JESDRX13", "Request is connected to VSP3 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 17, 19, 0x4, "VSP_JESDRX13", "Request is connected to VSP4 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 14, 16, 0x1, "VSP_JESDTX07", "Request is connected to VSP1 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 14, 16, 0x2, "VSP_JESDTX07", "Request is connected to VSP2 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 14, 16, 0x3, "VSP_JESDTX07", "Request is connected to VSP3 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 14, 16, 0x4, "VSP_JESDTX07", "Request is connected to VSP4 dma_ptr_rst_req[14]"),
	GCR_DESC(48, 11, 13, 0x1, "VSP_JESDTX10", "Request is connected to VSP1 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 11, 13, 0x2, "VSP_JESDTX10", "Request is connected to VSP2 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 11, 13, 0x3, "VSP_JESDTX10", "Request is connected to VSP3 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 11, 13, 0x4, "VSP_JESDTX10", "Request is connected to VSP4 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 6, 8, 0x1, "VSP_JESDTX06", "Request is connected to VSP1 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 6, 8, 0x2, "VSP_JESDTX06", "Request is connected to VSP2 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 6, 8, 0x3, "VSP_JESDTX06", "Request is connected to VSP3 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 6, 8, 0x4, "VSP_JESDTX06", "Request is connected to VSP4 dma_ptr_rst_req[13]"),
	GCR_DESC(48, 3, 5, 0x1, "VSP_JESDTX09", "Request is connected to VSP1 dma_ptr_rst_req[12]"),
	GCR_DESC(48, 3, 5, 0x2, "VSP_JESDTX09", "Request is connected to VSP2 dma_ptr_rst_req[12]"),
	GCR_DESC(48, 3, 5, 0x3, "VSP_JESDTX09", "Request is connected to VSP3 dma_ptr_rst_req[12]"),
	GCR_DESC(48, 3, 5, 0x4, "VSP_JESDTX09", "Request is connected to VSP4 dma_ptr_rst_req[12]"),
	GCR_DESC(48, 0, 2, 0x1, "VSP_JESDTX05", "Request is connected to VSP1 dma_ptr_rst_req[12]"),
	GCR_DESC(48, 0, 2, 0x2, "VSP_JESDTX05", "Request is connected to VSP2 dma_ptr_rst_req[12]"),
	GCR_DESC(48, 0, 2, 0x3, "VSP_JESDTX05", "Request is connected to VSP3 dma_ptr_rst_req[12]"),
	GCR_DESC(48, 0, 2, 0x4, "VSP_JESDTX05", "Request is connected to VSP4 dma_ptr_rst_req[12]"),
	GCR_DESC(49, 29, 31, 0x1, "VSP11_CH13", "JESDTX_2 dma request"),
	GCR_DESC(49, 29, 31, 0x2, "VSP11_CH13", "JESDTX_10 dma request"),
	GCR_DESC(49, 29, 31, 0x3, "VSP11_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(49, 29, 31, 0x4, "VSP11_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(49, 29, 31, 0x5, "VSP11_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(49, 29, 31, 0x6, "VSP11_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(49, 26, 28, 0x1, "VSP10_CH13", "JESDTX_2 dma request"),
	GCR_DESC(49, 26, 28, 0x2, "VSP10_CH13", "JESDTX_10 dma request"),
	GCR_DESC(49, 26, 28, 0x3, "VSP10_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(49, 26, 28, 0x4, "VSP10_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(49, 26, 28, 0x5, "VSP10_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(49, 26, 28, 0x6, "VSP10_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(49, 23, 25, 0x1, "VSP9_CH13", "JESDTX_2 dma request"),
	GCR_DESC(49, 23, 25, 0x2, "VSP9_CH13", "JESDTX_10 dma request"),
	GCR_DESC(49, 23, 25, 0x3, "VSP9_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(49, 23, 25, 0x4, "VSP9_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(49, 23, 25, 0x5, "VSP9_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(49, 23, 25, 0x6, "VSP9_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(49, 20, 22, 0x1, "VSP8_CH13", "JESDTX_2 dma request"),
	GCR_DESC(49, 20, 22, 0x2, "VSP8_CH13", "JESDTX_10 dma0 buffer ready"),
	GCR_DESC(49, 20, 22, 0x3, "VSP8_CH13", "cpri_rx1 dma request 19"),
	GCR_DESC(49, 20, 22, 0x4, "VSP8_CH13", "cpri_rx1 dma request 22"),
	GCR_DESC(49, 20, 22, 0x5, "VSP8_CH13", "cpri_rx2 dma request 19"),
	GCR_DESC(49, 20, 22, 0x6, "VSP8_CH13", "cpri_rx2 dma request 22"),
	GCR_DESC(49, 9, 11, 0x1, "VSP11_CH12", "JESDTX_1 dma request"),
	GCR_DESC(49, 9, 11, 0x2, "VSP11_CH12", "JESDTX_9 dma request"),
	GCR_DESC(49, 9, 11, 0x3, "VSP11_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(49, 9, 11, 0x4, "VSP11_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(49, 9, 11, 0x5, "VSP11_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(49, 9, 11, 0x6, "VSP11_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(49, 6, 8, 0x1, "VSP10_CH12", "JESDTX_1 dma request"),
	GCR_DESC(49, 6, 8, 0x2, "VSP10_CH12", "JESDTX_9 dma request"),
	GCR_DESC(49, 6, 8, 0x3, "VSP10_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(49, 6, 8, 0x4, "VSP10_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(49, 6, 8, 0x5, "VSP10_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(49, 6, 8, 0x6, "VSP10_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(49, 3, 5, 0x1, "VSP9_CH12", "JESDTX_1 dma request"),
	GCR_DESC(49, 3, 5, 0x2, "VSP9_CH12", "JESDTX_9 dma request"),
	GCR_DESC(49, 3, 5, 0x3, "VSP9_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(49, 3, 5, 0x4, "VSP9_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(49, 3, 5, 0x5, "VSP9_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(49, 3, 5, 0x6, "VSP9_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(49, 0, 2, 0x1, "VSP8_CH12", "JESDTX_1 dma request"),
	GCR_DESC(49, 0, 2, 0x2, "VSP8_CH12", "JESDTX_9 dma request"),
	GCR_DESC(49, 0, 2, 0x3, "VSP8_CH12", "cpri_rx1 dma request 20"),
	GCR_DESC(49, 0, 2, 0x4, "VSP8_CH12", "cpri_rx1 dma request 21"),
	GCR_DESC(49, 0, 2, 0x5, "VSP8_CH12", "cpri_rx2 dma request 20"),
	GCR_DESC(49, 0, 2, 0x6, "VSP8_CH12", "cpri_rx2 dma request 21"),
	GCR_DESC(50, 29, 31, 0x1, "VSP11_CH15", "JESDTX_4 dma request"),
	GCR_DESC(50, 29, 31, 0x2, "VSP11_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(50, 29, 31, 0x3, "VSP11_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(50, 29, 31, 0x4, "VSP11_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(50, 29, 31, 0x5, "VSP11_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(50, 26, 28, 0x1, "VSP10_CH15", "JESDTX_4 dma request"),
	GCR_DESC(50, 26, 28, 0x2, "VSP10_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(50, 26, 28, 0x3, "VSP10_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(50, 26, 28, 0x4, "VSP10_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(50, 26, 28, 0x5, "VSP10_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(50, 23, 25, 0x1, "VSP9_CH15", "JESDTX_4 dma request"),
	GCR_DESC(50, 23, 25, 0x2, "VSP9_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(50, 23, 25, 0x3, "VSP9_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(50, 23, 25, 0x4, "VSP9_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(50, 23, 25, 0x5, "VSP9_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(50, 20, 22, 0x1, "VSP8_CH15", "JESDTX_4 dma request"),
	GCR_DESC(50, 20, 22, 0x2, "VSP8_CH15", "cpri_rx1 dma request 17"),
	GCR_DESC(50, 20, 22, 0x3, "VSP8_CH15", "cpri_rx1 dma request 24"),
	GCR_DESC(50, 20, 22, 0x4, "VSP8_CH15", "cpri_rx2 dma request 17"),
	GCR_DESC(50, 20, 22, 0x5, "VSP8_CH15", "cpri_rx2 dma request 24"),
	GCR_DESC(50, 9, 11, 0x1, "VSP11_CH14", "JESDTX_3 dma request"),
	GCR_DESC(50, 9, 11, 0x2, "VSP11_CH14", "JESDRX_13 dma request"),
	GCR_DESC(50, 9, 11, 0x3, "VSP11_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(50, 9, 11, 0x4, "VSP11_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(50, 9, 11, 0x5, "VSP11_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(50, 9, 11, 0x6, "VSP11_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(50, 6, 8, 0x1, "VSP10_CH14", "JESDTX_3 dma request"),
	GCR_DESC(50, 6, 8, 0x2, "VSP10_CH14", "JESDRX_13 dma request"),
	GCR_DESC(50, 6, 8, 0x3, "VSP10_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(50, 6, 8, 0x4, "VSP10_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(50, 6, 8, 0x5, "VSP10_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(50, 6, 8, 0x6, "VSP10_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(50, 3, 5, 0x1, "VSP9_CH14", "JESDTX_3 dma request"),
	GCR_DESC(50, 3, 5, 0x2, "VSP9_CH14", "JESDRX_13 dma request"),
	GCR_DESC(50, 3, 5, 0x3, "VSP9_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(50, 3, 5, 0x4, "VSP9_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(50, 3, 5, 0x5, "VSP9_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(50, 3, 5, 0x6, "VSP9_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(50, 0, 2, 0x1, "VSP8_CH14", "JESDTX_3 dma request"),
	GCR_DESC(50, 0, 2, 0x2, "VSP8_CH14", "JESDRX_13 dma request"),
	GCR_DESC(50, 0, 2, 0x3, "VSP8_CH14", "cpri_rx1 dma request 18"),
	GCR_DESC(50, 0, 2, 0x4, "VSP8_CH14", "cpri_rx1 dma request 23"),
	GCR_DESC(50, 0, 2, 0x5, "VSP8_CH14", "cpri_rx2 dma request 18"),
	GCR_DESC(50, 0, 2, 0x6, "VSP8_CH14", "cpri_rx2 dma request 23"),
	GCR_DESC(51, 20, 22, 0x1, "VSP_JESDTX04", "Request is connected to VSP8 dma_ptr_rst_req[15]"),
	GCR_DESC(51, 20, 22, 0x2, "VSP_JESDTX04", "Request is connected to VSP9 dma_ptr_rst_req[15]"),
	GCR_DESC(51, 20, 22, 0x3, "VSP_JESDTX04", "Request is connected to VSP10 dma_ptr_rst_req[15]"),
	GCR_DESC(51, 20, 22, 0x4, "VSP_JESDTX04", "Request is connected to VSP11 dma_ptr_rst_req[15]"),
	GCR_DESC(51, 17, 19, 0x1, "VSP_JESDRX13", "Request is connected to VSP8 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 17, 19, 0x2, "VSP_JESDRX13", "Request is connected to VSP9 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 17, 19, 0x3, "VSP_JESDRX13", "Request is connected to VSP10 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 17, 19, 0x4, "VSP_JESDRX13", "Request is connected to VSP11 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 14, 16, 0x1, "VSP_JESDTX03", "Request is connected to VSP8 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 14, 16, 0x2, "VSP_JESDTX03", "Request is connected to VSP9 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 14, 16, 0x3, "VSP_JESDTX03", "Request is connected to VSP10 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 14, 16, 0x4, "VSP_JESDTX03", "Request is connected to VSP11 dma_ptr_rst_req[14]"),
	GCR_DESC(51, 11, 13, 0x1, "VSP_JESDTX10", "Request is connected to VSP8 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 11, 13, 0x2, "VSP_JESDTX10", "Request is connected to VSP9 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 11, 13, 0x3, "VSP_JESDTX10", "Request is connected to VSP10 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 11, 13, 0x4, "VSP_JESDTX10", "Request is connected to VSP11 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 6, 8, 0x1, "VSP_JESDTX02", "Request is connected to VSP8 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 6, 8, 0x2, "VSP_JESDTX02", "Request is connected to VSP9 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 6, 8, 0x3, "VSP_JESDTX02", "Request is connected to VSP10 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 6, 8, 0x4, "VSP_JESDTX02", "Request is connected to VSP11 dma_ptr_rst_req[13]"),
	GCR_DESC(51, 3, 5, 0x1, "VSP_JESDTX09", "Request is connected to VSP8 dma_ptr_rst_req[12]"),
	GCR_DESC(51, 3, 5, 0x2, "VSP_JESDTX09", "Request is connected to VSP9 dma_ptr_rst_req[12]"),
	GCR_DESC(51, 3, 5, 0x3, "VSP_JESDTX09", "Request is connected to VSP10 dma_ptr_rst_req[12]"),
	GCR_DESC(51, 3, 5, 0x4, "VSP_JESDTX09", "Request is connected to VSP11 dma_ptr_rst_req[12]"),
	GCR_DESC(51, 0, 2, 0x1, "VSP_JESDTX01", "Request is connected to VSP8 dma_ptr_rst_req[12]"),
	GCR_DESC(51, 0, 2, 0x2, "VSP_JESDTX01", "Request is connected to VSP9 dma_ptr_rst_req[12]"),
	GCR_DESC(51, 0, 2, 0x3, "VSP_JESDTX01", "Request is connected to VSP10 dma_ptr_rst_req[12]"),
	GCR_DESC(51, 0, 2, 0x4, "VSP_JESDTX01", "Request is connected to VSP11 dma_ptr_rst_req[12]"),
	GCR_DESC(52, 28, 29, 0x1, "JESDTX10 pointer reset request selection", "VSP_1_2_3_4_CH13 pointer reset request for JESDTX10"),
	GCR_DESC(52, 28, 29, 0x2, "JESDTX10 pointer reset request selection", "VSP_8_9_10_11_CH13 pointer reset request for JESDTX10"),
	GCR_DESC(52, 28, 29, 0x3, "JESDTX10 pointer reset request selection", "VSP_5_6_7_CH15 pointer reset request for JESDTX10"),
	GCR_DESC(52, 26, 27, 0x1, "JESDTX9 pointer reset request selection", "VSP_1_2_3_4_CH12 pointer reset request for JESDTX9"),
	GCR_DESC(52, 26, 27, 0x2, "JESDTX9 pointer reset request selection", "VSP_8_9_10_11_CH12 pointer reset request for JESDTX9"),
	GCR_DESC(52, 26, 27, 0x3, "JESDTX9 pointer reset request selection", "VSP_5_6_7_CH14 pointer reset request for JESDTX9"),
	GCR_DESC(52, 24, 25, 0x1, "JESDTX8 pointer reset request selection", "VSP_1_2_3_4_CH15 pointer reset request for JESDTX8"),
	GCR_DESC(52, 24, 25, 0x2, "JESDTX8 pointer reset request selection", "VSP_8_9_10_11_CH11 pointer reset request for JESDTX8"),
	GCR_DESC(52, 22, 23, 0x1, "JESDTX7 pointer reset request selection", "VSP_1_2_3_4_CH14 pointer reset request for JESDTX7"),
	GCR_DESC(52, 22, 23, 0x2, "JESDTX7 pointer reset request selection", "VSP_8_9_10_11_CH10 pointer reset request for JESDTX7"),
	GCR_DESC(52, 20, 21, 0x1, "JESDTX6 pointer reset request selection", "VSP_1_2_3_4_CH13 pointer reset request for JESDTX6"),
	GCR_DESC(52, 20, 21, 0x2, "JESDTX6 pointer reset request selection", "VSP_8_9_10_11_CH9 pointer reset request for JESDTX6"),
	GCR_DESC(52, 18, 19, 0x1, "JESDTX5 pointer reset request selection", "VSP_1_2_3_4_CH12 pointer reset request for JESDTX5"),
	GCR_DESC(52, 18, 19, 0x2, "JESDTX5 pointer reset request selection", "VSP_8_9_10_11_CH8 pointer reset request for JESDTX5"),
	GCR_DESC(52, 16, 17, 0x1, "JESDTX4 pointer reset request selection", "VSP_1_2_3_4_CH11 pointer reset request for JESDTX4"),
	GCR_DESC(52, 16, 17, 0x2, "JESDTX4 pointer reset request selection", "VSP_8_9_10_11_CH15 pointer reset request for JESDTX4"),
	GCR_DESC(52, 14, 15, 0x1, "JESDTX3 pointer reset request selection", "VSP_1_2_3_4_CH10 pointer reset request for JESDTX3"),
	GCR_DESC(52, 14, 15, 0x2, "JESDTX3 pointer reset request selection", "VSP_8_9_10_11_CH15 pointer reset request for JESDTX3"),
	GCR_DESC(52, 12, 13, 0x1, "JESDTX2 pointer reset request selection", "VSP_1_2_3_4_CH9 pointer reset request for JESDTX2"),
	GCR_DESC(52, 12, 13, 0x2, "JESDTX2 pointer reset request selection", "VSP_8_9_10_11_CH13 pointer reset request for JESDTX2"),
	GCR_DESC(52, 10, 11, 0x1, "JESDTX1 pointer reset request selection", "VSP_1_2_3_4_CH8 pointer reset request for JESDTX1"),
	GCR_DESC(52, 10, 11, 0x2, "JESDTX1 pointer reset request selection", "VSP_8_9_10_11_CH12 pointer reset request for JESDTX1"),
	GCR_DESC(52, 8, 9, 0x1, "JESDRX8 pointer reset request selection", "VSP_5_6_7_CH11 pointer reset request for JESDRX8"),
	GCR_DESC(52, 8, 9, 0x2, "JESDRX8 pointer reset request selection", "VSP_5_6_7_CH15 pointer reset request for JESDRX8"),
	GCR_DESC(52, 6, 7, 0x1, "JESDRX7 pointer reset request selection", "VSP_5_6_7_CH10 pointer reset request for JESDRX7"),
	GCR_DESC(52, 6, 7, 0x2, "JESDRX7 pointer reset request selection", "VSP_5_6_7_CH14 pointer reset request for JESDRX7"),
	GCR_DESC(52, 4, 5, 0x1, "JESDRX6 pointer reset request selection", "VSP_5_6_7_CH9 pointer reset request for JESDRX6"),
	GCR_DESC(52, 4, 5, 0x2, "JESDRX6 pointer reset request selection", "VSP_5_6_7_CH13 pointer reset request for JESDRX6"),
	GCR_DESC(52, 2, 3, 0x1, "JESDRX5 pointer reset request selection", "VSP_5_6_7_CH8 pointer reset request for JESDRX5"),
	GCR_DESC(52, 2, 3, 0x2, "JESDRX5 pointer reset request selection", "VSP_5_6_7_CH12 pointer reset request for JESDRX5"),
	GCR_DESC(52, 0, 1, 0x1, "JESDRX13 pointer reset request selection", "VSP_1_2_3_4_CH14 pointer reset request for JESDRX13"),
	GCR_DESC(52, 0, 1, 0x2, "JESDRX13 pointer reset request selection", "VSP_8_9_10_11_CH14 pointer reset request for JESDRX13"),
	GCR_DESC(56, 30, 31, 0x0, "VSP1_CH31", "cpri_rx1 dma request 1"),
	GCR_DESC(56, 30, 31, 0x1, "VSP1_CH31", "cpri_rx1 dma request 17"),
	GCR_DESC(56, 30, 31, 0x2, "VSP1_CH31", "cpri_rx2 dma request 1"),
	GCR_DESC(56, 30, 31, 0x3, "VSP1_CH31", "cpri_rx2 dma request 17"),
	GCR_DESC(56, 28, 29, 0x0, "VSP1_CH30", "cpri_rx1 dma request 2"),
	GCR_DESC(56, 28, 29, 0x1, "VSP1_CH30", "cpri_rx1 dma request 18"),
	GCR_DESC(56, 28, 29, 0x2, "VSP1_CH30", "cpri_rx2 dma request 2"),
	GCR_DESC(56, 28, 29, 0x3, "VSP1_CH30", "cpri_rx2 dma request 18"),
	GCR_DESC(56, 26, 27, 0x0, "VSP1_CH29", "cpri_rx1 dma request 3"),
	GCR_DESC(56, 26, 27, 0x1, "VSP1_CH29", "cpri_rx1 dma request 19"),
	GCR_DESC(56, 26, 27, 0x2, "VSP1_CH29", "cpri_rx2 dma request 3"),
	GCR_DESC(56, 26, 27, 0x3, "VSP1_CH29", "cpri_rx2 dma request 19"),
	GCR_DESC(56, 24, 25, 0x0, "VSP1_CH28", "cpri_rx1 dma request 4"),
	GCR_DESC(56, 24, 25, 0x1, "VSP1_CH28", "cpri_rx1 dma request 20"),
	GCR_DESC(56, 24, 25, 0x2, "VSP1_CH28", "cpri_rx2 dma request 4"),
	GCR_DESC(56, 24, 25, 0x3, "VSP1_CH28", "cpri_rx2 dma request 20"),
	GCR_DESC(56, 22, 23, 0x0, "VSP1_CH27", "cpri_rx1 dma request 5"),
	GCR_DESC(56, 22, 23, 0x1, "VSP1_CH27", "cpri_rx1 dma request 21"),
	GCR_DESC(56, 22, 23, 0x2, "VSP1_CH27", "cpri_rx2 dma request 5"),
	GCR_DESC(56, 22, 23, 0x3, "VSP1_CH27", "cpri_rx2 dma request 21"),
	GCR_DESC(56, 20, 21, 0x0, "VSP1_CH26", "cpri_rx1 dma request 6"),
	GCR_DESC(56, 20, 21, 0x1, "VSP1_CH26", "cpri_rx1 dma request 22"),
	GCR_DESC(56, 20, 21, 0x2, "VSP1_CH26", "cpri_rx2 dma request 6"),
	GCR_DESC(56, 20, 21, 0x3, "VSP1_CH26", "cpri_rx2 dma request 22"),
	GCR_DESC(56, 18, 19, 0x0, "VSP1_CH25", "cpri_rx1 dma request 7"),
	GCR_DESC(56, 18, 19, 0x1, "VSP1_CH25", "cpri_rx1 dma request 23"),
	GCR_DESC(56, 18, 19, 0x2, "VSP1_CH25", "cpri_rx2 dma request 7"),
	GCR_DESC(56, 18, 19, 0x3, "VSP1_CH25", "cpri_rx2 dma request 23"),
	GCR_DESC(56, 16, 17, 0x0, "VSP1_CH24", "cpri_rx1 dma request 8"),
	GCR_DESC(56, 16, 17, 0x1, "VSP1_CH24", "cpri_rx1 dma request 24"),
	GCR_DESC(56, 16, 17, 0x2, "VSP1_CH24", "cpri_rx2 dma request 8"),
	GCR_DESC(56, 16, 17, 0x3, "VSP1_CH24", "cpri_rx2 dma request 24"),
	GCR_DESC(56, 14, 15, 0x0, "VSP1_CH23", "cpri_rx1 dma request 9"),
	GCR_DESC(56, 14, 15, 0x1, "VSP1_CH23", "cpri_rx1 dma request 17"),
	GCR_DESC(56, 14, 15, 0x2, "VSP1_CH23", "cpri_rx2 dma request 9"),
	GCR_DESC(56, 14, 15, 0x3, "VSP1_CH23", "cpri_rx2 dma request 17"),
	GCR_DESC(56, 12, 13, 0x0, "VSP1_CH22", "cpri_rx1 dma request 10"),
	GCR_DESC(56, 12, 13, 0x1, "VSP1_CH22", "cpri_rx1 dma request 18"),
	GCR_DESC(56, 12, 13, 0x2, "VSP1_CH22", "cpri_rx2 dma request 10"),
	GCR_DESC(56, 12, 13, 0x3, "VSP1_CH22", "cpri_rx2 dma request 18"),
	GCR_DESC(56, 10, 11, 0x0, "VSP1_CH21", "cpri_rx1 dma request 11"),
	GCR_DESC(56, 10, 11, 0x1, "VSP1_CH21", "cpri_rx1 dma request 19"),
	GCR_DESC(56, 10, 11, 0x2, "VSP1_CH21", "cpri_rx2 dma request 11"),
	GCR_DESC(56, 10, 11, 0x3, "VSP1_CH21", "cpri_rx2 dma request 19"),
	GCR_DESC(56, 8, 9, 0x0, "VSP1_CH20", "cpri_rx1 dma request 12"),
	GCR_DESC(56, 8, 9, 0x1, "VSP1_CH20", "cpri_rx1 dma request 20"),
	GCR_DESC(56, 8, 9, 0x2, "VSP1_CH20", "cpri_rx2 dma request 12"),
	GCR_DESC(56, 8, 9, 0x3, "VSP1_CH20", "cpri_rx2 dma request 20"),
	GCR_DESC(56, 6, 7, 0x0, "VSP1_CH19", "cpri_rx1 dma request 13"),
	GCR_DESC(56, 6, 7, 0x1, "VSP1_CH19", "cpri_rx1 dma request 21"),
	GCR_DESC(56, 6, 7, 0x2, "VSP1_CH19", "cpri_rx2 dma request 13"),
	GCR_DESC(56, 6, 7, 0x3, "VSP1_CH19", "cpri_rx2 dma request 21"),
	GCR_DESC(56, 4, 5, 0x0, "VSP1_CH18", "cpri_rx1 dma request 14"),
	GCR_DESC(56, 4, 5, 0x1, "VSP1_CH18", "cpri_rx1 dma request 22"),
	GCR_DESC(56, 4, 5, 0x2, "VSP1_CH18", "cpri_rx2 dma request 14"),
	GCR_DESC(56, 4, 5, 0x3, "VSP1_CH18", "cpri_rx2 dma request 22"),
	GCR_DESC(56, 2, 3, 0x0, "VSP1_CH17", "cpri_rx1 dma request 15"),
	GCR_DESC(56, 2, 3, 0x1, "VSP1_CH17", "cpri_rx1 dma request 23"),
	GCR_DESC(56, 2, 3, 0x2, "VSP1_CH17", "cpri_rx2 dma request 15"),
	GCR_DESC(56, 2, 3, 0x3, "VSP1_CH17", "cpri_rx2 dma request 23"),
	GCR_DESC(56, 0, 1, 0x0, "VSP1_CH16", "cpri_rx1 dma request 16"),
	GCR_DESC(56, 0, 1, 0x1, "VSP1_CH16", "cpri_rx1 dma request 24"),
	GCR_DESC(56, 0, 1, 0x2, "VSP1_CH16", "cpri_rx2 dma request 16"),
	GCR_DESC(56, 0, 1, 0x3, "VSP1_CH16", "cpri_rx2 dma request 24"),
	GCR_DESC(57, 30, 31, 0x0, "VSP2_CH31", "cpri_rx1 dma request 1"),
	GCR_DESC(57, 30, 31, 0x1, "VSP2_CH31", "cpri_rx1 dma request 17"),
	GCR_DESC(57, 30, 31, 0x2, "VSP2_CH31", "cpri_rx2 dma request 1"),
	GCR_DESC(57, 30, 31, 0x3, "VSP2_CH31", "cpri_rx2 dma request 17"),
	GCR_DESC(57, 28, 29, 0x0, "VSP2_CH30", "cpri_rx1 dma request 2"),
	GCR_DESC(57, 28, 29, 0x1, "VSP2_CH30", "cpri_rx1 dma request 18"),
	GCR_DESC(57, 28, 29, 0x2, "VSP2_CH30", "cpri_rx2 dma request 2"),
	GCR_DESC(57, 28, 29, 0x3, "VSP2_CH30", "cpri_rx2 dma request 18"),
	GCR_DESC(57, 26, 27, 0x0, "VSP2_CH29", "cpri_rx1 dma request 3"),
	GCR_DESC(57, 26, 27, 0x1, "VSP2_CH29", "cpri_rx1 dma request 19"),
	GCR_DESC(57, 26, 27, 0x2, "VSP2_CH29", "cpri_rx2 dma request 3"),
	GCR_DESC(57, 26, 27, 0x3, "VSP2_CH29", "cpri_rx2 dma request 19"),
	GCR_DESC(57, 24, 25, 0x0, "VSP2_CH28", "cpri_rx1 dma request 4"),
	GCR_DESC(57, 24, 25, 0x1, "VSP2_CH28", "cpri_rx1 dma request 20"),
	GCR_DESC(57, 24, 25, 0x2, "VSP2_CH28", "cpri_rx2 dma request 4"),
	GCR_DESC(57, 24, 25, 0x3, "VSP2_CH28", "cpri_rx2 dma request 20"),
	GCR_DESC(57, 22, 23, 0x0, "VSP2_CH27", "cpri_rx1 dma request 5"),
	GCR_DESC(57, 22, 23, 0x1, "VSP2_CH27", "cpri_rx1 dma request 21"),
	GCR_DESC(57, 22, 23, 0x2, "VSP2_CH27", "cpri_rx2 dma request 5"),
	GCR_DESC(57, 22, 23, 0x3, "VSP2_CH27", "cpri_rx2 dma request 21"),
	GCR_DESC(57, 20, 21, 0x0, "VSP2_CH26", "cpri_rx1 dma request 6"),
	GCR_DESC(57, 20, 21, 0x1, "VSP2_CH26", "cpri_rx1 dma request 22"),
	GCR_DESC(57, 20, 21, 0x2, "VSP2_CH26", "cpri_rx2 dma request 6"),
	GCR_DESC(57, 20, 21, 0x3, "VSP2_CH26", "cpri_rx2 dma request 22"),
	GCR_DESC(57, 18, 19, 0x0, "VSP2_CH25", "cpri_rx1 dma request 7"),
	GCR_DESC(57, 18, 19, 0x1, "VSP2_CH25", "cpri_rx1 dma request 23"),
	GCR_DESC(57, 18, 19, 0x2, "VSP2_CH25", "cpri_rx2 dma request 7"),
	GCR_DESC(57, 18, 19, 0x3, "VSP2_CH25", "cpri_rx2 dma request 23"),
	GCR_DESC(57, 16, 17, 0x0, "VSP2_CH24", "cpri_rx1 dma request 8"),
	GCR_DESC(57, 16, 17, 0x1, "VSP2_CH24", "cpri_rx1 dma request 24"),
	GCR_DESC(57, 16, 17, 0x2, "VSP2_CH24", "cpri_rx2 dma request 8"),
	GCR_DESC(57, 16, 17, 0x3, "VSP2_CH24", "cpri_rx2 dma request 24"),
	GCR_DESC(57, 14, 15, 0x0, "VSP2_CH23", "cpri_rx1 dma request 9"),
	GCR_DESC(57, 14, 15, 0x1, "VSP2_CH23", "cpri_rx1 dma request 17"),
	GCR_DESC(57, 14, 15, 0x2, "VSP2_CH23", "cpri_rx2 dma request 9"),
	GCR_DESC(57, 14, 15, 0x3, "VSP2_CH23", "cpri_rx2 dma request 17"),
	GCR_DESC(57, 12, 13, 0x0, "VSP2_CH22", "cpri_rx1 dma request 10"),
	GCR_DESC(57, 12, 13, 0x1, "VSP2_CH22", "cpri_rx1 dma request 18"),
	GCR_DESC(57, 12, 13, 0x2, "VSP2_CH22", "cpri_rx2 dma request 10"),
	GCR_DESC(57, 12, 13, 0x3, "VSP2_CH22", "cpri_rx2 dma request 18"),
	GCR_DESC(57, 10, 11, 0x0, "VSP2_CH21", "cpri_rx1 dma request 11"),
	GCR_DESC(57, 10, 11, 0x1, "VSP2_CH21", "cpri_rx1 dma request 19"),
	GCR_DESC(57, 10, 11, 0x2, "VSP2_CH21", "cpri_rx2 dma request 11"),
	GCR_DESC(57, 10, 11, 0x3, "VSP2_CH21", "cpri_rx2 dma request 19"),
	GCR_DESC(57, 8, 9, 0x0, "VSP2_CH20", "cpri_rx1 dma request 12"),
	GCR_DESC(57, 8, 9, 0x1, "VSP2_CH20", "cpri_rx1 dma request 20"),
	GCR_DESC(57, 8, 9, 0x2, "VSP2_CH20", "cpri_rx2 dma request 12"),
	GCR_DESC(57, 8, 9, 0x3, "VSP2_CH20", "cpri_rx2 dma request 20"),
	GCR_DESC(57, 6, 7, 0x0, "VSP2_CH19", "cpri_rx1 dma request 13"),
	GCR_DESC(57, 6, 7, 0x1, "VSP2_CH19", "cpri_rx1 dma request 21"),
	GCR_DESC(57, 6, 7, 0x2, "VSP2_CH19", "cpri_rx2 dma request 13"),
	GCR_DESC(57, 6, 7, 0x3, "VSP2_CH19", "cpri_rx2 dma request 21"),
	GCR_DESC(57, 4, 5, 0x0, "VSP2_CH18", "cpri_rx1 dma request 14"),
	GCR_DESC(57, 4, 5, 0x1, "VSP2_CH18", "cpri_rx1 dma request 22"),
	GCR_DESC(57, 4, 5, 0x2, "VSP2_CH18", "cpri_rx2 dma request 14"),
	GCR_DESC(57, 4, 5, 0x3, "VSP2_CH18", "cpri_rx2 dma request 22"),
	GCR_DESC(57, 2, 3, 0x0, "VSP2_CH17", "cpri_rx1 dma request 15"),
	GCR_DESC(57, 2, 3, 0x1, "VSP2_CH17", "cpri_rx1 dma request 23"),
	GCR_DESC(57, 2, 3, 0x2, "VSP2_CH17", "cpri_rx2 dma request 15"),
	GCR_DESC(57, 2, 3, 0x3, "VSP2_CH17", "cpri_rx2 dma request 23"),
	GCR_DESC(57, 0, 1, 0x0, "VSP2_CH16", "cpri_rx1 dma request 16"),
	GCR_DESC(57, 0, 1, 0x1, "VSP2_CH16", "cpri_rx1 dma request 24"),
	GCR_DESC(57, 0, 1, 0x2, "VSP2_CH16", "cpri_rx2 dma request 16"),
	GCR_DESC(57, 0, 1, 0x3, "VSP2_CH16", "cpri_rx2 dma request 24"),
	GCR_DESC(58, 30, 31, 0x0, "VSP3_CH31", "cpri_rx1 dma request 1"),
	GCR_DESC(58, 30, 31, 0x1, "VSP3_CH31", "cpri_rx1 dma request 17"),
	GCR_DESC(58, 30, 31, 0x2, "VSP3_CH31", "cpri_rx2 dma request 1"),
	GCR_DESC(58, 30, 31, 0x3, "VSP3_CH31", "cpri_rx2 dma request 17"),
	GCR_DESC(58, 28, 29, 0x0, "VSP3_CH30", "cpri_rx1 dma request 2"),
	GCR_DESC(58, 28, 29, 0x1, "VSP3_CH30", "cpri_rx1 dma request 18"),
	GCR_DESC(58, 28, 29, 0x2, "VSP3_CH30", "cpri_rx2 dma request 2"),
	GCR_DESC(58, 28, 29, 0x3, "VSP3_CH30", "cpri_rx2 dma request 18"),
	GCR_DESC(58, 26, 27, 0x0, "VSP3_CH29", "cpri_rx1 dma request 3"),
	GCR_DESC(58, 26, 27, 0x1, "VSP3_CH29", "cpri_rx1 dma request 19"),
	GCR_DESC(58, 26, 27, 0x2, "VSP3_CH29", "cpri_rx2 dma request 3"),
	GCR_DESC(58, 26, 27, 0x3, "VSP3_CH29", "cpri_rx2 dma request 19"),
	GCR_DESC(58, 24, 25, 0x0, "VSP3_CH28", "cpri_rx1 dma request 4"),
	GCR_DESC(58, 24, 25, 0x1, "VSP3_CH28", "cpri_rx1 dma request 20"),
	GCR_DESC(58, 24, 25, 0x2, "VSP3_CH28", "cpri_rx2 dma request 4"),
	GCR_DESC(58, 24, 25, 0x3, "VSP3_CH28", "cpri_rx2 dma request 20"),
	GCR_DESC(58, 22, 23, 0x0, "VSP3_CH27", "cpri_rx1 dma request 5"),
	GCR_DESC(58, 22, 23, 0x1, "VSP3_CH27", "cpri_rx1 dma request 21"),
	GCR_DESC(58, 22, 23, 0x2, "VSP3_CH27", "cpri_rx2 dma request 5"),
	GCR_DESC(58, 22, 23, 0x3, "VSP3_CH27", "cpri_rx2 dma request 21"),
	GCR_DESC(58, 20, 21, 0x0, "VSP3_CH26", "cpri_rx1 dma request 6"),
	GCR_DESC(58, 20, 21, 0x1, "VSP3_CH26", "cpri_rx1 dma request 22"),
	GCR_DESC(58, 20, 21, 0x2, "VSP3_CH26", "cpri_rx2 dma request 6"),
	GCR_DESC(58, 20, 21, 0x3, "VSP3_CH26", "cpri_rx2 dma request 22"),
	GCR_DESC(58, 18, 19, 0x0, "VSP3_CH25", "cpri_rx1 dma request 7"),
	GCR_DESC(58, 18, 19, 0x1, "VSP3_CH25", "cpri_rx1 dma request 23"),
	GCR_DESC(58, 18, 19, 0x2, "VSP3_CH25", "cpri_rx2 dma request 7"),
	GCR_DESC(58, 18, 19, 0x3, "VSP3_CH25", "cpri_rx2 dma request 23"),
	GCR_DESC(58, 16, 17, 0x0, "VSP3_CH24", "cpri_rx1 dma request 8"),
	GCR_DESC(58, 16, 17, 0x1, "VSP3_CH24", "cpri_rx1 dma request 24"),
	GCR_DESC(58, 16, 17, 0x2, "VSP3_CH24", "cpri_rx2 dma request 8"),
	GCR_DESC(58, 16, 17, 0x3, "VSP3_CH24", "cpri_rx2 dma request 24"),
	GCR_DESC(58, 14, 15, 0x0, "VSP3_CH23", "cpri_rx1 dma request 9"),
	GCR_DESC(58, 14, 15, 0x1, "VSP3_CH23", "cpri_rx1 dma request 17"),
	GCR_DESC(58, 14, 15, 0x2, "VSP3_CH23", "cpri_rx2 dma request 9"),
	GCR_DESC(58, 14, 15, 0x3, "VSP3_CH23", "cpri_rx2 dma request 17"),
	GCR_DESC(58, 12, 13, 0x0, "VSP3_CH22", "cpri_rx1 dma request 10"),
	GCR_DESC(58, 12, 13, 0x1, "VSP3_CH22", "cpri_rx1 dma request 18"),
	GCR_DESC(58, 12, 13, 0x2, "VSP3_CH22", "cpri_rx2 dma request 10"),
	GCR_DESC(58, 12, 13, 0x3, "VSP3_CH22", "cpri_rx2 dma request 18"),
	GCR_DESC(58, 10, 11, 0x0, "VSP3_CH21", "cpri_rx1 dma request 11"),
	GCR_DESC(58, 10, 11, 0x1, "VSP3_CH21", "cpri_rx1 dma request 19"),
	GCR_DESC(58, 10, 11, 0x2, "VSP3_CH21", "cpri_rx2 dma request 11"),
	GCR_DESC(58, 10, 11, 0x3, "VSP3_CH21", "cpri_rx2 dma request 19"),
	GCR_DESC(58, 8, 9, 0x0, "VSP3_CH20", "cpri_rx1 dma request 12"),
	GCR_DESC(58, 8, 9, 0x1, "VSP3_CH20", "cpri_rx1 dma request 20"),
	GCR_DESC(58, 8, 9, 0x2, "VSP3_CH20", "cpri_rx2 dma request 12"),
	GCR_DESC(58, 8, 9, 0x3, "VSP3_CH20", "cpri_rx2 dma request 20"),
	GCR_DESC(58, 6, 7, 0x0, "VSP3_CH19", "cpri_rx1 dma request 13"),
	GCR_DESC(58, 6, 7, 0x1, "VSP3_CH19", "cpri_rx1 dma request 21"),
	GCR_DESC(58, 6, 7, 0x2, "VSP3_CH19", "cpri_rx2 dma request 13"),
	GCR_DESC(58, 6, 7, 0x3, "VSP3_CH19", "cpri_rx2 dma request 21"),
	GCR_DESC(58, 4, 5, 0x0, "VSP3_CH18", "cpri_rx1 dma request 14"),
	GCR_DESC(58, 4, 5, 0x1, "VSP3_CH18", "cpri_rx1 dma request 22"),
	GCR_DESC(58, 4, 5, 0x2, "VSP3_CH18", "cpri_rx2 dma request 14"),
	GCR_DESC(58, 4, 5, 0x3, "VSP3_CH18", "cpri_rx2 dma request 22"),
	GCR_DESC(58, 2, 3, 0x0, "VSP3_CH17", "cpri_rx1 dma request 15"),
	GCR_DESC(58, 2, 3, 0x1, "VSP3_CH17", "cpri_rx1 dma request 23"),
	GCR_DESC(58, 2, 3, 0x2, "VSP3_CH17", "cpri_rx2 dma request 15"),
	GCR_DESC(58, 2, 3, 0x3, "VSP3_CH17", "cpri_rx2 dma request 23"),
	GCR_DESC(58, 0, 1, 0x0, "VSP3_CH16", "cpri_rx1 dma request 16"),
	GCR_DESC(58, 0, 1, 0x1, "VSP3_CH16", "cpri_rx1 dma request 24"),
	GCR_DESC(58, 0, 1, 0x2, "VSP3_CH16", "cpri_rx2 dma request 16"),
	GCR_DESC(58, 0, 1, 0x3, "VSP3_CH16", "cpri_rx2 dma request 24"),
	GCR_DESC(59, 30, 31, 0x0, "VSP4_CH31", "cpri_rx1 dma request 1"),
	GCR_DESC(59, 30, 31, 0x1, "VSP4_CH31", "cpri_rx1 dma request 17"),
	GCR_DESC(59, 30, 31, 0x2, "VSP4_CH31", "cpri_rx2 dma request 1"),
	GCR_DESC(59, 30, 31, 0x3, "VSP4_CH31", "cpri_rx2 dma request 17"),
	GCR_DESC(59, 28, 29, 0x0, "VSP4_CH30", "cpri_rx1 dma request 2"),
	GCR_DESC(59, 28, 29, 0x1, "VSP4_CH30", "cpri_rx1 dma request 18"),
	GCR_DESC(59, 28, 29, 0x2, "VSP4_CH30", "cpri_rx2 dma request 2"),
	GCR_DESC(59, 28, 29, 0x3, "VSP4_CH30", "cpri_rx2 dma request 18"),
	GCR_DESC(59, 26, 27, 0x0, "VSP4_CH29", "cpri_rx1 dma request 3"),
	GCR_DESC(59, 26, 27, 0x1, "VSP4_CH29", "cpri_rx1 dma request 19"),
	GCR_DESC(59, 26, 27, 0x2, "VSP4_CH29", "cpri_rx2 dma request 3"),
	GCR_DESC(59, 26, 27, 0x3, "VSP4_CH29", "cpri_rx2 dma request 19"),
	GCR_DESC(59, 24, 25, 0x0, "VSP4_CH28", "cpri_rx1 dma request 4"),
	GCR_DESC(59, 24, 25, 0x1, "VSP4_CH28", "cpri_rx1 dma request 20"),
	GCR_DESC(59, 24, 25, 0x2, "VSP4_CH28", "cpri_rx2 dma request 4"),
	GCR_DESC(59, 24, 25, 0x3, "VSP4_CH28", "cpri_rx2 dma request 20"),
	GCR_DESC(59, 22, 23, 0x0, "VSP4_CH27", "cpri_rx1 dma request 5"),
	GCR_DESC(59, 22, 23, 0x1, "VSP4_CH27", "cpri_rx1 dma request 21"),
	GCR_DESC(59, 22, 23, 0x2, "VSP4_CH27", "cpri_rx2 dma request 5"),
	GCR_DESC(59, 22, 23, 0x3, "VSP4_CH27", "cpri_rx2 dma request 21"),
	GCR_DESC(59, 20, 21, 0x0, "VSP4_CH26", "cpri_rx1 dma request 6"),
	GCR_DESC(59, 20, 21, 0x1, "VSP4_CH26", "cpri_rx1 dma request 22"),
	GCR_DESC(59, 20, 21, 0x2, "VSP4_CH26", "cpri_rx2 dma request 6"),
	GCR_DESC(59, 20, 21, 0x3, "VSP4_CH26", "cpri_rx2 dma request 22"),
	GCR_DESC(59, 18, 19, 0x0, "VSP4_CH25", "cpri_rx1 dma request 7"),
	GCR_DESC(59, 18, 19, 0x1, "VSP4_CH25", "cpri_rx1 dma request 23"),
	GCR_DESC(59, 18, 19, 0x2, "VSP4_CH25", "cpri_rx2 dma request 7"),
	GCR_DESC(59, 18, 19, 0x3, "VSP4_CH25", "cpri_rx2 dma request 23"),
	GCR_DESC(59, 16, 17, 0x0, "VSP4_CH24", "cpri_rx1 dma request 8"),
	GCR_DESC(59, 16, 17, 0x1, "VSP4_CH24", "cpri_rx1 dma request 24"),
	GCR_DESC(59, 16, 17, 0x2, "VSP4_CH24", "cpri_rx2 dma request 8"),
	GCR_DESC(59, 16, 17, 0x3, "VSP4_CH24", "cpri_rx2 dma request 24"),
	GCR_DESC(59, 14, 15, 0x0, "VSP4_CH23", "cpri_rx1 dma request 9"),
	GCR_DESC(59, 14, 15, 0x1, "VSP4_CH23", "cpri_rx1 dma request 17"),
	GCR_DESC(59, 14, 15, 0x2, "VSP4_CH23", "cpri_rx2 dma request 9"),
	GCR_DESC(59, 14, 15, 0x3, "VSP4_CH23", "cpri_rx2 dma request 17"),
	GCR_DESC(59, 12, 13, 0x0, "VSP4_CH22", "cpri_rx1 dma request 10"),
	GCR_DESC(59, 12, 13, 0x1, "VSP4_CH22", "cpri_rx1 dma request 18"),
	GCR_DESC(59, 12, 13, 0x2, "VSP4_CH22", "cpri_rx2 dma request 10"),
	GCR_DESC(59, 12, 13, 0x3, "VSP4_CH22", "cpri_rx2 dma request 18"),
	GCR_DESC(59, 10, 11, 0x0, "VSP4_CH21", "cpri_rx1 dma request 11"),
	GCR_DESC(59, 10, 11, 0x1, "VSP4_CH21", "cpri_rx1 dma request 19"),
	GCR_DESC(59, 10, 11, 0x2, "VSP4_CH21", "cpri_rx2 dma request 11"),
	GCR_DESC(59, 10, 11, 0x3, "VSP4_CH21", "cpri_rx2 dma request 19"),
	GCR_DESC(59, 8, 9, 0x0, "VSP4_CH20", "cpri_rx1 dma request 12"),
	GCR_DESC(59, 8, 9, 0x1, "VSP4_CH20", "cpri_rx1 dma request 20"),
	GCR_DESC(59, 8, 9, 0x2, "VSP4_CH20", "cpri_rx2 dma request 12"),
	GCR_DESC(59, 8, 9, 0x3, "VSP4_CH20", "cpri_rx2 dma request 20"),
	GCR_DESC(59, 6, 7, 0x0, "VSP4_CH19", "cpri_rx1 dma request 13"),
	GCR_DESC(59, 6, 7, 0x1, "VSP4_CH19", "cpri_rx1 dma request 21"),
	GCR_DESC(59, 6, 7, 0x2, "VSP4_CH19", "cpri_rx2 dma request 13"),
	GCR_DESC(59, 6, 7, 0x3, "VSP4_CH19", "cpri_rx2 dma request 21"),
	GCR_DESC(59, 4, 5, 0x0, "VSP4_CH18", "cpri_rx1 dma request 14"),
	GCR_DESC(59, 4, 5, 0x1, "VSP4_CH18", "cpri_rx1 dma request 22"),
	GCR_DESC(59, 4, 5, 0x2, "VSP4_CH18", "cpri_rx2 dma request 14"),
	GCR_DESC(59, 4, 5, 0x3, "VSP4_CH18", "cpri_rx2 dma request 22"),
	GCR_DESC(59, 2, 3, 0x0, "VSP4_CH17", "cpri_rx1 dma request 15"),
	GCR_DESC(59, 2, 3, 0x1, "VSP4_CH17", "cpri_rx1 dma request 23"),
	GCR_DESC(59, 2, 3, 0x2, "VSP4_CH17", "cpri_rx2 dma request 15"),
	GCR_DESC(59, 2, 3, 0x3, "VSP4_CH17", "cpri_rx2 dma request 23"),
	GCR_DESC(59, 0, 1, 0x0, "VSP4_CH16", "cpri_rx1 dma request 16"),
	GCR_DESC(59, 0, 1, 0x1, "VSP4_CH16", "cpri_rx1 dma request 24"),
	GCR_DESC(59, 0, 1, 0x2, "VSP4_CH16", "cpri_rx2 dma request 16"),
	GCR_DESC(59, 0, 1, 0x3, "VSP4_CH16", "cpri_rx2 dma request 24"),
	GCR_DESC(60, 30, 31, 0x0, "VSP8_CH31", "cpri_rx1 dma request 1"),
	GCR_DESC(60, 30, 31, 0x1, "VSP8_CH31", "cpri_rx1 dma request 17"),
	GCR_DESC(60, 30, 31, 0x2, "VSP8_CH31", "cpri_rx2 dma request 1"),
	GCR_DESC(60, 30, 31, 0x3, "VSP8_CH31", "cpri_rx2 dma request 17"),
	GCR_DESC(60, 28, 29, 0x0, "VSP8_CH30", "cpri_rx1 dma request 2"),
	GCR_DESC(60, 28, 29, 0x1, "VSP8_CH30", "cpri_rx1 dma request 18"),
	GCR_DESC(60, 28, 29, 0x2, "VSP8_CH30", "cpri_rx2 dma request 2"),
	GCR_DESC(60, 28, 29, 0x3, "VSP8_CH30", "cpri_rx2 dma request 18"),
	GCR_DESC(60, 26, 27, 0x0, "VSP8_CH29", "cpri_rx1 dma request 3"),
	GCR_DESC(60, 26, 27, 0x1, "VSP8_CH29", "cpri_rx1 dma request 19"),
	GCR_DESC(60, 26, 27, 0x2, "VSP8_CH29", "cpri_rx2 dma request 3"),
	GCR_DESC(60, 26, 27, 0x3, "VSP8_CH29", "cpri_rx2 dma request 19"),
	GCR_DESC(60, 24, 25, 0x0, "VSP8_CH28", "cpri_rx1 dma request 4"),
	GCR_DESC(60, 24, 25, 0x1, "VSP8_CH28", "cpri_rx1 dma request 20"),
	GCR_DESC(60, 24, 25, 0x2, "VSP8_CH28", "cpri_rx2 dma request 4"),
	GCR_DESC(60, 24, 25, 0x3, "VSP8_CH28", "cpri_rx2 dma request 20"),
	GCR_DESC(60, 22, 23, 0x0, "VSP8_CH27", "cpri_rx1 dma request 5"),
	GCR_DESC(60, 22, 23, 0x1, "VSP8_CH27", "cpri_rx1 dma request 21"),
	GCR_DESC(60, 22, 23, 0x2, "VSP8_CH27", "cpri_rx2 dma request 5"),
	GCR_DESC(60, 22, 23, 0x3, "VSP8_CH27", "cpri_rx2 dma request 21"),
	GCR_DESC(60, 20, 21, 0x0, "VSP8_CH26", "cpri_rx1 dma request 6"),
	GCR_DESC(60, 20, 21, 0x1, "VSP8_CH26", "cpri_rx1 dma request 22"),
	GCR_DESC(60, 20, 21, 0x2, "VSP8_CH26", "cpri_rx2 dma request 6"),
	GCR_DESC(60, 20, 21, 0x3, "VSP8_CH26", "cpri_rx2 dma request 22"),
	GCR_DESC(60, 18, 19, 0x0, "VSP8_CH25", "cpri_rx1 dma request 7"),
	GCR_DESC(60, 18, 19, 0x1, "VSP8_CH25", "cpri_rx1 dma request 23"),
	GCR_DESC(60, 18, 19, 0x2, "VSP8_CH25", "cpri_rx2 dma request 7"),
	GCR_DESC(60, 18, 19, 0x3, "VSP8_CH25", "cpri_rx2 dma request 23"),
	GCR_DESC(60, 16, 17, 0x0, "VSP8_CH24", "cpri_rx1 dma request 8"),
	GCR_DESC(60, 16, 17, 0x1, "VSP8_CH24", "cpri_rx1 dma request 24"),
	GCR_DESC(60, 16, 17, 0x2, "VSP8_CH24", "cpri_rx2 dma request 8"),
	GCR_DESC(60, 16, 17, 0x3, "VSP8_CH24", "cpri_rx2 dma request 24"),
	GCR_DESC(60, 14, 15, 0x0, "VSP8_CH23", "cpri_rx1 dma request 9"),
	GCR_DESC(60, 14, 15, 0x1, "VSP8_CH23", "cpri_rx1 dma request 17"),
	GCR_DESC(60, 14, 15, 0x2, "VSP8_CH23", "cpri_rx2 dma request 9"),
	GCR_DESC(60, 14, 15, 0x3, "VSP8_CH23", "cpri_rx2 dma request 17"),
	GCR_DESC(60, 12, 13, 0x0, "VSP8_CH22", "cpri_rx1 dma request 10"),
	GCR_DESC(60, 12, 13, 0x1, "VSP8_CH22", "cpri_rx1 dma request 18"),
	GCR_DESC(60, 12, 13, 0x2, "VSP8_CH22", "cpri_rx2 dma request 10"),
	GCR_DESC(60, 12, 13, 0x3, "VSP8_CH22", "cpri_rx2 dma request 18"),
	GCR_DESC(60, 10, 11, 0x0, "VSP8_CH21", "cpri_rx1 dma request 11"),
	GCR_DESC(60, 10, 11, 0x1, "VSP8_CH21", "cpri_rx1 dma request 19"),
	GCR_DESC(60, 10, 11, 0x2, "VSP8_CH21", "cpri_rx2 dma request 11"),
	GCR_DESC(60, 10, 11, 0x3, "VSP8_CH21", "cpri_rx2 dma request 19"),
	GCR_DESC(60, 8, 9, 0x0, "VSP8_CH20", "cpri_rx1 dma request 12"),
	GCR_DESC(60, 8, 9, 0x1, "VSP8_CH20", "cpri_rx1 dma request 20"),
	GCR_DESC(60, 8, 9, 0x2, "VSP8_CH20", "cpri_rx2 dma request 12"),
	GCR_DESC(60, 8, 9, 0x3, "VSP8_CH20", "cpri_rx2 dma request 20"),
	GCR_DESC(60, 6, 7, 0x0, "VSP8_CH19", "cpri_rx1 dma request 13"),
	GCR_DESC(60, 6, 7, 0x1, "VSP8_CH19", "cpri_rx1 dma request 21"),
	GCR_DESC(60, 6, 7, 0x2, "VSP8_CH19", "cpri_rx2 dma request 13"),
	GCR_DESC(60, 6, 7, 0x3, "VSP8_CH19", "cpri_rx2 dma request 21"),
	GCR_DESC(60, 4, 5, 0x0, "VSP8_CH18", "cpri_rx1 dma request 14"),
	GCR_DESC(60, 4, 5, 0x1, "VSP8_CH18", "cpri_rx1 dma request 22"),
	GCR_DESC(60, 4, 5, 0x2, "VSP8_CH18", "cpri_rx2 dma request 14"),
	GCR_DESC(60, 4, 5, 0x3, "VSP8_CH18", "cpri_rx2 dma request 22"),
	GCR_DESC(60, 2, 3, 0x0, "VSP8_CH17", "cpri_rx1 dma request 15"),
	GCR_DESC(60, 2, 3, 0x1, "VSP8_CH17", "cpri_rx1 dma request 23"),
	GCR_DESC(60, 2, 3, 0x2, "VSP8_CH17", "cpri_rx2 dma request 15"),
	GCR_DESC(60, 2, 3, 0x3, "VSP8_CH17", "cpri_rx2 dma request 23"),
	GCR_DESC(60, 0, 1, 0x0, "VSP8_CH16", "cpri_rx1 dma request 16"),
	GCR_DESC(60, 0, 1, 0x1, "VSP8_CH16", "cpri_rx1 dma request 24"),
	GCR_DESC(60, 0, 1, 0x2, "VSP8_CH16", "cpri_rx2 dma request 16"),
	GCR_DESC(60, 0, 1, 0x3, "VSP8_CH16", "cpri_rx2 dma request 24"),
	GCR_DESC(62, 30, 31, 0x0, "VSP9_CH31", "cpri_rx1 dma request 1"),
	GCR_DESC(62, 30, 31, 0x1, "VSP9_CH31", "cpri_rx1 dma request 17"),
	GCR_DESC(62, 30, 31, 0x2, "VSP9_CH31", "cpri_rx2 dma request 1"),
	GCR_DESC(62, 30, 31, 0x3, "VSP9_CH31", "cpri_rx2 dma request 17"),
	GCR_DESC(62, 28, 29, 0x0, "VSP9_CH30", "cpri_rx1 dma request 2"),
	GCR_DESC(62, 28, 29, 0x1, "VSP9_CH30", "cpri_rx1 dma request 18"),
	GCR_DESC(62, 28, 29, 0x2, "VSP9_CH30", "cpri_rx2 dma request 2"),
	GCR_DESC(62, 28, 29, 0x3, "VSP9_CH30", "cpri_rx2 dma request 18"),
	GCR_DESC(62, 26, 27, 0x0, "VSP9_CH29", "cpri_rx1 dma request 3"),
	GCR_DESC(62, 26, 27, 0x1, "VSP9_CH29", "cpri_rx1 dma request 19"),
	GCR_DESC(62, 26, 27, 0x2, "VSP9_CH29", "cpri_rx2 dma request 3"),
	GCR_DESC(62, 26, 27, 0x3, "VSP9_CH29", "cpri_rx2 dma request 19"),
	GCR_DESC(62, 24, 25, 0x0, "VSP9_CH28", "cpri_rx1 dma request 4"),
	GCR_DESC(62, 24, 25, 0x1, "VSP9_CH28", "cpri_rx1 dma request 20"),
	GCR_DESC(62, 24, 25, 0x2, "VSP9_CH28", "cpri_rx2 dma request 4"),
	GCR_DESC(62, 24, 25, 0x3, "VSP9_CH28", "cpri_rx2 dma request 20"),
	GCR_DESC(62, 22, 23, 0x0, "VSP9_CH27", "cpri_rx1 dma request 5"),
	GCR_DESC(62, 22, 23, 0x1, "VSP9_CH27", "cpri_rx1 dma request 21"),
	GCR_DESC(62, 22, 23, 0x2, "VSP9_CH27", "cpri_rx2 dma request 5"),
	GCR_DESC(62, 22, 23, 0x3, "VSP9_CH27", "cpri_rx2 dma request 21"),
	GCR_DESC(62, 20, 21, 0x0, "VSP9_CH26", "cpri_rx1 dma request 6"),
	GCR_DESC(62, 20, 21, 0x1, "VSP9_CH26", "cpri_rx1 dma request 22"),
	GCR_DESC(62, 20, 21, 0x2, "VSP9_CH26", "cpri_rx2 dma request 6"),
	GCR_DESC(62, 20, 21, 0x3, "VSP9_CH26", "cpri_rx2 dma request 22"),
	GCR_DESC(62, 18, 19, 0x0, "VSP9_CH25", "cpri_rx1 dma request 7"),
	GCR_DESC(62, 18, 19, 0x1, "VSP9_CH25", "cpri_rx1 dma request 23"),
	GCR_DESC(62, 18, 19, 0x2, "VSP9_CH25", "cpri_rx2 dma request 7"),
	GCR_DESC(62, 18, 19, 0x3, "VSP9_CH25", "cpri_rx2 dma request 23"),
	GCR_DESC(62, 16, 17, 0x0, "VSP9_CH24", "cpri_rx1 dma request 8"),
	GCR_DESC(62, 16, 17, 0x1, "VSP9_CH24", "cpri_rx1 dma request 24"),
	GCR_DESC(62, 16, 17, 0x2, "VSP9_CH24", "cpri_rx2 dma request 8"),
	GCR_DESC(62, 16, 17, 0x3, "VSP9_CH24", "cpri_rx2 dma request 24"),
	GCR_DESC(62, 14, 15, 0x0, "VSP9_CH23", "cpri_rx1 dma request 9"),
	GCR_DESC(62, 14, 15, 0x1, "VSP9_CH23", "cpri_rx1 dma request 17"),
	GCR_DESC(62, 14, 15, 0x2, "VSP9_CH23", "cpri_rx2 dma request 9"),
	GCR_DESC(62, 14, 15, 0x3, "VSP9_CH23", "cpri_rx2 dma request 17"),
	GCR_DESC(62, 12, 13, 0x0, "VSP9_CH22", "cpri_rx1 dma request 10"),
	GCR_DESC(62, 12, 13, 0x1, "VSP9_CH22", "cpri_rx1 dma request 18"),
	GCR_DESC(62, 12, 13, 0x2, "VSP9_CH22", "cpri_rx2 dma request 10"),
	GCR_DESC(62, 12, 13, 0x3, "VSP9_CH22", "cpri_rx2 dma request 18"),
	GCR_DESC(62, 10, 11, 0x0, "VSP9_CH21", "cpri_rx1 dma request 11"),
	GCR_DESC(62, 10, 11, 0x1, "VSP9_CH21", "cpri_rx1 dma request 19"),
	GCR_DESC(62, 10, 11, 0x2, "VSP9_CH21", "cpri_rx2 dma request 11"),
	GCR_DESC(62, 10, 11, 0x3, "VSP9_CH21", "cpri_rx2 dma request 19"),
	GCR_DESC(62, 8, 9, 0x0, "VSP9_CH20", "cpri_rx1 dma request 12"),
	GCR_DESC(62, 8, 9, 0x1, "VSP9_CH20", "cpri_rx1 dma request 20"),
	GCR_DESC(62, 8, 9, 0x2, "VSP9_CH20", "cpri_rx2 dma request 12"),
	GCR_DESC(62, 8, 9, 0x3, "VSP9_CH20", "cpri_rx2 dma request 20"),
	GCR_DESC(62, 6, 7, 0x0, "VSP9_CH19", "cpri_rx1 dma request 13"),
	GCR_DESC(62, 6, 7, 0x1, "VSP9_CH19", "cpri_rx1 dma request 21"),
	GCR_DESC(62, 6, 7, 0x2, "VSP9_CH19", "cpri_rx2 dma request 13"),
	GCR_DESC(62, 6, 7, 0x3, "VSP9_CH19", "cpri_rx2 dma request 21"),
	GCR_DESC(62, 4, 5, 0x0, "VSP9_CH18", "cpri_rx1 dma request 14"),
	GCR_DESC(62, 4, 5, 0x1, "VSP9_CH18", "cpri_rx1 dma request 22"),
	GCR_DESC(62, 4, 5, 0x2, "VSP9_CH18", "cpri_rx2 dma request 14"),
	GCR_DESC(62, 4, 5, 0x3, "VSP9_CH18", "cpri_rx2 dma request 22"),
	GCR_DESC(62, 2, 3, 0x0, "VSP9_CH17", "cpri_rx1 dma request 15"),
	GCR_DESC(62, 2, 3, 0x1, "VSP9_CH17", "cpri_rx1 dma request 23"),
	GCR_DESC(62, 2, 3, 0x2, "VSP9_CH17", "cpri_rx2 dma request 15"),
	GCR_DESC(62, 2, 3, 0x3, "VSP9_CH17", "cpri_rx2 dma request 23"),
	GCR_DESC(62, 0, 1, 0x0, "VSP9_CH16", "cpri_rx1 dma request 16"),
	GCR_DESC(62, 0, 1, 0x1, "VSP9_CH16", "cpri_rx1 dma request 24"),
	GCR_DESC(62, 0, 1, 0x2, "VSP9_CH16", "cpri_rx2 dma request 16"),
	GCR_DESC(62, 0, 1, 0x3, "VSP9_CH16", "cpri_rx2 dma request 24"),
	GCR_DESC(63, 30, 31, 0x0, "VSP11_CH31", "cpri_rx1 dma request 1"),
	GCR_DESC(63, 30, 31, 0x1, "VSP11_CH31", "cpri_rx1 dma request 17"),
	GCR_DESC(63, 30, 31, 0x2, "VSP11_CH31", "cpri_rx2 dma request 1"),
	GCR_DESC(63, 30, 31, 0x3, "VSP11_CH31", "cpri_rx2 dma request 17"),
	GCR_DESC(63, 28, 29, 0x0, "VSP11_CH30", "cpri_rx1 dma request 2"),
	GCR_DESC(63, 28, 29, 0x1, "VSP11_CH30", "cpri_rx1 dma request 18"),
	GCR_DESC(63, 28, 29, 0x2, "VSP11_CH30", "cpri_rx2 dma request 2"),
	GCR_DESC(63, 28, 29, 0x3, "VSP11_CH30", "cpri_rx2 dma request 18"),
	GCR_DESC(63, 26, 27, 0x0, "VSP11_CH29", "cpri_rx1 dma request 3"),
	GCR_DESC(63, 26, 27, 0x1, "VSP11_CH29", "cpri_rx1 dma request 19"),
	GCR_DESC(63, 26, 27, 0x2, "VSP11_CH29", "cpri_rx2 dma request 3"),
	GCR_DESC(63, 26, 27, 0x3, "VSP11_CH29", "cpri_rx2 dma request 19"),
	GCR_DESC(63, 24, 25, 0x0, "VSP11_CH28", "cpri_rx1 dma request 4"),
	GCR_DESC(63, 24, 25, 0x1, "VSP11_CH28", "cpri_rx1 dma request 20"),
	GCR_DESC(63, 24, 25, 0x2, "VSP11_CH28", "cpri_rx2 dma request 4"),
	GCR_DESC(63, 24, 25, 0x3, "VSP11_CH28", "cpri_rx2 dma request 20"),
	GCR_DESC(63, 22, 23, 0x0, "VSP11_CH27", "cpri_rx1 dma request 5"),
	GCR_DESC(63, 22, 23, 0x1, "VSP11_CH27", "cpri_rx1 dma request 21"),
	GCR_DESC(63, 22, 23, 0x2, "VSP11_CH27", "cpri_rx2 dma request 5"),
	GCR_DESC(63, 22, 23, 0x3, "VSP11_CH27", "cpri_rx2 dma request 21"),
	GCR_DESC(63, 20, 21, 0x0, "VSP11_CH26", "cpri_rx1 dma request 6"),
	GCR_DESC(63, 20, 21, 0x1, "VSP11_CH26", "cpri_rx1 dma request 22"),
	GCR_DESC(63, 20, 21, 0x2, "VSP11_CH26", "cpri_rx2 dma request 6"),
	GCR_DESC(63, 20, 21, 0x3, "VSP11_CH26", "cpri_rx2 dma request 22"),
	GCR_DESC(63, 18, 19, 0x0, "VSP11_CH25", "cpri_rx1 dma request 7"),
	GCR_DESC(63, 18, 19, 0x1, "VSP11_CH25", "cpri_rx1 dma request 23"),
	GCR_DESC(63, 18, 19, 0x2, "VSP11_CH25", "cpri_rx2 dma request 7"),
	GCR_DESC(63, 18, 19, 0x3, "VSP11_CH25", "cpri_rx2 dma request 23"),
	GCR_DESC(63, 16, 17, 0x0, "VSP11_CH24", "cpri_rx1 dma request 8"),
	GCR_DESC(63, 16, 17, 0x1, "VSP11_CH24", "cpri_rx1 dma request 24"),
	GCR_DESC(63, 16, 17, 0x2, "VSP11_CH24", "cpri_rx2 dma request 8"),
	GCR_DESC(63, 16, 17, 0x3, "VSP11_CH24", "cpri_rx2 dma request 24"),
	GCR_DESC(63, 14, 15, 0x0, "VSP11_CH23", "cpri_rx1 dma request 9"),
	GCR_DESC(63, 14, 15, 0x1, "VSP11_CH23", "cpri_rx1 dma request 17"),
	GCR_DESC(63, 14, 15, 0x2, "VSP11_CH23", "cpri_rx2 dma request 9"),
	GCR_DESC(63, 14, 15, 0x3, "VSP11_CH23", "cpri_rx2 dma request 17"),
	GCR_DESC(63, 12, 13, 0x0, "VSP11_CH22", "cpri_rx1 dma request 10"),
	GCR_DESC(63, 12, 13, 0x1, "VSP11_CH22", "cpri_rx1 dma request 18"),
	GCR_DESC(63, 12, 13, 0x2, "VSP11_CH22", "cpri_rx2 dma request 10"),
	GCR_DESC(63, 12, 13, 0x3, "VSP11_CH22", "cpri_rx2 dma request 18"),
	GCR_DESC(63, 10, 11, 0x0, "VSP11_CH21", "cpri_rx1 dma request 11"),
	GCR_DESC(63, 10, 11, 0x1, "VSP11_CH21", "cpri_rx1 dma request 19"),
	GCR_DESC(63, 10, 11, 0x2, "VSP11_CH21", "cpri_rx2 dma request 11"),
	GCR_DESC(63, 10, 11, 0x3, "VSP11_CH21", "cpri_rx2 dma request 19"),
	GCR_DESC(63, 8, 9, 0x0, "VSP11_CH20", "cpri_rx1 dma request 12"),
	GCR_DESC(63, 8, 9, 0x1, "VSP11_CH20", "cpri_rx1 dma request 20"),
	GCR_DESC(63, 8, 9, 0x2, "VSP11_CH20", "cpri_rx2 dma request 12"),
	GCR_DESC(63, 8, 9, 0x3, "VSP11_CH20", "cpri_rx2 dma request 20"),
	GCR_DESC(63, 6, 7, 0x0, "VSP11_CH19", "cpri_rx1 dma request 13"),
	GCR_DESC(63, 6, 7, 0x1, "VSP11_CH19", "cpri_rx1 dma request 21"),
	GCR_DESC(63, 6, 7, 0x2, "VSP11_CH19", "cpri_rx2 dma request 13"),
	GCR_DESC(63, 6, 7, 0x3, "VSP11_CH19", "cpri_rx2 dma request 21"),
	GCR_DESC(63, 4, 5, 0x0, "VSP11_CH18", "cpri_rx1 dma request 14"),
	GCR_DESC(63, 4, 5, 0x1, "VSP11_CH18", "cpri_rx1 dma request 22"),
	GCR_DESC(63, 4, 5, 0x2, "VSP11_CH18", "cpri_rx2 dma request 14"),
	GCR_DESC(63, 4, 5, 0x3, "VSP11_CH18", "cpri_rx2 dma request 22"),
	GCR_DESC(63, 2, 3, 0x0, "VSP11_CH17", "cpri_rx1 dma request 15"),
	GCR_DESC(63, 2, 3, 0x1, "VSP11_CH17", "cpri_rx1 dma request 23"),
	GCR_DESC(63, 2, 3, 0x2, "VSP11_CH17", "cpri_rx2 dma request 15"),
	GCR_DESC(63, 2, 3, 0x3, "VSP11_CH17", "cpri_rx2 dma request 23"),
	GCR_DESC(63, 0, 1, 0x0, "VSP11_CH16", "cpri_rx1 dma request 16"),
	GCR_DESC(63, 0, 1, 0x1, "VSP11_CH16", "cpri_rx1 dma request 24"),
	GCR_DESC(63, 0, 1, 0x2, "VSP11_CH16", "cpri_rx2 dma request 16"),
	GCR_DESC(63, 0, 1, 0x3, "VSP11_CH16", "cpri_rx2 dma request 24"),
};

static struct kobject *gcr_kobj;
static uint32_t debug = DEBUG_MESSAGES;

static int gcr_open(struct inode *inode, struct file *filp);
static int gcr_release(struct inode *inode, struct file *filp);
static long gcr_ctrl(struct file *filp, unsigned int cmd, unsigned long arg);

static const struct file_operations gcr_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= gcr_ctrl,
	.open		= gcr_open,
	.release	= gcr_release,
};

long gcr_ctrl(struct file *file, unsigned int cmd, unsigned long ioctl_arg)
{
	int ret = 0;
	int i = 0;
	struct gcr_priv *priv;
	struct gcr_reg_map *gcr;
	unsigned int size = 0;
	struct gcr_parm *arg = NULL;
	unsigned int count = 0;

	priv = (struct gcr_priv *)file->private_data;
	gcr = priv->gcr_reg;

	size = sizeof(struct gcr_parm);
	arg = kmalloc(size, GFP_KERNEL);
	if (!arg) {
		ERR("gcr_ctrl: kmalloc failed for gcr_parm\n");
		ret = -EFAULT;
		goto end;
	}

	if (copy_from_user((void *)arg, (struct gcr_parm *)ioctl_arg, size)) {
		kfree(arg);
		ERR("gcr_ctrl: copy_from_user failed\n");
		ret = -EFAULT;
		goto end;
	}
	count = ((struct gcr_parm *)arg)->count;
	switch (cmd) {
	case GCR_CONFIG_CMD:
		{
			struct dma_intf_switch_parm_t *chan_parm = NULL;

			size = count * sizeof(
					struct dma_intf_switch_parm_t);
			chan_parm = kmalloc(size, GFP_KERNEL);
			if (!chan_parm) {
				ERR("GCR_CONFIG_CMD: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)chan_parm,
					((struct gcr_parm *)arg)->parm, size)) {
				ERR("GCR_CONFIG_CMD: copy from user failed\n");
				kfree(chan_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_VSPA_2_IF_CFG) {
				int i;
				struct dma_intf_switch_parm_t *p =
				 (struct dma_intf_switch_parm_t*)chan_parm;
				pr_info(DRIVER_NAME ": VSP->IF DMA Config\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("DMA: %s CH%02d VSP%02d DMA%02d FRM%d %s\n",
						p->dev_type == 1 ? "CPRI" : "JESD",
						p->chan_id,
						p->vsp_id,
						p->dma_request_id,
						p->cpri_framert_id,
						p->comm_type == 1 ? "UL" : "DL");
				}
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_vsp_intf_dma_cfg(chan_parm, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(chan_parm);
		}
		break;
	case GCR_CPRI_DMA_MUX_CMD:
		{
			struct cpri_dma_mux_config *cpri_mux_parm = NULL;

			size = count * sizeof(struct cpri_dma_mux_config);
			cpri_mux_parm = kmalloc(size, GFP_KERNEL);
			if (!cpri_mux_parm) {
				ERR("GCR_CPRI_DMA..: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)cpri_mux_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_CPRI_DMA..: copy_from_user failed\n");
				kfree(cpri_mux_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_CPRI_MUX_CFG) {
				int i;
				struct cpri_dma_mux_config *p =
				 (struct cpri_dma_mux_config*)cpri_mux_parm;
				pr_info(DRIVER_NAME ": CPRI MUX Config\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("MUX: CPRI%02d->%s%d CH%02d\n",
						p->src_cpri_complex,
						p->rxtx_id > 2 ?
							"TX" : "RX",
						(p->rxtx_id % 1) + 1,
						p->cpri_dma_req_out);
				}
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_cpri_dma_mux(cpri_mux_parm, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(cpri_mux_parm);
		}
		break;
	case GCR_INTER_VSP_CFG:
		{
			struct inter_vsp_dma_config_t *inter_vsp_parm = NULL;

			size = count * sizeof(
					struct inter_vsp_dma_config_t);
			inter_vsp_parm =  kmalloc(size, GFP_KERNEL);
			if (!inter_vsp_parm) {
				ERR("GCR_VSP->VSP..: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}

			if (copy_from_user((void *)inter_vsp_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_VSP->VSP..: copy_from_user failed\n");
				kfree(inter_vsp_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_VSPA_2_VSPA_CFG) {
				int i;
				struct inter_vsp_dma_config_t *p =
				 (struct inter_vsp_dma_config_t*)inter_vsp_parm;
				pr_info(DRIVER_NAME ": VSP->VSP DMA Config\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("DMA: CH%02d VSPA%02d->VSPA%02d\n",
						p->chan_id,
						p->src_vsp_id,
						p->dst_vsp_id);
				}
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_inter_vsp_dma_cfg(inter_vsp_parm, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(inter_vsp_parm);
		}
		break;
	case GCR_JESD_PTR_RST_CFG:
		{
			struct jesd_dma_ptr_rst_parm *jesd_ptr_parm = NULL;

			size = count * sizeof(
					struct jesd_dma_ptr_rst_parm);
			jesd_ptr_parm = kmalloc(size, GFP_KERNEL);
			if (!jesd_ptr_parm) {
				ERR("GCR_PTR_RST..: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)jesd_ptr_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_PTR_RST..: copy_from_user failed\n");
				kfree(jesd_ptr_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_JESD_PTR_RESET) {
				int i;
				struct jesd_dma_ptr_rst_parm *p =
				  (struct jesd_dma_ptr_rst_parm*)jesd_ptr_parm;
				pr_info(DRIVER_NAME ": Reset JESD pointer\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("RST: VSPA%02d, JESD_%s%02d, DMA %02d\n",
						p->vsp_id,
						p->comm_type==1 ? "UL" : "DL",
						p->jesd_id, p->chan_id);
				}
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_jesd_dma_ptr_rst_req(jesd_ptr_parm,
					count);
			mutex_unlock(&priv->gcr_lock);
			kfree(jesd_ptr_parm);
		}
		break;
	case GCR_WRITE_REG:
		{
			struct gcr_ctl_parm *param = NULL;

			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (!param) {
				ERR("GCR_REG_WRITE: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_REG_WRITE: copy_from_user failed\n");
				kfree(param);
				ret = -EFAULT;
				goto end;
			}

			mutex_lock(&priv->gcr_lock);
			gcr_write_set(param, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(param);
		}
		break;
	case GCR_READ_REG:
		{
			struct gcr_ctl_parm *param = NULL;
			unsigned int m_addr = 0;
			int data = 0;

			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (!param) {
				ERR("GCR_REG_READ: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}

			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_REG_READ: copy_from_user failed\n");
				kfree(param);
				ret = -EFAULT;
				goto end;
			}
			for (i = 0; i < count; i++) {
				mutex_lock(&priv->gcr_lock);
				m_addr = (param + i)->reg_offset;
				data = gcr_read_set(m_addr);
				mutex_unlock(&priv->gcr_lock);
				(param + i)->param = data;
			}
			if (copy_to_user((void *)arg->parm,  param, size)) {
				kfree(param);
				dev_err(priv->gcr_dev, "copy_to_user failed\n");
				ret = -EFAULT;
				goto end;
			}
			kfree(param);
		}
		break;
	default:
		ERR("unknown ioctl command %u\n", cmd);
		ret = -EINVAL;
		goto end;
		break;
	}

end:
	kfree(arg);
	return ret;
}

static int gcr_open(struct inode *inode, struct file *file)
{
	struct gcr_priv *priv = NULL;
	int rc = 0;
	priv = container_of(inode->i_cdev, struct gcr_priv, gcr_cdev);
	if (priv != NULL) {
		MOD_INC_USE_COUNT;
		file->private_data = priv;
	} else {
		rc = -ENODEV;
	}

	return rc;
}

static int gcr_release(struct inode *inode, struct file *fp)
{
	struct gcr_priv *priv = (struct gcr_priv *)fp->private_data;

	if (!priv)
		return -ENODEV;

	MOD_DEC_USE_COUNT;
	return 0;
}

/***************************** Sysfs *******************************/
//TODO - complete sysfs: registers

static ssize_t show_debug(struct device *dev,
                        struct device_attribute *devattr, char *buf)
{
        return sprintf(buf, "0x%08X\n", debug);
}

static ssize_t set_debug(struct device *dev, struct device_attribute *devattr,
                        const char *buf, size_t count)
{
        int err;
        unsigned int val;

        err = kstrtouint(buf, 0, &val);
        if (err) return err;

        debug = val;
        return count;
}

static unsigned int gcr_table_probed;
static struct gcr_field_desc *gcr_ptrs[11][32][24];
static DEFINE_SPINLOCK(gcr_table_lock);

static ssize_t show_gcr_status(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int vsp_id, ch_id, ch_function, arr_index;
	char vsp_ch_name[16];
	u32 data, mask;
	struct gcr_field_desc *ptr;

	/* Rearrange the table */
	spin_lock(&gcr_table_lock);
	if (!gcr_table_probed) {
		gcr_table_probed = 1;
		for (vsp_id = 0; vsp_id < 11; vsp_id++) {
			for (ch_id = 0; ch_id < 32; ch_id++) {
				sprintf(vsp_ch_name, "VSP%d_CH%d", vsp_id + 1, ch_id);
				ch_function = 0;
				for (arr_index = 0; arr_index < ARRAY_SIZE(gcr_descs); arr_index++) {
					if (!strcmp(gcr_descs[arr_index].field_name, vsp_ch_name)) {
						if (ch_function >= 24) {
							pr_err("%s overflow buffer\n", vsp_ch_name);
							break;
						} else
							gcr_ptrs[vsp_id][ch_id][ch_function] =
								&gcr_descs[arr_index];
						ch_function++;
					}
				}
			}
		}
	}
	spin_unlock(&gcr_table_lock);

	for (vsp_id = 0; vsp_id < 11; vsp_id++) {
		pr_info("**VSP%d GCR connections**\n", vsp_id + 1);
		for (ch_id = 0; ch_id < 32; ch_id++) {
			for (ch_function = 0; ch_function < 24; ch_function++) {
				ptr = gcr_ptrs[vsp_id][ch_id][ch_function];
				if (!ptr)
					continue;
				else {
					data = gcr_read_set(ptr->gcr_id);
					mask = (1 << (ptr->field_bit_to - ptr->field_bit_from + 1)) - 1;
					if ((mask & (data >> ptr->field_bit_from))
						== ptr->field_bit_val)
						pr_info("%s connect to %s\n",
							ptr->field_name, ptr->field_desc);
				}
			}
		}
		pr_info("\n");
	}
	return 0;
}

static DEVICE_ATTR(debug,  S_IWUSR | S_IRUGO, show_debug,    set_debug);
static DEVICE_ATTR(status, S_IRUGO, show_gcr_status, NULL);
//static DEVICE_ATTR(versions,         S_IRUGO, show_versions, NULL);

static struct attribute *attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_status.attr,
	NULL
};

static const struct attribute_group attr_group = {
        .attrs = attributes,
};

/************************** Init / Exit ****************************/

static int __init init_gcr(void)
{
	int ret = 0;
	int err = 0;
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();
	int gcr_major = 0;
	int gcr_minor = 0;
	dev_t devt;

	if (!priv) {
		pr_err(DRIVER_NAME ": no private data\n");
		ret = -ENODEV;
		goto out_err;
	}

	err = alloc_chrdev_region(&devt, 0, GCR_DEV_MAX, DRIVER_NAME);
	if (err) {
		pr_err(DRIVER_NAME ": alloc_chrdev_region() failed: %d\n", err);
		ret = -EFAULT;
		goto out_err;
	}

	gcr_major = MAJOR(devt);
	gcr_minor = MINOR(devt);
	priv->dev_t = MKDEV(gcr_major, gcr_minor);

	priv->gcr_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(priv->gcr_class)) {
		ret = PTR_ERR(priv->gcr_class);
		pr_err(DRIVER_NAME ": class_create() failed %d\n", ret);
		goto out_err_1;
	}

	cdev_init(&priv->gcr_cdev, &gcr_fops);
	priv->gcr_cdev.owner = THIS_MODULE;
	err = cdev_add(&priv->gcr_cdev, priv->dev_t, GCR_DEV_MAX);
	if (err) {
		pr_err(DRIVER_NAME ": cdev_add() failed %d\n", err);
		ret = -EFAULT;
		goto out_err_2;
	}

	priv->gcr_dev = device_create(priv->gcr_class, NULL, priv->dev_t,
				NULL, DRIVER_NAME);
	if (IS_ERR(priv->gcr_dev)) {
		pr_err(DRIVER_NAME ": device_create() failed %ld\n",
						 PTR_ERR(priv->gcr_dev));
		ret = -EFAULT;
		goto out_err_3;
	}

	gcr_kobj = kobject_create_and_add(DRIVER_NAME, &(priv->gcr_dev->kobj));
	if (!gcr_kobj) {
                pr_err(DRIVER_NAME ": kobject_create_and_add() failed\n");
		ret = -ENOMEM;
		goto out_err_4;
	}

        ret = sysfs_create_group(gcr_kobj, &attr_group);
        if (ret < 0) {
                pr_err(DRIVER_NAME ": sysfs_create_group() failed %d\n", ret);
		goto out_err_5;
        }

	mutex_init(&priv->gcr_lock);
	dev_set_drvdata(priv->gcr_dev, priv);

	return 0;

out_err_5:
	kobject_put(gcr_kobj);
out_err_4:
	device_destroy(priv->gcr_class, priv->dev_t);
out_err_3:
	cdev_del(&priv->gcr_cdev);
out_err_2:
	class_destroy(priv->gcr_class);
out_err_1:
	unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
out_err:
	return ret;
}

static void __exit exit_gcr(void)
{
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();

	if ((!priv) || (!priv->gcr_dev)) {
                pr_err(DRIVER_NAME ": exit() has no private data\n");
		return;
	}
	kobject_put(gcr_kobj);
	device_destroy(priv->gcr_class, priv->dev_t);
	cdev_del(&priv->gcr_cdev);
	class_destroy(priv->gcr_class);
	unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
}

module_init(init_gcr);
module_exit(exit_gcr);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale AFD4400 GCR Driver");

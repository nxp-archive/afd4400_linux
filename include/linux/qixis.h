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

#ifndef __QIXIS_H__
#define __QIXIS_H__

/* M1A3PE1500-2FGG484 */
#define DEVICE_NAME "M1A3PE1500"
#define DRIVER_NAME "d4400-fpga"
#define READ_NIBBLE     0xc0
#define WRITE_NIBBLE    0x80

#define QIXIS_ID	0x00
#define QIXIS_EVBBRD_VER	0X01
#define QIXIS_FPGA_MAJOR_VER	0x02
#define QIXIS_FPGA_MINOR_VER	0x03
#define QIXIS_CLK_JCPLL_STATUS	0x33
#define QIXIS_CLK_JCPLL_ADDH	0x34
#define	QIXIS_CLK_JCPLL_ADDL	0x35
#define	QIXIS_CLK_JCPLL_DATA	0x36
#define QIXIS_STAT_PRESENT	0x0B

#define	APPLY_STATUS	0x80

/* in milli sec */
#define MAX_DELAY	2
#define TRY_COUNT	5

#define	JCPLL_STATUS_PIN_REG	0x17
#define JCPLL_PWR_DWN_REG	0x230
#define	JCPLL_IO_UPDATE_REG	0x232

#define	VDD_CP_NORMAL	0x2d
#define	VDD_CP_BY_HALF	0xad

#define	PWR_DWN_PLL	0x01
#define	PWR_ENB_PLL	0x00

#define	UPDATE_IO	0x01

enum {
	JCPLL_SUCCESS = 0,
	JCPLL_XFER_ERR = 1
};

void qixis_write(u8 val, u8 offset);
u8 qixis_read(u8 offset);
int write_jcpll_reg(u8 val, u16 offset);
u8 read_jcpll_reg(u16 offset);
int qixis_lock_jcpll(void);
int qixis_unlock_jcpll(void);
int qixis_get_xcvr_present_status(int xcvr_id);

#endif

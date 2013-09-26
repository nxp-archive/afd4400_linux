/*
 * include/linux/cpri_axc.h
 * CPRI device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __CPRI_AXC_H
#define __CPRI_AXC_H
/* Axc parameter reg CPRInRACPR and CPRInTACPR */
#define AXC_SIZE_MASK				0x1fff0000
#define AXC_MEM_BLK_MASK			0x2000
#define AXC_BASE_ADDR_MASK			0X1fff
#define AXC_SW_MASK				0x1f0000
#define AXC_TH_MASK				0xfff
#define AXC_MAX_MASK				0xf

#define AXC_MEM_ENABLE_MASK			0x1
#define AXC_NUM_MASK				0x1f
#define AXC_POS_MASK				0x7
#define AXC_WIDTH_MASK				0x1f
#define CPRI_AXC_ID_INVAL			0xff

#define RX_TX_MEM_ALLOCATED			2

/* axc map table configuration defines */
#define AXC_K1_MASK				0x003f0000
#define AXC_K0_MASK				0x0000003f
#define AXC_MODE_MASK				0x00000003
#define AXC_TBL_WRITE_MASK			0x1000
#define AXC_TBL_SEG_ADDR_MASK			0xfff
#define AXC_ENABLE_MASK				0x1

#define AXC_CONF_SHIFT				8
#define AXC_CONF_MASK				0x80
#define AXC_SMPL_WDTH_MASK			0x00001f9f

#define SET_CMD					0x1
#define CLEAR_CMD				0X0
#define SEG_SIZE				32
#define BF_WRDS					16
#define SEG0_OFFSET				10
#define SEG1_OFFSET				20
#define Ki_INVALID				0xffff
#define NO_AXC_SET				0xffffffff
#define NUM_SUBSEG				3
#define INVALIDE_MBLK_ADDR			0xffffffff

#define CEIL_FUNC(val, count) ((((val / count) * count) == val) ? \
	(val / count) : ((val / count) + 1))
#define BIT_POS(word, size, bit) ((((word - 1) * size) + bit) % SEG_SIZE)
#define K_POS(seg) ((seg > (SEG_SIZE - 1)) ? (seg - (SEG_SIZE - 1)) : seg)
#define K_MASK(seg) (0x1 << ((seg > (SEG_SIZE - 1)) ? (seg - (SEG_SIZE - 1)) :\
			seg))
#define K_OFFSET(seg) ((seg < (SEG_SIZE - 1)) ? 0 : 1)

struct segment_param {
	struct segment *segment;
	struct axc *axc;
	unsigned int ki;
	unsigned char cmd;
	unsigned int bit_position;
	unsigned int axc_size;
};

struct axc_mem_info {
	unsigned int rx_mblk_addr[2];
	unsigned int tx_mblk_addr[2];
	unsigned int rx_mblk_size[2];
	unsigned int tx_mblk_size[2];
};


#endif /* __CPRI_AXC_H */

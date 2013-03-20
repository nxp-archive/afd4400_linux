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

#define AXC_MEM_ENABLE_MASK			0x1
#define AXC_NUM_MASK				0x1f
#define AXC_POS_MASK				0x7
#define AXC_WIDTH_MASK				0x1f
#define CPRI_AXC_ID_INVAL			0xff

/* axc map table configuration defines */
#define AXC_K1_MASK				0x003f0000
#define AXC_K0_MASK				0x0000003f
#define AXC_MODE_MASK				0x00000003
#define AXC_TBL_WRITE_MASK			0x800
#define AXC_TBL_SEG_ADDR_MASK			0x7ff
#define AXC_ENABLE_MASK				0x1


struct segment_param {
	struct segment *segment;
	struct axc *axc;
	u32 *reg_cfgmemaddr0;
	unsigned int ki;
	unsigned char nst_len;
};



#endif /* __CPRI_AXC_H */

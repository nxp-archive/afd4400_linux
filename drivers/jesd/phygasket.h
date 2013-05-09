/*
* drivers/jesd/phygasket.h
*
* Author: Freescale semiconductor, Inc.
*
* Copyright 2013 Freescale Semiconductor, Inc.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/
#ifndef PHYGASKET_H_	/* prevent circular inclusion */
#define PHYGASKET_H_	/* by using protection macros */

#define PHY_DRIVER_NAME "phygasket"

#define PHYGASKET_INIT_SUCCESS 1

#define MAX_PHYGASKET_IP 3


#define GASKET0 0
#define GASKET1 1
#define GASKET2 2

/*patgen defs*/
#define JESD204TX 1
#define TEST_PATT 2
#define RX_PHY	3
#define RX_LOOPBACK 4

struct phy_gasket_regs {
	u32 tx_lane0_ctrl;
	u32 tx_lane1_ctrl;
	u32 tx_lane2_ctrl;
	u32 tx_lane3_ctrl;
	u32 tx_lane4_ctrl;
	u32 tx_lane5_ctrl;
	u32 tx_lane6_ctrl;
	u32 tx_lane7_ctrl;
	u32 tx_swap;
	u32 reserved1[(0x103c - 0x1024)/sizeof(u32)];
	u32 rx_lane0_ctrl;
	u32 rx_lane1_ctrl;
	u32 rx_lane2_ctrl;
	u32 rx_lane3_ctrl;
	u32 rx_lane4_ctrl;
	u32 rx_lane5_ctrl;
	u32 rx_lane6_ctrl;
	u32 rx_lane7_ctrl;
	u32 rx_swap;
	u32 reserved2[(0x107C - 0x1064)/sizeof(u32)];
	u32 tx_patgen;
	u32 tx_softreset;
	u32 rx_softreset;
	u32 reserved3[(0x1FFC - 0x108C)/sizeof(u32)];
};
/** @brief phygasket which hold info for the phy gasket
* this instance is to be used by all the transports
*/
struct phygasket_private {
	struct phygasket *phy[MAX_PHYGASKET_IP];
	/*accomidate platform device*/
	struct device *dev;
};

struct phygasket {
	u8 init_flag;
	struct device_node *node;
	struct phy_gasket_regs *pregs;
	struct tx_lane_ctrl *tln_ctrl;
	struct rx_lane_ctrl *rln_ctrl;
	union pat_gen *pgen;
	struct list_head list;
};

struct patter_generator {
	unsigned int go:1;
	unsigned int res1:1;
	unsigned int rpt_ila:1;
	unsigned int res2:3;
	unsigned int init_cgs:2;
	unsigned int payload:3;
	unsigned int res3:4;
	unsigned int disp:1;
	unsigned int opmf:11;
	unsigned int res4:5;
};

union pat_gen {
	struct patter_generator pgenerator;
	u32 pattern;
};

struct tx_lane_ctrl {
	u8 ln_txmux;
	u8 ln_inv;
};

struct rx_lane_ctrl {
	u8 source;
	u8 ln_inv;
};

struct lane_id {
	char dts_lane_id[32];
	int identifier;
};


int phy_softreset(struct phygasket *phy, u8 device,
				u8 reset, u8 lane);
int phy_swap_lanes(struct phygasket *phy, u8 lane,
				u8 swap, u8 device);
int phy_inverse_lanes(struct phygasket *phy, u8 lane,
				u8 invert, u8 device);
int phy_gasket_lane_ctrl(struct phygasket *phy,
				u8 mode, u8 lane);
int do_patter_generator(struct phygasket *phy, struct patgen *pgen);
struct phygasket *map_phygasket(struct device_node *phy_node);
#endif /*PHYGASKET_H_*/

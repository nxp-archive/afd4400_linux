#ifndef SERDES_D4400_H_	/* prevent circular inclusion */
#define SERDES_D4400_H_	/* by using protection macros */
/*
 * serdes-d4400.h
 *
 * Author: Anil Jaiswal b44339@freescale.com
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/* The SerDes module is programmed by control/status registers (CSRs).
 * The CSRs are used for mode control, and to extract status information.
 * All accesses to and from the registers must be made as 32-bit accesses.
 * There is no support for accesses of sizes other than 32 bit
 */

#define MAX_NUMBER_OF_LANES 8

enum srds_grp_prot_id {
	SERDES_PROT_SGMII,
	SERDES_PROT_CPRI,
	SERDES_PROT_JESD_1_LANE,
	SERDES_PROT_JESD_2_LANE,
	SERDES_PROT_INVAL
};

enum srds_lane_prots {
	SERDES_LANE_PROTS_SGMII = 0x01,
	SERDES_LANE_PROTS_XFI  = 0x0A,
	SERDES_LANE_PROTS_CPRI = 0x0B,
	SERDES_LANE_PROTS_JESD204 = 0x0C,
};

enum srds_pll_id {
	SERDES_PLL_1,
	SERDES_PLL_2,
	SERDES_PLL_MAX
};

enum srds_vco_type {
	SERDES_LC_VCO,
	SERDES_RING_VCO,
	SERDES_VCO_TYPE_INVAL
};

enum srds_rfclk_sel_freq {
	REF_CLK_FREQ_100_MHZ,
	REF_CLK_FREQ_122_88_MHZ,
	REF_CLK_FREQ_156_MHZ,
	REF_CLK_FREQ_INVAL
};

enum srds_frate_sel_freq_vco {
	PLL_FREQ_5_GHZ,
	PLL_FREQ_4_9152_GHZ,
	PLL_FREQ_3_072_GHZ,
	PLL_FREQ_INVAL
};

enum srds_lane_speed_sel {
	FRAT_QUARTER,
	FRAT_HALF,
	FRAT_FULL,
	FRAT_DOUBLE,
	FRAT_INVAL = 0xFF
};

enum srds_lane_i_o_slew {
	IOSLEW_RCTL_SGMII_125Gb,
	IOSLEW_RCTL_SGMII_3125_Gb,
	IOSLEW_RCTL_CPRI_12_24_49_98Gb,
	IOSLEW_RCTL_CPRI_31_61Gb,
	IOSLEW_RCTL_JESD,
	IOSLEW_RCTL_INVAL
};

enum srds_proto_rx_eq_bst {
	RXEQ_BOOST_CPRI_12_24_31Gb,
	RXEQ_BOOST_CPRI_49_61_98Gb,
	RXEQ_BOOST_JESD_12_24_31Gb,
	RXEQ_BOOST_JESD_49_61_98Gb,
	RXEQ_BOOST_OTHERS
};

/* GK2OVD */
enum srds_gk20vd {
	GK2OVD_SGMII_125Gb,
	GK2OVD_XFI,
	GK2OVD_CPRI_24_31_49_61_98Gb,
	GK2OVDGb_CPRI,
	GK2OVDGb_JESD
};

/* GK3OVD */
enum srds_gk30vd  {
	GK3OV_SGMII_125Gb,
	GK3OVD_XFI,
	GK3OVD_CPRI_12Gb,
	GK3OVD_CPRI_24_31_49_61_98Gb,
	GK3OVD_JESD
};

/* GK2OVD_EN */
enum srds_gk20vd_en  {
	GK2OVD_EN_SGMII_125Gb,
	GK2OVD_EN_CPRI_12Gb,
	GK2OVD_EN_CPRI_24_31_49_61_98Gb,
	GK2OVD_EN_OTHER
};

/* GK3OVD_EN */
enum srds_gk30vd_en  {
	GK3OVD_EN_SGMII_125Gb,
	GK3OVD_EN_CPRI_12Gb,
	GK3OVD_EN_CPRI_24_31_49_61_98Gb,
	GK30VD_OTHER
};

/* internal recovered clock divider select
 */
enum srds_rcvrd_clk_div {
	DIV_BY_OF,
	DIV_BY_20,
	DIV_BY_24,
	DIV_BY_25,
	DIV_BY_30,
	DIV_BY_32,
	DIV_BY_40,
	DIV_BY_50
};

/* Control bits used for the CPRIn Protocol
 * SDRSxCPRInCR0[RX_DLY]
 */
enum srds_rx_dly {
	RX_DLY_1_2288Gbps,
	RX_DLY_2_4576Gbps,
	RX_DLY_3_0720Gbps,
	RX_DLY_4_9152Gbps,
	RX_DLY_6_1440Gbps,
	RX_DLY_9_8304Gbps
};

/* SDRSxCPRInCR1[TX_DLY]
 */
enum srds_tx_dly {
	TX_DLY1_2288Gbps,
	TX_DLY2_4576Gbps,
	TX_DLY3_0720Gbps,
	TX_DLY4_9152Gbps,
	TX_DLY6_1440Gbps,
	TX_DLY9_8304Gbps
};

enum srds_amp_red {
	AMP_RED_SGMII,
	AMP_RED_JESD,
	AMP_RED_OTHERS
};

enum srds_lane_id {
	LANE_A = 0,
	LANE_B,
	LANE_C,
	LANE_D,
	LANE_E,
	LANE_F,
	LANE_G,
	LANE_H,
	LANE_INVAL
};

/* SerDes PLLn Reset Control Register */
struct serdes_pll_regs {
	u32 pll_rstctl_reg;
	u32 pll_ctl_reg0;
	u32 pll_ctl_reg1;
	u32 reserved[2];	/* 0x00C-0x010 */
	u32 pll_ctl_reg4;
	u32 reserved2[2];
};

/* SerDes Lane_n Control and Status Registers */
struct serdes_per_lane_regs {
	u32 gcr0;
	u32 gcr1;
	u32 reserved_1[2];	/* 0x808-0x810 */
	u32 rx_ecr0;
	u32 rx_ecr1;
	u32 tx_ecr0;
	u32 reserved_2;		/* 0x81C-0x820 */
	u32 TTL_cr0;
	u32 TTL_cr1;
	u32 reserved_3[5];	/* 0x828-0x83C */
	u32 tcsr3;
};

/** \struct serdes_regs register structure
*  @brief This is serdes register set structure include the reserved if any.
*  each SerDes module from 0x000 to 0xFFF (4 KB).
*
*  General SerDes Control Registers set -
*	SerDes module is programmed by control/status
*	registers (CSRs) to offset 0x000 to 0x0FF 0x0FF.
*
*  Protocol Global Registers set -
*	Protocol Global Registers offset from 0x100 - 0x1e0
*  Per-Lane Control and Status Registers set -
*	Per-Lane Control and Status Registers to offset from 0x800-0x9F0
*/
struct serdes_regs {
	/* General SerDes Reset Control Registers (R/W)*/
	struct serdes_pll_regs pll_reg[2];
	u32 reserved_2[20];	/* 0x040-0x090 */
	u32 tx_calib_ctl_reg;
	u32 reserved_3[3];	/* 0x094-0x0A0 */
	u32 rx_calib_ctl_reg;
	u32 reserved_4[3];	/* 0x0A4-0x0B0 */
	u32 general_ctl_reg0;
	u32 reserved_5[11];	/* 0x0B4-0x0E0 */
	u32 pccr0;
	u32 pccr1;
	u32 pccr2;		/* 0x0E8 */
	u32 pccr3;
	u32 pccr4;
	u32 pccr5;
	u32 reserved_6[2];	/* 0x0F8-0x100 */
	/* Protocol Global Registers (Read Only)*/
	u32 lane0_pssr_reg;
	u32 reserved_7[7];	/* 0x104-0x120 */
	u32 lane1_pssr_reg;
	u32 reserved_8[7];	/* 0x124-0x140 */
	u32 lane2_pssr_reg;
	u32 reserved_9[7];	/* 0x144-0x160 */
	u32 lane3_pssr_reg;
	u32 reserved_10[7];	/* 0x164-0x180 */
	u32 lane4_pssr_reg;
	u32 reserved_11[7];	/* 0x184-0x1A0 */
	u32 lane5_pssr_reg;
	u32 reserved_12[7];	/* 0x1A4-0x1C0 */
	u32 lane6_pssr_reg;
	u32 reserved_13[7];	/* 0x1C4-0x1E0 */
	u32 lane7_pssr_reg;
	u32 reserved_14[391];	/* 0x1E4-0x800 */
	/* Per-Lane Control and Status Registers (R/W)*/
	struct serdes_per_lane_regs lane_csr[MAX_NUMBER_OF_LANES];
};

struct lane_gen_conf {
	enum srds_lane_i_o_slew		islew_id;
	enum srds_lane_i_o_slew		oslew_id;
	enum srds_lane_prots		lane_prot;
	u32 bit_rate_kbps;
#define SERDES_20BIT_EN		(1 << 0)
#define SERDES_TPLL_LES		(1 << 1)
#define SERDES_RPLL_LES		(1 << 2)
#define SERDES_FIRST_LANE	(1 << 3)
#define SERDES_LOOPBACK_EN	(1 << 4)
/*
 *	BIT definition for cflag
 *	BIT0:	1: 20Bit Enable
 *		0: 20Bit Disable
 *
 *	BIT1:	1: TPLL_LES bit 1
 *		0: TPLL_LES bit 0
 *
 *	BIT2:	1: RPLL_LES bit 1
 *		0: RPLL_LES bit 0
 *
 *	BIT3:	1: FIRST_LANE Lane m is lane 0 of the link
 *		0: FIRST_LANE bit Lane m is NOT lane 0 of the link
 *
 *	BIT4:	1: LPBK_EN: To enable loopback mode
 *		0: LPBK_EN: To disable loopback / enable Application mode
 */
	u32 cflag;
};

struct lane_rx_eq_conf {
	enum srds_proto_rx_eq_bst	rx_eq_bst_id;
	enum srds_gk20vd		gk20vd_e_id;
	enum srds_gk30vd		gk30vd_e_id;
	enum srds_gk20vd_en		gk20vd_en_e_id;
	enum srds_gk30vd_en		gk30vd_en_e_id;
};

struct lane_tx_eq_conf {
	enum srds_amp_red		amp_red_id;
};

struct lane_ttl_conf {
	enum srds_rcvrd_clk_div		clk_div_id;
};

struct cpri_delay_conf {
	enum srds_rx_dly		rx_dly_id;
	enum srds_tx_dly		tx_dly_id;
};

struct serdes_lane_params {
	enum srds_lane_id		lane_id;
	enum srds_grp_prot_id		grp_prot;
	struct lane_gen_conf		gen_conf;
	struct lane_rx_eq_conf		rx_eq_conf;
	struct lane_tx_eq_conf		tx_eq_conf;
	struct lane_ttl_conf		ttl_conf;
	struct cpri_delay_conf		cpri_delay;
	u32 cflag;
};

struct serdes_pll_params {
	enum srds_pll_id		pll_id;
	enum srds_vco_type		vco_type;
	enum srds_rfclk_sel_freq	rfclk_sel;
	enum srds_frate_sel_freq_vco	frate_sel;
	u32 cflag;
};

/** \struct SerDes Device Structure
* @brief structure for the serdes device */
struct serdes_dev {
	struct serdes_regs __iomem *regs;
	struct device_node *node;
	struct device *dev;
	struct list_head list;
	u32 serdes_id;
	u32 max_lane;
#define SERDES_PLL1_INIT_DONE		(1 << 0)
#define SERDES_PLL2_INIT_DONE		(1 << 1)
/*	BIT definition for cflaags
 *
 *	BIT0	0: PLL1 Not locked
 *		1: PLL1 locked
 *
 *	BIT1:	0: PLL2 Not locked
 *		1: PLL2 locked */
	u32 cflag;
	spinlock_t lock;
	struct serdes_pll_params pll[SERDES_PLL_MAX];
	struct serdes_lane_params lane[MAX_NUMBER_OF_LANES];
};

/* --------------------------------------------------------------------------*/
/**
 * @Synopsis Configures lane for multiple bit rates and other
 * configuration parameters
 *
 * @Param sdev_handle
 * @Param lane_param
 *
 * @Returns 0: On success
 *	Error code: On failure
 */
/* --------------------------------------------------------------------------*/
int serdes_init_lane(void *sdev_handle, struct serdes_lane_params *lane_param);

/* --------------------------------------------------------------------------*/
/**
 * @Synopsis Initialize SerDes PLL for Multiple frequency support for
 * required bit rates
 *
 * @Param sdev_handle
 * @Param pll
 *
 * @Returns 0: On success
 *	Error code: On failure
 */
/* --------------------------------------------------------------------------*/
int serdes_init_pll(void *sdev_handle, struct serdes_pll_params *pll);

/* --------------------------------------------------------------------------*/
/**
 * @Synopsis  Parses SerDes handle and returns serdes node pointer and lane ID
 *
 * @Param np
 * @Param serdesspec
 * @Param propname
 * @Param index
 *
 * @Returns 0		: On success
 *	Error code	: On failure
 */
/* --------------------------------------------------------------------------*/
int of_get_named_serdes(struct device_node *np,
		struct of_phandle_args *serdesspec, const char *propname,
		int index);

/* --------------------------------------------------------------------------*/
/**
 * @Synopsis  Attaches calling device with SerDes and returns SerDes Dev handle
 *
 * @Param serdes_dev_node
 *
 * @Returns     SerDes Dev Handle: On Success
 *		NULL: On Failure
 */
/* --------------------------------------------------------------------------*/
void *get_attached_serdes_dev(struct device_node *serdes_dev_node);

/* --------------------------------------------------------------------------*/
/**
 * @Synopsis  Power down the single lane, based on ID provided
 *
 * @Param sdev_handle
 * @Param lane_id
 *
 * @Returns 0: On success
 *	Error code: On failure
 */
/* --------------------------------------------------------------------------*/
int serdes_lane_power_down(void *sdev_handle, enum srds_lane_id lane_id);

/* --------------------------------------------------------------------------*/
/**
 * @Synopsis  Power up the single lane, based on ID provided
 *
 * @Param sdev_handle
 * @Param lane_id
 *
 * @Returns 0: On success
 *	Error code: On failure
 */
/* --------------------------------------------------------------------------*/
int serdes_lane_power_up(void *sdev_handle, enum srds_lane_id lane_id);

/* --------------------------------------------------------------------------*/
/**
 *@Synopsis serdes jcpll clock routing
 *
 * @Param sdev_handle
 *@Param lane
 *@Param pll
 *
 * @Returns 0: On success
 *     Error code: On failure
 */
/* --------------------------------------------------------------------------*/

void serdes_jcpll_enable(void *sdev_handle,
	struct serdes_lane_params *lane_param, struct serdes_pll_params *pll);
#endif

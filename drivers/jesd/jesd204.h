/*
* drivers/jesd/jesd204.h
* Author: Freescale semiconductor, Inc.
*
* Copyright 2013 Freescale Semiconductor, Inc.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/
#ifndef JESD204_H_	/* prevent circular inclusion */
#define JESD204_H_	/* by using protection macros */

#include <uapi/linux/jesd204.h> /*user and kernel interface header*/

/** @brief driver name jesd204 in here
*/
#define DRIVER_NAME "jesd204"

/** @brief a true false local for jesd204
*/
#define JFALSE 0
#define JTRUE1

/** @brief device type, for TX its 1 as identifier and for rx its 2
*/
#define DEVICE_TX	1
#define DEVICE_RX	2

/** @brief this is to be max devices that we are going to support
*/
#define TRANSPORT_PER_IP 2
#define MAX_LANES 2
/** @brief max lanes for transport identifier trans 0 == 1 and trans 1 == 2
*/
#define TRANPORT_0 1
#define TRANPORT_1 2
/** @brief max lanes for transport identifier trans 0 == 1 and trans 1 == 2
*/
#define LANEID_FOR_PRIMARY 1
#define LANEID_FOR_SECONDARY 2

#define LANEID_FOR_0 0
#define LANEID_FOR_1 1

/** @brief mask bits def's are in here
*/
#define UNDERF_ISR_BIT3		3
#define WCBOF_ISR_BIT4		4
#define WCBUF_ISR_BIT5		5
#define SYSREF_ISR_BIT8		8
#define SYNC_ISR_BIT9		9
#define DFIFO_BIT3		3

#define SKEW_ERR_BIT4		4
#define UEX_K_BIT5		5
#define NIT_BIT6		6
#define PARITY_ERR_BIT7		7

#define BP_8b10_BIT0		0
#define REVERSE_8b10_BIT1	1

#define FRMR_TPL_BIT0		0
#define FRMR_DLL_BIT1		1
#define S_RST_BIT1		1

#define REP_DAT_BIT5		5
#define QUEUE_TST_BIT4		4

#define ILS_MODE_BIT7		7
#define RX_DEV_BIT2		2

#define ENABLE_CLK_BIT0		0
#define SCR_CTRL_L0_BIT0	0
#define SCR_CTRL_L1_BIT1	1

#define L2SIDES_BIT0		0
#define BYP_ILAS_BIT2		2
#define BYP_ACG_BIT3		3
#define IDLE_SELECT_BIT1	1

#define WC_RC_PROTECT_BIT18	18
#define OCT_FIRST_BIT9		9
#define IQ_SWAP_BIT7		7
#define PHY_FIRT_BIT8		8
#define TN_CHK_CSUM_BIT12	12

#define SCRAMBL_BIT7		7
#define DMA_DIS_BIT17		17
#define TX_FRM_BIT1		1
#define RX_CTRL_BIT7		7


#define RX_DIS_BIT7		7
#define SYSREF_MASK_BIT20	20

#define SW_DMA_BIT17		17

#define BAD_DIS_SYNC		0
#define NIT_SYNC		1
#define UX_CHAR_SYNC		2
#define SYNC_ASSERT_BIT7	7
#define SYNC_ASSERT_BIT6	6
#define SYNC_ASSERT_BIT5	5


/** @brief transport flag bit test params -- start
*/
#define	RESERVED_BIT_0		0
#define	MS_OCT_FIRST_VALID	1/*for tx*/
#define	WCBUF_PROTECT_VALID	2/*for tx*/
#define	BYP_ACG_VALID		3/*only for tx*/
#define	BYP_ILAS_VALID		4/*only for tx*/
#define	L2SIDES_VALID		5/*only for tx*/
#define	SCR_CTRL_L1_VALID	6/*only for tx*/
#define	SCR_CTRL_L0_VALID	7/*only for tx*/
#define	IDLE_SELECT_VALID	8/*common for tx and rx transport*/
#define	IQ_SWAP_VALID		9/*common for tx and rx transport*/
#define	PHY_BIT_FIRST_VALID	10/*for rx*/
#define	TN_CHK_CSUM_VALID	11/*for rx*/
#define	PHY_OCT_FIRST_VALID	12/*for rx*/
#define	RCBUF_PROTECT_VALID	13/*for rx*/
#define	RESERVED_BIT_14		14
#define	RESERVED_BIT_15		15
#define	RESERVED_BIT_16_31	16/*reserved for future*/
/*trans_flg end*/

/**@brief isr def's, a linked isr def's*/
#define	ILS_SUCCESS		1
#define	ILS_FAIL		(ILS_SUCCESS + 1)
#define	CGS_SUCCESS		(ILS_FAIL + 1)
#define	CGS_FAIL		(CGS_SUCCESS + 1)
#define	CGS_NOT_CONFIGED	(CGS_FAIL + 1)
#define	FSF_SUCCESS		(CGS_NOT_CONFIGED + 1)
#define	FSF_FAIL		(FSF_SUCCESS + 1)
#define	FSF_NOT_CONFIGED	(FSF_FAIL + 1)
#define	CSUM_SUCCESS		(FSF_NOT_CONFIGED + 1)
#define	CSUM_FAIL		(CSUM_SUCCESS + 1)
#define	CSUM_NOT_CONFIGED	(CSUM_FAIL + 1)
#define	UEX_K_FAIL		(CSUM_NOT_CONFIGED + 1)
#define	NIT_FAIL		(UEX_K_FAIL + 1)
#define	BAD_DIS_FAIL		(NIT_FAIL + 1)
#define	PAC_OF_ERR		(BAD_DIS_FAIL + 1)
#define	PAC_UF_ERR		(PAC_OF_ERR + 1)
#define	RBUF_OF_ERR		(PAC_UF_ERR + 1)
#define	RBUF_UF_ERR		(RBUF_OF_ERR + 1)
#define	SFIFO_ERR		(RBUF_UF_ERR + 1)
#define	SYS_REF_ROSE		(SFIFO_ERR + 1)
#define	PHY_DATA_LOST		(SYS_REF_ROSE + 1)
#define	SYNC_DETECTED		(PHY_DATA_LOST + 1)
#define	UPAC_UF_ERR		(SYNC_DETECTED + 1)
#define	WBUF_OF_ERR		(UPAC_UF_ERR + 1)
#define	WBUF_UF_ERR		(WBUF_OF_ERR + 1)
#define START_TIMEOUT		(WBUF_UF_ERR + 1)
/*mark the end if needed do append*/
#define	ERROR_END		(START_TIMEOUT + 1)
/* isr def's end*/

/**@brief misc offsets, mask, mac's
*/
#define	RESET_IRQ_VECTOR	0x80
#define	NORMAL_MODE		0x00
#define	TEST_MODE		0x00
/* this is totaly dummy for signal handlers*/
#define	SIG_TEST		0x40

/** @brief error state for jesd204
 * error state that the device could be in
 */
enum error_identifier {
	no_error,
	unpack_uf,
	pack_uf,
	pack_of,
	rcbuf_uf,
	rcbuf_of,
	wcbuf_uf,
	wcbuf_of,
	baddis_t,
	nit_t,
	uexk_t,
	fsync_err,
	cgs_err,
	csum_err,
	ils_err,
	sync_fail,
	sig_fail,
};
/** @brief shall have all the instance for all registers include reserved sets
 *too, supported by the ip
*/
struct config_registers_tx {
	u32 tx_did;
	u32 tx_bid;
	u32 tx_lid_0;
	u32 tx_scr;
	u32 tx_opf;
	u32 tx_fpmf;
	u32 tx_cpd;
	u32 tx_cbps;
	u32 tx_nbcw;
	u32 tx_spcp;
	u32 tx_hdf;
	u32 tx_res1;
	u32 tx_res2;
	u32 tx_csum_0;
/*protected tranport 0 region*/
	u32 tx_reserved1[(0x044 - 0x038) / sizeof(u32)];
	u32 tx_lid_1;
	u32 tx_res_us_5;
	u32 tx_res_us_6;
	u32 tx_csum_1;
	u32 tx_reserved2[(0x2fc - 0x058) / sizeof(u32)];
/*protected tranport 0 region*/
	u32 tx_frm_ctrl;
	u32 tx_reserved3[(0x37f - 0x304) / sizeof(u32)];
	u32 tx_m_en;
	u32 tx_res3;
	u32 tx_l_en;
	u32 tx_res4;
	u32 tx_scr_ctrl;
	u32 tx_res5;
	u32 tx_ln_ctrl;
	u32 tx_reserved4[(0x3bc - 0x39c) / sizeof(u32)];
	u32 tx_ilas_len;
	u32 tx_res6;
	u32 tx_frm_tst;
	u32 tx_jedec_tst;
	u32 tx_sdiv;
	u32 tx_reserved5[(0x3fc - 0x3d4) / sizeof(u32)];
	u32 tx_transcontrol;
	u32 tx_irq_status;
	u32 tx_irq_enable;
	u32 tx_sync_fil_char;
	u32 tx_sync_delay;
	u32 tx_diag_sel;
	u32 tx_diag_data;/* this is a read only register */
	u32 tx_reserved6[(0x7fc - 0x420) / sizeof(u32)];
};
/** @brief shall have all the instance for all registers include reserved sets
* too supported by the ip
*/
struct config_registers_rx {
/*r- only*/
	u32 rx_rdid;
	u32 rx_rbid;
	u32 rx_rlid_0;
	u32 rx_rscr;
	u32 rx_ropf;
	u32 rx_rfpmf;
	u32 rx_rcpd;
	u32 rx_rcbps;
	u32 rx_rnbcw;
	u32 rx_rspcp;
	u32 rx_rhdf;
	u32 rx_res1;
	u32 rx_res2;
	u32 rx_rcsum_0;
	u32 rx_rcomp_csum_0;
	u32 rx_reserved1[(0x044 - 0x03c) / sizeof(u32)];
	u32 rx_rlid_1;
	u32 rx_reserved2[(0x050 - 0x04c) / sizeof(u32)];

	u32 rx_rcsum_1;
	u32 rx_rcomp_csum_1;
	u32 rx_reserved3[(0x13c - 0x05c) / sizeof(u32)];
/*rw*/
	u32 rx_did;
	u32 rx_bid;
	u32 rx_lid_0;
	u32 rx_scr;
	u32 rx_opf;
	u32 rx_fpmf;
	u32 rx_cpd;
	u32 rx_cbps;
	u32 rx_nbcw;
	u32 rx_spcp;
	u32 rx_hdf;
	u32 rx_res3;
	u32 rx_res4;
	u32 rx_csum_0;
	u32 rx_reserved4[(0x1a8 - 0x178) / sizeof(u32)];
/*dfrmr*/
	u32 rx_err_count_mon;
	u32 rx_lane_dskew;
	u32 rx_bad_parrity;
	u32 rx_nit;
	u32 rx_ux_char;
	u32 rx_cgs;
	u32 rx_fsf;
	u32 rx_g_csum;
	u32 rx_ilsf;
	u32 rx_skew_err;
	u32 rx_ctrl_0;
	u32 rx_ctrl_1;
	u32 rx_ctrl_2;
	u32 rx_ilas_len;
	u32 rx_res5;
	u32 rx_irq_ve_msk;
	u32 rx_sync_ass_msk;
	u32 rx_threshold_err;
	u32 rx_lane_en;
	u32 rx_reserved5[(0x1fc - 0x1f8) / sizeof(u32)];
/*misc*/
	u32 rx_transcontrol;
	u32 rx_status_reg;
	u32 rx_irq_status;
	u32 rx_irq_enable;
	u32 rx_err_report;
	u32 rx_rcv_delay;
	u32 rx_diag_sel;
	u32 rx_diag_data;/* this is a read only register */
	u32 rx_reserved6[(0x3ff - 0x220) / sizeof(u32)];
};
/** @brief \struct lanes which belong to a transport
*/
struct lane_device {
/* point to the transport we belong*/
	struct transport_device *trans_dev_params;
	struct lane_stats *l_stats;
	u8 id;
};
/** @brief /stuct jesd devices to be created for each transport
 *	hence the instance is defined in here.
 */
struct transport_device {
	char name[32];
	/*accomidate platform device*/
	struct device *dev;
	unsigned int identifier;
	/* device back pointers */
	struct jesd204_dev *jesd_parent;
	/*point to the lanes that we hold*/
	struct lane_device *lane_dev[MAX_LANES];
	u8 lanes_grabbed;
	/* this is to tell the mode we are in defaulted to normal mode*/
	u8 mode;
	/* either of one will be a null pointer */
	struct config_registers_tx *tx_regs;
	struct config_registers_rx *rx_regs;

	struct tasklet_struct do_tx_tasklet;
	struct tasklet_struct do_rx_tasklet;
	spinlock_t lock_tx_bf; /* for isr*/
	spinlock_t lock_rx_bf; /* for isr*/

	u32 transport_type; /*flag to know the transport type tx/rx*/

	struct siginfo info;
	struct task_struct *task;
	/*depending on the type of transport the delay is configured*/
	u8 delay;
	u8 ilsa_len;
	unsigned long sampling_rate;
	u32 tranport_flags;
	struct ils_params ils;
	u8 lanes_per_transport;

	enum jesd_state dev_state;
	/* this would yield the details for the error that is occurred when
	* the jesd state is error state
	*/
	enum error_identifier err_tp;
	/*error and debug details*/
	struct transport_stats t_stats;

	/*irq config*/
	/*struct irq_userconf irq_cnf;*/
	int irq;
	union irq_txuserconf irq_tx;
	struct irq_rxuserconf irq_rx;

	/*test config*/
	/*either of one will be a used in transport type*/
	struct conf_tx_tests tx_tests;
	struct conf_rx_tests rx_tests;

	/*phygasket config*/
	unsigned int gasket;
	struct device_node *phy_node;
	struct phygasket *phy;
	unsigned int lane_id[MAX_LANES + 1];

	struct cdev c_dev;/* char device structure */
	dev_t devt; /*for mkdev */

	void __iomem *base_address;/* virt. address of the control segment */

	wait_queue_head_t isr_wait;
	wait_queue_head_t to_wait;
	int sysref_rose;
	int isr_error;
	int evnt_jiffs; /* jiffies to max 2*/
	spinlock_t sysref_lock; /* for sysref rose*/

	struct tbgen_dev *tbg;
};
/* jesd god device structure instance*/
struct jesd204_dev {
	struct transport_device *trans_device[TRANSPORT_PER_IP];
	u8 max_lanes;
	u8 lanes_grabed;
	u8 transport_count;
	u32 trans_type;
	struct device *dev; /*accomidate platform device*/
	struct device_node *node;
	struct device_node *child;
	dev_t devt;
	struct list_head jesd_dev_list;
};

struct jesd204_private {
	struct list_head list; /*point to list of jesd204_dev's*/
	u8 tx_dev_num;
	u8 rx_dev_num;
};
/** @brief xport symbols for jesd
*/
int jesd_start_transport(struct transport_device *tdev);
#endif

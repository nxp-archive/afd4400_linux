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
#define TRANPORT_0 0
#define TRANPORT_1 1
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
	u32 tx_reserved1[(0x048 - 0x038) / sizeof(u32)];
	u32 tx_lid_1;
	u32 tx_res_us_5;
	u32 tx_res_us_6;
	u32 tx_csum_1;
	u32 tx_reserved2[(0x300 - 0x058) / sizeof(u32)];
/*protected tranport 0 region*/
	u32 tx_frm_ctrl;
	u32 tx_reserved3[(0x380 - 0x304) / sizeof(u32)];
	u32 tx_m_en;
	u32 tx_res3;
	u32 tx_l_en;
	u32 tx_res4;
	u32 tx_scr_ctrl;
	u32 tx_res5;
	u32 tx_ln_ctrl;
	u32 tx_reserved4[(0x3c0 - 0x39c) / sizeof(u32)];
	u32 tx_ilas_len;
	u32 tx_res6;
	u32 tx_frm_tst;
	u32 tx_jedec_tst;
	u32 tx_sdiv;
	u32 tx_reserved5[(0x400 - 0x3d4) / sizeof(u32)];
	u32 tx_transcontrol;
	u32 tx_irq_status;
	u32 tx_irq_enable;
	u32 tx_sync_fil_char;
	u32 tx_sync_delay;
	u32 tx_diag_sel;
	u32 tx_diag_data;/* this is a read only register */
	u32 tx_reserved6[(0x800 - 0x420) / sizeof(u32)];
};
/** @brief shall have all the instance for all registers include reserved sets
* too supported by the ip
*/
struct config_registers_rx {
	/*Rceived ILS params from transmitter*/
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
	u32 rx_reserved1[(0x048 - 0x03c) / sizeof(u32)];
	u32 rx_rlid_1;
	u32 rx_reserved2[(0x054 - 0x04c) / sizeof(u32)];

	u32 rx_rcsum_1;
	u32 rx_rcomp_csum_1;
	u32 rx_reserved3[(0x140 - 0x05c) / sizeof(u32)];
	/* Locally programmed ILS params*/
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
	u32 rx_reserved4[(0x1ac - 0x178) / sizeof(u32)];
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
	u32 rx_reserved5[(0x200 - 0x1f8) / sizeof(u32)];
/*misc*/
	u32 rx_transcontrol;
	u32 rx_status_reg;
	u32 rx_irq_status;
	u32 rx_irq_enable;
	u32 rx_err_report;
	u32 rx_rcv_delay;
	u32 rx_diag_sel;
	u32 rx_diag_data;/* this is a read only register */
	u32 rx_reserved6[(0x400 - 0x220) / sizeof(u32)];
};

/* Tn_KF_ISL*/
#define MAX_ILAS_LEN_MASK	0xff

/* Tn_DID */
#define BID_MASK		0x0f
#define LID_MASK		0x1f
#define LANES_PER_CONV_MASK	0x1f

#define CS_MASK			0x03
#define CS_SHIFT		6

#define N_MASK			0x1f
#define N_SHIFT			0x0

#define SUBCLASS_MASK		0x07
#define SUBCLASS_SHIFT		5

#define VERSION_MASK		0x07
#define VERSION_SHIFT		5

#define NP_MASK			0x1f
#define NP_SHIFT		0

#define S_MASK			0x1f
#define S_SHIFT			0

#define HD_ENABLE		0x00000080

#define CF_MASK			0x1f
#define CF_SHIFT		0

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
	atomic_t ref;
	u32 flags;
	u8 mode;
	enum jesd_state dev_state;
	u32 transport_type;
	struct jesd204_dev *jesd_parent;
	struct tbgen_dev *tbg;
	struct device *dev;
	unsigned int identifier;
	struct lane_device *lane_devs[MAX_LANES];
	int max_lanes;
	int active_lanes;
	/* either of one will be a null pointer */
	struct config_registers_tx *tx_regs;
	struct config_registers_rx *rx_regs;

	struct tasklet_struct do_tx_tasklet;
	struct tasklet_struct do_rx_tasklet;
	spinlock_t lock_tx_bf;
	spinlock_t lock_rx_bf;

	u8 delay;
	u8 ilas_len;
	struct ils_params ils;
	unsigned int sampling_rate;

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

	struct cdev c_dev;
	dev_t devt;

	wait_queue_head_t isr_wait;
	wait_queue_head_t to_wait;
	int sysref_rose;
	int isr_error;
	int evnt_jiffs;
	spinlock_t sysref_lock;

};

struct jesd204_dev {
	struct transport_device *trans_device[TRANSPORT_PER_IP];
	u8 max_lanes;
	u8 lanes_grabed;
	u8 transport_count;
	u32 trans_type;
	struct device *dev;
	struct device_node *node;
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

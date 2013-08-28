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
#include <mach/serdes-d4400.h>

#define DRIVER_NAME "jesd204"

#define TRANSPORTS_PER_JESD_DEV 2
#define MAX_LANES 2
#define JESD_SYNC_TIMEOUT_MS	500

#define DEVICE_TX	0
#define DEVICE_RX	1
#define TRANSPORT_FULL_RATE	1
#define TRANSPORT_HALF_RATE	2
#define TRANSPORT_QUATER_RATE	4
#define TRANSPORT_DEV_BY_8	8

#define LINK_STATUS_CODE_GRP_SYNC_DONE	(1 << 0)
#define LINK_STATUS_FRM_SYNC_DONE	(1 << 1)
#define LINK_STATUS_CSUM_DONE		(1 << 2)
#define LINK_STATUS_ILAS_DONE		(1 << 3)
#define LINK_STATUS_CODE_FRP_SYNC_MASK	(LINK_STATUS_CODE_GRP_SYNC_DONE | \
					LINK_STATUS_FRM_SYNC_DONE)
#define LINK_STATUS_USER_DATA_MASK	(LINK_STATUS_CODE_FRP_SYNC_MASK | \
						LINK_STATUS_CSUM_DONE |	  \
						LINK_STATUS_ILAS_DONE)
struct ils_regs {
	u32 did;
	u32 bid;
	u32 lid;
	u32 scr_l;
	u32 octets_per_frame_f;
	u32 frames_per_mf_k;
	u32 conv_per_dev_m;
	u32 control_bits_per_sample;
	u32 bits_per_coverter_wrd_np;
	u32 samples_per_coverter_per_fs_s;
	u32 high_density;
	u32 reserved1;
	u32 reserved2;
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
	u32 tx_reserved;
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
#define ADJCNT_MASK		0x0f
#define ADJCNT_SHIFT		4

/* Tn_LID */
#define LID_MASK		0x1f
#define LANES_PER_CONV_MASK	0x1f
#define ADJDIR			(1 << 6)
#define PHYADJ			(1 << 5)

/* Tn_CS_N_REG */
#define CS_MASK			0x03
#define CS_SHIFT		6
#define N_MASK			0x1f
#define N_SHIFT			0x0

/* Tn_NP_REG */
#define NP_MASK			0x1f
#define NP_SHIFT		0
#define SUBCLASS_MASK		0x07
#define SUBCLASS_SHIFT		5

/* Tn_S_REG */
#define VERSION_MASK		0x07
#define VERSION_SHIFT		5
#define S_MASK			0x1f
#define S_SHIFT			0

#define HD_ENABLE		0x00000080

#define CF_MASK			0x1f
#define CF_SHIFT		0

/* Tn_control */
#define CLKDIV_MASK		0x07
#define CLKDIV_SHIFT		4
#define CLK_ENABLE		(1 << 0)
#define DFIFO_SWRESET		(1 << 3)
#define SYSREF_MASK		(1 << 20)
#define SYNC_SELECT_TBGEN	(1 << 21)
#define SYNC_PIPELINE_MASK	0x1ff
#define SYNC_PIPELINE_SHIFT	23
#define SW_DMA_ENABLE		(1 << 17)
#define SW_DMA_ENABLE_SELECT	(1 << 16)
#define OTHR_TRANS_IDLE_SELECT	(1 << 1)
#define SWAP_IQ			(1 << 7)
#define PHYPACK_MS_OCTECT_FIRST	(1 << 9)
#define PHYORDER_MS_BIT_FIRST	(1 << 8)
#define STRICT_CGS		(1 << 19)

/* Tn_Lane_enable */
#define LANE_EN_MASK		0xff

/* Tn_M_enable */
#define M_EN_MASK		0x3

/* Tn_IRQ_STATUS & IRQ_ENABLE*/
#define IRQ_SYSREF_ROSE			(1 << 8)
#define IRQ_SYNC_RECIEVED		(1 << 9)
#define IRQ_DEFRAMER_IRQ		(1 << 0)
#define IRQ_TX_UNPACKER_UNDERFLOW	(1 << 3)
#define IRQ_TX_WCBUF_OVERFLOW		(1 << 4)
#define IRQ_RX_PACKER_UNDERLOW		(1 << 3)
#define IRQ_RX_RCBUF_OVERFLOW		(1 << 4)
#define IRQ_RX_RCBUF_UNDERFLOW		(1 << 5)
#define IRQ_RX_SFIFO_OVERFLOW		(1 << 6)

#define TX_IRQS_EN_MASK		IRQ_SYSREF_ROSE
#define RX_IRQS_EN_MASK		IRQ_SYSREF_ROSE

/* TN_IRQ_VECTOR_MASK */
#define DFRMR_IRQ_CGS		(1 << 0)
#define DFRMR_IRQ_FS		(1 << 1)
#define DFRMR_IRQ_GCS		(1 << 2)
#define DFRMR_IRQ_ILS		(1 << 3)
#define DFRMR_IRQ_ILD		(1 << 4)
#define DFRMR_IRQ_UNEX_K	(1 << 5)
#define DFRMR_IRQ_NIT_DIS	(1 << 6)
#define DFRMR_IRQ_BAD_DIS	(1 << 7)

/* STATUS flag registers (T0_CODE_GRP_SYNC,
 * T0_FRAME_SYNC_FLG, T0_GOOD_CHK_SUM_FLG
 * T0_INIT_LANE_SYNC_FLA
 */
#define DFRMR_IRQ_RESET		(1 << 7)
#define DFRMR_STATUS_FLG_SET	(1 << 0)
#define DFRMR_STATUS_MASK	0x3

/* Tn_frm_ctl*/
#define L2SIDES_EN		(1 << 0)
#define TRANSMIT_EN		(1 << 1)
#define BYP_ILAS		(1 << 2)
#define BYP_AGC			(1 << 3)
/* Tn_CTRLREG0*/
#define FSREQ			(1 << 2)
#define RX_DIS			(1 << 7)

/* Tn_DIAG_SEL and Tn_DIAG*/
#define DIAG_FRAMER_STATE_SEL	2
#define FRAMER_STATE_CODE_GRP_SYNC	0x0
#define FRAMER_STATE_ILAS		0x1
#define FRAMER_STATE_USER_DATA		0x2
#define FRAME_STATE_MASK		0x3
#define FRAME_STATE_RESERVED		0x3

/* Tn_SCR_IN_CTRL */
#define L0_SCR_EN	(1 << 0)
#define L1_SCR_EN	(1 << 1)

/* Tn_L_SCR */
#define SCR_EN			(1 << 7)
#define LANE_COUNT_MASK		0x1f

/* Tn_SYNC_ASSERT_MASK_CTL */
#define UNEX_K_S	(1 << 5)
#define NIT_DIS_S	(1 << 6)
#define BAD_DIS_S	(1 << 7)
#define ERR_THRESHOLD_MASK	0xff

/* Tn_FRM_TEST */
#define BYP_8B10B	(1 << 0)
#define REV_FRM_DOUT	(1 << 1)

/* Tn_JDEC_TEST */
#define TRANSPORT_TEST_SEQ_EN		(1 << 0)
#define DATA_LINK_TEST_PATTRN_EN	(1 << 1)

/* Tn_CTRL_REG2 */
#define AUTO_ECNTR_RESET	(1 << 3)
#define QUEUE_TEST_ERR		(1 << 4)
#define REP_DATA_TEST		(1 << 5)
#define ILS_TEST_MODE		(1 << 7)

/* checksum */
#define CSUM_MASK		0xff

/**/
struct lane_device {
	struct jesd_transport_dev *tdev;
	struct lane_stats l_stats;
	enum srds_lane_id serdes_lane_id;
	u8 id;
	u32 flags;
};
/*Lane Flags*/
#define LANE_FLAGS_ENABLED	(1 << 0)
#define LANE_FLAGS_FIRST_LANE	(1 << 1)
#define LANE_FLAGS_PRIMARY	(1 << 2)

/** @brief /stuct jesd devices to be created for each transport
 *	hence the instance is defined in here.
 */
struct jesd_transport_dev {
	char name[32];
	atomic_t ref;
	enum jesd_state state;
	enum jesd_state old_state;
	enum jesd_dev_type type;
	u32 config_flags;
	u32 config_bitmap;
	u32 dev_flags;
	u32 test_mode_flags;
	struct jesd204_dev *parent;
	void *tbgen_dev_handle;
	void *serdes_handle;
	void *timer_handle;
	void *txalign_timer_handle;
	struct work_struct link_monitor;
	int timer_id;
	struct device *dev;
	unsigned int id;
	struct lane_device *lane_devs[MAX_LANES];
	int max_lanes;
	int active_lanes;
	/* either of one will be a null pointer */
	struct config_registers_tx *tx_regs;
	struct config_registers_rx *rx_regs;

	struct tasklet_struct tx_tasklet;
	struct tasklet_struct rx_tasklet;
	spinlock_t lock;

	u8 delay;
	u8 ilas_len;
	struct ils_params ils;
	unsigned int data_rate;

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
	unsigned long sync_expire;
	unsigned long sync_timeout_ms;
};

/*dev_flags*/
#define DEV_FLG_NO_TRANSPORT_EVENTS	(1 << 0)
#define DEV_FLG_PHYGASKET_LOOPBACK_EN	(1 << 1)
#define DEV_FLG_SERDES_LOOPBACK_EN	(1 << 2)
#define DEV_FLG_TEST_PATTERNS_EN	(1 << 3)
#define DEV_FLG_LOOPBACK_MASK	(DEV_FLG_PHYGASKET_LOOPBACK_EN | \
				DEV_FLG_SERDES_LOOPBACK_EN)
/*config_bitmap*/
#define JESD_CONF_ILS_LEN_INIT		(1 << 0)
#define JESD_CONF_ILS_INIT		(1 << 1)
#define JESD_CONF_DEV_INIT		(1 << 2)
#define JESD_CONFIGURED_MASK		(JESD_CONF_ILS_LEN_INIT |	\
					JESD_CONF_ILS_INIT |		\
					JESD_CONF_DEV_INIT)
#define JESD_SET_CONFIG_MASK(tdev, mask) (tdev->config_bitmap |= mask)
#define JESD_CLR_CONFIG_MASK(tdev, mask) (tdev->config_bitmap &= ~mask)
#define JESD_CHCK_CONFIG_MASK(tdev, mask) (tdev->config_bitmap & mask)

struct jesd204_dev {
	struct jesd_transport_dev *transports[TRANSPORTS_PER_JESD_DEV];
	u8 max_lanes;
	u8 used_lanes;
	u8 transport_count;
	u32 trans_type;
	struct device *dev;
	struct device_node *node;
	dev_t devt;
	struct list_head jesd_dev_list;
};

struct jesd204_private {
	struct list_head list;
	u8 tx_dev_num;
	u8 rx_dev_num;
};

extern void gcr_jesd_init(void);
void jesd_enable_sysref_capture(struct jesd_transport_dev *tdev);
int  jesd_reg_dump_to_user(u32 *reg, unsigned int offset,
			unsigned int length, u32 *buf);
#endif

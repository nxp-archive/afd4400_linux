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
#include <uapi/linux/tbgen.h>
#include <mach/serdes-d4400.h>

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

/** @brief /stuct jesd devices to be created for each transport
 *	hence the instance is defined in here.
 */
struct jesd_transport {
	struct jesd_complex *jdev;
	struct completion sysref_completion;
	int id;
	atomic_t state;
	__iomem void *regs;
	int irq;
};

enum jesd_complex_type {
	JESD_TX,
	JESD_RX,
};

struct jesd_complex {
	char name[32];
	enum jesd_complex_type type;
	u32 complex_id;
	u32 pll_id;
	struct jesd_transport *tdev[2];
	struct phygasket *phy;
	struct device *dev;
	struct device_node *node;
	void *serdes_handle;
	u32 first_serdes;
	struct cdev cdev;
	struct list_head list;
	struct jesd_complex_params params;
	dev_t devt;
	u32 debug;
};

#endif

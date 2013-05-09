#ifndef TBGEN_H_	/* prevent circular inclusion */
#define TBGEN_H_	/* by using protection macros */
/*
 * include/linux/tbgen.h
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <uapi/linux/tbgen.h>

#define TBGEN_DRIVER_NAME "tbgen"

/* a total of 10 timers are supported now */
#define MAX_TX_ALIGNMENT_TIMERS	10
#define MAX_AXRF_ALIGNMENT_TIMERS 10
#define MAX_RX_ALIGNMENT_TIMERS 4
#define MAX_SRX_ALIGNMENT_TIMERS 4
#define MAX_SPI_ALIGNMENT_TIMERS 8
#define MAX_AGC_ALIGNMENT_TIMERS 8
#define MAX_TIMED_INTERRUPT_TIMER_CTRLS 12
#define MAX_GP_EVENT_TIMERS 8
#define MAX_TDD_TIMERS 8

#define JESD204_VERSION_A			1
#define JESD204_VERSION_B			2

#define FIRST_PASS				1
#define SECOND_PASS				2

#define TB_REF_614_4				0
#define TB_REF_983_04				1

/**@brief bit bangs
 *
*/
/*>! bang it for RFGCR reg*/
#define RFGEN_BIT0				0
#define REFSYNCSEL_BIT2			2
#define SYNCOUT_CTRL_BIT18		18

/*>! bang it for ctrl_1 reg*/
#define FSIE_BIT5				5
#define RFGERRIE_BIT4				4
/*>! bang it for tx ctrl reg*/
#define POLARITY_BIT7			7
#define SYNCERR_BIT3			3
#define ISYNCBYP_BIT2			2
#define OUTSEL_BIT1			1
#define ENBALE_TMR_BIT0			0
/*>! bancg for software reset*/
#define SWRST_BIT0			0

/**@brief
 * interrupt status bits
*/
#define TBG_RFGERIEB	0x00000010
#define TBG_FISE	0x00000020
#define TBG_TIIE	0x00000040
#define TBG_SSRIE	0x00000080
#define TBG_SERIE	0x01000000
#define TBG_RSYNCCIE	0x02000000


/** \enum tbg_mode
 *  @brief mode of tbgen
 */
enum tbg_mode {
	one_shot,
	continuous,
};

/** @brief tx timer configuration
 *
 */
struct tx_timer_conf {
	u32 timer_id;
	u64 tx_offset;
	u8  ref_frm_clk;
};
/** @brief rx timer configuration
 *
 */
struct rx_timer_conf {
	u32 timer_id;
	u64 rx_offset;
	u8  strobe_mode;
	u8  oneshot_mode;
};
/** @brief srx timer configuration
 *
 */
struct srx_timer_conf {
	u32 timer_id;
	u64 srx_offset;
	u32 alignment_interval;
	u8  strobe_mode;
	u8  oneshot_mode;
	u8  tmr_pulsewidth;
};
/** @brief axrf timer configuration
 *
 */
struct axrf_timer_conf {
	u32 timer_id;
	u64 tx_offset;
	u8  ref_frm_clk;
};


/** \struct tbgen_txtimer
 *  @brief structure for txtimer instances
 */
struct tbgen_txtimer {
	spinlock_t lock_tx_tmr;
	struct tx_timer_conf tmr;
	u8 configured;
/*	struct transport_device *tdev;*/
};
/** \struct tbgen_rxtimer
 *  @brief structure for txtimer instances
 */
struct tbgen_rxtimer {
	spinlock_t lock_rx_tmr;
	struct rx_timer_conf tmr;
	u8 configured;
/*	struct transport_device *tdev;*/
};
/** \struct tbgen_srxtimer
 *  @brief structure for txtimer instances
 */
struct tbgen_srxtimer {
	spinlock_t lock_srx_tmr;
	struct srx_timer_conf tmr;
	u8 configured;
/*	struct transport_device *tdev;*/
};
/** \struct tbgen_axrftimer
 *  @brief structure for txtimer instances
 */
struct tbgen_axrftimer {
	spinlock_t lock_axrf_tmr;
	struct axrf_timer_conf tmr;
	u8 configured;
};
/** \struct tbg_dev device structure
 *  @brief structure for the tbgen device
 */
struct tbgen_dev {
	__iomem struct tbg_regs *tbgregs;
	struct device_node *node;
	struct device *dev;
	struct resource res;
	u32 mode;
	void __iomem *base;
	int tbgen_major;
	int tbgen_minor;
	dev_t devt;
	struct cdev c_dev;
	struct tbgen_param tbg_param;
	struct tbg_rfg rfg;
	struct tbgen_txtimer tbg_txtmr[MAX_TX_ALIGNMENT_TIMERS];
	struct tbgen_axrftimer tbg_tx_axrf[MAX_AXRF_ALIGNMENT_TIMERS];
	struct tbgen_rxtimer tbg_rxtmr[MAX_RX_ALIGNMENT_TIMERS];
	struct tbgen_srxtimer tbg_srxtmr[MAX_SRX_ALIGNMENT_TIMERS];

	int irq;
	struct irq_conf c_irq;
	struct tasklet_struct do_tasklet;
	spinlock_t isr_lock;
	spinlock_t do_lock;
	u32 ien;
	enum tbgen_dev_state state;

	u8 mon_rfg_isr;
};
/** \struct alignment timer
 *  @brief base structure definition, derivatives of this struct would used for
 *	   register structure contruction
 */
struct align_timer {
	u32 alig_ctrl;
	u32 res;
	u32 align_oset_hi;
	u32 align_oset_lo;
};
/** \struct generic timer register strucutre
 *  @brief gerenric timer structure definition, derivatives of this struct
 *  would used for register structure contruction
 */
struct gen_timer_reg {
	u32 alig_ctrl;
	u32 align_intrvl;
	u32 align_oset_hi;
	u32 align_oset_lo;
};
/** \struct TDD strucutre
 *  @brief TDD structure definition, derivatives of this struct
 *  would used for register structure contruction
 */
struct tdd {
	u32 ctrl;
	u32 oset_hi;
	u32 oset_lo;
	u32 mode;
	u32 duration0;
	u32 duration1;
	u32 duration2;
	u32 duration3;
	u32 duration4;
	u32 duration5;
	u32 duration6;
	u32 duration7;
	u32 duration8;
	u32 duration9;
	u32 duration10;
	u32 duration11;
	u32 duration12;
	u32 duration13;
	u32 duration14;
	u32 duration15;
};
/** \struct tbg_regs register structure
 *  @brief this is tbgen register set structure include the reserved if any.
 *  clock set - tx alignment 10 timers expected to offset from 0x030 - 0xcc
 *  clock set - AXRF alignment expected to offset from 0x0D0 - 0x16C
 *  clock set - RX alignment expected to offset from 0x170 - 0x1AC
 *  clock set - sRX alignment expected to offset from 0x1B0 - 0x1EC
 *  clock set - SPI alignment expected to offset from 0x1F0 - 0x26C
 *  clock set - AGC alignment expected to offset from 0x270 - 0x2EC
 *  clock set - timed interrupt alignment expected to offset from 0x270 - 0x2EC
 *  clock set - general purpose alignment expected to offset from 0x270 - 0x2EC
 */
struct tbg_regs {
	u32 rfg_cr;
	u32 const rfg_err;
	u32 cntrl_0;
	u32 cntrl_1;
	u32 int_stat;
	u32 ti_stat;
	u32 sync_stat;
	u32 const ts_10_ms_hi;
	u32 const ts_10_ms_lo;
	u32 const tsync_hi;
	u32 const tsync_lo;
	u32 refclk_per_10ms;
	struct align_timer tx_tmr[MAX_TX_ALIGNMENT_TIMERS];
	struct align_timer axrf_tmr[MAX_AXRF_ALIGNMENT_TIMERS];
	struct gen_timer_reg  rx_tmr[MAX_RX_ALIGNMENT_TIMERS];
	struct gen_timer_reg  srx_tmr[MAX_SRX_ALIGNMENT_TIMERS];
	struct gen_timer_reg  spi_tmr[MAX_SPI_ALIGNMENT_TIMERS];
	struct gen_timer_reg  agc_tmr[MAX_AGC_ALIGNMENT_TIMERS];
	struct gen_timer_reg  titc_tmr[MAX_TIMED_INTERRUPT_TIMER_CTRLS];
	struct gen_timer_reg  gp_tmr[MAX_GP_EVENT_TIMERS];
/*reserved*/
	u32 reserved[(0x44f - 0x440) / sizeof(u32)];
/* tdd*/
	struct tdd	tdd_tmr[MAX_TDD_TIMERS];
/*gp reg set */
	u32 const tsgp_0_hi;
	u32 const tsgp_0_lo;
	u32 const tsgp_1_hi;
	u32 const tsgp_1_lo;
	u32 const tsgp_2_hi;
	u32 const tsgp_2_lo;
	u32 const tsgp_3_hi;
	u32 const tsgp_3_lo;
	u32 const mstr_cnt_hi;
	u32 const mstr_cnt_lo;
	u32 ssycn_oset_hi;
	u32 ssycn_oset_lo;
	u32 debug;
};

extern struct tbgen_dev *get_tbgen_device(void);
extern u32 get_ref_clock(struct tbgen_dev *tbg);
#endif

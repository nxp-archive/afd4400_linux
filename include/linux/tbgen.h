#ifndef __TBGEN_H__
#define __TBGEN_H__
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
#define TBG_RESET_TIMEOUT_MS	20
#define JESD204_VERSION_A			1
#define JESD204_VERSION_B			2

struct tbgen_timer {
	struct tbgen_dev *tbg;
	spinlock_t lock;
	struct timer_param timer_param;
	u32 status_flags;
	enum timer_type type;
	unsigned int id;
	void *jesd_dev_handle;
};

/* status flags */
#define STATUS_FLG_ENABLED			(1 << 0)
#define STATUS_FLG_CONFIGURED			(1 << 1)
#define STATUS_FLG_CAPTURE_SYSREF		(1 << 2)
#define STATUS_FLG_JESD_ATACHED			(1 << 3)
#define SET_STATUS_FLAG(timer, flag) (timer->status_flags |= flag)
#define CLEAR_STATUS_FLAG(timer, flag) (timer->status_flags &= ~flag)
#define CHECK_STATUS_FLAG(timer, flag) (timer->status_flags & flag)

enum sync_state {
	SYNC_CPRI_RX_RFP = 1,
	SYNC_SYSREF_IN_CONFIGURE,
	SYNC_SYSREF_IN_CONFIGURED,
	SYNC_INVALID
};

/** \struct tbg_dev device structure
 *  @brief structure for the tbgen device
 */

struct tbgen_dev {
	enum tbgen_dev_state state;
	enum sync_state sync_state;
	atomic_t ref;
	u32 dev_flags;
	u32 config_bitmap;
	u32 mode;
	u32 refclk_khz;
	u64 last_10ms_counter;
	struct device_node *node;
	struct device *dev;
	__iomem struct tbg_regs *tbgregs;
	int irq;
	struct tbgen_timer tbg_txtmr[MAX_TX_ALIGNMENT_TIMERS];
	struct tbgen_timer tbg_tx_axrf[MAX_AXRF_ALIGNMENT_TIMERS];
	struct tbgen_timer tbg_rxtmr[MAX_RX_ALIGNMENT_TIMERS];
	struct tbgen_timer tbg_srxtmr[MAX_SRX_ALIGNMENT_TIMERS];
	struct tbgen_timer tbg_titc_tmr[MAX_TIMED_INTERRUPT_TIMER_CTRLS];
	struct tbgen_timer tbg_gptmr[MAX_GP_EVENT_TIMERS];

	struct tasklet_struct tasklet;
	raw_spinlock_t          wait_q_lock;
	wait_queue_head_t       wait_q;
	spinlock_t lock;
	u32 ien;
	u8 mon_rfg_isr;
};

/* tbgen_dev->flags */
#define FLG_NO_INTERRUPTS	(1 << 0)
#define FLG_SYNC_CPRI_10ms	(1 << 1)
#define FLG_SYNC_INTERNAL_10ms	(1 << 2)

/* tbgen_dev->config_bitmap */
#define TBG_PLL_CONFIGURED	(1 << 0)
#define TBG_RFG_CONFIGURED	(1 << 1)
#define TBG_PLL_READY		(1 << 2)
#define TBG_RFG_READY		(1 << 3)
#define TBG_CONFIGURED_MASK	(TBG_PLL_CONFIGURED | TBG_RFG_CONFIGURED)
#define TBG_READY_MASK		(TBG_PLL_READY | TBG_RFG_READY)

#define TBG_SET_CONFIG_MASK(tbg, mask) (tbg->config_bitmap |= mask)
#define TBG_CLR_CONFIG_MASK(tbg, mask) (tbg->config_bitmap &= ~mask)
#define TBG_CHCK_CONFIG_MASK(tbg, mask) (tbg->config_bitmap & mask)

/** \struct alignment timer
 *  @brief base structure definition, derivatives of this struct would used for
 *	   register structure contruction
 */
struct txalign_timer_regs {
	u32 ctrl;
	u32 reserved;
	u32 osethi;
	u32 osetlo;
} __packed;

/* Common for both Tx alignment and generic timers */

#define TMRCTRL_EN		(1 << 0)
#define TMRCTRL_POL_ACTIVE_LOW	(1 << 7)

/* Tx alignment ctrl */
#define TXCTRL_OUTSEL_TXIDLE_EN	(1 << 1)
#define TXCTRL_ISYNC_BYP	(1 << 2)
#define TXCTRL_SREPEN		(1 << 3)

/** \struct generic timer register strucutre
 *  @brief gerenric timer structure definition, derivatives of this struct
 *  would used for register structure contruction
 */
struct gen_timer_regs {
	u32 ctrl;
	u32 interval;
	u32 osethi;
	u32 osetlo;
} __packed;

/* generic timer CTRL */
#define GEN_CTRL_STRB_TOGGLE	0x00
#define GEN_CTRL_STRB_PULSE	0x01
#define GEN_CTRL_STRB_CYCLE	0x02
#define GEN_CTRL_STRB_MASK	0x03
#define GEN_CTRL_STRB_SHIFT	4

#define GEN_CTRL_ONESHOT		(1 << 6)
#define GEN_CTRL_PULSE_WIDTH_MASK	0x0f
#define GEN_CTRL_PULSE_WIDTH_SHIFT	8

struct tdd_regs {
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
} __packed;

struct tbg_regs {
	u32 rfg_cr;
	u32 rfg_err;
	u32 cntrl_0;
	u32 cntrl_1;
	u32 int_stat;
	u32 ti_stat;
	u32 sync_stat;
	u32 ts_10_ms_hi;
	u32 ts_10_ms_lo;
	u32 tsync_hi;
	u32 tsync_lo;
	u32 refclk_per_10ms;
	struct txalign_timer_regs tx_tmr[MAX_TX_ALIGNMENT_TIMERS];
	struct txalign_timer_regs axrf_tmr[MAX_AXRF_ALIGNMENT_TIMERS];
	struct gen_timer_regs  rx_tmr[MAX_RX_ALIGNMENT_TIMERS];
	struct gen_timer_regs  srx_tmr[MAX_SRX_ALIGNMENT_TIMERS];
	struct gen_timer_regs  spi_tmr[MAX_SPI_ALIGNMENT_TIMERS];
	struct gen_timer_regs  agc_tmr[MAX_AGC_ALIGNMENT_TIMERS];
	struct gen_timer_regs  titc_tmr[MAX_TIMED_INTERRUPT_TIMER_CTRLS];
	u32 reserved1[4];
	struct gen_timer_regs gp_tmr[MAX_GP_EVENT_TIMERS];
	u32 reserved[(0x450 - 0x440) / sizeof(u32)];
	struct tdd_regs	tdd_tmr[MAX_TDD_TIMERS];
	u32 tsgp_0_hi;
	u32 tsgp_0_lo;
	u32 tsgp_1_hi;
	u32 tsgp_1_lo;
	u32 tsgp_2_hi;
	u32 tsgp_2_lo;
	u32 tsgp_3_hi;
	u32 tsgp_3_lo;
	u32 mstr_cnt_hi;
	u32 mstr_cnt_lo;
	u32 ssycn_oset_hi;
	u32 ssycn_oset_lo;
	u32 debug;
};

#define REFCLK_PER_10MS_MASK		0xffffff

/* RFGCR */
#define RFG_ERR_THRESHOLD_MASK		0xfff
#define RFG_ERR_THRESHOLD_SHIFT		4

#define SYNC_OUT_SHIFT			18
#define SYNC_OUT_INTERNAL_10MS		(1 << SYNC_OUT_SHIFT)
#define SYNC_OUT_CPRI_RX_RFP		(0 << SYNC_OUT_SHIFT)

#define SYNC_SEL_SHIFT			2
#define SYNC_SEL_CPRI_RX_RFP		(1 << SYNC_SEL_SHIFT)
#define SYNC_SEL_SYSREF			(0 << SYNC_SEL_SHIFT)

#define SYNC_ADV_SHIFT			19
#define SYNC_ADV_MASK			0x1f

#define START_INTERNAL_RF_SYNC		(1 << 17)
#define RFGEN				(1 << 0)

/* ctrl_0 */
#define SWRESET				(1 << 0)
#define FRAME_SYNC_SEL_SHIFT		12
#define FRAME_SYNC_SEL_SYSREF		0x00
#define FRAME_SYNC_SEL_CPRI_RX_RFP	0x01
#define FRAME_SYNC_SEL_RFG_OUTPUT	0x02
#define FRAME_SYNC_SEL_MASK		0x03

/* ctrl_1 & INTSTAT */
#define IRQ_RFGER			(1 << 4)
#define IRQ_FS				(1 << 5)
#define IRQ_TI				(1 << 6)
#define IRQ_SSR				(1 << 7)
#define IRQ_SER				(1 << 24)
#define IRQ_RESYNC			(1 << 25)
#define IRQ_EN_MASK			(IRQ_FS | IRQ_TI)

struct tbgen_dev *get_tbgen_device(void);
u32 get_ref_clock(struct tbgen_dev *tbg);
struct tbgen_timer *tbgen_get_timer(struct tbgen_dev *tbg, enum timer_type type,
	int timer_id);

int tbgen_timer_enable(struct tbgen_timer *timer);
int tbgen_timer_set_sysref_capture(struct tbgen_timer *timer, int enabled);
int tbgen_attach_timer(struct tbgen_timer *timer, void *jesd_dev_handle);
int tbgen_timer_disable(struct tbgen_timer *timer);
int tbgen_set_sync_loopback(struct tbgen_timer *timer, int enable);
int rfg_recapture_frame_sync(struct tbgen_dev *tbg);
#endif

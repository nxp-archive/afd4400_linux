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
#include <linux/cdev.h>
#include <linux/types.h>
#include <uapi/linux/tbgen.h>
#define TBGEN_DRIVER_NAME "tbgen"

/* a total of 10 timers are supported now */
#define TBG_RESET_TIMEOUT_MS	20

enum tbg_timer_state {
	/* Timer is in disabled state with TMREN = 0 */
	TIMER_STATE_IDLE,
	/* Timer is in configured state */
	TIMER_STATE_READY,
	/* Timer is in running state after the user issued the start ioctl*/
	TIMER_STATE_RUNNING,
};

struct tbg_timer {
	struct list_head list;
	atomic_t state;
	enum tbg_timer_type type;
	int id;
	u64	offset;
};

struct tbgen_dev {
	atomic_t state;
	atomic_t rfg_in_transition;
	struct device_node *node;
	struct device *dev;
	__iomem struct tbg_regs *tbgregs;
	int irq;
	struct list_head timer_list;
	struct completion rfg_complete;
	struct tbg_config config;

	/* Sysfs interface */
	u32 debug;
	struct class *dev_class;
	struct cdev dev_cdev;
};

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

struct tdd_regs {
	u32 ctrl;
	u32 osethi;
	u32 osetlo;
	u32 mode;
	u32 duration[16];
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
	struct txalign_timer_regs tx_tmr[TX_ALIGNMENT_TIMERS_CNT];
	struct txalign_timer_regs axrf_tmr[TX_AXRF_TIMERS_CNT];
	struct gen_timer_regs  rx_tmr[RX_ALIGNMENT_TIMERS_CNT];
	struct gen_timer_regs  srx_tmr[SRX_ALIGNMENT_TIMERS_CNT];
	struct gen_timer_regs  spi_tmr[SPI_ALIGNMENT_TIMERS_CNT];
	struct gen_timer_regs  agc_tmr[AGC_ALIGNMENT_TIMERS_CNT];
	struct gen_timer_regs  titc_tmr[TIMED_INTERRUPT_TIMER_CTRLS_CNT];
	u32 reserved1[4];
	struct gen_timer_regs gp_tmr[GP_EVENT_TIMERS_CNT];
	u32 reserved[(0x450 - 0x440) / sizeof(u32)];
	struct tdd_regs	tdd_tmr[TDD_TIMERS_CNT];
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

extern void d4400_pinmux_hack(int index);
enum tbg_dev_state tbgen_get_state(void);
unsigned int tbgen_get_pll_freq(void);
int tbgen_set_sync_loopback(int lane, int loopback);

#endif

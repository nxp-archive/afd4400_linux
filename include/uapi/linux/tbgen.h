/*
 * include/uapi/linux/tbgen.h
 *
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _UAPI_TBGEN_H
#define _UAPI_TBGEN_H


struct tx_alignment_config {
/* valid id from 0 to 9 */
	int id;
/* tx alignment timer control register */
	__u8 ref_to_frm_clk_ratio;
	__u8 pol;
	__u8 serepen;
	__u8 isyncbyp;
	__u8 outsel;
/* timer offset relative to sysref in number of tbgen clk pulse */
	__u64 offset;
};

struct tx_axrf_config {
/* valid id from 0 to 9 */
	int id;
/* axrf timer control register */
	__u8 pol;
	__u64 offset;
};

struct rx_alignment_config {
/* valid id from 0 to 3 */
	int id;
/* rx alignment control reg */
	__u8 pulsewidth;
	__u8 pol;
	__u8 oneshot;
	__u8 strbmode;
/* interval value in number of clk pulse */
	__u64 interval;
	__u64 offset;
};

struct srx_alignment_config {
/* valid id from 0 to 3 */
	int id;
/* srx alignment control reg */
	__u8 pulsewidth;
	__u8 pol;
	__u8 oneshot;
	__u8 strbmode;
/* srx alignment interval reg */
	__u32 interval;
	__u64 offset;
};

struct gp_event_config {
/* valid id from 0 to 7 */
	int id;
/* gp event control reg */
	__u8 pulsewidth;
	__u8 pol;
	__u8 oneshot;
	__u8 strbmode;
/* gp event interval reg */
	__u32 interval;
	__u64 offset;
};

struct tdd_config {
/* valid id from 0 to 7 */
	int id;
/* tdd control reg */
	__u16 pulsewidth;
	__u8 pulse_mode;
	__u8 rxtxen;
	__u8 buflength;
	__u8 fdd_sel;
	__u8 rx_en_sel;
	__u8 contseq;
	__u64 offset;
/* tdd mode reg */
	__u8 mode[16];
/* duration reg */
	__u32 duration[16];
};

#define TX_ALIGNMENT_TIMERS_CNT 10
#define TX_AXRF_TIMERS_CNT 10
#define RX_ALIGNMENT_TIMERS_CNT 4
#define SRX_ALIGNMENT_TIMERS_CNT 4
#define GP_EVENT_TIMERS_CNT 8
#define TDD_TIMERS_CNT 8
#define SPI_ALIGNMENT_TIMERS_CNT 8
#define AGC_ALIGNMENT_TIMERS_CNT 8
#define TIMED_INTERRUPT_TIMER_CTRLS_CNT 12

struct tbg_timer_mask {
	__u16 tx_alignment_mask;
	__u16 tx_axrf_mask;
	__u8 rx_alignment_mask;
	__u8 srx_alignment_mask;
	__u8 gp_event_mask;
	__u8 tdd_mask;
};

/** \enum TBGEN device states enum.
 *
 */
enum tbg_timer_type {
	TBG_TX_ALIGNMENT,
	TBG_TX_AXRF,
	TBG_RX_ALIGNMENT,
	TBG_SRX_ALIGNMENT,
	TBG_GP_EVENT,
	TBG_TDD,
};

/** \struct jesd alignemnet timer params config
 *
 */
struct tbg_timer_params {
	enum tbg_timer_type type;
	void *timer_config;
};

struct tbg_config {
/* TBGEN PLL frequency */
	enum pll_freq_khz {
		REF_CLK_614400KHZ = 614400,
		REF_CLK_737280KHZ = 737280,
		REF_CLK_983040KHZ = 983040,
	} pll_freq_khz;
/* The SYSREF which the TBGEN will sync to */
	enum radio_frame_src {
		SELF_GENERATED_10MS,
		CPRI_ALIGNED_10MS,
	} radio_frame_src;
/* Use this flag then the TG_RF_SYNC pin is controlled by the
 * GP_EVENT_7 timer. Otherwise the TG_RF_SYNC will output the 10ms SYNC signal.
 */
	enum tbgen_flag_mask {
		TBGEN_RF_SYNC_GP_EVENT_7 = (1 << 0),
	} flag;
/* The SYNCADV field of the RFGCR reg field */
	__u8 sync_adv;
/* The user expected value in RFGERR reg, ignored when rfgerr_target < 0 */
	int rfgerr_target;
};

/** \enum TBGEN device states enum.
 *
 */
enum tbg_dev_state {
	/* state after board power up */
	TBG_STATE_IDLE,
	/* TBGEN PLL is running */
	TBG_STATE_PLL_RUNNING,
	/* TBGEN PLL and radio frame generator is running */
	TBG_STATE_RFG_RUNNING,
	/* TBGEN PLL failure */
	TBG_STATE_PLL_ERR,
	/* radio frame generator failure */
	TBG_STATE_RFG_ERR,
};

struct tbg_device_info {
	enum tbg_dev_state state;
	enum pll_freq_khz pll_freq_khz;
};

#define TBGEN_NUM 't'

#define TBGEN_SET_PLL_RFG		_IOW(TBGEN_NUM, 0, struct tbg_config*)

#define TBGEN_CONFIG_TIMER		_IOW(TBGEN_NUM, 1, \
						struct tbg_timer_params*)

#define TBGEN_TIMERS_START		_IO(TBGEN_NUM, 2)

#define TBGEN_TIMERS_STOP		_IOW(TBGEN_NUM, 3, \
						struct tbg_timer_mask*)

#define TBGEN_GET_DEV_INFO		_IOR(TBGEN_NUM, 4, \
						struct tbg_device_info*)

#endif /* _UAPI_TBGEN_H */

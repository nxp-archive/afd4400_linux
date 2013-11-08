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

#ifndef TBGEN_UAPI_H_	/* prevent circular inclusion */
#define TBGEN_UAPI_H_	/* by using protection macros */

/** \enum TBGEN device states enum.
 *
 */
enum timer_type {
	JESD_TX_ALIGNMENT,
	JESD_TX_AXRF,
	JESD_RX_ALIGNMENT,
	JESD_SRX_ALIGNMENT,
	JESD_TITC_ALIGNMENT,
	JESD_GP_EVENT,
};
/** \enum TBGEN device states enum.
 *
 */
enum tbgen_dev_state {
	TBG_STATE_STANDBY,
	TBG_STATE_CONFIGURED,
	TBG_STATE_READY,
	TBG_STATE_PLL_FAILED,
	TBG_STATE_RFG_FAILED,
	TBG_STATE_RFG_RESET,
	TBG_STATE_INVALID
};

enum strobe_mode {
	STROBE_TOGGLE,
	STROBE_PULSE,
	STROBE_CYCLE
};

/** \struct timer params
 *   timer params for the alignment timer of jesd204
 */
struct timer_param {
	__u32 config_flags;
	enum strobe_mode strobe_mode;
	__u32 interval;
	__u8 pulse_width;
	__u64 offset;
};

/* config flags */
#define TIMER_CONF_ACTIVE_LOW			(1 << 0)
#define TIMER_CONF_SYNC_ERR_DETECT_EN		(1 << 1)
#define TIMER_CONF_SYNC_BYPASS			(1 << 2)
#define TIMER_CONF_OUTSEL_TX_IDLE_EN		(1 << 3)
#define TIMER_CONF_ONE_SHOT			(1 << 4)
#define TIMER_CONF_ENABLE_TIMER			(1 << 5)

/** \struct jesd alignemnet timer params config
 *
 */
struct alignment_timer_params {
	uint32_t id;
	enum timer_type type;
	struct timer_param timer;
};

/** \struct tbg_pll
 *  @brief structure for PLL
 */
struct tbg_pll {
	__u32 refclk_khz;
	__u8 clk_src;
};

/* Refclk values in KHZ */
#define REF_CLK_614MHZ          614400
#define REF_CLK_983MHZ          983040
#define REF_CLK_122MHZ          122880

#define TBG_CLK_SRC_DEVCLK	1
#define TBG_CLK_SRC_SGMII	2

/** \struct tbg_rfg
 * params for rfg init
 */
struct tbg_rfg {
	__u32 ctrl_flags;
	__u32 drift_threshold;
	__u8 sync_adv;
};

/*ctrl_flags*/
#define CTRL_FLG_SYNC_CPRI_10ms		(1 << 0)
#define CTRL_FLG_SYNC_INTERNAL_10ms	(1 << 1)

/** \struct register configuration
*/
struct tbgen_reg_read_buf {
	__u32  offset;
	__u32  len;
	__u32  *buf;
};

struct tbgen_wreg {
	__u32 offset;
	__u32 value;
};

struct tbgen_reg_write_buf {
	struct tbgen_wreg *regs;
	__u32 count;
};
/** \struct timer_ctrl
 *  @brief for disabling the timer under instance
 */
struct timer_ctrl {
	__u8 id;
	__u8 enable;
	enum timer_type type;
};
/** \struct tbgen_device_info
 *  @brief structure for txtimer instances
 */
struct tbgen_device_info {
	enum tbgen_dev_state state;
};
/** @ brief ioctl instances
 * tbgen_num is 't'
 */
#define TDGEN_NUM 't'
#define IOCTL_IDX	0x0

#define TBGEN_CONFIG_TIMER		_IOW(TDGEN_NUM, (IOCTL_IDX + 1), \
						struct alignment_timer_params*)
#define TBGEN_TIMER_CTRL		_IOW(TDGEN_NUM, (IOCTL_IDX + 2), \
						struct timer_ctrl*)
#define TBGEN_SET_PLL			_IOW(TDGEN_NUM, (IOCTL_IDX + 4), \
						struct tbg_pll)
#define TBGEN_RFG_INIT			_IOW(TDGEN_NUM, (IOCTL_IDX + 5), \
						struct tbg_rfg)
#define TBGEN_RFG_RESET			_IOW(TDGEN_NUM, (IOCTL_IDX + 6), \
						struct tbg_rfg)
#define TBGEN_RFG_ENABLE		_IO(TDGEN_NUM, (IOCTL_IDX + 7))
#define TBGEN_WRITE_REG			_IOW(TDGEN_NUM, (IOCTL_IDX + 8), \
						struct tbgen_reg_write_buf*)
#define TBGEN_READ_REG			_IOR(TDGEN_NUM, (IOCTL_IDX + 9), \
						struct tbgen_reg_read_buf*)
#define TBGEN_GET_MASTER_COUNTER	_IOR(TDGEN_NUM, (IOCTL_IDX + 10), __u64)
#define TBGEN_GET_L10MCNTR		_IOR(TDGEN_NUM, (IOCTL_IDX + 11), __u64)
#define TBGEN_GET_DEV_INFO		_IOR(TDGEN_NUM, (IOCTL_IDX + 12), \
						struct tbgen_device_info*)
#define TBGEN_RESET			_IO(TDGEN_NUM, (IOCTL_IDX + 13))
#define TBGEN_RECAPTURE_FRAME_SYNC	_IO(TDGEN_NUM, (IOCTL_IDX + 14))
#define TBGEN_GET_STATE			_IOR(TDGEN_NUM, (IOCTL_IDX + 15), \
						int)
#endif

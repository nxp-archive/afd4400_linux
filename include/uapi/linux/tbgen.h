#ifndef TBGEN_UAPI_H_	/* prevent circular inclusion */
#define TBGEN_UAPI_H_	/* by using protection macros */
/*
 * include/uapi/linux/tbgen_ioctl.h
 *
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
/** \enum TBGEN device states enum.
 *
 */
enum timer_type {
	JESD_TX_ALIGNMENT,
	JESD_TX_AXRF,
	JESD_RX_ALIGNMENT,
	JESD_SRX_ALIGNMENT,
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
};
/** \struct timer params
 *   timer params for the alignment timer of jesd204
 */
struct timer_param {
	uint8_t enable; /*this shall be a bool type 0 = disable 1 = enable*/
	uint8_t jxstrobe;
	uint8_t perodic;
	uint32_t perodic_int;
};
/** \struct jesd alignemnet timer params config
 *
 */
struct jesd_align_timer_params {
	uint32_t id;
	enum timer_type type;
	uint8_t jesd_no;
	struct timer_param timer;
	uint32_t jx_start_tmr;
};

/** \struct irq_conf
 *  @brief structure instance for interrupt configuration
 */
struct irq_conf {
	__u8 rfg_err_ie;
	__u8 frm_sync_ie;
	__u8 re_sync_ie;
};
/** \struct tbg_pll
 *  @brief structure for PLL
 */
struct tbg_pll {
	__u8 pll_mode; /*0 = 614.4, 1 = 983.04*/
	__u8 clock_src; /*0 = devclk 1, 1 = sgmii*/
	__u8 tbgen_in_standby; /*0 = disable pll 1=ebable pll in standy*/
};
/** \struct tbgen params
 * params for tbgen clock configuration and pll
 */
struct tbgen_param {
	struct tbg_pll tbgen_pll;
};
/** \struct tbg_rfg
 * params for rfg init
 */
struct tbg_rfg {
	__u32 drift_threshold;
	__u8 ref_syncsel;
	__u8 sync_adv;
};
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
/** \struct timer_disable
 *  @brief for disabling the timer under instance
 */
struct timer_disable {
	uint8_t	timer_identifier;
	enum timer_type tmr_type;
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
#define TBGEN_ALIGNMENT_TIMERS		_IOW(TDGEN_NUM, 0x801, \
						struct jesd_align_timer_params*)
#define TBGEN_REALIGNMENT_TIMERS	_IOW(TDGEN_NUM, 0x802, \
						struct jesd_align_timer_params*)
#define TBGEN_ISR_MASK			_IOW(TDGEN_NUM, 0x803, \
						struct irq_conf)
#define TBGEN_SET_PLL			_IOW(TDGEN_NUM, 0x804, \
						struct tbgen_param)
#define TBGEN_RFG			_IOW(TDGEN_NUM, 0x805, \
						struct tbg_rfg)
#define TBGEN_RFG_RESET			_IOW(TDGEN_NUM, 0x806, \
						struct tbg_rfg)
#define TBGEN_RFG_ENABLE		_IO(TDGEN_NUM, 0x807)
#define TBGEN_WRITE_REG			_IOW(TDGEN_NUM, 0x808, \
						struct tbgen_reg_write_buf*)
#define TBGEN_READ_REG			_IOR(TDGEN_NUM, 0x809, \
						struct tbgen_reg_read_buf*)
#define TBGEN_DISABLE_TIMER_INSTANCE	_IOW(TDGEN_NUM, 0x80a, \
						struct timer_disable*)
#define TBGEN_GET_MASTER_COUNTER	_IOR(TDGEN_NUM, 0x80b, __u64)
#define TBGEN_GET_L10MCNTR		_IOR(TDGEN_NUM, 0x80c, __u64)
#define TBGEN_GET_DEV_INFO		_IOR(TDGEN_NUM, 0x80d, \
						struct tbgen_device_info*)
#define TBGEN_RESET			_IO(TDGEN_NUM, 0x80e)
#endif

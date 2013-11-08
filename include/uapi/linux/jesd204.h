/*
 * include/uapi/linux/jesd204.h
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

#ifndef JESD204_UAPI_H_	/* prevent circular inclusion */
#define JESD204_UAPI_H_	/* by using protection macros */

#define IQ_RATE_153M		153600
#define IQ_RATE_122M		122880

/** @brief the state for jesd204
 * a signle state determining enum
 */
enum jesd_state {
	JESD_STATE_STANDBY,
	JESD_STATE_CONFIGURED,
	JESD_STATE_ENABLED,
	JESD_STATE_CODE_GRP_SYNC,
	JESD_STATE_ILAS,
	JESD_STATE_READY,
	JESD_STATE_SYNC_FAILED,
	JESD_STATE_LINK_ERROR,
	JESD_STATE_STOPPED,
	JESD_STATE_INVALID
};

enum jesd_dev_type {
	JESD_DEV_TX,
	JESD_DEV_RX,
	JESD_DEV_SRX,
	JESD_DEV_INVAL,
};
struct jesd_dev_params {
	__u8 delay;
	__u32 config_flags;
	__u32 lanes;
	int ilas_length;
};

/* config_flags */
#define CONF_SCRAMBLING_EN		(1 << 0)
#define CONF_LANE_SYNC_BOTH_SIDES_EN	(1 << 1)
#define CONF_BYPASS_AGC			(1 << 2)
#define CONF_BYPASS_ILAS		(1 << 3)
#define CONF_USE_OTHER_TRANSPORTS_IDLE	(1 << 4)
#define CONF_SWAP_IQ			(1 << 5)
#define CONF_PHY_MS_OCTET_FIRST_EN	(1 << 6)
#define CONF_PHY_ORDER_MS_BIT_FIRST_EN	(1 << 7)
#define CONF_STRICT_CGS			(1 << 8)
#define CONF_PHYGASKET_LOOPBACK_EN	(1 << 16)
#define CONF_SERDES_LOOPBACK_EN		(1 << 17)
/** @brief \struct ils params
 */
/* data_rate */
#define DATA_RATE_1_2288_G		1228800
#define DATA_RATE_2_4576_G		2457600
#define DATA_RATE_3_0720_G		3072000
#define DATA_RATE_4_9152_G		4915200
#define DATA_RATE_6_1440_G		6144000
#define DATA_RATE_9_8304_G		9830400


struct ils_params {
	__u8 device_id;
	__u8 bank_id;
	__u8 lane0_id;
	__u8 lane1_id;
	__u8 scrambling_scr;
	__u8 lanes_per_converter_l;
	__u8 octect_per_frame_f;
	__u8 frame_per_mf_k;
	__u8 conv_per_device_m;
	__u8 ctrl_bits_per_sample;
	__u8 control_wrds_per_frame;
	__u8 converter_resolution;
	__u8 bits_per_converter;
	__u8 samples_per_cnvrtr_per_frame;
	__u8 high_density_enable;
	__u8 subclass_ver;
	__u8 jesd_ver;
	__u8 csum_lane_0;
	__u8 csum_lane_1;
	unsigned long data_rate;
};
/** @brief irq of mask
 */
union irq_mask {
	struct {
		__u32 res0:1;
		__u32 res1:1;
		__u32 pof:1;
		__u32 puf:1;
		__u32 rcof:1;
		__u32 rcuf:1;
		__u32 sfifo:1;
		__u32 res7:1;
		__u32 sysref_rose:1;
		__u32 res9:1;
		__u32 phy_data_lost:1;
		__u32 res10:1;
		__u32 eof:1;
		__u32 emof:1;
		__u32 res17:17;
	} msk;
	__u32 irq_en;
};

/** @brief irq of vector mask
 */
union irq_vmsk {
	struct {
		__u32 cgs:1;
		__u32 fs:1;
		__u32 gcs:1;
		__u32 ils:1;
		__u32 ild:1;
		__u32 unex_k:1;
		__u32 nit_dis:1;
		__u32 bad_dis:1;
		__u32 res23:23;
	} vmsk;
	__u32 irq_ven;
};
/** @brief irq of tx transport instance
 */
union irq_txuserconf {
	struct {
		__u32 reserved0:1;
		__u32 reserved1:1;
		__u32 unpacker_uf_err_tx:1;
		__u32 wcbuf_of_err_tx:1;
		__u32 wcbuf_uf_err_tx:1;
		__u32 reserved4:1;
		__u32 reserved5:1;
		__u32 sysref_rose_tx:1;
		__u32 sync_received_tx:1;
		__u32 reserved22:22;
	} irq_tx;
	__u32 irq_txconf;
};
/** @brief irq of rx transport instance
 */
struct irq_rxuserconf {
	union irq_mask irq_m;
	union irq_vmsk irq_vm;
};
/** @brief \struct isr conf instance
*/
struct isrconf {
	union irq_mask irq_m;
	union irq_vmsk irq_vm;
	union irq_txuserconf irq_tx;
};
/** @brief \struct for transport tests
 */
struct test_set {
	__u8 bipass_8b10:1;
	__u8 reverse_8b10:1;
	__u8 tpl:1;
	__u8 dll:1;
	__u8 s_rst:1;
	__u8 lane_ctrl:1;
};
/** @brief \struct for transport pattern gen tests
 */
struct patgen {
	__u32 rpt_ila:1;
	__u32 init_cgs:2;
	__u32 payload:3;
	__u32 disp:1;
	__u32 opmf:11;
};
/** @brief \struct for transport tx instance for all tests
 */
struct conf_tx_tests {
	struct test_set frmr_tst;
	struct patgen tx_patgen; /*pattern generator in test modes*/
	__u32 flag; /*for swap*/
};
/** @brief \struct for transport defrmr test
 */
struct test_defrmr {
	__u8 que_tst_err:1;
	__u8 rep_data_tst:1;
	__u8 ils_mode:1;
	__u8 loopback:1;
};
/** @brief \struct for transport rx instance for all tests
 */
struct conf_rx_tests {
	struct test_defrmr drfmr_tst;
	__u32 flags; /*for swap*/
};
/** @brief \struct for write register instance of an offset
 */
struct jesd_reg_val {
	__u32 offset;
	__u32 value;
};
/** @brief \struct accumulate all instances of write reg with offsets
 * and update the count for number of instances attached.
 */
struct jesd_reg_write_buf {
	struct jesd_reg_val *regs;
	__u32 count;
};
/** @brief \struct register read
*/
struct jesd_reg_read_buf {
	__u32 offset;
	__u32 len;
	__u32 *buf;
};
/** @brief \struct tansport device info
*/
struct jesd_transport_dev_info {
	enum jesd_state state;
	struct ils_params ils;
	struct jesd_dev_params init_params;
};
/** @brief \struct restart timer inputs
 * this inputs shall be from the tbgen
 */
struct tbgen_params {
	__u32 timer_id;
};
/** @brief \struct stats for the device transport
 */
struct transport_stats {
	__u32 ofpacker;
	__u32 ufpacker;
	__u32 ofrcbuf;
	__u32 ufrcbuf;
	__u32 ofwcbuf;
	__u32 ufwcbuf;
	__u32 ofsfifo;
	__u32 ufsfifo;
	__u32 unpacker;
};
/** @brief \struct stats for the device transport lanes
 */
struct lane_stats {
	__u32 baddis_threshold;
	__u32 nit_threshold;
	__u32 uexk_threshold;
	__u32 frm_sync_error;
	__u32 cgs_error;
	__u32 csum_error;
	__u32 ils_error;
	__u32 skwor;
	__u32 sync_failure_count;
};
/** @brief \struct stansport stats
*/
struct tarns_dev_stats {
	struct transport_stats *tstats;
	struct lane_stats *l0stats;
	struct lane_stats *l1stats;
};

struct auto_sync_params {
	uint8_t  sync_assert_mask;
	uint8_t	 error_threshold;
};

/* sync_assert_mask */
#define SYNC_ASSERT_BAD_DIS		(1 << 0)
#define SYNC_ASSERT_NIT			(1 << 1)
#define SYNC_ASSERT_UNEXPECTED_K_CHARS	(1 << 2)

#define JESD204_IOCTL 'j'
#define JESD_IOCTL_IDX	0x801

#define JESD_DEVICE_INIT	_IOW(JESD204_IOCTL, (JESD_IOCTL_IDX + 1), \
						struct jesd_dev_params *)
#define JESD_SET_LANE_PARAMS	_IOW(JESD204_IOCTL, (JESD_IOCTL_IDX + 2), \
						struct ils_params *)
#define JESD_SET_ILS_LENGTH	_IOW(JESD204_IOCTL, (JESD_IOCTL_IDX + 3), \
						unsigned int)
#define JESD_TX_TEST_MODE	_IOR(JESD204_IOCTL, (JESD_IOCTL_IDX + 6), \
						struct conf_tx_tests *)
#define JESD_RX_TEST_MODE	_IOR(JESD204_IOCTL, (JESD_IOCTL_IDX + 7), \
						struct conf_rx_tests *)
#define JESD_FORCE_SYNC		_IO(JESD204_IOCTL, (JESD_IOCTL_IDX + 8))
#define JESD_WRITE_REG		_IOW(JESD204_IOCTL, (JESD_IOCTL_IDX + 9), \
						struct jesd_reg_write_buf *)
#define JESD_READ_REG		_IOR(JESD204_IOCTL, (JESD_IOCTL_IDX + 10), \
						struct jesd_reg_read_buf *)
#define JESD_DEVICE_STOP	_IO(JESD204_IOCTL, (JESD_IOCTL_IDX + 11))
#define JESD_GET_DEVICE_INFO	_IOR(JESD204_IOCTL, (JESD_IOCTL_IDX + 12), \
					struct jesd_transport_dev_info *)
#define JESD_GET_STATS		_IOW(JESD204_IOCTL, (JESD_IOCTL_IDX + 13), \
						struct tarns_dev_stats *)
#define JESD_CLEAR_STATS	_IO(JESD204_IOCTL, (JESD_IOCTL_IDX + 14))
#define JESD_GET_LANE_RX_RECEIVED_PARAMS _IOW(JESD204_IOCTL,\
					(JESD_IOCTL_IDX + 15), \
					struct ils_params *)
#define JESD_RX_AUTO_SYNC	_IOW(JESD204_IOCTL, (JESD_IOCTL_IDX + 16), \
						struct auto_sync_params *)
#define JESD_GET_DEVICE_STATE   _IOR(JESD204_IOCTL, (JESD_IOCTL_IDX + 17), \
						enum jesd_state)
#define JESD_READ_PHYGASKET_REG		_IOR(JESD204_IOCTL,\
						(JESD_IOCTL_IDX + 18), \
						struct jesd_reg_read_buf *)
#define JESD_DEVICE_START	_IO(JESD204_IOCTL, (JESD_IOCTL_IDX + 19))
/*IOCTL end*/
#endif

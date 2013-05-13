/*
* include/uapi/linux/jesd204.h
*
* Author: Freescale semiconductor, Inc.
*
* Copyright 2013 Freescale Semiconductor, Inc.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/
#ifndef JESD204_UAPI_H_	/* prevent circular inclusion */
#define JESD204_UAPI_H_	/* by using protection macros */

/** @brief the state for jesd204
 * a signle state determining enum
 */
enum jesd_state {
	JESD_STANDBY,
	CONFIGURED,
	SYNC_START,/* sync raised is the one that is used by TX only*/
	SYNCHRONIZED,/* sync raised is the one that is used by RX only*/
	SYNC_FAILED,
	TESTMODE,
	OPERATIONAL,
	SUSPENDED,
	ERROR,
};
/** @brief \struct configure the transport
*/
struct conf_tr {
	unsigned long sampling_rate;
	__u8 delay; /*Rx/Tx delay*/
	__u32 tran_flg;
	__u32 lanes;
};
/** @brief \struct ils params
 */
struct ils_params {
	__u8 device_id;
	__u8 bank_id;
	__u8 lane_0_id;
	__u8 lane_1_id;
	__u8 scrambling_scr;
	__u8 lanes_per_converter_l;
	__u8 octect_per_frame_f;
	__u8 frame_per_mf_k;
	__u8 conv_per_device_m;
	__u8 cs;
	__u8 n;
	__u8 np;
	__u8 s;
	__u8 cf;
	__u8 hd;
	__u8 subclass_ver;
	__u8 jesd_ver;
	__u8 csum_lane_0;
	__u8 csum_lane_1;
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
struct jesd_wreg {
	__u32 offset;
	__u32 value;
};
/** @brief \struct accumulate all instances of write reg with offsets
 * and update the count for number of instances attached.
 */
struct jesd_reg_write_buf {
	struct jesd_wreg *regs;
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
struct transport_device_info {
	enum jesd_state dev_state;
	struct ils_params ils;
	struct conf_tr init_params;
	__u32 ilas_len;
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
/** @brief auto sync params
 *	sync_err_bitmap: the bit map is as given below:
 *--------------------------------------------------
 *|Bit7 |Bit6 |Bit5 |Bit4 |Bit3 |Bit2 |Bit1 |Bit0   |
 *--------------------------------------------------
 *|NA   |NA   |NA   |NA	  |NA   |UX K |NIT  |BAD DIS|
 *--------------------------------------------------
 */
struct auto_sync_params {
	uint8_t  sync_err_bitmap;
	uint8_t	 error_threshold;
};

/*IOCTL intefrace details.. this shall be common across the driver and lib*/
/*dev comment Ram.I need to give the command a tag to get hold*/
#define JESD204_IOCTL 'j'

#define JESD_SET_TRANS_PARAMS	_IOW(JESD204_IOCTL, 0x801, \
						struct conf_tr*)
#define JESD_SET_LANE_PARAMS	_IOW(JESD204_IOCTL, 0x802, \
						struct ils_params*)
#define JESD_SET_ILS_LENGTH	_IOW(JESD204_IOCTL, 0x803, \
						unsigned char)
#define JESD_SET_INTERRUPTMASK	_IOW(JESD204_IOCTL, 0x804, \
						struct isrconf*)
#define JESD_GET_INTERRUPTMASK	_IOR(JESD204_IOCTL, 0x805, \
						struct isrconf*)
#define JESD_DEIVCE_START	_IO(JESD204_IOCTL, 0x806)
#define JESD_TX_TEST_MODE	_IOR(JESD204_IOCTL, 0x807, \
						struct conf_tx_tests*)
#define JESD_RX_TEST_MODE	_IOR(JESD204_IOCTL, 0x808, \
						struct conf_rx_tests*)
#define JESD_FORCE_SYNC		_IO(JESD204_IOCTL, 0x809)
#define JESD_WRITE_REG		_IOW(JESD204_IOCTL, 0x80a, \
						struct jesd_reg_write_buf*)
#define JESD_READ_REG		_IOR(JESD204_IOCTL, 0x80b, \
						struct jesd_reg_read_buf*)
#define JESD_SHUTDOWN		_IO(JESD204_IOCTL, 0x80c)
#define JESD_GET_DEVICE_INFO	_IOR(JESD204_IOCTL, 0x80d, \
						struct transport_device_info*)
#define JESD_DEV_RESTART	_IOW(JESD204_IOCTL, 0x80e, \
						struct tbgen_params*)
#define JESD_GET_STATS		_IOW(JESD204_IOCTL, 0x80f, \
						struct tarns_dev_stats*)
#define JESD_CLEAR_STATS	_IO(JESD204_IOCTL, 0x810)
#define JESD_GET_LANE_RX_RECEIVED_PARAMS _IOW(JESD204_IOCTL, 0x811, \
						struct ils_params*)
#define JESD_RX_AUTO_SYNC	_IOW(JESD204_IOCTL, 0x812, \
						struct auto_sync_params*)
#define JESD_GET_DEVICE_STATE   _IOR(JESD204_IOCTL, 0x813, \
						enum jesd_state*)
#define JESD_SET_DEVICE_STATE   _IOW(JESD204_IOCTL, 0x814, \
						enum jesd_state*)
/*IOCTL end*/
#endif

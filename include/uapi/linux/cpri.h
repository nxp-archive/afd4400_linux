/*
 * include/uapi/linux/cpri.h
 * CPRI device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __UAPI_CPRI_H
#define __UAPI_CPRI_H

struct cpri_map_offsets {
	__u8 rx_map_offset_hf;
	__u8 rx_map_offset_bf;

	__u8 tx_map_offset_hf;
	__u8 tx_map_offset_bf;

	__u8 tx_start_offset_hf;
	__u8 tx_start_offset_bf;
};

struct cpri_dev_ctrl {
	__u32 ctrl_mask;
#define DEV_START_UL				(1 << 1)
#define DEV_START_DL				(1 << 2)
#define DEV_STANDBY				(1 << 3)
#define DEV_STOP				(1 << 4)
#define DEV_SLEEP				(1 << 5)
	void *ctrl_data;
	int ctrl_size;
};

struct cpri_dev_init_params {
	__u32 ctrl_flags;
#define CPRI_DEV_MASTER				(1 << 1)
#define CPRI_DEV_SLAVE				(1 << 2)
#define CPRI_DAISY_CHAINED			(1 << 3)
#define CPRI_SET_10_ACKS			(1 << 6)
#define CPRI_CNT_6_RESET			(1 << 7)
#define CPRI_SYNC_PULSE_MODE			(1 << 8)
#define CPRI_TX_CW_INSERT			(1 << 9)
#define CPRI_CW					(1 << 10)
#define CPRI_CW130				(1 << 11)
#define CPRI_RST_REQ_BYP			(1 << 12)
#define CPRI_C1_REM_RES_OP			(1 << 13)
#define CPRI_C1_REM_RES_ACK_OP			(1 << 14)
#define CPRI_C2_REM_RES_OP			(1 << 15)
#define CPRI_C2_REM_RES_ACK_OP			(1 << 16)
#define CPRI_RX_IQ_SYNC				(1 << 17)
#define CPRI_TX_IQ_SYNC				(1 << 18)
#define CPRI_ETH_FWD				(1 << 19)
#define CPRI_GEN_RX_IQ_SYNC			(1 << 20)
#define CPRI_GEN_TX_IQ_SYNC			(1 << 21)
#define CPRI_SINGLE_BIT_ECC_ERROR_OUTPUT	(1 << 22)
#define CPRI_MULTI_BIT_ECC_ERROR_OUTPUT		(1 << 23)
	unsigned int max_axc_count;
	__u32 axi_vss_rx_trans_size;
	__u32 axi_vss_tx_trans_size;
	__u32 tx_framer_buffer_size;
	unsigned int k0;
	unsigned int k1;
};

struct sfp_info {
	/* Basic ID fields */
	__u8 type;
	__u8 ext_type;
	__u8 connector_type;
	__u8 compatibility_code[8];
	__u8 encoding;
	__u8 bitrate;
	__u8 link_len_9u_km;
	__u8 link_len_9u_100m;
	__u8 link_len_50u_10m;
	__u8 link_len_62_5u_10m;
	__u8 link_len_cu_m;
	__u8 vendor_name[16];
	__u8 vendor_oui[3];
	__u8 vendor_pn[16];
	__u8 vendor_rev[4];
	__u8 wavelength[2];
	__u8 check_code_b;

	/* Extended ID fields */
	__u8 options[2];
	__u8 bitrate_max;
	__u8 bitrate_min;
	__u8 vendor_sn[16];
	__u8 manf_date[8];
	__u8 diag_type;
	__u8 enhanced_options;
	__u8 sfp_compliance;
	__u8 check_code_e;
};

enum cpri_state {
	CPRI_STATE_SFP_DETACHED = 1,
	/* Initial state */
	CPRI_STATE_STANDBY,
	CPRI_STATE_CONFIGURED,
	CPRI_STATE_LINK_ERROR,
	/* State during autoneg*/
	CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS,
	CPRI_STATE_LINE_RATE_AUTONEG,
	CPRI_STATE_PROT_VER_AUTONEG,
	CPRI_STATE_ETH_RATE_AUTONEG,
	CPRI_STATE_AUTONEG_COMPLETE,
	CPRI_STATE_AXC_CONFIG,
	CPRI_STATE_AXC_MAP_INIT,
	CPRI_STATE_OPERATIONAL,
};

struct cpri_dev_info {
	enum cpri_state state;
	__u32 test_flags;
#define UL_TRANSPARENT				(1 << 1)
#define DL_TRANSPARENT				(1 << 2)
	__u32 dev_flags;
#define CPRI_PASSIVE_LINK			(1 << 1)
#define CPRI_TEST_MODE				(1 << 2)
#define CPRI_DATA_MODE				(1 << 3)
#define CPRI_NO_EVENT                           (1 << 4)
	__u32 hw_status;
#define RX_LOS_STATUS				(1 << 1)
#define RX_STATE_STATUS				(1 << 2)
#define RX_HFN_STATE_STATUS			(1 << 3)
#define RX_BFN_STATE_STATUS			(1 << 4)
#define RX_LOS_HOLD_STATUS			(1 << 5)
#define RX_STATE_HOLD_STATUS			(1 << 6)
#define RX_FREQ_ALARM_HOLD_STATUS		(1 << 7)
#define RX_RFP_HOLD_STATUS			(1 << 8)

#define RESET_GEN_DONE_HOLD_STATUS		(1 << 9)
#define RESET_GEN_DONE_STATUS			(1 << 10)
#define RESET_DETECT_HOLD_STATUS		(1 << 11)
#define RESET_DETECT_STATUS			(1 << 12)
	__u16 current_bfn;
	__u8 current_hfn;
	struct sfp_info sfp_info;
	struct cpri_dev_init_params init_params;
};

struct cpri_dev_stats {
	unsigned int tx_vss_frames;
	unsigned int rx_vss_frames;
	unsigned int tx_hfn_irqs;
	unsigned int rx_hfn_irqs;
	unsigned int tx_bfn_irqs;
	unsigned int rx_bfn_irqs;
	unsigned int l1_auto_neg_failures;
	unsigned int proto_auto_neg_failures;
	unsigned int cnm_auto_neg_failures;
	unsigned int vendor_config_failures;
	unsigned int rx_iq_overruns;
	unsigned int tx_iq_underruns;
	unsigned int vss_underruns;
	unsigned int vss_overruns;
	unsigned int rx_iq_overrun_err_count;
	unsigned int tx_iq_underrun_err_count;
	unsigned int rx_eth_mem_overrun_err_count;
	unsigned int tx_eth_underrun_err_count;
	unsigned int rx_eth_bd_underrun_err_count;
	unsigned int rx_hdlc_overrun_err_count;
	unsigned int tx_hdlc_underrun_err_count;
	unsigned int rx_hdlc_bd_underrun_err_count;
	unsigned int rx_vss_overrun_err_count;
	unsigned int tx_vss_underrun_err_count;
	unsigned int ecc_config_mem_err_count;
	unsigned int ecc_data_mem_err_count;
	unsigned int rx_eth_dma_overrun_err_count;
	unsigned int eth_fwd_rem_fifo_full_err_count;
	unsigned int ext_sync_loss_err_count;
	unsigned int rlos_err_count;
	unsigned int rlof_err_count;
	unsigned int rai_err_count;
	unsigned int rsdi_err_count;
	unsigned int llos_err_count;
	unsigned int llof_err_count;
	unsigned int rr_err_count;
	unsigned int fa_err_count;
	unsigned int rra_err_count;
	unsigned int rx_line_coding_violation;
};

struct cpri_reg_read_buf {
	__u32 start_offset;
	__u32 *reg_buff;
	int count;
};

struct cpri_reg {
	__u32 offset;
	__u32 value;
};

struct cpri_reg_write_buf {
	struct cpri_reg *regs;
	int count;
};

struct sfp_reg_read_buf {
	__u8 start_offset;
	__u8 *reg_buff;
	int count;
};

struct sfp_reg {
	__u8 offset;
	__u8 value;
};

struct sfp_reg_write_buf {
	struct sfp_reg *regs;
	int count;
};

/* Event notification data structures- start */
struct evt_device_notification_data {
	__u32 evt_map;
#define EVT_AUTO_NEG_DONE			(1 << 1)
#define EVT_AUTO_NEG_FAILURE			(1 << 2)
#define IEVT_IQ_RX_OVERRUN			(1 << 3)
#define IEVT_IQ_TX_UNDERRUN			(1 << 4)
#define IEVT_L1_INBAND				(1 << 5)
#define IEVT_L1_RESET				(1 << 6)
#define IEVT_LINK_ERROR				(1 << 7)
	__u32 evt_data;
#define EVT_L1TIMER_EXPIRED				(1 << 1)
#define EVT_PROT_VER_SETUP_TIMER_EXPIRED		(1 << 2)
#define EVT_CM_SETUP_TIMER_EXPIRED			(1 << 3)
#define EVT_SERDES_FAILURE				(1 << 4)

#define IEVT_LLOS				(1 << 1)
#define IEVT_LLOF				(1 << 2)
#define EVT_ETHLINK_RATE_MISMATCH		(1 << 3)
#define EVT_PROT_VER_MISMATCH			(1 << 4)
};

struct evt_vss_txrx_data {
	__u32 evt_map;
#define IEVT_VSS_TX_BUF_INDICATION		(1 << 1)
#define IEVT_VSS_TX_CHAN_INDICATION		(1 << 2)
#define IEVT_VSS_RX_INDICATION			(1 << 3)
	__u32 buff_idx;
};

enum event_type {
	EVT_TYPE_DEV_NOTIFICATIONS = 1,
	EVT_TYPE_VSS_TX,
	EVT_TYPE_VSS_RX
};
/* Event notification data structures - End */

enum cpri_prot_ver {
	VER_1 = 1,
	VER_2,
	VER_INVALID
};

enum hdlc_link_rate {
	RATE_0 = 1,
	RATE_240K,
	RATE_480K,
	RATE_960K,
	RATE_1920K,
	RATE_2400K,
	RATE_3840K,
	RATE_4800K,
	RATE_7680K
};

enum cpri_cm_mode {
	ETH = 1,
	HDLC
};

enum cpri_link_rate {
	RATE2_1228_8M = 1,
	RATE3_2457_6M,
	RATE4_3072_0M,
	RATE5_4915_2M,
	RATE6_6144_0M,
	RATE7_9830_4M
};

struct cpri_autoneg_params {
	__u32 flags;
#define CPRI_UPDATE_L1_AUTONEG_PARAMS		(1 << 1)
#define CPRI_UPDATE_PROTO_AUTONEG_PARAMS	(1 << 2)
#define CPRI_UPDATE_CnM_AUTONEG_PARAMS		(1 << 3)
#define CPRI_RX_SCRAMBLER_EN			(1 << 4)
#define CPRI_SERDES_LOOPBACK			(1 << 5)
	unsigned int l1_setup_timeout;
	/* Line rate auto neg parameters */
	unsigned int tx_on_time;
	unsigned int tx_off_time;
	unsigned int linerate_timeout;
	enum cpri_link_rate link_rate_low, link_rate_high;
	/*C&M rate auto neg parameters */
	unsigned int cnm_timeout;
	enum cpri_cm_mode cm_mode;
	enum hdlc_link_rate hdlc_rate_low, hdlc_rate_high;
	unsigned int eth_rates_count;
	unsigned int *eth_rates;
	/* Protocol auto neg parameters */
	unsigned int proto_timeout;
	enum cpri_prot_ver tx_prot_ver;
	__u32 tx_scr_seed;
};

struct cpri_delays_raw_cfg {
	__u32 rx_ex_delay_period;
	__u32 tx_ex_delay;
};

struct cpri_autoneg_output {
	enum cpri_link_rate common_link_rate;
	__u8 common_eth_link_rate;
	enum cpri_prot_ver common_prot_ver;
	__u8 rx_eth_ptr;
	/* Basic frame */
	unsigned int cpri_bf_word_size;
	unsigned int cpri_bf_iq_datablock_size;
	/*Size of vss data in one hyper frame in bytes*/
	unsigned int hf_vss_size;
	/* Scramble seed */
	unsigned int rx_scramble_seed_val;
};

struct cpri_delays_raw {
	/*external delays (SFP + SERDES)*/
	__u16 rx_ext_buf_delay_valid;
	__u16 rx_ext_buf_delay;
	/* variable delays */
	__u8 rx_byte_delay;
	__u8 rx_buf_delay;
	__u8 rx_align_delay;
	__u32 rx_roundtrip_delay;
};

enum mapping_method {
	MAPPING_METHOD_1 = 1,
	MAPPING_METHOD_3
};

struct axc_pos {
	__u8 axc_start_w;
	__u8 axc_start_b;
};

struct axc_info {
	unsigned int id;
	__u32 flags;
#define UL_AXCS					(1 << 1)
#define DL_AXCS					(1 << 2)
#define READ_ALL_AXCS				(1 << 3)
#define AXC_DATA_TYPE_IQ			(1 << 4)
#define AXC_DATA_TYPE_VSS			(1 << 5)
#define AXC_OVERSAMPLING_2X			(1 << 12)
#define AXC_CONVERSION_9E2_EN			(1 << 7)
#define AXC_TX_ROUNDING_EN			(1 << 14)
#define AXC_INTERLEAVING_EN			(1 << 30)
#define AXC_IQ_FORMAT_2				(1 << 31)
#define AXC_FLEXI_POSITION_EN			(1 << 11)
	enum mapping_method map_method;
	unsigned int buffer_size;
	unsigned int buffer_threshold;
	unsigned int sampling_freq;
	unsigned int S;
	unsigned int K;
	unsigned int na;
	unsigned int ns;
	__u8 sample_width;
	struct axc_pos *pos;
	__u32 axc_dma_ptr;
};

struct axc_config_params {
	unsigned int axc_count;
	struct axc_info *axcs;
	__u32 flags;
};

struct subsegment_info {
	__u32 axc_id;
	__u8 offset;
	__u8 map_size;
};

struct segment_info {
	struct subsegment_info subsegments[3];
	__u32 k;
};

struct axc_map_table_get {
	__u32 seg_count;
	struct segment_info *segments;
	__u32 flags;
};

enum axc_ctrl_op {
	AXC_ENABLE = 1,
	AXC_DISABLE,
	AXC_DELETE
};

struct cpri_axc_ctrl {
	enum axc_ctrl_op op;
	unsigned int axc_id;
	unsigned int direction;
};

struct cpri_vss_init_params {
	__u32 flags;
#define VSS_TX_CHAN				(1 << 1)
#define VSS_RX_CHAN				(1 << 2)
#define VSS_TX_STREAMED				(1 << 3)
#define VSS_TX_NON_STREAMED			(1 << 4)
#define VSS_TX_STREAMED_BUF			(1 << 5)
#define VSS_TX_CHANNEL_BUF			(1 << 6)
	unsigned int vss_buf_size_hf;
	unsigned int vss_buf_thresh_hf;
	__u32 buff_virt;
	__u32 buff_phys;
};

struct ctrl_chan {
	unsigned int chan;
	__u8 words[4][16];
	__u8 word_bitmap;
};

struct hf_ctrl_chans {
	struct ctrl_chan  *channels;
	unsigned int channel_count;
	unsigned int hf;
};

struct daisy_chain_param {
	unsigned int flag;
	__u8 cw130_bitmap;
	__u32 axc_bitmap;
	__u8 cw_bitmap[8];
	unsigned int axc_map[24];
};

enum autoneg_cmd {
	CPRI_DO_LINE_RATE_AUTONEG = 1,
	CPRI_DO_PROTOCOL_AUTONEG,
	CPRI_DO_CNM_AUTONEG,
	CPRI_DO_VENDOR_AUTONEG,
	CPRI_DO_AUTONEG_ALL,
};

enum recfg_cmd {
	CPRI_LINK_RECONFIG_INIT_REQ = 1,
	CPRI_INTERFACE_RECONFIG_INIT_REQ,
	CPRI_INTERFACE_RECONFIG_SETUP_REQ,
	CPRI_AXC_MAPPING_RECONFIG_INIT_REQ,
	CPRI_AXC_MAPPING_RECONFIG_SETUP_REQ
};

enum mem_type {
	SFP_MEM_EEPROM = 0,
	SFP_MEM_DIAG
};

/* AxC mapping direction flags */
#define AXC_MAPPING_DL				(1 << 1)
#define AXC_MAPPING_UL				(1 << 2)

/* VSS channel control flags */
#define VSS_TX_ENABLE				(1 << 1)
#define VSS_TX_DISABLE				(1 << 2)
#define VSS_TX_RESET				(1 << 3)
#define VSS_RX_ENABLE				(1 << 4)
#define VSS_RX_DISABLE				(1 << 5)
#define VSS_RX_RESET				(1 << 6)

/* Daisy chain control flags */
#define DAISY_CHAIN_ENABLE			(1 << 1)

#define CPRI_MAGIC 'C'

/* General configuration & status IOCTLS */
#define CPRI_INIT_DEV				_IOW(CPRI_MAGIC, 1, \
						struct cpri_dev_init_params *)
#define CPRI_CTRL_DEV				_IOW(CPRI_MAGIC, 2, \
						struct cpri_dev_ctrl *)
#define CPRI_SET_TESTMODE			_IOW(CPRI_MAGIC, 3, \
						unsigned int)
#define CPRI_GET_STATE				_IOR(CPRI_MAGIC, 4, \
						enum cpri_state)
#define CPRI_SET_STATE				_IOW(CPRI_MAGIC, 5, \
						enum cpri_state)
#define CPRI_GET_INFO				_IOR(CPRI_MAGIC, 6, \
						struct cpri_dev_info *)
#define CPRI_GET_STATS				_IOR(CPRI_MAGIC, 7, \
						struct cpri_dev_stats *)
#define CPRI_CLEAR_STATS			_IO(CPRI_MAGIC, 8)
#define CPRI_READ_REG				_IOR(CPRI_MAGIC, 9, \
						struct cpri_reg_read_buf *)
#define CPRI_WRITE_REG				_IOW(CPRI_MAGIC, 10, \
						struct cpri_reg_write_buf *)
#define CPRI_READ_REG_COMMON			_IOR(CPRI_MAGIC, 11, \
						struct cpri_reg_read_buf *)
#define CPRI_WRITE_REG_COMMON			_IOW(CPRI_MAGIC, 12, \
						struct cpri_reg_write_buf *)
#define SFP_READ_REG				_IOR(CPRI_MAGIC, 13, \
						struct cpri_reg_read_buf *)
#define SFP_READ_DIAG_REG			_IOR(CPRI_MAGIC, 14, \
						struct cpri_reg_read_buf *)
#define SFP_WRITE_REG				_IOW(CPRI_MAGIC, 15, \
						struct cpri_reg_write_buf *)
#define CPRI_REGISTER_EVENT			_IOW(CPRI_MAGIC, 16, \
						unsigned int)
#define CPRI_GET_EVENT				_IOW(CPRI_MAGIC, 17, \
						enum event_type)
#define CPRI_BFN_RESET				_IO(CPRI_MAGIC, 18)

/* Link start-up IOCTLS */
#define CPRI_START_AUTONEG			_IOW(CPRI_MAGIC, 20, \
						enum autoneg_cmd)
#define CPRI_START_RECONFIG			_IOW(CPRI_MAGIC, 21, \
						enum recfg_cmd)
#define CPRI_INIT_AUTONEG_PARAM			_IOW(CPRI_MAGIC, 22, \
						struct cpri_autoneg_params *)
#define CPRI_GET_AUTONEG_PARAM			_IOR(CPRI_MAGIC, 23, \
						struct cpri_autoneg_params *)
#define CPRI_GET_AUTONEG_OUTPUT			_IOR(CPRI_MAGIC, 24, \
						struct cpri_autoneg_output *)
#define CPRI_GET_RAWDELAY			_IOR(CPRI_MAGIC, 25, \
						struct cpri_delays_raw *)
#define CPRI_SET_RAWDELAY			_IOW(CPRI_MAGIC, 26, \
						struct cpri_delays_raw_cfg *)
#define CPRI_START_L1TIMER			_IO(CPRI_MAGIC, 27)
#define CPRI_STOP_L1TIMER			_IO(CPRI_MAGIC, 28)
#define CPRI_SET_L1TIMER			_IOW(CPRI_MAGIC, 29, long)

/* AxC mapping & configuration IOCTLS */
#define CPRI_SET_AXC_PARAM			_IOW(CPRI_MAGIC, 30, \
						struct axc_config_params *)
#define CPRI_GET_AXC_PARAM			_IOR(CPRI_MAGIC, 31, \
						struct axc_config_params *)
#define CPRI_CTRL_AXC				_IOW(CPRI_MAGIC, 32, \
						struct cpri_axc_ctrl *)
#define CPRI_MAP_INIT_AXC			_IOW(CPRI_MAGIC, 33, \
						unsigned int)
#define CPRI_MAP_CLEAR_AXC			_IO(CPRI_MAGIC, 34)
#define CPRI_GET_MAP_TABLE			_IOR(CPRI_MAGIC, 35, \
						struct axc_map_table_get *)
#define CPRI_BD_DUMP				_IOW(CPRI_MAGIC, 36, \
							unsigned int)

/* VSS channel configuration and data Tx/Rx IOCTLS */
#define CPRI_OPEN_VSS				_IOW(CPRI_MAGIC, 40, \
						struct vss_init_params *)
#define CPRI_CLOSE_VSS				_IOW(CPRI_MAGIC, 41, \
						unsigned int)
#define CPRI_CTRL_VSS				_IOW(CPRI_MAGIC, 42, \
						unsigned int)
#define CPRI_TX_VSS				_IOW(CPRI_MAGIC, 43, \
						struct hf_ctrl_chans *)

/* Control word table R/W IOCTLS */
#define CPRI_RX_CW				_IOR(CPRI_MAGIC, 51, \
						struct hf_ctrl_chans *)
#define CPRI_TX_CW				_IOW(CPRI_MAGIC, 50, \
						struct hf_ctrl_chans *)

/* Daisy chain configuration IOCTLS */
#define CPRI_INIT_DAISYCHAIN			_IOW(CPRI_MAGIC, 60, \
						struct daisy_chain_param *)
#define CPRI_READ_DAISYCHAIN			_IOR(CPRI_MAGIC, 61, \
						struct daisy_chain_param *)
#define CPRI_CTRL_DAISYCHAIN			_IOW(CPRI_MAGIC, 62, \
						unsigned int)

#endif /* __CPRI_IOCTL_H */

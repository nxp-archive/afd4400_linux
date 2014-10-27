/*
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
 */

#ifndef _UAPI_CPRI_H
#define _UAPI_CPRI_H

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


struct serdes_amp_data {
	__u32 sfp_max_mvolt;
	__u8 rxequil_boost_en;
};

struct sfp_reg_write_buf {
	struct sfp_reg *regs;
	int count;
};

struct frame_diff_buf {
	int framediff_hfn;
	int framediff_x;
};

struct monitor_config_en {
	uint64_t enable_mask;
	int timer_enable;
	int timer_interval;
};

struct monitor_config_disable {
	uint64_t disable_mask;
	int timer_disable;
};

/**
 * The bit position for all possible err mask
 */
enum cpri_errors_bitpos {
	RX_IQ_OVERRUN_BITPOS = 0,
	TX_IQ_UNDERRUN_BITPOS,
	RX_ETH_MEM_OVERRUN_BITPOS,
	TX_ETH_UNDERRUN_BITPOS,
	RX_ETH_BD_UNDERRUN_BITPOS,
	RX_HDLC_OVERRUN_BITPOS,
	TX_HDLC_UNDERRUN_BITPOS,
	RX_HDLC_BD_UNDERRUN_BITPOS,
	RX_VSS_OVERRUN_BITPOS,
	TX_VSS_UNDERRUN_BITPOS,
	ECC_CONFIG_MEM_BITPOS,
	ECC_DATA_MEM_BITPOS,
	RX_ETH_DMA_OVERRUN_BITPOS,
	ETH_FORWARD_REM_FIFO_FULL_BITPOS,
	EXT_SYNC_LOSS_BITPOS = 15,
	RLOS_BITPOS,
	RLOF_BITPOS,
	RAI_BITPOS,
	RSDI_BITPOS,
	LLOS_BITPOS,
	LLOF_BITPOS,
	RRE_BITPOS,
	FAE_BITPOS,
	RRA_BITPOS,
/**
 * Errors that's not in the CPRI interrupt registers
 */
	CPRI_USER_MONITOR_START,
	JCPLL_LOCK_LOSS_BITPOS = CPRI_USER_MONITOR_START,
	PROTO_VER_MISMATCH_BITPOS,
	ETH_PTR_MISMATCH_BITPOS,
	RX_LINE_RATE_CODING_VIOLATION_BITPOS,
	CPRI_RATE_SYNC_BITPOS,
/**
 * The SFP gpio error are expanded to 3 fields below
 */
	SFP_MONITOR_BITPOS,
/**
 * SFP presence gpio indication
 */
	SFP_PRESENCE_BITPOS = SFP_MONITOR_BITPOS,
/**
 * SFP rx loss of signal gpio indication
 */
	SFP_RXLOS_BITPOS,
/**
 * SFP tx fault indication
 */
	SFP_TXFAULT_BITPOS,
/**
 * total number of possible CPRI errors
 */
	CPRI_ERR_CNT,
 /**
  * reset ack function enable, not considered as an error,
  * but it's going to use the same kernel timer
  * to poll the status.
  */
	CPRI_RESET_ACK_BITPOS = CPRI_ERR_CNT,
};

#define RX_IQ_OVERRUN				\
	(1 << RX_IQ_OVERRUN_BITPOS)
#define TX_IQ_UNDERRUN				\
	(1 << TX_IQ_UNDERRUN_BITPOS)
#define RX_ETH_MEM_OVERRUN			\
	(1 << RX_ETH_MEM_OVERRUN_BITPOS)
#define TX_ETH_UNDERRUN				\
	(1 << TX_ETH_UNDERRUN_BITPOS)
#define RX_ETH_BD_UNDERRUN			\
	(1 << RX_ETH_BD_UNDERRUN_BITPOS)
#define RX_HDLC_OVERRUN				\
	(1 << RX_HDLC_OVERRUN_BITPOS)
#define TX_HDLC_UNDERRUN			\
	(1 << TX_HDLC_UNDERRUN_BITPOS)
#define RX_HDLC_BD_UNDERRUN			\
	(1 << RX_HDLC_BD_UNDERRUN_BITPOS)
#define RX_VSS_OVERRUN				\
	(1 << RX_VSS_OVERRUN_BITPOS)
#define TX_VSS_UNDERRUN				\
	(1 << TX_VSS_UNDERRUN_BITPOS)
#define ECC_CONFIG_MEM				\
	(1 << ECC_CONFIG_MEM_BITPOS)
#define ECC_DATA_MEM				\
	(1 << ECC_DATA_MEM_BITPOS)
#define RX_ETH_DMA_OVERRUN			\
	(1 << RX_ETH_DMA_OVERRUN_BITPOS)
#define ETH_FORWARD_REM_FIFO_FULL		\
	(1 << ETH_FORWARD_REM_FIFO_FULL_BITPOS)
#define EXT_SYNC_LOSS				\
	(1 << EXT_SYNC_LOSS_BITPOS)
#define RLOS					\
	(1 << RLOS_BITPOS)
#define RLOF					\
	(1 << RLOF_BITPOS)
#define RAI					\
	(1 << RAI_BITPOS)
#define RSDI					\
	(1 << RSDI_BITPOS)
#define LLOS					\
	(1 << LLOS_BITPOS)
#define LLOF					\
	(1 << LLOF_BITPOS)
#define RRE					\
	(1 << RRE_BITPOS)
#define FAE					\
	(1 << FAE_BITPOS)
#define RRA					\
	(1 << RRA_BITPOS)
#define JCPLL_LOCK_LOSS				\
	(1 << JCPLL_LOCK_LOSS_BITPOS)
#define PROTO_VER_MISMATCH			\
	(1 << PROTO_VER_MISMATCH_BITPOS)
#define ETH_PTR_MISMATCH			\
	(1 << ETH_PTR_MISMATCH_BITPOS)
#define RX_LINE_RATE_CODING_VIOLATION		\
	(1 << RX_LINE_RATE_CODING_VIOLATION_BITPOS)
#define CPRI_RATE_SYNC				\
	(1 << CPRI_RATE_SYNC_BITPOS)
#define SFP_MONITOR				\
	(1 << SFP_MONITOR_BITPOS)
#define CPRI_RESET_ACK				\
	(1ULL << CPRI_RESET_ACK_BITPOS)
/**
 * The errors that are polled by the kernel timer
 */
#define CPRI_USER_EVT_ALL			\
		(JCPLL_LOCK_LOSS		\
		| PROTO_VER_MISMATCH		\
		| ETH_PTR_MISMATCH		\
		| RX_LINE_RATE_CODING_VIOLATION \
		| CPRI_RATE_SYNC		\
		| CPRI_RESET_ACK		\
		| RRE | RAI | RSDI | RLOS | RLOF)

/**
 * The errors defined by CPRI event register
 */
#define CPRI_ERR_EVT_ALL	(RX_IQ_OVERRUN \
				| TX_IQ_UNDERRUN \
				| RX_ETH_MEM_OVERRUN \
				| TX_ETH_UNDERRUN \
				| RX_ETH_BD_UNDERRUN \
				| RX_HDLC_OVERRUN \
				| TX_HDLC_UNDERRUN \
				| RX_HDLC_BD_UNDERRUN \
				| RX_VSS_OVERRUN \
				| TX_VSS_UNDERRUN \
				| ECC_CONFIG_MEM \
				| ECC_DATA_MEM \
				| RX_ETH_DMA_OVERRUN \
				| ETH_FORWARD_REM_FIFO_FULL \
				| EXT_SYNC_LOSS \
				| RLOS \
				| RLOF \
				| RAI \
				| RSDI \
				| LLOS \
				| LLOF \
				| RRE \
				| FAE \
				| RRA)

enum cpri_prot_ver {
	VER_1 = 1,
	VER_2,
	VER_INVALID
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
	__u32 mode;
/**
 * Framer in RE slave mode
 */
#define RE_MODE_SLAVE (1 << 0)
/**
 * Framer in RE master mode in daisy chain.
 * Apply to "cp0_frmr1" or "cp1_frmr1" only
 */
#define	RE_MODE_MASTER (1 << 1)
/**
 * Framer in REC mode
 */
#define	REC_MODE (1 << 2)
/**
 * if (STICK_TO_RATE is set and rate_preferred has value in enum cpri_link_rate)
 * stick to rate_preferred in autoneg until @rate_neg_timeout seconds.
 *
 * if (STIRKC_TO_RATE is not set and rate_prefered has value
 * in enum_cpri_link_rate) rate autoneg starts from rate_preferred,
 * if not successful after @rate_neg_timeout
 * seconds, change to rate_high and goes down one level
 * till (includes) rate_low. Each rate try @rate_neg_timeout seconds
 * to see if it's synced.
 *
 * if (STICK_TO_RATE is not set and rate_preferred is 0)
 * rate autoneg starts from rate_high and goes down one level
 * till (includes) rate_low.
 * Each rate try @rate_neg_timeout seconds to see if it's synced.
 *
 * Generally speaking, if you know the rate you are going to use, you can use
 * that rate for rate_preferred. You should set
 * STICK_TO_RATE for the daisy chain master node because you want the
 * RE master node to always use the rate from RE slave
 * and not change by its self.
 *
 * During autoneg, AxC and CPRI-ethernet needs to be cleared and thus needs
 * re-enabling.
 */
#define STICK_TO_RATE (1 << 3)
/**
 * Stick to @tx_prot_ver till @proto_neg_timeout
 * seconds if this flag is set, otherwise adapt
 * to the protocal version it received from the framer
 */
#define STICK_TO_PROTO (1 << 4)
/**
 * Stick to @eth_ptr till @ethptr_neg_timeout
 * seconds if this flag is set, otherwise adapt
 * to the ethernet pointer value from the framer.
 */
#define STICK_TO_ETHPTR (1 << 5)
/**
 * Reset the pll and serdes if this flag is set.
 * Also if the CPRI is running in RE mode, the framer
 * that sets this flag will output the recovered clk
 * to JCPLL. Use this flag with caution, because you
 * don't want to reset the pll and serdes lanes when other
 * CPRI framer is already running, unless you really need to,
 * eg. restart rate autonegotiation.
 */
#define RESET_LINK	(1 << 6)
/**
 * Steps to be done in autoneg
 * If not set this autoneg step will be skipped.
 * If set, it will follow the sequence as
 * rate->protocal->ethernet pointer auto negotiation.
 * If the one of the steps is not successful and times out,
 * the further steps are skipped.
 */
	__u32 autoneg_steps;
#define RATE_AUTONEG (1 << 0)
#define PROTO_AUTONEG (1 << 1)
#define ETHPTR_AUTONEG (1 << 2)
/**
 * Timeout value for three autoneg steps
 */
	unsigned int rate_neg_timeout;
	unsigned int proto_neg_timeout;
	unsigned int ethptr_neg_timeout;
/**
 * Refer to flag STICK_TO_RATE for detail
 */
	enum cpri_link_rate rate_preferred;
	enum cpri_link_rate rate_low;
	enum cpri_link_rate rate_high;
	enum cpri_prot_ver tx_prot_ver;
/**
 * Valid ethernet pointer ranges from 20 to 63
 */
	unsigned int eth_ptr;
	__u32 tx_scr_seed;
};

struct cpri_autoneg_output {
	enum cpri_link_rate common_rate;
	__u8 common_eth_ptr;
	enum cpri_prot_ver common_prot_ver;
	unsigned int rx_scramble_seed;
};

struct axc_config {
	int axc_cnt;
	struct axc_info *axc_info;
};

struct axc_info {
	unsigned int id;
#define AXC_DIR_TX	(1 << 0)
#define AXC_DIR_RX	(1 << 1)
/**
 * AXC is enabled, for non daisy chain
 * AxC data will start sending DMA request to/from VSPA.
 * For daisy chain AxC the AUX table will
 * be programmed.
 */
#define AXC_EN	(1 << 2)
/**
 * AxC is disabled, for non daisy chain
 * AxC data will stop send DMA request to/from VSPA.
 * For daisy chain the AUX table for this AxC region
 * will be cleared.
 */
#define AXC_DISABLE	(1 << 3)
#define AXC_DAISY_CHAINED	(1 << 4)
/**
 * If set, set bit 14 of CPRInTACPRMSB
 */
#define AXC_TX_ROUNDING_EN	(1 << 14)
/**
 * If set, enable interleaving for this AxC
 */
#define AXC_INTERLEAVING_EN	(1 << 30)
/**
 * If set, enable 9E2 format
 */
#define AXC_9E2_EN	(1 << 13)
/**
 * If set, enable CPRInT/RACPRMSB bit 31
 */
#define AXC_IQ_FORMAT_2	(1 << 31)
/**
 * If set, enable CPRInT/RACPRMSB bit 12
 */
#define AXC_OVERSAMPLING_2X	(1 << 12)
	__u32 flags;
/**
 * AxC offset from word 1 in basic frame in bit
 */
	unsigned int container_offset;
/**
 * AxC size in bits
 */
	unsigned int container_size;
	unsigned char sampling_width;

/**
 * Refer to RX/TX AxC Buffer Management
 * in reference manual
 * for proper setting of these fields
 */
	unsigned int buffer_threshold;
	unsigned int buffer_size;
	unsigned int buffer_offset_addr;
#define MEMBLK0	(1 << 0)
#define MEMBLK1	(1 << 1)
	unsigned int memblk_sel;

};

struct rx_cw_params {
	int bf_index;
	__u8	*data;
	int len;
};

struct tx_cw_params {
	int bf_index;
	__u8 *data;
	int len;
	int operation;
/**
 * Read the tx control table
 */
#define TX_CW_READ	(1 << 0)
/**
 * Write the tx control table
 */
#define TX_CW_WRITE	(1 << 1)
/**
 * Bypass the tx control table
 */
#define TX_CW_BYPASS (1 << 2)
};

enum mem_type {
	SFP_MEM_EEPROM = 0,
	SFP_MEM_DIAG
};

enum cpri_state_bitpos {
	CPRI_RATE_BITPOS = 0,
	CPRI_PROTVER_BITPOS,
	CPRI_ETHPTR_BITPOS,
	CPRI_SFP_PRESENT_BITPOS,
/**
 * This bit is used by driver internally
 * and user can omit this bit.
 */
	CPRI_MONITOR_STARTED_BITPOS
};

#define CPRI_STATE_RATE_SUCCESS	(1 << CPRI_RATE_BITPOS)
#define	CPRI_STATE_PROTVER_SUCCESS (1 << CPRI_PROTVER_BITPOS)
#define	CPRI_STATE_ETHPTR_SUCCESS (1 << CPRI_ETHPTR_BITPOS)
#define	CPRI_STATE_SFP_PRESENT (1 << CPRI_SFP_PRESENT_BITPOS)
#define	CPRI_STATE_MONITOR_STARTED (1 << CPRI_MONITOR_STARTED_BITPOS)

struct cpri_axc_map_offset {
	__u8 map_tx_offset_x;
	__u8 map_tx_offset_z;
	__u8 start_tx_offset_x;
	__u8 start_tx_offset_z;
	__u8 map_rx_offset_x;
	__u8 map_rx_offset_z;
};

#define CPRI_MAGIC 'C'

#define CPRI_GET_STATE				_IOR(CPRI_MAGIC, 1, \
							unsigned int)
#define CPRI_READ_STATS				_IOR(CPRI_MAGIC, 2, \
							unsigned int)
#define CPRI_SET_MONITOR			_IOW(CPRI_MAGIC, 3, \
							unsigned int)
#define CPRI_CLEAR_MONITOR			_IOW(CPRI_MAGIC, 4, \
							unsigned int)

#define CPRI_AXC_OFFSET_REGS			_IOW(CPRI_MAGIC, 5, \
						struct cpri_axc_map_offset)
#define CPRI_READ_REG				_IOW(CPRI_MAGIC, 6, \
						struct cpri_reg_read_buf)
#define CPRI_WRITE_REG				_IOW(CPRI_MAGIC, 7, \
						struct cpri_reg_write_buf)
#define CPRI_READ_REG_COMMON			_IOR(CPRI_MAGIC, 8, \
						struct cpri_reg_read_buf)
#define CPRI_WRITE_REG_COMMON			_IOW(CPRI_MAGIC, 9, \
						struct cpri_reg_write_buf)
#define SFP_READ_REG				_IOR(CPRI_MAGIC, 10, \
						struct cpri_reg_read_buf)
#define SFP_READ_DIAG_REG			_IOR(CPRI_MAGIC, 11, \
						struct cpri_reg_read_buf)
#define SFP_WRITE_REG				_IOW(CPRI_MAGIC, 12, \
						struct cpri_reg_write_buf)
#define CPRI_GET_NFRAME_DIFF			_IOR(CPRI_MAGIC, 14, \
						 unsigned int)

#define CPRI_START_AUTONEG			_IOW(CPRI_MAGIC, 15, \
						struct cpri_autoneg_params)
#define CPRI_GET_AUTONEG_OUTPUT			_IOR(CPRI_MAGIC, 16, \
						struct cpri_autoneg_output)
#define CPRI_AXC_MAP				_IOW(CPRI_MAGIC, 17, \
						struct axc_config)
#define CPRI_AXC_CTRL				_IOW(CPRI_MAGIC, 18, \
						struct axc_config)
#define CPRI_BD_DUMP				_IOW(CPRI_MAGIC, 19, \
							unsigned int)
#define SERDES_AMP_SET				_IOW(CPRI_MAGIC, 20, \
						struct serdes_amp_data)

#define CPRI_RX_CTRL_TABLE			_IOR(CPRI_MAGIC, 21, \
						struct rx_cw_params)
#define CPRI_TX_CTRL_TABLE			_IOWR(CPRI_MAGIC, 22, \
						struct tx_cw_params)

#define CPRI_AXC_MAP_CONFIG			_IOW(CPRI_MAGIC, 23, \
						struct axc_map_config)

#endif /* _UAPI_CPRI_H */

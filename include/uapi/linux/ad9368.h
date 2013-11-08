/*
 * include/uapi/ad9368.h : AD9368 user space interface
 *
 * Author: Pankaj Chauhan  <pankaj.chauhan@freescale.com>
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

#ifndef __UAPI_AD9368_H__
#define __UAPI_AD9368_H__

#include <linux/types.h>

#define DEVICE_ID_AD93681 0
#define DEVICE_ID_AD9525  1
#define DEVICE_ID_AD93682 2
#define RF_NAME_SIZE		20
#define NUM_SYNTH_PARAMS	20
#define NUM_SYNTH_REGS		20

enum rf_band_width {
	BW_05_MHZ,
	BW_10_MHZ,
	BW_15_MHZ,
	BW_20_MHZ,
	BW_END
};

enum spi_cmd {
	SPI_WRITE = 1,
	SPI_READ,
	SPI_WAIT,
	SPI_WAIT_CALPLL_CAL,
	SPI_WAIT_CLKPLLCP_CAL,
	SPI_WAIT_TXFILTER_CAL,
	SPI_WAIT_RFDC_CAL,
	SPI_WAIT_TXQUAD_CAL,
	SPI_WAIT_CLKPLL_CAL,
	SPI_WAIT_INITARM_CAL,
	SPI_WAIT_RFPLLLOCK_CAL,
	SPI_WAIT_RFPLLCP_CAL,
	SPI_WAIT_ADCTUNER_CAL,
	SPI_WAIT_RXTIA_CAL,
	SPI_WAIT_RCAL_CAL,
	SPI_WAIT_RXADC_CAL,
	SPI_WAIT_ARMBUSY,
	SPI_WAIT_MAILBOX,
	SPI_WAIT_CALDONE,
	SPI_SET_CHANNEL,
	SPI_END
};

enum rf_gain_ctrl_mode {
	MGC,
	FASTATTACK_AGC,
	SLOWATTACK_AGC,
	HYBRID_AGC
};

struct rf_init_params {
	enum rf_band_width bw;
	unsigned int fq_band;
	unsigned int ants;
};

struct rf_phy_cmd {
	enum spi_cmd	cmd;
	__u32	param1;
	__u32	param2;
	__u32	param3;
};

struct rf_phy_cmd_set {
	struct rf_phy_cmd *cmds;
	unsigned int count;
};

struct rf_reg_buf {
	__u32 addr;
	void *buf;
	unsigned int count;
};

struct rf_write_reg_buf {
	__u32 addr;
	__u32 data;
};

struct rf_tx_buf {
	__u32 tx_if;
	__u32 tx_atten;
};

struct rf_tx_en_dis {
	__u32 tx_if;
	__u32 tx_cmd;
};

struct rf_rx_gain {
	int ant;
	int gain_db;
	unsigned int lmt_index;
	unsigned int lmt_gain;
	unsigned int lpf_gain;
	unsigned int digital_gain;
};

struct rf_gain_ctrl {
	int ant;
	enum rf_gain_ctrl_mode mode;
};


struct spi_ioc_transfer {
	__u64		tx_buf;
	__u64		rx_buf;

	__u32		len;
	__u32		speed_hz;

	__u16		delay_usecs;
	__u8		bits_per_word;
	__u8		cs_change;
	__u32		pad;
};

/*struct rf_synth_table {
	unsigned int count;
	unsigned long long * (*) params[NUM_SYNTH_PARAMS];
	__u8 * (*) reg_vals[NUM_SYNTH_REGS];
};*/

#define RF_MAGIC 'A'
#define RF_DEV_INIT		_IOWR(RF_MAGIC, 1, struct rf_init_params*)
#define	RF_RUN_PHY_CMDS		_IOW(RF_MAGIC, 2, struct rf_phy_cmd_set *)
#define RF_READ_PHY_REGS	_IOR(RF_MAGIC, 3, struct rf_reg_buf *)
#define RF_START		_IO(RF_MAGIC, 4)
#define RF_STOP			_IO(RF_MAGIC, 5)
#define RF_WRITE_PHY_REGS	_IOR(RF_MAGIC, 6, struct rf_write_reg_buf *)
#define RF_SET_TX_ATTEN		_IOW(RF_MAGIC, 7, struct rf_tx_buf *)
#define RF_EN_DIS_TX		_IOW(RF_MAGIC, 8, struct rf_tx_en_dis *)
#define RF_READ_RX_GAIN		_IOWR(RF_MAGIC, 9, struct rf_rx_gain *)
#define RF_WRITE_RX_GAIN	_IOW(RF_MAGIC, 10, struct rf_rx_gain *)
/*#define RF_INIT_SYNTH_TABLE	_IOW(RF_MAGIC, 11, struct rf_synth_table *)*/

#define RF_SET_GAIN_CTRL_MODE	_IOW(RF_MAGIC, 21, struct rf_gain_ctrl *)
#define RF_SPI_IOC_TRANSFER	_IOW(RF_MAGIC, 22, struct spi_ioc_transfer *)
#define RF_SET_SPI_STREAM_ID	_IOW(RF_MAGIC, 23, int *)
#define RF_RESET		_IOW(RF_MAGIC, 24, int *)

#endif

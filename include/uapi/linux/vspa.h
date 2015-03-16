/*
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
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

#ifndef _UAPI_VSPA_H
#define _UAPI_VSPA_H

#define VSPA_MAGIC_NUM 'V'

enum vspa_state {
	VSPA_STATE_UNKNOWN = 0,
	VSPA_STATE_POWER_DOWN,
	VSPA_STATE_UNPROGRAMMED_IDLE,
	VSPA_STATE_UNPROGRAMMED_BUSY,
	VSPA_STATE_LOADING,
	VSPA_STATE_STARTUP_ERR,
	VSPA_STATE_RUNNING_IDLE,
	VSPA_STATE_RUNNING_BUSY
};

enum vspa_power {
	VSPA_POWER_DOWN = 0,
	VSPA_POWER_UP,
	VSPA_POWER_CYCLE
};

enum vspa_event_type {
	VSPA_EVENT_NONE = 0,
	VSPA_EVENT_DMA,
	VSPA_EVENT_CMD,
	VSPA_EVENT_REPLY,
	VSPA_EVENT_SPM,
	VSPA_EVENT_MB64_IN,
	VSPA_EVENT_MB32_IN,
	VSPA_EVENT_MB64_OUT,
	VSPA_EVENT_MB32_OUT,
	VSPA_EVENT_ERROR
};

#define VSPA_FLAG_EXPECT_CMD_REPLY	(1<<0)
#define VSPA_FLAG_REPORT_CMD_REPLY	(1<<1)
#define VSPA_FLAG_REPORT_CMD_CONSUMED	(1<<2)
#define VSPA_FLAG_REPORT_DMA_COMPLETE	(1<<3)
#define VSPA_FLAG_REPORT_MB_COMPLETE	(1<<4)


#define VSPA_MSG(x)			(0x10 << x)

#define VSPA_MSG_PEEK			VSPA_MSG(VSPA_EVENT_NONE)
#define VSPA_MSG_DMA			VSPA_MSG(VSPA_EVENT_DMA)
#define VSPA_MSG_CMD			VSPA_MSG(VSPA_EVENT_CMD)
#define VSPA_MSG_REPLY			VSPA_MSG(VSPA_EVENT_REPLY)
#define VSPA_MSG_SPM			VSPA_MSG(VSPA_EVENT_SPM)
#define VSPA_MSG_MB64_IN		VSPA_MSG(VSPA_EVENT_MB64_IN)
#define VSPA_MSG_MB32_IN		VSPA_MSG(VSPA_EVENT_MB32_IN)
#define VSPA_MSG_MB64_OUT		VSPA_MSG(VSPA_EVENT_MB64_OUT)
#define VSPA_MSG_MB32_OUT		VSPA_MSG(VSPA_EVENT_MB32_OUT)
#define VSPA_MSG_ERROR			VSPA_MSG(VSPA_EVENT_ERROR)
#define VSPA_MSG_ALL			(0xFFF0)
#define VSPA_MSG_ALL_EVENTS		(0xFFE0)

#define VSPA_ERR_DMA_CFGERR		(0x10)
#define VSPA_ERR_DMA_XFRERR		(0x11)
#define VSPA_ERR_WATCHDOG		(0x12)

struct vspa_event {
	union {
	  uint32_t	control;
	  struct {
	    uint8_t	id;
	    uint8_t	err;
	    uint16_t	type;
	  };
	};
	uint16_t	pkt_size;
	uint16_t	buf_size;
	uint32_t	data[0];
};

struct vspa_dma_req {
	union {
	  uint32_t	control;
	  struct {
	    uint8_t	id;
	    uint8_t	flags;
	    uint8_t	rsvd;
	    uint8_t	type;
	  };
	};
	uint32_t	dmem_addr;
	uint32_t	axi_addr;
	uint32_t	byte_cnt;
	uint32_t	xfr_ctrl;
};

#define VSPA_MAX_ELD_FILENAME (256)
struct vspa_startup {
	uint32_t	cmd_buf_size;
	uint32_t	cmd_buf_addr;
	uint32_t	spm_buf_size;
	uint32_t	spm_buf_addr;
	uint32_t	watchdog_interval_msecs;
	uint8_t		cmd_dma_chan;
	char		filename[VSPA_MAX_ELD_FILENAME];
};

struct vspa_versions {
	uint32_t	vspa_hw_version;
	uint32_t	ippu_hw_version;
	uint32_t	vspa_sw_version;
	uint32_t	ippu_sw_version;
};

struct vspa_hardware {
	uint32_t	param0;
	uint32_t	param1;
	uint32_t	param2;
	uint32_t	axi_data_width; // bits
	uint32_t	dma_channels;
	uint32_t	gp_out_regs;
	uint32_t	gp_in_regs;
	uint32_t	dmem_bytes;
	uint32_t	ippu_bytes;
	uint32_t	arithmetic_units;
};

struct vspa_reg {
	uint32_t	reg;
	uint32_t	val;
};

struct vspa_mb32 {
	union {
	  uint32_t      control;
	  struct {
	    uint8_t     id;
	    uint8_t     flags;
	    uint8_t     rsvd0;
	    uint8_t     rsvd1;
	  };
	};
	uint32_t        data;
};

struct vspa_mb64 {
	union {
	  uint32_t      control;
	  struct {
	    uint8_t     id;
	    uint8_t     flags;
	    uint8_t     rsvd0;
	    uint8_t     rsvd1;
	  };
	};
	uint32_t        data_msb;
	uint32_t        data_lsb;
};

struct vspa_event_read {
	uint32_t	event_mask; // bit field of events to match
	int		timeout; // delay in mSecs (0 = noblock, -1 = forever)	
	size_t		buf_len; // max reply length in bytes
	struct vspa_event *buf_ptr;
};

/* get VSPA Hardware configuration */
#define VSPA_IOC_GET_HW_CFG	_IOR(VSPA_MAGIC_NUM, 0, struct vspa_hardware)

/* VSPA operational state */
#define VSPA_IOC_GET_STATE	_IOR(VSPA_MAGIC_NUM, 1, int)

/* Read register */
#define VSPA_IOC_REG_READ	_IOR(VSPA_MAGIC_NUM, 2, struct vspa_reg)

/* Write register */
#define VSPA_IOC_REG_WRITE	_IOW(VSPA_MAGIC_NUM, 3, struct vspa_reg)

/* VSPA HW and SW versions */
#define VSPA_IOC_GET_VERSIONS	_IOR(VSPA_MAGIC_NUM, 4, struct vspa_versions)

/* Power Management request for vspa */
#define VSPA_IOC_REQ_POWER	_IO(VSPA_MAGIC_NUM, 5)
/* DMA transaction */
#define VSPA_IOC_DMA		_IOW(VSPA_MAGIC_NUM, 6, struct vspa_dma_req)

/* Startup VSPA core */
#define VSPA_IOC_STARTUP	_IOW(VSPA_MAGIC_NUM, 7, struct vspa_startup)

/* Write Mailbox transactions */
#define VSPA_IOC_MB32_WRITE	_IOW(VSPA_MAGIC_NUM, 8, struct vspa_mb32)
#define VSPA_IOC_MB64_WRITE	_IOW(VSPA_MAGIC_NUM, 9, struct vspa_mb64)

/* Set Watchdog interval */
#define VSPA_IOC_WATCHDOG_INT	_IO(VSPA_MAGIC_NUM, 10)
#define VSPA_WATCHDOG_INTERVAL_DEFAULT	(1000)
#define VSPA_WATCHDOG_INTERVAL_MAX	(60000)

/* Set the event mask used for poll checks */
#define VSPA_IOC_SET_POLL_MASK	_IO(VSPA_MAGIC_NUM, 11)

/* Retrieve the next matching event */
#define VSPA_IOC_EVENT_READ	_IOW(VSPA_MAGIC_NUM, 12, struct vspa_event_read)

/* Driver debug value */
#define VSPA_IOC_GET_DEBUG	_IOR(VSPA_MAGIC_NUM, 13, int)
#define VSPA_IOC_SET_DEBUG	_IO(VSPA_MAGIC_NUM, 14)

#define VSPA_IOC_MAX (14)

#endif /* _UAPI_VSPA_H */

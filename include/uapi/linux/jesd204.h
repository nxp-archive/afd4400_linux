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

#ifndef _UAPI_JESD204_H
#define _UAPI_JESD204_H

enum jesd_state {
	/* JESD is not enabled in idle state */
	JESD_STATE_IDLE,
	/* JESD is configured in ready state */
	JESD_STATE_READY,
	/* Temporary state used by driver to track SYSREF capture */
	JESD_STATE_SYSREF_CAPTURE,
	/* JESD is synced and running */
	JESD_STATE_RUNNING,
	/* Temporary state used by driver to track JESD synchronization */
	JESD_STATE_SYNC_RETRY,
	/* JESD synchronization failure state */
	JESD_STATE_SYNC_FAILURE,
};

enum jesd_init_mask {
	JESD_INIT_SERDES_PLL = (1 << 0),
	JESD_INIT_SERDES = (1 << 1),
	JESD_INIT_REGS = (1 << 2),
};

enum jesd_power_off_mask {
/* This field is used only when both JESD transports are used */
	JESD_POWER_OFF_TRANSPORT0 = (1 << 0),
/* This field is used only when both JESD transports are used */
	JESD_POWER_OFF_TRANSPORT1 = (1 << 1),
/* Power off the serdes related to this JESD block */
	JESD_POWER_OFF_SERDES = (1 << 2),
/* Power off the serdes PLL, after all the serdes lanes are powered off */
	JESD_POWER_OFF_SERDES_PLL = (1 << 3),
};


enum jesd_transport_flag_mask {
	/* If set, will enable the IQ swap */
	IQ_SWAP = (1 << 0),
	/* If set, set the L2SIDES bit for jesd tx reg FRM_CTL */
	L2SIDES = (1 << 1),
	/* If set, RCBUF UNDERFLOW PROCECT will be set for jesd rx, required for rate 491msps */
	RCBUF_UNDERFLOW_PROTECT = (1 << 2),
	/* If set, bit 22 of jesd tx transport control register will be set */
	SYNCN_TO_REF = (1 << 3),
	/* If set, bit 21 of jesd tx transport control register will be set */
	SYNC_FROM_TBGEN = (1 << 4),
};

struct transport_params {
	__u32 jesd_transport_flag;
/* clk divider value. 1: full rate, 2 half rate, 4 quarter rate, 8 1/8 rate */
	__u8 clk_div;
/* sync delay */
	__u8 sync_delay;
/* ils parameters */
/* Device ID */
	__u8 DID;
/* Bank iD */
	__u8 BID;
/* Lane 0 ID */
	__u8 L0ID;
/* Lane 1 ID */
	__u8 L1ID;
/* Scrambling enable, 0 disable, 1 enable */
	__u8 SCR;
/* Number of Octects per frame, 2 or 4
 * Use 2 for dual lane mode and 4 for single lane mode
 */
	__u8 F;
/*
 * Number of frames per multiframe.
 */
	__u8 K;
/*
 * Number of converters per device, use 2 only
 */
	__u8 M;
/*
 * Number of lanes per converter device
 */
	__u8 L;
/*
 * Number of control bits per sample, use 0 only.
 */
	__u8 CS;
/*
 * Number of control words per frame clk per link, use 0 only
 */
	__u8 CF;
/*
 * Converter resolution, use 16 only
 */
	__u8 N;
/*
 * The total number of bits per converter word, use 16 only
 */
	__u8 NP;
/*
 * Samples per converter per frame, use 1 only
 */
	__u8 S;
/*
 * High definition format, use 0 only.
 */
	__u8 HD;

/*
 * Subclass version, 0 or 1.
 */
	__u8 SUBCLASSV;

/*
 * Jesd version, 0 jesd A, 1 jesd B
 */
	__u8 JESDV;
};

enum jesd_complex_flag_mask {
	PHY_GASKET_SWAP = (1 << 0),
	PHY_GASKET_LOOPBACK = (1 << 1),
	SERDES_LOOPBACK = (1 << 2)
};

struct jesd_complex_params {
	__u32 jesd_complex_flag;
	struct transport_params trans0_params;
	struct transport_params trans1_params;
};

#define JESD204_IOCTL 'j'

#define JESD_INIT_PARAM		_IOW(JESD204_IOCTL, 0, \
				struct jesd_init_params *)

#define JESD_INIT_HW	_IOW(JESD204_IOCTL, 1, int)

#define JESD_START	_IOW(JESD204_IOCTL, 2, int)

#define JESD_POWER_DOWN	_IOW(JESD204_IOCTL, 3, int)

#define JESD_FLUSH	_IOW(JESD204_IOCTL, 4, int)

#define JESD_CHANGE_SYNC	_IOW(JESD204_IOCTL, 5, int)

#endif /* _UAPI_JESD204_H */

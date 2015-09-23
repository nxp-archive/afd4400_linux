/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
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

/**
 * @file fsl-d4400-sys.h FSL D4400 System library header
 *
 * @brief This file contains FSL D4400 system data structure to be
 * used by user application.
 *
 */

#include <linux/util-d4400.h>

#ifndef _UAPI_FSL_D4400_SYS_H
#define _UAPI_FSL_D4400_SYS_H

struct d4400_sys_board_info {
	enum board_type board_type;
	enum board_rev board_rev;
};

/* IPMI multi-record Freescale board information */
struct ipmi_mrec_fsl_board_info {
	u32	freqband_MHz;
	u32	features;
};

struct fsl_4t4r_board {
	u32 ipmi_mrec_recordver;
	u32 freqband_MHz;
	u32 features;
	u32 gpio_led1;
	u32 gpio_led2;
};

#define FSL_D4400_SYS_MAGIC 'F'

#define FSL_D4400_SYS_BOARD_INFO		_IOR(FSL_D4400_SYS_MAGIC, 1, \
							struct d4400_sys_board_info)
#define FSL_D4400_SYS_XCVR_PRESENT		_IOR(FSL_D4400_SYS_MAGIC, 2, \
							unsigned int)
#define FSL_D4400_SYS_LEDS_SET_CLEAR		_IOR(FSL_D4400_SYS_MAGIC, 3, \
							unsigned int)
#define FSL_D4400_SYS_SET_REBOOT_METHOD		_IOR(FSL_D4400_SYS_MAGIC, 4, \
							unsigned int)
#endif /* _UAPI_FSL_D4400_SYS_H */

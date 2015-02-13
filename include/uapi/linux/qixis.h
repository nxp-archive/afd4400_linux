/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
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
 * @file qixis.h QIXIS library header
 *
 * @brief This file contains QIXIS data structure to be used by user application.
 *
 */

#ifndef _UAPI_QIXIS_H
#define _UAPI_QIXIS_H

enum qixis_board_type {
	QIXIS_BOARD_TYPE_UNKNOWN = -1,
	QIXIS_BOARD_TYPE_D4400EVB = 0,
	QIXIS_BOARD_TYPE_D4400RDB = 1,
};

enum qixis_board_rev {
	QIXIS_BOARD_REV_UNKNOWN = -1,
	QIXIS_BOARD_REV_A = 0,
	QIXIS_BOARD_REV_B = 1,
};

struct qixis_board_info {
	enum qixis_board_type board_type;
	enum qixis_board_rev board_rev;
};

struct qixis_jcpll_read_buf {
	unsigned int address;
	unsigned char *value_ptr;
};

struct qixis_jcpll_write_buf {
	unsigned int address;
	unsigned char value;
};

#define QIXIS_MAGIC 'Q'

#define QIXIS_BOARD_INFO			_IOR(QIXIS_MAGIC, 1, \
						struct qixis_board_info)
#define QIXIS_JCPLL_FREQ_TRACK			_IOW(QIXIS_MAGIC, 2, \
							unsigned int)
#define QIXIS_JCPLL_LOCKED			_IOR(QIXIS_MAGIC, 3, \
							unsigned int)
#define QIXIS_JCPLL_WRITE			_IOW(QIXIS_MAGIC, 4, \
						struct qixis_jcpll_write_buf)
#define QIXIS_JCPLL_READ			_IOR(QIXIS_MAGIC, 5, \
						struct qixis_jcpll_read_buf)
#define QIXIS_JCPLL_USE_REFC			_IOW(QIXIS_MAGIC, 6, \
							unsigned int)
#define QIXIS_XCVR_PRESENT			_IOR(QIXIS_MAGIC, 7, \
							unsigned int)
#define QIXIS_LEDS_SET_CLEAR			_IOW(QIXIS_MAGIC, 8, \
							unsigned int)
#define QIXIS_SET_REBOOT_METHOD			_IOW(QIXIS_MAGIC, 9, \
							unsigned int)
#endif /* _UAPI_QIXIS_H */

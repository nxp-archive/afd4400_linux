/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
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
struct ipmi_mrec_nxp_board_info {
	u32	freqband_MHz;
	u32	features;
};

/* Bit position of PA control signals (4t4r/4t4rk1) */
enum pa_sig_bit_pos {
	PA_ALARM1_B	= 0,
	PA_ALARM2_B	= 1,

	PA_PSU_EN1	= 2,
	PA_PSU_EN2	= 3,

	PA_OBS_SEL1A	= 4,
	PA_OBS_SEL1B	= 5,
	PA_OBS_SEL2A	= 6,
	PA_OBS_SEL2B	= 7,

	PA_TX_EN1	= 8,
	PA_TX_EN2	= 9,
	PA_TRX_SW1	= 10,
	PA_TRX_SW2	= 11,

	PA_SPARE1	= 12,

	/* PA con2 spare2 conflicts with spi-gpio clk, gpioD 29 */
	/* PA_SPARE2	= 13, */

	/* SS2 conficts with spi7/8 */
	/* PA_SS2	= 14, */

	/* LNA signals conflict with ROC1/2 cs-gpios */
	/* PA_LNA_CTL_1A	= 15,
	   PA_LNA_CTL_1B	= 16,
	   PA_LNA_CTL_2A	= 17,
	   PA_LNA_CTL_2B	= 18,
	*/

	PA_PINCNT,
};

enum pa_num {
	PA1 = 0,
	PA2,
};

#define PA_CON_MAX	2 /* 2 PA connectors */
#define PA_ALARM_MAX	2 /* 2 alarms per PA connector */

/* POC 4t4r/4t4rk1 PA connector control signals */
struct pa_con {
	int pa_alarm_enabled[PA_ALARM_MAX];
	int alarm_irq[PA_ALARM_MAX];
	int pins[PA_PINCNT];
	u32 sig;
			/*
			 * b[0] - PA_CON1_ALARM1_B A4
			 * b[1] - PA_CON1_ALARM2_B A5
			 * b[2] - PA_CON1_PSU_EN1 B4
			 * b[3] - PA_CON1_PSU_EN2 B5
			 * b[4] - PA_CON1_OBS_SEL1A B24
			 * b[5] - PA_CON1_OBS_SEL1B B25
			 * b[6] - PA_CON1_OBS_SEL2A B26
			 * b[7] - PA_CON1_OBS_SEL2B B27
			 * b[8] - PA_CON1_TX_EN1 C1
			 * b[9] - PA_CON1_TX_EN2 C3
			 * b[10] - PA_CON1_TRX_SW1 C9
			 * b[11] - PA_CON1_TRX_SW2 C11
			 * b[12] - PA_CON1_SPARE1 D24
			 * b[13] - PA_CON1_SPARE2 D25
			 * b[14] - PA_CON1_SS2
			 * b[15] - PA_CON1_LNA_CTL_1A C16
			 * b[16] - PA_CON1_LNA_CTL_1B C17
			 * b[17] - PA_CON1_LNA_CTL_2A C18
			 * b[18] - PA_CON1_LNA_CTL_2B C19
			 */
};

/* 4T4RK1 board has 4 rx lna 8v bias enable and 4 rx lna
 * status.
 */
#define RX_8VBIAS_EN_MAX	(4)
#define RX_8VBIAS_STAT_MAX	(4)


struct fsl_4t4r_board {
	u32 ipmi_mrec_ver;
	u32 freqband_MHz;
        /* features: PA wattage, number of Tx/Rx
         * b[31:16] - max PA wattage, A.B watts
         *    b[31:24] - A
         *    b[23:16] - B
         * b[15:8] - number of TX channels
         * b[7:0] - number of RX channels
         */
	u32 features;
	u32 max_pa_watts; /* A.B  b[15:8] - A, b[7:0] - B */
	u32 max_tx;
	u32 max_rx;
	u32 gpio_led1;
	u32 gpio_led2;
	struct pa_con pac[PA_CON_MAX];
	struct delayed_work pa_alarm_irq_wq;

	u32 pa_con1;	/* Bits corresponds to enum pa_sig_bit_pos */
	u32 pa_con2;	/* Bits corresponds to enum pa_sig_bit_pos */
	int pa_con1_pins[PA_PINCNT];
	int pa_con2_pins[PA_PINCNT];
};

struct fsl_4t4rk1_board {
	u32 ipmi_mrec_ver;
	u32 freqband_MHz;
        /* features: PA wattage, number of Tx/Rx
         * b[31:16] - max PA wattage, A.B watts
         *    b[31:24] - A
         *    b[23:16] - B
         * b[15:8] - number of TX channels
         * b[7:0] - number of RX channels
         */
	u32 features;
	u32 max_pa_watts; /* A.B  b[15:8] - A, b[7:0] - B */
	u32 max_tx;
	u32 max_rx;
	u32 gpio_led1;
	u32 gpio_led2;
	u32 gpio_led3;
	u32 gpio_led4;
	u32 gpio_rs485_dir;
	struct pa_con pac[PA_CON_MAX];
	struct delayed_work pa_alarm_irq_wq;

	/* Rx LNA 8v bias enable and status */
	u32 rx_8vbias_en; /* Each bit corresponds to an rx bias control
			   * b[0]-rx1, b[1]-rx2, b[2]-rx3
			   */
	int rx_8vbias_en_pins[RX_8VBIAS_EN_MAX];

	u32 rx_8vbias_stat; /* Each bit corresponds to an rx bias control
			     * b[0]-rx1, b[1]-rx2, b[2]-rx3
			     */
	int rx_8vbias_stat_pins[RX_8VBIAS_STAT_MAX];

	/* PA connector 1 is not available */
	u32 pa_con2;	/* Bits corresponds to enum pa_sig_bit_pos */
	int pa_con2_pins[PA_PINCNT];
};



#define NXP_D4400_SYS_MAGIC 'F'

#define NXP_D4400_SYS_BOARD_INFO		_IOR(NXP_D4400_SYS_MAGIC, 1, \
							struct d4400_sys_board_info)
#define NXP_D4400_SYS_XCVR_PRESENT		_IOR(NXP_D4400_SYS_MAGIC, 2, \
							unsigned int)
#define NXP_D4400_SYS_LEDS_SET_CLEAR		_IOR(NXP_D4400_SYS_MAGIC, 3, \
							unsigned int)
#define NXP_D4400_SYS_SET_REBOOT_METHOD		_IOR(NXP_D4400_SYS_MAGIC, 4, \
							unsigned int)
#endif /* _UAPI_FSL_D4400_SYS_H */

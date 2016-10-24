/*
 * Copyright 2016 NXP, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef D4400_IPMI_MREC_H
#define D4400_IPMI_MREC_H

/* IPMI multi-record NXP OEM record type ID */
#define IPMI_MREC_TID_NXP_RECORDVER     0xc0 /* Record version */
#define IPMI_MREC_TID_NXP_CALDATA       0xc1 /* Calibration data */
#define IPMI_MREC_TID_NXP_CALDATA_PTR   0xc2 /* Calibration data pointer */
#define IPMI_MREC_TID_NXP_BOARDINFO     0xc3 /* Board information */
#define IPMI_MREC_TID_NXP_TX_POWER      0xc4 /* Max transmit power */

/* IPMI multi-record Freescale board information, v1.00 */
struct ipmi_mrec_nxp_board_info_v1_00 {
	u32	freqband_MHz;
	u32	features;
};

/* IPMI multi-record Freescale board information, v1.01 */
struct ipmi_mrec_nxp_board_info_v1_01 {
        /* property1: frequency band in MHz */
        u32     property1;
	/* property2: PA wattage, number of Tx/Rx
	 * b[31:16] - max PA wattage, A.B watts
	 *    b[31:20] - A
	 *    b[19:16] - B (9 or less)
	 * b[15:8] - number of TX channels
	 * b[7:0] - number of RX channels
	 */
        u32     property2;
        /* Placeholder TBD - set to zero*/
        u32     property3;
        /* Placeholder TBD - set to zero*/
        u32     property4;
};

int d4400_mrec_decode_4t4r(struct ipmi_multirecord *mrecs, u32 *version,
	struct fsl_4t4r_board *bi);

int d4400_mrec_decode_4t4rk1(struct ipmi_multirecord *mrecs, u32 *version,
	struct fsl_4t4rk1_board *bi);

#endif /* D4400_IPMI_MREC_H */

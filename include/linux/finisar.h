/*
 * include/linux/finisar.h
 * FINISAR device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __FINISAR_H_
#define __FINISAR_H_

struct finisar_diag_info {
	/* Alarm and Warning Thresholds */
	u8 temp_hi_alarm[2];
	u8 temp_lo_alarm[2];
	u8 temp_hi_warn[2];
	u8 temp_lo_warn[2];
	u8 volt_hi_alarm[2];
	u8 volt_lo_alarm[2];
	u8 volt_hi_warn[2];
	u8 volt_lo_warn[2];
	u8 bias_hi_alarm[2];
	u8 bias_lo_alarm[2];
	u8 bias_hi_warn[2];
	u8 bias_lo_warn[2];
	u8 txpower_hi_alarm[2];
	u8 txpower_lo_alarm[2];
	u8 txpower_hi_warn[2];
	u8 txpower_lo_warn[2];
	u8 rxpower_hi_alarm[2];
	u8 rxpower_lo_alarm[2];
	u8 rxpower_hi_warn[2];
	u8 rxpower_lo_warn[2];
	u8 reserved1[16];
	u8 rx_pwr4[4];
	u8 rx_pwr3[4];
	u8 rx_pwr2[4];
	u8 rx_pwr1[4];
	u8 rx_pwr0[4];
	u8 tx_i_slope[2];
	u8 tx_i_offset[2];
	u8 tx_pwr_slope[2];
	u8 tx_pwr_offset[2];
	u8 temp_slope[2];
	u8 temp_offset[2];
	u8 volt_slope[2];
	u8 volt_offset[2];
	u8 reserved2[3];
	u8 checksum;
	/* A/D Values and Status Bits */
	u8 temp_msb;
	u8 temp_lsb;
	u8 vcc_msb;
	u8 vcc_lsb;
	u8 txbias_msb;
	u8 txbias_lsb;
	u8 txpow_msb;
	u8 txpow_lsb;
	u8 rxpow_msb;
	u8 rxpow_lsb;
	u8 reserved3[4];
	/* Optional Status/Control Bits */
	u8 opt_ctrl_status_flag;
#define TX_DISABLE_STATE	0x80
#define SOFT_TX_DISABLE		0x40
#define RESERVED_BIT5		0x20
#define	RX_RATE_SELECT_STATE	0x10
#define SOFT_RX_RATE_SELECT	0x8
#define TX_FAULT		0x4
#define LOS			0x2
#define DATA_READY_BAR		0x1
	u8 reserved4;
	u8 res_opt_alarm_warn_flag1;
#define TEMP_HIGH_ALARM		0x80
#define TEMP_LOW_ALARM		0x40
#define VCC_HIGH_ALARM		0x20
#define VCC_LOW_ALARM           0x10
#define TX_BIAS_HIGH_ALARM      0x8
#define TX_BIAS_LOW_ALARM       0x4
#define TX_POWER_HIGH_ALARM     0x2
#define TX_POWER_LOW_ALARM      0x1
	u8 res_opt_alarm_warn_flag2;
#define	RX_POWER_HIGH_ALARM	0x80
#define	RX_POWER_LOW_ALARM      0x40
#define	RESERVED_ALARM_BIT5	0x20
#define	RESERVED_ALARM_BIT4	0x10
#define	RESERVED_ALARM_BIT3     0x8
#define	RESERVED_ALARM_BIT2     0x4
#define	RESERVED_ALARM_BIT1     0x2
#define	RESERVED_ALARM_BIT0     0x1
	u8 reserved5;
	u8 reserved6;
	u8 res_opt_alarm_warn_flag3;
#define TEMP_HIGH_WARNING	0x80
#define TEMP_LOW_WARNING        0x40
#define VCC_HIGH_WARNING        0x20
#define VCC_LOW_WARNING         0x10
#define TX_BIAS_HIGH_WARNING    0x8
#define TX_BIAS_LOW_WARNING     0x4
#define TX_POWER_HIGH_WARNING   0x2
#define TX_POWER_LOW_WARNING    0x1
	u8 res_opt_alarm_warn_flag4;
#define RX_POWER_HIGH_WARNING	0x80
#define RX_POWER_LOW_WARNING    0x40
#define RESERVED_WARNING_BIT5	0x20
#define RESERVED_WARNING_BIT4	0x10
#define RESERVED_WARNING_BIT3	0x8
#define RESERVED_WARNING_BIT2	0x4
#define RESERVED_WARNING_BIT1	0x2
#define RESERVED_WARNING_BIT0	0x1
	u8 reserved7;
	u8 reserved8;
	u8 reserved9[3];
	u8 pw3;
	u8 pw2;
	u8 pw1;
	u8 pw0;
	u8 user_eeprom_select;
	u8 user_eeprom[120];
	u8 vendor[8];
};

struct finisar {
	/* Basic ID fields */
	u8 type;
	u8 ext_type;
	u8 connector_type;
	u8 compatibility_code[8];
	u8 encoding;
	u8 bitrate;
	u8 reserved1;
	u8 link_len_9u_km;
	u8 link_len_9u_100m;
	u8 link_len_50u_10m;
	u8 link_len_62_5u_10m;
	u8 link_len_cu_m;
	u8 reserved2;
	u8 vendor_name[16];
	u8 reserved3;
	u8 vendor_oui[3];
	u8 vendor_pn[16];
	u8 vendor_rev[4];
	u8 wavelength[2];
	u8 reserved4;
	u8 check_code_b;
	/* Extended ID fields */
	u8 options[2];
#define RATE_SELECT_BIT5_EN	0x20
#define TX_DISABLE_BIT4_EN	0x10
#define TX_FAULT_BIT3_EN	0x8
#define LOS_INVERTED_BIT2_EN	0x4
#define LOS_BIT1_EN		0x2
	u8 bitrate_max;
	u8 bitrate_min;
	u8 vendor_sn[16];
	u8 manf_date[8];
	u8 diag_type;
	u8 enhanced_options;
	u8 finisar_compliance;
	u8 check_code_e;
	u8 vendor_id_fields[32];
	u8 reserved5[128];
};

#define FINISAR_EEPROM_INFO_SIZE		128
#define FINISAR_BASIC_DATA_SIZE		63
#define FINISAR_EXT_DATA_SIZE		31
#define FINISAR_NUM_ADDR			2
#define FINISAR_EEPROM_NODE_NAME		"eeprom"
#define FINISAR_DIAG_NODE_NAME		"diagnostics"

#endif /* __FINISAR_H_ */

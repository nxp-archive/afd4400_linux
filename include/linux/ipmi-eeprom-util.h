/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef IPMI_EEPROM_UTIL_H
#define IPMI_EEPROM_UTIL_H

#include <linux/unistd.h>
#include <linux/linkage.h>

struct months_data {
	u8 str[4];
	int days;
};

/*
 * The implementation of the Intelligent Platform Management Interface
 * (IPMI) information is based on the Intel definition.
 */

/* Common Header area */
struct ipmi_common_hdr {
	u8 format_ver;
	u8 internal_use_offset;
	u8 chassis_offset;
	u8 board_offset;
	u8 product_offset;
	u8 multirecord_offset;
	u8 pad;

	u8 checksum;
};

/* Internal Use area */
struct ipmi_internal_use {
	u8 format_ver;
	u8 *data;
	u32 size; /* Size of area */
};

/* Chassis Info area */
struct ipmi_chassis_info {
	u8 format_ver;
	u8 lenx8;
	u8 chassis_type;
	u8 partnum_type;
	u8 partnum_len;
	char *partnum_str;
	u8 serial_type;
	u8 serial_len;
	char *serial_str;
	u8 *custom_field;

	u8 checksum;
	u32 size; /* Size of area */
};

/* Board Info area */
struct ipmi_board_info {
	u8 format_ver;
	u8 lenx8;
	u8 language_code;
	u32 mfg_year;
	u32 mfg_month;
	u32 mfg_day;
	u8 mfg_type;
	u8 mfg_len;
	char *mfg_str;
	u8 name_type;
	u8 name_len;
	char *name_str;
	u8 serial_type;
	u8 serial_len;
	char *serial_str;
	u8 partnum_type;
	u8 partnum_len;
	char *partnum_str;
	u8 ipmi_fileid_type;
	u8 ipmi_fileid_len;
	char *ipmi_fileid_str;

	u8 checksum;
	u32 size; /* Size of area */
};

/* Product Info area */
struct ipmi_product_info {
	u8 format_ver;
	u8 lenx8;
	u8 language_code;
	u8 mfg_type;
	u8 mfg_len;
	char *mfg_str;
	u8 name_type;
	u8 name_len;
	char *name_str;
	u8 partmodel_type;
	u8 partmodel_len;
	char *partmodel_str;
	u8 ver_type;
	u8 ver_len;
	char *ver_str;
	u8 serial_type;
	u8 serial_len;
	char *serial_str;
	u8 assettag_type;
	u8 assettag_len;
	char *assettag_str;
	u8 ipmi_fileid_type;
	u8 ipmi_fileid_len;
	char *ipmi_fileid_str;
	u8 *custom_field;

	u8 checksum;
	u32 size; /* Size of area */
};

#define IPMI_MULTIREC_REC_SIZE	5	/* IPMI v1.1 */

/* Record sub-area */
struct ipmi_record_hdr {
	u8 type_id;
	u8 list_ver;
	u8 len;
	u8 rec_csum; /* Checksum of record data */
	u8 hdr_csum; /* Checksum of the header only */

	u8 *data;
};

/* MultiRecord area */
struct ipmi_multirecord {
	int num_rec;
	struct ipmi_record_hdr **rechdr;
	u32 size; /* Size of area */
};

/* Top level IPMI information. */
struct ipmi_info {
	struct ipmi_common_hdr		common_hdr;
	struct ipmi_internal_use	internal_use;
	struct ipmi_chassis_info	chassis;
	struct ipmi_board_info		board;
	struct ipmi_product_info	product;
	struct ipmi_multirecord		multirec;

	u8 *rawbuf;
};

#define IPMI_COMMOM_HDR_SIZE	(8)
#define IPMI_EEPROM_DATA_SIZE	(256)
#define IPMI_INFO_STRUCT_SIZE	(sizeof(ipmi_info))

int ipmi_create(u8 *ipmi_rawbuf, struct ipmi_info *ipmi);
void ipmi_free(struct ipmi_info *ipmi);
void ipmi_print_common_hdr(struct ipmi_common_hdr *common_hdr);
void ipmi_print_board_info(struct ipmi_board_info *board);
void ipmi_printstr_record_hdr(char **p, struct ipmi_record_hdr *rechdr);

#endif /* IPMI_EEPROM_UTIL_H */

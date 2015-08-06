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

#ifndef __UAPI_XCVR_IF_H
#define __UAPI_XCVR_IF_H

#include <linux/ipmi-eeprom-util.h>
#include <linux/xcvr_if.h>

#define XCVERIF_IPMI_STR_SIZE	4096

enum xcvrif_eeprom_rw_flag {
	EEPROM_READ_OP = 0,
	EEPROM_WRITE_OP = 1,
};

struct xcvr_sys_info {
	int xcvr_cnt;		/* Number of xcvr cards detected */
};

struct xcvr_info {
	int index;		/* Index of xcvr card */
	int *eeprom_size;	/* Eeprom max size */
	int *eeprom_pagesize;	/* Eeprom page size */
	char *user_ipmi_str;	/* Impi text desc buffer in user space */
};

struct xcvr_eeprom {
	int index;		/* Index of xcvr card */
	int rw_flag;		/* Read or write operation */
	int offset;		/* Memory offset, starting loc */
	int len;		/* Length */
	u8  *user_buf;		/* Data buffer in user space */
};

struct xcvr_ipmi_info {
	int index;		/* Index of xcvr card */
	char *ipmi_str;		/* Impi text desc */
};

#define XCVRIF_MAGIC 'X'

#define XCVRIF_READ_SYS_INFO _IOR(XCVRIF_MAGIC, 0, struct xcvr_sys_info *)
#define XCVRIF_READ_XCVR_INFO _IOR(XCVRIF_MAGIC, 1, struct xcvr_info *)
#define XCVRIF_RW_EEPROM _IOR(XCVRIF_MAGIC, 2, struct xcvr_eeprom *)

#endif /* _UAPI_XCVR_H */

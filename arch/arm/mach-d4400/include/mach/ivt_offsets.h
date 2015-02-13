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

#ifndef __LINUX_IVT_OFFSETS_H
#define __LINUX_IVT_OFFSETS_H

/* Image Vector Table */
#define D4400_IVT_BARKER_CODE		0x01020304
#define D4400_IVT_TABLE_SIZE_BYTES	(4 * 8)	/* 8 32-bit words */
struct _ivt_table {
	u32 barker;
	u32 app_entry_ptr;
	u32 reserved1;
	u32 dcd_ptr;
	u32 boot_data_ptr;
	u32 ivt_ptr;
	u32 hash_ptr;
	u32 second_img_ptr;
};

union _ivt_data {
	u32 data32[D4400_IVT_TABLE_SIZE_BYTES / 4];
	struct _ivt_table ivt_table;
};

struct _boot_data {
	/*
	 * The start pointer is the *destination* location in DDR
	 * where the ivt/dcd/u-boot/hash will be copied to from flash.
	 */
	u32	*dest_ptr;
	u32	size_bytes;
};

/* DCD data */
struct _dcd_entry {
	u32	type;
	u32	addr;
	u32	data32;
};
struct _dcd_preamble {
	u32	barker;
	/* Size does NOT include preamble (barker and byte size) */
	u32	size_bytes;
};
struct _dcd_data {
	struct _dcd_preamble preamble;
	struct _dcd_entry entry[200];	/* Maximum of 200 per specification */
};
#endif /* __LINUX_IVT_OFFSETS_H */

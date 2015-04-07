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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ipmi-eeprom-util.h>

struct months_data months[] = {
	{ "Jan\0", 31 },
	{ "Feb\0", 28 },
	{ "Mar\0", 31 },
	{ "Apr\0", 30 },
	{ "May\0", 31 },
	{ "Jun\0", 30 },
	{ "Jul\0", 31 },
	{ "Aug\0", 31 },
	{ "Sep\0", 30 },
	{ "Oct\0", 31 },
	{ "Nov\0", 30 },
	{ "Dec\0", 31 }
};

/* For debug
void ipmi_print_common_hdr(struct ipmi_common_hdr *common_hdr)
{
	printk("\n");
	printk("common_hdr->format_ver          0x%02x\n",
		common_hdr->format_ver);
	printk("common_hdr->internal_use_offset 0x%02x\n",
		common_hdr->internal_use_offset);
	printk("common_hdr->chassis_offset      0x%02x\n",
		common_hdr->chassis_offset);
	printk("common_hdr->board_offset        0x%02x\n",
		common_hdr->board_offset);
	printk("common_hdr->product_offset      0x%02x\n",
		common_hdr->product_offset);
	printk("common_hdr->multirecord_offset  0x%02x\n",
		common_hdr->multirecord_offset);
	printk("common_hdr->pad                 0x%02x\n",
		common_hdr->pad );
	printk("common_hdr->checksum            0x%02x\n",
		common_hdr->checksum);
	printk("\n");
}
*/

/* For debug
void ipmi_print_board_info(struct ipmi_board_info *board)
{
	printk("\n");
	printk("board->format_ver               0x%02x\n",
		board->format_ver);
	printk("board->lenx8                    0x%02x\n",
		board->lenx8);
	printk("board->language_code            0x%02x\n",
		board->language_code);

	printk("board->mfg_year                 %i\n",
		board->mfg_year);
	printk("board->mfg_month                %s / %i\n",
		months[board->mfg_month].str, board->mfg_month+1);
	printk("board->mfg_day                  %i\n",
		board->mfg_day);

	printk("board->mfg_type                 0x%02x\n",
		board->mfg_type);
	printk("board->mfg_len                  0x%02x\n",
		board->mfg_len);
	printk("board->mfg_str                  %s\n",
		board->mfg_str);

	printk("board->name_type                0x%02x\n",
		board->name_type);
	printk("board->name_len                 0x%02x\n",
		board->name_len);
	printk("board->name_str                 %s\n",
		board->name_str);

	printk("board->serial_type              0x%02x\n",
		board->serial_type);
	printk("board->serial_len               0x%02x\n",
		board->serial_len);
	printk("board->serial_str               %s\n",
		board->serial_str);

	printk("board->partnum_type             0x%02x\n",
		board->partnum_type);
	printk("board->partnum_len              0x%02x\n",
		board->partnum_len);
	printk("board->partnum_str              %s\n",
		board->partnum_str);

	printk("board->ipmi_fileid_type          0x%02x\n",
		board->ipmi_fileid_type);
	printk("board->ipmi_fileid_len           0x%02x\n",
		board->ipmi_fileid_len);
	printk("board->ipmi_fileid_str           %s\n",
		board->ipmi_fileid_str);

	printk("board->checksum                 0x%02x\n",
		board->checksum);
	printk("\n");
}
*/

static void ipmi_cal_date(int minutes, u32 *year, u32 *month, u32 *day)
{
	int i;

	/* Date is specified as the passage of time in minutes
	 * since 1/1/96.
	 */
	*day = (minutes / 60 / 24);
	*year = (*day / 365);
	*day = *day - (*year * 365);
	*year += 1996;

	*month = 0;
	for (i = 0; i < 12; ++i) {
		if (*day <= months[i].days) {
			*month = i;
			break;
		} else
			*day -= months[i].days;
	}
}

static int ipmi_verify_checksum(u8 *buf, int offset, int num)
{
	int i;
	u8 checksum = 0;

	if ((offset > IPMI_EEPROM_DATA_SIZE)
	     || ((offset+num) > IPMI_EEPROM_DATA_SIZE)
	     || (num < 2)) /* Last byte in buf is expected to be checksum */
		return -1;

	for (i = offset; i < (num-1); ++i)
		checksum += buf[i];
	checksum = ~checksum + 1; /* 2's complement */

	if (checksum != buf[i])
		return -1;
	return 0;
}

static void ipmi_create_common_hdr(u8 *ipmi_rawbuf,
	struct ipmi_common_hdr *common_hdr)
{
	common_hdr->format_ver		= ipmi_rawbuf[0];
	common_hdr->internal_use_offset	= ipmi_rawbuf[1];
	common_hdr->chassis_offset	= ipmi_rawbuf[2];
	common_hdr->board_offset	= ipmi_rawbuf[3];
	common_hdr->product_offset	= ipmi_rawbuf[4];
	common_hdr->multirecord_offset	= ipmi_rawbuf[5];
	common_hdr->pad			= ipmi_rawbuf[6];
	common_hdr->checksum		= ipmi_rawbuf[7];
}

static inline int _inline_ipmi_make_str(char **deststr, u8 *srcstr, int size)
{
	if (size >= 1) {
		/* +1 for string null termination */
		*deststr = kzalloc(size+1, GFP_KERNEL);
		if (!*deststr)
			return -ENOMEM;
		memcpy(*deststr, srcstr, size);
	}
	return 0;
}

static int ipmi_create_board_info(u8 *ipmi_board_buf,
	struct ipmi_board_info *board)
{
	int err, len, offset, minutes;

	/* Length is in multiples of 8 bytes */
	len = ipmi_board_buf[1] * 8;

	/* Verify checksum */
	if (ipmi_verify_checksum(ipmi_board_buf, 0, len))
		return -1;

	offset = 0;
	board->format_ver	= ipmi_board_buf[offset] & 0x0f;

	offset += 1;
	board->lenx8		= ipmi_board_buf[offset] * 8;

	offset += 1;
	board->language_code	= ipmi_board_buf[offset];

	offset += 1;
	/* Date in minutes, 3 bytes, lsb first */
	minutes = (((int)ipmi_board_buf[offset+2]<<16) |
		    ((int)ipmi_board_buf[offset+1]<<8) |
		    ((int)ipmi_board_buf[offset+0]<<0));
	ipmi_cal_date(minutes, &board->mfg_year, &board->mfg_month,
		&board->mfg_day);

	offset += 3;
	board->mfg_type		= ipmi_board_buf[offset] >> 6;
	board->mfg_len		= ipmi_board_buf[offset] & 0x3f;
	offset += 1;
	err = _inline_ipmi_make_str(&board->mfg_str, &ipmi_board_buf[offset],
		board->mfg_len);
	if (err)
		goto err_out;

	offset += board->mfg_len;
	board->name_type	= ipmi_board_buf[offset] >> 6;
	board->name_len		= ipmi_board_buf[offset] & 0x3f;
	offset += 1;
	err = _inline_ipmi_make_str(&board->name_str, &ipmi_board_buf[offset],
		board->name_len);
	if (err)
		goto err_out;

	offset += board->name_len;
	board->serial_type	= ipmi_board_buf[offset] >> 6;
	board->serial_len	= ipmi_board_buf[offset] & 0x3f;
	offset += 1;
	err = _inline_ipmi_make_str(&board->serial_str, &ipmi_board_buf[offset],
		board->serial_len);
	if (err)
		goto err_out;

	offset += board->serial_len;
	board->partnum_type	= ipmi_board_buf[offset] >> 6;
	board->partnum_len	= ipmi_board_buf[offset] & 0x3f;
	offset += 1;
	err = _inline_ipmi_make_str(&board->partnum_str, &ipmi_board_buf[offset],
		board->partnum_len);
	if (err)
		goto err_out;

	offset += board->partnum_len;
	board->ipmi_fileid_type	= ipmi_board_buf[offset] >> 6;
	board->ipmi_fileid_len	= ipmi_board_buf[offset] & 0x3f;
	offset += 1;
	err = _inline_ipmi_make_str(&board->ipmi_fileid_str,
			&ipmi_board_buf[offset],
		board->ipmi_fileid_len);
	if (err)
		goto err_out;

	offset += board->ipmi_fileid_len;

	/* Customarily, following the IPMI File id is value 0xc0
	 * to indicate a zero field.  After that could be custom
	 * OEM information.  This data is not saved as its meaning
	 * is defined by the OEM.  A value 0xc1 is then expected
	 * to indicate no more fields.  The last byte in the board
	 * info area is the mandatory checksum.
	 */
	board->checksum		= len-1;
	return 0;

err_out:
	return err;
}

void ipmi_free(struct ipmi_info *ipmi)
{
	if (!ipmi)
		return;
	kfree(ipmi->internal_use.data);
	kfree(ipmi->chassis.partnum_str);
	kfree(ipmi->chassis.serial_str);

	kfree(ipmi->board.mfg_str);
	kfree(ipmi->board.name_str);
	kfree(ipmi->board.serial_str);
	kfree(ipmi->board.partnum_str);
	kfree(ipmi->board.ipmi_fileid_str);

	kfree(ipmi->product.mfg_str);
	kfree(ipmi->product.name_str);
	kfree(ipmi->product.partmodel_str);
	kfree(ipmi->product.ver_str);
	kfree(ipmi->product.serial_str);
	kfree(ipmi->product.assettag_str);
	kfree(ipmi->product.ipmi_fileid_str);
}

int ipmi_create(u8 *ipmi_rawbuf, struct ipmi_info *ipmi)
{
	int err, offset;

	if ((!ipmi) || (!ipmi_rawbuf))
		return -1;

	/* Null all data and pointers.  Asumes this is a new object. */
	memset(ipmi, 0, sizeof(struct ipmi_info));

	/* Verify checksum of common header, located at the beginning of
	 * the buffer, as this area is mandatory.
	 */
	if (ipmi_verify_checksum(ipmi_rawbuf, 0, IPMI_COMMOM_HDR_SIZE))
		return -1;

	/* Common header */
	ipmi_create_common_hdr(ipmi_rawbuf, &ipmi->common_hdr);

	/* Board info */
	if (ipmi->common_hdr.board_offset) {
		/* Offset is in 8 byte increments */
		offset = ipmi->common_hdr.board_offset * 8;
		err = ipmi_create_board_info(&ipmi_rawbuf[offset], &ipmi->board);
		if (err)
			goto err_out;
	}
	/* TODO: Implement Internal Use info */
	/* TODO: Implement Chassis info */
	/* TODO: Implement Product info */
	/* TODO: Implement Multirecord info */

	return 0;

err_out:
	ipmi_free(ipmi);
	return -1;
}

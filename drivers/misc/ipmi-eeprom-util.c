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

/* For debug
void ipmi_print_record_hdr(struct ipmi_record_hdr *record_hdr)
{
	int i;

	printk("\n");
	printk("record_hdr->type_id          0x%02x\n",
		record_hdr->type_id);
	printk("record_hdr->list_ver         0x%02x\n",
		record_hdr->list_ver);
	printk("record_hdr->len              0x%02x\n",
		record_hdr->len);
	printk("record_hdr->rec_csum         0x%02x\n",
		record_hdr->rec_csum);
	printk("record_hdr->hdr_csum         0x%02x\n",
		record_hdr->hdr_csum);
	for(i = 0; i < record_hdr->len; ++i) {
		if ((i % 16) == 0)
			printk("\n%02i:", i);
		printk(" %02x", record_hdr->data[i]);
	}
	printk("\n");
}
*/

void ipmi_printstr_record_hdr(char **p, struct ipmi_record_hdr *rechdr)
{
	int i;

	if (!rechdr)
		return;

	sprintf(*p, "\tId: x%02x Listver: x%02x Len: %i\n",
		rechdr->type_id, rechdr->list_ver, rechdr->len);
	*p += strlen(*p);

	for (i = 0; i < rechdr->len; ++i) {
		if ((i % 16) == 0) {
			if (i > 0) {
				sprintf(*p, "\n");
				*p += strlen(*p);
			}
			sprintf(*p, "\t  %02i:", i);
			*p += strlen(*p);
		}
		sprintf(*p, " %02x", (unsigned char)rechdr->data[i]);
		*p += strlen(*p);
	}
	sprintf(*p, "\n");
	*p += strlen(*p);
}

void ipmi_free_mrec(struct ipmi_multirecord *mrec)
{
	int i;

	if ((mrec->rechdr) && (mrec->num_rec > 0)) {
		for (i = 0; i < mrec->num_rec; ++i) {
			kfree(mrec->rechdr[i]->data);
			kfree(mrec->rechdr[i]);
		}
		kfree(mrec->rechdr);
	}
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

	kfree(ipmi->rawbuf);

	ipmi_free_mrec(&ipmi->multirec);
}

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

/* Checksum byte is expected to be at the end of data, buf[offset-1] */
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

static int ipmi_verify_checksum_dat(u8 *buf, int offset, int num, u8 csum)
{
	int i;
	u8 checksum = 0;

	if ((offset > IPMI_EEPROM_DATA_SIZE)
	     || ((offset+num) > IPMI_EEPROM_DATA_SIZE))
		return -1;

	for (i = offset; i < num; ++i)
		checksum += buf[i];
	checksum = ~checksum + 1; /* 2's complement */

	if (checksum != csum)
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
	board->size = len;

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
	err = _inline_ipmi_make_str(&board->partnum_str,
		&ipmi_board_buf[offset], board->partnum_len);
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

static int ipmi_create_multirec_info(u8 *ipmi_multirec_buf,
	struct ipmi_multirecord *mrec)
{
	int err = 0;
	int i;
	struct ipmi_record_hdr tmprechdr;
	struct ipmi_record_hdr *rechdr;
	int num_rec, rec;

	/* First pass, find out how many valid records */
	i = num_rec = 0;
	do {
		if (ipmi_verify_checksum(&ipmi_multirec_buf[i], 0,
			IPMI_MULTIREC_REC_SIZE))
			break;

		/* Copy each value individually to local struct instead of
		 * setting a record header ptr to the raw data to avoid
		 * data alignment issues.
		 */
		tmprechdr.type_id = ipmi_multirec_buf[i++];
		tmprechdr.list_ver = ipmi_multirec_buf[i++];
		tmprechdr.len = ipmi_multirec_buf[i++];
		tmprechdr.rec_csum = ipmi_multirec_buf[i++];
		tmprechdr.hdr_csum = ipmi_multirec_buf[i++];

		i += tmprechdr.len;
		++num_rec;

		/* Check for end of list, b[7] = 1 */
		if (tmprechdr.list_ver & 0x80)
			break;

	} while (i < (IPMI_EEPROM_DATA_SIZE - IPMI_COMMOM_HDR_SIZE));

	/* Save the size of the area */
	mrec->size = i;

	/* Create array of record header pointers */
	mrec->rechdr = kzalloc(sizeof(struct ipmi_record_hdr *) * num_rec,
		GFP_KERNEL);
	if (!mrec->rechdr) {
		err = -ENOMEM;
		goto err_out;
	}

	/* Second pass to save all the records including their data */
	mrec->num_rec = 0;
	i = 0;
	for (rec = 0; rec < num_rec; ++rec) {

		rechdr = kzalloc(sizeof(struct ipmi_record_hdr),
			GFP_KERNEL);
		if (!rechdr) {
			err = -ENOMEM;
			goto err_out0;
		}

		rechdr->type_id = ipmi_multirec_buf[i++];
		rechdr->list_ver = ipmi_multirec_buf[i++];
		rechdr->len = ipmi_multirec_buf[i++];
		rechdr->rec_csum = ipmi_multirec_buf[i++];
		rechdr->hdr_csum = ipmi_multirec_buf[i++];

		/* Record data which resides just after the record info
		 * according to IPMI spec v1.1.
		 */
		rechdr->data = kzalloc(rechdr->len, GFP_KERNEL);
		if (!rechdr->data) {
			kfree(rechdr);
			err = -ENOMEM;
			goto err_out0;
		}
		memcpy(rechdr->data, &ipmi_multirec_buf[i], rechdr->len);

		if (ipmi_verify_checksum_dat(rechdr->data, 0, rechdr->len,
			rechdr->rec_csum)) {
			kfree(rechdr->data);
			kfree(rechdr);
			err = -EINVAL;
			goto err_out0;
		}
		mrec->rechdr[rec] = rechdr;
		++mrec->num_rec;
		i += rechdr->len;
	}
	return err;

err_out0:
	ipmi_free_mrec(mrec);
err_out:
	return err;
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
		err = ipmi_create_board_info(&ipmi_rawbuf[offset],
			&ipmi->board);
		if (err)
			goto err_out;
	}

	/* Multirec info */
	if (ipmi->common_hdr.multirecord_offset) {
		/* Offset is in 8 byte increments */
		ipmi->multirec.num_rec = 0;
		offset = ipmi->common_hdr.multirecord_offset * 8;
		err = ipmi_create_multirec_info(&ipmi_rawbuf[offset],
			&ipmi->multirec);
		if (err)
			goto err_out;
	}
	/* TODO: Implement Internal Use info */
	/* TODO: Implement Chassis info */
	/* TODO: Implement Product info */

	/* Keep a copy of the raw buffer as it may be need to be
	 * modified for updates.  In most cases it may be easier
	 * to modify the raw ipmi buffer directly than to reconstruct
	 * it from the ipmi data structures.
	 */
	ipmi->rawbuf = kzalloc(IPMI_EEPROM_DATA_SIZE, GFP_KERNEL);
	memcpy(ipmi->rawbuf, ipmi_rawbuf, IPMI_EEPROM_DATA_SIZE);

	return 0;

err_out:
	ipmi_free(ipmi);
	return -1;
}

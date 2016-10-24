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

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/linkage.h>
#include <linux/slab.h>
#include <linux/ipmi-eeprom-util.h>
#include <uapi/linux/fsl-d4400-sys.h>
#include <linux/ipmi-mrec-d4400.h>

/* #define kzalloc(size, flags)    ((char*)malloc(size))
 * #define kfree(ptr)              free(ptr)
 */

static int d4400_mrec_decode_4t4r_v100(struct ipmi_record_hdr **rechdr, int num_rec,
	struct fsl_4t4r_board *bi)
{
	int ret = 0;
	int i;
	struct ipmi_record_hdr *hdr;
	struct ipmi_mrec_nxp_board_info_v1_00 *mrec_bi;

	if (!bi)
		return -EINVAL;

	for (i = 0; i < num_rec; ++i) {
		hdr = rechdr[i];

		switch (hdr->type_id) {
		case IPMI_MREC_TID_NXP_RECORDVER:
			/* Already processed */
			break;
		case IPMI_MREC_TID_NXP_BOARDINFO:
			mrec_bi = (struct ipmi_mrec_nxp_board_info_v1_00 *)
				hdr->data;
			bi->freqband_MHz = mrec_bi->freqband_MHz;
			bi->features = mrec_bi->features;
		}
	}
	return ret;
}

static int d4400_mrec_decode_4t4r_v101(struct ipmi_record_hdr **rechdr, int num_rec,
	struct fsl_4t4r_board *bi)
{
	int ret = 0;
	int i;
	struct ipmi_record_hdr *hdr;
	struct ipmi_mrec_nxp_board_info_v1_01 *mrec_bi;

	if (!bi)
		return -EINVAL;

	for (i = 0; i < num_rec; ++i) {
		hdr = rechdr[i];

		switch (hdr->type_id) {
		case IPMI_MREC_TID_NXP_RECORDVER:
			/* Already processed */
			break;
		case IPMI_MREC_TID_NXP_CALDATA:
		case IPMI_MREC_TID_NXP_CALDATA_PTR:
			/* Nothing to do for now */
			break;
		case IPMI_MREC_TID_NXP_BOARDINFO:
			/* Multi-record board info */
			mrec_bi = (struct ipmi_mrec_nxp_board_info_v1_01 *)
				hdr->data;
			bi->freqband_MHz = mrec_bi->property1;
			bi->max_pa_watts = (mrec_bi->property2 >> 16) & 0xffff;
			bi->max_tx = (mrec_bi->property2 >> 8) & 0xff;
			bi->max_rx = (mrec_bi->property2 >> 0) & 0xff;
			bi->features = 0;
		}
	}
	return ret;
}

/*
 * d4400_mrec_decode_4t4r - Decodes the given IPMI multi-record.
 *
 * mrecs - input
 * version - output
 * board_info - output
 */
int d4400_mrec_decode_4t4r(struct ipmi_multirecord *mrecs, u32 *version,
	struct fsl_4t4r_board *bi)
{
	int ret = 0;
	int i;
	struct ipmi_record_hdr *hdr;

	/* First, search for version to know how to decode mrec */
	*version = 0;
	for (i = 0; i < mrecs->num_rec; ++i) {
		hdr = mrecs->rechdr[i];
		if ( (hdr->type_id == IPMI_MREC_TID_NXP_RECORDVER) &&
			(hdr->len == 4)) {
			/* Found record version */
			*version =
				(u32)(hdr->data[0] << 0) |
				(u32)(hdr->data[1] << 8) |
				(u32)(hdr->data[2] << 16) |
				(u32)(hdr->data[3] << 24);
		}
	}
	/* Process records according to version */
	switch (*version) {
	case 0x00010000: /* v01.00 */
		ret = d4400_mrec_decode_4t4r_v100(mrecs->rechdr,
			mrecs->num_rec, bi);
		break;
	case 0x00010001: /* v01.01 */
		ret = d4400_mrec_decode_4t4r_v101(mrecs->rechdr,
			mrecs->num_rec, bi);
		break;
	default:
	case 0:
		/* IPMI multirec version not found */
		ret = -EINVAL;
		break;
	};
	return ret;
}

static int d4400_mrec_decode_4t4rk1_v100(struct ipmi_record_hdr **rechdr, int num_rec,
	struct fsl_4t4rk1_board *bi)
{
	int ret = 0;
	int i;
	struct ipmi_record_hdr *hdr;
	struct ipmi_mrec_nxp_board_info_v1_00 *mrec_bi;

	if (!bi)
		return -EINVAL;

	for (i = 0; i < num_rec; ++i) {
		hdr = rechdr[i];

		switch (hdr->type_id) {
		case IPMI_MREC_TID_NXP_RECORDVER:
			/* Already processed */
			break;
		case IPMI_MREC_TID_NXP_BOARDINFO:
			mrec_bi = (struct ipmi_mrec_nxp_board_info_v1_00 *)
				hdr->data;
			bi->freqband_MHz = mrec_bi->freqband_MHz;
			bi->features = mrec_bi->features;
		}
	}
	return ret;
}

static int d4400_mrec_decode_4t4rk1_v101(struct ipmi_record_hdr **rechdr, int num_rec,
	struct fsl_4t4rk1_board *bi)
{
	int ret = 0;
	int i;
	struct ipmi_record_hdr *hdr;
	struct ipmi_mrec_nxp_board_info_v1_01 *mrec_bi;

	if (!bi)
		return -EINVAL;

	for (i = 0; i < num_rec; ++i) {
		hdr = rechdr[i];

		switch (hdr->type_id) {
		case IPMI_MREC_TID_NXP_RECORDVER:
			/* Already processed */
			break;
		case IPMI_MREC_TID_NXP_CALDATA:
		case IPMI_MREC_TID_NXP_CALDATA_PTR:
			/* Nothing to do for now */
			break;
		case IPMI_MREC_TID_NXP_BOARDINFO:
			/* Multi-record board info */
			mrec_bi = (struct ipmi_mrec_nxp_board_info_v1_01 *)
				hdr->data;
			bi->freqband_MHz = mrec_bi->property1;
			bi->max_pa_watts = (mrec_bi->property2 >> 16) & 0xffff;
			bi->max_tx = (mrec_bi->property2 >> 8) & 0xff;
			bi->max_rx = (mrec_bi->property2 >> 0) & 0xff;
			bi->features = 0;
		}
	}
	return ret;
}

/*
 * d4400_mrec_decode_4t4rk1 - Decodes the given IPMI multi-record.
 *
 * mrecs - input
 * version - output
 * board_info - output
 */
int d4400_mrec_decode_4t4rk1(struct ipmi_multirecord *mrecs, u32 *version,
	struct fsl_4t4rk1_board *bi)
{
	int ret = 0;
	int i;
	struct ipmi_record_hdr *hdr;

	/* First, search for version to know how to decode mrec */
	*version = 0;
	for (i = 0; i < mrecs->num_rec; ++i) {
		hdr = mrecs->rechdr[i];
		if ( (hdr->type_id == IPMI_MREC_TID_NXP_RECORDVER) &&
			(hdr->len == 4)) {
			/* Found record version */
			*version =
				(u32)(hdr->data[0] << 0) |
				(u32)(hdr->data[1] << 8) |
				(u32)(hdr->data[2] << 16) |
				(u32)(hdr->data[3] << 24);
		}
	}
	/* Process records according to version */
	switch (*version) {
	case 0x00010000: /* v01.00 */
		ret = d4400_mrec_decode_4t4rk1_v100(mrecs->rechdr,
			mrecs->num_rec, bi);
		break;
	case 0x00010001: /* v01.01 */
		ret = d4400_mrec_decode_4t4rk1_v101(mrecs->rechdr,
			mrecs->num_rec, bi);
		break;
	default:
	case 0:
		/* IPMI multirec version not found */
		ret = -EINVAL;
		break;
	};
	return ret;
}

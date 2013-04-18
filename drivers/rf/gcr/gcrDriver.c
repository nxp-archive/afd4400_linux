/* GCR configuration driver
 * Authod: Anand Singh
 * Anand Singh<b44195@freescale.com>
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/cache.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/of_address.h>
#include <linux/gcr.h>

#define DRIVER_NAME "gcr_dev"
#define MOD_VERSION "0.1"

static int gcr_open(struct inode *inode, struct file *filp);
static int gcr_release(struct inode *inode, struct file *filp);
static long gcr_ctrl(struct file *filp, unsigned int cmd, unsigned long arg);
static int gcr_cfg_probe(struct platform_device *ofdev);
static int gcr_remove(struct platform_device *ofdev);

static const struct file_operations gcr_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= gcr_ctrl,
	.open		= gcr_open,
	.release	= gcr_release,
};


int jesd_conf_val_set1(unsigned char jesd_chan)
{
	int val = 0;
	switch (jesd_chan) {
	case 1:
		val = JESDTX_DMA_REQ(1);
		break;
	case 2:
		val = JESDTX_DMA_REQ(2);
		break;
	case 3:
		val = JESDTX_DMA_REQ(3);
		break;
	case 4:
		val = JESDTX_DMA_REQ(4);
		break;
	case 5:
		val = JESDTX_DMA_REQ(5);
		break;
	case 6:
		val = JESDTX_DMA_REQ(6);
		break;
	case 7:
		val = JESDTX_DMA_REQ(7);
		break;
	case 8:
		val = JESDTX_DMA_REQ(8);
		break;
	case 9:
		val = JESDTX_DMA_REQ(9);
		break;
	case 10:
		val = JESDTX_DMA_REQ(10);
		break;
	case 13:
		val = JESDRX_DMA_REQ(13);
		break;
	}

	return val;
}

int jesd_conf_val_set2(unsigned char jesd_chan, unsigned char gcr_id)
{
	int val = 0;
	switch (jesd_chan) {

	case 1:
		val = JESDRX_DMA_REQ(1);
		break;
	case 2:
		val = JESDRX_DMA_REQ(2);
		break;
	case 3:
		val = JESDRX_DMA_REQ(3);
		break;
	case 4:
		val = JESDRX_DMA_REQ(4);
		break;
	case 5:
		if (gcr_id == 41)
			val = JESDRX_DMA_REQ(41_5);
		else
			val = JESDRX_DMA_REQ(5);
		break;
	case 6:
		if (gcr_id == 41)
			val = JESDRX_DMA_REQ(41_6);
		else
			val = JESDRX_DMA_REQ(6);
		break;
	case 7:
		if (gcr_id == 41)
			val = JESDRX_DMA_REQ(41_7);
		else
			val = JESDRX_DMA_REQ(7);
		break;
	case 8:
		if (gcr_id == 5)
			val = JESDRX_DMA_REQ(5_8);
		else
			val = JESDRX_DMA_REQ(8);
		break;
	case 9:
		if (gcr_id == 5)
			val = JESDTX_DMA_REQ(9);
		else
			val = JESDRX_DMA_REQ(9);
		break;
	case 10:
		if (gcr_id == 5)
			val = JESDTX_DMA_REQ(10);
		else
			val = JESDRX_DMA_REQ(10);
		break;
	case 11:
		val = JESDRX_DMA_REQ(11);
		break;
	case 12:
		val = JESDRX_DMA_REQ(12);
		break;
	}
	return val;
}

int jesd_conf_val_set3(unsigned char jesd_chan)
{
	int val = 0;

	switch (jesd_chan) {

	case 1:
		val = JESDTX_DMA_REQ(1);
		break;
	case 2:
		val = JESDTX_DMA_REQ(2);
		break;
	case 3:
		val = JESDTX_DMA_REQ(3);
		break;
	case 4:
		val = JESDTX_DMA_REQ(4);
		break;
	case 5:
		val = JESDTX_DMA_REQ(5);
		break;
	case 6:
		val = JESDTX_DMA_REQ(6);
		break;
	case 7:
		val = JESDTX_DMA_REQ(7);
		break;
	case 8:
		val = JESDTX_DMA_REQ(8);
		break;
	}
	return val;
}

int cpri_conf_val_set1(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan)
{
	int val = 0;

	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX1_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(2JESD_18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(2JESD_19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(2JESD_21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(2JESD_22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(2JESD_23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(24);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX2_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX2_DMA_REQ(2JESD_18);
			break;
		case 19:
			val = CPRI_RX2_DMA_REQ(2JESD_19);
			break;
		case 20:
			val = CPRI_RX2_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX2_DMA_REQ(2JESD_21);
			break;
		case 22:
			val = CPRI_RX2_DMA_REQ(2JESD_22);
			break;
		case 23:
			val = CPRI_RX2_DMA_REQ(2JESD_23);
			break;
		case 24:
			val = CPRI_RX2_DMA_REQ(24);
			break;
		}
	}
	return val;
}

int cpri_conf_val_set2(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan, unsigned char gcr_id)
{
	int val = 0;
	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 19:
			val = CPRI_RX1_DMA_REQ(2JESD_19);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(2JESD_22);
			break;
		case 18:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_18);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_18);
			break;
		case 23:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_23);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_23);
			break;
		case 17:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_17);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_17);
			break;
		case 24:
			if (gcr_id == 5)
				val = CPRI_RX1_DMA_REQ(3JESD_24);
			else
				val = CPRI_RX1_DMA_REQ(2JESD_24);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(2JESD_21);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 19:
			val = CPRI_RX2_DMA_REQ(2JESD_19);
			break;
		case 22:
			val = CPRI_RX2_DMA_REQ(2JESD_22);
			break;
		case 18:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_18);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_18);
			break;
		case 23:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_23);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_23);
			break;
		case 17:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_17);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_17);
			break;
		case 24:
			if (gcr_id == 5)
				val = CPRI_RX2_DMA_REQ(3JESD_24);
			else
				val = CPRI_RX2_DMA_REQ(2JESD_24);
			break;

		case 20:
			val = CPRI_RX2_DMA_REQ(2JESD_20);
			break;
		case 21:
			val = CPRI_RX2_DMA_REQ(2JESD_21);
			break;
		}
	}
	return val;
}

int cpri_conf_val_set3(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan)
{
	int val = 0;

	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 1:
			val = CPRI_RX1_DMA_REQ(TX_1);
			break;
		case 2:
			val = CPRI_RX1_DMA_REQ(TX_2);
			break;
		case 3:
			val = CPRI_RX1_DMA_REQ(TX_3);
			break;
		case 4:
			val = CPRI_RX1_DMA_REQ(TX_4);
			break;
		case 5:
			val = CPRI_RX1_DMA_REQ(TX_5);
			break;
		case 6:
			val = CPRI_RX1_DMA_REQ(TX_6);
			break;
		case 7:
			val = CPRI_RX1_DMA_REQ(TX_7);
			break;
		case 8:
			val = CPRI_RX1_DMA_REQ(TX_8);
			break;
		case 9:
			val = CPRI_RX1_DMA_REQ(TX_9);
			break;
		case 11:
			val = CPRI_RX1_DMA_REQ(TX_11);
			break;
		case 12:
			val = CPRI_RX1_DMA_REQ(TX_12);
			break;
		case 13:
			val = CPRI_RX1_DMA_REQ(TX_13);
			break;
		case 14:
			val = CPRI_RX1_DMA_REQ(TX_14);
			break;
		case 15:
			val = CPRI_RX1_DMA_REQ(TX_15);
			break;
		case 16:
			val = CPRI_RX1_DMA_REQ(TX_16);
			break;
		case 17:
			val = CPRI_RX1_DMA_REQ(TX_17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(TX_18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(TX_19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(TX_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(TX_21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(TX_22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(TX_23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(TX_24);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 1:
			val = CPRI_RX1_DMA_REQ(TX_1);
			break;
		case 2:
			val = CPRI_RX1_DMA_REQ(TX_2);
			break;
		case 3:
			val = CPRI_RX1_DMA_REQ(TX_3);
			break;
		case 4:
			val = CPRI_RX1_DMA_REQ(TX_4);
			break;
		case 5:
			val = CPRI_RX1_DMA_REQ(TX_5);
			break;
		case 6:
			val = CPRI_RX1_DMA_REQ(TX_6);
			break;
		case 7:
			val = CPRI_RX1_DMA_REQ(TX_7);
			break;
		case 8:
			val = CPRI_RX1_DMA_REQ(TX_8);
			break;
		case 9:
			val = CPRI_RX1_DMA_REQ(TX_9);
			break;
		case 11:
			val = CPRI_RX1_DMA_REQ(TX_11);
			break;
		case 12:
			val = CPRI_RX1_DMA_REQ(TX_12);
			break;
		case 13:
			val = CPRI_RX1_DMA_REQ(TX_13);
			break;
		case 14:
			val = CPRI_RX1_DMA_REQ(TX_14);
			break;
		case 15:
			val = CPRI_RX1_DMA_REQ(TX_15);
			break;
		case 16:
			val = CPRI_RX1_DMA_REQ(TX_16);
			break;
		case 17:
			val = CPRI_RX1_DMA_REQ(TX_17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(TX_18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(TX_19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(TX_20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(TX_21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(TX_22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(TX_23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(TX_24);
			break;
		}
	}
	return val;
}
int cpri_conf_val_set4(enum cpri_core_info cpri_framert_id,
		unsigned char cpri_chan)
{
	int val = 0;

	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX1_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ(18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ(19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ(20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ(21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ(22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ(23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ(24);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 17:
			val = CPRI_RX2_DMA_REQ(17);
			break;
		case 18:
			val = CPRI_RX2_DMA_REQ(18);
			break;
		case 19:
			val = CPRI_RX2_DMA_REQ(19);
			break;
		case 20:
			val = CPRI_RX2_DMA_REQ(20);
			break;
		case 21:
			val = CPRI_RX2_DMA_REQ(21);
			break;
		case 22:
			val = CPRI_RX2_DMA_REQ(22);
			break;
		case 23:
			val = CPRI_RX2_DMA_REQ(23);
			break;
		case 24:
			val = CPRI_RX2_DMA_REQ(24);
			break;
		}
	}
	return val;
}

static inline int get_tx_module(enum vsp_id_t vsp)
{
	if (((vsp >= M2VSP_MIN) && (vsp <= M2VSP_MAX)))
		return 2;
	else if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
		return 1;
	return -EINVAL;
}

u32 vsp_intf_gcr_reg_cfg(unsigned char gcr_id,
		struct dma_intf_switch_parm_t *chan_parm,
		struct gcr_reg_map *gcr)
{
	unsigned int ret = 0;
	int val = 0;
	enum vsp_id_t vsp = chan_parm->vsp_id;
	enum cpri_core_info cpri_framert_id = chan_parm->cpri_framert_id;
	unsigned char cpri_chan = chan_parm->dma_request_id;
	enum dma_channel_id_t chan_id = chan_parm->chan_id;
	enum dma_req_dev_type_t dev_type = chan_parm->dev_type;
	unsigned char jesd_chan = chan_parm->dma_request_id;
	u32 reg;

	reg = readl(&gcr->gcr[gcr_id]);
	if ((gcr_id >= 46) || (gcr_id <= 50)) {
		if (dev_type == DEV_CPRI)
			val = cpri_conf_val_set1(cpri_framert_id, cpri_chan);
		else
			val = jesd_conf_val_set1(jesd_chan);
		if (!val)
			return 0;
		/* to check even and odd value of chan id */
		if ((chan_id & 0x1) && (get_tx_module(vsp) == 1)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << ((vsp - 1) * 3 + 20));
			ret = (reg | (val << ((vsp - 1) * 3 + 20)));
		} else if ((chan_id & 0x1) && (get_tx_module(vsp) == 2)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << ((vsp-M2VSP_MIN-1) * 3 + 20));
			ret = (reg | (val<<((vsp-M2VSP_MIN - 1) * 3 + 20)));
		} else if (~(chan_id & 0x1) && (get_tx_module(vsp) == 1)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << ((vsp - 1) * 3));
			ret = (reg | (val << ((vsp - 1) * 3)));
		} else if (~(chan_id & 0x1) && (get_tx_module(vsp) == 2)) {
			/* masked register bit cleared for applying value */
			reg &= ~(GCR_MSK << (((vsp - M2VSP_MIN) - 1) * 3));
			ret = (reg | (val << (((vsp - M2VSP_MIN) - 1) *	3)));
		}
	} else if ((gcr_id == 41) || (gcr_id == 42) || (gcr_id == 5)) {
		if (dev_type == DEV_CPRI)
			val = cpri_conf_val_set2(cpri_framert_id, cpri_chan,
					gcr_id);
		else
			val = jesd_conf_val_set2(jesd_chan, gcr_id);
		if (!val)
			return 0;
		switch (chan_id) {
		case 10:
		case 13:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << (((vsp - 5) * 3) + 20)));
			ret = (reg | (val << (((vsp - 5) * 3) + 20)));
			break;
		case 9:
		case 12:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << (((vsp - 5) * 3) + 10)));
			ret = (reg | (val << (((vsp - 5) * 3) + 10)));
			break;
		case 8:
		case 11:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << ((vsp - 5) * 3)));
			ret = (reg | (val << (((vsp - 5) * 3))));
			break;
		case 14:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << ((vsp - 5) * 3)));
			ret = (reg | (val << (((vsp - 5) * 3))));
			break;
		case 15:
			/* reg mask to clear the bits for value */
			reg &= (~(GCR_MSK << (((vsp - 5) * 3) + 10)));
			ret = (reg | (val << (((vsp - 5) * 3) + 10)));
			break;
		default:
			ret = 0;
			break;
		}
	} else if ((gcr_id >= 35) || (gcr_id <= 40)) {
		if (dev_type == DEV_CPRI)
			val = cpri_conf_val_set3(cpri_framert_id, cpri_chan);
		if (!val)
			return 0;
		/* reg mask to clear the bits for value */
		reg &= (~(GCR_MSK << (((vsp - 5) * 3) + (((chan_id - 16)
							 % 3) * 10))));
		ret = (reg | (val << (((vsp - 5) * 3) +	(((chan_id - 16) % 3)
						 * 10))));

	} else {
		if (dev_type == DEV_CPRI) {
			val = cpri_conf_val_set4(cpri_framert_id, cpri_chan);

		} else { /* device type is JESD */
			val = jesd_conf_val_set3(jesd_chan);
		}
		if (!val)
			return 0;

		if (chan_id & 0x1) {/* to check even and odd value of chan id */
			if (vsp < M2VSP_MIN) {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << ((vsp + 3) * 3)));
				ret = (reg | (val << ((vsp + 3) * 3)));
			} else {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << (((vsp - M2VSP_MIN)
							     + 3) * 3)));
				ret = (reg | (val << (((vsp - M2VSP_MIN)
								+ 3) * 3)));
			}
		} else {
			if (vsp < M2VSP_MIN) {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << (((vsp + 3) * 3) - 12)));
				ret = (reg | (val << (((vsp + 3) * 3) - 12)));
			} else {
				/* reg mask to clear the bits for value */
				reg &= (~(GCR_MSK << ((((vsp - M2VSP_MIN)
							   + 3) * 3) - 12)));
				ret = (reg  | (val <<
							((((vsp - M2VSP_MIN)
							   + 3) * 3) - 12)));
			}
		}
	}
	return ret;
}

u32 vsp_intf_gcr_reg_cfg_2bit(unsigned char gcr_id,
		struct dma_intf_switch_parm_t *chan_parm,
		struct gcr_reg_map *gcr)
{
	enum cpri_core_info cpri_framert_id = chan_parm->cpri_framert_id;
	unsigned char cpri_chan = chan_parm->dma_request_id;
	enum dma_channel_id_t chan_id = chan_parm->chan_id;
	int val = 0;
	u32 reg;

	reg = readl(&gcr->gcr[gcr_id]);
	if (cpri_framert_id == CPRI_FRAMER_1) {
		switch (cpri_chan) {
		case 1:
			val = CPRI_RX1_DMA_REQ_2BIT(1);
			break;
		case 2:
			val = CPRI_RX1_DMA_REQ_2BIT(2);
			break;
		case 3:
			val = CPRI_RX1_DMA_REQ_2BIT(3);
			break;
		case 4:
			val = CPRI_RX1_DMA_REQ_2BIT(4);
			break;
		case 5:
			val = CPRI_RX1_DMA_REQ_2BIT(5);
			break;
		case 6:
			val = CPRI_RX1_DMA_REQ_2BIT(6);
			break;
		case 7:
			val = CPRI_RX1_DMA_REQ_2BIT(7);
			break;
		case 8:
			val = CPRI_RX1_DMA_REQ_2BIT(8);
			break;
		case 9:
			val = CPRI_RX1_DMA_REQ_2BIT(9);
			break;
		case 10:
			val = CPRI_RX1_DMA_REQ_2BIT(10);
			break;
		case 11:
			val = CPRI_RX1_DMA_REQ_2BIT(11);
			break;
		case 12:
			val = CPRI_RX1_DMA_REQ_2BIT(12);
			break;
		case 13:
			val = CPRI_RX1_DMA_REQ_2BIT(13);
			break;
		case 14:
			val = CPRI_RX1_DMA_REQ_2BIT(14);
			break;
		case 15:
			val = CPRI_RX1_DMA_REQ_2BIT(15);
			break;
		case 16:
			val = CPRI_RX1_DMA_REQ_2BIT(16);
			break;
		case 17:
			val = CPRI_RX1_DMA_REQ_2BIT(17);
			break;
		case 18:
			val = CPRI_RX1_DMA_REQ_2BIT(18);
			break;
		case 19:
			val = CPRI_RX1_DMA_REQ_2BIT(19);
			break;
		case 20:
			val = CPRI_RX1_DMA_REQ_2BIT(20);
			break;
		case 21:
			val = CPRI_RX1_DMA_REQ_2BIT(21);
			break;
		case 22:
			val = CPRI_RX1_DMA_REQ_2BIT(22);
			break;
		case 23:
			val = CPRI_RX1_DMA_REQ_2BIT(23);
			break;
		case 24:
			val = CPRI_RX1_DMA_REQ_2BIT(24);
			break;
		}
	} else {
		switch (cpri_chan) {
		case 1:
			val = CPRI_RX2_DMA_REQ_2BIT(1);
			break;
		case 2:
			val = CPRI_RX2_DMA_REQ_2BIT(2);
			break;
		case 3:
			val = CPRI_RX2_DMA_REQ_2BIT(3);
			break;
		case 4:
			val = CPRI_RX2_DMA_REQ_2BIT(4);
			break;
		case 5:
			val = CPRI_RX2_DMA_REQ_2BIT(5);
			break;
		case 6:
			val = CPRI_RX2_DMA_REQ_2BIT(6);
			break;
		case 7:
			val = CPRI_RX2_DMA_REQ_2BIT(7);
			break;
		case 8:
			val = CPRI_RX2_DMA_REQ_2BIT(8);
			break;
		case 9:
			val = CPRI_RX2_DMA_REQ_2BIT(9);
			break;
		case 10:
			val = CPRI_RX2_DMA_REQ_2BIT(10);
			break;
		case 11:
			val = CPRI_RX2_DMA_REQ_2BIT(11);
			break;
		case 12:
			val = CPRI_RX2_DMA_REQ_2BIT(12);
			break;
		case 13:
			val = CPRI_RX2_DMA_REQ_2BIT(13);
			break;
		case 14:
			val = CPRI_RX2_DMA_REQ_2BIT(14);
			break;
		case 15:
			val = CPRI_RX2_DMA_REQ_2BIT(15);
			break;
		case 16:
			val = CPRI_RX2_DMA_REQ_2BIT(16);
			break;
		case 17:
			val = CPRI_RX2_DMA_REQ_2BIT(17);
			break;
		case 18:
			val = CPRI_RX2_DMA_REQ_2BIT(18);
			break;
		case 19:
			val = CPRI_RX2_DMA_REQ_2BIT(19);
			break;
		case 20:
			val = CPRI_RX2_DMA_REQ_2BIT(20);
			break;
		case 21:
			val = CPRI_RX2_DMA_REQ_2BIT(21);
			break;
		case 22:
			val = CPRI_RX2_DMA_REQ_2BIT(22);
			break;
		case 23:
			val = CPRI_RX2_DMA_REQ_2BIT(23);
			break;
		case 24:
			val = CPRI_RX2_DMA_REQ_2BIT(24);
			break;
		}
	}
	return ((reg & (~(GCR_MSK1 << ((chan_id - 16) * 2))))
			| (val << ((chan_id - 16) * 2)));
}


unsigned int get_up_link_gcr_reg(enum vsp_id_t vsp, enum dma_comm_type_t chan)
{
	unsigned char gcr_id = 0; /*invalid id*/

		if ((vsp > 7) || (vsp < 5))
			goto out;
		if (chan < 16) {
			if ((chan >= 8) || (chan <= 10))
				gcr_id = 41;
			else if ((chan >= 11) || (chan <= 13))
				gcr_id = 42;
			else if ((chan >= 15) || (chan <= 15))
				gcr_id = 5;
		} else if (chan >= 16) {
			if ((chan >= 16) || (chan <= 18))
				gcr_id = 35;
			else if ((chan >= 19) || (chan <= 21))
				gcr_id = 36;
			else if ((chan >= 22) || (chan <= 24))
				gcr_id = 37;
			else if ((chan >= 25) || (chan <= 27))
				gcr_id = 38;
			else if ((chan >= 28) || (chan <= 30))
				gcr_id = 39;
			else if (chan == 31)
				gcr_id = 40;
		}
out:
	return gcr_id;
}

unsigned char get_down_link_gcr_reg(enum vsp_id_t vsp,
		enum dma_comm_type_t chan)
{
	unsigned char gcr_id = 0; /*invalid id*/

	if (chan < 16) {
		if ((chan == 8) || (chan == 9)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 22;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 2;
		} else if ((chan == 10) || (chan == 11)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 23;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 3;
		} else if ((chan == 12) || (chan == 13)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 46;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 49;
		} else if ((chan == 14) || (chan == 15)) {

			if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
				gcr_id = 47;
			else if ((vsp >= M2VSP_MIN) &&
					(vsp <= M2VSP_MAX))
				gcr_id = 50;

		}

	} else if (chan >= 16) {

		if ((vsp >= M1VSP_MIN) && (vsp <= M1VSP_MAX))
			gcr_id = GCR_OFFSET_56 + (vsp-M1VSP_MIN);
		else if ((vsp >= M2VSP_MIN) &&
				(vsp <= M2VSP_MAX))
			gcr_id = GCR_OFFSET_60 + (vsp-M2VSP_MIN);
	}
	return gcr_id;
}


int gcr_vsp_intf_dma_cfg(struct dma_intf_switch_parm_t *chan_parm,
		unsigned char count, struct gcr_reg_map *gcr)
{
	unsigned int tmp = 0;
	unsigned char gcr_id = 0; /*invalid id*/
	u32 value = 0;
	enum dma_channel_id_t chan;
	enum dma_comm_type_t cmd;
	enum vsp_id_t vsp;

	for (tmp = 0; tmp < count; tmp++) {

		chan = (chan_parm + tmp)->chan_id;
		cmd = (chan_parm + tmp)->comm_type;
		vsp = (chan_parm + tmp)->vsp_id;
		switch (cmd) {
		case SETUP_DL:
			gcr_id = get_down_link_gcr_reg(vsp, chan);
		case SETUP_UL:
			gcr_id = get_up_link_gcr_reg(vsp, chan);
			break;
		}
		if (gcr_id != 0) {
			if (chan < 16)
				value = vsp_intf_gcr_reg_cfg(gcr_id,
						(chan_parm + tmp), gcr);
			else
				value = vsp_intf_gcr_reg_cfg_2bit(gcr_id,
						(chan_parm + tmp), gcr);
			break;
		}
	}
	if (value != 0) {
		writel(value, &gcr->gcr[gcr_id]);
		return 0;
	}
	return -EINVAL;
}


int gcr_cpri_dma_mux(struct cpri_dma_mux_config *cpri_mux_parm,
		unsigned count, struct gcr_reg_map *gcr)
{
	int tmp = 0;
	unsigned char gcr_id = 0; /*invalid id*/
	unsigned char cpri_dma_out = 0;
	u32 value = 0;
	enum cpri_rxtx_id rxtx_id;

	for (tmp = 0; tmp < count; tmp++) {
		rxtx_id = (cpri_mux_parm + tmp)->rxtx_id;
		switch (rxtx_id) {
		case CPRI_RX1:
			gcr_id = 20;
			break;
		case CPRI_RX2:
			gcr_id = 21;
			break;
		case CPRI_TX1:
			gcr_id = 29;
			break;
		case CPRI_TX2:
			gcr_id = 30;
			break;


		}
		value = readl(&gcr->gcr[gcr_id]);
		cpri_dma_out = (cpri_mux_parm + tmp)->cpri_dma_req_out;
		value = ((value & (~(GCR_MSK2<<cpri_dma_out)))|
				((((rxtx_id == CPRI_TX1) ||
				   (rxtx_id == CPRI_RX1))
				  ? 0 : 1)<<cpri_dma_out));
		writel(value, &gcr->gcr[gcr_id]);
	}
	return 0;
}


static inline u32 ivsp_gcr_reg_cfg(unsigned int reg, enum vsp_id_t vsp1,
		enum dma_channel_id_t chan, enum vsp_id_t vsp2)
{
	if ((vsp1 % 2) == 0)
		return ((reg&(~(GCR_MSK3 << (chan * 4)))) | ((vsp2 - 1) <<
					(chan*4)));
	else
		return ((reg&(~(GCR_MSK3 << ((chan-4) * 4))))|
				((vsp2 - 1) << ((chan - 4) * 4)));
}

inline int ivsp_parm_validate(enum vsp_id_t vsp1, enum vsp_id_t vsp2,
		enum dma_channel_id_t chan)
{
	return ((vsp1 == vsp2) | ((chan < INTER_VSP_CHAN_MIN) |
				(chan > INTER_VSP_CHAN_MAX)));
}


inline int ivsp_get_gcr_id(enum vsp_id_t srcVsp)
{

	if (srcVsp <= 2)
		return 8; /* gcr id returned */
	else if (srcVsp <= 4)
		return 9;
	else if (srcVsp <= 6)
		return 10;
	else if (srcVsp <= 8)
		return 11;
	else if (srcVsp <= 10)
		return 12;
	else
		return 13;
}


int gcr_inter_vsp_dma_cfg(struct inter_vsp_dma_config_t *vsp_parm,
		unsigned char count, struct gcr_reg_map *gcr)
{
	int tmp = 0;
	int ret = 1;
	u32 reg = 0;
	unsigned char gcr_id = 0; /*invalid id*/

	for (tmp = 0; tmp < count; tmp++) {
		ret = ivsp_parm_validate((vsp_parm + tmp)->src_vsp_id,
				(vsp_parm + tmp)->dst_vsp_id,
				(vsp_parm + tmp)->chan_id);
		if (ret)
			return -EINVAL;

		gcr_id = ivsp_get_gcr_id((vsp_parm + tmp)->src_vsp_id);
		reg = ivsp_gcr_reg_cfg(
				gcr->gcr[gcr_id],
				(vsp_parm + tmp)->chan_id,
				(vsp_parm + tmp)->src_vsp_id,
				(vsp_parm + tmp)->dst_vsp_id);
		writel(reg, &gcr->gcr[gcr_id]);
	}
	return 0;
}





int gcr_jesd_dma_ptr_rst_req(struct jesd_dma_ptr_rst_parm *ptr_rst_parm,
		unsigned count, struct gcr_reg_map *gcr)
{
	return 0;
}

static int gcr_write_set(struct gcr_ctl_parm *param,
		unsigned char count, struct gcr_reg_map *gcr)
{
	unsigned int mAddr, effectiveAddr;
	int tmp = 0;
	u32 reg;

	for (tmp = 0; tmp < count; tmp++) {
		mAddr = (param + tmp)->reg_offset;
		/* Patch offset into base param->address. */
		effectiveAddr = (unsigned int)gcr + mAddr;
		/* Read the param->param. */
		reg = (param+tmp)->param;
		writel(reg, (unsigned int *)effectiveAddr);
	}
	return 0;
}

u32 *gcr_read_reg(struct gcr_ctl_parm *param, struct device *gcr_dev,
		unsigned char count, struct gcr_reg_map *gcr)
{
	unsigned int mAddr, effectiveAddr;
	int tmp = 0;
	u32 *reg;

	reg = kzalloc((count * sizeof(u32)), GFP_KERNEL);
	dev_dbg(gcr_dev, "gcr register read dump:\n");
	for (tmp = 0; tmp < count; tmp++) {


		mAddr = param->reg_offset + tmp;
		/* Patch offset into base param->address. */
		effectiveAddr = (unsigned int)gcr + mAddr;
		/* Read the param->param. */
		*(reg + tmp) = readl((unsigned int *)effectiveAddr);
	}
	return reg;
}

static int gcr_open(struct inode *inode, struct file *file)
{
	struct gcr_priv *priv = NULL;

	priv = container_of(inode->i_cdev, struct gcr_priv, gcr_cdev);
	MOD_INC_USE_COUNT;
	file->private_data = priv;
	dev_dbg(((struct gcr_priv *)file->private_data)->gcr_dev,
			"gcr_dev:: Device opened\n");
	return 0;
}

long gcr_ctrl(struct file *file, unsigned int cmd, unsigned long ioctl_arg)
{
	int ret = 0;
	struct gcr_priv *priv;
	struct gcr_reg_map *gcr;
	unsigned int size;
	struct gcr_parm *arg = NULL;
	unsigned int count;
	u32 *reg_val;

	size = sizeof(struct gcr_parm);
	arg = kmalloc(size, GFP_KERNEL);
	if (copy_from_user((void *)arg, (struct gcr_parm *)ioctl_arg, size)) {
		kfree(arg);
		ret = -EFAULT;
		goto end;

	}
	count = ((struct gcr_parm *)arg)->count;
	priv = (struct gcr_priv *)file->private_data;
	gcr = priv->gcr_reg;
	dev_dbg(priv->gcr_dev, "IOCTL cmd:: %u\n", cmd);
	switch (cmd) {
	case GCR_CONFIG_CMD:
		{
			struct dma_intf_switch_parm_t *chan_parm;

			size = count * sizeof(
					struct dma_intf_switch_parm_t);
			chan_parm = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)chan_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(chan_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_vsp_intf_dma_cfg(chan_parm, count,
					gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(chan_parm);
		}
		break;
	case GCR_CPRI_DMA_MUX_CMD:
		{
			struct cpri_dma_mux_config *cpri_mux_parm;

			size = count * sizeof(
					struct cpri_dma_mux_config);
			cpri_mux_parm = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)cpri_mux_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(cpri_mux_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_cpri_dma_mux(cpri_mux_parm,
					count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(cpri_mux_parm);
		}
		break;
	case GCR_INTER_VSP_CFG:
		{
			struct inter_vsp_dma_config_t *inter_vsp_parm;

			size = count * sizeof(
					struct inter_vsp_dma_config_t);
			inter_vsp_parm =  kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)inter_vsp_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(inter_vsp_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_inter_vsp_dma_cfg(inter_vsp_parm,
					count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(inter_vsp_parm);

		}
		break;
	case GCR_JESD_PTR_RST_CFG:
		{
			struct jesd_dma_ptr_rst_parm *jesd_ptr_parm;

			size = count * sizeof(
					struct jesd_dma_ptr_rst_parm);
			jesd_ptr_parm = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)jesd_ptr_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(jesd_ptr_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_jesd_dma_ptr_rst_req(jesd_ptr_parm,
					count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(jesd_ptr_parm);
		}
		break;
	case GCR_WRITE_REG:
		{
			struct gcr_ctl_parm *param;

			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(param);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_write_set(param, count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(param);
		}
		break;
	case GCR_READ_REG:
		{
			struct gcr_ctl_parm *param;
			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(param);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			reg_val = gcr_read_reg(param, priv->gcr_dev, count,
					gcr);
			size = count * sizeof(u32);
			if (copy_to_user((void *)param->param, reg_val,	size)) {
				kfree(reg_val);
				ret = -EFAULT;
				goto end;
			}
			kfree(reg_val);
			kfree(param);
			mutex_unlock(&priv->gcr_lock);
		}
		break;
	default:
		dev_warn(priv->gcr_dev, "gcr_dev:: invalid ioctl cmd[%u]\n",
				cmd);
		ret = -EINVAL;
		goto end;
		break;
	}

end:
	kfree(arg);
	return ret;
}

static const struct of_device_id gcr_cfg_ids[] = {
	{ .compatible = "fsl,gcr_cfg"},
	{}
};

static struct platform_driver gcr_cfg_driver = {
	.probe = gcr_cfg_probe,
	.remove = gcr_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gcr_cfg_ids,
	},
};

static int gcr_cfg_probe(struct platform_device *ofdev)
{
	struct device_node *np = ofdev->dev.of_node;
	struct device *dev = &ofdev->dev;
	unsigned int property[2] = { 0, 0 };
	int ret = 0;
	int err;
	struct gcr_priv *priv;

	if (!np || !of_device_is_available(np)) {
		ret = -ENODEV;
		goto err_out;
	}
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_dbg(dev, "Failed to allocate gcr private data\n");
		ret = -ENOMEM;
		goto err_mem;
	}
	priv->gcr_dev = dev;
	ret = of_property_read_u32_array(np, "reg", property,
			ARRAY_SIZE(property));
	if (ret) {
		dev_err(&ofdev->dev, "gcr_dev:: of_prop_read Failed ...\n");
		ret = -EFAULT;
		goto err_mem;
	}
	priv->gcr_reg = of_iomap(ofdev->dev.of_node, 0);

	if (!priv->gcr_reg) {
		dev_err(dev, "gcr_dev:: ioremap_nocache Failed ...\n");
		ret = -EFAULT;
		goto err_mem;
	}
	err = alloc_chrdev_region(&priv->dev_t, 0, GCR_DEV_MAX, DRIVER_NAME);
	if (err) {
		pr_err("cannot register gcr driver\n");
		ret = -EFAULT;
		iounmap(priv->gcr_reg);
		goto err_mem;
	}
	cdev_init(&priv->gcr_cdev, &gcr_fops);
	err = cdev_add(&priv->gcr_cdev, priv->dev_t, 1);
	if (err) {
		pr_err("cannot add gcr driver\n");
		ret = -EFAULT;
		iounmap(priv->gcr_reg);
		goto err_mem;
	}

	mutex_init(&priv->gcr_lock);
	dev_set_drvdata(dev, priv);
	return 0;
err_mem:
	kfree(priv);
err_out:
	pr_err("gcr probe error\n");
	return ret;
}

static int gcr_release(struct inode *inode, struct file *fp)
{
	struct gcr_priv *priv = (struct gcr_priv *)fp->private_data;

	if (!priv)
		return -ENODEV;
	cdev_del(&priv->gcr_cdev);
	return 0;
}

static int gcr_remove(struct platform_device *ofdev)
{
	struct gcr_priv *priv = NULL;

	priv = (struct gcr_priv *)dev_get_drvdata(&ofdev->dev);
	iounmap(priv->gcr_reg);
	dev_set_drvdata(priv->gcr_dev, NULL);
	if (priv->dev_t)
		unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
	kfree(priv);
	return 0;
}

static int __init init_gcr(void)
{
	return platform_driver_register(&gcr_cfg_driver);
}

static void __exit exit_gcr(void)
{
	platform_driver_unregister(&gcr_cfg_driver);
	return;
}

module_init(init_gcr);
module_exit(exit_gcr);

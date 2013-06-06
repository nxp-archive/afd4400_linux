/*
* drivers/jesd/jesd204.c
* Author: Freescale
*
* Copyright 2013 Freescale Semiconductor, Inc.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/tbgen.h>
#include <linux/jesd204.h>
#include <linux/phygasket.h>

#define TRANS_TX_NAME "jesd_tx"
#define TRANS_RX_NAME "jesd_rx"

static struct class *jesd204_class;
static struct jesd204_private *pri;

static int jesd_init_transport(struct jesd_transport_dev *tdev,
				struct jesd_dev_params *trans_p);
static int jesd_set_ils_pram(struct jesd_transport_dev *tdev);
static int jesd_set_ilas_len(struct jesd_transport_dev *tdev, int length);
static int jesd_dev_stop(struct jesd_transport_dev *tdev);
static int jesd_force_sync(struct jesd_transport_dev *tdev);
static int get_device_name(struct jesd_transport_dev *tdev);

static int jesd_setclkdiv(struct jesd_transport_dev *tdev);

static int jesd_change_state(struct jesd_transport_dev *tdev,
	enum jesd_state new_state);

static void jesd_marks_syref_capture_ready(struct jesd_transport_dev *tdev,
	int enable);
static void jesd_restore_state(struct jesd_transport_dev *tdev);

static int jesd_conf_auto_sync(struct jesd_transport_dev *tdev,
	struct auto_sync_params *sync_params);

void jesd_update_reg(u32 *reg, u32 val, u32 mask)
{
	u32 reg_val;

	reg_val = ioread32(reg);
	reg_val &= ~mask;
	reg_val |= (val & mask);
	iowrite32(reg_val, reg);
}

void jesd_write_reg(u32 *reg, u32 val)
{
	iowrite32(val, reg);
}

void jesd_read_reg(u32 *reg)
{
	ioread32(reg);
}

static int jesd_config_phygasket(struct jesd_transport_dev *tdev)
{
	int rc = 0, i;
	struct lane_device *lane;
	enum phygasket_data_src phy_data_src;

	dev_info(tdev->dev, "%s: type %d, dev_flags %x\n", tdev->name,
		tdev->type, tdev->dev_flags);

	switch (tdev->type) {
	case JESD_DEV_TX:
		if (tdev->dev_flags & DEV_FLG_TEST_PATTERNS_EN)
			phy_data_src = PHY_DATA_TEST_PATTRN;
		else
			phy_data_src = PHY_DATA_JESDTX;
		break;
	case JESD_DEV_RX:
	case JESD_DEV_SRX:
		if (tdev->dev_flags & DEV_FLG_PHYGASKET_LOOPBACK_EN)
			phy_data_src = PHY_DATA_RX_LOOPBACK;
		else
			phy_data_src = PHY_DATA_JESDRX;
		break;
	default:
		rc = -EINVAL;
		goto out;
	}

	dev_info(tdev->dev, "%s: data src %d\n", tdev->name, phy_data_src);

	for (i = 0; i < tdev->active_lanes; i++) {
		lane = tdev->lane_devs[i];
		rc = phy_gasket_lane_ctrl(tdev->phy, phy_data_src, lane->id);
		if (rc) {
			dev_err(tdev->dev, "%s: Phy init Failed, lane %d\n",
				tdev->name, lane->id);
			goto out;
		}

	}

	if ((tdev->type == JESD_DEV_TX) &&
		(tdev->dev_flags & DEV_FLG_TEST_PATTERNS_EN))
		do_patter_generator(tdev->phy, &tdev->tx_tests.tx_patgen);

out:
	return rc;
}

void lane_stats_update(struct jesd_transport_dev *tdev,
						u32 stats_field,
						u32 lane_id)
{
	/*XXX: To be implemented */
}

static int jesd_setclkdiv(struct jesd_transport_dev *tdev)
{
	int line_rate, m, s, np, l;
	int fs, fc, clkdiv, ref, rc = 0;
	u32 *reg = NULL, mask;

	/*XXX: TBD set clk div based on tdev->data_rate and tdev->ref_clk*/
	ref = get_ref_clock(tdev->tbgen_dev_handle);

	m = tdev->ils.conv_per_device_m;
	s = tdev->ils.samples_per_cnvrtr_per_frame;
	np = tdev->ils.bits_per_converter;
	l = tdev->ils.lanes_per_converter_l;
	fs = tdev->ils.octect_per_frame_f;

	line_rate = (((m*s*np*fs)*(10/8))/l);
	fc = line_rate/10;

	if (tdev->type == JESD_DEV_TX)
		clkdiv = (ref/fc);
	else
		clkdiv = ((ref*2)/fc);

	/*XXX: For medusa setting it to 0 hardcoded for bringup.
	 *This needs cleanup
	 */
	clkdiv = 0;
	clkdiv &= ~CLKDIV_MASK; /*mask for three bits*/
	mask = CLKDIV_MASK;
	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_transcontrol;
	else
		reg = &tdev->rx_regs->rx_transcontrol;

	jesd_update_reg(reg, clkdiv, mask);

	return rc;
}

int jesd_reg_write_from_user(struct jesd_transport_dev *tdev,
	struct jesd_reg_val *user_buf, int count)
{
	int size, *reg_base, *reg, i, rc = 0;
	struct jesd_reg_val *reg_buf = NULL;

	size = count * sizeof(struct jesd_reg_val);
	reg_buf = kzalloc(size, GFP_KERNEL);
	if (!reg_buf) {
		dev_err(tdev->dev, "Failed to alloc reg buf\n");
		rc = -ENOMEM;
		goto out;
	}
	if (copy_from_user(reg_buf, user_buf, size)) {
		rc = -EFAULT;
		goto out;
	}

	if (tdev->type == JESD_DEV_TX) {
		size = sizeof(struct config_registers_tx);
		reg_base = (u32 *) tdev->tx_regs;
	} else {
		size = sizeof(struct config_registers_rx);
		reg_base = (u32 *)tdev->rx_regs;
	}

	for (i = 0; i < count; i++) {
		if (reg_buf[i].offset > size) {
			dev_err(tdev->dev, "Invalid offset 0x%x\n",
				reg_buf[i].offset);
			continue;
		}
		reg = (u32 *) ((u32) reg_base + (u32) reg_buf[i].offset);
		iowrite32(reg_buf[i].value, reg);
	}
out:
	kfree(reg_buf);
	return rc;
}

int jesd_reg_dump_to_user(u32 *reg, unsigned int offset, unsigned int length,
			u32 *buf)
{
	int i, rc = 0;
	u32 val;
	u32 *start_reg;

	start_reg = (u32 *)((u32) reg + offset);
	for (i = 0; i < length; i++) {
		val = ioread32(start_reg);
		if (put_user(val, buf)) {
			rc = -EFAULT;
			break;
		}
		start_reg++;
		buf++;
	}

	return rc;
}
EXPORT_SYMBOL(jesd_reg_dump_to_user);

static int jesd_conf_tx_tests(struct jesd_transport_dev *tdev,
	struct conf_tx_tests *tx_tests)
{
	int rc = 0;
	u32 *reg, val = 0;

	if (tdev->type != JESD_DEV_TX) {
		rc = -EINVAL;
		goto out;
	}

	/* Framer tests */
	reg = &tdev->tx_regs->tx_frm_tst;
	val = 0;
	if (tx_tests->frmr_tst.bipass_8b10)
		val |= BYP_8B10B;

	if (tx_tests->frmr_tst.reverse_8b10)
		val |= REV_FRM_DOUT;

	iowrite32(val, reg);

	/* JDEC tests */
	reg = &tdev->tx_regs->tx_jedec_tst;
	val = 0;

	if (tx_tests->frmr_tst.tpl)
		val |= TRANSPORT_TEST_SEQ_EN;

	if (tx_tests->frmr_tst.dll)
		val |= DATA_LINK_TEST_PATTRN_EN;

	iowrite32(val, reg);
	memcpy(&tdev->tx_tests, tx_tests, sizeof(struct conf_rx_tests));
out:
	return rc;
}

static int jesd_conf_rx_tests(struct jesd_transport_dev *tdev,
	struct conf_rx_tests *rx_tests)
{
	int rc = 0;
	u32 *reg, val;

	if ((tdev->type != JESD_DEV_RX) || (tdev->type != JESD_DEV_SRX)) {
		rc = -EINVAL;
		goto out;
	}

	reg = &tdev->rx_regs->rx_ctrl_2;
	val = 0;

	if (rx_tests->drfmr_tst.rep_data_tst) {
		val |= REP_DATA_TEST;
		if (rx_tests->drfmr_tst.que_tst_err)
			val |= QUEUE_TEST_ERR;
	}

	if (rx_tests->drfmr_tst.ils_mode)
		val |= ILS_TEST_MODE;

	iowrite32(val, reg);
	memcpy(&tdev->rx_tests, rx_tests, sizeof(struct conf_rx_tests));
out:
	return rc;
}


static int jesd_set_delays(struct jesd_transport_dev *tdev)
{
	int rc = 0;
	int delay;
	/* this shall be either tx or rx delay have the segregation here
	* itself config delay
	*/
	if (tdev->type == JESD_DEV_TX)
		iowrite32(tdev->delay & 0x15, &tdev->tx_regs->tx_sync_delay);
	else if ((tdev->type == JESD_DEV_RX) || (tdev->type == JESD_DEV_SRX)) {
		/*Set to 2 when K=5, F=4
		Set to 4 when K=10, F=2
		Set to 5 when K=6, F=4
		Set to 10 when K=12, F=2 as per RM*/
		if (tdev->ils.frame_per_mf_k == 5 &&
				tdev->ils.octect_per_frame_f == 4)
			delay = 2;
		else if (tdev->ils.frame_per_mf_k == 10 &&
				tdev->ils.octect_per_frame_f == 2)
			delay = 4;
		else if (tdev->ils.frame_per_mf_k == 6 &&
					tdev->ils.octect_per_frame_f == 4)
			delay = 5;
		else if (tdev->ils.frame_per_mf_k == 12 &&
					tdev->ils.octect_per_frame_f == 2)
			delay = 10;
		else
			rc = -EFAULT;

		if (rc < 0)
			return rc;
		else
			iowrite32(delay, &tdev->rx_regs->rx_rcv_delay);
	} else
		rc = -EINVAL;
	return rc;
}

void jesd_enable_sysref_capture(struct jesd_transport_dev *tdev)
{
	u32 *irq_status_reg, *irq_enable_reg, *ctrl_reg;
	u32 val, mask;

	if (tdev->type == JESD_DEV_TX) {
		irq_status_reg = &tdev->tx_regs->tx_irq_status;
		irq_enable_reg = &tdev->tx_regs->tx_irq_enable;
		ctrl_reg = &tdev->tx_regs->tx_transcontrol;
	} else {
		irq_status_reg = &tdev->rx_regs->rx_irq_status;
		irq_enable_reg = &tdev->rx_regs->rx_irq_enable;
		ctrl_reg = &tdev->rx_regs->rx_transcontrol;
	}
	/*Clear sysref rose */
	val = ~IRQ_SYSREF_ROSE;
	mask = IRQ_SYSREF_ROSE;
	jesd_update_reg(irq_status_reg, val, mask);

	/*unmask sysref detection  */
	val = ~SYSREF_MASK;
	mask = SYSREF_MASK;
	jesd_update_reg(ctrl_reg, val, mask);

	/*enable sysref rose IRQ*/
	val = IRQ_SYSREF_ROSE;
	mask = val;
	jesd_update_reg(irq_enable_reg, val, mask);

}

static int jesd_setup_tx_transport(struct jesd_transport_dev *tdev)
{
	int rc = 0, i;
	u32 *reg, val, mask, enable_mask = 0;

	for (i = 0; i < tdev->active_lanes; i++)
		enable_mask |= 1 << i;

	reg = &tdev->tx_regs->tx_l_en;
	mask = LANE_EN_MASK;
	jesd_update_reg(reg, enable_mask, mask);

	for (i = 0; i < tdev->ils.conv_per_device_m; i++)
		enable_mask |= 1 << i;

	reg = &tdev->tx_regs->tx_m_en;
	mask = M_EN_MASK;
	jesd_update_reg(reg, enable_mask, mask);

	/* XXX: setting up sync pipiline to 6 according to validation script
	 * need to come up with right way to do it
	 */
	reg = &tdev->tx_regs->tx_transcontrol;
	val = 6 << SYNC_PIPELINE_SHIFT;
	mask = SYNC_PIPELINE_MASK << SYNC_PIPELINE_SHIFT;
	jesd_update_reg(reg, val, mask);

	if (tdev->dev_flags & DEV_FLG_PHYGASKET_LOOPBACK_EN) {
		reg = &tdev->tx_regs->tx_transcontrol;
		val = SYNC_SELECT_TBGEN;
		mask = SYNC_SELECT_TBGEN;
		jesd_update_reg(reg, val, mask);
		tbgen_set_sync_loopback(tdev->timer_handle, 1);
	} else {
		reg = &tdev->tx_regs->tx_transcontrol;
		val = ~SYNC_SELECT_TBGEN;
		mask = SYNC_SELECT_TBGEN;
		jesd_update_reg(reg, val, mask);
		tbgen_set_sync_loopback(tdev->timer_handle, 0);
	}

	/*enable SYNC recievied IRQ*/
	reg = &tdev->tx_regs->tx_irq_enable;
	val = IRQ_SYNC_RECIEVED;
	mask = val;
	jesd_update_reg(reg, val, mask);

	return rc;
}

int jesd_setup_rx_transport(struct jesd_transport_dev *tdev)
{
	int rc = 0, i;
	u32 *reg, val, mask, enable_mask = 0;

	for (i = 0; i < tdev->active_lanes; i++)
		enable_mask |= 1 << i;

	reg = &tdev->rx_regs->rx_lane_en;
	mask = LANE_EN_MASK;
	jesd_update_reg(reg, enable_mask, mask);

	/* XXX: setting up sync pipiline to 3 according to validation script
	 * need to come up with right way to do it
	 */
	reg = &tdev->rx_regs->rx_transcontrol;
	val = 3 << SYNC_PIPELINE_SHIFT;
	mask = SYNC_PIPELINE_MASK << SYNC_PIPELINE_SHIFT;
	jesd_update_reg(reg, val, mask);

	return rc;
}

static int jesd_setup_transport(struct jesd_transport_dev *tdev)
{
	int rc = 0;

	if (tdev->type == JESD_DEV_TX)
		rc = jesd_setup_tx_transport(tdev);
	else
		rc = jesd_setup_rx_transport(tdev);

	return rc;
}

int jesd_start_transport(struct jesd_transport_dev *tdev)
{
	int rc = 0;

	rc = jesd_change_state(tdev, JESD_STATE_ENABLED);
	if (rc) {
		dev_err(tdev->dev, "%s:Cannot be enabled in state %d\n",
			__func__, tdev->old_state);
		goto out;
	}
	rc = jesd_config_phygasket(tdev);
	if (rc) {
		dev_err(tdev->dev, "Failed to configure phy-gasket, err %d\n",
			rc);
		goto out;
	}
	rc = jesd_setclkdiv(tdev);
	if (rc) {
		dev_err(tdev->dev, "Failed to set clkdiv, err %d\n", rc);
		goto out;
	}

	rc = jesd_set_delays(tdev);
	if (rc) {
		dev_err(tdev->dev, "Failed to set delays, err %d\n", rc);
		goto out;
	}

	rc = jesd_setup_transport(tdev);
	if (rc) {
		dev_err(tdev->dev, "Failed to enable transport, err %d\n", rc);
		goto out;
	}

	/* Makrk jesd link ready for sysref capture*/
	jesd_marks_syref_capture_ready(tdev, 1);
	return rc;
out:
	jesd_restore_state(tdev);
	return rc;
}

static void jesd_marks_syref_capture_ready(struct jesd_transport_dev *tdev,
	int enable)
{
	tbgen_timer_set_sysref_capture(tdev->timer_handle, enable);
	if (tdev->type == JESD_DEV_TX)
		tbgen_timer_set_sysref_capture(tdev->txalign_timer_handle,
			enable);
}

static int jesd_force_sync(struct jesd_transport_dev *tdev)
{
	u32 *reg, val, mask;

	reg = &tdev->rx_regs->rx_ctrl_0;
	val = FSREQ;
	mask = FSREQ;
	jesd_update_reg(reg, val, mask);

	return 0;
}


static int jesd_set_ilas_len(struct jesd_transport_dev *tdev, int ilas_len)
{
	int rc = 0;
	u32 val, *reg;

	dev_dbg(tdev->dev, "%s: ILAS lenght %d\n", tdev->name, ilas_len);

	val = (ilas_len / 4) - 1;

	if (val & ~MAX_ILAS_LEN_MASK) {
		dev_err(tdev->dev, "%s: Invalid ILAS lenght %d\n", tdev->name,
			ilas_len);
		rc = -EINVAL;
		goto out;
	}
	tdev->ilas_len = ilas_len;

	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_ilas_len;
	else
		reg = &tdev->rx_regs->rx_ilas_len;

	val &= MAX_ILAS_LEN_MASK;
	iowrite32(val, reg);

out:
	return rc;
}

static int init_tx_transport(struct jesd_transport_dev *tdev)
{
	int rc = 0;
	u32 config_flags, *reg, val = 0, mask;
	struct config_registers_tx *tx_regs = tdev->tx_regs;

	config_flags = tdev->config_flags;

	/* scrambling */
	reg = &tx_regs->tx_scr_ctrl;
	val = L0_SCR_EN;
	if (tdev->active_lanes == 2)
		val |= L1_SCR_EN;
	mask = val;

	if (!(config_flags & CONF_SCRAMBLING_EN))
		val = ~val;
	jesd_update_reg(reg, val, mask);

	/* Frame control */
	reg = &tx_regs->tx_frm_ctrl;
	val = 0;

	if (config_flags & CONF_LANE_SYNC_BOTH_SIDES_EN)
		val |= L2SIDES_EN;
	else
		val &= ~L2SIDES_EN;

	if (config_flags & CONF_BYPASS_ILAS)
		val |= BYP_ILAS;
	else
		val  = ~BYP_ILAS;

	if (config_flags & BYP_AGC)
		val |= BYP_AGC;
	else
		val &= ~BYP_AGC;

	mask = L2SIDES_EN | BYP_ILAS | BYP_AGC;
	jesd_update_reg(reg, val, mask);

	/* Transport control */
	reg = &tx_regs->tx_transcontrol;
	val = 0;

	if (config_flags & CONF_USE_OTHER_TRANSPORTS_IDLE)
		val |= OTHR_TRANS_IDLE_SELECT;
	else
		val &= ~OTHR_TRANS_IDLE_SELECT;

	if (config_flags & CONF_SWAP_IQ)
		val |= SWAP_IQ;
	else
		val &= ~SWAP_IQ;

	if (config_flags & CONF_PHY_MS_OCTET_FIRST_EN)
		val |= PHYPACK_MS_OCTECT_FIRST;
	else
		val &= ~PHYPACK_MS_OCTECT_FIRST;

	mask = OTHR_TRANS_IDLE_SELECT | SWAP_IQ | PHYPACK_MS_OCTECT_FIRST;
	jesd_update_reg(reg, val, mask);

	return rc;
}

static int init_rx_transport(struct jesd_transport_dev *tdev)
{
	int rc = 0;
	u32 config_flags, *reg, val = 0, mask;
	struct config_registers_rx *rx_regs = tdev->rx_regs;

	config_flags = tdev->config_flags;

	/* Transport control */
	reg = &rx_regs->rx_transcontrol;

	if (config_flags & CONF_SWAP_IQ)
		val |= SWAP_IQ;
	else
		val &= ~SWAP_IQ;

	if (config_flags & CONF_PHY_ORDER_MS_BIT_FIRST_EN)
		val |= PHYORDER_MS_BIT_FIRST;
	else
		val &= ~PHYORDER_MS_BIT_FIRST;

	if (config_flags & CONF_PHY_MS_OCTET_FIRST_EN)
		val |= PHYPACK_MS_OCTECT_FIRST;
	else
		val &= ~PHYPACK_MS_OCTECT_FIRST;

	if (config_flags & CONF_STRICT_CGS)
		val |= CONF_STRICT_CGS;
	else
		val &= ~CONF_STRICT_CGS;

	mask = STRICT_CGS | PHYORDER_MS_BIT_FIRST |
		PHYPACK_MS_OCTECT_FIRST | SWAP_IQ;
	jesd_update_reg(reg, val, mask);

	return rc;
}

static int __jesd_attach_timer(struct jesd_transport_dev *tdev,
	enum timer_type timer_type)
{
	int rc = 0;
	void *timer_handle;

	timer_handle = tbgen_get_timer(tdev->tbgen_dev_handle, timer_type,
			tdev->timer_id);

	if (!timer_handle) {
		dev_err(tdev->dev, "%s:Failed to get timer type %d, id %d\n",
				tdev->name, timer_type, tdev->timer_id);
		rc = -ENODEV;
		goto out;
	}

	rc = tbgen_attach_timer(timer_handle, tdev);
	if (rc) {
		dev_err(tdev->dev, "%s: Attach failed timer type %d, id %d\n",
			tdev->name, timer_type, tdev->timer_id);
		goto out;
	}

	if (timer_type == JESD_TX_ALIGNMENT)
		tdev->txalign_timer_handle = timer_handle;
	else
		tdev->timer_handle = timer_handle;

	dev_info(tdev->dev, "%s: Attached timer %d, type %d, handle 0x%p\n",
			tdev->name, tdev->timer_id, timer_type, timer_handle);
	return rc;
out:
	tdev->timer_handle = NULL;
	tdev->txalign_timer_handle = NULL;
	return rc;
}

static int jesd_attach_tbgen_timer(struct jesd_transport_dev *tdev)
{
	int rc = 0;

	tdev->tbgen_dev_handle = get_tbgen_device();

	if (!tdev->tbgen_dev_handle) {
		dev_err(tdev->dev, "Failed to get tbgen device\n");
		rc = -ENODEV;
		goto out;
	}
	switch (tdev->type) {
	case JESD_DEV_TX:
		rc = __jesd_attach_timer(tdev, JESD_TX_ALIGNMENT);
		if (!rc)
			rc = __jesd_attach_timer(tdev, JESD_TX_AXRF);
		break;
	case JESD_DEV_RX:
		rc = __jesd_attach_timer(tdev, JESD_RX_ALIGNMENT);
		break;
	case JESD_DEV_SRX:
		rc = __jesd_attach_timer(tdev, JESD_SRX_ALIGNMENT);
		break;
	default:
		rc = -EINVAL;
		goto out;
	}
	if (rc)
		goto out;

	return rc;
out:
	tdev->tbgen_dev_handle = NULL;
	return rc;
}

static void jesd_init_dev_flags(struct jesd_transport_dev *tdev,
	u32 config_flags)
{

	if (config_flags & CONF_PHYGASKET_LOOPBACK_EN)
		tdev->dev_flags |= DEV_FLG_PHYGASKET_LOOPBACK_EN;
	else
		tdev->dev_flags &= ~DEV_FLG_PHYGASKET_LOOPBACK_EN;
}

static int jesd_init_transport(struct jesd_transport_dev *tdev,
				struct jesd_dev_params *trans_p)
{
	int rc = 0, i;
	struct lane_device *lane_dev;
	struct jesd204_dev *jdev = tdev->parent;

	if (trans_p->lanes > tdev->max_lanes) {
		dev_err(tdev->dev, "%s: Invalid lanes (%d), max %d\n",
			tdev->name, trans_p->lanes, tdev->max_lanes);
		rc = -EINVAL;
		goto out;
	}

	if (jdev->used_lanes >= jdev->max_lanes) {
		dev_err(tdev->dev, "%s: Parent's all lanes are used\n",
			tdev->name);
		rc = -EBUSY;
		goto out;
	}
	/*create lanes*/
	for (i = 0; i < trans_p->lanes; i++) {
		lane_dev = kzalloc(sizeof(struct lane_device), GFP_KERNEL);
		if (!lane_dev) {
			dev_err(tdev->dev, "%s: Lane allocation Failed [%d]\n",
				tdev->name, i);
			rc = -ENOMEM;
			goto out;
		}
		lane_dev->id = i + tdev->id;
		lane_dev->enabled = 0;
		tdev->lane_devs[i] = lane_dev;
		jdev->used_lanes++;
	}

	tdev->data_rate = trans_p->data_rate;
	tdev->delay = trans_p->delay;
	tdev->config_flags = trans_p->config_flags;
	tdev->active_lanes = trans_p->lanes;
	jesd_init_dev_flags(tdev, trans_p->config_flags);

	dev_dbg(tdev->dev, "%s: data rate %d, dlay %x, flags %x, lanes %d\n",
		tdev->name, tdev->data_rate, tdev->delay, tdev->config_flags,
		tdev->active_lanes);

	if (tdev->type == JESD_DEV_TX) {
		rc = init_tx_transport(tdev);
		if (rc)
			goto out;
	} else {
		rc = init_rx_transport(tdev);
		if (rc)
			goto out;
	}

	rc = jesd_attach_tbgen_timer(tdev);
	if (rc) {
		dev_err(tdev->dev, "%s: Failed to attach tbgen timer, err %d\n",
			tdev->name, rc);
		goto out;
	}

	return rc;
out:
	for (i = 0; i < tdev->active_lanes; i++)
		kfree(tdev->lane_devs[i]);
	return rc;
}


static int config_frames_per_mf(struct jesd_transport_dev *tdev)
{
	u32 ref_clk, *reg, frames_per_mf = 0;
	int rc = 0;



#if 0
	ref_clk =  get_ref_clock(tdev->tbgen_dev_handle);

#else
	/*XXX: Hardcoded ref clock for bringup. It is not even 122.8 Mhz
	 *on medusa,  check values from h/w guys
	 */
	 ref_clk = 614;
#endif

	if (ref_clk == 614) {
		/* As per RM For ref_clk=614.4MHz:
		* Set to 5 when the link uses 1 lane (L=1)
		* Set to 10 when the link uses 2 lanes (L=2)
		*/
		if (tdev->ils.lanes_per_converter_l == 1)
			frames_per_mf = 5;
		else if (tdev->ils.lanes_per_converter_l == 2)
			frames_per_mf =  10;
	} else if (ref_clk == 983) {
		/* For ref_clk=983.04MHz:
		* Set to 5 when the link uses 1 lane (L=1)
		* Set to 11 when the link uses 2 lanes (L=2)
		*/
		if (tdev->ils.lanes_per_converter_l == 1)
			frames_per_mf = 6;
		else if (tdev->ils.lanes_per_converter_l == 2)
			frames_per_mf =  12;
	}

	if (tdev->ils.frame_per_mf_k != frames_per_mf) {
		/*Do not fail if application sets it different than
		 * RM recommended value, but print info for this setting
		 */
		dev_info(tdev->dev, "%s: lanes %d, Setting Frms/multifrm to %d",
				tdev->name, tdev->ils.lanes_per_converter_l,
				tdev->ils.frame_per_mf_k);
		dev_info(tdev->dev, "%s: recommend Frms/multifrm is %d",
				tdev->name, frames_per_mf);
		frames_per_mf = tdev->ils.frame_per_mf_k;
	}
	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_fpmf;
	else
		reg = &tdev->rx_regs->rx_fpmf;

	iowrite32((frames_per_mf - 1), reg);

	return rc;
}

static void set_did_bid(struct jesd_transport_dev *tdev)
{
	u32 *did_reg = NULL, *bid_reg = NULL;

	if (tdev->type == JESD_DEV_TX) {
		did_reg = &tdev->tx_regs->tx_did;
		bid_reg = &tdev->tx_regs->tx_bid;
	} else {
		did_reg = &tdev->rx_regs->rx_did;
		bid_reg = &tdev->rx_regs->rx_bid;
	}

	iowrite32(tdev->ils.device_id, did_reg);
	jesd_update_reg(bid_reg, tdev->ils.bank_id, BID_MASK);
}

static void set_lane_id(struct jesd_transport_dev *tdev)
{
	if (tdev->type == JESD_DEV_TX) {
		jesd_update_reg(&tdev->tx_regs->tx_lid_0,
			tdev->ils.lane0_id, LID_MASK);
		if (tdev->active_lanes == 2)
			jesd_update_reg(&tdev->tx_regs->tx_lid_1,
				tdev->ils.lane1_id, LID_MASK);
	} else {
		jesd_update_reg(&tdev->rx_regs->rx_lid_0, tdev->ils.lane0_id,
			LID_MASK);
	}

}

static void config_scr(struct jesd_transport_dev *tdev)
{
	u32 *reg, val, mask;

	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_scr;
	else
		reg = &tdev->rx_regs->rx_scr;

	if (tdev->ils.scrambling_scr)
		val = SCR_EN;
	else
		val = ~SCR_EN;

	mask = SCR_EN;
	jesd_update_reg(reg, val, mask);
}


static int config_l(struct jesd_transport_dev *tdev)
{
	int rc = 0;
	u32 val, *reg = NULL;

	if (!tdev->ils.lanes_per_converter_l) {
		dev_err(tdev->dev, "%s: Invalid lanes/converter %d\n",
			tdev->name, tdev->ils.lanes_per_converter_l);
		rc = -EINVAL;
		goto out;
	}
	val = tdev->ils.lanes_per_converter_l - 1;
	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_scr;
	else
		reg = &tdev->rx_regs->rx_scr;

	jesd_update_reg(reg, val, LANES_PER_CONV_MASK);

	return rc;
out:
	return rc;
}

static void config_octets_per_frm(struct jesd_transport_dev *tdev)
{
	u32 *reg = NULL;

	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_opf;
	else
		reg = &tdev->rx_regs->rx_opf;

	iowrite32((tdev->ils.octect_per_frame_f - 1), reg);
}

static void config_ctrlbits_per_sample(struct jesd_transport_dev *tdev)
{
	u32 val, *reg = NULL, mask;

	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_cbps;
	else
		reg = &tdev->rx_regs->rx_cbps;

	mask = CS_MASK << CS_SHIFT;
	val = (tdev->ils.ctrl_bits_per_sample & CS_MASK) << CS_SHIFT;
	jesd_update_reg(reg, val, mask);
}

static void config_converter_resolution(struct jesd_transport_dev *tdev)
{
	u32 val, mask, *reg = NULL;

	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_cbps;
	else
		reg = &tdev->rx_regs->rx_cbps;

	val = ((tdev->ils.converter_resolution - 1) & N_MASK) << N_SHIFT;
	mask = N_MASK << N_SHIFT;

	jesd_update_reg(reg, val, mask);
}


static void config_subclass(struct jesd_transport_dev *tdev)
{
	u32 *reg = NULL, val, mask;

	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_nbcw;
	else
		reg = &tdev->rx_regs->rx_nbcw;

	val = (tdev->ils.subclass_ver & SUBCLASS_MASK) << SUBCLASS_SHIFT;
	mask = SUBCLASS_MASK << SUBCLASS_SHIFT;

	jesd_update_reg(reg, val, mask);
}


static void config_version(struct jesd_transport_dev *tdev)
{
	u32 *reg = NULL, val, mask;

	if (tdev->type == JESD_DEV_TX)
		reg = &tdev->tx_regs->tx_spcp;
	else
		reg = &tdev->rx_regs->rx_spcp;

	val = (tdev->ils.jesd_ver & VERSION_MASK) << VERSION_SHIFT;
	mask = VERSION_MASK << VERSION_SHIFT;

	jesd_update_reg(reg, val, mask);
}


static int jesd_set_ils_pram(struct jesd_transport_dev *tdev)
{
	int rc = 0;
	u32 *reg = NULL, val, mask;

	set_did_bid(tdev);
	set_lane_id(tdev);
	config_scr(tdev);

	rc = config_l(tdev);
	if (rc)
		goto out;

	config_octets_per_frm(tdev);

	rc = config_frames_per_mf(tdev);
	if (rc < 0)
		goto out;

	if (tdev->type == JESD_DEV_TX)
		iowrite32((tdev->ils.conv_per_device_m - 1),
			&tdev->tx_regs->tx_cpd);
	else
		iowrite32((tdev->ils.conv_per_device_m - 1),
			&tdev->rx_regs->rx_cpd);

	config_ctrlbits_per_sample(tdev);
	config_converter_resolution(tdev);

	config_subclass(tdev);
	config_version(tdev);

	if (tdev->type == JESD_DEV_TX) {
		/*config NP*/
		reg = &tdev->tx_regs->tx_nbcw;
		val = ((tdev->ils.bits_per_converter - 1) & NP_MASK)
				<< NP_SHIFT;
		mask = NP_MASK << NP_SHIFT;
		jesd_update_reg(reg, val, mask);

		/*config S*/
		reg = &tdev->tx_regs->tx_spcp;
		val =  ((tdev->ils.samples_per_cnvrtr_per_frame - 1) & S_MASK)
				<<  S_SHIFT;
		mask = S_MASK << S_SHIFT;
		jesd_update_reg(reg, val, mask);

		/*config H */
		val = ioread32(&tdev->tx_regs->tx_hdf);
		if (tdev->ils.high_density_enable)
			val |= HD_ENABLE;
		else
			val &= ~HD_ENABLE;
		iowrite32(val, &tdev->tx_regs->tx_hdf);

		/*config CF*/
		reg = &tdev->tx_regs->tx_hdf;
		val =  (tdev->ils.control_wrds_per_frame & CF_MASK)
				<< CF_SHIFT;
		mask = CF_MASK << CF_SHIFT;
		jesd_update_reg(reg, val, mask);

	} else if (tdev->type == JESD_DEV_TX) {
			/*config NP*/
		reg = &tdev->rx_regs->rx_nbcw;
		val = ((tdev->ils.bits_per_converter - 1) & NP_MASK)
				<< NP_SHIFT;
		mask = NP_MASK << NP_SHIFT;
		jesd_update_reg(reg, val, mask);

		/*config S*/
		reg = &tdev->rx_regs->rx_spcp;
		val =  ((tdev->ils.samples_per_cnvrtr_per_frame - 1) & S_MASK)
				<<  S_SHIFT;
		mask = S_MASK << S_SHIFT;
		jesd_update_reg(reg, val, mask);

		/*config H */
		val = ioread32(&tdev->rx_regs->rx_hdf);
		if (tdev->ils.high_density_enable)
			val |= HD_ENABLE;
		else
			val &= ~HD_ENABLE;
		iowrite32(val, &tdev->rx_regs->rx_hdf);

		/*config CF*/
		reg = &tdev->rx_regs->rx_hdf;
		val =  (tdev->ils.control_wrds_per_frame & CF_MASK)
				<< CF_SHIFT;
		mask = CF_MASK << CF_SHIFT;
		jesd_update_reg(reg, val, mask);
	}

out:
	return rc;
}

static int jesd_tx_stop(struct jesd_transport_dev *tdev)
{
	u32 *reg, val, mask;
	struct config_registers_tx *tx_regs = tdev->tx_regs;

	/* Stop timers */
	tbgen_timer_disable(tdev->timer_handle);
	tbgen_timer_disable(tdev->txalign_timer_handle);

	reg = &tx_regs->tx_frm_ctrl;
	val = ~TRANSMIT_EN;
	mask = TRANSMIT_EN;
	jesd_update_reg(reg, val, mask);

	return 0;
}

static int jesd_rx_stop(struct jesd_transport_dev *tdev)
{
	u32 *reg, val, mask;
	struct config_registers_rx *rx_regs = tdev->rx_regs;

	reg = &rx_regs->rx_transcontrol;
	val = ~SW_DMA_ENABLE;
	mask = SW_DMA_ENABLE;
	jesd_update_reg(reg, val, mask);

	/* Stop timer*/
	tbgen_timer_disable(tdev->timer_handle);

	reg = &rx_regs->rx_ctrl_0;
	val = RX_DIS;
	mask = RX_DIS;
	jesd_update_reg(reg, val, mask);

	return 0;
}

static int jesd_dev_stop(struct jesd_transport_dev *tdev)
{
	int rc = 0;

	rc = jesd_change_state(tdev, JESD_STATE_STOPPED);
	if (rc) {
		dev_err(tdev->dev, "%s: Cannot be stopped in %d state\n",
			__func__, tdev->state);
		goto out;
	}

	if (tdev->type == JESD_DEV_TX)
		rc = jesd_tx_stop(tdev);
	else
		rc = jesd_rx_stop(tdev);

	return rc;
out:
	jesd_restore_state(tdev);
	return rc;
}

static void jesd_restore_state(struct jesd_transport_dev *tdev)
{
	tdev->state = tdev->old_state;
}

static int jesd_change_state(struct jesd_transport_dev *tdev,
	enum jesd_state new_state)
{
	enum jesd_state old_state = tdev->state;
	int rc = -EINVAL;

	if (new_state == old_state)
		goto out;

	switch (old_state) {
	case JESD_STATE_STANDBY:
		if (new_state == JESD_STATE_CONFIGURED)
			if (JESD_CHCK_CONFIG_MASK(tdev, JESD_CONFIGURED_MASK))
				rc = 0;
		break;
	case JESD_STATE_CONFIGURED:
		if (new_state == JESD_STATE_ENABLED)
			rc = 0;
		break;
	case JESD_STATE_ENABLED:
		if (new_state == JESD_STATE_CODE_GRP_SYNC)
			rc = 0;
		if (new_state == JESD_STATE_ILAS)
			rc = 0;
		if (new_state == JESD_STATE_READY)
			rc = 0;
		if (new_state == JESD_STATE_SYNC_FAILED)
			rc = 0;
		break;
	case JESD_STATE_SYNC_FAILED:
		if (new_state == JESD_STATE_CONFIGURED)
			rc = 0;
		if (new_state == JESD_STATE_ENABLED)
			rc = 0;
		break;
	case JESD_STATE_READY:
		if (new_state == JESD_STATE_STOPPED)
			rc = 0;
		if (new_state == JESD_STATE_LINK_ERROR)
			rc = 0;
	case JESD_STATE_LINK_ERROR:
	case JESD_STATE_STOPPED:
		if (new_state == JESD_STATE_CONFIGURED)
			rc = 0;
		if (new_state == JESD_STATE_ENABLED)
			rc = 0;
		break;
	default:
		dev_info(tdev->dev, "Invalid state transition %d -> %d\n",
			old_state, new_state);
		goto out;
	}

	if (!rc) {
		dev_info(tdev->dev, "%s: Transitiong state %d -> %d\n",
			tdev->name, old_state, new_state);
		tdev->state = new_state;
		tdev->old_state = old_state;
	}
out:
	return rc;
}

static long jesd204_ioctl(struct file *pfile, unsigned int cmd,
						unsigned long arg)
{
	int rc = -ENOSYS, size = 0, int_arg;
	u32 *regs = NULL;
	struct jesd_transport_dev *tdev = NULL;
	struct jesd_dev_params dev_params;
	struct jesd_transport_dev_info trans_dev_info;
	struct jesd_reg_read_buf regcnf;
	struct jesd_reg_write_buf reg_write_buf;
	struct auto_sync_params sync_params;
	struct conf_rx_tests rx_tests;
	struct conf_tx_tests tx_tests;

	tdev = pfile->private_data;

	switch (cmd) {
	case JESD_DEVICE_INIT:

		if (copy_from_user(&dev_params,
				(struct jesd_dev_params *)arg,
				 sizeof(struct jesd_dev_params))) {
			rc = -EFAULT;
			break;
		}

		rc = jesd_init_transport(tdev, &dev_params);
		if (!rc) {
			JESD_SET_CONFIG_MASK(tdev, JESD_CONF_DEV_INIT);
			jesd_change_state(tdev, JESD_STATE_CONFIGURED);
		}
		break;
	case JESD_SET_LANE_PARAMS:
		if (copy_from_user(&tdev->ils,
				(struct ils_params *)arg,
				 sizeof(struct ils_params))) {
			rc = -EFAULT;
			break;
		}
		rc = jesd_set_ils_pram(tdev);
		if (!rc) {
			JESD_SET_CONFIG_MASK(tdev, JESD_CONF_ILS_INIT);
			jesd_change_state(tdev, JESD_STATE_CONFIGURED);
		} else {
			/* If ILAS param init fails, then clean thigns up */
			memset(&tdev->ils, 0, sizeof(struct ils_params));
			jesd_set_ils_pram(tdev);
		}
		break;
	case JESD_SET_ILS_LENGTH:
		if (get_user(int_arg, (int *) arg)) {
			rc = -EFAULT;
			break;
		}
		rc = jesd_set_ilas_len(tdev, int_arg);
		if (!rc) {
			JESD_SET_CONFIG_MASK(tdev, JESD_CONF_ILS_LEN_INIT);
			jesd_change_state(tdev, JESD_STATE_CONFIGURED);
		}
		break;
	case JESD_DEVICE_START:
		rc = jesd_start_transport(tdev);
		break;
	case JESD_TX_TEST_MODE:
		if (copy_from_user(&tx_tests,
				(struct conf_tx_tests *)arg,
				sizeof(struct conf_tx_tests))) {
			rc = -EFAULT;
			goto fail;
		}
		rc = jesd_conf_tx_tests(tdev, &tx_tests);
		break;
	case JESD_RX_TEST_MODE:
		if (copy_from_user(&rx_tests,
				(struct conf_rx_tests *)arg,
				sizeof(struct conf_rx_tests))) {
			rc = -EFAULT;
			goto fail;
		}
		rc = jesd_conf_rx_tests(tdev, &rx_tests);
		break;
	case JESD_FORCE_SYNC:
		rc = jesd_force_sync(tdev);
		break;
	case JESD_GET_STATS:
		dev_err(tdev->dev, "%s: Not implemented\n", tdev->name);
		rc = -ENOSYS;
		break;
	case JESD_CLEAR_STATS:
		memset(&tdev->t_stats, 0, sizeof(struct lane_stats));
		memset(&tdev->lane_devs[0]->l_stats, 0,
			sizeof(struct lane_stats));
		memset(&tdev->lane_devs[1]->l_stats, 0,
			sizeof(struct lane_stats));
		break;
	case JESD_WRITE_REG:

		if (copy_from_user(&reg_write_buf,
			(struct jesd_reg_write_buf *)arg,
			sizeof(struct jesd_reg_write_buf))) {
			rc = -EFAULT;
			break;
		}
		rc = jesd_reg_write_from_user(tdev, reg_write_buf.regs,
			reg_write_buf.count);
		break;
	case JESD_READ_REG:
	case JESD_READ_PHYGASKET_REG:

		if (copy_from_user(&regcnf, (struct jesd_reg_read_buf *)arg,
			sizeof(struct jesd_reg_read_buf))) {
			rc = -EFAULT;
			goto fail;
		}

		if (tdev->type == JESD_DEV_TX) {
			size = sizeof(struct config_registers_tx);
			regs = (u32 *) tdev->tx_regs;
		} else {
			size = sizeof(struct config_registers_rx);
			regs = (u32 *)tdev->rx_regs;
		}

		if (cmd == JESD_READ_PHYGASKET_REG) {
			size = sizeof(struct phy_gasket_regs);
			regs = (u32 *)tdev->phy->pregs;
		}

		if (((regcnf.len * 4) + regcnf.offset > size)) {
			dev_err(tdev->dev, "%s: invalid len/offset (%d/0x%x)",
				tdev->name, regcnf.len, regcnf.offset);
			rc = -EINVAL;
			break;
		}

		rc = jesd_reg_dump_to_user(regs, regcnf.offset,
			regcnf.len, regcnf.buf);
		break;
	case JESD_DEVICE_STOP:
		rc = jesd_dev_stop(tdev);
		break;
	case JESD_GET_DEVICE_INFO:

		memcpy(&trans_dev_info.ils, &tdev->ils,
						sizeof(struct ils_params));
		trans_dev_info.init_params.data_rate =
						tdev->data_rate;
		trans_dev_info.init_params.delay =
						tdev->delay;
		trans_dev_info.init_params.config_flags =
						tdev->config_flags;
		trans_dev_info.init_params.lanes =
						tdev->active_lanes;
		trans_dev_info.init_params.ilas_length = tdev->ilas_len;
		trans_dev_info.state = tdev->state;

		if (copy_to_user((struct jesd_transport_dev_info *)arg,
					&trans_dev_info,
					sizeof(struct jesd_transport_dev_info)))
				rc = -EFAULT;
		break;
	case JESD_GET_LANE_RX_RECEIVED_PARAMS:
		dev_info(tdev->dev, "%s: Not Implemented\n", tdev->name);
		rc = -ENOSYS;
		break;
	case JESD_RX_AUTO_SYNC:
		if (copy_from_user(&sync_params, (struct auto_sync_params *)arg,
				sizeof(struct auto_sync_params))) {
			rc = -EFAULT;
			break;
		}
		rc = jesd_conf_auto_sync(tdev, &sync_params);
		break;
	case JESD_GET_DEVICE_STATE:
		if (put_user(tdev->state, (u32 *) arg))
			rc = -EFAULT;
		else
			rc = 0;
		break;
	default:
		rc = -ENOTTY;
		break;
	}
fail:
	return rc;
}

static int jesd_conf_auto_sync(struct jesd_transport_dev *tdev,
	struct auto_sync_params *sync_params)
{
	int rc = 0;
	u32 threshold, *reg, val = 0, mask, assert_mask;

	if (tdev->type == JESD_DEV_TX) {
		rc = -EINVAL;
		goto out;
	}

	assert_mask = sync_params->sync_assert_mask;
	reg = &tdev->rx_regs->rx_sync_ass_msk;

	if (assert_mask & SYNC_ASSERT_BAD_DIS)
		val |= BAD_DIS_S;
	else
		val &= ~BAD_DIS_S;

	if (assert_mask & SYNC_ASSERT_NIT)
		val |= NIT_DIS_S;
	else
		val &= ~NIT_DIS_S;

	if (assert_mask & SYNC_ASSERT_UNEXPECTED_K_CHARS)
		val |= UNEX_K_S;
	else
		val &= ~UNEX_K_S;

	mask = UNEX_K_S | NIT_DIS_S | BAD_DIS_S;
	jesd_update_reg(reg, val, mask);
	threshold = sync_params->error_threshold & ERR_THRESHOLD_MASK;
	iowrite32(threshold, &tdev->rx_regs->rx_threshold_err);
out:
	return rc;
}

static int jesd204_open(struct inode *inode, struct file *pfile)
{
	struct jesd_transport_dev *tdev = NULL;
	int rc = 0;

	tdev = container_of(inode->i_cdev, struct jesd_transport_dev, c_dev);
	if (tdev != NULL) {
		pfile->private_data = tdev;
		atomic_inc(&tdev->ref);
	} else {
		rc = -ENODEV;
	}

	return rc;
}

int jesd204_release(struct inode *inode, struct file *pfile)
{
	struct jesd_transport_dev *tdev = NULL;
	tdev = pfile->private_data;

	atomic_dec(&tdev->ref);
	return 0;
}


static void jesd_link_monitor(struct jesd_transport_dev *tdev)
{
	u32 *diag_sel_reg, *diag_data_reg, val1, val2;
	int requeue = 1;

	if (tdev->type == JESD_DEV_TX) {
		diag_sel_reg = &tdev->tx_regs->tx_diag_sel;
		diag_data_reg = &tdev->tx_regs->tx_diag_data;
	} else {
		diag_sel_reg = &tdev->rx_regs->rx_diag_sel;
		diag_data_reg = &tdev->rx_regs->rx_diag_data;
	}

	iowrite32(DIAG_FRAMER_STATE_SEL, diag_sel_reg);

	val1 = ioread32(diag_data_reg);
	val2 = ioread32(diag_data_reg);
	/* RM recommends to use value only when two consecutive reads
	 * return same value
	 */
	 if (val1 != val2) {
		dev_dbg(tdev->dev, "%s:diag_data not consistent (%x, %x)\n",
			tdev->name, val1, val2);
		goto out;
	}

	switch (val1) {
	case FRAMER_STATE_CODE_GRP_SYNC:
		jesd_change_state(tdev, JESD_STATE_CODE_GRP_SYNC);
		break;
	case FRAMER_STATE_ILAS:
		jesd_change_state(tdev, JESD_STATE_ILAS);
		break;
	case FRAMER_STATE_USER_DATA:
		jesd_change_state(tdev, JESD_STATE_READY);
		tbgen_timer_enable(tdev->timer_handle);
		/* We don't need sysref now till link is restarted */
		jesd_marks_syref_capture_ready(tdev, 0);
		requeue = 0;
		break;
	default:
		break;
	}

out:
	if (requeue) {
		if (tdev->sync_timeout_ms < jiffies) {
			schedule_work(&tdev->link_monitor);
		} else {
			dev_err(tdev->dev, "%s: Failed to syncronize\n",
				tdev->name);
			jesd_change_state(tdev, JESD_STATE_SYNC_FAILED);
		}
	}
}

void jesd_link_monitor_worker(struct work_struct *work)
{
	struct jesd_transport_dev *tdev;

	tdev = container_of(work, struct jesd_transport_dev, link_monitor);

	jesd_link_monitor(tdev);
}

static void jesd_handle_sysref_rose(struct jesd_transport_dev *tdev)
{
	u32 *ctrl_reg, *irq_en_reg, *reg, val, mask;

	if (tdev->type == JESD_DEV_TX) {
		ctrl_reg = &tdev->tx_regs->tx_transcontrol;
		irq_en_reg = &tdev->tx_regs->tx_irq_enable;
	} else {
		ctrl_reg = &tdev->rx_regs->rx_transcontrol;
		irq_en_reg = &tdev->rx_regs->rx_irq_enable;
	}

	/* reset Deskew FIFO */
	val = DFIFO_SWRESET;
	mask = DFIFO_SWRESET;
	jesd_update_reg(ctrl_reg, val, mask);
	udelay(20);
	val = ~DFIFO_SWRESET;
	jesd_update_reg(ctrl_reg, val, mask);

	val = SYSREF_MASK;
	mask = SYSREF_MASK;
	jesd_update_reg(ctrl_reg, val, mask);

	val = IRQ_SYSREF_ROSE;
	mask = IRQ_SYSREF_ROSE;
	jesd_update_reg(irq_en_reg, val, mask);

	/*Enable Tx/Rx*/
	if (tdev->type == JESD_DEV_TX) {
		reg = &tdev->tx_regs->tx_frm_ctrl;
		val = TRANSMIT_EN;
		mask = TRANSMIT_EN;
		jesd_update_reg(reg, val, mask);
	} else {
		reg = &tdev->rx_regs->rx_ctrl_0;
		val = ~RX_DIS;
		mask = RX_DIS;
		jesd_update_reg(reg, val, mask);
	}

	/*Enable TX alignment Timer for Tx or Raise SYNC for RX*/
	if (tdev->type == JESD_DEV_TX)
		tbgen_timer_enable(tdev->txalign_timer_handle);
	else
		jesd_force_sync(tdev);

	tdev->sync_expire = jiffies + msecs_to_jiffies(tdev->sync_timeout_ms);
	schedule_work(&tdev->link_monitor);
}

static void jesd_tx_tasklet(unsigned long data)
{
	struct jesd_transport_dev *tdev = (struct jesd_transport_dev *)data;
	struct config_registers_tx *tx_regs = tdev->tx_regs;
	u32 irq_status, *reg, val, mask;

	irq_status = ioread32(&tx_regs->tx_irq_status);
	iowrite32(irq_status, &tx_regs->tx_irq_status);

	if (irq_status & IRQ_SYSREF_ROSE)
		jesd_handle_sysref_rose(tdev);

	/*Enable interrupts we serviced*/
	val = irq_status;
	mask = irq_status;
	reg = &tx_regs->tx_irq_enable;
	jesd_update_reg(reg, val, mask);
}

static irqreturn_t jesd_tx_isr(int irq, void *param)
{
	struct jesd_transport_dev *tdev = param;
	struct config_registers_tx *tx_regs = tdev->tx_regs;
	u32 irq_status, *reg, mask, val;

	irq_status = ioread32(&tx_regs->tx_irq_status);
	if (!irq_status) {
		dev_info(tdev->dev, "%s: Spurious IRQ\n", tdev->name);
		return IRQ_NONE;
	}

	/*Mask off IRQs that we got*/
	val = ~irq_status;
	mask = irq_status;
	reg = &tx_regs->tx_irq_enable;
	jesd_update_reg(reg, val, mask);
	tasklet_schedule(&tdev->tx_tasklet);

	return IRQ_HANDLED;
}

static void jesd_rx_tasklet(unsigned long data)
{
	struct jesd_transport_dev *tdev = (struct jesd_transport_dev *)data;
	struct config_registers_rx *rx_regs = tdev->rx_regs;
	u32 irq_status, *reg, val, mask;

	irq_status = ioread32(&rx_regs->rx_irq_status);
	iowrite32(irq_status, &rx_regs->rx_irq_status);

	if (irq_status & IRQ_SYSREF_ROSE)
		jesd_handle_sysref_rose(tdev);

	/*Enable interrupts we serviced*/
	val = irq_status;
	mask = irq_status;
	reg = &rx_regs->rx_irq_enable;
	jesd_update_reg(reg, val, mask);
}

static irqreturn_t jesd_rx_isr(int irq, void *param)
{
	struct jesd_transport_dev *tdev = param;
	struct config_registers_rx *rx_regs = tdev->rx_regs;
	u32 irq_status, *reg, mask, val;

	irq_status = ioread32(&rx_regs->rx_irq_status);
	if (!irq_status) {
		dev_info(tdev->dev, "%s: Spurious IRQ\n", tdev->name);
		return IRQ_NONE;
	}

	/*Mask off IRQs that we got*/
	val = ~irq_status;
	mask = irq_status;
	reg = &rx_regs->rx_irq_enable;
	jesd_update_reg(reg, val, mask);
	tasklet_schedule(&tdev->rx_tasklet);

	return IRQ_HANDLED;
}

static int transport_register_irq(struct jesd_transport_dev *tdev)
{

	int rc = 0;

	if (tdev != NULL) {

		if (tdev->type == JESD_DEV_TX) {
			rc = request_irq(tdev->irq, jesd_tx_isr, 0,
					tdev->name, tdev);
			if (!rc)
				tasklet_init(&tdev->tx_tasklet,
					jesd_tx_tasklet, (unsigned long) tdev);
		} else {
			rc = request_irq(tdev->irq, jesd_rx_isr, 0,
				tdev->name, tdev);
			if (!rc)
				tasklet_init(&tdev->rx_tasklet,
					jesd_rx_tasklet, (unsigned long) tdev);
		}

		if (rc) {
			dev_err(tdev->dev, "%s:Failed to register irq\n",
				tdev->name);
			goto out;
		}

	}

out:
	return rc;
}

/** @brief file ops structure
 */
static const struct file_operations jesd204_fops = {
	.owner = THIS_MODULE,
	.open = jesd204_open,
	.unlocked_ioctl = jesd204_ioctl,
	.release = jesd204_release
};

static int create_c_dev(struct jesd_transport_dev *tdev,
			int jesd204_major,
			int jesd_minor)
{
	int rc = -ENODEV;
	struct jesd_transport_dev *t = tdev;
	t->devt = MKDEV(jesd204_major, jesd_minor + t->id);

	cdev_init(&t->c_dev, &jesd204_fops);
	t->c_dev.owner = THIS_MODULE;
	rc = cdev_add(&t->c_dev, t->devt, 1);

	return rc;
}

static struct jesd_transport_dev *
	create_jesd_transports(struct jesd204_dev *jdev,
	struct device_node *dev_node, unsigned int id, struct device *dev)
{
	int type = jdev->trans_type, rc;
	struct jesd_transport_dev *tdev = NULL;
	u32 *base, max_lanes;

	tdev = kzalloc(sizeof(struct jesd_transport_dev), GFP_KERNEL);

	if (!tdev) {
		dev_err(dev, "Failed to allocate transport [%d]\n", id);
		goto out;
	}
	atomic_set(&tdev->ref, 0);
	tdev->parent = jdev;
	tdev->id = id;
	tdev->dev = dev;
	tdev->type = type;
	tdev->sync_timeout_ms = JESD_SYNC_TIMEOUT_MS;
	base = of_iomap(dev_node, 0);

	if (!base) {
		dev_err(tdev->dev, "transport memap maping failure");
		goto out;
	}

	/*logical assignment of register to the transport*/
	if (type == JESD_DEV_TX) {
		tdev->tx_regs = (struct config_registers_tx *) base;
		tdev->rx_regs = NULL;
	} else if ((type == JESD_DEV_RX) || (type == JESD_DEV_SRX)) {
		tdev->rx_regs = (struct config_registers_rx *)base;
		tdev->tx_regs = NULL;
	} else {
		dev_err(tdev->dev, "Invalid transport dev type %x\n", type);
		goto out;
	}

	get_device_name(tdev);
	rc = of_property_read_u32(dev_node, "max-lanes", &max_lanes);

	if (rc) {
		dev_info(tdev->dev, "max_lanes not found, setting to 1\n");
		tdev->max_lanes = 1;
	} else {
		tdev->max_lanes = max_lanes;
		dev_info(tdev->dev, "%s: max_lanes %d\n", tdev->name,
			tdev->max_lanes);
	}
	if (tdev->max_lanes > jdev->max_lanes) {
		dev_err(tdev->dev, "%s: trans lanes [%d] > dev lanes [%d]\n",
			tdev->name, tdev->max_lanes, jdev->max_lanes);
		rc = -EINVAL;
		goto out;
	}

	rc = of_property_read_u32(dev_node, "timer-id", &tdev->timer_id);
	if (rc) {
		dev_err(tdev->dev, "%s: Timer id property not found\n",
			tdev->name);
		goto out;
	}

	INIT_WORK(&tdev->link_monitor, jesd_link_monitor_worker);

	return tdev;
out:
	kfree(tdev);
	return NULL;
}

void jdev_list_cleanup(void)
{
	struct jesd204_dev *cur, *nxt;

	list_for_each_entry_safe(cur, nxt, &pri->list, jesd_dev_list) {
		list_del(&cur->jesd_dev_list);
	}
}

static int get_device_name(struct jesd_transport_dev *tdev)
{
	int rc = 0;

	if (tdev->type == JESD_DEV_TX)
		sprintf(&tdev->name[0], "jesd_tx_%d", tdev->id);
	else if ((tdev->type == JESD_DEV_RX) || tdev->type == JESD_DEV_SRX)
		sprintf(&tdev->name[0], "jesd_rx_%d", tdev->id);
	else
			rc = -EINVAL;
	return rc;
}

static int enable_transport(struct jesd_transport_dev *tdev)
{

	u32 *trans_ctrl_reg = NULL, val, mask;

	if (tdev->type == JESD_DEV_TX)
		trans_ctrl_reg = &tdev->tx_regs->tx_transcontrol;
	else
		trans_ctrl_reg = &tdev->rx_regs->rx_transcontrol;

	val = CLK_ENABLE;
	mask = CLK_ENABLE;
	jesd_update_reg(trans_ctrl_reg, val, mask);

	return 0;
}

static int __init jesd204_of_probe(struct platform_device *pdev)
{
	int rc = -ENODEV;
	int jesd_major, jesd_minor, max_lanes;
	unsigned int id, i = 0;
	struct jesd204_dev *jdev = NULL;
	struct device_node *child = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct jesd_transport_dev *tdev = NULL;

	jdev = kzalloc(sizeof(struct jesd204_dev), GFP_KERNEL);

	if (!jdev) {
		rc = -ENOMEM;
		goto out;
	}

	jdev->dev = &pdev->dev;

	rc = alloc_chrdev_region(&jdev->devt, 0,
		TRANSPORTS_PER_JESD_DEV, "jesd204");
	if (rc) {
		dev_err(jdev->dev, "Char region allocation failed, err %d\n",
			rc);
		goto out;
	}

	jesd_major = MAJOR(jdev->devt);
	jesd_minor = MINOR(jdev->devt);

	jdev->node = pdev->dev.of_node;
	rc = of_property_read_u32(node, "lanes", &max_lanes);

	if (rc) {
		dev_err(tdev->dev, "max_lanes property not found\n\n");
		goto out;
	}
	jdev->max_lanes = max_lanes;

	for_each_child_of_node(node, child) {
		rc = of_property_read_u32(child,
						"transport-type",
						&jdev->trans_type);
		if (rc < 0) {
			dev_err(jdev->dev, "Trans type failed  %x",
				jdev->trans_type);
			goto out;
		}

		rc = of_property_read_u32(child, "transport-id", &id);

		if (rc < 0) {
			dev_err(jdev->dev, "transport-id property not found\n");
			goto out;
		}

		tdev = create_jesd_transports(jdev, child, id, &pdev->dev);

		if (!tdev) {
			dev_err(jdev->dev, "Failed to create transport %d\n",
				id);
			goto out;
		}

		rc = create_c_dev(tdev, jesd_major, jesd_minor);
		if (rc < 0) {
			dev_err(jdev->dev, "cdev add failed\n");
			goto out;
		}

		/* get the maped phygasket node into here */
		tdev->phy_node = of_parse_phandle(child,
						"phy-gasket", 0);
		tdev->phy = map_phygasket(tdev->phy_node);

		if (tdev->phy == NULL) {
			dev_err(jdev->dev, "phy gasket is not mapped\n");
			goto out;
		}

		tdev->irq = irq_of_parse_and_map(child, 0);
		if (tdev->irq) {
			rc = transport_register_irq(tdev);

			if (rc < 0) {
				dev_err(jdev->dev, "irq register failed\n");
				goto out;
			}
		} else {
			dev_info(jdev->dev, "%s: No IRQ, disabling events\n",
				tdev->name);
			tdev->dev_flags |= DEV_FLG_NO_TRANSPORT_EVENTS;
		}

		init_waitqueue_head(&tdev->isr_wait);
		init_waitqueue_head(&tdev->to_wait);

		spin_lock_init(&tdev->lock);
		device_create(jesd204_class, tdev->dev, tdev->devt,
			NULL, tdev->name);
		tdev->state = JESD_STATE_STANDBY;
		jdev->transports[i] = tdev;
		dev_info(jdev->dev, "%s JESD device created\n", tdev->name);
		jesd_minor++;
		i++;
	}

	if (rc < 0) {
		dev_info(jdev->dev, "jesd probe failed\n");
		goto out;
	}

	list_add_tail(&jdev->jesd_dev_list, &pri->list);
	dev_set_drvdata(&pdev->dev, jdev);

	enable_transport(tdev);
	return rc;

out:
	if (jdev)
		unregister_chrdev_region(jdev->devt, TRANSPORTS_PER_JESD_DEV);
	kfree(jdev);
	return rc;
}

static int __exit jesd204_of_remove(struct platform_device *pdev)
{
	int id = 0;
	int rc = 0;
	struct jesd204_dev *jdev = NULL;
	u32 *regs = NULL;

	jdev = (struct jesd204_dev *)dev_get_drvdata(&pdev->dev);

	if (!jdev) {
		rc = -ENODEV;
		goto error;
	}

	for (id = 0; id < TRANSPORTS_PER_JESD_DEV; id++) {
		jdev_list_cleanup();
		device_destroy(jesd204_class, jdev->transports[id]->devt);
		cdev_del(&jdev->transports[id]->c_dev);
		if (jdev->trans_type == JESD_DEV_TX)
			regs = (u32 *) jdev->transports[id]->tx_regs;
		else if (jdev->trans_type == JESD_DEV_RX)
			regs = (u32 *) jdev->transports[id]->rx_regs;
		iounmap(regs);
		cancel_work_sync(&jdev->transports[id]->link_monitor);
		kfree(jdev->transports[id]);
	}

	kfree(jdev);

error:
	return rc;
}


static struct of_device_id jesd204_match_dts_id[] = {
	{.compatible = "fsl,d4400-jesd204-tx",},
	{.compatible = "fsl,d4400-jesd204-rx",},
	{},
};

static struct platform_driver jesd204_driver = {
	.driver = {
		.name = "fsl-jesd204",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(jesd204_match_dts_id),
	},
	.probe = jesd204_of_probe,
	.remove = jesd204_of_remove,
};


static int __init jesd204_module_init(void)
{
	int rc;

	pri = kzalloc(sizeof(struct jesd204_private), GFP_KERNEL);

	if (!pri) {
		rc = -ENOMEM;
		goto jesd204_err;
	}
	INIT_LIST_HEAD(&pri->list);
	pri->tx_dev_num = 0;
	pri->rx_dev_num = 0;

	jesd204_class = class_create(THIS_MODULE, "jesd204");

	if (IS_ERR(jesd204_class)) {
		rc = PTR_ERR(jesd204_class);
		goto jesd204_err;
	}

	return platform_driver_register(&jesd204_driver);

jesd204_err:
		return rc;
}

static void __exit jesd204_module_cleanup(void)
{
	kfree(pri);
	class_destroy(jesd204_class);
	platform_driver_unregister(&jesd204_driver);
}

module_init(jesd204_module_init);
module_exit(jesd204_module_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("jesd204 driver");

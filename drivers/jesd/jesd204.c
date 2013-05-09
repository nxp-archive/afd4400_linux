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
#include <linux/tbgen.h>


#include "jesd204.h"
#include "phygasket.h"

#define JESD_TEST

#define TRANS_TX_NAME "jesd_tx"
#define TRANS_RX_NAME "jesd_rx"


static struct class *jesd204_class;
static struct jesd204_private *pri;

static int jesd_set_transport_pram(struct transport_device *tdev,
					struct conf_tr __user *trans_p);
static int jesd_set_ils_pram(struct transport_device *tdev);
static int jesd_set_ilsa_len(struct transport_device *tdev, int length);
static int shutdown(struct transport_device *tdev);
static int force_sync(struct transport_device *tdev);

/*support functions*/
static int setclkdiv(struct transport_device *tdev);
static int conf_phy_gasket(struct transport_device *tdev);

static void write_reg(u32 *reg,
			unsigned int offset,
			unsigned int length,
			u32 *buf);
static void read_reg(u32 *reg,
			unsigned int offset,
			unsigned int length,
			u32 *buf);

static int jesd_conf_tests(struct transport_device *tdev);
static void raise_exception(struct transport_device *tdev,
				unsigned int event_typ);

/**@brief inline for set, clear and test bits for 32 bit
*/
static inline void
jclear_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) &= ~(1 << (nr & 31));
}

static inline void
jset_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) |= (1 << (nr & 31));
}

static inline int
jtest_bit(int nr, const void *addr)
{
	return 1 & (((const __u32 *) addr)[nr >> 5] >> (nr & 31));
}

void jesd_set_bit(int nr, void *addr)
{
	u32 reg_val = 0;
	reg_val = ioread32(addr);
	jset_bit(nr, &reg_val);
	iowrite32(reg_val, addr);
}

void jesd_clear_bit(int nr, void *addr)
{
	u32 reg_val = 0;
	reg_val = ioread32(addr);
	jclear_bit(nr, &reg_val);
	iowrite32(reg_val, addr);
}

static void handel_skew_err(void)
{
	/*yet to be impelemented
	invalid function until req is cleared */
}

static int phy_config_tx_testmode(struct transport_device *tdev)
{
	int retcode = 0;
	phy_gasket_lane_ctrl(tdev->phy, TEST_PATT,
					tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
				tdev->lanes_per_transport == 2) {

		tdev->lane_dev[LANEID_FOR_1]->id =
					tdev->lane_id[LANEID_FOR_SECONDARY];

		if (tdev->lane_dev[LANEID_FOR_1]->id < 0) {
			retcode = -EINVAL;
			goto fail;
		}

		phy_gasket_lane_ctrl(tdev->phy, TEST_PATT,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}

	do_patter_generator(tdev->phy, &tdev->tx_tests.tx_patgen);

fail:
	return retcode;
}

static int phy_config_rx_testmode(struct transport_device *tdev)
{
	int retcode = 0;
	phy_gasket_lane_ctrl(tdev->phy, RX_LOOPBACK,
					tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
				tdev->lanes_per_transport == 2) {
		tdev->lane_dev[LANEID_FOR_1]->id =
				tdev->lane_id[LANEID_FOR_SECONDARY];

		if (tdev->lane_dev[LANEID_FOR_1]->id < 0) {
			retcode = -EINVAL;
			goto fail;
		}
		phy_gasket_lane_ctrl(tdev->phy, RX_LOOPBACK,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}

	do_patter_generator(tdev->phy, &tdev->tx_tests.tx_patgen);
fail:
	return retcode;
}

static int phy_config_tx_normalmode(struct transport_device *tdev)
{
	int retcode = 0;

	phy_gasket_lane_ctrl(tdev->phy, JESD204TX,
					tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
				tdev->lanes_per_transport == 2) {

		tdev->lane_dev[LANEID_FOR_1]->id =
				tdev->lane_id[LANEID_FOR_SECONDARY];

		if (tdev->lane_dev[LANEID_FOR_1]->id < 0) {
			retcode = -EINVAL;
			goto fail;
		}

		phy_gasket_lane_ctrl(tdev->phy, JESD204TX,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}
fail:
	return retcode;
}

static int phy_config_rx_normalmode(struct transport_device *tdev)
{
	int retcode = 0;

	phy_gasket_lane_ctrl(tdev->phy, RX_PHY,
					tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
					tdev->lanes_per_transport == 2) {

		tdev->lane_dev[LANEID_FOR_1]->id =
				tdev->lane_id[LANEID_FOR_SECONDARY];

		if (tdev->lane_dev[LANEID_FOR_1]->id < 0) {
			retcode = -EINVAL;
			goto fail;
		}

		phy_gasket_lane_ctrl(tdev->phy, RX_PHY,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}

fail:
	return retcode;
}


static void do_tx_soft_reset(struct transport_device *tdev)
{
	/*soft reset == 1*/
	phy_softreset(tdev->phy, DEVICE_TX, 1,
					tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
					tdev->lanes_per_transport == 2) {
		phy_softreset(tdev->phy, DEVICE_TX, 1,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}
	/*soft reset == 0*/
	phy_softreset(tdev->phy, DEVICE_TX, 0,
					tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
				tdev->lanes_per_transport == 2) {
		phy_softreset(tdev->phy, DEVICE_TX, 0,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}
}

static void do_rx_soft_reset(struct transport_device *tdev)
{
	/*soft reset == 1*/
	phy_softreset(tdev->phy, DEVICE_RX, 1,
					tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
					tdev->lanes_per_transport == 2) {
		phy_softreset(tdev->phy, DEVICE_RX, 1,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}
	/*soft reset == 0*/
	phy_softreset(tdev->phy, DEVICE_RX, 0,
				tdev->lane_dev[LANEID_FOR_0]->id);

	if (tdev->identifier == TRANPORT_0 &&
					tdev->lanes_per_transport == 2) {
		phy_softreset(tdev->phy, DEVICE_RX, 0,
				tdev->lane_dev[LANEID_FOR_1]->id);
	}
}

static void dfifo_reset(struct transport_device *tdev)
{
	if (tdev->transport_type == DEVICE_TX) {
		/*reset dfifo pointers now*/
		jesd_set_bit(DFIFO_BIT3, &tdev->tx_regs->tx_transcontrol);
		/*reset done*/
		jesd_clear_bit(DFIFO_BIT3, &tdev->tx_regs->tx_transcontrol);
	} else if (tdev->transport_type == DEVICE_RX) {
		/*reset dfifo pointers now*/
		jesd_set_bit(DFIFO_BIT3, &tdev->rx_regs->rx_transcontrol);
		/*reset done*/
		jesd_clear_bit(DFIFO_BIT3, &tdev->rx_regs->rx_transcontrol);
	}
}

static int conf_tx_phy_gasket(struct transport_device *tdev)
{
	int retcode = 0;

	tdev->lane_dev[LANEID_FOR_0]->id =
					tdev->lane_id[LANEID_FOR_PRIMARY];

	if (tdev->lane_dev[LANEID_FOR_0]->id < 0) {
		retcode = -EINVAL;
		goto fail;
	}

	if (tdev->dev_state == TESTMODE)
		retcode = phy_config_tx_testmode(tdev);
	else
		retcode = phy_config_tx_normalmode(tdev);

	if (retcode < 0)
		goto fail;

	phy_inverse_lanes(tdev->phy, tdev->lane_dev[LANEID_FOR_0]->id,
				tdev->tx_tests.flag, DEVICE_TX);
	do_tx_soft_reset(tdev);
	dfifo_reset(tdev);

fail:
	return retcode;
}

static int conf_rx_phy_gasket(struct transport_device *tdev)
{
	int retcode = 0;

	tdev->lane_dev[LANEID_FOR_0]->id =
					tdev->lane_id[LANEID_FOR_PRIMARY];

	if (tdev->lane_dev[LANEID_FOR_0]->id < 0) {
		retcode = -EINVAL;
		goto fail;
	}

	if (tdev->dev_state == TESTMODE)
		retcode = phy_config_rx_testmode(tdev);
	else
		phy_config_rx_normalmode(tdev);

	if (retcode < 0)
		goto fail;
	phy_inverse_lanes(tdev->phy, tdev->lane_dev[LANEID_FOR_0]->id,
				tdev->tx_tests.flag, DEVICE_RX);
	do_rx_soft_reset(tdev);
	dfifo_reset(tdev);

fail:
	return retcode;
}


static int conf_phy_gasket(struct transport_device *tdev)
{
	int retcode = 0;

	if (tdev->transport_type == DEVICE_TX)
		retcode = conf_tx_phy_gasket(tdev);
	else if (tdev->transport_type == DEVICE_RX)
		retcode = conf_rx_phy_gasket(tdev);
	else
		retcode = -ENODEV;

	return retcode;
}

void stop_link(struct transport_device *tdev, u32 timer_id)
{
	/*sysref_rose config*/
	if (tdev->transport_type == DEVICE_TX) {
		spin_lock(&tdev->sysref_lock);
		jesd_clear_bit(0,
			&tdev->tbg->tbgregs->tx_tmr[timer_id].alig_ctrl);
		jesd_clear_bit(SW_DMA_BIT17, &tdev->tx_regs->tx_transcontrol);
		spin_unlock(&tdev->sysref_lock);
	} else if (tdev->transport_type == DEVICE_RX) {
		spin_lock(&tdev->sysref_lock);
		jesd_clear_bit(SW_DMA_BIT17, &tdev->rx_regs->rx_transcontrol);
		jesd_clear_bit(0,
			&tdev->tbg->tbgregs->rx_tmr[timer_id].alig_ctrl);
		/*not sure why do we need to double clear this*/
		jesd_clear_bit(SW_DMA_BIT17, &tdev->rx_regs->rx_transcontrol);
		spin_unlock(&tdev->sysref_lock);
	}
}

void start_link(struct transport_device *tdev)
{
	/*sysref_rose config*/
	if (tdev->transport_type == DEVICE_TX) {
		spin_lock(&tdev->sysref_lock);

		tdev->sysref_rose = 0;
		/*SYSREF_ROSE bit 8 clear sysref*/
		jesd_set_bit(SYSREF_ISR_BIT8, &tdev->tx_regs->tx_irq_status);
		/*mask detection seen*/
		jesd_clear_bit(SYSREF_MASK_BIT20,
					&tdev->rx_regs->rx_transcontrol);
		/*disable interrupt and enable just for gag*/
		jesd_clear_bit(SYSREF_ISR_BIT8, &tdev->tx_regs->tx_irq_enable);
		jesd_set_bit(SYSREF_ISR_BIT8, &tdev->tx_regs->tx_irq_enable);

		wait_event_interruptible(tdev->isr_wait,
				tdev->sysref_rose == 1);
		tdev->sysref_rose = 0;
		spin_unlock(&tdev->sysref_lock);

	} else if (tdev->transport_type == DEVICE_RX) {
		spin_lock(&tdev->sysref_lock);
		tdev->sysref_rose = 0;
		/*SYSREF_ROSE bit 8 clear sysref*/
		jesd_set_bit(SYSREF_ISR_BIT8,
					&tdev->rx_regs->rx_irq_status);
		/*mask detection seen*/
		jesd_clear_bit(SYSREF_MASK_BIT20,
					&tdev->rx_regs->rx_transcontrol);
		/*disable interrupt and enable just for gag*/
		jesd_clear_bit(SYSREF_ISR_BIT8,
					&tdev->rx_regs->rx_irq_enable);
		jesd_set_bit(SYSREF_ISR_BIT8,
					&tdev->rx_regs->rx_irq_enable);

		wait_event_interruptible(tdev->isr_wait,
				tdev->sysref_rose == 1);
		tdev->sysref_rose = 0;
		spin_unlock(&tdev->sysref_lock);
	}
}

static int enable_dis(struct transport_device *tdev)
{
	int retcode = 0;

	spin_lock(&tdev->sysref_lock);
	jesd_clear_bit(RX_DIS_BIT7, &tdev->rx_regs->rx_ctrl_0);

	retcode = wait_event_interruptible_timeout(tdev->to_wait,
			(jtest_bit(1, &tdev->rx_regs->rx_diag_sel) == 1)
			&&
			(jtest_bit(0, &tdev->rx_regs->rx_diag_sel) == 0),
			tdev->evnt_jiffs);

	if (retcode < 0)
		raise_exception(tdev, START_TIMEOUT);

	spin_unlock(&tdev->sysref_lock);

	return retcode;
}

static int enable_tx(struct transport_device *tdev)
{
	int retcode = 0;
	unsigned int tx_en = 1;
	uint frm_ctrl;

	spin_lock(&tdev->sysref_lock);
	tx_en = tx_en << 1;
	frm_ctrl = (ioread32(&tdev->tx_regs->tx_frm_ctrl) | tx_en);
	iowrite32(frm_ctrl, &tdev->tx_regs->tx_frm_ctrl);

	retcode = wait_event_interruptible_timeout(tdev->to_wait,
			(jtest_bit(1, &tdev->tx_regs->tx_diag_sel) == 1)
			&&
			(jtest_bit(0, &tdev->tx_regs->tx_diag_sel) == 0),
			tdev->evnt_jiffs);

	if (retcode < 0)
		raise_exception(tdev, START_TIMEOUT);
	spin_unlock(&tdev->sysref_lock);

	return retcode;
}

static void raise_exception(struct transport_device *tdev,
			unsigned int event_typ)
{
		tdev->info.si_int = event_typ;
}


void lane_stats_update(struct transport_device *tdev,
						u32 stats_field,
						u32 lane_id)
{
	switch (stats_field) {
	case ILS_FAIL:
		tdev->lane_dev[lane_id]->l_stats->ils_error++;
		break;
	case CGS_FAIL:
		tdev->lane_dev[lane_id]->l_stats->cgs_error++;
		break;
	case FSF_FAIL:
		tdev->lane_dev[lane_id]->l_stats->frm_sync_error++;
		break;
	case UEX_K_FAIL:
		tdev->lane_dev[lane_id]->l_stats->uexk_threshold++;
		if (tdev->lane_dev[lane_id]->l_stats->uexk_threshold > 255) {
			tdev->rx_regs->rx_bad_parrity |= lane_id;
			 /*reset counter after we reach the limit*/
			tdev->rx_regs->rx_ux_char |= 0x20;
			tdev->lane_dev[lane_id]->l_stats->uexk_threshold = 0;
		}
		break;
	case NIT_FAIL:
		tdev->lane_dev[lane_id]->l_stats->nit_threshold++;
		if (tdev->lane_dev[lane_id]->l_stats->uexk_threshold > 255) {
			tdev->rx_regs->rx_bad_parrity |= lane_id;
			tdev->rx_regs->rx_nit |= 0x20;
			tdev->lane_dev[lane_id]->l_stats->uexk_threshold = 0;
		}
		break;
	case BAD_DIS_FAIL:
		tdev->lane_dev[lane_id]->l_stats->baddis_threshold++;
		if (tdev->lane_dev[lane_id]->l_stats->uexk_threshold > 255) {
			tdev->rx_regs->rx_bad_parrity |= lane_id;
			tdev->rx_regs->rx_bad_parrity |= 0x20;
			tdev->lane_dev[lane_id]->l_stats->uexk_threshold = 0;
		}
		break;
	default:
		break;
	}
}

u32 do_rx_cgs(struct transport_device *tdev)
{
	u32 cgs = CGS_NOT_CONFIGED;
	/*do we need to handle cgs*/
	if (jtest_bit(0, &tdev->rx_regs->rx_irq_ve_msk)) {
		if (jtest_bit(0, &tdev->rx_regs->rx_cgs)) {
			cgs = CGS_SUCCESS;
		} else{
			cgs = CGS_FAIL;
			lane_stats_update(tdev, CGS_FAIL, 0);
		}

		if (tdev->identifier == TRANPORT_0 &&
			 tdev->lanes_per_transport == 2) {

			if (jtest_bit(1, &tdev->rx_regs->rx_cgs))
				cgs = CGS_SUCCESS;
			else {
				cgs = CGS_FAIL;
				lane_stats_update(tdev, CGS_FAIL, 1);
			}
		}

		if (cgs == CGS_FAIL)
			raise_exception(tdev, cgs);
	}
	return cgs;
}

u32 do_rx_fsf(struct transport_device *tdev)
{
	u32 fsf = FSF_NOT_CONFIGED;
	/*do we need to handle fsf*/
	if (jtest_bit(1, &tdev->rx_regs->rx_irq_ve_msk)) {
		if (jtest_bit(0, &tdev->rx_regs->rx_fsf))
			fsf = FSF_SUCCESS;
		else{
			fsf = FSF_FAIL;
			lane_stats_update(tdev, FSF_FAIL, 0);
		}

		if (tdev->identifier == TRANPORT_0 &&
			tdev->lanes_per_transport == 2) {

			if (jtest_bit(1, &tdev->rx_regs->rx_fsf))
				fsf = FSF_SUCCESS;
			else{
				fsf = FSF_FAIL;
				lane_stats_update(tdev, FSF_FAIL, 1);
			}
		}

		if (fsf == FSF_FAIL)
			raise_exception(tdev, fsf);

	}
	return fsf;
}


void do_rx_csum(struct transport_device *tdev)
{
	u32 csum = CSUM_NOT_CONFIGED;
	/*do we need to handle cgs*/
	if (jtest_bit(1, &tdev->rx_regs->rx_irq_ve_msk)) {
		if (jtest_bit(0, &tdev->rx_regs->rx_g_csum))
				csum = CSUM_SUCCESS;
		else{
				csum = CSUM_FAIL;
				lane_stats_update(tdev, CSUM_FAIL, 0);
		}

		if (tdev->identifier == TRANPORT_0 &&
					tdev->lanes_per_transport == 2) {
			if (jtest_bit(1, &tdev->rx_regs->rx_g_csum))
				csum = CSUM_SUCCESS;
			else {
				csum = CSUM_FAIL;
				lane_stats_update(tdev, CSUM_FAIL, 1);
			}
		}

		if (csum == CSUM_FAIL)
			raise_exception(tdev, csum);

	}
}


void do_rx_ils(struct transport_device *tdev, u32 cgs, u32 fsf)
{
	u32 ils;
	/*do we need to handle cgs*/
	if (jtest_bit(3, &tdev->rx_regs->rx_irq_ve_msk)) {
		if (jtest_bit(0, &tdev->rx_regs->rx_ilsf))
			ils = ILS_SUCCESS;
		else{
			ils = ILS_FAIL;
			lane_stats_update(tdev, ILS_FAIL, 0);
		}

		if (tdev->identifier == TRANPORT_0 &&
				tdev->lanes_per_transport == 2) {

			if (jtest_bit(1, &tdev->rx_regs->rx_ilsf))
				ils = ILS_SUCCESS;
			else {
				ils = ILS_FAIL;
				lane_stats_update(tdev, ILS_FAIL, 1);
			}
		}

		if (ils == ILS_FAIL)
			raise_exception(tdev, ils);

		if (ils == ILS_SUCCESS &&
			fsf == FSF_SUCCESS &&
				cgs == CGS_SUCCESS) {
				tdev->dev_state = SYNCHRONIZED;
			}
	}
}


void reset_sync_interrupts(struct transport_device *tdev)
{
	tdev->rx_regs->rx_ilsf |= RESET_IRQ_VECTOR; /*reset ilsf interrupt.*/
	tdev->rx_regs->rx_cgs |= RESET_IRQ_VECTOR;/*reset cgs interrupt.*/
	tdev->rx_regs->rx_fsf |= RESET_IRQ_VECTOR;/*reset fsf interrupt.*/
	tdev->rx_regs->rx_g_csum |= RESET_IRQ_VECTOR; /*reset csum isr*/
}

void handle_sync_isr(struct transport_device *tdev)
{
	u32 cgs = do_rx_cgs(tdev);
	u32 fsf = do_rx_fsf(tdev);

	do_rx_ils(tdev, cgs, fsf);
}

void handle_maskvector(struct transport_device *tdev)
{
	/* this is for the sync user event*/
	handle_sync_isr(tdev);
	/*chk csum if we anre masked to support*/
	do_rx_csum(tdev);

	/*chk rx_skew_err test ild if we are masked to support*/
	if (jtest_bit(SKEW_ERR_BIT4, &tdev->rx_regs->rx_irq_ve_msk)) {
			/* yet to be supported*/
		handel_skew_err();
	}


	/*chk unex_k if we anre masked to support*/
	if (jtest_bit(UEX_K_BIT5, &tdev->rx_regs->rx_irq_ve_msk)) {
		if (tdev->rx_regs->rx_ux_char & 1)
				lane_stats_update(tdev, UEX_K_FAIL, 0);

		if (tdev->identifier == TRANPORT_0 &&
					tdev->lanes_per_transport == 2) {
			if (tdev->rx_regs->rx_ux_char & 2)
				lane_stats_update(tdev, UEX_K_FAIL, 1);
		}
	}

	/*chk nit if we are masked to support*/
	if (jtest_bit(NIT_BIT6, &tdev->rx_regs->rx_irq_ve_msk)) {
		if (tdev->rx_regs->rx_nit & 1)
			lane_stats_update(tdev, NIT_FAIL, 0);

		if (tdev->identifier == TRANPORT_0 &&
				tdev->lanes_per_transport == 2) {
			if (tdev->rx_regs->rx_nit & 2)
				lane_stats_update(tdev, NIT_FAIL, 1);
		}
	}

	if (jtest_bit(PARITY_ERR_BIT7, &tdev->rx_regs->rx_irq_ve_msk)) {
		if (tdev->rx_regs->rx_bad_parrity & 1)
				lane_stats_update(tdev, BAD_DIS_FAIL, 0);

		if (tdev->identifier == TRANPORT_0 &&
				tdev->lanes_per_transport == 2) {

			if (tdev->rx_regs->rx_bad_parrity & 2)
				lane_stats_update(tdev, BAD_DIS_FAIL, 1);
		}
	}
}

void handle_rx_irq(struct transport_device *tdev)
{
	u32 i;
	u32 isr_bit = 0;
	u32 status = tdev->rx_regs->rx_irq_status;
	u32 irq_enable = ioread32(&tdev->rx_regs->rx_irq_enable);
	for (i = 2; i < 13; i++) {
		isr_bit = (irq_enable & (1<<i));

		switch (isr_bit) {
		case 0x4:
			if (status & isr_bit) {
				raise_exception(tdev, PAC_OF_ERR);
				if (tdev->t_stats.ofpacker < 255)
					tdev->t_stats.ofpacker++;
				else
					tdev->t_stats.ofpacker = 0;
			}
			break;
		case 0x8:
			if (status & isr_bit) {
				raise_exception(tdev, PAC_UF_ERR);
				if (tdev->t_stats.ufpacker < 255)
					tdev->t_stats.ufpacker++;
				else
					tdev->t_stats.ufpacker = 0;
			}
			break;
		case 0x10:
			if (status & isr_bit) {
				raise_exception(tdev, RBUF_OF_ERR);
				if (tdev->t_stats.ofrcbuf < 255)
					tdev->t_stats.ofrcbuf++;
				else
					tdev->t_stats.ofrcbuf = 0;
			}
			break;
		case 0x20:
			if (status & isr_bit) {
				raise_exception(tdev, RBUF_UF_ERR);
				if (tdev->t_stats.ufrcbuf < 255)
					tdev->t_stats.ufrcbuf++;
				else
					tdev->t_stats.ufrcbuf = 0;
			}
			break;
		case 0x40:
			if (status & isr_bit) {
				raise_exception(tdev, SFIFO_ERR);
				if (tdev->t_stats.ufrcbuf < 255)
					tdev->t_stats.ofsfifo++;
				else
					tdev->t_stats.ofsfifo = 0;
			}
			break;
		case 0x100:
			if (status & isr_bit) {
				/*dfifo*/
				jesd_set_bit(DFIFO_BIT3,
					&tdev->rx_regs->rx_transcontrol);
				/*reset done*/
				jesd_clear_bit(DFIFO_BIT3,
					&tdev->rx_regs->rx_transcontrol);
				/*mask detection masked*/
				jesd_set_bit(SYSREF_MASK_BIT20,
					&tdev->rx_regs->rx_transcontrol);
				/*disable sysref rose*/
				jesd_clear_bit(SYSREF_ISR_BIT8,
					&tdev->tx_regs->tx_irq_enable);
				tdev->sysref_rose = 1;
				wake_up(&tdev->isr_wait);
				raise_exception(tdev, SYS_REF_ROSE);
			}
			break;
		case 0x400:
			if (status & isr_bit)
				raise_exception(tdev, PHY_DATA_LOST);
			break;
		default:
			break;
		}
	}
}

static int setclkdiv(struct transport_device *tdev)
{
	int line_rate, m, s, np, l;
	int fs, fc;
	int clkdiv;
	int ref;
	int retcode = 0;
	uint ctrl_reg;

	tdev->tbg = get_tbgen_device();

	if (tdev->tbg == NULL) {
		retcode = -EINVAL;
		goto fail;
	}

	ref = get_ref_clock(tdev->tbg);

	m = tdev->ils.conv_per_device_m;
	s = tdev->ils.s;
	np = tdev->ils.np;
	l = tdev->ils.lanes_per_converter_l;
	fs = tdev->ils.octect_per_frame_f;

	line_rate = (((m*s*np*fs)*(10/8))/l);
	fc = line_rate/10;

	if (tdev->transport_type == DEVICE_TX)
		clkdiv = (ref/fc);
	else if (tdev->transport_type == DEVICE_RX)
		clkdiv = ((ref*2)/fc);
	else{
		retcode = -EFAULT;
		goto fail;
	}

	/*config clkdiv*/
	if (clkdiv <= 3) {
		clkdiv = clkdiv & 0x03; /*mask for three bits*/
		clkdiv = clkdiv << 4; /*push to 4-5 bits*/
		if (tdev->transport_type == DEVICE_TX) {
			ctrl_reg = ioread32(&tdev->tx_regs->tx_transcontrol) |
					clkdiv;
			iowrite32(ctrl_reg, &tdev->tx_regs->tx_transcontrol);
		} else if (tdev->transport_type == DEVICE_RX) {
			ctrl_reg = ioread32(&tdev->rx_regs->rx_transcontrol) |
					clkdiv;
			iowrite32(ctrl_reg, &tdev->rx_regs->rx_transcontrol);
		} else
			retcode = -EFAULT;
	} else
		retcode = -EFAULT;

fail:
	return retcode;
}

static void write_reg(u32 *reg,
			unsigned int offset,
			unsigned int length,
			u32 *buf)
{
	int i;

	for (i = 0; i < offset; i++)
		reg++;

	for (i = 0; i < length; i++) {
		*reg = *buf;
		reg++;
		buf++;
	}
}

static void read_reg(u32 *reg,
			unsigned int offset,
			unsigned int length,
			u32 *buf)
{
	int i;

	for (i = 0; i < offset; i++)
		reg++;

	for (i = 0; i < length; i++) {
		*buf = *reg;
		reg++;
		buf++;
	}
}

static int jesd_conf_tests(struct transport_device *tdev)
{
	int retcode = 0;

	if (tdev->transport_type == DEVICE_TX) {
		if (tdev->tx_tests.frmr_tst.bipass_8b10 == 1)
			jesd_set_bit(BP_8b10_BIT0,
					&tdev->tx_regs->tx_frm_tst);
		else
			jesd_clear_bit(BP_8b10_BIT0,
					&tdev->tx_regs->tx_frm_tst);

		if (tdev->tx_tests.frmr_tst.reverse_8b10 == 1)
			jesd_set_bit(REVERSE_8b10_BIT1,
					&tdev->tx_regs->tx_frm_tst);
		else
			jesd_clear_bit(REVERSE_8b10_BIT1,
					&tdev->tx_regs->tx_frm_tst);

		if (tdev->tx_tests.frmr_tst.tpl == 1)
			jesd_set_bit(FRMR_TPL_BIT0,
					&tdev->tx_regs->tx_jedec_tst);
		else
			jesd_clear_bit(FRMR_TPL_BIT0,
					&tdev->tx_regs->tx_jedec_tst);

		if (tdev->tx_tests.frmr_tst.dll == 1)
			jesd_set_bit(FRMR_DLL_BIT1,
					&tdev->tx_regs->tx_jedec_tst);
		else
			jesd_clear_bit(FRMR_DLL_BIT1,
					&tdev->tx_regs->tx_jedec_tst);

		if (tdev->tx_tests.frmr_tst.s_rst == 1)
			jesd_set_bit(1, &tdev->tx_regs->tx_jedec_tst);


		if (tdev->tx_tests.frmr_tst.lane_ctrl == 1) {
			if (tdev->lanes_per_transport == 1) {
				tdev->tx_regs->tx_ln_ctrl =
					(tdev->tx_regs->tx_ln_ctrl & 0x1);
			} else if (tdev->lanes_per_transport == 2) {
				tdev->tx_regs->tx_ln_ctrl =
					(tdev->tx_regs->tx_ln_ctrl & 0x3);
			} else{
				retcode = -EFAULT;
				goto fail;
			}
		}
	} else if (tdev->transport_type == DEVICE_RX) {

		if (tdev->rx_tests.drfmr_tst.rep_data_tst == 1) {
			jesd_set_bit(REP_DAT_BIT5, &tdev->rx_regs->rx_ctrl_2);

			if (tdev->rx_tests.drfmr_tst.que_tst_err == 1)
				jesd_set_bit(QUEUE_TST_BIT4,
					&tdev->rx_regs->rx_ctrl_2);
			else
				jesd_clear_bit(QUEUE_TST_BIT4,
					&tdev->rx_regs->rx_ctrl_2);
		} else
			jesd_clear_bit(REP_DAT_BIT5,
					&tdev->rx_regs->rx_ctrl_2);

		if (tdev->rx_tests.drfmr_tst.ils_mode == 1)
			jesd_set_bit(ILS_MODE_BIT7,
					&tdev->rx_regs->rx_ctrl_2);
		else
			jesd_clear_bit(ILS_MODE_BIT7,
					&tdev->rx_regs->rx_ctrl_2);

	} else
		retcode = -EFAULT;
fail:
	return retcode;
}


static int conf_transport_delays(struct transport_device *tdev)
{
	struct transport_device *t = tdev;
	int retcode = 0;
	int delay;
	/* this shall be either tx or rx delay have the segregation here
	* itself config delay
	*/
	if (t->transport_type == DEVICE_TX)
		iowrite32(t->delay & 0x15, &t->tx_regs->tx_sync_delay);
	else if (t->transport_type == DEVICE_RX) {
		/*Set to 2 when K=5, F=4
		Set to 4 when K=10, F=2
		Set to 5 when K=6, F=4
		Set to 10 when K=12, F=2 as per RM*/
		if (t->ils.frame_per_mf_k == 5 &&
				t->ils.octect_per_frame_f == 4)
			delay = 2;
		else if (t->ils.frame_per_mf_k == 10 &&
				t->ils.octect_per_frame_f == 2)
			delay = 4;
		else if (t->ils.frame_per_mf_k == 6 &&
					t->ils.octect_per_frame_f == 4)
			delay = 5;
		else if (t->ils.frame_per_mf_k == 12 &&
					t->ils.octect_per_frame_f == 2)
			delay = 10;
		else
			retcode = -EFAULT;

		if (retcode < 0)
			return retcode;
		else
			iowrite32(delay, &t->rx_regs->rx_rcv_delay);
	} else
		retcode = -EINVAL;
	return retcode;
}

static int enable_transports(struct transport_device *tdev)
{
	int retcode = 0;

	if (tdev->transport_type == DEVICE_TX) {
		/*Always set to 3*/
		iowrite32(3, &tdev->tx_regs->tx_m_en);
		iowrite32(0, &tdev->tx_regs->tx_ln_ctrl);

		if (tdev->lanes_per_transport == 1)
			iowrite32(1, &tdev->tx_regs->tx_l_en);
		else if (tdev->lanes_per_transport == 2)
			iowrite32(3, &tdev->tx_regs->tx_l_en);
		else
			retcode = -EFAULT;

		if (retcode == 0) {
			tdev->evnt_jiffs = 2;
			start_link(tdev);
			retcode = enable_tx(tdev);
		}

	} else if (tdev->transport_type == DEVICE_RX) {

		if (tdev->lanes_per_transport == 1)
			iowrite32(0x01, &tdev->rx_regs->rx_lane_en);
		else if (tdev->lanes_per_transport == 2)
			iowrite32(0x03, &tdev->rx_regs->rx_lane_en);
		else{
			iowrite32(0x00, &tdev->rx_regs->rx_lane_en);
			retcode = -EFAULT;
		}

		if (retcode == 0) {
			tdev->evnt_jiffs = 2;
			start_link(tdev);
			/* RX_DIS to enable rx transport*/
			retcode = enable_dis(tdev);
		}
	}

	return retcode;
}

int jesd_start_transport(struct transport_device *tdev)
{
	int retcode = 0;
/* thus is to be implemented if needed*/
/*	set_serdes();*/

	conf_phy_gasket(tdev);
/* thus is to be implemented and integrated*/
	retcode = setclkdiv(tdev);
	if (retcode < 0)
		goto fail;

	retcode = conf_transport_delays(tdev);
	if (retcode < 0)
		goto fail;

	retcode = enable_transports(tdev);
fail:
	return retcode;
}
EXPORT_SYMBOL(jesd_start_transport);


int jesd_restart_transport(struct transport_device *tdev,
				u32 timer_id)
{
	int retcode = 0;

	stop_link(tdev, timer_id);
	start_link(tdev);
	if (tdev->transport_type == DEVICE_RX) {
		/* RX_DIS to enable rx transport*/
		retcode = enable_dis(tdev);
	} else if (tdev->transport_type == DEVICE_TX) {
		/* tx enable*/
		retcode = enable_tx(tdev);
	}
	return retcode;
}
EXPORT_SYMBOL(jesd_restart_transport);

static int force_sync(struct transport_device *tdev)
{
	int retcode = 0;

	if (tdev->transport_type == DEVICE_RX)
		jesd_set_bit(RX_DEV_BIT2, &tdev->rx_regs->rx_ctrl_0);
	else
		retcode = -EINVAL;

	return retcode;
}


static int jesd_set_ilsa_len(struct transport_device *tdev, int kf_ilas)
{
	int retcode = 0;
	uint ils_len;

	tdev->ilsa_len = (kf_ilas & 0xff);

	if (tdev->transport_type == DEVICE_TX) {
		ils_len = (ioread32(&tdev->tx_regs->tx_ilas_len) |
							(4*(kf_ilas+1)));
		iowrite32(ils_len, &tdev->tx_regs->tx_ilas_len);
	} else if (tdev->transport_type == DEVICE_RX) {
		ils_len = (ioread32(&tdev->rx_regs->rx_ilas_len) |
							(4*(kf_ilas+1)));
		iowrite32(ils_len, &tdev->rx_regs->rx_ilas_len);
	} else
		retcode = -EINVAL;

	return retcode;
}

static int jesd_set_transport_pram(struct transport_device *tdev,
					struct conf_tr __user *trans_p)
{
	int retcode = 0;
	int i = 0;

	tdev->sampling_rate = trans_p->sampling_rate;
	tdev->delay = trans_p->delay;
	tdev->tranport_flags = trans_p->tran_flg;
	/*number of lanes attached to the transport*/
	tdev->lanes_per_transport = trans_p->lanes;
	tdev->mode = NORMAL_MODE;

	/*do not allow more transport*/
	if (tdev->jesd_parent->lanes_grabed <= tdev->jesd_parent->max_lanes) {
		tdev->jesd_parent->lanes_grabed =
					tdev->jesd_parent->lanes_grabed +
					tdev->lanes_per_transport;

	if (tdev->identifier == TRANPORT_1) {
		if (tdev->jesd_parent->trans_device[0]->lanes_per_transport
			== 2)
				retcode = -EINVAL;
	}

	} else
		retcode = -EINVAL;


	/*create lanes*/
	if ((tdev->lanes_per_transport > 0) &&
		(tdev->lanes_per_transport <= MAX_LANES)) {
		/*we can have 2 lanes for transport 0 only and for transport 1
		*just 1 lane
		*/
		if (tdev->identifier == 1)
			tdev->lanes_per_transport = 1;

		for (i = 0; i < tdev->lanes_per_transport; i++) {
			tdev->lane_dev[i] =
				kzalloc(sizeof(struct lane_device),
					GFP_KERNEL);
			/*assing the back pointer to the transport we belong*/
			if (tdev->lane_dev[i] != NULL)
				tdev->lane_dev[i]->trans_dev_params = tdev;
		}


		if (tdev->lanes_per_transport == 1)
			tdev->lane_dev[1] = NULL;


		if (tdev->transport_type == DEVICE_TX) {
			/* enable clock in the control reg of transport before
			* configure the transport or use it
			*/
			jesd_set_bit(ENABLE_CLK_BIT0,
					&tdev->tx_regs->tx_transcontrol);

			/* test, set/clear the scr ctrl reg*/
			/* conf ->SCR_CTRL_L0 */
			if (jtest_bit(SCR_CTRL_L0_VALID,
						&tdev->tranport_flags))
				jesd_set_bit(SCR_CTRL_L0_BIT0,
						&tdev->tx_regs->tx_scr_ctrl);
			else
				jesd_clear_bit(SCR_CTRL_L0_BIT0,
						&tdev->tx_regs->tx_scr_ctrl);

			/* conf ->SCR_CTRL_L1 */
			if (jtest_bit(SCR_CTRL_L1_VALID,
					&tdev->tranport_flags))
				jesd_set_bit(SCR_CTRL_L1_BIT1,
					&tdev->tx_regs->tx_scr_ctrl);
			else
				jesd_clear_bit(SCR_CTRL_L1_BIT1,
					&tdev->tx_regs->tx_scr_ctrl);

			/* test, set/clear framer ctrl reg
			* receive and transmit support lane
			* synchronization
			*/
			if (jtest_bit(L2SIDES_VALID,
					&tdev->tranport_flags))
				jesd_set_bit(L2SIDES_BIT0,
					&tdev->tx_regs->tx_frm_ctrl);
			else
				jesd_clear_bit(L2SIDES_BIT0,
					&tdev->tx_regs->tx_frm_ctrl);

			/*Bypass Initial Lane Alignment*/
			if (jtest_bit(BYP_ILAS_VALID, &tdev->tranport_flags))
				jesd_set_bit(BYP_ILAS_BIT2,
					&tdev->tx_regs->tx_frm_ctrl);
			else
				jesd_clear_bit(BYP_ILAS_BIT2,
					&tdev->tx_regs->tx_frm_ctrl);

			/*Bypass Alignment Character Generation*/
			if (jtest_bit(BYP_ACG_VALID,
					&tdev->tranport_flags))
				jesd_set_bit(BYP_ACG_BIT3,
						&tdev->tx_regs->tx_frm_ctrl);
			else
				jesd_clear_bit(BYP_ACG_BIT3,
						&tdev->tx_regs->tx_frm_ctrl);

			/* test, set/clear transport control register
			* idle select
			*/
			if (jtest_bit(IDLE_SELECT_VALID,
					&tdev->tranport_flags))
				jesd_set_bit(IDLE_SELECT_BIT1,
					&tdev->tx_regs->tx_transcontrol);
			else
				jesd_clear_bit(IDLE_SELECT_BIT1,
					&tdev->tx_regs->tx_transcontrol);

			/*Write Circular Buffer Overflow Protection*/
			if (jtest_bit(WCBUF_PROTECT_VALID,
						&tdev->tranport_flags))
				jesd_set_bit(WC_RC_PROTECT_BIT18,
					&tdev->tx_regs->tx_transcontrol);
			else
				jesd_clear_bit(WC_RC_PROTECT_BIT18,
					&tdev->tx_regs->tx_transcontrol);

			/*PHY Packer Most-Significant Octet First
			--> 0 The first 8b10b symbol received from the
			framer is put in the least-significant octet of
			the transport\92s 20-bit output
			--> 1 The first 8b10b symbol received from the
			framer is put in the most-significant octet of
			the transport\92s output
			*/
			if (jtest_bit(MS_OCT_FIRST_VALID,
						&tdev->tranport_flags))
				jesd_set_bit(OCT_FIRST_BIT9,
					&tdev->tx_regs->tx_transcontrol);
			else
				jesd_clear_bit(OCT_FIRST_BIT9,
					&tdev->tx_regs->tx_transcontrol);

			/*IQ swap*/
			if (jtest_bit(IQ_SWAP_VALID,
					&tdev->tranport_flags))
				jesd_set_bit(IQ_SWAP_BIT7,
					&tdev->tx_regs->tx_transcontrol);
			else
				jesd_clear_bit(IQ_SWAP_BIT7,
					&tdev->tx_regs->tx_transcontrol);

		} else if (tdev->transport_type == DEVICE_RX) {
			/* enable clock in the control reg of transport before
			* Configure the transport or use it*/
			jesd_set_bit(ENABLE_CLK_BIT0,
				&tdev->rx_regs->rx_transcontrol);
			/*IQ swap*/
			if (jtest_bit(IQ_SWAP_VALID, &tdev->tranport_flags))
				jesd_set_bit(IQ_SWAP_BIT7,
					&tdev->rx_regs->rx_transcontrol);
			else
				jesd_clear_bit(IQ_SWAP_BIT7,
					&tdev->rx_regs->rx_transcontrol);

			/*PHY Order: Most-Significant Octet First
			* --> 0 The first octet received is in the
			*	least-significant octet of the 20-bit PHY input
			* --> 1 The first octet received is In the
			*	most-significant octet of the 20-bit PHY input
			*/
			if (jtest_bit(PHY_OCT_FIRST_VALID,
						&tdev->tranport_flags))
				jesd_set_bit(OCT_FIRST_BIT9,
					&tdev->rx_regs->rx_transcontrol);
			else
				jesd_clear_bit(OCT_FIRST_BIT9,
					&tdev->rx_regs->rx_transcontrol);

			/*Read Circular Buffer Overflow Protection*/
			if (jtest_bit(RCBUF_PROTECT_VALID,
						&tdev->tranport_flags))
				jesd_set_bit(WC_RC_PROTECT_BIT18,
					&tdev->rx_regs->rx_transcontrol);
			else
				jesd_clear_bit(WC_RC_PROTECT_BIT18,
					&tdev->rx_regs->rx_transcontrol);

			/*idle select*/
			if (jtest_bit(IDLE_SELECT_VALID,
					&tdev->tranport_flags))
				jesd_set_bit(IDLE_SELECT_BIT1,
					&tdev->rx_regs->rx_transcontrol);
			else
				jesd_clear_bit(IDLE_SELECT_BIT1,
					&tdev->rx_regs->rx_transcontrol);

			/*PHY Order: Most-Significant Bit First
			*--> 0 The first bit received is in the
			*least-significant bit of a given octet from the PHY
			*--> 1 The first bit received is in the
			*most-significant bit of a given octet from the PHY
			*/
			if (jtest_bit(PHY_BIT_FIRST_VALID,
						&tdev->tranport_flags))
				jesd_set_bit(PHY_FIRT_BIT8,
					&tdev->rx_regs->rx_transcontrol);
			else
				jesd_clear_bit(PHY_FIRT_BIT8,
					&tdev->rx_regs->rx_transcontrol);

			/*Checksum Calculation Selection
			*--> 0 Checksums are calculated by summing the
			* individual fields in the link configuration table
			* (JESD204 compliant)
			*--> 1 Checksums are calculated by summing the registers
			* containing the packed link configuration fields
			*/
			if (jtest_bit(TN_CHK_CSUM_VALID,
					&tdev->tranport_flags))
				jesd_set_bit(TN_CHK_CSUM_BIT12,
					&tdev->rx_regs->rx_transcontrol);
			else
				jesd_clear_bit(TN_CHK_CSUM_BIT12,
					&tdev->rx_regs->rx_transcontrol);
		} else
			retcode = -EINVAL;

	} else
			retcode = -EINVAL;
	return retcode;
}


static int config_fpmf(struct transport_device *tdev)
{
	int retcode = 0;
	struct transport_device *t = tdev;
	/*conf K get the ref clock first*/
	u32 ref_clk;

	tdev->tbg = (struct tbgen_dev *)get_tbgen_device();

	if (tdev->tbg == NULL) {
		retcode = -EINVAL;
		goto fail;
	}

	ref_clk =  get_ref_clock(tdev->tbg);

	if (ref_clk == 614) {
		/* As per RM For ref_clk=614.4MHz:
		* Set to 4 when the link uses 1 lane (L=1)
		* Set to 9 when the link uses 2 lanes (L=2)
		*/
		if (t->ils.lanes_per_converter_l == 1) {
			if (t->transport_type == DEVICE_TX)
				iowrite32(4, &tdev->tx_regs->tx_fpmf);
			else if (t->transport_type == DEVICE_RX)
				iowrite32(4, &tdev->rx_regs->rx_fpmf);
			else
				retcode = -EINVAL;
		} else if (tdev->ils.lanes_per_converter_l == 2) {
			if (t->transport_type == DEVICE_TX)
				iowrite32(9, &tdev->tx_regs->tx_fpmf);
			else if (t->transport_type == DEVICE_RX)
				iowrite32(9, &tdev->rx_regs->rx_fpmf);
			else
				retcode = -EINVAL;
		} else
			retcode = -EINVAL;
	} else if (ref_clk == 983) {
		/* For ref_clk=983.04MHz:
		* Set to 5 when the link uses 1 lane (L=1)
		* Set to 11 when the link uses 2 lanes (L=2)
		*/
		if (tdev->ils.lanes_per_converter_l == 1) {
			if (t->transport_type == DEVICE_TX)
				iowrite32(5, &tdev->tx_regs->tx_fpmf);
			else if (t->transport_type == DEVICE_RX)
				iowrite32(5, &tdev->rx_regs->rx_fpmf);
			else
				retcode = -EINVAL;
		} else if (tdev->ils.lanes_per_converter_l == 2) {

			if (t->transport_type == DEVICE_TX)
				iowrite32(11, &tdev->tx_regs->tx_fpmf);
			else if (t->transport_type == DEVICE_RX)
				iowrite32(11, &tdev->rx_regs->rx_fpmf);
			else
				retcode = -EINVAL;
		} else
			retcode = -EINVAL;
	} else{
		retcode = -EINVAL;
	}
fail:
	return retcode;
}

static int set_did_bid(struct transport_device *tdev)
{
	int retcode = 0;
	uint id;
	struct transport_device *t = tdev;


	if (t->transport_type == DEVICE_TX) {
		/* fill register set */
		id = ioread32(&t->tx_regs->tx_did) | (t->ils.device_id & 0xff);
		iowrite32(id, &t->tx_regs->tx_did);

		id = ioread32(&t->tx_regs->tx_bid) | (t->ils.bank_id & 0xf);
		iowrite32(id, &t->tx_regs->tx_bid);
	} else if (t->transport_type == DEVICE_RX) {
		/* fill register set */
		id = ioread32(&t->rx_regs->rx_did) | (t->ils.device_id & 0xff);
		iowrite32(id, &t->rx_regs->rx_did);

		id = (ioread32(&t->rx_regs->rx_bid) | (t->ils.bank_id & 0xf));
		iowrite32(id, &t->rx_regs->rx_bid);
	} else
		retcode = -EINVAL;
	return retcode;
}

static int set_lane_id(struct transport_device *tdev)
{
	int retcode = 0;
	uint lid;
	struct transport_device *t = tdev;

	if (t->transport_type == DEVICE_TX) {
		/*lane 0 id*/
		lid = (ioread32(&tdev->tx_regs->tx_lid_0) |
					(tdev->ils.lane_0_id & 0x1f));
		iowrite32(lid, &tdev->tx_regs->tx_lid_0);
		/*lane 1 id*/
		if (tdev->lanes_per_transport == 2)
			lid = (ioread32(&tdev->tx_regs->tx_lid_1) |
					(tdev->ils.lane_1_id & 0x1f));
			iowrite32(lid, &tdev->tx_regs->tx_lid_1);
	} else
		retcode = -EINVAL;

	return retcode;
}

static int config_scr(struct transport_device *tdev)
{
	int retcode = 0;
	struct transport_device *t = tdev;

	if (t->ils.scrambling_scr == 0) {
		if (t->transport_type == DEVICE_TX)
			jesd_clear_bit(SCRAMBL_BIT7, &t->tx_regs->tx_scr);
		else if (t->transport_type == DEVICE_RX)
			jesd_clear_bit(SCRAMBL_BIT7, &t->rx_regs->rx_scr);
		else
			retcode = -EINVAL;
	} else{
		if (t->transport_type == DEVICE_TX)
			jesd_set_bit(SCRAMBL_BIT7, &t->tx_regs->tx_scr);
		else if (t->transport_type == DEVICE_RX)
			jesd_set_bit(SCRAMBL_BIT7, &t->rx_regs->rx_scr);
		else
			retcode = -EINVAL;
	}
	return retcode;
}


static int config_l(struct transport_device *tdev)
{
	int retcode = 0;
	u8 L;
	uint scr;
	struct transport_device *t = tdev;
/*config L Number of lanes per converter device minus 1 as per RM*/
	if (t->ils.lanes_per_converter_l > 0) {
		L = tdev->ils.lanes_per_converter_l - 1;
		if (t->transport_type == DEVICE_TX) {
			scr = (ioread32(&tdev->tx_regs->tx_scr) | (L));
			iowrite32(scr, &tdev->tx_regs->tx_scr);
		} else if (t->transport_type == DEVICE_RX) {
			scr = (ioread32(&tdev->rx_regs->rx_scr) | (L));
			iowrite32(scr, &tdev->rx_regs->rx_scr);
		} else
			retcode = -EINVAL;
	} else
		retcode = -EFAULT;
	return retcode;
}

static int config_opf(struct transport_device *tdev)
{
	int retcode;
	struct transport_device *t = tdev;
	u8 opf = tdev->ils.octect_per_frame_f - 1;
	if (t->transport_type == DEVICE_TX)
		iowrite32(opf, &tdev->tx_regs->tx_opf);
	else if (t->transport_type == DEVICE_RX)
		iowrite32(opf, &tdev->rx_regs->rx_opf);
	else
		retcode = -EINVAL;
	return	retcode;
}

static int config_cs(struct transport_device *tdev)
{
	int retcode;
	u8 cs;
	uint cbps;
	struct transport_device *t = tdev;

	cs = t->ils.cs;
	cs = cs & 3;
	cs = cs << 6;

	if (t->transport_type == DEVICE_TX) {
		cbps = ioread32(&t->tx_regs->tx_cbps) | (cs);
		iowrite32(cbps, &t->tx_regs->tx_cbps);
	} else if (t->transport_type == DEVICE_RX) {
		cbps = ioread32(&t->rx_regs->rx_cbps) | (cs);
		iowrite32(cbps, &t->rx_regs->rx_cbps);
	} else
		retcode = -EINVAL;

	return retcode;
}

static int config_n(struct transport_device *t)
{

	int retcode;
	uint n_cbps;

	if (t->transport_type == DEVICE_TX) {
		n_cbps = (ioread32(&t->tx_regs->tx_cbps) | (t->ils.n & 0x1f));
		iowrite32(n_cbps, &t->tx_regs->tx_cbps);
	} else if (t->transport_type == DEVICE_RX) {
		n_cbps = (ioread32(&t->rx_regs->rx_cbps) | (t->ils.n & 0x1f));
		iowrite32(n_cbps, &t->rx_regs->rx_cbps);
	} else
		retcode = -EINVAL;

	return retcode;
}


static int mask_subclass(struct transport_device *t)
{
	int retcode = 0;
	u8 sub_class = t->ils.subclass_ver;

	sub_class = sub_class & 7;
	sub_class = sub_class << 5;

	if (t->transport_type == DEVICE_TX)
		iowrite32(ioread32(&t->tx_regs->tx_nbcw) | sub_class,
							&t->tx_regs->tx_nbcw);
	else if (t->transport_type == DEVICE_RX)
		iowrite32(ioread32(&t->rx_regs->rx_nbcw) | sub_class,
							&t->rx_regs->rx_nbcw);
	else
		retcode = -EINVAL;

	return retcode;
}


static int assign_jesdver(struct transport_device *tdev)
{

	int retcode = 0;
	struct transport_device *t = tdev;
	u8 jesdv = t->ils.jesd_ver;

	jesdv = jesdv & 7;
	jesdv = jesdv << 5;

	if (t->transport_type == DEVICE_TX)
		iowrite32(ioread32(&t->tx_regs->tx_spcp) | jesdv,
							&t->tx_regs->tx_spcp);
	else if (t->transport_type == DEVICE_RX)
		iowrite32(ioread32(&t->rx_regs->rx_spcp) | jesdv,
							&t->rx_regs->rx_spcp);
	else
		retcode = -EINVAL;

	return retcode;
}


static int jesd_set_ils_pram(struct transport_device *tdevice)
{
	int retcode = 0;
	uint csum;
	struct transport_device *tdev = tdevice;

	retcode = set_did_bid(tdev);
	if (retcode < 0)
		goto fail;

	retcode = set_lane_id(tdev);
	if (retcode < 0)
		goto fail;

	retcode = config_scr(tdev);
	if (retcode < 0)
		goto fail;

	retcode = config_l(tdev);
	if (retcode < 0)
		goto fail;

	retcode = config_opf(tdev);
	if (retcode < 0)
		goto fail;

	retcode = config_fpmf(tdev);
	if (retcode < 0)
		goto fail;

	if (tdev->transport_type == DEVICE_TX)
		/* config M Number of converters minus 1
		*Always set to 1 as per RM tdev->ils.conv_per_device_m
		*/
		iowrite32(1, &tdev->tx_regs->tx_cpd);
	else if (tdev->transport_type == DEVICE_RX)
		iowrite32(1, &tdev->rx_regs->rx_cpd);
	else{
		retcode = -EINVAL;
		goto fail;
	}

	retcode = config_cs(tdev);
	if (retcode < 0)
		goto fail;

	retcode = config_n(tdev);
	if (retcode < 0)
		goto fail;

	retcode = mask_subclass(tdev);
	if (retcode < 0)
		goto fail;

	retcode = assign_jesdver(tdev);
	if (retcode < 0)
		goto fail;

	if (tdev->transport_type == DEVICE_TX) {
		/*config NP*/
		iowrite32(ioread32(&tdev->tx_regs->tx_nbcw) |
					(tdev->ils.np & 0x1f),
					&tdev->tx_regs->tx_nbcw);
		/*config S*/
		iowrite32(ioread32(&tdev->tx_regs->tx_spcp) |
					(tdev->ils.s & 0x1f),
					&tdev->tx_regs->tx_spcp);
		/*config H */
		iowrite32(ioread32(&tdev->tx_regs->tx_hdf) |
					((tdev->ils.hd & 1) << 7),
					&tdev->tx_regs->tx_hdf);
		/*config CF*/
		iowrite32(ioread32(&tdev->tx_regs->tx_spcp) |
					(tdev->ils.cf & 0x1f),
					&tdev->tx_regs->tx_spcp);
		/*csum lane 0*/
		csum = tdev->ils.csum_lane_0 & 0xFF;
		csum = ioread32(&tdev->tx_regs->tx_csum_0) | csum;

		iowrite32(csum, &tdev->tx_regs->tx_csum_0);

		if (tdev->lanes_per_transport == 2)
			/*csum lane 1*/
			csum = tdev->ils.csum_lane_1 & 0xFF;
			csum = ioread32(&tdev->tx_regs->tx_csum_1);
			iowrite32(csum, &tdev->tx_regs->tx_csum_1);

	} else if (tdev->transport_type == DEVICE_TX) {
		/*config NP*/
		iowrite32(ioread32(&tdev->rx_regs->rx_nbcw) |
					(tdev->ils.np & 0x1f),
					&tdev->rx_regs->rx_nbcw);
		/*config S*/
		iowrite32(ioread32(&tdev->rx_regs->rx_spcp) |
					(tdev->ils.s & 0x1f),
					&tdev->rx_regs->rx_spcp);
		/*config H */
		iowrite32(ioread32(&tdev->rx_regs->rx_hdf) |
					((tdev->ils.hd & 1) << 7),
					&tdev->rx_regs->rx_hdf);
		/*config CF*/
		iowrite32(ioread32(&tdev->rx_regs->rx_spcp) |
					(tdev->ils.cf & 0x1f),
					&tdev->rx_regs->rx_spcp);
		/*csum lane 0*/
		csum = tdev->ils.csum_lane_0 & 0xFF;
		csum = ioread32(&tdev->rx_regs->rx_csum_0) | csum;
		iowrite32(csum, &tdev->rx_regs->rx_csum_0);
	} else
		retcode = -EFAULT;
fail:
	return retcode;
}

static int shutdown(struct transport_device *tdev)
{
	int retcode = 0;
	/*disable dma and transport*/
	if (tdev->transport_type == DEVICE_TX) {
		jesd_clear_bit(DMA_DIS_BIT17, &tdev->tx_regs->tx_transcontrol);
		jesd_clear_bit(TX_FRM_BIT1, &tdev->tx_regs->tx_frm_ctrl);
	} else if (tdev->transport_type == DEVICE_RX) {
		jesd_clear_bit(DMA_DIS_BIT17, &tdev->rx_regs->rx_transcontrol);
		jesd_clear_bit(RX_CTRL_BIT7, &tdev->rx_regs->rx_ctrl_0);
	} else{
		retcode = -EFAULT;
		return retcode;
	}
	tdev->dev_state = SUSPENDED;
	return retcode;
}

void write_multi_regs(struct transport_device *tdev,
			u32 offset,
			u32 len,
			u32 value)
{
	if (tdev->transport_type == DEVICE_TX)
		write_reg(&tdev->tx_regs->tx_did, offset,
						len, &value);
	else if (tdev->transport_type == DEVICE_RX)
		write_reg(&tdev->rx_regs->rx_rdid, offset,
						len, &value);
}

static long jesd204_ioctl(struct file *pfile, unsigned int cmd,
						unsigned long arg)
{
	int retcode = -ENOSYS;
	u32 count = 0;
	void __user *argp = (void __user *)arg;
	struct transport_device *transdev = NULL;
	struct conf_tr *trans_p;
	struct transport_device_info trans_dev_info;
	struct jesd_reg_read_buf *regcnf;
	struct jesd_reg_write_buf write_reg;
	struct isrconf isrcnf;
	struct tarns_dev_stats gstats;
	struct tbgen_params tbgen_tmr_params;
	struct ils_params *ilsparams;
	struct auto_sync_params *sync_params;
	struct jesd_reg_write_buf *p;
	enum jesd_state *state;

	transdev = pfile->private_data;

	/*processes ioctl*/
	switch (cmd) {
	case JESD_SET_TRANS_PARAMS:
		trans_p = (struct conf_tr __user *) arg;
		retcode = jesd_set_transport_pram(transdev, trans_p);
		transdev->dev_state = CONFIGURED;
		break;
	case JESD_SET_LANE_PARAMS:
		if (copy_from_user(&transdev->ils,
				(struct ils_params *)arg,
				 sizeof(struct ils_params))) {
			retcode = -EFAULT;
			break;
		}
		retcode = jesd_set_ils_pram(transdev);
		break;
	case JESD_SET_ILS_LENGTH:
		jesd_set_ilsa_len(transdev, (int)arg);
		break;
	case JESD_SET_INTERRUPTMASK:

		if (copy_from_user(&isrcnf,
			(struct isrconf *)arg,
			sizeof(struct isrconf))) {
			retcode = -EFAULT;
			break;
		}

		if (transdev->transport_type == DEVICE_TX) {
			transdev->irq_tx.irq_txconf =
						isrcnf.irq_tx.irq_txconf;
			iowrite32(transdev->irq_tx.irq_txconf & 0x338,
					&transdev->tx_regs->tx_irq_enable);
		} else if (transdev->transport_type == DEVICE_RX) {
			transdev->irq_rx.irq_m.irq_en =
						isrcnf.irq_m.irq_en;
			transdev->irq_rx.irq_vm.irq_ven =
						isrcnf.irq_vm.irq_ven;

			iowrite32(transdev->irq_rx.irq_m.irq_en & 0x357c,
					&transdev->rx_regs->rx_irq_enable);
			iowrite32(transdev->irq_rx.irq_vm.irq_ven & 0xff,
					&transdev->rx_regs->rx_irq_ve_msk);
		} else
			retcode = -EFAULT;

		break;
	case JESD_GET_INTERRUPTMASK:
		isrcnf.irq_tx.irq_txconf = transdev->irq_tx.irq_txconf;
		isrcnf.irq_m.irq_en = transdev->irq_rx.irq_m.irq_en;
		isrcnf.irq_vm.irq_ven = transdev->irq_rx.irq_vm.irq_ven;

		if (transdev->transport_type == DEVICE_TX) {
			isrcnf.irq_m.irq_en = 0;
			isrcnf.irq_vm.irq_ven = 0;
		} else if (transdev->transport_type == DEVICE_RX) {
			isrcnf.irq_tx.irq_txconf = 0;
		} else
			retcode = -EFAULT;
			goto fail;


		if (copy_to_user((u32 *)argp,
				&isrcnf, sizeof(struct isrconf)))
					retcode = -EFAULT;
		break;
	case JESD_DEIVCE_START:
		retcode = jesd_start_transport(transdev);
		if (retcode < 0)
			transdev->dev_state = SYNC_FAILED;
		else
			transdev->dev_state = OPERATIONAL;
		break;
	case JESD_DEV_RESTART:
		if (copy_from_user(&tbgen_tmr_params,
			(struct tbgen_params *)arg,
			sizeof(struct tbgen_params))) {
			retcode = -EFAULT;
			break;
		}
		jesd_restart_transport(transdev,
					tbgen_tmr_params.timer_id);
		if (retcode < 0)
			transdev->dev_state = SYNC_FAILED;
		else
			transdev->dev_state = OPERATIONAL;
		break;
	case JESD_TX_TEST_MODE:
		if (transdev->transport_type == DEVICE_TX) {
			if (copy_from_user(&transdev->tx_tests,
					(struct conf_tx_tests *)arg,
					sizeof(struct conf_tx_tests))) {
				retcode = -EFAULT;
				goto fail;
			}
		} else{
			retcode = -EFAULT;
			goto fail;
		}
		jesd_conf_tests(transdev);
		transdev->dev_state = TESTMODE;
		break;
	case JESD_RX_TEST_MODE:
		if (transdev->transport_type == DEVICE_TX) {
			if (copy_from_user(&transdev->rx_tests,
					(struct conf_rx_tests *)arg,
					sizeof(struct conf_rx_tests))) {
				retcode = -EFAULT;
				goto fail;
			}
		} else{
			retcode = -EFAULT;
				goto fail;
		}
		jesd_conf_tests(transdev);
		transdev->dev_state = TESTMODE;
		break;
	case JESD_FORCE_SYNC:
		force_sync(transdev);
		break;
	case JESD_GET_STATS:
		memcpy(&gstats.tstats, &transdev->t_stats,
					sizeof(struct transport_stats));
		memcpy(&gstats.l0stats, &transdev->lane_dev[0]->l_stats,
					sizeof(struct lane_stats));
		/* only for trans 0 we have two lanes, by default clean this*/
		memset(&gstats.l1stats, 0, sizeof(struct lane_stats));

		if (transdev->identifier == TRANPORT_0 &&
			 transdev->lanes_per_transport == 2) {
			memcpy(&gstats.l1stats,
				&transdev->lane_dev[1]->l_stats,
				sizeof(struct lane_stats));
		}

		if (copy_to_user((struct tarns_dev_stats *)arg,
					&gstats,
					sizeof(struct tarns_dev_stats)))
			retcode = -EFAULT;
		break;
	case JESD_CLEAR_STATS:
		memset(&transdev->t_stats, 0, sizeof(struct lane_stats));
		memset(&transdev->lane_dev[0]->l_stats, 0,
			sizeof(struct lane_stats));
		memset(&transdev->lane_dev[1]->l_stats, 0,
			sizeof(struct lane_stats));
		break;
	case JESD_WRITE_REG:
		p = (struct jesd_reg_write_buf *) arg;
		count = p->count;

		write_reg.regs = kzalloc((sizeof(struct jesd_wreg) * count),
					GFP_KERNEL);
		if (copy_from_user(&write_reg,
				(struct jesd_reg_write_buf *)argp,
				sizeof(struct jesd_reg_write_buf)*count)) {
				retcode = -EFAULT;
				break;
		}

		while (count) {
			write_multi_regs(transdev,
					write_reg.regs->offset,
					1,
					write_reg.regs->value);
			write_reg.regs++;
			count--;
		}

		kfree(write_reg.regs);
		break;
	case JESD_READ_REG:
		regcnf = kzalloc(sizeof(struct jesd_reg_read_buf), GFP_KERNEL);
		if (regcnf == NULL)
			retcode = -ENOMEM;

		if (retcode < 0)
			goto fail;

		if (copy_from_user(&regcnf, (struct jesd_reg_read_buf *)arg,
					sizeof(struct jesd_reg_read_buf))) {
			retcode = -EFAULT;
			break;
		}

		if (transdev->transport_type == DEVICE_TX) {
			if (regcnf->len > sizeof(struct config_registers_tx)) {
				regcnf->len =
					sizeof(struct config_registers_tx);
				regcnf->offset = 0;
			}
			read_reg(&transdev->tx_regs->tx_did, regcnf->offset,
						regcnf->len, regcnf->buf);
		} else if (transdev->transport_type == DEVICE_RX) {
			if (regcnf->len > sizeof(struct config_registers_rx)) {
				regcnf->len =
					sizeof(struct config_registers_rx);
				regcnf->offset = 0;
			}
			read_reg(&transdev->rx_regs->rx_rdid, regcnf->offset,
						regcnf->len, regcnf->buf);
		} else
			retcode = -EFAULT;

		if (retcode < 0)
			break;

		if (copy_to_user((struct jesd_reg_read_buf *)arg, &regcnf,
					sizeof(struct jesd_reg_read_buf)))
			retcode = -EFAULT;

		kfree(regcnf);
		break;
	case JESD_SHUTDOWN:
		shutdown(transdev);
		break;
	case JESD_GET_DEVICE_INFO:

		memcpy(&trans_dev_info.ils, &transdev->ils,
						sizeof(struct ils_params));
		trans_dev_info.init_params.sampling_rate =
						transdev->sampling_rate;
		trans_dev_info.init_params.delay =
						transdev->delay;
		trans_dev_info.init_params.tran_flg =
						transdev->tranport_flags;
		trans_dev_info.init_params.lanes =
						transdev->lanes_per_transport;
		trans_dev_info.ilas_len = transdev->ilsa_len;
		trans_dev_info.dev_state = transdev->dev_state;

		if (copy_to_user((struct transport_device_info *)arg,
					&trans_dev_info,
					sizeof(struct transport_device_info)))
				retcode = -EFAULT;
		break;
	case JESD_GET_LANE_RX_RECEIVED_PARAMS:
		if (transdev->transport_type == DEVICE_RX) {
			ilsparams = kzalloc(sizeof(struct ils_params),
							GFP_KERNEL);
			ilsparams->device_id = transdev->rx_regs->rx_did;
			ilsparams->bank_id = transdev->rx_regs->rx_rdid;
			ilsparams->lane_0_id = transdev->rx_regs->rx_rlid_0;
			ilsparams->lane_1_id = transdev->rx_regs->rx_rlid_1;
			ilsparams->scrambling_scr = jtest_bit(7,
						&transdev->rx_regs->rx_rscr);
			ilsparams->lanes_per_converter_l =
					(transdev->rx_regs->rx_rscr & 0x1f);
			ilsparams->octect_per_frame_f =
					transdev->rx_regs->rx_ropf;
			ilsparams->frame_per_mf_k =
					transdev->rx_regs->rx_rfpmf;
			ilsparams->conv_per_device_m =
					transdev->rx_regs->rx_rcpd;
			ilsparams->cs = (transdev->rx_regs->rx_rcbps >> 6);
			ilsparams->n = (transdev->rx_regs->rx_rcbps & 0x1f);
			ilsparams->np = (transdev->rx_regs->rx_rnbcw & 0x1f);
			ilsparams->s = (transdev->rx_regs->rx_rspcp & 0x1f);
			ilsparams->cf = (transdev->rx_regs->rx_rhdf & 0x1f);
			ilsparams->hd = (transdev->rx_regs->rx_rhdf >> 7);
			ilsparams->subclass_ver =
					(transdev->rx_regs->rx_rnbcw >> 5);
			ilsparams->jesd_ver =
					(transdev->rx_regs->rx_rspcp >> 5);
			ilsparams->csum_lane_0 = transdev->rx_regs->rx_rcsum_0;
			ilsparams->csum_lane_1 = transdev->rx_regs->rx_rcsum_1;
			if (copy_to_user((struct ils_params *)arg,
						ilsparams,
						sizeof(struct ils_params)))
					retcode = -EFAULT;
			kfree(ilsparams);
		}
		break;
	case JESD_RX_AUTO_SYNC:
		if (transdev->transport_type == DEVICE_RX) {
			sync_params = kzalloc(sizeof(struct auto_sync_params),
							GFP_KERNEL);
			if (copy_from_user(sync_params,
				(struct auto_sync_params *)arg,
				sizeof(struct auto_sync_params))) {
				retcode = -EFAULT;
				break;
			}
			transdev->rx_regs->rx_threshold_err =
					(sync_params->error_threshold & 0xff);

			/*enable the vectors for enabling resync*/
			if (jtest_bit(BAD_DIS_SYNC,
						&sync_params->sync_err_bitmap))
				jesd_set_bit(SYNC_ASSERT_BIT7,
					&transdev->rx_regs->rx_sync_ass_msk);
			else if (jtest_bit(NIT_SYNC,
						&sync_params->sync_err_bitmap))
				jesd_set_bit(SYNC_ASSERT_BIT6,
					&transdev->rx_regs->rx_sync_ass_msk);
			else if	 (jtest_bit(UX_CHAR_SYNC,
						&sync_params->sync_err_bitmap))
				jesd_set_bit(SYNC_ASSERT_BIT5,
					&transdev->rx_regs->rx_sync_ass_msk);

			kfree(sync_params);
		} else
			return -EINVAL;
		break;
	case JESD_GET_DEVICE_STATE:
		*state = arg;
		*state = transdev->dev_state;
		break;
	case JESD_SET_DEVICE_STATE:
		*state = arg;
		transdev->dev_state = *state;
		break;
	default:
		retcode = -ENOTTY;
		break;
	}
fail:
	return retcode;
}


static int jesd204_open(struct inode *inode, struct file *pfile)
{
	struct transport_device *tdev = NULL;
	int retcode = 0;

	tdev = container_of(inode->i_cdev, struct transport_device, c_dev);
	if (tdev != NULL)
		pfile->private_data = tdev;
	else
		retcode = -ENODEV;

	return retcode;
}

int jesd204_release(struct inode *inode, struct file *pfile)
{
	int retcode = 0;
	struct transport_device *tdev = NULL;

	tdev = pfile->private_data;

	if (tdev != NULL) {
		if (tdev->jesd_parent->lanes_grabed > 0) {
			tdev->jesd_parent->lanes_grabed =
						tdev->jesd_parent->lanes_grabed
						- tdev->lanes_per_transport;
		}
	} else
		retcode = -ENODEV;

	return retcode;
}

static void bh_txhandler(struct transport_device *tdev,
			u32 in_status_reg,
			u32 in_isr_mask)
{
	u32 status_reg = in_status_reg;
	u32 isr_mask = in_isr_mask;

	if (jtest_bit(SYSREF_ISR_BIT8, &status_reg)) {

		jesd_set_bit(SYSREF_ISR_BIT8, &tdev->tx_regs->tx_irq_status);
		/*dfifo*/
		jesd_set_bit(DFIFO_BIT3, &tdev->tx_regs->tx_transcontrol);
		jesd_clear_bit(DFIFO_BIT3, &tdev->tx_regs->tx_transcontrol);
		/*disable sysref rose*/
		jesd_clear_bit(SYSREF_ISR_BIT8, &tdev->tx_regs->tx_irq_enable);
		tdev->sysref_rose = 1;
		wake_up(&tdev->isr_wait);
		/*sync detected change state and send event*/
		raise_exception(tdev, SYS_REF_ROSE);
	}
	/*by default we support sync and sys_ref_rose and report it*/
	if (jtest_bit(SYNC_ISR_BIT9, &status_reg)) {
		/*sync detected change state and send event*/
		raise_exception(tdev, SYNC_DETECTED);
		jesd_set_bit(SYNC_ISR_BIT9, &tdev->tx_regs->tx_irq_status);
		tdev->dev_state = SYNC_START;
	}

	if (jtest_bit(UNDERF_ISR_BIT3, &isr_mask)) {
		/*unpacker_underflow supported*/
		if (jtest_bit(UNDERF_ISR_BIT3, &status_reg))
			/*unpacker_underflow detected */
			raise_exception(tdev, UPAC_UF_ERR);

		jesd_set_bit(UNDERF_ISR_BIT3, &tdev->tx_regs->tx_irq_status);
	}

	if (jtest_bit(WCBOF_ISR_BIT4, &isr_mask)) {
		/*Write Circular Buffer overflow supported*/
		if (jtest_bit(WCBOF_ISR_BIT4, &status_reg))
			/*Write Circular Buffer overflow detected*/
			raise_exception(tdev, WBUF_OF_ERR);

		jesd_set_bit(WCBOF_ISR_BIT4, &tdev->tx_regs->tx_irq_status);
	}

	if (jtest_bit(WCBUF_ISR_BIT5, &isr_mask)) {
		/*Write Circular Buffer underflow supported*/
		if (jtest_bit(WCBUF_ISR_BIT5, &status_reg))
			/*Write Circular Buffer underflow detected*/
			raise_exception(tdev, WBUF_UF_ERR);

		jesd_set_bit(WCBUF_ISR_BIT5, &tdev->tx_regs->tx_irq_status);
	}
}

static void bh_rxhandler(struct transport_device *tdev)
{
	/* this is for the nit, uex_k, bad_dis skworr and csum user event*/
	/*check if deframer isr needs to be triggered*/
	if (tdev->rx_regs->rx_irq_status & 1)
		handle_maskvector(tdev);
	/*do irq*/
	handle_rx_irq(tdev);

	/*reset isr that are processred by this thime*/
	reset_sync_interrupts(tdev);
}

static void tran_tx_isr_tasklet(unsigned long data)
{
	struct transport_device *tdev = (struct transport_device *)data;
	u32 isr_mask = ioread32(&tdev->tx_regs->tx_irq_enable);
	u32 status_reg = ioread32(&tdev->tx_regs->tx_irq_status);

	spin_lock(&tdev->lock_tx_bf);
	/*disable for processing*/
	iowrite32(0x00, &tdev->tx_regs->tx_irq_enable);
	iowrite32(0x00, &tdev->tx_regs->tx_irq_status);

	bh_txhandler(tdev, status_reg, isr_mask);

	/*enable as per catche*/
	iowrite32(status_reg, &tdev->tx_regs->tx_irq_status);
	iowrite32(isr_mask, &tdev->tx_regs->tx_irq_enable);
	spin_unlock(&tdev->lock_tx_bf);
}

static void tran_rx_isr_tasklet(unsigned long data)
{
	struct transport_device *tdev = (struct transport_device *)data;
	u32 isr_mask = ioread32(&tdev->rx_regs->rx_irq_enable);
	u32 status_reg = ioread32(&tdev->rx_regs->rx_irq_status);

	spin_lock(&tdev->lock_rx_bf);
	iowrite32(0x00, &tdev->rx_regs->rx_irq_status);
	iowrite32(0x00, &tdev->rx_regs->rx_irq_enable);

	bh_rxhandler(tdev);

	iowrite32(status_reg, &tdev->rx_regs->rx_irq_status);
	iowrite32(isr_mask, &tdev->rx_regs->rx_irq_enable);
	spin_unlock(&tdev->lock_rx_bf);
}

static irqreturn_t do_isr(int irq, void *param)
{
	struct transport_device *tdev = param;

	if (tdev->transport_type == DEVICE_TX)
		tasklet_hi_schedule(&tdev->do_tx_tasklet);
	else if (tdev->transport_type == DEVICE_RX)
		tasklet_hi_schedule(&tdev->do_rx_tasklet);

	return IRQ_HANDLED;
}

static int transport_register_irq(struct transport_device *tdev)
{

	int retcode = 0;

	if (tdev != NULL) {
		/* would populate after DTS*/
		retcode = request_irq(tdev->irq,
					do_isr,
					0,/*no flag*/
					"jesd",
					tdev);
		tasklet_init(&tdev->do_tx_tasklet, tran_tx_isr_tasklet,
						 (unsigned long)tdev);
		tasklet_init(&tdev->do_rx_tasklet, tran_rx_isr_tasklet,
						 (unsigned long)tdev);

		spin_lock_init(&tdev->lock_tx_bf);
		spin_lock_init(&tdev->lock_rx_bf);
		spin_lock_init(&tdev->sysref_lock);
	}
	return retcode;
}

/** @brief file ops structure
 */
static const struct file_operations jesd204_fops = {
	.owner = THIS_MODULE,
	.open = jesd204_open,
	.unlocked_ioctl = jesd204_ioctl,
	.release = jesd204_release
};

static int create_c_dev(struct transport_device *tdev,
			int jesd204_major,
			int jesd_minor)
{
	int retcode = -ENODEV;
	struct transport_device *t = tdev;
	t->devt = MKDEV(jesd204_major, jesd_minor + t->identifier);

	cdev_init(&t->c_dev, &jesd204_fops);
	t->c_dev.owner = THIS_MODULE;
	retcode = cdev_add(&t->c_dev, t->devt, 1);

	return retcode;
}

static int create_jesd_transports(struct jesd204_dev *jdev,
					unsigned int id,
					struct device *dev)
{
	int retcode = 0;
	int type = jdev->trans_type;
	void __iomem *base;
	struct jesd204_dev *jesddev = jdev;
	u32 addr;

	jesddev->trans_device[id] = kzalloc(sizeof(struct transport_device),
					GFP_KERNEL);

	if (!jesddev->trans_device[id]) {
		dev_err(dev, "transport allocation failure");
		retcode = -ENOMEM;
		goto error0;
	}

	jesddev->trans_device[id]->jesd_parent = jesddev;
	jesddev->trans_device[id]->identifier = id;
	jesddev->trans_device[id]->jesd_parent->lanes_grabed = 0;

	retcode = (unsigned int) of_property_read_u32(jesddev->child,
						"reg",
						&addr);
		if (retcode < 0) {
			dev_err(dev, "addr is  %x", addr);
			goto error0;
		}

	base = of_iomap(jesddev->child, 0);

	if (!base) {
		dev_err(dev, "transport memap maping failure");
		retcode = -ENOMEM;
		goto error1;
	}

	jesddev->trans_device[id]->base_address = base;
	/*logical assignment of register to the transport*/
	if (type == DEVICE_TX) {
		jesddev->trans_device[id]->tx_regs =
				(struct config_registers_tx *)base;
		jesddev->trans_device[id]->transport_type = type;
		jesddev->trans_device[id]->rx_regs = NULL;
	} else if (type == DEVICE_RX) {
		jesddev->trans_device[id]->rx_regs =
				(struct config_registers_rx *)base;
		jesddev->trans_device[id]->transport_type = type;
		jesddev->trans_device[id]->tx_regs = NULL;
	} else {
		retcode = -EFAULT;
		goto error1;
	}

	if (jesddev->trans_device[id]->identifier == TRANPORT_0)
		jesddev->trans_device[id]->jesd_parent->max_lanes = 2;
	else
		jesddev->trans_device[id]->jesd_parent->max_lanes = 1;

error1:
	kfree(jesddev->trans_device[id]);
error0:
	return retcode;
}

void jesddev_list_cleanup(void)
{
	struct jesd204_dev *cur, *nxt;

	list_for_each_entry_safe(cur, nxt, &pri->list, jesd_dev_list) {
		list_del(&cur->jesd_dev_list);
	}
}

int get_device_name(struct transport_device *tdev)
{
	int retcode = 0;
	if (tdev->transport_type == DEVICE_TX) {
		snprintf(tdev->name, 32, "jesd_tx_%d", pri->tx_dev_num);
			pri->tx_dev_num = pri->tx_dev_num + 1;
	} else if (tdev->transport_type == DEVICE_RX) {
		snprintf(tdev->name, 32, "jesd_rx_%d", pri->rx_dev_num);
			pri->rx_dev_num = pri->rx_dev_num + 1;
	} else
			retcode = -EINVAL;
	return retcode;
}

static int __init jesd204_of_probe(struct platform_device *pdev)
{
	int retcode = -ENODEV;
	int jesd_major, jesd_minor;
	unsigned int id;
	struct jesd204_dev *jesddev = NULL;
	struct device_node *child = NULL;
	struct device_node *node = pdev->dev.of_node;
	dev_t devt;

	jesddev = kzalloc(sizeof(struct jesd204_dev), GFP_KERNEL);

	if (!jesddev) {
		retcode = -ENOMEM;
		goto failed0;
	}

	/*accomidate dev*/
	jesddev->node = pdev->dev.of_node;

	for_each_child_of_node(node, child) {
		retcode = of_property_read_u32(child,
						"trans-type",
						&jesddev->trans_type);
		if (retcode < 0) {
			dev_err(&pdev->dev, "Trans type failed  %x",
				jesddev->trans_type);
			goto failed2;
		}

		if (jesddev->trans_type == DEVICE_TX) {
			retcode = alloc_chrdev_region(&devt,
							0,
							TRANSPORT_PER_IP,
							"jesd204-tx");
		} else if (jesddev->trans_type == DEVICE_RX) {
			retcode = alloc_chrdev_region(&devt,
							0,
							TRANSPORT_PER_IP,
							"jesd204-rx");
		} else {
			retcode = -EINVAL;
			dev_info(&pdev->dev, "invalid transport type");
		}
		if (retcode < 0)
			goto failed1;

		jesd_major = MAJOR(devt);
		jesd_minor = MINOR(devt);

		retcode = of_property_read_u32(child, "trans-id", &id);

		if (retcode < 0) {
			dev_err(&pdev->dev, "%s %x trans id\n",
				DRIVER_NAME, id);
			goto failed2;
		}

		if (id > TRANSPORT_PER_IP) {
			dev_err(&pdev->dev, "%s %i too many device\n",
				DRIVER_NAME, id);
			retcode = -ENODEV;
			goto failed2;
		}

		jesddev->child = child;
		retcode = create_jesd_transports(jesddev, id, &pdev->dev);

		if (retcode < 0) {
			dev_err(&pdev->dev, "%x create failed\n",
				retcode);
			goto failed2;
		}

		jesddev->trans_device[id]->dev = &pdev->dev;
		jesddev->trans_device[id]->transport_type = jesddev->trans_type;

		retcode = create_c_dev(jesddev->trans_device[id],
						jesd_major, jesd_minor);
		if (retcode < 0) {
			dev_err(&pdev->dev, "cdev add failed\n");
			goto failed2;
		}

		/* get the maped phygasket node into here */
		jesddev->trans_device[id]->phy_node = of_parse_phandle(child,
						"phy-gasket", 0);
		jesddev->trans_device[id]->phy =
			map_phygasket(jesddev->trans_device[id]->phy_node);

		if (jesddev->trans_device[id]->phy == NULL) {
			dev_err(&pdev->dev, "phy gasket is not mapped\n");
			goto failed2;
		}

		jesddev->trans_device[id]->irq = irq_of_parse_and_map(child, 0);

		retcode = transport_register_irq(jesddev->trans_device[id]);

		if (retcode < 0) {
			dev_err(&pdev->dev, "irq register failed\n");
			goto failed2;
		}

		retcode = of_property_read_u32_array(child,
					"lane",
					jesddev->trans_device[id]->lane_id,
				ARRAY_SIZE(jesddev->trans_device[id]->lane_id));

		if (retcode < 0) {
			dev_err(&pdev->dev, "device lane config error\n");
			goto failed2;
		}

		jesddev->trans_device[id]->lanes_per_transport =
					jesddev->trans_device[id]->lane_id[0];

		init_waitqueue_head
				(&jesddev->trans_device[id]->isr_wait);
		init_waitqueue_head
				(&jesddev->trans_device[id]->to_wait);

		if (jesddev->trans_type == DEVICE_TX) {
			device_create(jesd204_class,
					jesddev->trans_device[id]->dev,
					jesddev->trans_device[id]->devt,
					NULL,
					"%s%d",
					TRANS_TX_NAME,
					id);
		} else if (jesddev->trans_type == DEVICE_RX) {
			device_create(jesd204_class,
					jesddev->trans_device[id]->dev,
					jesddev->trans_device[id]->devt,
					NULL,
					"%s%d",
					TRANS_RX_NAME,
					id);
		} else
			dev_info(&pdev->dev, "invalid tansport\n");

		jesddev->trans_device[id]->dev_state = JESD_STANDBY;
		dev_info(&pdev->dev, "jesd transport added\n");
	}

	if (retcode < 0) {
		dev_info(&pdev->dev, "jesd probe failed\n");
		goto failed2;
	}

	list_add_tail(&jesddev->jesd_dev_list, &pri->list);
	dev_set_drvdata(&pdev->dev, jesddev);

	dev_info(&pdev->dev, "jesd probe passed\n");
	return retcode;

failed2:
	unregister_chrdev_region(devt, TRANSPORT_PER_IP);
failed1:
	kfree(jesddev);
failed0:
	dev_info(&pdev->dev, "jesd probe failed\n");
	return retcode;
}

static int __exit jesd204_of_remove(struct platform_device *pdev)
{
	int id = 0;
	int retcode = 0;
	struct jesd204_dev *jesddev = NULL;

	jesddev = (struct jesd204_dev *)dev_get_drvdata(&pdev->dev);

	if (!jesddev) {
		retcode = -ENODEV;
		goto error;
	}

	for (id = 0; id < TRANSPORT_PER_IP; id++) {
		jesddev_list_cleanup();
		device_destroy(jesd204_class, jesddev->trans_device[id]->devt);
		cdev_del(&jesddev->trans_device[id]->c_dev);
		if (jesddev->trans_type == DEVICE_TX)
			jesddev->trans_device[id]->tx_regs = NULL;
		else if (jesddev->trans_type == DEVICE_RX)
			jesddev->trans_device[id]->rx_regs = NULL;
		iounmap(jesddev->trans_device[id]->base_address);
		kfree(jesddev->trans_device[id]);
	}

	kfree(jesddev);
	dev_set_drvdata(&pdev->dev, NULL);
error:
	return retcode;		/* success */
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
	int retcode;


	pri = kzalloc(sizeof(struct jesd204_private), GFP_KERNEL);

	if (!pri) {
		retcode = -ENOMEM;
		goto jesd204_err;
	}
	INIT_LIST_HEAD(&pri->list);
	pri->tx_dev_num = 0;
	pri->rx_dev_num = 0;

	jesd204_class = class_create(THIS_MODULE, "jesd204");

	if (IS_ERR(jesd204_class)) {
		retcode = PTR_ERR(jesd204_class);
		goto jesd204_err;
	}

	return platform_driver_register(&jesd204_driver);
jesd204_err:
		return retcode;
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

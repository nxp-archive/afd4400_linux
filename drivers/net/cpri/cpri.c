/*
 * drivers/net/cpri/cpri.c
 * CPRI device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/pid.h>

#include <linux/cpri.h>

static struct of_device_id cpri_match[] = {
	{.compatible = "fsl,d4400-cpri",},
	{.compatible = "fsl,medusa-cpri",},
	{},
};

static struct class *cpri_class;

static LIST_HEAD(cpri_dev_list);
raw_spinlock_t cpri_list_lock;

struct cpri_framer *get_attached_cpri_dev(struct device_node **sfp_dev_node)
{
	struct cpri_dev *cpri_dev = NULL;
	struct cpri_framer *framer = NULL;
	struct sfp_dev *sfp_dev;
	struct device_node *node = NULL;
	int i;

	raw_spin_lock(&cpri_list_lock);
	list_for_each_entry(cpri_dev, &cpri_dev_list, list) {

		for (i = 0; i < cpri_dev->framers; i++) {
			framer = cpri_dev->framer[i];
			if (*sfp_dev_node == framer->sfp_dev_node) {
				node = framer->sfp_dev_node;
				break;
			}
		}

		if ((framer->sfp_dev == NULL) && node) {
			sfp_dev = container_of(sfp_dev_node, struct sfp_dev,
						dev_node);
			framer->sfp_dev = sfp_dev;
			cpri_state_machine(framer,
						CPRI_STATE_STANDBY);
			break;
		}
	}
	raw_spin_unlock(&cpri_list_lock);

	return framer;
}
EXPORT_SYMBOL(get_attached_cpri_dev);

static int cpri_open(struct inode *inode, struct file *fp)
{
	struct cpri_framer *framer = NULL;
	int rc = 0;

	framer = container_of(inode->i_cdev, struct cpri_framer, cdev);

	if (framer != NULL) {
		fp->private_data = framer;
		dev_info(framer->cpri_dev->dev, "framer id:%d\n", framer->id);
	}
	else
		rc = -ENODEV;

	return rc;
}

static int cpri_release(struct inode *inode, struct file *fp)
{
	struct cpri_framer *framer = (struct cpri_framer *)fp->private_data;

	if (!framer)
		return -ENODEV;

	return 0;
}

static const struct file_operations cpri_fops = {
	.owner = THIS_MODULE,
	.open = cpri_open,
	.unlocked_ioctl = cpri_ioctl,
	.release = cpri_release,
};

void framer_int_enable(struct cpri_framer *framer)
{
	u32 val = 0;

	val = ETH_EVENT_EN_MASK  | CONTROL_INT_LEVEL_MASK;

	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_rctrltiminginten,
			MASK_ALL, val);
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tctrltiminginten,
			MASK_ALL, val);

	val = (RX_IQ_OVERRUN | TX_IQ_UNDERRUN |
		TX_VSS_UNDERRUN | RX_VSS_OVERRUN |
		ECC_CONFIG_MEM | ECC_DATA_MEM | RX_ETH_MEM_OVERRUN |
		TX_ETH_UNDERRUN | RX_ETH_BD_UNDERRUN | RX_ETH_DMA_OVERRUN |
		ETH_FORWARD_REM_FIFO_FULL);
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_errinten, MASK_ALL, val);
}

static void cpri_interrupt_enable(struct cpri_dev *cpdev)
{

	int loop = 0;
	u32 val;
	for (loop = 1; loop < CPRI_INT_COUNT; loop++) {
		val = (((loop - 1) << 28) | LEVEL_MASK |
			IEVENT_IQ_THRESHOLD_EN_MASK | IEVENT_ETH_EN_MASK);
		cpri_reg_write(&cpdev->lock,
				&cpdev->regs->cpri_intctrl[loop],
				MASK_ALL, val);
	}
}

static irqreturn_t cpri_rxtiming(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_HFN_MASK | IEVENT_BFN_MASK;
	events = cpri_reg_get_val(&framer->regs->cpri_revent, mask);

	if (events & IEVENT_HFN_MASK)
		framer->stats.rx_hfn_irqs++;
	else if (events & IEVENT_BFN_MASK)
		framer->stats.rx_bfn_irqs++;

	/* Clear event by writing 1 */
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_revent,
				events, events);

	return IRQ_HANDLED;
}

static irqreturn_t cpri_txtiming(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_HFN_MASK | IEVENT_BFN_MASK | IEVENT_IQ_THRESHOLD_MASK;
	events = cpri_reg_get_val(&framer->regs->cpri_tevent, mask);

	if (events &  IEVENT_HFN_MASK)
		framer->stats.tx_hfn_irqs++;
	if (events &  IEVENT_BFN_MASK)
		framer->stats.tx_bfn_irqs++;

	/* Clear event by writing 1 */
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tevent,
				events, events);

	return IRQ_HANDLED;
}

static irqreturn_t cpri_txcontrol(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;

	events = cpri_reg_get_val(&framer->regs->cpri_tevent, MASK_ALL);

	/* Handle tx ethernet event - Called function will disable the
	 * the interrupt and schedules for bottom half. Interrupt will
	 * be enabled once the tx packets are processed
	 */
	if (events &  IEVENT_ETH_MASK) {
		cpri_reg_write(&framer->regs_lock,
				&framer->regs->cpri_tctrltiminginten,
				ETH_EVENT_EN_MASK, ~ETH_EVENT_EN_MASK);
		cpri_eth_handle_tx(framer);
	}

	/* TODO: Handle VSS threshold event here */
	/* Clear events */
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tevent,
				events, events);

	return IRQ_HANDLED;
}

static irqreturn_t cpri_rxcontrol(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;

	events = cpri_reg_get_val(&framer->regs->cpri_revent, MASK_ALL);

	/* Handle rx ethernet event - Called function will disable the
	 * the interrupt and schedules for bottom half. Interrupt will
	 * be enabled once the rx packets are processed
	 */
	if (events & IEVENT_ETH_MASK) {
		cpri_reg_write(&framer->regs_lock,
				&framer->regs->cpri_rctrltiminginten,
				ETH_EVENT_EN_MASK, ~ETH_EVENT_EN_MASK);
		cpri_eth_handle_rx(framer);
	}

	/* TODO: Handle VSS threshold event here */

	/* Clear events */
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_revent,
				events, events);

	return IRQ_HANDLED;
}

static int cpri_register_framer_irqs(struct cpri_framer *framer)
{
	int err;

/* mask all the interrupt */

	err = request_irq(framer->irq_rx_c, cpri_rxcontrol, 0,
			"cpri rxcontrol interrupt", framer);
	if (err < 0)
		goto out;

	err = request_irq(framer->irq_tx_c , cpri_txcontrol, 0,
			"cpri txcontrol interrupt", framer);
	if (err < 0)
		goto irq_rxcontrol;

	err = request_irq(framer->irq_rx_t, cpri_rxtiming, 0,
			"cpri rxtiming interrupt", framer);
	if (err < 0)
		goto irq_txcontrol;

	err = request_irq(framer->irq_tx_t, cpri_txtiming, 0,
			"cpri txtiming interrupt", framer);
	if (err < 0)
		goto irq_rxtiming;

	return 0;


irq_rxtiming:
	free_irq(framer->irq_rx_t, framer);
irq_txcontrol:
	free_irq(framer->irq_tx_c, framer);
irq_rxcontrol:
	free_irq(framer->irq_rx_c, framer);
out:
	dev_err(framer->cpri_dev->dev, "can't get IRQ\n");
	return err;
}

void cpri_mask_irq_events(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	/* disable timing interrupt events here -
	 * enabled on there respective init
	 */
	cpri_reg_clear(&regs->cpri_rctrltiminginten,
			MASK_ALL);

	cpri_reg_clear(&regs->cpri_tctrltiminginten,
			MASK_ALL);

	/* TBD: CPRIICR is not set in this driver. It is not clear
	 * why we have this physical interrupt line and the similar
	 * configuration like the above
	 */
	/* disable all error events by default */
	cpri_reg_clear(&regs->cpri_errinten,
			MASK_ALL);
}

int process_framer_irqs(struct device_node *child, struct cpri_framer *framer)
{
	struct device *dev = framer->cpri_dev->dev;
	/* Get the IRQ lines and register them per framer */
	framer->irq_rx_t = irq_of_parse_and_map(child, 0);
	framer->irq_tx_t = irq_of_parse_and_map(child, 1);
	framer->irq_rx_c = irq_of_parse_and_map(child, 2);
	framer->irq_tx_c = irq_of_parse_and_map(child, 3);
	dev_info(dev, "\nirq parsed: %d, %d, %d, %d\n",
			framer->irq_rx_t,
			framer->irq_tx_t,
			framer->irq_rx_c,
			framer->irq_tx_c);

	return cpri_register_framer_irqs(framer);
}

int cpri_state_validation(enum cpri_state present_state,
		enum cpri_state new_state)
{
	int ret = -EINVAL;

	switch (present_state) {
	case CPRI_STATE_SFP_DETACHED:
	case CPRI_STATE_STANDBY:
		if (new_state <= CPRI_STATE_CONFIGURED ||
				new_state <= CPRI_STATE_LINK_ERROR)
			ret = 0;
		break;
	case CPRI_STATE_CONFIGURED:
	case CPRI_STATE_LINK_ERROR:
	case CPRI_STATE_LINE_RATE_AUTONEG:
	case CPRI_STATE_LINE_RATE_AUTONEG_INPROGRESS:
		if (new_state <= CPRI_STATE_PROT_VER_AUTONEG ||
				new_state <= CPRI_STATE_LINK_ERROR)
			ret = 0;

		break;
	case CPRI_STATE_PROT_VER_AUTONEG:
		if (new_state <= CPRI_STATE_ETH_RATE_AUTONEG ||
				new_state <= CPRI_STATE_LINK_ERROR)
			ret = 0;

		break;
	case CPRI_STATE_ETH_RATE_AUTONEG:
		if (new_state <= CPRI_STATE_AUTONEG_COMPLETE ||
				new_state <= CPRI_STATE_LINK_ERROR)
			ret = 0;
		break;
	case CPRI_STATE_AUTONEG_COMPLETE:
		if (new_state <= CPRI_STATE_AXC_CONFIG)
			ret = 0;
		break;
	case CPRI_STATE_AXC_CONFIG:
		if ((new_state <= CPRI_STATE_AXC_MAP_INIT) ||
				(new_state <= CPRI_STATE_AXC_CONFIG))
			ret = 0;
		break;
	case CPRI_STATE_AXC_MAP_INIT:
		if ((new_state <= CPRI_STATE_OPERATIONAL) ||
				(new_state <= CPRI_STATE_AXC_MAP_INIT))
			ret = 0;
		break;

	case CPRI_STATE_OPERATIONAL:
		if ((new_state <= CPRI_STATE_PROT_VER_AUTONEG) ||
				(new_state <= CPRI_STATE_OPERATIONAL))
			ret = 0;

		break;
	default:
		break;
	}
	return ret;
}

/* Framer state update during error events. This update is used
 * by autoneg code to determine the proper entry in to the autoneg
 * state
 */
void cpri_state_machine(struct cpri_framer *framer, enum cpri_state new_state)
{
	enum cpri_state present_state = framer->framer_state;
	struct device *dev = framer->cpri_dev->dev;

		if (!cpri_state_validation(present_state, new_state))
			framer->framer_state = new_state;
		else {
			dev_err(dev, "CPRI-Invalid state change request: %d\n",
					new_state);
		}
	return;
}

static void do_framer_state_update(struct cpri_framer *framer, u32 mask)
{
	if ((mask & RLOS) | (mask & RLOF) | (mask & RAI) | (mask & RSDI)) {
		if (timer_pending(&framer->l1_timer))
			del_timer_sync(&framer->l1_timer);
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
	}
#if 0
	if ((mask & LLOS) | (mask & LLOF))
		cpri_state_machine(framer, CPRI_STATE_SFP_DETACHED);
#endif

	if (mask & RRE) {
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);
		if (timer_pending(&framer->l1_timer))
			del_timer_sync(&framer->l1_timer);
	}

	if (mask & FAE)
		cpri_state_machine(framer,
				CPRI_STATE_LINK_ERROR);

}

/* Stats update during error events */
static void do_err_stats_update(struct cpri_framer *framer,
			unsigned long mask)
{
	struct device *dev = framer->cpri_dev->dev;
	if (mask & RX_IQ_OVERRUN)
		framer->stats.rx_iq_overrun_err_count++;
	else if (mask & TX_IQ_UNDERRUN)
		framer->stats.tx_iq_underrun_err_count++;
	else if (mask & RX_ETH_MEM_OVERRUN)
		framer->stats.rx_eth_mem_overrun_err_count++;
	else if (mask & TX_ETH_UNDERRUN)
		framer->stats.tx_eth_underrun_err_count++;
	else if (mask & RX_ETH_BD_UNDERRUN)
		framer->stats.rx_eth_bd_underrun_err_count++;
	else if (mask & RX_HDLC_OVERRUN)
		framer->stats.rx_hdlc_overrun_err_count++;
	else if (mask & TX_HDLC_UNDERRUN)
		framer->stats.tx_hdlc_underrun_err_count++;
	else if (mask & RX_HDLC_BD_UNDERRUN)
		framer->stats.rx_hdlc_bd_underrun_err_count++;
	else if (mask & RX_VSS_OVERRUN)
		framer->stats.rx_vss_overrun_err_count++;
	else if (mask & TX_VSS_UNDERRUN)
		framer->stats.tx_vss_underrun_err_count++;
	else if (mask & ECC_CONFIG_MEM)
		framer->stats.ecc_config_mem_err_count++;
	else if (mask & ECC_DATA_MEM)
		framer->stats.ecc_data_mem_err_count++;
	else if (mask & RX_ETH_DMA_OVERRUN)
		framer->stats.rx_eth_dma_overrun_err_count++;
	else if (mask & ETH_FORWARD_REM_FIFO_FULL)
		framer->stats.eth_fwd_rem_fifo_full_err_count++;
	else if (mask & EXT_SYNC_LOSS)
		framer->stats.ext_sync_loss_err_count++;
	else if (mask & RLOS)
		framer->stats.rlos_err_count++;
	else if (mask & RLOF)
		framer->stats.rlof_err_count++;
	else if (mask & RAI)
		framer->stats.rai_err_count++;
	else if (mask & RSDI)
		framer->stats.rsdi_err_count++;
	else if (mask & LLOS)
		framer->stats.llos_err_count++;
	else if (mask & LLOF)
		framer->stats.llof_err_count++;
	else if (mask & RRE)
		framer->stats.rr_err_count++;
	else if (mask & FAE)
		framer->stats.fa_err_count++;
	else if (mask & RRA)
		framer->stats.rra_err_count++;
	else
		dev_err(dev, "Invalid mask during stats update\n");
}

static void cpri_err_tasklet(unsigned long arg)
{
	struct cpri_framer *framer = (struct cpri_framer *) arg;
	u32 err_evt;
	u32 mask;

	err_evt = cpri_reg_get_val(&framer->regs->cpri_errevent,
				CPRI_ERR_EVT_ALL);
	/* Handle the error events */
	/* TBD: Process events and notify user
	 * synchronously for all the framers here.
	 */

	mask = (RX_IQ_OVERRUN | TX_IQ_UNDERRUN |
		TX_VSS_UNDERRUN | RX_VSS_OVERRUN |
		ECC_CONFIG_MEM | ECC_DATA_MEM | TX_ETH_UNDERRUN);
	if (err_evt && mask) {
		cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_errevent,
			err_evt, err_evt);
	}
	/* Update stats */
	do_err_stats_update(framer, err_evt);

	/* Update state */
	do_framer_state_update(framer, err_evt);

	/* Handle ethernet error if any */
	if(framer->frmr_ethflag == CPRI_ETH_SUPPORTED)
		cpri_eth_handle_error(framer);

		/* Restore the interrupt mask */
	err_evt = mask & err_evt;
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_errinten,
			err_evt, err_evt);

	return;
}


static irqreturn_t cpri_err_handler(int irq, void *cookie)
{
	struct cpri_dev *dev = (struct cpri_dev *)cookie;
	struct cpri_framer *framer;
	u32 err_status, err_evt;

	/* Check which framer has got error */
	err_status = cpri_reg_get_val(&dev->regs->cpri_errstatus,
				C1_ERR_MASK | C2_ERR_MASK);

	if (err_status & C1_ERR_MASK) {
		framer = dev->framer[0];
		err_evt = cpri_reg_get_val(&framer->regs->cpri_errevent,
				CPRI_ERR_EVT_ALL);
		/* Disable the error interrupt mask */
		cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_errinten,
			err_evt, (err_evt ^ err_evt));

		dev_info(dev->dev, "err_evt occur ----- >0x%x\n", err_evt);
		/* Schedule the bottom-half for handling error event */
		tasklet_schedule(dev->err_tasklet_frmr0);

	} else if (err_status & C2_ERR_MASK) {
		framer = dev->framer[1];
		err_evt = cpri_reg_get_val(
			&framer->regs->cpri_errevent, CPRI_ERR_EVT_ALL);
		/* Disable the error interrupt mask */
		cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_errinten,
			err_evt, (err_evt ^ err_evt));

		dev_info(dev->dev, "err_evt occur ----- >0x%x\n", err_evt);
		/* Schedule the bottom-half for handling error event */
		tasklet_schedule(dev->err_tasklet_frmr1);

	} else {
		dev_err(dev->dev, "spurious error interrupt\n");
		return IRQ_NONE;
	}


	return IRQ_HANDLED;
}

static int cpri_register_irq(struct cpri_dev *cpdev)
{
	int err = 0;
	int loop = 0;
	for (loop = 1; loop < CPRI_INT_COUNT; loop++) {
		cpri_reg_set_val(&cpdev->regs->cpri_intctrl[loop],
			MASK_ALL, 0);
	}

	err = request_irq(cpdev->irq_err, cpri_err_handler, 0,
				"cpri error interrupt", cpdev);
	/* Handling only the error interrupt (111) */
	if (err < 0)
		dev_err(cpdev->dev, "can't get IRQ\n");
	else {
		/* Initialise tasklet dynamically for bottom-half
		 * so that it can be be scheduled when event occurs
		 */
		/* framer-0 tasklet */
		cpdev->err_tasklet_frmr0 = kmalloc(
				sizeof(struct tasklet_struct), GFP_KERNEL);
		tasklet_init(cpdev->err_tasklet_frmr0, cpri_err_tasklet,
				(unsigned long) cpdev->framer[0]);
		/* framer-1 tasklet */
		cpdev->err_tasklet_frmr1 = kmalloc(
				sizeof(struct tasklet_struct), GFP_KERNEL);
		tasklet_init(cpdev->err_tasklet_frmr1, cpri_err_tasklet,
				(unsigned long) cpdev->framer[1]);
	}

	/* TBD: General interrupts are not handled yet - 99-102.
	 * Purpose of these interrupts are not clear
	 */

	return err;
}

static int cpri_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child = NULL;
	struct device *dev = &pdev->dev;
	struct cpri_dev *cpri_dev;
	struct cpri_framer *framer = NULL;
	dev_t devt;
	int cpri_major, cpri_minor;
	unsigned int framer_id, max_framers;
	int ret = 0, i, rc = 0;
	struct cpri_common_regs __iomem *common_regs;
	const struct of_device_id *id;
	unsigned char dev_name[20];

	if (!np || !of_device_is_available(np)) {
		rc = -ENODEV;
		goto err_out;
	}

	id = of_match_node(cpri_match, np);

	/* Allocate cpri device structure */
	cpri_dev = kzalloc(sizeof(struct cpri_dev), GFP_KERNEL);
	if (!cpri_dev) {
		dev_dbg(dev, "Failed to allocate cpri dev\n");
		rc = -ENOMEM;
		goto err_out;
	}

	/* Populate cpri device structure */
	of_property_read_u32(np, "id", &cpri_dev->dev_id);
	if (cpri_dev->dev_id < 0) {
		dev_dbg(dev, "Failed to get cpri id\n");
		rc = -ENODEV;
		goto err_mem;
	}

	if (id) {
		if (!strcmp(id->compatible, "fsl,medusa-cpri"))
			cpri_dev->dev_flags |= CPRI_MEDUSA_OCRAM;
	else
			cpri_dev->dev_flags |= CPRI_D4400;
	}

	sprintf(dev_name, "%s%d", DEV_NAME, cpri_dev->dev_id);

	cpri_dev->regs = of_iomap(np, 0);
	if (!cpri_dev->regs) {
		dev_dbg(dev, "Failed to map cpri reg addr\n");
		rc = -ENOMEM;
		goto err_mem;
	}

	cpri_dev->dev = dev;


	cpri_dev->irq_gen1 = irq_of_parse_and_map(np, 0);
	cpri_dev->irq_gen2 = irq_of_parse_and_map(np, 1);
	cpri_dev->irq_gen3 = irq_of_parse_and_map(np, 2);
	cpri_dev->irq_gen4 = irq_of_parse_and_map(np, 3);
	cpri_dev->irq_err = irq_of_parse_and_map(np, 4);

	/* Allocating dynamic major and minor nos for framer interface */
	rc = alloc_chrdev_region(&devt, 0,
			MAX_FRAMERS_PER_COMPLEX, dev_name);
	if (rc < 0) {
		dev_dbg(dev, "alloc_chrdev_region() failed\n");
		goto err_chrdev;
	}
	cpri_major = MAJOR(devt);
	cpri_minor = MINOR(devt);

	cpri_dev->framers = 0;

	max_framers = MAX_FRAMERS_PER_COMPLEX;
	cpri_dev->framer = kmalloc((sizeof(struct cpri_framer *) * max_framers),
					GFP_KERNEL);
	if (!cpri_dev->framer) {
		dev_err(dev, "Failed to allocate framer container\n");
		rc = -ENOMEM;
		goto err_mem;
	}
	common_regs = cpri_dev->regs;

	for_each_child_of_node(np, child) {

		of_property_read_u32(child, "framer-id", &framer_id);
		if (framer_id < 0) {
			dev_err(dev, "Failed to get framer id\n");
			rc = -EINVAL;
			goto err_chrdev;
		}

		cpri_dev->framers++;

		cpri_dev->framer[framer_id-1] =
			kzalloc(sizeof(struct cpri_framer), GFP_KERNEL);
		if (!cpri_dev->framer[framer_id-1]) {
			dev_err(dev, "Failed to allocate framer\n");
			rc = -ENOMEM;
			goto err_chrdev;
		}

		/* Populate framer strcuture */
		framer = cpri_dev->framer[framer_id-1];
		framer->cpri_dev = cpri_dev;
		framer->cpri_node = np;
		framer->id = framer_id;
		framer->framer_state = CPRI_STATE_SFP_DETACHED;
		dev_info(dev, "framer id:%d\n", framer->id);

		framer->regs = of_iomap(child, 0);
		if (!framer->regs) {
			dev_err(dev, "Failed to map framer reg addr\n");
			rc = -ENOMEM;
			goto err_chrdev;
		}
		dev_dbg(dev, "framer->regs ---> %p\n", framer->regs);
		/* here enable clock for framer register access */
		if (framer->id == 2)
			cpri_reg_set(
				&common_regs->cpri_ctrlclk, C2_CLK_MASK);
		else
			cpri_reg_set(
				&common_regs->cpri_ctrlclk, C1_CLK_MASK);
		/* end here */

		of_property_read_u32(child, "max-axcs",
			(u32 *)&framer->max_axcs);
		if (framer->max_axcs < 0) {
			dev_err(dev, "Failed to get max axc\n");
			rc = -ENODEV;
			goto err_chrdev;
		}
		dev_info(dev, "max axcs:%d\n", framer->max_axcs);
		ret = init_axc_mem_blk(framer, child);
		if (ret) {
			dev_err(dev, "cpri axc memory init failed\n");
			ret = -EFAULT;
			goto err_chrdev;
		}

		/* Create cdev for each framer */
		framer->dev_t = MKDEV(cpri_major, cpri_minor + (framer_id-1));
		cdev_init(&framer->cdev, &cpri_fops);
		framer->cdev.owner = THIS_MODULE;
		rc = cdev_add(&framer->cdev, framer->dev_t, 1);
		if (rc < 0) {
			dev_err(dev, "cdev_add() failed\n");
			goto err_chrdev;
		}
		device_create(cpri_class, framer->cpri_dev->dev, framer->dev_t,
			NULL, "%s_frmr%d", dev_name, framer->id-1);

		raw_spin_lock_init(&framer->regs_lock);
		raw_spin_lock_init(&framer->tx_cwt_lock);

		cpri_mask_irq_events(framer);
		if (process_framer_irqs(child, framer) < 0) {
			framer->dev_flags |= CPRI_NO_EVENT;
			dev_err(dev, "framer events not supported\n");
		}

		/* Get the SFP handle and determine the cpri state */
		framer->sfp_dev_node = of_parse_phandle(child,
					"sfp-handle", 0);

		framer->sfp_dev = get_attached_sfp_dev(framer->sfp_dev_node);
		if (framer->sfp_dev != NULL) {
			cpri_state_machine(framer,
						CPRI_STATE_STANDBY);
		} else {
			cpri_state_machine(framer, CPRI_STATE_SFP_DETACHED);
			dev_err(dev, "no sfp, framer state remains detached\n");
		}

		INIT_WORK(&framer->lineautoneg_task, cpri_linkrate_autoneg);
		INIT_WORK(&framer->protoautoneg_task, cpri_proto_ver_autoneg);
		INIT_WORK(&framer->ethautoneg_task, cpri_eth_autoneg);
		INIT_WORK(&framer->allautoneg_task, cpri_autoneg_all);

		if (cpri_eth_init(pdev, framer, child) < 0) {
			dev_err(dev, "ethernet init failed 'cpri eth is not supported'\n");
		}
	}

	/* Setup cpri device interrupts */
	if (cpri_register_irq(cpri_dev) < 0) {
		dev_err(dev, "cpri dev irq init failure\n");
		goto err_cdev;
	}

	/* enable interrupt here */
	cpri_interrupt_enable(cpri_dev);


	/* Add to the list of cpri devices - this is required
	 * as the probe is called for multiple cpri complexes
	 */
	raw_spin_lock(&cpri_list_lock);
	list_add_tail(&cpri_dev->list, &cpri_dev_list);
	raw_spin_unlock(&cpri_list_lock);

	dev_set_drvdata(dev, cpri_dev);

	dev_info(dev, "cpri probe done\n");

	return 0;

err_cdev:
	cdev_del(&framer->cdev);
	for (i = 0; i < cpri_dev->framers; i++)
		device_destroy(cpri_class, cpri_dev->framer[i]->dev_t);
	class_destroy(cpri_class);
err_chrdev:
	unregister_chrdev_region(devt, MAX_FRAMERS_PER_COMPLEX);
	kfree(cpri_dev->framer);
err_mem:
	kfree(cpri_dev);
err_out:
	dev_err(dev, "cpri probe failure\n");

	return rc;
}

static int cpri_remove(struct platform_device *pdev)
{
	struct cpri_dev *cpri_dev = NULL, *cpdev;
	struct cpri_framer *framer;
	struct list_head *pos, *nx;
	int i;

	cpri_dev = (struct cpri_dev *)dev_get_drvdata(&pdev->dev);
	if (!cpri_dev)
		return -ENODEV;

	iounmap(cpri_dev->regs);
	free_irq(cpri_dev->irq_gen1, cpri_dev);
	free_irq(cpri_dev->irq_gen2, cpri_dev);
	free_irq(cpri_dev->irq_gen3, cpri_dev);
	free_irq(cpri_dev->irq_gen4, cpri_dev);
	free_irq(cpri_dev->irq_err, cpri_dev);

	/* Removing ethernet interfaces that are bound to this device
	 * and also the framer resources
	 */
	for (i = 0; i < cpri_dev->framers; i++) {
		framer = cpri_dev->framer[i];
		cpri_eth_deinit(pdev, framer);
		iounmap(framer->regs);
		free_irq(framer->irq_rx_t, framer);
		free_irq(framer->irq_tx_t, framer);
		free_irq(framer->irq_rx_c, framer);
		free_irq(framer->irq_tx_c, framer);
		axc_buf_cleanup(framer);
		cdev_del(&framer->cdev);
		cancel_work_sync(&framer->lineautoneg_task);
		cancel_work_sync(&framer->protoautoneg_task);
		cancel_work_sync(&framer->ethautoneg_task);
		cancel_work_sync(&framer->allautoneg_task);
		device_destroy(cpri_class, framer->dev_t);
		unregister_chrdev_region(framer->dev_t, 1);
		if (framer->autoneg_param.eth_rates != NULL)
			kfree(framer->autoneg_param.eth_rates);
		kfree(framer);
	}

	/* Removing framer container */
	kfree(cpri_dev->framer);

	/* Removing the tasklet */
	tasklet_kill(cpri_dev->err_tasklet_frmr0);
	tasklet_kill(cpri_dev->err_tasklet_frmr1);

	/* Deleting the cpri device from list */
	raw_spin_lock(&cpri_list_lock);
	list_for_each_safe(pos, nx, &cpri_dev_list) {
		cpdev = list_entry(pos, struct cpri_dev, list);
		if (cpdev == cpri_dev) {
			list_del(&cpdev->list);
			kfree(cpdev);
			break;
		}
	}
	raw_spin_unlock(&cpri_list_lock);

	dev_set_drvdata(cpri_dev->dev, NULL);

	kfree(cpri_dev);

	return 0;
}

static struct platform_driver cpri_driver = {
	.driver = {
		.name = "cpri",
		.owner = THIS_MODULE,
		.of_match_table = cpri_match,
	},
	.probe = cpri_probe,
	.remove = cpri_remove,
};

static int __init cpri_mod_init(void)
{
	int rc;

	cpri_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(cpri_class)) {
		pr_err("class_create() failed for cpri class dev\n");
		rc = PTR_ERR(cpri_class);
		return rc;
	}

	return platform_driver_register(&cpri_driver);
}

static void __exit cpri_exit(void)
{
	struct list_head *pos, *nx;
	struct cpri_dev *cpdev = NULL;

	platform_driver_unregister(&cpri_driver);

	class_destroy(cpri_class);

	raw_spin_lock(&cpri_list_lock);
	list_for_each_safe(pos, nx, &cpri_dev_list) {
		cpdev = list_entry(pos, struct cpri_dev, list);
		list_del(&cpdev->list);
		kfree(cpdev);
	}
	raw_spin_unlock(&cpri_list_lock);
}

module_init(cpri_mod_init);
module_exit(cpri_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("cpri driver");

/*
 * drivers/rf/cpri/cpri.c
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
#include <linux/of_address.h>
#include <linux/pid.h>

#include "cpri.h"

#define CLASS_NAME	"cp"

static struct class *cpri_class;

static LIST_HEAD(cpri_dev_list);
raw_spinlock_t cpri_list_lock;

static int cpri_open(struct inode *inode, struct file *fp)
{
	struct cpri_framer *framer = NULL;
	int rc = 0;
	int fr_id;

	fr_id = iminor(inode);
	framer = container_of(inode->i_cdev, struct cpri_framer, cdev);
	if (framer != NULL)
		fp->private_data = framer;
	else
		rc = -ENODEV;

	return rc;
}

static int cpri_release(struct inode *inode, struct file *fp)
{
	struct cpri_framer *framer = (struct cpri_framer *)fp->private_data;

	if (!framer)
		return -ENODEV;

	device_destroy(cpri_class, framer->dev_t);
	cdev_del(&framer->cdev);

	return 0;
}

static const struct file_operations cpri_fops = {
	.owner = THIS_MODULE,
	.open = cpri_open,
	.unlocked_ioctl = cpri_ioctl,
	.release = cpri_release,
};

static irqreturn_t cpri_rxtiming(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_HFN_MASK | IEVENT_BFN_MASK;
	events = cpri_reg_get_val(&framer->regs_lock,
			&framer->regs->cpri_revent, mask);

	if (events & IEVENT_HFN_MASK)
		framer->stats.rx_hfn++;
	else if (events & IEVENT_BFN_MASK)
		framer->stats.rx_bfn++;

	/* Clear event by writing 1 */
	cpri_reg_set_val(&framer->regs_lock, &framer->regs->cpri_revent,
				events, events);

	return IRQ_HANDLED;
}

static irqreturn_t cpri_txtiming(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_HFN_MASK | IEVENT_BFN_MASK;
	events = cpri_reg_get_val(&framer->regs_lock,
			&framer->regs->cpri_tevent, mask);

	if (events &  IEVENT_HFN_MASK)
		framer->stats.tx_hfn++;
	if (events &  IEVENT_BFN_MASK)
		framer->stats.tx_bfn++;

	/* Clear event by writing 1 */
	cpri_reg_set_val(&framer->regs_lock, &framer->regs->cpri_tevent,
				events, events);

	return IRQ_HANDLED;
}

static irqreturn_t cpri_txcontrol(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_ETH_MASK | IEVENT_VSS_THRESHOLD_MASK;
	events = cpri_reg_get_val(&framer->regs_lock,
			&framer->regs->cpri_tevent, mask);

	/* Handle tx ethernet event - Called function will disable the
	 * the interrupt and schedules for bottom half. Interrupt will
	 * be enabled once the tx packets are processed
	 */
	if (events &  IEVENT_ETH_MASK)
		cpri_eth_handle_tx(framer);

	/* TODO: Handle VSS threshold event here */

	/* Clear events */
	cpri_reg_set_val(&framer->regs_lock, &framer->regs->cpri_tevent,
				events, events);

	return IRQ_HANDLED;
}

static irqreturn_t cpri_rxcontrol(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_ETH_MASK | IEVENT_VSS_THRESHOLD_MASK;
	events = cpri_reg_get_val(&framer->regs_lock,
			&framer->regs->cpri_revent, mask);

	/* Handle rx ethernet event - Called function will disable the
	 * the interrupt and schedules for bottom half. Interrupt will
	 * be enabled once the rx packets are processed
	 */
	if (events & IEVENT_ETH_MASK)
		cpri_eth_handle_rx(framer);

	/* TODO: Handle VSS threshold event here */

	/* Clear events */
	cpri_reg_set_val(&framer->regs_lock, &framer->regs->cpri_revent,
				events, events);

	return IRQ_HANDLED;
}

static int cpri_register_framer_irqs(struct cpri_framer *framer)
{
	int err;

	err = request_irq(framer->irq_rx_c, cpri_rxcontrol, 0,
			"rxcontrol interrupt", framer);
	if (err < 0)
		goto out;

	err = request_irq(framer->irq_tx_c , cpri_txcontrol, 0,
			"txcontrol interrupt", framer);
	if (err < 0)
		goto out;

	err = request_irq(framer->irq_rx_t, cpri_rxtiming, 0,
			"rxtiming interrupt", framer);
	if (err < 0)
		goto out;

	err = request_irq(framer->irq_tx_t, cpri_txtiming, 0,
			"txtiming interrupt", framer);
	if (err < 0)
		goto out;

	return 0;

out:
	dev_err(framer->cpri_dev->dev, "can't get IRQ\n");
	return err;
}

static void cpri_configure_irq_events(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;

	/* Enable all control interrupt events */
	cpri_reg_set(&framer->regs_lock,
			&regs->cpri_rctrl,
			ETH_EN_MASK | VSS_EN_MASK | IQ_EN_MASK);

	cpri_reg_set(&framer->regs_lock,
			&regs->cpri_tctrl,
			ETH_EN_MASK | VSS_EN_MASK | IQ_EN_MASK);

	/* Enable all control & timing interrupt events */
	cpri_reg_set(&framer->regs_lock,
			&regs->cpri_rctrltiminginten,
			BFN_TIMING_EVENT_EN_MASK \
			| HFN_TIMING_EVENT_EN_MASK \
			| ETH_EVENT_EN_MASK \
			| VSS_EVENT_EN_MASK);

	cpri_reg_set(&framer->regs_lock,
			&regs->cpri_tctrltiminginten,
			BFN_TIMING_EVENT_EN_MASK \
			| HFN_TIMING_EVENT_EN_MASK \
			| ETH_EVENT_EN_MASK \
			| VSS_EVENT_EN_MASK);

	/* TBD: CPRIICR is not set in this driver. It is not clear
	 * why we have this physical interrupt line and the similar
	 * configuration like the above
	 */

	/* Enable all error events by default */
	cpri_reg_set(&framer->regs_lock,
			&regs->cpri_errinten,
			CPRI_ERR_EVT_ALL);
}

/*
 * Framer state update during error events. This update is used
 * by autoneg code to determine the proper entry in to the autoneg
 * state
 */
static void do_framer_state_update(struct cpri_framer *framer,
			unsigned long mask)
{
	if ((mask & IEVENT_RLOS)
		| (mask & IEVENT_RLOF)
		| (mask & IEVENT_RAI)
		| (mask & IEVENT_RSDI)) {
		if (timer_pending(&framer->l1_timer))
			del_timer_sync(&framer->l1_timer);
		framer->framer_state = L1INBAND_ERROR;
	}

	if ((mask & IEVENT_LLOS) | (mask & IEVENT_LLOF))
		framer->framer_state = SFP_DETACHED;

	if (mask & IEVENT_RRE) {
		framer->framer_state = REC_RESET;
		if (timer_pending(&framer->l1_timer))
			del_timer_sync(&framer->l1_timer);
	}

	if (mask & IEVENT_FAE)
		framer->framer_state = SERDES_ERROR;
}

/* Stats update during error events */
static void do_err_stats_update(struct cpri_framer *framer,
			unsigned long mask)
{
	struct device *dev = framer->cpri_dev->dev;

	if (mask & IEVENT_RX_IQ_OVERRUN)
		framer->stats.rx_iq_overrun_err_count++;
	else if (mask & IEVENT_TX_IQ_UNDERRUN)
		framer->stats.tx_iq_underrun_err_count++;
	else if (mask & IEVENT_RX_ETH_MEM_OVERRUN)
		framer->stats.rx_eth_mem_overrun_err_count++;
	else if (mask & IEVENT_TX_ETH_UNDERRUN)
		framer->stats.tx_eth_underrun_err_count++;
	else if (mask & IEVENT_RX_ETH_BD_UNDERRUN)
		framer->stats.rx_eth_bd_underrun_err_count++;
	else if (mask & IEVENT_RX_HDLC_OVERRUN)
		framer->stats.rx_hdlc_overrun_err_count++;
	else if (mask & IEVENT_TX_HDLC_UNDERRUN)
		framer->stats.tx_hdlc_underrun_err_count++;
	else if (mask & IEVENT_RX_HDLC_BD_UNDERRUN)
		framer->stats.rx_hdlc_bd_underrun_err_count++;
	else if (mask & IEVENT_RX_VSS_OVERRUN)
		framer->stats.rx_vss_overrun_err_count++;
	else if (mask & IEVENT_TX_VSS_UNDERRUN)
		framer->stats.tx_vss_underrun_err_count++;
	else if (mask & IEVENT_ECC_CONFIG_MEM)
		framer->stats.ecc_config_mem_err_count++;
	else if (mask & IEVENT_ECC_DATA_MEM)
		framer->stats.ecc_data_mem_err_count++;
	else if (mask & IEVENT_RX_ETH_DMA_OVERRUN)
		framer->stats.rx_eth_dma_overrun_err_count++;
	else if (mask & IEVENT_ETH_FORWARD_REM_FIFO_FULL)
		framer->stats.eth_fwd_rem_fifo_full_err_count++;
	else if (mask & IEVENT_EXT_SYNC_LOSS)
		framer->stats.ext_sync_loss_err_count++;
	else if (mask & IEVENT_RLOS)
		framer->stats.rlos_err_count++;
	else if (mask & IEVENT_RLOF)
		framer->stats.rlof_err_count++;
	else if (mask & IEVENT_RAI)
		framer->stats.rai_err_count++;
	else if (mask & IEVENT_RSDI)
		framer->stats.rsdi_err_count++;
	else if (mask & IEVENT_LLOS)
		framer->stats.llos_err_count++;
	else if (mask & IEVENT_LLOF)
		framer->stats.llof_err_count++;
	else if (mask & IEVENT_RRE)
		framer->stats.rr_err_count++;
	else if (mask & IEVENT_FAE)
		framer->stats.fa_err_count++;
	else if (mask & IEVENT_RRA)
		framer->stats.rra_err_count++;
	else
		dev_err(dev, "Invalid mask during stats update\n");
}

static void cpri_err_tasklet(unsigned long arg)
{
	struct cpri_dev *dev = (struct cpri_dev *) arg;
	struct cpri_framer *framer;
	u32 err_status, err_evt;
	int i;

	/* Check which framer has got error */
	err_status = cpri_reg_get_val(&dev->lock, &dev->regs->cpri_errstatus,
				C1_ERR_MASK | C2_ERR_MASK);

	if (err_status & C1_ERR_MASK) {
		framer = dev->framer[0];
		err_evt = cpri_reg_get_val(&framer->regs_lock,
				&framer->regs->cpri_errevent, CPRI_ERR_EVT_ALL);
	} else {
		framer = dev->framer[1];
		err_evt = cpri_reg_get_val(&framer->regs_lock,
				&framer->regs->cpri_errevent, CPRI_ERR_EVT_ALL);
	}

	/* Handle the error events */
	for (i = 0; i < dev->framers; i++) {
		framer = dev->framer[i];
		/* TBD: Process events and notify user
		 * synchronously for all the framers here.
		 */


		/* Update stats */
		do_err_stats_update(framer, err_evt);

		/* Update state */
		do_framer_state_update(framer, err_evt);

		/* Handle ethernet error if any */
		cpri_eth_handle_error(framer);

		/* Restore the interrupt mask */
		cpri_reg_set_val(&framer->regs_lock,
			&framer->regs->cpri_errinten, err_evt, err_evt);
	}

	return;
}

static irqreturn_t cpri_err_handler(int irq, void *cookie)
{
	struct cpri_dev *dev = (struct cpri_dev *)cookie;
	struct cpri_framer *framer;
	u32 err_status, err_evt;

	/* Check which framer has got error */
	err_status = cpri_reg_get_val(&dev->lock, &dev->regs->cpri_errstatus,
				C1_ERR_MASK | C2_ERR_MASK);

	if (err_status & C1_ERR_MASK) {
		framer = dev->framer[0];
		err_evt = cpri_reg_get_val(&framer->regs_lock,
				&framer->regs->cpri_errevent, CPRI_ERR_EVT_ALL);
	} else if (err_status & C2_ERR_MASK) {
		framer = dev->framer[1];
		err_evt = cpri_reg_get_val(&framer->regs_lock,
				&framer->regs->cpri_errevent, CPRI_ERR_EVT_ALL);
	} else {
		dev_err(dev->dev, "spurious error interrupt\n");
		return IRQ_NONE;
	}

	/* Disable the error interrupt mask */
	cpri_reg_set_val(&framer->regs_lock, &framer->regs->cpri_errinten,
				err_evt, ~err_evt);

	/* Schedule the bottom-half for handling error event */
	tasklet_schedule(dev->err_tasklet);

	return IRQ_HANDLED;
}

static int cpri_register_irq(struct cpri_dev *cpdev)
{
	int err = 0;

	/* Handling only the error interrupt (111) */
	err = request_irq(cpdev->irq_err, cpri_err_handler, 0,
				"error interrupt", cpdev);
	if (err < 0)
		dev_err(cpdev->dev, "can't get IRQ\n");
	else {
		/* Initialise tasklet dynamically for bottom-half
		 * so that it can be be scheduled when event occurs
		 */
		cpdev->err_tasklet = kmalloc(sizeof(struct tasklet_struct),
					GFP_KERNEL);
		tasklet_init(cpdev->err_tasklet, cpri_err_tasklet,
				(unsigned long) cpdev);
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
	static dev_t dev_t;
	int cpri_major, cpri_minor;
	unsigned long fr_id;
	int rc = 0;

	if (!np || !of_device_is_available(np)) {
		rc = -ENODEV;
		goto err_out;
	}

	/* Allocate cpri device structure */
	cpri_dev = kzalloc(sizeof(struct cpri_dev), GFP_KERNEL);
	if (!cpri_dev) {
		dev_dbg(dev, "Failed to allocate cpri_dev\n");
		rc = -ENOMEM;
		goto err_mem;
	}

	/* Polulate cpri device structure */
	cpri_dev->dev_id = (unsigned long) of_get_property(np,
						"id",
						NULL);
	cpri_dev->dev_name = (char *) of_get_property(np,
						"name",
						NULL);
	cpri_dev->regs = of_iomap(np, 0);
	cpri_dev->dev = dev;
	raw_spin_lock_init(&cpri_dev->lock);
	cpri_dev->irq_gen1 = irq_of_parse_and_map(np, 0);
	cpri_dev->irq_gen2 = irq_of_parse_and_map(np, 1);
	cpri_dev->irq_gen3 = irq_of_parse_and_map(np, 2);
	cpri_dev->irq_gen4 = irq_of_parse_and_map(np, 3);
	cpri_dev->irq_err = irq_of_parse_and_map(np, 4);

	/* Setup cpri device interrupts */
	if (cpri_register_irq(cpri_dev) < 0) {
		dev_err(dev, "cpri dev irq init failure\n");
		goto err_mem;
	}

	/* Allocating dynamic major and minor nos for framer interface */
	rc = alloc_chrdev_region(&dev_t, 0, MAX_FRAMERS_PER_COMPLEX,
					cpri_dev->dev_name);
	cpri_major = MAJOR(dev_t);
	cpri_minor = MINOR(dev_t);
	if (rc < 0) {
		dev_dbg(dev, "alloc_chrdev_region() failed\n");
		unregister_chrdev_region(dev_t, MAX_FRAMERS_PER_COMPLEX);
		goto err_node;
	}

	/* Populate framer strcuture */
	cpri_dev->framers = 0;
	for_each_child_of_node(np, child) {
		fr_id = (unsigned long) of_get_property(child,
						"framer-id",
						NULL);
		dev_info(dev, "framer id: %lu\n", fr_id);
		cpri_dev->framers++; /* counting framers */
		cpri_dev->framer[fr_id] = kzalloc(sizeof(struct cpri_framer),
						GFP_KERNEL);
		framer->id = fr_id;
		framer->regs = of_iomap(child, 0);
		framer = cpri_dev->framer[fr_id];
		framer->cpri_dev = cpri_dev;
		framer->max_axcs = (unsigned int)of_get_property(child,
						"max-axcs", NULL);
		/* TODO: Get AxC buffer size here */
		/* Create cdev for each framer */
		dev_t = MKDEV(cpri_major, cpri_minor + fr_id);
		cdev_init(&framer->cdev, &cpri_fops);
		rc = cdev_add(&framer->cdev, dev_t, 1);
		if (rc < 0) {
			dev_err(dev, "cdev_add() failed\n");
			goto err_node;
		}
		framer->dev_t = dev_t;
		device_create(cpri_class, framer->cpri_dev->dev,
				framer->dev_t, NULL, "%s%lu",
				cpri_dev->dev_name, framer->id);
		raw_spin_lock_init(&framer->regs_lock);
		raw_spin_lock_init(&framer->tx_cwt_lock);
		/* Get the IRQ lines and register them per framer */
		framer->irq_rx_t = irq_of_parse_and_map(np, 0);
		framer->irq_tx_t = irq_of_parse_and_map(np, 1);
		framer->irq_rx_c = irq_of_parse_and_map(np, 2);
		framer->irq_tx_c = irq_of_parse_and_map(np, 3);
		/* Configure framer events */
		cpri_configure_irq_events(framer);
		/* Configure framer events */
		rc = cpri_register_framer_irqs(framer);
		if (rc < 0)
			goto err_node;

		/* Setup ethernet interface */
		if (cpri_eth_init(pdev, framer, child) < 0) {
			dev_err(dev, "ethernet init failed\n");
			goto err_node;
		}
	}

	/* Add to the list of cpri devices - this is required
	 * as the probe is called for multiple cpri complexes
	 */
	raw_spin_lock(&cpri_list_lock);
	list_add_tail(&cpri_dev->list, &cpri_dev_list);
	raw_spin_unlock(&cpri_list_lock);

	dev_set_drvdata(dev, cpri_dev);

	return 0;

err_node:
	cdev_del(&framer->cdev);
	unregister_chrdev_region(dev_t, 1);
	free_irq(cpri_dev->irq_err, cpri_dev);
err_mem:
	kfree(cpri_dev);
err_out:
	dev_err(dev, "cpri probe error\n");

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
		kfree(framer);
	}

	/* Removing the tasklet */
	tasklet_kill(cpri_dev->err_tasklet);

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

	kfree(cpri_dev);

	dev_set_drvdata(cpri_dev->dev, NULL);

	return 0;
}

static struct of_device_id cpri_match[] = {
	{.compatible = "fsl,cpri",},
	{},
};

static struct platform_driver cpri_driver = {
	.driver = {
		.name = "fsl-cpri",
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

	class_destroy(cpri_class);

	raw_spin_lock(&cpri_list_lock);
	list_for_each_safe(pos, nx, &cpri_dev_list) {
		cpdev = list_entry(pos, struct cpri_dev, list);
		list_del(&cpdev->list);
		kfree(cpdev);
	}
	raw_spin_unlock(&cpri_list_lock);

	platform_driver_unregister(&cpri_driver);
}

module_init(cpri_mod_init);
module_exit(cpri_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("cpri driver");

/*
 * drivers/net/cpri/cpri.c
 * CPRI device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
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
#include <linux/gpio.h>
#include <trace/events/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>


#include <linux/cpri.h>
#include <linux/qixis.h>

static struct of_device_id cpri_match[] = {
	{.compatible = "fsl,d4400-cpri",},
	{.compatible = "fsl,medusa-cpri",},
	{},
};

struct cpri_priv *priv;

struct cpri_dev *get_pair_cpri_dev(struct cpri_dev *cpri_dev)
{
	struct cpri_dev *cpri_dev_pair = NULL;

	list_for_each_entry(cpri_dev_pair, &priv->cpri_dev_list, list) {
		if ((cpri_dev_pair) && (cpri_dev_pair->dev_id !=
					cpri_dev->dev_id))
			return cpri_dev_pair;
	}
	return NULL;
}

struct cpri_framer *get_attached_cpri_dev(struct device_node **sfp_dev_node)
{
	struct cpri_dev *cpri_dev = NULL;
	struct cpri_framer *framer = NULL;
	struct sfp_dev *sfp_dev;
	struct device_node *node = NULL;
	int i;

	spin_lock(&priv->cpri_list_lock);
	list_for_each_entry(cpri_dev, &priv->cpri_dev_list, list) {

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
			break;
		}
	}
	spin_unlock(&priv->cpri_list_lock);

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
		dev_dbg(framer->cpri_dev->dev, "framer id:%d", framer->id);
	} else
		rc = -ENODEV;

	return rc;
}

static void sfp_read_pins(struct work_struct *work)
{
	struct cpri_framer *framer =
		container_of(work, struct cpri_framer, sfp_wq);
	struct sfp_dev *sfp = framer->sfp_dev;

	if (gpio_get_value_cansleep(sfp->prs))
		atomic_inc(&framer->err_cnt[SFP_PRESENCE_BITPOS]);
	if (gpio_get_value_cansleep(sfp->rxlos))
		atomic_inc(&framer->err_cnt[SFP_RXLOS_BITPOS]);
	if (gpio_get_value_cansleep(sfp->txfault))
		atomic_inc(&framer->err_cnt[SFP_TXFAULT_BITPOS]);

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

/* CPRI SFP interrupt handler.
 * Read the interrupt gpio pins to clear
 * the interrupt.
 */
static void handle_sfp_irq(struct work_struct *work)
{
	struct cpri_priv *priv =
			container_of(work, struct cpri_priv, sfp_irq_wq);
	struct cpri_dev *cpri_dev = NULL;
	struct cpri_framer *framer;
	struct sfp_dev *sfp;
	int i;

	list_for_each_entry(cpri_dev, &priv->cpri_dev_list, list) {
		for (i = 0; i < cpri_dev->framers; i++) {
			framer = cpri_dev->framer[i];
			sfp = framer->sfp_dev;
			/*TODO: pca9555 is not accessible from i2c5 node */
			/* This is to unmask the interrupt line gpio 15 */
				gpio_get_value_cansleep(sfp->prs);
				gpio_get_value_cansleep(sfp->rxlos);
				gpio_get_value_cansleep(sfp->txfault);
			}
		}
}

/* CPRI rx timing interrupt handler
 * Right now we are not doing anything
 * It can handle rx HFN and BFN interrupts, if needed
 */
static irqreturn_t cpri_rxtiming(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_HFN_MASK | IEVENT_BFN_MASK;
	events = cpri_read(&framer->regs->cpri_revent) & mask;

	/* Clear event by writing 1 */
	cpri_reg_write(&framer->regs_lock,
		&framer->regs->cpri_revent, events, events);

	return IRQ_HANDLED;
}

/* CPRI tx timing interrupt handler
 * Right now we are not doing anything
 * It can handle tx HFN and BFN interrupts, if needed
 */
static irqreturn_t cpri_txtiming(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask = 0;

	mask = IEVENT_HFN_MASK | IEVENT_BFN_MASK;
	events = cpri_read(&framer->regs->cpri_tevent) & mask;

	/* Clear event by writing 1 */
	cpri_reg_write(&framer->regs_lock,
		&framer->regs->cpri_tevent, events, events);
	return IRQ_HANDLED;
}

/* CPRI tx control interrupt handler
 * Right now we are handling only the RETHE interrupt from eth
 * It can handle RETHE, RHDLCE, RVSSE, if needed
 * The RETHE is enabled by "ifconfig".
 */
static irqreturn_t cpri_txcontrol(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask;

	mask = IEVENT_ETH_MASK | IEVENT_HDLC_MASK | IEVENT_VSS_THRESHOLD_MASK;
	events = cpri_read(&framer->regs->cpri_tevent) & mask;

	/* Handle tx ethernet event - Called function will disable the
	 * the interrupt and schedules for bottom half. Interrupt will
	 * be enabled once the tx packets are processed
	 */
	if (events & IEVENT_ETH_MASK) {
		cpri_reg_write(&framer->regs_lock,
				&framer->regs->cpri_tctrltiminginten,
				ETH_EVENT_EN_MASK, ~ETH_EVENT_EN_MASK);
		cpri_eth_handle_tx(framer);
	}

	/* Clear events */
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tevent,
				events, events);

	return IRQ_HANDLED;
}

/* CPRI rx control interrupt handler
 * Right now we are handling only the TETHE interrupt from eth
 * It can handle TETHE, THDLCE, TVSSE, if needed
 * The TETHE is enabled by "ifconfig"
 */
static irqreturn_t cpri_rxcontrol(int irq, void *cookie)
{
	struct cpri_framer *framer = (struct cpri_framer *)cookie;
	u32 events;
	u32 mask;

	mask = IEVENT_ETH_MASK | IEVENT_HDLC_MASK | IEVENT_VSS_THRESHOLD_MASK;
	events = cpri_read(&framer->regs->cpri_revent) & mask;

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

	/* Clear events */
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_revent,
				events, events);

	return IRQ_HANDLED;
}

static int cpri_register_framer_irqs(struct cpri_framer *framer)
{
	int err;

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
	dev_err(framer->cpri_dev->dev, "can't get IRQ");
	return err;
}

/* Disable and clear all possible interrupt sources.
 * Will be enabled according to user need.
 */
void cpri_mask_irq_events(struct cpri_framer *framer)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct cpri_common_regs __iomem *comm_regs = framer->cpri_dev->regs;
	u32 mask;

	int i;

	/* Disable CPRI timing interrupt */
	cpri_reg_clear(&regs->cpri_rctrltiminginten,
			MASK_ALL);

	cpri_reg_clear(&regs->cpri_tctrltiminginten,
			MASK_ALL);

	/* Disable CPRIICRy interrupt */
	for (i = 0; i < 4; i++)
		cpri_reg_clear(&comm_regs->cpri_intctrl[i],
			MASK_ALL);

	/* Disable CPRI err interrupt */
	cpri_reg_clear(&regs->cpri_errinten,
			MASK_ALL);

	mask = IEVENT_ETH_MASK | IEVENT_HDLC_MASK |
		IEVENT_VSS_THRESHOLD_MASK |
		IEVENT_HFN_MASK | IEVENT_BFN_MASK;

	cpri_write(mask, &framer->regs->cpri_revent);
	cpri_write(mask, &framer->regs->cpri_tevent);

	cpri_write(CPRI_ERR_EVT_ALL,
		&framer->regs->cpri_errevent);


}

/* Register timing and control interrutps */
static int process_framer_irqs(struct device_node *child,
		struct cpri_framer *framer)
{
	struct device *dev = framer->cpri_dev->dev;
	/* Get the IRQ lines and register them per framer */
	framer->irq_rx_t = irq_of_parse_and_map(child, 0);
	framer->irq_tx_t = irq_of_parse_and_map(child, 1);
	framer->irq_rx_c = irq_of_parse_and_map(child, 2);
	framer->irq_tx_c = irq_of_parse_and_map(child, 3);
	dev_dbg(dev, "irq parsed: %d, %d, %d, %d",
			framer->irq_rx_t,
			framer->irq_tx_t,
			framer->irq_rx_c,
			framer->irq_tx_c);

	return cpri_register_framer_irqs(framer);
}



/* Stats update during error events */
static void do_err_stats_update(struct cpri_framer *framer,
			unsigned long err_evt)
{
	unsigned long mask = framer->cpri_enabled_monitor & err_evt;
	int i;

	for (i = 0; i < CPRI_USER_MONITOR_START; i++) {
		/* The RRE and RRA should be treated differently now
		 * due to the chip bug.
		 */
		if (mask & (RRE | RRA)) {
			if (framer->autoneg_params.mode & RE_MODE_SLAVE)
				atomic_inc(&framer->err_cnt[RRE_BITPOS]);
			else
				atomic_inc(&framer->err_cnt[RRA_BITPOS]);
		}

		/* Update others */
		mask &= (~(RRE | RRA));
		if (mask & (1 << i))
			atomic_inc(&framer->err_cnt[i]);
	}
}

static void cpri_err_tasklet(unsigned long arg)
{
	struct cpri_framer *framer = (struct cpri_framer *) arg;
	u32 err_evt;
	u32 mask;

	err_evt = cpri_read(&framer->regs->cpri_errevent);
	mask = framer->cpri_enabled_monitor & CPRI_ERR_EVT_ALL;

	/* This read operation is required by RM */
	if (err_evt & RX_ETH_MEM_OVERRUN)
		cpri_read(&framer->regs->cpri_rethexstatus);

	cpri_reg_write(&framer->regs_lock,
		&framer->regs->cpri_errevent,
		err_evt, err_evt);

	do_err_stats_update(framer, err_evt);

	/* Handle ethernet error if any */
	if (framer->frmr_ethflag == CPRI_ETH_SUPPORTED)
		cpri_eth_handle_error(framer);

	/* Restore the interrupt mask */
	err_evt = mask & err_evt;
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_errinten,
			err_evt, err_evt);
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

		tasklet_schedule(&dev->err_tasklet_frmr0);

	} else if (err_status & C2_ERR_MASK) {
		framer = dev->framer[1];
		err_evt = cpri_reg_get_val(
			&framer->regs->cpri_errevent, CPRI_ERR_EVT_ALL);
		/* Disable the error interrupt mask */
		cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_errinten,
			err_evt, (err_evt ^ err_evt));

		tasklet_schedule(&dev->err_tasklet_frmr1);

	}
	return IRQ_HANDLED;
}

static irqreturn_t cpri_sfp_int_handler(int irq, void *cookie)
{
	schedule_work(&priv->sfp_irq_wq);
	return IRQ_HANDLED;
}

static int cpri_init_sfp_irq(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	INIT_WORK(&priv->sfp_irq_wq, handle_sfp_irq);

	priv->sfp_int = of_get_named_gpio(np,
			"sfp-plug-int", 0);
	if (priv->sfp_int < 0) {
		dev_err(dev, "Fail to get SFP i2c expander int gpio#");
		goto out1;
	}

	ret = gpio_request(priv->sfp_int, "sfp-int");
	if (ret) {
		dev_err(dev, "gpio_request fail");
		goto out1;
	}
	gpio_direction_input(priv->sfp_int);

	priv->sfp_irq = gpio_to_irq(priv->sfp_int);
	if (priv->sfp_irq < 0) {
		dev_err(dev, "gpio_to_irq fail");
		goto out2;
	}

	ret = request_irq(priv->sfp_irq, cpri_sfp_int_handler,
		IRQF_TRIGGER_FALLING, "sfp-int", priv);
	if (ret) {
		dev_err(dev, "request_irq fail");
		goto out2;
	}

	return 0;
out2:
	gpio_free(priv->sfp_int);
out1:
	dev_err(dev, "%s failed", __func__);
	return -EFAULT;
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
		dev_err(cpdev->dev, "can't get IRQ");
	else {
		/* framer-0 tasklet */
		tasklet_init(&cpdev->err_tasklet_frmr0, cpri_err_tasklet,
				(unsigned long) cpdev->framer[0]);
		/* framer-1 tasklet */
		tasklet_init(&cpdev->err_tasklet_frmr1, cpri_err_tasklet,
				(unsigned long) cpdev->framer[1]);
	}

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
	int i, rc = 0;
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
	if (cpri_dev == NULL) {
		dev_err(dev, "cpri_dev alloc fail");
		rc = -ENOMEM;
		goto err_out;
	}

	/* Populate cpri device structure */
	rc = of_property_read_u32(np, "id", &cpri_dev->dev_id);
	if (rc) {
		dev_err(dev, "Failed to get cpri id");
		goto err_mem;
	}

	sprintf(dev_name, "%s%d", DEV_NAME, cpri_dev->dev_id);

	cpri_dev->regs = of_iomap(np, 0);
	if (!cpri_dev->regs) {
		dev_err(dev, "Failed to map cpri reg addr");
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
		dev_dbg(dev, "alloc_chrdev_region() failed");
		goto err_chrdev;
	}
	cpri_major = MAJOR(devt);
	cpri_minor = MINOR(devt);

	cpri_dev->framers = 0;

	max_framers = MAX_FRAMERS_PER_COMPLEX;
	cpri_dev->framer = kmalloc((sizeof(struct cpri_framer *) * max_framers),
					GFP_KERNEL);
	if (!cpri_dev->framer) {
		dev_err(dev, "Failed to allocate framer container");
		rc = -ENOMEM;
		goto err_mem;
	}
	common_regs = cpri_dev->regs;

	for_each_child_of_node(np, child) {

		rc = of_property_read_u32(child, "framer-id", &framer_id);
		if (rc) {
			dev_err(dev, "Failed to get framer id");
			goto err_chrdev;
		}

		cpri_dev->framers++;

		cpri_dev->framer[framer_id] =
			kzalloc(sizeof(struct cpri_framer), GFP_KERNEL);
		if (cpri_dev->framer[framer_id] == NULL) {
			dev_err(dev, "Failed to allocate framer");
			rc = -ENOMEM;
			goto err_chrdev;
		}

		/* Populate framer strcuture */
		framer = cpri_dev->framer[framer_id];
		framer->cpri_dev = cpri_dev;
		framer->cpri_node = np;
		framer->id = framer_id;

		framer->regs = of_iomap(child, 0);
		if (!framer->regs) {
			dev_err(dev, "Failed to map framer reg addr");
			rc = -ENOMEM;
			goto err_chrdev;
		}
		dev_dbg(dev, "framer->regs ---> %p", framer->regs);
		/* Enable clock for framer register access */
		if (framer->id == 1)
			cpri_reg_set(
				&common_regs->cpri_ctrlclk, C2_CLK_MASK);
		else
			cpri_reg_set(
				&common_regs->cpri_ctrlclk, C1_CLK_MASK);

		rc = of_property_read_u32(child, "max-axcs",
			(u32 *)&framer->max_axcs);
		if (rc) {
			dev_err(dev, "Failed to get max axc");
			goto err_chrdev;
		}

		rc = of_property_read_u32(child, "memblk-size",
			(u32 *)&framer->axc_memblk_size);
		if (rc) {
			dev_err(dev, "Failed to get memblk-size");
			goto err_chrdev;
		}

		/* Create cdev for each framer */
		framer->dev_t = MKDEV(cpri_major, cpri_minor + framer_id);
		cdev_init(&framer->cdev, &cpri_fops);
		framer->cdev.owner = THIS_MODULE;
		rc = cdev_add(&framer->cdev, framer->dev_t, 1);
		if (rc < 0) {
			dev_err(dev, "cdev_add failed");
			goto err_chrdev;
		}
		device_create(priv->cpri_class,
			framer->cpri_dev->dev, framer->dev_t,
			NULL, "%s_frmr%d", dev_name, framer->id);

		spin_lock_init(&framer->regs_lock);
		spin_lock_init(&framer->err_en_lock);
		spin_lock_init(&framer->rx_cw_lock);
		spin_lock_init(&framer->tx_cw_lock);
		sema_init(&framer->axc_sem, 1);

		INIT_WORK(&framer->sfp_wq, sfp_read_pins);

		cpri_mask_irq_events(framer);
		if (process_framer_irqs(child, framer) < 0) {
			dev_err(dev, "framer events not supported");
			goto err_cdev;
		}

		framer->sfp_dev_node = of_parse_phandle(child,
					"sfp-handle", 0);
		/* The sfp driver should be init first
		 * otherwise it will return NULL
		 */
		framer->sfp_dev = get_attached_sfp_dev(framer->sfp_dev_node);
		if (framer->sfp_dev == NULL) {
			dev_err(dev, "get_attached_sfp_dev fail");
			goto err_cdev;
		}
		if (cpri_eth_init(pdev, framer, child) < 0)
			dev_err(dev, "ethernet init failed");

	}
	/* Setup cpri err interrupts */
	if (cpri_register_irq(cpri_dev) < 0) {
		dev_err(dev, "cpri dev irq init failure");
		goto err_cdev;
	}
	/* Add to the list of cpri devices - this is required
	 * as the probe is called for multiple cpri complexes
	 */
	spin_lock(&priv->cpri_list_lock);
	list_add_tail(&cpri_dev->list, &priv->cpri_dev_list);
	spin_unlock(&priv->cpri_list_lock);

	dev_set_drvdata(dev, cpri_dev);

	/* Only the CPRI complex 1 has this SFP gpio interrupt entry */
	if (cpri_dev->dev_id == 0)
		cpri_init_sfp_irq(pdev);

	dev_info(dev, "%s done", __func__);

	return 0;

err_cdev:
	cdev_del(&framer->cdev);
	for (i = 0; i < cpri_dev->framers; i++)
		device_destroy(priv->cpri_class, cpri_dev->framer[i]->dev_t);
	class_destroy(priv->cpri_class);
err_chrdev:
	unregister_chrdev_region(devt, MAX_FRAMERS_PER_COMPLEX);
	kfree(cpri_dev->framer);
err_mem:
	kfree(cpri_dev);
err_out:
	dev_err(dev, "%s failure", __func__);

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
		cdev_del(&framer->cdev);
		del_timer(&framer->link_monitor_timer);
		cancel_work_sync(&framer->sfp_wq);
		device_destroy(priv->cpri_class, framer->dev_t);
		unregister_chrdev_region(framer->dev_t, 1);
		kfree(framer);
	}

	/* Removing framer container */
	kfree(cpri_dev->framer);

	/* Removing the tasklet */
	tasklet_kill(&cpri_dev->err_tasklet_frmr0);
	tasklet_kill(&cpri_dev->err_tasklet_frmr1);
	/* Deleting the cpri device from list */
	spin_lock(&priv->cpri_list_lock);
	list_for_each_safe(pos, nx, &priv->cpri_dev_list) {
		cpdev = list_entry(pos, struct cpri_dev, list);
		if (cpdev == cpri_dev) {
			list_del(&cpdev->list);
			kfree(cpdev);
			break;
		}
	}
	spin_unlock(&priv->cpri_list_lock);

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
	priv = kzalloc(sizeof(struct cpri_priv), GFP_KERNEL);
	if (priv == NULL) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	priv->cpri_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(priv->cpri_class)) {
		pr_err("class_create() failed for cpri class dev");
		rc = PTR_ERR(priv->cpri_class);
		kfree(priv);
		return rc;
	}
	INIT_LIST_HEAD(&priv->cpri_dev_list);
	spin_lock_init(&priv->cpri_list_lock);

	return platform_driver_register(&cpri_driver);
}

static void __exit cpri_exit(void)
{
	gpio_free(priv->sfp_int);
	free_irq(priv->sfp_irq, priv);

	platform_driver_unregister(&cpri_driver);

	class_destroy(priv->cpri_class);
	kfree(priv);
}

module_init(cpri_mod_init);
module_exit(cpri_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("cpri driver");

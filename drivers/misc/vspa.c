/*
 * drivers/misc/vspa.c
 * VSPA device driver
 * Author: Vineet Sharma b44341@freescale.co
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
#include <linux/mm.h>
#include <linux/poll.h>

#include <linux/vspa_dev.h>

#define MAX_VSPA_PER_SOC	11
#define VSPA_DEVICE_NAME "vspa"


/* Number of VSPA devices */
static int vspa_devs;
static s32 vspa_major;

module_param(vspa_major, int, 0);
module_param(vspa_devs, int, 0);

static void vspa_configure_irq_events(struct vspa_device *vspadev)
{
	u32 __iomem *regs = vspadev->regs;

	/* Disable all IRQ */
	vspa_reg_write(regs + IRQEN_REG_OFFSET, 0);
}

static int vspa_open(struct inode *inode, struct file *fp)
{
	struct vspa_device *vspadev = NULL;
	int rc = 0;
	int vsap_id;

	vsap_id = iminor(inode);
	vspadev = container_of(inode->i_cdev, struct vspa_device, cdev);
	if (vspadev != NULL) {
		if (vspadev->state < VSPA_OPENED) {
			vspadev->state = VSPA_OPENED;
			fp->private_data = vspadev;
			vspa_configure_irq_events(vspadev);
		} else {
				rc = -EBUSY;
		}
	} else {
		rc = -ENODEV;
	}

	return rc;
}

static int vspa_release(struct inode *inode, struct file *fp)
{
	struct vspa_device *device = (struct vspa_device *)fp->private_data;

	if (!device)
		return -ENODEV;

	device->state =	VSPA_CLOSED;
	return 0;
}

static irqreturn_t vspa_irq_handler(int irq, void *dev)
{
	struct vspa_device *vspadev = (struct vspa_device *)dev;

	spin_lock(&vspadev->lock);
	vspadev->status_reg =
		vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET);
	if (vspadev->status_reg > 0) {
		wake_up_interruptible(&vspadev->irq_wait_q);
		/* disable irqs */
		vspa_reg_write(vspadev->regs + IRQEN_REG_OFFSET, 0);
	}
	spin_unlock(&vspadev->lock);
	return IRQ_HANDLED;
}

static int vspa_register_irqs(struct vspa_device *vspadev)
{
	int rc = 0;

#ifdef FSL_MEDUSA
	rc = request_irq(vspadev->irq_vspa_done, vspa_irq_handler,
		0, VSPA_DEVICE_NAME, vspadev);
	if (rc < 0)
		return rc;

	rc = request_irq(vspadev->irq_vspa_msg , vspa_irq_handler,
		0, VSPA_DEVICE_NAME, vspadev);
	if (rc < 0)
		return rc;

	rc = request_irq(vspadev->irq_dma_cmp, vspa_irq_handler,
		0, VSPA_DEVICE_NAME, vspadev);
	if (rc < 0)
		return rc;

	rc = request_irq(vspadev->irq_dma_err, vspa_irq_handler,
		0, VSPA_DEVICE_NAME, vspadev);
	if (rc < 0)
		return rc;
#else
	rc = request_irq(vspadev->vsap_irq_no, vspa_irq_handler,
		0, VSPA_DEVICE_NAME, vspadev)
	if (rc < 0)
		return rc;
#endif
	return 0;
}

static long vspa_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	int rc = 0;
	struct vspa_device *dev = (struct vspa_device *)filp->private_data;

	switch (cmd) {
	case IOCTL_REQ_IRQ:
		rc = vspa_register_irqs(dev);
		return rc;

	default:
		return -EINVAL;
	}
}

static unsigned int vspa_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct vspa_device *dev = (struct vspa_device *)filp->private_data;

	poll_wait(filp, &dev->irq_wait_q, wait);
	if (vspa_reg_read(dev->regs + STATUS_REG_OFFSET) > 0)
		mask = (POLLIN | POLLRDNORM);

	return mask;
}

static int vspa_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long off = 0, phy_addr = 0;
	int rc = 0;
	struct vspa_device *dev = filp->private_data;
	unsigned long start  = 0, len = 0;

	if (NULL == dev)
		return -ENODEV;

	/* Based on Offset we decide, if we map the vspa registers
	 * or vspa DBG registers */
	if (vma->vm_pgoff == VSPA_DBG_OFFSET) {

		phy_addr = (unsigned long)dev->dbg_start;

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + dev->dbg_size);

	} else if (vma->vm_pgoff == VSPA_REG_OFFSET) {

		phy_addr = (unsigned long)dev->mem_start;

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + dev->mem_size);
	} else {
		return -EINVAL;
	}

	off = vma->vm_pgoff << PAGE_SHIFT;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	vma->vm_flags |= VM_IO;

	rc = io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
	if (rc < 0)
		return -EAGAIN;

	dev->state = VSPA_MMAPED;
	return 0;
}

static const struct file_operations vspa_fops = {
	.owner = THIS_MODULE,
	.open = vspa_open,
	.poll = vspa_poll,
	.unlocked_ioctl = vspa_ioctl,
	.mmap = vspa_mmap,
	.release = vspa_release,
};

static int
vspa_getdts_properties(struct device_node *np, struct vspa_device *data)
{
	u32 prop  = 0;

	if (!np)
		return -EINVAL;

	if (of_property_read_u32(np, "vsparegstart", &prop) < 0)
			return -EINVAL;
		data->mem_start = (u32 *)prop;

	if (of_property_read_u32(np, "vspareglen", &prop) < 0)
			return -EINVAL;
		data->mem_size = prop;

	if (of_property_read_u32(np, "vspadev-id", &prop) < 0)
			return -EINVAL;
		data->id = prop;

	pr_debug("vspadev-id %d\n", data->id);
	pr_debug("mem_start %p\n", data->mem_start);
	pr_debug("mem_size %x\n", data->mem_size);

	if (of_property_read_u32(np, "dbgregstart", &prop) < 0)
		pr_err("dbgregstart attribute not found for %s%d\n",
				VSPA_DEVICE_NAME, data->id);
	else
		data->dbg_start = (u32 *)prop;

	if (of_property_read_u32(np, "dbgreglen", &prop) < 0)
		pr_err("dbgreglen attribute not found for %s%d\n",
				VSPA_DEVICE_NAME, data->id);
	else
		data->dbg_size = prop;

	pr_debug("dbg_start %p\n", data->dbg_start);
	pr_debug("dbg_size %x\n", data->dbg_size);

#ifndef FSL_MEDUSA
	if (of_get_property(np, "interrupts", NULL)) {
		data->vspa_irq_no = irq_of_parse_and_map(np, 0);
	} else {
		pr_err("Interrupt number not found for %s%d\n",
				 VSPA_DEVICE_NAME, data->id);
			return -EINVAL;
	}
#else
	data->irq_dma_cmp = irq_of_parse_and_map(np, 0);
	data->irq_dma_err = irq_of_parse_and_map(np, 1);
	data->irq_vspa_done = irq_of_parse_and_map(np, 2);
	data->irq_vspa_msg = irq_of_parse_and_map(np, 3);

	pr_info("irq_dma_cmp %d\n", data->irq_dma_cmp);
	pr_info("irq_dma_err %d\n", data->irq_dma_err);
	pr_info("irq_vspa_done %d\n", data->irq_vspa_done);
	pr_info("irq_vspa_msg %d\n", data->irq_vspa_msg);
#endif
	return 0;
}

static int
vspa_construct_device(struct vspa_device *dev, int minor)
{
	int rc = 0;
	dev_t devno = MKDEV(vspa_major, minor);

	BUG_ON(dev == NULL);

	cdev_init(&dev->cdev, &vspa_fops);
	dev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&dev->cdev, devno, 1);
	if (rc < 0) {
		pr_err("[target] Error %d while trying to add %s%d",
			rc, VSPA_DEVICE_NAME, minor);
		return rc;
	}
	return 0;
}

static int vspa_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct vspa_device *vdev = NULL;
	int minor = vspa_devs;
	dev_t dev;
	int result = 0;

	/* Register our major, and accept a dynamic number. */
	if (vspa_major) {
		dev = MKDEV(vspa_major, minor);
	} else {
		pr_err("VSPA Device %s allocation failed\n",
					vspa_major ? "minor" : "major");
		result = -ENODEV;
		goto reg_fail;
	}

	/* Allocate vspa device structure */
	 vdev = kzalloc(sizeof(struct vspa_device), GFP_KERNEL);
	 if (!vdev) {
		pr_err("Failed to allocate vspa_dev\n");
	    result = -ENOMEM;
	    goto mem_fail;
	 }
	if (vspa_getdts_properties(np, vdev) < 0) {
		pr_err("VSPA DTS entry parse failed.\n");
		result = -EINVAL;
		goto prop_fail;
	}

	result = vspa_construct_device(vdev, vdev->id);
	if (result == 0) {
		vdev->regs = of_iomap(np, 0);
		spin_lock_init(&vdev->lock);
		init_waitqueue_head(&(vdev->irq_wait_q));
		vdev->state = VSPA_PROBED;
		vdev->minor = vspa_devs;
		vspa_devs++;
		pr_info("vspa probe successful\n");
		return 0;
	} else {
		return result;
	}

reg_fail:
	return result;
prop_fail:
	kfree(vdev);
mem_fail:
	unregister_chrdev_region(dev, 1);
	return	result;
}

static int vspa_remove(struct platform_device *ofpdev)
{
	struct  vspa_device *device = dev_get_drvdata(&ofpdev->dev);

	BUG_ON(device == NULL);

	dev_set_drvdata(&ofpdev->dev, NULL);
	iounmap(device->regs);
#ifdef FSL_MEDUSA
	free_irq(device->irq_dma_cmp, device);
	free_irq(device->irq_dma_err, device);
	free_irq(device->irq_vspa_done, device);
	free_irq(device->irq_vspa_msg, device);
#else
	free_irq(device->vspa_irq_no, device);
#endif
	kfree(device);

	unregister_chrdev(vspa_major, VSPA_DEVICE_NAME);
	cdev_del(&device->cdev);
	return 0;
}

static struct of_device_id vspa_match[] = {
	{.compatible = "fsl,vspa",},
	{},
};

static struct platform_driver vspa_driver = {
	.driver = {
		.name = "fsl-vspa",
		.owner = THIS_MODULE,
		.of_match_table = vspa_match,
	},
	.probe = vspa_probe,
	.remove = vspa_remove,
};

static int __init vspa_mod_init(void)
{
	int rc = 0;
	dev_t dev = MKDEV(vspa_major, 0);

	/* Register our major, and accept a dynamic number. */
	rc = alloc_chrdev_region(&dev, 0,
			MAX_VSPA_PER_SOC, VSPA_DEVICE_NAME);
	if (rc < 0) {
		pr_err("Device fsl-vspa allocation failed %d\n", rc);
		return rc;
	}
	vspa_major = MAJOR(dev);

	return platform_driver_register(&vspa_driver);
}

static void __exit vspa_exit(void)
{
	platform_driver_unregister(&vspa_driver);
}

module_init(vspa_mod_init);
module_exit(vspa_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vineet Sharma (b44341@freescale.com)");
MODULE_DESCRIPTION("vspa driver");

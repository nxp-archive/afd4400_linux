/*
 * drivers/misc/vspa.c
 * VSPA device driver
 * Author: Vineet Sharma b44341@freescale.com
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
#include <mach/gpc.h>

/* Number of minor number allocation required */
#define MAX_VSPA 11

#define VSPA_DEVICE_NAME "vspa"


/* Number of VSPA devices probed on system */
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

	vspadev = container_of(inode->i_cdev, struct vspa_device, cdev);
	if (vspadev != NULL) {
		if (vspadev->state < VSPA_OPENED) {
			vspadev->state = VSPA_OPENED;
			fp->private_data = vspadev;
			vspa_configure_irq_events(vspadev);
		} else if (vspadev->state >= VSPA_OPENED) {
			fp->private_data = vspadev;
		}
	} else {
		dev_err(vspadev->dev, "No device context found for %s %d\n",
					VSPA_DEVICE_NAME, iminor(inode));

		rc = -ENODEV;
	}

	return rc;
}

static int vspa_release(struct inode *inode, struct file *fp)
{
	struct vspa_device *vspadev = (struct vspa_device *)fp->private_data;

	if (!vspadev) {
		dev_err(vspadev->dev, "No device context found for %s %d\n",
					VSPA_DEVICE_NAME, iminor(inode));
		return -ENODEV;
	}
	vspadev->state = VSPA_CLOSED;

	return 0;
}

static irqreturn_t vspa_irq_handler(int irq, void *dev)
{
	struct vspa_device *vspadev = (struct vspa_device *)dev;

	spin_lock(&vspadev->lock);
	vspadev->status_reg =
		vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET);
	if (vspadev->status_reg > 0) {
		/* disable irqs */
		vspa_reg_write(vspadev->regs + IRQEN_REG_OFFSET, 0);
		wake_up_interruptible(&vspadev->irq_wait_q);
	}
	spin_unlock(&vspadev->lock);
	return IRQ_HANDLED;
}

static int vspa_register_irqs(struct vspa_device *vspadev)
{
	int rc = 0;

	if (!vspadev->irq_enabled) {
		rc = request_irq(vspadev->vspa_irq_no, vspa_irq_handler,
		0, VSPA_DEVICE_NAME, vspadev);
		if (rc < 0)
			return rc;
		vspadev->irq_enabled = 1;
	}
	return 0;
}

static long vspa_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	int rc = 0;
	struct vspa_device *vspadev = (struct vspa_device *)filp->private_data;

	switch (cmd) {
	case IOCTL_REQ_IRQ:
		rc = vspa_register_irqs(vspadev);
		return rc;

	case IOCTL_REQ_PDN:
		return d4400_gpc_vspa_full_pow_gate(vspadev->id);

	case IOCTL_REQ_PUP:
		return d4400_gpc_vspa_full_pow_up(vspadev->id);

	default:
		return -EINVAL;
	}
}

static unsigned int vspa_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct vspa_device *vspadev = (struct vspa_device *)filp->private_data;

	poll_wait(filp, &vspadev->irq_wait_q, wait);
	if (vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET) > 0)
		mask = (POLLIN | POLLRDNORM);

	return mask;
}

static int vspa_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long off = 0, phy_addr = 0;
	int rc = 0;
	struct vspa_device *vspadev = filp->private_data;
	unsigned long start  = 0, len = 0;

	if (NULL == vspadev)
		return -ENODEV;

	/* Based on Offset we decide, if we map the vspa registers
	 * (VSPA_DBG_OFFSET) 0: VSPA debugg resister set
	 * (VSPA_REG_OFFSET) 4096: VSPA IP register set
	 * (VSPA_DS_OFFSET) 8192: Persiatnt memory allocted by the driver for
	 * meta data
	 * */
	if (vma->vm_pgoff == (VSPA_DBG_OFFSET >> PAGE_SHIFT)) {

		phy_addr = (unsigned long)vspadev->dbg_regs;

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + vspadev->dbg_size);

		/* These are IO addresses */
		vma->vm_flags |= VM_IO;

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	} else if (vma->vm_pgoff == (VSPA_REG_OFFSET >> PAGE_SHIFT)) {

		phy_addr = (unsigned long)vspadev->mem_regs;

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + vspadev->mem_size);

		/* These are IO addresses */
		vma->vm_flags |= VM_IO;

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	} else if (vma->vm_pgoff == (VSPA_DS_OFFSET >> PAGE_SHIFT)) {


		/* Calculate the length */
		len = vma->vm_end - vma->vm_start;

		/* We allocate memory for metadata once only */
		if (vspadev->mem_context == NULL) {

			vspadev->mem_context = kzalloc(len, GFP_KERNEL);

			if (vspadev->mem_context == NULL) {
				dev_err(vspadev->dev, "No memory for device context %s\n",
					VSPA_DEVICE_NAME);
				return -ENOMEM;
			}
		}

		phy_addr = virt_to_phys((void *)vspadev->mem_context);

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + len);

	} else {
		return -EINVAL;
	}

	/* Reset it, as the pg_off is used as index for the type of memory */
	vma->vm_pgoff = 0;

	off = vma->vm_pgoff << PAGE_SHIFT;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	rc = io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
	if (rc < 0)
		return -EAGAIN;

	vspadev->state = VSPA_MMAPED;
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
vspa_getdts_properties(struct device_node *np, struct vspa_device *vsapdev)
{
	u32 prop  = 0;

	if (!np)
		return -EINVAL;

	if (of_property_read_u32(np, "vspadev-id", &prop) < 0)
			return -EINVAL;
		vsapdev->id = prop;

	if (of_property_read_u32(np, "dbgregstart", &prop) < 0) {
		dev_err(vsapdev->dev, "dbgregstart attribute not found for %s%d\n",
				VSPA_DEVICE_NAME, vsapdev->id);
		return -EINVAL;
	}

	vsapdev->dbg_regs = (u32 *)prop;

	if (of_property_read_u32(np, "dbgreglen", &prop) < 0) {
		dev_err(vsapdev->dev, "dbgreglen attribute not found for %s%d\n",
				VSPA_DEVICE_NAME, vsapdev->id);
		return -EINVAL;
	}

	vsapdev->dbg_size = prop;

	if (of_get_property(np, "interrupts", NULL)) {
		vsapdev->vspa_irq_no = irq_of_parse_and_map(np, 0);
	} else {
		dev_err(vsapdev->dev, "Interrupt number not found for %s%d\n",
				 VSPA_DEVICE_NAME, vsapdev->id);
		return -EINVAL;
	}

	return 0;
}

static int
vspa_construct_device(struct vspa_device *vspadev, int minor)
{
	int rc = 0;
	dev_t devno = MKDEV(vspa_major, minor);

	BUG_ON(vspadev == NULL);

	cdev_init(&vspadev->cdev, &vspa_fops);
	vspadev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&vspadev->cdev, devno, 1);
	if (rc < 0) {
		dev_err(vspadev->dev, "[target] Error %d while trying to add %s%d",
			rc, VSPA_DEVICE_NAME, minor);
	}
	return rc;
}

static int vspa_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct vspa_device *vspadev = NULL;
	struct resource res;
	dev_t dev;
	int result = 0;

	/* Allocate vspa device structure */
	vspadev = kzalloc(sizeof(struct vspa_device), GFP_KERNEL);

	if (!vspadev) {
		pr_err("Failed to allocate vspa_dev\n");
		result = -ENOMEM;
		return result;
	}

	vspadev->dev = &pdev->dev;

	if (vspa_getdts_properties(np, vspadev) < 0) {
		dev_err(vspadev->dev, "VSPA DTS entry parse failed.\n");
		result = -EINVAL;
		goto prop_fail;
	}

	/* Register our major, and accept a dynamic number. */
	if (vspa_major) {
		dev = MKDEV(vspa_major, vspadev->id);
	} else {

		/* Register our major, and accept a dynamic number. */
		result = alloc_chrdev_region(&dev, 0,
				MAX_VSPA, VSPA_DEVICE_NAME);
		if (result < 0) {
			dev_err(vspadev->dev, "Device fsl-vspa allocation failed %d\n",
					result);
			goto reg_fail;
		}
		vspa_major = MAJOR(dev);
	}

	result = vspa_construct_device(vspadev, vspadev->id);
	if (result == 0) {
		if (of_address_to_resource(np, 0, &res)) {
			result = -EINVAL;
			goto prop_fail;
		}

		vspadev->mem_regs = (u32 __iomem *)res.start;
		vspadev->mem_size =  resource_size(&res);
		vspadev->regs = ioremap((unsigned int)vspadev->mem_regs,
				vspadev->mem_size);

		spin_lock_init(&vspadev->lock);
		init_waitqueue_head(&(vspadev->irq_wait_q));
		vspadev->state = VSPA_PROBED;
		vspa_devs++;
		dev_dbg(vspadev->dev, "vspa probe successful, id:%d\n", vspadev->id);
		return 0;
	} else {
		goto cons_fail;
	}

cons_fail:
	unregister_chrdev_region(dev, 1);
reg_fail:
prop_fail:
	kfree(vspadev);
	return result;
}

static int vspa_remove(struct platform_device *ofpdev)
{
	struct  vspa_device *vspadev = dev_get_drvdata(&ofpdev->dev);

	BUG_ON(vspadev == NULL);

	/* Disbale all irqs of VSPA */
	vspa_configure_irq_events(vspadev);

	dev_set_drvdata(&ofpdev->dev, NULL);

	iounmap(vspadev->regs);

	if (vspadev->mem_context != NULL)
		kfree(vspadev->mem_context);

	free_irq(vspadev->vspa_irq_no, vspadev);

	unregister_chrdev(vspa_major, VSPA_DEVICE_NAME);
	cdev_del(&vspadev->cdev);
	kfree(vspadev);

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

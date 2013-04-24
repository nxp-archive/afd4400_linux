/* drivers/misc/landshark.c
 *
 * Landshark Driver
 * This driver is designed for support to user space Landshark memory
 * allocator Library.
 *
 * Author: Arpit Goel
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 *  Theory of operation
 *
 *  The driver reads DTS entry and provides this information to user LANDSHARK
 *  library. It also implements mmap() functionality for the physical memory
 *  available from kernel.
 *  */
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/mman.h>
#include <linux/of.h>
#include "landshark.h"

static s32 major;

struct phys_mem_info landshark_phys_mem = {
	.physical_addr = 0,
	.physical_addr_len = 0,
};

static const struct file_operations landshark_fops = {
	.owner		=	THIS_MODULE,
	.llseek		=	NULL,
	.read		=	NULL,
	.write		=	NULL,
	.readdir	=	NULL,
	.poll		=	NULL,
	.unlocked_ioctl	=	landshark_ioctl,
	.mmap		=	landshark_mmap,
	.open		=	landshark_open,
	.flush		=	NULL,
	.release	=	landshark_release,
	.fsync		=	NULL,
	.fasync		=	NULL,
	.lock		=	NULL,
};

long landshark_ioctl(struct file *file,	unsigned int cmd, unsigned long arg)
{
	int result = 0;
	switch (cmd) {
	case LANDSHARK_MEM_INFO_GET:/* for writing data to arg */
		result = copy_to_user((struct phys_mem_info *)arg,
				&landshark_phys_mem,
				sizeof(landshark_phys_mem));
		if (result < 0)
			return result;
		break;
	default:
		result = -EINVAL;
	}
	return result;
}

static s32 landshark_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int result = 0;
	if (landshark_phys_mem.physical_addr_len <
			(vma->vm_end - vma->vm_start)) {
		result = -EINVAL;
		goto return_result;
	}

	/* Marking page as non cached */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	result = remap_pfn_range(vma,
			vma->vm_start,
			landshark_phys_mem.physical_addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);

	if (result != 0)
		result = -EAGAIN;
return_result:
	return result;
}

static s32 landshark_open(struct inode *inode, struct file *file)
{
	MOD_INC_USE_COUNT;
	pr_debug("landshark_dev:: Device open\n");
	return 0;
}

static s32 landshark_release(struct inode *inode, struct file *filp)
{
	MOD_DEC_USE_COUNT;
	pr_debug("landshark_dev:: Device closed\n");
	return 0;
}

static const struct of_device_id landshark_ids[] = {
	{ .compatible = "fsl,landshark"},
	{	}
};


static struct platform_driver landshark_driver = {
	.probe = landshark_probe,
	.remove = landshark_remove,
	.driver = {
		.name = "fsl,landshark",
		.owner = THIS_MODULE,
		.of_match_table = landshark_ids,
	 },
};

MODULE_DEVICE_TABLE(platform, landshark_ids);

static s32 landshark_probe(struct platform_device *ofdev)
{
	struct device_node *np = ofdev->dev.of_node;
	int result = 0;
	long page_size = PAGE_SIZE;

	/*registering LANDSHARK driver*/
	major = register_chrdev(LANDSHARK_MAJOR, MOD_NAME, &landshark_fops);
	if (major < 0) {
		pr_err("LANDSHARK driver registration failed: err %x\n", major);
		result = major;
		goto return_result;
	}
	/*Read DTS entry*/
	if (of_property_read_u32(np, "length",
				&landshark_phys_mem.physical_addr_len) < 0) {
		pr_err("LANDSHARK length not found in Device Tree.\n");
		result = -EINVAL;
		goto return_result;
	}
	if (of_property_read_u32(np, "address",
				&landshark_phys_mem.physical_addr) < 0) {
		pr_err("LANDSHARK address not found in Device Tree.\n");
		result = -EINVAL;
		goto return_result;
	}
	/*Alingning length and address to PAGE_SIZE boundaries*/
	landshark_phys_mem.physical_addr =
		(((int) landshark_phys_mem.physical_addr + page_size - 1)
		& ~(page_size - 1));
	landshark_phys_mem.physical_addr_len =
		(((int) landshark_phys_mem.physical_addr_len)
		 & ~(page_size - 1));

return_result:
	return result;
}

static s32 landshark_remove(struct platform_device *ofdev)
{
	unregister_chrdev(major, MOD_NAME);
	return 0;
}

static s32 __init init_landshark(void)
{
	return platform_driver_register(&landshark_driver);

}

static void __exit exit_landshark(void)
{
	platform_driver_unregister(&landshark_driver);
	return;
}

module_init(init_landshark);
module_exit(exit_landshark);

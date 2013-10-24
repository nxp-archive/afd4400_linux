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
#include <linux/slab.h>
#include "landshark.h"

static s32 major;

/* Information regarding the buffer pool and size */
struct phys_mem_info landshark_phys_mem = {
	.dmaregion_addr = 0,
	.dmaregion_addr_len = 0,
};

/* Information regarding the metadata and its size */
struct pinit_info landshark_pinit_info = {
	.nondma_memory	= 0,
	.pinit_done = 0,
	.pinit_size = 0,
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
	struct pinit_info ls_pinit_info = { 0, 0, 0};

	switch (cmd) {

	case LANDSHARK_MEM_INFO_GET:/* for writing data to arg */
		result = copy_to_user((struct phys_mem_info *)arg,
				&landshark_phys_mem,
				sizeof(landshark_phys_mem));
		break;

	/* This IOCTL is used to set the information regarding
	 * the process intialization of the pool information, it is only
	 * intialized once,and cleared on final exit of the process.
	 * On final exit allocated memory (for metadata) is free'd
	 */
	case LANDSHARK_INIT_INFO_SET:
		result = copy_from_user(
				(struct pinit_info *)&ls_pinit_info,
				(struct pinit_info *)arg,
				sizeof(struct pinit_info));
		if (result < 0)
			return result;

		landshark_pinit_info.pinit_done = ls_pinit_info.pinit_done;

		/* If the request is to release the metadata ? */
		if ((ls_pinit_info.pinit_done == 0)
				&& (ls_pinit_info.pinit_size == 0)) {

			kfree((void *)landshark_pinit_info.nondma_memory);

			landshark_pinit_info.pinit_size = 0;
			landshark_pinit_info.nondma_memory = 0;
		}
		break;

	/* This IOCTL is used by the user library to get the size
	 * of the metadata memory and accordingly the process can
	 * invoke mmap to map the metadata into its address space
	 */
	case LANDSHARK_INIT_INFO_GET:
		result = copy_to_user((struct pinit_info *)arg,
				(struct pinit_info *)&landshark_pinit_info,
				sizeof(struct pinit_info));
		break;

	default:
		result = -EINVAL;
	}
	return result;
}

static s32 landshark_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int result = 0;
	unsigned long phy_addr = 0;
	unsigned long start  = 0, len = 0;


	if (vma->vm_pgoff == 0) {

		if (landshark_phys_mem.dmaregion_addr_len <
				(vma->vm_end - vma->vm_start)) {
			result = -ENOMEM;
			goto return_result;
		}

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

		result = remap_pfn_range(vma,
				vma->vm_start,
				landshark_phys_mem.dmaregion_addr >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot);

	} else /* Allocate and mmap persistent metadata memory */ {

		len = vma->vm_end - vma->vm_start;

		/* Memory for metadat is allocated only once,
		 * and all process refer to the same allocated memory */
		if ((void *)landshark_pinit_info.nondma_memory == NULL) {

			landshark_pinit_info.nondma_memory =
				(unsigned int) kzalloc(len, GFP_KERNEL);

			if ((void *)landshark_pinit_info.nondma_memory == NULL) {
				pr_err("No memory for storing landshark meta data\n");
				return -ENOMEM;
			}
			landshark_pinit_info.pinit_size = len;
		}

		phy_addr = virt_to_phys((void *)landshark_pinit_info.nondma_memory);

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + landshark_pinit_info.pinit_size);

		result = remap_pfn_range(vma,
				vma->vm_start,
				(landshark_pinit_info.nondma_memory >> PAGE_SHIFT),
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot);
	}
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

	/* registering LANDSHARK driver*/
	major = register_chrdev(LANDSHARK_MAJOR, MOD_NAME, &landshark_fops);
	if (major < 0) {
		pr_err("LANDSHARK driver registration failed: err %x\n", major);
		result = major;
		goto return_result;
	}
	/* Read DTS entry*/
	if (of_property_read_u32(np, "length",
				&landshark_phys_mem.dmaregion_addr_len) < 0) {
		pr_err("LANDSHARK length not found in Device Tree.\n");
		result = -EINVAL;
		goto return_result;
	}
	if (of_property_read_u32(np, "address",
				&landshark_phys_mem.dmaregion_addr) < 0) {
		pr_err("LANDSHARK address not found in Device Tree.\n");
		result = -EINVAL;
		goto return_result;
	}
	/* Alingning length and address to PAGE_SIZE boundaries*/
	landshark_phys_mem.dmaregion_addr =
		(((int) landshark_phys_mem.dmaregion_addr + page_size - 1)
		& ~(page_size - 1));
	landshark_phys_mem.dmaregion_addr_len =
		(((int) landshark_phys_mem.dmaregion_addr_len)
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

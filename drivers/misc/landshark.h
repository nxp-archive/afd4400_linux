/* drivers/misc/landshark.h
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
**/

#ifndef __LANDSHARK_H
#define __LANDSHARK_H

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT

#define MOD_NAME "landshark"
#define MOD_VERSION "0.2"

#define LANDSHARK_MAJOR 0

struct phys_mem_info {

	/* Physical address of the memory pool */
	unsigned int dmaregion_addr;

	/* size of the physical memory pool */
	unsigned int dmaregion_addr_len;
};

long landshark_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static s32 landshark_mmap(struct file *filp, struct vm_area_struct *vma);
static s32 landshark_open(struct inode *inode, struct file *file);
static s32 landshark_release(struct inode *inode, struct file *filp);

static s32 landshark_probe(struct platform_device *ofdev);
static s32 landshark_remove(struct platform_device *ofdev);

#endif /*__LANDSHARK_H*/

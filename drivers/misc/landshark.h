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
 *  Theory of operation
 *
 *  The driver reads DTS entry and provides this information to user LANDSHARK
 *  library. It also implements mmap() functionality for the physical memory
 *  available from kernel.
 *  */

#ifndef __LANDSHARK_H
#define __LANDSHARK_H


#define LANDSHARK_MAGIC_NUM 'M'
#define LANDSHARK_MEM_INFO_GET _IOW(LANDSHARK_MAGIC_NUM, 1, int)

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT

#define MOD_NAME "landshark"
#define MOD_VERSION "0.1"

#define LANDSHARK_MAJOR 0

struct phys_mem_info {
	unsigned int physical_addr;
	unsigned int physical_addr_len;
};

long landshark_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static s32 landshark_mmap(struct file *filp, struct vm_area_struct *vma);
static s32 landshark_open(struct inode *inode, struct file *file);
static s32 landshark_release(struct inode *inode, struct file *filp);

static s32 landshark_probe(struct platform_device *ofdev);
static s32 landshark_remove(struct platform_device *ofdev);

#endif/*__LANDSHARK_H*/

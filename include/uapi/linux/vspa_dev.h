/*
 * include/uapi/linux/vspa_dev.h
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

#ifndef __VSPADEV_H_
#define __VSPADEV_H_

#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/vspa_uapi.h>

/* FIX: This is only to be defined on MEDUSA board */
#define FSL_MEDUSA	1

#define MAX_VSPA_PER_SOC    11

#define IRQEN_REG_OFFSET	3
#define STATUS_REG_OFFSET	4

/* These state are only for debug purpose only */
enum vspa_state {

	/* Initial state when the device identified*/
	VSPA_PROBED,

	/* State when the device is closed by user space*/
	VSPA_CLOSED,

	/* State when the device is opened by user space*/
	VSPA_OPENED,

	/* State when the device is mmaped by user space*/
	VSPA_MMAPED
};

void vspa_reg_write(void __iomem *addr, u32 val)
{
	return iowrite32(val, addr);
}

unsigned int vspa_reg_read(void __iomem *addr)
{
	return ioread32(addr);
}

/* The below structure contains all the information for the
* vspa device required by the kernel driver
*/
struct vspa_device {

	/* VSPA instance */
	int     id;

	/* Physical address of the IP registers */
	u32 __iomem *mem_start;

	/* Size of IP register map of vspa */
	resource_size_t mem_size;

	/* Physical address of the vspa dbg registers */
	u32 __iomem *dbg_start;

	/* Size of debug registers mmap of vspa */
	resource_size_t dbg_size;

	/* Lock to prevent multiple access */
	spinlock_t lock;

	/* virt. address of the control registers */
	u32 __iomem *regs;

	struct device *dev;

	/* Char device structure */
	struct cdev cdev;

	/* Major minor information */
	dev_t dev_t;

	/* Status register content of IP register set*/
	uint32_t   status_reg;

	/* Wait queue for irq event notifications*/
	wait_queue_head_t irq_wait_q;

#ifndef FSL_MEDUSA
	uint32_t vspa_irq_no;
#else
	uint32_t irq_dma_cmp;
	uint32_t irq_dma_err;
	uint32_t irq_vspa_done;
	uint32_t irq_vspa_msg;
#endif
	/* Minor number of the VSAP instance */
	unsigned    minor;

	/* Current state of the device */
	enum vspa_state state;
};

#endif

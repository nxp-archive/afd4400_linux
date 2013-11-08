/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 */

#ifndef __VSPADEV_H_
#define __VSPADEV_H_

#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/vspa_uapi.h>


/* IP register offset for the registers used by the driver */
#define IRQEN_REG_OFFSET 3

#define STATUS_REG_OFFSET 4

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
	u32 __iomem *mem_regs;

	/* Physical address of the device context memory */
	u32 __iomem *mem_context;

	/* Size of IP register map of vspa */
	resource_size_t mem_size;

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

	uint32_t vspa_irq_no;

	/* Physical address of the vspa dbg registers */
	u32 __iomem *dbg_regs;

	/* Size of debug registers mmap of vspa */
	resource_size_t dbg_size;

	/*Interrupt enabled for this device*/

	int irq_enabled;

	/* Current state of the device */
	enum vspa_state state;
};

#endif

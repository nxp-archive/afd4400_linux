/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

/*
 * drivers/misc/vspa.c
 * VSPA device driver
 *
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
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

//TODO - Add CMD_ERR event
//TODO - SPM error handing

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
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <mach/gpc.h>

#include <uapi/linux/vspa.h>
#include "vspa.h"

#define MAX_VSPA 11

#define VSPA_DEVICE_NAME "vspa"

static int cmd_buffer_bytes = 1024;
module_param(cmd_buffer_bytes, int, 0644);
MODULE_PARM_DESC(cmd_buffer_bytes, " Size of command buffer in bytes, "\
				"default is 1024");

static int reply_buffer_bytes = 1024;
module_param(reply_buffer_bytes, int, 0644);
MODULE_PARM_DESC(reply_buffer_bytes, " Size of reply buffer in bytes, "\
				"default is 1024");

static int spm_buffer_bytes = 4096;
module_param(spm_buffer_bytes, int, 0644);
MODULE_PARM_DESC(spm_buffer_bytes, " Size of spm buffer in bytes, "\
				"default is 4096");

/* Number of VSPA devices probed on system */
static int vspa_devs = 0;
static s32 vspa_major = 0;
static s32 vspa_minor = 0;
static struct class *vspa_class = NULL;

#define VSPA_HALT_TIMEOUT (100000)
#define VSPA_STARTUP_TIMEOUT (100000)

/* Debug and error reporting macros */
#define IF_DEBUG(x)	if (vspadev->debug & (x))
#define ERR(...)	{if (vspadev->debug & DEBUG_MESSAGES) \
				pr_err(VSPA_DEVICE_NAME __VA_ARGS__);}

/*********************** Accessor Functions ***************************/

static inline void vspa_reg_write(void __iomem *addr, u32 val)
{
	return iowrite32(val, addr);
}

static inline unsigned int vspa_reg_read(void __iomem *addr)
{
	return ioread32(addr);
}

/*********************** DMA Request Queue ****************************/

static int dma_next_index(int curr)
{
	return (curr == (DMA_QUEUE_ENTRIES - 1)) ? 0 : curr+1;
}

static void dma_reset_queue(struct vspa_device *vspadev)
{
	unsigned long irqflags;

	spin_lock_irqsave(&vspadev->dma_tx_queue_lock, irqflags);
	vspadev->dma_queue.idx_dma = 0;
	spin_unlock_irqrestore(&vspadev->dma_tx_queue_lock, irqflags);
	spin_lock(&vspadev->dma_enqueue_lock);
	vspadev->dma_queue.idx_queue = 0;
	vspadev->dma_queue.idx_chk = 0;
	vspadev->dma_queue.chan = 0;
	vspadev->cmd_dma_chan   = 0;
	spin_unlock(&vspadev->dma_enqueue_lock);
}

/* sometimes called from IRQ handler */
static int dma_transmit(struct vspa_device *vspadev)
{
	int ret = 0;
	u32 __iomem *regs = vspadev->regs;
	struct dma_queue *dq = &(vspadev->dma_queue);
	struct vspa_dma_req *dr;
	unsigned long irqflags;

	spin_lock_irqsave(&vspadev->dma_tx_queue_lock, irqflags);
	dr = &(dq->entry[dq->idx_dma]);
	if (dq->idx_dma == dq->idx_queue) /* Queue is empty */
		ret = -ENOMSG;
	else if (dq->idx_dma != dq->idx_chk) /* DMA is in process */
		ret = -EBUSY;
	else {
		/* Program the DMA transfer */
		vspa_reg_write(regs + DMA_DMEM_ADDR_REG_OFFSET, dr->dmem_addr);
		vspa_reg_write(regs + DMA_AXI_ADDR_REG_OFFSET,  dr->axi_addr);
		vspa_reg_write(regs + DMA_BYTE_CNT_REG_OFFSET,  dr->byte_cnt);
		vspa_reg_write(regs + DMA_XFR_CTRL_REG_OFFSET,  dr->xfr_ctrl);
		/* Update the queue */
		dq->idx_dma = dma_next_index(dq->idx_dma);
	}
	spin_unlock_irqrestore(&vspadev->dma_tx_queue_lock, irqflags);
	return ret;
}

static int dma_enqueue(struct vspa_device *vspadev, struct vspa_dma_req *req)
{
	int ret = 0;
	struct dma_queue *dq = &(vspadev->dma_queue);
	struct vspa_dma_req *dr;
	int next_slot;

	spin_lock(&vspadev->dma_enqueue_lock);

	dr = &(dq->entry[dq->idx_queue]);
	next_slot = dma_next_index(dq->idx_queue);

	if (next_slot == dq->idx_chk) /* Queue is full */
		ret = -ENOMEM;
	else {
//pr_info("DMA Enqueue(%d)\n", dq->idx_queue);
		dr->control   = req->control;
		dr->dmem_addr = req->dmem_addr;
		dr->axi_addr  = req->axi_addr;
		dr->byte_cnt  = req->byte_cnt;
		dr->xfr_ctrl  = req->xfr_ctrl | 0x4000; /* IRQ_EN */
		dq->chan      = dr->xfr_ctrl & 0x1F;

		IF_DEBUG(DEBUG_DMA)
			pr_info("vspa%d dma: %08X %08X %08X %08X %08X\n",
				vspadev->id, dr->control, dr->dmem_addr,
				dr->axi_addr, dr->byte_cnt, dr->xfr_ctrl);
// barrier
		dq->idx_queue = next_slot;
	}
	spin_unlock(&vspadev->dma_enqueue_lock);
	dma_transmit(vspadev);
	return ret;
}

/************  Command / Reply Memory Pool Management **************/
static void cbuffer_reset(struct circular_buffer *cbuf)
{
	cbuf->write_idx = 0;
	cbuf->read_idx = 0;
}

static void cbuffer_init(struct circular_buffer *cbuf, uint32_t size,
	uint32_t *paddr, uint32_t *vaddr)
{
	cbuf->size = size;
	cbuf->paddr = paddr;
	cbuf->vaddr = vaddr;
	cbuffer_reset(cbuf);
}

/* Make sure size is AXI aligned */
static int cbuffer_add(struct circular_buffer *cbuf, uint32_t size)
{
	uint32_t space;
	uint32_t index;
	uint32_t new_idx;

	if (cbuf->write_idx >= cbuf->read_idx) {
		index = cbuf->write_idx;
		space = cbuf->size - index;
		if (space < size) {
			index = 0;
			space = cbuf->read_idx;
		}
	} else {
		index = cbuf->write_idx;
		space = cbuf->read_idx - index;
	}

	if (space < size) {
		/* as a last resort resort reset the indexes if empty */
		if (cbuf->write_idx == cbuf->read_idx && cbuf->size >= size) {
			cbuf->write_idx = 0;
			cbuf->read_idx = 0;
			index = 0;
		} else
			return -ENOBUFS;
	}

	new_idx = index + size;
	if (new_idx == cbuf->size)
		new_idx = 0;
	if (new_idx == cbuf->read_idx)
		return -ENOBUFS;

	cbuf->write_idx = new_idx;
	return index;
}

static void cbuffer_free(struct circular_buffer *cbuf, uint32_t index, uint32_t size)
{
	cbuf->read_idx = index + size;
	if (cbuf->read_idx >= cbuf->size)
		cbuf->read_idx = 0;
}

static void pool_print(struct memory_pool *pool)
{
	int i;
	pr_info("Pool: free = %04X, tail = %d\n", pool->free_bds, pool->tail);
	for (i=0; i < BD_ENTRIES; i++) {
		pr_info("[%02d] = %4d %4d %3d %3d %2d\n", i,
			pool->bd[i].start, pool->bd[i].wstart,
			pool->bd[i].size, pool->bd[i].free,
			pool->bd[i].next);
	}
}

/* Bit map manipulation for checking a free command descriptor */
#define GET_FREE_BD(n) { \
	n = ffs(pool->free_bds); \
	if ((n > BD_ENTRIES) || (n == 0)) \
		n = -1; \
	else {\
		n = n - 1;\
		pool->free_bds &= ~(1UL << n);\
		} \
	}

static void pool_reset(struct memory_pool *pool)
{
	int i;
	for (i=0; i < BD_ENTRIES; i++) {
		pool->bd[i].wstart = -1;
		pool->bd[i].start = 0;
		pool->bd[i].size = 0;
		pool->bd[i].free = 0;
		pool->bd[i].next = -1;
	}
	pool->free_bds = (1 << BD_ENTRIES) - 1;
	GET_FREE_BD(i);
	pool->bd[i].start = pool->size;
	pool->bd[i].free = pool->size;
	pool->bd[i].next = i;
	pool->tail = i;
}

static void pool_init(struct memory_pool *pool, uint32_t size,
	uint32_t *paddr, uint32_t *vaddr)
{
	pool->free_bds = 0;
	pool->size = size;
	pool->paddr = paddr;
	pool->vaddr = vaddr;
	pool_reset(pool);
}

/**
 * @brief : This gets the free command descriptor
 * for placing the command of requested size
 * @return : valid BD index else negative
 */

static int8_t pool_get_bd(struct memory_pool *pool, uint32_t size)
{
	int i = -1, j = -1, k = -1;

	if (size == 0)
		return -EINVAL;

//pr_info("Pool_get_bd: size = %d\n", size);
	/* Find next buffer descriptor that has enough free space */
	for (i = pool->tail; pool->bd[i].free < size; i = pool->bd[i].next) {
		/* Did we wrap around and no large block found? */
		if (pool->bd[i].next == pool->tail)
			return -ENOSPC;
	}

	/* We found a bigger block ? */
	if (pool->bd[i].free > size) {
		/* We need to break it up*/
		GET_FREE_BD(j);
		if (j == -1)
			return -ENOSPC;
		pool->tail = j;
		k = pool->bd[i].next;
		pool->bd[j].next = k;
		pool->bd[i].next = j;
		pool->bd[j].free = pool->bd[i].free - size;
		pool->bd[j].size = 0;
		pool->bd[j].start = pool->bd[i].start - size;
	} else {
		pool->tail = i;
	}
	pool->bd[i].free = 0;
	pool->bd[i].size = size;
	pool->bd[i].wstart = pool->bd[i].start - pool->bd[i].size;
	return i;
}

static void pool_free_bd(struct memory_pool *pool, uint8_t index)
{
	uint32_t size;

//pr_info("pool_free: %d\n", index);
	if (index < 0 || index >= BD_ENTRIES)
		return;

	pool->bd[index].wstart = -1;
	size = pool->bd[index].size;
	pool->bd[index].size = 0;
	pool->bd[index].free = size;
}

static void pool_consolidate(struct memory_pool *pool, uint32_t wstart)
{
	int index, i;
	int cons = 0;

	for (index = 0; pool->bd[index].next > 0; index = i) {
		i = pool->bd[index].next;
		/* This and next BD must both have free space */
		if (pool->bd[index].free == 0 || pool->bd[i].free == 0)
			continue;
		/* Don't collapse BD at wstart */
		if (pool->bd[i].start == wstart)
			continue;

		pool->bd[index].free += pool->bd[i].free;
		pool->bd[index].next = pool->bd[i].next;
		pool->bd[i].wstart = -1;
		pool->bd[i].start = 0;
		pool->bd[i].size = 0;
		pool->bd[i].free = 0;
		pool->bd[i].next = -1;
		if (pool->tail == i)
			pool->tail = index;
		pool->free_bds |= (1UL << i);
		i = pool->bd[index].next;
		cons++;
	}
}

/******************** Sequence ID Management ***********************/

static void seqid_reset(struct vspa_device *vspadev)
{
	int seqid;

	vspadev->active_seqids = 0;
	vspadev->last_seqid = 0;
	for (seqid = 0; seqid < MAX_SEQIDS; seqid++)
		vspadev->seqid[seqid].cmd_id = -1;
}

static int seqid_get_next(struct vspa_device *vspadev)
{
	int next;
	uint32_t ids = (vspadev->active_seqids) << MAX_SEQIDS |
				vspadev->active_seqids;

	ids = ids >> (vspadev->last_seqid + 1);
	next = ffz(ids);
	if (next > MAX_SEQIDS)
		return -ENOSR;
	next += (vspadev->last_seqid + 1);
	next &= MAX_SEQIDS - 1;

	IF_DEBUG(DEBUG_SEQID)
		pr_info("seqid: active %04X, last %d, ids %08X, next %d\n",
		vspadev->active_seqids, vspadev->last_seqid, ids, next);

	vspadev->active_seqids |= 1 << next;
	vspadev->last_seqid = next;
	vspadev->seqid[next].cmd_id = -1;
	return next;
}

static void seqid_release(struct vspa_device *vspadev, int seqid)
{
	IF_DEBUG(DEBUG_SEQID)
		pr_info("seqid_release(%d)\n", seqid);

	if (seqid < 0 || seqid >= MAX_SEQIDS)
		return;

	if (vspadev->seqid[seqid].cmd_id >= 0)
	{
		vspadev->seqid[seqid].cmd_id = -1;

		if (vspadev->seqid[seqid].cmd_buffer_idx >= 0) {
			cbuffer_free(&vspadev->cmd_buffer,
				vspadev->seqid[seqid].cmd_buffer_idx,
				vspadev->seqid[seqid].cmd_buffer_size);
			vspadev->seqid[seqid].cmd_buffer_idx = -1;
		}

		if (vspadev->seqid[seqid].cmd_bd_index >= 0) {
			pool_free_bd(&vspadev->cmd_pool,
				vspadev->seqid[seqid].cmd_bd_index);
			vspadev->seqid[seqid].cmd_bd_index = -1;
		}

		if (vspadev->seqid[seqid].reply_bd_index >= 0) {
			pool_free_bd(&vspadev->reply_pool,
				vspadev->seqid[seqid].reply_bd_index);
			vspadev->seqid[seqid].reply_bd_index = -1;
		}
	}
	vspadev->active_seqids &= ~(1 << seqid);
}

/************************** Event Queue *******************************/

static inline uint32_t events_present(struct vspa_device *vspadev)
{
	uint32_t mask = vspadev->event_list_mask;
	struct event_queue *eq = &(vspadev->event_queue);

	if (eq->idx_enqueue != eq->idx_dequeue) /* Queue is not empty */
		mask |= vspadev->event_queue_mask;
	return mask;
}

static inline int event_next_index(int curr)
{
	return (curr == (EVENT_QUEUE_ENTRIES - 1)) ? 0 : curr + 1;
}

static void event_reset_queue(struct vspa_device *vspadev)
{
	int i;
	unsigned long irqflags;

	spin_lock_irqsave(&vspadev->event_queue_lock, irqflags);
	vspadev->event_queue_mask = 0;
	vspadev->event_queue.idx_enqueue = 0;
	vspadev->event_queue.idx_dequeue = 0;
	vspadev->event_queue.idx_queued = 0;
	spin_unlock_irqrestore(&vspadev->event_queue_lock, irqflags);

	spin_lock(&vspadev->event_list_lock);
	vspadev->event_list_mask = 0;
	INIT_LIST_HEAD(&vspadev->events_free_list);
	INIT_LIST_HEAD(&vspadev->events_queued_list);
	for (i = 0; i < EVENT_LIST_ENTRIES; i++)
		list_add(&vspadev->events[i].list,
			 &vspadev->events_free_list);
	spin_unlock(&vspadev->event_list_lock);
}

/* This routine is usually called from IRQ handlers */
static void event_enqueue(struct vspa_device *vspadev, uint8_t type,
	uint8_t id, uint8_t err, uint32_t data0, uint32_t data1)
{
	struct event_queue *eq = &(vspadev->event_queue);
	struct event_entry *er;
	int next_slot;
	unsigned long irqflags;

	spin_lock_irqsave(&vspadev->event_queue_lock, irqflags);

	if (eq->idx_enqueue == eq->idx_dequeue) /* Queue is empty */
		vspadev->event_queue_mask = 0;

	next_slot = event_next_index(eq->idx_enqueue);

	if (next_slot == eq->idx_dequeue) /* Queue is full */
	{
		er = &(eq->entry[eq->idx_queued]);
		if (type > 7) type = 0;
		er->lost |= 1 << type;
	} else {
		er = &(eq->entry[eq->idx_enqueue]);
		er->type  = type;
		er->id    = id;
		er->err   = err;
		er->data0 = data0;
		er->data1 = data1;
		er->lost = 0;
		eq->idx_queued = eq->idx_enqueue;
		eq->idx_enqueue = next_slot;

		vspadev->event_queue_mask |= 0x10 << type;
		wake_up_interruptible(&vspadev->event_wait_q);
	}
	spin_unlock_irqrestore(&vspadev->event_queue_lock, irqflags);
}

static void event_list_update(struct vspa_device *vspadev)
{
	struct event_list *ptr;
	struct event_list *last;
	struct event_queue *eq = &(vspadev->event_queue);
	struct event_entry *er;
	uint32_t mask = 1;
	int reply_bd_idx;

	if (eq->idx_enqueue == eq->idx_dequeue)
		return;

	spin_lock(&vspadev->event_list_lock);

	if (list_empty(&vspadev->events_queued_list))
		last = NULL;
	else
		last = list_last_entry(&vspadev->events_queued_list,
				       struct event_list, list);
		mask = 0;

	while (eq->idx_enqueue != eq->idx_dequeue) {
		er = &(eq->entry[eq->idx_dequeue]);
// TODO - check SPM for command error messages
		/* coalese error events of the same type */
		if (last &&
		    last->type == ((0x10<<VSPA_EVENT_ERROR)|VSPA_EVENT_ERROR) &&
		    er->type == VSPA_EVENT_ERROR &&
		    er->id == last->id &&
		    er->id == VSPA_ERR_WATCHDOG) {
			last->data0 = er->data0;
			last->data1++;
			IF_DEBUG(DEBUG_EVENT)
				pr_info("vspa%d: co %04X %02X %02X %08X %08X\n",
					vspadev->id, last->type, last->id,
					last->err, last->data0, last->data1);
		} else {
			if (list_empty(&vspadev->events_free_list)) {
				mask = 0;
				ERR("%d: Event queue overflowed\n",
								 vspadev->id);
// TODO - add lost events
				ptr = list_first_entry(
						&vspadev->events_queued_list,
						struct event_list, list);
				reply_bd_idx = ptr->data1 >> 24;
				if ((ptr->type == VSPA_EVENT_REPLY) &&
				    (reply_bd_idx < BD_ENTRIES)) {
					pool_free_bd(&vspadev->reply_pool,
								reply_bd_idx);
				}
				list_move_tail(&ptr->list,
						 &vspadev->events_free_list);
			}
			ptr = list_first_entry(&vspadev->events_free_list,
					       struct event_list, list);
			ptr->err   = er->err;
			ptr->id    = er->id;
			ptr->type  = (0x10 << er->type) | er->type;
			ptr->data0 = er->data0;
			ptr->data1 = er->data1;
			IF_DEBUG(DEBUG_EVENT)
				pr_info("vspa%d: up %04X %02X %02X %08X %08X\n",
					vspadev->id, ptr->type, ptr->id,
					ptr->err, ptr->data0, ptr->data1);
			list_move_tail(&ptr->list, &vspadev->events_queued_list);
			last = ptr;
			vspadev->event_list_mask |= 0x10 << er->type;
		}
		eq->idx_dequeue = event_next_index(eq->idx_dequeue);
	}

	/* update event list mask if needed */
	if (mask == 0) {
		list_for_each_entry(ptr, &vspadev->events_queued_list, list) {
			mask |= ptr->type;
		}
		vspadev->event_list_mask = mask & VSPA_MSG_ALL_EVENTS;
	}

	spin_unlock(&vspadev->event_list_lock);
// TODO - report lost events
}

static const int event_size[16] = {
	0, 0, 0, 0, 0, 8, 4, 0, 0, 8, 0, 0, 0, 0, 0, 0
};

static int read_event(struct vspa_device *vspadev,
	struct vspa_event_read *evt_rd)
{
	struct event_list *ptr, *ptr1;
	uint32_t buf[sizeof(struct vspa_event)/sizeof(uint32_t) + 2];
	struct vspa_event *evt = (struct vspa_event*)buf;
	uint32_t *payload = &evt->data[0];
	int err;
	unsigned int mask;
	int type;
	uint32_t *src_ptr;
	uint32_t src_len;
	size_t length;
	struct list_head *prev;
	int timeout;
	unsigned long start_time, new_time;
	size_t len;
	int reply_bd_idx, start;

	if (vspadev->state == VSPA_STATE_UNKNOWN)
		return -ENODATA;

	start_time = jiffies;

	event_list_update(vspadev);

	/* Convert timeout to jiffies */
	timeout = evt_rd->timeout;
	if (timeout > 0) timeout = msecs_to_jiffies(timeout);
	/* If no filter is specified then match all message types */
	mask = evt_rd->event_mask & VSPA_MSG_ALL;
	if ((mask & VSPA_MSG_ALL_EVENTS) == 0)
		mask |= VSPA_MSG_ALL_EVENTS;

	spin_lock(&vspadev->event_list_lock);
	while (((vspadev->event_list_mask & mask) == 0) &&
		(vspadev->state > VSPA_STATE_POWER_DOWN)) {
		/* nothing to read */
		spin_unlock(&vspadev->event_list_lock);
		if (vspadev->state <= VSPA_STATE_POWER_DOWN)
			return -ENODATA;
		if (timeout == 0) /* non-blocking */
			return -EAGAIN;
//pr_err("vspa%d sleep: mask %04X, elm %04X, ep %04X, st %d\n", vspadev->id, mask, vspadev->event_list_mask, events_present(vspadev), vspadev->state);
		if (timeout < 0) {
			err = wait_event_interruptible(vspadev->event_wait_q,
				((events_present(vspadev) & mask) ||
				 (vspadev->state <= VSPA_STATE_POWER_DOWN)));
			if (err < 0)
				return err;
		} else {
			err = wait_event_interruptible_timeout(
				vspadev->event_wait_q,
				((events_present(vspadev) & mask) ||
				 (vspadev->state <= VSPA_STATE_POWER_DOWN)),
				timeout);
			if (err < 0)
				return err;
			new_time = jiffies;
			timeout -= new_time - start_time;
			start_time = new_time;
			if (timeout < 0) timeout = 0;
		}
//pr_err("vspa%d wakup: mask %04X, elm %04X, ep %04X, st %d\n", vspadev->id, mask, vspadev->event_list_mask, events_present(vspadev), vspadev->state);
		event_list_update(vspadev);
		/* otherwise loop, but first reacquire the lock */
		spin_lock(&vspadev->event_list_lock);
	}

	/* Find first event that matches the mask */
	type = 0;
	list_for_each_entry(ptr, &vspadev->events_queued_list, list) {
//pr_err("vspa%d: read %04X check %04X\n", vspadev->id, mask, ptr->type);
		if (ptr->type & mask) {
			type = ptr->type & 0xF;
			break;
		}
	}
	if (!type) {
		spin_unlock(&vspadev->event_list_lock);
		return (vspadev->state <= VSPA_STATE_POWER_DOWN) ? -ENODATA :
								   -EAGAIN;
	}
	src_len = 0;
	src_ptr = NULL;
	reply_bd_idx = -1;
	if (type == VSPA_EVENT_REPLY) {
		reply_bd_idx = ptr->data1 >> 24;
		if (reply_bd_idx >= BD_ENTRIES)
			reply_bd_idx = -1;
		else {
			start   = vspadev->reply_pool.bd[reply_bd_idx].wstart;
			if (start >= 0) {
				src_len = ptr->data1 & 0xFFFF;
				src_ptr = &vspadev->reply_pool.vaddr[start];
			}
			IF_DEBUG(DEBUG_REPLY) {
				int i;
				uint32_t *st = src_ptr;
				pr_info("vspa%d: evt_read Reply %08X %d->%04X %d\n",
					vspadev->id, ptr->data1, reply_bd_idx,
					start, src_len);
				for (i=0; i < src_len/4; i--)
					pr_info("%02d = %08X\n", i, *st++);
			}
		}
	} else if (type == VSPA_EVENT_SPM) {
		start   = ptr->data1 >> 16;
		src_len = ptr->data1 & 0xFFFF;
		src_ptr = &vspadev->spm_buf_vaddr[start];
		IF_DEBUG(DEBUG_SPM) {
			int i;
			uint32_t *st = src_ptr;
			pr_info("vspa%d: evt_read SPM %08X %04X %d\n",
				vspadev->id, ptr->data1, start, src_len);
			for (i = src_len/4; i>0; i--)
				pr_info("[%2d] %08X\n", i, *st++);
		}
	}

	length = event_size[type];
	evt->control  = ptr->control;
	evt->type     = type;
	evt->pkt_size = length ? length : src_len;
	payload[0]    = ptr->data0;
	payload[1]    = ptr->data1;

	src_len = evt_rd->buf_len - sizeof(*evt);
	if (src_len > evt->pkt_size) src_len = evt->pkt_size;
	if (src_len < 0) src_len = 0;
	evt->buf_size = src_len;

	IF_DEBUG(DEBUG_EVENT) {
		pr_info("vspa%d: evt_read %2d %2d %2d %2d %08X %08X\n",
			vspadev->id, type, evt->err, evt->pkt_size,
			 evt->buf_size, ptr->data0, ptr->data1);
		if (src_len) {
			int i;
			uint32_t *st = src_ptr;
			for (i = src_len/4; i>0; i--)
				pr_info("[%2d] %08X\n", i, *st++);
		}
	}

	/* free the message unless PEEKing */
	if (!(mask & VSPA_MSG_PEEK)) {
		prev = ptr->list.prev;
		list_move_tail(&ptr->list, &vspadev->events_free_list);
		/* If not at the end of the list try coalescing messages */
		if (prev->next != &vspadev->events_queued_list &&
			    prev != &vspadev->events_queued_list) {
			/* try to coalese Watchdog errors */
			ptr = list_entry(prev, struct event_list, list);
			ptr1 = list_first_entry(prev, struct event_list, list);
			if (ptr->type ==
				((0x10<<VSPA_EVENT_ERROR)|VSPA_EVENT_ERROR) &&
			    ptr1->type ==
				((0x10<<VSPA_EVENT_ERROR)|VSPA_EVENT_ERROR) &&
			    ptr->id == ptr1->id &&
			    ptr->id == VSPA_ERR_WATCHDOG) {
				ptr->data0 = ptr1->data0;
				ptr->data1++;
				list_move_tail(&ptr1->list,
					&vspadev->events_free_list);
			}
		}
		/* update event list mask */
		mask = 0;
		list_for_each_entry(ptr, &vspadev->events_queued_list, list) {
			mask |= ptr->type;
		}
		vspadev->event_list_mask = mask & VSPA_MSG_ALL_EVENTS;
	}

	/* only copy up to the length of the message */
	length += sizeof(*evt);
	len = evt_rd->buf_len;
	if (length > len)
		length = len;
	err = copy_to_user(evt_rd->buf_ptr, evt, length);

	/* copy the data buffer contents if needed */
	len -= length;
	if (err == 0 && src_len > 0 && len > 0) {
		if (src_len > len)
			src_len = len;
		length += src_len;
		err = copy_to_user(&(evt_rd->buf_ptr->data[0]), src_ptr, src_len);
	}

	/* Release reply buffer */
	if ((reply_bd_idx >= 0) && (!(mask & VSPA_MSG_PEEK)))
		pool_free_bd(&vspadev->reply_pool, reply_bd_idx);

	spin_unlock(&vspadev->event_list_lock);
//pr_err("vspa%d: err %d, length %d bytes\n", vspadev->id, err, length);

	return err ? -EFAULT : length;
}

/************************ Command Buffer ***************************/

static void cmd_reset(struct vspa_device *vspadev)
{
	vspadev->first_cmd = 1;
	vspadev->cmd_pool.size = 0;
	cbuffer_reset(&vspadev->cmd_buffer);
	pool_reset(&vspadev->cmd_pool);
	pool_reset(&vspadev->reply_pool);
	seqid_reset(vspadev);
	/* clear SPM buffer */
	memset(vspadev->spm_buf_vaddr, 0 ,vspadev->spm_buf_bytes);
	vspadev->spm_addr = (vspadev->spm_buf_bytes - 4) >> 2;
}

/************************ SPM Processing ***************************/

static void spm_update(struct vspa_device *vspadev, uint32_t flags)
{
	uint32_t ptr;
	int size;
	int size_max;
	int err;

	err = 0;
	if (flags & DMA_FLAG_CFGERR)
		err = VSPA_ERR_DMA_CFGERR;
	else if (flags & DMA_FLAG_XFRERR)
		err = VSPA_ERR_DMA_XFRERR;
//TODO handle error case
	ptr = vspadev->spm_buf_vaddr[vspadev->spm_addr];
	IF_DEBUG(DEBUG_SPM) {
		pr_info("vspa%d: SPM DMA IRQ, %d\n", vspadev->id, flags);
		pr_info("[%04X] = %08X\n", vspadev->spm_addr, ptr);
		IF_DEBUG(DEBUG_TEST_SPM) {
			int i;
			for (i=0; i < (vspadev->spm_buf_bytes>>2); i++) {
				if (vspadev->spm_buf_vaddr[i])
					pr_info("[%04X] = %08X\n", i,
						vspadev->spm_buf_vaddr[i]);
			}
		}
	}

	while (ptr) {
		if (ptr >= (uint32_t)vspadev->spm_buf_paddr)
			ptr -= (uint32_t)vspadev->spm_buf_paddr;
		if (ptr >= vspadev->spm_buf_bytes || ptr & 3) {
			vspadev->spm_buf_vaddr[vspadev->spm_addr] = 0;
			ERR("%d: bad SPM ptr %08X\n", vspadev->id, ptr);
//TODO report error
		} else {
			ptr >>= 2;
			size_max = ptr > vspadev->spm_addr ?
					(vspadev->spm_buf_bytes >> 2) - ptr :
					vspadev->spm_addr - ptr;
			size = (vspadev->spm_buf_vaddr[ptr] >> 16) & 0xFF;
			if (size >= size_max)
				size = size_max - 1;
			err = 0; // TODO flags ??
			event_enqueue(vspadev, VSPA_EVENT_SPM, 0, err,
				      vspadev->spm_buf_vaddr[ptr],
				      (ptr << 16) | (size << 2));
			vspadev->spm_addr = ptr - 1;
			IF_DEBUG(DEBUG_SPM) {
				while (size >= 0) {
					pr_info("[%04X] = %08X\n", ptr,
						vspadev->spm_buf_vaddr[ptr]);
					ptr++;
					size--;
				}
			}
		}
		ptr = vspadev->spm_buf_vaddr[vspadev->spm_addr];
		IF_DEBUG(DEBUG_SPM) {
			pr_info("vspa%d: SPM cont\n", vspadev->id);
			pr_info("[%04X] = %08X\n", vspadev->spm_addr, ptr);
		}
	}
}

/************************ Mailbox Queues ***************************/

static void mbox_reset(struct vspa_device *vspadev)
{
	vspadev->mb32_queue.idx_enqueue = 0;
	vspadev->mb32_queue.idx_dequeue = 0;
	vspadev->mb32_queue.idx_complete = 0;
	vspadev->mb64_queue.idx_enqueue = 0;
	vspadev->mb64_queue.idx_dequeue = 0;
	vspadev->mb64_queue.idx_complete = 0;
}

static int mbox_next_slot(struct mbox_queue *queue)
{
	int next_slot = queue->idx_enqueue + 1;

	if (next_slot == MBOX_QUEUE_ENTRIES)
		next_slot = 0;

	if (next_slot == queue->idx_dequeue) /* Queue is full */
		return -ENOMEM;

	return next_slot;
}

static int mb32_transmit(struct vspa_device *vspadev)
{
	u32 __iomem *regs = vspadev->regs;
	struct mbox_queue *mq = &(vspadev->mb32_queue);
	unsigned long irqflags;
	struct vspa_mb32 *me;
	int next_slot;
	int ret = 0;

	spin_lock_irqsave(&vspadev->mb32_lock, irqflags);
	if (mq->idx_dequeue == mq->idx_enqueue) /* Queue is empty */
		ret = -ENOMSG;
	else if (mq->idx_dequeue != mq->idx_complete) /* MBOX is in process */
		ret = -EBUSY;
	else {
		me = &(vspadev->mb32[mq->idx_dequeue]);
		vspa_reg_write(regs + HOST_OUT_32_REG_OFFSET, me->data);
		next_slot = mq->idx_dequeue + 1;
		if (next_slot == MBOX_QUEUE_ENTRIES)
			next_slot = 0;
		mq->idx_dequeue = next_slot;
	}
	spin_unlock_irqrestore(&vspadev->mb32_lock, irqflags);
	return ret;
}

static int mb64_transmit(struct vspa_device *vspadev)
{
	u32 __iomem *regs = vspadev->regs;
	struct mbox_queue *mq = &(vspadev->mb64_queue);
	unsigned long irqflags;
	struct vspa_mb64 *me;
	int next_slot;
	int ret = 0;

	spin_lock_irqsave(&vspadev->mb64_lock, irqflags);
	if (mq->idx_dequeue == mq->idx_enqueue) /* Queue is empty */
		ret = -ENOMSG;
	else if (mq->idx_dequeue != mq->idx_complete) /* MBOX is in process */
		ret = -EBUSY;
	else {
		me = &(vspadev->mb64[mq->idx_dequeue]);
		vspa_reg_write(regs + HOST_OUT_64_MSB_REG_OFFSET, me->data_msb);
		vspa_reg_write(regs + HOST_OUT_64_LSB_REG_OFFSET, me->data_lsb);
		next_slot = mq->idx_dequeue + 1;
		if (next_slot == MBOX_QUEUE_ENTRIES)
			next_slot = 0;
		mq->idx_dequeue = next_slot;
	}
	spin_unlock_irqrestore(&vspadev->mb64_lock, irqflags);
	return ret;
}

/*************************** Watchdog ******************************/

static void watchdog_callback(unsigned long data)
{
	struct vspa_device *vspadev = (struct vspa_device*)data;
	u32 __iomem *regs = vspadev->regs;
	uint32_t val;

	if (vspadev->state == VSPA_STATE_RUNNING_IDLE &&
	    vspadev->watchdog_interval_msecs > 0) {
		val = vspa_reg_read(regs + GP_OUT3_REG_OFFSET) >> 16;
		IF_DEBUG(DEBUG_WATCHDOG)
			pr_info("vspa%d: watchdog_value = %04X\n",
							vspadev->id, val);
		if (val == vspadev->watchdog_value) {
			event_enqueue(vspadev, VSPA_EVENT_ERROR,
					VSPA_ERR_WATCHDOG, 0, val, 0);
		}
		vspadev->watchdog_value = val;
		val = vspa_reg_read(regs + CONTROL_REG_OFFSET);
		val |= 1; // HOST_GO
		vspa_reg_write(regs + CONTROL_REG_OFFSET, val);
		mod_timer(&vspadev->watchdog_timer, jiffies +
			msecs_to_jiffies(vspadev->watchdog_interval_msecs));
	} else {
		complete(&vspadev->watchdog_complete);
	}

	event_list_update(vspadev);
}

/**************************** IRQ  *********************************/

static void vspa_enable_dma_irqs(struct vspa_device *vspadev)
{
	u32 __iomem *regs = vspadev->regs;

	uint32_t irqen  = vspa_reg_read(regs + IRQEN_REG_OFFSET);
	vspa_reg_write(regs + DMA_IRQ_STAT_REG_OFFSET, 0xFFFFFFFFUL);
	vspa_reg_write(regs + DMA_COMP_STAT_REG_OFFSET, 0xFFFFFFFFUL);
	vspa_reg_write(regs + DMA_XFRERR_STAT_REG_OFFSET, 0xFFFFFFFFUL);
	vspa_reg_write(regs + DMA_CFGERR_STAT_REG_OFFSET, 0xFFFFFFFFUL);

	/* Enable DMA complete and DMA error IRQs */
	irqen |= 0x30;
	vspa_reg_write(regs + IRQEN_REG_OFFSET, irqen);
}

static void vspa_enable_mailbox_irqs(struct vspa_device *vspadev)
{
	u32 __iomem *regs = vspadev->regs;

	uint32_t irqen  = vspa_reg_read(regs + IRQEN_REG_OFFSET);
	vspa_reg_write(regs + HOST_FLAGS0_REG_OFFSET, 0xFFFFFFFFUL);
	vspa_reg_write(regs + HOST_FLAGS1_REG_OFFSET, 0xFFFFFFFFUL);

	vspa_reg_read(regs + HOST_IN_64_MSB_REG_OFFSET);
	vspa_reg_read(regs + HOST_IN_64_LSB_REG_OFFSET);
	vspa_reg_read(regs + HOST_IN_32_REG_OFFSET);
	vspa_reg_write(regs + STATUS_REG_OFFSET, STATUS_REG_IRQ_VCPU_MSG);

	/*Enable Mailbox anf Flag IRQs */
	irqen |= 0x4C;
	vspa_reg_write(regs + IRQEN_REG_OFFSET, irqen);
}

/* Command Reply has been sent */
static irqreturn_t vspa_flags1_irq_handler(int irq, void *dev)
{
	int seqid;
	uint32_t clr_flags0 = 0;
	struct vspa_device *vspadev = (struct vspa_device *)dev;
	u32 __iomem *regs = vspadev->regs;
	uint32_t flags1 = vspa_reg_read(regs + HOST_FLAGS1_REG_OFFSET);
	uint32_t flags0 = vspa_reg_read(regs + HOST_FLAGS0_REG_OFFSET);
	vspa_reg_write(regs + HOST_FLAGS1_REG_OFFSET, flags1);

//	spin_lock(&vspadev->irq_lock);
	if (vspadev->irq_bits) { pr_err("VSPA%d flg1 irqs = %d\n",
			vspadev->id, vspadev->irq_bits);
	}
	vspadev->irq_bits |= 1;

	IF_DEBUG(DEBUG_FLAGS1_IRQ)
		pr_info("vspa%d: flags1 = %08x, flags0 = %08x\n",
				 vspadev->id, flags1, flags0);

	/* Mailbox complete is monitored directly on MBOX interrupt */
	if (flags1 & (1<<31)) {
		flags1 &= ~(1<<31);
		clr_flags0 = 1 << 31;
		IF_DEBUG(DEBUG_SPM) {
			pr_info("vspa%d: SPM Flags1 IRQ\n", vspadev->id);
		}
		spm_update(vspadev, 0);
//TODO - handle SPM if no SPM DMA ?
	}

	while (flags1) {
		seqid = ffs(flags1) - 1;
//pr_info("flags1 = %08X, flags0 = %08X, seqid = %d\n", flags1, flags0, seqid);
		flags1 &= ~(1 << seqid);
		if (flags0 & (1 << seqid)) {
			clr_flags0 |= 1 << seqid;
			if (vspadev->seqid[seqid].flags &
						VSPA_FLAG_REPORT_CMD_CONSUMED)
				event_enqueue(vspadev, VSPA_EVENT_CMD,
				      vspadev->seqid[seqid].cmd_id, 0,
				      vspadev->seqid[seqid].payload1,
				      vspadev->seqid[seqid].reply_size);
		}
		if (vspadev->seqid[seqid].flags & VSPA_FLAG_REPORT_CMD_REPLY)
			event_enqueue(vspadev, VSPA_EVENT_REPLY,
			      vspadev->seqid[seqid].cmd_id, 0,
			      vspadev->seqid[seqid].payload1,
			      vspadev->seqid[seqid].reply_size |
			      vspadev->seqid[seqid].reply_bd_index << 24);
		vspadev->seqid[seqid].reply_bd_index = -1;
		seqid_release(vspadev, seqid);
	}
	if (clr_flags0)
		vspa_reg_write(regs + HOST_FLAGS0_REG_OFFSET, clr_flags0);

//	spin_unlock(&vspadev->irq_lock);
	vspadev->irq_bits &= ~1;

	return IRQ_HANDLED;
}

/* Command has been consumed or Mailbox transfer completed */
static irqreturn_t vspa_flags0_irq_handler(int irq, void *dev)
{
	int seqid;
	int idx;
	uint32_t msb, lsb;
	struct vspa_device *vspadev = (struct vspa_device *)dev;
	u32 __iomem *regs = vspadev->regs;
	uint32_t status = vspa_reg_read(regs + STATUS_REG_OFFSET);
	uint32_t flags0 = vspa_reg_read(regs + HOST_FLAGS0_REG_OFFSET);
	vspa_reg_write(regs + HOST_FLAGS0_REG_OFFSET, flags0);

//	spin_lock(&vspadev->irq_lock);
	if (vspadev->irq_bits) { pr_err("VSPA%d flg0 irqs = %d\n",
			vspadev->id, vspadev->irq_bits);
	}
	vspadev->irq_bits |= 2;

	IF_DEBUG(DEBUG_FLAGS0_IRQ)
		pr_info("vspa%d: flags0 = %08x, status = %08x\n",
			vspadev->id, flags0, status);

	/* Handle Mailbox interrupts */
	if (status & STATUS_REG_IRQ_VCPU_MSG) {
		status = vspa_reg_read(regs + HOST_MBOX_STATUS_REG_OFFSET);
		IF_DEBUG(DEBUG_FLAGS0_IRQ)
			pr_info("mbox status = %08x\n", status);
		if (status & MBOX_STATUS_IN_64_BIT) {
			msb = vspa_reg_read(regs + HOST_IN_64_MSB_REG_OFFSET);
			lsb = vspa_reg_read(regs + HOST_IN_64_LSB_REG_OFFSET);
			IF_DEBUG(DEBUG_MBOX64_IN)
				pr_info("vspa%d: mbox64 in %08X %08X/n",
					vspadev->id, msb, lsb);
			event_enqueue(vspadev, VSPA_EVENT_MB64_IN, 0, 0,
								msb, lsb);
		}
		if (status & MBOX_STATUS_IN_32_BIT) {
			msb = vspa_reg_read(regs + HOST_IN_32_REG_OFFSET);
			IF_DEBUG(DEBUG_MBOX32_IN)
				pr_info("vspa%d: mbox32 in %08X/n",
					vspadev->id, msb);
			event_enqueue(vspadev, VSPA_EVENT_MB32_IN, 0, 0,
								msb, 0);
		}
		if (status & MBOX_STATUS_OUT_64_BIT) {
			idx = vspadev->mb64_queue.idx_dequeue;
			IF_DEBUG(DEBUG_MBOX64_OUT)
				pr_info("vspa%d: mbox64 out idx %d consumed/n",
					vspadev->id, idx);
			if (vspadev->mb64[idx].flags &
						VSPA_FLAG_REPORT_MB_COMPLETE)
				event_enqueue(vspadev, VSPA_EVENT_MB64_OUT,
						 vspadev->mb64[idx].id, 0,
						 vspadev->mb64[idx].data_msb,
						 vspadev->mb64[idx].data_lsb);
			vspadev->mb64_queue.idx_complete = idx;
			mb64_transmit(vspadev);
		}
		if (status & MBOX_STATUS_OUT_32_BIT) {
			idx = vspadev->mb32_queue.idx_dequeue;
			IF_DEBUG(DEBUG_MBOX32_OUT)
				pr_info("vspa%d: mbox32 out idx %d consumed/n",
					vspadev->id, idx);
			if (vspadev->mb32[idx].flags &
						VSPA_FLAG_REPORT_MB_COMPLETE)
				event_enqueue(vspadev, VSPA_EVENT_MB32_OUT,
						 vspadev->mb32[idx].id, 0,
						 vspadev->mb32[idx].data, 0);
			vspadev->mb32_queue.idx_complete = idx;
			mb32_transmit(vspadev);
		}
		vspa_reg_write(regs + STATUS_REG_OFFSET,
						STATUS_REG_IRQ_VCPU_MSG);
	}

	/* Handle Flags0 interrupts */
	while (flags0) {
		seqid = ffs(flags0) - 1;
//pr_info("flags0 = %08X, seqid = %d\n", flags0, seqid);
		flags0 &= ~(1 << seqid);
		/* Optionally send CMD consumed event message */
		if (vspadev->seqid[seqid].flags & VSPA_FLAG_REPORT_CMD_CONSUMED)
			event_enqueue(vspadev, VSPA_EVENT_CMD,
				      vspadev->seqid[seqid].cmd_id, 0,
				      vspadev->seqid[seqid].payload1,
				      vspadev->seqid[seqid].reply_size);
		/* Release SEQID if no reply is expected */
		if (!(vspadev->seqid[seqid].flags & VSPA_FLAG_EXPECT_CMD_REPLY))
			seqid_release(vspadev, seqid);
		else { /* otherwise just release the command buffer */
			if (vspadev->seqid[seqid].cmd_bd_index >= 0) {
				pool_free_bd(&vspadev->cmd_pool,
				     vspadev->seqid[seqid].cmd_bd_index);
				vspadev->seqid[seqid].cmd_bd_index = -1;
			}
		}

	}

//	spin_unlock(&vspadev->irq_lock);
	vspadev->irq_bits &= ~2;

	return IRQ_HANDLED;
}

static irqreturn_t vspa_dma_irq_handler(int irq, void *dev)
{
	struct vspa_device *vspadev = (struct vspa_device *)dev;
	u32 __iomem *regs = vspadev->regs;
	struct dma_queue *dq = &(vspadev->dma_queue);
	struct vspa_dma_req *dr;
	int flags = 0;
	int spm_flags = 0;
	u32 status;
	u32 stat;
	u32 mask;
	u32 spm_mask;
	int err;

//	spin_lock(&vspadev->irq_lock);
	if (vspadev->irq_bits) { pr_err("VSPA%d dma irqs = %d\n",
			vspadev->id, vspadev->irq_bits);
	}
	vspadev->irq_bits |= 4;

	status = vspa_reg_read(regs + STATUS_REG_OFFSET);
	IF_DEBUG(DEBUG_DMA_IRQ)
	pr_info("vspa%d: dma_irq %08x, COMP %08x, XFRERR %08x, CFGERR %08x\n",
			vspadev->id,
			vspa_reg_read(regs + DMA_IRQ_STAT_REG_OFFSET),
			vspa_reg_read(regs + DMA_COMP_STAT_REG_OFFSET),
			vspa_reg_read(regs + DMA_XFRERR_STAT_REG_OFFSET),
			vspa_reg_read(regs + DMA_CFGERR_STAT_REG_OFFSET));

	mask = 1 << vspadev->dma_queue.chan;
	/* legacy VSPA images consider all other DMA channels to be SPM */
	if (vspadev->spm_dma_chan == VSPA_DMA_CHANNELS)
		spm_mask = ~mask;
	else if (vspadev->spm_dma_chan < VSPA_DMA_CHANNELS)
		spm_mask = vspadev->spm_dma_chan < VSPA_DMA_CHANNELS ?
					1 << vspadev->spm_dma_chan : 0;
	else
		spm_mask = 0;

	/* Check for completed DMAs */
	if (status & STATUS_REG_IRQ_DMA_COMP) {
		stat = vspa_reg_read(regs + DMA_IRQ_STAT_REG_OFFSET);
		if (stat & mask) {
			flags |= DMA_FLAG_COMPLETE;
			vspa_reg_write(regs + DMA_IRQ_STAT_REG_OFFSET, mask);
			vspa_reg_write(regs + DMA_COMP_STAT_REG_OFFSET, mask);
			stat &= ~mask;
		}
		if (stat & spm_mask) {
			spm_flags |= DMA_FLAG_COMPLETE;
			vspa_reg_write(regs+DMA_IRQ_STAT_REG_OFFSET, spm_mask);
			vspa_reg_write(regs+DMA_COMP_STAT_REG_OFFSET, spm_mask);
			stat &= ~spm_mask;
		}
		/* Watch for completed DMAs from VSPA with incorrect IRQ_EN */
		if (stat) {
			ERR("%d: DMA IRQ STAT %08x\n", vspadev->id, stat);
			vspa_reg_write(regs + DMA_IRQ_STAT_REG_OFFSET, stat);
		}
	}
	/* Check for DMA errors from any channel */
	if (status & STATUS_REG_IRQ_DMA_ERR) {
		/* DMA transfer errors */
		stat = vspa_reg_read(regs + DMA_XFRERR_STAT_REG_OFFSET);
		if (stat) {
			if (stat & mask) /* Transfer error from DMA channel */
				flags |= DMA_FLAG_XFRERR;
			if (stat & spm_mask) /* Transfer error from SPM DMA */
				spm_flags |= DMA_FLAG_XFRERR;
			if (stat & ~(mask | spm_mask)) { /* Other channels */
				ERR("%d: DMA XFRERR %08x\n", vspadev->id, stat);
			}
			vspa_reg_write(regs+DMA_XFRERR_STAT_REG_OFFSET, stat);
		}
		/* DMA configuration errors */
		stat = vspa_reg_read(regs + DMA_CFGERR_STAT_REG_OFFSET);
		if (stat) {
			if (stat & mask) /* Config error from DMA channel */
				flags |= DMA_FLAG_CFGERR;
			if (stat & spm_mask) /* Config error from SPM DMA */
				spm_flags |= DMA_FLAG_CFGERR;
			if (stat & ~(mask | spm_mask)) { /* Other channels */
				ERR("%d: DMA CFGERR %08x\n", vspadev->id, stat);
			}
			vspa_reg_write(regs+DMA_CFGERR_STAT_REG_OFFSET, stat);
		}
	}

	if (flags) {
		dr = &(dq->entry[dq->idx_chk]);
		if (dr->type == VSPA_EVENT_DMA) {
			err = 0;
			if (flags & DMA_FLAG_CFGERR)
				err = VSPA_ERR_DMA_CFGERR;
			else if (flags & DMA_FLAG_XFRERR)
				err = VSPA_ERR_DMA_XFRERR;
			if (dr->flags & VSPA_FLAG_REPORT_DMA_COMPLETE || err) {
				event_enqueue(vspadev, VSPA_EVENT_DMA, dr->id,
					err, dr->dmem_addr, dr->byte_cnt);
			}
		} else if (dr->type == VSPA_EVENT_CMD) {
			if (dr->id >= MAX_SEQIDS)
				; // skip multiple DMAs
			else if (vspadev->seqid[dr->id].cmd_buffer_idx >= 0) {
				cbuffer_free(&vspadev->cmd_buffer,
					vspadev->seqid[dr->id].cmd_buffer_idx,
					vspadev->seqid[dr->id].cmd_buffer_size);
				vspadev->seqid[dr->id].cmd_buffer_idx = -1;
			}
		} else
			ERR("%d: unknown DMA type %d\n", vspadev->id, dr->type);
		dq->idx_chk = dq->idx_dma;
		dma_transmit(vspadev);
	}

	if (spm_flags) {
		spm_update(vspadev, spm_flags);
	}

//	spin_unlock(&vspadev->irq_lock);
	vspadev->irq_bits &= ~4;

	return IRQ_HANDLED;
}

static irqreturn_t vspa_gen_irq_handler(int irq, void *dev)
{
	struct vspa_device *vspadev = (struct vspa_device *)dev;
	u32 __iomem *regs = vspadev->regs;
	uint32_t irqen  = vspa_reg_read(regs + IRQEN_REG_OFFSET);
	uint32_t status = vspa_reg_read(regs + STATUS_REG_OFFSET);

//	spin_lock(&vspadev->irq_lock);
	if (vspadev->irq_bits) {
		pr_err("VSPA%d gen irqs = %d\n",
			vspadev->id, vspadev->irq_bits);
	}
	vspadev->irq_bits |= 8;

	vspa_reg_write(regs + STATUS_REG_OFFSET, status);
	pr_info("vspa%d: IRQEN %08x, STATUS %08x => %08X\n",
		vspadev->id, irqen, status,
		vspa_reg_read(regs + STATUS_REG_OFFSET));

//	spin_unlock(&vspadev->irq_lock);
	vspadev->irq_bits &= ~8;

	return IRQ_HANDLED;
}

/************************ Power Up / Down *************************/

static int powerdown(struct vspa_device *vspadev)
{
	u32 __iomem *regs = vspadev->regs;
	u32 __iomem *dbg_regs = vspadev->dbg_regs;
	int ret = 0;
	int ctr;
	int powered;

	/* check if the VSPA core is powered */
	powered = d4400_gpc_vspa_full_pow(vspadev->id);
	if (powered < 0)
		return powered;

	/* Disable all interrupts */
	if (powered)
		vspa_reg_write(regs + IRQEN_REG_OFFSET, 0x0);

	vspadev->state = VSPA_STATE_UNKNOWN;
	vspadev->versions.vspa_sw_version = ~0;
	vspadev->versions.ippu_sw_version = ~0;
	vspadev->eld_filename[0] = '\0';
	vspadev->watchdog_interval_msecs = VSPA_WATCHDOG_INTERVAL_DEFAULT;

	/* shut the timer down */
	mod_timer(&vspadev->watchdog_timer, jiffies);

	if (powered) {
		/* Enable the invasive (halting) debug mode */
		vspa_reg_write(dbg_regs + DBG_GDBEN_REG_OFFSET, 0x1);

		/* Stop all VSPA activities using “force_halt” */
		vspa_reg_write(dbg_regs + DBG_RCR_REG_OFFSET, 0x4);

		/* Wait for the “halted” bit to be set */
		for (ctr = VSPA_HALT_TIMEOUT; ctr; ctr--) {
			udelay(1);
			ret = vspa_reg_read(dbg_regs + DBG_RCSTATUS_REG_OFFSET)
								 & (1 << 13);
			if (ret)
				break;
		}
		if (!(ret)) {
			ERR("%d: powerdown() timeout waiting for halt\n",
								 vspadev->id);
			ret = -ETIME;
		} else {
			ret = d4400_gpc_vspa_full_pow_gate(vspadev->id);
		}
		if (ret)
			ERR("%d: powerdown() => %d\n", vspadev->id, ret);
	}
	vspadev->state = ret ? VSPA_STATE_UNKNOWN : VSPA_STATE_POWER_DOWN;
	return ret;
}

static int powerup(struct vspa_device *vspadev)
{
	int ret;
	u32 __iomem *regs = vspadev->regs;

	if (vspadev->state != VSPA_STATE_POWER_DOWN)
		return -EPERM;

	ret = d4400_gpc_vspa_full_pow_up(vspadev->id);
	if (ret) {
		ERR("%d: powerup() ret = %d\n", vspadev->id, ret);
		powerdown(vspadev);
	} else {
		/* Disable all interrupts */
		vspa_reg_write(regs + IRQEN_REG_OFFSET, 0x0);
		dma_reset_queue(vspadev);
		event_reset_queue(vspadev);
		mbox_reset(vspadev);
		cmd_reset(vspadev);
	}
	vspadev->state = ret ? VSPA_STATE_UNKNOWN :
					VSPA_STATE_UNPROGRAMMED_IDLE;
	return ret;
}

static int startup(struct vspa_device *vspadev)
{
	u32 __iomem *regs = vspadev->regs;
	uint32_t val, msb, lsb;
	uint32_t dma_channels;
	uint32_t vspa_sw_version, ippu_sw_version;
	int ctr;

	if (vspadev->state != VSPA_STATE_LOADING)
		return -EPERM;

	IF_DEBUG(DEBUG_STARTUP) {
		pr_info("vspa%d: startup()\n", vspadev->id);
		pr_info("Filename: '%s'\n", vspadev->eld_filename);
		pr_info("Command buffer: addr = %08X, bytes = %08X\n",
			vspadev->cmdbuf_addr, vspadev->cmd_pool.size * 4);
		pr_info("SPM %s buffer: addr = %p, bytes = %08X\n",
			vspadev->spm_user_buf ? "User" : "Driver",
			vspadev->spm_buf_paddr, vspadev->spm_buf_bytes);
	}

	/* Ask the VSPA to go */
	val = vspa_reg_read(regs + CONTROL_REG_OFFSET);
	val |= 1; // HOST_GO
	vspa_reg_write(regs + CONTROL_REG_OFFSET, val);

	/* Wait for the 64 bit mailbox bit to be set */
	for (ctr = VSPA_STARTUP_TIMEOUT; ctr; ctr--) {
		if (vspa_reg_read(regs + HOST_MBOX_STATUS_REG_OFFSET) &
							MBOX_STATUS_IN_64_BIT)
			break;
		udelay(1);
	}
	if (!ctr) {
		ERR("%d: timeout waiting for Boot Complete msg\n", vspadev->id);
		goto startup_fail;
	}
	msb = vspa_reg_read(regs + HOST_IN_64_MSB_REG_OFFSET);
	lsb = vspa_reg_read(regs + HOST_IN_64_LSB_REG_OFFSET);
	IF_DEBUG(DEBUG_STARTUP)
		pr_info("Boot Ok Msg: msb = %08X, lsb = %08X\n", msb, lsb);
	/* Check Boot Complete message */
	if (msb != 0xF1000000) {
		ERR("%d: Boot Complete msg did not match\n", vspadev->id);
		goto startup_fail;
	}
	dma_channels = lsb;

	vspa_sw_version = vspa_reg_read(regs + SWVERSION_REG_OFFSET);
	ippu_sw_version = vspa_reg_read(regs + IPPU_SWVERSION_REG_OFFSET);

	/* Set SPM buffer */
	msb = (0x70 << 24) | vspadev->spm_buf_bytes;
	lsb = (uint32_t)vspadev->spm_buf_paddr;
	vspa_reg_write(regs + HOST_OUT_64_MSB_REG_OFFSET, msb);
	vspa_reg_write(regs + HOST_OUT_64_LSB_REG_OFFSET, lsb);
	/* Wait for the 64 bit mailbox bit to be set */
	for (ctr = VSPA_STARTUP_TIMEOUT; ctr; ctr--) {
		if (vspa_reg_read(regs + HOST_MBOX_STATUS_REG_OFFSET) &
							MBOX_STATUS_IN_64_BIT)
			break;
		udelay(1);
	}
	if (!ctr) {
		ERR("%d: timeout waiting for SPM Ack msg\n", vspadev->id);
		goto startup_fail;
	}
	msb = vspa_reg_read(regs + HOST_IN_64_MSB_REG_OFFSET);
	lsb = vspa_reg_read(regs + HOST_IN_64_LSB_REG_OFFSET);
	IF_DEBUG(DEBUG_STARTUP)
		pr_info("SPM Ack Msg: msb = %08X, lsb = %08X\n", msb, lsb);
	if (msb != 0xF0700000) {
		ERR("%d: SPM Ack error %08X\n", vspadev->id, msb);
		goto startup_fail;
	}

	if (dma_channels) {
		vspadev->spm_dma_chan   = (dma_channels >> 24) & 0xFF;
		vspadev->bulk_dma_chan  = (dma_channels >> 16) & 0xFF;
		vspadev->reply_dma_chan = (dma_channels >>  8) & 0xFF;
		vspadev->cmd_dma_chan   = (dma_channels      ) & 0xFF;
	} else { /* legacy images */
		vspadev->spm_dma_chan   = VSPA_DMA_CHANNELS;
		vspadev->bulk_dma_chan  = 0xFF;
		vspadev->reply_dma_chan = 0xFF;
		vspadev->cmd_dma_chan   = vspadev->legacy_cmd_dma_chan;
	}

	IF_DEBUG(DEBUG_STARTUP) {
		pr_info("SW Version: vspa = %08X, ippu = %08X\n",
				vspa_sw_version, ippu_sw_version);
		pr_info("DMA chan: spm %02X, bulk %02X, reply %02X, cmd %02X\n",
			vspadev->spm_dma_chan, vspadev->bulk_dma_chan,
			vspadev->reply_dma_chan, vspadev->cmd_dma_chan);
	}

	vspadev->versions.vspa_sw_version = vspa_sw_version;
	vspadev->versions.ippu_sw_version = ippu_sw_version;
	vspadev->watchdog_value =
			~(vspa_reg_read(regs + GP_OUT3_REG_OFFSET) >> 16);
	vspadev->state = VSPA_STATE_RUNNING_IDLE;
	vspa_enable_mailbox_irqs(vspadev);
	init_completion(&vspadev->watchdog_complete);
	watchdog_callback((unsigned long) vspadev);
	return 0;

startup_fail:
	vspadev->versions.vspa_sw_version = ~0;
	vspadev->versions.ippu_sw_version = ~0;
	vspadev->state = VSPA_STATE_STARTUP_ERR;
	vspa_reg_write(vspadev->regs + IRQEN_REG_OFFSET, 0);
	return -EIO;
}

static int full_state(struct vspa_device *vspadev)
{
	int state = vspadev->state;
	if (state == VSPA_STATE_UNPROGRAMMED_IDLE ||
	    state == VSPA_STATE_RUNNING_IDLE) {
		if (vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET) & 0x100)
			state++;
	}
	return state;
}

/************************** System Functions ************************/

static int vspa_open(struct inode *inode, struct file *fp)
{
	struct vspa_device *vspadev = NULL;
	int rc = 0;

	vspadev = container_of(inode->i_cdev, struct vspa_device, cdev);
	if (vspadev != NULL) {
		fp->private_data = vspadev;
		event_list_update(vspadev);
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
	int rc = 0;

	if (vspadev != NULL) {
		event_list_update(vspadev);
	} else {
		dev_err(vspadev->dev, "No device context found for %s %d\n",
					VSPA_DEVICE_NAME, iminor(inode));
		rc = -ENODEV;
	}

	return 0;
}

static long vspa_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct vspa_device *vspadev = (struct vspa_device *)fp->private_data;
	u32 __iomem *regs = vspadev->regs;
	u32 __iomem *dbg_regs = vspadev->dbg_regs;
	struct vspa_dma_req dma_req;
	struct vspa_startup startup_desc;
	struct vspa_reg reg;
	struct vspa_mb32 mb32;
	struct vspa_mb64 mb64;
	struct vspa_event_read evt_rd;
	uint32_t align_bytes = vspadev->hardware.axi_data_width >> 3;
	int state;

	IF_DEBUG(DEBUG_IOCTL)
		pr_info("vspa%d: cmd %08x, arg %08lx\n", vspadev->id, cmd, arg);

	/* Ignore invalid commands */
	if (_IOC_TYPE(cmd) != VSPA_MAGIC_NUM) return -ENOTTY;
	if (_IOC_NR(cmd)   >  VSPA_IOC_MAX) return -ENOTTY;

	/* Check user space memory access is valid */
	if (_IOC_DIR(cmd) & _IOC_READ)
		rc = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		rc = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (rc) return -EFAULT;

	event_list_update(vspadev);

	switch (cmd) {
	case VSPA_IOC_GET_HW_CFG:
		rc = copy_to_user((struct vspa_hardware*)arg,
			&vspadev->hardware, sizeof(struct vspa_hardware));
		if (rc)
			rc = -EFAULT;
		break;

	case VSPA_IOC_GET_STATE:
		state = full_state(vspadev);
		rc = __put_user(state, (int*)arg);
		if (rc)
			rc = -EFAULT;
		break;

	case VSPA_IOC_REG_READ:
		rc = copy_from_user(&reg, (struct vspa_reg*)arg, sizeof(reg));
		if (rc)
			rc = -EFAULT;
		else if (reg.reg >= 0x400)
			rc = -EINVAL;
		else {
			uint32_t r = reg.reg;
			if (r < 0x200)
				reg.val = vspa_reg_read(regs + r);
			else
				reg.val = vspa_reg_read(dbg_regs + (r&0x1FF));
			rc = copy_to_user((struct vspa_reg*)arg, &reg,
								sizeof(reg));
			if (rc)
				rc = -EFAULT;
		}
		break;

	case VSPA_IOC_REG_WRITE:
		rc = copy_from_user(&reg, (struct vspa_reg*)arg, sizeof(reg));
		if (rc)
			rc = -EFAULT;
		else if (reg.reg >= 0x400)
			rc = -EINVAL;
		else {
			uint32_t r = reg.reg;
			if (r < 0x200)
				vspa_reg_write(regs + r, reg.val);
			else
				vspa_reg_write(dbg_regs + (r & 0x1FF),
								reg.val);
		}
		break;

	case VSPA_IOC_GET_VERSIONS:
		rc = copy_to_user((struct vspa_versions*)arg,
				  &vspadev->versions,
				  sizeof(struct vspa_versions));
		if (rc)
			rc = -EFAULT;
		break;

	case VSPA_IOC_REQ_POWER:
		IF_DEBUG(DEBUG_IOCTL)
			pr_info("vspa%d: power %ld\n", vspadev->id, arg);
		spin_lock(&vspadev->control_lock);
		switch (arg) {
		case VSPA_POWER_DOWN:  rc = powerdown(vspadev); break;
		case VSPA_POWER_CYCLE: rc = powerdown(vspadev); /*FALLTHROUGH*/
		case VSPA_POWER_UP:    if (rc==0) rc = powerup(vspadev); break;
		default:	       rc = -EINVAL; break;
		}
		spin_unlock(&vspadev->control_lock);
		break;

	case VSPA_IOC_DMA:
		if (vspadev->state == VSPA_STATE_UNPROGRAMMED_IDLE) {
			vspadev->state = VSPA_STATE_LOADING;
			/* Enable DMA error and DMA complete interrupts */
			vspa_enable_dma_irqs(vspadev);
		} else if (vspadev->state < VSPA_STATE_LOADING)
			return -EPERM;

		rc = copy_from_user(&dma_req, (struct vspa_dma_req*)arg,
							sizeof(dma_req));
		if (rc)
			rc = -EFAULT;
		else {
			if ((dma_req.dmem_addr & (align_bytes - 1)) ||
			    (dma_req.axi_addr & (align_bytes - 1))) {
				rc = -EINVAL;
			} else {
				dma_req.type = VSPA_EVENT_DMA;
				dma_req.xfr_ctrl = (dma_req.xfr_ctrl & ~0x1F) |
						   vspadev->cmd_dma_chan;
				rc = dma_enqueue(vspadev, &dma_req);
			}
		}

/*pr_info("vspa%d: ctrl %08x, dmem %08x, axi %08x, cnt %08x, ctrl %08x\n",
	 vspadev->id, dma_req.control, dma_req.dmem_addr, dma_req.axi_addr, dma_req.byte_cnt, dma_req.xfr_ctrl);
*/
		break;

	case VSPA_IOC_STARTUP:
		if (vspadev->state != VSPA_STATE_LOADING)
			return -EPERM;

		rc = copy_from_user(&startup_desc,
			(struct vspa_startup*)arg, sizeof(startup_desc));
		if (rc)
			rc = -EFAULT;
		else if ((startup_desc.cmd_buf_size & (align_bytes - 1)) ||
			 (startup_desc.cmd_buf_addr & (align_bytes - 1)))
			rc = -EINVAL;
		else if (startup_desc.spm_buf_size && (
			 (startup_desc.spm_buf_size & (align_bytes - 1)) ||
			 (startup_desc.spm_buf_addr & (align_bytes - 1))))
			rc = -EINVAL;
		if (rc)
			 break;

		spin_lock(&vspadev->control_lock);
		IF_DEBUG(DEBUG_IOCTL)
			pr_info("vspa%d: cmd_addr %08x, cmd_size %08x,"
				" cmd_dma_chan %d, file '%s'\n",
				vspadev->id, startup_desc.cmd_buf_addr,
				startup_desc.cmd_buf_size,
				startup_desc.cmd_dma_chan,
				startup_desc.filename);

		vspadev->legacy_cmd_dma_chan=startup_desc.cmd_dma_chan;
		vspadev->cmdbuf_addr = startup_desc.cmd_buf_addr;
		vspadev->cmd_pool.size = startup_desc.cmd_buf_size / 4;
		if (vspadev->spm_user_buf) {
			vspadev->spm_user_buf = 0;
			iounmap(vspadev->spm_buf_vaddr);
		}
		if (startup_desc.spm_buf_size) {
			vspadev->spm_user_buf = 1;
			vspadev->spm_buf_bytes = startup_desc.spm_buf_size;
			vspadev->spm_buf_paddr =
					(uint32_t*)startup_desc.spm_buf_addr;
			vspadev->spm_buf_vaddr = ioremap(
						(int)vspadev->spm_buf_paddr,
						vspadev->spm_buf_bytes);
		} else {
			vspadev->spm_buf_bytes = vspadev->spm_buffer_bytes;
			vspadev->spm_buf_paddr = vspadev->spm_buffer_paddr;
			vspadev->spm_buf_vaddr = vspadev->spm_buffer_vaddr;
		}
		vspadev->spm_addr = (vspadev->spm_buf_bytes - 4) >> 2;
		memset(vspadev->spm_buf_vaddr, 0, vspadev->spm_buf_bytes);

		vspadev->watchdog_interval_msecs =
						VSPA_WATCHDOG_INTERVAL_DEFAULT;
		strncpy(vspadev->eld_filename, startup_desc.filename,
							VSPA_MAX_ELD_FILENAME);
		vspadev->eld_filename[VSPA_MAX_ELD_FILENAME-1] = '\0';
		pool_reset(&vspadev->cmd_pool);
		rc = startup(vspadev);
		spin_unlock(&vspadev->control_lock);
		break;

	case VSPA_IOC_MB32_WRITE:
		if (vspadev->state != VSPA_STATE_RUNNING_IDLE)
			return -EPERM;
		rc = copy_from_user(&mb32,
			(struct vspa_mb32*)arg, sizeof(mb32));
		if (rc)
			rc = -EFAULT;
		else {
			int next_slot;
			spin_lock(&vspadev->control_lock);
			next_slot = mbox_next_slot(&vspadev->mb32_queue);
			if (next_slot < 0) {
				ERR("%d: mb32 queue full\n", vspadev->id);
				rc = -ENOMEM;
			} else {
				vspadev->mb32[next_slot] = mb32;
				vspadev->mb32_queue.idx_enqueue = next_slot;
				IF_DEBUG(DEBUG_MBOX32_OUT)
					pr_info("vspa%d: mbox32 out idx %d: "
						"%08x queued/n",
						vspadev->id, next_slot,
						mb32.data);
				mb32_transmit(vspadev);
			}
			spin_unlock(&vspadev->control_lock);
		}
		break;

	case VSPA_IOC_MB64_WRITE:
		if (vspadev->state != VSPA_STATE_RUNNING_IDLE)
			return -EPERM;
		rc = copy_from_user(&mb64,
			(struct vspa_mb64*)arg, sizeof(mb64));
		if (rc)
			rc = -EFAULT;
		else {
			int next_slot;
			spin_lock(&vspadev->control_lock);
			next_slot = mbox_next_slot(&vspadev->mb64_queue);
			if (next_slot < 0) {
				ERR("%d: mb64 queue full\n", vspadev->id);
				rc = -ENOMEM;
			} else {
				vspadev->mb64[next_slot] = mb64;
				vspadev->mb64_queue.idx_enqueue = next_slot;
				IF_DEBUG(DEBUG_MBOX64_OUT)
					pr_info("vspa%d: mbox64 out idx %d: "
						"%08x %08x queued/n",
						vspadev->id, next_slot,
						mb64.data_msb, mb64.data_lsb);
				mb64_transmit(vspadev);
			}
			spin_unlock(&vspadev->control_lock);
		}
		break;

	case VSPA_IOC_WATCHDOG_INT:
		if (arg > VSPA_WATCHDOG_INTERVAL_MAX)
			rc = -EINVAL;
		else {
			if (arg >= 0)
				arg = VSPA_WATCHDOG_INTERVAL_DEFAULT;
			else if (arg < VSPA_WATCHDOG_INTERVAL_MIN)
				arg = VSPA_WATCHDOG_INTERVAL_MIN;
			vspadev->watchdog_interval_msecs = arg;
			if (vspadev->state == VSPA_STATE_RUNNING_IDLE) {
				mod_timer(&vspadev->watchdog_timer,
					jiffies + msecs_to_jiffies(arg));
			}
		}
		break;

	case VSPA_IOC_SET_POLL_MASK:
		arg &= VSPA_MSG_ALL_EVENTS;
		vspadev->poll_mask = arg ? arg : VSPA_MSG_ALL_EVENTS;
		break;

	case VSPA_IOC_EVENT_READ:
		rc = copy_from_user(&evt_rd,
			(struct vspa_event_read*)arg, sizeof(evt_rd));
		if (rc)
			rc = -EFAULT;
		else {
			rc = read_event(vspadev, &evt_rd);
//pr_err("read_event mask %04X, rc %d\n", evt_rd.event_mask, rc);
			if (rc > 0) rc = 0;
		}
		break;

	case VSPA_IOC_GET_DEBUG:
		rc = __put_user(vspadev->debug, (int*)arg);
		if (rc)
			rc = -EFAULT;
		break;

	case VSPA_IOC_SET_DEBUG:
		vspadev->debug = arg;
		break;

	default:
		rc = -ENOTTY;
		break;
	}

	return rc;
}

static unsigned int vspa_poll(struct file *fp, poll_table *wait)
{
	unsigned int mask = 0;
	struct vspa_device *vspadev = (struct vspa_device *)fp->private_data;
	event_list_update(vspadev);

	spin_lock(&vspadev->event_list_lock);

	poll_wait(fp, &vspadev->event_wait_q, wait);

	if (vspadev->state == VSPA_STATE_RUNNING_IDLE)
		mask |= (POLLOUT | POLLWRNORM);
	if (vspadev->event_list_mask & vspadev->poll_mask)
		mask |= (POLLIN | POLLRDNORM);
	else if (vspadev->state < VSPA_STATE_LOADING)
		mask = POLLHUP;

	spin_unlock(&vspadev->event_list_lock);

	return mask;
}

static ssize_t vspa_read(struct file *fp, char __user *buf,
			size_t len, loff_t *off)
{
	struct vspa_device *vspadev = (struct vspa_device *)fp->private_data;
	struct vspa_event_read evt_rd;
	int ret;

	evt_rd.event_mask = (uint32_t)off;
	evt_rd.timeout = (fp->f_flags & O_NONBLOCK) ? 0 : -1;
	evt_rd.buf_len = len;
	evt_rd.buf_ptr = (struct vspa_event*)buf;

	ret = read_event(vspadev, &evt_rd);
	return (ret == -ENODATA) ? 0 : ret;
}

static ssize_t vspa_write(struct file *fp, const char __user *buf,
			 size_t len, loff_t *off)
{
	struct vspa_device *vspadev = (struct vspa_device *)fp->private_data;
	int err;
	int8_t index;
	uint32_t *dmabuf;
	int seqid;
	int words;
	struct vspa_dma_req dr;
	int i;
	int start_written = 1;
	uint32_t start_addr;
	int cb_size;
	int cb_idx;
	int cmd_id = 0;
	uint32_t flags = 0;
	int reply_idx = -1;
	int align_bytes = vspadev->hardware.axi_data_width >> 3;
	int align_words = vspadev->hardware.axi_data_width >> 5;

	IF_DEBUG(DEBUG_IOCTL)
		pr_info("vspa%d: vspa_write() len = %d\n", vspadev->id, len);

	event_list_update(vspadev);

	if (vspadev->state != VSPA_STATE_RUNNING_IDLE)
		return -EPERM;

	if (len > CMD_MAX_SZ_BYTES) {
		ERR("%d: command exceeds %d bytes\n", vspadev->id,
							CMD_MAX_SZ_BYTES);
		return -EMSGSIZE;
	}

	if ((len & 3) != 0 || len == 0) {
		ERR("%d: command must be multiple of 4 bytes and non zero\n",
							 vspadev->id);
		return -EINVAL;
	}

	spin_lock(&vspadev->control_lock);
	seqid = seqid_get_next(vspadev);
	if (seqid < 0) {
		ERR("%d: no sequence id available\n", vspadev->id);
		err = -ENOSR;
		goto seqid_fail;
	}

	pool_consolidate(&vspadev->cmd_pool, vspadev->last_wstart);
	pool_consolidate(&vspadev->reply_pool, -1);

	words = len/4;
	if (vspadev->first_cmd) words++;
	words = align_words * ((words + align_words - 1) / align_words);
	index = pool_get_bd(&vspadev->cmd_pool, words);
	IF_DEBUG(DEBUG_CMD_BD)
		pool_print(&vspadev->cmd_pool);
	if (index < 0) {
		ERR("%d: no cmd pool bd available\n", vspadev->id);
		err = -ENOSPC;
		goto seqid_release;
	}

	cb_size = words + 4;
	cb_idx = cbuffer_add(&vspadev->cmd_buffer, cb_size);
	if (cb_idx < 0) {
		ERR("%d: no cmd buffer space available\n", vspadev->id);
		err = -ENOBUFS;
		goto pool_free;
	}

	start_addr = vspadev->cmd_pool.bd[index].wstart + 1;
	dmabuf = &vspadev->cmd_buffer.vaddr[cb_idx];
	/* on failure, set the len to 0 to return empty packet to the device */
	err = copy_from_user(dmabuf, buf, len);
	if (err) {
		ERR("%d: write() copy_from_user() failed\n", vspadev->id);
	} else {
		flags = dmabuf[0] & 0xFF;
		if (flags & VSPA_FLAG_REPORT_CMD_REPLY)
			flags |= VSPA_FLAG_EXPECT_CMD_REPLY;
		if ((dmabuf[2] > 0 && dmabuf[3] == 0) ||
		    (dmabuf[3] & (align_bytes - 1)) ||
		    (dmabuf[5] & (align_bytes - 1))) {
			ERR("%d: buffers must be AXI aligned\n", vspadev->id);
			err = -EINVAL;
			goto pool_free;
		}
		if (dmabuf[4] > 0) {
			flags |= VSPA_FLAG_EXPECT_CMD_REPLY;
			if (dmabuf[5] == 0) {
				int reply_words = (dmabuf[4]+3)>>2;
				reply_words = align_words * ((reply_words +
						align_words - 1) / align_words);
				reply_idx = pool_get_bd(&vspadev->reply_pool,
								 reply_words);
				IF_DEBUG(DEBUG_REPLY_BD)
					pool_print(&vspadev->reply_pool);
				if (reply_idx < 0) {
					ERR("%d: no reply pool bd available\n",
						vspadev->id);
					err = -ENOSPC;
					goto pool_free;
				}
				dmabuf[5] = (unsigned int)
					&vspadev->reply_pool.paddr[
				vspadev->reply_pool.bd[reply_idx].wstart ];
			}
		}
		cmd_id = dmabuf[1] >> 24;
		vspadev->seqid[seqid].flags = flags;
		vspadev->seqid[seqid].cmd_id = cmd_id;
		vspadev->seqid[seqid].cmd_buffer_idx = cb_idx;
		vspadev->seqid[seqid].cmd_buffer_size = cb_size;
		vspadev->seqid[seqid].cmd_bd_index = index;
		vspadev->seqid[seqid].reply_bd_index = reply_idx;
		vspadev->seqid[seqid].reply_size = dmabuf[4];
		vspadev->seqid[seqid].payload1 = dmabuf[6];

		dr.dmem_addr = vspadev->cmd_pool.bd[index].wstart * 4 +
							vspadev->cmdbuf_addr;
		dr.axi_addr  = (unsigned int)&vspadev->cmd_buffer.paddr[cb_idx];
		dr.xfr_ctrl = 0x2000; // VCPU_GO
		dr.xfr_ctrl |= vspadev->cmd_dma_chan;

		dmabuf[0] = 0;
		dmabuf[1] = (dmabuf[1] & 0x00FFFFFF) | (seqid << 24);
		dmabuf[words] = start_addr;
		for (i = len/4; i < words; i++) /* filler words */
			dmabuf[i] = 0;
		if (vspadev->first_cmd)
			dmabuf[words-1] = start_addr;
		else if ((vspadev->cmd_pool.bd[index].wstart + words) ==
							vspadev->last_wstart)
			words++;
		else
			start_written = 0;

		dr.byte_cnt  = words * 4;

		IF_DEBUG(DEBUG_CMD) {
			pr_info("vspa%d: CmdSeqId %02X, flags %04X\n",
				 vspadev->id, cmd_id, flags);
			for (i = 0; i < words; i++)
				pr_info("%02d [%02X] = %08X\n", i,
					vspadev->cmd_pool.bd[index].wstart + i,
					dmabuf[i]);
		}

		dr.type = VSPA_EVENT_CMD;
		dr.id = start_written ? seqid : 0xFF;
		dr.flags = flags;
		err = dma_enqueue(vspadev, &dr);
		if (err)
			ERR("%d: write() dma_enqueue() failed\n", vspadev->id);
	}

	/* Start pointer location is not contiguous so do another DMA */
	if (!err && !start_written) {
		dr.dmem_addr = vspadev->last_wstart * 4 + vspadev->cmdbuf_addr;
		dr.axi_addr  = (unsigned int)
				&vspadev->cmd_buffer.paddr[cb_idx + words];
		dr.byte_cnt  = 4;
		IF_DEBUG(DEBUG_CMD)
			pr_info("-- [%02X] = %08X\n", dr.dmem_addr / 4,
								dmabuf[words]);
		dr.type = VSPA_EVENT_CMD;
		dr.id = seqid;
		dr.flags = flags;
		err = dma_enqueue(vspadev, &dr);
		if (err)
			ERR("%d: write() dma_enqueue()2 failed\n", vspadev->id);
	}

	if (err) {
		seqid_release(vspadev, seqid);
	} else {
		vspadev->first_cmd = 0;
		vspadev->last_wstart = start_addr - 1;
	}
	spin_unlock(&vspadev->control_lock);

	return err ? err : len;

pool_free:
	pool_free_bd(&vspadev->cmd_pool, index);
seqid_release:
	seqid_release(vspadev, seqid);
seqid_fail:
	spin_unlock(&vspadev->control_lock);
	return err;
}

static int vspa_mmap(struct file *fp, struct vm_area_struct *vma)
{
	unsigned long off = 0, phy_addr = 0;
	int rc = 0;
	struct vspa_device *vspadev = fp->private_data;
	unsigned long start  = 0, len = 0;

	if (NULL == vspadev)
		return -ENODEV;

	/* Based on Offset we decide, if we map the vspa registers
	 * (VSPA_DBG_OFFSET) 0: VSPA debug resister set
	 * (VSPA_REG_OFFSET) 4096: VSPA IP register set
	 * */
	if (vma->vm_pgoff == (VSPA_DBG_OFFSET >> PAGE_SHIFT)) {

		phy_addr = (unsigned long)vspadev->dbg_addr;

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + vspadev->dbg_size);

		/* These are IO addresses */
		vma->vm_flags |= VM_IO;

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	} else if (vma->vm_pgoff == (VSPA_REG_OFFSET >> PAGE_SHIFT)) {

		phy_addr = (unsigned long)vspadev->mem_addr;

		/* Align the start address */
		start = phy_addr & PAGE_MASK;

		/* Align the size to PAGE Size */
		len = PAGE_ALIGN((start & ~PAGE_MASK) + vspadev->mem_size);

		/* These are IO addresses */
		vma->vm_flags |= VM_IO;

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

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

	return 0;
}

static const struct file_operations vspa_fops = {
	.owner = THIS_MODULE,
	.open = vspa_open,
	.mmap = vspa_mmap,
	.poll = vspa_poll,
	.read = vspa_read,
	.write = vspa_write,
	.unlocked_ioctl = vspa_ioctl,
	.release = vspa_release,
};

/***************************** Sysfs *******************************/
//TODO - complete sysfs: registers, bds

static ssize_t show_stat(struct device *dev,
			struct device_attribute *devattr, char *buf);

static ssize_t show_text(struct device *dev,
			struct device_attribute *devattr, char *buf);

static ssize_t set_debug(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct vspa_device *vspadev = dev_get_drvdata(dev);
	int err;
	unsigned int val;

	err = kstrtouint(buf, 0, &val);
	if (err)
		return err;

	vspadev->debug = val;
	return count;
}

static ssize_t show_debug(struct device *dev,
                        struct device_attribute *devattr, char *buf)
{
        struct vspa_device *vspadev = dev_get_drvdata(dev);
        return sprintf(buf, "0x%08X\n", vspadev->debug);
}

static ssize_t show_versions(struct device *dev,
                        struct device_attribute *devattr, char *buf)
{
        struct vspa_device *vspadev = dev_get_drvdata(dev);
        return sprintf(buf, "0x%08X 0x%08X 0x%08X 0x%08X\n",
			vspadev->versions.vspa_hw_version,
			vspadev->versions.ippu_hw_version,
			vspadev->versions.vspa_sw_version,
			vspadev->versions.ippu_sw_version);
}

static const char *event_name[16] = {
	"?00?", "DMA", "CMD", "REPLY", "SPM", "MB64I", "MB32I", "MB64O",
	"MB32O", "ERR", "?10?", "?11?", "?12?", "?13?", "?14?", "?15?"
};

static ssize_t show_events(struct device *dev,
                        struct device_attribute *devattr, char *buf)
{
        struct vspa_device *vspadev = dev_get_drvdata(dev);
	struct event_list *ptr;
	size_t ctr, len, total_len;

	event_list_update(vspadev);

	spin_lock(&vspadev->event_list_lock);
	ctr = 0;
	total_len = 0;
	list_for_each_entry(ptr, &vspadev->events_queued_list, list) {
		len = sprintf(buf, "[%2d] %-5s %02X %02X %08X %08X\n",
			ctr++, event_name[ptr->type & 0xF], ptr->id,
			ptr->err, ptr->data0, ptr->data1);
		total_len += len;
		buf += len;
		if (total_len >= (PAGE_SIZE - 50)) {
			len = sprintf(buf, "...\n");
			total_len += len;
			break;
		}
	}
	spin_unlock(&vspadev->event_list_lock);
	return total_len;
}

static ssize_t show_seqids(struct device *dev,
                        struct device_attribute *devattr, char *buf)
{
        struct vspa_device *vspadev = dev_get_drvdata(dev);
	struct seqid *s;
	int i;
	size_t len, total_len;

	spin_lock(&vspadev->control_lock);
	total_len = 0;
	for (i = 0; i < 32; i++) {
		if (vspadev->active_seqids & (1<<i)) {
			s = &(vspadev->seqid[i]);
			len = sprintf(buf, "[%2d] %02X %04X %2d %4d %4d %2d %4d %08X\n",
				i, s->cmd_id, s->flags, s->cmd_bd_index,
				s->cmd_buffer_idx, s->cmd_buffer_size,
				s->reply_bd_index, s->reply_size, s->payload1);
			total_len += len;
			buf += len;
			if (total_len >= (PAGE_SIZE - 80)) {
				len = sprintf(buf, "...\n");
				total_len += len;
				break;
			}
		}
	}
	spin_unlock(&vspadev->control_lock);
	return total_len;
}

static DEVICE_ATTR(debug,  S_IWUSR | S_IRUGO, show_debug,    set_debug);
static DEVICE_ATTR(eld_filename,     S_IRUGO, show_text,     NULL);
static DEVICE_ATTR(events,           S_IRUGO, show_events,   NULL);
static DEVICE_ATTR(seqids,           S_IRUGO, show_seqids,   NULL);
static DEVICE_ATTR(state,            S_IRUGO, show_stat,     NULL);
static DEVICE_ATTR(state_name,       S_IRUGO, show_text,     NULL);
static DEVICE_ATTR(versions,         S_IRUGO, show_versions, NULL);

static struct attribute *attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_eld_filename.attr,
	&dev_attr_events.attr,
	&dev_attr_seqids.attr,
	&dev_attr_state.attr,
	&dev_attr_state_name.attr,
	&dev_attr_versions.attr,
	NULL
};

static const struct attribute_group attr_group = {
	.attrs = attributes,
};

static ssize_t show_stat(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	unsigned int val;
	struct vspa_device *vspadev = dev_get_drvdata(dev);
	if (devattr == &dev_attr_state)	val = full_state(vspadev);
	else val = 0;
	return sprintf(buf, "%d\n", val);
}

static const char *state_name[] = {
	"unknown", "powerdown", "unprogrammed_idle", "unprogrammed_busy",
	"loading", "startup_error", "running_idle", "running_busy"
};

static ssize_t show_text(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	const char *ptr;
	struct vspa_device *vspadev = dev_get_drvdata(dev);
	if (devattr == &dev_attr_eld_filename)	  ptr = vspadev->eld_filename;
	else if (devattr == &dev_attr_state_name) ptr =
						state_name[full_state(vspadev)];
	else ptr = "??";
	return sprintf(buf, "%s\n", ptr == NULL ? "NULL" : ptr);
}

/************************* Probe / Remove **************************/

static int
vspa_getdts_properties(struct device_node *np, struct vspa_device *vspadev)
{
	u32 prop  = 0;

	if (!np)
		return -EINVAL;

	if (of_property_read_u32(np, "vspadev-id", &prop) < 0) {
		dev_err(vspadev->dev, "vspadev-id attribute not found\n");
		return -EINVAL;
	}
	vspadev->id = prop;

	if (of_property_read_u32(np, "dbgregstart", &prop) < 0) {
		dev_err(vspadev->dev, "dbgregstart attribute not found for %s%d\n",
				VSPA_DEVICE_NAME, vspadev->id);
		return -EINVAL;
	}
	vspadev->dbg_addr = (u32 *)prop;

	if (of_property_read_u32(np, "dbgreglen", &prop) < 0) {
		dev_err(vspadev->dev, "dbgreglen attribute not found for %s%d\n",
				VSPA_DEVICE_NAME, vspadev->id);
		return -EINVAL;
	}
	vspadev->dbg_size = prop;

	vspadev->cmd_buffer_bytes = cmd_buffer_bytes;
	if (of_property_read_u32(np, "cmd_buf_size", &prop) >= 0)
		vspadev->cmd_buffer_bytes = prop;
	if (vspadev->cmd_buffer_bytes > (32*1024) ||
	    vspadev->cmd_buffer_bytes & 255) {
		dev_err(vspadev->dev, "invalid cmd_buffer_size %d for %s%d\n",
			vspadev->cmd_buffer_bytes, VSPA_DEVICE_NAME,
			vspadev->id);
		return -EINVAL;
	}

	vspadev->reply_buffer_bytes = reply_buffer_bytes;
	if (of_property_read_u32(np, "reply_buf_size", &prop) >= 0)
		vspadev->reply_buffer_bytes = prop;
	if (vspadev->reply_buffer_bytes > (32*1024) ||
	    vspadev->reply_buffer_bytes & 255) {
		dev_err(vspadev->dev, "invalid reply_buffer_size %d for %s%d\n",
			vspadev->reply_buffer_bytes, VSPA_DEVICE_NAME,
			vspadev->id);
		return -EINVAL;
	}

	vspadev->spm_buffer_bytes = spm_buffer_bytes;
	if (of_property_read_u32(np, "spm_buf_size", &prop) >= 0)
		vspadev->spm_buffer_bytes = prop;
	if (vspadev->spm_buffer_bytes > (32*1024) ||
	    vspadev->spm_buffer_bytes & 255) {
		dev_err(vspadev->dev, "invalid spm_buffer_size %d for %s%d\n",
			vspadev->spm_buffer_bytes, VSPA_DEVICE_NAME,
			vspadev->id);
		return -EINVAL;
	}

	if (of_get_property(np, "interrupts", NULL)) {
		vspadev->flags1_irq_no    = irq_of_parse_and_map(np, 0);
		if (vspadev->flags1_irq_no == 0) {
			dev_err(vspadev->dev, "Interrupt 0 not found for %s%d\n",
				 VSPA_DEVICE_NAME, vspadev->id);
			return -EINVAL;
		}
		vspadev->flags0_irq_no = irq_of_parse_and_map(np, 1);
		if (vspadev->flags0_irq_no == 0) {
			dev_err(vspadev->dev, "Interrupt 1 not found for %s%d\n",
				 VSPA_DEVICE_NAME, vspadev->id);
			return -EINVAL;
		}
		vspadev->dma_irq_no       = irq_of_parse_and_map(np, 2);
		if (vspadev->dma_irq_no == 0) {
			dev_err(vspadev->dev, "Interrupt 2 not found for %s%d\n",
				 VSPA_DEVICE_NAME, vspadev->id);
			return -EINVAL;
		}
		vspadev->general_irq_no   = irq_of_parse_and_map(np, 3);
		if (vspadev->general_irq_no == 0) {
			dev_err(vspadev->dev, "Interrupt 3 not found for %s%d\n",
				 VSPA_DEVICE_NAME, vspadev->id);
			return -EINVAL;
		}
	} else {
		dev_err(vspadev->dev, "Interrupt numbers not found for %s%d\n",
				 VSPA_DEVICE_NAME, vspadev->id);
		return -EINVAL;
	}

	return 0;
}

static int vspa_probe(struct platform_device *pdev)
{
	struct vspa_device *vspadev = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct vspa_hardware *hw;
	struct device *sysfs_dev;
	struct resource res;
	dev_t devno;
	u8 device_name[10];
	int err = 0;
	dma_addr_t dma_paddr;
	size_t dma_size;
	uint32_t* dma_vaddr;
	uint32_t param0, param1, param2;

	BUG_ON(vspa_major == 0 || vspa_class == NULL);

	/* Allocate vspa device structure */
	vspadev = kzalloc(sizeof(struct vspa_device), GFP_KERNEL);
	if (!vspadev) {
		ERR(": failed to allocate vspa_dev\n");
		err = -ENOMEM;
		goto err_out;
	}

	vspadev->dev = dev;

	if (vspa_getdts_properties(np, vspadev) < 0) {
		dev_err(dev, "VSPA DTS entry parse failed.\n");
		err = -EINVAL;
		goto err_struct;
	}

	devno = MKDEV(vspa_major, vspa_minor + vspadev->id);
	sprintf(device_name, VSPA_DEVICE_NAME "%d", vspadev->id);

	/* Setup DMA regions */
	dma_size = vspadev->spm_buffer_bytes +
		   vspadev->cmd_buffer_bytes +
		   vspadev->reply_buffer_bytes;
	dma_vaddr = dma_zalloc_coherent(dev, dma_size, &dma_paddr,
					GFP_DMA);
	if (dma_vaddr == NULL) {
		dev_err(dev, "Failed to setup DMA buffer.\n");
		err = -ENOMEM;
		goto err_struct;
	}

	if (of_address_to_resource(np, 0, &res)) {
		err = -EINVAL;
		goto err_of;
	}

	dev_set_drvdata(&pdev->dev, vspadev);
	vspadev->state = VSPA_STATE_UNKNOWN;
	vspadev->debug = DEBUG_MESSAGES; // TODO set to 0
	vspadev->spm_buffer_paddr = &((uint32_t*)dma_paddr)[0];
	vspadev->spm_buffer_vaddr = &dma_vaddr[0];
	vspadev->spm_user_buf = 0;
	cbuffer_init(&vspadev->cmd_buffer,
		     vspadev->cmd_buffer_bytes >> 2,
		     &((uint32_t*)dma_paddr)[vspadev->spm_buffer_bytes >> 2],
	             &dma_vaddr[vspadev->spm_buffer_bytes>>2]);
	pool_init(&vspadev->cmd_pool, 256, 0, 0);
	pool_init(&vspadev->reply_pool,
		     vspadev->reply_buffer_bytes >> 2,
		     &((uint32_t*)dma_paddr)[(vspadev->spm_buffer_bytes >> 2) +
					     (vspadev->cmd_buffer_bytes >> 2)],
		     &dma_vaddr[(vspadev->spm_buffer_bytes >> 2) +
	                                     (vspadev->cmd_buffer_bytes >> 2)]);
	vspadev->mem_addr = (u32 __iomem *)res.start;
	vspadev->mem_size =  resource_size(&res);
	vspadev->regs = ioremap((unsigned int)vspadev->mem_addr,
			vspadev->mem_size);
	vspadev->dbg_regs = ioremap((unsigned int)vspadev->dbg_addr,
			vspadev->dbg_size);

	hw = &vspadev->hardware;
	param0 = vspa_reg_read(vspadev->regs + PARAM0_REG_OFFSET);
	hw->param0           = param0;
	param1 = vspa_reg_read(vspadev->regs + PARAM1_REG_OFFSET);
	hw->param1           = param1;
	hw->axi_data_width   = 32 << ((param1 >> 28) & 7);
	hw->dma_channels     = (param1 >> 16) & 0xFF;
	hw->gp_out_regs      = (param1 >> 8) & 0xFF;
	hw->gp_in_regs       = param1 & 0xFF;
	param2 = vspa_reg_read(vspadev->regs + PARAM2_REG_OFFSET);
	hw->param2           = param2;
	hw->dmem_bytes       = ((param2 >> 8) & 0x3FF) * 400;
	hw->ippu_bytes       = (param2 >> 31) * 4096;
	hw->arithmetic_units = param2 & 0xFF;

	vspadev->versions.vspa_hw_version =
			vspa_reg_read(vspadev->regs+HWVERSION_REG_OFFSET);
	vspadev->versions.ippu_hw_version =
			vspa_reg_read(vspadev->regs+IPPU_HWVERSION_REG_OFFSET);
	vspadev->versions.vspa_sw_version = ~0;
	vspadev->versions.ippu_sw_version = ~0;
	vspadev->eld_filename[0] = '\0';
	vspadev->watchdog_interval_msecs = VSPA_WATCHDOG_INTERVAL_DEFAULT;
	init_completion(&vspadev->watchdog_complete);
	setup_timer(&vspadev->watchdog_timer, watchdog_callback,
						(unsigned long)vspadev);

	spin_lock_init(&vspadev->dma_tx_queue_lock);
	spin_lock_init(&vspadev->dma_enqueue_lock);
	spin_lock_init(&vspadev->event_list_lock);
	spin_lock_init(&vspadev->event_queue_lock);
	spin_lock_init(&vspadev->mb32_lock);
	spin_lock_init(&vspadev->mb64_lock);
	spin_lock_init(&vspadev->control_lock);
	vspadev->poll_mask = VSPA_MSG_ALL;
//	spin_lock_init(&vspadev->irq_lock);
	init_waitqueue_head(&(vspadev->event_wait_q));
//TODO power cycle ?
	event_reset_queue(vspadev);
	dma_reset_queue(vspadev);
	mbox_reset(vspadev);
	cmd_reset(vspadev);

	err = request_irq(vspadev->flags1_irq_no, vspa_flags1_irq_handler,
				0, device_name, vspadev);
	if (err < 0) {
		dev_err(dev, "%s: flags1 request_irq() err = %d\n",
			device_name, err);
		goto err_ioremap;
	}
	err = request_irq(vspadev->flags0_irq_no, vspa_flags0_irq_handler,
				0, device_name, vspadev);
	if (err < 0) {
		dev_err(dev, "%s: flags0 request_irq() err = %d\n",
			device_name, err);
		goto err_irq0;
	}
	err = request_irq(vspadev->dma_irq_no, vspa_dma_irq_handler,
				0, device_name, vspadev);
	if (err < 0) {
		dev_err(dev, "%s: dma request_irq() err = %d\n",
			device_name, err);
		goto err_irq1;
	}
	err = request_irq(vspadev->general_irq_no, vspa_gen_irq_handler,
				0, device_name, vspadev);
	if (err < 0) {
		dev_err(dev, "%s: general request_irq() err = %d\n",
			device_name, err);
		goto err_irq2;
	}

	cdev_init(&vspadev->cdev, &vspa_fops);
	vspadev->cdev.owner = THIS_MODULE;

	err = cdev_add(&vspadev->cdev, devno, 1);
	if (err < 0) {
		dev_err(dev, "Error %d while adding %s", err, device_name);
		goto err_add;
	}

	/* Create sysfs device */
	sysfs_dev = device_create(vspa_class, dev, devno, NULL, device_name);
	if (IS_ERR(sysfs_dev)) {
		err = PTR_ERR(sysfs_dev);
		dev_err(dev, "Error %d while creating %s", err, device_name);
		goto err_create;
	}

	err = sysfs_create_group(&dev->kobj, &attr_group);
	if (err < 0) {
		dev_err(dev, "error %d while creating group %s", err,
								device_name);
		goto err_group;
	}

	/* Make sure all interrupts are disabled */
	vspa_reg_write(vspadev->regs + IRQEN_REG_OFFSET, 0);
	dev_info(dev, "%s: hwver 0x%08x, %d AUs, dmem %d bytes\n",
			device_name, vspadev->regs[HWVERSION_REG_OFFSET],
			hw->arithmetic_units, hw->dmem_bytes);
	vspa_devs++;
	return 0;

err_group:
	device_destroy(vspa_class, devno);
err_create:
	cdev_del(&vspadev->cdev);
err_add:
	free_irq(vspadev->general_irq_no,vspadev);
err_irq2:
	free_irq(vspadev->dma_irq_no,vspadev);
err_irq1:
	free_irq(vspadev->flags0_irq_no,vspadev);
err_irq0:
	free_irq(vspadev->flags1_irq_no,vspadev);
err_ioremap:
	iounmap(vspadev->regs);
	iounmap(vspadev->dbg_regs);
err_of:
	dma_free_coherent(dev, dma_size, dma_vaddr, dma_paddr);
err_struct:
	kfree(vspadev);
err_out:
	return err;
}

static int vspa_remove(struct platform_device *ofpdev)
{
	struct device *dev = &ofpdev->dev;
	struct vspa_device *vspadev = dev_get_drvdata(&ofpdev->dev);

	BUG_ON(vspadev == NULL || vspa_class == NULL);

	/* shutdown timer cleanly */
	vspadev->watchdog_interval_msecs = 0;
	init_completion(&vspadev->watchdog_complete);
	mod_timer(&vspadev->watchdog_timer, jiffies);
	wait_for_completion(&vspadev->watchdog_complete);
	del_timer_sync(&vspadev->watchdog_timer);

	/* Disbale all irqs of VSPA */
	vspa_reg_write(vspadev->regs + IRQEN_REG_OFFSET, 0);
// TODO - powerdown
	free_irq(vspadev->flags1_irq_no,vspadev);
	free_irq(vspadev->flags0_irq_no,vspadev);
	free_irq(vspadev->dma_irq_no,vspadev);
	free_irq(vspadev->general_irq_no,vspadev);

	if (vspadev->spm_user_buf) {
		vspadev->spm_user_buf = 0;
		iounmap(vspadev->spm_buf_vaddr);
	}

	dev_set_drvdata(&ofpdev->dev, NULL);

	iounmap(vspadev->regs);
	iounmap(vspadev->dbg_regs);

	sysfs_remove_group(&dev->kobj, &attr_group);
	device_destroy(vspa_class, MKDEV(vspa_major, vspa_minor + vspadev->id));
	cdev_del(&vspadev->cdev);
	kfree(vspadev);

	if (vspa_devs > 0)
		vspa_devs--;

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
	int err;
	dev_t devno;

	/* Register our major, and accept a dynamic number. */
	err = alloc_chrdev_region(&devno, 0, MAX_VSPA, VSPA_DEVICE_NAME);
	if (err < 0) {
		pr_err("vspa: can't get major number: %d\n", err);
		goto chrdev_fail;
	}
	vspa_major = MAJOR(devno);
	vspa_minor = MINOR(devno);

	/* Create the device class if required */
	vspa_class = class_create(THIS_MODULE, VSPA_DEVICE_NAME);
	if (IS_ERR(vspa_class)) {
		err = PTR_ERR(vspa_class);
		pr_err("vspa: class_create() failed %d\n", err);
		goto class_fail;
	}

	err = platform_driver_register(&vspa_driver);
	if (err == 0)
		return 0;

	class_destroy(vspa_class);
class_fail:
	unregister_chrdev_region(vspa_major, MAX_VSPA);
chrdev_fail:
	return err;
}

static void __exit vspa_exit(void)
{
	platform_driver_unregister(&vspa_driver);
	class_destroy(vspa_class);
	unregister_chrdev_region(vspa_major, MAX_VSPA);
}

module_init(vspa_mod_init);
module_exit(vspa_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale VSPA Driver");

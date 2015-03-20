/*
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

#ifndef __VSPA_H_
#define __VSPA_H_

#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <uapi/linux/vspa.h>

// Debug bit assignments
#define DEBUG_MESSAGES			(1<<0)
#define DEBUG_STARTUP			(1<<1)
#define DEBUG_CMD			(1<<2)
#define DEBUG_REPLY			(1<<3)
#define DEBUG_SPM			(1<<4)
#define DEBUG_DMA			(1<<5)
#define DEBUG_EVENT			(1<<6)
#define DEBUG_WATCHDOG			(1<<7)
#define DEBUG_MBOX64_OUT		(1<<8)
#define DEBUG_MBOX32_OUT		(1<<9)
#define DEBUG_MBOX64_IN			(1<<10)
#define DEBUG_MBOX32_IN			(1<<11)
#define DEBUG_DMA_IRQ			(1<<12)
#define DEBUG_FLAGS0_IRQ		(1<<13)
#define DEBUG_FLAGS1_IRQ		(1<<14)
#define DEBUG_IOCTL			(1<<15)
#define DEBUG_SEQID			(1<<16)
#define DEBUG_CMD_BD			(1<<17)
#define DEBUG_REPLY_BD			(1<<18)
#define DEBUG_TEST_SPM			(1<<24)

/* IP register offset for the registers used by the driver */
#define HWVERSION_REG_OFFSET		(0x0000>>2)
#define SWVERSION_REG_OFFSET		(0x0004>>2)
#define CONTROL_REG_OFFSET		(0x0008>>2)
#define IRQEN_REG_OFFSET		(0x000C>>2)
#define STATUS_REG_OFFSET		(0x0010>>2)
#define HOST_FLAGS0_REG_OFFSET		(0x0014>>2)
#define HOST_FLAGS1_REG_OFFSET		(0x0018>>2)
#define PARAM0_REG_OFFSET		(0x001C>>2)
#define PARAM1_REG_OFFSET		(0x0020>>2)
#define PARAM2_REG_OFFSET		(0x0024>>2)
#define EXT_GO_ENABLE_REG_OFFSET	(0x0028>>2)
#define EXT_GO_STATUS_REG_OFFSET	(0x002C>>2)
#define DMA_DMEM_ADDR_REG_OFFSET	(0x00B0>>2)
#define DMA_AXI_ADDR_REG_OFFSET		(0x00B4>>2)
#define DMA_BYTE_CNT_REG_OFFSET		(0x00B8>>2)
#define DMA_XFR_CTRL_REG_OFFSET		(0x00BC>>2)
#define DMA_STAT_ABORT_REG_OFFSET	(0x00C0>>2)
#define DMA_IRQ_STAT_REG_OFFSET		(0x00C4>>2)
#define DMA_COMP_STAT_REG_OFFSET	(0x00C8>>2)
#define DMA_XFRERR_STAT_REG_OFFSET	(0x00CC>>2)
#define DMA_CFGERR_STAT_REG_OFFSET	(0x00D0>>2)
#define DMA_XRUN_STAT_REG_OFFSET	(0x00D4>>2)
#define DMA_GO_STAT_REG_OFFSET		(0x00D8>>2)
#define DMA_FIFO_STAT_REG_OFFSET	(0x00DC>>2)

#define GP_OUT1_REG_OFFSET		(0x0580>>2)
#define GP_OUT2_REG_OFFSET		(0x0584>>2)
#define GP_OUT3_REG_OFFSET		(0x0588>>2)

#define HOST_OUT_32_REG_OFFSET		(0x0660>>2)
#define HOST_OUT_64_MSB_REG_OFFSET	(0x0664>>2)
#define HOST_OUT_64_LSB_REG_OFFSET	(0x0668>>2)
#define HOST_IN_32_REG_OFFSET		(0x066C>>2)
#define HOST_IN_64_MSB_REG_OFFSET	(0x0670>>2)
#define HOST_IN_64_LSB_REG_OFFSET	(0x0674>>2)
#define HOST_MBOX_STATUS_REG_OFFSET	(0x0678>>2)

#define IPPU_CONTROL_REG_OFFSET		(0x0700>>2)
#define IPPU_STATUS_REG_OFFSET		(0x0704>>2)
#define IPPU_ARG_BASEADDR_REG_OFFSET	(0x070C>>2)
#define IPPU_HWVERSION_REG_OFFSET	(0x0710>>2)
#define IPPU_SWVERSION_REG_OFFSET	(0x0714>>2)

#define DBG_GDBEN_REG_OFFSET		(0x0800>>2)
#define DBG_RCR_REG_OFFSET		(0x0804>>2)
#define DBG_RCSTATUS_REG_OFFSET		(0x0808>>2)


#define STATUS_REG_PDN_ACTIVE		(0x80000000)
#define STATUS_REG_PDN_DONE		(0x40000000)
#define STATUS_REG_BUSY			(0x00000100)
#define STATUS_REG_IRQ_VCPU_MSG		(0x00000040)
#define STATUS_REG_IRQ_DMA_ERR		(0x00000020)
#define STATUS_REG_IRQ_DMA_COMP		(0x00000010)
#define STATUS_REG_IRQ_FLAGS1		(0x00000008)
#define STATUS_REG_IRQ_FLAGS0		(0x00000004)
#define STATUS_REG_IRQ_IPPU_DONE	(0x00000002)
#define STATUS_REG_IRQ_DONE		(0x00000001)

#define MBOX_STATUS_IN_64_BIT		(0x00000008)
#define MBOX_STATUS_IN_32_BIT		(0x00000004)
#define MBOX_STATUS_OUT_64_BIT		(0x00000002)
#define MBOX_STATUS_OUT_32_BIT		(0x00000001)

#define VSPA_DMA_CHANNELS		(32)

/* mmap offset argument for vspa regsiters */
#define VSPA_REG_OFFSET			(0)

/* mmap offset argument for dbg regsiter */
#define VSPA_DBG_OFFSET			(4096)

#define DMA_FLAG_COMPLETE		(1<<0)
#define DMA_FLAG_XFRERR			(1<<1)
#define DMA_FLAG_CFGERR			(1<<2)


#define MBOX_QUEUE_ENTRIES		(16)
struct mbox_queue {
	volatile int	idx_enqueue;
	volatile int	idx_dequeue;
	volatile int	idx_complete;
};

#define EVENT_LIST_ENTRIES		(256)
struct event_list {
	struct list_head list;
	union {
	  uint32_t	control;
	  struct {
	    uint8_t	id;
	    uint8_t	err;
	    uint16_t	type;
	  };
	};
	uint32_t	data0;
	uint32_t	data1;
};

struct event_entry {
	union {
	  uint32_t      control;
	  struct {
	    uint8_t     id;
	    uint8_t     err;
	    uint8_t     rsvd;
	    uint8_t     type;
	  };
	};
	uint32_t	data0;
	uint32_t	data1;
	uint32_t	lost;
};

#define EVENT_QUEUE_ENTRIES		(256)
struct event_queue {
	struct event_entry entry[EVENT_QUEUE_ENTRIES];
	volatile int    idx_enqueue;	/* Index for next item to enqueue */
	volatile int    idx_queued;	/* Index for last item enqueued */
	volatile int    idx_dequeue;	/* Index for next item to dequeue */
};

#define DMA_QUEUE_ENTRIES		(16)
struct dma_queue {
	struct vspa_dma_req entry[DMA_QUEUE_ENTRIES];
	int		chan;
	int		idx_queue;	/* Index for next item to enqueue */
	int		idx_dma;	/* Index for next item to DMA */
	int		idx_chk;	/* Index for next item to check */
};

struct memory_pool_bd {
	uint32_t	start;
	uint32_t	size;
	uint32_t	free;
	uint32_t	wstart;
	int8_t		next;
};

//TODO - support 32 reply bds
#define BD_ENTRIES		(16)
struct memory_pool {
	uint32_t	free_bds;
	struct memory_pool_bd bd[BD_ENTRIES];
	uint32_t	tail;
	uint32_t	size;
	uint32_t	*paddr;
	uint32_t	*vaddr;
};

struct circular_buffer {
	uint32_t	write_idx;
	uint32_t	read_idx;
	uint32_t	size;
	uint32_t	used;
	uint32_t	*paddr;
	uint32_t	*vaddr;
};

#define MAX_SEQIDS			(16)
struct seqid {
	uint32_t	flags;
	int		cmd_id;
	int		cmd_buffer_idx;
	int		cmd_buffer_size;
	int		cmd_bd_index;
	int		reply_bd_index;
	int		reply_size;
	uint32_t	payload1;
};

/* The below structure contains all the information for the
* vspa device required by the kernel driver
*/
struct vspa_device {

	/* VSPA instance */
	int		id;

	struct device	*dev;

	/* Char device structure */
	struct cdev	cdev;

	/* Major minor information */
	dev_t		dev_t;

	/* Current state of the device */
	enum vspa_state	state;
	uint32_t	debug;

	/* IRQ numbers */
	uint32_t	flags1_irq_no;
	uint32_t	flags0_irq_no;
	uint32_t	dma_irq_no;
	uint32_t	general_irq_no;

	/* IP registers */
	resource_size_t	mem_size; /* size */
	u32 __iomem	*mem_addr;    /* physical address */
	u32 __iomem	*regs;	/* virtual address */

	/* Debug registers */
	resource_size_t	dbg_size; /* size */
	u32 __iomem	*dbg_addr;    /* physical address */
	u32 __iomem	*dbg_regs;    /* virtual address */

	/* Buffer sizes */
	uint32_t	spm_buffer_bytes;
	uint32_t	cmd_buffer_bytes;
	uint32_t	reply_buffer_bytes;
	uint32_t	*spm_buffer_paddr;
	uint32_t	*spm_buffer_vaddr;

	/* Working set for SPM buffer */
	int		spm_user_buf;
	uint32_t	spm_buf_bytes;
	uint32_t	*spm_buf_paddr;
	uint32_t	*spm_buf_vaddr;

	/* DMA queue */
	struct dma_queue dma_queue;
	spinlock_t	dma_enqueue_lock;
	spinlock_t	dma_tx_queue_lock; /* called from irq handler */
	unsigned long	dma_tx_queue_lock_flags;

	/* Event queue */
	struct event_queue event_queue;
	struct event_list events[EVENT_LIST_ENTRIES];
	struct list_head events_free_list;
	struct list_head events_queued_list;
	spinlock_t	event_list_lock;
	spinlock_t	event_queue_lock; /* called from irq handler */

	/* Memory pools */
	struct circular_buffer cmd_buffer;
	struct memory_pool cmd_pool;
	struct memory_pool reply_pool;

	uint32_t	cmdbuf_addr;
	uint32_t	spm_addr;

	/* Sequence IDs */
	uint32_t	active_seqids;
	uint32_t	last_seqid;
	struct seqid	seqid[MAX_SEQIDS];

	uint32_t	first_cmd;
	uint32_t	last_wstart;

	struct mbox_queue mb32_queue;
	struct mbox_queue mb64_queue;
	struct vspa_mb32 mb32[MBOX_QUEUE_ENTRIES];
	struct vspa_mb64 mb64[MBOX_QUEUE_ENTRIES];
	spinlock_t	mb32_lock; /* called from irq handler */
	spinlock_t	mb64_lock; /* called from irq handler */

	/* DMA channel usage */
	uint8_t		spm_dma_chan;
	uint8_t		bulk_dma_chan;
	uint8_t		reply_dma_chan;
	uint8_t		cmd_dma_chan;

	uint8_t		legacy_cmd_dma_chan;
	char		eld_filename[VSPA_MAX_ELD_FILENAME];
	struct vspa_versions versions;
	struct vspa_hardware hardware;

	/* Watchdog */
	struct timer_list watchdog_timer;
	uint32_t	watchdog_interval_msecs;
	uint32_t	watchdog_value;
	struct completion watchdog_complete;

	/* IRQ handling */
//TODO	spinlock_t irq_lock;
	uint32_t	irq_bits; // TODO - test code

	spinlock_t	control_lock;

	/* Wait queue for event notifications*/
	wait_queue_head_t event_wait_q;
	uint32_t	event_list_mask;
	uint32_t	event_queue_mask;
	uint32_t	poll_mask;
};

#endif /* _VSPA_H */

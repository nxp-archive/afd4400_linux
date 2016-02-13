/*
 * include/linux/cpri_hdlc.h
 * CPRI device driver - Ethernet MAC
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __CPRI_HDLC_H_
#define __CPRI_HDLC_H_

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>

#define BD_FLAG_SHIFT	24
#define BD_LEN_SHIFT	8
#define BD_LENGTH_MASK		0x00ffff00
/* needed for write ops */
#define BD_LFLAG_FSHIFT(flags) ((flags) << BD_FLAG_SHIFT)
#define BD_LFLAG_LSHIFT(len) ((len << BD_LEN_SHIFT) & BD_LENGTH_MASK)
/* needed for read ops */
#define BD_LSTATUS_LSHIFT(len) ((len >> BD_LEN_SHIFT) &\
		(BD_LENGTH_MASK >> BD_LEN_SHIFT))
#define BD_LSTATUS_SSHIFT(status) ((status) >> BD_FLAG_SHIFT)

#define CPRI_HDLC_NEXT_INDX(cur, rsize) \
		((((cur)+1) >= (rsize)) ? 0 : ((cur)+1))

#define cpri_hdlc_skip_bd(bdp, stride, base, ring_size) ({ \
	typeof(bdp) new_bd = (bdp) + (stride); \
	(new_bd >= (base) + (ring_size)) ? (new_bd - (ring_size)) : new_bd; })

#define cpri_hdlc_next_bde(bdp, base, ring_size) \
			cpri_hdlc_skip_bd(bdp, 1, base, ring_size)


struct cpri_hdlc_bd_entity {
	u32 lstatus;
	u32 buf_ptr;
};

/* HDLC RX buffer descriptor */
struct cpri_hdlc_rx_bd {
	/* HDLC RX DMA BD entry */
	struct cpri_hdlc_bd_entity *rx_bd_base;
	/* Base physical address for RX BD entry */
	dma_addr_t rx_bd_dma_base;
	/* HDLC RX buffer virtual address */
	char **rxbuf;
	/* HDLC RX buffer physical address */
	dma_addr_t *rxbuf_paddr;
	/* HDLC RX valid buffer */
	char **rxbuf_valid;
	/* How many RX BD */
	u8 rx_bd_ring_cnt;
	/* The rx bd index to for dma to write to */
	u8 rx_bd_index;
	/* How many RX buf */
	u8 rx_buf_cnt;
	/* Received rx buffers count */
	u8 rx_avail_cnt;
	/* The rx buffer for to read from */
	u8 rx_read_ptr;
	/* The rx buffer for to write to */
	u8 rx_write_ptr;
};

/* HDLC TX buffer descriptor */
struct cpri_hdlc_tx_bd {
	/* HDLC TX DMA BD entry */
	struct cpri_hdlc_bd_entity *tx_bd_base;
	/* Base physical address for TX BD entry */
	dma_addr_t tx_bd_dma_base;
	/* Used to store the data pointer */
	char **txbuf;
	/* How many TX BDs */
	u8 tx_bd_ring_cnt;
	/* The next index to write to */
	u8 current_index;
	/* When the HDLC TX HW consumes TX BD, update the dirty_index */
	u8 dirty_index;
	/* Number of free TX BDs */
	u8 num_txbdfree;
};

struct cpri_hdlc_priv {
	struct cpri_framer *framer;
	void *bd_vaddr;
	dma_addr_t bd_paddr;
	struct cpri_hdlc_tx_bd tx_bd;
	struct cpri_hdlc_rx_bd rx_bd;
	u32 rx_buffer_size;
	struct cpri_hdlc_stats stats;
	atomic_t users;
	spinlock_t tx_lock;
	spinlock_t rx_lock;
	wait_queue_head_t rx_queue;
	wait_queue_head_t tx_queue;
	wait_queue_head_t tx_release_queue;
	struct cpri_hdlc_config hdlc_config;
	long timeout;
};

#define CPRI_HDLC_BD_TO_LE(bd_le, bd) \
do {\
	(bd_le)->lstatus = be32_to_cpu((bd)->lstatus); \
	(bd_le)->buf_ptr = be32_to_cpu((bd)->buf_ptr); \
} while (0)

#define CPRI_HDLC_BD_TO_BE(bd_be, bd) \
do {\
	(bd_be)->buf_ptr = cpu_to_be32((bd)->buf_ptr); \
	(bd_be)->lstatus = cpu_to_be32((bd)->lstatus); \
} while (0)


#define CPRI_HDLC_BD_TX_READY		0x80
#define CPRI_HDLC_BD_RX_EMPTY		0x80
#define CPRI_HDLC_BD_RX_ABORT		0x02
#define CPRI_HDLC_BD_RX_CRC		0x04
#define CPRI_HDLC_BD_RX_PLE		0x08
#define CPRI_HDLC_BD_RX_BOF		0x10
#define CPRI_HDLC_MII_ERR		0x400

/* Defaults */
#define CPRI_HDLC_DEF_TX_RING_CNT	16
#define CPRI_HDLC_DEF_RX_RING_CNT	16
#define CPRI_HDLC_DEF_RX_BUF_CNT	128
#define CPRI_HDLC_DEF_RX_BUF_SIZE	1024
#define CPRI_HDLC_DEF_RX_BUF_RESERVED	4

#define CPRI_HDLC_RX_BUF_SZ_MASK	0x0000ffff
/* CPRInREBDRS */
#define CPRI_HDLC_RX_BD_RING_SZ_MASK		0x000000ff
/* CPRInTEBDRS */
#define CPRI_HDLC_TX_BD_RING_SZ_MASK		0x000000ff
/* CPRInREBDRBA */
#define CPRI_HDLC_RX_BD_RING_BASE_MASK		0xffffffff
/* CPRInREBDRBAMSB */
#define CPRI_HDLC_RX_BD_RING_BASE_MSB_MASK	0x0000000f
/* CPRInTEBDRBA */
#define CPRI_HDLC_TX_BD_RING_BASE_MASK		0xffffffff
/* CPRInTEBDRBAMSB */
#define CPRI_HDLC_TX_BD_RING_BASE_MSB_MASK	0x0000000f
/* CPRInRERPR */
#define CPRI_HDLC_RX_BD_R_PTR_WRAP_MASK		0x00000100
#define CPRI_HDLC_RX_BD_R_PTR_MASK		0x000000ff
/* CPRInTERPR */
#define CPRI_HDLC_TX_BD_R_PTR_WRAP_MASK		0x00000100
#define CPRI_HDLC_TX_BD_R_PTR_MASK		0x000000ff
/* CPRInREWPR */
#define CPRI_HDLC_RX_BD_W_PTR_WRAP_MASK		0x00000100
#define CPRI_HDLC_RX_BD_W_PTR_MASK		0x000001ff
/* CPRInTEWPR */
#define CPRI_HDLC_TX_BD_W_PTR_WRAP_MASK		0x00000100
#define CPRI_HDLC_TX_BD_W_PTR_MASK		0x000001ff

/* Control Registers */
/* CPRIn_HDLC_RX_CONTROL */
#define CPRI_HDLC_RX_CTRL_DISCARD_MASK		0x00000001
/* CPRInRCR */
#define CPRI_HDLC_RX_ENABLE_MASK		0x00000004
/* CPRInRCR */
#define CPRI_HDLC_TX_ENABLE_MASK		0x00000004

#endif

/*
 * include/linux/cpri_eth.h
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

#ifndef __CPRI_ETH_H_
#define __CPRI_ETH_H_

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
/* For TX BD, 16b word size accesses are made from cpri eth for len and flags*/
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

#define CPRI_ETH_NEXT_INDX(cur, rsize) \
		((((cur)+1) >= (rsize)) ? 0 : ((cur)+1))

#define cpri_skip_bd(bdp, stride, base, ring_size) ({ \
	typeof(bdp) new_bd = (bdp) + (stride); \
	(new_bd >= (base) + (ring_size)) ? (new_bd - (ring_size)) : new_bd; })

#define cpri_eth_next_bde(bdp, base, ring_size) \
			cpri_skip_bd(bdp, 1, base, ring_size)
#define CPRI_ETH_PTR_MIN	0x14

struct cpri_eth_extra_stats {
	unsigned long kernel_dropped;

	/* ERROR events */
	unsigned long rx_dma_overrun;
	unsigned long rx_overrun;
	unsigned long rx_underrun;
	unsigned long tx_underrun;

	/* ETH RX EX STATUS */
	unsigned long mii_error;
	unsigned long small_pkt;
	unsigned long rx_overflow;
	unsigned long dmac_mismatch;
	unsigned long crc_error;
	unsigned long cpri_rx_error;
	unsigned long long_frame;
	unsigned long short_frame;
};

struct cpri_eth_bd_entity {
		u32  lstatus;
		u32 buf_ptr;
};

struct cpri_eth_rx_bd {
	struct net_device *ndev;
	spinlock_t rxlock;
	struct cpri_eth_bd_entity *rx_bd_base;
	dma_addr_t rx_bd_dma_base;
	struct	sk_buff **rx_skbuff;
	struct cpri_eth_bd_entity *rx_bd_current;
	unsigned int skb_currx;
	unsigned char rx_bd_ring_size;
};

struct cpri_eth_tx_bd {
	struct net_device *ndev;
	spinlock_t txlock;
	struct cpri_eth_bd_entity *tx_bd_base;
	dma_addr_t tx_bd_dma_base;
	struct sk_buff **tx_skbuff;
	struct cpri_eth_bd_entity *tx_bd_current;
	struct cpri_eth_bd_entity *tx_bd_dirty;
	unsigned int skb_curtx;
	unsigned int skb_dirtytx;
	unsigned int  num_txbdfree;
	unsigned char tx_bd_ring_size;
};


struct cpri_eth_priv {
	struct platform_device *ofdev;
	struct net_device *ndev;
	struct napi_struct napi;

	struct cpri_framer *framer;

	struct cpri_eth_tx_bd *tx_bd;
	struct cpri_eth_rx_bd *rx_bd;

	struct work_struct reset_task;
	struct work_struct error_task;
	struct tasklet_struct tasklet;

	unsigned int flags;
	unsigned int rx_buffer_size;
	unsigned int tx_start_thresh;
	unsigned int tx_coales_thresh;
	unsigned int rx_coales_thresh;
	unsigned int fwdif_status;

	/* Unaligned DMA ring addresses */
	dma_addr_t addr;
	void *vaddr;

	struct net_device_stats stats;
	struct cpri_eth_extra_stats extra_stats;
};

#define CPRI_ETH_BD_TO_LE(bd_le, bd) \
do {\
	(bd_le)->lstatus = be32_to_cpu((bd)->lstatus); \
	(bd_le)->buf_ptr = be32_to_cpu((bd)->buf_ptr); \
} while (0)

#define CPRI_ETH_BD_TO_BE(bd_be, bd) \
do {\
	(bd_be)->buf_ptr = cpu_to_be32((bd)->buf_ptr); \
	(bd_be)->lstatus = cpu_to_be32((bd)->lstatus); \
} while (0)


#define CPRI_ETH_NAPI_WEIGHT		64
#define CPRI_ETH_RXBUF_ALIGNMENT	16
#define CPRI_ETH_TX_TIMEOUT		(1000*HZ)
#define CPRI_ETH_DISABLED		0
#define CPRI_ETH_ENABLED		1
#define CPRI_ETH_BD_RING_ALIGN		16
#define CPRI_ETH_BD_TX_READY		0x80
#define CPRI_ETH_BD_RX_EMPTY		0x80
#define CPRI_ETH_BD_RX_ABORT		0x02
#define CPRI_ETH_BD_RX_CRC		0x04
#define CPRI_ETH_BD_RX_PLE		0x08
#define CPRI_ETH_BD_RX_BOF		0x10


/* Defaults */
#define CPRI_ETH_DEF_TX_RING_SIZE	254
#define CPRI_ETH_DEF_RX_RING_SIZE	254
#define CPRI_ETH_DEF_TX_START_THRESH	4
#define CPRI_ETH_DEF_RX_BUF_SIZE	4096
#define CPRI_ETH_DEF_MTU		1500
#define CPRI_ETH_DEF_RX_COAL_THRESH	0
#define CPRI_ETH_DEF_TX_COAL_THRESH	0

/* Flags */
#define CPRI_ETH_HW_CRC_STRIP		0x00000001
#define CPRI_ETH_MAC_FAIL_PASS		0x00000002
#define CPRI_ETH_TRIG_TX_ABORT		0x00000004
#define CPRI_ETH_TRIG_TX_READY		0x00000008
#define CPRI_ETH_TRIG_RX_ABORT		0x00000010
#define CPRI_ETH_TRIG_RX_READY		0x00000020
#define CPRI_ETH_TRIG_TX		0x00000040
#define CPRI_ETH_TRIG_RX		0x00000080
#define CPRI_ETH_GLOBAL_TRIG		0x00000100

#define CPRI_ETH_TRIG (CPRI_ETH_TRIG_TX_ABORT | \
		CPRI_ETH_TRIG_RX_ABORT | CPRI_ETH_TRIG_TX_READY | \
		CPRI_ETH_TRIG_RX_READY | CPRI_ETH_TRIG_TX | \
		CPRI_ETH_TRIG_RX | CPRI_ETH_GLOBAL_TRIG)

#define CPRI_ETH_LONG_FRAME		0x00000200
#define CPRI_ETH_RX_PREAMBLE_ABORT	0x00000400
#define CPRI_ETH_BCAST			0x00000800
#define CPRI_ETH_MCAST_FLT		0x00001000
#define CPRI_ETH_MAC_CHECK		0x00002000
#define CPRI_ETH_LEN_CHECK		0x00004000
#define CPRI_ETH_LITTLE_END		0x00008000
#define CPRI_ETH_HW_CRC_EN		0x00010000
#define CPRI_ETH_HW_CRC_CHECK		0x00020000
#define CPRI_ETH_STORE_FWD		0x00040000

#define CPRI_ETH_DEF_FLAGS (CPRI_ETH_TRIG | CPRI_ETH_BCAST | \
		CPRI_ETH_MCAST_FLT | CPRI_ETH_LEN_CHECK | \
		CPRI_ETH_HW_CRC_EN | CPRI_ETH_HW_CRC_CHECK | \
		CPRI_ETH_MAC_CHECK | CPRI_ETH_HW_CRC_STRIP)

/* Config Registers */
/* CPRin_ETH_CONFIG_1 */
#define CPRI_ETH_HW_CRC_STRIP_MASK		0x00200000
#define CPRI_ETH_MAC_FAIL_PASS_MASK		0x00100000
#define CPRI_ETH_TRIG_TX_ABORT_MASK		0x00040000
#define CPRI_ETH_TRIG_TX_RDY_MASK		0x00020000
#define CPRI_ETH_TRIG_RX_ABORT_MASK		0x00004000
#define CPRI_ETH_TRIG_RX_RDY_MASK		0x00002000
#define CPRI_ETH_TRIG_TX_MASK			0x00001000
#define CPRI_ETH_TRIG_RX_MASK			0x00000800
#define CPRI_ETH_GLOBAL_TRIG_MASK		0x00000400
#define CPRI_ETH_LONG_FRAME_MASK		0x00000200
#define CPRI_ETH_RX_PR_ABORT_MASK		0x00000100
#define CPRI_ETH_BCAST_MASK			0x00000080
#define CPRI_ETH_MCAST_FLT_MASK			0x00000040
#define CPRI_ETH_MAC_CHECK_MASK			0x00000020
#define CPRI_ETH_LEN_CHECK_MASK			0x00000010
#define CPRI_ETH_LITTLE_END_MASK		0x00000002
/* CPRIn_ETH_CONFIG_2 */
#define CPRI_ETH_HW_CRC_EN_MASK			0x00000001
/* CPRIn_ETH_CONFIG_3 */
#define CPRI_ETH_TX_THRESH_MASK			0x00007f00
#define CPRI_ETH_HW_CRC_CHECK_MASK		0x00000002
#define CPRI_ETH_STORE_FWD_MASK			0x00000001
/* CPRIn_ETH_ADDR_MSB */
#define CPRI_ETH_ADDR_MSB_MASK			0x0000ffff
/* CPRIn_ETH_ADDR_LSB */
#define CPRI_ETH_ADDR_LSB_MASK			0xffffffff
/* CPRIn_ETH_HASH_TABLE */
#define CPRI_ETH_MCAST_MASK			0xffffffff

/* Buffer registers */
/* CPRInRETHBS */
#define CPRI_ETH_RX_BUF_SZ_MASK			0x0000ffff
/* CPRInREBDRS */
#define CPRI_ETH_RX_BD_RING_SZ_MASK		0x000000ff
/* CPRInTEBDRS */
#define CPRI_ETH_TX_BD_RING_SZ_MASK		0x000000ff
/* CPRInREBDRBA */
#define CPRI_ETH_RX_BD_RING_BASE_MASK		0xffffffff
/* CPRInREBDRBAMSB */
#define CPRI_ETH_RX_BD_RING_BASE_MSB_MASK	0x0000000f
/* CPRInTEBDRBA */
#define CPRI_ETH_TX_BD_RING_BASE_MASK		0xffffffff
/* CPRInTEBDRBAMSB */
#define CPRI_ETH_TX_BD_RING_BASE_MSB_MASK	0x0000000f
/* CPRInRETHCT */
#define CPRI_ETH_RX_COAL_THRES_MASK		0x000000ff
/* CPRInTETHCT */
#define CPRI_ETH_TX_COAL_THRES_MASK		0x000000ff
/* CPRInRETHCS */
#define CPRI_ETH_RX_COAL_STATUS_MASK		0x000000ff
/* CPRInTETHCS */
#define CPRI_ETH_TX_COAL_STATUS_MASK		0x000000ff
/* CPRInRERPR */
#define CPRI_ETH_RX_BD_R_PTR_WRAP_MASK		0x00000100
#define CPRI_ETH_RX_BD_R_PTR_MASK		0x000001ff
/* CPRInTERPR */
#define CPRI_ETH_TX_BD_R_PTR_WRAP_MASK		0x00000100
#define CPRI_ETH_TX_BD_R_PTR_MASK		0x000000ff
/* CPRInREWPR */
#define CPRI_ETH_RX_BD_W_PTR_WRAP_MASK		0x00000100
#define CPRI_ETH_RX_BD_W_PTR_MASK		0x000001ff
/* CPRInTEWPR */
#define CPRI_ETH_TX_BD_W_PTR_WRAP_MASK		0x00000100
#define CPRI_ETH_TX_BD_W_PTR_MASK		0x000001ff
/* CPRInRETHBD */
#define CPRI_ETH_RX_BD_EMPTY_MASK		0x80000000
#define CPRI_ETH_RX_BD_BOF_MASK			0x10000000
#define CPRI_ETH_RX_BD_PLE_MASK			0x08000000
#define CPRI_ETH_RX_BD_CRC_MASK			0x04000000
#define CPRI_ETH_RX_BD_ABORT_MASK		0x02000000
#define CPRI_ETH_RX_BD_LEN_MASK			0x00ffff00
#define CPRI_ETH_RX_BD_PTR_MSB_MASK		0x0000000f
#define CPRI_ETH_RX_BD_PTR_MASK			0xffffffff
/* CPRInTETHBD */
#define CPRI_ETH_TX_BD_READY_MASK		0x80000000
#define CPRI_ETH_TX_BD_LEN_MASK			0x00ffff00
#define CPRI_ETH_TX_BD_PTR_MSB_MASK		0x0000000f
#define CPRI_ETH_TX_BD_PTR_MASK			0xffffffff

/* Control Registers */
/* CPRIn_ETH_RX_CONTROL */
#define CPRI_ETH_RX_CTRL_DISCARD_MASK		0x00000001
/* CPRInEFICR */
#define CPRI_ETH_FWD_ENABLE_MASK		0x00000001
/* CPRInRCR */
#define CPRI_ETH_RX_ENABLE_MASK			0x00000002
/* CPRInRCR */
#define CPRI_ETH_TX_ENABLE_MASK			0x00000002

/* Status Registers */
/* CPRIn_ETH_RX_STATUS */
#define CPRI_ETH_RX_ST_MAC_MASK			0x00006000
#define CPRI_ETH_RX_ST_CRC_ERR_MASK		0x00001000
#define CPRI_ETH_RX_ST_MII_ERR_MASK		0x00000400
#define CPRI_ETH_RX_ST_SPKT_MASK		0x00000200
#define CPRI_ETH_RX_ST_OFLW_MASK		0x00000100
#define CPRI_ETH_RX_ST_ABORT_MASK		0x00000004
/* CPRIn_ETH_RX_EX_STATUS */
#define CPRI_ETH_RX_EST_MII_PERR_MASK		0x00000400
#define CPRI_ETH_RX_EST_SPKT_MASK		0x00000200
#define CPRI_ETH_RX_EST_OFLW_MASK		0x00000100
#define CPRI_ETH_RX_EST_DMAC_MM_MASK		0x00000080
#define CPRI_ETH_RX_EST_CRC_ERR_MASK		0x00000040
#define CPRI_ETH_RX_EST_RX_ERR_MASK		0x00000010
#define CPRI_ETH_RX_EST_LFRM_MASK		0x00000008
#define CPRI_ETH_RX_EST_SFRM_MASK		0x00000004
/* CPRInRER */
#define CPRI_ETH_RX_EV_MASK			0x00000020
/* CPRInTER */
#define CPRI_ETH_TX_EV_MASK			0x00000020
/* CPRInEER */
#define CPRI_ETH_REM_FF_MASK			0x00002000
#define CPRI_ETH_RX_DMA_OVR_MASK		0x00001000
#define CPRI_ETH_RX_BD_UDR_MASK			0x00000010
#define CPRI_ETH_TX_UDR_MASK			0x00000008
#define CPRI_ETH_RX_OVR_MASK			0x00000004
/* CPRInRSR */
#define CPRI_ETH_RX_DMA_STATUS_MASK		0x00000002
/* CPRInTSR */
#define CPRI_ETH_TX_DMA_STATUS_MASK		0x00000002

/* Interrupt enable Registers */
/* CPRInRCIER */
#define CPRI_ETH_RX_EV_EN_MASK			0x00000004
/* CPRInTCIER */
#define CPRI_ETH_TX_EV_EN_MASK			0x00000004
/* CPRInEIER */
#define CPRI_ETH_REM_FF_EN_MASK			0x00002000
#define CPRI_ETH_RX_DMA_OVR_EN_MASK		0x00001000
#define CPRI_ETH_RX_BD_UDR_EN_MASK		0x00000010
#define CPRI_ETH_TX_UDR_EN_MASK			0x00000008
#define CPRI_ETH_RX_OVR_EN_MASK			0x00000004
/* CPRIICR */
#define CPRI_ETH_ICR_ERS_MASK			0xf0000000
#define CPRI_ETH_EVENT_EN_MASK			0x00000020

#endif

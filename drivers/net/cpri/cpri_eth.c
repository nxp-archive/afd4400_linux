/*
 * drivers/net/cpri/cpri_eth.h
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

#include <linux/cpri.h>
/*defines for using OCRAM for Medusa*/
/*16KB used for CPRI*/
#define OCRAM_CPRI_TOTAL_LEN (16 * 1024)

#define OCRAM_CPRI_BD_LEN 512

#define MEDUSA_TOTAL_TX_BUFF 4
#define MEDUSA_TOTAL_RX_BUFF 5

#define OCRAM_CPRI_TX_LEN (MEDUSA_TOTAL_TX_BUFF *\
		(CPRI_ETH_DEF_RX_BUF_SIZE + CPRI_ETH_RXBUF_ALIGNMENT))

#define OCRAM_CPRI_RX_LEN (MEDUSA_TOTAL_RX_BUFF *\
		(CPRI_ETH_DEF_RX_BUF_SIZE + CPRI_ETH_RXBUF_ALIGNMENT))

#define OCRAM_CPRI_BD_MEM 0x0d100000
#define OCRAM_MEM_OFFSET_FPGA 0x0c000000

#define OCRAM_CPRI_TX_MEM (OCRAM_CPRI_BD_MEM + OCRAM_CPRI_BD_LEN)
#define OCRAM_CPRI_RX_MEM (OCRAM_CPRI_TX_MEM + OCRAM_CPRI_TX_LEN)

/*OCRAM Tx/Rx buffer housekeeping*/
struct cpri_ocram {
	unsigned int status;
	unsigned int ocram_ptr;
};

struct rxtxmem {
	struct cpri_ocram tx_mem_ocram[MEDUSA_TOTAL_TX_BUFF];
	struct cpri_ocram rx_mem_ocram[MEDUSA_TOTAL_RX_BUFF];
};

struct rxtxmem cpri_rtxm;

enum {
	OCRAM_BUF_FREE = 0,
	OCRAM_BUF_ALLOCATED
};

unsigned int cpri_ocram_vaddr;

/*mem copy to ocram: ensuring 32BIT access always*/
void *cpri_memcpy_ocram(void *dst, void *src, unsigned int size)
{
	int *temp_dst = dst;
	int *temp_src = src;

	if (0 == (size%4))
		size = size/4;
	else
		size = (size + 4)/4;

	while (size--)
		*temp_dst++ = *temp_src++;
	return dst;
}

static void cpri_mem_sync(void)
{
	asm("dmb");
	asm("dsb");
}

int cpri_free_ocram_bd(int buff_ptr)
{
	int tx_cnt;

	for (tx_cnt = 0; tx_cnt < MEDUSA_TOTAL_TX_BUFF; tx_cnt++) {
		if ((buff_ptr + OCRAM_MEM_OFFSET_FPGA) ==
				cpri_rtxm.tx_mem_ocram[tx_cnt].ocram_ptr) {
			cpri_rtxm.tx_mem_ocram[tx_cnt].status = OCRAM_BUF_FREE;
			return 0;
		}
	}
	return -1;
}

static int cpri_eth_init_bd_regs(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;
	struct cpri_framer_regs *regs = framer->regs;
	struct cpri_eth_tx_bd *tx_bd = priv->tx_bd;
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;

	/* Rx buffer size */
	cpri_reg_set_val(&regs->cpri_rethbufsize,
		CPRI_ETH_RX_BUF_SZ_MASK, CPRI_ETH_DEF_RX_BUF_SIZE-1);

	/* Ring size */
	cpri_reg_set_val(&regs->cpri_tethbdringsize,
		CPRI_ETH_TX_BD_RING_SZ_MASK, CPRI_ETH_DEF_TX_RING_SIZE);

	cpri_reg_set_val(&regs->cpri_rethbdringsize,
		CPRI_ETH_RX_BD_RING_SZ_MASK, CPRI_ETH_DEF_RX_RING_SIZE);

	/* Ring Base address */
	if (priv->framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM)
		cpri_reg_set_val(&regs->cpri_tethbdringbaddr,
			CPRI_ETH_TX_BD_RING_BASE_MASK,
			(tx_bd->tx_bd_dma_base - OCRAM_MEM_OFFSET_FPGA));
	else
		cpri_reg_set_val(&regs->cpri_tethbdringbaddr,
			CPRI_ETH_TX_BD_RING_BASE_MASK, tx_bd->tx_bd_dma_base);

	cpri_reg_clear(&regs->cpri_tethbdringbaddrmsb,
		CPRI_ETH_TX_BD_RING_BASE_MSB_MASK);

	if (priv->framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM)
		cpri_reg_set_val(&regs->cpri_rethbdringbaddr,
			CPRI_ETH_RX_BD_RING_BASE_MASK,
			(rx_bd->rx_bd_dma_base - OCRAM_MEM_OFFSET_FPGA));
	else
		cpri_reg_set_val(&regs->cpri_rethbdringbaddr,
			CPRI_ETH_RX_BD_RING_BASE_MASK,
			rx_bd->rx_bd_dma_base);

	cpri_reg_clear(&regs->cpri_rethbdringbaddrmsb,
		CPRI_ETH_RX_BD_RING_BASE_MSB_MASK);
	/* Coalescing threshold */
	cpri_reg_set_val(&regs->cpri_tethcoalthresh,
		CPRI_ETH_TX_COAL_THRES_MASK, priv->tx_coales_thresh);

	cpri_reg_set_val(&regs->cpri_rethcoalthresh,
		CPRI_ETH_RX_COAL_THRES_MASK, priv->rx_coales_thresh);

	return 0;
}

static void cpri_eth_clear_bd_regs(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	/* Rx buffer size */
	cpri_reg_clear(&framer->regs->cpri_rethbufsize,
			CPRI_ETH_RX_BUF_SZ_MASK);

	/* Ring size */
	cpri_reg_clear(&framer->regs->cpri_tethbdringsize,
			CPRI_ETH_TX_BD_RING_SZ_MASK);

	cpri_reg_clear(&framer->regs->cpri_rethbdringsize,
			CPRI_ETH_RX_BD_RING_SZ_MASK);

	/* Ring Base address */
	cpri_reg_clear(&framer->regs->cpri_tethbdringbaddr,
			CPRI_ETH_TX_BD_RING_BASE_MASK);

	cpri_reg_clear(&framer->regs->cpri_tethbdringbaddrmsb,
			CPRI_ETH_TX_BD_RING_BASE_MSB_MASK);

	cpri_reg_clear(&framer->regs->cpri_rethbdringbaddr,
			CPRI_ETH_RX_BD_RING_BASE_MASK);

	cpri_reg_clear(&framer->regs->cpri_rethbdringbaddrmsb,
			CPRI_ETH_RX_BD_RING_BASE_MSB_MASK);

	/* Coalescing threshold */
	cpri_reg_clear(&framer->regs->cpri_tethcoalthresh,
			CPRI_ETH_TX_COAL_THRES_MASK);

	cpri_reg_clear(&framer->regs->cpri_rethcoalthresh,
			CPRI_ETH_RX_COAL_THRES_MASK);

	return;
}

static void cpri_eth_align_skb(struct sk_buff *skb)
{
	unsigned int reserve;

	/* We need the data buffer to be aligned properly.  We will reserve
	 * as many bytes as needed to align the data properly
	 */
	reserve = CPRI_ETH_RXBUF_ALIGNMENT -
		(((unsigned long) skb->data) & (CPRI_ETH_RXBUF_ALIGNMENT - 1));

	if (reserve < CPRI_ETH_RXBUF_ALIGNMENT)
		skb_reserve(skb, reserve);
}


static struct sk_buff *cpri_eth_new_skb(struct net_device *ndev)
{
	struct sk_buff *skb = NULL;
	struct cpri_eth_priv *priv = netdev_priv(ndev);

	skb = netdev_alloc_skb(ndev, priv->rx_buffer_size +
						CPRI_ETH_RXBUF_ALIGNMENT);
	if (!skb)
		return NULL;

	cpri_eth_align_skb(skb);

	return skb;
}

static void cpri_eth_free_rx_skbs(struct net_device *ndev, int limit)
{
	int i;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;
	struct cpri_eth_bd_entity *rx_iter;

	for (i = 0, rx_iter = rx_bd->rx_bd_base;
			i < limit;
			i++, rx_iter++) {

		dma_unmap_single(&priv->ofdev->dev, rx_iter->buf_ptr,
				priv->rx_buffer_size, DMA_FROM_DEVICE);

		dev_kfree_skb_any(rx_bd->rx_skbuff[i]);
		rx_bd->rx_skbuff[i] = NULL;
	}

	return;
}

static void cpri_init_rxbdp(struct cpri_eth_rx_bd *rx_bd,
		struct cpri_eth_bd_entity *bdp,
			    dma_addr_t buf, unsigned int bd_index)
{
	struct net_device *ndev = rx_bd->ndev;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;
	struct cpri_eth_bd_entity txbde_le;
	struct cpri_reg_data data[1];
	unsigned int bd_nxtindex;

	txbde_le.buf_ptr = buf;
	txbde_le.lstatus = BD_LFLAG_FSHIFT(CPRI_ETH_BD_RX_EMPTY);
	CPRI_ETH_BD_TO_BE(bdp, &txbde_le); /* b-endian */
	bd_nxtindex = CPRI_ETH_NEXT_INDX(bd_index, CPRI_ETH_DEF_RX_RING_SIZE);
	data[0].val = bd_nxtindex;
	data[0].mask = CPRI_ETH_RX_BD_R_PTR_MASK;
	if (!data[0].val) {
		/* also set the wrap bit */
		data[0].val = CPRI_ETH_RX_BD_W_PTR_WRAP_MASK;
	}


	cpri_reg_vset_val(&framer->regs->cpri_rethwriteptr, data);

}


static void cpri_new_rxbdp(struct cpri_eth_rx_bd *rx_bd,
		struct cpri_eth_bd_entity *bdp,
		struct sk_buff *skb)
{
	struct net_device *ndev = rx_bd->ndev;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	static int i;
	dma_addr_t buf;

	if (priv->framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM) {
		cpri_init_rxbdp(rx_bd, bdp,
			(OCRAM_CPRI_RX_MEM +
			(i % MEDUSA_TOTAL_RX_BUFF)*
			(priv->rx_buffer_size) -
			OCRAM_MEM_OFFSET_FPGA), i);
		i++;
	} else {
		buf = dma_map_single(&priv->ofdev->dev, skb->data,
				priv->rx_buffer_size, DMA_FROM_DEVICE);
		cpri_init_rxbdp(rx_bd, bdp, buf, i);
		i++;
	}

}

static int cpri_eth_init_rx_skbs(struct cpri_eth_rx_bd *rx_bd)
{
	int i;
	struct net_device *ndev = rx_bd->ndev;
	struct cpri_eth_bd_entity *rx_iter;
	struct sk_buff *skb;

	rx_iter = rx_bd->rx_bd_base;
	for (i = 0; i < rx_bd->rx_bd_ring_size; i++) {
		skb = cpri_eth_new_skb(ndev);
		if (!skb)
			goto skb_cleanup;

		rx_bd->rx_skbuff[i]  = skb;
		cpri_new_rxbdp(rx_bd, rx_iter, skb);
		rx_iter++;
	}
	return 0;

skb_cleanup:
	cpri_eth_free_rx_skbs(ndev, i);
	return -ENOMEM;
}

static int cpri_eth_alloc_skb_resources(struct net_device *ndev)
{
	int i;
	void *vaddr;
	dma_addr_t addr;
	unsigned int dma_alloc_size;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_tx_bd *tx_bd = priv->tx_bd;
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;


	dma_alloc_size = (sizeof(struct cpri_eth_bd_entity) *
				rx_bd->rx_bd_ring_size) +
				(sizeof(struct cpri_eth_bd_entity) *
				tx_bd->tx_bd_ring_size);


	/* Allocate memory for the buffer descriptors */
	if (priv->framer->cpri_dev->dev_flags & CPRI_D4400) {
		vaddr = dma_alloc_coherent(&priv->ofdev->dev,
				dma_alloc_size + CPRI_ETH_BD_RING_ALIGN,
				&addr, GFP_KERNEL);
		if (!vaddr) {
			netdev_err(ndev, "Could not allocate bd's!\n");
			return -ENOMEM;
		}
		memset(vaddr, 0, dma_alloc_size);
	} else {

		/*Allocate memory from OCRAM*/
		/*Do FIXED allocation as landshark is not available for kernel*/
		vaddr = ioremap(OCRAM_CPRI_BD_MEM, OCRAM_CPRI_TOTAL_LEN);
		addr = OCRAM_CPRI_BD_MEM;
		cpri_ocram_vaddr = (int)vaddr;
	}

	/* Store the aligned pointers */
	priv->addr = addr;
	priv->vaddr = vaddr;


	/* Check alignment */
	vaddr = (void *)(((unsigned long)vaddr +
		 (CPRI_ETH_BD_RING_ALIGN - 1)) & ~(CPRI_ETH_BD_RING_ALIGN - 1));

	addr = ((unsigned long)addr + (CPRI_ETH_BD_RING_ALIGN - 1)) &
				~(CPRI_ETH_BD_RING_ALIGN - 1);

	/* Initialize Tx BD */
	tx_bd->tx_bd_base = vaddr;
	tx_bd->tx_bd_dma_base = addr;
	tx_bd->tx_bd_current = tx_bd->tx_bd_base;
	tx_bd->tx_bd_dirty = tx_bd->tx_bd_current;

	tx_bd->tx_skbuff = kmalloc(sizeof(*tx_bd->tx_skbuff) *
				tx_bd->tx_bd_ring_size, GFP_KERNEL);
	if (!tx_bd->tx_skbuff) {
		netdev_err(ndev, "Could not allocate tx_skbuff\n");
		goto tx_alloc_fail;
	}

	for (i = 0; i < tx_bd->tx_bd_ring_size; i++)
		tx_bd->tx_skbuff[i] = NULL;

	tx_bd->skb_curtx = 0;
	tx_bd->skb_dirtytx = 0;
	tx_bd->num_txbdfree = tx_bd->tx_bd_ring_size;

	/* Initialize Rx BD */
	addr  += sizeof(struct cpri_eth_bd_entity) * tx_bd->tx_bd_ring_size;
	vaddr += sizeof(struct cpri_eth_bd_entity) * tx_bd->tx_bd_ring_size;
	rx_bd->rx_bd_base = vaddr;
	rx_bd->rx_bd_dma_base =	addr;
	rx_bd->rx_bd_current = rx_bd->rx_bd_base;

	rx_bd->rx_skbuff = kmalloc(sizeof(*rx_bd->rx_skbuff) *
					rx_bd->rx_bd_ring_size, GFP_KERNEL);
	if (!rx_bd->rx_skbuff) {
		netdev_err(ndev, "Could not allocate rx_skbuff\n");
		goto rx_alloc_fail;
	}

	rx_bd->skb_currx = 0;
	rx_bd->ndev = ndev;

	if (cpri_eth_init_rx_skbs(rx_bd))
		goto skb_init_fail;

	if (cpri_eth_init_bd_regs(ndev))
		goto bd_init_fail;

	return 0;

bd_init_fail:
	cpri_eth_clear_bd_regs(ndev);
	cpri_eth_free_rx_skbs(ndev, priv->rx_buffer_size);
skb_init_fail:
	kfree(rx_bd->rx_skbuff);
rx_alloc_fail:
	rx_bd->rx_skbuff = 0;
	rx_bd->rx_bd_current = 0;
	rx_bd->rx_bd_dma_base = 0;
	rx_bd->rx_bd_base = 0;
	kfree(tx_bd->tx_skbuff);
tx_alloc_fail:
	tx_bd->tx_skbuff = 0;
	tx_bd->tx_bd_dirty = 0;
	tx_bd->tx_bd_current = 0;
	tx_bd->tx_bd_dma_base = 0;
	tx_bd->tx_bd_base = 0;
	dma_free_coherent(&priv->ofdev->dev,
		dma_alloc_size + CPRI_ETH_BD_RING_ALIGN, priv->vaddr,
			priv->addr);
	priv->addr = 0;
	priv->vaddr = 0;
	return -ENOMEM;
}

static void cpri_eth_free_skb_resources(struct net_device *ndev)
{
	unsigned int dma_alloc_size, i;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_tx_bd *tx_bd = priv->tx_bd;
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;

	cpri_eth_clear_bd_regs(ndev);
	for (i = 0; i < tx_bd->tx_bd_ring_size; i++) {
		if (tx_bd->tx_skbuff[i])
			dev_kfree_skb_any(tx_bd->tx_skbuff[i]);
	}

	for (i = 0; i < rx_bd->rx_bd_ring_size; i++) {
		if (rx_bd->rx_skbuff[i])
			dev_kfree_skb_any(rx_bd->rx_skbuff[i]);
	}

	kfree(tx_bd->tx_skbuff);
	tx_bd->tx_skbuff = 0;

	kfree(rx_bd->rx_skbuff);
	rx_bd->rx_skbuff = 0;

	dma_alloc_size = (sizeof(struct cpri_eth_bd_entity) *
				rx_bd->rx_bd_ring_size) +
				(sizeof(struct cpri_eth_bd_entity) *
				tx_bd->tx_bd_ring_size);

	if (priv->framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM)
		iounmap((void *)cpri_ocram_vaddr);
	else
		dma_free_coherent(&priv->ofdev->dev,
				dma_alloc_size + CPRI_ETH_BD_RING_ALIGN,
				priv->vaddr, priv->addr);


	tx_bd->tx_bd_base = 0;
	tx_bd->tx_bd_dma_base = 0;
	tx_bd->tx_bd_current = 0;
	tx_bd->tx_bd_dirty = 0;
	tx_bd->skb_curtx = tx_bd->skb_dirtytx = 0;
	tx_bd->num_txbdfree = 0;

	rx_bd->rx_bd_base = 0;
	rx_bd->rx_bd_dma_base =	0;
	rx_bd->rx_bd_current = 0;
	rx_bd->skb_currx = 0;

	priv->addr = 0;
	priv->vaddr = 0;

	return;
}

static int cpri_eth_tx_resume(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	/* This flag needs to be enabled before 're'-enabling tx */
	cpri_reg_set(&framer->regs->cpri_rethctrl,
			CPRI_ETH_RX_CTRL_DISCARD_MASK);

	cpri_reg_set(&framer->regs->cpri_tcr,
			CPRI_ETH_TX_ENABLE_MASK);
	return 0;
}

static int cpri_eth_rx_resume(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	cpri_reg_set(&framer->regs->cpri_rcr,
			CPRI_ETH_RX_ENABLE_MASK);
	return 0;
}

static int cpri_eth_tx_rx_resume(struct net_device *ndev)
{
	cpri_eth_tx_resume(ndev);
	cpri_eth_rx_resume(ndev);
	return 0;
}

static int cpri_eth_tx_halt(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	cpri_reg_clear(&framer->regs->cpri_tcr,
			CPRI_ETH_TX_ENABLE_MASK);
	return 0;
}

static int cpri_eth_rx_halt(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	cpri_reg_clear(&framer->regs->cpri_rcr,
			CPRI_ETH_RX_ENABLE_MASK);
	return 0;
}

static int cpri_eth_tx_rx_halt(struct net_device *ndev)
{
	cpri_eth_tx_halt(ndev);
	cpri_eth_rx_halt(ndev);
	return 0;
}

static int cpri_eth_open(struct net_device *ndev)
{
	int err;
	unsigned long flags;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->framer->cpri_dev->dev;


	raw_spin_lock_irqsave(&priv->tx_bd->txlock, flags);
	raw_spin_lock(&priv->rx_bd->rxlock);

	err = cpri_eth_alloc_skb_resources(ndev);

	raw_spin_unlock(&priv->rx_bd->rxlock);
	raw_spin_unlock_irqrestore(&priv->tx_bd->txlock, flags);

	if (err) {
		dev_err(dev, "cpri skb resouces allocation failed!!\n");
		return err;
	}

	napi_enable(&priv->napi);

	cpri_eth_tx_rx_resume(ndev);

	netif_start_queue(ndev);

	/* prevent tx timeout */
	ndev->trans_start = jiffies;
	ndev->flags |= IFF_UP;
	return 0;
}

static int cpri_eth_close(struct net_device *ndev)
{
	unsigned long flags;
	struct cpri_eth_priv *priv = netdev_priv(ndev);

	cpri_eth_tx_rx_halt(ndev);

	napi_disable(&priv->napi);
	netif_stop_queue(ndev);

	tasklet_disable(&priv->tasklet);

	cancel_work_sync(&priv->reset_task);
	cancel_work_sync(&priv->error_task);

	raw_spin_lock_irqsave(&priv->tx_bd->txlock, flags);
	raw_spin_lock(&priv->rx_bd->rxlock);

	cpri_eth_free_skb_resources(ndev);

	raw_spin_unlock(&priv->rx_bd->rxlock);
	raw_spin_unlock_irqrestore(&priv->tx_bd->txlock, flags);

	return 0;
}

static int cpri_eth_restart(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	unsigned long flags;

	/* same sequence as cpri_eth_close() followed by cpri_eth_open() */

	/* Stop */
	cpri_eth_tx_rx_halt(ndev);

	napi_disable(&priv->napi);
	netif_stop_queue(ndev);

	tasklet_disable(&priv->tasklet);

	/* This fn is called from reset_task workqueue.
	 * Dont cancel it!
	 * cancel_work_sync(&priv->reset_task)
	 */
	cancel_work_sync(&priv->error_task);

	netif_carrier_off(ndev);

	raw_spin_lock_irqsave(&priv->tx_bd->txlock, flags);
	raw_spin_lock(&priv->rx_bd->rxlock);

	cpri_eth_free_skb_resources(ndev);

	/* Start */
	cpri_eth_alloc_skb_resources(ndev);

	raw_spin_unlock(&priv->rx_bd->rxlock);
	raw_spin_unlock_irqrestore(&priv->tx_bd->txlock, flags);

	napi_enable(&priv->napi);

	cpri_eth_tx_rx_resume(ndev);

	netif_start_queue(ndev);

	/* prevent tx timeout */
	ndev->trans_start = jiffies;

	netif_carrier_on(ndev);

	return 0;
}

static inline void cpri_eth_stats_incr(struct net_device *ndev,
		unsigned long *sptr)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->statslock, flags);
	(*sptr)++;
	raw_spin_unlock_irqrestore(&priv->statslock, flags);
	return;
}

static inline void cpri_eth_stats_incr_val(struct net_device *ndev,
		unsigned long *sptr, unsigned int val)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->statslock, flags);
	(*sptr) += val;
	raw_spin_unlock_irqrestore(&priv->statslock, flags);
	return;
}

static int cpri_eth_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	unsigned long flags = 0;
	struct cpri_reg_data data[1];
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;
	struct cpri_eth_tx_bd *tx_bd = priv->tx_bd;
	struct cpri_eth_bd_entity *txbde;
	struct cpri_eth_bd_entity txbde_le;
	struct cpri_eth_bd_entity *base = tx_bd->tx_bd_base;
	unsigned int tx_bd_cnt = 0, tx_bd_free = 0, num_free_tx_bd = 0;
	struct device *dev = priv->framer->cpri_dev->dev;

	raw_spin_lock_irqsave(&tx_bd->txlock, flags);

	if (framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM) {
		/* Stop Tx if OCRAM MEM is allocated */
		/* In Medusa only MEDUSA_TOTAL_TX_BUFF Tx BD is available
		 * verify that if none is free fail gracefully
		 */
		for (tx_bd_cnt = 0; tx_bd_cnt < MEDUSA_TOTAL_TX_BUFF;
				tx_bd_cnt++) {
			if (OCRAM_BUF_ALLOCATED !=
				cpri_rtxm.tx_mem_ocram[tx_bd_cnt].status) {
				if (0 == num_free_tx_bd)
					tx_bd_free = tx_bd_cnt;
				num_free_tx_bd++;
			}
		}
		if (0 == num_free_tx_bd) {
			netif_stop_queue(ndev);
			cpri_eth_stats_incr(ndev, &priv->stats.tx_fifo_errors);
			cpri_eth_stats_incr(ndev, &priv->stats.tx_dropped);
			raw_spin_unlock_irqrestore(&tx_bd->txlock, flags);
			dev_err(dev, "cpri No free tx bd's!!\n");
			return NETDEV_TX_BUSY;

		} else {
			/*Free buffer available at location tx_bd_free*/
		}
	}
	/* check if there is space to queue this packet */
	if (!tx_bd->num_txbdfree) {
		/* no space, stop the queue */
		netif_stop_queue(ndev);
		cpri_eth_stats_incr(ndev, &priv->stats.tx_fifo_errors);
		cpri_eth_stats_incr(ndev, &priv->stats.tx_dropped);
		raw_spin_unlock_irqrestore(&tx_bd->txlock, flags);
		dev_err(dev, "cpri no space to queue tx frame!!\n");
		return NETDEV_TX_BUSY;
	}

	/* update transmit stats */
	cpri_eth_stats_incr_val(ndev, &priv->stats.tx_bytes, skb->len);
	cpri_eth_stats_incr(ndev, &priv->stats.tx_packets);

	/* update the descriptor */
	if (framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM) {
		/*Copy buffer to OCRAM address*/
		txbde_le.lstatus = 0;
		txbde_le.lstatus |= BD_LFLAG_LSHIFT(skb_headlen(skb));

		cpri_rtxm.tx_mem_ocram[tx_bd_free].status = OCRAM_BUF_ALLOCATED;
		cpri_rtxm.tx_mem_ocram[tx_bd_free].ocram_ptr =
			(OCRAM_CPRI_TX_MEM + tx_bd_free *
			(CPRI_ETH_DEF_RX_BUF_SIZE + CPRI_ETH_RXBUF_ALIGNMENT));

		cpri_memcpy_ocram((int *)(cpri_ocram_vaddr
				+ (cpri_rtxm.tx_mem_ocram[tx_bd_free].ocram_ptr
				- OCRAM_CPRI_BD_MEM)), skb->data,
				skb_headlen(skb));
		cpri_mem_sync();
		txbde_le.buf_ptr = cpri_rtxm.tx_mem_ocram[tx_bd_free].ocram_ptr
			- OCRAM_MEM_OFFSET_FPGA;
		txbde_le.lstatus |= BD_LFLAG_FSHIFT(CPRI_ETH_BD_TX_READY);
		tx_bd_free++;
	} else {
		txbde_le.lstatus = 0;
		txbde_le.lstatus |= BD_LFLAG_LSHIFT(skb_headlen(skb));
		txbde_le.buf_ptr = dma_map_single(&priv->ofdev->dev, skb->data,
				skb_headlen(skb), DMA_TO_DEVICE);
		txbde_le.lstatus |= BD_LFLAG_FSHIFT(CPRI_ETH_BD_TX_READY);
	}


	/* CPRI IP expects the buffer descriptors in big endian
	 * format; section 22.6
	 */
	txbde = tx_bd->tx_bd_current;
	CPRI_ETH_BD_TO_BE(txbde, &txbde_le); /* b-endian */
	netdev_sent_queue(ndev, skb->len);

	wmb();

	tx_bd->tx_skbuff[tx_bd->skb_curtx] = skb;

	tx_bd->skb_curtx =
		CPRI_ETH_NEXT_INDX(tx_bd->skb_curtx, CPRI_ETH_DEF_TX_RING_SIZE);
	tx_bd->tx_bd_current =
		CPRI_ETH_NEXT_BDE(txbde, base, tx_bd->tx_bd_ring_size);

	(tx_bd->num_txbdfree)--;

	if (!tx_bd->num_txbdfree) {
		netif_stop_queue(ndev);
		cpri_eth_stats_incr(ndev, &priv->stats.tx_fifo_errors);
		dev_err(dev, "cpri no space to queue!!\n");
	}
	data[0].val = tx_bd->skb_curtx;
	data[0].mask = CPRI_ETH_TX_BD_R_PTR_MASK;
	if (!data[0].val) {
		/* also set the wrap bit */
		data[0].val = CPRI_ETH_TX_BD_W_PTR_WRAP_MASK;
	}

	raw_spin_unlock_irqrestore(&tx_bd->txlock, flags);

	cpri_reg_vset_val(&framer->regs->cpri_tethwriteptr, data);

	return NETDEV_TX_OK;
}

static inline void cpri_eth_config_clear(struct net_device *ndev,
	unsigned *addr, unsigned int flag, unsigned int mask)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);

	priv->flags &= ~flag;
	cpri_reg_clear(addr, mask);

	return;
}

static inline void cpri_eth_config_set(struct net_device *ndev, unsigned *addr,
	unsigned int flag, unsigned int mask)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);

	priv->flags |= flag;
	cpri_reg_set(addr, mask);

	return;
}

static inline void cpri_eth_config_update(struct net_device *ndev,
	unsigned *reg, unsigned int flags, unsigned int flag, unsigned int mask)
{
	if (flags & flag)
		cpri_eth_config_set(ndev, reg, flag, mask);
	else
		cpri_eth_config_clear(ndev, reg, flag, mask);

	return;
}

static int cpri_eth_config(struct net_device *ndev, unsigned int flags)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	unsigned changed = priv->flags ^ flags;

	if (changed & CPRI_ETH_HW_CRC_STRIP) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_HW_CRC_STRIP, CPRI_ETH_HW_CRC_STRIP_MASK);

	} else if (changed & CPRI_ETH_MAC_FAIL_PASS) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_MAC_FAIL_PASS, CPRI_ETH_MAC_FAIL_PASS_MASK);

	} else if (changed & CPRI_ETH_TRIG_TX_ABORT) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_TX_ABORT, CPRI_ETH_TRIG_TX_ABORT_MASK);

	} else if (changed & CPRI_ETH_TRIG_TX_READY) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_TX_READY, CPRI_ETH_TRIG_TX_RDY_MASK);

	} else if (changed & CPRI_ETH_TRIG_RX_ABORT) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_RX_ABORT, CPRI_ETH_TRIG_RX_ABORT_MASK);

	} else if (changed & CPRI_ETH_TRIG_RX_READY) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_RX_READY, CPRI_ETH_TRIG_RX_RDY_MASK);

	} else if (changed & CPRI_ETH_TRIG_TX) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_TX, CPRI_ETH_TRIG_TX_MASK);

	} else if (changed & CPRI_ETH_TRIG_RX) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_RX, CPRI_ETH_TRIG_RX_MASK);

	} else if (changed & CPRI_ETH_LONG_FRAME) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_LONG_FRAME, CPRI_ETH_LONG_FRAME_MASK);

	} else if (changed & CPRI_ETH_RX_PREAMBLE_ABORT) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_RX_PREAMBLE_ABORT, CPRI_ETH_RX_PR_ABORT_MASK);

	} else if (changed & CPRI_ETH_BCAST) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_BCAST, CPRI_ETH_BCAST_MASK);

	} else if (changed & CPRI_ETH_MCAST_FLT) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_MCAST_FLT, CPRI_ETH_MCAST_FLT_MASK);

	} else if (changed & CPRI_ETH_MAC_CHECK) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_MAC_CHECK, CPRI_ETH_MAC_CHECK_MASK);

	} else if (changed & CPRI_ETH_LEN_CHECK) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_LEN_CHECK, CPRI_ETH_LEN_CHECK_MASK);

	} else if (changed & CPRI_ETH_LITTLE_END) {
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_LITTLE_END, CPRI_ETH_LITTLE_END_MASK);

	} else if (changed & CPRI_ETH_HW_CRC_EN) {
		/* For TX, We support on HW generated FCS for now! */
		if (flags & CPRI_ETH_HW_CRC_EN) {
			cpri_eth_config_update(ndev,
				&framer->regs->cpri_ethcfg2,
				flags, CPRI_ETH_HW_CRC_EN,
				CPRI_ETH_HW_CRC_EN_MASK);
		}

	} else if (changed & CPRI_ETH_HW_CRC_CHECK) {
		cpri_eth_config_update(ndev,
				&framer->regs->cpri_ethcfg3, flags,
				CPRI_ETH_HW_CRC_CHECK,
				CPRI_ETH_HW_CRC_CHECK_MASK);

	} else {
		if (changed & CPRI_ETH_STORE_FWD) {
			cpri_eth_config_update(ndev,
					&framer->regs->cpri_ethcfg3,
					flags, CPRI_ETH_STORE_FWD,
					CPRI_ETH_STORE_FWD_MASK);
		}
	}

	return 0;
}

static int cpri_eth_set_features(struct net_device *ndev,
				 netdev_features_t features)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	netdev_features_t changed = features ^ ndev->features;

	if (!(changed & (NETIF_F_RXFCS | NETIF_F_RXALL)))
		return 0;

	ndev->features = features;

	if (changed & NETIF_F_RXFCS) {
		if (features & NETIF_F_RXFCS) {
			cpri_eth_config(ndev,
				priv->flags & ~CPRI_ETH_HW_CRC_STRIP);
		} else {
			cpri_eth_config(ndev,
				priv->flags | CPRI_ETH_HW_CRC_STRIP);
		}
	}

	if (changed & NETIF_F_RXALL) {
		/* MAC_FAIL_PASS_EN is not handled here. As enabling the
		 * ETH_FWD_IF requires this flag; it is always set
		 */
		if (features & NETIF_F_RXALL) {
			cpri_eth_config(ndev,
				(priv->flags | CPRI_ETH_LONG_FRAME) &
				~CPRI_ETH_RX_PREAMBLE_ABORT &
				~CPRI_ETH_HW_CRC_CHECK);
		} else {
			cpri_eth_config(ndev,
				(priv->flags & ~CPRI_ETH_LONG_FRAME) |
				CPRI_ETH_RX_PREAMBLE_ABORT |
				CPRI_ETH_HW_CRC_CHECK);
		}
	}
	return 0;
}

static unsigned int cpri_eth_mac_hash(char *dmac)
{
	unsigned int crc, data, n, i;

	crc = 0xFFFFFFFF;
	for (n = 0; n < 6; n++) {
		data = dmac[n];
		for (i = 0; i < 8; i++) {
			if (((crc>>31)&0x1) == (data&0x1))
				crc = crc<<1;
			else
				crc = (crc<<1) ^ 0x04C11DB7;
			data = data>>1;
		}
	}
	return crc&0x1F;
}

static void cpri_eth_set_mcast_hash(struct net_device *ndev)
{
	unsigned int mask = 0;
	struct netdev_hw_addr *ha;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	netdev_for_each_mc_addr(ha, ndev) {
		/* cpri_eth_mac_hash return values 0-31 */
		mask |= (1 << cpri_eth_mac_hash(ha->addr));
	}

	cpri_reg_set_val(&framer->regs->cpri_ethhashtbl,
		CPRI_ETH_MCAST_MASK, mask);
	return;
}


static void cpri_eth_set_rx_mode(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	if (ndev->flags & IFF_PROMISC)
		cpri_eth_config(ndev, priv->flags | CPRI_ETH_MAC_CHECK);
	else
		cpri_eth_config(ndev, priv->flags & ~CPRI_ETH_MAC_CHECK);

	if (ndev->flags & IFF_ALLMULTI) {
		/* Enable all multicast */
		cpri_reg_set_val(&framer->regs->cpri_ethhashtbl,
			CPRI_ETH_MCAST_MASK, 0xffffffff);

		cpri_eth_config(ndev, priv->flags | CPRI_ETH_MCAST_FLT);
	} else {
		if (netdev_mc_empty(ndev)) {
			cpri_eth_config(ndev,
				priv->flags & ~CPRI_ETH_MCAST_FLT);
			return;
		}

		/* We need to enable specific multicast filters */
		cpri_eth_set_mcast_hash(ndev);

		cpri_eth_config(ndev, priv->flags | CPRI_ETH_MCAST_FLT);
	}
	return;
}

static void cpri_eth_reset_task(struct work_struct *work)
{
	struct cpri_eth_priv *priv = container_of(work, struct cpri_eth_priv,
			reset_task);
	struct net_device *ndev = priv->ndev;

	if (ndev->flags & IFF_UP)
		cpri_eth_restart(ndev);
}

static void cpri_eth_timeout(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);

	cpri_eth_stats_incr(ndev, &priv->stats.tx_fifo_errors);

	schedule_work(&priv->reset_task);
}

static int cpri_eth_set_mac_addr(struct net_device *ndev, void *p)
{
	unsigned int msb = 0, lsb = 0;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, addr->sa_data, ETH_ALEN);

	memcpy(&lsb, ((char *)ndev->dev_addr) + 2, 4);
	memcpy(((char *)&msb) + 2, ndev->dev_addr, 2);

	/* MAC is already in big endian. So swap it once, so that
	 * when writel swaps it will result in big endian
	 */
	cpri_reg_set_val(&framer->regs->cpri_ethaddrlsb,
		CPRI_ETH_ADDR_LSB_MASK, cpu_to_be32(lsb));
	cpri_reg_set_val(&framer->regs->cpri_ethaddrmsb,
		CPRI_ETH_ADDR_MSB_MASK, cpu_to_be32(msb));

	return 0;
}

static struct net_device_stats *cpri_eth_get_stats(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	unsigned long rx_errors = 0, flags;

	raw_spin_lock_irqsave(&priv->statslock, flags);

	memcpy(&ndev->stats, &priv->stats, sizeof(struct net_device_stats));

	rx_errors += priv->extra_stats.rx_dma_overrun;
	rx_errors += priv->extra_stats.rx_overrun;
	rx_errors += priv->extra_stats.rx_underrun;
	rx_errors += priv->extra_stats.mii_error;
	rx_errors += priv->extra_stats.small_pkt;
	rx_errors += priv->extra_stats.dmac_mismatch;
	rx_errors += priv->extra_stats.crc_error;
	rx_errors += priv->extra_stats.cpri_rx_error;
	rx_errors += priv->extra_stats.long_frame;
	rx_errors += priv->extra_stats.short_frame;

	ndev->stats.rx_errors = rx_errors;
	ndev->stats.tx_errors = priv->extra_stats.tx_underrun;

	raw_spin_unlock_irqrestore(&priv->statslock, flags);

	return &ndev->stats;
}

static const struct net_device_ops cpri_eth_netdev_ops = {
	.ndo_open = cpri_eth_open,
	.ndo_stop = cpri_eth_close,
	.ndo_start_xmit = cpri_eth_xmit,
	.ndo_set_features = cpri_eth_set_features,
	.ndo_set_rx_mode = cpri_eth_set_rx_mode,
	.ndo_tx_timeout = cpri_eth_timeout,
	.ndo_set_mac_address = cpri_eth_set_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_get_stats = cpri_eth_get_stats,
};

/* Interrupt handlers */
/* These will be called from CPRI code, in IRQ context */
static void cpri_eth_tx_cleanup(unsigned long data)
{
	struct sk_buff *skb = NULL;
	int skb_dirtytx = 0;
	int howmany = 0;
	unsigned int bytes_sent = 0;
	unsigned long flags;

	struct cpri_eth_priv *priv = (struct cpri_eth_priv *)data;
	struct cpri_eth_bd_entity *txbde, *base, txbde_le;
	struct cpri_eth_tx_bd *tx_bd = priv->tx_bd;
	int tx_ring_size = tx_bd->tx_bd_ring_size;
	struct net_device *ndev = priv->ndev;
	struct cpri_framer *framer = priv->framer;

	raw_spin_lock_irqsave(&tx_bd->txlock, flags);

	base = tx_bd->tx_bd_base;
	txbde = tx_bd->tx_bd_dirty;
	skb_dirtytx = tx_bd->skb_dirtytx;

	/* Not using the Tx Read pointer here.
	cpri_reg_get_val(&framer->regs->cpri_tethrdptrring,
		CPRI_ETH_TX_BD_R_PTR_MASK, &val);
	*/

	while (txbde != tx_bd->tx_bd_current) {

		CPRI_ETH_BD_TO_LE(&txbde_le, txbde);/* passing b-endian */

		if (BD_LSTATUS_SSHIFT(txbde_le.lstatus) && CPRI_ETH_BD_TX_READY)
			/* Tx DMA will clear this bit after Tx complete */
			break;

		skb = tx_bd->tx_skbuff[skb_dirtytx];

		if (framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM)
			cpri_free_ocram_bd(txbde_le.buf_ptr);
		else
			dma_unmap_single(&priv->ofdev->dev, txbde_le.buf_ptr,
				skb_headlen(skb), DMA_TO_DEVICE);

		bytes_sent += skb->len;

		dev_kfree_skb_any(skb);
		tx_bd->tx_skbuff[skb_dirtytx] = NULL;
		txbde->buf_ptr = 0;
		txbde->lstatus = 0;

		txbde = CPRI_ETH_NEXT_BDE(txbde, base, tx_ring_size);

		skb_dirtytx = CPRI_ETH_NEXT_INDX(skb_dirtytx,
					CPRI_ETH_DEF_TX_RING_SIZE);

		howmany++;

		(tx_bd->num_txbdfree)++;
	}

	if (netif_queue_stopped(ndev) && tx_bd->num_txbdfree)
		netif_wake_queue(ndev);

	/* Update dirty indicators */
	tx_bd->skb_dirtytx = skb_dirtytx;
	tx_bd->tx_bd_dirty = txbde;

	raw_spin_unlock_irqrestore(&tx_bd->txlock, flags);

	netdev_completed_queue(ndev, howmany, bytes_sent);

	cpri_eth_tx_resume(ndev);

	return;
}

int cpri_eth_handle_tx(struct cpri_framer *framer)
{
	struct cpri_eth_priv *priv = framer->eth_priv;

	cpri_eth_tx_halt(priv->ndev);

	tasklet_schedule(&framer->eth_priv->tasklet);

	return 0;
}
EXPORT_SYMBOL(cpri_eth_handle_tx);

static void cpri_eth_check_rx_ex_status_reg(struct net_device *ndev)
{
	unsigned int estatus;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	/* Read clears the ex. status register */
	estatus = cpri_reg_get_val(
			&framer->regs->cpri_rethexstatus, 0xffffffff);

	if (!estatus)
		return;

	if (estatus & CPRI_ETH_RX_EST_MII_PERR_MASK) {
		cpri_eth_stats_incr(ndev, &priv->extra_stats.mii_error);
	} else if (estatus & CPRI_ETH_RX_EST_SPKT_MASK) {
		cpri_eth_stats_incr(ndev, &priv->extra_stats.small_pkt);
	} else if (estatus & CPRI_ETH_RX_EST_OFLW_MASK) {
		cpri_eth_stats_incr(ndev, &priv->extra_stats.rx_overflow);
	} else if (estatus & CPRI_ETH_RX_EST_DMAC_MM_MASK) {
		cpri_eth_stats_incr(ndev, &priv->extra_stats.dmac_mismatch);
	} else if (estatus & CPRI_ETH_RX_EST_CRC_ERR_MASK) {
		cpri_eth_stats_incr(ndev, &priv->extra_stats.crc_error);
	} else if (estatus & CPRI_ETH_RX_EST_RX_ERR_MASK) {
		cpri_eth_stats_incr(ndev, &priv->extra_stats.cpri_rx_error);
	} else if (estatus & CPRI_ETH_RX_EST_LFRM_MASK) {
		cpri_eth_stats_incr(ndev, &priv->extra_stats.long_frame);
	} else {
		if (estatus & CPRI_ETH_RX_EST_SFRM_MASK) {
			cpri_eth_stats_incr(ndev,
				&priv->extra_stats.short_frame);
		}
	}
}

static int cpri_eth_rx_pkt_error(struct net_device *ndev,
				 unsigned int skb_currx)
{
	struct cpri_eth_bd_entity *rxbde, rxbde_le;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;

	rxbde = rx_bd->rx_bd_base + skb_currx;

	CPRI_ETH_BD_TO_LE(&rxbde_le, rxbde); /* b-endian */

	if (BD_LSTATUS_SSHIFT(rxbde_le.lstatus) && CPRI_ETH_BD_RX_CRC)
		cpri_eth_stats_incr(ndev, &priv->stats.rx_crc_errors);
	else if (BD_LSTATUS_SSHIFT(rxbde_le.lstatus) && CPRI_ETH_BD_RX_PLE)
		cpri_eth_stats_incr(ndev, &priv->stats.rx_length_errors);
	else if (BD_LSTATUS_SSHIFT(rxbde_le.lstatus) && CPRI_ETH_BD_RX_BOF)
			cpri_eth_stats_incr(ndev, &priv->stats.rx_over_errors);

	/* Re use the same skb and DMA mapping */
	/* Just clear the ctrl info */
	rxbde_le.lstatus = BD_LFLAG_FSHIFT(CPRI_ETH_BD_RX_EMPTY);
	CPRI_ETH_BD_TO_BE(rxbde, &rxbde_le); /* B-endian */
	return 0;
}

static int cpri_eth_rx_pkt(struct net_device *ndev, unsigned int skb_currx)
{
	int pkt_len, ret;
	struct sk_buff *newskb, *skb;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;
	struct cpri_eth_bd_entity *rxbde, rxbde_le;
	int bf_ptr;

	memset(&rxbde_le, 0, sizeof(struct cpri_eth_bd_entity));

	rxbde = rx_bd->rx_bd_base + skb_currx;
	skb = rx_bd->rx_skbuff[skb_currx];

	CPRI_ETH_BD_TO_LE(&rxbde_le, rxbde); /* b-endian */
	pkt_len = BD_LSTATUS_LSHIFT(rxbde_le.lstatus);

	if (priv->framer->cpri_dev->dev_flags & CPRI_MEDUSA_OCRAM) {
		bf_ptr =  rxbde_le.buf_ptr;
		cpri_memcpy_ocram(skb->data,
			(void *)(bf_ptr + OCRAM_MEM_OFFSET_FPGA
				+ cpri_ocram_vaddr - OCRAM_CPRI_BD_MEM),
				pkt_len);
	} else
		dma_unmap_single(&priv->ofdev->dev, rxbde_le.buf_ptr,
				priv->rx_buffer_size, DMA_FROM_DEVICE);

	/* Add another skb for the future */
	newskb = cpri_eth_new_skb(ndev);

	if (skb->pkt_type == PACKET_MULTICAST)
		cpri_eth_stats_incr(ndev, &priv->stats.multicast);

	if (unlikely(!newskb)) {
		/* we drop the current packet and reuse the skb */
		cpri_eth_stats_incr(ndev, &priv->stats.rx_dropped);
		newskb = skb;
	} else {
		/* Increment the number of packets */
		cpri_eth_stats_incr(ndev, &priv->stats.rx_packets);


		if (!(priv->flags & CPRI_ETH_HW_CRC_STRIP)) {
			/* We need to strip it */
			pkt_len -= ETH_FCS_LEN;
		}

		skb_put(skb, pkt_len);
		cpri_eth_stats_incr_val(ndev, &priv->stats.rx_bytes, pkt_len);

		/* No checksum offloading */
		skb_checksum_none_assert(skb);

		/* Tell the skb what kind of packet this is */
		skb->protocol = eth_type_trans(skb, ndev);

		ret = netif_receive_skb(skb);

		if (NET_RX_DROP == ret) {
			cpri_eth_stats_incr(ndev, &priv->stats.rx_dropped);
			cpri_eth_stats_incr(ndev,
				&priv->extra_stats.kernel_dropped);
		}
	}

	rx_bd->rx_skbuff[skb_currx] = newskb;

	cpri_new_rxbdp(rx_bd, rxbde, skb);
	return 0;
}

static int cpri_eth_clean_rx_ring(struct net_device *ndev, int budget)
{
	int howmany = 0, pkterr = 0;
	unsigned int skb_currx = 0;
	unsigned long flags;
	struct cpri_eth_bd_entity *rxbde, *base;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;


	raw_spin_lock_irqsave(&rx_bd->rxlock, flags);
	base = (struct cpri_eth_bd_entity *)rx_bd->rx_bd_dma_base;
	rxbde = rx_bd->rx_bd_current;
	skb_currx = rx_bd->skb_currx;

	rmb();

	/* No LE conversion here. Ctrl is only a byte value */

	while (!(BD_LSTATUS_LSHIFT(rxbde->lstatus) && CPRI_ETH_BD_RX_EMPTY)
					&& (howmany < budget)) {
		rmb();

		if (BD_LSTATUS_LSHIFT(rxbde->lstatus) && CPRI_ETH_BD_RX_ABORT) {
			cpri_eth_rx_pkt_error(ndev, skb_currx);
			pkterr++;
		} else {
			cpri_eth_rx_pkt(ndev, skb_currx);
		}

		howmany++;

		rxbde =
			CPRI_ETH_NEXT_BDE(rxbde, base, rx_bd->rx_bd_ring_size);
		skb_currx =
			CPRI_ETH_NEXT_INDX(skb_currx, rx_bd->rx_bd_ring_size);


	}

	if (pkterr)
		cpri_eth_check_rx_ex_status_reg(ndev);

	rx_bd->rx_bd_current = rxbde;
	rx_bd->skb_currx = skb_currx;

	raw_spin_unlock_irqrestore(&rx_bd->rxlock, flags);

	return howmany;
}

static void cpri_eth_error_task(struct work_struct *work)
{
	struct cpri_eth_priv *priv;
	struct net_device *ndev;
	unsigned int errval;
	struct cpri_framer *framer;

	priv = container_of(work, struct cpri_eth_priv, error_task);
	ndev = priv->ndev;
	framer = priv->framer;

	errval = cpri_reg_get_val(&framer->regs->cpri_errinten, 0xffffffff);

	if (errval & CPRI_ETH_REM_FF_EN_MASK) {
		/* Remote FIFO full */
		/* We are not registered for this event. skip it! */
	} else if ((errval & CPRI_ETH_RX_DMA_OVR_EN_MASK) ||
		 (errval & CPRI_ETH_RX_OVR_EN_MASK)) {

		if (errval & CPRI_ETH_RX_DMA_OVR_EN_MASK)
			cpri_eth_stats_incr(ndev,
				&priv->extra_stats.rx_dma_overrun);
		else
			cpri_eth_stats_incr(ndev,
				&priv->extra_stats.rx_overrun);

		/* Rx Overrun - Restart Rx DMA*/
		napi_disable(&priv->napi);
		cpri_eth_rx_halt(ndev);
		cpri_eth_check_rx_ex_status_reg(ndev);
		/* Set 1 to clear the overflow bit */
		cpri_reg_set(&framer->regs->cpri_errinten,
			CPRI_ETH_RX_DMA_OVR_EN_MASK);
		napi_enable(&priv->napi);
		cpri_eth_rx_resume(ndev);

	} else if (errval & CPRI_ETH_RX_BD_UDR_EN_MASK) {

		cpri_eth_stats_incr(ndev, &priv->extra_stats.rx_underrun);

		/* Make space in the rx ring and enable dma*/
		napi_disable(&priv->napi);
		cpri_eth_rx_halt(ndev);

		cpri_eth_clean_rx_ring(ndev, CPRI_ETH_NAPI_WEIGHT);

		napi_enable(&priv->napi);
		cpri_eth_rx_resume(ndev);

	} else if (errval & CPRI_ETH_TX_UDR_EN_MASK) {

		cpri_eth_stats_incr(ndev, &priv->extra_stats.tx_underrun);

		netif_tx_disable(ndev);
		cpri_eth_tx_halt(ndev);
		cpri_eth_tx_resume(ndev);
		netif_wake_queue(ndev);
	}

	return;
}

int cpri_eth_handle_error(struct cpri_framer *framer)
{
	struct cpri_eth_priv *priv = framer->eth_priv;

	schedule_work(&priv->error_task);

	return 0;
}
EXPORT_SYMBOL(cpri_eth_handle_error);

static int cpri_eth_poll(struct napi_struct *napi, int budget)
{
	int howmany;
	struct cpri_eth_priv *priv =
		container_of(napi, struct cpri_eth_priv, napi);
	struct net_device *ndev = priv->ndev;

	howmany = cpri_eth_clean_rx_ring(ndev, budget);

	if (howmany < budget) {
		napi_complete(napi);
		cpri_eth_rx_resume(ndev);
	}

	return howmany;
}

int cpri_eth_handle_rx(struct cpri_framer *framer)
{
	struct cpri_eth_priv *priv = framer->eth_priv;

	if (napi_schedule_prep(&priv->napi)) {
		cpri_eth_rx_halt(priv->ndev);
		__napi_schedule(&priv->napi);
	}
	return 0;
}
EXPORT_SYMBOL(cpri_eth_handle_rx);

int cpri_eth_of_init(struct platform_device *ofdev,
				struct net_device **pdev,
				struct device_node *frnode)
{
	struct net_device *ndev = NULL;
	struct cpri_eth_priv *priv = NULL;
	const void *mac_addr;
	int err = 0;

	ndev = alloc_etherdev(sizeof(*priv));
	*pdev = ndev;

	if (NULL == ndev)
		return -ENOMEM;

	mac_addr = of_get_mac_address(frnode);
	if (mac_addr) {
		memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
		memcpy(ndev->perm_addr, mac_addr, ETH_ALEN);
	} else {
		netdev_err(ndev, "MAC address not specified\n");
		err = -EINVAL;
		goto mac_failed;
	}

	priv = netdev_priv(ndev);

	memset(priv, 0, sizeof(*priv));
	priv->ofdev = ofdev;
	priv->ndev = ndev;
	netif_napi_add(ndev, &priv->napi, cpri_eth_poll, CPRI_ETH_NAPI_WEIGHT);
	/* Initialize few fields in the BD; the rest are
	 * initialized in cpri_eth_alloc_skb_resources()
	 */
	priv->tx_bd = kzalloc(sizeof(struct cpri_eth_tx_bd), GFP_KERNEL);
	if (!priv->tx_bd) {
		err = -ENOMEM;
		netdev_err(ndev, "Failed to allocate tx bd\n");
		goto tx_alloc_failed;
	}
	priv->tx_bd->tx_bd_ring_size = CPRI_ETH_DEF_TX_RING_SIZE;
	raw_spin_lock_init(&priv->tx_bd->txlock);

	priv->rx_bd = kzalloc(sizeof(struct cpri_eth_rx_bd), GFP_KERNEL);
	if (!priv->rx_bd) {
		err = -ENOMEM;
		netdev_err(ndev, "Failed to allocate rx bd\n");
		goto rx_alloc_failed;
	}
	priv->rx_bd->rx_bd_ring_size = CPRI_ETH_DEF_RX_RING_SIZE;
	raw_spin_lock_init(&priv->rx_bd->rxlock);
	INIT_WORK(&priv->reset_task, cpri_eth_reset_task);
	INIT_WORK(&priv->error_task, cpri_eth_error_task);
	tasklet_init(&priv->tasklet, cpri_eth_tx_cleanup, (unsigned long)priv);
	priv->flags = CPRI_ETH_DEF_FLAGS;
	priv->rx_buffer_size = CPRI_ETH_DEF_RX_BUF_SIZE;
	priv->tx_start_thresh = CPRI_ETH_DEF_TX_START_THRESH;
	priv->rx_coales_thresh = CPRI_ETH_DEF_RX_COAL_THRESH;
	priv->tx_coales_thresh = CPRI_ETH_DEF_TX_COAL_THRESH;

	priv->fwdif_status = CPRI_ETH_DISABLED;

	raw_spin_lock_init(&priv->initlock);
	raw_spin_lock_init(&priv->statslock);
	return 0;

rx_alloc_failed:
	kfree(priv->tx_bd);
mac_failed:
tx_alloc_failed:
	free_netdev(ndev);
	*pdev = 0;
	return err;
}


static void cpri_eth_release_priv(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);

	/* Ensure that tasklets/workqueue/napi/rx/tx path etc.
	 * are all disabled before reaching here
	 */
	netif_napi_del(&priv->napi);
	kfree(priv->tx_bd);
	kfree(priv->rx_bd);

	return;
}


static int cpri_eth_fwd_if_enable(struct net_device *ndev)
{
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;

	cpri_eth_config_set(ndev, &framer->regs->cpri_ethcfg1,
			CPRI_ETH_HW_CRC_STRIP, CPRI_ETH_HW_CRC_STRIP_MASK);
	cpri_eth_config_set(ndev, &framer->regs->cpri_ethcfg1,
			CPRI_ETH_MAC_FAIL_PASS, CPRI_ETH_MAC_FAIL_PASS_MASK);

	cpri_reg_set(&framer->regs->cpri_ethfwdctrl,
			CPRI_ETH_FWD_ENABLE_MASK);

	return 0;
}

/* This function will be invoked by CPRI code after doing
 * the initialization
 */
void cpri_eth_enable(struct cpri_framer *framer)
{
	struct net_device *ndev = framer->eth_priv->ndev;
	struct cpri_eth_priv *priv = netdev_priv(ndev);

	/* Fwd IF Cannot be disabled once enabled enabling
	 * must be done before CPRInRCR(RETHE) and
	 * CPRInTCR(TETHE)
	 */
	if (priv->fwdif_status == CPRI_ETH_DISABLED) {
		cpri_eth_fwd_if_enable(ndev);
		priv->fwdif_status = CPRI_ETH_ENABLED;
	} else
		return;

	/* Enable interrupt events */
	cpri_reg_set(&framer->regs->cpri_rctrltiminginten,
			CPRI_ETH_RX_EV_EN_MASK);
	cpri_reg_set(&framer->regs->cpri_tctrltiminginten,
			CPRI_ETH_TX_EV_EN_MASK);

	/* We cant do much when the remote fifo is full. Skip this event
	 * cpri_reg_set(&framer->regs->cpri_errinten,
	 *			CPRI_ETH_REM_FF_EN_MASK);
	 */

	cpri_reg_set(&framer->regs->cpri_errinten,
			CPRI_ETH_RX_DMA_OVR_EN_MASK);
	cpri_reg_set(&framer->regs->cpri_errinten,
			CPRI_ETH_RX_BD_UDR_EN_MASK);
	cpri_reg_set(&framer->regs->cpri_errinten,
			CPRI_ETH_TX_UDR_EN_MASK);
	cpri_reg_set(&framer->regs->cpri_errinten,
			CPRI_ETH_RX_OVR_EN_MASK);

	netif_carrier_on(ndev);

	return;
}
EXPORT_SYMBOL(cpri_eth_enable);


int cpri_eth_init(struct platform_device *ofdev, struct cpri_framer *framer,
			struct device_node *frnode)
{
	struct net_device *ndev = NULL;
	struct cpri_eth_priv *priv = NULL;
	int err = 0;

	err = cpri_eth_of_init(ofdev, &ndev, frnode);
	if (err < 0)
		return err;

	priv = netdev_priv(ndev);

	priv->framer = framer;

	SET_NETDEV_DEV(ndev, &ofdev->dev);

	/* Fill in the dev structure */
	ndev->watchdog_timeo = CPRI_ETH_TX_TIMEOUT;
	ndev->mtu = CPRI_ETH_DEF_MTU;
	ndev->netdev_ops = &cpri_eth_netdev_ops;
	sprintf(ndev->name, "%s%d_eth", DEV_NAME, framer->id-1);
	/* TODO: dev->ethtool_ops = &cpri_eth_ethtool_ops; */

	ndev->features |= NETIF_F_HIGHDMA;

	ndev->hw_features |= (NETIF_F_RXCSUM |
				NETIF_F_RXALL |
				NETIF_F_RXFCS);

	cpri_eth_tx_rx_halt(ndev);
	cpri_eth_config(ndev, CPRI_ETH_DEF_FLAGS);

	err = register_netdev(ndev);
	if (err) {
		netdev_err(ndev, "Cannot register net device, aborting\n");
		goto register_fail;
	}

	/* TODO: cpri_eth_init_sysfs(ndev); */

	framer->eth_priv = priv;

	return 0;

register_fail:
	cpri_eth_release_priv(ndev);
	free_netdev(ndev);
	return err;
}
EXPORT_SYMBOL(cpri_eth_init);

void cpri_eth_parm_init(struct cpri_framer *framer)
{
	cpri_eth_tx_rx_halt(framer->eth_priv->ndev);
	cpri_eth_config(framer->eth_priv->ndev, CPRI_ETH_DEF_FLAGS);
}

void cpri_eth_deinit(struct platform_device *ofdev, struct cpri_framer *framer)
{
	struct net_device *ndev = framer->eth_priv->ndev;

	BUG_ON(ndev->flags & IFF_UP);

	cpri_eth_release_priv(framer->eth_priv->ndev);

	unregister_netdev(framer->eth_priv->ndev);
	free_netdev(ndev);

	framer->eth_priv = 0;

	return;
}
EXPORT_SYMBOL(cpri_eth_deinit);

/*
 * drivers/net/cpri/cpri_hdlc.c
 * CPRI device driver - HDLC MAC
 * Author: NXP semiconductor, Inc.
 *
 * Copyright 2016 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/cpri.h>
#include <linux/fs.h>
#include "cpri.h"

void cpri_hdlc_of_init(struct cpri_framer *framer)
{
	spin_lock_init(&framer->hdlc_priv.tx_lock);
	spin_lock_init(&framer->hdlc_priv.rx_lock);
	atomic_set(&framer->hdlc_priv.users, 0);

	framer->hdlc_priv.hdlc_config.timeout = -1;
	framer->hdlc_priv.timeout = MAX_SCHEDULE_TIMEOUT;
	framer->hdlc_priv.hdlc_config.enable_hdlc_rx_hw_len_chk = 1;
	framer->hdlc_priv.hdlc_config.enable_hdlc_rx_sw_len_chk = 0;
	framer->hdlc_priv.hdlc_config.enable_hdlc_tx_crc_chk = 1;
	framer->hdlc_priv.hdlc_config.enable_hdlc_rx_crc_chk = 1;
	framer->hdlc_priv.hdlc_config.hdlc_min_rx = -1;
	memset(&framer->hdlc_priv.stats, 0, sizeof(struct cpri_hdlc_stats));
}

static void cpri_hdlc_set_config_reg(struct cpri_framer *framer)
{
	unsigned int rx_len_mask = 0, tx_crc_mask = 0, rx_crc_mask = 0;

	if (framer->hdlc_priv.hdlc_config.enable_hdlc_rx_hw_len_chk)
		rx_len_mask = 0x10;
	if (framer->hdlc_priv.hdlc_config.enable_hdlc_tx_crc_chk)
		tx_crc_mask = 0x1;
	if (framer->hdlc_priv.hdlc_config.enable_hdlc_rx_crc_chk)
		rx_crc_mask = 0x2;

	cpri_write((0x00067C00 | rx_len_mask), &framer->regs->cpri_hdlccfg1);
	cpri_write(tx_crc_mask, &framer->regs->cpri_hdlccfg2);
	cpri_write((0x400 | rx_crc_mask), &framer->regs->cpri_hdlccfg3);

}

static void cpri_hdlc_init_bd_regs(struct cpri_framer *framer)
{
	struct cpri_framer_regs *regs = framer->regs;
	struct cpri_hdlc_tx_bd *tx_bd = &framer->hdlc_priv.tx_bd;
	struct cpri_hdlc_rx_bd *rx_bd = &framer->hdlc_priv.rx_bd;

	/* Rx buffer size */
	cpri_reg_set_val(&regs->cpri_rhdlcbufsize,
		CPRI_HDLC_RX_BUF_SZ_MASK, CPRI_HDLC_DEF_RX_BUF_SIZE - 1);

	/* Ring size */
	cpri_reg_set_val(&regs->cpri_thdlcbdringsize,
		CPRI_HDLC_TX_BD_RING_SZ_MASK, CPRI_HDLC_DEF_TX_RING_CNT);

	cpri_reg_set_val(&regs->cpri_rhdlcbdringsize,
		CPRI_HDLC_RX_BD_RING_SZ_MASK, CPRI_HDLC_DEF_RX_RING_CNT);

	/* Ring Base address */
	cpri_reg_set_val(&regs->cpri_thdlcbdringbaddr,
		CPRI_HDLC_TX_BD_RING_BASE_MASK, tx_bd->tx_bd_dma_base);

	cpri_reg_clear(&regs->cpri_thdlcbdringbaddrmsb,
		CPRI_HDLC_TX_BD_RING_BASE_MSB_MASK);

	cpri_reg_set_val(&regs->cpri_rhdlcbdringbaddr,
		CPRI_HDLC_RX_BD_RING_BASE_MASK,
		rx_bd->rx_bd_dma_base);

	cpri_reg_clear(&regs->cpri_rhdlcbdringbaddrmsb,
		CPRI_HDLC_RX_BD_RING_BASE_MSB_MASK);

	cpri_reg_set(&framer->regs->cpri_rhdlcctrl,
			CPRI_HDLC_RX_CTRL_DISCARD_MASK);

	cpri_write(0x100, &framer->regs->cpri_rhdlcwriteptr);
	cpri_reg_clear(&framer->regs->cpri_thdlcwriteptr, MASK_ALL);
	cpri_hdlc_set_config_reg(framer);

	cpri_reg_set(&framer->regs->cpri_rcr,
			CPRI_HDLC_RX_ENABLE_MASK);

	cpri_reg_set(&framer->regs->cpri_tcr,
			CPRI_HDLC_TX_ENABLE_MASK);

	/* Start the HDLC interrupts */
	cpri_reg_set(&framer->regs->cpri_rctrltiminginten,
			(CONTROL_INT_LEVEL_MASK | HDLC_EVENT_EN_MASK));

	cpri_reg_set(&framer->regs->cpri_tctrltiminginten,
			(CONTROL_INT_LEVEL_MASK | HDLC_EVENT_EN_MASK));
}

static void cpri_hdlc_clear_bd_regs(struct cpri_framer *framer)
{

	cpri_reg_clear(&framer->regs->cpri_rcr,
			CPRI_HDLC_RX_ENABLE_MASK);

	cpri_reg_clear(&framer->regs->cpri_tcr,
			CPRI_HDLC_TX_ENABLE_MASK);

	cpri_reg_set(&framer->regs->cpri_rhdlcctrl,
			CPRI_HDLC_RX_CTRL_DISCARD_MASK);
	/* Rx buffer size */
	cpri_reg_clear(&framer->regs->cpri_rhdlcbufsize,
			CPRI_HDLC_RX_BUF_SZ_MASK);

	/* Ring size */
	cpri_reg_clear(&framer->regs->cpri_thdlcbdringsize,
			CPRI_HDLC_TX_BD_RING_SZ_MASK);

	cpri_reg_clear(&framer->regs->cpri_rhdlcbdringsize,
			CPRI_HDLC_RX_BD_RING_SZ_MASK);

	/* Ring Base address */
	cpri_reg_clear(&framer->regs->cpri_thdlcbdringbaddr,
			CPRI_HDLC_TX_BD_RING_BASE_MASK);

	cpri_reg_clear(&framer->regs->cpri_thdlcbdringbaddrmsb,
			CPRI_HDLC_TX_BD_RING_BASE_MSB_MASK);

	cpri_reg_clear(&framer->regs->cpri_rhdlcbdringbaddr,
			CPRI_HDLC_RX_BD_RING_BASE_MASK);

	cpri_reg_clear(&framer->regs->cpri_rhdlcbdringbaddrmsb,
			CPRI_HDLC_RX_BD_RING_BASE_MSB_MASK);

	/* Reset write ptr */
	cpri_reg_clear(&framer->regs->cpri_rhdlcwriteptr, MASK_ALL);
	cpri_reg_clear(&framer->regs->cpri_thdlcwriteptr, MASK_ALL);

	cpri_reg_clear(&framer->regs->cpri_rctrltiminginten,
			HDLC_EVENT_EN_MASK);
	cpri_reg_clear(&framer->regs->cpri_tctrltiminginten,
			HDLC_EVENT_EN_MASK);
}

static void hdlc_init_rxbdp(struct cpri_framer *framer,
				struct cpri_hdlc_rx_bd *rx_bd,
				dma_addr_t buf, unsigned int bd_index, int init)
{
	struct cpri_hdlc_bd_entity rxbde_le;
	unsigned int bd_nxtindex;
	struct cpri_hdlc_bd_entity *bdp = rx_bd->rx_bd_base + bd_index;

	rxbde_le.buf_ptr = buf;
	rxbde_le.lstatus = BD_LFLAG_FSHIFT(CPRI_HDLC_BD_RX_EMPTY);
	CPRI_HDLC_BD_TO_BE(bdp, &rxbde_le); /* b-endian */
	dmb();

	if (!init) {
		bd_nxtindex = CPRI_HDLC_NEXT_INDX(bd_index, CPRI_HDLC_DEF_RX_RING_CNT);
		if (bd_nxtindex == 0) {
			if (!(cpri_read(&framer->regs->cpri_rhdlcwriteptr) & 0x100))
				bd_nxtindex = 0X100;
		}
		cpri_write(bd_nxtindex, &framer->regs->cpri_rhdlcwriteptr);
	}
}

static int __cpri_hdlc_init(struct cpri_framer *framer)
{
	int k, q, j, l, ret = 0;
	void *vaddr;
	dma_addr_t addr;
	unsigned int dma_alloc_size;
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	struct cpri_hdlc_tx_bd *tx_bd = &priv->tx_bd;
	struct cpri_hdlc_rx_bd *rx_bd = &priv->rx_bd;

	if (atomic_inc_return(&priv->users) > 1)
		return 0;
	memset(tx_bd, 0, sizeof(struct cpri_hdlc_tx_bd));
	memset(rx_bd, 0, sizeof(struct cpri_hdlc_rx_bd));

	priv->rx_buffer_size = CPRI_HDLC_DEF_RX_BUF_SIZE + CPRI_HDLC_DEF_RX_BUF_RESERVED;
	tx_bd->tx_bd_ring_cnt = CPRI_HDLC_DEF_TX_RING_CNT;
	rx_bd->rx_bd_ring_cnt = CPRI_HDLC_DEF_RX_RING_CNT;
	rx_bd->rx_buf_cnt = CPRI_HDLC_DEF_RX_BUF_CNT;

	dma_alloc_size = ((sizeof(struct cpri_hdlc_bd_entity) *
				rx_bd->rx_bd_ring_cnt) +
				(sizeof(struct cpri_hdlc_bd_entity) *
				tx_bd->tx_bd_ring_cnt));

	/* Allocate memory for the buffer descriptors */
	vaddr = dma_alloc_coherent(&framer->pdev->dev,
			dma_alloc_size,
			&addr, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;

	tx_bd->txbuf = kmalloc(sizeof(u32 *) *
				tx_bd->tx_bd_ring_cnt, GFP_KERNEL);
	if (!tx_bd->txbuf) {
		ret = -ENOMEM;
		goto fail0;
	}

	memset(vaddr, 0, dma_alloc_size);
	priv->bd_vaddr = vaddr;
	priv->bd_paddr = addr;

	/* Initialize Tx BD */
	tx_bd->tx_bd_base = vaddr;
	tx_bd->tx_bd_dma_base = addr;
	tx_bd->num_txbdfree = tx_bd->tx_bd_ring_cnt;

	/* Initialize Rx BD */
	addr  += sizeof(struct cpri_hdlc_bd_entity) * tx_bd->tx_bd_ring_cnt;
	vaddr += sizeof(struct cpri_hdlc_bd_entity) * tx_bd->tx_bd_ring_cnt;

	rx_bd->rx_bd_base = vaddr;
	rx_bd->rx_bd_dma_base =	addr;

	rx_bd->rxbuf = kmalloc(sizeof(u32 *) *
				rx_bd->rx_bd_ring_cnt, GFP_KERNEL);
	if (!rx_bd->rxbuf) {
		ret = -ENOMEM;
		goto fail1;
	}

	rx_bd->rxbuf_paddr = kmalloc(sizeof(dma_addr_t) *
			rx_bd->rx_bd_ring_cnt, GFP_KERNEL);
	if (!rx_bd->rxbuf_paddr) {
		ret = -ENOMEM;
		goto fail2;
	}

	for (q = 0; q < rx_bd->rx_bd_ring_cnt; q++) {
		rx_bd->rxbuf[q] = kmalloc(priv->rx_buffer_size, GFP_KERNEL);
		if (!rx_bd->rxbuf[q]) {
			ret = -ENOMEM;
			goto fail3;
		}
	}

	for (k = 0; k < rx_bd->rx_bd_ring_cnt; k++) {
		rx_bd->rxbuf_paddr[k] =
			dma_map_single(&framer->pdev->dev,
			rx_bd->rxbuf[k],
			CPRI_HDLC_DEF_RX_BUF_SIZE, DMA_FROM_DEVICE);

		if (dma_mapping_error(&framer->pdev->dev, rx_bd->rxbuf_paddr[k])) {
			ret = -EFAULT;
			goto fail4;
		}
		hdlc_init_rxbdp(framer, rx_bd, rx_bd->rxbuf_paddr[k], k, 1);
	}

	rx_bd->rxbuf_valid = kmalloc(sizeof(u32 *) *
				rx_bd->rx_buf_cnt, GFP_KERNEL);
	if (!rx_bd->rxbuf_valid) {
		ret = -ENOMEM;
		goto fail4;
	}

	for (l = 0; l < rx_bd->rx_buf_cnt; l++) {
		rx_bd->rxbuf_valid[l] = kmalloc(priv->rx_buffer_size, GFP_KERNEL);
		if (!rx_bd->rxbuf_valid[l]) {
			ret = -ENOMEM;
			goto fail5;
		}
	}
	init_waitqueue_head(&priv->rx_queue);
	init_waitqueue_head(&priv->tx_queue);
	init_waitqueue_head(&priv->tx_release_queue);
	cpri_hdlc_init_bd_regs(framer);
	return 0;

fail5:
	for (j = 0; j < l; j++)
		kfree(rx_bd->rxbuf_valid[j]);
	kfree(rx_bd->rxbuf_valid);
fail4:
	for (j = 0; j < k; j++)
		dma_unmap_single(&framer->pdev->dev,
				rx_bd->rxbuf_paddr[j],
				CPRI_HDLC_DEF_RX_BUF_SIZE, DMA_FROM_DEVICE);
fail3:
	for (j = 0; j < q; j++)
		kfree(rx_bd->rxbuf[j]);
	kfree(rx_bd->rxbuf_paddr);
fail2:
	kfree(rx_bd->rxbuf);
fail1:
	kfree(tx_bd->txbuf);
fail0:
	dma_free_coherent(&framer->pdev->dev,
			dma_alloc_size,
			priv->bd_vaddr, priv->bd_paddr);
	return ret;
}

int cpri_hdlc_init(struct cpri_framer *framer)
{
	int ret;

	spin_lock(&framer->hdlc_priv.tx_lock);
	spin_lock(&framer->hdlc_priv.rx_lock);
	ret = __cpri_hdlc_init(framer);
	spin_unlock(&framer->hdlc_priv.rx_lock);
	spin_unlock(&framer->hdlc_priv.tx_lock);

	return ret;
}

static void __cpri_hdlc_free_buffs(struct cpri_framer *framer)
{
	unsigned int dma_alloc_size, i;
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	struct cpri_hdlc_tx_bd *tx_bd = &priv->tx_bd;
	struct cpri_hdlc_rx_bd *rx_bd = &priv->rx_bd;

	/* Free RX BD */
	for (i = 0; i < rx_bd->rx_bd_ring_cnt; i++) {
		dma_unmap_single(&framer->pdev->dev,
			rx_bd->rxbuf_paddr[i],
			priv->rx_buffer_size, DMA_FROM_DEVICE);
		kfree(rx_bd->rxbuf[i]);
	}

	for (i = 0; i < rx_bd->rx_buf_cnt; i++)
		kfree(rx_bd->rxbuf_valid[i]);

	kfree(rx_bd->rxbuf);
	kfree(rx_bd->rxbuf_valid);
	kfree(rx_bd->rxbuf_paddr);

	dma_alloc_size = (sizeof(struct cpri_hdlc_bd_entity) *
			rx_bd->rx_bd_ring_cnt) +
			(sizeof(struct cpri_hdlc_bd_entity) *
			tx_bd->tx_bd_ring_cnt);

	dma_free_coherent(&framer->pdev->dev,
		dma_alloc_size, priv->bd_vaddr, priv->bd_paddr);

	kfree(tx_bd->txbuf);
	memset(tx_bd, 0, sizeof(*tx_bd));
	memset(rx_bd, 0, sizeof(*rx_bd));
	priv->bd_paddr = 0;
	priv->bd_vaddr = 0;
}

void __cpri_hdlc_release(struct cpri_framer *framer)
{
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	struct cpri_hdlc_bd_entity *txbde, txbde_le;
	struct cpri_hdlc_tx_bd *tx_bd = &priv->tx_bd;
	int len;

	if (!atomic_dec_and_test(&priv->users))
		return;

	wait_event_interruptible_timeout(priv->tx_release_queue,
			(tx_bd->num_txbdfree == tx_bd->tx_bd_ring_cnt), HZ);

	spin_lock(&framer->hdlc_priv.tx_lock);
	spin_lock(&framer->hdlc_priv.rx_lock);
	cpri_hdlc_clear_bd_regs(framer);
	while (tx_bd->dirty_index != tx_bd->current_index) {
		txbde = tx_bd->dirty_index + tx_bd->tx_bd_base;
		CPRI_HDLC_BD_TO_LE(&txbde_le, txbde);
		len = BD_LSTATUS_LSHIFT(txbde_le.lstatus & BD_LENGTH_MASK);
		dma_unmap_single(&framer->pdev->dev, txbde_le.buf_ptr,
			len, DMA_TO_DEVICE);
		kfree(tx_bd->txbuf[tx_bd->dirty_index]);
		tx_bd->dirty_index =
			CPRI_HDLC_NEXT_INDX(tx_bd->dirty_index,
				CPRI_HDLC_DEF_TX_RING_CNT);
	}
	__cpri_hdlc_free_buffs(framer);
	spin_unlock(&framer->hdlc_priv.rx_lock);
	spin_unlock(&framer->hdlc_priv.tx_lock);
}

static int cpri_hdlc_xmit(struct cpri_framer *framer, char *data, u32 len, int nonblock)
{
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	struct cpri_hdlc_tx_bd *tx_bd = &priv->tx_bd;
	struct cpri_hdlc_bd_entity *txbde;
	struct cpri_hdlc_bd_entity txbde_le;
	struct cpri_reg_data reg_data[1];
	u32 val;
	int ret;

	/* check if there is space to queue this packet */
	spin_lock(&framer->hdlc_priv.tx_lock);
	while (tx_bd->num_txbdfree == 0) {
		spin_unlock(&framer->hdlc_priv.tx_lock);
		if (nonblock) {
			priv->stats.tx_dropped++;
			return -EAGAIN;
		}
		ret = wait_event_interruptible_timeout(priv->tx_queue,
				tx_bd->num_txbdfree, priv->timeout);
		if (ret == 0 || ret < 0)
			priv->stats.tx_dropped++;
		if (ret == 0)
			return -EAGAIN;
		else if (ret < 0)
			return ret;
		spin_lock(&framer->hdlc_priv.tx_lock);
	}

	(tx_bd->num_txbdfree)--;
	txbde_le.lstatus = 0;
	txbde_le.lstatus |= (len << 8);
	txbde_le.buf_ptr = dma_map_single(&framer->pdev->dev, data,
		len, DMA_TO_DEVICE);

	if (dma_mapping_error(&framer->pdev->dev, txbde_le.buf_ptr)) {
		priv->stats.tx_dropped++;
		dev_err(&framer->dev, "CPRI HDLC TX dma_mapping_error\n");
		spin_unlock(&framer->hdlc_priv.tx_lock);
		return -EAGAIN;
	}
	txbde_le.lstatus |= 0x80000000;
	txbde = tx_bd->tx_bd_base + tx_bd->current_index;

	CPRI_HDLC_BD_TO_BE(txbde, &txbde_le); /* b-endian */
	tx_bd->txbuf[tx_bd->current_index] = data;

	tx_bd->current_index =
		CPRI_HDLC_NEXT_INDX(tx_bd->current_index, CPRI_HDLC_DEF_TX_RING_CNT);

	reg_data[0].val = tx_bd->current_index;
	reg_data[0].mask = CPRI_HDLC_TX_BD_R_PTR_MASK;

	if (reg_data[0].val == 0) {
		/* also set the wrap bit */
		reg_data[0].mask = CPRI_HDLC_TX_BD_W_PTR_MASK;
		val = cpri_read(&framer->regs->cpri_thdlcwriteptr);
		if ((val  >> 8) & 0x1)
			reg_data[0].val &= ~CPRI_HDLC_TX_BD_W_PTR_WRAP_MASK;
		else
			reg_data[0].val |= CPRI_HDLC_TX_BD_W_PTR_WRAP_MASK;
	}

	cpri_reg_vset_val(&framer->regs->cpri_thdlcwriteptr, reg_data);
	spin_unlock(&framer->hdlc_priv.tx_lock);

	return len;
}

ssize_t cpri_hdlc_read(struct file *file,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct cpri_framer *framer = file->private_data;
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	struct cpri_hdlc_rx_bd *rx_bd = &priv->rx_bd;
	u32 lstatus, pkt_len;
	u8 read_ptr;
	int ret;

	if (count < CPRI_HDLC_DEF_RX_BUF_RESERVED)
		return -EINVAL;

	spin_lock(&framer->hdlc_priv.rx_lock);
	while (rx_bd->rx_avail_cnt == 0) {
		spin_unlock(&framer->hdlc_priv.rx_lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible_timeout(priv->rx_queue,
				rx_bd->rx_avail_cnt, priv->timeout);
		if (ret == 0)
			return -EAGAIN;
		else if (ret < 0)
			return ret;

		spin_lock(&framer->hdlc_priv.rx_lock);
	}
	read_ptr = rx_bd->rx_read_ptr;
	memcpy(&lstatus, &rx_bd->rxbuf_valid[read_ptr][0],
			CPRI_HDLC_DEF_RX_BUF_RESERVED);
	pkt_len = (lstatus >> 8) & 0xFFFF;
	pkt_len = (pkt_len  < (count - CPRI_HDLC_DEF_RX_BUF_RESERVED)) ?
				pkt_len : (count - CPRI_HDLC_DEF_RX_BUF_RESERVED);

	rx_bd->rx_read_ptr = CPRI_HDLC_NEXT_INDX(read_ptr,
		rx_bd->rx_buf_cnt);
	rx_bd->rx_avail_cnt--;

	if (copy_to_user(&buf[0], &rx_bd->rxbuf_valid[read_ptr][CPRI_HDLC_DEF_RX_BUF_SIZE],
			CPRI_HDLC_DEF_RX_BUF_RESERVED)) {
		spin_unlock(&framer->hdlc_priv.rx_lock);
		return -EFAULT;
	}

	if (pkt_len > 0) {
		if (copy_to_user(&buf[CPRI_HDLC_DEF_RX_BUF_RESERVED],
					&rx_bd->rxbuf_valid[read_ptr][0],
					pkt_len)) {
			spin_unlock(&framer->hdlc_priv.rx_lock);
			return -EFAULT;
		}
	}

	spin_unlock(&framer->hdlc_priv.rx_lock);
	return pkt_len + CPRI_HDLC_DEF_RX_BUF_RESERVED;
}

ssize_t cpri_hdlc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cpri_framer *framer = file->private_data;
	char *hdlc_data;
	int ret = -EAGAIN;
	int nonblock = (file->f_flags & O_NONBLOCK);

	if (count == 0 || count > 0xFFFF)
		return -EINVAL;

	hdlc_data = kmalloc(count, GFP_KERNEL);
	if (!hdlc_data) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(hdlc_data, buf, count) != 0) {
		ret = -EFAULT;
		kfree(hdlc_data);
		goto out;
	}

	ret = cpri_hdlc_xmit(framer, hdlc_data, count, nonblock);
	if (ret < 0)
		kfree(hdlc_data);

out:
	return ret;
}

long cpri_hdlc_ioctl(struct file *fp, unsigned int cmd,
			unsigned long arg)
{
	struct cpri_framer *framer = fp->private_data;
	void __user *ioargp = (void __user *)arg;

	switch (cmd) {
	case CPRI_HDLC_READ_STATUS:
		if (copy_to_user(ioargp,
			&framer->hdlc_priv.stats,
			sizeof(struct cpri_hdlc_stats)))
			return -EFAULT;
		break;

	case CPRI_HDLC_CLEAR_STATUS:
		spin_lock(&framer->hdlc_priv.tx_lock);
		spin_lock(&framer->hdlc_priv.rx_lock);
		memset(&framer->hdlc_priv.stats, 0,
			sizeof(struct cpri_hdlc_stats));
		spin_unlock(&framer->hdlc_priv.rx_lock);
		spin_unlock(&framer->hdlc_priv.tx_lock);
		break;

	case CPRI_HDLC_SET_CONFIG:
		if (copy_from_user(&framer->hdlc_priv.hdlc_config, ioargp,
				sizeof(struct cpri_hdlc_config)) != 0)
			return -EFAULT;
		if (framer->hdlc_priv.hdlc_config.timeout < 0)
			framer->hdlc_priv.timeout = MAX_SCHEDULE_TIMEOUT;
		else
			framer->hdlc_priv.timeout =
				framer->hdlc_priv.hdlc_config.timeout * HZ;
		cpri_hdlc_set_config_reg(framer);
		break;

	default:
		return -ENOIOCTLCMD;

	}
	return 0;
}

/* Interrupt handlers */
/* These will be called from CPRI code, in IRQ context */
void cpri_hdlc_tx_cleanup(struct cpri_framer *framer)
{
	int len;
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	struct cpri_hdlc_bd_entity *txbde, txbde_le;
	struct cpri_hdlc_tx_bd *tx_bd = &priv->tx_bd;

	spin_lock(&priv->tx_lock);
	while (tx_bd->dirty_index != tx_bd->current_index) {
		txbde = tx_bd->dirty_index + tx_bd->tx_bd_base;
		CPRI_HDLC_BD_TO_LE(&txbde_le, txbde);/* passing b-endian */

		if (txbde_le.lstatus & 0x80000000)
			/* Tx DMA will clear this bit after Tx complete */
			break;
		len = (txbde_le.lstatus >> 8) & 0xFFFF;
		dma_unmap_single(&framer->pdev->dev, txbde_le.buf_ptr,
			len, DMA_TO_DEVICE);
		kfree(tx_bd->txbuf[tx_bd->dirty_index]);

		/* update transmit stats */
		priv->stats.tx_packets++;
		priv->stats.tx_bytes += len;
		txbde->buf_ptr = 0;
		txbde->lstatus = 0;
		tx_bd->dirty_index =
			CPRI_HDLC_NEXT_INDX(tx_bd->dirty_index, CPRI_HDLC_DEF_TX_RING_CNT);
		(tx_bd->num_txbdfree)++;
	}
	spin_unlock(&priv->tx_lock);
	wake_up_interruptible(&priv->tx_queue);
	if (tx_bd->num_txbdfree == tx_bd->tx_bd_ring_cnt)
		wake_up_interruptible(&priv->tx_release_queue);
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tctrltiminginten,
			HDLC_EVENT_EN_MASK, HDLC_EVENT_EN_MASK);
}

static void cpri_hdlc_rx_pkt_error(struct cpri_framer *framer,
				 u32 lstatus)
{
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	u32 status;

	priv->stats.rx_abort_err++;
	if (BD_LSTATUS_SSHIFT(lstatus) & CPRI_HDLC_BD_RX_CRC)
		priv->stats.rx_crc_err++;
	if (BD_LSTATUS_SSHIFT(lstatus) & CPRI_HDLC_BD_RX_PLE)
		priv->stats.rx_ple_err++;
	if (BD_LSTATUS_SSHIFT(lstatus) & CPRI_HDLC_BD_RX_BOF)
		priv->stats.rx_overflow_err++;
	status = cpri_read(&framer->regs->cpri_rhdlcstatus);
	if (status & CPRI_HDLC_MII_ERR)
		priv->stats.rx_mii_err++;
}

static int valid_rx_packet(const struct cpri_hdlc_priv *priv, u32 pkt_len)
{
	if (priv->hdlc_config.enable_hdlc_rx_sw_len_chk) {
		if (priv->hdlc_config.hdlc_min_rx  < 0)
			return 1;
		else if (pkt_len < priv->hdlc_config.hdlc_min_rx)
			return 0;
	}
	return 1;
}

int cpri_hdlc_handle_rx(struct cpri_framer *framer)
{
	int howmany;
	struct cpri_hdlc_priv *priv = &framer->hdlc_priv;
	struct cpri_hdlc_rx_bd *rx_bd = &priv->rx_bd;
	struct cpri_hdlc_bd_entity *rxbde;
	struct cpri_hdlc_bd_entity rxbde_le;
	u32 pkt_len;

	spin_lock(&framer->hdlc_priv.rx_lock);
	for (howmany = 0; howmany < rx_bd->rx_bd_ring_cnt; howmany++) {

		rxbde = rx_bd->rx_bd_base + rx_bd->rx_bd_index;
		rmb();
		CPRI_HDLC_BD_TO_LE(&rxbde_le, rxbde); /* b-endian */

		if (rxbde_le.lstatus & 0x80000000)
			break;

		/* Copy the receive status into the buffer */
		memcpy(&rx_bd->rxbuf[rx_bd->rx_bd_index][CPRI_HDLC_DEF_RX_BUF_SIZE],
				&rxbde_le.lstatus, CPRI_HDLC_DEF_RX_BUF_RESERVED);

		if ((rxbde_le.lstatus >> 24) &
				CPRI_HDLC_BD_RX_ABORT) {
			cpri_hdlc_rx_pkt_error(framer, rxbde_le.lstatus);
		} else {
			pkt_len = (rxbde_le.lstatus & 0xFFFF00) >> 8;
			if (valid_rx_packet(priv, pkt_len)) {
				pkt_len = (pkt_len < priv->rx_buffer_size)
					? pkt_len : priv->rx_buffer_size;

				dma_sync_single_for_cpu(&framer->pdev->dev,
					rx_bd->rxbuf_paddr[rx_bd->rx_bd_index],
					pkt_len, DMA_FROM_DEVICE);

				memcpy(rx_bd->rxbuf_valid[rx_bd->rx_write_ptr],
					rx_bd->rxbuf[rx_bd->rx_bd_index],
					priv->rx_buffer_size);

				priv->stats.rx_packets++;
				priv->stats.rx_bytes += pkt_len;

				if (rx_bd->rx_avail_cnt < rx_bd->rx_buf_cnt)
					rx_bd->rx_avail_cnt++;
				else {
					priv->stats.rx_wrapped_err++;
					rx_bd->rx_read_ptr = rx_bd->rx_write_ptr;
				}
				rx_bd->rx_write_ptr = CPRI_HDLC_NEXT_INDX(rx_bd->rx_write_ptr,
					rx_bd->rx_buf_cnt);
			}
		}

		hdlc_init_rxbdp(framer, rx_bd,
			rx_bd->rxbuf_paddr[rx_bd->rx_bd_index],
			rx_bd->rx_bd_index, 0);

		rx_bd->rx_bd_index = CPRI_HDLC_NEXT_INDX(rx_bd->rx_bd_index,
			rx_bd->rx_bd_ring_cnt);

	}
	spin_unlock(&framer->hdlc_priv.rx_lock);
	wake_up_interruptible(&priv->rx_queue);
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_rctrltiminginten,
			HDLC_EVENT_EN_MASK, HDLC_EVENT_EN_MASK);

	return howmany;
}

int cpri_hdlc_open(struct inode *inode, struct file *fp)
{
	struct cpri_framer *framer = NULL;

	framer = container_of(inode->i_cdev, struct cpri_framer, hdlc_cdev);
	if (framer != NULL)
		fp->private_data = framer;
	else
		return -ENODEV;

	if (!test_bit(CPRI_HDLC_BITPOS,
		&framer->cpri_state))
		return -EAGAIN;
	return cpri_hdlc_init(framer);

}

int cpri_hdlc_release(struct inode *inode, struct file *fp)
{
	struct cpri_framer *framer = fp->private_data;

	if (!framer)
		return -ENODEV;

	__cpri_hdlc_release(framer);
	return 0;
}

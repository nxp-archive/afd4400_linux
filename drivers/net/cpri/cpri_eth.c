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
static int cpri_eth_xmit(struct sk_buff *newskb, struct net_device *ndev)
{
	unsigned long flags = 0;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_framer *framer = priv->framer;
	struct cpri_eth_tx_bd *tx_bd = priv->tx_bd;
	struct cpri_eth_bd_entity *txbde;
	struct cpri_eth_bd_entity txbde_le;
	struct cpri_eth_bd_entity *base = tx_bd->tx_bd_base;
	struct device *dev = priv->framer->cpri_dev->dev;
	struct cpri_reg_data reg_data[1];
	int loop;
	u32 val;

	val = CPRI_ETH_RXBUF_ALIGNMENT -
		(((unsigned long) newskb->data) & (CPRI_ETH_RXBUF_ALIGNMENT - 1));
	 if (pskb_expand_head(newskb, val, val, GFP_KERNEL)) {
		dev_err(dev, "cpri reallocation failed!!\n");
		return -EFAULT;
	 }

	/* check if there is space to queue this packet */
	if (!tx_bd->num_txbdfree) {
		/* no space, stop the queue */
		netif_stop_queue(ndev);
		priv->stats.tx_fifo_errors++;
		priv->stats.tx_dropped++;
		dev_err(dev, "cpri no space to queue tx frame!!\n");
		return NETDEV_TX_BUSY;
	}


	/* update the descriptor */

	txbde_le.lstatus = 0;
	txbde_le.lstatus |= BD_LFLAG_LSHIFT(skb_headlen(newskb));
	txbde_le.buf_ptr = dma_map_single(&priv->ofdev->dev, newskb->data,
			skb_headlen(newskb), DMA_TO_DEVICE);
	txbde_le.lstatus |= BD_LFLAG_FSHIFT(CPRI_ETH_BD_TX_READY);

	tx_bd->tx_skbuff[tx_bd->skb_curtx] = newskb;
	/* CPRI IP expects the buffer descriptors in big endian
	 * format; section 22.6
	 */
	txbde = tx_bd->tx_bd_current;
	dev_dbg(dev, "transmitted bd: lstatus: 0x%x, buf_ptr: 0x%x\n",
			txbde_le.lstatus, txbde_le.buf_ptr);
	for (loop = 0; loop < skb_headlen(newskb);) {
		dev_dbg(dev, "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
		*((newskb->data + loop)), *((newskb->data + loop + 1)),
		*((newskb->data + loop + 2)), *((newskb->data + loop + 3)),
		*((newskb->data + loop + 4)), *((newskb->data + loop + 5)),
		*((newskb->data + loop + 6)), *((newskb->data + loop + 7)));
		loop += 8;
	}

	spin_lock_irqsave(&tx_bd->txlock, flags);
	cpri_eth_bd_to_be(txbde, &txbde_le); /* b-endian */
	/* update transmit stats */
	cpri_eth_stats_incr_val(&priv->stats.tx_bytes, newskb->len);
	priv->stats.tx_packets++;

	netdev_sent_queue(ndev, newskb->len);
	tx_bd->skb_curtx =
		CPRI_ETH_NEXT_INDX(tx_bd->skb_curtx, CPRI_ETH_DEF_TX_RING_SIZE);
	tx_bd->tx_bd_current =
		cpri_eth_next_bde(txbde, base, tx_bd->tx_bd_ring_size);

	reg_data[0].val = tx_bd->skb_curtx;
	reg_data[0].mask = CPRI_ETH_TX_BD_R_PTR_MASK;

	if (reg_data[0].val == 0) {
		/* also set the wrap bit */
		reg_data[0].mask = CPRI_ETH_RX_BD_W_PTR_MASK;
		val = cpri_read(&framer->regs->cpri_tethwriteptr);
		if ((val  >> 8) & 0x1)
			reg_data[0].val &= ~CPRI_ETH_TX_BD_W_PTR_WRAP_MASK;
		else
			reg_data[0].val |= CPRI_ETH_TX_BD_W_PTR_WRAP_MASK;
	}

	(tx_bd->num_txbdfree)--;
	if (!tx_bd->num_txbdfree) {
		netif_stop_queue(ndev);
		priv->stats.tx_fifo_errors++;
		dev_err(dev, "cpri no space to queue!!\n");
	}


	cpri_reg_vset_val(&framer->regs->cpri_tethwriteptr, reg_data);
	spin_unlock_irqrestore(&tx_bd->txlock, flags);

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

	priv->flags |= flags;
	/* TBD : stripping/MAC-Addr/MAC-Check/Multi cast */
	if (flags & CPRI_ETH_HW_CRC_STRIP)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_HW_CRC_STRIP, CPRI_ETH_HW_CRC_STRIP_MASK);
#if 0
	if (flags & CPRI_ETH_MAC_FAIL_PASS)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_MAC_FAIL_PASS, CPRI_ETH_MAC_FAIL_PASS_MASK);
	if (flags & CPRI_ETH_MAC_CHECK)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_MAC_CHECK, CPRI_ETH_MAC_CHECK_MASK);
	if (flags & CPRI_ETH_MCAST_FLT)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_MCAST_FLT, CPRI_ETH_MCAST_FLT_MASK);
#endif

	if (flags & CPRI_ETH_LONG_FRAME)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_LONG_FRAME, CPRI_ETH_LONG_FRAME_MASK);

	if (flags & CPRI_ETH_RX_PREAMBLE_ABORT)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_RX_PREAMBLE_ABORT, CPRI_ETH_RX_PR_ABORT_MASK);
	if (flags & CPRI_ETH_BCAST)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_BCAST, CPRI_ETH_BCAST_MASK);

	if (flags & CPRI_ETH_LEN_CHECK)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_LEN_CHECK, CPRI_ETH_LEN_CHECK_MASK);


	if (flags & CPRI_ETH_TRIG_TX_ABORT)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_TX_ABORT, CPRI_ETH_TRIG_TX_ABORT_MASK);

	if (flags & CPRI_ETH_TRIG_RX_ABORT)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_RX_ABORT, CPRI_ETH_TRIG_RX_ABORT_MASK);

	if (flags & CPRI_ETH_TRIG_TX_READY)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_TX_READY, CPRI_ETH_TRIG_TX_RDY_MASK);


	if (flags & CPRI_ETH_TRIG_RX_READY)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_RX_READY, CPRI_ETH_TRIG_RX_RDY_MASK);

	if (flags & CPRI_ETH_TRIG_TX)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_TX, CPRI_ETH_TRIG_TX_MASK);

	if (flags & CPRI_ETH_GLOBAL_TRIG)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_GLOBAL_TRIG, CPRI_ETH_TRIG_RX_MASK);

	if (flags & CPRI_ETH_TRIG_RX)
		cpri_eth_config_update(ndev, &framer->regs->cpri_ethcfg1, flags,
			CPRI_ETH_TRIG_RX, CPRI_ETH_GLOBAL_TRIG_MASK);


		/* For TX, We support on HW generated FCS for now! */
	if (flags & CPRI_ETH_HW_CRC_EN)
		cpri_eth_config_update(ndev,
			&framer->regs->cpri_ethcfg2,
			flags, CPRI_ETH_HW_CRC_EN,
			CPRI_ETH_HW_CRC_EN_MASK);

	if (flags & CPRI_ETH_HW_CRC_CHECK)
		cpri_eth_config_update(ndev,
				&framer->regs->cpri_ethcfg3, flags,
				CPRI_ETH_HW_CRC_CHECK,
				CPRI_ETH_HW_CRC_CHECK_MASK);

#if 0	/* need to update to support foward interface */
	if (flags & CPRI_ETH_STORE_FWD)
		cpri_eth_config_update(ndev,
				&framer->regs->cpri_ethcfg3,
				flags, CPRI_ETH_STORE_FWD,
				CPRI_ETH_STORE_FWD_MASK);
#endif


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

	/* crc stripping need to be debugged with hardware */
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
	u32 val;

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
		cpri_eth_config(ndev, priv->flags & CPRI_ETH_TRIG_TX_READY);
		cpri_eth_config(ndev, priv->flags & CPRI_ETH_TRIG_RX_READY);
		cpri_eth_config(ndev, priv->flags & CPRI_ETH_TRIG_RX);
		cpri_eth_config(ndev, priv->flags & CPRI_ETH_TRIG_TX);
		val = (CPRI_ETH_TRIG_TX_ABORT_MASK | CPRI_ETH_TRIG_TX_RDY_MASK |
			CPRI_ETH_TRIG_RX_RDY_MASK | CPRI_ETH_TRIG_TX_MASK |
			CPRI_ETH_TRIG_RX_MASK);
		cpri_reg_write(&framer->regs_lock,
				&framer->regs->cpri_ethcfg1,
				val, val);
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

	priv->stats.tx_fifo_errors++;

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
	struct device *dev = priv->framer->cpri_dev->dev;
	unsigned long rx_errors = 0;


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
	dev_dbg(dev, "rx_dma_overrun: %ld\n", priv->extra_stats.rx_dma_overrun);
	dev_dbg(dev, "rx_overrun: %ld\n", priv->extra_stats.rx_overrun);
	dev_dbg(dev, "rx_underrun: %ld\n", priv->extra_stats.rx_underrun);
	dev_dbg(dev, "mii_error: %ld\n", priv->extra_stats.mii_error);
	dev_dbg(dev, "small_pkt: %ld\n", priv->extra_stats.small_pkt);
	dev_dbg(dev, "dmac_mismatch: %ld\n", priv->extra_stats.dmac_mismatch);
	dev_dbg(dev, "crc_error: %ld\n", priv->extra_stats.crc_error);
	dev_dbg(dev, "cpri_rx_error: %ld\n", priv->extra_stats.cpri_rx_error);
	dev_dbg(dev, "long_frame: %ld\n", priv->extra_stats.long_frame);
	dev_dbg(dev, "short_frame: %ld\n", priv->extra_stats.short_frame);

	ndev->stats.rx_errors = rx_errors;
	ndev->stats.tx_errors = priv->extra_stats.tx_underrun;
	dev_dbg(dev, "tx_underrun: %ld\n", priv->extra_stats.tx_underrun);


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
	struct cpri_framer *framer = priv->framer;
	struct cpri_eth_bd_entity *txbde, *base, txbde_le;
	struct cpri_eth_tx_bd *tx_bd = priv->tx_bd;
	int tx_ring_size = tx_bd->tx_bd_ring_size;
	struct net_device *ndev = priv->ndev;

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

		dma_unmap_single(&priv->ofdev->dev, txbde_le.buf_ptr,
			skb_headlen(skb), DMA_TO_DEVICE);

		bytes_sent += skb->len;

		dev_kfree_skb_any(skb);
		tx_bd->tx_skbuff[skb_dirtytx] = NULL;
		txbde->buf_ptr = 0;
		txbde->lstatus = 0;

		txbde = cpri_eth_next_bde(txbde, base, tx_ring_size);

		skb_dirtytx = CPRI_ETH_NEXT_INDX(skb_dirtytx,
					CPRI_ETH_DEF_TX_RING_SIZE);


		howmany++;

	spin_lock_irqsave(&tx_bd->txlock, flags);
	(tx_bd->num_txbdfree)++;
	spin_unlock_irqrestore(&tx_bd->txlock, flags);
	}

	if (netif_queue_stopped(ndev) && tx_bd->num_txbdfree)
		netif_wake_queue(ndev);

	/* Update dirty indicators */
	tx_bd->skb_dirtytx = skb_dirtytx;
	tx_bd->tx_bd_dirty = txbde;


	netdev_completed_queue(ndev, howmany, bytes_sent);
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tctrltiminginten,
			ETH_EVENT_EN_MASK, ETH_EVENT_EN_MASK);
	return;
}

int cpri_eth_handle_tx(struct cpri_framer *framer)
{

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

	if (estatus & CPRI_ETH_RX_EST_MII_PERR_MASK)
		priv->extra_stats.mii_error++;
	else if (estatus & CPRI_ETH_RX_EST_SPKT_MASK)
		priv->extra_stats.small_pkt++;
	else if (estatus & CPRI_ETH_RX_EST_OFLW_MASK)
		priv->extra_stats.rx_overflow++;
	else if (estatus & CPRI_ETH_RX_EST_DMAC_MM_MASK)
		priv->extra_stats.dmac_mismatch++;
	else if (estatus & CPRI_ETH_RX_EST_CRC_ERR_MASK)
		priv->extra_stats.crc_error++;
	else if (estatus & CPRI_ETH_RX_EST_RX_ERR_MASK)
		priv->extra_stats.cpri_rx_error++;
	else if (estatus & CPRI_ETH_RX_EST_LFRM_MASK)
		priv->extra_stats.long_frame++;
	else
		if (estatus & CPRI_ETH_RX_EST_SFRM_MASK)
			priv->extra_stats.short_frame++;
}

static int cpri_eth_rx_pkt_error(struct net_device *ndev,
				 unsigned int skb_currx)
{
	struct cpri_eth_bd_entity *rxbde, rxbde_le;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;

	rxbde = rx_bd->rx_bd_base + skb_currx;

	CPRI_ETH_BD_TO_LE(&rxbde_le, rxbde); /* b-endian */

	if (BD_LSTATUS_SSHIFT(rxbde_le.lstatus) & CPRI_ETH_BD_RX_CRC)
		priv->stats.rx_crc_errors++;
	else if (BD_LSTATUS_SSHIFT(rxbde_le.lstatus) & CPRI_ETH_BD_RX_PLE)
		priv->stats.rx_length_errors++;
	else if (BD_LSTATUS_SSHIFT(rxbde_le.lstatus) & CPRI_ETH_BD_RX_BOF)
		priv->stats.rx_over_errors++;

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
	int loop;
	struct device *dev = priv->framer->cpri_dev->dev;

	memset(&rxbde_le, 0, sizeof(struct cpri_eth_bd_entity));

	rxbde = rx_bd->rx_bd_base + skb_currx;
	rmb();
	skb = rx_bd->rx_skbuff[skb_currx];
	/* Add another skb for the future */
	newskb = cpri_eth_new_skb(ndev);
	rmb();

	CPRI_ETH_BD_TO_LE(&rxbde_le, rxbde); /* b-endian */
	pkt_len = BD_LSTATUS_LSHIFT(rxbde_le.lstatus);
	dma_unmap_single(&priv->ofdev->dev, rxbde_le.buf_ptr,
			priv->rx_buffer_size, DMA_FROM_DEVICE);


	if (skb->pkt_type == PACKET_MULTICAST)
		priv->stats.multicast++;

	if (unlikely(!newskb)) {
		/* we drop the current packet and reuse the skb */
		priv->stats.rx_dropped++;
		newskb = skb;
	} else {
		/* Increment the number of packets */
		priv->stats.rx_packets++;
		if (!(priv->flags & CPRI_ETH_HW_CRC_STRIP)) {
			/* We need to strip it */
			pkt_len -= ETH_FCS_LEN;
		}
		cpri_eth_stats_incr_val(&priv->stats.rx_bytes, pkt_len);

		dev_dbg(dev, "rx_bd- buf_ptr: 0x%x, lsts: 0x%x, len: %d\n",
				rxbde_le.buf_ptr, rxbde_le.lstatus, pkt_len);
		for (loop = 0; loop < pkt_len;) {
			dev_dbg(dev, "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
			*((skb->data + loop)), *((skb->data + loop + 1)),
			*((skb->data + loop + 2)), *((skb->data + loop + 3)),
			*((skb->data + loop + 4)), *((skb->data + loop + 5)),
			*((skb->data + loop + 6)), *((skb->data + loop + 7)));
			loop += 8;
		}

		skb_put(skb, pkt_len);
		cpri_eth_stats_incr_val(&priv->stats.rx_bytes, pkt_len);
		/* No checksum offloading */
		skb_checksum_none_assert(skb);

		/* Tell the skb what kind of packet this is */
		skb->protocol = eth_type_trans(skb, ndev);

		ret = netif_receive_skb(skb);

		if (NET_RX_DROP == ret) {
			priv->stats.rx_dropped++;
			priv->extra_stats.kernel_dropped++;
		}
	}


	rx_bd->rx_skbuff[skb_currx] = newskb;

	cpri_new_rxbdp(rx_bd, rxbde, newskb, skb_currx);
	return 0;
}

static int cpri_eth_clean_rx_ring(struct net_device *ndev, int budget)
{
	int howmany = 0, pkterr = 0;
	unsigned int skb_currx = 0;
	struct cpri_eth_bd_entity *rxbde, *base;
	struct cpri_eth_priv *priv = netdev_priv(ndev);
	struct cpri_eth_rx_bd *rx_bd = priv->rx_bd;
	struct cpri_eth_bd_entity rxbde_le;


	spin_lock(&priv->rx_bd->rxlock);
	base = (struct cpri_eth_bd_entity *)rx_bd->rx_bd_base;
	rxbde = rx_bd->rx_bd_current;
	CPRI_ETH_BD_TO_LE(&rxbde_le, rxbde); /* b-endian */
	skb_currx = rx_bd->skb_currx;


	/* No LE conversion here. Ctrl is only a byte value */

	while (!(BD_LSTATUS_SSHIFT(rxbde_le.lstatus) & CPRI_ETH_BD_RX_EMPTY)
					&& (howmany < budget)) {

		if (BD_LSTATUS_SSHIFT(rxbde_le.lstatus) &
				CPRI_ETH_BD_RX_ABORT) {
			cpri_eth_rx_pkt_error(ndev, skb_currx);
			pkterr++;
		} else
			cpri_eth_rx_pkt(ndev, skb_currx);

		howmany++;

		rxbde =
			cpri_eth_next_bde(rxbde, base, rx_bd->rx_bd_ring_size);
		skb_currx =
			CPRI_ETH_NEXT_INDX(skb_currx, rx_bd->rx_bd_ring_size);
		CPRI_ETH_BD_TO_LE(&rxbde_le, rxbde); /* b-endian */

	}

	if (pkterr)
		cpri_eth_check_rx_ex_status_reg(ndev);

	rx_bd->rx_bd_current = rxbde;
	rx_bd->skb_currx = skb_currx;
	spin_unlock(&priv->rx_bd->rxlock);

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
			priv->extra_stats.rx_dma_overrun++;
		else
			priv->extra_stats.rx_overrun++;

		/* Rx Overrun - Restart Rx DMA*/
		napi_disable(&priv->napi);
		cpri_eth_check_rx_ex_status_reg(ndev);
		/* Set 1 to clear the overflow bit */
		cpri_reg_set(&framer->regs->cpri_errinten,
			CPRI_ETH_RX_DMA_OVR_EN_MASK);
		napi_enable(&priv->napi);

	} else if (errval & CPRI_ETH_RX_BD_UDR_EN_MASK) {

		priv->extra_stats.rx_underrun++;

		/* Make space in the rx ring and enable dma*/
		napi_disable(&priv->napi);

		cpri_eth_clean_rx_ring(ndev, CPRI_ETH_NAPI_WEIGHT);

		napi_enable(&priv->napi);

	} else if (errval & CPRI_ETH_TX_UDR_EN_MASK) {

		priv->extra_stats.tx_underrun++;

		netif_tx_disable(ndev);
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
	struct cpri_framer *framer = priv->framer;

	howmany = cpri_eth_clean_rx_ring(ndev, budget);

	if (howmany < budget) {
		napi_complete(napi);
		cpri_reg_write(&framer->regs_lock,
				&framer->regs->cpri_rctrltiminginten,
				ETH_EVENT_EN_MASK, ETH_EVENT_EN_MASK);
	}

	return howmany;
}

int cpri_eth_handle_rx(struct cpri_framer *framer)
{
	struct cpri_eth_priv *priv = framer->eth_priv;

	if (napi_schedule_prep(&priv->napi)) {
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
	spin_lock_init(&priv->tx_bd->txlock);

	priv->rx_bd = kzalloc(sizeof(struct cpri_eth_rx_bd), GFP_KERNEL);
	if (!priv->rx_bd) {
		err = -ENOMEM;
		netdev_err(ndev, "Failed to allocate rx bd\n");
		goto rx_alloc_failed;
	}
	priv->rx_bd->rx_bd_ring_size = CPRI_ETH_DEF_RX_RING_SIZE;
	priv->rx_bd->skb_currx = 0;
	spin_lock_init(&priv->rx_bd->rxlock);
	INIT_WORK(&priv->reset_task, cpri_eth_reset_task);
	INIT_WORK(&priv->error_task, cpri_eth_error_task);
	tasklet_init(&priv->tasklet, cpri_eth_tx_cleanup, (unsigned long)priv);
	priv->flags = CPRI_ETH_DEF_FLAGS;
	priv->rx_buffer_size = CPRI_ETH_DEF_RX_BUF_SIZE;
	priv->tx_start_thresh = CPRI_ETH_DEF_TX_START_THRESH;
	priv->rx_coales_thresh = CPRI_ETH_DEF_RX_COAL_THRESH;
	priv->tx_coales_thresh = CPRI_ETH_DEF_TX_COAL_THRESH;

	priv->fwdif_status = CPRI_ETH_DISABLED;

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
#if 0	/* need to update to support foward interface */
	struct cpri_framer *framer = priv->framer;
#endif

	cpri_eth_config(ndev, priv->flags & CPRI_ETH_HW_CRC_STRIP);
	cpri_eth_config(ndev, priv->flags & CPRI_ETH_MAC_FAIL_PASS);
#if 0	/* need to update to support foward interface */
	cpri_reg_set(&framer->regs->cpri_ethfwdctrl,
			CPRI_ETH_FWD_ENABLE_MASK);
#endif

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
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_rctrltiminginten, MASK_ALL,
			(CONTROL_INT_LEVEL_MASK | ETH_EVENT_EN_MASK));
	cpri_reg_write(&framer->regs_lock,
			&framer->regs->cpri_tctrltiminginten, MASK_ALL,
			(CONTROL_INT_LEVEL_MASK | ETH_EVENT_EN_MASK));
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


void reset_eth_regs(struct cpri_framer *framer)
{
	cpri_reg_set_val(&framer->regs->cpri_ethcfg1,
		MASK_ALL, 0);
}

int cpri_eth_init(struct platform_device *ofdev, struct cpri_framer *framer,
			struct device_node *frnode)
{
	struct net_device *ndev = NULL;
	struct cpri_eth_priv *priv = NULL;
	int err = 0;

	reset_eth_regs(framer);
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
	sprintf(ndev->name, "%s%d_eth%d", DEV_NAME, framer->cpri_dev->dev_id,
			framer->id-1);
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

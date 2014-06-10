/*
 * drivers/net/cpri/cpri_cw.c
 * CPRI device driver - control word utility functions
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

static u32 cw_read(u32 *addr, u32 mask)
{
	return cpri_reg_get_val(addr, mask);
}

/* clear and disable control table */
void clear_control_tx_table(struct cpri_framer *framer)
{
	u32 value = 0, i;
	struct cpri_framer_regs *regs = framer->regs;

	cpri_reg_clear(&regs->cpri_tctrldata0,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrldata1,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrldata2,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrldata3,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrlinserttb1,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_tctrlinserttb2,
		MASK_ALL);
	cpri_reg_clear(&regs->cpri_config,
		TX_CW_INSERT_EN_MASK);

	wmb();

	for (i = 0; i <= MAX_TCTA_ADDR; i++) {
		value = i << TCTA_ADDR_OFFSET;
		value |= TCT_WRITE_MASK;
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
				MASK_ALL, value);
	}
}

/* read_rx_cw - reads the rx control word
 * @bf_index: the 0-255 index number in one hyper frame.
 * @buf: pass 16 bytes to buf and data will be placed to it.
 */
void read_rx_cw(struct cpri_framer *framer, int bf_index, u8 *buf)
{
	struct cpri_framer_regs *regs = framer->regs;
	u32 *reg_addr;
	u32 data;
	int i;
	raw_spin_lock(&framer->rx_cwt_lock);
	cpri_reg_set_val(&regs->cpri_rctrlattrib,
				TCT_ADDR_MASK, bf_index);
	reg_addr = (&regs->cpri_rctrldata0);
	wmb();
	/* data is in big endian */
	for (i = 0; i < 4; i++) {
		data = cw_read(reg_addr, MASK_ALL);
		*((u32*)buf) = be32_to_cpu(data);
		buf += 4;
		reg_addr++;
	}
	raw_spin_unlock(&framer->rx_cwt_lock);
}

/* rdwr_tx_cw - read/write the tx control table.
 * @bf_index : 0-255 number in one hyper frame
 * @operation: use only one of the following:
 * 	1. TX_CW_READ: reads the tx control table,
 * 	data will be put into buf.
 * 	2. TX_CW_WRITE: write to tx control table,
 * 	using the 16 byte data in buf, the cpri control
 * 	word in optical link will be updated.
 *	Note that not all 0-255 bfs can be updated.
 * 	3. TX_CRTL_TBL_BYPASS: this control word will be
 * 	inserted from framer instead(eg. daisy chained)
 */
void rdwr_tx_cw(struct cpri_framer *framer, int bf_index, int operation, u8 *buf)
{
	struct cpri_framer_regs *regs = framer->regs;
	int i;
	u32 *reg_tcd0, *reg_tctie;
	int data;
	int mask = 0;

	reg_tcd0 = (&regs->cpri_tctrldata0);
	if (operation & TX_CW_READ) {
		raw_spin_lock(&framer->tx_cwt_lock);
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
				MASK_ALL,
				bf_index << TCTA_ADDR_OFFSET);

		for (i = 0; i < 4; i++) {
			data = cw_read(reg_tcd0, MASK_ALL);
			*((u32*)buf) = be32_to_cpu(data);
			buf += 4;
			reg_tcd0++;
		}
		raw_spin_unlock(&framer->tx_cwt_lock);
		return;
	}

	if ((bf_index >= 0) && (bf_index <= 15)) {
		reg_tctie = (&regs->cpri_tctrlinserttb1);
		mask = 1 << bf_index;
	} else if ((bf_index >= 64) && (bf_index <= 79)) {
		reg_tctie = (&regs->cpri_tctrlinserttb1);
		mask = 1 << (bf_index - 48);
	} else if ((bf_index >= 128) && (bf_index <= 143)) {
		reg_tctie = (&regs->cpri_tctrlinserttb2);
		mask = 1 << (bf_index - 128);
	} else if ((bf_index >= 192) && (bf_index <= 207)) {
		reg_tctie = (&regs->cpri_tctrlinserttb2);
		mask = 1 << (bf_index - 176);
	} else {
		reg_tctie = (&regs->cpri_tctrlinserttb1);
		mask = 0;
	}
	if (operation & TX_CTRL_TBL_BYPASS) {
		cpri_reg_clear(reg_tctie, mask);
	} else {
		cpri_reg_set(&regs->cpri_config,
				TX_CW_INSERT_EN_MASK);
		raw_spin_lock(&framer->tx_cwt_lock);
		for (i = 0; i < 4; i++) {
			data = *((u32*)buf);
			data = cpu_to_be32(data);
			cpri_reg_set_val(reg_tcd0,
			MASK_ALL, data);
			buf += 4;
			reg_tcd0++;
		}
		cpri_reg_set_val(&regs->cpri_tctrlattrib,
			MASK_ALL,
			(bf_index << TCTA_ADDR_OFFSET)
			| TCT_WRITE_MASK);
		cpri_reg_set_val(reg_tctie,
			MASK_ALL, mask);
		raw_spin_unlock(&framer->tx_cwt_lock);
		return;
	}

}

/* This function configures the tx vss DMA */
int tx_vss_config(struct cpri_framer *framer, int buffer_size, int threshold,
			int axi_trans_size, int vss_int_en)
{
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	dma_addr_t phys_addr;
	void *virt_addr;
	
	framer->vss_tx_ctrl.buff_size = buffer_size;
	framer->vss_tx_ctrl.threshold = threshold;

	virt_addr = dma_alloc_coherent(dev, buffer_size, &phys_addr, GFP_KERNEL);
	if (virt_addr == NULL) {
		dev_err(dev, "can't alloc buffer for vss dma!");
		return -ENOMEM;
	}
	memset(virt_addr, 0, buffer_size);

	framer->vss_tx_ctrl.base_addr_phys = (u32)phys_addr;
	framer->vss_tx_ctrl.base_addr_virt = (u32)virt_addr;

	/* set vss base address */
	cpri_reg_set_val(&regs->cpri_tvssbaddr,
				MASK_ALL,
				phys_addr);
	cpri_reg_set_val(&regs->cpri_tvssbaddrmsb,
				MASK_ALL,
				phys_addr >> 28);
	/* set vss buffer size */
	cpri_reg_set_val(&regs->cpri_tvssbufsize,
				MASK_ALL,
				buffer_size - 1);
	/* set vss threshold */
	cpri_reg_set_val(&regs->cpri_tvssthresh,
				MASK_ALL,
				threshold);
	/* set axi transaction size */
	cpri_reg_set_val(&regs->cpri_tvssaxisize,
				MASK_ALL,
				axi_trans_size / 16);
	/* rx vss interrupt enable */
	if (vss_int_en)
		cpri_reg_set(&regs->cpri_tctrltiminginten, 1);
	else
		cpri_reg_clear(&regs->cpri_tctrltiminginten, 1);
	
	cpri_reg_set(&regs->cpri_config, 1);

	/* During the debug we find out that
	 * the vss enabling in this way could lead
	 * to alignment problem, so the vss DMA is
	 * not enabled. The vss data will be written
	 * through the tx control table instead
	 */
	/* Enable vss transfer, last step */
	/* cpri_reg_set(&regs->cpri_tcr, 1 << 3); */

	return 0;
}	
	

/* This function configures rx vss DMA */
int rx_vss_config(struct cpri_framer *framer, int buffer_size, int threshold,
			int axi_trans_size, int vss_int_en)
{
	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	dma_addr_t phys_addr;
	void *virt_addr;

	framer->vss_rx_ctrl.buff_size = buffer_size;
	framer->vss_rx_ctrl.threshold = threshold;

	virt_addr = dma_alloc_coherent(dev, buffer_size, &phys_addr, GFP_KERNEL);
	if (virt_addr == NULL) {
		dev_err(dev, "can't alloc buffer for vss dma!");
		return -ENOMEM;
	}
	framer->vss_rx_ctrl.base_addr_phys = (u32)phys_addr;
	framer->vss_rx_ctrl.base_addr_virt = (u32)virt_addr;

	/* set vss base address */
	cpri_reg_set_val(&regs->cpri_rvssbaddr,
				MASK_ALL,
				phys_addr);
	cpri_reg_set_val(&regs->cpri_rvssbaddrmsb,
				MASK_ALL,
				phys_addr >> 28);

	/* set vss buffer size */
	cpri_reg_set_val(&regs->cpri_rvssbufsize,
				MASK_ALL,
				buffer_size - 1);
	/* set vss threshold */
	cpri_reg_set_val(&regs->cpri_rvssthresh,
				MASK_ALL,
				threshold);
	/* ser axi transaction size */
	cpri_reg_set_val(&regs->cpri_rvssaxisize,
				MASK_ALL,
				axi_trans_size / 16);
		
	/* rx vss interrupt enable */
	if (vss_int_en)
		cpri_reg_set(&regs->cpri_rctrltiminginten, 1);
	else
		cpri_reg_clear(&regs->cpri_rctrltiminginten, 1);

	/* Enable vss transfer, last step */
	cpri_reg_set(&regs->cpri_rcr, 1 << 3);

	return 0;
}


/* TBD: this function is used for
 * transient data processing in rx vss.
 * It's TBD how the user is going to implement it.
 */
void vss_rx_processing(struct work_struct *work)
{
        /* struct cpri_framer *framer =
		container_of(work, struct cpri_framer,
		vss_rx_task);
	*/
	return;
}


void vss_tx_processing(struct work_struct *work)
{
	return;
}


void rx_vss_data_read(struct cpri_framer *framer, int cnt, u8 *buf)
{
	u32 *virt_addr;

	virt_addr = (u32*)(framer->vss_rx_ctrl.base_addr_virt);
	memcpy(buf, virt_addr, cnt);
}

void tx_vss_data_write(struct cpri_framer *framer, int cnt, u8 *buf)
{
	u32 *virt_addr;
	
	virt_addr = (u32*)(framer->vss_tx_ctrl.base_addr_virt);
	memcpy(virt_addr, buf, cnt);
}

void vss_deconfig(struct cpri_framer *framer)
{

	struct cpri_framer_regs *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	
	/* disable vss transfer */
	cpri_reg_clear(&regs->cpri_config, 1);
	cpri_reg_clear(&regs->cpri_rcr, 1 << 3);
	cpri_reg_clear(&regs->cpri_tcr, 1 << 3);
	
	/* disable interrupt */
	cpri_reg_clear(&regs->cpri_rctrltiminginten, 1);
	cpri_reg_clear(&regs->cpri_tctrltiminginten, 1);
	
	/* Reset tx vss register back */
	cpri_reg_clear(&regs->cpri_tvssbaddr, MASK_ALL);

	cpri_reg_clear(&regs->cpri_tvssbaddrmsb, MASK_ALL);

	cpri_reg_set_val(&regs->cpri_tvssbufsize,
				MASK_ALL, 0xF);

	cpri_reg_clear(&regs->cpri_tvssthresh, MASK_ALL);

	cpri_reg_set_val(&regs->cpri_tvssaxisize,
				MASK_ALL, 8);
	
	/* Reset rx vss register back */
	cpri_reg_clear(&regs->cpri_rvssbaddr, MASK_ALL);

	cpri_reg_clear(&regs->cpri_rvssbaddrmsb, MASK_ALL);

	cpri_reg_set_val(&regs->cpri_rvssbufsize,
				MASK_ALL, 0xF);

	cpri_reg_clear(&regs->cpri_rvssthresh, MASK_ALL);

	cpri_reg_set_val(&regs->cpri_rvssaxisize,
				MASK_ALL, 8);
	
	/* free DMA memory buffer */
	if (framer->vss_rx_ctrl.base_addr_virt)
		dma_free_coherent(dev, framer->vss_rx_ctrl.buff_size,
		(void *)(framer->vss_rx_ctrl.base_addr_virt),
		framer->vss_rx_ctrl.base_addr_phys);
	framer->vss_rx_ctrl.base_addr_virt = 0;

	if (framer->vss_tx_ctrl.base_addr_virt)
		dma_free_coherent(dev, framer->vss_tx_ctrl.buff_size,
		(void *)(framer->vss_tx_ctrl.base_addr_virt),
		framer->vss_tx_ctrl.base_addr_phys);
	framer->vss_tx_ctrl.base_addr_virt = 0;
	
	schedule_timeout_interruptible(msecs_to_jiffies(2));
}


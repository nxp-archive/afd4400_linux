/*
 * drivers/net/cpri/cpri_axc.c
 * CPRI device driver - axc map
 * Author: Anand Singh.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
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
#include <linux/of_address.h>
#include <linux/pid.h>

#include <linux/cpri.h>
#include <linux/cpri_axc.h>

static inline void OPR_BITMAP(u32 *addr, unsigned val, unsigned cmd)
{
	if (cmd)
		*addr |= val;
	else
		*addr &= ~val;
}

static int get_segment_id(struct axc_pos *axc_pos, unsigned short size)
{
	int seg_num;

	seg_num = ((((axc_pos->axc_start_w - 1) * size) +
				axc_pos->axc_start_b) / SEG_SIZE);
	if (seg_num < 0)
		return 0;
	return seg_num;
}

void dump_all_segment(struct cpri_framer *framer, u32 cmd)
{
	int loop;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	u32 val;
	u32 val1;
	u32 val2;

	dev_dbg(dev, "========= CPRI Segment dump ==========");
	dev_dbg(dev, " ");
	for (loop = 0; loop < 3072; loop++) {

		val = loop;

		if (cmd & UL_AXCS) {
			cpri_reg_clear(&regs->cpri_tcma, MASK_ALL);
			cpri_reg_write(&framer->regs_lock, &regs->cpri_tcma,
				(AXC_TBL_WRITE_MASK | AXC_TBL_SEG_ADDR_MASK),
				val);
			mdelay(1);
			val2  = cpri_reg_get_val(&regs->cpri_tcmd0, MASK_ALL);
			val1  = cpri_reg_get_val(&regs->cpri_tcmd1, MASK_ALL);
			dev_dbg(dev, "ul_seg[%d] tcmd0: 0x%x, tcmd1: 0x%x ...",
					val, val2, val1);
		} else {
			cpri_reg_clear(&regs->cpri_rcma, MASK_ALL);
			cpri_reg_write(&framer->regs_lock, &regs->cpri_rcma,
				(AXC_TBL_WRITE_MASK | AXC_TBL_SEG_ADDR_MASK),
				val);
			mdelay(1);
			val2 = cpri_reg_get_val(&regs->cpri_rcmd0, MASK_ALL);
			val1  = cpri_reg_get_val(&regs->cpri_rcmd1, MASK_ALL);
			dev_dbg(dev, "dl_seg[%d] rcmd0: 0x%x, rcmd1: 0x%x ...",
					val, val2, val1);
		}
	}
	dev_dbg(dev, " ");
	dev_dbg(dev, " ");

}


void clear_axc_map_tx_rx_table(struct cpri_framer *framer)
{
	int loop;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	u32 val;

	for (loop = 0; loop < 3072; loop++) {
		cpri_reg_clear(&regs->cpri_tcma, MASK_ALL);
		cpri_reg_clear(&regs->cpri_rcma, MASK_ALL);
		cpri_reg_clear(&regs->cpri_tcmd0, MASK_ALL);
		cpri_reg_clear(&regs->cpri_tcmd1, MASK_ALL);
		cpri_reg_clear(&regs->cpri_rcmd0, MASK_ALL);
		cpri_reg_clear(&regs->cpri_rcmd1, MASK_ALL);
		val = (AXC_TBL_WRITE_MASK | loop);
		cpri_reg_write(&framer->regs_lock, &regs->cpri_rcma,
			(AXC_TBL_WRITE_MASK | AXC_TBL_SEG_ADDR_MASK),
			val);
		mdelay(1);
		cpri_reg_clear(&regs->cpri_rcma, MASK_ALL);
		cpri_reg_write(&framer->regs_lock, &regs->cpri_tcma,
			(AXC_TBL_WRITE_MASK | AXC_TBL_SEG_ADDR_MASK),
			val);
		mdelay(1);
	}
	cpri_reg_clear(&regs->cpri_tcma, MASK_ALL);
	cpri_reg_clear(&regs->cpri_rcma, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_tbl_config, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_config, MASK_ALL);

	cpri_reg_clear(&regs->cpri_map_k_select_rx1, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_k_select_tx1, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_k_select_rx2, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_k_select_tx2, MASK_ALL);


}

void cleanup_segment_table_data(struct cpri_framer *framer)
{
	unsigned int loop;

	kfree(framer->ul_map_table.segments);
	kfree(framer->dl_map_table.segments);
	kfree(framer->ul_map_table.segment_bitmap);
	kfree(framer->dl_map_table.segment_bitmap);
	for (loop = 0; loop < framer->max_segments; loop++) {
		kfree(framer->ul_map_table.k0_bitmap[loop]);
		kfree(framer->dl_map_table.k0_bitmap[loop]);
		kfree(framer->ul_map_table.k1_bitmap[loop]);
		kfree(framer->dl_map_table.k1_bitmap[loop]);
	}
	kfree(framer->ul_map_table.k0_bitmap);
	kfree(framer->dl_map_table.k0_bitmap);
	kfree(framer->ul_map_table.k1_bitmap);
	kfree(framer->dl_map_table.k1_bitmap);
}

int populate_segment_table_data(struct cpri_framer *framer)
{
	struct device *dev = framer->cpri_dev->dev;
	unsigned int loop;
	unsigned int k0_k1_max;

	k0_k1_max = (framer->framer_param.k0 > framer->framer_param.k1) ?
			framer->framer_param.k0 : framer->framer_param.k1;
	framer->max_segments = ((framer->autoneg_output.cpri_bf_word_size *
			BF_WRDS) * k0_k1_max) / SEG_SIZE;

	if (framer->max_segments <= 0) {
		dev_err(dev, "segment tble population failed 'err max seg'!!");
		return -EINVAL;
	}

	framer->ul_map_table.segments = kzalloc(framer->max_segments *
			sizeof(struct segment),
			GFP_KERNEL);
	if (framer->ul_map_table.segments == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	framer->ul_map_table.segment_bitmap = kzalloc((framer->max_segments *
				sizeof(u32)), GFP_KERNEL);
	if (framer->ul_map_table.segment_bitmap == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	framer->dl_map_table.segments = kzalloc(
			(framer->max_segments * sizeof(struct axc_map_table)),
			GFP_KERNEL);
	if (framer->dl_map_table.segments == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	framer->dl_map_table.segment_bitmap = kzalloc((framer->max_segments *
						sizeof(u32)), GFP_KERNEL);
	if (framer->dl_map_table.segment_bitmap == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	framer->ul_map_table.k0_bitmap = kzalloc(framer->max_segments *
			sizeof(u32 *), GFP_KERNEL);
	if (framer->ul_map_table.k0_bitmap == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	framer->ul_map_table.k1_bitmap = kzalloc(framer->max_segments *
			sizeof(u32 *), GFP_KERNEL);
	if (framer->ul_map_table.k1_bitmap == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	framer->dl_map_table.k0_bitmap = kzalloc(framer->max_segments *
			sizeof(u32 *), GFP_KERNEL);
	if (framer->dl_map_table.k0_bitmap == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}

	framer->dl_map_table.k1_bitmap = kzalloc(framer->max_segments *
			sizeof(u32 *), GFP_KERNEL);
	if (framer->dl_map_table.k1_bitmap == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	for (loop = 0; loop < framer->max_segments; loop++) {
		framer->ul_map_table.k0_bitmap[loop] = kzalloc((2 *
				sizeof(u32)), GFP_KERNEL);
		if (framer->ul_map_table.k0_bitmap[loop] == NULL) {
			dev_err(dev, "System mem exhausted !! kzalloc fail\n");
			return -ENOMEM;
		}
		framer->ul_map_table.k1_bitmap[loop] = kzalloc((2 *
				sizeof(u32)), GFP_KERNEL);
		if (framer->ul_map_table.k1_bitmap[loop] == NULL) {
			dev_err(dev, "System mem exhausted !! kzalloc fail\n");
			return -ENOMEM;
		}
		framer->dl_map_table.k0_bitmap[loop] = kzalloc((2 *
				sizeof(u32)), GFP_KERNEL);
		if (framer->dl_map_table.k0_bitmap[loop] == NULL) {
			dev_err(dev, "System mem exhausted !! kzalloc fail\n");
			return -ENOMEM;
		}
		framer->dl_map_table.k1_bitmap[loop] = kzalloc((2 *
				sizeof(u32)), GFP_KERNEL);
		if (framer->dl_map_table.k1_bitmap[loop] == NULL) {
			dev_err(dev, "System mem exhausted !! kzalloc fail\n");
			return -ENOMEM;
		}
	}
	framer->ul_map_table.k0_max = 0;
	framer->ul_map_table.k1_max = 0;
	framer->dl_map_table.k0_max = 0;
	framer->dl_map_table.k1_max = 0;
	return 0;
}

int init_axc_mem_blk(struct cpri_framer *framer, struct device_node *child)
{
	struct axc_mem_info mblk_info;
	unsigned int property[4] = { 0, 0, 0, 0 };
	struct axc_buf_head *tx_mblk;
	struct axc_buf_head *rx_mblk;
	int ret = 0;
	struct device *dev = framer->cpri_dev->dev;

	framer->dl_axcs = NULL;
	framer->ul_axcs = NULL;
	memset(&mblk_info, 0, sizeof(struct axc_mem_info));
	memset(&framer->tx_buf_head, 0, sizeof(struct axc_buf_head));
	memset(&framer->rx_buf_head, 0, sizeof(struct axc_buf_head));
	/* read dts entry */
	ret = of_property_read_u32_array(child, "memblk-rx", property,
				ARRAY_SIZE(property));
	mblk_info.tx_mblk_addr[0] = INVALIDE_MBLK_ADDR;
	mblk_info.tx_mblk_addr[1] = INVALIDE_MBLK_ADDR;
	mblk_info.rx_mblk_addr[0] = INVALIDE_MBLK_ADDR;
	mblk_info.rx_mblk_addr[1] = INVALIDE_MBLK_ADDR;
	if (ret) {
		dev_err(dev, "cpri_dev: dts memblk-rx error\n");
		ret = -EFAULT;
		goto mem_err;
	}
	framer->rx_mblk_hardware_addr0 = property[0];
	framer->rx_mblk_hardware_addr1 = property[2];
	mblk_info.rx_mblk_addr[0] = property[0] -
			framer->rx_mblk_hardware_addr0;
	mblk_info.rx_mblk_addr[1] = property[2] -
			framer->rx_mblk_hardware_addr1;
	mblk_info.rx_mblk_size[0] = property[1];
	mblk_info.rx_mblk_size[1] = property[3];
	memset(property, 0, sizeof(property));
	ret = of_property_read_u32_array(child, "memblk-tx", property,
				ARRAY_SIZE(property));
	if (ret) {
		dev_err(dev, "cpri_dev: dts memblk-tx error\n");
		ret = -EFAULT;
		goto mem_err;
	}
	framer->tx_mblk_hardware_addr0 = property[0];
	framer->tx_mblk_hardware_addr1 = property[2];
	mblk_info.tx_mblk_addr[0] = property[0] -
			framer->tx_mblk_hardware_addr0;
	mblk_info.tx_mblk_addr[1] = property[2] -
			framer->tx_mblk_hardware_addr1;
	mblk_info.tx_mblk_size[0] = property[1];
	mblk_info.tx_mblk_size[1] = property[3];
	dev_dbg(dev, "cpri_axc: memblk info-----\n");
	dev_dbg(dev, "cpri_axc:Rx addr0: 0x%x size0: %d\n",
			mblk_info.rx_mblk_addr[0],
			mblk_info.rx_mblk_size[0]);
	dev_dbg(dev, "cpri_axc:Rx addr1: 0x%x size1: %d\n",
			mblk_info.rx_mblk_addr[1],
			mblk_info.rx_mblk_size[1]);
	dev_dbg(dev, "cpri_axc:Tx addr0: 0x%x size0: %d\n",
			mblk_info.tx_mblk_addr[0],
			mblk_info.tx_mblk_size[0]);
	dev_dbg(dev, "cpri_axc:Tx addr1: 0x%x size1: %d\n",
			mblk_info.tx_mblk_addr[1],
			mblk_info.tx_mblk_size[1]);
	ret = of_property_read_u32(child, "max-axcs", property);
	if (ret) {
		dev_err(dev, "cpri_dev: dts max-axcs error\n");
		ret = -EINVAL;
		goto mem_err;
	}
	framer->framer_param.max_axc_count = property[0];
	dev_dbg(dev, "cpri_axc maximum axc count: %d\n",
			framer->framer_param.max_axc_count);


	tx_mblk = &framer->tx_buf_head;
	rx_mblk = &framer->rx_buf_head;
	tx_mblk->blk_bitmap = 0;
	rx_mblk->blk_bitmap = 0;
	/* tx mem block initiallization */
	/* tx mem blk 1 */
	if (mblk_info.tx_mblk_addr[0] != INVALIDE_MBLK_ADDR) {
		tx_mblk->blk_bitmap |= 1;
		tx_mblk->max_bufs = framer->framer_param.max_axc_count;
		tx_mblk->blocks[0].base = mblk_info.tx_mblk_addr[0];
		tx_mblk->blocks[0].size = mblk_info.tx_mblk_size[0];
		tx_mblk->blocks[0].next_free_addr = tx_mblk->blocks[0].base;
	}
	/* tx mem blk 2 */
	if (mblk_info.tx_mblk_addr[1] != INVALIDE_MBLK_ADDR) {
		tx_mblk->blk_bitmap |= 1 << 1;
		tx_mblk->blocks[1].base = mblk_info.tx_mblk_addr[1];
		tx_mblk->blocks[1].size = mblk_info.tx_mblk_size[1];
		tx_mblk->blocks[1].next_free_addr = tx_mblk->blocks[1].base;
	}
	/* rx mem block initiallization */
	/* rx mem blk 1 */
	if (mblk_info.rx_mblk_addr[0] != INVALIDE_MBLK_ADDR) {
		rx_mblk->blk_bitmap |= 1;
		rx_mblk->max_bufs = framer->framer_param.max_axc_count;
		rx_mblk->blocks[0].base = mblk_info.rx_mblk_addr[0];
		rx_mblk->blocks[0].size = mblk_info.rx_mblk_size[0];
		rx_mblk->blocks[0].next_free_addr = rx_mblk->blocks[0].base;
	}
	/* rx mem blk 2 */
	if (mblk_info.rx_mblk_addr[1] != INVALIDE_MBLK_ADDR) {
		rx_mblk->blk_bitmap |= 1 << 1;
		rx_mblk->blocks[1].base = mblk_info.rx_mblk_addr[1];
		rx_mblk->blocks[1].size = mblk_info.rx_mblk_size[1];
		rx_mblk->blocks[1].next_free_addr = rx_mblk->blocks[1].base;
	}

	spin_lock_init(&tx_mblk->lock);
	spin_lock_init(&rx_mblk->lock);
	INIT_LIST_HEAD(&tx_mblk->free_list);
	INIT_LIST_HEAD(&rx_mblk->free_list);
	framer->tx_buf_head.allocated_bufs = 0;
	framer->rx_buf_head.allocated_bufs = 0;
	dev_dbg(dev, "cpri_axc: meminfo init success\n");
	return ret;
mem_err:
	return ret;
}

int init_framer_axc_param(struct cpri_framer *framer)
{
	struct device *dev = framer->cpri_dev->dev;
	unsigned int max_axc_count = framer->framer_param.max_axc_count;
	int ret = 0;

	cleanup_segment_table_data(framer);
	if (max_axc_count > framer->max_axcs) {
		dev_err(dev, "Axc count is not supported : max val %d\n",
				framer->max_axcs);
		dev_dbg(dev, " Change device init param for axc val\n");
		return -EINVAL;
	}
	if (framer->ul_axcs != NULL) /* this is to care reinit command */
		kfree(framer->ul_axcs);
	framer->ul_axcs = kzalloc((max_axc_count + 1) * sizeof(struct axc *),
			GFP_KERNEL);
	if (framer->ul_axcs == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	if (framer->dl_axcs != NULL) /* this is to care reinit command */
		kfree(framer->dl_axcs);
	framer->dl_axcs = kzalloc((max_axc_count + 1) * sizeof(struct axc *),
			GFP_KERNEL);
	if (framer->dl_axcs == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}

	clear_axc_map_tx_rx_table(framer);

	ret = populate_segment_table_data(framer);
	if (ret)
		dev_err(dev, "Segment table population failed\n");
	return ret;
}


struct axc_buf *axc_alloc(struct axc *axc, u32 size)
{
	struct device *dev = axc->framer->cpri_dev->dev;
	struct axc_buf *axc_buf = NULL;
	struct axc_buf *list_axc_buf = NULL;
	struct axc_buf *prev_axc_buf = NULL;
	struct axc_buf_head *mblk = NULL;
	int mem_blk = 0xff;
	struct list_head *pos, *nx;
	unsigned int blk_present0 = 0;
	unsigned int blk_present1 = 0;
	unsigned int condition1 = 0;
	unsigned int condition2 = 0;


	if (axc->flags & UL_AXCS) {
		mblk = &axc->framer->tx_buf_head;
		blk_present0 = (mblk->blk_bitmap & 0x1);
		blk_present1 = (mblk->blk_bitmap & (0x1 << 1));
	} else {
		mblk = &axc->framer->rx_buf_head;
		blk_present0 = (mblk->blk_bitmap & 0x1);
		blk_present1 = (mblk->blk_bitmap & (0x1 << 1));
	}

	if (!(mblk->blk_bitmap | 0x3)) {
		dev_err(dev, "there is no uplink mem entry in dts\n");
			return NULL;
		}

	spin_lock(&mblk->lock);
	condition1 = ((mblk->blocks[0].base + mblk->blocks[0].size) <
			(mblk->blocks[0].next_free_addr + size)) ? 0 : 1;
	condition2 = ((mblk->blocks[1].base + mblk->blocks[1].size) <
			(mblk->blocks[1].next_free_addr + size)) ? 0 : 1;
	if (mblk->allocated_bufs < mblk->max_bufs) {

		if ((blk_present0) && (condition1))
			mem_blk = 0;
		else if ((blk_present1) && (condition2))
			mem_blk = 1;
	}

	if ((mem_blk == 0) | (mem_blk == 1)) {
		axc_buf = kzalloc(sizeof(struct axc_buf), GFP_KERNEL);
		if (axc_buf == NULL) {
			dev_err(dev, "System mem exhausted !! kzalloc fail\n");
			return NULL;
		}

		axc_buf->addr = mblk->blocks[mem_blk].next_free_addr;
		dev_dbg(dev, "axc mem allocated addr: 0x%x", axc_buf->addr);

		axc_buf->size = size;
		axc_buf->mem_blk = mem_blk;
		axc_buf->axc = axc;
		mblk->blocks[mem_blk].next_free_addr += size;
		mblk->allocated_bufs++;
		goto out;
	}
	/* check if axc_buf size is available in free list
	*/
	if (list_empty(&mblk->free_list))
		goto out;

	list_for_each_safe(pos, nx, &mblk->free_list) {
		axc_buf = list_entry(pos, struct axc_buf, list);
		if (axc_buf->size >= size) {
			if ((axc_buf->size - size) > 0) {
				list_axc_buf = kzalloc(sizeof(struct axc_buf),
						GFP_KERNEL);
				if (axc_buf == NULL) {
					dev_err(dev, "memory exhausted !!\n");
					return NULL;
				}

				list_axc_buf->addr = axc_buf->addr;
				list_axc_buf->size = size;
				list_axc_buf->mem_blk = axc_buf->mem_blk;
				axc_buf->size = axc_buf->size - size;
				axc_buf->addr = list_axc_buf->addr + size;
				list_axc_buf->axc = axc;
				list_del(&axc_buf->list);
				list_for_each_safe(pos, nx, &mblk->free_list) {
					prev_axc_buf = list_entry(pos,
						struct axc_buf, list);
					if (prev_axc_buf->size >=
							axc_buf->size) {
						__list_add(&axc_buf->list,
								pos, nx);
					break;
					}
				}
				axc_buf = list_axc_buf;
				goto out;
			}
			list_del(&axc_buf->list);
			goto out;
		}
	}
	axc_buf = NULL;
out:
	spin_unlock(&mblk->lock);
	return axc_buf;
}

void axc_free(struct axc_buf *axc_buf)
{
	struct axc_buf_head *mblk = NULL;
	struct list_head *pos, *nx;
	struct axc *axc = axc_buf->axc;
	struct axc_buf *list_axc_buf;

	if (axc->flags & UL_AXCS)
		mblk = &axc->framer->tx_buf_head;
	else if (axc->flags & DL_AXCS)
		mblk = &axc->framer->rx_buf_head;

	spin_lock(&mblk->lock);
	axc_buf->axc = NULL;
	if (list_empty(&mblk->free_list))
		list_add_tail(&axc_buf->list, &mblk->free_list);
		goto out;

	list_for_each_entry(list_axc_buf, &mblk->free_list, list) {
		if ((list_axc_buf->addr + list_axc_buf->size) ==
				axc_buf->addr) {
				list_axc_buf->size +=
					axc_buf->size;
				kfree(axc_buf);
				goto out;
			} else if ((axc_buf->addr + axc_buf->size) ==
					list_axc_buf->addr) {
				list_axc_buf->size += axc_buf->size;
				list_axc_buf->addr =
					axc_buf->addr;
				kfree(axc_buf);
				goto out;
			}
	}

	list_for_each_safe(pos, nx, &mblk->free_list) {
		list_axc_buf = list_entry(pos, struct axc_buf, list);
		if (list_axc_buf->size >= axc_buf->size) {

			__list_add(&axc_buf->list, pos, nx);
			goto out;
		}
	}
out:
	spin_unlock(&mblk->lock);
	return;
}

void axc_buf_cleanup(struct cpri_framer *framer)
{
	struct axc_buf_head *tx_mblk = &framer->tx_buf_head;
	struct axc_buf_head *rx_mblk = &framer->rx_buf_head;
	struct axc_buf *list_axc_buf;
	struct list_head *pos, *nx;

	/* free tx free list */
	spin_lock(&tx_mblk->lock);
	list_for_each_safe(pos, nx, &tx_mblk->free_list) {
		list_axc_buf = list_entry(pos, struct axc_buf, list);
		list_del(&list_axc_buf->list);
		kfree(list_axc_buf);
	}
	spin_unlock(&tx_mblk->lock);

	spin_lock(&rx_mblk->lock);
	list_for_each_safe(pos, nx, &rx_mblk->free_list) {
		list_axc_buf = list_entry(pos, struct axc_buf, list);
		list_del(&list_axc_buf->list);
		kfree(list_axc_buf);
	}
	spin_unlock(&rx_mblk->lock);

	cleanup_segment_table_data(framer);
	return;
}

int calculate_axc_size(struct axc *axc)
{
	unsigned int axc_size = 0;
	unsigned int nc;
	if (axc->map_method == MAPPING_METHOD_1) {
		/* ceil func((S * SW) / K)
		 */
		axc_size = 2 * CEIL_FUNC((axc->S * axc->sampling_width),
				axc->K);

	} else {
		/* ceil func((S * na)/K)
		*/
		nc = CEIL_FUNC((axc->S * axc->na), axc->K);
		/* axc_size is 2M *nc */
		axc_size = ((2 * axc->sampling_width) * nc);
	}
	return axc_size;
}

unsigned int calculate_nst_position(struct axc *axc, unsigned int k_curr)
{
	unsigned int ki = 0;
	unsigned int nc = 0;
	unsigned mlt = 0;
	if (axc->map_method == MAPPING_METHOD_3) {
		/* ceil func((S * na) / k)
		 */
		nc = CEIL_FUNC((axc->S * axc->na), axc->K);
		mlt = (nc * k_curr * axc->K);
		/* floor func((nc * Ki * K) / Nv
		 */
		ki = (unsigned int)(mlt / axc->nst);
	}
	return ki;
}

/* this func will validate axc set parm data */
int axc_validate(struct cpri_framer *framer, struct axc_info *param,
		unsigned int count, u32 flag)
{
	struct device *dev = framer->cpri_dev->dev;
	unsigned char k0 = framer->framer_param.k0;
	unsigned char k1 = framer->framer_param.k1;
	unsigned int loop;
	unsigned int kpos;
	unsigned int seg;
	unsigned int word_size = framer->autoneg_output.cpri_bf_word_size;
	unsigned int seg_in_one_basic_frame;
	unsigned int axc_size;
	unsigned int size;
	unsigned int bit_position;
	unsigned int bit_set;
	struct axc axc_param;
	struct axc_pos *axc_pos;
	struct axc_map_table *map_table;
	struct segment *segments;
	struct segment *segment;
	u32 *segment_bitmap;
	u32 bf_map[2] = { 0 };
	u32 bf_map_val;
	u32 *k0_ptr, *k1_ptr;


	seg_in_one_basic_frame = (word_size * BF_WRDS) / SEG_SIZE;
	if (flag & DL_AXCS)
		map_table = &framer->dl_map_table;
	else
		map_table = &framer->ul_map_table;

	segment_bitmap = map_table->segment_bitmap;
	segments = map_table->segments;

	axc_pos = param->pos;
	axc_param.S = param->S;
	axc_param.K = param->K;
	axc_param.sampling_width = param->sample_width;
	axc_param.na = param->na;
	axc_param.map_method = param->map_method;
	size = calculate_axc_size(&axc_param);
	axc_size = size;
	if ((axc_size <= 2) || (axc_size < param->ns)) {
		dev_err(dev, "axc_size < min_axc_size value axc_size: %d\n",
				 axc_size);
		goto validation_err;
	}

	if (axc_size > (word_size * (BF_WRDS - 1))) {
		dev_err(dev, "axc_size > max_axc_size axc_size: %d\n",
				axc_size);
		goto validation_err;
	}

	for (loop = 0; loop < param->K; loop++) {
		k0_ptr = map_table->k0_bitmap[loop];
		k1_ptr = map_table->k1_bitmap[loop];
		bf_map[0] = k0_ptr[0];
		bf_map[1] = k0_ptr[1];
		bf_map[0] = k1_ptr[0];
		bf_map[1] = k1_ptr[1];
		axc_size = size;
		if (flag & AXC_FLEXI_POSITION_EN)
			axc_pos = (axc_pos + loop);
		seg = get_segment_id(axc_pos, word_size);
		bit_position = BIT_POS(axc_pos->axc_start_w, word_size,
					axc_pos->axc_start_b);
		seg = seg + (loop * seg_in_one_basic_frame);
		segment = segments + seg;
		if (segment->k == k0) {
			if ((k0 % param->K) != 0) {
				dev_err(dev, "Kval k0/k1-%d/%d seg: %d\n",
				k0, k1, seg);
				goto validation_err;
			}
		} else if (segment->k == k1) {
			if ((k1 % param->K) != 0) {
				dev_err(dev, "%s -line: %d K not multiple\n",
				__func__, __LINE__);
				goto validation_err;
			}
		}

		/* check if segment allready occupied */
		if ((segment->k == k1) ||
				(segment->k == k0)) {
			bf_map_val = bf_map[K_OFFSET(seg)];
			kpos = K_POS(seg);
			if ((bf_map_val >> kpos) & 0x1) {
				bit_set = bit_position;
				while ((SEG_SIZE - bit_set)) {
					if (((*(segment_bitmap + seg) >>
						bit_set) & 0x1)) {
						dev_err(dev, "%s -line: %d\n",
							__func__, __LINE__);
						goto validation_err;
					}
					bit_set++;
				}
			}
		}

		if (axc_size > (SEG_SIZE - bit_position)) {
			seg += 1;
			bit_position = 0;
			axc_size -= (SEG_SIZE - bit_position);
		} else
			axc_size = 0;
		/* check if sufficiently large slot available */
		while (axc_size) {
			if (axc_size > (bit_position + SEG_SIZE)) {
				bf_map_val = bf_map[K_OFFSET(seg)];
				kpos = K_POS(seg);
				if ((bf_map_val >> kpos) & 0x1) {
					dev_err(dev, "bitmap: 0x%x, seg: %d\n",
							bf_map_val, seg);
					goto validation_err;
				}
				axc_size -= SEG_SIZE;
				seg += 1;
			} else {
				bit_set = 0;
				while (axc_size) {
					if (((*(segment_bitmap + seg) >>
							bit_set) & 0x1)) {
						dev_err(dev, "seg - %d\n",
							seg);
						goto validation_err;
					}
					bit_set++;
					axc_size--;
				}
			}
		}
		/* no need validate for all k's in case
		 * of fixed position
		*/
	}
	return 0;
validation_err:
	dev_err(dev, "%s -line: %d validation error\n",  __func__, __LINE__);
	return -EINVAL;
}

void set_segment_map(struct segment_param *seg_param)
{
	unsigned int subseg_num = 0;
	struct subsegment *subsegment;
	unsigned char k;
	unsigned int loop;
	struct segment  *segment = seg_param->segment;
	struct axc *axc = seg_param->axc;
	unsigned char k0 = axc->framer->framer_param.k0;
	unsigned char k1 = axc->framer->framer_param.k1;
	unsigned char cmd = seg_param->cmd;
	unsigned int bit_position = seg_param->bit_position;
	unsigned int axc_size = seg_param->axc_size;
	unsigned int num_seg = 1;
	unsigned int map_size = 1;

	if (!(k0 % axc->K))
		k = k0;
	else
		k = k1;
	segment->k = k;

	if (bit_position < SEG0_OFFSET)
		subseg_num = 0;
	else if (bit_position < SEG1_OFFSET)
		subseg_num = 1;
	else
		subseg_num = 2;

	subsegment = &segment->subsegments[subseg_num];

	if (subseg_num == 0) {
		subsegment->offset = (cmd) ? bit_position : 0;
		map_size = ((SEG_SIZE - bit_position) < axc_size) ?
			(SEG_SIZE - bit_position) : axc_size;
		subsegment->map_size = (cmd) ? map_size : 0;
		subsegment->axc = (cmd) ? axc : NULL;
		if (axc_size <= (SEG_SIZE - bit_position))
			goto mapping_done;
		axc_size = axc_size - (SEG_SIZE - bit_position);
	} else if (subseg_num == 1) {
		bit_position = bit_position - SEG0_OFFSET;
		subsegment->offset = (cmd) ? bit_position : 0;
		map_size = ((22 - bit_position) < axc_size) ?
			(22 - bit_position) : axc_size;
		subsegment->map_size = (cmd) ? map_size : 0;
		subsegment->axc = (cmd) ? axc : NULL;
		if (axc_size < (22 - bit_position))
			goto mapping_done;
		axc_size = axc_size - (22 - bit_position);
	} else {
		bit_position = bit_position - SEG1_OFFSET;
		subsegment->offset = (cmd) ? bit_position : 0;
		map_size = (((12 - bit_position) < axc_size) ?
			(12 - bit_position) : axc_size);
		subsegment->map_size = (cmd) ? map_size : 0;
		subsegment->axc = (cmd) ? axc : NULL;
		if (axc_size < (12 - bit_position))
			goto mapping_done;
		axc_size = axc_size - (12 - bit_position);
	}


	segment = (segment + 1);
	num_seg = CEIL_FUNC(axc_size, SEG_SIZE);

	for (loop = 0; loop < num_seg; loop++) {
		segment->k = (cmd) ? k : 0;
		subseg_num = 0;
		subsegment = &segment->subsegments[subseg_num];
		subsegment->offset = 0;
		subsegment->axc = (cmd) ? axc : NULL;
		if (axc_size < (2 * SEG_SIZE))
			map_size = axc_size;
		else
			map_size = (SEG_SIZE < axc_size) ? SEG_SIZE : axc_size;
		subsegment->map_size = (cmd) ? map_size : 0;
		if ((axc_size - map_size) <= 0)
			break;
		axc_size = axc_size - SEG_SIZE;
		segment = (segment + 1);
	}
mapping_done:
	return;
}

void set_segment_map_with_stuffing(struct segment_param *seg_param)
{
	unsigned int axc_size1 = 0;
	unsigned int axc_size2 = 0;
	unsigned char bit_pos1 = 0;
	unsigned char bit_pos2 = 0;
	struct segment  *segment_1 = NULL;
	struct segment  *segment_2 = NULL;
	unsigned char k;
	struct segment  *segment = seg_param->segment;
	struct axc *axc = seg_param->axc;
	unsigned char k0 = axc->framer->framer_param.k0;
	unsigned char k1 = axc->framer->framer_param.k1;
	unsigned int bit_position = seg_param->bit_position;
	unsigned int axc_size = seg_param->axc_size;
	unsigned int ki = seg_param->ki;
	unsigned char nst_len = 2 * axc->sampling_width;


	if (!(k0 % axc->K))
		k = k0;
	else
		k = k1;

	axc_size = seg_param->axc_size;
	segment->k = k;
	if (axc->map_method == MAPPING_METHOD_3)
		nst_len = 2 * axc->sampling_width;
	else
		nst_len = axc->nst;

	if (bit_position < ki) {
		axc_size1 = ki;
		segment_1 = segment;
		bit_pos1 = bit_position;
		axc_size2 = axc_size - (ki + nst_len);
		segment_2 = (segment_1 + ((ki + nst_len) / SEG_SIZE));
		bit_pos2 = (ki + nst_len) % SEG_SIZE;

	} else {
		axc_size1 = axc_size - nst_len;
		segment_1 = (segment + nst_len / SEG_SIZE);
		bit_pos1 = (bit_position + nst_len) % SEG_SIZE;
		axc_size2 = 0;
		segment_2 = NULL;
		bit_pos2 = 0;
	}

	if (axc_size1) {
		seg_param->segment = segment_1;
		seg_param->axc = axc;
		seg_param->bit_position = bit_pos1;
		seg_param->axc_size = axc_size1;
		set_segment_map(seg_param);
	}

	if (axc_size2) {
		seg_param->segment = segment_2;
		seg_param->axc = axc;
		seg_param->bit_position = bit_pos2;
		seg_param->axc_size = axc_size2;
		set_segment_map(seg_param);
	}

}

void map_table_struct_entry_ctrl(struct axc *axc, unsigned char cmd)
{
	struct device *dev = axc->framer->cpri_dev->dev;
	unsigned char k0 = axc->framer->framer_param.k0;
	unsigned char k1 = axc->framer->framer_param.k1;
	unsigned char k;
	unsigned int seg;
	unsigned int seg_offset;
	unsigned int loop;
	unsigned int pos_loop;
	struct cpri_framer *framer = axc->framer;
	unsigned int word_size = framer->autoneg_output.cpri_bf_word_size;
	unsigned short seg_in_one_basic_frame;
	unsigned int usable_seg_in_one_basic_frame;
	struct axc_map_table *map_table;
	unsigned int axc_size;
	unsigned int size;
	unsigned int bit_position;
	struct segment  *segment;
	struct axc_pos *axc_pos;
	struct segment_param seg_param;
	unsigned int loop1;
	u32 *segment_bitmap;
	u32 *sbitmap;
	u32 *k_map;
	u32 val;
	u32 bit_set;
	unsigned int k_max;
	u32 bitmap;
	unsigned int ki = 0;
	unsigned int nst_flag;
	unsigned nc;
	u8 kf;
	u32 *k_ptr;

	seg_in_one_basic_frame = (word_size * BF_WRDS) / SEG_SIZE;
	usable_seg_in_one_basic_frame =
		framer->autoneg_output.cpri_bf_iq_datablock_size / SEG_SIZE;
	dev_dbg(dev, "%s total seg: %d usable seg: %d\n", __func__,
			seg_in_one_basic_frame, usable_seg_in_one_basic_frame);
	bit_set = 0;
	bitmap = 0;
	/* check for uplink or downling */
	if (axc->flags & DL_AXCS)
		map_table = &framer->dl_map_table;
	else
		map_table = &framer->ul_map_table;

	segment_bitmap = map_table->segment_bitmap;
	axc_pos = axc->pos;
	k = axc->K;
	size = calculate_axc_size(axc);
	if (!(k0 % k))
			k_max = k0;
	else
		k_max = k1;

	for (loop = 0; loop < k_max; loop++) {
		ki = 0;
		axc_size = size;
		pos_loop = k_max % axc->K;
		if (axc->flags & AXC_FLEXI_POSITION_EN)
			axc_pos = (axc_pos + pos_loop);
		seg = get_segment_id(axc_pos, word_size);
		dev_dbg(dev, " word: %d, bit position: %d",
				axc_pos->axc_start_w, axc_pos->axc_start_b);
		bit_position = BIT_POS(axc_pos->axc_start_w, word_size,
					axc_pos->axc_start_b);
		dev_dbg(dev, "word_sz: %d, seg: %d, axc_sz: %d, bit_pos : %d",
				word_size, seg,	axc_size, bit_position);
		seg = seg + (loop * seg_in_one_basic_frame);
		seg_offset = seg;
		if (axc_size <= (SEG_SIZE - bit_position))
			bit_set = axc_size - 1;
		else
			bit_set = SEG_SIZE - bit_position;

		/* update the bitmap flag for first seg with bitposition offset
		 */
		while (bit_set <= 0)
			bitmap |= (0x1 << bit_set--);

		bitmap = bitmap << bit_position;
		sbitmap = (segment_bitmap + seg);
		OPR_BITMAP(sbitmap, bitmap, cmd);

		if (!(k0 % axc->K)) {
			kf = K_OFFSET(seg);
			k_ptr = map_table->k0_bitmap[loop];
			k_map = &k_ptr[kf];
			val = K_MASK(seg);
			OPR_BITMAP(k_map, val, cmd);
			k = k0;
			if (map_table->k0_max < axc->K)
				map_table->k0_max = axc->K;
		} else {
			kf = K_OFFSET(seg);
			k_ptr = map_table->k1_bitmap[loop];
			k_map = &k_ptr[kf];
			val = K_MASK(seg);
			OPR_BITMAP(k_map, val, cmd);
			k = k1;
			if (map_table->k1_max < axc->K)
				map_table->k1_max = axc->K;
		}
		/* update the bitmap flag for rest of segments
		 */
		if (axc_size > (SEG_SIZE - bit_position)) {
			bit_set = axc_size - (SEG_SIZE - bit_position);
			bitmap = 0;
			if (bitmap >= SEG_SIZE)
				bitmap = 0xffffffff;
			else
				while (bit_set <= 0)
					bitmap |= (0x1 << bit_set--);

			for (loop1 = 0; loop1 < bit_set; loop1++) {
				bitmap |= (0x1 << loop1);
				if (!(loop1 % SEG_SIZE)) {
					seg += 1;
					if (k0 == k) {
						kf = K_OFFSET(seg);
						k_ptr =
						map_table->k0_bitmap[loop];
						k_map = &k_ptr[kf];
						val = K_MASK(seg);
						OPR_BITMAP(k_map, val, cmd);
					} else {
						kf = K_OFFSET(seg);
						k_ptr =
						map_table->k1_bitmap[loop];
						k_map = &k_ptr[kf];
						val = K_MASK(seg);
						OPR_BITMAP(k_map, val, cmd);
					}
					sbitmap = (segment_bitmap + seg);
					OPR_BITMAP(sbitmap, bitmap, cmd);
					bitmap = 0;
				}
			}
		}

		nst_flag = 0;
		/* fill segment struct parameter */
		segment = (map_table->segments + seg_offset);
		/* here bug can be if ki != 0 and stuffing bit size is
		*  smaller than end of segment than there is no way
		*  where we can program same subseg 2ce
		*/

		if (axc->nst) {
			if ((axc->map_method == MAPPING_METHOD_3) &&
					(!(loop % axc->K) ||
					 (axc->nst < loop))) {
				ki = calculate_nst_position(axc,
					loop);
				nst_flag = 1;
				nc = CEIL_FUNC((axc->S * axc->na), axc->K);
				ki = (ki % nc);
				ki = ki * 2 * axc->sampling_width;
				ki = k + bit_position;
			} else if (!(loop % axc->K)) {
				nst_flag = 1;
				ki = bit_position;
			}
		}
		seg_param.segment = segment;
		seg_param.axc = axc;
		seg_param.bit_position = bit_position;
		seg_param.cmd = cmd;
		seg_param.axc_size = size;
		seg_param.ki = ki;
		if (nst_flag)
			set_segment_map_with_stuffing(&seg_param);
		else
			set_segment_map(&seg_param);
	}
	return;
}

void program_axc_conf_reg(struct axc *axc)
{
	struct cpri_framer_regs __iomem *regs = axc->framer->regs;
	u32 *reg_acpr;
	u32 *reg_acprmsb;
	u32 *cpri_map_smpl_cfg;
	u32 val;

	if (axc->flags & UL_AXCS) {
		reg_acpr = &regs->cpri_tacpr[axc->id];
		reg_acprmsb = &regs->cpri_tacprmsb[axc->id];
		cpri_map_smpl_cfg = &regs->cpri_map_smpl_cfg_tx;

	} else {
		reg_acpr = &regs->cpri_racpr[axc->id];
		reg_acprmsb = &regs->cpri_racprmsb[axc->id];
		cpri_map_smpl_cfg = &regs->cpri_map_smpl_cfg_rx;
	}

	val = ((axc->id << AXC_CONF_SHIFT) | AXC_CONF_MASK |
		axc->sampling_width);
	/* axc parameter configuration */
	cpri_reg_set_val(cpri_map_smpl_cfg,
			AXC_SMPL_WDTH_MASK, val);

	cpri_reg_set_val(reg_acpr,
			AXC_SIZE_MASK,
			(axc->axc_buf->size - 1));
	if (axc->axc_buf->mem_blk == 0)
		cpri_reg_clear(reg_acpr,
				AXC_MEM_BLK_MASK);
	else
		cpri_reg_set(reg_acpr,
				AXC_MEM_BLK_MASK);

	cpri_reg_set_val(reg_acpr,
			AXC_BASE_ADDR_MASK,
			axc->axc_buf->addr);

	/* set format param etc */
	if (axc->flags & AXC_IQ_FORMAT_2)
		cpri_reg_set(
				reg_acprmsb,
				AXC_IQ_FORMAT_2);
	else
		cpri_reg_clear(
				reg_acprmsb,
				AXC_IQ_FORMAT_2);

	if (axc->flags & AXC_INTERLEAVING_EN)
		cpri_reg_set(reg_acprmsb,
				AXC_INTERLEAVING_EN);
	else
		cpri_reg_clear(reg_acprmsb,
				AXC_INTERLEAVING_EN);
	if (axc->flags & UL_AXCS) {
		if (axc->flags & AXC_TX_ROUNDING_EN)
			cpri_reg_set(reg_acprmsb,
					AXC_TX_ROUNDING_EN);
		else
			cpri_reg_clear(reg_acprmsb,
					AXC_TX_ROUNDING_EN);
	} else {
		if (axc->flags & AXC_CONVERSION_9E2_EN)
			cpri_reg_set(reg_acprmsb,
					AXC_CONVERSION_9E2_EN);
		else
			cpri_reg_clear(reg_acprmsb,
					AXC_CONVERSION_9E2_EN);
	}
	if (axc->flags & AXC_OVERSAMPLING_2X)
		cpri_reg_set(reg_acprmsb,
				AXC_OVERSAMPLING_2X);
	else
		cpri_reg_clear(reg_acprmsb,
				AXC_OVERSAMPLING_2X);

	/* sample width and thresold paramter setting */
	cpri_reg_set_val(reg_acprmsb,
			AXC_SW_MASK,
			axc->sampling_width);
	cpri_reg_set_val(
			reg_acprmsb,
			AXC_TH_MASK,
			axc->buffer_threshold);
}

int fill_axc_param(struct cpri_framer *framer, struct axc_info *axc_parm,
		struct axc *axc, u32 flag)
{
	axc->id = axc_parm->id;
	axc->flags = axc_parm->flags;
	if (flag & DL_AXCS)
		axc->flags |= DL_AXCS;
	else
		axc->flags |= UL_AXCS;
	axc->framer = framer;
	axc->map_method = axc_parm->map_method;
	axc->pos = axc_parm->pos;
	axc->sampling_width = axc_parm->sample_width;
	axc->buffer_threshold = axc_parm->buffer_threshold;
	axc->S = axc_parm->S;
	axc->K = axc_parm->K;
	axc->na = axc_parm->na;
	axc->nst = axc_parm->ns;
	axc->axc_buf = axc_alloc(axc, axc_parm->buffer_size);
	if (axc->axc_buf == NULL)
			return -ENOMEM;
	return 0;
}

void clean_curr_allocate_axc(struct axc **axcs, u32 axc_pos)
{
	int temp = 0;
	struct axc *axc;

	while ((axc_pos & NO_AXC_SET)) {
		if ((axc_pos >> temp) & 0x1) {
			axc = axcs[temp];
			axcs[temp] = NULL;
			axc_free(axc->axc_buf);
			kfree(axc->pos);
			kfree(axc);
			axc_pos &= ~(0x1 << temp);
		}
		temp++;
	}
	return;
}

int cpri_axc_param_set(struct cpri_framer *framer, unsigned long arg_ioctl)
{
	unsigned int size;
	unsigned int loop;
	struct axc_config_params *arg;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int count;
	u32 flag;
	struct axc_info *axc_param;
	struct axc_info *param;
	struct axc **axcs;
	struct axc *axc;
	struct axc_pos *pos;
	struct axc_map_table *map_table;
	u32 axc_pos = 0;
	unsigned char k0 = framer->framer_param.k0;
	unsigned char k1 = framer->framer_param.k1;
	int ret = 0;


	arg = kzalloc(sizeof(struct axc_config_params), GFP_KERNEL);
	if (arg == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	if (copy_from_user((void *)arg,
				(struct axc_config_params *)arg_ioctl,
				sizeof(struct axc_config_params))) {
		kfree(arg);
		dev_err(dev, "copy from user failed\n");
		return -EFAULT;
	}
	count = arg->axc_count;
	flag = arg->flags;
	size = count * sizeof(struct axc_info);
	axc_param = kzalloc(size, GFP_KERNEL);
	if (axc_param == NULL) {
		dev_err(dev, "System memory exhausted !! kzalloc fail\n");
		return -ENOMEM;
	}
	if (copy_from_user((void *)axc_param, arg->axcs, size)) {
		kfree(axc_param);
		kfree(arg);
		dev_err(dev, "copy from user failed\n");
		return -EFAULT;
	}

	/* axc allocation */
	if (flag & DL_AXCS) {
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
	} else {
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
	}

	for (loop = 0; loop < count; loop++) {
		pos = NULL;
		param = (axc_param + loop);
		if (param->flags & AXC_FLEXI_POSITION_EN)
			size = (param->K * sizeof(struct axc_pos));
		else
			size = sizeof(struct axc_pos);
		pos = kzalloc(size, GFP_KERNEL);
		if (axc_param == NULL) {
			dev_err(dev, "memory exhausted !!\n");
			return -ENOMEM;
		}
		if (copy_from_user((void *)pos, param->pos, size)) {
			kfree(pos);
			dev_err(dev, "copy from user failed\n");
			ret = -EFAULT;
			goto mem_err;
		}
		param->pos = pos;

		if (!((!(k0 % param->K)) | (!(k1 % param->K)))) {
			dev_err(dev, "axc-'K' must be multpl of K0/K1-%d/%d\n",
					k0, k1);
			ret = -EINVAL;
			goto validation_err;
		}
		if (!((param->map_method == MAPPING_METHOD_1) ||
				(param->map_method == MAPPING_METHOD_3))) {
			dev_err(dev, "supported mapping methods 1 and 3!!\n");
			ret = -EINVAL;
			goto validation_err;
		}


		if (axc_validate(framer, param, count, flag) != 0) {
			dev_err(dev, "axc info parameter error\n");
			kfree(axc_param);
			ret = -EINVAL;
			goto clean_axc_pos;
		}
		/* allocate axc and map with values and axc_buff */
		axc = kzalloc(sizeof(struct axc), GFP_KERNEL);
		if (axc == NULL) {
			dev_err(dev, "memory exhausted !!\n");
			ret = -ENOMEM;
			goto clean_axc_pos;
		}
		if (fill_axc_param(framer, param, axc, flag) != 0) {
			dev_err(dev, "error while setting axc param\n");
			kfree(axc->pos);
			kfree(axc);
			kfree(axc_param);
			/* free allocated buffers in dl_axc_list */
			ret = -ENOMEM;
			goto clean_axc_pos;
		}
		/* program axc conf register */
		map_table_struct_entry_ctrl(axc, SET_CMD);
		axc_pos |= (0x1 << axc->id);
		axcs[axc->id] = axc;

	}
	/* axc allocation done */
	kfree(axc_param);
	cpri_state_machine(framer, CPRI_STATE_AXC_CONFIG);
	return 0;
clean_axc_pos:
	dev_err(dev, "axc set error free axc_pos\n");
	clean_curr_allocate_axc(axcs, axc_pos);
validation_err:
	return ret;
mem_err:
	clean_curr_allocate_axc(axcs, axc_pos);
	return ret;
}

int clear_segment_param(struct segment *segment, struct axc *axc,
		u32 *reg_rcmd0)
{
	unsigned char subsegloop;
	unsigned char shift;
	unsigned int bit_mapped = 0;
	struct subsegment *subsegment;
	struct cpri_framer *framer;
	u32 *reg_rcmd1 = (reg_rcmd0 + sizeof(u32));
	u32 position;
	u32 axc_num;
	u32 tcm_enable;
	u32 tcm_width;

	framer = axc->framer;
	for (subsegloop = 0; subsegloop < 3; subsegloop++) {
		subsegment = &segment->subsegments[subsegloop];
		if (subsegment->axc == NULL)
			continue;
		if (subsegment->axc->id == axc->id) {
			if (subsegloop < 2)
				shift = subsegloop;
			else
				shift = 0;
			position = AXC_POS_MASK << (6 +	(shift * BF_WRDS));
			tcm_width = AXC_WIDTH_MASK << (11 + (shift * BF_WRDS));
			axc_num = AXC_NUM_MASK << (1 + (shift * BF_WRDS));
			tcm_enable = AXC_MEM_ENABLE_MASK << (0 +
					(shift * BF_WRDS));

			if (subsegloop == 2) {
				cpri_reg_clear(
						reg_rcmd1, tcm_enable);
				cpri_reg_set_val(
						reg_rcmd1, position, 0);
				cpri_reg_set_val(
						reg_rcmd1, tcm_width, 0);
				cpri_reg_set_val(
						reg_rcmd1, axc_num, 0);
			} else {
				cpri_reg_clear(
						reg_rcmd0, tcm_enable);
				cpri_reg_set_val(
						reg_rcmd0, position, 0);
				cpri_reg_set_val(
						reg_rcmd0, tcm_width, 0);
				cpri_reg_set_val(
						reg_rcmd0, axc_num, 0);
			}
			bit_mapped += subsegment->map_size;
		}
	}
	return bit_mapped;
}

void clear_axc_param(struct axc *axc)
{
	struct cpri_framer_regs __iomem *regs = axc->framer->regs;
	u32 *reg_acpr;
	u32 *reg_acprmsb;

	/* disable rx tx axc id for configuration */
	if (axc->flags & UL_AXCS) {
		reg_acpr = &regs->cpri_tacpr[axc->id];
		reg_acprmsb = &regs->cpri_tacprmsb[axc->id];

	} else {
		reg_acpr = &regs->cpri_racpr[axc->id];
		reg_acprmsb = &regs->cpri_racprmsb[axc->id];
	}

	/* axc parameter configuration */
	cpri_reg_set_val(reg_acpr, AXC_SIZE_MASK, 0);
	cpri_reg_clear(&reg_acpr, AXC_MEM_BLK_MASK);
	cpri_reg_set_val(reg_acpr, AXC_BASE_ADDR_MASK, 0);
	/* sample width and thresold paramter setting */
	cpri_reg_set_val(reg_acprmsb, AXC_SW_MASK, 0);
	cpri_reg_set_val(reg_acprmsb, AXC_TH_MASK, 0);
	axc_free(axc->axc_buf);
	return;
}

void delete_axc(struct cpri_framer *framer, unsigned char axc_id,
		unsigned int direction)
{
	struct device *dev = framer->cpri_dev->dev;
	unsigned int axc_size;
	unsigned int size;
	unsigned int seg;
	struct axc *axc;
	struct axc_map_table *map_table;
	struct segment *segment;
	unsigned int seg_in_one_basic_frame;
	unsigned int bit_mapped = 0;
	unsigned int loop;
	unsigned int word_size = framer->autoneg_output.cpri_bf_word_size;
	u32 *reg_cma;
	u32 *reg_rcmd0;
	u32 *reg_rcmd1;
	u32 *reg_cr;
	struct cpri_framer_regs __iomem *regs = framer->regs;

	seg_in_one_basic_frame = (word_size * BF_WRDS) / SEG_SIZE;
	if (direction & UL_AXCS) {
		axc = framer->ul_axcs[axc_id];
		map_table = &framer->ul_map_table;
		reg_cma = &regs->cpri_tcma;
		reg_rcmd0 = &regs->cpri_tcmd0;
		reg_rcmd1 = &regs->cpri_tcmd1;
		reg_cr = &regs->cpri_tcr;
	} else {
		axc = framer->dl_axcs[axc_id];
		map_table = &framer->dl_map_table;
		reg_cma = &regs->cpri_rcma;
		reg_rcmd0 = &regs->cpri_rcmd0;
		reg_rcmd1 = &regs->cpri_rcmd1;
		reg_cr = &regs->cpri_rcr;
	}
	if (axc == NULL)
		dev_err(dev, "axc delete failed 'axc %d not configured'\n",
				axc_id);
	size = calculate_axc_size(axc);
	for (loop = 0; loop < axc->K; loop++) {
		axc_size = size;
		seg = get_segment_id(axc->pos, word_size);
		seg = seg + (loop * seg_in_one_basic_frame);
		while (axc_size) {
			segment = (map_table->segments + seg);
			cpri_reg_clear(
					reg_cma, AXC_TBL_WRITE_MASK);
			bit_mapped = clear_segment_param(segment, axc,
					reg_rcmd0);
			cpri_reg_set_val(reg_cma,
					AXC_TBL_SEG_ADDR_MASK, seg);
			cpri_reg_set(reg_cma,
					AXC_TBL_WRITE_MASK);

			axc_size -= bit_mapped;
			if ((axc_size) && (axc_size > SEG0_OFFSET))
				seg++;
			else
				axc_size = 0;
		}
	}
	map_table_struct_entry_ctrl(axc, CLEAR_CMD);
	clear_axc_param(axc);
	if (direction & UL_AXCS)
		framer->ul_axcs[axc_id] = NULL;
	else
		framer->dl_axcs[axc_id] = NULL;
	kfree(axc->pos);
	kfree(axc);
	return;
}

int cpri_axc_param_ctrl(struct cpri_framer *framer, unsigned long arg)
{
	struct cpri_axc_ctrl *ctrl;
	unsigned int size;
	u32 *reg_accr;

	size = sizeof(struct cpri_axc_ctrl);
	ctrl = kmalloc(size, GFP_KERNEL);
	if (copy_from_user((void *)ctrl, (struct cpri_axc_ctrl *)arg, size)) {
		kfree(ctrl);
		return -EFAULT;
	}
	if ((ctrl->direction & UL_AXCS))
		reg_accr = &framer->regs->cpri_taccr;
	else
		reg_accr = &framer->regs->cpri_raccr;

	switch (ctrl->op) {
	case AXC_ENABLE:
		cpri_reg_set(reg_accr,
				(AXC_ENABLE_MASK << ctrl->axc_id));

		break;
	case AXC_DISABLE:
		cpri_reg_clear(reg_accr,
				(AXC_ENABLE_MASK << ctrl->axc_id));
		break;
	case AXC_DELETE:
		cpri_reg_clear(reg_accr,
				(AXC_ENABLE_MASK << ctrl->axc_id));
		delete_axc(framer, ctrl->axc_id, ctrl->direction);
		break;
	}
	return 0;
}

int segment_param_set(struct segment *segment, struct axc *axc,
		u32 *reg_rcmd0)
{
	unsigned char subsegloop;
	unsigned char shift;
	unsigned int bit_mapped = 0;
	unsigned int axc_size;
	int map_width = 0;
	struct subsegment *subsegment;
	struct cpri_framer *framer = axc->framer;
	struct device *dev = framer->cpri_dev->dev;
	u32 position;
	u32 axc_num;
	u32 tcm_enable;
	u32 tcm_width;
	u32 *reg_rcmd1 = (reg_rcmd0 + sizeof(u32));

	axc_size = calculate_axc_size(axc);

	for (subsegloop = 0; subsegloop < 3; subsegloop++) {
		subsegment = &segment->subsegments[subsegloop];
		if (subsegment->axc == NULL)
			continue;
		if (subsegment->axc->id != axc->id)
			continue;

		if (subsegloop < 2)
			shift = subsegloop;
		else
			shift = 0;

		position = AXC_POS_MASK << (6 + (shift * BF_WRDS));
		tcm_width = AXC_WIDTH_MASK << (11 + (shift * BF_WRDS));
		axc_num = AXC_NUM_MASK << (1 + (shift * BF_WRDS));
		tcm_enable = AXC_MEM_ENABLE_MASK << (shift * BF_WRDS);

		bit_mapped += subsegment->map_size;
		dev_dbg(dev, "axcsize: %d bit_mapped: %d subseg->mapsize: %d",
				axc_size, bit_mapped, subsegment->map_size);
		axc_size -= bit_mapped;
		map_width = subsegment->map_size;

		if (subsegloop == 2) {
			cpri_reg_clear(reg_rcmd1, MASK_ALL);
			cpri_reg_clear(reg_rcmd0, MASK_ALL);
			dev_dbg(dev, "rcmd1 -> subsegNum[%d], sbseg_offst: %d",
					subsegloop, subsegment->offset);
			dev_dbg(dev, "tcm_enable: 0x%x, pos[0x%x] - 0x%x\n",
					tcm_enable, position,
					(subsegment->offset / 2));
			dev_dbg(dev, "tcm_width[0x%x]- %d, axc_num[0x%x]- %d\n",
					tcm_width, (map_width / 2),
					axc_num, axc->id);
			cpri_reg_set_val(reg_rcmd1, position,
				(subsegment->offset / 2));
			cpri_reg_set_val(reg_rcmd1, tcm_width,
				(map_width / 2));
			cpri_reg_set_val(reg_rcmd1, axc_num, axc->id);
			break;
		} else {
			cpri_reg_clear(reg_rcmd0, MASK_ALL);
			cpri_reg_clear(reg_rcmd1, MASK_ALL);
			dev_dbg(dev, "rcmd0 -> subsegNum[%d], sbseg_offst: %d",
					subsegloop, subsegment->offset);
			dev_dbg(dev, "pos[0x%x] - 0x%x\n",
					position, (subsegment->offset / 2));
			dev_dbg(dev, "tcm_width[0x%x]- %d, axc_num[0x%x]- %d\n",
					tcm_width, (map_width / 2),
					axc_num, axc->id);

			cpri_reg_set_val(reg_rcmd0, position,
				(subsegment->offset / 2));
			cpri_reg_set_val(reg_rcmd0, tcm_width,
				(map_width / 2));
			cpri_reg_set_val(reg_rcmd0, axc_num, axc->id);
			cpri_reg_set(reg_rcmd0, tcm_enable);
			break;
		}
	}
	return bit_mapped;
}

/* While k value overlap check, we check whether k0/k1 map have k0/k1 reserved
 * in continuation or not if not in continuation then send error. 2ndly we
 * check whether k0 and k1 overlap if overlap send error. If both test pass then
 * this api will return validation is success '0'
 */

int check_for_kval_overlap(struct cpri_framer *framer, unsigned int flag)
{
	struct device *dev = framer->cpri_dev->dev;
	int loop;
	int k_val;
	int mask;
	int mask1;
	struct axc_map_table *map_table;
	u32 cnt = 0;
	u32 cnt1 = 0;
	u32 k0_map[2] = { 0 };
	u32 k1_map[2] = { 0 };
	u32 *cpri_map_k_select1;
	u32 *cpri_map_k_select2;
	unsigned int k0_k1_max;

	k0_k1_max = (framer->framer_param.k0 > framer->framer_param.k1) ?
	framer->framer_param.k0 : framer->framer_param.k1;

	if (flag & UL_AXCS) {
		map_table = &framer->dl_map_table;
		cpri_map_k_select1 = &framer->regs->cpri_map_k_select_rx1;
		cpri_map_k_select2 = &framer->regs->cpri_map_k_select_rx2;
	} else {
		map_table = &framer->ul_map_table;
		cpri_map_k_select1 = &framer->regs->cpri_map_k_select_tx1;
		cpri_map_k_select2 = &framer->regs->cpri_map_k_select_tx2;
	}

	for (loop = 0; loop < k0_k1_max; loop++) {
		memset(k0_map, 0, 2 * sizeof(u32));
		memset(k1_map, 0, 2 * sizeof(u32));
		k0_map[0] = (map_table->k0_bitmap[loop])[0];
		k0_map[1] = (map_table->k0_bitmap[loop])[1];
		k1_map[0] = (map_table->k1_bitmap[loop])[0];
		k1_map[1] = (map_table->k1_bitmap[loop])[1];

		if ((k0_map[0] & k1_map[0]) || (k0_map[1] & k1_map[1])) {
			dev_info(dev, "k0[0]:%d, k0[1]:%d, k1[0]:%d, k1[1]:%d",
				k0_map[0], k0_map[1], k1_map[0], k1_map[1]);
			return -EINVAL;
		}

		if (k0_map[0] && 1)
			cnt = ffs(k0_map[0]);
		else if (k0_map[1] && 1)
			cnt = ffs(k0_map[1]) + SEG_SIZE;

		if (k1_map[0] && 1)
			cnt1 = ffs(k1_map[0]);
		else if (k1_map[1] && 1)
			cnt1 = ffs(k1_map[1]) + SEG_SIZE;

		if (cnt1 > cnt) {
			if (cnt1 <= SEG_SIZE) {
				/* subtracting 2 to get valid mask */
				k_val = cnt1 - 2;
				mask = 0;
				if (k_val) {
					while (k_val >= 0) {
						mask |= (0x1 << k_val);
						k_val--;
					}
				}
				if ((k0_map[0] & (~mask)) || (k0_map[1] && 1))
					goto INVAL;
			} else {
				mask = 0;
				k_val = (cnt1 - SEG_SIZE) - 2;
				if (k_val) {
					while (k_val >= 0) {
						mask |= (0x1 << k_val);
						k_val--;
					}
				}
				if (k0_map[1] & (~mask))
					goto INVAL;
			}
		} else {
			if (cnt <= SEG_SIZE) {
				k_val = cnt - 2;
				mask = 0;
				if (k_val) {
					while (k_val >= 0) {
						mask |= (0x1 << k_val);
						k_val--;
					}
				}
				if ((k1_map[0] & (~mask)) || (k1_map[1] && 1))
					goto INVAL;
			} else {
				mask = 0;
				k_val = (cnt - SEG_SIZE) - 2;
				if (k_val) {
					while (k_val >= 0) {
						mask1 |= (0x1 << k_val);
						k_val--;
					}
				}
				if (k1_map[1] & (~mask))
					goto INVAL;
			}
		}
		memset(k0_map, 0, 2 * sizeof(u32));
		memset(k1_map, 0, 2 * sizeof(u32));
	}


	cpri_reg_clear(cpri_map_k_select1,
			(map_table->k0_bitmap[0])[0]);
	cpri_reg_set(cpri_map_k_select1,
			(map_table->k1_bitmap[0])[0]);

	cpri_reg_clear(cpri_map_k_select2,
			(map_table->k0_bitmap[0])[1]);
	cpri_reg_set(cpri_map_k_select2,
			(map_table->k1_bitmap[0])[1]);

		return 0;
INVAL:
	dev_info(dev, "k0[0]/k1[0]:0x%x/0x%x, k0[1].k1[1]:0x%x/0x%x, msk:0x%x",
		k0_map[0], k1_map[0], k0_map[1], k1_map[1], mask);
	return -EINVAL;

}

static inline unsigned short get_nst_size(struct axc *axc, unsigned int loop)
{
	if ((axc->map_method == MAPPING_METHOD_3) &&
			(axc->nst < loop))
		return 2 * axc->sampling_width;
	else if (!loop)
		return axc->nst;
	return 0;
}


int cpri_axc_map_tbl_init(struct cpri_framer *framer, unsigned long direction)
{
	struct device *dev = framer->cpri_dev->dev;
	u32 *cpri_status;
	unsigned int loop;
	unsigned int k_loop;
	unsigned int loop2;
	unsigned int k;
	unsigned int k_max;
	unsigned int seg;
	unsigned int axc_seg;
	unsigned int map_seg;
	struct axc **axcs;
	struct axc *axc;
	struct axc_pos *axc_pos;
	int axc_size;
	int axc_size_retained;
	unsigned int bit_mapped;
	struct axc_map_table *map_table;
	struct segment *segment;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	unsigned int seg_in_one_basic_frame;
	unsigned int max_axc_count = framer->framer_param.max_axc_count;
	unsigned int word_size = framer->autoneg_output.cpri_bf_word_size;
	u32 *reg_cma;
	u32 *reg_rcmd0;
	u32 *reg_rcmd1;
	u32 *reg_accr;
	u32 *reg_cr;

	if (check_for_kval_overlap(framer, direction)) {
		dev_err(dev, "k0/k1 overlaps! '- delete axcs and set again'\n");
		return -EINVAL;
	}
	seg_in_one_basic_frame = (word_size * BF_WRDS) / SEG_SIZE;
	/* set k0 k1 register parameter & mode */
	/* axc advance mapping mode reset */
	cpri_reg_set_val(&regs->cpri_map_config,
			AXC_MODE_MASK,
			0x0);


	if (direction & UL_AXCS) {
		reg_cma = &regs->cpri_tcma;
		reg_rcmd0 = &regs->cpri_tcmd0;
		reg_rcmd1 = &regs->cpri_tcmd1;
		cpri_status = &regs->cpri_tstatus;
		reg_accr = &regs->cpri_taccr;
		reg_cr = &regs->cpri_tcr;
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
	} else {
		reg_cma = &regs->cpri_rcma;
		reg_rcmd0 = &regs->cpri_rcmd0;
		reg_rcmd1 = &regs->cpri_rcmd1;
		cpri_status = &regs->cpri_rstatus;
		reg_accr = &regs->cpri_raccr;
		reg_cr = &regs->cpri_rcr;
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
	}

	/* clear map table data for configuration */
	cpri_reg_clear(reg_cr, AXC_ENABLE_MASK);
	loop = 1;
	while ((loop & 0x1)) {
		loop = cpri_reg_get_val(cpri_status, AXC_ENABLE_MASK);
		mdelay(1);
	}
	cpri_reg_set_val(reg_accr, MASK_ALL, 0);
	cpri_reg_set_val(&regs->cpri_map_tbl_config,
			AXC_K0_MASK, 0);
	cpri_reg_set_val(&regs->cpri_map_tbl_config,
			AXC_K1_MASK, 0);


	/* configure num axcs for UL/DL */
	cpri_reg_set_val(&regs->cpri_rgenmode, AXC_MAX_MASK,
			framer->framer_param.max_axc_count);
	cpri_reg_set_val(&regs->cpri_tgenmode, AXC_MAX_MASK,
			framer->framer_param.max_axc_count);
	/* set k1, k2 value */
	cpri_reg_set_val(&regs->cpri_map_tbl_config, AXC_K0_MASK,
			framer->framer_param.k0);
	cpri_reg_set_val(&regs->cpri_map_tbl_config, AXC_K1_MASK,
			framer->framer_param.k1);

	for (loop = 0; loop < max_axc_count; loop++) {
		if (axcs[loop] == NULL)
			continue;
		axc = axcs[loop];
		axc_pos = axc->pos;
		axc_size = calculate_axc_size(axc);
		axc_size_retained = axc_size;
		k = axc->K;
		/* configure tcmd 0/1 or rcmd 0/1 register for segment param */
		if (!(framer->framer_param.k0 % k))
			k_max = framer->framer_param.k0;
		else
			k_max = framer->framer_param.k1;

		program_axc_conf_reg(axc);

		seg = get_segment_id(axc_pos, word_size);
		axc_seg = seg;
		dev_dbg(dev, "k_max: %d, axc_seg: %d, axcid: %d",
				k_max, axc_seg, loop);
		for (loop2 = 0; loop2 < k_max;) {
			map_seg = seg = axc_seg + (loop2 *
					seg_in_one_basic_frame);
			for (k_loop = 0; k_loop < k; k_loop++) {
				if (axc->flags & AXC_FLEXI_POSITION_EN)
					axc_pos = (axc_pos + k_loop);
				seg = map_seg + (k_loop *
						seg_in_one_basic_frame);
				axc_size = axc_size_retained;

				if (axc->nst)
					axc_size -= get_nst_size(axc, k_loop);

				/* configure tcma/rcma register for
				 * segment mapping
				 */
				while (axc_size) {
					segment = (map_table->segments + seg);
					dev_dbg(dev, "%s segment num: %d\n",
							__func__, seg);
					cpri_reg_clear(reg_cma, MASK_ALL);
					bit_mapped = segment_param_set(segment,
								axc, reg_rcmd0);
					cpri_reg_write(&framer->regs_lock,
						reg_cma, AXC_TBL_SEG_ADDR_MASK |
						AXC_TBL_WRITE_MASK,
						AXC_TBL_WRITE_MASK | seg);
					axc_size -= bit_mapped;
					if (axc_size > 0)
						seg++;
					else
						break;
				}
			}
			loop2 += k;
		}
	}
	cpri_reg_clear(reg_cma, MASK_ALL);

	cpri_state_machine(framer, CPRI_STATE_AXC_MAP_INIT);
	return 0;
}

int cpri_axc_map_tbl_flush(struct cpri_framer *framer, unsigned long direction)
{
	unsigned int loop;
	struct axc **axcs;
	struct axc *axc;
	struct axc_map_table *map_table;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	u32 *reg_cr;
	u32 *reg_accr;
	unsigned int max_axc_count = framer->framer_param.max_axc_count;


	/* set k0 k1 register parameter & mode */
	cpri_reg_set_val(
			&regs->cpri_map_tbl_config,
			AXC_K0_MASK, 0);
	cpri_reg_set_val(
			&regs->cpri_map_tbl_config,
			AXC_K1_MASK, 0);
	cpri_reg_clear(&regs->cpri_map_config, (0x1 << 6));
	cpri_reg_clear(&regs->cpri_map_config, (0x1 << 7));
	cpri_reg_set_val(&regs->cpri_map_config,
			AXC_MODE_MASK, 0x0);

	if (direction & UL_AXCS) {
		reg_accr = &regs->cpri_taccr;
		reg_cr = &regs->cpri_tcr;
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
	} else {
		reg_cr = &regs->cpri_rcr;
		reg_accr = &regs->cpri_raccr;
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
	}

	loop = 0;
	while (loop < max_axc_count) {
		if (axcs[loop] == NULL) {
			loop++;
			continue;
		}
		axc = axcs[loop];
		loop++;

		cpri_reg_clear(reg_accr,
				(AXC_ENABLE_MASK << axc->id));
		delete_axc(framer, axc->id, direction);
	}
	/* enable axc recieve and transmit control reg */
	cpri_reg_clear(reg_cr, AXC_ENABLE_MASK);
	return 0;
}


int cpri_map_table_get(struct cpri_framer *framer, unsigned long arg_ioctl)
{
	struct device *dev = framer->cpri_dev->dev;
	unsigned int size;
	unsigned int loop;
	unsigned int seg_loop;
	struct axc_map_table_get *arg;
	u32 flag;
	struct axc_map_table *map_table;
	struct segment_info *segs_info;
	struct segment_info *seg_info;
	struct segment *segment;


	arg = kzalloc(sizeof(struct axc_map_table_get), GFP_KERNEL);
	if (arg == NULL) {
		dev_err(dev, "memory exhausted !!\n");
		return -ENOMEM;
	}
	if (copy_from_user((void *)arg,
				(struct axc_map_table_get *)arg_ioctl,
				sizeof(struct axc_map_table_get))) {
		kfree(arg);
		return -EFAULT;
	}
	flag = arg->flags;
	segs_info = kzalloc((arg->seg_count * sizeof(struct segment_info)),
			GFP_KERNEL);
	if (segs_info == NULL) {
		dev_err(dev, "memory exhausted !!\n");
		return -ENOMEM;
	}
	/* axc allocation */
	if (flag & DL_AXCS)
		map_table = &framer->dl_map_table;
	else
		map_table = &framer->ul_map_table;
	for (loop = 0; loop < arg->seg_count; loop++) {
		segment = (map_table->segments + loop);
		seg_info = (segs_info + loop);
		seg_info->k = segment->k;
		for (seg_loop = 0; seg_loop < NUM_SUBSEG; seg_loop++) {
				if (segment->subsegments[seg_loop].axc != NULL)
					seg_info->subsegments[seg_loop].axc_id =
					segment->subsegments[seg_loop].axc->id;
			seg_info->subsegments[seg_loop].offset =
				segment->subsegments[seg_loop].offset;
			seg_info->subsegments[seg_loop].map_size =
				segment->subsegments[seg_loop].map_size;
		}
	}


	if (copy_to_user((struct axc_map_table_get *)arg_ioctl,
			arg, sizeof(struct axc_map_table_get)))
			return -EFAULT;
	size = arg->seg_count * sizeof(struct segment_info);
	if (copy_to_user((struct segment *)arg->segments,
			segs_info, size))
			return -EFAULT;
	kfree(arg);
	kfree(segs_info);
	return 0;
}

int cpri_axc_param_get(struct cpri_framer *framer, unsigned long arg_ioctl)
{
	struct device *dev = framer->cpri_dev->dev;
	unsigned int size;
	unsigned int loop;
	struct axc_config_params *arg = NULL;
	unsigned int count;
	unsigned int ret;
	u32 flag;
	struct axc_info *axc_param = NULL;
	struct axc_info *param;
	struct axc **axcs;
	struct axc *axc;
	struct axc_map_table *map_table;
	u32 axc_hardware_base_addr0;
	u32 axc_hardware_base_addr1;

	arg = kzalloc(sizeof(struct axc_config_params), GFP_KERNEL);
	if (arg == NULL) {
		dev_err(dev, "memory exhausted !!\n");
		return -ENOMEM;
	}
	if (copy_from_user((void *)arg,
				(struct axc_config_params *)arg_ioctl,
				sizeof(struct axc_config_params))) {
		kfree(arg);
		return -EFAULT;
	}
	flag = arg->flags;
	if (flag & READ_ALL_AXCS)
		count = framer->max_axcs;
	else
		count = 1;
	arg->axc_count = count;
	size = count * sizeof(struct axc_info);
	axc_param = kzalloc(size, GFP_KERNEL);
	if (axc_param == NULL) {
		dev_err(dev, "memory exhausted !!\n");
		ret = -ENOMEM;
		goto get_err;
	}
	if (copy_from_user((void *)axc_param,
					arg->axcs,
					size)) {
			kfree(axc_param);
			dev_err(dev, "copy from user err!!\n");
			ret = -EFAULT;
			goto get_err;
	}

	/* axc allocation */
	if (flag & DL_AXCS) {
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
		axc_hardware_base_addr0 = framer->rx_mblk_hardware_addr0;
		axc_hardware_base_addr1 = framer->rx_mblk_hardware_addr1;
	} else {
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
		axc_hardware_base_addr0 = framer->tx_mblk_hardware_addr0;
		axc_hardware_base_addr1 = framer->tx_mblk_hardware_addr1;
	}

	for (loop = 0; loop < count; loop++) {
		/* allocate axc and map with values and axc_buff */
		param = (axc_param + loop);
		if (count == 1) {
			axc = axcs[axc_param->id];
			if (axc == NULL) {
				dev_err(dev, "AxcId :%d Not configured\n",
						 axc_param->id);
				ret = -EINVAL;
				goto get_err;
			}
		} else
			axc = axcs[loop];

		if (axc != NULL) {
			param->id = axc->id;
			param->flags = axc->flags;
			param->map_method = axc->map_method;
			param->sample_width = axc->sampling_width;
			param->buffer_size = axc->axc_buf->size;
			param->buffer_threshold = axc->buffer_threshold;
			param->S = axc->S;
			param->K = axc->K;
			param->na = axc->na;
			param->ns = axc->nst;
			if (axc->axc_buf->mem_blk)
				param->axc_dma_ptr = axc->axc_buf->addr +
					axc_hardware_base_addr1;
			else
				param->axc_dma_ptr = axc->axc_buf->addr +
					axc_hardware_base_addr0;
			if (copy_to_user(param->pos, axc->pos,
						sizeof(struct axc_pos))) {
				dev_err(dev, "copy to user err id: %d!!\n",
						loop);
				ret = -EFAULT;
				goto get_err;
			}
		} else
			param->id = CPRI_AXC_ID_INVAL;
	}
	if (copy_to_user(arg->axcs, axc_param, size)) {
			dev_err(dev, "copy to user err !!\n");
			ret = -EFAULT;
			goto get_err;
	}
	/* axc allocation done */
	return 0;
get_err:
	if (axc_param != NULL)
		kfree(axc_param);
	if (arg != NULL)
		kfree(arg);
	return ret;
}

int cpri_axc_ioctl(struct cpri_framer *framer, unsigned long arg,
		unsigned int cmd)
{
	int err = 0;
	struct device *dev = framer->cpri_dev->dev;
	u32 dir;

	switch (cmd) {
	case CPRI_SET_AXC_PARAM:
		dev_dbg(dev, "AxC set command\n");
		err = cpri_axc_param_set(framer, arg);
		if (err < 0)
			goto out;
		break;
	case CPRI_GET_AXC_PARAM:
		dev_dbg(dev, "AxC get command\n");
		err = cpri_axc_param_get(framer, arg);
		if (err < 0)
			goto out;
		break;
	case CPRI_GET_MAP_TABLE:
		dev_dbg(dev, "AxC get map table\n");
		err = cpri_map_table_get(framer, arg);
		if (err < 0)
			goto out;
		break;
	case CPRI_MAP_INIT_AXC:
		dev_dbg(dev, "AxC map table init\n");
		if (copy_from_user(&dir, (u32 *)arg,
					sizeof(u32)) != 0) {
			err = -EFAULT;
			goto out;
		}
		err = cpri_axc_map_tbl_init(framer, dir);
		if (err < 0)
			goto out;
		break;
	case CPRI_CTRL_AXC:
		dev_dbg(dev, "AxC ctrl command: framer->id: %d\n", framer->id);
		err = cpri_axc_param_ctrl(framer, arg);
		if (err < 0)
			goto out;
		break;
	case CPRI_MAP_CLEAR_AXC:
		dev_dbg(dev, "AxC map table clear\n");
		if (copy_from_user(&dir, (u32 *)arg,
					sizeof(u32)) != 0) {
			err = -EFAULT;
			goto out;
		}
		err = cpri_axc_map_tbl_flush(framer, arg);
		if (err < 0)
			goto out;
		break;
	}
	return 0;
out:
	dev_err(dev, "AxC ioctl failure\n");
	return err;
}

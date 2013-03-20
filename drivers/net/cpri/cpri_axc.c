/* cpri_axc_param_get *//*
 * drivers/rf/cpri/cpri_axc.c
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
#include <linux/cpri_ioctl.h>
#include <linux/cpri_axc.h>

int init_framer_axc_param(struct cpri_framer *framer)
{
	struct device *dev = framer->cpri_dev->dev;
	unsigned int max_axc_count = framer->framer_param.max_axc_count;
	unsigned int cpri_bf_iq_datablock_size =
		framer->autoneg_output.cpri_bf_iq_datablock_size;

	if (max_axc_count > framer->max_axcs) {
		dev_err(dev, "Axc count is not supported : max val %d\n",
				framer->max_axcs);
		dev_dbg(dev, " Change device init param for axc val\n");
		return -EINVAL;
	}
	framer->ul_axcs = kzalloc(max_axc_count * sizeof(struct axc *),
			GFP_KERNEL);
	framer->dl_axcs = kzalloc(max_axc_count * sizeof(struct axc *),
			GFP_KERNEL);
	framer->max_segments = ((((cpri_bf_iq_datablock_size / 32) * 32) >=
			cpri_bf_iq_datablock_size) ?
			(cpri_bf_iq_datablock_size / 32) :
			((cpri_bf_iq_datablock_size / 32) + 1)) *
			((framer->framer_param.k0 > framer->framer_param.k1) ?
			framer->framer_param.k0 : framer->framer_param.k1);
	framer->ul_map_table.segments = kzalloc(framer->max_segments *
			sizeof(struct axc_map_table),
			GFP_KERNEL);
	framer->ul_map_table.segment_bitmap = kzalloc((framer->max_segments *
				sizeof(u32)), GFP_KERNEL);
	framer->ul_map_table.k0_max = 0;
	framer->ul_map_table.k1_max = 0;
	framer->dl_map_table.k0_max = 0;
	framer->dl_map_table.k1_max = 0;

	framer->dl_map_table.segments = kzalloc(
			(framer->max_segments * sizeof(struct axc_map_table)),
			GFP_KERNEL);
	framer->dl_map_table.segment_bitmap = kzalloc((framer->max_segments *
						sizeof(u32)), GFP_KERNEL);
	return 0;
}

void init_axc_mem_blk(struct cpri_framer *framer,
		struct axc_mem_info *mblk_info)
{
	struct axc_buf_head *tx_mblk = framer->tx_buf_head;
	struct axc_buf_head *rx_mblk = framer->rx_buf_head;

	/* tx mem block initiallization */
	/* tx mem blk 1 */
	if (mblk_info->tx_mblk_addr[0] != 0) {
		tx_mblk->blk_bitmap = 1;
		tx_mblk->max_bufs = framer->framer_param.max_axc_count;
		tx_mblk->blocks[0].base = mblk_info->tx_mblk_addr[0];
		tx_mblk->blocks[0].size = mblk_info->tx_mblk_size[0];
		tx_mblk->blocks[0].next_free_addr = tx_mblk->blocks[0].base;
	}
	/* tx mem blk 2 */
	if (mblk_info->tx_mblk_addr[1] != 0) {
		tx_mblk->blk_bitmap = 1 << 1;
		tx_mblk->blocks[1].base = mblk_info->tx_mblk_addr[1];
		tx_mblk->blocks[1].size = mblk_info->tx_mblk_size[1];
		tx_mblk->blocks[1].next_free_addr = tx_mblk->blocks[1].base;
	}
	/* rx mem block initiallization */
	/* rx mem blk 1 */
	if (mblk_info->rx_mblk_addr[0] != 0) {
		rx_mblk->blk_bitmap = 1;
		rx_mblk->max_bufs = framer->framer_param.max_axc_count;
		rx_mblk->blocks[0].base = mblk_info->rx_mblk_addr[0];
		rx_mblk->blocks[0].size = mblk_info->rx_mblk_size[0];
		rx_mblk->blocks[0].next_free_addr = rx_mblk->blocks[0].base;
	}
	/* rx mem blk 2 */
	if (mblk_info->rx_mblk_addr[1] != 0) {
		rx_mblk->blk_bitmap = 1 << 1;
		rx_mblk->blocks[1].base = mblk_info->rx_mblk_addr[1];
		rx_mblk->blocks[1].size = mblk_info->rx_mblk_size[1];
		rx_mblk->blocks[1].next_free_addr = rx_mblk->blocks[1].base;
	}

	spin_lock_init(&tx_mblk->lock);
	spin_lock_init(&rx_mblk->lock);
	INIT_LIST_HEAD(&tx_mblk->free_list);
	INIT_LIST_HEAD(&rx_mblk->free_list);
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


	if (axc->flags && UL_AXCS)
		mblk = axc->framer->tx_buf_head;
	if (axc->flags && DL_AXCS)
		mblk = axc->framer->rx_buf_head;

	if (!((mblk->blk_bitmap & 0x1) ||
			(mblk->blk_bitmap && (0x1<<1)))) {
		dev_err(dev, "there is no uplink mem entry in dts\n");
		return NULL;
	}

	spin_lock(&mblk->lock);
	if ((mblk->blk_bitmap & 0x1) &&
			(mblk->allocated_bufs < mblk->max_bufs)) {
		if ((mblk->blocks[0].base + mblk->blocks[0].size) >
				(mblk->blocks[0].next_free_addr + size))
			mem_blk = 0;
	} else if ((mblk->blk_bitmap && (0x1<<1)) &&
			(mblk->allocated_bufs < mblk->max_bufs)) {
		if ((mblk->blocks[1].base + mblk->blocks[1].size) >
				(mblk->blocks[1].next_free_addr + size))
			mem_blk = 1;
	}

	if (mem_blk != 0xff) {
		axc_buf = kzalloc(sizeof(struct axc_buf), GFP_KERNEL);
		axc_buf->addr = mblk->blocks[mem_blk].next_free_addr;
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
				list_axc_buf->addr = axc_buf->addr;
				list_axc_buf->size = size;
				list_axc_buf->mem_blk = axc_buf->mem_blk;
				axc_buf->size = list_axc_buf->size - size;
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

	if (axc->flags && UL_AXCS)
		mblk = axc->framer->tx_buf_head;
	else if (axc->flags && DL_AXCS)
		mblk = axc->framer->rx_buf_head;

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
	struct axc_buf_head *tx_mblk = framer->tx_buf_head;
	struct axc_buf_head *rx_mblk = framer->rx_buf_head;
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
	return;
}

int calculate_axc_size(struct axc *axc)
{
	unsigned int axc_size = 0;
	unsigned int Nc;
	if (axc->map_method == MAPPING_METHOD_1)
		/* ceil func((S * SW) / K)
		 */
		axc_size =
		((((axc->S * axc->sampling_width) / axc->K) * axc->K) ==
		(axc->S * axc->sampling_width)) ?
		((axc->S * axc->sampling_width) / axc->K) :
		(((axc->S * axc->sampling_width) / axc->K) + 1);
	else {
		/* ceil func((S * Na)/K)
		*/
		Nc = ((((axc->S * axc->Na) / axc->K) * axc->K) ==
			(axc->S * axc->Na)) ?
		(((axc->S * axc->Na) / axc->K)) :
		((((axc->S * axc->Na) / axc->K)) + 1);

		axc_size = ((2 * axc->sampling_width) * Nc);
	}
	return axc_size;
}

unsigned int calculate_nst_position(struct axc *axc, unsigned int k_curr)
{
	unsigned int ki = 0;
	unsigned int Nc = 0;
	unsigned mlt = 0;
	if (axc->map_method == MAPPING_METHOD_3) {
		/* ceil func((S * Na) / k)
		 */
		Nc = ((((axc->S * axc->Na) / axc->K) * axc->K) ==
			(axc->S * axc->Na)) ?
		((axc->S * axc->Na) / axc->K) :
		(((axc->S * axc->Na) / axc->K) + 1);

		mlt = (Nc * k_curr * axc->K);
		/* floor func((Nc * Ki * K) / Nv
		 */
		ki = (unsigned int)(mlt / axc->Nst);
	}
	return ki;
}

/* this func will validate axc set parm data */
int axc_validate(struct cpri_framer *framer, struct axc_info *axc_parm,
		unsigned int count, u32 flag)
{
	unsigned char k0 = framer->framer_param.k0;
	unsigned char k1 = framer->framer_param.k1;
	unsigned int loop;
	unsigned int temp;
	unsigned int seg;
	unsigned int word_size =
		framer->autoneg_output.cpri_bf_iq_datablock_size / 16;
	unsigned int axc_size;
	unsigned int bit_position;
	unsigned int bit_set;
	struct axc_info *param;
	struct axc axc_param;
	struct axc_map_table *map_table;
	struct segment *segments;
	u32 *segment_bitmap;
	u32 bf_map[2];
	u32 bf_map_val;

	if ((flag >> 1) && DL_AXCS)
		map_table = &framer->dl_map_table;
	else
		map_table = &framer->ul_map_table;

	segment_bitmap = map_table->segment_bitmap;
	segments = map_table->segments;
	bf_map[0] |= map_table->k0_bitmap[0];
	bf_map[1] |= map_table->k0_bitmap[1];
	bf_map[0] |= map_table->k1_bitmap[0];
	bf_map[1] |= map_table->k1_bitmap[1];

	for (loop = 0; loop < count; loop++) {
		param = (axc_parm + loop);
		if (!((!(k0 % param->K)) || (!(k1 % param->K))))
			goto validation_err;
		if (!((param->map_method == MAPPING_METHOD_1) ||
				(param->map_method == MAPPING_METHOD_3)))
			goto validation_err;

		seg = ((param->axc_start_W * word_size) +
				param->axc_start_B) / 32;
		bit_position = ((param->axc_start_W * word_size) +
				param->axc_start_B) % 32;

		axc_param.S = param->S;
		axc_param.K = param->K;
		axc_param.sampling_width = param->sample_width;
		axc_param.Na = param->Na;
		axc_param.map_method = param->map_method;
		axc_size = calculate_axc_size(&axc_param);

		if (axc_size <= 2)
			goto validation_err;

		if ((segments + seg)->k == k0) {
			if ((k0 % param->K) != 0)
				goto validation_err;
		} else if ((segments + seg)->k == k1) {
			if ((k1 % param->K) != 0)
				goto validation_err;
		}

		/* check if segment allready occupied */
		if (((segments + seg)->k == k1) ||
				((segments + seg)->k == k0)) {
			bf_map_val = bf_map[(seg < 31) ? 0 : 1];
			temp = (seg > 31) ? (seg - 31) : seg;
			if ((bf_map_val >> temp) & 0x1) {
				bit_set = bit_position;
				while ((32 - bit_set)) {
					if (((*(segment_bitmap + seg) >>
						bit_set) & 0x1))
						goto validation_err;
					bit_set++;
				}
			}
		}

		if (axc_size > (32 - bit_position)) {
			seg += 1;
			bit_position = 0;
			axc_size -= (32 - bit_position);
		} else
			axc_size = 0;
		/* check if sufficiently large slot available */
		while (axc_size) {
			if (axc_size > (bit_position + 32)) {
				bf_map_val = bf_map[(seg < 31) ? 0 : 1];
				temp = (seg > 31) ? (seg - 31) : seg;
				if ((bf_map_val >> temp) & 0x1)
					goto validation_err;
				axc_size -= 32;
			} else {
				bit_set = 0;
				while (axc_size) {
					if (((*(segment_bitmap + seg) >>
							bit_set) & 0x1))
						goto validation_err;
					bit_set++;
					axc_size--;
				}
			}
		}

	}
	return 0;
validation_err:
	return -EINVAL;
}

void fill_map_table_struc_entry(struct axc *axc)
{
	unsigned char k0 = axc->framer->framer_param.k0;
	unsigned char k1 = axc->framer->framer_param.k0;
	unsigned char k;
	unsigned int seg;
	unsigned int seg_offset;
	unsigned int num_seg = 1;
	unsigned int num_subseg = 0;
	struct cpri_framer *framer = axc->framer;
	unsigned int word_size =
		framer->autoneg_output.cpri_bf_iq_datablock_size / 16;
	struct axc_map_table *map_table;
	unsigned int axc_size;
	int ceil;
	unsigned int bit_position;
	struct segment  *segment;
	struct subsegment *subsegment;
	unsigned int loop;
	unsigned char first = 1;
	u32 *segment_bitmap;
	u32 bit_set;
	u32 bitmap;

	bit_set = 0;
	bitmap = 0;
	if ((axc->flags >> 1) && DL_AXCS)
		map_table = &framer->dl_map_table;
	else
		map_table = &framer->ul_map_table;

	segment_bitmap = map_table->segment_bitmap;
	seg = ((axc->axc_addr_word * word_size) +
			axc->axc_addr_byte) / 32;
	bit_position = ((axc->axc_addr_word * word_size) +
			axc->axc_addr_byte) % 32;
	axc_size = calculate_axc_size(axc);

	if (axc_size <= (32 - bit_position))
		bit_set = axc_size - 1;
	else
		bit_set = 32 - bit_position;

	while (bit_set <= 0)
		bitmap |= (0x1 << bit_set--);

	bitmap = bitmap << bit_position;
	*(segment_bitmap + seg) |= bitmap;
	if (!(k0 % axc->K)) {
		map_table->k0_bitmap[(seg < 31) ? 0 : 1] |=
						(0x1 << ((seg > 31) ?
						 (seg - 31) : seg));
		k = k0;
		if (map_table->k0_max < axc->K)
			map_table->k0_max = axc->K;
	} else {
		map_table->k1_bitmap[(seg < 31) ? 0 : 1] |=
						(0x1 << ((seg > 31) ?
						(seg - 31) : seg));
		k = k1;
		if (map_table->k1_max < axc->K)
			map_table->k1_max = axc->K;
	}
	if (axc_size > (32 - bit_position)) {
		if (bit_position)
			bit_set = axc_size - (32 - bit_position);
		else
			bit_set = axc_size;
		ceil = (((bit_set / 32) * 32) == bit_set) ? (bit_set / 32) :
			((bit_set / 32) + 1);
		num_seg = ceil;
		seg_offset = seg;
		bitmap = 0;
		for (loop = 0; loop < bit_set; loop++) {
			bitmap |= (0x1 << loop);
			if ((loop % 32) == 0) {
				seg_offset += 1;

				if (k0 == k)
					map_table->k0_bitmap[
					(seg_offset < 31) ? 0 : 1] |=
					(0x1 <<	((seg_offset < 31) ?
					(seg_offset - 31) : seg_offset));
				else
					map_table->k1_bitmap[
					(seg_offset < 31) ? 0 : 1] |=
					(0x1 << ((seg_offset < 31) ?
					(seg_offset - 31) : seg_offset));
			}
		}
	}

	/* fill segment struct parameter */
	segment = (map_table->segments + seg);
	first = 1;
	bitmap = 0;
	segment->k = k;
	if (first) {
		if (bit_position < 10)
			num_subseg = 0;
		else if ((bit_position >= 10) && (bit_position < 20))
			num_subseg = 1;
		else
			num_subseg = 2;

		subsegment = &segment->subsegments[num_subseg];

		if (num_subseg == 0) {
			segment->segment_bitmap = 0x7;
			subsegment->offset = bit_position;
			subsegment->map_size = ((
					(32 - bit_position) <
					axc_size) ? (32 -
					bit_position) : axc_size);
			subsegment->axc = axc;
			if (axc_size < (32 - bit_position))
				return;
			axc_size = axc_size - (32 - bit_position);
		} else if (num_subseg == 1) {
			segment->segment_bitmap |= 0x6;
			subsegment->offset = bit_position;
			subsegment->map_size = ((
					(22 - bit_position) <
					axc_size) ? (22 -
					bit_position) : axc_size);
			subsegment->axc = axc;
			if (axc_size < (22 - bit_position))
				return;
			axc_size = axc_size - (22 - bit_position);
		} else {
			segment->segment_bitmap = 0x4;
			subsegment->offset = bit_position;
			subsegment->map_size = ((
					(12 - bit_position) <
					axc_size) ? (12 -
					bit_position) : axc_size);
			subsegment->axc = axc;
			if (axc_size < (12 - bit_position))
				return;
			axc_size = axc_size - (12 - bit_position);
		}
	}
	segment = (segment + 1);
	for (loop = 0; loop < num_seg; loop++) {
		segment->k = k;
		num_subseg = 0;
		subsegment = &segment->subsegments[num_subseg];
		segment->segment_bitmap |= 0x7;
		subsegment->offset = 0;
		subsegment->axc = axc;
		subsegment->map_size = ((32 < axc_size) ?
						32 : axc_size);
		if (axc_size < 32)
			return;
		axc_size = axc_size - 32;
	}
	return;
}

void program_axc_conf_reg(struct axc *axc)
{
	struct cpri_framer *framer = axc->framer;
	struct cpri_framer_regs __iomem *regs = axc->framer->regs;
	u32 *reg_acpr;
	u32 *reg_acprmsb;

	/* disable rx tx axc id for configuration */
	if (axc->flags & UL_AXCS) {
		reg_acpr = &regs->cpri_taxcparam[axc->id];
		reg_acprmsb = &regs->cpri_taxcparammsb[axc->id];

	} else {
		reg_acpr = &regs->cpri_raxcparam[axc->id];
		reg_acprmsb = &regs->cpri_raxcparammsb[axc->id];
	}

	/* axc parameter configuration */
	cpri_reg_set_val(&framer->regs_lock,
			reg_acpr,
			AXC_SIZE_MASK,
			(axc->axc_buf->size - 1));
	if (axc->axc_buf->mem_blk == 0)
		cpri_reg_clear(&framer->regs_lock,
				&reg_acpr,
				AXC_MEM_BLK_MASK);
	else
		cpri_reg_set(&framer->regs_lock,
				reg_acpr,
				AXC_MEM_BLK_MASK);

	cpri_reg_set_val(&framer->regs_lock,
			reg_acpr,
			AXC_BASE_ADDR_MASK,
			axc->axc_buf->addr);

	/* set format param etc */
	if (axc->flags & AXC_IQ_FORMAT_2)
		cpri_reg_set(&framer->regs_lock,
				reg_acprmsb,
				AXC_IQ_FORMAT_2);
	else
		cpri_reg_clear(&framer->regs_lock,
				reg_acprmsb,
				AXC_IQ_FORMAT_2);

	if (axc->flags & AXC_INTERLEAVING_EN)
		cpri_reg_set(&framer->regs_lock,
				reg_acprmsb,
				AXC_INTERLEAVING_EN);
	else
		cpri_reg_clear(&framer->regs_lock,
				reg_acprmsb,
				AXC_INTERLEAVING_EN);
	if (axc->flags & UL_AXCS) {
		if (axc->flags & AXC_TX_ROUNDING_EN)
			cpri_reg_set(&framer->regs_lock,
					reg_acprmsb,
					AXC_TX_ROUNDING_EN);
		else
			cpri_reg_clear(&framer->regs_lock,
					reg_acprmsb,
					AXC_TX_ROUNDING_EN);
	} else {
		if (axc->flags & AXC_CONVERSION_9E2_EN)
			cpri_reg_set(&framer->regs_lock,
					reg_acprmsb,
					AXC_CONVERSION_9E2_EN);
		else
			cpri_reg_clear(&framer->regs_lock,
					reg_acprmsb,
					AXC_CONVERSION_9E2_EN);
	}
	if (axc->flags & AXC_OVERSAMPLING_2X)
		cpri_reg_set(&framer->regs_lock,
				reg_acprmsb,
				AXC_OVERSAMPLING_2X);
	else
		cpri_reg_clear(&framer->regs_lock,
				reg_acprmsb,
				AXC_OVERSAMPLING_2X);

	/* sample width and thresold paramter setting */
	cpri_reg_set_val(&framer->regs_lock,
			reg_acprmsb,
			AXC_SW_MASK,
			axc->sampling_width);
	cpri_reg_set_val(&framer->regs_lock,
			reg_acprmsb,
			AXC_TH_MASK,
			axc->buffer_threshold);
}

int fill_axc_param(struct cpri_framer *framer, struct axc_info *axc_parm,
		struct axc *axc, u32 flag)
{
	axc->id = axc_parm->id;
	axc->flags = axc_parm->flags;
	if ((flag >> 1) && DL_AXCS)
		axc->flags |= DL_AXCS;
	else
		axc->flags |= UL_AXCS;
	axc->framer = framer;
	axc->map_method = axc_parm->map_method;
	axc->axc_addr_word = axc_parm->axc_start_W;
	axc->axc_addr_byte = axc_parm->axc_start_B;
	axc->sampling_width = axc_parm->sample_width;
	axc->buffer_threshold = axc_parm->buffer_threshold;
	axc->S = axc_parm->S;
	axc->K = axc_parm->K;
	axc->Na = axc_parm->Na;
	axc->Nst = axc_parm->Ns;
	axc->axc_buf = axc_alloc(axc, axc_parm->buffer_size);
	if (axc->axc_buf == NULL)
			return -ENOMEM;
	return 0;
}


int cpri_axc_param_set(struct cpri_framer *framer, unsigned long arg_ioctl)
{
	unsigned int size;
	unsigned int loop;
	unsigned int temp;
	struct axc_config_params *arg;
	struct device *dev = framer->cpri_dev->dev;
	unsigned int count;
	u32 flag;
	struct axc_info *axc_param;
	struct axc_info *param;
	struct axc **axcs;
	struct axc *axc;
	struct axc_map_table *map_table;
	u32 axc_pos = 0;


	arg = kzalloc(sizeof(struct axc_config_params), GFP_KERNEL);
	if (copy_from_user((void *)arg,
				(struct axc_config_params *)arg,
				sizeof(struct axc_config_params))) {
		kfree(arg);
		return -EFAULT;
	}
	count = arg->axc_count;
	flag = arg->flags;
	size = count * sizeof(struct axc_info);
	axc_param = kzalloc(size, GFP_KERNEL);
	if (copy_from_user((void *)axc_param, arg->axcs, size)) {
		kfree(axc_param);
		return -EFAULT;
	}

	if (axc_validate(framer, axc_param, count, flag) != 0) {
		dev_err(dev, "axc info parameter error\n");
		kfree(axc_param);
		return -EINVAL;
	}

	/* axc allocation */
	if ((flag >> 1) && DL_AXCS) {
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
	} else {
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
	}

	for (loop = 0; loop < count; loop++) {
		/* allocate axc and map with values and axc_buff */
		param = (axc_param + loop);
		axc = kzalloc(sizeof(struct axc), GFP_KERNEL);
		if (fill_axc_param(framer, param, axc, flag) != 0) {
			dev_err(dev, "axc info parameter error\n");
			kfree(axc);
			kfree(axc_param);
			/* free allocated buffers in dl_axc_list */
			temp = 0;
			while ((axc_pos & 0xffffffff)) {
				if ((axc_pos >> temp) & 0x1) {
					axc = axcs[temp];
					axcs[temp] = NULL;
					axc_free(axc->axc_buf);
					kfree(axc);
					axc_pos &= ~(0x1 << temp);
				}
				temp++;
			}

			return -EINVAL;
		}
		/* program axc conf register */
		program_axc_conf_reg(axc);
		fill_map_table_struc_entry(axc);
		axc_pos |= (0x1 << axc->id);
		axcs[axc->id] = axc;

	}
	/* axc allocation done */
	kfree(axc_param);
	return 0;
}

int clear_segment_param(struct segment *segment, struct axc *axc,
		u32 *reg_cfgmemaddr0)
{
	unsigned char subsegloop;
	unsigned char shift;
	unsigned int bit_mapped = 0;
	struct subsegment *subsegment;
	struct cpri_framer *framer;
	u32 *reg_cfgmemaddr1 = (reg_cfgmemaddr0 + sizeof(u32));
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
			position = AXC_POS_MASK << (6 +	(shift * 16));
			tcm_width = AXC_WIDTH_MASK << (11 + (shift * 16));
			axc_num = AXC_NUM_MASK << (1 + (shift * 16));
			tcm_enable = AXC_NUM_MASK << (0 + (shift * 16));

			if (subsegloop == 2) {
				cpri_reg_clear(&framer->regs_lock,
						reg_cfgmemaddr1, tcm_enable);
				cpri_reg_set_val(&framer->regs_lock,
						reg_cfgmemaddr1, position, 0);
				cpri_reg_set_val(&framer->regs_lock,
						reg_cfgmemaddr1, tcm_width, 0);
				cpri_reg_set_val(&framer->regs_lock,
						reg_cfgmemaddr1, axc_num, 0);
			} else {
				cpri_reg_clear(&framer->regs_lock,
						reg_cfgmemaddr0, tcm_enable);
				cpri_reg_set_val(&framer->regs_lock,
						reg_cfgmemaddr0, position, 0);
				cpri_reg_set_val(&framer->regs_lock,
						reg_cfgmemaddr0, tcm_width, 0);
				cpri_reg_set_val(&framer->regs_lock,
						reg_cfgmemaddr0, axc_num, 0);
			}
			bit_mapped += subsegment->map_size;
			subsegment->offset = 0;
			subsegment->map_size = 0;
		}
	}
	segment->segment_bitmap = 0;
	segment->k = 0;
	return bit_mapped;
}

void clear_axc_param(struct axc *axc)
{
	struct cpri_framer_regs __iomem *regs = axc->framer->regs;
	struct cpri_framer *framer = axc->framer;
	u32 *reg_acpr;
	u32 *reg_acprmsb;

	/* disable rx tx axc id for configuration */
	if (axc->flags & UL_AXCS) {
		reg_acpr = &regs->cpri_taxcparam[axc->id];
		reg_acprmsb = &regs->cpri_taxcparammsb[axc->id];

	} else {
		reg_acpr = &regs->cpri_raxcparam[axc->id];
		reg_acprmsb = &regs->cpri_raxcparammsb[axc->id];
	}

	/* axc parameter configuration */
	cpri_reg_set_val(&framer->regs_lock, reg_acpr, AXC_SIZE_MASK, 0);
	cpri_reg_clear(&framer->regs_lock, &reg_acpr, AXC_MEM_BLK_MASK);
	cpri_reg_set_val(&framer->regs_lock, reg_acpr, AXC_BASE_ADDR_MASK, 0);
	/* sample width and thresold paramter setting */
	cpri_reg_set_val(&framer->regs_lock, reg_acprmsb, AXC_SW_MASK, 0);
	cpri_reg_set_val(&framer->regs_lock, reg_acprmsb, AXC_TH_MASK, 0);
	axc_free(axc->axc_buf);
}

void delete_axc(struct cpri_framer *framer, unsigned char axc_id,
		unsigned int direction)
{
	unsigned int axc_size;
	unsigned char bit_position;
	unsigned int seg;
	struct axc *axc;
	struct axc_map_table *map_table;
	struct segment *segment;
	unsigned int bit_mapped = 0;
	unsigned int bitmap = 0;
	unsigned int word_size =
		framer->autoneg_output.cpri_bf_iq_datablock_size / 16;
	u32 *reg_cfgmemaddr;
	u32 *reg_cfgmemaddr0;
	u32 *reg_cfgmemaddr1;
	u32 *reg_ctrl;
	struct cpri_framer_regs __iomem *regs = framer->regs;

	if (direction & UL_AXCS) {
		axc = framer->ul_axcs[axc_id];
		map_table = &framer->ul_map_table;
		reg_cfgmemaddr = &regs->cpri_tcfgmemaddr;
		reg_cfgmemaddr0 = &regs->cpri_tcfgmemaddr0;
		reg_cfgmemaddr1 = &regs->cpri_tcfgmemaddr1;
		reg_ctrl = &regs->cpri_tctrl;
	} else {
		axc = framer->dl_axcs[axc_id];
		map_table = &framer->dl_map_table;
		reg_cfgmemaddr = &regs->cpri_rcfgmemaddr;
		reg_cfgmemaddr0 = &regs->cpri_rcfgmemaddr0;
		reg_cfgmemaddr1 = &regs->cpri_rcfgmemaddr1;
		reg_ctrl = &regs->cpri_rctrl;
	}
	axc_size = calculate_axc_size(axc);
	seg = ((axc->axc_addr_word * word_size) +
			axc->axc_addr_byte) / 32;
	bit_position = ((axc->axc_addr_word * word_size) +
			axc->axc_addr_byte) % 32;
	while (axc_size) {
		segment = (map_table->segments + seg);
		cpri_reg_set_val(&framer->regs_lock, reg_cfgmemaddr,
				AXC_TBL_SEG_ADDR_MASK, seg);
		cpri_reg_set(&framer->regs_lock, reg_cfgmemaddr,
				AXC_TBL_WRITE_MASK);
		bit_mapped = clear_segment_param(segment, axc,
				reg_cfgmemaddr0);
		cpri_reg_clear(&framer->regs_lock,
				reg_cfgmemaddr, AXC_TBL_WRITE_MASK);
		axc_size -= bit_mapped;
		if ((axc_size) && (axc_size > 10))
			seg++;
		else
			axc_size = 0;
		while (bit_mapped >= 0)
			bitmap |= (0x1 << bit_mapped--);
		bitmap = ~(bitmap << bit_position);
		*(map_table->segment_bitmap + seg) &= bitmap;
		bit_position = 0;
	}
	clear_axc_param(axc);
	if (direction & UL_AXCS)
		framer->ul_axcs[axc_id] = NULL;
	else
		framer->dl_axcs[axc_id] = NULL;
	kfree(axc);
}

int cpri_axc_param_ctrl(struct cpri_framer *framer, unsigned long arg)
{
	struct cpri_axc_ctrl *ctrl;
	unsigned int size;
	u32 *reg_axcctrl;

	size = sizeof(struct cpri_axc_ctrl);
	ctrl = kmalloc(size, GFP_KERNEL);
	if (copy_from_user((void *)ctrl, (struct cpri_axc_ctrl *)arg, size)) {
		kfree(ctrl);
		return -EFAULT;
	}
	if ((ctrl->direction & UL_AXCS))
		reg_axcctrl = &framer->regs->cpri_taxcctrl;
	else
		reg_axcctrl = &framer->regs->cpri_raxcctrl;

	switch (ctrl->op) {
	case AXC_ENABLE:
		cpri_reg_set(&framer->regs_lock, reg_axcctrl,
				(AXC_ENABLE_MASK << ctrl->axc_id));

		break;
	case AXC_DISABLE:
		cpri_reg_clear(&framer->regs_lock, reg_axcctrl,
				(AXC_ENABLE_MASK << ctrl->axc_id));
		break;
	case AXC_DELETE:
		cpri_reg_clear(&framer->regs_lock, reg_axcctrl,
				(AXC_ENABLE_MASK << ctrl->axc_id));
		delete_axc(framer, ctrl->axc_id, ctrl->direction);
		break;
	}
	return 0;
}

int set_segment_param(struct segment_param *seg_parm)
{
	unsigned char subsegloop;
	unsigned char shift;
	unsigned int bit_mapped = 0;
	unsigned int axc_size;
	int map_width = 0;
	int map_width_ns = 0;
	struct subsegment *subsegment;
	struct cpri_framer *framer;
	u32 position;
	u32 axc_num;
	u32 tcm_enable;
	u32 tcm_width;
	u32 bit_offset;
	struct segment *segment = seg_parm->segment;
	struct axc *axc = seg_parm->axc;
	u32 *reg_cfgmemaddr0 = seg_parm->reg_cfgmemaddr0;
	u32 *reg_cfgmemaddr1 = (reg_cfgmemaddr0 + sizeof(u32));
	unsigned int ki = seg_parm->ki;
	unsigned char nst_len = seg_parm->nst_len;


	axc_size = calculate_axc_size(axc);
	framer = axc->framer;
	for (subsegloop = 0; subsegloop < 3; subsegloop++) {
		map_width_ns = 0;
		bit_offset = 0;
		subsegment = &segment->subsegments[subsegloop];
		if (subsegment->axc == NULL) {
			if (ki != 0xffff)
				ki -= 10;
			continue;
		}
		if (subsegment->axc->id != axc->id) {
			if (ki != 0xffff)
				ki -= 10;
			continue;
		}

		if (subsegloop < 2)
			shift = subsegloop;
		else
			shift = 0;

		position = AXC_POS_MASK << (6 +	(shift * 16));
		tcm_width = AXC_WIDTH_MASK << (11 + (shift * 16));
		axc_num = AXC_NUM_MASK << (1 + (shift * 16));
		tcm_enable = AXC_NUM_MASK << (shift * 16);

		bit_mapped += subsegment->map_size;
		axc_size -= bit_mapped;
		map_width = subsegment->map_size;
		if (axc_size <= 16) {
			map_width += axc_size;
			bit_mapped += axc_size;
		}

		if (ki == 0xffff) {
			if (subsegloop == 2) {
				cpri_reg_set(&framer->regs_lock,
					reg_cfgmemaddr1, tcm_enable);
				cpri_reg_set_val(&framer->regs_lock,
					reg_cfgmemaddr1, position,
					(subsegment->offset / 2));
				cpri_reg_set_val(&framer->regs_lock,
					reg_cfgmemaddr1, tcm_width,
					(map_width / 2));
				cpri_reg_set_val(&framer->regs_lock,
					reg_cfgmemaddr1, axc_num,
					axc->id);
			} else {

				cpri_reg_set(&framer->regs_lock,
					reg_cfgmemaddr0,
					tcm_enable);
				cpri_reg_set_val(&framer->regs_lock,
					reg_cfgmemaddr0,
					position,
					(subsegment->offset / 2));
				cpri_reg_set_val(&framer->regs_lock,
					reg_cfgmemaddr0,
					tcm_width,
					(map_width / 2));
				cpri_reg_set_val(&framer->regs_lock,
					reg_cfgmemaddr0,
					axc_num,
					axc->id);
			}
			continue;
		}

		/* here bug can be if ki != 0 and stuffing bit size is
		*  smaller than end of segment than there is no way
		*  where we can program same subseg 2ce
		*/
		if (subsegloop == 2) {
			if (subsegment->offset < ki) {
				map_width_ns = ki - subsegment->offset;
				cpri_reg_set(&framer->regs_lock,
				reg_cfgmemaddr1, tcm_enable);
				cpri_reg_set_val(&framer->regs_lock,
				reg_cfgmemaddr1, position,
				(subsegment->offset / 2));
				cpri_reg_set_val(&framer->regs_lock,
				reg_cfgmemaddr1, tcm_width,
				(map_width_ns / 2));
				cpri_reg_set_val(&framer->regs_lock,
				reg_cfgmemaddr1, axc_num,
				axc->id);
				map_width = 0;
				} else if (nst_len < (12 -
						subsegment->offset)) {
					bit_offset = ki + nst_len;
					if (map_width < nst_len)
						map_width -= (map_width_ns
								+ nst_len);
					else
						map_width = 0;
				} else
					map_width = 0;
				if (map_width <= 0)
					continue;
				cpri_reg_set(&framer->regs_lock,
					reg_cfgmemaddr1, tcm_enable);
				cpri_reg_set_val(&framer->regs_lock,
				reg_cfgmemaddr1, position, (bit_offset / 2));
				cpri_reg_set_val(&framer->regs_lock,
				reg_cfgmemaddr1, tcm_width, (map_width / 2));
				cpri_reg_set_val(&framer->regs_lock,
					reg_cfgmemaddr1, axc_num, axc->id);
				continue;
		}
		/* this is for subsegloop < 2 */
		if (subsegment->offset < ki) {
			map_width_ns = ki - subsegment->offset;
			cpri_reg_set(&framer->regs_lock,
			reg_cfgmemaddr1, tcm_enable);
			cpri_reg_set_val(&framer->regs_lock,
			reg_cfgmemaddr1, position,
			(subsegment->offset / 2));
			cpri_reg_set_val(&framer->regs_lock,
			reg_cfgmemaddr1, tcm_width, (map_width_ns /
				2));
			cpri_reg_set_val(&framer->regs_lock,
			reg_cfgmemaddr1, axc_num, axc->id);
		}

		if ((subsegloop == 0) && ((ki + nst_len) <
				(32 - ki))) {
			bit_offset = ki + nst_len;
			map_width -= (map_width_ns + nst_len);
		} else if ((subsegloop == 1) && ((ki + nst_len) < (22 - ki))) {
			map_width -= (map_width_ns + nst_len);
			bit_offset = ki + nst_len;

		} else
			map_width = 0;

		if (map_width <= 0)
			continue;

		cpri_reg_set(&framer->regs_lock,
			reg_cfgmemaddr1, tcm_enable);
		cpri_reg_set_val(&framer->regs_lock,
		reg_cfgmemaddr1, position, (bit_offset / 2));
		cpri_reg_set_val(&framer->regs_lock,
		reg_cfgmemaddr1, tcm_width, (map_width / 2));
		cpri_reg_set_val(&framer->regs_lock,
		reg_cfgmemaddr1, axc_num, axc->id);
	}
	return bit_mapped;
}

int cpri_axc_map_tbl_init(struct cpri_framer *framer, unsigned long direction)
{
	unsigned int loop;
	unsigned int loop2;
	unsigned int k;
	unsigned int k_max;
	unsigned int ki;
	unsigned int Nc;
	unsigned int seg;
	unsigned int map_table_seg;
	unsigned char nst_len = 0;
	struct axc **axcs;
	struct axc *axc;
	struct segment_param *seg_parm;
	unsigned int axc_size;
	unsigned int bit_mapped;
	struct axc_map_table *map_table;
	struct segment *segment;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	unsigned int bit_position;
	unsigned int nst_flag = 0;
	unsigned int num_bf_seg;
	unsigned int seg_map_position = 0;
	unsigned int cpri_bf_iq_datablock_size =
		framer->autoneg_output.cpri_bf_iq_datablock_size;
	unsigned int word_size =
		framer->autoneg_output.cpri_bf_iq_datablock_size / 16;
	u32 *reg_cfgmemaddr;
	u32 *reg_cfgmemaddr0;
	u32 *reg_cfgmemaddr1;
	u32 *reg_ctrl;



	num_bf_seg = ((((framer->autoneg_output.cpri_bf_iq_datablock_size /
						32) * 32) >=
			cpri_bf_iq_datablock_size) ?
			(cpri_bf_iq_datablock_size / 32) :
			((cpri_bf_iq_datablock_size / 32) + 1));
	/* set k0 k1 register parameter & mode */
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_maptblcfg,
			AXC_K0_MASK,
			framer->framer_param.k0);
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_maptblcfg,
			AXC_K1_MASK,
			framer->framer_param.k1);
	/* axc advance mapping mode */
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_mapcfg,
			AXC_MODE_MASK,
			0x1);

	if (direction & UL_AXCS) {
		reg_cfgmemaddr = &regs->cpri_tcfgmemaddr;
		reg_cfgmemaddr0 = &regs->cpri_tcfgmemaddr0;
		reg_cfgmemaddr1 = &regs->cpri_tcfgmemaddr1;
		reg_ctrl = &regs->cpri_tctrl;
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
	} else {
		reg_cfgmemaddr = &regs->cpri_rcfgmemaddr;
		reg_cfgmemaddr0 = &regs->cpri_rcfgmemaddr0;
		reg_cfgmemaddr1 = &regs->cpri_rcfgmemaddr1;
		reg_ctrl = &regs->cpri_rctrl;
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
	}

	seg_parm = kzalloc(sizeof(struct segment_param), GFP_KERNEL);
	loop = 0;
	while (axcs[loop] != NULL) {
		axc = axcs[loop];
		loop++;
		k = axc->K;
		seg = ((axc->axc_addr_word * word_size) +
				axc->axc_addr_byte) / 32;
		bit_position = ((axc->axc_addr_word * word_size) +
				axc->axc_addr_byte) % 32;
		axc_size = calculate_axc_size(axc);
		/* configure tcmd 0/1 or rcmd 0/1 register for segment param */
		map_table_seg = seg;
		if (!(framer->framer_param.k0 % k))
			k_max = map_table->k0_max;
		else
			k_max = map_table->k1_max;

		for (loop2 = 0; loop2 < k_max;) {
			for (loop = 0; loop < k; loop++) {
				nst_flag = 0;
				seg_map_position = 0;
				/* configure tcma/rcma register for
				 * segment mapping
				 */
				seg = map_table_seg + (num_bf_seg * loop);
				if ((axc->Nst) && (axc->Nst >= loop) &&
					(axc->map_method == MAPPING_METHOD_3)) {
					ki = calculate_nst_position(axc, loop);
					nst_flag = 1;
					Nc = (((((axc->S * axc->Na) / axc->K) *
						axc->K) == (axc->S * axc->Na)) ?
						((axc->S * axc->Na) / axc->K) :
						(((axc->S * axc->Na) / axc->K) +
						 1));
					ki = (ki % Nc);
					ki = ki * 2 * axc->sampling_width;
					nst_len = 2 * axc->sampling_width;

				}
			while (axc_size) {

				segment = (map_table->segments +
						seg - (num_bf_seg * loop));
				cpri_reg_set(&framer->regs_lock,
						reg_cfgmemaddr,
						AXC_TBL_WRITE_MASK);
				cpri_reg_set_val(&framer->regs_lock,
						reg_cfgmemaddr,
						AXC_TBL_SEG_ADDR_MASK,
						seg);
				if ((nst_flag) && ((ki == seg_map_position) ||
					ki <= (seg_map_position + 32))) {
					ki -= seg_map_position; /* bring ki at
								segment offset
								position
								*/
					seg_parm->segment = segment;
					seg_parm->axc = axc;
					seg_parm->reg_cfgmemaddr0 =
						reg_cfgmemaddr0;
					seg_parm->ki = ki;
					seg_parm->nst_len = nst_len;
					bit_mapped = set_segment_param(
							seg_parm);
					if ((32 - ki) >= nst_len)
						nst_flag = 0;
					else {
						nst_len = 32 - ki;
						ki = seg_map_position +
							bit_mapped;
					}

				} else {
					seg_parm->segment = segment;
					seg_parm->axc = axc;
					seg_parm->reg_cfgmemaddr0 =
						reg_cfgmemaddr0;
					seg_parm->ki = 0xffff;
					bit_mapped = set_segment_param(
							seg_parm);
				}
				cpri_reg_clear(&framer->regs_lock,
						reg_cfgmemaddr,
						AXC_TBL_WRITE_MASK);
				axc_size -= bit_mapped;
				seg_map_position += bit_mapped;
				if (axc_size)
					seg++;
				else
					break;


			}
			}
		loop2 += k;
		}
	}
	/* enable axc recieve and transmit control reg */
	cpri_reg_set(&framer->regs_lock, reg_ctrl, AXC_ENABLE_MASK);
	kfree(seg_parm);
	return 0;
}

int cpri_axc_map_tbl_flush(struct cpri_framer *framer, unsigned long direction)
{
	unsigned int loop;
	struct axc **axcs;
	struct axc *axc;
	struct axc_map_table *map_table;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	unsigned int num_bf_seg;
	unsigned int cpri_bf_iq_datablock_size =
		framer->autoneg_output.cpri_bf_iq_datablock_size;
	u32 *reg_cfgmemaddr;
	u32 *reg_cfgmemaddr0;
	u32 *reg_cfgmemaddr1;
	u32 *reg_ctrl;
	u32 *reg_axcctrl;



	num_bf_seg = ((((framer->autoneg_output.cpri_bf_iq_datablock_size /
						32) * 32) >=
			cpri_bf_iq_datablock_size) ?
			(cpri_bf_iq_datablock_size / 32) :
			((cpri_bf_iq_datablock_size / 32) + 1));
	/* set k0 k1 register parameter & mode */
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_maptblcfg,
			AXC_K0_MASK, 0);
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_maptblcfg,
			AXC_K1_MASK, 0);
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_mapcfg, (0x1 << 6));
	cpri_reg_clear(&framer->regs_lock, &regs->cpri_mapcfg, (0x1 << 7));
	cpri_reg_set_val(&framer->regs_lock,
			&regs->cpri_mapcfg,
			AXC_MODE_MASK, 0x0);

	if (direction & UL_AXCS) {
		reg_cfgmemaddr = &regs->cpri_tcfgmemaddr;
		reg_cfgmemaddr0 = &regs->cpri_tcfgmemaddr0;
		reg_cfgmemaddr1 = &regs->cpri_tcfgmemaddr1;
		reg_axcctrl = &regs->cpri_taxcctrl;
		reg_ctrl = &regs->cpri_tctrl;
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
	} else {
		reg_cfgmemaddr = &regs->cpri_rcfgmemaddr;
		reg_cfgmemaddr0 = &regs->cpri_rcfgmemaddr0;
		reg_cfgmemaddr1 = &regs->cpri_rcfgmemaddr1;
		reg_ctrl = &regs->cpri_rctrl;
		reg_axcctrl = &regs->cpri_raxcctrl;
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
	}

	loop = 0;
	while (axcs[loop] != NULL) {
		axc = axcs[loop];
		loop++;

		cpri_reg_clear(&framer->regs_lock, reg_axcctrl,
				(AXC_ENABLE_MASK << axc->id));
		delete_axc(framer, axc->id, direction);
	}
	/* enable axc recieve and transmit control reg */
	cpri_reg_clear(&framer->regs_lock, reg_ctrl, AXC_ENABLE_MASK);
	return 0;
}

/* for this function i will if the same handing can be put in ioctl call itself
*/
int cpri_axc_param_get(struct cpri_framer *framer, unsigned long arg_ioctl)
{
	unsigned int size;
	unsigned int loop;
	struct axc_config_params *arg;
	unsigned int count;
	u32 flag;
	struct axc_info *axc_param;
	struct axc_info *param;
	struct axc **axcs;
	struct axc *axc;
	struct axc_map_table *map_table;


	arg = kzalloc(sizeof(struct axc_config_params), GFP_KERNEL);
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
	if (copy_from_user((void *)axc_param,
				arg->axcs,
				size)) {
		kfree(axc_param);
		return -EFAULT;
	}

	/* axc allocation */
	if ((flag >> 1) && DL_AXCS) {
		axcs = framer->dl_axcs;
		map_table = &framer->dl_map_table;
	} else {
		axcs = framer->ul_axcs;
		map_table = &framer->ul_map_table;
	}

	for (loop = 0; loop < count; loop++) {
		/* allocate axc and map with values and axc_buff */
		param = (axc_param + loop);
		axc = axcs[loop];
		if (axc != NULL) {
			axc->id = axc_param->id;
			axc->flags = axc_param->flags;
			if ((flag >> 1) && DL_AXCS)
				axc->flags |= DL_AXCS;
			else
				axc->flags |= UL_AXCS;
			param->id = axc->id;
			param->flags = axc->flags;
			param->map_method = axc->map_method;
			param->axc_start_W = axc->axc_addr_word;
			param->axc_start_B = axc->axc_addr_byte;
			param->sample_width = axc->sampling_width;
			param->buffer_threshold = axc->buffer_threshold;
			param->S = axc->S;
			param->K = axc->K;
			param->Na = axc->Na;
			param->Ns = axc->Nst;
		} else
			param->id = CPRI_AXC_ID_INVAL;
	}
	arg->axcs = axc_param;
	if (copy_to_user((struct axc_config_params *)arg_ioctl,
			arg,
			sizeof(struct axc_config_params))) {
			return -EFAULT;
		}
	if (copy_to_user((void *)(arg_ioctl + sizeof(struct axc_config_params)),
			axc_param,
			size)) {
			return -EFAULT;
		}
	/* axc allocation done */
	kfree(axc_param);
	kfree(arg);
	return 0;
}

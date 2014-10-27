/*
 * drivers/net/cpri/cpri_axc.c
 * CPRI device driver - axc map
 * Author: Anand Singh
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * The code in this file realizes a subset of the CPRI IP block mapping
 * capability, namely K=1. For K>1 needs more advanced mapping and will
 * be added later if needed.
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

void clear_axc_map_tx_rx_table(struct cpri_framer *framer)
{
	int loop;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	u32 val;

	cpri_write(0, &regs->cpri_rcmd0);
	cpri_write(0, &regs->cpri_rcmd1);
	cpri_write(0, &regs->cpri_tcmd0);
	cpri_write(0, &regs->cpri_tcmd1);
	wmb();
	for (loop = 0; loop < 3072; loop++) {
		val = (AXC_TBL_WRITE_MASK | loop);
		cpri_write(val, &regs->cpri_rcma);
		cpri_write(val, &regs->cpri_tcma);
	}
}

static int iq_bits_per_bf(struct cpri_framer *framer)
{
	struct cpri_autoneg_output *output = &(framer->autoneg_output);
	struct device *dev = framer->cpri_dev->dev;
	int bits_per_bf;

	switch (output->common_rate) {
	case RATE2_1228_8M:
		bits_per_bf = BF_IQ_BITS_240;
		break;
	case RATE3_2457_6M:
		bits_per_bf = BF_IQ_BITS_480;
		break;
	case RATE4_3072_0M:
		bits_per_bf = BF_IQ_BITS_600;
		break;
	case RATE5_4915_2M:
		bits_per_bf = BF_IQ_BITS_960;
		break;
	case RATE6_6144_0M:
		bits_per_bf = BF_IQ_BITS_1200;
		break;
	case RATE7_9830_4M:
		bits_per_bf = BF_IQ_BITS_1920;
		break;
	default:
		dev_err(dev, "Invalid common link rate: %d\n",
				output->common_rate);
		bits_per_bf = 0;
	break;
	}
	return bits_per_bf;
}
/* Program the mapping table.
 * Note that due to silicon bug,
 * reading of mapping table is not allowed
 * because it will corrupt the mapping table data.
 * That means AXC_TBL_WRITE_MASK needs to be set for t/rcma reg.
 */
static void map_axc_tx_rx_table(struct cpri_framer *framer,
		const struct axc_cmd_regs *cmd_regs) {

	int loop;
	struct cpri_framer_regs __iomem *regs = framer->regs;
	u32 val;

	for (loop = 0; loop < 3072; loop++) {
		cpri_write(cmd_regs->rcmd0[loop], &regs->cpri_rcmd0);
		cpri_write(cmd_regs->rcmd1[loop], &regs->cpri_rcmd1);
		cpri_write(cmd_regs->tcmd0[loop], &regs->cpri_tcmd0);
		cpri_write(cmd_regs->tcmd1[loop], &regs->cpri_tcmd1);
		wmb();
		val = (AXC_TBL_WRITE_MASK | loop);
		cpri_write(val, &regs->cpri_rcma);
		cpri_write(val, &regs->cpri_tcma);
	}

}

/* Program the AUX interface.
 * It can cover the IQ data area, not including the CW section.
 */
static void program_aux_interface(struct cpri_framer *framer,
		const struct axc_info *axc)
{
	unsigned int size;
	unsigned int last_bit_pos;
	unsigned int bit_pos;
	u32 bitmap;
	u8 index;
	u32 word_size = 0;

	word_size = iq_bits_per_bf(framer) / 15;

	index = (axc->container_offset + word_size) / 32;
	size = axc->container_size;
	bit_pos = (axc->container_offset + word_size) % 32;

	while (size) {
			last_bit_pos = size;
			size = ((size + bit_pos) >= SEG_SIZE) ? (size -
				(SEG_SIZE - bit_pos)) : 0;
			if (size) {
				while (bit_pos < SEG_SIZE)
					bitmap |= (0x1 << bit_pos++);
			} else {
				last_bit_pos = (bit_pos + last_bit_pos);
				while (bit_pos < last_bit_pos)
					bitmap |= (0x1 << bit_pos++);
			}
			bitmap = be32_to_cpu(bitmap);

			if (axc->flags & AXC_EN)
				cpri_reg_set(
					&framer->regs->cpri_auxmask[index],
					bitmap);
			else
				cpri_reg_clear(
					&framer->regs->cpri_auxmask[index],
					bitmap);

			bit_pos = 0;
			bitmap = 0;
			index++;
		}
}

/* Program the config for each AxC */
static void program_axc_conf_reg(struct cpri_framer *framer,
		const struct axc_info *axc)
{
	u32 *reg_acpr;
	u32 *reg_acprmsb;
	u32 *cpri_map_smpl_cfg;
	u32 val;
	struct cpri_framer_regs __iomem *regs = framer->regs;

	if (axc->flags & AXC_DIR_TX) {
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
	/* Program AxC sample width */
	cpri_reg_set_val(cpri_map_smpl_cfg, AXC_SMPL_WDTH_MASK, val);

	cpri_reg_set_val(reg_acpr,
			AXC_SIZE_MASK, axc->buffer_size - 1);

	/* Choose memory block to use */
	if (axc->memblk_sel == MEMBLK0)
		cpri_reg_clear(reg_acpr, AXC_MEM_BLK_MASK);
	else
		cpri_reg_set(reg_acpr, AXC_MEM_BLK_MASK);

	/* Set memory block base address */
	cpri_reg_set_val(reg_acpr, AXC_BASE_ADDR_MASK, axc->buffer_offset_addr);

	/* Set format param */
	if (axc->flags & AXC_IQ_FORMAT_2)
		cpri_reg_set(reg_acprmsb, AXC_IQ_FORMAT_2);
	else
		cpri_reg_clear(reg_acprmsb, AXC_IQ_FORMAT_2);

	/* Config interleaving */
	if (axc->flags & AXC_INTERLEAVING_EN)
		cpri_reg_set(reg_acprmsb, AXC_INTERLEAVING_EN);
	else
		cpri_reg_clear(reg_acprmsb, AXC_INTERLEAVING_EN);

	/* TX rounding and 9E2 format */
	if (axc->flags & AXC_DIR_TX) {
		if (axc->flags & AXC_TX_ROUNDING_EN)
			cpri_reg_set(reg_acprmsb, AXC_TX_ROUNDING_EN);
		else
			cpri_reg_clear(reg_acprmsb, AXC_TX_ROUNDING_EN);
	} else {
		if (axc->flags & AXC_9E2_EN)
			cpri_reg_set(reg_acprmsb, AXC_9E2_EN);
		else
			cpri_reg_clear(reg_acprmsb, AXC_9E2_EN);
	}

	/* Config the oversampling */
	if (axc->flags & AXC_OVERSAMPLING_2X)
		cpri_reg_set(reg_acprmsb, AXC_OVERSAMPLING_2X);
	else
		cpri_reg_clear(reg_acprmsb, AXC_OVERSAMPLING_2X);

	/* Sample width and thresold setting */
	cpri_reg_set_val(reg_acprmsb, AXC_SW_MASK, axc->sampling_width);
	cpri_reg_set_val(reg_acprmsb, AXC_TH_MASK, axc->buffer_threshold);
}

/* Find the segment ID */
static inline int find_seg_tomap(int bit_pos)
{
	return bit_pos / 32;
}

/* Find one of the three subsegments in segment */
static inline int find_subseg_tomap(int bit_pos)
{
	int offset;
	offset = bit_pos % 32;
	if (offset < 10)
		return 0;
	else if (offset < 20)
		return 1;
	else
		return 2;

}


/* Find the offset within subsegment */
static inline int pos_in_subseg(int bit_pos)
{
	int offset;
	offset = bit_pos % 32;

	if (offset < 10)
		return offset;
	else if (offset < 20)
		return offset - 10;
	else
		return offset - 20;

}

/* The AxC mapping code, right now it
 * supports K=1 only, but DFE can handle K > 1
 * The driver will be updated later if it's needed
 */
static void fill_map_table(struct axc_info axc, struct axc_cmd_regs *cmd_regs)
{
	int bits_to_map, remaining_bits;
	int current_pos;
	int val;
	int seg, sub_seg, subseg_offset;

	remaining_bits = axc.container_size;
	current_pos = axc.container_offset;

	while (remaining_bits) {
		seg = find_seg_tomap(current_pos);
		sub_seg = find_subseg_tomap(current_pos);
		subseg_offset = pos_in_subseg(current_pos);

		if (remaining_bits <= 32)
			bits_to_map = remaining_bits;
		else
			bits_to_map = 32;

		remaining_bits -= bits_to_map;
		current_pos += bits_to_map;

		val = ((bits_to_map / 2) << 11) | ((subseg_offset / 2) << 6)
				| (axc.id << 1) | 1;

		if (axc.flags & AXC_DIR_RX) {
			if (sub_seg == 0)
				cmd_regs->rcmd0[seg] |= val;
			else if (sub_seg == 1)
				cmd_regs->rcmd0[seg] |= (val << 16);
			else
				cmd_regs->rcmd1[seg] = val;
		} else if (axc.flags & AXC_DIR_TX) {
			if (sub_seg == 0)
				cmd_regs->tcmd0[seg] |= val;
			else if (sub_seg == 1)
				cmd_regs->tcmd0[seg] |= (val << 16);
			else
				cmd_regs->tcmd1[seg] = val;
		}
	}
}


/* Checks for axc overlap
 * @return value:
 * 1: overlap in basic frame
 * 2: overlap in mem buffer
 * 0: no overlap
 */
static int check_axc_overlap(const struct axc_info *axc0,
		const struct axc_info *axc1)
{
	int range_low0, range_low1, range_hi0, range_hi1;
	int err = 0;

	range_low0 = axc0->container_offset;
	range_hi0 = axc0->container_offset + axc0->container_size;

	range_low1 = axc1->container_offset;
	range_hi1 = axc1->container_offset + axc1->container_size;

	if (!((range_low1 >= range_hi0) || (range_hi1 <= range_low0)))
		err = 1;

	range_low0 = axc0->buffer_offset_addr;
	range_hi0 = axc0->buffer_size + axc0->buffer_offset_addr;

	range_low1 = axc1->buffer_offset_addr;
	range_hi1 = axc1->buffer_size + axc1->buffer_offset_addr;

	if ((axc0->memblk_sel == axc1->memblk_sel)) {
		if (!((range_low1 >= range_hi0) || (range_hi1 <= range_low0)))
			err |= 2;
	}

	return err;
}

/* Check for any AxC error */
#define MAX_POSSIBLE_AXC	48
static int check_axc_conflict(struct cpri_framer *framer,
		struct axc_info *axc, int clear)
{
	int bits_per_bf;
	struct device *dev = framer->cpri_dev->dev;
	static struct axc_info *axc_info_tx[MAX_POSSIBLE_AXC];
	static struct axc_info *axc_info_rx[MAX_POSSIBLE_AXC];
	static int memblk_tx0, memblk_tx1, memblk_rx0, memblk_rx1;
	struct axc_info **axc_info_his;
	int i;
	const char *axc_dir;
	int ret;

	if (clear) {
		for (i = 0; i < framer->max_axcs; i++) {
			axc_info_rx[i] = NULL;
			axc_info_tx[i] = NULL;
			memblk_tx0 = 0;
			memblk_rx0 = 0;
			memblk_tx1 = 0;
			memblk_rx1 = 0;
		}
		return 0;
	}


	bits_per_bf = iq_bits_per_bf(framer);
	axc_dir = (axc->flags & AXC_DIR_TX) ? "TX" : "RX";

	/* Do some basic check */
	if (axc->container_offset < 0) {
		dev_err(dev, "%s AxC%d err: container offset negative",
			axc_dir, axc->id);
		goto out;
	}
	if (axc->container_size <= 0) {
		dev_err(dev, "%s AxC%d err: container size 0 or negative",
			axc_dir, axc->id);
		goto out;
	}
	if ((axc->container_offset + axc->container_size) >
		bits_per_bf) {
		dev_err(dev, "%s AxC%d err: out of basic frame range",
			axc_dir, axc->id);
		goto out;
	}
	if ((axc->flags & AXC_DIR_RX) &&
		(axc->flags & AXC_DAISY_CHAINED)) {
		dev_err(dev, "%s AxC%d flag err: AXC_DIR_RX & AXC_DAISY_CHAIN",
			axc_dir, axc->id);
		goto out;
	}
	/* Daisy chain skip further test */
	if (axc->flags & AXC_DAISY_CHAINED)
		return 0;

	if (axc->container_offset % 2) {
		dev_err(dev, "%s AxC%d err: can't start on odd offset",
			axc_dir, axc->id);
		goto out;
	}
	if (axc->container_size % 2) {
		dev_err(dev, "%s AxC%d err: can't use odd container size",
			axc_dir, axc->id);
		goto out;
	}

	if (axc->container_size % axc->sampling_width) {
		dev_err(dev, "%s AxC%d err: container_size & sample width mismatch",
			axc_dir, axc->id);
		goto out;
	}

	/* Buffer size should be 16 byte aligned */
	if ((axc->buffer_size <= 0) || (axc->buffer_size % 16)) {
		dev_err(dev, "%s AxC%d err: buffer size invalid",
			axc_dir, axc->id);
		goto out;

	}
	if ((axc->buffer_threshold <= 0) || (axc->buffer_threshold % 16)) {
		dev_err(dev, "%s AxC%d err: buffer threhold invalid",
			axc_dir, axc->id);
		goto out;
	}
	if ((axc->buffer_offset_addr < 0) || (axc->buffer_offset_addr % 16)) {
		dev_err(dev, "%s AxC%d err: buffer base address invalid",
			axc_dir, axc->id);
		goto out;
	}
	if ((axc->buffer_size + axc->buffer_offset_addr)
		> framer->axc_memblk_size) {
		dev_err(dev, "%s AxC%d err: buffer size out of range",
			axc_dir, axc->id);
		goto out;
	}
	if (axc->id > (framer->max_axcs - 1)) {
		dev_err(dev, "%s AxC%d err: over max axc count %d",
			axc_dir, axc->id, framer->max_axcs);
		goto out;
	}

	if (axc->flags & AXC_DIR_TX) {
		axc_info_his = axc_info_tx;
		if (axc->memblk_sel & MEMBLK0)
			memblk_tx0++;
		else
			memblk_tx1++;
	} else {
		axc_info_his = axc_info_rx;
		if (axc->memblk_sel & MEMBLK1)
			memblk_rx0++;
		else
			memblk_rx1++;
	}

	if ((memblk_tx0 > 12) || (memblk_tx1 > 12) || (memblk_rx0 > 12)
		|| (memblk_rx1 > 12)) {
		dev_err(dev, "%s AxC%d err: memblk can't hold more than 12 AxCs",
				axc_dir, axc->id);
		goto out;
	}

	/* Checks if one AxC steps on other AxC */
	for (i = 0; i < framer->max_axcs; i++) {
		if (axc_info_his[i] == NULL)
			continue;
		else if (axc_info_his[i]->id == axc->id) {
			dev_err(dev, "%s AxC%d err: duplicate AxC id",
					axc_dir, axc->id);
			goto out;
		} else {
			ret = check_axc_overlap(axc, axc_info_his[i]);
				if (ret & 1) {
					dev_err(dev, "%s AxC%d and %s AxC%d overlap in basic frame",
						axc_dir, axc->id,
						axc_dir, axc_info_his[i]->id);
					goto out;
				} else if (ret & 2) {
					dev_err(dev, "%s AxC%d and %s AxC%d in memory buffer",
						axc_dir, axc->id,
						axc_dir, axc_info_his[i]->id);
					goto out;
				}
		}
	}

	if (axc->flags & AXC_DIR_TX)
		axc_info_tx[axc->id] = axc;
	else
		axc_info_rx[axc->id] = axc;

	return 0;
out:
	return -EINVAL;
}

/* This function handles the enable or disable of each AxC */
static int cpri_axc_ctrl(struct cpri_framer *framer,
		struct axc_info *axc_info, int axc_cnt)
{

	struct cpri_framer_regs __iomem *regs = framer->regs;
	int i;
	int err = 0;


	if (down_interruptible(&framer->axc_sem))
		return -EINTR;

	for (i = 0; i < axc_cnt; i++) {
		if (axc_info[i].flags & AXC_DAISY_CHAINED)
			program_aux_interface(framer, &axc_info[i]);

		if (axc_info[i].flags & AXC_DIR_RX) {
			if (axc_info[i].flags & AXC_EN) {
				cpri_reg_set(&regs->cpri_raccr,
					(AXC_ENABLE_MASK << axc_info[i].id));
				cpri_reg_set(&regs->cpri_raxciqvspthreshinten,
					(AXC_ENABLE_MASK << axc_info[i].id));
			} else {
				cpri_reg_clear(&regs->cpri_raccr,
					(AXC_ENABLE_MASK << axc_info[i].id));
				cpri_reg_clear(&regs->cpri_raxciqvspthreshinten,
					(AXC_ENABLE_MASK << axc_info[i].id));
			}
		} else if (axc_info[i].flags & AXC_DIR_TX) {
			if (axc_info[i].flags & AXC_EN) {
				cpri_reg_set(&regs->cpri_taccr,
					AXC_ENABLE_MASK << axc_info[i].id);
				cpri_reg_set(&regs->cpri_taxciqvspthreshinten,
					(AXC_ENABLE_MASK << axc_info[i].id));
			} else {
				cpri_reg_clear(&regs->cpri_taccr,
					(AXC_ENABLE_MASK << axc_info[i].id));
				cpri_reg_clear(&regs->cpri_taxciqvspthreshinten,
					(AXC_ENABLE_MASK << axc_info[i].id));
			}
		} else
			err = -EINVAL;
	}

	cpri_reg_set(&regs->cpri_tcr, 1);
	cpri_reg_set(&regs->cpri_rcr, 1);
	up(&framer->axc_sem);
	return err;
}

/* This function will program the mapping interface
 * and other AxC related reg.
 */
static int cpri_axc_map(struct cpri_framer *framer,
		struct axc_info *axc_info, int axc_cnt)
{
	struct cpri_framer_regs __iomem *regs = framer->regs;
	struct device *dev = framer->cpri_dev->dev;
	int i;
	int err = 0;
	int riqs, tiqs;
	struct axc_cmd_regs *cmd_regs = NULL;

	cmd_regs = kmalloc(sizeof(struct axc_cmd_regs), GFP_KERNEL);

	if (cmd_regs == NULL) {
		dev_err(dev, "System memory exhausted!\n");
		err = -ENOMEM;
		goto out1;
	}
	memset(cmd_regs, 0, sizeof(struct axc_cmd_regs));

	if (down_interruptible(&framer->axc_sem)) {
		err = -EINTR;
		goto out2;
	}
	/* Clear it once */
	check_axc_conflict(framer, NULL, 1);

	/* Program the sw mapping table */
	for (i = 0; i < axc_cnt; i++) {
		if (check_axc_conflict(framer, &axc_info[i], 0)) {
			err = -EINVAL;
			goto out3;
		}
	}

	cpri_reg_clear(&regs->cpri_tcr, 1);
	cpri_reg_clear(&regs->cpri_rcr, 1);

	/* Wait for rx/tx AxCs to be disabled */
	for (i = 0; i < 10; i++) {
		riqs = cpri_read(&regs->cpri_rstatus) & 0x1;
		tiqs = cpri_read(&regs->cpri_tstatus) & 0x1;
		if (!riqs && !tiqs)
			break;
		else
			msleep_interruptible(20);
	}
	if (i == 10) {
		dev_err(dev, "fail to disable riqs/tiqs bit");
		err = -EIO;
		goto out3;
	}

	/* Disable all AxCs before mapping */
	cpri_write(0, &regs->cpri_raccr);
	cpri_write(0, &regs->cpri_taccr);
	cpri_write(0, &regs->cpri_map_config);
	/* Clean the mapping table */
	clear_axc_map_tx_rx_table(framer);
	/* Clean the AUX table */
	for (i = 0; i < 64; i++)
		cpri_write(0, &regs->cpri_auxmask[i]);

	/* Program the sw mapping table */
	for (i = 0; i < axc_cnt; i++) {
		if (!(axc_info[i].flags & AXC_DAISY_CHAINED))
			fill_map_table(axc_info[i], cmd_regs);
	}
	/* Program the hardware mapping table */
	map_axc_tx_rx_table(framer, cmd_regs);

	for (i = 0; i < axc_cnt; i++) {
		if (!(axc_info[i].flags & AXC_DAISY_CHAINED))
			program_axc_conf_reg(framer, &axc_info[i]);
	}

	/* Set k1, k2 value and map mode */
	cpri_reg_set_val(&regs->cpri_map_tbl_config, AXC_K0_MASK, 1);
	cpri_reg_set_val(&regs->cpri_map_tbl_config, AXC_K1_MASK, 1);
	cpri_reg_clear(&regs->cpri_map_k_select_rx1, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_k_select_tx1, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_k_select_rx2, MASK_ALL);
	cpri_reg_clear(&regs->cpri_map_k_select_tx2, MASK_ALL);
	cpri_write(framer->max_axcs, &regs->cpri_rgenmode);
	cpri_write(framer->max_axcs, &regs->cpri_tgenmode);
	cpri_reg_set_val(&regs->cpri_map_config, AXC_MODE_MASK, 1);

out3:
	up(&framer->axc_sem);
out2:
	kfree(cmd_regs);
out1:
	return err;
}

int cpri_axc_ioctl(struct cpri_framer *framer, unsigned long arg,
		unsigned int cmd)
{
	int err = 0;
	struct device *dev = framer->cpri_dev->dev;
	void __user *ioargp = (void __user *)arg;
	struct axc_config axc_config;
	struct axc_info *axc_info;
	int axc_cnt;
	int size;

	if (copy_from_user(&axc_config,
				(struct axc_config *) ioargp,
				sizeof(struct axc_config)) != 0) {
		dev_err(dev, "Copy from user failed\n");
		return -EFAULT;
	}

	axc_cnt = axc_config.axc_cnt;
	size = axc_cnt * sizeof(struct axc_info);
	axc_info = kzalloc(size, GFP_KERNEL);

	if (axc_info == NULL) {
		dev_err(dev, "System memory exhausted!\n");
		return -ENOMEM;
	}

	if (copy_from_user(axc_info, axc_config.axc_info, size) != 0) {
		dev_err(dev, "Copy from user failed\n");
		err = -EFAULT;
		goto out;
	}

	switch (cmd) {
	case CPRI_AXC_MAP:
		err = cpri_axc_map(framer, axc_info, axc_cnt);
		break;

	case CPRI_AXC_CTRL:
		err = cpri_axc_ctrl(framer, axc_info, axc_cnt);
		break;

	default:
		goto out;
	}

	return 0;
out:
	kfree(axc_info);
	if (err)
		dev_err(dev, "AxC ioctl failure\n");
	return err;
}

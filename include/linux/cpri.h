/*
 * include/linux/cpri.h
 * CPRI device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __CPRI_H
#define __CPRI_H

#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/bitops.h>
#include <linux/semaphore.h>

#include <uapi/linux/cpri.h>
#include <linux/sfp.h>
#include <linux/gcr.h>
#include <linux/cpri_eth.h>
#include <mach/src.h>

struct cpri_framer;
struct sfp_dev;

/* CPRI base prototypes */
long cpri_ioctl(struct file *fp, unsigned int cmd,
			unsigned long arg);

/* Autoneg prototypes */
int cpri_autoneg_ioctl(struct cpri_framer *framer, unsigned int cmd,
			unsigned long arg);
/* Control word function prototypes */
void clear_control_tx_table(struct cpri_framer *framer);
void clear_axc_map_tx_rx_table(struct cpri_framer *framer);

/* Ethernet exported functions */
extern int cpri_eth_init(struct platform_device *ofdev,
			struct cpri_framer *framer,
			struct device_node *frnode);
extern void cpri_eth_deinit(struct platform_device *ofdev,
		struct cpri_framer *framer);
extern int cpri_eth_handle_rx(struct cpri_framer *framer);
extern int cpri_eth_handle_tx(struct cpri_framer *framer);
extern int cpri_eth_handle_error(struct cpri_framer *framer);

/* SFP exported functions */
extern void cpri_framer_handle_sfp_pin_changes(struct cpri_framer *framer,
					unsigned changed, unsigned state);
extern struct cpri_framer
	*get_attached_cpri_dev(struct device_node **sfp_dev_node);
extern void sfp_set_attached_framer(struct sfp_dev *sfp,
					struct cpri_framer *framer);
extern struct sfp_dev *get_attached_sfp_dev(struct device_node *sfp_dev_node);
extern void sfp_set_tx_enable(struct sfp_dev *sfp, unsigned value);
extern int sfp_raw_write(struct sfp_dev *sfp, u8 *buf, u8 offset,
		unsigned int count, enum mem_type type);
extern int sfp_raw_read(struct sfp_dev *sfp, u8 *buf, u8 offset,
		unsigned int count, enum mem_type type);
extern int sfp_check_gpios(struct sfp_dev *sfp);
extern void d4400_rev_clk_select(u8 cpri_id, u8 clk_dev);
extern int sfp_update_realtime_info(struct sfp_dev *sfp);
extern bool sfp_is_ready(const struct sfp_dev *sfp);
extern bool sfp_has_rx_signal(const struct sfp_dev *sfp);
extern bool sfp_has_tx_fault(const struct sfp_dev *sfp);

/* AxC mapping functions */
int cpri_axc_ioctl(struct cpri_framer *framer, unsigned long arg,
		unsigned int cmd);
void cpri_mask_irq_events(struct cpri_framer *framer);
void init_eth(struct cpri_framer *framer);
struct cpri_dev *get_pair_cpri_dev(struct cpri_dev *cpri_dev);
signed int set_sfp_input_amp_limit(struct cpri_framer *framer,
		u32 max_volt, u8 flag);
void read_rx_cw(struct cpri_framer *framer, int bf_index, u8 *buf);
void rdwr_tx_cw(struct cpri_framer *framer,
			int bf_index, int operation, u8 *buf);
void cpri_set_monitor(struct cpri_framer *framer,
		const struct monitor_config_en *monitor_cfg_en);
void cpri_clear_monitor(struct cpri_framer *framer,
		const struct monitor_config_disable *monitor_cfg_dis);
void src_cpri_hwrst(int enable);

void cpri_cw130_config(struct cpri_framer *framer, u32 enable_mask);
#endif /* __CPRI_H */

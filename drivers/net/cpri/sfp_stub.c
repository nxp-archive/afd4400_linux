/*
 * drivers/net/cpri/sfp_stub.c
 * Stub SFP device driver (in case SFP driver is not built)
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2016 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/cpri.h>
#include "cpri.h"

__attribute__((weak))
struct sfp_dev *get_attached_sfp_dev(struct device_node *sfp_dev_node)
{
	return NULL;
}

__attribute__((weak))
void sfp_set_attached_framer(struct sfp_dev *sfp, struct cpri_framer *framer)
{
	if (sfp)
		sfp->attached_framer = framer;
}

__attribute__((weak))
int sfp_raw_write(struct sfp_dev *sfp,
		u8 *buf,
		u8 offset,
		unsigned int count,
		enum mem_type type)
{
	return -ENODEV;
}

__attribute__((weak))
int sfp_raw_read(struct sfp_dev *sfp,
		u8 *buf,
		u8 offset,
		unsigned int count,
		enum mem_type type)
{
	return -ENODEV;
}

__attribute__((weak))
void sfp_set_tx_enable(struct sfp_dev *sfp, unsigned value)
{
}

__attribute__((weak))
int sfp_check_gpios(struct sfp_dev *sfp)
{
	return 0;
}

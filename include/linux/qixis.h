/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 */

#ifndef __QIXIS_H__
#define __QIXIS_H__

#include <uapi/linux/qixis.h>

extern int gcr_jesd_init(void);
extern int gcr_jesd_en_termination(void);

int qixis_board_type(void);
const char *qixis_board_type_name(void);
int qixis_board_rev(void);
char qixis_board_rev_letter(void);
int qixis_jcpll_write_reg(u8 val, u16 offset);
int qixis_jcpll_read_reg(u16 offset);
int qixis_jcpll_freq_fixed(void);
int qixis_jcpll_freq_track(void);
int qixis_jcpll_use_refc(int enable);
int qixis_jcpll_locked(void);
int qixis_xcvr_present(int xcvr_id);
int qixis_leds_set_clear(unsigned int set, unsigned int clear);
int qixis_set_reboot_method(unsigned int method);
int qixis_xcvr_eeprom_power(void);

#endif

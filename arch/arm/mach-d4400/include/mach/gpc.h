/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#ifndef _MACH_GPC_H_
#define  _MACH_GPC_H_
/* vspa_id max = 10 , start ID = 0 */
extern int d4400_gpc_vspa_full_pow_up(u8 vspa_id);
extern int d4400_gpc_vspa_full_pow_gate(u8 vspa_id);
extern int d4400_gpc_vspa_full_pow(u8 vspa_id);
#endif

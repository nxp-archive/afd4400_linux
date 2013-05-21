/*
     * Copyright 2013 Freescale Semiconductor, Inc. All Rights Reserved.
     *
     * This program is free software; you can redistribute it and/or
     * modify it under the terms of the GNU General Public License
     * as published by the Free Software Foundation; either version 2
     * of the License, or (at your option) any later version.
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

#ifndef __ASM_ARCH_D4400_H__
#define __ASM_ARCH_D4400_H__

#include <linux/types.h>

#ifndef __ASM_ARCH_D4400_HARDWARE_H__
#error "Do not include directly."
#endif

#define D4400_CHIP_REVISION_1_0		0x10
#define D4400_CHIP_REVISION_UNKNOWN	0xff

#ifndef __ASSEMBLY__
extern unsigned int __d4400_cpu_type;
#endif

#endif /*  __ASM_ARCH_D4400_H__ */


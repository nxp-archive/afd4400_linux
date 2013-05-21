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
 * MA  02110-1301, USA.
 */

#ifndef __ASM_ARCH_D4400_HARDWARE_H__
#define __ASM_ARCH_D4400_HARDWARE_H__

#include <asm/sizes.h>

#define addr_in_module(addr, mod) \
	((unsigned long)(addr) - mod ## _BASE_ADDR < mod ## _SIZE)

#define D4400_IO_P2V_MODULE(addr, module)				\
	(((addr) - module ## _BASE_ADDR) < module ## _SIZE ?		\
	 (addr) - (module ## _BASE_ADDR) + (module ## _BASE_ADDR_VIRT) : 0)

/*FIXME-D4400
 * This is rather complicated for humans and ugly to verify, but for a machine
 * it's OK.  Still more as it is usually only applied to constants.  The upsides
 * on using this approach are:
 *
 *  - mapping on D4400 machine
 *  - works for assembler, too
 *  - no need to nurture #defines for virtual addresses
 *
 * The downside it, it's hard to verify.
 *
 * Obviously this needs to be injective for each SoC.  In general it maps the
 * whole address space to [0xee000000, 0xefffffff].
 *
 * It applies the following mappings for the d4400 SoC:
 *
 * CCM	        0x01094000+0x004000	->	0xee094000+0x004000
 * UART4	0x010f8000+0x004000	->	0xef0f8000+0x004000
 */
#define D4400_IO_P2V(x)	(						\
			(((x) & 0x80000000) >> 7) |			\
			(0xee000000 +					\
			(((x) & 0x50000000) >> 6) +			\
			(((x) & 0x0b000000) >> 4) +			\
			(((x) & 0x000fffff))))
/* FIXME-D4400 */
#define D4400_IO_ADDRESS(x)	IOMEM(D4400_IO_P2V(x))

#include "d4400.h"

#define d4400_map_entry(soc, name, _type)	{			\
	.virtual = soc ## _IO_P2V(soc ## _ ## name ## _BASE_ADDR),	\
	.pfn = __phys_to_pfn(soc ## _ ## name ## _BASE_ADDR),		\
	.length = soc ## _ ## name ## _SIZE,				\
	.type = _type,							\
}

/* There's a off-by-one betweem the gpio bank number and the gpiochip */
/* range e.g. GPIO_1_5 is gpio 5 under linux */
#define D4400_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))

#endif /* __ASM_ARCH_D4400_HARDWARE_H__ */


/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
*/

#include <linux/init.h>
#include <asm/page.h>
#include <asm/sizes.h>
#include <asm/mach/map.h>

#include "hardware.h"

#define D4400_UART1_BASE_ADDR	0x010EC000
#define D4400_UART2_BASE_ADDR	0x010F0000
#define D4400_UART3_BASE_ADDR	0x010F4000
#define D4400_UART4_BASE_ADDR	0x010F8000

/*
 * D4400_UART_BASE_ADDR is put in the middle to force the expansion
 * of D4400_UART##n##_BASE_ADDR.
 */
#define D4400_UART_BASE_ADDR(n)	D4400_UART##n##_BASE_ADDR
#define D4400_UART_BASE(n)	D4400_UART_BASE_ADDR(n)
#define D4400_DEBUG_UART_BASE	D4400_UART_BASE(CONFIG_DEBUG_D4400_UART_PORT)

static struct map_desc d4400_lluart_desc = {
#ifdef CONFIG_DEBUG_D4400_UART
	.virtual	= D4400_IO_P2V(D4400_DEBUG_UART_BASE),
	.pfn		= __phys_to_pfn(D4400_DEBUG_UART_BASE),
	.length		= 0x4000,
	.type		= MT_DEVICE,
#endif
};

void __init d4400_lluart_map_io(void)
{
	if (d4400_lluart_desc.virtual)
		iotable_init(&d4400_lluart_desc, 1);
}


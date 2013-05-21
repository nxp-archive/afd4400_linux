/* arch/arm/mach-imx/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 *  Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifdef CONFIG_DEBUG_D4400_UART
#define D4400_UART1_BASE_ADDR   0x010EC000
#define D4400_UART2_BASE_ADDR   0x010F0000
#define D4400_UART3_BASE_ADDR   0x010F4000
#define D4400_UART4_BASE_ADDR   0x010F8000

/*
 * D4400_UART_BASE_ADDR is put in the middle to force the expansion
 * of D4400_UART##n##_BASE_ADDR.
 */
#define D4400_UART_BASE_ADDR(n) D4400_UART##n##_BASE_ADDR
#define D4400_UART_BASE(n)      D4400_UART_BASE_ADDR(n)
#define D4400_DEBUG_UART_BASE   D4400_UART_BASE(CONFIG_DEBUG_D4400_UART_PORT)
#define D4400_IO_P2V(x) (                                               \
                        (((x) & 0x80000000) >> 7) |                     \
                        (0xee000000 +                                   \
                        (((x) & 0x50000000) >> 6) +                     \
                        (((x) & 0x0b000000) >> 4) +                     \
                        (((x) & 0x000fffff))))
/* FIXME-D4400 */
#define UART_VADDR	D4400_IO_P2V(UART_PADDR)
#else


#define IMX6Q_UART1_BASE_ADDR	0x02020000
#define IMX6Q_UART2_BASE_ADDR	0x021e8000
#define IMX6Q_UART3_BASE_ADDR	0x021ec000
#define IMX6Q_UART4_BASE_ADDR	0x021f0000
#define IMX6Q_UART5_BASE_ADDR	0x021f4000

/*
 * IMX6Q_UART_BASE_ADDR is put in the middle to force the expansion
 * of IMX6Q_UART##n##_BASE_ADDR.
 */
#define IMX6Q_UART_BASE_ADDR(n)	IMX6Q_UART##n##_BASE_ADDR
#define IMX6Q_UART_BASE(n)	IMX6Q_UART_BASE_ADDR(n)
#define IMX6Q_DEBUG_UART_BASE	IMX6Q_UART_BASE(CONFIG_DEBUG_IMX6Q_UART_PORT)
/*
 * FIXME: This is a copy of IMX_IO_P2V in hardware.h, and needs to
 * stay sync with that.  It's hard to maintain, and should be fixed
 * globally for multi-platform build to use a fixed virtual address
 * for low-level debug uart port across platforms.
 */
#define IMX_IO_P2V(x)	(						\
			(((x) & 0x80000000) >> 7) |			\
			(0xf4000000 +					\
			(((x) & 0x50000000) >> 6) +			\
			(((x) & 0x0b000000) >> 4) +			\
			(((x) & 0x000fffff))))

#define UART_VADDR	IMX_IO_P2V(UART_PADDR)
#endif


#ifdef CONFIG_DEBUG_IMX1_UART
#define UART_PADDR	0x00206000
#elif defined (CONFIG_DEBUG_IMX25_UART)
#define UART_PADDR	0x43f90000
#elif defined (CONFIG_DEBUG_IMX21_IMX27_UART)
#define UART_PADDR	0x1000a000
#elif defined (CONFIG_DEBUG_IMX31_IMX35_UART)
#define UART_PADDR	0x43f90000
#elif defined (CONFIG_DEBUG_IMX51_UART)
#define UART_PADDR	0x73fbc000
#elif defined (CONFIG_DEBUG_IMX50_IMX53_UART)
#define UART_PADDR	0x53fbc000
#elif defined (CONFIG_DEBUG_IMX6Q_UART)
#define UART_PADDR	IMX6Q_DEBUG_UART_BASE
#elif defined (CONFIG_DEBUG_D4400_UART)
#define UART_PADDR	D4400_DEBUG_UART_BASE
#endif

		.macro	addruart, rp, rv, tmp
		ldr	\rp, =UART_PADDR	@ physical
		ldr	\rv, =UART_VADDR	@ virtual
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #0x40]	@ TXDATA
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
1002:		ldr	\rd, [\rx, #0x98]	@ SR2
		tst	\rd, #1 << 3		@ TXDC
		beq	1002b			@ wait until transmit done
		.endm

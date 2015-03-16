/*
 * Copyright 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_D4400_COMMON_H__
#define __ASM_ARCH_D4400_COMMON_H__

#define BITS_MASK(len, offset)  (len << offset)

struct platform_device;
struct clk;

extern void epit_timer_init(void __iomem *base, int irq);
extern int d4400_clocks_init(void);
extern struct platform_device *d4400_register_gpio(char *name, int id,
			resource_size_t iobase, resource_size_t iosize,
			int irq, int irq_high);
extern void d4400_set_cpu_type(unsigned int type);
extern void d4400_restart(char, const char *);

extern void d4400_print_silicon_rev(const char *cpu, int srev);

#define d4400_handle_irq gic_handle_irq

#ifdef CONFIG_DEBUG_LL
extern void d4400_lluart_map_io(void);
#else
static inline void d4400_lluart_map_io(void) {}
#endif
extern void d4400_clock_map_io(void);

#ifdef CONFIG_PM
extern void d4400_pm_init(void);
#else
static inline void d4400_pm_init(void) {}
#endif

extern int d4400_ccm_vspa_full_pow_up(u8 vspa_id);
extern int d4400_ccm_vspa_full_pow_gate(u8 vspa_id);
extern int d4400_ccm_vspa_full_pow(u8 vspa_id);

extern void d4400_gpc_init(void);
extern int d4400_scm_init(void);

#endif


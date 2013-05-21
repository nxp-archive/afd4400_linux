/*
 * D4400 pinmux core definitions
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DRIVERS_PINCTRL_D4400_H
#define __DRIVERS_PINCTRL_D4400_H

struct platform_device;

/**
 * struct d4400_pin_group - describes an D4400 pin group
 * @name: the name of this specific pin group
 * @pins: an array of discrete physical pins used in this group, taken
 *	from the driver-local pin enumeration space
 * @npins: the number of pins in this group array, i.e. the number of
 *	elements in .pins so we can iterate over that array
 * @mux_mode: the mux mode for each pin in this group. The size of this
 *	array is the same as pins.
 * @configs: the config for each pin in this group. The size of this
 *	array is the same as pins.
 */
struct d4400_pin_group {
	const char *name;
	unsigned int *pins;
	unsigned npins;
	unsigned int *mux_mode;
	unsigned long *configs;
};

/**
 * struct d4400_pmx_func - describes D4400 pinmux functions
 * @name: the name of this specific function
 * @groups: corresponding pin groups
 * @num_groups: the number of groups
 */
struct d4400_pmx_func {
	const char *name;
	const char **groups;
	unsigned num_groups;
};

/**
 * struct d4400_pin_reg - describe a pin reg map
 * @pid: pin id
 * @mux_reg: mux register offset
 * @conf_reg: config register offset
 * @mux_mode: mux mode
 */
struct d4400_pin_reg {
	u16 pid;
	u16 mux_reg;
	u16 conf_reg;
	u8 mux_mode;
};

struct d4400_pinctrl_soc_info {
	struct device *dev;
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
	const struct d4400_pin_reg *pin_regs;
	unsigned int npin_regs;
	struct d4400_pin_group *groups;
	unsigned int ngroups;
	struct d4400_pmx_func *functions;
	unsigned int nfunctions;
};

#define NO_MUX		0xFFFF
#define NO_PAD		0xFFFF

#define D4400_PIN_REG(id, conf, mux, mode)	\
	{					\
		.pid = id,			\
		.conf_reg = conf,		\
		.mux_reg = mux,			\
		.mux_mode  = mode,		\
	}

#define D4400_PINCTRL_PIN(pin) PINCTRL_PIN(pin, #pin)

#define PAD_CTL_MASK(len)	((1 << len) - 1)
#define D4400_MUX_MASK	0x7F
#define D4400_PAD_MAIN_MASK 0x3FFF
#define D4400_PAD_DDR_MASK 0x1FF

int d4400_pinctrl_remove(struct platform_device *pdev);
#endif /* __DRIVERS_PINCTRL_D4400_H */

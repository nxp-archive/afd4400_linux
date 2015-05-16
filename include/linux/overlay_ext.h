/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __LINUX_OVERLAY_EXT_H
#define __LINUX_OVERLAY_EXT_H

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/linkage.h>

/**
 * These flags are used for the static initialization of struct property.
 */
#define PROP_NUM	(1) /* Treat property value as number(s) */
#define PROP_STR	(2) /* Treat property value as string(s) */

/**
 * struct fragment_node	- Holds a complete overlay fragment node.
 *
 * @target:	The target string specifies a node reference in the live
 *              tree to overlay.
 * @ov_prop_array: Points to an array of properties to be added/modified
 *              in the target node.
 * @np_array:   Points to an array of nodes to create in the target node.
 *              Each node in the array can point to arrays of properties
 *              and child nodes to be created.
 */
struct fragment_node {
	char *target;
	struct property *ov_prop_array;
	struct device_node *np_array;
};

extern struct device_node *ov_create_tree(struct fragment_node *fnp);
extern int ov_load(struct fragment_node *fnp_array);
#endif /* __LINUX_OVERLAY_EXT_H */

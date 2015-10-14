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

#ifndef _UTIL_D4400_H
#define _UTIL_D4400_H

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/linkage.h>

/* Name string is matched against board name.  Only
 * part of the substring needs to match.
 */
#define D4400_EVB_NAME_STR	"EVB"
#define D4400_RDB_NAME_STR	"RDB"
#define D4400_4T4R_NAME_STR	"4T4R"

/* Name string is matched against revision name.  Only
 * part of the substring needs to match.
 */
#define REVA_STR	"REV A"
#define REVB_STR	"REV B"
#define REVC_STR	"REV C"
#define REVD_STR	"REV D"
#define REVE_STR	"REV E"

enum board_type {
        FSL_BOARD_TYPE_UNKNOWN = -1,
        FSL_BOARD_TYPE_D4400EVB = 0,
        FSL_BOARD_TYPE_D4400RDB = 1,
        FSL_BOARD_TYPE_D44004T4R = 2,
};

enum board_rev {
        FSL_BOARD_REV_UNKNOWN = -1,
        FSL_BOARD_REVA = 0,
        FSL_BOARD_REVB = 1,
        FSL_BOARD_REVC = 2,
        FSL_BOARD_REVD = 3,
        FSL_BOARD_REVE = 4,
};

int fsl_get_board_type(char *name_str);
int fsl_get_board_rev(char *partnum_str);
enum board_type ipmi_get_board_type(char *name_str);
enum board_rev ipmi_get_board_rev(char *partnum_str);

#endif /* _UTIL_D4400_H */

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

#include <linux/types.h>
#include <linux/util-d4400.h>

int fsl_get_board_type(char *name_str)
{
	enum board_type type = BOARD_TYPE_UNKNOWN;

	if (strstr(name_str, D4400_EVB_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_EVB;
	else if (strstr(name_str, D4400_RDB_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_RDB;
	else if (strstr(name_str, D4400_4T4RK1_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_4T4RK1;
	else if (strstr(name_str, D4400_4T4R_NAME_STR) != NULL)
		/* strstr() used for catch-all for boards with "4T4R"
		 * substring.
		*/
		type = BOARD_TYPE_D4400_4T4R;
	else if (strstr(name_str, D4400_21RRH_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_21RRH;

	return type;
}

int fsl_get_board_rev(char *partnum_str)
{
	enum board_rev rev = BOARD_REV_UNKNOWN;

	if (strstr(partnum_str, REVA_STR) != NULL)
		rev = BOARD_REVA;
	else if (strstr(partnum_str, REVB_STR) != NULL)
		rev = BOARD_REVB;
	else if (strstr(partnum_str, REVC_STR) != NULL)
		rev = BOARD_REVC;
	else if (strstr(partnum_str, REVD_STR) != NULL)
		rev = BOARD_REVD;
	else if (strstr(partnum_str, REVE_STR) != NULL)
		rev = BOARD_REVE;

	return rev;
}

enum board_type ipmi_get_board_type(char *name_str)
{
	enum board_type type = BOARD_TYPE_UNKNOWN;

	if (strstr(name_str, D4400_EVB_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_EVB;
	else if (strstr(name_str, D4400_RDB_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_RDB;
	else if (strstr(name_str, D4400_4T4RK1_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_4T4RK1;
	else if (strstr(name_str, D4400_4T4R_NAME_STR) != NULL)
		/* strstr() used for catch-all for boards with "4T4R"
		 * substring.
		*/
		type = BOARD_TYPE_D4400_4T4R;
	else if (strstr(name_str, D4400_21RRH_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_21RRH;

	return type;
}

enum board_rev ipmi_get_board_rev(char *partnum_str)
{
	enum board_rev rev = BOARD_REV_UNKNOWN;

	if (strstr(partnum_str, REVA_STR) != NULL)
		rev = BOARD_REVA;
	else if (strstr(partnum_str, REVB_STR) != NULL)
		rev = BOARD_REVB;
	else if (strstr(partnum_str, REVC_STR) != NULL)
		rev = BOARD_REVC;
	else if (strstr(partnum_str, REVD_STR) != NULL)
		rev = BOARD_REVD;
	else if (strstr(partnum_str, REVE_STR) != NULL)
		rev = BOARD_REVE;

	return rev;
}

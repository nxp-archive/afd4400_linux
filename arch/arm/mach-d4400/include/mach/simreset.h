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

#ifndef __LINUX_SIMRESET_H
#define __LINUX_SIMRESET_H

#include <linux/unistd.h>
#include <linux/linkage.h>

/* CCM Clock module address range */
#define D4400_CCM_MODULE_ADDRESS_START	(0x01094000)
#define D4400_CCM_MODULE_ADDRESS_END	(0x01097ffc)

/* L2 cache control registers */
#define L2CC_CACHE_CONTROL_REG_BASE_ADDRESS (0x10900000)
#define L2CC_REG1_CONTROL_OFFSET	(0x100)
#define L2CC_REG1_AUX_CONTROL_OFFSET	(0x104)
#define L2CC_INT_MASK_OFFSET		(0x214)
#define L2CC_INT_CLEAR_OFFSET		(0x220)
#define L2CC_REG15_PREFETCH_CONTROL_OFFSET (0xf60)
#define L2CC_REG7_INV_PA_OFFSET		(0x770)
#define L2CC_REG7_INV_WAY_OFFSET	(0x77c)
#define L2CC_REG7_CLEAN_PA_OFFSET	(0x7b0)
#define L2CC_REG7_CLEAN_INDEX_OFFSET	(0x7b8)
#define L2CC_REG7_CLEAN_WAY_OFFSET	(0x7bc)
#define L2CC_REG7_CLEAN_INV_PA_OFFSET	(0x7f0)
#define L2CC_REG7_CLEAN_INV_INDEX_OFFSET (0x7f8)
#define L2CC_REG7_CLEAN_INV_WAY_OFFSET	(0x7fc)

#define L2CC_REG1_CONTROL_REG \
	(L2CC_REG1_CONTROL_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG1_AUX_CONTROL_REG \
	(L2CC_REG1_AUX_CONTROL_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_INT_MASK_REG \
	(L2CC_INT_MASK_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_INT_CLEAR_REG \
	(L2CC_INT_CLEAR_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_INV_PA_REG \
	(L2CC_REG7_INV_PA_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_INV_WAY_REG \
	(L2CC_REG7_INV_WAY_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_CLEAN_PA_REG \
	(L2CC_REG7_CLEAN_PA_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_CLEAN_INDEX_REG \
	(L2CC_REG7_CLEAN_INDEX_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_CLEAN_WAY_REG \
	(L2CC_REG7_CLEAN_WAY_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_CLEAN_INV_PA_REG \
	(L2CC_REG7_CLEAN_INV_PA_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_CLEAN_INV_INDEX_REG \
	(L2CC_REG7_CLEAN_INV_INDEX_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)
#define L2CC_REG7_CLEAN_INV_WAY_REG \
	(L2CC_REG7_CLEAN_INV_WAY_OFFSET +\
	L2CC_CACHE_CONTROL_REG_BASE_ADDRESS)

/* System reset controller SBMR register */
#define SRC_REG_BASE_ADDRESS		(0x108c000)
#define SRC_SBMR_REG_ADRESS		(0x0 + SRC_REG_BASE_ADDRESS)
#define SRC_SBMR_BMOD_OFFSET		0
#define SRC_SBMR_BMOD_MASK		(0x3 << SRC_SBMR_BMOD_OFFSET)
#define SRC_SBMR_SER_DL_SEL_OFFSET	3
#define SRC_SBMR_SER_DL_SEL_MASK	(1 << SRC_SBMR_SER_DL_SEL_OFFSET)
#define SRC_SBMR_MEM_TYPE_OFFSET	4
#define SRC_SBMR_MEM_TYPE_MASK		(1 << SRC_SBMR_MEM_TYPE_OFFSET)
#define SRC_SBMR_MEM_BUS_WIDTH_OFFSET	5
#define SRC_SBMR_MEM_BUS_WIDTH_MASK	(0x3 << SRC_SBMR_MEM_BUS_WIDTH_OFFSET)
#define SRC_SBMR_ETH_MODE_OFFSET	10
#define SRC_SBMR_ETH_MODE_MASK		(0x3 << SRC_SBMR_ETH_MODE_OFFSET)
#define SRC_SBMR_BIV_EN_OFFSET		12
#define SRC_SBMR_BIV_EN_MASK		(1 << SRC_SBMR_BIV_EN_OFFSET)
#define SRC_SBMR_SPI_FLASH_VENID_OFFSET	13
#define SRC_SBMR_SPI_FLASH_VENID_MASK	(0x3 << SRC_SBMR_SPI_FLASH_VENID_OFFSET)
#define SRC_SBMR_REF_FREQ_OFFSET	15
#define SRC_SBMR_REF_FREQ_OFFSET_MASK	(1 << SRC_SBMR_REF_FREQ_OFFSET)

/* **************************************************************************
 * The internal RAM usage for simulated restart is detailed here.
 *
 * The selection of internal RAM location to use is done mainly with the
 * D4400_INT_RAM_DTS_ENTRY_NODE_STR define which points to the OCRAM in DTS
 * tree to use.
 *
 * The memory map of the internal RAM usage is as follows:
 *
 *             Internal RAM Map
 *  +-----------------------------------------+ <- Internal RAM end
 *  |   Simrestart Marker (4 bytes)           |	   (top of memory)
 *  |                                         |
 *  |   See defines:                          |
 *  |   D4400_SIMMARKER_MAX_SIZE_BYTES        |
 *  |   D4400_INT_RAM_SIMMARKER_OFFSET        |
 *  |   D4400_INT_RAM_SIMMARKER_ADDRESS       |
 *  +-----------------------------------------+ <- (Internal RAM end - 4 bytes)
 *  |   Reserved for future use (28 bytes)    |
 *  |                                         |
 *  |   See defines:                          |
 *  |   D4400_RESERVED1_MAX_SIZE_BYTES        |
 *  |   D4400_INT_RAM_RESERVED1_OFFSET        |
 *  |   D4400_INT_RAM_RESERVED1_ADDRESS       |
 *  +-----------------------------------------+ <- (Internal RAM end - 32 bytes)
 *  |   IVT table (includes DCD)              |
 *  |   copied from flash to here.            |
 *  |                                         |
 *  |   See defines:                          |
 *  |   D4400_IVT_MAX_SIZE_BYTES              |
 *  |   D4400_INT_RAM_IVT_OFFSET              |
 *  |   D4400_INT_RAM_IVT_ADDRESS             |
 *  |                                         |
 *  |                                         |
 *  |                                         |
 *  |                                         |
 *  +-----------------------------------------+
 *  |   Register default table copied         |
 *  |   from d4400_reg_default[] to           |
 *  |   here.                                 |
 *  |                                         |
 *  |   See defines:                          |
 *  |   D4400_REG_DEFAULT_DATA_MAX_SIZE_BYTES |
 *  |   D4400_INT_RAM_REG_DEFAULT_OFFSET      |
 *  |   D4400_INT_RAM_REG_DEFAULT_ADDRESS     |
 *  |                                         |
 *  |                                         |
 *  |                                         |
 *  |                                         |
 *  +-----------------------------------------+
 *  |   Start of stack, grows downwards.      |
 *  |                                         |
 *  |   See define:                           |
 *  |   D4400_INT_RAM_STACK_ADDRESS           |
 *  | - - - - - - - - - - - - - - - - - - - - |
 *  |                                         |
 *  |                                         |
 *  |                                         |
 *  |                                         |
 *  |  Stub function code start at beginning  |
 *  |  of internal RAM.                       |
 *  |                                         |
 *  |  See defines:                           |
 *  |  D4400_STUB_FUNC_SIZE_MAX_BYTES         |
 *  |  D4400_INT_RAM_STUB_FUNC_OFFSET         |
 *  +-----------------------------------------+ <- Internal RAM start
 *                                                  (bottom of memory)
 */

/* Various internal RAM locations */
#define D4400_OCRAM1_START_ADDRESS	(0x60000000)
#define D4400_OCRAM1_SIZE_BYTES		(64 * 1024)	/* 64KB   */
#define D4400_OCRAM2_START_ADDRESS	(0x61000000)
#define D4400_OCRAM2_SIZE_BYTES		(64 * 1024)	/* 64KB   */
#define D4400_OCRAM3_START_ADDRESS	(0x70000000)
#define D4400_OCRAM3_SIZE_BYTES		(64 * 1024)	/* 64KB   */
#define D4400_OCRAM4_START_ADDRESS	(0x71000000)
#define D4400_OCRAM4_SIZE_BYTES		(64 * 1024)	/* 64KB   */
#define D4400_OCRAM5_START_ADDRESS	(0xF0000000)
#define D4400_OCRAM5_SIZE_BYTES		(64 * 1024)	/* 64KB   */
#define D4400_OCRAM6_START_ADDRESS	(0xF1000000)
#define D4400_OCRAM6_SIZE_BYTES		(64 * 1024)	/* 64KB   */

/* Internal RAM used to run stub function. */
#define D4400_INT_RAM_DTS_ENTRY_NODE_STR "fsl,d4400-ocram1"
#define D4400_INT_RAM_START_ADDRESS	D4400_OCRAM1_START_ADDRESS
#define D4400_INT_RAM_SIZE_BYTES	D4400_OCRAM1_SIZE_BYTES

/*
 * Simrestart marker is to indicate to Linux that the reboot is
 * simulated.  Linux can use this information during its initialization
 * and perform accordingly.  The marker is located at the end of
 * internal RAM.
 */
#define D4400_SIMMARKER_MAX_SIZE_BYTES	(4)
#define D4400_SIMMARKER_VALUE_WORD	(0x92914849)
#define D4400_INT_RAM_SIMMARKER_OFFSET \
	(D4400_INT_RAM_SIZE_BYTES - D4400_SIMMARKER_MAX_SIZE_BYTES)
#define D4400_INT_RAM_SIMMARKER_ADDRESS \
	(D4400_INT_RAM_START_ADDRESS + D4400_INT_RAM_SIMMARKER_OFFSET)

/* Reserved area for future use. */
#define D4400_RESERVED1_MAX_SIZE_BYTES	(28)
#define D4400_INT_RAM_RESERVED1_OFFSET \
		(D4400_INT_RAM_SIZE_BYTES - D4400_RESERVED1_MAX_SIZE_BYTES -\
		D4400_SIMMARKER_MAX_SIZE_BYTES)
#define D4400_INT_RAM_RESERVED1_ADDRESS \
		(D4400_INT_RAM_START_ADDRESS + D4400_INT_RAM_SIMMARKER_OFFSET)

/*
 * IVT (Image Vector Table) table is currently located towards the end
 * of internal RAM before the simrestart marker.
 */
#define D4400_IVT_MAX_SIZE_BYTES	(4 * 1024)
#define D4400_INT_RAM_IVT_OFFSET \
	(D4400_INT_RAM_SIZE_BYTES - D4400_IVT_MAX_SIZE_BYTES -\
	D4400_RESERVED1_MAX_SIZE_BYTES - D4400_SIMMARKER_MAX_SIZE_BYTES)
#define D4400_INT_RAM_IVT_ADDRESS \
		(D4400_INT_RAM_START_ADDRESS + D4400_INT_RAM_IVT_OFFSET)

/*
 * Register default data table is located towards end of internal RAM
 * before the IVT data. It has the same structure as the DCD table.
 */
/* 1KB => 85 entries with 3 bytes per entry */
#define D4400_REG_DEFAULT_DATA_MAX_SIZE_BYTES	(1024)
#define D4400_INT_RAM_REG_DEFAULT_OFFSET \
		(D4400_INT_RAM_SIZE_BYTES - \
		D4400_REG_DEFAULT_DATA_MAX_SIZE_BYTES -\
		D4400_RESERVED1_MAX_SIZE_BYTES -\
		D4400_SIMMARKER_MAX_SIZE_BYTES -\
		D4400_IVT_MAX_SIZE_BYTES)
#define D4400_INT_RAM_REG_DEFAULT_ADDRESS \
	(D4400_INT_RAM_START_ADDRESS + D4400_INT_RAM_REG_DEFAULT_OFFSET)

/* Stub function is located at beginning of internal RAM. */
#define D4400_STUB_FUNC_SIZE_MAX_BYTES \
		(D4400_INT_RAM_SIZE_BYTES - D4400_IVT_MAX_SIZE_BYTES -\
		D4400_REG_DEFAULT_DATA_MAX_SIZE_BYTES)
#define D4400_INT_RAM_STUB_FUNC_OFFSET	(0) /* Offset starting at int. RAM */
#define D4400_INT_RAM_STUB_FUNC_ADDRESS	D4400_INT_RAM_START_ADDRESS

/*
 * Stack address for stub function is currently located towards
 * the end of RAM just before the reg default and IVT table.  The stack
 * grows downwards.
 */
#define D4400_INT_RAM_STACK_ADDRESS	D4400_INT_RAM_REG_DEFAULT_ADDRESS

/* Internal ram (OCRAM) for stub function to run in. */
struct internal_ram {
	u32 data32[D4400_INT_RAM_SIZE_BYTES / 4];
};

/* Weim (NOR) flash ram where ivt, u-boot, and linux resides. */
#define D4400_WEIM_NOR_FLASH_ADDRESS		(0x30000000) /* Per spec */
#define D4400_WEIM_NOR_FLASH_IVT_ADDRESS	(0x30001000) /* Per spec */
#define D4400_WEIM_NOR_FLASH_IVT_OFFSET		(0x1000) /* Per spec */
#define D4400_WEIM_NOR_FLASH_SIZE_MAX		(128 * 1024 * 1024)
struct weim_flash_mem {
	u32 data32[D4400_WEIM_NOR_FLASH_SIZE_MAX / 4];
};

void simreset_stubfunc(unsigned int ivt_table_ptr);
void simreset_stubfunc_end(void);
void exe_reset(unsigned int stubfunc_addr, unsigned int ivt_addr,
	unsigned int stack_addr);

void sim_reset(void);
int load_reg_default(struct internal_ram *internal_ram_base);
int load_stub_func(struct internal_ram *internal_ram_base);
int load_from_weim_nor_flash(struct weim_flash_mem *weim_flash_base,
	struct internal_ram *internal_ram_base);
int load_internal_ram(void);
int query_simreset_marker(void);
void set_simreset_marker(void);
void clear_simreset_marker(void);
void check_ram_simreset_marker(void);
void clear_ram_simreset_marker(void);
#endif /* __LINUX_SIMRESET_H */

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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <mach/simreset.h>
#include <mach/src_priv.h>

const unsigned int d4400_reg_default[] = {
	0xdeadbeef,	/* Unused (dcd barker) */
	0x00000000,	/* Placeholder for size of register entries. */
/*
 *	Type		Address		Val
 */
	/* AIPS1 */
	/* 0x00000004, 0x01000000, 0xffffffff, */
};

/*
 * Load register default values.  The default value table has the same
 * structure format as the DCD entry.
 */
int load_reg_default(struct internal_ram *internal_ram_base)
{
	int ret = 0;
	unsigned long *reg_data = (unsigned long *)d4400_reg_default;
	unsigned long reg_data_size_bytes =
		(unsigned long)sizeof(d4400_reg_default);
	int i;

	if (reg_data_size_bytes > D4400_REG_DEFAULT_DATA_MAX_SIZE_BYTES) {
		pr_warn("Register default data size exeeds allocated\n");
		pr_warn("internal RAM size of %i bytes.\n",
			D4400_REG_DEFAULT_DATA_MAX_SIZE_BYTES);
		ret = 1;
	} else {
		/* Load register default data into internal RAM. */
		for (i = 0; i < (reg_data_size_bytes/4); ++i) {
			iowrite32(reg_data[i], &internal_ram_base->data32[i +
				(D4400_INT_RAM_REG_DEFAULT_OFFSET/4)]);
		}

		/* Size of table in bytes is 1 word offset into table */
		iowrite32(reg_data_size_bytes, &internal_ram_base->data32[1 +
			(D4400_INT_RAM_REG_DEFAULT_OFFSET/4)]);
	}
	return ret;
}

/* Load stub function into internal RAM. */
int load_stub_func(struct internal_ram *internal_ram_base)
{
	int ret = 0;
	unsigned long *stub_start = (unsigned long *)simreset_stubfunc;
	unsigned long stub_size_bytes =
		(unsigned long)simreset_stubfunc_end -
		(unsigned long)simreset_stubfunc;
	int i;

	if (stub_size_bytes > D4400_STUB_FUNC_SIZE_MAX_BYTES) {
		pr_warn("Stub function size exceeds available internal\n");
		pr_warn("RAM size of %i bytes.\n",
			D4400_STUB_FUNC_SIZE_MAX_BYTES);
		ret = 1; /* Failed */
	} else {
		/* Load stub function into internal RAM. */
		for (i = 0; i < (stub_size_bytes/4); ++i) {
			iowrite32(stub_start[i],
			&internal_ram_base->data32[i +
			(D4400_INT_RAM_STUB_FUNC_OFFSET/4)]);
		}
	}
	return ret;
}

/* Load IVT table from weim/NOR flash into internal RAM. */
int load_from_weim_nor_flash(struct weim_flash_mem *weim_flash_base,
	struct internal_ram *internal_ram_base)
{
	int ret = 0;
	unsigned long i;
	unsigned long val32;

	for (i = 0; i < (D4400_IVT_MAX_SIZE_BYTES/4); ++i) {
		val32 = ioread32(&weim_flash_base->data32[i +
			(D4400_WEIM_NOR_FLASH_IVT_OFFSET/4)]);
		iowrite32(val32, &internal_ram_base->data32[i +
			(D4400_INT_RAM_IVT_OFFSET/4)]);
	}
	return ret;
}

/* Load necessary data into internal RAM for simulated reset. */
int load_internal_ram(void)
{
	int ret = 0;
	struct device_node *np_int_ram;		/* Internal ram */
	struct device_node *np_sys_reset_regs;	/* System reset registers */
	struct device_node *np_weim_flash_mem;	/* Weim/Nor flash memory */
	struct device_node *np_qspi_flash_mem;	/* Qspi flash memory */

	np_int_ram = of_find_compatible_node(NULL, NULL,
		D4400_INT_RAM_DTS_ENTRY_NODE_STR);
	np_weim_flash_mem =
		of_find_compatible_node(NULL, NULL, "fsl,d4400-weim-flash");
	np_qspi_flash_mem =
		of_find_compatible_node(NULL, NULL, "fsl,d4400-qspi-flash");
	np_sys_reset_regs =
		of_find_compatible_node(NULL, NULL, "fsl,src-d4400");

	if ((!np_int_ram) || (!np_sys_reset_regs)) {
		if (!np_int_ram)
			pr_warn("Internal RAM not found in DTS tree.\n");
		if (!np_sys_reset_regs)
			pr_warn("System reset registers not found in DTS tree.\n");
		ret = 1; /* Failed */
	} else {
		/* Retrieve internal ram addr to load DCD & stub function. */
		struct internal_ram *internal_ram_base =
			of_iomap(np_int_ram, 0);
		/* System reset reg is used to determine type of flash. */
		struct src_regs *sys_reset_regs =
			of_iomap(np_sys_reset_regs, 0);
		/*
		 * Determine types of flash and load the IVT header into
		 * internal RAM.  Query the flash type: 0-weim/nor 1-qspi
		 */
		if (ioread32(&sys_reset_regs->sbmr) & SRC_SBMR_MEM_TYPE_MASK) {
			/* Qspi flash */
			if (!np_qspi_flash_mem) {
				pr_warn("Qspi not found in DTS tree.\n");
				ret = 1;
			} else {
				/* Qspi flash */
				/* TODO: Implement qspi flash read */
				pr_warn("TODO: Implement qspi flash for simulated reset!\n");
			}
		} else {
			/* Weim nor flash */
			struct weim_flash_mem *flash_base =
				of_iomap(np_weim_flash_mem, 0);
			ret = load_from_weim_nor_flash(flash_base,
				internal_ram_base);
		}

		/* Load stub function into internal RAM. */
		if (ret == 0)
			ret = load_stub_func(internal_ram_base);
		/* Load register default values into internal RAM. */
		if (ret == 0)
			ret = load_reg_default(internal_ram_base);
	}
	return ret;
}

/* Perform simulated reset. */
void sim_reset(void)
{
	/*
	 * If stub function load to internal RAM is NOT successful, then
	 * return to do normal soft reboot.
	 */
	if (load_internal_ram() == 0) {
		/* Execution is transferred to stub function.
		 * This function never returns.
		 */
		exe_reset(D4400_INT_RAM_STUB_FUNC_ADDRESS,
			D4400_INT_RAM_IVT_ADDRESS,
			D4400_INT_RAM_STACK_ADDRESS);
	}
}

/* Simreset marker: 0-reboot from hard reset, 1-simulated reset
 *  Note that this marker is different than the marker in RAM.
 *  This marker is to remember what we read from the marker in RAM
 *  because the marker in RAM is cleared after it is read (in case
 *  a simulate reset happens). During Linux intialization this marker
 *  is used to determine how we initalize drivers.  After initialization
 *  is done this marker is cleared.
 */
static int simreset_marker;
int query_simreset_marker(void)
{
	return simreset_marker;
}
void set_simreset_marker(void)
{
	simreset_marker = 1;
}
void clear_simreset_marker(void)
{
	simreset_marker = 0;
}

/*
 * Check the simreset marker in RAM.  The marker is reset to zero.  This
 * function should be called once on startup.
 */
void check_ram_simreset_marker(void)
{
	struct device_node *np_int_ram; /* Internal ram */
	np_int_ram = of_find_compatible_node(NULL, NULL,
		D4400_INT_RAM_DTS_ENTRY_NODE_STR);

	if (!np_int_ram) {
		pr_warn("Internal RAM not found in DTS tree.\n");
		pr_warn("Simulated reset marker check skipped.\n");
	} else {
		/* Retrieve internal ram addr to retrieve simreset marker. */
		struct internal_ram *int_ram = of_iomap(np_int_ram, 0);
		unsigned int marker = ioread32(
			&int_ram->data32[D4400_INT_RAM_SIMMARKER_OFFSET/4]);
		/* Check marker in RAM. */
		if (marker == D4400_SIMMARKER_VALUE_WORD) {
			/* Set the marker to indicate this is a simulated
			 * reboot.
			 */
			set_simreset_marker();
			pr_info("System is booting from simulated reset.\n");
		} else {
			/* Clear the marker to indicates this is not a
			 * simulated reboot.
			 */
			clear_simreset_marker();
			pr_info("System is booting from hard reset.\n");
		}

		/* Clear the marker in RAM. */
		iowrite32(0,
			&int_ram->data32[D4400_INT_RAM_SIMMARKER_OFFSET/4]);
	}
}

/* Clear the simreset marker in RAM.  This should be done if system
 * is rebooting with cpu reset so that we can tell upon booting up
 * that the reboot is NOT a simulated reset.
 */
void clear_ram_simreset_marker(void)
{
	struct device_node *np_int_ram; /* Internal ram */
	np_int_ram = of_find_compatible_node(NULL, NULL,
		D4400_INT_RAM_DTS_ENTRY_NODE_STR);

	if (!np_int_ram) {
		pr_warn("Internal RAM not found in DTS tree.\n");
		pr_warn("Simulated reset marker NOT set.\n");
	} else {
		/* Set internal ram addr to retrieve simreset marker. */
		struct internal_ram *int_ram = of_iomap(np_int_ram, 0);

		/* Clear the marker in RAM. */
		iowrite32(0,
			&int_ram->data32[D4400_INT_RAM_SIMMARKER_OFFSET/4]);
	}
}

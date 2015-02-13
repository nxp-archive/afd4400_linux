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
#include <mach/ivt_offsets.h>

/* Stub function runs in internal RAM. */
void simreset_stubfunc(unsigned int int_ram_ivt_addr)
{
	unsigned long *src_addr =
		(unsigned long *)D4400_WEIM_NOR_FLASH_ADDRESS;
	struct _ivt_table *ivt = (struct _ivt_table *)int_ram_ivt_addr;
	struct _boot_data *boot;
	struct _dcd_data *reg_default;
	int i, max_words;

	boot = (struct _boot_data *)(int_ram_ivt_addr +
		(ivt->boot_data_ptr - ivt->ivt_ptr));

	/* Program default register values. */
	reg_default = (struct _dcd_data *)D4400_INT_RAM_REG_DEFAULT_ADDRESS;
	max_words = reg_default->preamble.size_bytes /
			sizeof(struct _dcd_entry);

	for (i = 0; i < max_words; ++i)	{
		if (reg_default->entry[i].type == 4) {
			*((unsigned long *)reg_default->entry[i].addr) =
				reg_default->entry[i].data32;
		}
	}

#if 0 /* Programming DCD is not currently needed. */
	/*
	 * Calculate addresses of internal RAM location for boot and
	 * dcd data. The ivt table contains destination addresses in DDR.
	 * The calculation below gets the relative offset into the table
	 * that points to the boot/dcd information adds in the internal
	 * RAM address.
	 */
	struct _dcd_data *dcd;
	dcd = (struct _dcd_data *)(int_ram_ivt_addr +
		(ivt->dcd_ptr - ivt->ivt_ptr));

	/* Program the DCD registers. */
	max_words = dcd->preamble.size_bytes / sizeof(struct _dcd_entry);
	for (i = 0; i < max_words; ++i) {
		if (
			(dcd->entry[i].addr >= D4400_CCM_MODULE_ADDRESS_START)
			&&
			(dcd->entry[i].addr <= D4400_CCM_MODULE_ADDRESS_END)
		   ) {
			/* NOTE: Reprogramming clock registers in the DCD
			 * can cause the system to hang as some clock changes
			 * require the PLL to be off and a sequence to be
			 * followed in order for clock changes to be
			 * sycnchronized.  All clock register changes are
			 * skipped for simulated reset.
			 */
			continue;
		}
		if (dcd->entry[i].type == 4) {
			*((unsigned long *)dcd->entry[i].addr) =
				dcd->entry[i].data32;
		} else if (dcd->entry[i].type == 2) {
			*((unsigned short *)dcd->entry[i].addr) =
				(unsigned short)dcd->entry[i].data32;
		} else if (dcd->entry[i].type == 1) {
			*((unsigned char *)dcd->entry[i].addr) =
				(unsigned char)dcd->entry[i].data32;
		}
	}
#endif

	/* Load u-boot from flash to DDR. */
	{
		unsigned long *sbmr = (unsigned long *)SRC_SBMR_REG_ADRESS;

		max_words = boot->size_bytes / 4;
		if ((max_words * 4) < boot->size_bytes)
			++max_words;

		if (*sbmr & SRC_SBMR_MEM_TYPE_MASK) {
			/*  Copy from qspi flash to DDR */
			/* TODO: Implement Qspi read. */
			while (1) { asm volatile ("nop"); }
		} else {
			/* Copy from weim/nor flash to DDR */
			unsigned int *dest = (unsigned int *)boot->dest_ptr;
			do {
				*dest++ = *src_addr++;
			} while (--max_words);
		}
	}

	/*
	 * Insert simreset marker.  The reboot initialization will read
	 * the marker to determine if the reboot was a power cycle or
	 * simulated reset.
	 */
	*((unsigned long *)D4400_INT_RAM_SIMMARKER_ADDRESS) =
		D4400_SIMMARKER_VALUE_WORD;

	/* Transfer control to bootloader. */
	asm volatile ("mov pc, %[value]\n\t" : : [value]"r" (ivt->app_entry_ptr));
}
/*
 * NOTE: DO NOT move the stub end function.  It must be placed
 * after the simreset_stubfunc() so that the stub function
 * size can be calculated.
 */
void simreset_stubfunc_end(void)
{
	asm volatile ("nop");
}

/* Execute simulated reset */
void exe_reset(unsigned int stubfunc_addr, unsigned int ivt_addr,
	unsigned int stack_addr)
{
	/* Delay is for any pending operation(s) to complete. */
	mdelay(200);

	/* Turn off IRQ/FIQ. */
	asm volatile ("mrs  r4, cpsr");
	asm volatile ("and  r5, r4, #0x1f");	/* mask mode bits */
	asm volatile ("teq  r5, #0x1a");	/* test for HYP mode */
	asm volatile ("bicne r4, r4, #0x1f");	/* clear all mode bits */
	asm volatile ("orrne r4, r4, #0x13");	/* set SVC mode */
	asm volatile ("orr  r4, r4, #0xc0");	/* disable FIQ & IRQ */
	asm volatile ("msr  cpsr,r4");

	/* Invalidate L1 I/D */
	asm volatile ("mov  r4, #0");		/* set up for MCR */
	asm volatile ("mcr  p15, 0, r4, c8, c7, 0"); /* invalidate TLBs */
	asm volatile ("mcr  p15, 0, r4, c7, c5, 0"); /* invalidate icache */
	asm volatile ("mcr  p15, 0, r4, c7, c5, 6"); /* invalidate BP array */
	asm volatile ("mcr  p15, 0, r4, c7, c10, 4 @ DSB");
	asm volatile ("mcr  p15, 0, r4, c7, c5, 4 @ ISB");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");

	/* Invalidate and disable L2 cache */
	{
		struct device_node *np_L2cc; /* L2 cache control registers */
		unsigned long *pL2cc_base_reg;
		np_L2cc =
			of_find_compatible_node(NULL, NULL, "arm,pl310-cache");
		pL2cc_base_reg = of_iomap(np_L2cc, 0);

		/* Prefetch control */
		iowrite32(0x00000000,
			&pL2cc_base_reg[L2CC_REG15_PREFETCH_CONTROL_OFFSET/4]);
		/* L2 irq masked */
		iowrite32(0x00000000, &pL2cc_base_reg[L2CC_INT_MASK_OFFSET/4]);
		/* Clear irq status */
		iowrite32(0x000001ff, &pL2cc_base_reg[L2CC_INT_CLEAR_OFFSET/4]);

		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");

		/* Invalidate Way L2 cache by setting each of 16 Way bits. */
		iowrite32(0xffff,
			&pL2cc_base_reg[L2CC_REG7_CLEAN_INV_WAY_OFFSET/4]);
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");

		/* Operation is done when Way bits are cleared. */
		while (ioread32(
			&pL2cc_base_reg[L2CC_REG7_CLEAN_INV_WAY_OFFSET/4]) &
			0xffff)
		{ asm volatile ("nop");	}
		/* L2 cache off */
		iowrite32(0x00000000,
			&pL2cc_base_reg[L2CC_REG1_CONTROL_OFFSET/4]);

		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
	}

	/* Disable MMU stuff and caches */
	asm volatile ("mrc  p15, 0, r4, c1, c0, 0");
	asm volatile ("bic  r4, r4, #0x00002000"); /* clear b13 (--V-) */
	asm volatile ("bic  r4, r4, #0x00000007"); /* clear b2:0 (-CAM) */
	asm volatile ("orr  r4, r4, #0x00000002"); /* set b1 (--A-) Align */
	asm volatile ("bic  r4, r4, #0x00000800"); /* clear b11 (Z---) BTB */
	/* I-cache off */
	asm volatile ("bic  r4, r4, #0x00001000"); /* clear b12 (I) I-cache */
	/* D-cache off */
	asm volatile ("bic  r4, r4, #0x00000004"); /* clear b2 (D) D-cache */
	asm volatile ("mcr  p15, 0, r4, c1, c0, 0");

#ifdef CONFIG_ARM_ERRATA_716044
	asm volatile ("mrc  p15, 0, r4, c1, c0, 0"); /* rd sys control reg */
	asm volatile ("orr  r4, r4, #1 << 11"); /* set b11 */
	asm volatile ("mcr  p15, 0, r4, c1, c0, 0"); /* wr sys control reg */
#endif

#ifdef CONFIG_ARM_ERRATA_742230
	asm volatile ("mrc  p15, 0, r4, c15, c0, 1"); /* rd diagnostic reg */
	asm volatile ("orr  r4, r4, #1 << 4"); /* set b4 */
	asm volatile ("mcr  p15, 0, r4, c15, c0, 1"); /* wr diagnostic reg */
#endif

#ifdef CONFIG_ARM_ERRATA_743622
	asm volatile ("mrc  p15, 0, r4, c15, c0, 1"); /* rd diagnostic reg */
	asm volatile ("orr  r4, r4, #1 << 6"); /* set b6 */
	asm volatile ("mcr  p15, 0, r4, c15, c0, 1"); /* wr diagnostic reg */
#endif

#ifdef CONFIG_ARM_ERRATA_751472
	asm volatile ("mrc  p15, 0, r4, c15, c0, 1"); /* rd diagnostic reg */
	asm volatile ("orr  r4, r4, #1 << 11"); /* set b11 */
	asm volatile ("mcr  p15, 0, r4, c15, c0, 1"); /* wr diagnostic reg */
#endif
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");

	/* Save stub func address (passes into this function in r0) for
	 * a moment as we need r0 to store the ivt table address for the
	 * stub function.
	*/
	asm volatile ("mov  r4, %[value]\n\t" : : [value]"r" (stubfunc_addr));

	/* Put the ivt table address in r0 as parameter for stub function */
	asm volatile ("mov  r0, %[value]\n\t" : : [value]"r" (ivt_addr));

	/* Update stack pointer which should be pointing to internal RAM */
	asm volatile ("mov  sp, %[value]\n\t" : : [value]"r" (stack_addr));

	/* Transfer execution to stub function */
	asm volatile ("mov  pc, r4\n\t");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("b  ."); /* @ Inifinite loop */
}

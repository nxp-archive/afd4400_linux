#include <linux/module.h>
#include <linux/io.h>

#include "hardware.h"

unsigned int __d4400_cpu_type;
EXPORT_SYMBOL(__d4400_cpu_type);

void d4400_set_cpu_type(unsigned int type)
{
	__d4400_cpu_type = type;
}

void d4400_print_silicon_rev(const char *cpu, int srev)
{
	if (srev == D4400_CHIP_REVISION_UNKNOWN)
		pr_info("CPU identified as %s, unknown revision\n", cpu);
	else
		pr_info("CPU identified as %s, silicon rev %d\n",
				cpu, srev & 0xff);
}



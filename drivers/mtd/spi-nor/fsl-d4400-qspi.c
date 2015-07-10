/*
 * Copyright 2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <mach/src.h>
#include <mach/qspi-d4400.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define QSPI_MOD_NAME "qspi"
#define DRIVER_NAME "D4400 qspi"
#define FSL_QSPI_MAX_CHIP 1

/* qspi->debug */
#define DEBUG_QSPI_MESSAGES		(1<<0)
#define DEBUG_QSPI_GENERAL		(1<<1)

/* Debug and error reporting macros */
#define IF_DEBUG(x) if (qspi->debug & (x))
#define ERR(...) {if (qspi->debug & DEBUG_QSPI_MESSAGES) \
				pr_err(QSPI_MOD_NAME __VA_ARGS__); }

static int d4400_flash_vendor_val[] = {
	(1 << 3),	/* 0 - Winbond */
	(2 << 3),	/* 1 - Spansion */
	(3 << 3),	/* 2 - Macronix */
	(4 << 3),	/* 3 - Numonyx */
};

enum fsl_qspi_devtype {
	FSL_QUADSPI_D4400,
};

struct fsl_qspi_devtype_data {
	enum fsl_qspi_devtype devtype;
	int rxfifo;
	int txfifo;
	int ahb_buf_size;
	u32 clk_hz;		/* Requested clock speed */
	u32 clk_actual_hz;	/* Actual clock speed */
	u32 divisor;		/* Clock divisor */
};

static struct fsl_qspi_devtype_data d4400_data = {
	.devtype = FSL_QUADSPI_D4400,
	.rxfifo = 128,
	.txfifo = 64,
	.ahb_buf_size = 128,
};

struct fsl_qspi {
	struct mtd_info mtd[FSL_QSPI_MAX_CHIP];
	struct spi_nor nor[FSL_QSPI_MAX_CHIP];
	void __iomem *iobase;
	void __iomem *amba_base; /* Used when read from AHB bus */
	u32 memmap_phy;
	struct clk *clksel, *clkgate;
	struct device *dev;
	struct completion c;
	struct fsl_qspi_devtype_data *devtype_data;
	u32 nor_size;
	u32 nor_num;

	/* Sysfs support */
	dev_t			devt;
	struct class		*class;
	struct cdev		dev_cdev;
	int			major;
	int			minor;
	atomic_t		ref;
	u32			debug;
};
static struct fsl_qspi *qspi;

struct mtd_info *fsl_qspi_get_mtd(int index)
{
	if (index < FSL_QSPI_MAX_CHIP)
		return &qspi->mtd[index];
	else
		return NULL;
}
EXPORT_SYMBOL(fsl_qspi_get_mtd);

static inline int is_d4400_qspi(struct fsl_qspi *q)
{
	return q->devtype_data->devtype == FSL_QUADSPI_D4400;
}

static inline u32 fsl_qspi_endian_xchg(struct fsl_qspi *q, u32 a)
{
	return is_d4400_qspi(q) ? __swab32(a) : a;
}

static const struct of_device_id fsl_qspi_dt_ids[] = {
	{ .compatible = "fsl,d4400-qspi", .data = &d4400_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_qspi_dt_ids);

/* Called when apps perform an open on /sys/class/qspi */
static int sysfs_qspi_open(struct inode *inode, struct file *pfile)
{
	pfile->private_data = qspi;
	atomic_inc(&qspi->ref);
	return 0;
}

static ssize_t sysfs_qspi_read(struct file *filep, char __user *buf,
	size_t size, loff_t *offset)
{
	int ret = 0;
	ret = put_user(0, (int *)buf);
	return ret;
}

int sysfs_qspi_release(struct inode *inode, struct file *pfile)
{
	atomic_dec(&qspi->ref);
	return 0;
}

/* File ops for sysfs */
static const struct file_operations sysfs_qspi_fops = {
	.owner = THIS_MODULE,
	.open = sysfs_qspi_open,
	.read = sysfs_qspi_read,
	.release = sysfs_qspi_release,
};

static ssize_t set_debug(struct device *dev,
	struct device_attribute *devattr,
	const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	qspi->debug = val;
	return count;
}

static ssize_t show_debug(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d / 0x%x\n", qspi->debug, qspi->debug);
}

static ssize_t show_qspi_info(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	char *p = buf;
	void __iomem *base = qspi->iobase;
	u32 mcr;

	if (!qspi)
		return sprintf(buf, "No qspi info\n");

	mcr = readl(base + QSPI_MCR_REG);

	sprintf(p, "\tFlash dev cnt      : %i\n", qspi->nor_num);
	p += strlen(p);

	sprintf(p, "\tMCR reg            : x%08x\n", mcr);
	p += strlen(p);

	if (mcr & QSPI_MCR_EXT_ADD_MASK)
		sprintf(p, "\tExt addr mode      : 32-bit address\n");
	else
		sprintf(p, "\tExt addr mode      : 24-bit address\n");
	p += strlen(p);

	sprintf(p, "\tVendor             : x%02x\n",
		((mcr & QSPI_MCR_VMID_MASK) >> QSPI_MCR_VMID_OFFSET));
	p += strlen(p);

	sprintf(p, "\tClk speed/div      : %i Khz / x%02x\n",
		qspi->devtype_data->clk_actual_hz / 1000,

		((mcr & QSPI_MCR_SCLKCFG_MASK) >> QSPI_MCR_SCLKCFG_OFFSET));
	p += strlen(p);

	return strlen(buf);
}

static ssize_t show_flash_info(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	char *p = buf;
	struct spi_nor *nor;
	struct mtd_info *mtd;

	if (!qspi)
		return sprintf(buf, "No qspi info\n");

	nor = &qspi->nor[0];
	mtd = &qspi->mtd[0];

	sprintf(p, "\tFlash name         : %s\n", nor->flash_name);
	p += strlen(p);
	sprintf(p, "\tSize               : %-3i MB\n", (u32)mtd->size / 1024 / 1024);
	p += strlen(p);
	sprintf(p, "\tErase size         : %-3i KB\n", mtd->erasesize / 1024);
	p += strlen(p);
	sprintf(p, "\tPage/write size    : %-3i bytes\n", nor->page_size);
	p += strlen(p);
	sprintf(p, "\tErase opcode       : x%02x\n", nor->erase_opcode);
	p += strlen(p);
	sprintf(p, "\tRead opcode        : x%02x\n", nor->read_opcode);
	p += strlen(p);
	sprintf(p, "\tProgram opcode     : x%02x\n", nor->program_opcode);
	p += strlen(p);
	sprintf(p, "\tMTD flags          : x%08x\n", mtd->flags);
	p += strlen(p);

	sprintf(p, "\tI/O mode           : ");
	p += strlen(p);
	if (nor->flash_read == SPI_NOR_QUAD)
		sprintf(p, "Quad\n");
	else if (nor->flash_read == SPI_NOR_DUAL)
		sprintf(p, "Dual\n");
	else if (nor->flash_read == SPI_NOR_NORMAL)
		sprintf(p, "Single\n");
	else if (nor->flash_read == SPI_NOR_FAST)
		sprintf(p, "Fast\n");
	else
		sprintf(p, "Undefined\n");
	p += strlen(p);

	return strlen(buf);
}

static DEVICE_ATTR(debug,	S_IWUSR | S_IRUGO, show_debug, set_debug);
static DEVICE_ATTR(qspi_info,	S_IRUGO, show_qspi_info, NULL);
static DEVICE_ATTR(flash_info,	S_IRUGO, show_flash_info, NULL);

static struct attribute *attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_qspi_info.attr,
	&dev_attr_flash_info.attr,
	NULL
};
static const struct attribute_group attr_group = {
	.attrs = attributes,
};

static irqreturn_t fsl_qspi_irq_handler(int irq, void *dev_id)
{
	struct fsl_qspi *q = dev_id;
	u32 reg;

	/* Clear interrupt */
	reg = readl(q->iobase + QSPI_FR_REG);
	writel(reg, q->iobase + QSPI_FR_REG);

	if (reg & QSPI_FR_TFF_MASK)
		complete(&q->c);

	/* Clear and invalidate rx/tx buffer */
	reg = readl(q->iobase + QSPI_MCR_REG);
	writel(reg | QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK,
		q->iobase + QSPI_MCR_REG);

	dev_dbg(q->dev, "QUADSPI_FR : 0x%.8x\n", reg);
	return IRQ_HANDLED;
}

static u32 fsl_qspi_get_clk_freq(u32 clk_src_hz, u32 div)
{
	div += 2;
	if (!div)
		/* Just in case, avoid div by zero */
		return 0;
	return (clk_src_hz << 1) / div;
}

static u32 fsl_qspi_cal_clkdiv(u32 clk_src_hz, u32 max_hz)
{
	u32 div = 0;
	u32 div_shf;

	if (max_hz < clk_src_hz) {
		/* Qspi can divide in 1/2 increments
			0000	0.0	0
			0000	0.5	0
			0000	1.0	0
			0001	1.5	1
			0010	2.0	2
			0011	2.5	3
			0100	3.0	4
			0101	3.5	5
			0110	4.0	6
			0111	4.5	7
			1000	5.0	8
			1001	5.5	9
			1010	6.0	10
			1011	6.5	11
			1100	7.0	12
			1101	7.5	13
			1110	8.0	14
			1111	8.5	15
		*/
		div_shf = (clk_src_hz  << 1) / max_hz;
		div = div_shf - 2;

		if (div > 0x0F) /* 0x0F is max, div by 8.5 */
		  div = 0x0F;
	}
	return div;
}

static int fsl_qspi_set_clkdiv(struct spi_nor *nor, u32 div)
{
	struct fsl_qspi *q = nor->priv;
	void __iomem *base = q->iobase;
	u32 reg;

	if (div > 0x0f) {
		dev_err(q->dev, "Invalid clock divisor %i\n", div);
		return -EINVAL;
	}
	reg = readl(base + QSPI_MCR_REG);

	/* Before new settings, set MDIS = 1 to allow change. */
	writel(reg | QSPI_MCR_MDIS_MASK, base + QSPI_MCR_REG);

	/* Clear to enable clk. */
	reg &= ~QSPI_MCR_MDIS_MASK;

	/* Set the clock divisor */
	reg &= ~QSPI_MCR_SCLKCFG_MASK;
	reg |= (div << QSPI_MCR_SCLKCFG_OFFSET);

	/* Make changes effective */
	writel(reg, base + QSPI_MCR_REG);

	return 0;
}

static void fsl_qspi_set_addr_mode(struct spi_nor *nor, u32 flash_size)
{
	struct fsl_qspi *q = nor->priv;
	void __iomem *base = q->iobase;
	u32 reg;

	reg = readl(base + QSPI_MCR_REG);

	/* Before new settings, set MDIS = 1 to allow change. */
	writel(reg | QSPI_MCR_MDIS_MASK, base + QSPI_MCR_REG);

	/* Clear to enable clk. */
	reg &= ~QSPI_MCR_MDIS_MASK;

	if (flash_size > 0x1000000)
		/* 32-bit mode */
		reg |= QSPI_MCR_EXT_ADD_MASK;
	else
		/* 24-bit mode */
		reg &= ~QSPI_MCR_EXT_ADD_MASK;

	/* Make changes effective */
	writel(reg, base + QSPI_MCR_REG);
}

static int fsl_qspi_set_amba_read_opcode(struct spi_nor *nor)
{
	struct fsl_qspi *q = nor->priv;
	void __iomem *base = q->iobase;
	u32 reg;

	/* Find out the D4400 addressing mode */
	reg = readl(base + QSPI_MCR_REG);
	reg &= QSPI_MCR_EXT_ADD_MASK;

	/* Depending on the address mode of the D4400, allow
	 * appropriate commands.
	 */
	switch (nor->read_opcode) {

	/* Valid read commands for 3-byte addressing */
	case SPINOR_OP_READ:
	case SPINOR_OP_READ_FAST:
	case SPINOR_OP_READ_1_1_2:
	case SPINOR_OP_READ_1_1_4:
		if (!reg) /* 3-byte addr mode */
			break;
		else {
			dev_err(q->dev, "Unsupported 3-byte addr read cmd 0x%.2x\n",
				nor->read_opcode);
			return -EINVAL;
		}

	/* Valid read commands for 4-byte addressing */
	case SPINOR_OP_READ4:
	case SPINOR_OP_READ4_FAST:
	case SPINOR_OP_READ4_1_1_2:
	case SPINOR_OP_READ4_1_1_4:
		if (reg) /* 4-byte addr mode */
			break;
		else {
			dev_err(q->dev, "Unsupported 4-byte addr read cmd 0x%.2x\n",
				nor->read_opcode);
			return -EINVAL;
		}
	}

	/* AMBA control reg (for AMBA bus read commands)
	 * Read 8 * 8 or 64 bytes into AHB buffer, see RM.
	 */
	writel(0x800 | nor->read_opcode, base + QSPI_ACR_REG);

	return 0;
}

/* We use this function to do some basic init for spi_nor_scan(). */
static int fsl_qspi_setup(struct fsl_qspi *q)
{
	void __iomem *base = q->iobase;
	u32 reg;

	/* Latency, default */
	writel(0x08, base + QSPI_LCR_REG);

	/* Rx buffer RBDRx read using IP bus reg.  Note that this
	 * setting refers to accessing from IP bus reg or AMBA bus
	 * reg mem mapped to 0x6000000.  It does NOT affect the host
	 * AMBA bus read of the qspi flash memory mapped region
	 * starting at 0x8000000. */
	writel(QSPI_RBCT_RXBRD_MASK, base + QSPI_RBCT_REG);

	/* AMBA control reg (for AMBA bus read command) */
	writel(0x803, base + QSPI_ACR_REG);  /* Default = 0x803 */

	/* Control settings */
	reg = readl(base + QSPI_MCR_REG);

	/* Before new settings, set MDIS = 1 to allow change. */
	reg |= QSPI_MCR_MDIS_MASK;
	writel(reg, base + QSPI_MCR_REG);

	/* Set clk divisor to max for init.  It can be changed later
	 * after max speed of flash device has been determined.
	 */
	reg &= ~QSPI_MCR_SCLKCFG_MASK;
	reg |= (0x0F << QSPI_MCR_SCLKCFG_OFFSET); /* 0x0F = ~18MHz */

	/* Clear rx/tx buffers. */
	reg |= (QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK); /* Clr bufs */

	/* Configure flash type */
	reg &= ~QSPI_MCR_VMID_MASK;
	reg |= d4400_flash_vendor_val[src_get_serial_flash_type()];

	/* Drive IO[3:2] high (default) during idle */
	reg |= (QSPI_MCR_ISD2FA_MASK | QSPI_MCR_ISD3FA_MASK);

	/* Set 24-bit addressing, default.  Driver func will change this
	 * if necessary.
	 */
	reg &= ~QSPI_MCR_EXT_ADD_MASK;

	/* Make changes effective */
	writel(reg, base + QSPI_MCR_REG);

	/* Enable module clock. Note that when MDIS=0, some MCR reg
	 * bits cannot be modified like clk div and 24/32bit addr mode.
	 */
	reg &= ~QSPI_MCR_MDIS_MASK;
	writel(reg, base + QSPI_MCR_REG);

	/* Enable transaction finished interrupt */
	reg = readl(base + QSPI_RSER_REG);
	reg |= QSPI_RSER_TFIE_MASK;
	writel(reg, base + QSPI_RSER_REG);

	/* Clear pending interrupts */
	reg = readl(q->iobase + QSPI_FR_REG);
	writel(reg, q->iobase + QSPI_FR_REG);

	return 0;
}

static int fsl_qspi_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	/* Nothing to do for now */
	return 0;
}

static void fsl_qspi_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	/* Nothing to do for now */
}

static inline void fsl_qspi_invalid(struct fsl_qspi *q)
{
	/* Invalidate AHB buffer */
	/* Not applicable for D4400 qspi */
}

/* Translate the command if needed */
static int fsl_qspi_cmd_translate(struct fsl_qspi *q, u8 cmd)
{
	switch (cmd) {
	case SPINOR_OP_READ_1_1_4:
	case SPINOR_OP_WREN:
	case SPINOR_OP_WRDI:
	case SPINOR_OP_RDSR:
	case SPINOR_OP_SE:
	case SPINOR_OP_CHIP_ERASE:
	case SPINOR_OP_PP:
	case SPINOR_OP_RDID:
	case SPINOR_OP_WRSR:
	case SPINOR_OP_RDCR:
	case SPINOR_OP_EN4B:
	case SPINOR_OP_BRWR:
	case SPINOR_OP_PP_4B:
	case SPINOR_OP_PP_4B_1_1_4:
	case SPINOR_OP_SE_4B:
		/* No translation needed */
		return cmd;
	default:
		dev_err(q->dev, "Unsupported cmd 0x%.2x\n", cmd);
		break;
	}
	return -EINVAL;
}

static int fsl_qspi_runcmd(struct fsl_qspi *q, u8 cmd, unsigned int addr,
	int len)
{
	void __iomem *base = q->iobase;
	int xcmd;
	u32 mcr, icr, reg2;
	int err;

	init_completion(&q->c);
	dev_dbg(q->dev, "to 0x%.8x, len:%d, cmd:%.2x\n", addr, len, cmd);

	/* Save the reg */
	mcr = readl(base + QSPI_MCR_REG);

	/* Set flash address */
	writel(addr, base + QSPI_SFAR_REG);

	/* Clear rx buffer */
	writel(mcr | QSPI_MCR_CLR_RXF_MASK, base + QSPI_MCR_REG);

	/* Set rx watermark, use IP reg commands (not AHB) */
	writel(QSPI_RBCT_RXBRD_USEIPS,	base + QSPI_RBCT_REG);

	/* Wait for not busy status */
	do {
		reg2 = readl(base + QSPI_SR_REG);
		if (reg2 & (QSPI_SR_IP_ACC_MASK | QSPI_SR_AHB_ACC_MASK)) {
			udelay(1);
			dev_dbg(q->dev, "The controller is busy, 0x%x\n", reg2);
			continue;
		}
		break;
	} while (1);

	/* Translate the command */
	xcmd = fsl_qspi_cmd_translate(q, cmd);
	icr = (len << QSPI_ICR_ICO_OFFSET) | (xcmd << QSPI_ICR_IC_OFFSET);

	/* Send command to flash */
	writel(icr, base + QSPI_ICR_REG);

	/* Wait for the interrupt. */
	err = wait_for_completion_timeout(&q->c, msecs_to_jiffies(1000));
	if (!err) {
		dev_err(q->dev,
			"cmd 0x%.2x timeout, addr@%.8x, FR:0x%.8x, SR:0x%.8x\n",
			cmd, addr, readl(base + QSPI_FR_REG),
			readl(base + QSPI_SR_REG));
		err = -ETIMEDOUT;
	} else
		err = 0;

	/* restore the MCR */
	writel(mcr, base + QSPI_MCR_REG);

	return err;
}

/* Read out the data from the QUADSPI_RBDR buffer registers. */
static void fsl_qspi_read_data(struct fsl_qspi *q, int len, u8 *rxbuf)
{
	u32 tmp;
	int i = 0;

	while (len > 0) {
		tmp = readl(q->iobase + QSPI_RXDATA_BASE_REG + (i * 4));
		tmp = fsl_qspi_endian_xchg(q, tmp);
		dev_dbg(q->dev, "read data:0x%.8x\n", tmp);

		if (len >= 4) {
			*((u32 *)rxbuf) = tmp;
			rxbuf += 4;
		} else {
			memcpy(rxbuf, &tmp, len);
			break;
		}

		len -= 4;
		i++;
	}
}

static int fsl_qspi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret;
	struct fsl_qspi *q = nor->priv;

	ret = fsl_qspi_runcmd(q, opcode, 0, len);
	if (ret)
		return ret;

	fsl_qspi_read_data(q, len, buf);
	return 0;
}

static int fsl_qspi_read(struct spi_nor *nor, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
	struct fsl_qspi *q = nor->priv;

	dev_dbg(q->dev, "read from (0x%.8x),len:%d\n",
		(u32)q->amba_base + (u32)from, len);

	/* Read out the data directly from the AHB buffer.*/
	memcpy(buf, q->amba_base + from, len);

	*retlen += len;
	return 0;
}

static int fsl_qspi_nor_write(struct fsl_qspi *q, struct spi_nor *nor,
				u8 opcode, unsigned int to, u32 *txbuf,
				unsigned count, size_t *retlen)
{
	int ret, i, j;
	u32 tmp;

	dev_dbg(q->dev, "to 0x%.8x, len : %d\n", to, count);

	/* Clear the TX FIFO. */
	tmp = readl(q->iobase + QSPI_MCR_REG);
	writel(tmp | QSPI_MCR_CLR_TXF_MASK, q->iobase + QSPI_MCR_REG);

	/* Fill the TX data to the FIFO */
	for (j = 0, i = ((count + 3) / 4); j < i; j++) {
		tmp = fsl_qspi_endian_xchg(q, *txbuf);
		writel(tmp, q->iobase + QSPI_TBDR_REG);
		txbuf++;
	}

	/* Trigger it */
	ret = fsl_qspi_runcmd(q, opcode, to, count);

	if (ret == 0 && retlen)
		*retlen += count;

	return ret;
}

static int fsl_qspi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len,
			int write_enable)
{
	struct fsl_qspi *q = nor->priv;
	int ret;

	if (!buf) {
		ret = fsl_qspi_runcmd(q, opcode, 0, 1);
		if (ret)
			return ret;

		if (opcode == SPINOR_OP_CHIP_ERASE)
			fsl_qspi_invalid(q);

	} else if (len > 0) {
		ret = fsl_qspi_nor_write(q, nor, opcode, 0,
					(u32 *)buf, len, NULL);
	} else {
		dev_err(q->dev, "invalid cmd %d\n", opcode);
		ret = -EINVAL;
	}

	return ret;
}

static void fsl_qspi_write(struct spi_nor *nor, loff_t to,
		size_t len, size_t *retlen, const u_char *buf)
{
	struct fsl_qspi *q = nor->priv;

	fsl_qspi_nor_write(q, nor, nor->program_opcode, to,
				(u32 *)buf, len, retlen);

	/* invalid the data in the AHB buffer. */
	fsl_qspi_invalid(q);
}

static int fsl_qspi_erase(struct spi_nor *nor, loff_t offs)
{
	struct fsl_qspi *q = nor->priv;
	int ret;

	dev_dbg(nor->dev, "%dKiB at 0x%08x\n",
		nor->mtd->erasesize / 1024,  (u32)offs);

	ret = fsl_qspi_runcmd(q, nor->erase_opcode, offs, 0);
	if (ret)
		return ret;

	fsl_qspi_invalid(q);
	return 0;
}

static int fsl_qspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct mtd_part_parser_data ppdata;
	struct device *dev = &pdev->dev;
	struct fsl_qspi *q;
	struct resource *res;
	struct spi_nor *nor;
	struct mtd_info *mtd;
	int i = 0;
	const struct of_device_id *of_id =
		of_match_device(fsl_qspi_dt_ids, &pdev->dev);
	u32 amba_addr[2] = {0, 0};
	struct device *sysfs_dev;

	if (!np) {
		dev_err(&pdev->dev, "can't get the device node\n");
		return -EINVAL;
	}

	q = devm_kzalloc(dev, sizeof(*q), GFP_KERNEL);
	if (!q)
		return -ENOMEM;
	qspi = q;

	/* Number of flash devices */
	q->nor_num = of_get_child_count(dev->of_node);
	if (!q->nor_num || q->nor_num > FSL_QSPI_MAX_CHIP)
		return -ENODEV;

	/* Qspi register resources */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "QuadSPI");
	q->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(q->iobase))
		return PTR_ERR(q->iobase);

	/* Qspi AMBA bus memory mapped address: [0]-start [1]-size */
	ret = of_property_read_u32_array(np, "amba-addr", amba_addr, 2);
	if (ret) {
		dev_err(dev, "Missing 'amba-addr' field in DTB\n");
		return -EINVAL;
	}
	q->memmap_phy = amba_addr[0]; /* Start addr */

	/* Manually setup resource object for iomap */
	res->start = amba_addr[0];
	res->end = amba_addr[0] + amba_addr[1];
	res->flags = IORESOURCE_MEM;
	q->amba_base = devm_ioremap_resource(dev, res);

	/* Setup clock */
	q->clksel = devm_clk_get(dev, "sel");
	if (IS_ERR(q->clksel))
		return PTR_ERR(q->clksel);

	q->clkgate = devm_clk_get(dev, "gate");
	if (IS_ERR(q->clkgate))
		return PTR_ERR(q->clkgate);

	ret = clk_prepare_enable(q->clksel);
	if (ret) {
		dev_err(dev, "Can not enable the qspi_sel clock\n");
		return ret;
	}
	ret = clk_prepare_enable(q->clkgate);
	if (ret) {
		dev_err(dev, "Can not enable the qspi clock gate\n");
		goto clk_failed;
	}

	/* Find the irq */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to get the irq\n");
		goto irq_failed;
	}

	/* Request irq */
	ret = devm_request_irq(dev, ret,
			fsl_qspi_irq_handler, 0, pdev->name, q);
	if (ret) {
		dev_err(dev, "Failed to request irq.\n");
		goto irq_failed;
	}

	q->dev = dev;
	q->devtype_data = (struct fsl_qspi_devtype_data *)of_id->data;
	platform_set_drvdata(pdev, q);

	ret = fsl_qspi_setup(q);
	if (ret)
		goto irq_failed;

	/* Iterate the subnodes. */
	i = 0;
	for_each_available_child_of_node(dev->of_node, np) {
		char modalias[40];
		u32 tmp;
		enum read_mode io_mode;

		nor = &q->nor[i];
		mtd = &q->mtd[i];

		nor->mtd = mtd;
		nor->dev = dev;
		nor->priv = q;
		mtd->priv = nor;

		/* Fill the hooks */

		nor->read_reg = fsl_qspi_read_reg;
		nor->write_reg = fsl_qspi_write_reg;
		nor->read = fsl_qspi_read;
		nor->write = fsl_qspi_write;
		nor->erase = fsl_qspi_erase;
		nor->prepare = fsl_qspi_prep;
		nor->unprepare = fsl_qspi_unprep;

		ret = of_modalias_node(np, modalias, sizeof(modalias));
		if (ret < 0)
			goto irq_failed;

		/* Get IO mode.  If none exists, just default to single. */
		io_mode = SPI_NOR_NORMAL;
		ret = of_property_read_u32(np, "io-mode", &tmp);
		if (!ret) {
			if (tmp == 2)
				io_mode = SPI_NOR_DUAL;
			else if (tmp == 4)
				io_mode = SPI_NOR_QUAD;
		}

		/* Get desired clock freq and set it */
		ret = of_property_read_u32(np, "spi-max-frequency",
				&q->devtype_data->clk_hz);
		if (ret < 0)
			goto irq_failed;
		q->devtype_data->divisor =
			fsl_qspi_cal_clkdiv(clk_get_rate(q->clkgate),
				q->devtype_data->clk_hz);
		ret = fsl_qspi_set_clkdiv(nor, q->devtype_data->divisor);
		if (ret < 0)
			goto irq_failed;

		/* Get actual clk speed to print */
		q->devtype_data->clk_actual_hz =
			fsl_qspi_get_clk_freq(clk_get_rate(q->clkgate),
			q->devtype_data->divisor);

		/* Idendtify flash and setup things up:
		 * - Flash sizes (size, sector size, etc)
		 * - Read/write opcodes
		 * - Io mode (single, dual, quad)
		 * ...and other stuff.
		 */
		ret = spi_nor_scan(nor, modalias, io_mode);
		if (ret)
			goto irq_failed;

		ppdata.of_node = np;
		ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
		if (ret)
			goto irq_failed;

		/* Set the correct NOR size now. */
		if (q->nor_size == 0)
			q->nor_size = mtd->size;

		/* Set D4400 address mode according to flash size.  Do this
		 * first before setting the AMBA bus read opcode as the read
		 * opcode depends on the address mode setting.
		 */
		fsl_qspi_set_addr_mode(nor, mtd->size);

		/* Set AMBA read opcode.  Set the D4400 address mode first. */
		ret = fsl_qspi_set_amba_read_opcode(nor);
		if (ret)
			goto irq_failed;

		/*
		 * The TX FIFO is 64 bytes in the D4400, but the Page Program
		 * may writes 265 bytes per time. The write is working in the
		 * unit of the TX FIFO, not in the unit of the SPI NOR's page
		 * size.
		 *
		 * So shrink the spi_nor->page_size if it is larger then the
		 * TX FIFO.
		 */
		if (nor->page_size > q->devtype_data->txfifo)
			nor->page_size = q->devtype_data->txfifo;

		/* Log info during boot. */
		dev_info(dev, "Clock %i Khz,  divisor %i\n",
			q->devtype_data->clk_actual_hz/1000,
			q->devtype_data->divisor);
		i++;
	}

	/* Create char device */
	ret = alloc_chrdev_region(&q->devt, 0, 1, QSPI_MOD_NAME);
	if (ret) {

		ret = -ENOMEM;
		goto irq_failed;
	}
	q->major = MAJOR(q->devt);
	q->minor = MINOR(q->devt);

	/* Create device class for sysfs */
	q->class = class_create(THIS_MODULE, QSPI_MOD_NAME);
	if (IS_ERR(q->class)) {
		ret = PTR_ERR(q->class);
		goto out_class;
	}

	/* Create character device */
	cdev_init(&q->dev_cdev, &sysfs_qspi_fops);
	q->dev_cdev.owner = THIS_MODULE;
	ret = cdev_add(&q->dev_cdev, q->devt, 1);
	if (ret) {
		pr_err(QSPI_MOD_NAME ": Error %d while adding cdev",
			ret);
		goto out_cdev;
	}

	/* Create the device node in /dev */
	sysfs_dev = device_create(q->class, &pdev->dev,
		q->devt, NULL, QSPI_MOD_NAME);
	if (NULL == sysfs_dev) {
		ret = PTR_ERR(sysfs_dev);
		pr_err(QSPI_MOD_NAME ": Error %d while creating device",
			 ret);
		goto out_dev;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret < 0) {
		pr_err(QSPI_MOD_NAME ": Error %d while creating group",
			ret);
		goto out_group;
	}

	return ret;

out_group:
	device_destroy(q->class, q->devt);
out_dev:
	cdev_del(&q->dev_cdev);
out_cdev:
	class_destroy(q->class);
out_class:
	unregister_chrdev_region(q->devt, 1);
irq_failed:
	clk_disable_unprepare(q->clksel);
clk_failed:
	clk_disable_unprepare(q->clkgate);
	kfree(q);
	return ret;
}

static int fsl_qspi_remove(struct platform_device *pdev)
{
	struct fsl_qspi *q = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < q->nor_num; i++)
		mtd_device_unregister(&q->mtd[i]);

	/* Disable the hardware and interrupts */
	writel(QSPI_MCR_MDIS_MASK, q->iobase + QSPI_MCR_REG);
	writel(0x0, q->iobase + QSPI_RSER_REG);

	clk_unprepare(q->clksel);
	clk_unprepare(q->clkgate);

	sysfs_remove_group(&pdev->dev.kobj, &attr_group);
	device_destroy(q->class, q->devt);
	cdev_del(&q->dev_cdev);
	class_destroy(q->class);
	unregister_chrdev_region(q->devt, 1);

	return 0;
}

static struct platform_driver fsl_qspi_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(fsl_qspi_dt_ids),
		   },
	.probe = fsl_qspi_probe,
	.remove = fsl_qspi_remove,
};

static int __init fsl_qspi_init(void)
{
	int ret = 0;

	/* If boot mem type is not qspi, don't bother loading driver. */
	if (!src_get_boot_mem_type())
		return -EINVAL;

	ret = platform_driver_register(&fsl_qspi_driver);
	return ret;
}

static void __exit fsl_qspi_exit(void)
{
	platform_driver_unregister(&fsl_qspi_driver);
}

module_init(fsl_qspi_init);
module_exit(fsl_qspi_exit);

MODULE_DESCRIPTION("QSPI Master Controller driver");
MODULE_AUTHOR("Freescale, Inc.");

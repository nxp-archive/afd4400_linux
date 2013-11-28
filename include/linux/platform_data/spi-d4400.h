
#ifndef __D4400_SPI_H_
#define __D4400_SPI_H_

#define D4400_CSPIRXDATA                0x00
#define D4400_CSPITXDATA                0x04
#define D4400_CSPICTRL                  0x08
#define D4400_CSPIINT                   0x0c
#define d4400_RESET                     0x1c

/* generic defines to abstract from the different register layouts */
#define D4400_INT_RR    (1 << 0) /* Receive data ready interrupt */
#define D4400_INT_TE    (1 << 1) /* Transmit FIFO empty interrupt */

/* Sharing same chipselects between 2 different SLAVE on spi5 */
#define D4400_MEDUSA_SPI5_MASTER_BUS_NUMBER     5

#define D4400_MEDUSA_SPI5_CS0   0
#define D4400_MEDUSA_SPI5_CS1   1
#define D4400_MEDUSA_SPI5_CS2   2
#define D4400_MEDUSA_SPI5_CS3   3
#define D4400_MEDUSA_SPI5_CS4   4
#define D4400_MEDUSA_SPI5_CS5   5

/*
 * struct spi_d4400_master - device.platform_data for SPI controller devices.
 * @chipselect: Array of chipselects for this master. Numbers >= 0 mean gpio
 *              pins, numbers < 0 mean internal CSPI chipselects according
 *              to D4400_SPI_CS(). Normally you want to use gpio based chip
 *              selects as the CSPI module tries to be intelligent about
 *              when to assert the chipselect: The CSPI module deasserts the
 *              chipselect once it runs out of input data. The other problem
 *              is that it is not possible to mix between high active and low
 *              active chipselects on one single bus using the internal
 *              chipselects. Unfortunately Freescale decided to put some
 *              chipselects on dedicated pins which are not usable as gpios,
 *              so we have to support the internal chipselects.
 * @num_chipselect: ARRAY_SIZE(chipselect)
 */
struct spi_d4400_master {
	int	*chipselect;
	int	num_chipselect;
};

struct spi_d4400_config {
	unsigned int speed_hz;
	unsigned int bpw;
	unsigned int mode;
	u8 cs;
};

enum spi_d4400_devtype {
	D4400_ECSPI,    /* ECSPI on D4400 */
};

struct spi_d4400_data;

struct spi_d4400_devtype_data {
	void (*intctrl)(struct spi_d4400_data *, int);
	int (*config)(struct spi_d4400_data *, struct spi_d4400_config *);
	void (*trigger)(struct spi_d4400_data *);
	int (*rx_available)(struct spi_d4400_data *);
	void (*reset)(struct spi_d4400_data *);
	enum spi_d4400_devtype devtype;
};

struct spi_d4400_data {
	struct spi_master *master;

	spinlock_t              lock;

	struct completion xfer_done;
	void __iomem *base;
	int irq;
	struct clk *clk_per;
	struct clk *clk_ipg;
	unsigned long spi_clk;
	unsigned int count;

	void (*tx)(struct spi_d4400_data *);
	void (*rx)(struct spi_d4400_data *);
	void *rx_buf;
	const void *tx_buf;
	unsigned int txfifo; /* number of words pushed in tx FIFO */

	const struct spi_d4400_devtype_data *devtype_data;
	int chipselect[0];
};

#define D4400_SPI_BUF_RX(type)                                          \
static void spi_d4400_buf_rx_##type(struct spi_d4400_data *spi_d4400)           \
{                                                                       \
	unsigned int val = readl(spi_d4400->base + D4400_CSPIRXDATA);   \
	                                                                        \
	if (spi_d4400->rx_buf) {                                                \
		*(type *)spi_d4400->rx_buf = val;                               \
		spi_d4400->rx_buf += sizeof(type);                      \
	}                                                               \
}

#define D4400_SPI_BUF_TX(type)                                          \
static void spi_d4400_buf_tx_##type(struct spi_d4400_data *spi_d4400)           \
{                                                                       \
	type val = 0;                                                   \
	                                                                        \
	if (spi_d4400->tx_buf) {                                                \
		val = *(type *)spi_d4400->tx_buf;                               \
		spi_d4400->tx_buf += sizeof(type);                      \
	}                                                               \
                                                                        \
	spi_d4400->count -= sizeof(type);                                       \
                                                                        \
	writel(val, spi_d4400->base + D4400_CSPITXDATA);                        \
}

D4400_SPI_BUF_RX(u8)
D4400_SPI_BUF_TX(u8)
D4400_SPI_BUF_RX(u16)
D4400_SPI_BUF_TX(u16)
D4400_SPI_BUF_RX(u32)
D4400_SPI_BUF_TX(u32)

#define D4400_SPI_CS(no)	((no) - 32)

#define D4400_ECSPI_CTRL                0x08
#define D4400_ECSPI_CTRL_ENABLE         (1 <<  0)
#define D4400_ECSPI_CTRL_XCH            (1 <<  2)
#define D4400_ECSPI_CTRL_MODE_MASK      (0xf << 4)
#define D4400_ECSPI_CTRL_POSTDIV_OFFSET 8
#define D4400_ECSPI_CTRL_PREDIV_OFFSET  12
#define D4400_ECSPI_CTRL_CS(cs)         ((cs) << 18)
#define D4400_ECSPI_CTRL_BL_OFFSET      20

#define D4400_ECSPI_CONFIG      0x0c
#define D4400_ECSPI_CONFIG_SCLKPHA(cs)  (1 << ((cs) +  0))
#define D4400_ECSPI_CONFIG_SCLKPOL(cs)  (1 << ((cs) +  4))
#define D4400_ECSPI_CONFIG_SBBCTRL(cs)  (1 << ((cs) +  8))
#define D4400_ECSPI_CONFIG_SSBPOL(cs)   (1 << ((cs) + 12))
#define D4400_ECSPI_CONFIG_SCLKCTL(cs)  (1 << ((cs) + 20))

#define D4400_ECSPI_INT         0x10
#define D4400_ECSPI_INT_TEEN            (1 <<  0)
#define D4400_ECSPI_INT_RREN            (1 <<  3)

#define D4400_ECSPI_STAT                0x18
#define D4400_ECSPI_STAT_RR             (1 <<  3)

#endif /* __D4400_SPI_H_*/

#ifndef _LINUX_XCVR_H
#define _LINUX_XCVR_H

extern void d4400_pinmux_hack(int index);

#define SPI_LEN_NORMAL 4
#define SPI_LEN_PE4312 1
#define SPI_LEN_AD7689 2

struct spi_device_params {
	int id;
	int len;
	int bits_per_word;
	int cs_change;
	int mode;
	int max_speed_hz;
};

#endif

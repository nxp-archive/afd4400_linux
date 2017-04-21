#ifndef _UAPI_LINUX_XCVR_H
#define _UAPI_LINUX_XCVR_H

#include <linux/types.h>

enum xcvr_present_info {
	ROC0_PRESENT = (1 << 0),
	ROC1_PRESENT = (1 << 1),
	WB_PRESENT = (1 << 2),
	WB2_PRESENT = (1 << 3),
};

enum rw_operation {
	READ_OP,
	WRITE_OP,
};

enum wideband_spi_chip_id {
	WB_SPIADC_SENSOR,
	WB_AD9680,
	WB_AD9144,
	WB_SRX_ATTN,
	WB_DCCLO_LMX2485,
	WB_TXLO_LMX2485,
	WB_SRXLO_LMX2485,
	WB_REFLO_ADF4002,
	WB_TX1_ATTN,
	WB_TX2_ATTN,
	WB_GBM_SRX_OFFSET_DAC,
	WB_SRX_DIFF_OFFSET_DAC,
	WB_SPI_CNT
};

/* Wideband 2 */
#define WB_SRX_ATTN_PE43711		WB_SRX_ATTN
#define WB_DCCLO_LMX2492		WB_DCCLO_LMX2485
#define WB_TXLO_LMX2492			WB_TXLO_LMX2485
#define WB_SRXLO_LMX2492		WB_SRXLO_LMX2485
#define WB_TX1_ATTN_PE43711		WB_TX1_ATTN
#define WB_TX2_ATTN_PE43711		WB_TX2_ATTN
#define WB_GBM_SRX_OFFSET_DAC_AD5457R	WB_GBM_SRX_OFFSET_DAC
#define WB_SRX_DIFF_OFFSET_DAC_AD5457R	WB_SRX_DIFF_OFFSET_DAC

/* Output gpio pins from DFE to wideband
 * used as enable signal.
 */
enum dfe_output_to_wideband_gpio {
	WB_TX1_ENABLE,
	WB_TX2_ENABLE,
	WB_SRX1_SELECT_FWD,
	WB_SRX2_SELECT_FWD,
	WB_SRX1_SELECT,
	WB_SRX_ENABLE,
	WB_GBM_ENABLE_B,
	WB_COMMON_TXSRX_LO,
	WB_OUTPUT_PINCNT,

};

/* Input gpio pins from wideband to DFE,
 * used as status signal.
 */
enum dfe_input_wideband_gpio {
	WB_100FC_LD,
	WB_DCC_LD,
	WB_TXLO_LD,
	WB_SRXLO_LD,
	WB_PAP_OUT0,
	WB_PAP_OUT1,
	WB_FAST_DET_A,
	WB_FAST_DET_B,
	WB_INPUT_PINCNT
};

enum roc_spi_chip_id {
	AD93681,
	AD93682,
	AD9525,
	ROC_SPI_CNT,
};

enum dfe_output_to_roc_gpio {
	AD93681_RXEN,
	AD93682_TXEN,
	AD93682_SRXEN,
	ROC_OUTPUT_PINCNT,
};

enum board_type {
	ROC0,
	ROC1,
	WIDEBAND,
	WIDEBAND2,
};

struct xcvr_spi_buf {
	enum board_type type;
	unsigned int  chip_id;
	/* READ_OP or WRITE_OP*/
	enum rw_operation operation;
	char tx_buf[8];
	char rx_buf[8];
};

struct xcvr_gpio_buf {
	enum board_type type;
	unsigned int gpio_id;
	/* READ_OP or WRITE_OP*/
	enum rw_operation operation;
/* Stores the read val or use this val to write,
 * depending on the operation field.
 */
	int val;
};

/* Only AD93681_ROC0/1, AD93682_ROC0/1, AD9144 accepts reset */
struct xcvr_reset_buf {
	enum board_type type;
	int chip_id;
};

#define XCVR_MAGIC 'Z'

#define XCVR_READ_INFO _IOR(XCVR_MAGIC, 0, int)
#define XCVR_SPI_OPERATION _IOWR(XCVR_MAGIC, 1, struct xcvr_spi_buf *)
#define XCVR_GPIO_OPERATION _IOWR(XCVR_MAGIC, 2, struct xcvr_gpio_buf *)
#define XCVR_RESET_OPERATION _IOW(XCVR_MAGIC, 3, struct xcvr_reset_ctrl *)

#endif

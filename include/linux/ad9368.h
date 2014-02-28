/*
 * include/linux/ad9368.h
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __AD9368_H__
#define __AD9368_H__

#include <linux/cdev.h>
#include <linux/types.h>
#include <uapi/linux/ad9368.h>


/* For creating instruction word/command */
#define OPCODE_WRITE		0x80
#define OPCODE_READ		0x00
#define BYTES_TRANSFER_MASK	0x70
#define BYTES_CLK_TRANS_MASK	0x60
#define SHIFT_BYTES_TRANSFER	4
#define SHIFT_CLK_BYTES_TRANS	5
#define REG_ADDRESS_MASK	0x03FF
#define REG_AD9525_ADDRESS_MASK	0x07FF
#define COMMAND_MASK		0xFFFF0000
#define COMMAND_LEN		2
#define OPCODE_WRITE_AD9368	0x80
#define OPCODE_READ_AD9525	0x80
#define OPCODE_WRITE_AD9525	0x00
#define OPCODE_INIT_REG		0x00
#define OPCODE_INIT_VAL		0x01
#define OPCODE_MIRROR_VAL	0x81
#define AD9525_READBACK_REG	0x04
#define AD9525_PUSH_WRITE1_REG	0x02
#define AD9525_PUSH_WRITE2_REG	0x32

#define TRANSACTION_BYTES		3
#define MAX_CLK_READ_TRANS_SIZE		3
#define MAX_READ_TRANSACTION_SIZE	8
#define RXBUF_SIZE			10
#define RXBUF_CLK_SIZE			5
#define NUM_GPIOS			2
#define MAX_GPIO_CONFIGS		(1 << NUM_GPIOS)
#define GPIO_INVAL			(~0)

#define BBPLL_LOCK_MASK 0x80
#define PRODUCT_CODE_AD93681_REG 0x04
#define PRODUCT_CODE_AD93682_REG 0x04
#define PRODUCT_CODE_AD9525_REG 0x04
#define PRODUCT_ID_MASK 0x08
#define PRODUCT_ID_9368 0x08
#define REV_MASK	0x07

/* Calibration status Registers */
#define REG_CH1_OVERFLOW	0x05E
#define REG_RX_CAL_STATUS	0x244
#define REG_TX_CAL_STATUS	0x284
#define REG_CALIBRATION_CONTROL	0x161
#define REG_CALIBRATION_CONFIG1	0x169
#define REG_RX_CP_CONFIG	0x23D
#define REG_TX_CP_CONFIG	0x27D
#define REG_ARMBUSY			0x414
#define REG_ARMSTATUS			0x41C
#define REG_CALPLL_LOCK		0x14D
#define REG_CLKPLL_LOCK		0x125
#define REG_INITARM			0x41B
#define REG_RXADC		0x24B
#define VAL_CAL_CONF1_TRACKOFF	0xC0

#define REG_RF_CP_CONFIG	0x18D
#define REG_CLKPLLCP			0x122
#define MASK_CLKPLL_SET		0x01
#define REG_CP_OVERRANGE_VCO_LOCK 0x190
#define MASK_RFPLLLOCK_SET	0x01
/* Calibration status masks */
#define MASK_BBPLL_LOCK		0x80
#define MASK_RX_CP_CAL_VALID	0x80
#define MASK_TX_CP_CAL_VALID	0x80
#define MASK_RX_BB_TUNE		0x80
#define MASK_ARMBUSY		0x80
#define MASK_CALPLL_LOCK	0x80
#define MASK_CLKPLL_LOCK	0x80
#define MASK_CLKPLLCP		0x80
#define MASK_INITARM		0xFF
#define MASK_RXADC		0x20
#define MASK_TX_BB_TUNE		0x01
#define MASK_DC_CAL_BBSTART	0x00
#define MASK_ADC_TUNE_CAL_START	0x80
#define MASK_RCAL_START		0x40
#define MASK_RXTIA_CAL_START	0x20
#define MASK_DC_CAL_RFSTART	0x02
#define MASK_RFPLLCP_CAL	0x80
#define MASK_TXQUAD_CAL		0x08

/*ENSM config1 register*/
#define REG_ENSM_CONF1			0x014
#define ENSM_CONF1_TO_ALERT		(1 << 0)
#define ENSM_CONF1_AUTO_GAIN_LOCK	(1 << 1)
#define ENSM_CONF1_FORCE_ALERT		(1 << 2)
#define ENSM_CONF1_LEVEL_MODE		(1 << 3)
#define ENSM_CONF1_ENSM_PIN_CTL_EN	(1 << 4)
#define ENSM_CONF1_FORCE_TX_ON		(1 << 5)
#define ENSM_CONF1_FORCE_RX_ON		(1 << 6)
#define ENSM_CONF_RX_EN_CAL		(1 << 7)

/*ENSM state - Read only*/
#define REG_DEV_STATE			0x017
#define ENSM_STATE_SHIFT		0x0
#define ENSM_STATE_MASK			0x0f

#define ENSM_STATE_SLEEP_WAIT		0x0
#define ENSM_STATE_ALERT		0x5
#define ENSM_STATE_TX			0x6
#define ENSM_STATE_TX_FLUSH		0x7
#define ENSM_STATE_RX			0x8
#define ENSM_STATE_RX_FLUSH		0x9
#define ENSM_STATE_FDD			0xA
#define ENSM_STATE_FDD_FLUSH		0xB
#define ENSM_STATE_INVALID		0xff

#define MAX_RFICS 20
#define MAX_PHY_DEVS_PER_CARD	3

#define MEDUSA_DEV_ID           0
#define DFE_DEV_ID              1

#define BAD_DIS					(1 << 7)
#define NOT_IN_TABLE				(1 << 6)
#define UNEXP_K_CHARS				(1 << 5)
#define BAD_CS					(1 << 2)

#define CLEAR_ERROR_IRQ				(1 << 7)
#define DISABLE_ERROR_COUNTER			(1 << 6)
#define RESET_ERROR_COUNTER			(1 << 5)

#define JESD_ERR_EVT_ALL	(BAD_DIS \
				| NOT_IN_TABLE \
				| UNEXP_K_CHARS \
				| BAD_CS)

#define REG_SUB_JESD_ADDR	0x09C
#define REG_SUB_JESD_DATA	0x09D
#define REG_WRITE_EN_JESD	0x09E
#define REG_INT_EN_JESD		0x7A
#define REG_BAD_CS_JESD		0x72
#define REG_BAD_DIS_JESD	0x6D
#define REG_NOT_IN_TABLE_JESD	0x6E
#define REG_UNEXP_K_CHARS_JESD	0x6F
#define REG_READ_COUNTER_JESD	0x6B



struct rf_device_id {
	char name[RF_NAME_SIZE];
	int device_id;
};

enum band_config_mode {
	GPIO_MODE,
	FPGA_MODE,
	INVALID_MODE
};

struct jesd_dev_stats {
	unsigned int bad_disparity_err_count;
	unsigned int not_in_table_err_count;
	unsigned int unexpected_k_chars_err_count;
	unsigned int bad_checksum_err_count;
};

struct ad_dev_info {
	struct spi_device *ad_spi;
	u8	rx_buf[10];
	int	device_id;
	u8 prev_ensm_state;
	int ensm_pin_ctl_en;
	unsigned int freq_band;
	spinlock_t	spi_lock;
	struct	list_head list;
	int reset_gpio;
	int pa_en_gpio;
	int lna_en_gpio;
	int rf_num;
};

struct rf_phy_dev {
	char	name[RF_NAME_SIZE];
	struct device *dev;
	struct device_node	*rf_dev_node;
	u32	phy_id;
	struct xcvr_dev	*xcvr_dev;
	struct	list_head list;
	void	*priv;
};

struct xcvr_dev {
	struct device		*dev;
	struct list_head	list;
	int			device_id;
	u8			device_flag;
	struct	rf_phy_ops	*ops;
	unsigned int		phy_devs;
	struct rf_phy_dev	*phy_dev[3];
	struct device_node	*rf_dev_node[3];
	dev_t			dev_t;
	struct cdev		cdev;
	int			*cs_gpios;
	unsigned int		irq_tx;
	unsigned int		irq_rx;
	unsigned int		err_status;
	struct work_struct      err_task;
	struct jesd_dev_stats	stats[2];
	int gpio_tx_enable;
	int gpio_srx_enable;
	int gpio_rx_enable;
	void *src_handle;
	int src_tx_reset;
	int src_rx_reset;
	atomic_t		ref;
};

struct rf_phy_ops {
	int	(*init)(struct xcvr_dev *phy,
				struct rf_init_params *params);

	int	(*run_cmds)(struct xcvr_dev *phy,
				struct rf_phy_cmd *cmds, int count);

	int	(*read_regs)(struct xcvr_dev *phy, u32 start,
				u32 count, u32 *buff);

	int	(*write_reg)(struct xcvr_dev *phy, u32 reg,
			 u32 data, u8 probe);
	int	(*set_tx_atten)(struct xcvr_dev *phy, u32 reg, u32 data);
	int	(*en_dis_tx)(struct xcvr_dev *phy, u32 tx_if, u32 cmd);
	int	(*get_rx_gain)(struct xcvr_dev *phy,
			struct rf_rx_gain *rx_gain);
	int	(*set_rx_gain)(struct xcvr_dev *phy,
			struct rf_rx_gain *rx_gain);
	int	(*start)(struct xcvr_dev *phy);
	int	(*stop)(struct xcvr_dev *phy);
	int	(*set_gain_ctrl_mode)(struct xcvr_dev *phy,
			struct rf_gain_ctrl *gain_ctrl);
	int	(*spi_ioc_transfer)(struct xcvr_dev *phy,
			struct spi_ioc_transfer *u_xfers, unsigned n_xfers);
};

extern struct rf_phy_dev *get_attached_phy_dev(struct device_node *rf_dev_node);
extern struct xcvr_dev *get_attached_xcvr_dev(struct device_node **dev_node,
	struct rf_phy_dev *phy_dev);
extern int ad9368_read(struct rf_phy_dev *phy_dev, u32 start, u32 count,
		u32 *buff);
extern int ad9368_write(struct rf_phy_dev *phy_dev, u32 reg,
		 u32 data, u8 probe);
extern int ad9368_run_cmds(struct rf_phy_dev *phy_dev,
		struct rf_phy_cmd *cmds,
		int count);
extern int rfdev_message(struct rf_phy_dev *rf_dev,
	struct spi_ioc_transfer *u_xfers, unsigned n_xfers);

extern int ad_init(struct rf_phy_dev *phy, struct rf_init_params *params);
extern int ad9368_start(struct rf_phy_dev *phy_dev);
extern int check_cal_done(struct rf_phy_dev *phy_dev, u32 reg, u32 mask,
		u32 bitval);
extern int check_cal_done_4regs(struct rf_phy_dev *phy_dev, u32 reg, u32 mask);
#endif /* __AD9368_H__ */

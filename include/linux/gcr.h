#ifndef _GCR_DRIVER_H_
#define _GCR_DRIVER_H_

#include <linux/cdev.h>
#include <linux/mutex.h>
#include <uapi/linux/cpri.h>
#include <uapi/linux/gcr.h>

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT

/* GCR value to init JESD */
#define TPD_BGR_EN		0x00140000
#define TBGEN_E0_E1_EN		0x30

/* gcr 1 device is max for user app */
#define GCR_DEV_MAX    1

/* GCR addresses and size */
#define CFG_GCR_BASE             0x012C0000
#define CFG_GCR_SIZE             0x16C

#define CFG_NUM_GSR_REG		7

/* limit defines for varrious IP's */
#define DMA_CHAN_MIN_ID 0
#define DMA_CHAN_MAX_ID 31

#define JESD_RX_PTR_RST_MAX 13
#define JESD_RX_PTR_RST_MIN 1
#define JESD_TX_PTR_RST_MAX 10
#define JESD_TX_PTR_RST_MIN 1

#define MAX_VSP_ID 11
#define MIN_VSP_ID 1

#define MIN_VSP_MODULE1_RX_ID 1
#define M1VSP_MIN 1
#define M1VSP_MAX 4
#define M2VSP_MIN 8
#define M2VSP_MAX 11
#define M3VSP_MIN 5
#define M3VSP_MAX 7

#define GCR_OFFSET_56 56
#define GCR_OFFSET_60 60




#define MAX_INTER_VSP_DMA_CHAN 4 /*allocated to inter VSP communication */
/* end here */


/* inter vsp macro's */
#define GCR_MSK3 0xf
#define INTER_VSP_CHAN_MIN 4
#define INTER_VSP_CHAN_MAX 7
/* end here */

/* bit shift value for jesd PRR */
#define JESD_PRR_SHIFT_VAL_00	0
#define JESD_PRR_SHIFT_VAL_03	3
#define JESD_PRR_SHIFT_VAL_06	6
#define JESD_PRR_SHIFT_VAL_11	11
#define JESD_PRR_SHIFT_VAL_14	14
#define JESD_PRR_SHIFT_VAL_17	17
#define JESD_PRR_SHIFT_VAL_19	19
#define JESD_PRR_SHIFT_VAL_20	20
#define JESD_PRR_SHIFT_VAL_23	23
#define JESD_PRR_SHIFT_VAL_25	25
#define JESD_PRR_SHIFT_VAL_29	29


/* prepheral gcr dma channel config bit mask */
#define GCR_CPRI_RX1_DMA_REQ_2BIT_1  0x0
#define GCR_CPRI_RX1_DMA_REQ_2BIT_2  0x1
#define GCR_CPRI_RX2_DMA_REQ_2BIT_1  0x2
#define GCR_CPRI_RX2_DMA_REQ_2BIT_2  0x3

#define GCR_CPRI_TX1_DMA_REQ_2BIT_1  0x1
#define GCR_CPRI_TX1_DMA_REQ_2BIT_2  0x2
#define GCR_CPRI_TX2_DMA_REQ_2BIT_1  0x3
#define GCR_CPRI_TX2_DMA_REQ_2BIT_2  0x4

/* 3 bit mask for gcr reg  bits key mapped DMA request */
#define CPRI_RX1_DMA_REQ_2	0x2
#define CPRI_RX1_DMA_REQ_3	0x3
#define CPRI_RX1_DMA_REQ_4	0x4

#define CPRI_RX2_DMA_REQ_4	0x4
#define CPRI_RX2_DMA_REQ_5	0x5
#define CPRI_RX2_DMA_REQ_6	0x6

#define CPRI_TX1_DMA_REQ_3	0x3
#define CPRI_TX1_DMA_REQ_4	0x4
#define CPRI_TX1_DMA_REQ_5	0x5

#define CPRI_TX2_DMA_REQ_5	0x5
#define CPRI_TX2_DMA_REQ_6	0x6
#define CPRI_TX2_DMA_REQ_7	0x7

#define JESD_RX_DMA_REQ_1	0x1
#define JESD_RX_DMA_REQ_2	0x2
#define JESD_RX_DMA_REQ_3	0x3

#define JESD_TX_DMA_REQ_1	0x1
#define JESD_TX_DMA_REQ_2	0x2

/* MACRO defines */
#define GCR_MSK 0x7

#define CPRI_RX1_DMA_REQ_2BIT(x) GCR_CPRI_RX1_DMA_REQ_2BIT_##x
#define CPRI_RX2_DMA_REQ_2BIT(x) GCR_CPRI_RX2_DMA_REQ_2BIT_##x

#define CPRI_TX1_DMA_REQ_2BIT(x) GCR_CPRI_TX1_DMA_REQ_2BIT_##x
#define CPRI_TX2_DMA_REQ_2BIT(x) GCR_CPRI_TX2_DMA_REQ_2BIT_##x

#define GCR_MSK1 0x3
#define GCR_MSK2 0x1

#define GCR72_SYNC_CTRL	72
#define BGR_EN_TX10_SYNC 0x100000

#define GCR4_CPRI_CTRL	4
#define GCR6_CPRI_CTRL	6
#define CPRI_PHY_LINK_RESET	0xFFFFFFC1
#define SET_LINE_RATE	1
#define CLEAR_LINE_RATE	0
#define GCR0_PLL_SYS_DEV_CLK_OFFSET     19
#define GCR0_PLL_SYS_DEV_CLK_MASK       (1<<GCR0_PLL_SYS_DEV_CLK_OFFSET)
#define GCR0_PLL_SYS_RGMII_CLK_OFFSET   20
#define GCR0_PLL_SYS_RGMII_CLK_MASK     (1<<GCR0_PLL_SYS_RGMII_CLK_OFFSET)
#define GCR0_PLL_DDR_DEV_CLK_OFFSET     21
#define GCR0_PLL_DDR_DEV_CLK_MASK       (1<<GCR0_PLL_DDR_DEV_CLK_OFFSET)
#define GCR0_PLL_DDR_RGMII_CLK_OFFSET   22
#define GCR0_PLL_DDR_RGMII_CLK_MASK     (1<<GCR0_PLL_DDR_RGMII_CLK_OFFSET)

#define SYS_PARENT_CLK_SRC (GCR0_PLL_SYS_DEV_CLK_MASK |\
		GCR0_PLL_SYS_RGMII_CLK_MASK)

#define DDR_PARENT_CLK_SRC (GCR0_PLL_DDR_DEV_CLK_MASK |\
		GCR0_PLL_DDR_RGMII_CLK_MASK)

enum gcr_get_pll_parent_param_t {
	GET_SYS_PLL_PARENT,
	GET_DDR_PLL_PARENT,
};

enum gcr_set_pll_parent_param_t {
	SET_SYS_PLL_PARENT,
	SET_DDR_PLL_PARENT
};

enum parent_src_clk_t {
	PARENT_SRC_DEVCLK,
	PARENT_SRC_SGMIICLK
};

struct gcr_reg_map {

	u32 gcr[90];	/* 0x0000 to 0x0168*/
};

struct gcr_priv {
	struct gcr_reg_map *gcr_reg;
	struct device_node *dev_node;
	struct mutex gcr_lock;
	struct device *gcr_dev;
	struct cdev gcr_cdev;
	struct class *gcr_class;
	dev_t dev_t;
};

extern void gcr_config_cpri_line_rate(unsigned char cpri_id,
		enum cpri_link_rate linerate, unsigned char cmd);
extern void gcr_linkrate_autoneg_reset(unsigned char cpri_id);
void gcr_sync_update(u32 mask, u32 val);
void *get_scm_priv(void);
extern u32 gcr_read_set(u32 gcr_id);
extern void gcr_write_set(struct gcr_ctl_parm *param,
		unsigned char count);
extern int gcr_jesd_dma_ptr_rst_req(struct jesd_dma_ptr_rst_parm *ptr_rst_parm,
		u8 count);
extern int gcr_inter_vsp_dma_cfg(struct inter_vsp_dma_config_t *vsp_parm,
		unsigned char count);
extern int gcr_cpri_dma_mux(struct cpri_dma_mux_config *cpri_mux_parm,
		unsigned count);
extern int gcr_vsp_intf_dma_cfg(struct dma_intf_switch_parm_t *chan_parm,
		unsigned char count);
extern u32 gcr_set_pll_parent(enum gcr_set_pll_parent_param_t gcr_param,
		enum parent_src_clk_t parent_src_clk);
extern u32 gcr_get_pll_parent(enum gcr_get_pll_parent_param_t gcr_param);
#endif/* _GCR_DRIVER_H_ */

#ifndef _GCR_DRIVER_H_
#define _GCR_DRIVER_H_

#include <linux/cdev.h>
#include <linux/mutex.h>
#include <uapi/linux/cpri.h>
#include <uapi/linux/gcr.h>

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT

/* gcr 10 devices are max for user app */
#define GCR_DEV_MAX    10

/* GCR addresses and size */
#define CFG_GCR_BASE             0x012C0000
#define CFG_GCR_SIZE             0x16C

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

#define GCR_OFFSET_56 56
#define GCR_OFFSET_60 60




#define MAX_INTER_VSP_DMA_CHAN 4 /*allocated to inter VSP communication */
/* end here */


/* inter vsp macro's */
#define GCR_MSK3 0xf
#define INTER_VSP_CHAN_MIN 4
#define INTER_VSP_CHAN_MAX 7



/* end here */


/* prepheral gcr dma channel config bit mask */
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_1  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_1  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_2  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_2  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_3  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_3  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_4  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_4  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_5  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_5  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_6  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_6  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_7  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_7  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_8  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_8  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_9  0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_9  0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_10 0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_10 0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_11 0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_11 0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_12 0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_12 0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_13 0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_13 0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_14 0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_14 0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_15 0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_15 0x2
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_16 0x0
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_16 0x2

#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_17 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_17 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_18 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_18 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_19 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_19 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_20 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_20 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_21 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_21 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_22 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_22 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_23 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_23 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_2BIT_24 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_2BIT_24 0x3

/* 3 bit mask for gcr reg  bits key mapped DMA request */
#define GCR_CPRI_FRMR1_DMA_REQ_18 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_18 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_23 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_23 0x5

#define GCR_CPRI_FRMR1_DMA_REQ_17 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_17 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_24 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_24 0x5

#define GCR_CPRI_FRMR1_DMA_REQ_20 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_20 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_21 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_21 0x5

#define GCR_CPRI_FRMR1_DMA_REQ_19 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_19 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_22 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_22 0x5

#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_17 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_17 0x5
#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_24 0x4
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_24 0x6

#define GCR_CPRI_FRMR1_DMA_REQ_3JESD_17 0x4
#define GCR_CPRI_FRMR2_DMA_REQ_3JESD_17 0x6
#define GCR_CPRI_FRMR1_DMA_REQ_3JESD_24 0x5
#define GCR_CPRI_FRMR2_DMA_REQ_3JESD_24 0x7

#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_18 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_18 0x5
#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_23 0x4
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_23 0x6


#define GCR_CPRI_FRMR1_DMA_REQ_3JESD_18 0x4
#define GCR_CPRI_FRMR2_DMA_REQ_3JESD_18 0x6
#define GCR_CPRI_FRMR1_DMA_REQ_3JESD_23 0x5
#define GCR_CPRI_FRMR2_DMA_REQ_3JESD_23 0x7

#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_19 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_19 0x5
#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_22 0x4
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_22 0x6

#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_20 0x3
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_20 0x5
#define GCR_CPRI_FRMR1_DMA_REQ_2JESD_21 0x4
#define GCR_CPRI_FRMR2_DMA_REQ_2JESD_21 0x6

/* gcr dma tx reg mask */

#define GCR_CPRI_FRMR1_DMA_REQ_TX_1  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_1  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_2  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_2  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_3  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_3  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_4  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_4  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_5  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_5  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_6  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_6  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_7  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_7  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_8  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_8  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_9  0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_9  0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_10 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_10 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_11 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_11 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_12 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_12 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_13 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_13 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_14 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_14 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_15 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_15 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_16 0x1
#define GCR_CPRI_FRMR2_DMA_REQ_TX_16 0x3
#define GCR_CPRI_FRMR1_DMA_REQ_TX_17 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_17 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_TX_18 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_18 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_TX_19 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_19 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_TX_20 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_20 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_TX_21 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_21 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_TX_22 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_22 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_TX_23 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_23 0x4
#define GCR_CPRI_FRMR1_DMA_REQ_TX_24 0x2
#define GCR_CPRI_FRMR2_DMA_REQ_TX_24 0x4


/* GCR JESD Bit Mask Value */
#define GCR_JESD_RX_DMA_REQ_1 0x1
#define GCR_JESD_RX_DMA_REQ_2 0x1
#define GCR_JESD_RX_DMA_REQ_3 0x1
#define GCR_JESD_RX_DMA_REQ_4 0x1
#define GCR_JESD_RX_DMA_REQ_5 0x1
#define GCR_JESD_RX_DMA_REQ_41_5 0x2 /* different in gcr 41 */
#define GCR_JESD_RX_DMA_REQ_6 0x1
#define GCR_JESD_RX_DMA_REQ_41_6 0x2 /* different in gcr 41 */
#define GCR_JESD_RX_DMA_REQ_7 0x1
#define GCR_JESD_RX_DMA_REQ_41_7 0x2 /* different in gcr 41 */
#define GCR_JESD_RX_DMA_REQ_8 0x2
#define GCR_JESD_RX_DMA_REQ_5_8 0x1 /* here for GCR5 value is 1 */
#define GCR_JESD_RX_DMA_REQ_9 0x2
#define GCR_JESD_RX_DMA_REQ_10 0x2
#define GCR_JESD_RX_DMA_REQ_11 0x3
#define GCR_JESD_RX_DMA_REQ_12 0x3
#define GCR_JESD_RX_DMA_REQ_13 0x2



#define GCR_JESD_TX_DMA_REQ_1 0x1
#define GCR_JESD_TX_DMA_REQ_2 0x1
#define GCR_JESD_TX_DMA_REQ_3 0x1
#define GCR_JESD_TX_DMA_REQ_4 0x1
#define GCR_JESD_TX_DMA_REQ_5 0x1
#define GCR_JESD_TX_DMA_REQ_6 0x1
#define GCR_JESD_TX_DMA_REQ_7 0x1
#define GCR_JESD_TX_DMA_REQ_8 0x1
#define GCR_JESD_TX_DMA_REQ_9 0x2
#define GCR_JESD_TX_DMA_REQ_10 0x2






/* MACRO defines */
#define CPRI_RX1_DMA_REQ(x) GCR_CPRI_FRMR1_DMA_REQ_##x
#define CPRI_RX2_DMA_REQ(x) GCR_CPRI_FRMR2_DMA_REQ_##x
#define JESDTX_DMA_REQ(x) GCR_JESD_TX_DMA_REQ_##x
#define JESDRX_DMA_REQ(x) GCR_JESD_RX_DMA_REQ_##x
#define GCR_MSK 0x7


#define CPRI_RX1_DMA_REQ_2BIT(x) GCR_CPRI_FRMR1_DMA_REQ_2BIT_##x
#define CPRI_RX2_DMA_REQ_2BIT(x) GCR_CPRI_FRMR2_DMA_REQ_2BIT_##x
#define GCR_MSK1 0x3
#define GCR_MSK2 0x1

#define GCR4_CPRI_CTRL	4
#define GCR6_CPRI_CTRL	6

struct gcr_reg_map {

	u32 gcr[83];	/* 0x0000 to 0x0168*/
};

struct gcr_priv {
	struct gcr_reg_map *gcr_reg;
	struct device_node *dev_node;
	struct mutex gcr_lock;
	struct device *gcr_dev;
	struct cdev gcr_cdev;
	dev_t dev_t;
};

extern struct gcr_priv *get_attached_gcr_dev(struct device_node *gcr_dev_node);
extern void gcr_set_cpri_line_rate(enum cpri_link_rate linerate,
		unsigned char cpri_id, struct gcr_priv *priv);
extern void gcr_linkrate_autoneg_reset(unsigned char cpri_id,
		struct gcr_priv *priv);
void *get_scm_priv(void);
extern u32 *gcr_read_reg(struct gcr_ctl_parm *param, struct device *gcr_dev,
		unsigned char count, struct gcr_reg_map *gcr);
extern int gcr_write_set(struct gcr_ctl_parm *param,
		unsigned char count, struct gcr_reg_map *gcr);
extern int gcr_jesd_dma_ptr_rst_req(struct jesd_dma_ptr_rst_parm *ptr_rst_parm,
		unsigned count, struct gcr_reg_map *gcr);
extern int gcr_inter_vsp_dma_cfg(struct inter_vsp_dma_config_t *vsp_parm,
		unsigned char count, struct gcr_reg_map *gcr);
extern int gcr_cpri_dma_mux(struct cpri_dma_mux_config *cpri_mux_parm,
		unsigned count, struct gcr_reg_map *gcr);
extern int gcr_vsp_intf_dma_cfg(struct dma_intf_switch_parm_t *chan_parm,
		unsigned char count, struct gcr_reg_map *gcr);

#endif/* _GCR_DRIVER_H_ */

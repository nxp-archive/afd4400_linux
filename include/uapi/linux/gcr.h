/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __UAPI_GCR_H
#define __UAPI_GCR_H


#define GCR_IOC_MAGIC           'G'

#define GCR_CONFIG_CMD         _IOW(GCR_IOC_MAGIC, 1, struct gcr_parm *)
#define GCR_CPRI_DMA_MUX_CMD   _IOW(GCR_IOC_MAGIC, 2, struct gcr_parm *)
#define GCR_INTER_VSP_CFG      _IOW(GCR_IOC_MAGIC, 3, struct gcr_parm *)
#define GCR_JESD_PTR_RST_CFG   _IOW(GCR_IOC_MAGIC, 4, struct gcr_parm *)
#define GCR_WRITE_REG          _IOW(GCR_IOC_MAGIC, 5, struct gcr_parm *)
#define GCR_READ_REG           _IOWR(GCR_IOC_MAGIC, 6, struct gcr_parm *)


#define MAX_VSP_IN_DFE   11
#define MAX_VSP_DMA_CHAN 32

struct gcr_parm {
	void *parm;
	unsigned char count;
};
/**
 * Common definitions used in all parts of the API.
 **/

/**  CPRI core group CommonDefs*/
enum cpri_core_info {
	CPRI_CORE_CMPLX_1 = 1,       /**< CPRI core complex group coreId */
	CPRI_CORE_CMPLX_2,       /**< CPRI core complex group coreId */
	CPRI_FRAMER_1 = 1,           /**< CPRI core complex framerId */
	CPRI_FRAMER_2,           /**< CPRI core complex framerId */
	CPRI_RX = 1,                 /**< CPRI cmd framer is selected for rx */
	CPRI_TX                  /**< CPRI cmd framer is selected for tx */
};

/**  DFE inteface device Id's*/
enum dma_req_dev_type_t {
	DEV_CPRI = 1,
	DEV_JESD
};

/**  VSP DMA channel Id's*/
enum dma_channel_id_t {
	DMA_CHAN_0,
	DMA_CHAN_1,
	DMA_CHAN_2,
	DMA_CHAN_3,
	DMA_CHAN_4,
	DMA_CHAN_5,
	DMA_CHAN_6,
	DMA_CHAN_7,
	DMA_CHAN_8,
	DMA_CHAN_9,
	DMA_CHAN_10,
	DMA_CHAN_11,
	DMA_CHAN_12,
	DMA_CHAN_13,
	DMA_CHAN_14,
	DMA_CHAN_15,
	DMA_CHAN_16,
	DMA_CHAN_17,
	DMA_CHAN_18,
	DMA_CHAN_19,
	DMA_CHAN_20,
	DMA_CHAN_21,
	DMA_CHAN_22,
	DMA_CHAN_23,
	DMA_CHAN_24,
	DMA_CHAN_25,
	DMA_CHAN_26,
	DMA_CHAN_27,
	DMA_CHAN_28,
	DMA_CHAN_29,
	DMA_CHAN_30,
	DMA_CHAN_31
};



enum vsp_id_t {
	VSP1 = 1,
	VSP2,
	VSP3,
	VSP4,
	VSP5,
	VSP6,
	VSP7,
	VSP8,
	VSP9,
	VSP10,
	VSP11
};

enum cpri_dma_map {
	CPRI_DMA0,
	CPRI_DMA1,
	CPRI_DMA2,
	CPRI_DMA3,
	CPRI_DMA4,
	CPRI_DMA5,
	CPRI_DMA6,
	CPRI_DMA7,
	CPRI_DMA8,
	CPRI_DMA9,
	CPRI_DMA10,
	CPRI_DMA11,
	CPRI_DMA12,
	CPRI_DMA13,
	CPRI_DMA14,
	CPRI_DMA15,
	CPRI_DMA16,
	CPRI_DMA17,
	CPRI_DMA18,
	CPRI_DMA19,
	CPRI_DMA21,
	CPRI_DMA22,
	CPRI_DMA23
};

enum jesd_id_t {
	JESD1 = 1,
	JESD2,
	JESD3,
	JESD4,
	JESD5,
	JESD6,
	JESD7,
	JESD8,
	JESD9,
	JESD10,
	JESD11,
	JESD12,
	JESD13

};

enum dma_comm_type_t {
	SETUP_UL = 1,
	SETUP_DL
};


/** dma switch command input parameters. */
struct dma_intf_switch_parm_t {
	enum dma_req_dev_type_t dev_type;     /**< inteface device type
						cpri/jesd */
	enum dma_channel_id_t chan_id;    /**< dma channel ID'd valid value
					    0 to 31 */
	enum vsp_id_t vsp_id;                 /**< VSP id values is the key
						for GCR config values 1 to 11*/
	unsigned char dma_request_id; /**< cpri axc id or JESD mapped
					     on dma request
					     valid value 1 to 24 */
	enum cpri_core_info cpri_framert_id; /**< cpri framer id valid value
					       framer1 to framer2 */
	enum dma_comm_type_t comm_type;      /**< dma channel req communication
					       type for Rx or Tx */
};


enum cpri_rxtx_id {
	CPRI_RX1 = 1,
	CPRI_RX2,
	CPRI_TX1,
	CPRI_TX2
};

struct cpri_dma_mux_config {
	enum cpri_core_info src_cpri_complex;
	unsigned char cpri_dma_req_out; /**< cpri axc id mapped on dma request
					  valid value 1 to 24 */
	enum cpri_rxtx_id rxtx_id;
};


struct inter_vsp_dma_config_t {
	enum vsp_id_t src_vsp_id;
	enum vsp_id_t dst_vsp_id;
	enum dma_channel_id_t chan_id;
};

struct jesd_dma_ptr_rst_parm {
	enum vsp_id_t vsp_id;
	enum jesd_id_t jesd_id;
	enum dma_channel_id_t chan_id;
	enum dma_comm_type_t comm_type;      /**< dma channel will be used for
					       JESD RX or TX. */
};

struct  gcr_ctl_parm {
	unsigned int reg_offset;
	unsigned int param;
};

#endif

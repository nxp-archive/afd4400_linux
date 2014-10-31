/*
 * include/linux/cpri.h
 * CPRI device driver
 * Author: Freescale semiconductor, Inc.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __CPRI_H
#define __CPRI_H

#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/bitops.h>
#include <linux/semaphore.h>

#include <uapi/linux/cpri.h>
#include <linux/sfp.h>
#include <linux/gcr.h>
#include <linux/cpri_eth.h>
#include <mach/src.h>

/* Private data structure for driver */
struct cpri_priv {
	struct class *cpri_class;
	struct list_head cpri_dev_list;
	spinlock_t cpri_list_lock;
/* The I2C GPIO expander interrupt output is shared
 * by 4 framers. Only CPRI complex 1 has this entry so
 * it will initialized once.
 * @sfp_int: the GPIO number for the I2C GPIO expander interrupt
 * @sfp_irq: the irq number for I2C GPIO expander interrupt
 */
	int sfp_int;
	int sfp_irq;
	struct work_struct sfp_irq_wq;
};

struct cpri_dev {
	unsigned int dev_id;
	struct cpri_common_regs __iomem *regs;
	struct device *dev;
	spinlock_t lock;
	unsigned int irq_gen1;
	unsigned int irq_gen2;
	unsigned int irq_gen3;
	unsigned int irq_gen4;
	unsigned int irq_err;
	struct tasklet_struct err_tasklet_frmr0;
	struct tasklet_struct err_tasklet_frmr1;
	unsigned int framers;
	u32 dev_flags;
	struct list_head list;
	struct cpri_framer **framer;
};

enum d4400_rev_clk_dev {
	REV_CLK_DIV_1,
	REV_CLK_DIV_2,
	REV_CLK_DIV_4,
	REV_CLK_DIV_8,
};
#define MAX_SFP_ADDR	2

struct sfp_dev {
	u32 id;
	struct device_node *dev_node;
	unsigned int addr_cnt;
	unsigned int addr[MAX_SFP_ADDR];
	enum mem_type type;
	int use_smbus;
	struct cpri_framer *pair_framer;
	struct sfp info;
	struct mutex lock;
	u8 *writebuf;
	unsigned write_max;
	/* SFP stats */
	unsigned int txfault;
	unsigned int rxlos;
	unsigned int prs;
	unsigned int tx_disable;
	struct list_head list;
	struct i2c_client *client;
};

struct cpri_common_regs {
	u32 cpri_ctrlclk;	/* 0x7E00 - CPRI Control Clocks */
	u32 reserved0[3];
	u32 cpri_intctrl[5];	/* 0x7E10 to 0x7E20 - CPRI Interrupt Control */
	u32 reserved1[11];
	u32 cpri_rcpuctrlinten;	/* 0x7E50 - CPRI Rx CPU Ctrl Interrupt En */
	u32 cpri_tcpuctrlinten;	/* 0x7E54 - CPRI Tx CPU Ctrl Interrupt En */
	u32 cpri_rgensync;	/* 0x7E58 - Gen Rx Sync */
	u32 cpri_tgensync;	/* 0x7E5C - Gen Transmit Sync */
	u32 reserved2[13];
	u32 cpri_errstatus;	/* 0x7E94 - CPRI Err Status */
	u32 reserved3[8];
	u32 cpri_remresetoutputctrl; /* 0x7EB8 - CPRI Rem Reset Output Ctrl */
};

struct cpri_framer_regs {
	/* Framer registers */
	u32 reserved;
	u32 cpri_status;	/* 0x4  - Status */
	u32 cpri_config;	/* 0x8  - Configuration */
	u32 reserved0[4];
	u32 cpri_lcv;		/* 0x1C - Rx Line Code Violation Count */
	u32 cpri_bfn;		/* 0x20 - Recovered BFN Counter */
	u32 cpri_hfn;		/* 0x24 - Recovered HFN Counter */
	u32 reserved1;
	u32 cpri_hwreset;	/* 0x2C - HW Reset from Control Word */
	u32 reserved2[3];
	u32 cpri_cmconfig;	/* 0x3C - Control & Management Config */
	u32 cpri_cmstatus;	/* 0x40 - Control & Management Status */
	u32 cpri_rdelay_ctrl;	/* 0x44 - Rx Delay */
	u32 cpri_rdelay;	/* 0x48 - Rx Delay */
	u32 cpri_rnddelay;	/* 0x4C - Round Trip Delay */
	u32 cpri_exdelaycfg;	/* 0x50 - Extended Delay Measur Conf */
	u32 cpri_exdelaystatus;	/* 0x54 - Ext'd Delay Measurt Status */
	u32 cpri_tprotver;	/* 0x58 - Tx Protocol Version */
	u32 cpri_tscrseed;	/* 0x5C - Tx Scrambler Seed */
	u32 cpri_rscrseed;	/* 0x60 - Rx Scrambler Seed */
	u32 reserved4[1];
	u32 cpri_txsyncdelay;	/* 0x6C - Tx Control */
	u32 cpri_tx_control;	/* 0x6C - Tx Control */
	u32 reserved5[4];
	u32 cpri_serdescfg;	/* 0x80 - SerDes Interface Config */
	u32 reserved6[15];
	u32 cpri_map_config;	/* 0xC0 - Mapping Configuration */
	u32 reserved7[1];
	u32 cpri_map_tbl_config;	/* 0xC8 - Mapping Table Config */
	u32 reserved8[6];
	u32 cpri_map_offset_rx;	/* 0xE4 - RX AxC Container Map Block Off */
	u32 cpri_map_offset_tx;	/* 0xE8 - TX AxC Container Map Block Off */
	u32 reserved9;
	u32 cpri_tstartoffset;	/* 0xF0 - Off for TX Start Sync Output */
	u32 reserved10[3];
	u32 cpri_iq_rx_buf_status[2]; /* 0x100 to 0x104 - Map Buf RX Status */
	u32 reserved11[6];
	u32 cpri_iq_tx_buf_status[2]; /* 0x120 tp 0x124 - Map Buf TX Status */
	u32 reserved12[38];
	u32 cpri_map_smpl_cfg_rx;	/* 0x1C0 - CPRI Rx AxC SW Config */
	u32 cpri_map_smpl_cfg_tx;	/* 0x1C4 - CPRI Tx AxC SW Config */
	u32 reserved13[2];
	u32 cpri_map_k_select_rx1;	/* 0x1D0 - CPRI Rx K Selection Reg1 */
	u32 cpri_map_k_select_rx2;	/* 0x1D4 - CPRI Rx K Selection Reg2 */
	u32 reserved14[2];
	u32 cpri_map_k_select_tx1;	/* 0x1E0 - CPRI Tx K Selection Reg1 */
	u32 cpri_map_k_select_tx2;	/* 0x1E4 - CPRI Tx K Selection Reg2 */
	u32 reserved15[6];
	u32 cpri_rethstatus;	/* 0x200 - Eth Receive Status */
	u32 reserved16;
	u32 cpri_ethcfg1;	/* 0x208 - Eth Feat En/Dis and Trig En Bits */
	u32 cpri_ethcfg2;	/* 0x20C - Ethernet Miscellaneous Config */
	u32 cpri_rethctrl;	/* 0x210 - Ethernet RX Packet Discard */
	u32 reserved17[5];
	u32 cpri_rethexstatus;	/* 0x228 - Ethernet RX External Status */
	u32 cpri_ethaddrmsb;	/* 0x22C - Ethernet 16 MSB of MAC Address */
	u32 cpri_ethaddrlsb;	/* 0x230 - Ethernet 32 LSB of MAC Address */
	u32 cpri_ethhashtbl;	/* 0x234 - Eth Hash Tbl to Filt Mult Traf */
	u32 reserved18[3];
	u32 cpri_ethcfg3;	/* 0x244 - Ethernet Configuration 3 */
	u32 cpri_rethframercnt;	/* 0x248 - Ethernet Receive Frame Counter */
	u32 cpri_tethcntframer;	/* 0x24C - Ethernet Transmit Frame Counter */
	u32 cpri_rethdmamismcnt;/* 0x250 - Eth RX DMAC Mismatch Frame Counter*/
	u32 reserved19[43];
	u32 cpri_rhdlcstatus;	/* 0x300 - HDLC Receive Status */
	u32 reserved20;
	u32 cpri_hdlccfg1;	/* 0x308 - HDLC Diff Feat En/Dis & Trig En */
	u32 cpri_hdlccfg2;	/* 0x30C - HDLC Miscellaneous Configuration */
	u32 cpri_rhdlcctrl;	/* 0x310 - HDLC RX Packet Discard */
	u32 reserved21[5];
	u32 cpri_rhdlcexstatus;	/* 0x328 - HDLC RX External Status */
	u32 reserved22[6];
	u32 cpri_hdlccfg3;	/* 0x344 - HDLC Configuration 3 */
	u32 cpri_rhdlcframecnt;	/* 0x348 - HDLC Receive Frame Counter */
	u32 cpri_thdlcframecnt;	/* 0x34C - HDLC Transmit Frame Counter */
	/* DMA registers */
	u32 reserved23[47];
	u32 cpri_rvssaxisize;	/* 0x40C - Rx VSS AXI Transaction Size */
	u32 cpri_tvssaxisize;	/* 0x410 - Tx VSS AXI Transaction Size */
	u32 reserved24[22];
	u32 cpri_rvssbufsize;	/* 0x46C - Rx VSS Buf Size */
	u32 cpri_tvssbufsize;	/* 0x470 - Tx VSS Buf Size */
	u32 cpri_rethbufsize;	/* 0x474 - Rx Eth Buf Size */
	u32 cpri_rhdlcbufsize;	/* 0x478 - Rx HDLC Buf Size */
	u32 reserved25[6];
	u32 cpri_rethbdringsize; /* 0x494 - Rx Eth BD Ring Size */
	u32 cpri_tethbdringsize; /* 0x498 - Tx Eth BD Ring Size */
	u32 cpri_rhdlcbdringsize; /* 0x49C - Rx HDLC BD Ring Size */
	u32 cpri_thdlcbdringsize; /* 0x4A0 - Tx HDLC BD Ring Size */
	u32 reserved26;
	u32 cpri_rgenmode;	/* 0x4A8 - Rx Gen CPRI Mode */
	u32 cpri_tgenmode;	/* 0x4AC - Tx Gen CPRI Mode */
	u32 reserved27[2];
	u32 cpri_tbufsize;	/* 0x4B8 - Tx CPRI Framer Buf Size */
	u32 cpri_tctrlinserttb1; /* 0x4BC - Tx Ctrl Tbl Insert En 1 */
	u32 cpri_tctrlinserttb2; /* 0x4C0 - Tx Ctrl Tbl Insert En 2 */
	u32 reserved28;
	u32 cpri_timer_cfg;	/* 0x4C8 - Timer Configuration */
	u32 cpri_rframepulsewidth; /* 0x4CC - Rx Frame Pulse Width */
	u32 cpri_tframepulsewidth; /* 0x4D0 - Tx Frame Pulse Width */
	u32 reserved29[15];
	u32 cpri_rvssbaddr;	/* 0x510 - Rx VSS Base Address */
	u32 cpri_rvssbaddrmsb;	/* 0x514 - Rx VSS Base Address MSB */
	u32 reserved30[2];
	u32 cpri_tvssbaddr;	/* 0x520 - Tx VSS Base Address */
	u32 cpri_tvssbaddrmsb;	/* 0x524 - Tx VSS Base Address MSB */
	u32 reserved31[2];
	u32 cpri_rethbdringbaddr; /* 0x530 - Rx Ethernet BD Ring Base Add */
	u32 cpri_rethbdringbaddrmsb; /* 0x534 - Rx Eth BD Ring baddr MSB */
	u32 reserved32[2];
	u32 cpri_tethbdringbaddr; /* 0x540 - Tx Eth BD Ring baddr */
	u32 cpri_tethbdringbaddrmsb; /* 0x544 - ,, MSB */
	u32 reserved33[2];
	u32 cpri_rhdlcbdringbaddr; /* 0x550 - Rx HDLC BD Ring Base Addr */
	u32 cpri_rhdlcbdringbaddrmsb; /* 0x554 - ,, MSB */
	u32 reserved34[2];
	u32 cpri_thdlcbdringbaddr; /* 0x560 - Tx HDLC BD Ring Base Addr */
	u32 cpri_thdlcbdringbaddrmsb; /* 0x564 - ,, MSB */
	u32 reserved35[34];
	u32 cpri_auxctrl;	/* 0x5F0 - CPRI Aux Control */
	u32 reserved36[3];
	/* Control registers */
	u32 cpri_rcr;		/* 0x600 - Rx Control */
	u32 cpri_tcr;		/* 0x604 - Tx Control */
	u32 cpri_raccr;	/* 0x608 - Rx AxC Control */
	u32 reserved37;
	u32 cpri_taccr;	/* 0x610 - Tx AxC Control */
	u32 reserved38[14];
	u32 cpri_rvssthresh;	/* 0x64C - Tx VSS Threshold */
	u32 cpri_tvssthresh;	/* 0x650 Tx Eth Coal Threshold */
	u32 cpri_rethcoalthresh; /* 0x654 - Rx Ethx Coalx Threshold */
	u32 cpri_tethcoalthresh; /* 0x658 - - Rx VSS Threshold */
	u32 reserved39;
	u32 cpri_rctrltiminginten; /* 0x660 - CPRI Rx Ctrl & Timing Int En */
	u32 cpri_tctrltiminginten; /* 0x664 - CPRI Tx Ctrl & Timing Int En */
	u32 reserved40;
	u32 cpri_eccerrindicateen; /* 0x66C - CPRI ECC Error Indication En */
	u32 cpri_errinten;	/* 0x670 - CPRI Error Interrupt En */
	u32 cpri_timeren;	/* 0x674 - Timer Enable */
	u32 cpri_rethwriteptr;	/* 0x678 - Rx Ethernet Write Ptr Ring */
	u32 cpri_tethwriteptr;	/* 0x67C - Tx Ethernet Write Ptr Ring */
	u32 cpri_rhdlcwriteptr;	/* 0x680 - Rx HDLC Write Ptr Ring */
	u32 cpri_thdlcwriteptr;	/* 0x684 - Tx HDLC Write Ptr Ring */
	u32 reserved41[6];
	u32 cpri_rctrlattrib;	/* 0x6A0 - Rx Ctrl Attribute Register */
	u32 cpri_rctrldata0;	/* 0x6A4 - Rx Ctrl Data register 0 */
	u32 cpri_rctrldata1;	/* 0x6A8 - Rx Ctrl Data register 1*/
	u32 cpri_rctrldata2;	/* 0x6AC - Rx Ctrl Data Register 2 */
	u32 cpri_rctrldata3;	/* 0x6B0 - Rx Ctrl Data Register 3 */
	u32 cpri_tctrlattrib;	/* 0x6B4 - Tx Ctrl Attribute Register */
	u32 cpri_tctrldata0;	/* 0x6B8 - Tx Ctrl Data Register 0 */
	u32 cpri_tctrldata1;	/* 0x6BC - Tx Ctrl Data register 1 */
	u32 cpri_tctrldata2;	/* 0x6C0 - Tx Ctrl Data Register 2 */
	u32 cpri_tctrldata3;	/* 0x6C4 - Tx Ctrl Data Register 3 */
	u32 reserved42[14];
	u32 cpri_racpr[24];	/* 0x700 to 0x75C - Rx Ant Carrier Param */
	u32 reserved43[8];
	u32 cpri_racprmsb[24];	/* 0x780 to 0x7DC  - ,, MSB */
	u32 reserved44[72];
	u32 cpri_tacpr[24];	/* 0x900 to 0x95C - Tx Ant Carrier Param */
	u32 reserved45[8];
	u32 cpri_tacprmsb[24]; /* 0x980 to 0x9DC - ,, Param MSB */
	u32 reserved46[72];
	u32 cpri_auxmask[64];	/* 0xB00 to 0xBFC - Aux Interface Mask */
	u32 cpri_tcmd0;	/* 0xC00 - Tx Config Memory Address */
	u32 cpri_tcmd1;	/* 0xC04 - Tx Config Memory Address */
	u32 cpri_rcmd0;	/* 0xC08 - Rx Config Memory Data */
	u32 cpri_rcmd1;	/* 0xC0C - Rx Config Memory Data */
	u32 cpri_raxierrstatus;	/* 0xC10 - Rx AXI Error Status */
	u32 cpri_taxierrstatus;	/* 0xC14 - Tx AXI Error Status */
	u32 cpri_raxierrinten;	/* 0xC18 - Rx AXI Error Int En */
	u32 cpri_taxierrinten;	/* 0xC1C - Tx AXI Error Int En */
	u32 cpri_raxciqthreshintstatus;	/* 0xC20 - Rx AxC IQ Thr Int Status */
	u32 cpri_taxciqthreshintstatus;	/* 0xC24 - Tx AxC IQ Thr Int Status */
	u32 cpri_raxciqthreshinten; /* 0xC28 - Rx AxC IQ Threshold Int En */
	u32 cpri_taxciqthreshinten; /* 0xC2C - Tx AxC IQ Threshold Int En */
	u32 cpri_tcma;	/* 0xC30 - Tx Config Memory Address */
	u32 cpri_rcma;	/* 0xC34 - Rx Config Memory Address */
	u32 cpri_ethfwdctrl;	/* 0xc38 - CPRI Eth Forward Inter Ctrl */
	u32 reserved48;
	u32 cpri_auxcwdmask[8];	/* 0xC40-0xC5C - CPRI Aux Interface CWM */
	u32 cpri_auxcwdmasken;	/* 0xC60 - Aux Interface CWM En */
	u32 cpri_auxcwd130mask;	/* 0xC64 - Aux Interface CW 130 Mask*/
	u32 cpri_framediffctrl;	/* 0xC68 - Frame Diff Ctrl */
	u32 cpri_cwddelay;	/* 0xC6C - Control Word Delay */
	u32 cpri_framediffstatus; /* 0xC70 - Frame Diff Status */
	u32 cpri_framediffdriftstatus; /* 0xC74 - Frame Diff Drift Status */
	/* Status registers */
	u32 reserved49[38];
	u32 cpri_rvssbufdispl;	/* 0xD10 - Rx VSS Bufx Displacement */
	u32 cpri_tvssbufdispl;	/* 0xD14 - Tx VSS Bufxr Displacement */
	u32 cpri_rethbufdesc[2]; /* 0xD18 and 0xD1C - Rx Ethx Buf Descriptor*/
	u32 cpri_tethbufdesc[2]; /* 0xD20 and 0xD24 - Tx Ethx Buf Descriptor */
	u32 cpri_rethrdptrring;	/* 0xD28 - Rx Ethernet Read Ptr Ring */
	u32 cpri_tethrdptrring;	/* 0xD2C - Tx Eth Read Ptr Ring */
	u32 cpri_rhdlcbufdesc[2]; /* 0xD30 &  0xD34 - Rx HDLC Buf Descriptor */
	u32 cpri_thdlcbufdesc[2]; /* 0xD38 & 0xD3C - Tx HDLC Buf Descriptor */
	u32 cpri_rhdlcrdptrring; /* 0xD40 - Rx HDLC Read Pointer Ring */
	u32 cpri_thdlcrdptrring; /* 0xD44 - Txt HDLC Read Pointer Ring */
	u32 cpri_revent;	/* 0xD48 - Rx Event Register */
	u32 cpri_tevent;	/* 0xD4C - Tx Event Register */
	u32 cpri_errevent;	/* 0xD50 - Errx Event Register */
	u32 cpri_rethcoalstatus; /* 0xD54 - Rx Ethernet Coalescing Status */
	u32 cpri_tethcoalstatus; /* 0xD58 - Tx Ethernet Coalescing Status */
	u32 cpri_timerstatus;	/* 0xD5C - TIMERn Status */
	u32 cpri_rstatus;	/* 0xD60 - Rx Status */
	u32 cpri_tstatus;	/* 0xD64 - Tx Status  */
	u32 cpri_multibiteccerrstatus; /* 0xD68 - Multi Bit ECC Error Status */
	u32 reserved51;
	u32 cpri_thfnctr;	/* 0xD70 - CPRI Tx HFN Counter */
	u32 cpri_tbfnctr;	/* 0xD74 - CPRI Tx BFN Counter */
	u32 reserved52[98];
	u32 cpri_raxciqvspthreshinten;	/* 0xF00 - Rx AxC IQ VSP Thr Int En */
	u32 cpri_taxciqvspthreshinten; /* 0xF04 - Tx AxC IQ VSP Thr Int En */
	u32 reserved53[6];
	u32 cpri_raxcuruninten;	/* 0xF20 - Rx Ant Carr Underrun Int En */
	u32 cpri_raxcurunintstatus; /* 0xF24 - Rx Ant Carr Urun Int Status */
	u32 cpri_taxcoruninten;	/* 0xF28 - Tx Ant Carr Orun Int En */
	u32 cpri_taxcorunintstatus; /* 0xF2C - Tx Ant Carr Orun Int Status */
};


struct cpri_framer {
	/* Overall data structures per framer */
	unsigned int id;
	struct device_node *cpri_node;
	struct cpri_framer_regs __iomem *regs;
	struct cpri_dev *cpri_dev;
	struct cdev cdev;
	dev_t dev_t;
	spinlock_t regs_lock;
	spinlock_t err_en_lock;
	spinlock_t rx_cw_lock;
	spinlock_t tx_cw_lock;
	struct semaphore axc_sem;
	struct device_node *sfp_dev_node;
	struct sfp_dev *sfp_dev;
	struct of_phandle_args serdesspec;
	void *serdes_handle;
	/* ISR events and bottom half */
	unsigned int irq_rx_t;
	unsigned int irq_tx_t;
	unsigned int irq_rx_c;
	unsigned int irq_tx_c;
	/* Timer events for autoneg */
	unsigned long timer_expiry_events;
	/* Configuration and stats */
	unsigned long cpri_state;
	u64 cpri_enabled_monitor;
	/* Each element corresponds to one error */
	atomic_t err_cnt[CPRI_ERR_CNT];
	/* Work struct the monitor the sfp */
	struct work_struct sfp_wq;
	/* Autoneg data structures per framer*/
	struct cpri_autoneg_params autoneg_params;
	struct cpri_autoneg_output autoneg_output;
	/* AxC data structures per framer */
	unsigned int max_axcs;
	u32 axc_memblk_size;
	/* Ethernet data structures per framer */
	struct cpri_eth_priv *eth_priv;
	/* The timer to poll the link status */
	struct timer_list link_monitor_timer;
	/* misc */
	unsigned char frmr_ethflag;
};

enum cpri_linerate {
	CPRI_LINE_RATE_1 = 1,
	CPRI_LINE_RATE_2,
	CPRI_LINE_RATE_3,
	CPRI_LINE_RATE_4,
	CPRI_LINE_RATE_5,
	CPRI_LINE_RATE_6,
	CPRI_LINE_RATE_7
};

/* This is to simulate the HW mapping table */
struct axc_cmd_regs {
	u32 tcmd0[3072];
	u32 tcmd1[3072];
	u32 rcmd0[3072];
	u32 rcmd1[3072];
};

#define CLASS_NAME	"cp"
#define DEV_NAME	"cp"

#define RATE_TIMEREXP_BITPOS		0
#define PROTVER_TIMEREXP_BITPOS		1
#define ETHPTR_TIMEREXP_BITPOS		2

#define IEVENT_SFP_TXFAULT			(1 << 1)
#define IEVENT_SFP_LOS				(1 << 2)
#define IEVENT_SFP_PRS				(1 << 3)

#define MASK_ALL				0xFFFFFFFF
#define BITS_PER_U32				32
#define REG_SIZE				0x20
#define MAX_FRAMERS_PER_COMPLEX			2
#define FRAMER_1_ID				0x1
#define FRAMER_2_ID				0x2
#define CLK_ENABLE				0x01
#define BF_W_SIZE_16				16
#define BF_W_SIZE_32				32
#define BF_W_SIZE_40				40
#define BF_W_SIZE_64				64
#define BF_W_SIZE_80				80
#define BF_W_SIZE_128				128
#define BF_IQ_BITS_240				240
#define BF_IQ_BITS_480				480
#define BF_IQ_BITS_600				600
#define BF_IQ_BITS_960				960
#define BF_IQ_BITS_1200				1200
#define BF_IQ_BITS_1920				1920

/* ------------- Register masks - Start ------------- */
/* CPRI Status Register (CPRIn_STATUS) */
#define RX_LOS_MASK				0x001
#define RX_STATE_MASK				0x002
#define RX_HFN_STATE_MASK			0x004
#define RX_BFN_STATE_MASK			0x008
#define RX_LOS_HOLD_MASK			0x100
#define RX_STATE_HOLD_MASK			0x200
#define RX_FREQ_ALARM_HOLD_MASK			0x400
#define RX_RFP_HOLD_MASK			0x800

/* CPRI Configuration (CPRIn_CONFIG) */
#define CONF_SET_10_ACKS_MASK			0x4000
#define CONF_CNT_6_RESET_MASK			0x0800
#define CONF_SYNC_PULSE_MODE_MASK		0x0400
#define CONF_RX_EN_MASK				0x0200
#define CONF_TX_EN_MASK				0x0020
#define SLAVE_MODE_MASK				0x0002
#define TX_CW_INSERT_EN_MASK			0x0001

/* CPRI Receive Line Coding Violation Counter (CPRIn_LCV) */
#define CNT_LCV_MASK				0xFF

/* CPRI Recovered BFN Counter (CPRIn_BFN) */
#define RECOVERED_BFN_CNT_MASK			0xFFF

/* CPRI Recovered HFN Counter (CPRIn_HFN) */
#define RECOVERED_HFN_CNT_MASK			0xFF

/* CPRI Hardware Reset from Control Word (CPRIn_HW_RESET) */
#define RESET_GEN_DONE_HOLD_MASK		0x80
#define RESET_GEN_DONE_MASK			0x40
#define RESET_DETECT_HOLD_MASK			0x20
#define RESET_DETECT_MASK			0x10
#define RESET_OUT_EN_MASK			0x04
#define RESET_GEN_FORCE_EN_MASK			0x02
#define RESET_GEN_EN_MASK			0x01

/* CPRI Control and Management Configuration (CPRIn_CM_CONFIG) */
#define TX_FAST_CM_PTR_MASK			0x3F

/* CPRI Control and Management Status (CPRIn_CM_STATUS) */
#define RX_FAST_CM_PTR_VALID_MASK		0x40
#define RX_FAST_CM_PTR_MASK			0x3F

/* CPRI Receive Delay (CPRIn_RX_DELAY) */
#define RX_ALIGN_DELAY_MASK			0x7F0000
#define RX_BUF_DELAY_MASK			0x3FC
#define RX_BYTE_DELAY_MASK			0x3

/* cw dealy enable mask */
#define CW_DELAY_EN				0x1

/* CPRI Round Trip Delay (CPRIn_ROUND_DELAY) */
#define RX_ROUND_TRIP_DELAY_MASK		0xFFFFF

/* CPRIn_EX_DELAY_CONFIG */
#define TX_EX_DELAY_MASK			0x3F0000
#define RX_EX_DELAY_PERIOD_MASK			0x1FF

/* CPRIn_EX_DELAY_STATUS */
#define RX_EX_BUF_DELAY_VALID_MASK		0x80000000
#define RX_EX_BUF_DELAY_MASK			0x7FFF

/* CPRI Transmit Protocol Version (CPRIn_TX_PROT_VER) */
#define PROTO_VER_MASK				0xFF

/* CPRIn_TX_SCR_SEED & CPRIn_RX_SCR_SEED */
#define SCR_SEED_MASK				0x7FFFFFFF
#define RX_SCR_EN_MASK				0x80000000

/* CPRI Transmit Control (CPRIn_TX_CONTROL) */
#define TX_RESET_BFN_MASK			0x1

/* CPRI Mapping Configuration (CPRIn_MAP_CONFIG) */
#define TX_TRANSPARENT_MODE_MASK		0x80
#define RX_TRANSPARENT_MODE_MASK		0x40
#define MAP_MODE_MASK				0x03

/* CPRIn_MAP_OFFSET_TX, CPRIn_MAP_OFFSET_RX & CPRIn_START_OFFSET_TX */
#define MAP_OFFSET_X_MASK			0x00FF
#define MAP_OFFSET_Z_MASK			0xFF00

/* CPRInRVSSMTS & CPRInTVSSMTS */
#define AXI_TRANSAC_SIZE_MASK			0xF

/* Transmit CPRI Framer Buffer Size(CPRInTCFBS) */
#define FR_BUF_SIZE_MASK			0x7F

/* CPRI Auxiliary Control Register (CPRInAUXCR) */
#define AUX_MODE_MASK				0x1

/* Rx & Tx Control Register (CPRInRCR/CPRInTCR)) */
#define IQ_EN_MASK				0x0001
#define ETH_EN_MASK				0x0002
#define HDLC_EN_MASK				0x0004
#define VSS_EN_MASK				0x0001
#define IQ_SYNC_EN_MASK				0x8000

/* CPRInRCIER & CPRInTCIER */
#define CPRI_INT_COUNT				5
#define TIMING_INT_LEVEL_MASK			0x80000000
#define BFN_TIMING_EVENT_EN_MASK		0x00020000
#define HFN_TIMING_EVENT_EN_MASK		0x00010000
#define CONTROL_INT_LEVEL_MASK			0x00008000
#define ETH_EVENT_EN_MASK			0x00000004
#define HDLC_EVENT_EN_MASK			0x00000002
#define VSS_EVENT_EN_MASK			0x00000001
#define CPRI_INT_MASK				0xf00100f5
#define FRAMER_TX_RX_INT_MASK			0x80038007

/* CPRI ECC Error Indication Enable Register (CPRInECCEIER) */
#define SINGLE_BIT_ECC_ERROR_OUTPUT_EN_MASK	0x1
#define MULTI_BIT_ECC_ERROR_OUTPUT_EN_MASK	0x2

/* CPRInCWMASKEN */
#define CW_EN_MASK				0x1
#define CW130_EN_MASK				0x2

/* Mask in control word 2, 194 and 130 in CPRI spec */
#define CW2_MASK	0x3
#define CW194_MASK	0x3F
#define CW130_RST	0x1
#define CW130_RAI	0x2
#define CW130_SDI	0x4
#define CW130_LOS	0x8
#define CW130_LOF	0x10

/* CPRInRER & CPRInTER */
#define IEVENT_BFN_MASK				0x80
#define IEVENT_HFN_MASK				0x40
#define IEVENT_ETH_MASK				0x20
#define IEVENT_HDLC_MASK			0x10
#define IEVENT_VSS_THRESHOLD_MASK		0x08
#define IEVENT_IQ_THRESHOLD_MASK		0x01

/* CPRInRSR & CPRInTSR */
#define VSS_EVENT_EN_STATUS_MASK		0x8
#define HDLC_EVENT_EN_STATUS_MASK		0x4
#define ETH_EVENT_EN_STATUS_MASK		0x2
#define IQ_EVENT_EN_STATUS_MASK			0x1

/* Multi Bit ECC Error Status Register (CPRInMBEESR) */
#define ECC_ERR_ALL_MASK			0x3FFFFC
#define ECC_ERR_CW_DEL_MEM_MASK			0x200000
#define ECC_ERR_ETH_FWD_MEM_MASK		0x100000
#define ECC_ERR_TX_IQ_MEM1_MASK			0x080000
#define ECC_ERR_TX_IQ_MEM0_MASK			0x040000
#define ECC_ERR_RX_IQ_MEM1_MASK			0x020000
#define ECC_ERR_RX_IQ_MEM0_MASK			0x010000
#define ECC_ERR_RX_CFG_MEM_MASK			0x008000
#define ECC_ERR_TX_CFG_MEM_MASK			0x004000
#define ECC_ERR_FR_RX_IQ_MEM_MASK		0x002000
#define ECC_ERR_FR_TX_IQ_MEM_MASK		0x001000
#define ECC_ERR_RX_DMA_MEM_MASK			0x000800
#define ECC_ERR_TX_DMA_MEM_MASK			0x000400
#define ECC_ERR_RX_CWD_MEM_MASK			0x000200
#define ECC_ERR_TX_CWD_MEM_MASK			0x000100
#define ECC_ERR_FR_RX_ETH_MEM_MASK		0x000080
#define ECC_ERR_FR_TX_ETH_MEM_MASK		0x000040
#define ECC_ERR_FR_RX_HLC_MEM_MASK		0x000020
#define ECC_ERR_FR_TX_HDLC_MEM_MASK		0x000010
#define ECC_ERR_AUX_MEM_MASK			0x000008
#define ECC_ERR_FR_ELASTIC_MEM_MASK		0x000004

/* CPRI Transmit HFN Counter (CPRInTXHFN) */
#define TX_HFN_COUNTER_MASK			0xFF

/* CPRI Transmit BFN Counter (CPRInTXBFN) */
#define TX_BFN_COUNTER_MASK			0xFF

/* CPRI Control Clocks Registers (CPRICCR) */
#define C1_CLK_MASK				0x1
#define C2_CLK_MASK				0x2

/* CPRI Interrupt Control Register y (CPRIICR<y>) */
#define EVENT_REG_SELECT_MASK			0xF0000000
#define LEVEL_MASK				0x00010000
#define IEVENT_BFN_FR_TIMING_EN_MASK		0x00000080
#define IEVENT_HFN_FR_TIMING_EN_MASK		0x00000040
#define IEVENT_ETH_EN_MASK			0x00000020
#define IEVENT_HDLC_EN_MASK			0x00000010
#define IEVENT_VSS_EN_MASK			0x00000008
#define IEVENT_IQ_THRESHOLD_EN_MASK		0x00000001

/* General Receive Synchronization Register (CPRIGRSR) */
#define GEN_IQ_SYNC_MASK			0x1

/* CPRI Error Status Register (CPRIESR) */
#define C1_ERR_MASK				0x1
#define C2_ERR_MASK				0x2

/* CPRI Remote Reset Output Control Register (CPRIRROCR) */
#define C1_REM_RES_OP_EN_MASK			0x001
#define C2_REM_RES_OP_EN_MASK			0x002
#define C1_REM_RES_ACK_OP_EN_MASK		0x100
#define C2_REM_RES_ACK_OP_EN_MASK		0x200

/* Transmit Control Attribute Register (CPRInTCA)*/
#define TCT_ADDR_MASK				0x3FC00
#define TCT_WRITE_MASK				0x40000
#define MAX_TCTA_ADDR				255
#define TCTA_ADDR_OFFSET			10
/* ----------- Register masks - End ---------------- */

/* Control word constants */
#define CHAN_MAX				64
#define WDOFF_B0				0
#define WDOFF_B2				2
#define MAX_CWBYTES				16
#define NUM_IDX					4
#define WORD_IDX0				0
#define WORD_IDX1				1
#define WORD_IDX2				2
#define WORD_IDX3				3
#define CWIDX_ETHPTR				3
#define CWIDX_PROTVER				0
#define FIND_CW_NUM(chan_no, cw_idx)		(chan_no + CHAN_MAX * cw_idx)
#define SIZE_BYTE				8
#define BYTE_MASK				0xFF
#define SIZE_REGBYTES				4
#define CW_BYTE0				24
#define CW_BYTE1				16
#define CW_BYTE2				8
#define CW_BYTE3				0


#define CPRI_ETH_NOT_SUPPORTED		0
#define CPRI_ETH_SUPPORTED			(1 << 0)
#define CPRI_ETH_AUTONEG_REQ		(1 << 1)

#define CLEAR_LINE_RATE	0
#define SET_LINE_RATE	1

/* CPRI daisy chaining defines */
#define CPRI_SYNC_ESA_MASK			0x30000000
#define CPRI_PAIRED_SYNC			0x2
#define CPRI_SELF_SYNC				0x3
#define CPRI_TMR_EN				0x80000000

/* CPRI n frame difference calculation logic */
#define CPRI_FRAME_DIFF_STATUS_BIT		0x1
#define NFRAME_DIFF_COUNT_LOOP			100

#define SERDES_PLL_LOCK_RETRY_CNT		500

static inline u32 cpri_read(const void __iomem *addr)
{
	u32 val;
	val = readl(addr);
	return val;
}

static inline void cpri_write(u32 val, void __iomem *addr)
{
	writel(val, addr);
}

static inline void cpri_reg_clear(void __iomem *addr,
				u32 mask)
{
	u32 val;

	val = cpri_read(addr);
	val &= ~mask;
	cpri_write(val, addr);
}

static inline void cpri_reg_set(void __iomem *addr,
				u32 mask)
{
	u32 val;

	val = cpri_read(addr);
	val |= mask;
	cpri_write(val, addr);
}

static inline void cpri_reg_write(spinlock_t *lock,
		void __iomem *addr,  u32 mask, u32 val)
{
	u32 tmp = 0;
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	tmp = cpri_read(addr);
	tmp |= mask;
	tmp &= (~mask | val);
	cpri_write(tmp, addr);
	spin_unlock_irqrestore(lock, flags);
}

static inline void cpri_reg_set_val(void __iomem *addr,
				u32 mask,
				u32 val)
{
	u32 tmp = 0;
	register u32 cnt = 0;

	cnt = ffs(mask);
	cnt--;

	tmp = cpri_read(addr);
	tmp |= mask;
	tmp &= (~mask  | (val << cnt));
	cpri_write(tmp, addr);
}

static inline u32 cpri_reg_get_val(const void __iomem *addr, u32 mask)
{
	u32 val;
	register u32 cnt = 0;

	cnt = ffs(mask);
	cnt--;

	val = cpri_read(addr);

	return (val & mask)>>cnt;
}

struct cpri_reg_data {
	u32 mask;
	u32 val;
};

static inline void cpri_reg_vset_val(void __iomem *addr,
		struct cpri_reg_data *data)
{
	u32 tmp = 0;

	tmp = cpri_read(addr);
	tmp |= data->mask;
	tmp &= (~(data->mask) | data->val);

	cpri_write(tmp, addr);
}

/* CPRI base prototypes */
long cpri_ioctl(struct file *fp, unsigned int cmd,
			unsigned long arg);

/* Autoneg prototypes */
int cpri_autoneg_ioctl(struct cpri_framer *framer, unsigned int cmd,
			unsigned long arg);
/* Control word function prototypes */
void clear_control_tx_table(struct cpri_framer *framer);
void clear_axc_map_tx_rx_table(struct cpri_framer *framer);

/* Ethernet exported functions */
extern int cpri_eth_init(struct platform_device *ofdev,
			struct cpri_framer *framer,
			struct device_node *frnode);
extern void cpri_eth_deinit(struct platform_device *ofdev,
		struct cpri_framer *framer);
extern int cpri_eth_handle_rx(struct cpri_framer *framer);
extern int cpri_eth_handle_tx(struct cpri_framer *framer);
extern int cpri_eth_handle_error(struct cpri_framer *framer);

/* SFP exported functions */
extern struct cpri_framer
	*get_attached_cpri_dev(struct device_node **sfp_dev_node);
extern struct sfp_dev *get_attached_sfp_dev(struct device_node *sfp_dev_node);
void fill_sfp_detail(struct sfp_dev *sfp_dev, u8 *buf);
extern void set_sfp_txdisable(struct sfp_dev *sfp, unsigned value);
extern int sfp_raw_write(struct sfp_dev *sfp, u8 *buf, u8 offset,
		unsigned int count, enum mem_type type);
int sfp_raw_read(struct sfp_dev *sfp, u8 *buf, u8 offset,
		unsigned int count, enum mem_type type);
extern void d4400_rev_clk_select(u8 cpri_id, u8 clk_dev);

/* AxC mapping functions */
int cpri_axc_ioctl(struct cpri_framer *framer, unsigned long arg,
		unsigned int cmd);
void cpri_mask_irq_events(struct cpri_framer *framer);
void init_eth(struct cpri_framer *framer);
struct cpri_dev *get_pair_cpri_dev(struct cpri_dev *cpri_dev);
signed int set_sfp_input_amp_limit(struct cpri_framer *framer,
		u32 max_volt, u8 flag);
void read_rx_cw(struct cpri_framer *framer, int bf_index, u8 *buf);
void rdwr_tx_cw(struct cpri_framer *framer,
			int bf_index, int operation, u8 *buf);
void cpri_set_monitor(struct cpri_framer *framer,
		const struct monitor_config_en *monitor_cfg_en);
void cpri_clear_monitor(struct cpri_framer *framer,
		const struct monitor_config_disable *monitor_cfg_dis);
void src_cpri_hwrst(int enable);
void cpri_config_hwrst(struct cpri_framer *framer, int enable);
#endif /* __CPRI_H */

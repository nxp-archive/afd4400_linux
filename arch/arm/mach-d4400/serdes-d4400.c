/*
 * SerDes device driver for D4400
 *
 * Author: Anil Jaiswal b44339@freescale.com
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/io.h>

#include <mach/serdes-d4400.h>
#include <mach/serdes-priv.h>

#define GET_FRAT_MULT(f, b)	((b == (f >> 2)) ? FRAT_QUARTER :\
				((b == (f >> 1)) ? FRAT_HALF    :\
				((b == (f)       ? FRAT_FULL    :\
				((b == (f << 1)) ? FRAT_DOUBLE  :\
				 FRAT_INVAL)))))

#define F(frate)  ((frate & 0x06) >> 1)

static LIST_HEAD(serdes_dev_list);
raw_spinlock_t serdes_list_lock;

static void serdes_disable_all_lanes(enum srds_pll_id pll_id,
		struct serdes_regs *base_reg);
/**@brief inline for set, clear and test bits for 32 bit */
static inline void
sclear_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) &= ~(1 << (nr & 31));
}

static inline void
sset_bit(int nr, void *addr)
{
	*((__u32 *) addr + (nr >> 5)) |= (1 << (nr & 31));
}

static inline int
srds_test_bit(int nr, const void *addr)
{
	return 1 & (((const __u32 *) addr)[nr >> 5] >> (nr & 31));
}

void srds_set_bit(int nr, void *addr)
{
	u32 reg_val = 0;
	reg_val = ioread32(addr);
	sset_bit(nr, &reg_val);
	iowrite32(reg_val, addr);
}

void srds_clear_bit(int nr, void *addr)
{
	u32 reg_val = 0;
	reg_val = ioread32(addr);
	sclear_bit(nr, &reg_val);
	iowrite32(reg_val, addr);
}

void srds_update_reg(u32 *reg, u32 val, u32 mask)
{
	u32 reg_val;

	reg_val = ioread32(reg);
	reg_val &= ~mask;
	reg_val |= (val & mask);
	iowrite32(reg_val, reg);
}

void srds_write_reg(u32 *reg, u32 val)
{
	iowrite32(val, reg);
}

inline u32 srds_read_reg(u32 *reg)
{
	return ioread32(reg);
}

static int serdes_set_group_protocol(enum srds_grp_prot_id grp_prot,
		struct serdes_regs *base_reg)
{
	int rc = 0;
	u32 *reg, val = 0, mask = 0;

	switch (grp_prot) {
	case SERDES_PROT_SGMII:
		reg = (u32 *) (&base_reg->pccr1);
		val = SRDS_GRP_PROT_SGMIIA_ENABLE |
			SRDS_GRP_PROT_SGMIIB_ENABLE;
		mask = SRDS_GRP_PROT_SGMIIA_CFG_MASK |
			SRDS_GRP_PROT_SGMIIB_CFG_MASK;
		break;
	case SERDES_PROT_CPRI:
		reg = (u32 *) (&base_reg->pccr3);
		val = SRDS_GRP_PROT_CPRIG4_SINGLE_CHANNEL;
		mask = SRDS_GRP_PROT_CPRIG4_CFG_MASK;
		break;
	case SERDES_PROT_JESD_1_LANE:
		reg = (u32 *) (&base_reg->pccr5);
		val = SRDS_GRP_PROT_JESDG1_SINGLE_CHANNEL;
		mask = SRDS_GRP_PROT_JESDG1_CFG_MASK;
		break;
	case SERDES_PROT_JESD_2_LANE:
		reg = (u32 *) (&base_reg->pccr5);
		val = SRDS_GRP_PROT_JESDG2_DOUBLE_CHANNEL;
		mask = SRDS_GRP_PROT_JESDG2_CFG_MASK;
		break;
	default:
		rc = -EINVAL;
		return rc;
	}
	srds_update_reg(reg, val, mask);

	return rc;
}

/*
 * The ring oscillator (1) should not be used for protocol
 * with data rate greater than or equal to 8 Gbps.
 *
 * 5.00 GHz: quarter speed: 0 or 1
 * 4.9152 GHz full/half/quarter speed: 0 or 1
 * 4.9152 GHz double speed: 0
 * 3.125 GHz full speed: 1
 * 3.072 GHz full/double speed: 1
 */
static u32 serdes_rate_select(u32 vco, u32 frate, u32 brate)
{
	/* hash [vco][mult][frat] */
	const u32 hash[2][4][3] = {
			{{0x02, 0x02, 0xff},
			{0xff, 0x01, 0xff},
			{0xff, 0x00, 0xff},
			{0xff, 0x03, 0xff} },

			{{0x02, 0x02, 0xff},
			{0xff, 0x01, 0xff},
			{0xff, 0x00, 0x00},
			{0xff, 0xff, 0x03} } };

	const u32 FREQ[16] = {[0] = 5000000, [1 ... 2] = 0,
			      [3] = 4915200, [4 ... 11] = 0,
			      [12] = 3072000, [13 ... 15] = 0};
	u32 mult;


	if (FREQ[frate] == 0 || brate == 0)
		return 0xFF;
	mult = GET_FRAT_MULT(FREQ[frate], brate);

	return hash[vco][mult][F(frate)];
}

/* configure lane
 * 1. verify TPLL and RPLL Value
 * 2. if (TPL != RPL) return error
 * 3. for lane 0-1 PLL will always be 1
 * 4. for other lanes if TPL = 1 > PLL1 if TPL = 0 PLL2
 * 5. get VCO type and frequency from PLL registers
 *
 */

static int serdes_set_lane_config(struct serdes_lane_params *lane_param,
		struct serdes_regs *base_reg)
{
	u32 *reg, val = 0, mask = 0;
	u32 vco_type, rrate, cflag, frate = 0;
	u32 pll_id, tpll_id;
	int rc = 0;
	int lane_id = lane_param->lane_id;
	enum bitrate_mult;

	cflag = lane_param->gen_conf.cflag;

	/* TPLL_LES and RPLL_LES must be same */
	if (((cflag & 0x4) >> 2) ^ ((cflag & 0x2) >> 1))
		return -EINVAL;

	/* Get PLL ID according to lane */
	tpll_id = (cflag & 0x2) >> 1;
	if ((!tpll_id) && (lane_id > LANE_B))
		pll_id = SERDES_PLL_2;
	else
		pll_id = SERDES_PLL_1;

	/*Reset and disable lane */
	reg = (u32 *) (&base_reg->lane_csr[lane_id].gcr0);
	val = ~(SRDS_LN_GCR_TRST_B_MASK |
			SRDS_LN_GCR_RRST_B_MASK);
	mask = (SRDS_LN_GCR_TRST_B_MASK |
			SRDS_LN_GCR_RRST_B_MASK);
	srds_update_reg(reg, val, mask);
	udelay(1);

	/* Get VCO freq (frate) */
	reg = (u32 *)(&base_reg->pll_reg[pll_id].pll_ctl_reg0);
	frate = (srds_read_reg(reg) >> 0x10) & SRDS_PLLCR_FRATE_SEL_BITS;

	reg = (u32 *)(&base_reg->pll_reg[pll_id].pll_ctl_reg1);
	vco_type = (srds_read_reg(reg) >> SRDS_PLLCR_VCO_EN_BIT) & 0x1;

	/* get valid rrat / trat bits */
	rrate =  serdes_rate_select(vco_type,
			frate, lane_param->gen_conf.bit_rate_kbps);
	if (0xFF == rrate)
		return -EINVAL;

	reg = (u32 *) (&base_reg->lane_csr[lane_id].gcr0);
	val = ((rrate << SRDS_LN_GCR_RRAT_SEL) |
		(rrate << SRDS_LN_GCR_TRAT_SEL) |
		(tpll_id << SRDS_LN_GCR_TPLL_LES) |
		(tpll_id << SRDS_LN_GCR_RPLL_LES));

	/* 20 bit enable flag */
	val |= (cflag & 0x1) << SRDS_LN_GCR_IF20BIT_EN;
	/* First lane enable flag */
	val |= (cflag & 0x8) << (SRDS_LN_GCR_FIRST_LN - 3);

	/* Set lane protocol
	 * TODO: Currently supporting lane protocol for CPRI and JESD
	 * */
	if ((lane_param->gen_conf.lane_prot != SERDES_LANE_PROTS_CPRI) &&
		(lane_param->gen_conf.lane_prot != SERDES_LANE_PROTS_JESD204))
		return -EINVAL;

	val |= lane_param->gen_conf.lane_prot << SRDS_LN_GCR_PROTS_SEL;
	srds_write_reg(reg, val);

	reg = (u32 *) (&base_reg->lane_csr[lane_id].tcsr3);
	val = (lane_param->gen_conf.cflag & SERDES_LOOPBACK_EN) <<
		(SRDS_LN_TCS_LOOPBACK_BIT - 4);
	mask = SRDS_LOOPBACK_EN;
	srds_update_reg(reg, val, mask);

	wmb();
	/*Enable lane */
	reg = (u32 *) (&base_reg->lane_csr[lane_id].gcr0);
	val = (SRDS_LN_GCR_TRST_B_MASK | SRDS_LN_GCR_RRST_B_MASK);
	mask = (SRDS_LN_GCR_TRST_B_MASK | SRDS_LN_GCR_RRST_B_MASK);
	srds_update_reg(reg, val, mask);
	udelay(1);

	return rc;
}

static int serdes_set_pll_config(struct serdes_dev *sdev,
	struct serdes_pll_params *pll, struct serdes_regs *base_reg)
{
	u32 *reg, val = 0, mask = 0;
	int rc = 0;
	unsigned int timeout;

	/*Disable all lanes*/
	serdes_disable_all_lanes(pll->pll_id, sdev->regs);

	/*power down*/
	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_rstctl_reg);
	val = ~SRDS_PLL_RSTCTL_SDEN_MASK;
	mask = SRDS_PLL_RSTCTL_SDEN_MASK;
	srds_update_reg(reg, val, mask);
	udelay(1);

	/*power up again*/
	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_rstctl_reg);
	val =  SRDS_PLL_RSTCTL_SDEN_MASK;
	mask = SRDS_PLL_RSTCTL_SDEN_MASK;
	srds_update_reg(reg, val, mask);
	udelay(1);

	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_rstctl_reg);
	val = SRDS_PLL_RSTCTL_RSTREQ_MASK;
	mask = SRDS_PLL_RSTCTL_RSTREQ_MASK;
	srds_update_reg(reg, val, mask);

	timeout = jiffies + msecs_to_jiffies(SRDS_PLL_TIMEOUT_MS);

	val = ioread32(reg);
	while (!(val & SRDS_PLL_RSTCTL_RST_DONE_MASK)) {
		if (jiffies > timeout) {
			dev_info(sdev->dev, "Failed to reset pll %d\n",
				pll->pll_id);
			rc = -EBUSY;
			goto out;
		}
		val = ioread32(reg);
	}

	/*Change PLL settings*/
	/* PLL frequency configurations */
	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_ctl_reg0);
	mask = SRDS_PLLCR_FRATE_SEL_MASK;
	switch (pll->frate_sel) {
	case PLL_FREQ_3_072_GHZ:
		val = FREQ_MULTIPLE_3_072_GHZ;
		break;
	case PLL_FREQ_4_9152_GHZ:
		val = FREQ_MULTIPLE_4_912_GHZ;
		break;
	case PLL_FREQ_5_GHZ:
		val = FREQ_MULTIPLE_5_00_GHZ;
		break;
	default:
		rc = -EINVAL;
		return rc;
	}
	srds_update_reg(reg, val, mask);

	/* reference clock setting */
	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_ctl_reg0);
	mask = SRDS_PLLCR_RFCLK_SEL_BITS;
	switch (pll->rfclk_sel) {
	case REF_CLK_FREQ_100_MHZ:
		val = SRDS_PLLCR_RFCLK_100;
		break;
	case REF_CLK_FREQ_122_88_MHZ:
		val = SRDS_PLLCR_RFCLK_122_88;
		break;
	case REF_CLK_FREQ_156_MHZ:
		val = SRDS_PLLCR_RFCLK_156;
		break;
	default:
		rc = -EINVAL;
		return rc;
	}
	srds_update_reg(reg, val, mask);

	/* Select VCO for PLL */
	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_ctl_reg1);
	mask = SRDS_PLLCR_VCO_EN_MASK;
	switch (pll->vco_type) {
	case SERDES_LC_VCO:
		val = SRDS_PLLCR_VCO_LC;
		break;
	case SERDES_RING_VCO:
		val = SRDS_PLLCR_VCO_RING;
		break;
	default:
		rc = -EINVAL;
		return rc;
	}
	srds_update_reg(reg, val, mask);

	/* Reset and Enable */
	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_rstctl_reg);
	val = SRDS_PLL_RSTCTL_RSTREQ_MASK;
	mask = SRDS_PLL_RSTCTL_RSTREQ_MASK;
	srds_update_reg(reg, val, mask);

	timeout = jiffies + msecs_to_jiffies(SRDS_PLL_TIMEOUT_MS);

	val = ioread32(reg);
	while (!(val & SRDS_PLL_RSTCTL_RST_DONE_MASK)) {
		if (jiffies > timeout) {
			dev_info(sdev->dev, "Failed to reset pll %d\n",
				pll->pll_id);
			rc = -EBUSY;
			goto out;
		}
		val = ioread32(reg);
	}

	timeout = jiffies + msecs_to_jiffies(SRDS_PLL_TIMEOUT_MS);

	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_ctl_reg0);

	val = ioread32(reg);
	while (!(val & SRDS_PLLCR_PLL_LCK_MASK)) {
		if (jiffies > timeout) {
			dev_info(sdev->dev, "Failed to lock pll %d\n",
				pll->pll_id);
			rc = -EBUSY;
			goto out;
		}
		val = ioread32(reg);
	}

	wmb();

	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_rstctl_reg);
	val = (SRDS_PLL_RSTCTL_SDEN_MASK | SRDS_PLL_RSTCTL_PLLRST_B_MASK |
		SRDS_PLL_RSTCTL_SRDST_B_MASK);
	mask = val;
	srds_update_reg(reg, val, mask);

	udelay(1);

out:
	return rc;
}

static void serdes_disable_all_lanes(enum srds_pll_id pll_id,
		struct serdes_regs *base_reg)
{
	u32 *reg, val, mask;

	reg = (u32 *)(&base_reg->pll_reg[pll_id].pll_rstctl_reg);

	val = ~SRDS_PLL_RSTCTL_SRDST_B_MASK;
	mask = SRDS_PLL_RSTCTL_SRDST_B_MASK;
	srds_update_reg(reg, val, mask);
	udelay(1);

	return;
}

void serdes_jcpll_enable(void *sdev_handle,
	struct serdes_lane_params *lane_param, struct serdes_pll_params *pll)
{
	u32 *reg, val = 0;
	int lane_id = lane_param->lane_id;
	struct serdes_dev *sdev = (struct serdes_dev *)sdev_handle;
	struct serdes_regs *base_reg = sdev->regs;

	reg = (u32 *) (&base_reg->lane_csr[lane_id].TTL_cr0);
	val = srds_read_reg(reg);

	if (lane_param->gen_conf.bit_rate_kbps == 1228800)
		val |= SRDS_LN_7_TTLCR_FLT_SEL_BITS;
	else
		val &= ~SRDS_LN_7_TTLCR_FLT_SEL_BITS;
	srds_write_reg(reg, val);
	reg = (u32 *) (&base_reg->lane_csr[lane_id].TTL_cr1);
	val = srds_read_reg(reg);
	val &= ~SRDS_LN_TTLCR_DIV_RCVCLK_BIT;
	if (pll->vco_type == SERDES_RING_VCO)
		val |= SRDS_LN_TTLCR_DIV25_RCVCLK_BIT;
	else
		val |= SRDS_LN_TTLCR_DIV40_RCVCLK_BIT;
	srds_write_reg(reg, val);

	reg = (u32 *)(&base_reg->pll_reg[pll->pll_id].pll_ctl_reg4);
	val = srds_read_reg(reg);
	val &= ~(SRDS_PLLCR_RCLKEN_MASK | SRDS_PLLCR_LANE_MASK);
	val |= (SRDS_PLLCR_RCLKEN_MASK | lane_param->lane_id);
	srds_write_reg(reg, val);

}
EXPORT_SYMBOL_GPL(serdes_jcpll_enable);


/* default:PLL on (POFF == 0)
 * for both SRDSxPLL1RSTCTL  & SRDSxPLL2RSTCTL register
 * PLL reset sequence in progress.
 */
int serdes_init_pll(void *sdev_handle,
		struct serdes_pll_params *pll)
{
	int rc = 0;
	struct serdes_dev *sdev = (struct serdes_dev *)sdev_handle;


	if ((!sdev) || !(sdev->regs)) {
		dev_err(sdev->dev, "Invalid dev info\n");
		rc = -EINVAL;
		goto out;
	}

	if ((!pll) || (pll->pll_id >= SERDES_PLL_MAX)) {
		dev_err(sdev->dev, "Invalid PLL Params\n");
		rc = -EINVAL;
		goto out;
	}

#if 0
	/* If Pll is already initialized then return */
	if (sdev->cflag & (1 << pll->pll_id)) {
		dev_info(sdev->dev, "PLL %d already initialized\n",
				pll->pll_id);
		rc = -EALREADY;
		goto out;
	}
#endif

	/* PLL configuration */
	rc = serdes_set_pll_config(sdev, pll, sdev->regs);
	if (rc < 0) {
		dev_err(sdev->dev, "Invalid PLL config\n");
		goto out;
	}


	/* Update PLL initialized state */
	srds_update_reg(&sdev->cflag,
			(1 << pll->pll_id),
			(1 << pll->pll_id));
out:
	return rc;
}
EXPORT_SYMBOL_GPL(serdes_init_pll);

int serdes_init_lane(void *sdev_handle,
		struct serdes_lane_params *lane_param)
{
	int rc = 0;
	struct serdes_dev *sdev = (struct serdes_dev *)sdev_handle;

	if ((!sdev) || !(sdev->regs)) {
		dev_err(sdev->dev, "Invalid dev info\n");
		rc = -EINVAL;
		goto out;
	}
	if ((!lane_param) || (lane_param->lane_id >= LANE_INVAL)) {
		dev_err(sdev->dev, "Invalid lane Params\n");
		rc = -EINVAL;
		goto out;
	}

	rc = serdes_set_group_protocol(lane_param->grp_prot, sdev->regs);
	if (rc < 0) {
		dev_err(sdev->dev, "Invalid greoup protocol ID\n");
		goto out;
	}

	rc = serdes_set_lane_config(lane_param, sdev->regs);
	if (rc < 0) {
		dev_err(sdev->dev, "Invalid Lane config params\n");
		goto out;
	}
out:
	return rc;
}
EXPORT_SYMBOL_GPL(serdes_init_lane);

int serdes_lane_power_up(void *sdev_handle,
		enum srds_lane_id lane_id)
{
	int rc = 0;
	u32 *reg, val, mask;
	struct serdes_dev *sdev = (struct serdes_dev *)sdev_handle;

	if ((!sdev) || !(sdev->regs)) {
		dev_err(sdev->dev, "Invalid dev info\n");
		rc = -EINVAL;
		goto out;
	}

	if (lane_id >= LANE_INVAL) {
		dev_err(sdev->dev, "Invalid lane Params\n");
		rc = -EINVAL;
		goto out;
	}

	/* Power up the Rx portion of the lane */
	reg = (u32 *)sdev->regs->lane_csr[lane_id].gcr0;
	val = ~SRDS_LN_GCR_RX_PD_MASK;
	mask = SRDS_LN_GCR_RX_PD_MASK;
	srds_update_reg(reg, val, mask);

	/* Power up the Tx portion of the lane */
	reg = (u32 *)sdev->regs->lane_csr[lane_id].gcr0;
	val = ~SRDS_LN_GCR_TX_PD_MASK;
	mask = SRDS_LN_GCR_TX_PD_MASK;
	srds_update_reg(reg, val, mask);

out:
	return rc;
}
EXPORT_SYMBOL_GPL(serdes_lane_power_up);

int serdes_lane_power_down(void *sdev_handle,
		enum srds_lane_id lane_id)
{
	int rc = 0;
	u32 *reg, val, mask;
	struct serdes_dev *sdev = (struct serdes_dev *)sdev_handle;

	if ((!sdev) || !(sdev->regs)) {
		dev_err(sdev->dev, "Invalid dev info\n");
		rc = -EINVAL;
		goto out;
	}

	if (lane_id >= LANE_INVAL) {
		dev_err(sdev->dev, "Invalid lane Params\n");
		rc = -EINVAL;
		goto out;
	}

	/* Power down the Rx portion of the lane */
	reg = (u32 *)sdev->regs->lane_csr[lane_id].gcr0;
	val = SRDS_LN_GCR_RX_PD_MASK;
	mask = SRDS_LN_GCR_RX_PD_MASK;
	srds_update_reg(reg, val, mask);

	/* Power down the Tx portion of the lane */
	reg = (u32 *)sdev->regs->lane_csr[lane_id].gcr0;
	val = SRDS_LN_GCR_TX_PD_MASK;
	mask = SRDS_LN_GCR_TX_PD_MASK;
	srds_update_reg(reg, val, mask);

out:
	return rc;
}
EXPORT_SYMBOL_GPL(serdes_lane_power_down);

int serdes_reg_dump(void *sdev_handle, unsigned int offset,
			unsigned int length, u32 *buf)
{
	u32 *start_reg;
	int i;
	struct serdes_dev *sdev = (struct serdes_dev *)sdev_handle;

	if ((!sdev) || (!sdev->regs)) {
		dev_err(sdev->dev, "Invalid dev info\n");
		return -EINVAL;
	}

	start_reg = (u32 *)((u32)(sdev->regs) + offset);
	for (i = 0; i < length; i++) {
		*buf = ioread32(start_reg);
		start_reg++;
		buf++;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(serdes_reg_dump);

int of_get_named_serdes(struct device_node *np,
		struct of_phandle_args *serdesspec, const char *propname,
		int index)
{
	/* Return -EPROBE_DEFER to support probe() functions to be called
	 * later when the GPIO actually becomes available
	 */
	if (of_parse_phandle_with_args(np, propname, "#serdes-cells", index,
					 serdesspec)) {
		return -EINVAL;
	}
	of_node_put(serdesspec->np);
	return 0;
}
EXPORT_SYMBOL_GPL(of_get_named_serdes);

void *get_attached_serdes_dev(struct device_node *serdes_dev_node)
{
	struct serdes_dev *serdes_dev = NULL;

	if (list_empty(&serdes_dev_list))
		return NULL;

	raw_spin_lock(&serdes_list_lock);

	list_for_each_entry(serdes_dev, &serdes_dev_list, list) {
		if (serdes_dev_node == serdes_dev->node)
			break;
	}

	raw_spin_unlock(&serdes_list_lock);

	return serdes_dev;
}
EXPORT_SYMBOL_GPL(get_attached_serdes_dev);

static int serdes_d4400_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct serdes_dev *serdes_dev;
	int rc = 0;

	if (!np || !of_device_is_available(np)) {
		rc = -ENODEV;
		goto err_out;
	}

	/* Allocate serdes device structure */
	serdes_dev = kzalloc(sizeof(struct serdes_dev), GFP_KERNEL);
	if (!serdes_dev) {
		dev_dbg(dev, "Failed to allocate serdes dev\n");
		rc = -ENOMEM;
		goto err_out;
	}

	/* read serdes ID being configured */
	if (of_property_read_u32(np, "serdes-id", &serdes_dev->serdes_id) < 0) {
		dev_dbg(dev, "Failed to get serdes id\n");
		rc = -ENODEV;
		goto err_mem;
	}

	/* read lane info */
	if (of_property_read_u32(np, "max-lane", &serdes_dev->max_lane) < 0) {
		dev_dbg(dev, "Failed to get max lane info\n");
		rc = -ENODEV;
		goto err_mem;
	}

	serdes_dev->regs = of_iomap(np, 0);
	if (!serdes_dev->regs) {
		dev_dbg(dev, "Failed to map serdes reg\n");
		rc = -ENOMEM;
		goto err_mem;
	}

	serdes_dev->dev = dev;
	serdes_dev->node = np;
	/* Initialize cflag with 0 */
	serdes_dev->cflag = 0;

	spin_lock_init(&serdes_dev->lock);

	/* Add the populated serdes_dev to global serdes_dev_list.
	 * This is required since the probe will get called for
	 * multiple serdes devices.
	 */
	raw_spin_lock(&serdes_list_lock);
	list_add_tail(&serdes_dev->list, &serdes_dev_list);
	raw_spin_unlock(&serdes_list_lock);

	dev_info(dev, "%s probe successfull\n", __func__);

	return rc;

err_mem:
	kfree(serdes_dev);
err_out:
	dev_err(dev, "serdes probe failure\n");

	return rc;
}

static int __exit serdes_d4400_remove(struct platform_device *pdev)
{
	struct serdes_dev *sdev = NULL;
	struct list_head *pos, *nx;

	sdev = (struct serdes_dev *)dev_get_drvdata(&pdev->dev);
	if (!sdev)
		return -ENODEV;

	raw_spin_lock(&serdes_list_lock);
	list_for_each_safe(pos, nx, &serdes_dev_list) {
		sdev = list_entry(pos, struct serdes_dev, list);
		list_del(&sdev->list);
		kfree(sdev);
	}
	raw_spin_unlock(&serdes_list_lock);

	kfree(sdev);
	return 0;
}

static struct of_device_id serdes_d4400_dt_ids[] = {
	{ .compatible = "fsl,serdes-d4400", },
	{ /* sentinel */ }
};

static struct platform_driver serdes_d4400_driver = {
	.driver		= {
		.name	= "serdes-d4400",
		.owner	= THIS_MODULE,
		.of_match_table = serdes_d4400_dt_ids,
	},
	.probe		= serdes_d4400_probe,
	.remove		= serdes_d4400_remove,
};

static int __init serdes_d4400_init(void)
{
	return platform_driver_register(&serdes_d4400_driver);
}

static void __exit serdes_d4400_exit(void)
{
	platform_driver_unregister(&serdes_d4400_driver);
}

module_init(serdes_d4400_init);
module_exit(serdes_d4400_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("serdes driver");

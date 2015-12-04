/*
 * drivers/tbgen/tbgen.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/tbgen.h>
#include <mach/mach-d4400.h>

#define TBGEN_PRINT_ERR		(1 << 0)
#define TBGEN_PRINT_INFO	(1 << 1)

#define tbg_err(...)	do {						\
				if (tbg->debug & TBGEN_PRINT_ERR)	\
					pr_err(__VA_ARGS__);		\
			} while (0)

#define tbg_info(...)	do {						\
				if (tbg->debug & TBGEN_PRINT_INFO)	\
					pr_info(__VA_ARGS__);		\
			} while (0)

struct tbgen_dev *tbg;

#define SWRST			(1 << 0)
#define RFG_TIMEOUT_MS		60
#define SYNCOUT_CTRL		(1 << 18)
#define START_INT_RAD_FRM	(1 << 17)
#define OUT_REFSYNC_SEL		(1 << 16)
#define REFSYNCSEL		(1 << 2)
#define RFGEN			(1 << 0)
#define FSIE			(1 << 5)
#define TMREN			(1 << 0)
#define FRMSYNCSEL_RFG		(3 << 12)
#define FRMSYNCSEL_CPRI		(1 << 12)

enum tbg_dev_state tbgen_get_state(void)
{
	return atomic_read(&tbg->state);
}
EXPORT_SYMBOL(tbgen_get_state);

unsigned int tbgen_get_pll_freq(void)
{
	return tbg->config.pll_freq_khz;
}
EXPORT_SYMBOL(tbgen_get_pll_freq);

int tbgen_set_sync_loopback(int lane, int loopback)
{
	struct tbg_regs *tbgregs = tbg->tbgregs;
	u32 val;

	if (lane > 10)
		tbg_err("%s error\n", __func__);

	val = readl(&tbgregs->debug);
	loopback = loopback ? 1 : 0;
	val &= ~(1 << lane);
	val |= (loopback << lane);
	writel(val, &tbgregs->debug);
	return 0;
}
EXPORT_SYMBOL(tbgen_set_sync_loopback);

static int tbgen_rfg_init(void)
{
	struct tbg_regs *tbgregs = tbg->tbgregs;
	u32 val_cr, val_ctrl;
	unsigned long timeout = 0;

	/* reset rfg */
	atomic_set(&tbg->rfg_in_transition, 0);
	writel(0, &tbgregs->rfg_cr);

	val_ctrl = readl(&tbgregs->cntrl_0);
	writel((val_ctrl | SWRST), &tbgregs->cntrl_0);
	while (timeout < RFG_TIMEOUT_MS) {
		if (!(readl(&tbgregs->cntrl_0) & SWRST))
			break;
		schedule_timeout_interruptible(msecs_to_jiffies(1));
		timeout++;
	}
	if (timeout >= RFG_TIMEOUT_MS) {
		tbg_err("tbgen RFG reset time out\n");
		return -ETIME;
	}
	/* Config the 10ms counter */
	if (tbg->config.pll_freq_khz == REF_CLK_614400KHZ ||
		tbg->config.pll_freq_khz == REF_CLK_737280KHZ ||
		tbg->config.pll_freq_khz == REF_CLK_983040KHZ)
		writel(tbg->config.pll_freq_khz * 10,
			&tbgregs->refclk_per_10ms);
	else
		return -EINVAL;

	if (tbg->config.radio_frame_src == SELF_GENERATED_10MS) {
		val_cr = (SYNCOUT_CTRL | START_INT_RAD_FRM);
		writel(FRMSYNCSEL_RFG, &tbgregs->cntrl_0);
	} else {
		val_cr = (REFSYNCSEL | SYNCOUT_CTRL);
		writel(FRMSYNCSEL_CPRI, &tbgregs->cntrl_0);
	}

	if (tbg->config.flag & TBGEN_RF_SYNC_GP_EVENT_7)
		val_cr |= OUT_REFSYNC_SEL;

	val_cr |= (tbg->config.sync_adv & 0x1f) << 19;

	val_ctrl = readl(&tbgregs->cntrl_1);
	writel(val_ctrl | FSIE, &tbgregs->cntrl_1);
	writel(FSIE, &tbgregs->int_stat);

	writel(val_cr | RFGEN, &tbgregs->rfg_cr);

	timeout = wait_for_completion_timeout(&tbg->rfg_complete,
				msecs_to_jiffies(RFG_TIMEOUT_MS));

	writel(val_ctrl, &tbgregs->cntrl_1);
	writel(FSIE, &tbgregs->int_stat);

	if (!timeout) {
		tbg_err("RFG can't get triggered, please check trigger source\n");
		return -ETIME;
	}

	if (tbg->config.rfgerr_target >= 0) {
		if (readl(&tbgregs->rfg_err) == tbg->config.rfgerr_target)
			return 0;
		else
			return -EAGAIN;
	}
	return 0;
}

/* Save this function for later use */
/*
static u64 tbgen_get_master_counter(void)
{
	u64 master_counter;
	u32 tmph1, tmph2, tmpl;

	tmph2 = readl(&tbg->tbgregs->mstr_cnt_hi);
	do {
		tmph1 = tmph2;
		tmpl  = readl(&tbg->tbgregs->mstr_cnt_lo);
		tmph2 = readl(&tbg->tbgregs->mstr_cnt_hi);
	} while (tmph1 != tmph2);

	master_counter = (((u64)tmph1) << 32) | tmpl;

	return master_counter;
}
*/

static u64 tbgen_get_l10_mcntr(void)
{
	u64 ts_10ms_cnt;
	u32 tmph1, tmph2, tmpl;

	tmph2 = readl(&tbg->tbgregs->ts_10_ms_hi);
	do {
		tmph1 = tmph2;
		tmpl  = readl(&tbg->tbgregs->ts_10_ms_lo);
		tmph2 = readl(&tbg->tbgregs->ts_10_ms_hi);
	} while (tmph1 != tmph2);

	ts_10ms_cnt = (((u64)tmph1) << 32) | tmpl;

	return ts_10ms_cnt;
}


static struct tbg_timer *tbg_find_timer(enum tbg_timer_type type, int id)
{
	struct tbg_timer *ptr;

	list_for_each_entry(ptr, &tbg->timer_list, list) {
		if (ptr->type == type && ptr->id == id)
			return ptr;
	}

	return NULL;
}

static int tbg_set_timer(const struct tbg_timer_params *param)
{
	u32 ctrl_reg = 0, tdd_mode_reg = 0;
	int i;
	int ret = 0;
	void *config_buf;
	struct tbg_timer *timer;
	struct tbg_regs *tbgregs = tbg->tbgregs;
	struct tx_alignment_config *tx_alignment_config;
	struct tx_axrf_config *tx_axrf_config;
	struct rx_alignment_config *rx_alignment_config;
	struct srx_alignment_config *srx_alignment_config;
	struct gp_event_config *gp_event_config;
	struct tdd_config *tdd_config;

	/* tdd config takes the largest possible space */
	config_buf = kmalloc(sizeof(struct tdd_config), GFP_KERNEL);
	if (!config_buf)
		return -ENOMEM;

	switch (param->type) {
	case TBG_TX_ALIGNMENT:
		if (copy_from_user(config_buf, param->timer_config,
			sizeof(struct tx_alignment_config))) {
			ret = -EFAULT;
			goto out;
		}
		tx_alignment_config = (struct tx_alignment_config *)config_buf;
		timer = tbg_find_timer(param->type, tx_alignment_config->id);
		if (!timer) {
			ret = -ENODEV;
			goto out;
		}
		timer->offset = tx_alignment_config->offset;
		ctrl_reg =
			(tx_alignment_config->ref_to_frm_clk_ratio & 0x7) << 8 |
			(tx_alignment_config->pol & 0x1) << 7 |
			(tx_alignment_config->serepen & 0x1) << 3 |
			(tx_alignment_config->isyncbyp & 0x1) << 2 |
			(tx_alignment_config->outsel & 0x1) << 1;
		if (!timer->offset) {
			ctrl_reg |= TMREN;
			writel(0, &tbgregs->tx_tmr[tx_alignment_config->id].osethi);
			wmb();
			writel(0, &tbgregs->tx_tmr[tx_alignment_config->id].osetlo);
		}
		writel(ctrl_reg,
			&tbgregs->tx_tmr[tx_alignment_config->id].ctrl);
		break;

	case TBG_TX_AXRF:
		if (copy_from_user(config_buf, param->timer_config,
			sizeof(struct tx_axrf_config))) {
			ret = -EFAULT;
			goto out;
		}
		tx_axrf_config = (struct tx_axrf_config *)config_buf;
		timer = tbg_find_timer(param->type, tx_axrf_config->id);
		if (!timer) {
			ret = -ENODEV;
			goto out;
		}
		timer->offset = tx_axrf_config->offset;
		ctrl_reg = (tx_axrf_config->pol & 0x1) << 7;
		if (!timer->offset) {
			ctrl_reg |= TMREN;
			writel(0, &tbgregs->axrf_tmr[tx_axrf_config->id].osethi);
			wmb();
			writel(0, &tbgregs->axrf_tmr[tx_axrf_config->id].osetlo);
		}
		writel(ctrl_reg,
			&tbgregs->axrf_tmr[tx_axrf_config->id].ctrl);
		break;

	case TBG_RX_ALIGNMENT:
		if (copy_from_user(config_buf, param->timer_config,
			sizeof(struct rx_alignment_config))) {
			ret = -EFAULT;
			goto out;
		}
		rx_alignment_config = (struct rx_alignment_config *)config_buf;
		timer = tbg_find_timer(param->type, rx_alignment_config->id);
		if (!timer) {
			ret = -ENODEV;
			goto out;
		}
		timer->offset = rx_alignment_config->offset;
		ctrl_reg = (rx_alignment_config->pulsewidth & 0xF) << 8 |
				(rx_alignment_config->pol & 0x1) << 7 |
				(rx_alignment_config->oneshot & 0x1) << 6 |
				(rx_alignment_config->strbmode & 0x3) << 4;
		if (!timer->offset) {
			ctrl_reg |= TMREN;
			writel(0, &tbgregs->rx_tmr[rx_alignment_config->id].osethi);
			wmb();
			writel(0, &tbgregs->rx_tmr[rx_alignment_config->id].osetlo);
		}
		writel(ctrl_reg,
			&tbgregs->rx_tmr[rx_alignment_config->id].ctrl);
		writel(rx_alignment_config->interval,
			&tbgregs->rx_tmr[rx_alignment_config->id].interval);

		break;

	case TBG_SRX_ALIGNMENT:
		if (copy_from_user(config_buf, param->timer_config,
			sizeof(struct srx_alignment_config))) {
			ret = -EFAULT;
			goto out;
		}
		srx_alignment_config =
				(struct srx_alignment_config *)config_buf;
		timer = tbg_find_timer(param->type, srx_alignment_config->id);
		if (!timer) {
			ret = -ENODEV;
			goto out;
		}
		timer->offset = srx_alignment_config->offset;
		ctrl_reg = (srx_alignment_config->pulsewidth & 0xF) << 8 |
				(srx_alignment_config->pol & 0x1) << 7 |
				(srx_alignment_config->oneshot & 0x1) << 6 |
				(srx_alignment_config->strbmode & 0x3) << 4;
		if (!timer->offset) {
			ctrl_reg |= TMREN;
			writel(0, &tbgregs->srx_tmr[srx_alignment_config->id].osethi);
			wmb();
			writel(0, &tbgregs->srx_tmr[srx_alignment_config->id].osetlo);
		}
		writel(ctrl_reg,
			&tbgregs->srx_tmr[srx_alignment_config->id].ctrl);

		writel(srx_alignment_config->interval,
			&tbgregs->srx_tmr[srx_alignment_config->id].interval);

		break;

	case TBG_GP_EVENT:
		if (copy_from_user(config_buf, param->timer_config,
			sizeof(struct gp_event_config))) {
			ret = -EFAULT;
			goto out;
		}
		gp_event_config = (struct gp_event_config *)config_buf;
		timer = tbg_find_timer(param->type, gp_event_config->id);
		if (!timer) {
			ret = -ENODEV;
			goto out;
		}
		timer->offset = gp_event_config->offset;
		ctrl_reg = (gp_event_config->pulsewidth & 0xF) << 8 |
				(gp_event_config->pol & 0x1) << 7 |
				(gp_event_config->oneshot & 0x1) << 6 |
				(gp_event_config->strbmode & 0x3) << 4;
		if (!timer->offset) {
			ctrl_reg |= TMREN;
			writel(0, &tbgregs->gp_tmr[gp_event_config->id].osethi);
			wmb();
			writel(0, &tbgregs->gp_tmr[gp_event_config->id].osetlo);
		}
		writel(ctrl_reg,
			&tbgregs->gp_tmr[gp_event_config->id].ctrl);

		writel(gp_event_config->interval,
			&tbgregs->gp_tmr[gp_event_config->id].interval);

		break;

	case TBG_TDD:
		if (copy_from_user(config_buf, param->timer_config,
			sizeof(struct tdd_config))) {
			ret = -EFAULT;
			goto out;
		}
		tdd_config = (struct tdd_config *)config_buf;
		timer = tbg_find_timer(param->type, tdd_config->id);
		if (!timer) {
			ret = -ENODEV;
			goto out;
		}
		timer->offset = tdd_config->offset;
		ctrl_reg = (tdd_config->pulsewidth & 0xFFFF) << 16 |
				(tdd_config->pulse_mode & 0x3) << 10 |
				(tdd_config->rxtxen & 0x3) << 8 |
				(tdd_config->buflength & 0xF) << 4 |
				(tdd_config->fdd_sel & 0x1) << 3 |
				(tdd_config->rx_en_sel & 0x1) << 2 |
				(tdd_config->contseq & 0x1) << 1;
		if (!timer->offset) {
			ctrl_reg |= TMREN;
			writel(0, &tbgregs->tdd_tmr[tdd_config->id].osethi);
			wmb();
			writel(0, &tbgregs->tdd_tmr[tdd_config->id].osetlo);
		}
		writel(ctrl_reg,
			&tbgregs->tdd_tmr[tdd_config->id].ctrl);

		for (i = 0; i < 16; i++) {
			tdd_mode_reg |= (tdd_config->mode[i] & 0x3) << (i * 2);
			writel(tdd_config->duration[i],
				&tbgregs->tdd_tmr[tdd_config->id].duration[i]);
		}
		writel(tdd_mode_reg, &tbgregs->tdd_tmr[tdd_config->id].mode);
		break;
	default:
		ret = -ENODEV;
	}
out:
	if (!ret) {
		if (timer->offset)
			atomic_set(&timer->state, TIMER_STATE_READY);
		else
			atomic_set(&timer->state, TIMER_STATE_RUNNING);
	}
	kfree(config_buf);
	return ret;
}

static void tbgen_prepare_timer(struct tbg_timer *timer,
		int en, u64 timestamp,
		u32 *reg_lo, u32 *reg_hi, u32 *reg_en)
{
	u64 future_timestamp;
	u32 val;

	if (en) {
		if (atomic_read(&timer->state) != TIMER_STATE_READY)
			return;
		else {
			future_timestamp = timestamp + timer->offset;
			writel((future_timestamp >> 32) & 0xFFFFFFFF, reg_hi);
			wmb();
			writel(future_timestamp & 0xFFFFFFFF, reg_lo);
			wmb();
			val = readl(reg_en);
			writel(val | TMREN, reg_en);
			atomic_set(&timer->state, TIMER_STATE_RUNNING);
		}
	} else {
		if (atomic_read(&timer->state) == TIMER_STATE_RUNNING) {
			val = readl(reg_en);
			val &= ~TMREN;
			writel(val, reg_en);
			atomic_set(&timer->state, TIMER_STATE_IDLE);
		}
	}
}

static void __tbgen_onoff_timer(int en, u64 timestamp,
		const struct tbg_timer_mask *mask)
{

	int i;
	struct tbg_timer *timer;
	struct tbg_regs *regs = tbg->tbgregs;

	for (i = 0; i < TX_ALIGNMENT_TIMERS_CNT; i++) {
		timer = tbg_find_timer(TBG_TX_ALIGNMENT, i);
		if (!en && !(mask->tx_alignment_mask & (1 << i)))
			continue;
		tbgen_prepare_timer(timer, en, timestamp,
			&regs->tx_tmr[i].osetlo,
			&regs->tx_tmr[i].osethi,
			&regs->tx_tmr[i].ctrl);
	}

	for (i = 0; i < TX_AXRF_TIMERS_CNT; i++) {
		timer = tbg_find_timer(TBG_TX_AXRF, i);
		if (!en && !(mask->tx_axrf_mask & (1 << i)))
			continue;
		tbgen_prepare_timer(timer, en, timestamp,
				&regs->axrf_tmr[i].osetlo,
				&regs->axrf_tmr[i].osethi,
				&regs->axrf_tmr[i].ctrl);
	}

	for (i = 0; i < RX_ALIGNMENT_TIMERS_CNT; i++) {
		timer = tbg_find_timer(TBG_RX_ALIGNMENT, i);
		if (!en && !(mask->rx_alignment_mask & (1 << i)))
			continue;
		tbgen_prepare_timer(timer, en, timestamp,
				&regs->rx_tmr[i].osetlo,
				&regs->rx_tmr[i].osethi,
				&regs->rx_tmr[i].ctrl);
	}

	for (i = 0; i < SRX_ALIGNMENT_TIMERS_CNT; i++) {
		timer = tbg_find_timer(TBG_SRX_ALIGNMENT, i);
		if (!en && !(mask->srx_alignment_mask & (1 << i)))
			continue;
		tbgen_prepare_timer(timer, en, timestamp,
				&regs->srx_tmr[i].osetlo,
				&regs->srx_tmr[i].osethi,
				&regs->srx_tmr[i].ctrl);
	}

	for (i = 0; i < GP_EVENT_TIMERS_CNT; i++) {
		timer = tbg_find_timer(TBG_GP_EVENT, i);
		if (!en && !(mask->gp_event_mask & (1 << i)))
			continue;
		tbgen_prepare_timer(timer, en, timestamp,
				&regs->gp_tmr[i].osetlo,
				&regs->gp_tmr[i].osethi,
				&regs->gp_tmr[i].ctrl);
	}

	for (i = 0; i < TDD_TIMERS_CNT; i++) {
		timer = tbg_find_timer(TBG_TDD, i);
		if (!en && !(mask->tdd_mask & (1 << i)))
			continue;
		tbgen_prepare_timer(timer, en, timestamp,
			&regs->tdd_tmr[i].osetlo,
			&regs->tdd_tmr[i].osethi,
			&regs->tdd_tmr[i].ctrl);
	}
}

static int tbg_on_timer(void)
{
	u32 val;
	unsigned long timeout;

	if (atomic_read(&tbg->state) != TBG_STATE_RFG_RUNNING) {
		tbg_err("tbgen not ready yet, please check current state\n");
		return -EFAULT;
	}
	val = readl(&tbg->tbgregs->cntrl_1);
	writel(FSIE, &tbg->tbgregs->int_stat);
	writel(val | FSIE, &tbg->tbgregs->cntrl_1);

	timeout = wait_for_completion_timeout(&tbg->rfg_complete,
			msecs_to_jiffies(RFG_TIMEOUT_MS));

	val = readl(&tbg->tbgregs->cntrl_1);
	writel(val & ~FSIE, &tbg->tbgregs->cntrl_1);
	writel(FSIE, &tbg->tbgregs->int_stat);

	if (!timeout) {
		tbg_err("tbgen isn't getting the 10ms trigger for some reason\n");
			return -ETIME;
	}
	return 0;
}

long tbgen_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct tbg_timer_params timer_param;
	struct tbg_device_info info;
	struct tbg_timer_mask mask;

	switch (cmd) {
	case TBGEN_SET_PLL_RFG:
		if (copy_from_user(&tbg->config, (struct tbg_config *)arg,
				sizeof(struct tbg_config))) {
			ret = -EFAULT;
			break;
		}
		/* Init tbgen pll */
		ret = d4400_clk_tbgen_pll_set_rate(tbg->config.pll_freq_khz * 1000UL);
		if (ret) {
			atomic_set(&tbg->state, TBG_STATE_PLL_ERR);
			tbg_err("Fail to initialize tbgen pll\n");
			return ret;
		} else
			atomic_set(&tbg->state, TBG_STATE_PLL_RUNNING);

		/* Init RFG */
		ret = tbgen_rfg_init();
		if (ret)
			atomic_set(&tbg->state, TBG_STATE_RFG_ERR);
		else
			atomic_set(&tbg->state, TBG_STATE_RFG_RUNNING);
		break;

	case TBGEN_CONFIG_TIMER:
		if (copy_from_user(&timer_param, (struct tbg_timer_params *)arg,
				sizeof(struct tbg_timer_params))) {
			ret = -EFAULT;
			break;
		}
		ret = tbg_set_timer(&timer_param);
		if (ret)
			tbg_err("Fail to configure tbgen timer\n");
		break;

	case TBGEN_TIMERS_START:
		ret = tbg_on_timer();
		if (ret)
			tbg_err("Fail to turn on timer\n");
		break;

	case TBGEN_TIMERS_STOP:
		if (copy_from_user(&mask, (struct tbg_timer_mask *)arg,
				sizeof(struct tbg_timer_mask))) {
			ret = -EFAULT;
			break;
		}
		__tbgen_onoff_timer(0, 0, &mask);
		break;

	case TBGEN_GET_DEV_INFO:
		info.state = atomic_read(&tbg->state);
		info.pll_freq_khz = tbg->config.pll_freq_khz;
		if (copy_to_user((void *)arg, &info,
				sizeof(struct tbg_device_info)))
			return -EINVAL;

		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct file_operations tbg_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = tbgen_ioctl,
};

static struct miscdevice tbg_miscdev = {
	MISC_DYNAMIC_MINOR,
	"tbgen",
	&tbg_fops
};

static ssize_t set_debug(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	int err;
	unsigned int val;

	err = kstrtouint(buf, 0, &val);
	if (err)
		return err;

	tbg->debug = val;
	return count;
}

static ssize_t show_debug(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d / 0x%x\n", tbg->debug, tbg->debug);
}

static ssize_t show_tbg_status(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	ssize_t len = 0, temp_len;
	struct tbg_timer *ptr;

	const char *state_names[] = {
		"TBG_STATE_IDLE",
		"TBG_STATE_PLL_RUNNING",
		"TBG_STATE_RFG_RUNNING",
		"TBG_STATE_PLL_ERR",
		"TBG_STATE_RFG_ERR",
	};
	const char *sync_names[] = {
		"SELF_GENERATED_10MS",
		"CPRI_ALIGNED_10MS",
	};

	const char *timer_names[] = {
		"TBG_TX_ALIGNMENT",
		"TBG_TX_AXRF",
		"TBG_RX_ALIGNMENT",
		"TBG_SRX_ALIGNMENT",
		"TBG_GP_EVENT",
		"TBG_TDD",
	};
	const char *timer_state[] = {
		"IDLE",
		"READY",
		"RUNNING"
	};

	list_for_each_entry(ptr, &tbg->timer_list, list) {
		temp_len = sprintf(&buf[len], "%s%d:%s\n",
			timer_names[ptr->type], ptr->id,
			timer_state[atomic_read(&ptr->state)]);
		len += temp_len;
	}

	temp_len = sprintf(&buf[len], "tbgen pll %d Khz\n"
				"status %s\nsync src %s\n",
			tbg->config.pll_freq_khz,
			state_names[atomic_read(&tbg->state)],
			sync_names[tbg->config.radio_frame_src]);

	len += temp_len;
	return len;
}

static DEVICE_ATTR(debug,	S_IWUSR | S_IRUGO, show_debug, set_debug);
static DEVICE_ATTR(tbg_status,	S_IRUGO, show_tbg_status, NULL);

static struct attribute *attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_tbg_status.attr,
	NULL
};

static const struct attribute_group attr_group = {
	.attrs = attributes,
};

static irqreturn_t tbgen_isr(int irq, void *dev)
{
	struct tbgen_dev *tbg = (struct tbgen_dev *)dev;
	struct tbg_regs *tbgregs = tbg->tbgregs;
	u32 int_stat, val;
	u64 timestamp;

	int_stat = readl(&tbgregs->int_stat);
	if (!int_stat)
		return IRQ_NONE;
	if (int_stat & FSIE) {
		if (atomic_read(&tbg->state) == TBG_STATE_PLL_RUNNING) {
			if (tbg->config.radio_frame_src == SELF_GENERATED_10MS ||
				atomic_read(&tbg->rfg_in_transition))
					complete(&tbg->rfg_complete);
			else {
				val = readl(&tbgregs->rfg_cr);
				val &= ~REFSYNCSEL;
				writel(val, &tbgregs->rfg_cr);
				atomic_set(&tbg->rfg_in_transition, 1);
			}
		}
		if (atomic_read(&tbg->state) == TBG_STATE_RFG_RUNNING) {
			timestamp = tbgen_get_l10_mcntr();
			__tbgen_onoff_timer(1, timestamp, NULL);
			complete(&tbg->rfg_complete);
		}
	}
	writel(int_stat, &tbgregs->int_stat);

	return IRQ_HANDLED;
}

static int tbgen_of_probe(struct platform_device *pdev)
{
	int ret = 0, id;
	struct tbg_timer *timer;

	tbg = devm_kzalloc(&pdev->dev, sizeof(*tbg), GFP_KERNEL);

	if (!tbg)
		return -ENOMEM;

	INIT_LIST_HEAD(&tbg->timer_list);
	/* Add each timer to the list */
	for (id = 0; id < TX_ALIGNMENT_TIMERS_CNT; id++) {
		timer = devm_kzalloc(&pdev->dev, sizeof(*timer), GFP_KERNEL);
		if (!timer)
			return -ENOMEM;
		list_add(&timer->list, &tbg->timer_list);
		timer->type = TBG_TX_ALIGNMENT;
		timer->id = id;
	}

	for (id = 0; id < TX_AXRF_TIMERS_CNT; id++) {
		timer = devm_kzalloc(&pdev->dev, sizeof(*timer), GFP_KERNEL);
		if (!timer)
			return -ENOMEM;
		list_add(&timer->list, &tbg->timer_list);
		timer->type = TBG_TX_AXRF;
		timer->id = id;
	}

	for (id = 0; id < RX_ALIGNMENT_TIMERS_CNT; id++) {
		timer = devm_kzalloc(&pdev->dev, sizeof(*timer), GFP_KERNEL);
		if (!timer)
			return -ENOMEM;
		list_add(&timer->list, &tbg->timer_list);
		timer->type = TBG_RX_ALIGNMENT;
		timer->id = id;
	}

	for (id = 0; id < SRX_ALIGNMENT_TIMERS_CNT; id++) {
		timer = devm_kzalloc(&pdev->dev, sizeof(*timer), GFP_KERNEL);
		if (!timer)
			return -ENOMEM;
		list_add(&timer->list, &tbg->timer_list);
		timer->type = TBG_SRX_ALIGNMENT;
		timer->id = id;
	}

	for (id = 0; id < GP_EVENT_TIMERS_CNT; id++) {
		timer = devm_kzalloc(&pdev->dev, sizeof(*timer), GFP_KERNEL);
		if (!timer)
			return -ENOMEM;
		list_add(&timer->list, &tbg->timer_list);
		timer->type = TBG_GP_EVENT;
		timer->id = id;
	}

	for (id = 0; id < TDD_TIMERS_CNT; id++) {
		timer = devm_kzalloc(&pdev->dev, sizeof(*timer), GFP_KERNEL);
		if (!timer)
			return -ENOMEM;
		list_add(&timer->list, &tbg->timer_list);
		timer->type = TBG_TDD;
		timer->id = id;
	}

	tbg->debug = TBGEN_PRINT_ERR;
	tbg->node = pdev->dev.of_node;
	tbg->dev = &pdev->dev;
	tbg->tbgregs = of_iomap(tbg->node, 0);
	if (!tbg->tbgregs)
		return -ENOMEM;

	init_completion(&tbg->rfg_complete);

	tbg->irq = irq_of_parse_and_map(tbg->node, 0);
	if (tbg->irq) {
		ret = request_irq(tbg->irq, tbgen_isr, 0, "tbgen", tbg);
		if (ret) {
			tbg_err("Fail to register tbgen IRQ\n");
			goto release_mem;
		}
	} else {
		tbg_err("Fail to map tbgen IRQ\n");
		ret = -ENODEV;
		goto release_mem;
	}

	ret = misc_register(&tbg_miscdev);
	if (ret < 0) {
		tbg_err("misc_register fail for tbgen\n");
		goto release_irq;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret) {
		tbg_err("Error creating tbgen sysfs files\n");
		goto release_misc;
	}

	writel(0, &tbg->tbgregs->cntrl_1);
	return 0;

release_misc:
	misc_deregister(&tbg_miscdev);
release_irq:
	free_irq(tbg->irq, tbg);
release_mem:
	iounmap(tbg->tbgregs);

	return ret;
}

static int tbgen_of_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &attr_group);
	misc_deregister(&tbg_miscdev);
	free_irq(tbg->irq, tbg);
	iounmap(tbg->tbgregs);
	return 0;
}

static struct of_device_id tbgen_match[] = {
	{.compatible = "fsl,d4400-tbgen",},
	{},
};

static struct platform_driver tbgen_driver = {
	.driver = {
		.name = "fsl-tbgen",
		.owner = THIS_MODULE,
		.of_match_table = tbgen_match,
	},
	.probe = tbgen_of_probe,
	.remove = tbgen_of_remove,
};


static int __init tbgen_module_init(void)
{
	return platform_driver_register(&tbgen_driver);
}

static void __exit tbgen_module_clean(void)
{
	platform_driver_unregister(&tbgen_driver);
}

module_init(tbgen_module_init);
module_exit(tbgen_module_clean);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("tbgen driver");

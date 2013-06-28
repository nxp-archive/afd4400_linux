/* GCR configuration driver
 * Authod: Anand Singh
 * Anand Singh<b44195@freescale.com>
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/gcr.h>

#define DRIVER_NAME "gcr_dev"
#define MOD_VERSION "0.1"

static int gcr_open(struct inode *inode, struct file *filp);
static int gcr_release(struct inode *inode, struct file *filp);
static long gcr_ctrl(struct file *filp, unsigned int cmd, unsigned long arg);

static const struct file_operations gcr_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= gcr_ctrl,
	.open		= gcr_open,
	.release	= gcr_release,
};

long gcr_ctrl(struct file *file, unsigned int cmd, unsigned long ioctl_arg)
{
	int ret = 0;
	struct gcr_priv *priv;
	struct gcr_reg_map *gcr;
	unsigned int size;
	struct gcr_parm *arg = NULL;
	unsigned int count;
	u32 *reg_val;

	size = sizeof(struct gcr_parm);
	arg = kmalloc(size, GFP_KERNEL);
	if (copy_from_user((void *)arg, (struct gcr_parm *)ioctl_arg, size)) {
		kfree(arg);
		ret = -EFAULT;
		goto end;

	}
	count = ((struct gcr_parm *)arg)->count;
	priv = (struct gcr_priv *)file->private_data;
	gcr = priv->gcr_reg;
	dev_dbg(priv->gcr_dev, "IOCTL cmd:: %u\n", cmd);
	switch (cmd) {
	case GCR_CONFIG_CMD:
		{
			struct dma_intf_switch_parm_t *chan_parm;

			size = count * sizeof(
					struct dma_intf_switch_parm_t);
			chan_parm = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)chan_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(chan_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_vsp_intf_dma_cfg(chan_parm, count,
					gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(chan_parm);
		}
		break;
	case GCR_CPRI_DMA_MUX_CMD:
		{
			struct cpri_dma_mux_config *cpri_mux_parm;

			size = count * sizeof(
					struct cpri_dma_mux_config);
			cpri_mux_parm = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)cpri_mux_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(cpri_mux_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_cpri_dma_mux(cpri_mux_parm,
					count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(cpri_mux_parm);
		}
		break;
	case GCR_INTER_VSP_CFG:
		{
			struct inter_vsp_dma_config_t *inter_vsp_parm;

			size = count * sizeof(
					struct inter_vsp_dma_config_t);
			inter_vsp_parm =  kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)inter_vsp_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(inter_vsp_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_inter_vsp_dma_cfg(inter_vsp_parm,
					count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(inter_vsp_parm);

		}
		break;
	case GCR_JESD_PTR_RST_CFG:
		{
			struct jesd_dma_ptr_rst_parm *jesd_ptr_parm;

			size = count * sizeof(
					struct jesd_dma_ptr_rst_parm);
			jesd_ptr_parm = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)jesd_ptr_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(jesd_ptr_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_jesd_dma_ptr_rst_req(jesd_ptr_parm,
					count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(jesd_ptr_parm);
		}
		break;
	case GCR_WRITE_REG:
		{
			struct gcr_ctl_parm *param;

			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(param);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_write_set(param, count, gcr);
			mutex_unlock(&priv->gcr_lock);
			kfree(param);
		}
		break;
	case GCR_READ_REG:
		{
			struct gcr_ctl_parm *param;
			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				kfree(param);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			reg_val = gcr_read_reg(param, priv->gcr_dev, count,
					gcr);
			size = count * sizeof(u32);
			if (copy_to_user((void *)param->param, reg_val,	size)) {
				kfree(reg_val);
				ret = -EFAULT;
				goto end;
			}
			kfree(reg_val);
			kfree(param);
			mutex_unlock(&priv->gcr_lock);
		}
		break;
	default:
		dev_warn(priv->gcr_dev, "gcr_dev:: invalid ioctl cmd[%u]\n",
				cmd);
		ret = -EINVAL;
		goto end;
		break;
	}

end:
	kfree(arg);
	return ret;
}

static int gcr_open(struct inode *inode, struct file *file)
{
	struct gcr_priv *priv = NULL;

	priv = container_of(inode->i_cdev, struct gcr_priv, gcr_cdev);
	MOD_INC_USE_COUNT;
	file->private_data = priv;
	dev_dbg(((struct gcr_priv *)file->private_data)->gcr_dev,
			"gcr_dev:: Device opened\n");
	return 0;
}

static int gcr_release(struct inode *inode, struct file *fp)
{
	struct gcr_priv *priv = (struct gcr_priv *)fp->private_data;

	if (!priv)
		return -ENODEV;
	cdev_del(&priv->gcr_cdev);
	return 0;
}

static int __init init_gcr(void)
{
	int ret = 0;
	int err;
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();

	if ((!priv) || (!priv->gcr_dev)) {
		ret = -ENODEV;
		goto err_dev;
	}

	err = alloc_chrdev_region(&priv->dev_t, 0, GCR_DEV_MAX, DRIVER_NAME);
	if (err) {
		pr_err("cannot register gcr driver\n");
		ret = -EFAULT;
	}

	cdev_init(&priv->gcr_cdev, &gcr_fops);
	err = cdev_add(&priv->gcr_cdev, priv->dev_t, 1);
	if (err) {
		pr_err("cannot add gcr driver\n");
		ret = -EFAULT;
	}
	mutex_init(&priv->gcr_lock);
err_dev:
	return ret;
}

static void __exit exit_gcr(void)
{
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();

	if ((!priv) || (!priv->gcr_dev))
		return;
	unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
}

module_init(init_gcr);
module_exit(exit_gcr);

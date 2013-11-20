/*
 * drivers/char/scm_if.c
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
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
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

#define DRIVER_NAME "scm"
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
	int i = 0;
	struct gcr_priv *priv;
	struct gcr_reg_map *gcr;
	unsigned int size = 0;
	struct gcr_parm *arg = NULL;
	unsigned int count = 0;

	priv = (struct gcr_priv *)file->private_data;
	gcr = priv->gcr_reg;

	size = sizeof(struct gcr_parm);
	arg = kmalloc(size, GFP_KERNEL);
	if (!arg) {
		dev_err(priv->gcr_dev, "kmalloc failed for gcr_parm\n");
		ret = -EFAULT;
		goto end;
	}

	if (copy_from_user((void *)arg, (struct gcr_parm *)ioctl_arg, size)) {
		kfree(arg);
		dev_err(priv->gcr_dev, "copy_from_user failed\n");
		ret = -EFAULT;
		goto end;
	}
	count = ((struct gcr_parm *)arg)->count;
	switch (cmd) {
	case GCR_CONFIG_CMD:
		{
			struct dma_intf_switch_parm_t *chan_parm = NULL;

			size = count * sizeof(
					struct dma_intf_switch_parm_t);
			chan_parm = kmalloc(size, GFP_KERNEL);
			if (!chan_parm) {
				dev_err(priv->gcr_dev, "chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)chan_parm,
					((struct gcr_parm *)arg)->parm, size)) {
				dev_err(priv->gcr_dev, "copy_from_user failed\n");
				kfree(chan_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_vsp_intf_dma_cfg(chan_parm, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(chan_parm);
		}
		break;
	case GCR_CPRI_DMA_MUX_CMD:
		{
			struct cpri_dma_mux_config *cpri_mux_parm = NULL;

			size = count * sizeof(struct cpri_dma_mux_config);
			cpri_mux_parm = kmalloc(size, GFP_KERNEL);
			if (!cpri_mux_parm) {
				dev_err(priv->gcr_dev, "cpri_mux_parm allc fail\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)cpri_mux_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				dev_err(priv->gcr_dev, "copy_from_user failed\n");
				kfree(cpri_mux_parm);
				ret = -EFAULT;
				goto end;
			}

			mutex_lock(&priv->gcr_lock);
			ret = gcr_cpri_dma_mux(cpri_mux_parm, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(cpri_mux_parm);
		}
		break;
	case GCR_INTER_VSP_CFG:
		{
			struct inter_vsp_dma_config_t *inter_vsp_parm = NULL;

			size = count * sizeof(
					struct inter_vsp_dma_config_t);
			inter_vsp_parm =  kmalloc(size, GFP_KERNEL);
			if (!inter_vsp_parm) {
				dev_err(priv->gcr_dev, "intr_vsp_parm allc fail\n");
				ret = -EFAULT;
				goto end;
			}

			if (copy_from_user((void *)inter_vsp_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				dev_err(priv->gcr_dev, "copy_from_user failed\n");
				kfree(inter_vsp_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_inter_vsp_dma_cfg(inter_vsp_parm, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(inter_vsp_parm);
		}
		break;
	case GCR_JESD_PTR_RST_CFG:
		{
			struct jesd_dma_ptr_rst_parm *jesd_ptr_parm = NULL;

			size = count * sizeof(
					struct jesd_dma_ptr_rst_parm);
			jesd_ptr_parm = kmalloc(size, GFP_KERNEL);
			if (!jesd_ptr_parm) {
				dev_err(priv->gcr_dev, "jesd_ptr_parm allc fail\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)jesd_ptr_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				dev_err(priv->gcr_dev, "copy_from_user failed\n");
				kfree(jesd_ptr_parm);
				ret = -EFAULT;
				goto end;
			}
			mutex_lock(&priv->gcr_lock);
			ret = gcr_jesd_dma_ptr_rst_req(jesd_ptr_parm,
					count);
			mutex_unlock(&priv->gcr_lock);
			kfree(jesd_ptr_parm);
		}
		break;
	case GCR_WRITE_REG:
		{
			struct gcr_ctl_parm *param = NULL;

			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (!param) {
				dev_err(priv->gcr_dev, "Faild to alloc param(W)\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				dev_err(priv->gcr_dev, "copy_from_user failed\n");
				kfree(param);
				ret = -EFAULT;
				goto end;
			}

			mutex_lock(&priv->gcr_lock);
			gcr_write_set(param, count);
			mutex_unlock(&priv->gcr_lock);
			kfree(param);
		}
		break;
	case GCR_READ_REG:
		{
			struct gcr_ctl_parm *param = NULL;
			unsigned int m_addr = 0;
			int data = 0;

			size = count * sizeof(
					struct gcr_ctl_parm);
			param = kmalloc(size, GFP_KERNEL);
			if (!param) {
				dev_err(priv->gcr_dev, "Faild to alloc param(R)\n");
				ret = -EFAULT;
				goto end;
			}

			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				dev_err(priv->gcr_dev, "copy_from_user failed\n");
				kfree(param);
				ret = -EFAULT;
				goto end;
			}
			for (i = 0; i < count; i++) {
				mutex_lock(&priv->gcr_lock);
				m_addr = (param + i)->reg_offset;
				data = gcr_read_set(m_addr);
				mutex_unlock(&priv->gcr_lock);
				(param + i)->param = data;
			}
			if (copy_to_user((void *)arg->parm,  param, size)) {
				kfree(param);
				dev_err(priv->gcr_dev, "copy_to_user failed\n");
				ret = -EFAULT;
				goto end;
			}
			kfree(param);
		}
		break;
	default:
		dev_info(priv->gcr_dev, "gcr_dev:: invalid ioctl cmd[%u]\n",
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
	int rc = 0;
	priv = container_of(inode->i_cdev, struct gcr_priv, gcr_cdev);
	if (priv != NULL) {
		MOD_INC_USE_COUNT;
		file->private_data = priv;
	} else {
		rc = -ENODEV;
	}

	return rc;
}

static int gcr_release(struct inode *inode, struct file *fp)
{
	struct gcr_priv *priv = (struct gcr_priv *)fp->private_data;

	if (!priv)
		return -ENODEV;

	MOD_DEC_USE_COUNT;
	return 0;
}

static int __init init_gcr(void)
{
	int ret = 0;
	int err = 0;
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();
	int gcr_major = 0;
	int gcr_minor = 0;
	dev_t devt;

	if (!priv) {
		ret = -ENODEV;
		goto out_err;
	}

	priv->gcr_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(priv->gcr_class)) {
		ret = PTR_ERR(priv->gcr_class);
		goto out_err;
	}

	err = alloc_chrdev_region(&devt, 0, GCR_DEV_MAX, DRIVER_NAME);
	if (err) {
		ret = -EFAULT;
		goto out_err_1;
	}

	gcr_major = MAJOR(devt);
	gcr_minor = MINOR(devt);
	priv->dev_t = MKDEV(gcr_major, gcr_minor);

	cdev_init(&priv->gcr_cdev, &gcr_fops);
	err = cdev_add(&priv->gcr_cdev, priv->dev_t, GCR_DEV_MAX);
	if (err) {
		ret = -EFAULT;
		goto out_err_1;
	}

	priv->gcr_dev = device_create(priv->gcr_class, NULL, priv->dev_t,
				NULL, DRIVER_NAME);
	if (IS_ERR(priv->gcr_dev)) {
		ret = -EFAULT;
		goto out_err_2;
	}

	mutex_init(&priv->gcr_lock);
	dev_set_drvdata(priv->gcr_dev, priv);

	return 0;

out_err_2:
	cdev_del(&priv->gcr_cdev);
out_err_1:
	class_destroy(priv->gcr_class);
	unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
out_err:
	return ret;
}

static void __exit exit_gcr(void)
{
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();
	device_destroy(priv->gcr_class, priv->dev_t);
	cdev_del(&priv->gcr_cdev);
	class_destroy(priv->gcr_class);

	if ((!priv) || (!priv->gcr_dev))
		return;
	unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
}

module_init(init_gcr);
module_exit(exit_gcr);

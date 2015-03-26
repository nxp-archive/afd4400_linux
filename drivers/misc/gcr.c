/*
 * drivers/misc/gcr.c
 * AFD4400 GCR device driver
 *
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
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
#include <linux/module.h>
#include <linux/gcr.h>

#define DRIVER_NAME "gcr"
#define MOD_VERSION "0.1"

/* Debug and error reporting macros */
#define IF_DEBUG(x)     if (debug & (x))
#define ERR(...)        {if (debug & DEBUG_MESSAGES) \
                                pr_err(DRIVER_NAME ": " __VA_ARGS__);}

static struct kobject *gcr_kobj;
static uint32_t debug = DEBUG_MESSAGES;

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
		ERR("gcr_ctrl: kmalloc failed for gcr_parm\n");
		ret = -EFAULT;
		goto end;
	}

	if (copy_from_user((void *)arg, (struct gcr_parm *)ioctl_arg, size)) {
		kfree(arg);
		ERR("gcr_ctrl: copy_from_user failed\n");
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
				ERR("GCR_CONFIG_CMD: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)chan_parm,
					((struct gcr_parm *)arg)->parm, size)) {
				ERR("GCR_CONFIG_CMD: copy from user failed\n");
				kfree(chan_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_VSPA_2_IF_CFG) {
				int i;
				struct dma_intf_switch_parm_t *p =
				 (struct dma_intf_switch_parm_t*)chan_parm;
				pr_info(DRIVER_NAME ": VSP->IF DMA Config\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("DMA: %s CH%02d VSP%02d DMA%02d FRM%d %s\n",
						p->dev_type == 1 ? "CPRI" : "JESD",
						p->chan_id,
						p->vsp_id,
						p->dma_request_id,
						p->cpri_framert_id,
						p->comm_type == 1 ? "UL" : "DL");
				}
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
				ERR("GCR_CPRI_DMA..: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)cpri_mux_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_CPRI_DMA..: copy_from_user failed\n");
				kfree(cpri_mux_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_CPRI_MUX_CFG) {
				int i;
				struct cpri_dma_mux_config *p =
				 (struct cpri_dma_mux_config*)cpri_mux_parm;
				pr_info(DRIVER_NAME ": CPRI MUX Config\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("MUX: CPRI%02d->%s%d CH%02d\n",
						p->src_cpri_complex,
						p->rxtx_id > 2 ?
							"TX" : "RX",
						(p->rxtx_id % 1) + 1,
						p->cpri_dma_req_out);
				}
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
				ERR("GCR_VSP->VSP..: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}

			if (copy_from_user((void *)inter_vsp_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_VSP->VSP..: copy_from_user failed\n");
				kfree(inter_vsp_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_VSPA_2_VSPA_CFG) {
				int i;
				struct inter_vsp_dma_config_t *p =
				 (struct inter_vsp_dma_config_t*)inter_vsp_parm;
				pr_info(DRIVER_NAME ": VSP->VSP DMA Config\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("DMA: CH%02d VSPA%02d->VSPA%02d\n",
						p->chan_id,
						p->src_vsp_id,
						p->dst_vsp_id);
				}
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
				ERR("GCR_PTR_RST..: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)jesd_ptr_parm,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_PTR_RST..: copy_from_user failed\n");
				kfree(jesd_ptr_parm);
				ret = -EFAULT;
				goto end;
			}
			IF_DEBUG(DEBUG_JESD_PTR_RESET) {
				int i;
				struct jesd_dma_ptr_rst_parm *p =
				  (struct jesd_dma_ptr_rst_parm*)jesd_ptr_parm;
				pr_info(DRIVER_NAME ": Reset JESD pointer\n");
				for (i = 0; i < count; i++, p++) {
					pr_info("RST: VSPA%02d, JESD_%s%02d, DMA %02d\n",
						p->vsp_id,
						p->comm_type==1 ? "UL" : "DL",
						p->jesd_id, p->chan_id);
				}
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
				ERR("GCR_REG_WRITE: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}
			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_REG_WRITE: copy_from_user failed\n");
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
				ERR("GCR_REG_READ: chan_parm alloc failed\n");
				ret = -EFAULT;
				goto end;
			}

			if (copy_from_user((void *)param,
						((struct gcr_parm *)arg)->parm,
						size)) {
				ERR("GCR_REG_READ: copy_from_user failed\n");
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
		ERR("unknown ioctl command %u\n", cmd);
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

/***************************** Sysfs *******************************/
//TODO - complete sysfs: registers

static ssize_t show_debug(struct device *dev,
                        struct device_attribute *devattr, char *buf)
{
        return sprintf(buf, "0x%08X\n", debug);
}

static ssize_t set_debug(struct device *dev, struct device_attribute *devattr,
                        const char *buf, size_t count)
{
        int err;
        unsigned int val;

        err = kstrtouint(buf, 0, &val);
        if (err) return err;

        debug = val;
        return count;
}

static DEVICE_ATTR(debug,  S_IWUSR | S_IRUGO, show_debug,    set_debug);
//static DEVICE_ATTR(versions,         S_IRUGO, show_versions, NULL);

static struct attribute *attributes[] = {
        &dev_attr_debug.attr,
        NULL
};

static const struct attribute_group attr_group = {
        .attrs = attributes,
};

/************************** Init / Exit ****************************/

static int __init init_gcr(void)
{
	int ret = 0;
	int err = 0;
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();
	int gcr_major = 0;
	int gcr_minor = 0;
	dev_t devt;

	if (!priv) {
		pr_err(DRIVER_NAME ": no private data\n");
		ret = -ENODEV;
		goto out_err;
	}

	err = alloc_chrdev_region(&devt, 0, GCR_DEV_MAX, DRIVER_NAME);
	if (err) {
		pr_err(DRIVER_NAME ": alloc_chrdev_region() failed: %d\n", err);
		ret = -EFAULT;
		goto out_err;
	}

	gcr_major = MAJOR(devt);
	gcr_minor = MINOR(devt);
	priv->dev_t = MKDEV(gcr_major, gcr_minor);

	priv->gcr_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(priv->gcr_class)) {
		ret = PTR_ERR(priv->gcr_class);
		pr_err(DRIVER_NAME ": class_create() failed %d\n", ret);
		goto out_err_1;
	}

	cdev_init(&priv->gcr_cdev, &gcr_fops);
	priv->gcr_cdev.owner = THIS_MODULE;
	err = cdev_add(&priv->gcr_cdev, priv->dev_t, GCR_DEV_MAX);
	if (err) {
		pr_err(DRIVER_NAME ": cdev_add() failed %d\n", err);
		ret = -EFAULT;
		goto out_err_2;
	}

	priv->gcr_dev = device_create(priv->gcr_class, NULL, priv->dev_t,
				NULL, DRIVER_NAME);
	if (IS_ERR(priv->gcr_dev)) {
		pr_err(DRIVER_NAME ": device_create() failed %ld\n",
						 PTR_ERR(priv->gcr_dev));
		ret = -EFAULT;
		goto out_err_3;
	}

	gcr_kobj = kobject_create_and_add(DRIVER_NAME, &(priv->gcr_dev->kobj));
	if (!gcr_kobj) {
                pr_err(DRIVER_NAME ": kobject_create_and_add() failed\n");
		ret = -ENOMEM;
		goto out_err_4;
	}

        ret = sysfs_create_group(gcr_kobj, &attr_group);
        if (ret < 0) {
                pr_err(DRIVER_NAME ": sysfs_create_group() failed %d\n", ret);
		goto out_err_5;
        }

	mutex_init(&priv->gcr_lock);
	dev_set_drvdata(priv->gcr_dev, priv);

	return 0;

out_err_5:
	kobject_put(gcr_kobj);
out_err_4:
	device_destroy(priv->gcr_class, priv->dev_t);
out_err_3:
	cdev_del(&priv->gcr_cdev);
out_err_2:
	class_destroy(priv->gcr_class);
out_err_1:
	unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
out_err:
	return ret;
}

static void __exit exit_gcr(void)
{
	struct gcr_priv *priv = (struct gcr_priv *)get_scm_priv();

	if ((!priv) || (!priv->gcr_dev)) {
                pr_err(DRIVER_NAME ": exit() has no private data\n");
		return;
	}
	kobject_put(gcr_kobj);
	device_destroy(priv->gcr_class, priv->dev_t);
	cdev_del(&priv->gcr_cdev);
	class_destroy(priv->gcr_class);
	unregister_chrdev_region(priv->dev_t, GCR_DEV_MAX);
}

module_init(init_gcr);
module_exit(exit_gcr);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale AFD4400 GCR Driver");

/* Necessary includes for device drivers */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h> /* kmalloc() */
#include <linux/fs.h> /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/types.h> /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h> /* O_ACCMODE */
#include <asm/system.h> /* cli(), *_flags */
#include <asm/uaccess.h> /* copy_from/to_user */

int reg_open(struct inode *inode, struct file *filp);
int reg_release(struct inode *inode, struct file *filp);

long reg_ioctl(struct file *file,
	unsigned int ioctl_num,	unsigned long arg);
void reg_exit(void);
int reg_init(void);

static	int major;
/* Structure that declares the usual file */
/* access functions */
struct file_operations reg_fops = {
	.owner = THIS_MODULE,
	.open = reg_open,
	.unlocked_ioctl = reg_ioctl,
	.release = reg_release,

};

int reg_open(struct inode *inode, struct file *filp) {

	/* Success */
	return 0;
}

/* This function is called whenever a process tries to 
 * do an ioctl on our device file. We get two extra 
 * parameters (additional to the inode and file 
 * structures, which all device functions get): the number
 * of the ioctl called and the parameter given to the 
 * ioctl function.
 *
 * If the ioctl is write or read/write (meaning output 
 * is returned to the calling process), the ioctl call 
 * returns the output of this function.
 */
#define IOCTL_REG_READ 0
#define IOCTL_REG_WRITE 1
#define DEVICE_NAME "reg"

struct data_req {
	u32 addr;
	u32 data;
};

long reg_ioctl(struct file *file, unsigned int ioctl_num, unsigned long arg){

	struct data_req * dr = (struct data_req *)arg;

	/* Switch according to the ioctl called */
	switch (ioctl_num) {
	case IOCTL_REG_READ:
		if (copy_to_user(&dr->data, (u32 *)dr->addr, 4))
                	return -EINVAL;
		//printk("Read Reg(%08X) = %08X\n", dr->addr, dr->data);
		break;

	case IOCTL_REG_WRITE:
		*((u32 *)dr->addr) = dr->data;
		break;

	default:
		;
	}
	return 0;
}



int reg_release(struct inode *inode, struct file *filp) {
 
	/* Success */
	return 0;
}
int reg_init(void) {

	/* Registering device */
	major = register_chrdev(0, DEVICE_NAME, &reg_fops);
	if (major < 0) {
		printk("reg: cannot obtain major number %d\n", major);
		return major;
	}

	printk("Inserting reg module\n"); 
	printk("I was assigned major number %d. To talk to\n", major);
	printk("the driver, create a dev file with\n");
	printk("'mknod /dev/%s c %d 0'.\n", DEVICE_NAME, major);
	return 0;
}

void reg_exit(void) {
  /* Freeing the major number */
  unregister_chrdev(major, "reg");
  printk("Removing reg module\n");

}
MODULE_LICENSE("GPL");
module_init(reg_init);
module_exit(reg_exit);




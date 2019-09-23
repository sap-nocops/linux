#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/pmm.h>

/*
 *	The various file operations we support.
 */

static long pmm_ioctl(struct file *file, unsigned int cmd,
                      unsigned long arg);
static const struct file_operations pmm_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = pmm_ioctl,
};

static struct miscdevice pmm_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "power",
	.fops	= &pmm_fops,
};

#if 0
/* TODO: we should make a linked list */
struct drv_list
{
	struct pmm_driver payload;
	struct drv_list *prev;
	struct drv_list *next;
};

static struct drc_list *pmm_reg_drivers = NULL;
#else
static struct pmm_driver pmm_driver_list[20];
static char list_inited = 1;
#endif

static void pmm_init_list(void)
{
	int i;
	if (list_inited == 0)
		return;
	
	for (i = 0; i < 20; i++) {
		pmm_driver_list[i].on		= NULL;
		pmm_driver_list[i].off		= NULL;
		pmm_driver_list[i].sleep	= NULL;
		pmm_driver_list[i].payload	= NULL;
		pmm_driver_list[i].name[0]	= 0;
		pmm_driver_list[i].private	= 1;
	}
	list_inited = 0;
}

/* Add new driver to the list */
int pmm_add_driver(struct pmm_driver *drv)
{
	int i;
	int ret = -1;
	struct pmm_driver *cur;

	pmm_init_list();

	printk("PMM: Registering `%s'...\n", drv->name);

	for (i = 0; i < 20; i++) {
		cur = &(pmm_driver_list[i]);

		if (strncmp(drv->name, cur->name, 20) == 0)
			goto exit;

		if (cur->private != 0) {
			strncpy(cur->name, drv->name, 20);
			cur->on = drv->on;
			cur->off = drv->off;
			cur->sleep = drv->sleep;
			cur->payload = drv->payload;
			cur->private = 0;
			ret = 0;

			goto exit;
		}
	}

exit:
	return ret;
}
EXPORT_SYMBOL(pmm_add_driver);

/* Del driver from the list */
int pmm_del_driver(struct pmm_driver *drv)
{
	/* Nothing for now */
	pmm_init_list();

	return 0;
}
EXPORT_SYMBOL(pmm_del_driver);


static long pmm_ioctl(struct file *file, unsigned int cmd,
                      unsigned long arg)
{
	long ret = -EIO;
	void __user *argp = (void __user *)arg;
	char devname[20];
	int i;
	struct pmm_driver *cur;

	pmm_init_list();

	if (copy_from_user(devname, argp, sizeof(devname))) {
		ret = -EFAULT;
		goto exit;
	}

	/* select driver from arg */
	for(i = 0; i < 20; i++) {
		cur = &(pmm_driver_list[i]);
		if (cur->private == 0) {
			if ( (strncmp(devname, cur->name, 20) == 0) )
				goto exit_loop;
		}
	}
	goto exit;

exit_loop:
	switch (cmd) {
	default:
		break;
	case PMM_DRIVER_ON:
		if (cur->on)
			ret = cur->on(cur->payload);
		break;
	case PMM_DRIVER_SLEEP:
		if (cur->sleep)
			ret = cur->sleep(cur->payload);
		break;
	case PMM_DRIVER_OFF:
		if (cur->off)
			ret = cur->off(cur->payload);
		break;
	}

exit:
	return ret;
}

static int __init pmm_init(void)
{
	printk("PMM Init.\n");
	misc_register(&pmm_dev);
	return 0;
}

static void __exit pmm_exit(void)
{
	misc_deregister(&pmm_dev);
}

module_init(pmm_init);
module_exit(pmm_exit);

MODULE_AUTHOR("Manoel Trapier <manoelt@bookeen.com>");
MODULE_AUTHOR("Yoann Sculo <yoanns@bookeen.com>");
MODULE_LICENSE("GPL");

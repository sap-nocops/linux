/*
 *  linux/drivers/char/mem.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Added devfs support. 
 *    Jan-11-1998, C. Scott Ananian <cananian@alumni.princeton.edu>
 *  Shared /dev/zero mmaping support, Feb 2000, Kanoj Sarcar <kanoj@sgi.com>
 */

#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/crash_dump.h>
#include <linux/backing-dev.h>
#include <linux/bootmem.h>
#include <linux/splice.h>
#include <linux/pfn.h>
#include <linux/proc_fs.h>
#include <linux/smp_lock.h>
#include <linux/kernel.h>

#include <asm/uaccess.h>
#include <asm/io.h>

static int open_proc(struct inode *inode, struct file *file)
{
	return 0;
}

static int release_proc(struct inode *inode, struct file *file)
{
	return 0;
}
 
static ssize_t read_proc(struct file *filp, char __user *buffer, size_t length,loff_t * offset)
{
	return -EINVAL;
}
 
static ssize_t write_proc(struct file *filp, const char *buff, size_t len, loff_t * off)
{
	char opts[256];
	char* end = NULL;

	copy_from_user(opts, buff, min(len, 255));
	if (len > 255)
		opts[255] = 0;
	else
		opts[len] = 0;
	
	
	unsigned long start = simple_strtoul(opts, &end, 16);
	if (*end == ' ')
		end++;
	
	unsigned long size = simple_strtoul(end, &end, 16);
	if (size > 0) {
	int i;
	u32 __iomem * x;
	mm_segment_t oldfs;
	int ret;
	struct file *filp = NULL;
	unsigned long long offset = 0;

	x = ioremap(start, size);
	
	pr_err("dump: %x %x\n", start, size);

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open("/tmp/dump", O_WRONLY|O_CREAT|O_TRUNC, 0777);
	if (!IS_ERR(filp)) {
		vfs_write(filp, x, size, &offset);
		filp_close(filp, NULL);
	}

	set_fs(oldfs);

	iounmap(x);
	}

	return len;
}

static struct file_operations proc_fops = {
        .open = open_proc,
        .read = read_proc,
        .write = write_proc,
        .release = release_proc
};
       
static int chr_dev_init(void)
{
        proc_create("dump", 0666, NULL, &proc_fops);

	return 0;
}

static void chr_dev_exit(void)
{
}

module_init(chr_dev_init);
module_exit(chr_dev_exit);

/*
 * drivers/misc/devicesn.c
 *
 * Copyright (c) 2013 Yoann Sculo
 *               2011 Manoel Trapier
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <asm/mach-types.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
 
/* Get nv kernel param */
static char device_serial_number[20];
static char board_serial_number[10];

#ifdef MODULE
#include <linux/module.h>
#include <linux/moduleparam.h>
module_param_string(sn, device_serial_number, sizeof(device_serial_number), S_IRUGO);
MODULE_PARM_DESC(device_serial_number, "sn");
module_param_string(bsn, board_serial_number, sizeof(board_serial_number), S_IRUGO);
MODULE_PARM_DESC(board_serial_number, "bsn");
#else
#include <linux/init.h>

static int sn_option_setup(char *str)
{
	int i = 0;
	while(str[i] != 0) { 
		if (i > 19) break;
		device_serial_number[i] = str[i];
		i++;
	}
	device_serial_number[i] = 0;
	return 1;
}

static int bsn_option_setup(char *str)
{
	int i = 0;
	while(str[i] != 0) { 
		if (i > 9) break;
		board_serial_number[i] = str[i];
		i++;
	}
	board_serial_number[i] = 0;
	return 1;
}

// TODO : use kernel parameters instead
// __setup("sn=", sn_option_setup);
// __setup("bsn=", bsn_option_setup);

#endif

static struct proc_dir_entry *snProcDir;
static struct proc_dir_entry *bsnProcDir;
static struct proc_dir_entry *deviceProcDir;

static int device_procReadIo (char *page, char **start, off_t off, int count,
                               int *eof, void *data)
{
	if (machine_is_sun5i())
		return sprintf(page, "CYBOOK_AWA13");
	else
		return sprintf(page, "UNKNOWN_DEVICE");
}

static int bsn_procReadIo (char *page, char **start, off_t off, int count,
                               int *eof, void *data)
{
	int len;

	len = sprintf (page, "%s", board_serial_number);

	return len;
}

static int sn_procReadIo (char *page, char **start, off_t off, int count,
                               int *eof, void *data)
{
	int len;

	len = sprintf (page, "%s", device_serial_number);

	return len;
}

static int __init devicesn_init(void)
{
	if (machine_is_sun5i())
		printk("AWA13 Platform [AllwinnerA13@1Ghz]\n");

	sn_option_setup("CF605WE4SD810001P1");
	bsn_option_setup("0017079");

	printk("sn = `%s' | bsn = `%s'\n", device_serial_number, board_serial_number);

	snProcDir = create_proc_entry("sn", 0444, NULL);
	snProcDir->read_proc  = sn_procReadIo;
	bsnProcDir = create_proc_entry("bsn", 0444, NULL);
	bsnProcDir->read_proc  = bsn_procReadIo;
	deviceProcDir = create_proc_entry("device", 0444, NULL);
	deviceProcDir->read_proc  = device_procReadIo;

	return 0;
}

static void __exit devicesn_exit(void)
{
	remove_proc_entry("sn", NULL);
	remove_proc_entry("bsn", NULL);
	remove_proc_entry("device", NULL);
}

module_init(devicesn_init);
module_exit(devicesn_exit);

#ifdef MODULE

MODULE_AUTHOR("Manoel Trapier <manoelt@bookeen.com>");
MODULE_AUTHOR("Yoann Sculo <yoanns@bookeen.com>");
MODULE_DESCRIPTION("Simple Device Serial Number");
MODULE_LICENSE("GPL v2");

#endif

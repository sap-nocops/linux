/* Copyright Statement: 
 * 
 * This software/firmware and related documentation ("MediaTek Software") are 
 * protected under relevant copyright laws. The information contained herein 
 * is confidential and proprietary to MediaTek Inc. and/or its licensors. 
 * Without the prior written permission of MediaTek inc. and/or its licensors, 
 * any reproduction, modification, use or disclosure of MediaTek Software, 
 * and information contained herein, in whole or in part, shall be strictly prohibited. 
 */
/* MediaTek Inc. (C) 2010. All rights reserved. 
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES 
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") 
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON 
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT. 
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE 
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR 
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH 
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES 
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES 
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK 
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR 
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK' S ENTIRE AND 
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, 
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, 
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO 
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
 * 
 * The following software/firmware and/or related documentation ("MediaTek Software") 
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's 
 * applicable license agreements with MediaTek Inc. 
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/time.h>

//#include <mach/mt6573_gpio.h>
//#include <mach/mt6573_pll.h>

#include "tpd_directives.h"
#include "tpd_defs.h"

#include <mach/sys_config.h>
#include <mach/gpio.h>

#define FIRMWARE_PATH	"/opt/cypressv1.hex"

static void tpd_upgrader_init_pin(void);
static void tpd_upgrader_deinit_pin(void); 

static long tpd_upgrader_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int tpd_upgrader_open(struct inode *inode, struct file *file);
static int tpd_upgrader_release(struct inode *ignored, struct file *file);

extern int tpd_initialize(void);
extern int tpd_verify_id(void);
extern int tpd_erase_target(void);
extern int tpd_read_status(void);
extern int tpd_verify_setup(unsigned char, unsigned char);
extern int tpd_read_byte_loop(void);
extern int tpd_program_target_block(unsigned char, unsigned char);
extern int tpd_load_security_data(unsigned char, struct file*);
extern int tpd_load_program_data(unsigned char, unsigned char, struct file*);
extern int tpd_secure_target_flash(void);
extern int tpd_verify_security(void);
extern int tpd_target_bank_checksum(unsigned int*);

extern void tpd_restart_target(void);
extern unsigned int tpd_load_target(void);

// Print debug information 
// [default disable = 0, enable = 1]
int debug = 0;

// Skip checking mechanism
// [default disable = 0, enable = 1]
int check = 0;

static short param = 0;
module_param(param, short, 0644);
MODULE_PARM_DESC(param, "Setup debug variable");

mm_segment_t old_fs;   

int gpio_i2c_scl_hdle = 0;
int gpio_i2c_sda_hdle = 0;
int gpio_i2c_reset_hdle = 0;

user_gpio_set_t gpio_i2c_scl_info;
user_gpio_set_t gpio_i2c_sda_info;
user_gpio_set_t gpio_i2c_reset_info;

/**
 * ctp_free_platform_
resource - corresponding with ctp_init_platform_resource *
 */
static void tpd_free_platform_resource(void)
{
	printk("=======%s=========.\n", __func__);
		
    if(gpio_i2c_reset_hdle){
		gpio_release(gpio_i2c_reset_hdle, 2);

	}

	return;
}


/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO:  i/o err. 
 * 
*/
static int tpd_init_platform_resource(void)
{
	int ret = 0;

	gpio_i2c_reset_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_i2c_reset_hdle) {
		pr_warning("%s: tpd_wakeuprequest gpio fail!\n", __func__);
		gpio_i2c_reset_hdle = 0;
	}

	gpio_i2c_scl_hdle = gpio_request_ex("twi1_para", "twi1_scl");
	if(!gpio_i2c_scl_hdle) {
		pr_warning("%s: gpio_i2c_scl_hdle twi1_scl fail!\n", __func__);
		gpio_i2c_scl_hdle = 0;
	}

	gpio_i2c_sda_hdle = gpio_request_ex("twi1_para", "twi1_sda");
	if(!gpio_i2c_sda_hdle) {
		pr_warning("%s: gpio_i2c_sda_hdle twi1_sda fail!\n", __func__);
		gpio_i2c_sda_hdle = 0;
	}	
	
	gpio_get_one_pin_status(gpio_i2c_scl_hdle,&gpio_i2c_scl_info,"twi1_scl",1);
	gpio_get_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);
	gpio_get_one_pin_status(gpio_i2c_reset_hdle,&gpio_i2c_reset_info,"ctp_wakeup",1);

	gpio_i2c_scl_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_scl_hdle,&gpio_i2c_scl_info,"twi1_scl",1);

	gpio_i2c_sda_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

	gpio_i2c_reset_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_reset_hdle,&gpio_i2c_reset_info,"ctp_wakeup",1);

	return ret;
exit_ioremap_failed:
	tpd_free_platform_resource();
	return ret;
}

static void tpd_upgrader_deinit_pin(void) 
{
//    mt_set_gpio_mode(I2C_SCL, 0x01);
//    mt_set_gpio_mode(I2C_SDA, 0x01);
}

static void tpd_upgrader_init_pin(void) 
{
	int reg_val = 0;
	
	tpd_init_platform_resource();
	gpio_write_one_pin_value(gpio_i2c_reset_hdle, I2C_HIGHT, "ctp_wakeup");
	gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");
	gpio_write_one_pin_value(gpio_i2c_sda_hdle, I2C_LOW, "twi1_sda");	

/*
	while(1){

		gpio_i2c_sda_info.mul_sel = 1;
		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);
//		gpio_set_one_pin_io_status(gpio_i2c_sda_hdle,1,"twi1_sda");
		gpio_write_one_pin_value(gpio_i2c_sda_hdle, I2C_LOW, "twi1_sda");	

//		msleep(2);
//		gpio_i2c_sda_info.mul_sel = 0;
//		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

//		reg_val = gpio_read_one_pin_value(gpio_i2c_sda_hdle, "twi1_sda");
//		printk("gpio read reg_val %d \n",reg_val);
		msleep(2000);

		gpio_write_one_pin_value(gpio_i2c_sda_hdle, I2C_HIGHT, "twi1_sda");	

		msleep(2000);

//		gpio_i2c_sda_info.mul_sel = 1;
//		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);
		gpio_set_one_pin_io_status(gpio_i2c_sda_hdle,1,"twi1_sda");

		gpio_write_one_pin_value(gpio_i2c_sda_hdle, I2C_HIGHT, "twi1_sda");	

		msleep(2);
		gpio_i2c_sda_info.mul_sel = 0;
		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

		reg_val = gpio_read_one_pin_value(gpio_i2c_sda_hdle, "twi1_sda");
		printk("gpio read reg_val %d \n",reg_val);		
		msleep(2000);
		
	}
*/
	
/*
    mt_set_gpio_mode(I2C_RST, RST_GPIO_MODE);
    mt_set_gpio_mode(I2C_SCL, SCL_GPIO_MODE);
    mt_set_gpio_mode(I2C_SDA, SDA_GPIO_MODE);
    mt_set_gpio_dir(I2C_RST, GPIO_DIR_OUT);
    mt_set_gpio_dir(I2C_SCL, GPIO_DIR_OUT);
    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);

    mt_set_gpio_pull_enable(I2C_RST, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(I2C_RST, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(I2C_SCL, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(I2C_SCL, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(I2C_SDA, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(I2C_SDA, GPIO_PULL_UP);

    mt_set_gpio_out(I2C_RST, GPIO_OUT_ONE);
    mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
    mt_set_gpio_out(I2C_SDA, GPIO_OUT_ZERO); 
*/
	
}

//long tpd_upgrader_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
//{
//    return 0;
//}

static long tpd_upgrader_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct file_operations tpd_upgrader_fops = {
    .owner   = THIS_MODULE,
//    .ioctl   = tpd_upgrader_ioctl,
	.unlocked_ioctl  = tpd_upgrader_ioctl,
    .open    = tpd_upgrader_open,
    .release = tpd_upgrader_release,
};

static struct miscdevice tpd_upgrader_dev = {
    .minor   = MISC_DYNAMIC_MINOR,
    .name    = "tpd-upgrader",
    .fops    = &tpd_upgrader_fops,
};

static int tpd_upgrader_open(struct inode *inode, struct file *file) 
{
    int ret;
    ret = nonseekable_open(inode, file);
    
    if (unlikely(ret)) 
        return ret;
    file->private_data = NULL;
    return 0;
}

static int tpd_upgrader_release(struct inode *ignored, struct file *file) 
{
    debug_printk("tpd_upgrader release\n");
    return 0;
}

static int __init tpd_upgrader_init(void)
{
    int i, retval;
    struct file *fp;
    unsigned char buffer[128];
    
    unsigned char bank_counter;
    unsigned int  block_counter;
    unsigned int  checksum_data;
    unsigned int  checksum_target;

    debug_printk("tpd_upgrader initialize\n");
    
    printk("\nMediaTek MT6573 Cypress Touch Panel Firmware Upgrade Tool\n\n");

    /* 
     * debug = 0 means turn off debug mode, debug = 1 means turn on debug mode
     * check = 0 means turn off check mode, check = 1 means turn on check mode
     */
    
    switch (param)
    {
        case 0:
            debug = 0;   
            check = 0;
            break;
        case 1:
            debug = 1;
            check = 0;  
            break;
        case 2:
            debug = 0;
            check = 1;  
            break;
        case 3:
            debug = 1;  
            check = 1;
            break;
        default:
            debug = 0;
            check = 0; 
    }    

    retval = misc_register(&tpd_upgrader_dev);
    if(unlikely(retval)) 
    {
        printk(KERN_ERR "failed to register misc device!\n");
        return retval;
    }

    tpd_upgrader_init_pin();

    printk("start downloading firmware.hex to the target\n\n");

    printk("stage 1: [");

    if (!tpd_initialize()) 
    {
        printk("\n");
        printk("tpd initializazion failed\n\n");
        return FAIL;
    }

    printk("]\n");
    printk("\ntpd initializazion success\n\n");

    printk("stage 2: [");

    if (!tpd_verify_id()) 
    { 
        printk("\n");
        printk("tpd verify id failed\n\n");
        tpd_restart_target();
        tpd_upgrader_deinit_pin();
        return FAIL;
    }

    printk("]\n");
    printk("\ntpd verify id success\n\n");

    printk("stage 3: [");
    if (!tpd_erase_target()) 
    {
        printk("\n");
        printk("tpd erase target failed\n\n");
        return FAIL;
    }
    printk("]\n");
    printk("\ntpd erase target success\n\n");

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(FIRMWARE_PATH, O_RDONLY, 0);
    if (!fp) {
        printk("\n");
        printk("can not open file [firmware.hex]\n");
        return FAIL;
    }

    printk("stage 4: [");
    checksum_data = 0;
    for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++) {
        for (block_counter = 0; block_counter < BLOCKS_PER_BANK; block_counter++) {
            tpd_load_program_data(bank_counter, (unsigned char)block_counter, fp);
            checksum_data += tpd_load_target();
            																		
            if (!tpd_program_target_block(bank_counter,(unsigned char)block_counter)) {
                printk("\n");
                printk("tpd_program_target_block failed\n\n");
                return FAIL;
            }

            if (!tpd_read_status()) {
                printk("\n");
                printk("tpd_read_status failed\n\n");
                return FAIL;
            }
            printk("#");
        }
    }
    printk("]\n");
    printk("\ntpd load and program target success\n\n");

    filp_close(fp, NULL);
    set_fs(old_fs);
    
    if (check) {
        old_fs = get_fs();
        set_fs(KERNEL_DS);
    
        fp = filp_open(FIRMWARE_PATH, O_RDONLY, 0);
        if (!fp)
        {
            printk("\n");
            printk("can not open file [firmware.hex]\n");
            return FAIL;
        }
        
        printk("stage 5: [");
        for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)
        {
            for (block_counter = 0; block_counter < BLOCKS_PER_BANK; block_counter++) 
            {
                tpd_load_program_data(bank_counter, (unsigned char)block_counter, fp);
    			
                if (!tpd_verify_setup(bank_counter,(unsigned char)block_counter)) 
                {
                    printk("\n");
                    printk("tpd_verify_setup fail\n");
                    return FAIL;
                }
    
                if (!tpd_read_status()) 
                { 
                    printk("\n");
                    printk("tpd_read_status fail\n");
                    return FAIL;
                }
    
                if (!tpd_read_byte_loop()) 
                {
                    printk("\n");
                    printk("tpd_read_byte_loop fail\n");
                    return FAIL;
                }
                printk("#");
            }
        }
        printk("]\n");
        printk("\ntpd load and verify target success\n\n");
    
        filp_close(fp, NULL);
        set_fs(old_fs);	
    }
    
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(FIRMWARE_PATH, O_RDONLY, 0);
    if (!fp)
    {
        printk("\n");
        printk("can not open file [firmware.hex]\n");
        return FAIL;
    }

    printk("stage 6: [");
    for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)
    {
        // Load one bank of security data from hex file into buffer
        if (!tpd_load_security_data(bank_counter, fp)) 
        {
            printk("\n");
            printk("tpd_load_security_data fail\n\n");
            return FAIL;
        }

        // Secure one bank of the target flash
        if (!tpd_secure_target_flash()) 
        {
            printk("\n");
            printk("tpd_secure_target_flash fail\n\n");
            return FAIL;
        }
        printk("#");
    }
    printk("]\n");
    printk("\ntpd load and secure target success\n\n");

    filp_close(fp, NULL);
    set_fs(old_fs);	
    
    if (check) {
        old_fs = get_fs();
        set_fs(KERNEL_DS);
    
        fp = filp_open(FIRMWARE_PATH, O_RDONLY, 0);
        if (!fp)
        {
            printk("\n");
            printk("can not open file [firmware.hex]\n");
            return FAIL;
        }
    
        printk("stage 7: [");
        if (!tpd_load_security_data(bank_counter, fp)) 
        {
    	    printk("\n");
    	    printk("tpd_load_security_data fail\n\n");
            return FAIL;
        }
    
        filp_close(fp, NULL);
        set_fs(old_fs);	
    
        if (!tpd_verify_security()) 
        {
            return FAIL;
        }
        printk("]\n");
        printk("\ntpd load and verify security success\n\n");
    }
    
    printk("stage 8: [");
    checksum_target = 0;
    for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)
    {
        if (!tpd_target_bank_checksum(&checksum_target)) 
        {
            printk("\n");
            printk("tpd_target_bank_checksum fail.\n\n");
            return FAIL;
        }
        printk("#");
    }

    debug_printk("checksum_target = %02x, checksum_data = %02x\n", checksum_target, (checksum_data & 0xFFFF));
    if (checksum_target != (checksum_data & 0xFFFF))
    {
        printk("\n");
        printk("checksum_target != checksum_data\n\n");
        return FAIL;
    }  
    printk("#");
    printk("]\n");
    printk("\ntpd checksum verify success\n\n");

    tpd_restart_target();

    tpd_upgrader_deinit_pin();

    printk("\n");
    printk("download firmware and configuration success.\n\n");

    return 0;
} 

static void __exit tpd_upgrader_exit(void) 
{
    int ret;
    
    debug_printk("tpd_upgrader exit\n");
    
    tpd_upgrader_deinit_pin();
    
    ret = misc_deregister(&tpd_upgrader_dev);
    if(unlikely(ret))
        debug_printk(KERN_ERR "failed to unregister misc device!\n");
}

module_init(tpd_upgrader_init);
module_exit(tpd_upgrader_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("MediaTek MT6573 Cypress Touch Panel Firmware Upgrader");
MODULE_AUTHOR("Chun-Wei Chen<chun-wei.chen@mediatek.com>");

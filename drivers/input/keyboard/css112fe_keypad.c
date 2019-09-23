/*
*  mma7660.c - Linux kernel modules for 3-Axis Orientation/Motion
*  Detection Sensor 
*
*  Copyright (C) 2009-2010 Freescale Semiconductor Hong Kong Ltd.
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/random.h>
#include <linux/input-polldev.h>
//add hxm
#include <linux/input.h>
#include <mach/sys_config.h>
#include "../../input/touchscreen/ctp_platform_ops.h"
#include <linux/suspend.h>

//#ifdef defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_PM)
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
	#include <linux/power/aw_pm.h>
//#endif

#define CSS_DEBUG 0

#ifdef CSS_DEBUG
#define printk_d(fmt, args...)  printk("[Css112fe] " "%s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##args)
#define printk_i(fmt, args...)  printk("[Css112fe] " "%s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#define printk_i(fmt, args...)
#endif

/*
* Defines
*/
int css112fe_init_hw(void);

#define Css112fe_DRV_NAME	"Css112fe"

#define Css112fe_ADDRESS   0x15 
//hxm add
static int gpio_reset_enable = 1;
static int gpio_reset_hdle = 0;

static user_gpio_set_t  gpio_int_info[1];
static int gpio_int_hdle = 0;
#define CTP_IRQ_NO			(gpio_int_info[0].port_num)
#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
static void* __iomem gpio_addr = NULL;
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};

struct Css112fe_data 
{
	char*  input_name;
};

struct work_struct Css112fe_work;	
static struct workqueue_struct *Css112fe_wq = NULL;

static struct input_dev *SN5ikbd_dev;

#define  SNKEY_MAX_CNT  		(64)

#define INPUT_SNDEV_NAME	("SN5i-keyboard")

static unsigned int SN_scankeycodes[SNKEY_MAX_CNT]=
{	
#ifdef CONFIG_ANDROID
	[0 ] = KEY_HOME,	/* Middle button */
	[1 ] = KEY_RIGHT,	/* Right button  */
	[2 ] = KEY_LEFT,	/* Left button   */
#else
	[0 ] = KEY_MENU,	/* Middle button */
	[1 ] = KEY_NEXT,	/* Right button  */
	[2 ] = KEY_BACK,	/* Left button   */
#endif
	[3 ] = KEY_RESERVED  ,//
	[4 ] = KEY_RESERVED,   
	[5 ] = KEY_RESERVED,//
	[6 ] = KEY_RESERVED,        
	[7 ] = KEY_RESERVED,
	[8 ] = KEY_RESERVED,
	[9 ] = KEY_RESERVED,
	[10] = KEY_RESERVED,
	[11] = KEY_RESERVED,
	[12] = KEY_RESERVED,
};
//end
static struct i2c_client *Css112fe_i2c_client;
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;

s32 css112fe_write_reg(int reg, u8 val)
{
	return i2c_smbus_write_byte_data(Css112fe_i2c_client,reg,val);
}


s32 css112fe_read_reg(u8 reg,u8 *val)
{
	(*val)=i2c_smbus_read_byte_data(Css112fe_i2c_client, reg);
	return *val;
}
static void ctp_clear_penirq(void)
{
	int reg_val;
	//clear the IRQ_EINT29 interrupt pending
	//printk_d("clear pend irq pending\n");
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
    //pr_info("888 reg_val=%d \n", reg_val);
	
	//writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	//writel(reg_val&(1<<(IRQ_EINT21)),gpio_addr + PIO_INT_STAT_OFFSET);
	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
		//printk_d("==CTP_IRQ_NO=\n");              
		writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	}
	//pr_info("999 reg_val=%d \n", reg_val);
	return;
}
static int  css112fe_read_slideValue(void)
{
     u8 value = 0;
//     printk_d("css112fe_read_slideValue\n");
     while(1)
     {
		css112fe_read_reg(0x21,&value);
		printk_d("value read = 0x%x,\n",value);
		if(value == 0x00)
		{
//			printk_d("Finger had remove:iCount=%d,iOldValue=0x%x\n",iCount,iOldValue);
			return 0;
	   	}
	   	else
	   	{
	   	   ///the finger still in press/move
	   	   ///we need record the first value

		   ////check the value and report the button
			if((value >= 0x1) && (value < 0x33))
			{
				///report the right button
				input_report_key(SN5ikbd_dev, SN_scankeycodes[2], 1);
				input_sync(SN5ikbd_dev);
				input_report_key(SN5ikbd_dev, SN_scankeycodes[2], 0);
				input_sync(SN5ikbd_dev);
			}
			else
			if((value >= 0x33) && (value < 0x65))
			{
				//report enter button
				input_report_key(SN5ikbd_dev, SN_scankeycodes[0], 1);
				input_sync(SN5ikbd_dev);
				input_report_key(SN5ikbd_dev, SN_scankeycodes[0], 0);
				input_sync(SN5ikbd_dev);
			}
			else 
			if((value >= 0x65) && (value <= 0xc8))
			{
				input_report_key(SN5ikbd_dev, SN_scankeycodes[1], 1);
				input_sync(SN5ikbd_dev);
				input_report_key(SN5ikbd_dev, SN_scankeycodes[1], 0);
				input_sync(SN5ikbd_dev);
			}
			break;
		}
	}
	msleep(100);	
    return 0;
}

static int css112fe_clear_irq(void)
{
	int ret=0;
	u8 value = 0;
	
	do{
	    ///clear the irq
		css112fe_write_reg(0x1f,0x01);
		msleep(100);
		///try to read the reg 0x1f
		ret=css112fe_read_reg(0x1f,&value);
	}while(value ==0x03);

	return ret;
}

static void Css112fe_touch_work_func(struct work_struct *work)
{ 
	css112fe_read_slideValue();
	css112fe_clear_irq();
}

static irqreturn_t Css112fe_EINT_Handler(int irq, void *dev_id)	
{
	int reg_val;	
	//printk_d("hxm @@@ SN_EINT_Handler\n");	
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO)))
	{	
		writel(reg_val&(1<<(CTP_IRQ_NO)),gpio_addr + PIO_INT_STAT_OFFSET);
		queue_work(Css112fe_wq, &Css112fe_work);
	}
	else
	{
	  //  printk_d("Other Interrupt\n");
	    return IRQ_NONE;
	}
	return IRQ_HANDLED;
}



static int Css112fe_set_irq_mode(char *major_key , char *subkey, ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	//config gpio to int mode
	pr_info("%s: config gpio to int mode. \n", __func__);
//#ifndef SYSCONFIG_GPIO_ENABLE
//#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		pr_info("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	printk_d("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
//#endif

#if 1 
	pr_info(" INTERRUPT CONFIG\n");
    gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	//pr_info("%s, gpio_addr = 0x%x. \n", __func__, gpio_addr);
	if(!gpio_addr) {
		ret = -EIO;
		//pr_info("4444 gpio_addr=%d \n", gpio_addr);
	}

	reg_num = (gpio_int_info[0].port_num)%8;
	
    pr_info("111 reg_num=%d,gpio_int_info[0].port_num=%d \n", reg_num,gpio_int_info[0].port_num);
	
	reg_addr = (gpio_int_info[0].port_num)/8;

    pr_info("222 reg_addr=%d,gpio_int_info[0].port_num=%d \n", reg_addr,gpio_int_info[0].port_num);
	
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);

       //pr_info("333reg_val=%d,gpio_addr=%d ,int_cfg_addr[reg_addr]=%d \n", reg_val,gpio_int_info[0].port_num,gpio_addr,int_cfg_addr[reg_addr]);
	
	reg_val &= (~(7 << (reg_num * 4)));

    pr_info("4444 reg_val=%d \n", reg_val);
	
	
	reg_val |= (int_mode << (reg_num * 4));

    pr_info("5555 reg_val=%d , int_mode=%d\n", reg_val,int_mode);
	
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);
                                                               
	ctp_clear_penirq();
                                                               
	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
	   pr_info("666 reg_val=%d \n", reg_val);
	reg_val |= (1 << (gpio_int_info[0].port_num));
	   pr_info("777 reg_val=%d \n", reg_val);
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;  
}

int css112fe_init_hw(void)
{
	int ret=0;
	u8 value = 0;
	printk_d("css112fe_init_hw\n");

	css112fe_write_reg(0xaf,0x3a);
	////write 0x00  0x0
	css112fe_write_reg(0x1f,0x0);
	css112fe_write_reg(0x01,0x01);
	css112fe_write_reg(0x02,0x02);
	css112fe_write_reg(0x03,0x03);
	css112fe_write_reg(0x04,0x04);
	
	////write 0x10  0x5
	css112fe_write_reg(0x10,0x5);

	/*
	Reg: ScanCfg @12H
	*/
	css112fe_write_reg(0x12,0x18);
	css112fe_write_reg(0x13,0x03);
    css112fe_write_reg(0x1e,0x17);

	do{
	    ///clear the irq
		css112fe_write_reg(0x1f,0x01);
		msleep(100);
		///try to read the reg 0x1f
		ret=css112fe_read_reg(0x1f,&value);
	}while(value == 0x03);


	return ret;
}
static int Css112fe=5;
static int __devinit Css112fe_probe(struct i2c_client *client,
								   const struct i2c_device_id *id)
{

	int err = -1;

	printk_d("%s %s:L%d \n",__FILE__,__func__,__LINE__);

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) 
	
	{
		printk_d(KERN_ERR "hxm @@@ i2c_check_functionality");		
		return -EIO;
	}
	Css112fe_i2c_client = client;

	Css112fe_wq = create_singlethread_workqueue("Css112fe_wq");	

    INIT_WORK(&Css112fe_work, Css112fe_touch_work_func);
	
	err = css112fe_init_hw();
	if(err < 0){
		goto exit_irq_request_failed;
	}

	err = Css112fe_set_irq_mode("Css112fe_para", "Css112fe_int_port", CTP_IRQ_MODE);
	if(0 != err){
		pr_info("%s:ctp_ops.set_irq_mode err. \n", __func__);

	}

	printk_d("%s %s:L%d \n",__FILE__,__func__,__LINE__);
    //err = request_irq(SW_INT_IRQNO_PIO, Css112fe_EINT_Handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_SHARED,"Css112fe", NULL); 
	err = request_irq(SW_INT_IRQNO_PIO, Css112fe_EINT_Handler, IRQF_TRIGGER_FALLING|IRQF_SHARED,"Css112fe", &Css112fe); 
	if (err < 0) 
	{
		printk_d("err : %d\n",err);
		dev_err(&client->dev, "Css112fe_probe: request irq failed..\n");
		goto exit_irq_request_failed;
	}

	return 0;
	
	exit_irq_request_failed:
	{
		dev_err(&client->dev, "Css112fe_probe:exit_irq_request_failed..\n");
		return err;
	}
}

static int __devexit Css112fe_remove(struct i2c_client *client)
{
	printk_d("%s %s %d \n",__FILE__,__func__,__LINE__); 
	return 0;
}

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
static int css112fe_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
 /*    if (g_suspend_state == PM_SUSPEND_PARTIAL){
         standby_wakeup_event |= SUSPEND_WAKEUP_SRC_PIO;
 */
        if ( (g_suspend_state != PM_SUSPEND_MEM) && (g_suspend_state != PM_SUSPEND_BOOTFAST) ) {
			standby_wakeup_event |= SUSPEND_WAKEUP_SRC_PIO;
     }
     return 0;
}


static int css112fe_i2c_resume(struct i2c_client *client)
{
     if (g_suspend_state == PM_SUSPEND_PARTIAL){
	 	
     } else{
	css112fe_init_hw();
	//queue_work(Css112fe_wq, &Css112fe_work);

     }
	return 0;
}
#endif

static const struct i2c_device_id Css112fe_id[] = {
	{ Css112fe_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, Css112fe_id);

static struct i2c_driver Css112fe_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= Css112fe_DRV_NAME,
		.owner	= THIS_MODULE,
	},
#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
	.suspend = css112fe_i2c_suspend,
	.resume	= css112fe_i2c_resume,
#endif
	.probe	= Css112fe_probe,
	.remove	= Css112fe_remove,
	.id_table = Css112fe_id,
	.address_list	= u_i2c_addr.normal_i2c,

};

int Css112fe_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	
	if(twi_id == adapter->nr)
	{
		pr_info("%s: hxm @@@ Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, Css112fe_DRV_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, Css112fe_DRV_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}

static int Css112fe_fetch_sysconfig_para(void)
{
	int ret = -1;
	int device_used = -1;
	__u32 twi_addr = 0;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
		
	printk_d("========%s=hxm @@@==================\n", __func__);
	 
	if(SCRIPT_PARSER_OK != (ret = script_parser_fetch("Css112fe_para", "Css112fe_used", &device_used, 1))){
	                pr_err("%s: hxm@@@ script_parser_fetch err.ret = %d. \n", __func__, ret);
	                goto script_parser_fetch_err;
	}
	if(1 == device_used){
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("Css112fe_para", "Css112fe_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
			pr_err("%s: line: %d hxm @@@script_parser_fetch err. \n", __func__, __LINE__);
			goto script_parser_fetch_err;
		}
		if(SCRIPT_PARSER_OK != script_parser_fetch("Css112fe_para", "Css112fe_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
			pr_err("%s: line: %d: hxm @@@ script_parser_fetch err. \n", name, __LINE__);
			goto script_parser_fetch_err;
		}
		u_i2c_addr.dirty_addr_buf[0] = twi_addr;
		u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;

		printk_d("%s: after: hxm @@@ Css112fe_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", \
			__func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);

		if(SCRIPT_PARSER_OK != script_parser_fetch("Css112fe_para", "Css112fe_twi_id", &twi_id, 1)){
			pr_err("%s: script_parser_fetch err. \n", name);
			goto script_parser_fetch_err;
		}
		
		printk_d("%s: twi_id is %d. \n", __func__, twi_id);

		ret = 0;
		
	}else{
		pr_err("%s: hxm @@@@Css112fe__unused. \n",  __func__);
		ret = -1;
	}

	return ret;

script_parser_fetch_err:
	pr_notice("=========hxm @@@ script_parser_fetch_err============\n");
	return ret;

}

static void Css112fe_reset(void)
{
	printk_d("Jim_dream SN7326_reset");
	if(gpio_reset_enable){

		gpio_reset_hdle = gpio_request_ex("Css112fe_para", "Css112fe_reset");
		if(!gpio_reset_hdle) {
			pr_warning("%s: tp_reset request gpio fail!\n", __func__);
			gpio_reset_enable = 0;
		}

		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 1, "Css112fe_reset")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(20);
		pr_info("%s. \n", __func__);

		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 0, "Css112fe_reset")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(20);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 1, "Css112fe_reset")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(20);

	}
}


static int __init Css112fe_init(void)
{
    int ret = -1;
	int i;

	if(Css112fe_fetch_sysconfig_para())
	{	
		printk_d("%s: hxm @@@err.\n", __func__);
		return -1;	
	}
	printk_d("%s %s %d \n",__FILE__,__func__,__LINE__); 
	
	printk_d("%s: hxm @@@@after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	//Css112fe_reset();

	Css112fe_driver.detect = Css112fe_detect;
	ret = i2c_add_driver(&Css112fe_driver);
	if (ret < 0) 
	{
		printk_d(KERN_INFO " hxm @@@ add Css112fe_init i2c driver failed\n");
		return -ENODEV;
	}
    SN5ikbd_dev = input_allocate_device();
	if (!SN5ikbd_dev) 
	{
		printk_d(KERN_ERR "snm5ikbd: not enough memory for input device\n");
		input_free_device(SN5ikbd_dev);
		return -ENOMEM;
	}

	SN5ikbd_dev->name = INPUT_SNDEV_NAME;  
	SN5ikbd_dev->phys = "sun4ikbd/input1"; 
	SN5ikbd_dev->id.bustype = BUS_HOST;      
	SN5ikbd_dev->id.vendor = 0x0001;
	SN5ikbd_dev->id.product = 0x0001;
	SN5ikbd_dev->id.version = 0x0100;

#ifdef REPORT_REPEAT_KEY_BY_INPUT_CORE
	SN5ikbd_dev->evbit[0] = BIT_MASK(EV_KEY)|BIT_MASK(EV_REP);
	printk_d("REPORT_REPEAT_KEY_BY_INPUT_CORE is defined, support report repeat key value. \n");
#else
	SN5ikbd_dev->evbit[0] = BIT_MASK(EV_KEY);
#endif

	for (i = 0; i < SNKEY_MAX_CNT; i++)
		set_bit(SN_scankeycodes[i], SN5ikbd_dev->keybit);

	ret = input_register_device(SN5ikbd_dev);
	if (ret)
	{		
		printk_d("input_register_device failure. \n");
	}
	printk_d(KERN_INFO " hxm @@@ add Css112fe_init i2c driver\n");
	return ret;
}


static void __exit Css112fe_exit(void)
{
	printk_d("%s %s %d \n",__FILE__,__func__,__LINE__); 
	i2c_del_driver(&Css112fe_driver);
}


MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CSS112FE driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(Css112fe_init);
module_exit(Css112fe_exit);

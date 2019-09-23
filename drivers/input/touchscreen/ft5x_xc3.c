/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd. 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 *  for this touchscreen to work, it's slave addr must be set to 0x7e | 0x70
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x_xc3.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#include <linux/power/aw_pm.h>
#endif

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>
#include "ctp_platform_ops.h"
#include <linux/suspend.h>
/* hxm del*/
#define FOR_TSLIB_TEST
#define PRINT_INT_INFO
#define PRINT_POINT_INFO
#define DEBUG

//#define TOUCH_KEY_SUPPORT
//xc3 addr is 0x24
#ifdef TOUCH_KEY_SUPPORT
//#define TOUCH_KEY_LIGHT_SUPPORT
//#define TOUCH_KEY_FOR_EVB13
//#define TOUCH_KEY_FOR_ANGDA
#ifdef TOUCH_KEY_FOR_ANGDA
#define TOUCH_KEY_X_LIMIT	(60000)
#define TOUCH_KEY_NUMBER	(4)
#endif
#ifdef TOUCH_KEY_FOR_EVB13
#define TOUCH_KEY_LOWER_X_LIMIT	(848)
#define TOUCH_KEY_HIGHER_X_LIMIT	(852)
#define TOUCH_KEY_NUMBER	(5)
#endif
#endif


struct i2c_dev{
struct list_head list;	
struct i2c_adapter *adap;
struct device *dev;
};

static struct class *i2c_dev_class;
static LIST_HEAD (i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);
#define FT5X_NAME	"ft5x_ts"
static struct i2c_client *this_client;
#ifdef TOUCH_KEY_LIGHT_SUPPORT
static int gpio_light_hdle = 0;
#endif
#ifdef TOUCH_KEY_SUPPORT
static int key_tp  = 0;
static int key_val = 0;
#endif

#ifdef PRINT_POINT_INFO 
#define print_point_info(fmt, args...)   \
        do{                              \
                pr_info(fmt, ##args);     \
        }while(0)
#else
#define print_point_info(fmt, args...)   //
#endif

#ifdef PRINT_INT_INFO 
#define print_int_info(fmt, args...)     \
        do{                              \
                pr_info(fmt, ##args);     \
        }while(0)
#else
#define print_int_info(fmt, args...)   //
#endif
///////////////////////////////////////////////
//specific tp related macro: need be configured for specific tp
#define CTP_IRQ_NO			(gpio_int_info[0].port_num)

#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
#define CTP_NAME			FT5X_NAME
#define TS_RESET_LOW_PERIOD		(10)
#define TS_INITIAL_HIGH_PERIOD		(30)
#define TS_WAKEUP_LOW_PERIOD	(20)
#define TS_WAKEUP_HIGH_PERIOD	(20)
#define TS_POLL_DELAY			(10)	/* ms delay between samples */
#define TS_POLL_PERIOD			(10)	/* ms delay between samples */
#define SCREEN_MAX_X			(screen_max_x)
#define SCREEN_MAX_Y			(screen_max_y)
#define PRESS_MAX			(255)


static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int gpio_reset_hdle = 0;
static int gpio_wakeup_enable = 1;
static int gpio_reset_enable = 1;
static user_gpio_set_t  gpio_int_info[1];

extern int main_encryptic(void);
extern int encrypt_nx_main(void);

static int screen_max_x = 0;
static int screen_max_y = 0;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;

/*
 * ctp_get_pendown_state  : get the int_line data state, 
 * 
 * return value:
 *             return PRESS_DOWN: if down
 *             return FREE_UP: if up,
 *             return 0: do not need process, equal free up.
 */
static int ctp_get_pendown_state(void)
{
	unsigned int reg_val;
	static int state = FREE_UP;

	//get the input port state
	reg_val = readl(gpio_addr + PIOH_DATA);
	//printk("reg_val = %x\n",reg_val);
	if(!(reg_val & (1<<CTP_IRQ_NO))){
		state = PRESS_DOWN;
		print_int_info("pen down. \n");
	}else{ //touch panel is free up
		state = FREE_UP;
		print_int_info("free up. \n");
	}
	return state;
}

/**
 * ctp_clear_penirq - clear int pending
 *
 */
static void ctp_clear_penirq(void)
{
	int reg_val;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);

	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
//		print_int_info("==CTP_IRQ_NO:%d=\n",CTP_IRQ_NO);              
		writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	}
	return;
}

/**
 * ctp_set_irq_mode - according sysconfig's subkey "ctp_int_port" to config int port.
 * 
 * return value: 
 *              0:      success;
 *              others: fail; 
 */
static int ctp_set_irq_mode(char *major_key , char *subkey, ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	//config gpio to int mode
	printk("%s: config gpio to int mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		printk("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	printk("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	printk(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);
                                                               
	ctp_clear_penirq();
                                                               
	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;  
}

/**
 * ctp_set_gpio_mode - according sysconfig's subkey "ctp_io_port" to config io port.
 *
 * return value: 
 *              0:      success;
 *              others: fail; 
 */
static int ctp_set_gpio_mode(void)
{
	//int reg_val;
	int ret = 0;
	//config gpio to io mode
	printk("%s: config gpio to io mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_io_port");
	if(!gpio_int_hdle){
		printk("request ctp_io_port failed. \n");
		ret = -1;
		goto request_tp_io_port_failed;
	}
#endif
	return ret;

request_tp_io_port_failed:
	return ret;
}

/**
 * ctp_judge_int_occur - whether interrupt occur.
 *
 * return value: 
 *              0:      int occur;
 *              others: no int occur; 
 */
static int ctp_judge_int_occur(void)
{
	//int reg_val[3];
	int reg_val;
	int ret = -1;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO))){
		ret = 0;
	}
	return ret; 	
}

/**
 * ctp_free_platform_resource - corresponding with ctp_init_platform_resource
 *
 */
static void ctp_free_platform_resource(void)
{
	if(gpio_addr){
		iounmap(gpio_addr);
	}
	
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	
	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}
	
	if(gpio_reset_hdle){
		gpio_release(gpio_reset_hdle, 2);
	}

	return;
}


/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO :  i/o err.
 *
 */
static int ctp_init_platform_resource(void)
{
	int ret = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	//printk("%s, gpio_addr = 0x%x. \n", __func__, gpio_addr);
	if(!gpio_addr) {
		ret = -EIO;
		goto exit_ioremap_failed;	
	}
	//    gpio_wakeup_enable = 1;
	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_wakeup_hdle) {
		pr_warning("%s: tp_wakeup request gpio fail!\n", __func__);
		gpio_wakeup_enable = 0;
	}

	gpio_reset_hdle = gpio_request_ex("ctp_para", "ctp_reset");
	if(!gpio_reset_hdle) {
		pr_warning("%s: tp_reset request gpio fail!\n", __func__);
		gpio_reset_enable = 0;
	}

	return ret;

exit_ioremap_failed:
	ctp_free_platform_resource();
	return ret;
}


/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];
	__u32 twi_addr = 0;
	//__u32 twi_id = 0;
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;

	printk("%s. \n", __func__);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(1 != ctp_used){
		pr_err("%s: ctp_unused. \n",  __func__);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(strcmp(CTP_NAME, name)){
		pr_err("%s: name %s does not match CTP_NAME. \n", __func__, name);
		pr_err(CTP_NAME);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	//big-endian or small-endian?
	//printk("%s: before: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	printk("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//printk("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	printk("%s: ctp_twi_id is %d. \n", __func__, twi_id);
	
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	printk("%s: screen_max_x = %d. \n", __func__, screen_max_x);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	printk("%s: screen_max_y = %d. \n", __func__, screen_max_y);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	printk("%s: revert_x_flag = %d. \n", __func__, revert_x_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	printk("%s: revert_y_flag = %d. \n", __func__, revert_y_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)){
		pr_err("ft5x_ts: script_parser_fetch err. \n");
		goto script_parser_fetch_err;
	}
	printk("%s: exchange_x_y_flag = %d. \n", __func__, exchange_x_y_flag);

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}

/**
 * ctp_reset - function
 *
 */
static void ctp_reset(void)
{
	if(gpio_reset_enable){
		printk("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 0, "ctp_reset")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_RESET_LOW_PERIOD); 
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 1, "ctp_reset")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_INITIAL_HIGH_PERIOD);
	}
}

/**
 * ctp_wakeup - function
 *
 */
static void ctp_wakeup(void)
{
	if(1 == gpio_wakeup_enable){  
		printk("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_HIGH_PERIOD);

	}
	return;
}
/**
 * ctp_detect - Device detection callback for automatic device creation
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		printk("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}
////////////////////////////////////////////////////////////////

static struct ctp_platform_ops ctp_ops = {
	.get_pendown_state = ctp_get_pendown_state,
	.clear_penirq	   = ctp_clear_penirq,
	.set_irq_mode      = ctp_set_irq_mode,
	.set_gpio_mode     = ctp_set_gpio_mode,	
	.judge_int_occur   = ctp_judge_int_occur,
	.init_platform_resource = ctp_init_platform_resource,
	.free_platform_resource = ctp_free_platform_resource,
	.fetch_sysconfig_para = ctp_fetch_sysconfig_para,
	.ts_reset =          ctp_reset,
	.ts_wakeup =         ctp_wakeup,
	.ts_detect = ctp_detect,
};

int fts_ctpm_fw_upgrade_with_i_file(void);

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	spin_lock(&i2c_dev_list_lock);
	
	list_for_each_entry(i2c_dev,&i2c_dev_list,list){
		printk("--line = %d ,i2c_dev->adapt->nr = %d,index = %d.\n",__LINE__,i2c_dev->adap->nr,index);
		if(i2c_dev->adap->nr == index){
		     goto found;
		}
	}
	i2c_dev = NULL;
	
found: 
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev ;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap) 
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS){
		printk("i2c-dev:out of device minors (%d) \n",adap->nr);
		return ERR_PTR (-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev){
		return ERR_PTR(-ENOMEM);
	}
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev;
}


static int ft5x_i2c_rxdata(char *rxdata, int length);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
	s16 touch_ID1;
	s16 touch_ID2;
	s16 touch_ID3;
	s16 touch_ID4;
	s16 touch_ID5;
    u8  touch_point;
};

struct ft5x_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};


/* ---------------------------------------------------------------------
*
*   Focal Touch panel upgrade related driver
*
*
----------------------------------------------------------------------*/

typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit 

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS        (0x70>>1)

static void delay_ms(FTS_WORD  w_ms)
{
	//platform related, please implement this function
	msleep( w_ms );
}

/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_read_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;

	ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret != dw_lenth){
		printk("ret = %d. \n", ret);
		printk("i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_write_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret != dw_lenth){
		printk("i2c_write_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}


/***************************************************************************************/

/*
[function]: 
    read out the register value.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[out]    :the returned register value;
    bt_len[in]        :length of pbt_buf, should be set to 2;        
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
u8 fts_register_read(u8 e_reg_name, u8* pbt_buf, u8 bt_len)
{
	u8 read_cmd[3]= {0};
	u8 cmd_len     = 0;

	read_cmd[0] = e_reg_name;
	cmd_len = 1;    

	/*call the write callback function*/
	//    if(!i2c_write_interface(I2C_CTPM_ADDRESS, &read_cmd, cmd_len))
	//    {
	//        return FTS_FALSE;
	//    }


	if(!i2c_write_interface(I2C_CTPM_ADDRESS, read_cmd, cmd_len))	{//change by zhengdixu
		return FTS_FALSE;
	}

	/*call the read callback function to get the register value*/        
	if(!i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len)){
		return FTS_FALSE;
	}
	return FTS_TRUE;
}

/*
*ft5x02_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft5x02_i2c_Read(struct i2c_client *client,  char * writebuf, int writelen, 
							char *readbuf, int readlen)
{
	int ret;

	if(writelen > 0)
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= client->addr,
				.flags	= 0,
				.len	= writelen,
				.buf	= writebuf,
			},
			{
				.addr	= client->addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			pr_err("function:%s. i2c read error: %d\n", __func__, ret);
	}
	else
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= client->addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("function:%s. i2c read error: %d\n", __func__, ret);
	}
	return ret;
}
/*
*write data by i2c 
*/
int ft5x02_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= writelen,
			.buf	= writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}


int ft5x02_write_reg(struct i2c_client * client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft5x02_i2c_Write(client, buf, sizeof(buf));
}
EXPORT_SYMBOL(ft5x02_write_reg);

int ft5x02_read_reg(struct i2c_client * client, u8 regaddr, u8 * regvalue)
{
	return ft5x02_i2c_Read(client, &regaddr, 1, regvalue, 1);
}
EXPORT_SYMBOL(ft5x02_read_reg);

int fts_register_write(u8 e_reg_name, u8 bt_value)
{
	FTS_BYTE write_cmd[2] = {0};

	write_cmd[0] = e_reg_name;
	write_cmd[1] = bt_value;

	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, 2); 
}


int cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);//change by zhengdixu
}

int byte_write(u8* pbt_buf, u16 dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

int byte_read(u8* pbt_buf, u8 bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

#define    FTS_PACKET_LENGTH       128 //2//4//8//16//32//64//128//256

static unsigned char CTPM_FW[]=
{
#include "ft_app.i"
};
unsigned char fts_ctpm_get_i_file_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, u16 dw_lenth)
{
    u8 reg_val[2] = {0};
    FTS_BOOL i_ret = 0;
    u16 i = 0;
    

    u16  packet_number;
    u16  j;
    u16  temp;
    u16  lenght;
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8  auc_i2c_write_buf[10];
    u8 bt_ecc;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    delay_ms(100);
    fts_register_write(0xfc,0xaa);
    delay_ms(50);
     /*write 0x55 to register 0xfc*/
    fts_register_write(0xfc,0x55);
    printk("Step 1: Reset CTPM test\n");

    delay_ms(30);

    /*********Step 2:Enter upgrade mode *****/
     auc_i2c_write_buf[0] = 0x55;
     auc_i2c_write_buf[1] = 0xaa;
     i = 0;
     do{
        i++;
        i_ret = i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
        printk("Step 2: Enter update mode. \n");
        delay_ms(5);
     }while((FTS_FALSE == i_ret) && i<5);

    /*********Step 3:check READ-ID***********************/
    /*send the opration head*/
    i = 0;
    do{
        if(i > 3)
        {
          cmd_write(0x07,0x00,0x00,0x00,1);
		  return ERR_READID; 
        }
        /*read out the CTPM ID*/
        printk("====Step 3:check READ-ID====");
        cmd_write(0x90,0x00,0x00,0x00,4);
        byte_read(reg_val,2);
        i++;
        delay_ms(5);
        printk("Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }while((reg_val[1] != 0x03)&&(reg_val[1] != 0x06));//while(reg_val[0] != 0x79 || reg_val[1] != 0x03);

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);
    delay_ms(1500);
    printk("Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        delay_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        delay_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    //cmd_write(0xcc,0x00,0x00,0x00,1);//把0xcc当作寄存器地址，去读出一个字节
   // byte_read(reg_val,1);//change by zhengdixu

	fts_register_read(0xcc, reg_val,1);
	
    printk("Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
       cmd_write(0x07,0x00,0x00,0x00,1);
		return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);
    msleep(30);
    return ERR_OK;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    fts_register_write(0, 0x40);  
    delay_ms(100);                       //make sure already enter factory mode
    fts_register_write(2, 0x4);               //write command to start calibration
    delay_ms(300);
    for(i=0;i<100;i++)
    {
        fts_register_read(0,&uc_temp,1);
        if ( ((uc_temp&0x70)>>4) == 0x0)    //return to normal mode, calibration finish
        {
            break;
        }
        delay_ms(200);
        printk("[FTS] waiting calibration %d\n",i);
    }
    
    printk("[FTS] calibration OK.\n");
    
    msleep(300);
    fts_register_write(0, 0x40);              //goto factory mode
    delay_ms(100);                       //make sure already enter factory mode
   fts_register_write(2, 0x5);               //store CLB result
    delay_ms(300);
    fts_register_write(0, 0x0);               //return to normal mode 
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}
void getVerNo(u8* buf, int len)
{
	u8 start_reg=0x0;
	int ret = -1;
	//int status = 0;
	int i = 0;
	start_reg = 0xa6;

#if 0
	printk("read 0xa6 one time. \n");
	if(FTS_FALSE == fts_register_read(0xa6, buf, len)){
        return ;
	}
	
	for (i=0; i< len; i++) 
	{
		printk("=========buf[%d] = 0x%x \n", i, buf[i]);
	}
	
	printk("read 0xa8. \n");
	if(FTS_FALSE == fts_register_read(0xa8, buf, len)){
        return ;
	}
	for (i=0; i< len; i++) 
	{
		printk("=========buf[%d] = 0x%x \n", i, buf[i]);
	}

	ft5x_i2c_rxdata(buf, len);
	
    for (i=0; i< len; i++) 
        {
            printk("=========buf[%d] = 0x%x \n", i, buf[i]);
        }

    byte_read(buf, len);
    for (i=0; i< len; i++) 
    {
        printk("=========buf[%d] = 0x%x \n", i, buf[i]);
    }
          
#endif

	ret =fts_register_read(0xa6, buf, len);
	//et = ft5406_read_regs(ft5x0x_ts_data_test->client,start_reg, buf, 2);
	if (ret < 0) 
	{
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return;
	}
	for (i=0; i<2; i++) 
	{
		printk("=========buf[%d] = 0x%x \n", i, buf[i]);
	}


	return;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret = 0;
	unsigned char a;
	unsigned char b;
#define BUFFER_LEN (2)            //len == 2 
	unsigned char buf[BUFFER_LEN] = {0};
   
	//=========FW upgrade========================*/
	printk("%s. \n", __func__);

	pbt_buf = CTPM_FW;
	msleep(100);
	getVerNo(buf, BUFFER_LEN);
	a = buf[0];
	b = fts_ctpm_get_i_file_ver();
	printk("a == %hu,  b== %hu \n",a, b);

	/*
	  * when the firmware in touch panel maybe corrupted,
	  * or the firmware in host flash is new, need upgrade
	  */
	if ( 0xa6 == a ||a < b ){
		/*call the upgrade function*/
		i_ret =  fts_ctpm_fw_upgrade(&pbt_buf[0],sizeof(CTPM_FW));
		if (i_ret != 0){
			printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
		} else {
			printk("[FTS] upgrade successfully.\n");
			fts_ctpm_auto_clb();  //start auto CLB
		}

	}
	
	return i_ret;
	
}

unsigned char fts_ctpm_get_upg_ver(void)
{
	unsigned int ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2){
		return CTPM_FW[0];
	}
	else{
		//TBD, error handling?
		return 0xff; //default value
	}
}

static int ft5x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

        //printk("IIC add = %x\n",this_client->addr);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		printk("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x_set_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static void ft5x_ts_release(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
#ifdef CONFIG_FT5X0X_MULTITOUCH	
#ifdef TOUCH_KEY_SUPPORT
	if(1 == key_tp){
		input_report_key(data->input_dev, key_val, 0);
		print_point_info("Release Key = %d\n",key_val);		
	} else{
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	}
#else
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#endif

#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	
	input_sync(data->input_dev);
	return;

}

static int ft5x_read_data(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	unsigned char buf[32]={0};
	int ret = -1;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x_i2c_rxdata(buf, 31);
#else
	ret = ft5x_i2c_rxdata(buf, 31);
#endif
	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = buf[2] & 0x07;// 000 0111
	//print_point_info("touch point = %d\n",event->touch_point);


	if (event->touch_point == 0) {
		ft5x_ts_release();
		return 1; 
	}

#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
	case 2:
		event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
		event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
		//printk("case 22222 @@@befor22222222222 befor swap: event->x2 = %d, event->y2 = %d,event->touch_point=%d. \n", event->x2, event->y2,event->touch_point);
		if(1 == exchange_x_y_flag){
			swap(event->x2, event->y2);
		}
		if(1 == revert_x_flag){
			event->x2 = SCREEN_MAX_X - event->x2;
		}
		if(1 == revert_y_flag){
			event->y2 = SCREEN_MAX_Y - event->y2;
		}
		//printk("before swap: event->x2 = %d, event->y2 = %d. \n", event->x2, event->y2);
		
		//event->touch_ID2=(s16)(buf[0x0b] & 0xF0)>>4;
		//printk("case 22222 @@@1222222222222 after swap: event->x2 = %d, event->y2 = %d,event->touch_point=%d,event->touch_ID2=%d \n", event->x2, event->y2,event->touch_point,event->touch_ID2);
	case 1:
		event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
		event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
		//printk("case 11111 @@@111111 befor swap: event->x1 = %d, event->y1 = %d,event->touch_point=%d .\n", event->x1, event->y1,event->touch_point);
#ifdef TOUCH_KEY_FOR_ANGDA
		if(event->x1 < TOUCH_KEY_X_LIMIT)
		{
			if(1 == revert_x_flag){
				event->x1 = SCREEN_MAX_X - event->x1;
			}
			if(1 == revert_y_flag){
				event->y1 = SCREEN_MAX_Y - event->y1;
			}
			//printk("before swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
			if(1 == exchange_x_y_flag){
				swap(event->x1, event->y1);
			}
		}
#elif defined(TOUCH_KEY_FOR_EVB13)
		if((event->x1 > TOUCH_KEY_LOWER_X_LIMIT)&&(event->x1<TOUCH_KEY_HIGHER_X_LIMIT))
		{

			//printk("before swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
			if(1 == exchange_x_y_flag){
				swap(event->x1, event->y1);
			}
			if(1 == revert_x_flag){
				event->x1 = SCREEN_MAX_X - event->x1;
			}
			if(1 == revert_y_flag){
				event->y1 = SCREEN_MAX_Y - event->y1;
			}
		}
#else

		//printk("before swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
		if(1 == exchange_x_y_flag){
			swap(event->x1, event->y1);
		}
		if(1 == revert_x_flag){
			event->x1 = SCREEN_MAX_X - event->x1;
		}
		if(1 == revert_y_flag){
			event->y1 = SCREEN_MAX_Y - event->y1;
		}
#endif

		// printk("after swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
             
		event->touch_ID1=(s16)(buf[0x05] & 0xF0)>>4;
             //printk("case 11111 @@@ 1111111 after swap: event->x1 = %d, event->y1 = %d,event->touch_point=%d ,event->touch_ID1=%d.\n", event->x1, event->y1,event->touch_point,event->touch_ID1); 
		break;
	default:
		return -1;
	}
#else
	if (event->touch_point == 1) {
		event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
		event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
	}
#endif
	event->pressure = 200;

	//dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
	//event->x1, event->y1, event->x2, event->y2);


    return 0;
}

#ifdef TOUCH_KEY_LIGHT_SUPPORT
static void ft5x_lighting(void)
{
	if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_light_hdle, 1, "ctp_light")){
		printk("ft5x_ts_light: err when operate gpio. \n");
	}    
	msleep(15);
	if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_light_hdle, 0, "ctp_light")){
		printk("ft5x_ts_light: err when operate gpio. \n");
	}         

	return;
}
#endif

static void ft5x_report_multitouch(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

#ifdef TOUCH_KEY_SUPPORT
	if(1 == key_tp){
		return;
	}
#endif


	switch(event->touch_point) {
	case 2:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,2);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		printk("===x2 = %d,y2 = %d ,event->touch_ID2=%d===\n",event->x2,event->y2,event->touch_ID2);
	case 1:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 1);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		printk("===x1 = %d,y1 = %d ,event->touch_ID1=%d===\n",event->x1,event->y1,event->touch_ID1);
		break;
	default:
		printk("==touch_point default =\n");
		break;
	}
	
	input_sync(data->input_dev);
	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
	return;
}

#ifndef CONFIG_FT5X0X_MULTITOUCH
static void ft5x_report_singletouch(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
	input_sync(data->input_dev);
	printk(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);

	return;
}
#endif

#ifdef TOUCH_KEY_SUPPORT
static void ft5x_report_touchkey(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	//print_point_info("x=%d===Y=%d\n",event->x1,event->y1);

#ifdef TOUCH_KEY_FOR_ANGDA
	if((1==event->touch_point)&&(event->x1 > TOUCH_KEY_X_LIMIT)){
		key_tp = 1;
		if(event->y1 < 40){
			key_val = 1;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);  
			print_point_info("===KEY 1====\n");
		}else if(event->y1 < 90){
			key_val = 2;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			print_point_info("===KEY 2 ====\n");
		}else{
			key_val = 3;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			print_point_info("===KEY 3====\n");	
		}
	} else{
		key_tp = 0;
	}
#endif
#ifdef TOUCH_KEY_FOR_EVB13
	if((1==event->touch_point)&&((event->x1 > TOUCH_KEY_LOWER_X_LIMIT)&&(event->x1<TOUCH_KEY_HIGHER_X_LIMIT))){
		key_tp = 1;
		if(event->y1 < 5){
			key_val = 1;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);  
			print_point_info("===KEY 1====\n");     
		}else if((event->y1 < 45)&&(event->y1>35)){
			key_val = 2;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			print_point_info("===KEY 2 ====\n");
		}else if((event->y1 < 75)&&(event->y1>65)){
			key_val = 3;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			print_point_info("===KEY 3====\n");
		}else if ((event->y1 < 105)&&(event->y1>95))	{
			key_val = 4;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			print_point_info("===KEY 4====\n");	
		}
	}else{
		key_tp = 0;
	}
#endif

#ifdef TOUCH_KEY_LIGHT_SUPPORT
	ft5x_lighting();
#endif
	return;
}
#endif

static void ft5x_report_value(void)
{

	//printk("==ft5x_report_value =\n");
#ifdef TOUCH_KEY_SUPPORT
	ft5x_report_touchkey();
#endif

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ft5x_report_multitouch();
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	ft5x_report_singletouch();
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	return;
}	

static void ft5x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	//printk("==ft5x_ts_pen_irq_work work 1=\n");
	ret = ft5x_read_data();
	if (ret == 0) {
		ft5x_report_value();
	}

}

static irqreturn_t ft5x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x_ts_data *ft5x_ts = dev_id;
		
//	printk("==========------ft5x_ts TS Interrupt-----============\n"); 
	if(!ctp_ops.judge_int_occur()){
		//print_int_info("==IRQ_EINT%d=\n",CTP_IRQ_NO);
		ctp_ops.clear_penirq();
		if (!work_pending(&ft5x_ts->pen_event_work)) 
		{
			//print_int_info("Enter work\n");
			queue_work(ft5x_ts->ts_workqueue, &ft5x_ts->pen_event_work);
		}
	}else{
		print_int_info("Other Interrupt\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void ft5x_ts_suspend(struct early_suspend *handler)
{
    printk("ft5x_ts_suspend: write FT5X0X_REG_PMODE .\n");
	if (g_suspend_state != PM_SUSPEND_PARTIAL){
	msleep(20); 
    ft5x_set_reg(0x00,0x02);
	msleep(10); 
	} else {
		standby_wakeup_event |= SUSPEND_WAKEUP_SRC_PIO;
	}
	
}

static void ft5x_ts_resume(struct early_suspend *handler)
{
	printk("==ft5x_ts_resume==g_suspend_state=%d \n",g_suspend_state);
	if (g_suspend_state != PM_SUSPEND_PARTIAL) 
	{
	//reset
	ctp_ops.ts_reset();
	}
	

}
#endif  //CONFIG_HAS_EARLYSUSPEND

static int ft5x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x_ts_data *ft5x_ts;
	struct input_dev *input_dev;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int err = 0;

#ifdef TOUCH_KEY_SUPPORT
	int i = 0;
#endif

	printk("====%s begin=====.  \n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}


	ft5x_ts = kzalloc(sizeof(*ft5x_ts), GFP_KERNEL);
	if (!ft5x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	this_client->addr = client->addr;
	i2c_set_clientdata(client, ft5x_ts);

#ifdef TOUCH_KEY_LIGHT_SUPPORT
	gpio_light_hdle = gpio_request_ex("ctp_para", "ctp_light");
#endif

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	fts_ctpm_fw_upgrade_with_i_file();
#endif


//	printk("==INIT_WORK=\n");
	INIT_WORK(&ft5x_ts->pen_event_work, ft5x_ts_pen_irq_work);
	ft5x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x_ts->input_dev = input_dev;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);	
#ifdef FOR_TSLIB_TEST
	set_bit(BTN_TOUCH, input_dev->keybit);
#endif
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, 4, 0, 0);
#ifdef TOUCH_KEY_SUPPORT
	key_tp = 0;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	for (i = 1; i < TOUCH_KEY_NUMBER; i++)
		set_bit(i, input_dev->keybit);
#endif
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name		= CTP_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
//hxm add
#ifdef CONFIG_HAS_EARLYSUSPEND	
	printk("==register_early_suspend =\n");	
	ft5x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;	
	ft5x_ts->early_suspend.suspend = ft5x_ts_suspend;
	ft5x_ts->early_suspend.resume	= ft5x_ts_resume;	
	register_early_suspend(&ft5x_ts->early_suspend);
#endif
//end	

#ifdef CONFIG_FT5X0X_MULTITOUCH
	printk("CONFIG_FT5X0X_MULTITOUCH is defined. \n");
#endif
	err = ctp_ops.set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);
	if(0 != err){
		printk("%s:ctp_ops.set_irq_mode err. \n", __func__);
		goto exit_set_irq_mode;
	}
	//err = request_irq(SW_INT_IRQNO_PIO, ft5x_ts_interrupt,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_SHARED, "ft5x_ts", ft5x_ts);
    err = request_irq(SW_INT_IRQNO_PIO, ft5x_ts_interrupt,IRQF_TRIGGER_FALLING|IRQF_SHARED, "ft5x_ts", ft5x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x_ts_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

    	i2c_dev = get_free_i2c_dev(client->adapter);	
	if (IS_ERR(i2c_dev)){	
		err = PTR_ERR(i2c_dev);		
		return err;	
	}
	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,client->adapter->nr), NULL, "aw_i2c_ts%d", client->adapter->nr);	
	if (IS_ERR(dev))	{		
			err = PTR_ERR(dev);		
			return err;	
	}

	printk("==%s over =\n", __func__);
	return 0;

exit_irq_request_failed:
exit_set_irq_mode:
	cancel_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
	enable_irq(SW_INT_IRQNO_PIO);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(SW_INT_IRQNO_PIO, ft5x_ts);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ft5x_ts_remove(struct i2c_client *client)
{

	struct ft5x_ts_data *ft5x_ts = i2c_get_clientdata(client);
	ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	
	printk("==ft5x_ts_remove=\n");
	free_irq(SW_INT_IRQNO_PIO, ft5x_ts);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x_ts->early_suspend);
#endif
	input_unregister_device(ft5x_ts->input_dev);
	input_free_device(ft5x_ts->input_dev);
	cancel_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
	kfree(ft5x_ts);
    
	i2c_set_clientdata(client, NULL);
	ctp_ops.free_platform_resource();

	return 0;

}

static const struct i2c_device_id ft5x_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ft5x_ts_id);

static struct i2c_driver ft5x_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= ft5x_ts_probe,
	.remove		= __devexit_p(ft5x_ts_remove),
	//.suspend = ft5x_ts_suspend,
	//.resume  = ft5x_ts_resume,
	.id_table	= ft5x_ts_id,
	.driver	= {
		.name	= CTP_NAME,
		.owner	= THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
};

static int aw_open(struct inode *inode, struct file *file)
{
	int subminor;
	int ret = 0;	
	struct i2c_client *client;
	struct i2c_adapter *adapter;	
	struct i2c_dev *i2c_dev;	

	printk("====%s======.\n", __func__);
	
	#ifdef AW_DEBUG	
	        printk("enter aw_open function\n");
	#endif	
	
	subminor = iminor(inode);
	#ifdef AW_DEBUG	
	      printk("subminor=%d\n",subminor);
	#endif	
	
	//lock_kernel();	
	i2c_dev = i2c_dev_get_by_minor(2);	
	if (!i2c_dev)	{	
		printk("error i2c_dev\n");		
		return -ENODEV;	
	}
	
	adapter = i2c_get_adapter(i2c_dev->adap->nr);	
	if (!adapter)	{		
		return -ENODEV;	
	}	
	
	client = kzalloc(sizeof(*client), GFP_KERNEL);	
	
	if (!client)	{		
		i2c_put_adapter(adapter);		
		ret = -ENOMEM;	
	}	
	snprintf(client->name, I2C_NAME_SIZE, "pctp_i2c_ts%d", adapter->nr);
	client->driver = &ft5x_ts_driver;
	client->adapter = adapter;		
	file->private_data = client;
		
	return 0;
}

static long aw_ioctl(struct file *file, unsigned int cmd,unsigned long arg ) 
{
	//struct i2c_client *client = (struct i2c_client *) file->private_data;

	printk("====%s====.\n",__func__);

	#ifdef AW_DEBUG
	       printk("line :%d,cmd = %d,arg = %d.\n",__LINE__,cmd,arg);
	#endif
	
	switch (cmd) {
		case UPGRADE:
		printk("==UPGRADE_WORK=\n");
		fts_ctpm_fw_upgrade_with_i_file();
		// calibrate();
		
		break;
		
		default:
		break;
			 
	}
	
	return 0;
}

static int aw_release (struct inode *inode, struct file *file) 
{
	struct i2c_client *client = file->private_data;
	#ifdef AW_DEBUG
	    printk("enter aw_release function.\n");
	#endif
	
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;
	return 0;	  
}

static const struct file_operations aw_i2c_ts_fops ={	
	.owner = THIS_MODULE, 	
	//.read = aw_read, 
	//.write = aw_write,	
	.open = aw_open, 	
	.unlocked_ioctl = aw_ioctl,	
	.release = aw_release, 
};

static int __init ft5x_ts_init(void)
{ 
	int ret = -1;
	int err = -1;

	printk("===========================%s=====================\n", __func__);

	// if(main_encryptic()!=0 && encrypt_nx_main()!=0)
	// {
	//   printk("encrypt Ic error");
	//   while(1);
	// }

	if (ctp_ops.fetch_sysconfig_para)
	{
		if(ctp_ops.fetch_sysconfig_para()){
			printk("%s: err.\n", __func__);
			return -1;
		}
	}
	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	err = ctp_ops.init_platform_resource();
	if(0 != err){
	    printk("%s:ctp_ops.init_platform_resource err. \n", __func__);    
	}

	//reset
	ctp_ops.ts_reset();
	//wakeup
	//ctp_ops.ts_wakeup();  
	
	ft5x_ts_driver.detect = ctp_ops.ts_detect;

	ret= register_chrdev(I2C_MAJOR,"aw_i2c_ts",&aw_i2c_ts_fops );	
	if(ret) {	
		printk(KERN_ERR "%s:register chrdev failed\n",__FILE__);	
		return ret;
	}
	
	i2c_dev_class = class_create(THIS_MODULE,"aw_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {		
		ret = PTR_ERR(i2c_dev_class);		
		class_destroy(i2c_dev_class);	
	}

	ret = i2c_add_driver(&ft5x_ts_driver);
	return ret;
}

static void __exit ft5x_ts_exit(void)
{
	printk("==ft5x_ts_exit==\n");
	i2c_del_driver(&ft5x_ts_driver);
}

late_initcall(ft5x_ts_init);
module_exit(ft5x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x TouchScreen driver");
MODULE_LICENSE("GPL");


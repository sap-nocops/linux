#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/pwm_backlight.h>
#include <linux/memblock.h>
//#include <linux/gpio.h>
#include <mach/sys_config.h>
#include <linux/cyttsp4_bus.h>
#include <linux/cyttsp4_core.h>
#include <linux/cyttsp4_mt.h>


#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_INCLUDE_FW
#include "cyttsp4_img.h"
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_AUTO_LOAD_TOUCH_PARAMS
#include "cyttsp4_params.h"
static struct touch_settings cyttsp4_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};
#else
static struct touch_settings cyttsp4_sett_param_regs = {
	.data = NULL,
	.size = 0,
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = NULL,
	.size = 0,
	.tag = 0,
};
#endif

static struct cyttsp4_loader_platform_data _cyttsp4_loader_platform_data = {
	.fw = &cyttsp4_firmware,
	.param_regs = &cyttsp4_sett_param_regs,
	.param_size = &cyttsp4_sett_param_size,
	.flags = CY_FLAG_AUTO_CALIBRATE,
};

#define CYTTSP4_USE_I2C
/* #define CYTTSP4_USE_SPI */


#ifdef CYTTSP4_USE_I2C
#define CYTTSP4_I2C_NAME "cyttsp4_i2c_adapter"
#define CYTTSP4_I2C_TCH_ADR 0x24
#define CYTTSP4_LDR_TCH_ADR 0x24
#endif

#define CY_MAXX 758
#define CY_MAXY 1024
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

#define CY_IGNORE_VALUE 0xFFFF

static int cyttsp4_xres(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;

	if (EGPIO_SUCCESS != gpio_write_one_pin_value(rst_gpio, 1, "ctp_reset"))
		printk("%s: err when operate gpio. \n", __func__);

	mdelay(20);
	if (EGPIO_SUCCESS != gpio_write_one_pin_value(rst_gpio, 0, "ctp_reset"))
		printk("%s: err when operate gpio. \n", __func__);

	mdelay(40);

	if (EGPIO_SUCCESS != gpio_write_one_pin_value(rst_gpio, 1, "ctp_reset"))
		printk("%s: err when operate gpio. \n", __func__);

	mdelay(20);

	return rc;
}

static int cyttsp4_init(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (on) {
		rc = gpio_request_ex("ctp_para", "ctp_reset");

		if (!rc ) {
			dev_err(dev,
				"%s: Fail request rst_gpio=%d\n", __func__,
				rst_gpio);
		} else {
			pdata->rst_gpio = rc;
			rc = gpio_set_one_pin_io_status(pdata->rst_gpio,1,"ctp_reset");
			rc = gpio_write_one_pin_value(pdata->rst_gpio, 1, "ctp_reset");

			if (rc != EGPIO_SUCCESS ) {
				pr_err("%s: Fail set output rst_gpio=%d\n",
					__func__, rst_gpio);
			} else {
				rc = gpio_request_ex("ctp_para", "ctp_int_port");

				if (!rc) {
					dev_err(dev,
						"%s: Fail request irq_gpio=%d\n",
						__func__, irq_gpio);
				} else {
					pdata->irq_gpio = rc;
					rc = gpio_set_one_pin_io_status(pdata->irq_gpio,0,"ctp_int_port");
				}
			}
		}
	} else {
		gpio_release(pdata->irq_gpio,0);
		gpio_release(pdata->rst_gpio,0);

	}
	dev_dbg(dev,
		"%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}

static int cyttsp4_wakeup(struct cyttsp4_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;


	if (ignore_irq)
		atomic_set(ignore_irq, 1);

	rc = gpio_set_one_pin_io_status(pdata->rst_gpio,1,"ctp_int_port");
	rc = gpio_write_one_pin_value(pdata->rst_gpio, 0, "ctp_int_port");

	if (!rc) {
		dev_err(dev,
			"%s: Fail set output gpio=%d\n",
			__func__, irq_gpio);
	} else {

		mdelay(2);
		rc = gpio_set_one_pin_io_status(pdata->irq_gpio,0,"ctp_int_port");
		if (!rc) {

			dev_err(dev,
				"%s: Fail set input gpio=%d\n",
				__func__, irq_gpio);
		}
	}

	if (ignore_irq)
		atomic_set(ignore_irq, 0);
	
	msleep(10);
	
	dev_dbg(dev,
		"%s: WAKEUP CYTTSP gpio=%d r=%d\n", __func__,
		irq_gpio, rc);

	return rc;
}

static int cyttsp4_sleep(struct cyttsp4_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

static int cyttsp4_power(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp4_wakeup(pdata, dev, ignore_irq);

	return cyttsp4_sleep(pdata, dev, ignore_irq);
}

static int cyttsp4_irq_stat(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
	return 0; 
}


#define PIO_INT_CFG0_OFFSET 	(0x200)
#define PIO_INT_CFG1_OFFSET 	(0x204)
#define PIO_INT_CFG2_OFFSET 	(0x208)
#define PIO_INT_CFG3_OFFSET 	(0x20c)
#define PIO_INT_CTRL_OFFSET          (0x210)
#define PIO_INT_STAT_OFFSET          (0x214)
#define CTP_IRQ_NO			(gpio_int_info[0].port_num)
#define PIO_BASE_ADDRESS	(0x01c20800)
#define PIO_RANGE_SIZE		(0x400)
#define SYSCONFIG_GPIO_ENABLE

void* __iomem gpio_addr = NULL;
user_gpio_set_t  gpio_int_info[1];
int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
				PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};

static int ctp_judge_int_occur(void)
{
	int reg_val;
	int ret = -1;
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO))){
		ret = 0;
	}
	return ret; 	
}

static int ctp_clear_penirq(void)
{
	int reg_val;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	reg_val = reg_val|(1<<(CTP_IRQ_NO));
	writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);


	return 0;
}

static int ctp_set_irq_mode(ext_int_mode int_mode)
{
	int ret = 0;
	int reg_value;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;


	int gpio_int_hdle = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);

	//config gpio to int mode
	pr_info("%s: config gpio to int mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_int_port");	
	if(!gpio_int_hdle){
		pr_info("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, "ctp_int_port", 1);
	pr_info("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	pr_info(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);                                         
	//ctp_clear_penirq();

	reg_value = readl(gpio_addr + PIO_INT_STAT_OFFSET);

	if ((reg_value = (reg_value&(1<<(CTP_IRQ_NO))))) {
		writel(reg_value,gpio_addr + PIO_INT_STAT_OFFSET);
	}
         /*****************************/ 
             
	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);	
  	udelay(1);
#endif
request_tp_int_port_failed:
	return ret;  
}


/* Button to keycode conversion */
static u16 cyttsp4_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_HOME,		/* 102 */
	KEY_MENU,		/* 139 */
	KEY_BACK,		/* 158 */
	KEY_SEARCH,		/* 217 */
	KEY_VOLUMEDOWN,		/* 114 */
	KEY_VOLUMEUP,		/* 115 */
	KEY_CAMERA,		/* 212 */
	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp4_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp4_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp4_btn_keys),
	.tag = 0,
};

static struct cyttsp4_core_platform_data _cyttsp4_core_platform_data = {
	.irq_gpio = 0,
	.rst_gpio = 0,
	.xres = cyttsp4_xres,
	.init = cyttsp4_init,
	.power = cyttsp4_power,
	.irq_stat = cyttsp4_irq_stat,
	.set_irq_mode = ctp_set_irq_mode,
	.judge_int_occur = ctp_judge_int_occur,
	.clear_penirq = ctp_clear_penirq,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL, /* &cyttsp4_sett_param_regs, */
		NULL, /* &cyttsp4_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp4_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp4_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp4_sett_btn_keys,	/* button-to-keycode table */
	},
	.loader_pdata = &_cyttsp4_loader_platform_data,
};

static struct cyttsp4_core_info cyttsp4_core_info __initdata = {
	.name = CYTTSP4_CORE_NAME,
	.id = "main_ttsp_core",
	.adap_id = CYTTSP4_I2C_NAME,
	.platform_data = &_cyttsp4_core_platform_data,
};

static const uint16_t cyttsp4_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	CY_IGNORE_VALUE, 0, 255, 0, 0,
	CY_IGNORE_VALUE, 0, 255, 0, 0,
	CY_IGNORE_VALUE, -128, 127, 0, 0,
};

struct touch_framework cyttsp4_framework = {
	.abs = (uint16_t *)&cyttsp4_abs[0],
	.size = ARRAY_SIZE(cyttsp4_abs),
	.enable_vkeys = 0,
};

static struct cyttsp4_mt_platform_data _cyttsp4_mt_platform_data = {
	.frmwrk = &cyttsp4_framework,
	.flags = 0x0,
	.inp_dev_name = CYTTSP4_MT_NAME,
};

struct cyttsp4_device_info cyttsp4_mt_info __initdata = {
	.name = CYTTSP4_MT_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp4_mt_platform_data,
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_BUTTON
static struct cyttsp4_btn_platform_data _cyttsp4_btn_platform_data = {
	.inp_dev_name = CYTTSP4_BTN_NAME,
};

struct cyttsp4_device_info cyttsp4_btn_info __initdata = {
	.name = CYTTSP4_BTN_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp4_btn_platform_data,
};
#endif

static ssize_t cyttps4_virtualkeys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":"
		__stringify(KEY_BACK) ":1360:90:160:180"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_MENU) ":1360:270:160:180"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_HOME) ":1360:450:160:180"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_SEARCH) ":1360:630:160:180"
		"\n");
}

static struct kobj_attribute cyttsp4_virtualkeys_attr = {
	.attr = {
		.name = "virtualkeys.cyttsp4_mt",
		.mode = S_IRUGO,
	},
	.show = &cyttps4_virtualkeys_show,
};

static struct attribute *cyttsp4_properties_attrs[] = {
	&cyttsp4_virtualkeys_attr.attr,
	NULL
};

static struct attribute_group cyttsp4_properties_attr_group = {
	.attrs = cyttsp4_properties_attrs,
};

void __init a13_cyttsp4_init(void)
{
	struct kobject *properties_kobj;
	int ret = 0;
	//touch pins
	/*ret = gpio_request(MX6SL_PIN_TOUCH_INTB, "touch_intb");
	if(unlikely(ret)) return;
	
	ret = gpio_request(MX6SL_PIN_TOUCH_RST, "touch_rst");
	if(unlikely(ret)) goto free_intb;
	
	ret = gpio_request(MX6SL_PIN_TOUCH_SWDL, "touch_not_used");
	if(unlikely(ret)) goto free_rst;
	
	
	gpio_direction_input(MX6SL_PIN_TOUCH_INTB);
	gpio_direction_output(MX6SL_PIN_TOUCH_RST, 1);
	gpio_direction_input(MX6SL_PIN_TOUCH_SWDL);*/
	
	
	/* Register core and devices */
	cyttsp4_register_core_device(&cyttsp4_core_info);
	cyttsp4_register_device(&cyttsp4_mt_info);

//	cyttsp4_register_device(&cyttsp4_btn_info);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				&cyttsp4_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: failed to create board_properties\n", __func__);

	return;

//free_rst:
	//gpio_free(MX6SL_PIN_TOUCH_RST);
//free_intb:
	//gpio_free(MX6SL_PIN_TOUCH_INTB);
}

EXPORT_SYMBOL(a13_cyttsp4_init);

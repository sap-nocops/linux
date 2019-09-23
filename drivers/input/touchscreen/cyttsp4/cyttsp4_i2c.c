/*
 * cyttsp4_i2c.c
 * Cypress TrueTouch(TM) Standard Product V4 I2C Driver module.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2012 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 * Modified by: Cypress Semiconductor for test with device
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/cyttsp4_bus.h>
#include <linux/cyttsp4_core.h>
#include "cyttsp4_i2c.h"

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <mach/sys_config.h>

#define CY_I2C_DATA_SIZE  (3 * 256)


//Extracted from Goodix driver
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_GPIO_GET_INFO(gpiohandler, gpioinfo, pin)	gpio_get_one_pin_status(gpiohandler, gpioinfo, pin, 1)
#define GTP_GPIO_OUTPUT(gpiohandler,pin,level) 		do{\
								gpio_set_one_pin_io_status(gpiohandler,1,pin);\
								gpio_write_one_pin_value(gpiohandler,level,pin);\
							}while(0)
#define GTP_GPIO_REQUEST(pin)    			gpio_request_ex("ctp_para", pin) 
#define GTP_GPIO_FREE(pin)              		gpio_release(pin,0)

struct cyttsp4_i2c {
	struct i2c_client *client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	struct hrtimer timer;
	struct mutex lock;
	atomic_t timeout;
};

/* Addresses to scan */
union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
} u_i2c_addr = {{0x00},};

static __u32 twi_addr	= 0;
static __u32 twi_id	= 0;

int gtp_power_gpio_18=0,gtp_power_gpio_33=0;

static int ctp_fetch_sysconfig_para(void)
{
	int ret = 0;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];	
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
	printk("cyttsp4_i2c:%s. \n", __func__);
	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)) {
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}

	if (1 != ctp_used) {
		pr_err("%s: ctp_unused. \n", __func__);
		return -1;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para","ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))) {		
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))) {
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}


	u_i2c_addr.dirty_addr_buf[0] = twi_addr;

	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	printk("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);


	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))) {
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}

	printk("%s: ctp_twi_id is %d. \n", __func__, twi_id);

   //First, disable 3.3 V 
    gtp_power_gpio_33 = gpio_request_ex("ctp_para", "ctp_powerpin_33");
    if (!gtp_power_gpio_33) 
    {
        GTP_ERROR("Failed to request GPIO:ctp_powerpin_33, ERRNO:%d", (s32)gtp_power_gpio_33);
        GTP_GPIO_FREE(gtp_power_gpio_33);
    }
    else 
    {
    	printk("Disable pin 3,3V\n");
    	GTP_GPIO_OUTPUT(gtp_power_gpio_33, "ctp_powerpin_33", 0);	
    }


    gtp_power_gpio_18 = gpio_request_ex("ctp_para", "ctp_powerpin_18");
    if (!gtp_power_gpio_18) 
    {
        GTP_ERROR("Failed to request GPIO:ctp_powerpin_18, ERRNO:%d", (s32)gtp_power_gpio_18);
        GTP_GPIO_FREE(gtp_power_gpio_18);
    }
    else 
    {
    	printk("Enable pin 1,8V\n");
    	GTP_GPIO_OUTPUT(gtp_power_gpio_18, "ctp_powerpin_18", 1);	//enable 1,8 V output
    }
//TODO : Clean up this mess !
	return ret;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}

int cyttsp4_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	//adapter1=adapter;
	if (twi_id == adapter->nr) {
		printk("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CYTTSP4_I2C_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CYTTSP4_I2C_NAME, I2C_NAME_SIZE);
		return 0;
	} else {
		// printk("error to adapter this ic adapter->nr=%d\n",adapter->nr);
		return -ENODEV;
	}
}



static int cyttsp4_i2c_read_block_data(struct cyttsp4_i2c *ts_i2c, u8 addr,
	size_t length, void *values)
{
	int rc;
	struct i2c_msg xfer[2];
	
	xfer[0].addr = ts_i2c->client->addr;
	xfer[0].flags = 0; //I2C_M_TEN;
	xfer[0].len = 1;
	xfer[0].buf = &addr;
	
	xfer[1].addr = ts_i2c->client->addr;
	xfer[1].flags = 1; //I2C_M_TEN | I2C_M_RD;
	xfer[1].len = length;
	xfer[1].buf = values;
	rc = i2c_transfer(ts_i2c->client->adapter, xfer, 2);

	return (rc < 0) ? rc : rc == 2 ? 0 : -EIO;

}


static int cyttsp4_i2c_write_block_data(struct cyttsp4_i2c *ts_i2c, u8 addr,
	size_t length, const void *values)
{
	int rc;

	if (sizeof(ts_i2c->wr_buf) < (length + 1))
		return -ENOMEM;

	ts_i2c->wr_buf[0] = addr;
	memcpy(&ts_i2c->wr_buf[1], values, length);
	length += 1;

	/* write data */
	rc = i2c_master_send(ts_i2c->client, ts_i2c->wr_buf, length);

	return (rc < 0) ? rc : rc != length ? -EIO : 0;
}

static int cyttsp4_i2c_write(struct cyttsp4_adapter *adap, u8 addr,
	const void *buf, int size)
{
	struct cyttsp4_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_write_block_data(ts, addr, size, buf);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

static int cyttsp4_i2c_read(struct cyttsp4_adapter *adap, u8 addr,
	void *buf, int size)
{
	struct cyttsp4_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_read_block_data(ts, addr, size, buf);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

static struct cyttsp4_ops ops = {
	.write = cyttsp4_i2c_write,
	.read = cyttsp4_i2c_read,
};

static int __devinit cyttsp4_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct cyttsp4_i2c *ts_i2c;
	struct device *dev = &client->dev;
	char const *adap_id = dev_get_platdata(dev);
	char const *id;
	int rc;

	dev_info(dev, "%s: Starting %s probe...\n", __func__, CYTTSP4_I2C_NAME);

	dev_dbg(dev, "%s: debug on\n", __func__);
	dev_vdbg(dev, "%s: verbose debug on\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "%s: fail check I2C functionality\n", __func__);
		rc = -EIO;
		goto error_alloc_data_failed;
	}

	ts_i2c = kzalloc(sizeof(struct cyttsp4_i2c), GFP_KERNEL);
	if (ts_i2c == NULL) {
		dev_err(dev, "%s: Error, kzalloc.\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	mutex_init(&ts_i2c->lock);
	ts_i2c->client = client;
	client->dev.bus = &i2c_bus_type;
	i2c_set_clientdata(client, ts_i2c);
	dev_set_drvdata(&client->dev, ts_i2c);

	if (adap_id)
		id = adap_id;
	else
		id = CYTTSP4_I2C_NAME;

	dev_dbg(dev, "%s: add adap='%s' (CYTTSP4_I2C_NAME=%s)\n", __func__, id,
		CYTTSP4_I2C_NAME);

	pm_runtime_enable(&client->dev);

	rc = cyttsp4_add_adapter(id, &ops, dev);
	if (rc) {
		dev_err(dev, "%s: Error on probe %s\n", __func__,
			CYTTSP4_I2C_NAME);
		goto add_adapter_err;
	}

	dev_info(dev, "%s: Successful probe %s\n", __func__, CYTTSP4_I2C_NAME);

	return 0;

add_adapter_err:
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
error_alloc_data_failed:
	return rc;
}

/* registered in driver struct */
static int __devexit cyttsp4_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cyttsp4_i2c *ts_i2c = dev_get_drvdata(dev);
	char const *adap_id = dev_get_platdata(dev);
	char const *id;

	if (adap_id)
		id = adap_id;
	else
		id = CYTTSP4_I2C_NAME;

	dev_info(dev, "%s\n", __func__);
	cyttsp4_del_adapter(id);
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
	return 0;
}

static const struct i2c_device_id cyttsp4_i2c_id[] = {
	{ CYTTSP4_I2C_NAME, 0 },  { }
};

static struct i2c_driver cyttsp4_i2c_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = CYTTSP4_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp4_i2c_probe,
	.remove = __devexit_p(cyttsp4_i2c_remove),
	.id_table = cyttsp4_i2c_id,
	.address_list	= u_i2c_addr.normal_i2c,
};

static int __init cyttsp4_i2c_init(void)
{
	int rc=0;

	if (ctp_fetch_sysconfig_para()) {
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	cyttsp4_i2c_driver.detect = cyttsp4_detect;

 	rc = i2c_add_driver(&cyttsp4_i2c_driver);

	pr_info("%s: Cypress TTSP I2C Touchscreen Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
module_init(cyttsp4_i2c_init);

static void __exit cyttsp4_i2c_exit(void)
{
	i2c_del_driver(&cyttsp4_i2c_driver);
	pr_info("%s: module exit\n", __func__);
}
module_exit(cyttsp4_i2c_exit);

MODULE_ALIAS(CYTTSP4_I2C_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cyttsp4_i2c_id);

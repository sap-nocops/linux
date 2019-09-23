#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach/sys_config.h>



static int __devinit bookeen_touch_init(void)
{
	int tp_power_gpio=0,tp_power_gpio_off=0;

	printk("---------------------------bookeen_touch_init----------------------------------------------\n");
    tp_power_gpio = gpio_request_ex("ctp_para", "ctp_powerpin");
    tp_power_gpio_off = gpio_request_ex("ctp_para", "ctp_powerpin_off");
    gpio_set_one_pin_io_status(tp_power_gpio, 1 ,"ctp_powerpin");
	gpio_write_one_pin_value(tp_power_gpio, 1 ,"ctp_powerpin");

    gpio_set_one_pin_io_status(tp_power_gpio_off, 0 ,"ctp_powerpin_off");
	gpio_write_one_pin_value(tp_power_gpio_off, 0 ,"ctp_powerpin_off");
	return 0;
}

static void __exit bookeen_touch_exit(void)
{
	int tp_power_gpio;

	printk("---------------------------bookeen_touch_exit----------------------------------------------\n");
    tp_power_gpio = gpio_request_ex("ctp_para", "ctp_powerpin");
    gpio_set_one_pin_io_status(tp_power_gpio, 0 ,"ctp_powerpin");
	gpio_write_one_pin_value(tp_power_gpio, 0 ,"ctp_powerpin");
	return 0;
}

module_init(bookeen_touch_init);
module_exit(bookeen_touch_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");

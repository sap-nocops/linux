/*
 * Papyrus epaper power control HAL
 *
 *      Copyright (C) 2009 Dimitar Dimitrov, MM Solutions
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 * TPS6518x power control is facilitated using I2C control and WAKEUP GPIO
 * pin control. The other VCC GPIO Papyrus' signals must be tied to ground.
 *
 * TODO:
 * 	- Instead of polling, use interrupts to signal power up/down
 * 	  acknowledge.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <mach/sys_config.h>
#include "../../video/sun5i/disp/OSAL/OSAL_Pin.h"
#include "../../input/touchscreen/ctp_platform_ops.h"

static __u32 twi_addr	= 0;
static __u32 twi_id	= 0;

/* Addresses to scan */
union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
} u_i2c_addr = {{0x00},};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31))
#include <linux/i2c/pmic-tps65185-i2c.h>
#else
#define PAPYRUS2_1P1_I2C_ADDRESS		0x68
#endif

//#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
//  #include <asm/gpio.h>
//#else
//  #include <linux/gpio.h>
//#endif

#define TPS65185_I2C_NAME		"tps65185"
#define PAPYRUS_STANDBY_DWELL_TIME	0
#define PAPYRUS_VCOM_MAX_MV		0
#define PAPYRUS_VCOM_MIN_MV		-5110

#if 0
#define tps65185_printk(fmt, args...) printk(KERN_INFO "[tps] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define tps65185_printk(fmt, args...) 
#endif

/* After waking up from sleep, Papyrus
   waits for VN to be discharged and all
   voltage ref to startup before loading
   the default EEPROM settings. So accessing
   registers too early after WAKEUP could
   cause the register to be overridden by
   default values */
#define PAPYRUS_EEPROM_DELAY_MS 50
/* Papyrus WAKEUP pin must stay low for
   a minimum time */
#define PAPYRUS_SLEEP_MINIMUM_MS 110
/* Temp sensor might take a little time to
   settle eventhough the status bit in TMST1
   state conversion is done - if read too early
   0C will be returned instead of the right temp */
#define PAPYRUS_TEMP_READ_TIME_MS 10

/* Powerup sequence takes at least 24 ms - no need to poll too frequently */
#define HW_GET_STATE_INTERVAL_MS 24

struct papyrus_sess {
	struct i2c_adapter *adap;
	struct i2c_client *client;
	uint8_t enable_reg_shadow;
	uint8_t enable_reg;
	uint8_t vadj;
	uint8_t vcom1;
	uint8_t vcom2;
	uint8_t vcom2off;
	uint8_t int_en1;
	uint8_t int_en2;
	uint8_t upseq0;
	uint8_t upseq1;
	uint8_t dwnseq0;
	uint8_t dwnseq1;
	uint8_t tmst1;
	uint8_t tmst2;

	/* Custom power up/down sequence settings */
	struct {
		/* If options are not valid we will rely on HW defaults. */
		bool valid;
		unsigned int dly[8];
	} seq;
	struct timeval standby_tv;
	unsigned int v3p3off_time_ms;

	/* True if a high WAKEUP brings Papyrus out of reset. */
	int wakeup_active_high;
	int vcomctl_active_high;
};


#define PAPYRUS_I2C_BUS_NUM		(1)
#define PAPYRUS_ADDR_TMST_VALUE		0x00
#define PAPYRUS_ADDR_ENABLE		0x01
#define PAPYRUS_ADDR_VADJ		0x02
#define PAPYRUS_ADDR_VCOM1_ADJUST	0x03
#define PAPYRUS_ADDR_VCOM2_ADJUST	0x04
#define PAPYRUS_ADDR_INT_ENABLE1	0x05
#define PAPYRUS_ADDR_INT_ENABLE2	0x06
#define PAPYRUS_ADDR_INT_STATUS1	0x07
#define PAPYRUS_ADDR_INT_STATUS2	0x08
#define PAPYRUS_ADDR_UPSEQ0		0x09
#define PAPYRUS_ADDR_UPSEQ1		0x0a
#define PAPYRUS_ADDR_DWNSEQ0		0x0b
#define PAPYRUS_ADDR_DWNSEQ1		0x0c
#define PAPYRUS_ADDR_TMST1		0x0d
#define PAPYRUS_ADDR_TMST2		0x0e
#define PAPYRUS_ADDR_PG_STATUS		0x0f
#define PAPYRUS_ADDR_REVID		0x10

// INT_ENABLE1
#define PAPYRUS_INT_ENABLE1_ACQC_EN	1
#define PAPYRUS_INT_ENABLE1_PRGC_EN	0

// INT_STATUS1
#define PAPYRUS_INT_STATUS1_ACQC	1
#define PAPYRUS_INT_STATUS1_PRGC	0

// VCOM2_ADJUST
#define PAPYRUS_VCOM2_ACQ		7
#define PAPYRUS_VCOM2_PROG		6
#define PAPYRUS_VCOM2_HIZ		5

#define PAPYRUS_MV_TO_VCOMREG(MV)	((MV) / 10)

#define V3P3_EN_MASK			0x20
#define PAPYRUS_V3P3OFF_DELAY_MS	100

struct papyrus_hw_state {
	uint8_t tmst_value;
	uint8_t int_status1;
	uint8_t int_status2;
	uint8_t pg_status;
};

static uint8_t papyrus2_i2c_addr = PAPYRUS2_1P1_I2C_ADDRESS;

void papyrus_set_powerup(int on_or_off)
{
	user_gpio_set_t  gpio_info[1];
	__hdle hdl;

	gpio_info->port=5;	// 1//1:PIOA,2:PIOB,3:PIOC......
	gpio_info->port_num=7;	// PIO number
	gpio_info->mul_sel=1;	// 1//1:output,0:input
	gpio_info->pull = 1;
	gpio_info->drv_level=1;
	gpio_info->data = on_or_off;

	hdl = OSAL_GPIO_Request(gpio_info, 1);
	OSAL_GPIO_Release(hdl, 2);
}

void papyrus_set_vcomup(int on_or_off)
{
	user_gpio_set_t  gpio_info[1];
	__hdle hdl;

	gpio_info->port=4;	// 1//1:PIOA,2:PIOB,3:PIOC......
	gpio_info->port_num=19;	// PIO number
	gpio_info->mul_sel=1;	// 1//1:output,0:input
	gpio_info->pull = 1;
	gpio_info->drv_level=1;
	gpio_info->data = on_or_off;

	hdl = OSAL_GPIO_Request(gpio_info, 1);
	OSAL_GPIO_Release(hdl, 2);
}

void papyrus_set_wakeup(int on_or_off)
{
	user_gpio_set_t  gpio_info[1];
	__hdle hdl;

	gpio_info->port=4;	// 1//1:PIOA,2:PIOB,3:PIOC......
	gpio_info->port_num=18;	// PIO number
	gpio_info->mul_sel=1;	// 1//1:output,0:input
	gpio_info->pull = 1;
	gpio_info->drv_level=1;
	gpio_info->data = on_or_off;

	hdl = OSAL_GPIO_Request(gpio_info, 1);
	OSAL_GPIO_Release(hdl, 2);
}

static int papyrus_hw_setreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t val)
{
	int stat;
	uint8_t txbuf[2] = { regaddr, val };
	int retry = 50;
	struct i2c_msg msgs[] = {
		{
			.addr = papyrus2_i2c_addr,
			.flags = 0,
			.len = 2,
			.buf = txbuf,
		}
	};
send:
	stat = i2c_transfer(sess->adap, msgs, ARRAY_SIZE(msgs));

	if (stat < 0) {
		pr_err("papyrus: I2C send error: %d\n", stat);
		if (retry > 0) {
			printk(" retry %d ...\n", retry);
			retry--;
			goto send;
		}
	}
	else if (stat != ARRAY_SIZE(msgs)) {
		pr_err("papyrus: I2C send N mismatch: %d\n", stat);
		if (retry > 0) {
			printk(" retry %d ...\n", retry);
			retry--;
			goto send;
		}
		stat = -EIO;
	} else
		stat = 0;

	return stat;
}

static int papyrus_hw_getreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t *val)
{
	int stat;
	int retry = 50;
	struct i2c_msg msgs[] = {
		{
			.addr = papyrus2_i2c_addr,
			.flags = 0,
			.len = 1,
			.buf = &regaddr,
		},
		{
			.addr = papyrus2_i2c_addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
	};

send:
	stat = i2c_transfer(sess->adap, msgs, ARRAY_SIZE(msgs));

	if (stat < 0)
    	{
		pr_err("papyrus: I2C read error: %d\n", stat);
        	pr_err("Papyrus i2c addr %x %s\n",papyrus2_i2c_addr,__FILE__); 
		if (retry > 0) {
			printk(" retry %d ...\n", retry);
			retry--;
			goto send;
		}
   	 }
	else if (stat != ARRAY_SIZE(msgs)) {
		pr_err("papyrus: I2C read N mismatch: %d\n", stat);
		if (retry > 0) {
			printk(" retry %d ...\n", retry);
			retry--;
			goto send;
		}
		stat = -EIO;
	} else
		stat = 0;

	return stat;
}


static void papyrus_hw_get_pg(struct papyrus_sess *sess,
							  struct papyrus_hw_state *hwst)
{
	int stat;

	stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &hwst->pg_status);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
}

/*
static void papyrus_hw_get_state(struct papyrus_sess *sess, struct papyrus_hw_state *hwst)
{
	int stat;

	stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_TMST_VALUE, &hwst->tmst_value);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_INT_STATUS1, &hwst->int_status1);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_INT_STATUS2, &hwst->int_status2);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &hwst->pg_status);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
}
*/

static void papyrus_hw_send_powerup(struct papyrus_sess *sess)
{
	int stat = 0;

	papyrus_set_vcomup(0);
	papyrus_set_powerup(0);
	papyrus_set_wakeup(0);
	papyrus_set_vcomup(1);

	mdelay(10); // 1ms
	papyrus_set_wakeup(1);
	mdelay(10); // 1ms

	/* set VADJ */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VADJ, sess->vadj);

	/* set UPSEQs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ1, sess->upseq1);

	/* set DWNSEQs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ0, sess->dwnseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ1, sess->dwnseq1);

	/* set TMSTs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_TMST2, sess->tmst2);

	/* set INT_ENABLEs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_INT_ENABLE1, sess->int_en1);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_INT_ENABLE2, sess->int_en2);

	/* Enable 3.3V switch to the panel */
	sess->enable_reg_shadow |= V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	/* switch to active mode, VCOM buffer disabled */
	sess->enable_reg_shadow = 0xaf;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
	//papyrus_set_powerup(1);
}


static void papyrus_hw_send_powerdown(struct papyrus_sess *sess)
{
	int stat;

	/* keep XXX_PWR_EN signals enabled and activate STANDBY */
	sess->enable_reg_shadow = 0x6f;
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	msleep(sess->v3p3off_time_ms);

	/* 3.3V switch must be turned off last */
	sess->enable_reg_shadow &= ~V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	/* disable CPLD */
	// gpio_direction_output(CPLD_RESET_GPIO, 0);
	papyrus_set_powerup(0);

	// papyrus_set_wakeup(!sess->wakeup_active_high);

	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);

	do_gettimeofday(&sess->standby_tv);
}

static int papyrus_hw_get_revid(struct papyrus_sess *sess)
{
	int stat;
	uint8_t revid;

	stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_REVID, &revid);
	tps65185_printk();

	if (stat) {
		pr_err("papyrus: I2C error: %d\n", stat);
		return stat;
	} else {
		return revid;
	}
}

static int papyrus_hw_arg_init(struct papyrus_sess *sess)
{
#if 0
	sess->vadj = 0x03;

	sess->upseq0 = 0xE4;
	sess->upseq1 = 0x55;

	sess->dwnseq0 = 0x1B;
	sess->dwnseq1 = 0xC0;

#else
	sess->vadj = 0x06;

	sess->upseq0 = 0xE4;
	sess->upseq1 = 0x55;

	sess->dwnseq0 = 0x1E;
	sess->dwnseq1 = 0xFF;
#endif

	return 0;
}

static int papyrus_hw_init(struct papyrus_sess *sess)
{
	int stat = 0;
	int i = 0;

	printk("===========================%s @@1====================\n", __func__);

	papyrus_set_powerup(0);
	sess->wakeup_active_high = 1;
	sess->vcomctl_active_high = 1;

	papyrus_set_wakeup(!sess->wakeup_active_high);
	/* wait to reset papyrus */
	msleep(PAPYRUS_SLEEP_MINIMUM_MS);

	papyrus_set_wakeup(sess->wakeup_active_high);
	papyrus_set_vcomup(sess->vcomctl_active_high);
	msleep(PAPYRUS_EEPROM_DELAY_MS);

	//sess->adap = i2c_get_adapter(PAPYRUS_I2C_BUS_NUM);
	//sess->adap =adapter1;
	//if (!sess->adap) {
		//pr_err("cannot get I2C adapter %u\n",PAPYRUS_I2C_BUS_NUM);
		//printk("cannot get I2C adapter %u\n",PAPYRUS_I2C_BUS_NUM);
		//stat = -ENODEV;
		//goto free_gpios;
	//}

	stat = papyrus_hw_get_revid(sess);
	if (stat < 0)
		goto cleanup_i2c_adapter;
	printk("papyrus: detected device with ID=%02x (TPS6518%dr%dp%d)\n",
			stat, stat & 0xF, (stat & 0xC0) >> 6, (stat & 0x30) >> 4);
	stat = 0;
	return stat;
	
cleanup_i2c_adapter:
	i2c_put_adapter(sess->adap);
//free_gpios:
	// ctp_free_platform_resource();
	pr_err("papyrus: ERROR: could not initialize I2C papyrus!\n");
	return stat;

}

struct pmic_sess {
	bool powered;
	struct delayed_work powerdown_work;
	unsigned int dwell_time_ms;
	struct delayed_work vcomoff_work;
	unsigned int vcomoff_time_ms;
	int v3p3off_time_ms;
	const struct pmic_driver *drv;
	void *drvpar;
	int temp_man_offset;
	int revision;
	int is_inited;
};

static int papyrus_hw_read_temperature(struct pmic_sess *pmsess, int *t)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	int stat;
	int ntries = 50;
	uint8_t tb;

	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_TMST1, 0x80);

	do {
		stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_TMST1, &tb);
	} while (!stat && ntries-- && (((tb & 0x20) == 0) || (tb & 0x80)));

	if (stat)
		return stat;

	msleep(PAPYRUS_TEMP_READ_TIME_MS);
	stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_TMST_VALUE, &tb);
	*t = (int)(int8_t)tb;

	printk("current temperature is xxxxxxxxxxxxxx%%%%%%%%%d\n",*t);

	return stat;
}


static void papyrus_hw_power_req(struct pmic_sess *pmsess, bool up)
{
	struct papyrus_sess *sess = pmsess->drvpar;

	pr_debug("papyrus: i2c pwr req: %d\n", up);
	if (up)
		papyrus_hw_send_powerup(sess);
	else
		papyrus_hw_send_powerdown(sess);
}


static bool papyrus_hw_power_ack(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	struct papyrus_hw_state hwst;
	int st;
	int retries_left = 10;

	do {
		papyrus_hw_get_pg(sess, &hwst);

		pr_debug("hwst: tmst_val=%d, ist1=%02x, ist2=%02x, pg=%02x\n",
				hwst.tmst_value, hwst.int_status1,
				hwst.int_status2, hwst.pg_status);
		printk("power_ack hwst.pg_status =%x\n",hwst.pg_status );
		hwst.pg_status &= 0xfa;
		if (hwst.pg_status == 0xfa)
			st = 1;
		else if (hwst.pg_status == 0x00)
			st = 0;
		else {
			st = -1;	/* not settled yet */
			msleep(HW_GET_STATE_INTERVAL_MS);
		}
		retries_left--;
		printk("power_ack st=%d\n",st);
	} while ((st == -1) && retries_left);

	if ((st == -1) && !retries_left)
		pr_err("papyrus: power up/down settle error (PG = %02x)\n", hwst.pg_status);

	return !!st;
}

static void papyrus_hw_cleanup(struct papyrus_sess *sess)
{
	i2c_put_adapter(sess->adap);
}

static int papyrus_set_enable(struct pmic_sess *pmsess, int enable)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->enable_reg = enable;
	return 0;
}

static int papyrus_set_vcom_voltage(struct pmic_sess *pmsess, int vcom_mv)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(-vcom_mv) & 0x00FF);
	sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(-vcom_mv) & 0x0100) >> 8);
	return 0;
}

static int papyrus_set_vcom1(struct pmic_sess *pmsess, uint8_t vcom1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vcom1 = vcom1;
	return 0;
}

static int papyrus_set_vcom2(struct pmic_sess *pmsess, uint8_t vcom2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	// TODO; Remove this temporary solution to set custom vcom-off mode
	//       Add PMIC setting when this is to be a permanent feature
	pr_debug("papyrus_set_vcom2 vcom2off 0x%02x\n", vcom2);
	sess->vcom2off = vcom2;
	return 0;
}

static int papyrus_set_vadj(struct pmic_sess *pmsess, uint8_t vadj)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vadj = vadj;
	return 0;
}

static int papyrus_set_int_en1(struct pmic_sess *pmsess, uint8_t int_en1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->int_en1 = int_en1;
	return 0;
}

static int papyrus_set_int_en2(struct pmic_sess *pmsess, uint8_t int_en2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->int_en2 = int_en2;
	return 0;
}

static int papyrus_set_upseq0(struct pmic_sess *pmsess, uint8_t upseq0)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->upseq0 = upseq0;
	return 0;
}

static int papyrus_set_upseq1(struct pmic_sess *pmsess, uint8_t upseq1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->upseq1 = upseq1;
	return 0;
}

static int papyrus_set_dwnseq0(struct pmic_sess *pmsess, uint8_t dwnseq0)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->dwnseq0 = dwnseq0;
	return 0;
}

static int papyrus_set_dwnseq1(struct pmic_sess *pmsess, uint8_t dwnseq1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->dwnseq1 = dwnseq1;
	return 0;
}

static int papyrus_set_tmst1(struct pmic_sess *pmsess, uint8_t tmst1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->tmst1 = tmst1;
	return 0;
}

static int papyrus_set_tmst2(struct pmic_sess *pmsess, uint8_t tmst2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->tmst2 = tmst2;
	return 0;
}

static int papyrus_vcom_switch(struct pmic_sess *pmsess, bool state)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	int stat;

	sess->enable_reg_shadow &= ~((1u << 4) | (1u << 6) | (1u << 7));
	sess->enable_reg_shadow |= (state ? 1u : 0) << 4;

	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	/* set VCOM off output */
	if (!state && sess->vcom2off != 0) {
		stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,
						sess->vcom2off);
	}

	return stat;
}

static bool papyrus_standby_dwell_time_ready(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	struct timeval current_tv;
	long total_secs;

	do_gettimeofday(&current_tv);
	mb();
	total_secs = current_tv.tv_sec - sess->standby_tv.tv_sec;

	if (total_secs < PAPYRUS_STANDBY_DWELL_TIME)
		return false;

	return true;
}

static void papyrus_pm_sleep(struct pmic_sess *sess)
{
#if !defined(FB_OMAP3EP_PAPYRUS_PM_STANDBY)
	struct papyrus_sess *s = sess->drvpar;
	papyrus_set_wakeup(!s->wakeup_active_high);
	papyrus_set_vcomup(!s->vcomctl_active_high);
	msleep(PAPYRUS_SLEEP_MINIMUM_MS);
#endif
	pr_debug("%s\n", __func__);
}

static void papyrus_pm_resume(struct pmic_sess *sess)
{
#if !defined(FB_OMAP3EP_PAPYRUS_PM_STANDBY)
	struct papyrus_sess *s = sess->drvpar;
	papyrus_set_vcomup(s->vcomctl_active_high);
	papyrus_set_wakeup(s->wakeup_active_high);
	msleep(PAPYRUS_EEPROM_DELAY_MS);
#endif
	pr_debug("%s\n", __func__);
}

static int papyrus_probe(struct pmic_sess *pmsess,struct i2c_client *client)
{
	struct papyrus_sess *sess;
	int stat =0;
	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (!sess)
		return -ENOMEM;
	sess->client = client;
	sess->adap = client->adapter;

	//sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(2550) & 0x00FF);
	//sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(2550) & 0x0100) >> 8);

	papyrus_hw_arg_init(sess);
	
	if (pmsess->v3p3off_time_ms == -1)
		sess->v3p3off_time_ms = PAPYRUS_V3P3OFF_DELAY_MS;
	else
		sess->v3p3off_time_ms = pmsess->v3p3off_time_ms;

	do_gettimeofday(&sess->standby_tv);

	stat = papyrus_hw_init(sess);//, pmsess->drv->id);//hxm del fot test
	if (stat)
		goto free_sess;

	sess->enable_reg_shadow = 0;
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,sess->enable_reg_shadow);
	if (stat)
		goto free_sess;

	pmsess->drvpar = sess;

	pmsess->revision = papyrus_hw_get_revid(sess);
	return 0;

free_sess:
	kfree(sess);
	return stat;
}

static void papyrus_remove(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	tps65185_printk();

	papyrus_hw_cleanup(sess);

	kfree(sess);
	pmsess->drvpar = 0;
}

static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];

	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
	printk("%s. \n", __func__);

	if (SCRIPT_PARSER_OK != script_parser_fetch("TPS65185_PARA", "tps65185_used", &ctp_used, 1)) {
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}

	if (ctp_used != 1) {
		pr_err("%s: tps65185_unused. \n", __func__);
		return ret;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch_ex("TPS65185_PARA","tps65185_name", (int *)(&name), &type, sizeof(name)/sizeof(int))) {
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}

	/*if(strcmp(TPS65185_I2C_NAME, name)){
		pr_err("%s: name %s does not match TPS65185_I2C_NAME. \n", __func__, name);
		pr_err(TPS65185_I2C_NAME);
		return ret;
	}
	*/
	if (SCRIPT_PARSER_OK != script_parser_fetch("TPS65185_PARA", "tps65185_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))) {
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}

	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;

	printk("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);

	if (SCRIPT_PARSER_OK != script_parser_fetch("TPS65185_PARA", "tps65185_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))) {
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}

	printk("%s: ctp_twi_id is %d. \n", __func__, twi_id);

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}

int TP65185_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	//adapter1=adapter;

	if (twi_id == adapter->nr) {
		printk("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, TPS65185_I2C_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, TPS65185_I2C_NAME, I2C_NAME_SIZE);
		return 0;
	} else {
		// printk("error to adapter this ic adapter->nr=%d\n",adapter->nr);
		return -ENODEV;
	}
}

struct pmic_driver {
	const char *id;

	int vcom_min;
	int vcom_max;
	int vcom_step;

	int (*hw_read_temperature)(struct pmic_sess *sess, int *t);
	bool (*hw_power_ack)(struct pmic_sess *sess);
	void (*hw_power_req)(struct pmic_sess *sess, bool up);

	int (*set_enable)(struct pmic_sess *sess, int enable);
	int (*set_vcom_voltage)(struct pmic_sess *sess, int vcom_mv);
	int (*set_vcom1)(struct pmic_sess *sess, uint8_t vcom1);
	int (*set_vcom2)(struct pmic_sess *sess, uint8_t vcom2);
	int (*set_vadj)(struct pmic_sess *sess, uint8_t vadj);
	int (*set_int_en1)(struct pmic_sess *sess, uint8_t int_en1);
	int (*set_int_en2)(struct pmic_sess *sess, uint8_t int_en2);
	int (*set_upseq0)(struct pmic_sess *sess, uint8_t upseq0);
	int (*set_upseq1)(struct pmic_sess *sess, uint8_t upseq1);
	int (*set_dwnseq0)(struct pmic_sess *sess, uint8_t dwnseq0);
	int (*set_dwnseq1)(struct pmic_sess *sess, uint8_t dwnseq1);
	int (*set_tmst1)(struct pmic_sess *sess, uint8_t tmst1);
	int (*set_tmst2)(struct pmic_sess *sess, uint8_t tmst2);

	int (*set_vp_adjust)(struct pmic_sess *sess, uint8_t vp_adjust);
	int (*set_vn_adjust)(struct pmic_sess *sess, uint8_t vn_adjust);
	int (*set_vcom_adjust)(struct pmic_sess *sess, uint8_t vcom_adjust);
	int (*set_pwr_seq0)(struct pmic_sess *sess, uint8_t pwr_seq0);
	int (*set_pwr_seq1)(struct pmic_sess *sess, uint8_t pwr_seq1);
	int (*set_pwr_seq2)(struct pmic_sess *sess, uint8_t pwr_seq2);
	int (*set_tmst_config)(struct pmic_sess *sess, uint8_t tmst_config);
	int (*set_tmst_os)(struct pmic_sess *sess, uint8_t tmst_os);
	int (*set_tmst_hyst)(struct pmic_sess *sess, uint8_t tmst_hyst);

	int (*hw_vcom_switch)(struct pmic_sess *sess, bool state);

	int (*hw_set_dvcom)(struct pmic_sess *sess, int state);

	int (*hw_init)(struct pmic_sess *sess ,struct i2c_client *);
	void (*hw_cleanup)(struct pmic_sess *sess);

	bool (*hw_standby_dwell_time_ready)(struct pmic_sess *sess);
	void (*hw_pm_sleep)(struct pmic_sess *sess);
	void (*hw_pm_resume)(struct pmic_sess *sess);

};

const struct pmic_driver pmic_driver_tps65185_i2c = {
	.id = "tps65185-i2c",

	.vcom_min = PAPYRUS_VCOM_MIN_MV,
	.vcom_max = PAPYRUS_VCOM_MAX_MV,
	.vcom_step = 10,

	.hw_read_temperature = papyrus_hw_read_temperature,
	.hw_power_ack = papyrus_hw_power_ack,
	.hw_power_req = papyrus_hw_power_req,

	.set_enable = papyrus_set_enable,
	.set_vcom_voltage = papyrus_set_vcom_voltage,
	.set_vcom1 = papyrus_set_vcom1,
	.set_vcom2 = papyrus_set_vcom2,
	.set_vadj = papyrus_set_vadj,
	.set_int_en1 = papyrus_set_int_en1,
	.set_int_en2 = papyrus_set_int_en2,
	.set_upseq0 = papyrus_set_upseq0,
	.set_upseq1 = papyrus_set_upseq1,
	.set_dwnseq0 = papyrus_set_dwnseq0,
	.set_dwnseq1 = papyrus_set_dwnseq1,
	.set_tmst1 = papyrus_set_tmst1,
	.set_tmst2 = papyrus_set_tmst2,

	.hw_vcom_switch = papyrus_vcom_switch,

	.hw_init = papyrus_probe,
	.hw_cleanup = papyrus_remove,

	.hw_standby_dwell_time_ready = papyrus_standby_dwell_time_ready,
	.hw_pm_sleep = papyrus_pm_sleep,
	.hw_pm_resume = papyrus_pm_resume,
};

struct pmic_sess pmic_sess_data;



 void papyrus_hw_send_powerup_led()
{
	int stat = 0;
	char data;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
	papyrus_set_wakeup(1);
	papyrus_hw_getreg(sess, PAPYRUS_ADDR_ENABLE, &data);
	data |= 0x20;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,data);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
	papyrus_set_powerup(1);
	msleep(25);
	papyrus_set_vcomup(1);

	
}
#if 0
void papyrus_hw_send_powerup_led()
{
	int stat = 0;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;

	papyrus_set_powerup(0);
	papyrus_set_wakeup(0);

	/* wait to reset papyrus */
	msleep(10);
	papyrus_set_wakeup(1);
	papyrus_set_vcomup(1);
	msleep(10);

	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);

	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);

	papyrus_set_powerup(1);
}
#endif 
void papyrus_hw_send_powerdown_led()
{
	papyrus_set_powerup(0);
	papyrus_set_wakeup(0);
	papyrus_set_vcomup(0);
}


static int tps65185_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	tps65185_printk("I2C addr:%x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("I2C check functionality failed.");
		return -ENODEV;
	}

	if (pmic_driver_tps65185_i2c.hw_init((struct pmic_sess *)&pmic_sess_data,client) != 0) {
		printk("pmic_driver_tps65185_i2c hw_init failed.");
		return -ENODEV;
	}

	pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,1);

	pmic_sess_data.is_inited = 1;

	return 0;
}

static int tps65185_remove(struct i2c_client *client)
{
	pmic_driver_tps65185_i2c.hw_cleanup((struct pmic_sess *)&pmic_sess_data);
	memset(&pmic_sess_data,0,sizeof(struct pmic_sess));
	return 0;
}


static const struct i2c_device_id tps65185_id[] = {
	{ TPS65185_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tps65185_driver = {
	.class = I2C_CLASS_HWMON,
	.probe	= tps65185_probe,
	.remove 	= tps65185_remove,
	.id_table	= tps65185_id,
	.driver = {
		.name	  = TPS65185_I2C_NAME,
		.owner	  = THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
};

static int __init tps65185_init(void)
{
	printk("===========================%s=====================\n", __func__);

	if (ctp_fetch_sysconfig_para()) {
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	tps65185_driver.detect = TP65185_detect;

	return i2c_add_driver(&tps65185_driver);
}

static void __exit tps65185_exit(void)
{
	return i2c_del_driver(&tps65185_driver);
}

int tps65185_v3p3_set(void)
{
	int stat = 0;
	struct pmic_sess tpmic_sess_data = pmic_sess_data;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;

	printk("#############  tps65185_v3p3_set ##############\n");

	papyrus_set_powerup(1);
	papyrus_set_wakeup(1);

	mdelay(10); // 1ms

	sess->enable_reg_shadow |= V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);
	mdelay(10); // 1ms

	printk("#############  tps65185_v3p3_set fini ##############\n");
	return 0;
}

int tps65185_v3p3_reset(void)
{
	int stat = 0;
	struct pmic_sess tpmic_sess_data = pmic_sess_data;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;

	printk("#############  tps65185_v3p3_reset ##############\n");
	sess->enable_reg_shadow &= ~V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);
	papyrus_set_wakeup(0);
	papyrus_set_powerup(0);

	printk("#############  tps65185_v3p3_reset fini ##############\n");

	return 0;
}

int tps65185_vcom_set(int vcom_mv)
{
	//struct i2c_client *client=NULL; 
	struct pmic_sess tpmic_sess_data = pmic_sess_data;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
	uint8_t rev_val = 0;
	int stat;
	int read_vcom_mv = 0;

	printk("tps65185_vcom_set enter.\n");

	if (!tpmic_sess_data.is_inited)
		return -1;

	papyrus_set_powerup(0);
	papyrus_set_wakeup(0);

	/* wait to reset papyrus */
	msleep(PAPYRUS_SLEEP_MINIMUM_MS);
	papyrus_set_wakeup(1);
	papyrus_set_vcomup(1);
	msleep(PAPYRUS_EEPROM_DELAY_MS);
	   
	// Set vcom voltage
	pmic_driver_tps65185_i2c.set_vcom_voltage((struct pmic_sess *)&tpmic_sess_data,vcom_mv);

	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST,sess->vcom1);

	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,sess->vcom2);
	
	printk("sess->vcom1 = 0x%x sess->vcom2 = 0x%x\n",sess->vcom1,sess->vcom2);

	// PROGRAMMING
	sess->vcom2 |= 1<<PAPYRUS_VCOM2_PROG;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,sess->vcom2);
	rev_val = 0;

	while (!(rev_val & (1<<PAPYRUS_INT_STATUS1_PRGC))) {
		read_vcom_mv++;

		if (read_vcom_mv > 50)
			break;

		stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_INT_STATUS1, &rev_val);
		printk("PAPYRUS_ADDR_INT_STATUS1 = 0x%x\n",rev_val);
		msleep(50);
	}
	
	// VERIFICATION
	printk("sess->vcom1 = 0x%x sess->vcom2 = 0x%x\n",sess->vcom1,sess->vcom2);
	
	papyrus_set_wakeup(0);
	msleep(10);
	papyrus_set_wakeup(1);
	msleep(10);	
	
	read_vcom_mv = 0;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, &rev_val);
	printk("rev_val VCOM1 = 0x%x\n",rev_val);
	read_vcom_mv += rev_val;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, &rev_val);
	printk("rev_val VCOM2= 0x%x\n",rev_val);
	read_vcom_mv += ((rev_val & 0x0001)<<8);
	printk("read_vcom_mv = %d\n",read_vcom_mv);

	//while(1)
	//{
	//	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_INT_STATUS1, &rev_val);
		//tps65185_printk("PAPYRUS_ADDR_INT_STATUS1 = 0x%x\n",rev_val);	
	//	msleep(2000);
	//}

	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);

	return 0;
}

int tps65185_probe_after_ebc(void)
{
	struct i2c_client *client=NULL;

	tps65185_printk("tps65185_probe_after_ebc enter.\n");

	if (pmic_driver_tps65185_i2c.hw_init((struct pmic_sess *)&pmic_sess_data,client) != 0) {
		printk("pmic_driver_tps65185_i2c hw_init failed.");
		return -ENODEV;
	}

	pmic_sess_data.is_inited = 1;

	// tps65185_vcom_set(-1970);
	// tps65185_vcom_set(-2190);
	// tps65185_vcom_set(-2040);
	// tps65185_vcom_set(-2200);

	tps65185_vcom_set(-1600);

	printk("tps65185_probe_after_ebc ok.\n");

	return 0;
}

int tps65185_power_set(int up_or_down)
{
	printk("tps65185_power_set  up_or_down =%d\n",up_or_down);

	if (pmic_sess_data.is_inited)
		pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,up_or_down);

	return 0;
}

int tps65185_temperature_get(int *temp)
{
	if (pmic_sess_data.is_inited)
		return pmic_driver_tps65185_i2c.hw_read_temperature((struct pmic_sess *)&pmic_sess_data,temp);
	else
		return 0;
}
module_init(tps65185_init);
module_exit(tps65185_exit);

MODULE_DESCRIPTION("ti tps65185 pmic");
MODULE_LICENSE("GPL");


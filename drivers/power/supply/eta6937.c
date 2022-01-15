/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros*/
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
//#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <linux/i2c.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
//#include "upmu_common.h"
#include "eta6937.h"
#include "charger_class.h"
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>

#include "mtk_charger.h"

static DEFINE_MUTEX(eta6937_i2c_access);
static DEFINE_MUTEX(eta6937_access_lock);

#define eta6937_SLAVE_ADDR_WRITE   0xD4
#define eta6937_SLAVE_ADDR_Read    0xD5

#define ETA6937_VENDER_CODE 0x02

#define HIGH_BATTERY_VOLTAGE_SUPPORT
#define HIGH_BATTERY_4_4_V
//#define YK676_V60_CUSTOMER_TRX_S607_HDPLUS

static int eta6937_changer_vender_code = 0;

#define ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
#ifdef ETA6937_TIMER_DEBUG 
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kthread.h>

wait_queue_head_t  eta6937_otg_wait_que;
struct hrtimer eta6937_otg_kthread_timer;
int otg_set_tmr_flag =1;
int eta6937_otg_status_flag =0;
void eta6937_set_tmr_rst(unsigned int val);

void _wake_up_eta6937_otg(void)
{
	otg_set_tmr_flag = 1;
	wake_up(&eta6937_otg_wait_que);
}

enum hrtimer_restart otg_rst_timer_func(struct hrtimer *timer)
{
	_wake_up_eta6937_otg();
	return HRTIMER_NORESTART;
}

void eta6937_otg_init_timer(void)
{
	ktime_t ktime = ktime_set(10, 0);
	hrtimer_init(&eta6937_otg_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	eta6937_otg_kthread_timer.function = otg_rst_timer_func;
	hrtimer_start(&eta6937_otg_kthread_timer, ktime, HRTIMER_MODE_REL);
}

void eta6937_otg_start_timer(void)
{
	ktime_t ktime = ktime_set(10, 0);

	printk("eta6937_otg_start_timer\n");
	hrtimer_start(&eta6937_otg_kthread_timer, ktime, HRTIMER_MODE_REL);
}

int eta6937_otg_routine_thread(void *arg)
{
	printk("eta6937_otg_routine_thread enter\n");
	while (1) {
		printk("eta6937_otg_routine_thread while(1)\n");
		wait_event(eta6937_otg_wait_que, (otg_set_tmr_flag == 1));
		otg_set_tmr_flag =0;
		if(eta6937_otg_status_flag ==1)
		{
			//xjl 20200528
			printk("eta6937_otg_routine_thread eta6937_set_tmr_rst(1)\n");
			eta6937_set_tmr_rst(1);
			eta6937_otg_start_timer();		
		}
	}
	
	return 0;
}
#endif

/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/

/*eta6937 REG06 VREG[5:0]*/
const u32 eta6937_VBAT_CV_VTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000,4000000, 4020000, 4040000, 
	4060000,4080000, 4100000, 4120000,
	4140000,4160000, 4180000, 4200000, 
	4220000,4240000, 4260000, 4280000,
	4300000,4320000, 4340000, 4360000,
	4380000,4400000, 4420000, 4440000
};

/*eta6937 REG04 ICHG[6:0]*/
const u32 eta6937_CS_VTH[] = {
	55000, 65000, 75000, 85000,
	95000, 105000, 115000, 125000,
	135000, 145000, 155000, 165000,
	175000, 185000, 195000, 205000,
	215000, 225000, 235000
};

/*eta6937 REG00 IINLIM[5:0]*/
const u32 eta6937_INPUT_CS_VTH[] = {
	10000, 50000, 80000,200000
};

#ifdef CONFIG_OF
#else

#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define eta6937_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define eta6937_BUSNUM 0
#endif

#endif

struct eta6937_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;
	//struct pinctrl *pinctrl;
	//struct pinctrl_state *psc_chg_en_low;
	//struct pinctrl_state *psc_chg_en_high;
} *g_eta6937_info;;

static const struct charger_properties eta6937_chg_props = {
	.alias_name = "eta6937", //eta6937-user
};

static unsigned int g_input_current;
DEFINE_MUTEX(g_input_current_mutex);
static struct i2c_client *new_client;
static const struct i2c_device_id eta6937_i2c_id[] = { {"eta6937", 0}, {} };

static int eta6937_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

static unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	if (val < array_size)
		return parameter[val];

	printk("eta6937 Can't find the parameter\n");
	return parameter[0];
}

static unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	unsigned int i;

	pr_debug_ratelimited("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	printk("eta6937 NO register value match\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					 unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				pr_debug_ratelimited("zzf_%d<=%d, i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}
		
		printk("eta6937 Can't find closest level\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}
		
		printk("eta6937 Can't find closest level\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char eta6937_reg[eta6937_REG_NUM] = { 0 };

int g_eta6937_hw_exist;

#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int eta6937_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&eta6937_i2c_access);

	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		new_client->ext_flag = 0;

		mutex_unlock(&eta6937_i2c_access);
		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	new_client->ext_flag = 0;

	mutex_unlock(&eta6937_i2c_access);
	return 1;
}

unsigned int eta6937_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&eta6937_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {

		new_client->ext_flag = 0;
		mutex_unlock(&eta6937_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&eta6937_i2c_access);
	return 1;
}
#else
unsigned int eta6937_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&eta6937_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = new_client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			printk("eta6937 skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&eta6937_i2c_access);

	return ret == xfers ? 1 : -1;
}

unsigned int eta6937_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&eta6937_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			printk("eta6937 skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&eta6937_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int eta6937_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char eta6937_reg = 0;
	unsigned int ret = 0;

	ret = eta6937_read_byte(RegNum, &eta6937_reg);

	printk("eta6937 [eta6937_read_interface] Reg[%x]=0x%x\n", RegNum, eta6937_reg);

	eta6937_reg &= (MASK << SHIFT);
	*val = (eta6937_reg >> SHIFT);

	printk("eta6937 [eta6937_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int eta6937_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char eta6937_reg = 0;
	unsigned char eta6937_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&eta6937_access_lock);
	ret = eta6937_read_byte(RegNum, &eta6937_reg);

	eta6937_reg_ori = eta6937_reg;
	eta6937_reg &= ~(MASK << SHIFT);
	eta6937_reg |= (val << SHIFT);

	ret = eta6937_write_byte(RegNum, eta6937_reg);
	mutex_unlock(&eta6937_access_lock);
	printk("eta6937 [eta6937_config_interface] write Reg[%x]=0x%x from 0x%x\n", RegNum,
		    eta6937_reg, eta6937_reg_ori);

	return ret;
}

/* write one register directly */
unsigned int eta6937_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = eta6937_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0 */

void eta6937_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
}

unsigned int eta6937_get_otg_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_OTG_MASK),
				     (unsigned char) (CON0_OTG_SHIFT)
	    );
	return val;
}

void eta6937_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_STAT_MASK),
				       (unsigned char) (CON0_EN_STAT_SHIFT)
	    );
}

#if 0
unsigned int eta6937_get_chip_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

#else
int eta6937_get_chip_status(struct charger_device *chg_dev, bool *done)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				(&val), (unsigned char) (CON0_STAT_MASK),(unsigned char) (CON0_STAT_SHIFT)
	    );
	switch (val) {
	case eta6937_CHG_STATUS_READY:
	case eta6937_CHG_STATUS_PROGRESS:
	case eta6937_CHG_STATUS_FAULT:
		*done = false;
		break;
	case eta6937_CHG_STATUS_DONE:
		*done = true;
		break;
	default:
		*done = false;
		break;
	}
	return ret;
}
#endif

unsigned int eta6937_get_boost_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_BOOST_MASK),
				     (unsigned char) (CON0_BOOST_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_fault_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void eta6937_set_input_charging_current(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LIN_LIMIT_MASK),
				       (unsigned char) (CON1_LIN_LIMIT_SHIFT)
	    );
}

void eta6937_set_v_low(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LOW_V_MASK),
				       (unsigned char) (CON1_LOW_V_SHIFT)
	    );
}

void eta6937_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_TE_MASK),
				       (unsigned char) (CON1_TE_SHIFT)
	    );
}

void eta6937_set_ce(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT)
	    );
}

void eta6937_set_hz_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_HZ_MODE_MASK),
				       (unsigned char) (CON1_HZ_MODE_SHIFT)
	    );
}

void eta6937_set_opa_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OPA_MODE_MASK),
				       (unsigned char) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void eta6937_set_oreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OREG_MASK),
				       (unsigned char) (CON2_OREG_SHIFT)
	    );
}

void eta6937_set_otg_pl(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_PL_MASK),
				       (unsigned char) (CON2_OTG_PL_SHIFT)
	    );
}

void eta6937_set_otg_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_EN_MASK),
				       (unsigned char) (CON2_OTG_EN_SHIFT)
	    );
}

/* CON3 */

unsigned int eta6937_get_vender_code(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON3),
				     (&val), (unsigned char) (CON3_VENDER_CODE_MASK),
				     (unsigned char) (CON3_VENDER_CODE_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON3),
				     (&val), (unsigned char) (CON3_PIN_MASK),
				     (unsigned char) (CON3_PIN_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON3),
				     (&val), (unsigned char) (CON3_REVISION_MASK),
				     (unsigned char) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void eta6937_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_RESET_MASK),
				       (unsigned char) (CON4_RESET_SHIFT)
	    );
	 eta6937_set_i_safe(ISAFE);
	 eta6937_set_v_safe(VSAFE);   
	    
}

void eta6937_set_iocharge(unsigned int val)
{
	unsigned int ret = 0;
	ret = eta6937_config_interface((unsigned char) (eta6937_CON4),
				       (unsigned char) (val%8),
				       (unsigned char) (CON4_I_CHR_MASK),
				       (unsigned char) (CON4_I_CHR_SHIFT)
	    );
	ret = eta6937_config_interface((unsigned char) (eta6937_CON5),
				       (unsigned char) (val/8),
				       (unsigned char) (CON5_I_CHR_MASK),
				       (unsigned char) (CON5_I_CHR_SHIFT)
	    );	
}

void eta6937_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_TERM_MASK),
				       (unsigned char) (CON4_I_TERM_SHIFT)
	    );
}

void eta6937_set_io_level(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IO_LEVEL_MASK),
				       (unsigned char) (CON5_IO_LEVEL_SHIFT)
	    );
}

unsigned int eta6937_get_sp_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON5),
				     (&val), (unsigned char) (CON5_SP_STATUS_MASK),
				     (unsigned char) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_en_level(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON5),
				     (&val), (unsigned char) (CON5_EN_LEVEL_MASK),
				     (unsigned char) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}

void eta6937_set_vsp(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_VSP_MASK),
				       (unsigned char) (CON5_VSP_SHIFT)
	    );
}

/* CON6 */

void eta6937_set_i_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_ISAFE_MASK),
				       (unsigned char) (CON6_ISAFE_SHIFT)
	    );
}

void eta6937_set_v_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VSAFE_MASK),
				       (unsigned char) (CON6_VSAFE_SHIFT)
	    );
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
static void eta6937_hw_component_detect(void) //xjl 20200526
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface(0x03, &val, 0xFF, 0x0);
	
	if (val == 0x51 || val == 0x52 || val == 0x53 || val == 0x54)
		g_eta6937_hw_exist = 0;
	else
		g_eta6937_hw_exist = 1;

	printk("[eta6937_hw_component_detect] exist=%d, Reg[0x03]=0x%x\n",
		 g_eta6937_hw_exist, val);
}

extern void set_gpio_charge_en(int enable);
static int eta6937_enable_charging(struct charger_device *chg_dev, bool en)
{
	//int ret;
	int status = 0;
	//struct eta6937_info *info = g_eta6937_info;
	if (en) {
		eta6937_set_ce(0);
		eta6937_set_hz_mode(0);
		eta6937_set_opa_mode(0);
		eta6937_set_i_safe(ISAFE);
	 	eta6937_set_v_safe(VSAFE); 
		eta6937_set_ce(0);
		eta6937_set_te(1);
		//eta6937_set_iterm(2);//150ma iterm
		eta6937_set_tmr_rst(1);
		set_gpio_charge_en(0); 
		//ret = pinctrl_select_state(info->pinctrl, info->psc_chg_en_low);
		//if (ret){
		//	printk("eta6937 mycat Error pinctrl_select_state low");
    	//}
    	//else{
      	//	printk("eta6937 mycat Ok pinctrl_select_state low");
    	//}
		printk("eta6937 eta6937 calm enable charging\n");
	} else {
		eta6937_set_ce(1);
		eta6937_set_hz_mode(1);
		//disable charging: psc_chg_en_high
		set_gpio_charge_en(1); 
		//ret = pinctrl_select_state(info->pinctrl, info->psc_chg_en_high);
		//if (ret){
		//	printk("eta6937 mycat Error pinctrl_select_state high");
    	//}
    	//else{
      	//	printk("eta6937 mycat Ok pinctrl_select_state high");
    	//}
		printk("eta6937 eta6937 calm disable charging\n");
	}

	return status;
}

static int eta6937_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	u32 status = 0;
	u32 array_size;
	u8 reg_value;
	/* Get current level */

	array_size = ARRAY_SIZE(eta6937_CS_VTH);
	eta6937_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*(u32 *) ichg = charging_value_to_parameter(eta6937_CS_VTH, array_size, reg_value);

	return status;
}

#if 1//defined(YK676_V60_CUSTOMER_TRX_S607_HDPLUS)
extern bool chr_current_limi;
#endif
static int eta6937_set_current(struct charger_device *chg_dev, u32 current_value)
{
	u32 status = 0;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;

	printk("liml_bat eta6937_set_current=%d\n",current_value);
#if 1//defined(defined(YK676_V60_CUSTOMER_TRX_S607_HDPLUS)
	if(chr_current_limi)
		return status;
#endif

	//插拔otg设备，寄存器值会reset到默认值，此时重设初始化值 add by wtwd 20211012
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	#if defined(HIGH_BATTERY_4_4_V)
	eta6937_reg_config_interface(0x06, 0x7B); //7F
	eta6937_reg_config_interface(0x02, 0xBA); //B6
	eta6937_reg_config_interface(0x01, 0xC8);
	eta6937_reg_config_interface(0x04, 0x03); //200ma
	#endif
#endif

	current_value /= 10;
	if (current_value <= 35000) {
		eta6937_set_io_level(1);
	} else {
		eta6937_set_io_level(0);
		array_size = ARRAY_SIZE(eta6937_CS_VTH);
		set_chr_current = bmt_find_closest_level(eta6937_CS_VTH, array_size, current_value);
		printk("eta6937 charging_set_current  set_chr_current=%d\n", set_chr_current);
		
		register_value = charging_parameter_to_value(eta6937_CS_VTH, array_size, set_chr_current);
		printk("eta6937 charging_set_current  register_value=%d\n", register_value);
		eta6937_set_iocharge(register_value);
	}
	return status;
}

static int eta6937_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	int ret = 0;
	*aicr = g_input_current;
	return ret;
}

static int eta6937_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	u32 status = 0;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;
 
	printk("liml_bat eta6937_set_input_current=%d\n",current_value);
	
	mutex_lock(&g_input_current_mutex);
	current_value /= 10;
	if (current_value > 80000) {
		register_value = 0x3;
	} else {
		array_size = ARRAY_SIZE(eta6937_INPUT_CS_VTH);
		set_chr_current = bmt_find_closest_level(eta6937_INPUT_CS_VTH, array_size, current_value);
		register_value = charging_parameter_to_value(eta6937_INPUT_CS_VTH, array_size, set_chr_current);
	}
	g_input_current = set_chr_current;
	//eta6937_set_input_charging_current(register_value);
	
	mutex_unlock(&g_input_current_mutex);
	return status;
}

static int eta6937_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	u32 status = 0;
	u32 register_value;
	u32 array_size;
	u32 set_cv_voltage;
	
	array_size = ARRAY_SIZE(eta6937_VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(eta6937_VBAT_CV_VTH, array_size, cv);
	register_value = charging_parameter_to_value(eta6937_VBAT_CV_VTH, ARRAY_SIZE(eta6937_VBAT_CV_VTH), set_cv_voltage);
	printk("eta6937 eta6937_set_cv_voltage  register_value=%d,set_cv_voltage=%d\n", register_value,set_cv_voltage);
	eta6937_set_oreg(register_value);

	return status;
}

//eta6937 otg要不断的喂狗
void eta6937_set_otg_enable(void)
{
	eta6937_set_opa_mode(1);
#ifdef ETA6937_TIMER_DEBUG
	eta6937_otg_status_flag = 1;
	eta6937_otg_start_timer();
#endif
	printk("eta6937_enable_otg enable\n");
}
EXPORT_SYMBOL(eta6937_set_otg_enable);

void eta6937_set_otg_disable(void)
{
	eta6937_set_opa_mode(0);
#ifdef ETA6937_TIMER_DEBUG
	eta6937_otg_status_flag = 0;
#endif
	printk("eta6937_enable_otg unable\n");
}
EXPORT_SYMBOL(eta6937_set_otg_disable);

//added by xen 20170928
static int eta6937_enable_otg(struct charger_device *chg_dev, bool en)
{
	if (en)
	{
		eta6937_set_opa_mode(1);
		printk("eta6937_enable_otg enable\n");
	}
	else
	{
		eta6937_set_opa_mode(0);
		printk("eta6937_enable_otg unable\n");
	}

#ifdef ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
	eta6937_otg_status_flag = en;
	if(en == 1)
		eta6937_otg_start_timer();
#endif
	//eta6937_set_otg_en(en); //xjl 20200528 for otg
	//eta6937_set_opa_mode(1);

	return 0;
}

#if 0
static int eta6937_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	printk("eta6937 %s: event = %d\n", __func__, event);
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}
#else
static int eta6937_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	int ret = 0;
	unsigned char val;
	
	if (chg_dev == NULL)
		return -EINVAL;
		
	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				(&val), (unsigned char) (CON0_STAT_MASK),(unsigned char) (CON0_STAT_SHIFT));
				
	printk("eta6937 %s: val = %d\n", __func__, val);
	switch (val)
	 {
	case 2:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case 1:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}
#endif

static int eta6937_dump_register(struct charger_device *chg_dev)
{
	int i = 0;
	
	printk("eta6937 [eta6937] ");
	for (i = 0; i <= eta6937_REG_NUM; i++) {
		eta6937_read_byte(i, &eta6937_reg[i]);
		printk("eta6937 [0x%x]=0x%x ", i, eta6937_reg[i]);
	}
	eta6937_set_tmr_rst(1);
	printk("eta6937 \n");

	return 0;
}

static int eta6937_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	eta6937_set_tmr_rst(1);
	return 0;
}

static struct charger_ops eta6937_chg_ops = {
	/* Normal charging */
	.dump_registers = eta6937_dump_register,
	.enable = eta6937_enable_charging,
	.get_charging_current = eta6937_get_current,
	.set_charging_current = eta6937_set_current,
	.set_input_current = eta6937_set_input_current,
	.get_input_current = eta6937_get_input_current,
	.set_constant_voltage = eta6937_set_cv_voltage,
	.kick_wdt = eta6937_reset_watch_dog_timer,
	.is_charging_done = eta6937_get_chip_status,
	/* OTG */
	.enable_otg = eta6937_enable_otg,
	.event = eta6937_do_event,
};

static int eta6937_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct eta6937_info *info = NULL;

	printk("eta6937 [eta6937_driver_probe] enter\n");

	info = devm_kzalloc(&client->dev, sizeof(struct eta6937_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	g_eta6937_info = info;
	client->addr = (eta6937_SLAVE_ADDR_WRITE>>1); //xjl 20200526
	new_client = client;
	info->dev = &client->dev;
	info->chg_dev_name = "primary_chg";
	//info->chg_props.alias_name = "eta6937";

	//ret = eta6937_parse_dt(info, &client->dev);
	//if (ret < 0)
	//	return ret;

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
		&client->dev, info, &eta6937_chg_ops, &eta6937_chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		printk("eta6937 %s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	/* --------------------- */
	eta6937_hw_component_detect();
	eta6937_changer_vender_code = eta6937_get_vender_code();
	printk("eta6937 get vendor id: (0x%x)\n", eta6937_changer_vender_code);
	/* eta6937_hw_init(); //move to charging_hw_xxx.c */

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	#if defined(HIGH_BATTERY_4_4_V)
		eta6937_reg_config_interface(0x06, 0x7B); //7F
		eta6937_reg_config_interface(0x02, 0xBA); //B6
	#else
		eta6937_reg_config_interface(0x06, 0x77);
		eta6937_reg_config_interface(0x02, 0xAA);
	#endif
#else
	eta6937_reg_config_interface(0x06, 0x70);
	eta6937_reg_config_interface(0x02, 0x8E);
#endif
	//设置充电不限电流add by wtwd 20211012
	eta6937_reg_config_interface(0x01, 0xC8);
	eta6937_reg_config_interface(0x04, 0x03); //200ma
	
	/*
	info->psy = power_supply_get_by_name("charger");
	if (!info->psy) {
		printk("eta6937 %s: get power supply failed\n", __func__);
		return -EINVAL;
	}
	*/

	eta6937_dump_register(info->chg_dev);
#ifdef ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
	init_waitqueue_head(&eta6937_otg_wait_que);
	kthread_run(eta6937_otg_routine_thread, 0, "eta6937_otg_thread");
	eta6937_otg_init_timer();
#endif

	printk("eta6937 [eta6937_driver_probe] successful\n");

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_eta6937;
static ssize_t show_eta6937_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("eta6937 [show_eta6937_access] 0x%x\n", g_reg_value_eta6937);
	return sprintf(buf, "%u\n", g_reg_value_eta6937);
}

static ssize_t store_eta6937_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	printk("eta6937 [store_eta6937_access]\n");

	if (buf != NULL && size != 0) {

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			pr_info(
			    "[store_eta6937_access] write eta6937 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = eta6937_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = eta6937_read_interface(reg_address, &g_reg_value_eta6937, 0xFF, 0x0);
			pr_info(
			    "[store_eta6937_access] read eta6937 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_eta6937);
			pr_info(
			    "[store_eta6937_access] Please use \"cat eta6937_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(eta6937_access, 0664, show_eta6937_access, store_eta6937_access);	/* 664 */

static int eta6937_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	printk("eta6937 ******** eta6937_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_eta6937_access);

	return 0;
}

struct platform_device eta6937_user_space_device = {
	.name = "eta6937-user",
	.id = -1,
};

static struct platform_driver eta6937_user_space_driver = {
	.probe = eta6937_user_space_probe,
	.driver = {
		   .name = "eta6937-user",
		   },
};

#ifdef CONFIG_OF
static const struct of_device_id eta6937_of_match[] = {
	{.compatible = "mediatek,eta6937"},
	{},
};
#else
static struct i2c_board_info i2c_eta6937 __initdata = {
	I2C_BOARD_INFO("eta6937", (eta6937_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver eta6937_driver = {
	.driver = {
		   .name = "eta6937",
#ifdef CONFIG_OF
		   .of_match_table = eta6937_of_match,
#endif
		   },
	.probe = eta6937_driver_probe,
	.id_table = eta6937_i2c_id,
};

static int __init eta6937_init(void)
{
	int ret = 0;

	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	printk("eta6937 [eta6937_init] init start with i2c DTS");
#else
	printk("eta6937 [eta6937_init] init start. ch=%d\n", eta6937_BUSNUM);
	i2c_register_board_info(eta6937_BUSNUM, &i2c_eta6937, 1);
#endif
	if (i2c_add_driver(&eta6937_driver) != 0) {
		pr_info(
			    "[eta6937_init] failed to register eta6937 i2c driver.\n");
	} else {
		pr_info(
			    "[eta6937_init] Success to register eta6937 i2c driver.\n");
	}

	/* eta6937 user space access interface */
	ret = platform_device_register(&eta6937_user_space_device);
	if (ret) {
		printk("eta6937 ****[eta6937_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&eta6937_user_space_driver);
	if (ret) {
		printk("eta6937 ****[eta6937_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

static void __exit eta6937_exit(void)
{
	printk("[eta6937_exit] enter\n");

	i2c_del_driver(&eta6937_driver);
}
module_init(eta6937_init);
module_exit(eta6937_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C eta6937 Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");

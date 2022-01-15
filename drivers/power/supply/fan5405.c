#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include "linux/kthread.h"

#include <linux/sched/clock.h>
#include <uapi/linux/sched/types.h>


#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
//#include <mt-plat/battery_meter.h>
//#include <mt-plat/battery_common.h>
//#include <mt-plat/charging.h>
#include "fan5405.h"
//#include "upmu_common.h"
#include "charger_class.h"
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>

#include "mtk_charger.h"
#define fan5405_SLAVE_ADDR_WRITE   0xD4
#define fan5405_SLAVE_ADDR_READ    0xD5

#define PSC5415A_VENDER_CODE 0x07
#define BQ24157_VENDER_CODE 0x02

#define YK_CHARGER_USE_PCS5425E 1


#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
#define OTG_NAME "psc5425a_otg"

int otg_thread_status = 0;
static DECLARE_WAIT_QUEUE_HEAD(otg_waiter);
static int psc5425a_thread(void *unused);
struct task_struct *thread_psc5425a = NULL;
#endif
static int changer_vender_code =0;


const u32 VBAT_CV_VTH[] = {
	BATTERY_VOLT_03_500000_V, BATTERY_VOLT_03_520000_V, BATTERY_VOLT_03_540000_V,
	    BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V, BATTERY_VOLT_03_600000_V, BATTERY_VOLT_03_620000_V,
	    BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V, BATTERY_VOLT_03_680000_V, BATTERY_VOLT_03_700000_V,
	    BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V, BATTERY_VOLT_03_760000_V, BATTERY_VOLT_03_780000_V,
	    BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V, BATTERY_VOLT_03_840000_V, BATTERY_VOLT_03_860000_V,
	    BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V, BATTERY_VOLT_03_920000_V, BATTERY_VOLT_03_940000_V,
	    BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V, BATTERY_VOLT_04_000000_V, BATTERY_VOLT_04_020000_V,
	    BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V, BATTERY_VOLT_04_080000_V, BATTERY_VOLT_04_100000_V,
	    BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V, BATTERY_VOLT_04_160000_V, BATTERY_VOLT_04_180000_V,
	    BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V, BATTERY_VOLT_04_240000_V, BATTERY_VOLT_04_260000_V,
	    BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_320000_V, BATTERY_VOLT_04_340000_V,
	    BATTERY_VOLT_04_360000_V,
	BATTERY_VOLT_04_380000_V, BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_420000_V,
	    BATTERY_VOLT_04_440000_V
};

/***********************************************************************************
//note by xen 20170817
//if use PSC5425 charger IC,actually current value is:
600mA,1000mA,1300mA,1600mA,1900mA,2200mA,2410mA,NG  (Rsense=0.033)
495mA,925mA, 1073mA,1320mA,1565mA,1815mA,1980mA,2310  (Rsense=0.040)
***********************************************************************************/

const u32 CS_VTH[] = {
#if defined(YK_CHARGER_USE_PCS5425E)
	CHARGE_CURRENT_700_00_MA, CHARGE_CURRENT_850_00_MA,CHARGE_CURRENT_1150_00_MA,
	CHARGE_CURRENT_1250_00_MA,
	CHARGE_CURRENT_1500_00_MA,CHARGE_CURRENT_1750_00_MA,CHARGE_CURRENT_1900_00_MA,
	CHARGE_CURRENT_2000_00_MA
#else
	CHARGE_CURRENT_550_00_MA, CHARGE_CURRENT_650_00_MA, CHARGE_CURRENT_750_00_MA,
	    CHARGE_CURRENT_850_00_MA,
#if 0 //FAN5405
	CHARGE_CURRENT_950_00_MA, CHARGE_CURRENT_1050_00_MA, CHARGE_CURRENT_1150_00_MA,
	    CHARGE_CURRENT_1250_00_MA
#else //FAN54015
    CHARGE_CURRENT_1050_00_MA, CHARGE_CURRENT_1150_00_MA, CHARGE_CURRENT_1350_00_MA, 
        CHARGE_CURRENT_1450_00_MA
#endif
#endif
};

//xjl 20200401
const u32 CS_VTH_XILIJIE[] = {
	CHARGE_CURRENT_550_00_MA, CHARGE_CURRENT_650_00_MA, CHARGE_CURRENT_750_00_MA,
	    CHARGE_CURRENT_850_00_MA,
    	CHARGE_CURRENT_950_00_MA,CHARGE_CURRENT_1050_00_MA, CHARGE_CURRENT_1150_00_MA, 
	    CHARGE_CURRENT_1250_00_MA
};

const u32 INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_800_00_MA,
	    CHARGE_CURRENT_MAX
};

const u32 VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
					const unsigned int val)
{
	if (val < array_size)
		return parameter[val];
	//battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
	printk("Can't find the parameter \r\n");
	return parameter[0];
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
					const unsigned int val)
{
	u32 i;

	for (i = 0; i < array_size; i++)
		if (val == *(parameter + i))
			return i;

	//battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	printk("NO register value match \r\n");

	return 0;
}


static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1; //KAL_TRUE;
	else
		max_value_in_last_element = 0; //KAL_FALSE;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0; i--)	/* max value in the last element */
			if (pList[i] <= level)
				return pList[i];

		printk("Can't find closest level, small value first \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++)	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];

		printk("Can't find closest level, large value first \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

struct fan5405_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;
};

static struct i2c_client *new_client;
static const struct i2c_device_id fan5405_i2c_id[] = { {"fan5405", 0}, {} };

int chargin_hw_init_done = 0;
static int fan5405_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id fan5405_of_match[] = {
	{.compatible = "mediatek,fan5405",},
	{},
};

MODULE_DEVICE_TABLE(of, fan5405_of_match);
#endif

static struct i2c_driver fan5405_driver = {
	.driver = {
		   .name = "fan5405",
#ifdef CONFIG_OF
		   .of_match_table = fan5405_of_match,
#endif
	},
	.probe = fan5405_driver_probe,
	.id_table = fan5405_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char fan5405_reg[fan5405_REG_NUM] = { 0 };

static DEFINE_MUTEX(fan5405_i2c_access);

int g_fan5405_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write fan5405]
  *
  *********************************************************/
int fan5405_read_byte(unsigned char cmd, unsigned char *returnData)
{
	//char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;
	struct i2c_msg msg[2]; //modified by xen 20170911
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&fan5405_i2c_access);

#if 1 //modified by xen 20170911
	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &cmd;

	msg[1].addr = new_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &readData;

	ret = i2c_transfer(adap, msg, 2);
	if (ret < 0) {
		mutex_unlock(&fan5405_i2c_access);
		return 0;
	}
#else
	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		new_client->ext_flag = 0;

		mutex_unlock(&fan5405_i2c_access);
		return 0;
	}
	readData = cmd_buf[0];
#endif
	*returnData = readData;

	//new_client->ext_flag = 0;

	mutex_unlock(&fan5405_i2c_access);
	return 1;
}

int fan5405_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
	struct i2c_msg msg; //modified by xen 20170911
	struct i2c_adapter *adap = new_client->adapter;

#if 1 //modified by xen 20170911
	mutex_lock(&fan5405_i2c_access);
	write_data[0] = cmd;
	write_data[1] = writeData;
	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = (char *)write_data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret < 0) {
		mutex_unlock(&fan5405_i2c_access);
		return 0;
	}
#else
	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {

		new_client->ext_flag = 0;
		mutex_unlock(&fan5405_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
#endif
	mutex_unlock(&fan5405_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int fan5405_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan5405_reg = 0;
	int ret = 0;

	ret = fan5405_read_byte(RegNum, &fan5405_reg);

	printk("[fan5405_read_interface] Reg[%x]=0x%x\n", RegNum, fan5405_reg);

	fan5405_reg &= (MASK << SHIFT);
	*val = (fan5405_reg >> SHIFT);

	printk("[fan5405_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int fan5405_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan5405_reg = 0;
	int ret = 0;

	ret = fan5405_read_byte(RegNum, &fan5405_reg);
	printk("[fan5405_config_interface] Reg[%x]=0x%x\n", RegNum, fan5405_reg);

	fan5405_reg &= ~(MASK << SHIFT);
	fan5405_reg |= (val << SHIFT);

	if (RegNum == fan5405_CON4 && val == 1 && MASK == CON4_RESET_MASK
	    && SHIFT == CON4_RESET_SHIFT) {
		/* RESET bit */
	} else if (RegNum == fan5405_CON4) {
		fan5405_reg &= ~0x80;	/* RESET bit read returs 1, so clear it */
	}

	ret = fan5405_write_byte(RegNum, fan5405_reg);
	printk("[fan5405_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    fan5405_reg);

	return ret;
}

/* write one register directly */
unsigned int fan5405_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	int ret = 0;

	ret = fan5405_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0 */

void fan5405_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
}

unsigned int fan5405_get_otg_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_OTG_MASK),
				     (unsigned char) (CON0_OTG_SHIFT)
	    );
	return val;
}

void fan5405_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_STAT_MASK),
				       (unsigned char) (CON0_EN_STAT_SHIFT)
	    );
}

unsigned int fan5405_get_chip_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_boost_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_BOOST_MASK),
				     (unsigned char) (CON0_BOOST_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_fault_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void fan5405_set_input_charging_current(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LIN_LIMIT_MASK),
				       (unsigned char) (CON1_LIN_LIMIT_SHIFT)
	    );
}

void fan5405_set_v_low(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LOW_V_MASK),
				       (unsigned char) (CON1_LOW_V_SHIFT)
	    );
}

void fan5405_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_TE_MASK),
				       (unsigned char) (CON1_TE_SHIFT)
	    );
}

void fan5405_set_ce(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT)
	    );
}

void fan5405_set_hz_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_HZ_MODE_MASK),
				       (unsigned char) (CON1_HZ_MODE_SHIFT)
	    );
}

void fan5405_set_opa_mode(unsigned int val)
{
	unsigned int ret = 0;

	printk("fan5405_set_opa_mode: otg: val=%d \n", val);

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OPA_MODE_MASK),
				       (unsigned char) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void fan5405_set_oreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OREG_MASK),
				       (unsigned char) (CON2_OREG_SHIFT)
	    );
}

void fan5405_set_otg_pl(unsigned int val)
{
	unsigned int ret = 0;

	printk("fan5405_set_otg_pl: otg: val=%d \n", val);


	ret = fan5405_config_interface((unsigned char) (fan5405_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_PL_MASK),
				       (unsigned char) (CON2_OTG_PL_SHIFT)
	    );
}

void fan5405_set_otg_en(unsigned int val)
{
	unsigned int ret = 0;

	printk("fan5405_set_otg_en: otg: en=%d \n", val);

	ret = fan5405_config_interface((unsigned char) (fan5405_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_EN_MASK),
				       (unsigned char) (CON2_OTG_EN_SHIFT)
	    );
}


/* CON3 */

unsigned int fan5405_get_vender_code(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON3),
				     (&val), (unsigned char) (CON3_VENDER_CODE_MASK),
				     (unsigned char) (CON3_VENDER_CODE_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON3),
				     (&val), (unsigned char) (CON3_PIN_MASK),
				     (unsigned char) (CON3_PIN_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON3),
				     (&val), (unsigned char) (CON3_REVISION_MASK),
				     (unsigned char) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void fan5405_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_RESET_MASK),
				       (unsigned char) (CON4_RESET_SHIFT)
	    );
}

void fan5405_set_iocharge(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_CHR_MASK),
				       (unsigned char) (CON4_I_CHR_SHIFT)
	    );
}

void fan5405_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_TERM_MASK),
				       (unsigned char) (CON4_I_TERM_SHIFT)
	    );
}

/* CON5 */

void fan5405_set_dis_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_DIS_VREG_MASK),
				       (unsigned char) (CON5_DIS_VREG_SHIFT)
	    );
}

void fan5405_set_io_level(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IO_LEVEL_MASK),
				       (unsigned char) (CON5_IO_LEVEL_SHIFT)
	    );
}

unsigned int fan5405_get_sp_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON5),
				     (&val), (unsigned char) (CON5_SP_STATUS_MASK),
				     (unsigned char) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_en_level(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON5),
				     (&val), (unsigned char) (CON5_EN_LEVEL_MASK),
				     (unsigned char) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}

void fan5405_set_vsp(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_VSP_MASK),
				       (unsigned char) (CON5_VSP_SHIFT)
	    );
}

/* CON6 */

void fan5405_set_i_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_ISAFE_MASK),
				       (unsigned char) (CON6_ISAFE_SHIFT)
	    );
}

void fan5405_set_v_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON6),
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
void fan5405_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface(0x03, &val, 0xFF, 0x0);

	if (val == 0)
		g_fan5405_hw_exist = 0;
	else
		g_fan5405_hw_exist = 1;

	printk("[fan5405_hw_component_detect] exist=%d, Reg[03]=0x%x\n", g_fan5405_hw_exist, val);
}

int is_fan5405_exist(void)
{
	printk("[is_fan5405_exist] g_fan5405_hw_exist=%d\n", g_fan5405_hw_exist);

	return g_fan5405_hw_exist;
}

static int fan5405_dump_register(struct charger_device *chg_dev)//void)
{
	int i = 0;

	printk("[fan5405] ");
	for (i = 0; i < fan5405_REG_NUM; i++) {
		fan5405_read_byte(i, &fan5405_reg[i]);
		printk("[0x%x]=0x%x ", i, fan5405_reg[i]);
	}
	printk("\n");
	return 0;
}

#if 1//def YK_PSC5415A_SUPPORT
extern void set_gpio_charge_en(int enable);
#endif

#if 0
static int fan5405_enable_charging_enable(void)
{
		fan5405_set_ce(0);
		#if 1//def YK_PSC5415A_SUPPORT
		if(1) //(PSC5415A_VENDER_CODE==changer_vender_code) //xjl 20200422 for xilijie
        	set_gpio_charge_en(0); 
		#endif
		fan5405_set_hz_mode(0);
		fan5405_set_opa_mode(0);
}

static int fan5405_do_event(struct charger_device *chg_dev, unsigned int event, unsigned int args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	printk("%s: event = %d\n", __func__, event);
	switch (event) {
	case EVENT_FULL:
			set_gpio_charge_en(1); 
		break;
	case EVENT_RECHARGE:
		fan5405_enable_charging_enable(); 
		break;
	case EVENT_DISCHARGE:
		set_gpio_charge_en(1); 
		break;
	default:
		break;
	}

	return 0;
}
#endif

static int fan5405_enable_charging(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;

printk("fan5405_enable_charging:%d\n", en);
	if (en) {
		fan5405_set_ce(0);
		#if 1//def YK_PSC5415A_SUPPORT
		if(1) //(PSC5415A_VENDER_CODE==changer_vender_code) //xjl 20200422 for xilijie
        	set_gpio_charge_en(0); 
		#endif
		fan5405_set_hz_mode(0);
		fan5405_set_opa_mode(0);
	} else {
#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device())
#endif
        {
		#if 1//def YK_PSC5415A_SUPPORT
		if(1) //(PSC5415A_VENDER_CODE==changer_vender_code) //xjl 20200422 for xilijie
            	set_gpio_charge_en(1); 
		#endif
			//fan5405_set_ce(1); //xjl 20200422 for xilijie
        }
	}

	return status;
}

static int fan5405_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	int status = 0;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;
	/*static kal_int16 pre_register_value; */
	array_size = ARRAY_SIZE(VBAT_CV_VTH);
	/*pre_register_value = -1; */
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv);

	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size, set_cv_voltage);
	printk("fan5405_set_cv_voltage:charging_set_cv_voltage register_value=0x%x %d %d\n", register_value, cv, set_cv_voltage);
	fan5405_set_oreg(register_value);

	return status;
}

static int fan5405_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	int status = 0;
	unsigned int array_size;
	unsigned char reg_value;

if(PSC5415A_VENDER_CODE==changer_vender_code) //xjl 20200401
{
	array_size = ARRAY_SIZE(CS_VTH);
	fan5405_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*ichg = charging_value_to_parameter(CS_VTH, array_size, reg_value);
}
else
{
	array_size = ARRAY_SIZE(CS_VTH_XILIJIE);
	fan5405_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*ichg = charging_value_to_parameter(CS_VTH_XILIJIE, array_size, reg_value);
}
	return status;
}
#if 1//defined(YK676_CUSTOMER_TRX_S606_HDPLUS)
extern bool chr_current_limi;
#endif
static int fan5405_set_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	printk("fan5405_set_current :%d\n", current_value);

#if 0//defined(YK676_CUSTOMER_TRX_S606_HDPLUS)
        if(chr_current_limi)
           return status;
#endif
	if (current_value <= 350000) { //modified by xen for test 20180117
		fan5405_set_io_level(1);
	} else {
		fan5405_set_io_level(0);
		if(PSC5415A_VENDER_CODE==changer_vender_code) //xjl 20200401
		{
		array_size = ARRAY_SIZE(CS_VTH);
		set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
		register_value = charging_parameter_to_value(CS_VTH, array_size, set_chr_current);
		}
		else
		{
		array_size = ARRAY_SIZE(CS_VTH_XILIJIE);
		set_chr_current = bmt_find_closest_level(CS_VTH_XILIJIE, array_size, current_value);
		register_value = charging_parameter_to_value(CS_VTH_XILIJIE, array_size, set_chr_current);
		}
		printk("fan5405_set_current:charging_set_current register_value=0x%x %d %d\n", register_value, current_value, set_chr_current);   

		fan5405_set_iocharge(register_value);
	}
#if 1//defined(YK676_CUSTOMER_TRX_S606_HDPLUS)
        if(chr_current_limi)
		{
		   fan5405_set_io_level(0);
		   fan5405_set_iocharge(0);
		}
#endif	

	return status;
}

/*
static int fan5405_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	unsigned int status = 0;
	unsigned int array_size;
	unsigned int register_value;

	array_size = ARRAY_SIZE(INPUT_CSTH);
	register_value = fan5405_get_input_charging_current();
	*aicr = charging_parameter_to_value(INPUT_CSTH, array_size, register_value);

	return status;
}*/

static int fan5405_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	printk("fan5405_set_input_current :%d\n", current_value);
	if (current_value > 500000) {  //modified by xen for test 20180117,CHARGE_CURRENT_500_00_MA
		register_value = 0x3;
	} else {
		array_size = ARRAY_SIZE(INPUT_CS_VTH);
		set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
		register_value =
	 charging_parameter_to_value(INPUT_CS_VTH, array_size, set_chr_current);
	}

//xjl 20191101 guofang charge
	if(register_value == 0x0)
		register_value = 0x1;

	//fan5405_set_input_charging_current(register_value);

	return status;
}

static int fan5405_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	unsigned int status = 0;
	unsigned int ret_val;

	ret_val = fan5405_get_chip_status();
	printk("fan5405_get_charging_status = %d\n",ret_val);
	if (ret_val == 0x2)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
void set_wake_up_otg(int wake_up_otg)
{
	otg_thread_status = wake_up_otg;
	printk("fan5405 otg_thread_status = %d\n",otg_thread_status);
	wake_up_interruptible(&otg_waiter);
}
#endif

void fan5405_set_otg_enable(void)
{
	    printk("fan5405_set_otg_enable: otg: \n");
		fan5405_set_opa_mode(1);
		fan5405_set_otg_pl(0);
		fan5405_set_otg_en(1);
#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
		set_wake_up_otg(1);		
#endif	
}
EXPORT_SYMBOL(fan5405_set_otg_enable);

void fan5405_set_otg_disable(void)
{
	    printk("fan5405_set_otg_disable: otg: \n");
		fan5405_reg_config_interface(0x01, 0xf0);//xjl 20191104 0x30-0xf0
		fan5405_reg_config_interface(0x02, 0x8e);
#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
		set_wake_up_otg(0);		
#endif			
}
EXPORT_SYMBOL(fan5405_set_otg_disable);
//added by xen 20170928
static int fan5405_charger_enable_otg(struct charger_device *chg_dev, bool en)
{
	printk("fan5405_charger_enable_otg en=%d \n", en);
	if (en)
	{
		fan5405_set_opa_mode(1);
		fan5405_set_otg_pl(0);
		fan5405_set_otg_en(1);
#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
		set_wake_up_otg(1);		
#endif		
	}
	else
	{
		printk("fan5405_charger_enable_otg 0x01 0xf0\n");
		fan5405_reg_config_interface(0x01, 0xf0);//xjl 20191104 0x30-0xf0
		fan5405_reg_config_interface(0x02, 0x8e);
#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
		set_wake_up_otg(0);		
#endif
	}
	return 0;
}

static int fan5405_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	fan5405_set_tmr_rst(1);
	return 0;
}

static struct charger_ops fan5405_chg_ops = {

	/* Normal charging */
	.dump_registers = fan5405_dump_register,
	.enable = fan5405_enable_charging,
	.get_charging_current = fan5405_get_current,
	.set_charging_current = fan5405_set_current,
	//.get_input_current = fan5405_get_input_current,
	.set_input_current = fan5405_set_input_current,
	/*.get_constant_voltage = hl7005_get_battery_voreg,*/
	.set_constant_voltage = fan5405_set_cv_voltage,
	.kick_wdt = fan5405_reset_watch_dog_timer,

	.enable_otg = fan5405_charger_enable_otg,
	.is_charging_done = fan5405_get_charging_status,

	//.event = fan5405_do_event,
};

#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
static int psc5425a_thread(void *unused)
{
	//int i;
	//chr_err("zhb====> psc5425a_thread\n");
	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(otg_waiter,(otg_thread_status!=0));
		set_current_state(TASK_RUNNING);
		//psc5425a_read_byte(1, &psc5425a_reg[1]);
		fan5405_read_byte(1, &fan5405_reg[1]);
		printk("fan5405 psc5425a_thread [0x01]=0x%x ", fan5405_reg[1]);
                // light modify 20000 to 1000
		mdelay(1000);
	} while (!kthread_should_stop());

	return 0;
}
#endif

static int fan5405_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	static bool charging_init_flag = 0;
	struct fan5405_info *info = NULL;
	info = devm_kzalloc(&client->dev, sizeof(struct fan5405_info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	client->addr = (fan5405_SLAVE_ADDR_WRITE>>1);
	new_client = client;
//xen 20170927
	info->dev = &client->dev;
	info->chg_dev_name = "primary_chg";
	info->chg_props.alias_name = "fan5405";
//end

          //new_client->timing = 100; //zxs 20160201,modified by xen 20170911

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
		&client->dev, info, &fan5405_chg_ops, &info->chg_props);

	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_err("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
	thread_psc5425a = kthread_run(psc5425a_thread,0, OTG_NAME);
	if (IS_ERR(thread_psc5425a)) {
		printk("fan5405 mouse register kthread_run failed (%s)\n", __func__);

  	}
#endif

	fan5405_hw_component_detect();

	#if 1//defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
		#if 1//defined(HIGH_BATTERY_4_4_V)
         fan5405_reg_config_interface(0x06,0x7A); //ISAFE = 1450mA, VSAFE = 4.4V
		#else
         fan5405_reg_config_interface(0x06,0x77); // ISAFE = 1250mA, VSAFE = 4.34V
		#endif
	#else
	fan5405_reg_config_interface(0x06,0x70);
	#endif

	fan5405_reg_config_interface(0x00, 0xC0);	/* kick chip watch dog */
	fan5405_reg_config_interface(0x01, 0xc8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	fan5405_reg_config_interface(0x02, 0xBE); //0xB2 //xjl 20200414 for 4.3V charge //0xAE
	fan5405_reg_config_interface(0x05, 0x04);  //0x03   yx 20161201
	if (!charging_init_flag) {
#if defined(YK736_CUSTOMER_CAIFU9_KS939A_HDPLUS)
                fan5405_reg_config_interface(0x04, 0x0A);	/* 146mA */
#else
		fan5405_reg_config_interface(0x04, 0x5C);	/* 146mA */ //0x1A
#endif
		charging_init_flag = 1;
	}

	changer_vender_code=fan5405_get_vender_code();
	printk("[fan5405_driver_probe] fan5405_vendor_code = 0x%x\n", changer_vender_code); //xjl 20200401
	fan5405_dump_register(info->chg_dev);
	chargin_hw_init_done = 1;

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_fan5405 = 0;
static ssize_t show_fan5405_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[show_fan5405_access] 0x%x\n", g_reg_value_fan5405);
	return sprintf(buf, "%u\n", g_reg_value_fan5405);
}

static ssize_t store_fan5405_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	printk("[store_fan5405_access]\n");

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

			printk("[store_fan5405_access] write fan5405 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = fan5405_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = fan5405_read_interface(reg_address, &g_reg_value_fan5405, 0xFF, 0x0);
			printk("[store_fan5405_access] read fan5405 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_fan5405);
			printk("[store_fan5405_access] Please use \"cat fan5405_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(fan5405_access, 0664, show_fan5405_access, store_fan5405_access);	/* 664 */

static int fan5405_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	printk("******** fan5405_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_fan5405_access);

	return 0;
}

struct platform_device fan5405_user_space_device = {
	.name = "fan5405-user",
	.id = -1,
};

static struct platform_driver fan5405_user_space_driver = {
	.probe = fan5405_user_space_probe,
	.driver = {
		   .name = "fan5405-user",
	},
};

static int __init fan5405_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&fan5405_driver) != 0) {
		printk("[fan5405_init] failed to register fan5405 i2c driver.\n");
	} else {
		printk("[fan5405_init] Success to register fan5405 i2c driver.\n");
	}

	/* fan5405 user space access interface */
	ret = platform_device_register(&fan5405_user_space_device);
	if (ret) {
		printk("****[fan5405_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&fan5405_user_space_driver);
	if (ret) {
		printk("****[fan5405_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

static void __exit fan5405_exit(void)
{
#if defined(YK_CHARGER_USE_PCS5425E) //xjl 20190826
	if(thread_psc5425a){
		chr_err(" cannel thread_psc5425a\n");
		kthread_stop(thread_psc5425a);
	}
#endif

	i2c_del_driver(&fan5405_driver);
}

module_init(fan5405_init);
module_exit(fan5405_exit);
//subsys_initcall(fan5405_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C fan5405 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");

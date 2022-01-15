// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/module.h>

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/upmu_hw.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>	/* hwPowerOn */
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#else
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
//#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#else
#include <mt-plat/mt_gpio.h>
#endif

#include "lcm_define.h"
#include "lcm_drv.h"
#include "lcm_gpio.h"


#ifdef CONFIG_MTK_LEGACY
#if defined(GPIO_LCD_BIAS_ENP_PIN)
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#else
#define GPIO_65132_EN 0
#endif
#else
#ifdef CONFIG_PINCTRL
static struct pinctrl *_lcm_gpio;
static struct pinctrl_state *_lcm_gpio_mode_default;
static struct pinctrl_state *_lcm_gpio_mode[MAX_LCM_GPIO_MODE];
static unsigned char _lcm_gpio_mode_list[MAX_LCM_GPIO_MODE][128] = {
	"lcm_mode_00",
	"lcm_mode_01",
	"lcm_mode_02",
	"lcm_mode_03",
	"lcm_mode_04",
	"lcm_mode_05",
	"lcm_mode_06",
	"lcm_mode_07"
};

static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_BL_EN;
#endif

/* function definitions */
static int __init _lcm_gpio_init(void);
static void __exit _lcm_gpio_exit(void);
static int _lcm_gpio_probe(struct platform_device *pdev);
static int _lcm_gpio_remove(struct platform_device *pdev);

#ifdef CONFIG_OF
static const struct of_device_id _lcm_gpio_of_ids[] = {
	{.compatible = "mediatek,lcm_mode",},
	{},
};
MODULE_DEVICE_TABLE(of, _lcm_gpio_of_ids);
#endif

static struct platform_driver _lcm_gpio_driver = {
	.driver = {
		.name = LCM_GPIO_DEVICE,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(_lcm_gpio_of_ids),
	},
	.probe = _lcm_gpio_probe,
	.remove = _lcm_gpio_remove,
};
module_platform_driver(_lcm_gpio_driver);
#endif


#ifdef CONFIG_MTK_LEGACY
#else
/* LCM GPIO probe */
static int _lcm_gpio_probe(struct platform_device *pdev)
{
#ifdef CONFIG_PINCTRL
	int ret;
	unsigned int mode;
	const struct of_device_id *match;
	struct device	*dev = &pdev->dev;

	pr_debug("[LCM][GPIO] enter %s, %d\n", __func__, __LINE__);

	_lcm_gpio = devm_pinctrl_get(dev);
	if (IS_ERR(_lcm_gpio)) {
		ret = PTR_ERR(_lcm_gpio);
		pr_info("[LCM][ERROR] Cannot find _lcm_gpio!\n");
		return ret;
	}
	_lcm_gpio_mode_default = pinctrl_lookup_state(_lcm_gpio, "default");
	if (IS_ERR(_lcm_gpio_mode_default)) {
		ret = PTR_ERR(_lcm_gpio_mode_default);
		pr_info("[LCM][ERROR] Cannot find lcm_mode_default %d!\n",
			ret);
	}
	for (mode = LCM_GPIO_MODE_00; mode < MAX_LCM_GPIO_MODE; mode++) {
		_lcm_gpio_mode[mode] =
			pinctrl_lookup_state(_lcm_gpio,
				_lcm_gpio_mode_list[mode]);
		if (IS_ERR(_lcm_gpio_mode[mode]))
			pr_info("[LCM][ERROR] Cannot find lcm_mode:%d! skip it.\n",
			mode);

	}

	if (dev->of_node) {
		match = of_match_device(of_match_ptr(_lcm_gpio_of_ids), dev);
		if (!match) {
			pr_info("[LCM][ERROR] No device match found\n");
			return -ENODEV;
		}
	}
	GPIO_LCD_PWR_EN =
		of_get_named_gpio(dev->of_node, "lcm_power_gpio", 0);
	GPIO_LCD_BL_EN =
		of_get_named_gpio(dev->of_node, "lcm_bl_gpio", 0);

	ret = gpio_request(GPIO_LCD_PWR_EN, "lcm_power_gpio");
	if (ret < 0)
		pr_info("[LCM][ERROR] Unable to request GPIO_LCD_PWR_EN\n");
	ret = gpio_request(GPIO_LCD_BL_EN, "lcm_bl_gpio");
	if (ret < 0)
		pr_info("[LCM][ERROR] Unable to request GPIO_LCD_BL_EN\n");

	pr_debug("[LCM][GPIO] _lcm_gpio_get_info end!\n");
#endif

	return 0;
}


static int _lcm_gpio_remove(struct platform_device *pdev)
{
#ifdef CONFIG_PINCTRL
	gpio_free(GPIO_LCD_BL_EN);
	gpio_free(GPIO_LCD_PWR_EN);
#endif

	return 0;
}


/* called when loaded into kernel */
static int __init _lcm_gpio_init(void)
{
	pr_debug("MediaTek LCM GPIO driver init\n");
	if (platform_driver_register(&_lcm_gpio_driver) != 0) {
		pr_info("unable to register LCM GPIO driver.\n");
		return -1;
	}
	return 0;
}


/* should never be called */
static void __exit _lcm_gpio_exit(void)
{
	pr_debug("MediaTek LCM GPIO driver exit\n");
	platform_driver_unregister(&_lcm_gpio_driver);
}
#endif


static enum LCM_STATUS _lcm_gpio_check_data(char type,
	const struct LCM_DATA_T1 *t1)
{
	switch (type) {
	case LCM_GPIO_MODE:
		switch (t1->data) {
		case LCM_GPIO_MODE_00:
		case LCM_GPIO_MODE_01:
		case LCM_GPIO_MODE_02:
		case LCM_GPIO_MODE_03:
		case LCM_GPIO_MODE_04:
		case LCM_GPIO_MODE_05:
		case LCM_GPIO_MODE_06:
		case LCM_GPIO_MODE_07:
			break;

		default:
			pr_info("[LCM][ERROR] %s/%d: %d, %d\n",
				__func__, __LINE__, type, t1->data);
			return LCM_STATUS_ERROR;
		}
		break;

	case LCM_GPIO_DIR:
		switch (t1->data) {
		case LCM_GPIO_DIR_IN:
		case LCM_GPIO_DIR_OUT:
			break;

		default:
			pr_info("[LCM][ERROR] %s/%d: %d, %d\n",
				__func__, __LINE__, type, t1->data);
			return LCM_STATUS_ERROR;
		}
		break;

	case LCM_GPIO_OUT:
		switch (t1->data) {
		case LCM_GPIO_OUT_ZERO:
		case LCM_GPIO_OUT_ONE:
			break;

		default:
			pr_info("[LCM][ERROR] %s/%d: %d, %d\n",
				__func__, __LINE__, type, t1->data);
			return LCM_STATUS_ERROR;
		}
		break;

	default:
		pr_info("[LCM][ERROR] %s/%d: %d\n",
			__func__, __LINE__, type);
		return LCM_STATUS_ERROR;
	}

	return LCM_STATUS_OK;
}


enum LCM_STATUS lcm_gpio_set_data(char type, const struct LCM_DATA_T1 *t1)
{
	/* check parameter is valid */
	if (_lcm_gpio_check_data(type, t1) == LCM_STATUS_OK) {
		switch (type) {
#ifdef CONFIG_MTK_LEGACY
		case LCM_GPIO_MODE:
			mt_set_gpio_mode(GPIO_65132_EN, (unsigned int)t1->data);
			break;

		case LCM_GPIO_DIR:
			mt_set_gpio_dir(GPIO_65132_EN, (unsigned int)t1->data);
			break;

		case LCM_GPIO_OUT:
			mt_set_gpio_out(GPIO_65132_EN, (unsigned int)t1->data);
			break;
#else
#ifdef CONFIG_PINCTRL
		case LCM_GPIO_MODE:
			pr_debug("[LCM][GPIO] %s/%d: set mode: %d\n",
				__func__, __LINE__, (unsigned int)t1->data);
			pinctrl_select_state(_lcm_gpio,
				_lcm_gpio_mode[(unsigned int)t1->data]);
			break;

		case LCM_GPIO_DIR:
			pr_debug("[LCM][GPIO] %s/%d: set dir: %d, %d\n",
				__func__, __LINE__, GPIO_LCD_PWR_EN,
				(unsigned int)t1->data);
			gpio_direction_output(GPIO_LCD_PWR_EN, (int)t1->data);
			break;

		case LCM_GPIO_OUT:
			pr_debug("[LCM][GPIO] %s/%d: set out: %d, %d\n",
				__func__, __LINE__, GPIO_LCD_PWR_EN,
				(unsigned int)t1->data);
			gpio_set_value(GPIO_LCD_PWR_EN, (int)t1->data);
			break;
#else
		case LCM_GPIO_MODE:
		case LCM_GPIO_DIR:
		case LCM_GPIO_OUT:
			break;
#endif
#endif
		default:
			pr_info("[LCM][ERROR] %s/%d: %d\n",
				__func__, __LINE__, type);
			return LCM_STATUS_ERROR;
		}
	} else {
		pr_info("[LCM][ERROR] %s: 0x%x, 0x%x\n",
			__func__, type, t1->data);
		return LCM_STATUS_ERROR;
	}

	return LCM_STATUS_OK;
}


#ifdef CONFIG_MTK_LEGACY
#else
module_init(_lcm_gpio_init);
module_exit(_lcm_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek LCM GPIO driver");
MODULE_AUTHOR("Joey Pan<joey.pan@mediatek.com>");
#endif

#else  //added by xen for LCD_ID control 20171106
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/module.h>

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>	/* hwPowerOn */
#include <mt-plat/upmu_common.h>
#else
#include <mt-plat/upmu_common.h>
#endif

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
//#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#else
//#include <mt-plat/mtk_gpio.h> //xjl 20180531
#endif

#include "lcm_define.h"
#include "lcm_drv.h"
//#include "lcm_gpio.h"

#define YK_LCM_5V_IC_SUPPORT
#define LCM_GPIO_DEVICE	"lcd_id_gpio"
static int lcm_id_high = 0xFF;
static int gpio_lcd_id;


int gpio_charge_en;
#if defined(YK_LCM_5V_IC_SUPPORT)
int gpio_lcm_enp;
int gpio_lcm_enn;
int gpio_tp_reset;
#endif

int get_lcd_id_state(void)
{
	int input_value = 0;
	int ret = 0;

	if(0xFF != lcm_id_high) 
	{
	    return lcm_id_high;
	}

	ret = gpio_request(gpio_lcd_id, "lcd_id");
	if (ret<0){
	   pr_debug("lcd-id gpio_request failed!\n");
	   return -ENODEV;
	}

	ret = gpio_direction_input(gpio_lcd_id);
	if (ret<0){
	   pr_debug("lcd-id gpio_direction_input failed!\n");
	   return -ENODEV;
	}

	input_value = gpio_get_value(gpio_lcd_id);
	gpio_free(gpio_lcd_id);

	lcm_id_high = input_value;  //save lcd_id state

        return input_value;
}



void set_gpio_charge_en(int enable)
{
	int input_value = 0;

	gpio_set_value(gpio_charge_en, enable);
	input_value = gpio_get_value(gpio_charge_en);
	pr_debug("get gpio_charge_en = %d\n", input_value);
}
EXPORT_SYMBOL(set_gpio_charge_en);

#if defined(YK_LCM_5V_IC_SUPPORT)
int set_gpio_lcm_enp(int enable)
{
	int input_value = 0;
	
	if(enable)
		enable=1;
	
	gpio_set_value(gpio_lcm_enp, enable);
	input_value = gpio_get_value(gpio_lcm_enp);
	pr_debug("get gpio_lcm_enp = %d\n", input_value);
	return 0;
}
EXPORT_SYMBOL(set_gpio_lcm_enp);

int set_gpio_lcm_enn(int enable)
{
	int input_value = 0;
	
	if(enable)
		enable=1;
	gpio_set_value(gpio_lcm_enn, enable);
	input_value = gpio_get_value(gpio_lcm_enn);
	pr_debug("get gpio_lcm_enn = %d\n", input_value);
	return 0;
}
EXPORT_SYMBOL(set_gpio_lcm_enn);

void set_gpio_tp_reset(int enable)
{
	int input_value = 0;

	gpio_set_value(gpio_tp_reset, enable);
	input_value = gpio_get_value(gpio_tp_reset);
	pr_debug("get gpio_tp_reset = %d\n", input_value);
}
EXPORT_SYMBOL(set_gpio_tp_reset);

int set_gpio_lcm_enp_enn(int enable)
{
	int input_value = 0;
	
	if(enable)
		enable=1;
	
	gpio_set_value(gpio_lcm_enp, enable);
	input_value = gpio_get_value(gpio_lcm_enp);
	pr_debug("get gpio_lcm_enp = %d\n", input_value);
	gpio_set_value(gpio_lcm_enn, enable);
	input_value = gpio_get_value(gpio_lcm_enn);
	pr_debug("get gpio_lcm_enn = %d\n", input_value);
	return 0;
}
EXPORT_SYMBOL(set_gpio_lcm_enp_enn);

#include <linux/delay.h>
int set_gpio_lcm_enn_enp(int enable)
{
	int input_value = 0;
	
	if(enable)
		enable=1;
	gpio_set_value(gpio_lcm_enn, enable);
	input_value = gpio_get_value(gpio_lcm_enn);
	pr_debug("get gpio_lcm_enn = %d\n", input_value);
	mdelay(2);	
	gpio_set_value(gpio_lcm_enp, enable);
	input_value = gpio_get_value(gpio_lcm_enp);
	pr_debug("get gpio_lcm_enp = %d\n", input_value);

	return 0;
}
EXPORT_SYMBOL(set_gpio_lcm_enn_enp);
#endif

static int _lcm_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_debug("[LCM][GPIO] enter %s, %d\n", __func__, __LINE__);
	gpio_lcd_id = of_get_named_gpio(pdev->dev.of_node, "gpio_lcd_id", 0);
	gpio_charge_en = of_get_named_gpio(pdev->dev.of_node, "gpio_charge_en", 0);
	pr_debug("[LCM][GPIO] _lcm_gpio_get_info end!gpio_lcd_id=%d\n", gpio_lcd_id);
	pr_debug("[LCM][GPIO] _lcm_gpio_get_info end!gpio_charge_en=%d\n", gpio_charge_en);
#if defined(YK_LCM_5V_IC_SUPPORT)
	gpio_lcm_enp = of_get_named_gpio(pdev->dev.of_node, "gpio_lcm_enp", 0);
	gpio_lcm_enn = of_get_named_gpio(pdev->dev.of_node, "gpio_lcm_enn", 0);
	gpio_tp_reset = of_get_named_gpio(pdev->dev.of_node, "gpio_tp_reset", 0);
	ret = gpio_request(gpio_lcm_enp, "gpio_lcm_enp");
	if (ret<0){
	   pr_debug("gpio_lcm_enp gpio_request failed!\n");
	   return -ENODEV;
	}
	gpio_direction_output(gpio_lcm_enp, 1);
	gpio_set_value(gpio_lcm_enp, 1);

	ret = gpio_request(gpio_lcm_enn, "gpio_lcm_enn");
	if (ret<0){
	   pr_debug("gpio_lcm_enn gpio_request failed!\n");
	   return -ENODEV;
	}
	gpio_direction_output(gpio_lcm_enn, 1);
	gpio_set_value(gpio_lcm_enn, 1);
	
	ret = gpio_request(gpio_tp_reset, "gpio_tp_reset");
	if (ret<0){
	   pr_debug("gpio_tp_reset gpio_request failed!\n");
	   return -ENODEV;
	}
	gpio_direction_output(gpio_tp_reset, 1);
	gpio_set_value(gpio_tp_reset, 0);	
#endif

//gpio_charge_en
	ret = gpio_request(gpio_charge_en, "gpio_charge_en");
	if (ret<0){
	   pr_debug("gpio_charge_en gpio_request failed!\n");
	   return -ENODEV;
	}
	gpio_direction_output(gpio_charge_en, 1);
	gpio_set_value(gpio_charge_en, 0);
    
	return 0;
}

static int _lcm_gpio_remove(struct platform_device *pdev)
{

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id _lcm_gpio_of_ids[] = {
	{.compatible = "mediatek,lcd_id_gpio",},
	{},
};
MODULE_DEVICE_TABLE(of, _lcm_gpio_of_ids);
#endif

static struct platform_driver _lcm_gpio_driver = {
	.driver = {
		.name = LCM_GPIO_DEVICE,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(_lcm_gpio_of_ids),
	},
	.probe = _lcm_gpio_probe,
	.remove = _lcm_gpio_remove,
};
module_platform_driver(_lcm_gpio_driver);

static int __init _lcm_gpio_init(void)
{
	pr_debug("MediaTek LCM GPIO driver init\n");
	if (platform_driver_register(&_lcm_gpio_driver) != 0) {
		pr_err("unable to register LCM GPIO driver.\n");
		return -1;
	}
	return 0;
}


/* should never be called */
static void __exit _lcm_gpio_exit(void)
{
	pr_debug("MediaTek LCM GPIO driver exit\n");
	platform_driver_unregister(&_lcm_gpio_driver);
}

module_init(_lcm_gpio_init);
module_exit(_lcm_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek LCM GPIO driver");
MODULE_AUTHOR("Joey Pan<joey.pan@mediatek.com>");

#else  //#ifndef BUILD_LK
#include "lcm_drv.h"
#include <platform/mt_gpio.h>
//#include "cust_gpio_usage.h"

#define YK_LCM_5V_IC_SUPPORT
#if 1//defined(YK185_CONFIG)
#define GPIO_LCM_ID_PIN    (3|0x80000000)
#else
#define GPIO_LCM_ID_PIN    (9|0x80000000)  //for yk736
#endif

#if defined(YK_LCM_5V_IC_SUPPORT)
#define GPIO_LCM_ENP_PIN    (150|0x80000000)
#define GPIO_LCM_ENN_PIN    (153|0x80000000)
#define GPIO_TP_RESET_PIN   (174|0x80000000)
#endif

static int lcm_id_high = 0xFF;
int get_lcd_id_state(void)
{
    unsigned char lcd_id = 0;
    //Solve Coverity scan warning : check return value
    unsigned int ret = 0;
    //only recognise once
    if(0xFF != lcm_id_high) 
    {
        return lcm_id_high;
    }
    //Solve Coverity scan warning : check return value
    ret = mt_set_gpio_mode(GPIO_LCM_ID_PIN, GPIO_MODE_GPIO);

    ret = mt_set_gpio_dir(GPIO_LCM_ID_PIN, GPIO_DIR_IN);
   
    lcd_id = mt_get_gpio_in(GPIO_LCM_ID_PIN);

    //if (lcd_id == 1)
    //{
    //	ret = mt_set_gpio_pull_select(GPIO_LCM_ID_PIN, GPIO_PULL_UP); 
    //}
    //else if (lcd_id == 0)
    //{
    //	ret = mt_set_gpio_pull_select(GPIO_LCM_ID_PIN, GPIO_PULL_DOWN);
    //}
    //else
    //{
    //	ret = mt_set_gpio_pull_select(GPIO_LCM_ID_PIN, GPIO_PULL_DISABLE);
    //}

    lcm_id_high = lcd_id;
    return lcd_id;
}

#if defined(YK_LCM_5V_IC_SUPPORT)
int set_gpio_lcm_enp(int enable)
{	
	if(enable)
		enable=1;
	 LCM_LOGI("KERNEL-LCM: %s : enable= %d\n", __func__,enable);
	
    mt_set_gpio_mode(GPIO_LCM_ENP_PIN, GPIO_MODE_GPIO);
    mt_set_gpio_dir(GPIO_LCM_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_ENP_PIN, enable);
	return 0;
}
EXPORT_SYMBOL(set_gpio_lcm_enp);

int set_gpio_lcm_enn(int enable)
{	
	if(enable)
		enable=1;
	 LCM_LOGI("KERNEL-LCM: %s : enable= %d\n", __func__,enable);
    mt_set_gpio_mode(GPIO_LCM_ENN_PIN, GPIO_MODE_GPIO);
    mt_set_gpio_dir(GPIO_LCM_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_ENN_PIN, enable);
	return 0;
}
EXPORT_SYMBOL(set_gpio_lcm_enn);


void set_gpio_tp_reset(int enable)
{
	LCM_LOGI("KERNEL-LCM: %s : enable= %d\n", __func__,enable);
    mt_set_gpio_mode(GPIO_TP_RESET_PIN, GPIO_MODE_GPIO);
    mt_set_gpio_dir(GPIO_TP_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TP_RESET_PIN, enable);
}
EXPORT_SYMBOL(set_gpio_tp_reset);
#endif

#endif
#endif

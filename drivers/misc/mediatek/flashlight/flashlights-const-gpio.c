/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef CONSTANT_DTNAME
#define CONSTANT_DTNAME "mediatek,flashlights_constant_gpio"
#endif

/* TODO: define driver name */
#define CONSTANT_NAME "flashlights-constant-gpio"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(constant_mutex);
static struct work_struct constant_work;

/* define pinctrl */
/* TODO: define pinctrl */
//#define CONSTANT_PINCTRL_PIN_XXX 0
//#define CONSTANT_PINCTRL_PINSTATE_LOW 0
//#define CONSTANT_PINCTRL_PINSTATE_HIGH 1
//#define CONSTANT_PINCTRL_STATE_XXX_HIGH "xxx_high"
//#define CONSTANT_PINCTRL_STATE_XXX_LOW  "xxx_low"
//static struct pinctrl *constant_pinctrl;
//static struct pinctrl_state *constant_xxx_high;
//static struct pinctrl_state *constant_xxx_low;
#define CONSTANT_PINCTRL_PIN_ENABLE 0
#define CONSTANT_PINCTRL_PIN_STROBE 1
#define CONSTANT_PINCTRL_PIN_SUB_ENABLE 2 //xen 20171028 for sub flashlight
#define CONSTANT_PINCTRL_PINSTATE_LOW 0
#define CONSTANT_PINCTRL_PINSTATE_HIGH 1
#define CONSTANT_PINCTRL_STATE_ENABLE_HIGH "main_enable_pin_high"
#define CONSTANT_PINCTRL_STATE_ENABLE_LOW  "main_enable_pin_low"
#define CONSTANT_PINCTRL_STATE_STROBE_HIGH "main_strobe_pin_high"
#define CONSTANT_PINCTRL_STATE_STROBE_LOW  "main_strobe_pin_low"
#define CONSTANT_PINCTRL_STATE_SUB_ENABLE_HIGH "sub_enable_pin_high" //xen 20171028 for sub flashlight
#define CONSTANT_PINCTRL_STATE_SUB_ENABLE_LOW  "sub_enable_pin_low"

static struct pinctrl *constant_pinctrl;
static struct pinctrl_state *constant_enable_high;
static struct pinctrl_state *constant_enable_low;
static struct pinctrl_state *constant_strobe_high;
static struct pinctrl_state *constant_strobe_low;
static struct pinctrl_state *constant_sub_enable_high; //xen 20171028 for sub flashlight
static struct pinctrl_state *constant_sub_enable_low;

/* define usage count */
static int use_count;
static int g_duty = -1;

/* platform data */
struct constant_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

extern int isink_flashlight_en(int enable);

#define GPIO_OUT_ONE   1
#define GPIO_OUT_ZERO  0
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int constant_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

        printk("constant-flashlight constant_pinctrl_init\n");
	/* get pinctrl */
	constant_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(constant_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(constant_pinctrl);
	}

	/* TODO: Flashlight XXX pin initialization */
	constant_enable_high = pinctrl_lookup_state(constant_pinctrl, CONSTANT_PINCTRL_STATE_ENABLE_HIGH);
	if (IS_ERR(constant_enable_high)) {
		pr_err("Failed to init (%s)\n", CONSTANT_PINCTRL_STATE_ENABLE_HIGH);
		ret = PTR_ERR(constant_enable_high);
	}
	constant_enable_low = pinctrl_lookup_state(constant_pinctrl, CONSTANT_PINCTRL_STATE_ENABLE_LOW);
	if (IS_ERR(constant_enable_low)) {
		pr_err("Failed to init (%s)\n", CONSTANT_PINCTRL_STATE_ENABLE_LOW);
		ret = PTR_ERR(constant_enable_low);
	}

	constant_strobe_high = pinctrl_lookup_state(constant_pinctrl, CONSTANT_PINCTRL_STATE_STROBE_HIGH);
	if (IS_ERR(constant_strobe_high)) {
		pr_err("Failed to init (%s)\n", CONSTANT_PINCTRL_STATE_STROBE_HIGH);
		ret = PTR_ERR(constant_strobe_high);
	}
	constant_strobe_low = pinctrl_lookup_state(constant_pinctrl, CONSTANT_PINCTRL_STATE_STROBE_LOW);
	if (IS_ERR(constant_strobe_low)) {
		pr_err("Failed to init (%s)\n", CONSTANT_PINCTRL_STATE_STROBE_LOW);
		ret = PTR_ERR(constant_strobe_low);
	}

#ifdef SUB_FLASHLIGHT_SUPPORT
#if !defined(YK673_CONFIG)
//added by xen for sub flashlight 20171028
	constant_sub_enable_high = pinctrl_lookup_state(constant_pinctrl, CONSTANT_PINCTRL_STATE_SUB_ENABLE_HIGH);
	if (IS_ERR(constant_sub_enable_high)) {
		pr_err("Failed to init (%s)\n", CONSTANT_PINCTRL_STATE_SUB_ENABLE_HIGH);
		ret = PTR_ERR(constant_sub_enable_high);
	}
	constant_sub_enable_low = pinctrl_lookup_state(constant_pinctrl, CONSTANT_PINCTRL_STATE_SUB_ENABLE_LOW);
	if (IS_ERR(constant_sub_enable_low)) {
		pr_err("Failed to init (%s)\n", CONSTANT_PINCTRL_STATE_SUB_ENABLE_LOW);
		ret = PTR_ERR(constant_sub_enable_low);
	}

#endif
#endif
	return ret;
}

static int constant_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(constant_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case CONSTANT_PINCTRL_PIN_ENABLE:
		if (state == CONSTANT_PINCTRL_PINSTATE_LOW && !IS_ERR(constant_enable_low))
			pinctrl_select_state(constant_pinctrl, constant_enable_low);
		else if (state == CONSTANT_PINCTRL_PINSTATE_HIGH && !IS_ERR(constant_enable_high))
			pinctrl_select_state(constant_pinctrl, constant_enable_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case CONSTANT_PINCTRL_PIN_STROBE:
		if (state == CONSTANT_PINCTRL_PINSTATE_LOW && !IS_ERR(constant_strobe_low))
			pinctrl_select_state(constant_pinctrl, constant_strobe_low);
		else if (state == CONSTANT_PINCTRL_PINSTATE_HIGH && !IS_ERR(constant_strobe_high))
			pinctrl_select_state(constant_pinctrl, constant_strobe_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	#ifdef SUB_FLASHLIGHT_SUPPORT
	#if !defined(YK673_CONFIG)
	case CONSTANT_PINCTRL_PIN_SUB_ENABLE: //added by xen for sub flashlight 20171028
		if (state == CONSTANT_PINCTRL_PINSTATE_LOW && !IS_ERR(constant_sub_enable_low))
			pinctrl_select_state(constant_pinctrl, constant_sub_enable_low);
		else if (state == CONSTANT_PINCTRL_PINSTATE_HIGH && !IS_ERR(constant_sub_enable_high))
			pinctrl_select_state(constant_pinctrl, constant_sub_enable_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	#endif
	#endif
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * constant operations
 *****************************************************************************/
/* flashlight enable function */
static int constant_enable(int channel) //void) //channel-0 means main flashlight,channel-1 means sub flashlight
{
	//int pin = 0, state = 0;

	/* TODO: wrap enable function */
        if (channel==0) { //main flashlight
        	if (g_duty!=0)
        	{
		        constant_pinctrl_set(CONSTANT_PINCTRL_PIN_STROBE, CONSTANT_PINCTRL_PINSTATE_HIGH);
		        constant_pinctrl_set(CONSTANT_PINCTRL_PIN_ENABLE, CONSTANT_PINCTRL_PINSTATE_HIGH);
        	}
        	else
        	{
		        constant_pinctrl_set(CONSTANT_PINCTRL_PIN_ENABLE, CONSTANT_PINCTRL_PINSTATE_HIGH);
		        constant_pinctrl_set(CONSTANT_PINCTRL_PIN_STROBE, CONSTANT_PINCTRL_PINSTATE_LOW);
        	}

	//ISINK01
	#if defined(YK736_CONFIG)||defined(YK739_CONFIG)||defined(YK676V2_CONFIG)
		isink_flashlight_en(1);
	#endif
	}
	else {
		if (!IS_ERR(constant_sub_enable_high))
        	constant_pinctrl_set(CONSTANT_PINCTRL_PIN_SUB_ENABLE, CONSTANT_PINCTRL_PINSTATE_HIGH);

	#if defined(YK673_CONFIG)
		isink_flashlight_en(1);
	#endif
	}

	pr_debug("Flashlight: constant_Enable channel=%d,line=%d\n", channel, __LINE__);
	printk("Flashlight: constant_Enable channel=%d,line=%d\n", channel, __LINE__);

	return 0;

	//return constant_pinctrl_set(pin, state);
}

/* flashlight disable function */
static int constant_disable(int channel) //void)//channel-0 means main flashlight,channel-1 means sub flashlight
{
	//int pin = 0, state = 0;

	/* TODO: wrap disable function */
        if (channel==0) { //main flashlight
		constant_pinctrl_set(CONSTANT_PINCTRL_PIN_ENABLE, CONSTANT_PINCTRL_PINSTATE_LOW);
		constant_pinctrl_set(CONSTANT_PINCTRL_PIN_STROBE, CONSTANT_PINCTRL_PINSTATE_LOW);

	//ISINK01
	#if defined(YK736_CONFIG)||defined(YK739_CONFIG)||defined(YK676V2_CONFIG)
		isink_flashlight_en(0);
	#endif
	}
	else {
		if (!IS_ERR(constant_sub_enable_low))
		constant_pinctrl_set(CONSTANT_PINCTRL_PIN_SUB_ENABLE, CONSTANT_PINCTRL_PINSTATE_LOW);

	#if defined(YK673_CONFIG)
		isink_flashlight_en(0);
	#endif
	}

	pr_debug("Flashlight constant_Disable channel=%d,line=%d\n", channel, __LINE__);
	printk("Flashlight constant_Disable channel=%d,line=%d\n", channel, __LINE__);
	return 0;
	//return constant_pinctrl_set(pin, state);
}

/* set flashlight level */
static int constant_set_level(int level)
{
	//int pin = 0, state = 0;

	/* TODO: wrap set level function */
	pr_debug("Flashlight constant_set_level=%d\n", level);
	g_duty = level;
	return 0;
	//return constant_pinctrl_set(pin, state);
}

/* flashlight init */
static int constant_init(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap init function */
	constant_pinctrl_set(CONSTANT_PINCTRL_PIN_ENABLE, CONSTANT_PINCTRL_PINSTATE_LOW);
	constant_pinctrl_set(CONSTANT_PINCTRL_PIN_STROBE, CONSTANT_PINCTRL_PINSTATE_LOW);

	constant_pinctrl_set(CONSTANT_PINCTRL_PIN_SUB_ENABLE, CONSTANT_PINCTRL_PINSTATE_LOW);  //sub flashlight

	pr_debug("Flashlight constant_init!!\n");

	return 0;
	//return constant_pinctrl_set(pin, state);
}

/* flashlight uninit */
static int constant_uninit(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap uninit function */
	constant_disable(0);
	constant_disable(1);
	return 0;
	//return constant_disable();
	//return constant_pinctrl_set(pin, state);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer constant_timer;
static unsigned int constant_timeout_ms;

static void constant_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	constant_disable(0);
	constant_disable(1);
}

static enum hrtimer_restart constant_timer_func(struct hrtimer *timer)
{
	schedule_work(&constant_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int constant_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		constant_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		constant_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (constant_timeout_ms) {
				s = constant_timeout_ms / 1000;
				ns = constant_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&constant_timer, ktime,
						HRTIMER_MODE_REL);
			}
			constant_enable(channel); //main or sub flashlight
		} else {
			constant_disable(channel); //main or sub flashlight
			hrtimer_cancel(&constant_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int constant_open(void)
{
	/* Move to set driver for saving power */
	pr_debug("flashlights-constant-gpio constant_open\n");
	return 0;
}

static int constant_release(void)
{
	/* Move to set driver for saving power */
	pr_debug("Release: %d\n", use_count);
	return 0;
}

static int constant_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	pr_debug("constant_set_driver: %d\n", set);
	mutex_lock(&constant_mutex);
	if (set) {
		if (!use_count)
			ret = constant_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = constant_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&constant_mutex);

	return ret;
}

static ssize_t constant_strobe_store(struct flashlight_arg arg)
{
	constant_set_driver(1);
	constant_set_level(arg.level);
	constant_timeout_ms = 0;
	constant_enable(0); //
	constant_enable(1); //
	msleep(arg.dur);
	constant_disable(0);
	constant_disable(1); //
	constant_set_driver(0);

	return 0;
}

static struct flashlight_operations constant_ops = {
	constant_open,
	constant_release,
	constant_ioctl,
	constant_strobe_store,
	constant_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int constant_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * constant_init();
	 */

	return 0;
}

static int constant_parse_dt(struct device *dev,
		struct constant_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				CONSTANT_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int constant_probe(struct platform_device *pdev)
{
	struct constant_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");
	printk("Flashlight:constant_probe start!\n");

	/* init pinctrl */
	if (constant_pinctrl_init(pdev)) {
		pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	printk("Flashlight:constant_probe start-1!\n");
	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = constant_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}
	printk("Flashlight:constant_probe start-2!\n");
	/* init work queue */
	INIT_WORK(&constant_work, constant_work_disable);

	/* init timer */
	hrtimer_init(&constant_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	constant_timer.function = constant_timer_func;
	constant_timeout_ms = 100;

	/* init chip hw */
	constant_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	printk("Flashlight:constant_probe start-3! channal_num=%d\n",pdata->channel_num);
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&constant_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(CONSTANT_NAME, &constant_ops)) {
			printk("flashlight_dev_register error!\n");
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int constant_remove(struct platform_device *pdev)
{
	struct constant_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(CONSTANT_NAME);

	/* flush work queue */
	flush_work(&constant_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id constant_gpio_of_match[] = {
	{.compatible = CONSTANT_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, constant_gpio_of_match);
#else
static struct platform_device constant_gpio_platform_device[] = {
	{
		.name = CONSTANT_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, constant_gpio_platform_device);
#endif

static struct platform_driver constant_platform_driver = {
	.probe = constant_probe,
	.remove = constant_remove,
	.driver = {
		.name = CONSTANT_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = constant_gpio_of_match,
#endif
	},
};

static int __init flashlight_constant_init(void)
{
	int ret;

	pr_debug("Init start.\n");
	printk("flashlight_constant_init Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&constant_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&constant_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");
	printk("flashlight_constant_init Init done.\n");
	return 0;
}

static void __exit flashlight_constant_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&constant_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_constant_init);
module_exit(flashlight_constant_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight CONSTANT GPIO Driver");


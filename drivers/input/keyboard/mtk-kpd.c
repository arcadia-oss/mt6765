// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 * Author Terry Chang <terry.chang@mediatek.com>
 */
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#define KPD_NAME	"mtk-kpd"

#define KP_STA			(0x0000)
#define KP_MEM1			(0x0004)
#define KP_MEM2			(0x0008)
#define KP_MEM3			(0x000c)
#define KP_MEM4			(0x0010)
#define KP_MEM5			(0x0014)
#define KP_DEBOUNCE		(0x0018)
#define KP_SEL			(0x0020)
#define KP_EN			(0x0024)

#define KP_COL0_SEL		(1 << 10)
#define KP_COL1_SEL		(1 << 11)
#define KP_COL2_SEL		(1 << 12)

#define KPD_DEBOUNCE_MASK	((1U << 14) - 1)
#define KPD_DOUBLE_KEY_MASK	(1U << 0)

#define KPD_NUM_MEMS	5
#define KPD_MEM5_BITS	8
#define KPD_NUM_KEYS	72	/* 4 * 16 + KPD_MEM5_BITS */

struct mtk_keypad {
	struct input_dev *input_dev;
	struct wakeup_source *suspend_lock;
	struct tasklet_struct tasklet;
	struct clk *clk;
	void __iomem *base;
	unsigned int irqnr;
	u32 key_debounce;
	u32 hw_map_num;
	u32 hw_init_map[KPD_NUM_KEYS];
	u16 keymap_state[KPD_NUM_MEMS];
};

/* for keymap handling */
static void kpd_keymap_handler(unsigned long data);

static int kpd_pdrv_probe(struct platform_device *pdev);
static struct platform_driver kpd_pdrv;

static void kpd_get_keymap_state(void __iomem *kp_base, u16 state[])
{
	state[0] = readw(kp_base + KP_MEM1);
	state[1] = readw(kp_base + KP_MEM2);
	state[2] = readw(kp_base + KP_MEM3);
	state[3] = readw(kp_base + KP_MEM4);
	state[4] = readw(kp_base + KP_MEM5);
	pr_debug("kpd register = %x %x %x %x %x\n",
		state[0], state[1], state[2], state[3], state[4]);
}

//add by ybx
extern char mtk_main_cam_name[50];
static ssize_t cam_main_info_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%s\n", mtk_main_cam_name);
	return res;
}

static DRIVER_ATTR(cam_main_info, 0644, cam_main_info_show,NULL);

extern char mtk_main2_cam_name[50];
static ssize_t cam_main2_info_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%s\n", mtk_main2_cam_name);
	return res;
}

static DRIVER_ATTR(cam_main2_info, 0644, cam_main2_info_show,NULL);

extern char mtk_sub_cam_name[50];
static ssize_t cam_sub_info_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%s\n", mtk_sub_cam_name);
	return res;
}

static DRIVER_ATTR(cam_sub_info, 0644, cam_sub_info_show,NULL);

extern char mtk_tp_info[128];
static ssize_t tp_info_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%s\n", mtk_tp_info);
	return res;
}

static DRIVER_ATTR(tp_info, 0644, tp_info_show,NULL);

extern char mtk_tp_version[128];
static ssize_t tp_version_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%s\n", mtk_tp_version);
	return res;
}

static DRIVER_ATTR(tp_version, 0644, tp_version_show,NULL);

extern char mtk_flash_cid[128];
static ssize_t flash_info_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%s\n", mtk_flash_cid);
	return res;
}

static DRIVER_ATTR(flash_info, 0644, flash_info_show,NULL);

#if defined(TOUCHPANEL_GESTURE) //xen 20170831 by tp gesture controller by MMI
struct input_dev *kpd_input_dev;
u8 gesture_open_state = 0;
static ssize_t kpd_store_tp_gesture_state(struct device_driver *ddri, const char *buf, size_t count)
{
	if(strncmp("yes",buf,3)==0){
		gesture_open_state = 1;
		pr_info("[TP_GESTURE] tp_sysfs_tpgesturet_store on.\n");
	}
	else if(strncmp("no",buf,2)==0){
		gesture_open_state = 0;
		pr_info("[TP_GESTURE] tp_sysfs_tpgesturet_store off.\n");
	}

	return count;
}

static ssize_t kpd_show_tp_gesture_state(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", gesture_open_state);
	return res;
}

static DRIVER_ATTR(kpd_tp_gesture_state, S_IWUSR | S_IRUGO, kpd_show_tp_gesture_state, kpd_store_tp_gesture_state);
#endif

extern unsigned int yk_stop_percent;
static ssize_t kpd_store_stop_charging_percent(struct device_driver *ddri,
               const char *buf, size_t count)
{
	int ret;
	ret = kstrtouint(buf, 0, &yk_stop_percent);
	if (ret) {
		pr_info("kpd yk_stop_percent: Invalid values\n");
		return -EINVAL;
	}
	
	return count;
}

static ssize_t kpd_show_stop_charging_percent(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	res = snprintf(buf, PAGE_SIZE, "%d\n", yk_stop_percent);
	return res;
}

static DRIVER_ATTR(stop_charging_percent, 0644, kpd_show_stop_charging_percent,
               kpd_store_stop_charging_percent);

static struct driver_attribute *kpd_attr_list[] = {
	&driver_attr_tp_info,
	&driver_attr_tp_version,
	&driver_attr_cam_main_info,
	&driver_attr_cam_main2_info,
	&driver_attr_cam_sub_info,
	&driver_attr_flash_info,
#if defined(TOUCHPANEL_GESTURE) //xen 20170831 by tp gesture controller by MMI
	&driver_attr_kpd_tp_gesture_state,
#endif
	&driver_attr_stop_charging_percent,
};

static int kpd_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = ARRAY_SIZE(kpd_attr_list);

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, kpd_attr_list[idx]);
		if (err) {
			pr_info("driver_create_file (%s) = %d\n",
				kpd_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int kpd_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = ARRAY_SIZE(kpd_attr_list);

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, kpd_attr_list[idx]);

	return err;
}
//add end ybx

#if defined (TOUCHPANEL_GESTURE)  // xjl 2014-01-16
void kpd_touchpanel_gesture_handler(int key_code)
{
	//for vibrate soon
	//upmu_set_rg_vibr_en(1); //xjl 20140526
	//mdelay(50);
	//upmu_set_rg_vibr_en(0);

	input_report_key(kpd_input_dev, key_code, 1);
	input_sync(kpd_input_dev);
	input_report_key(kpd_input_dev, key_code, 0);
	input_sync(kpd_input_dev);
}
#endif

static void kpd_keymap_handler(unsigned long data)
{
	int i, j;
	int pressed;
	u16 new_state[KPD_NUM_MEMS], change, mask;
	u16 hw_keycode, keycode;
	void *dest;
	struct mtk_keypad *keypad = (struct mtk_keypad *)data;

	kpd_get_keymap_state(keypad->base, new_state);

	__pm_wakeup_event(keypad->suspend_lock, 500);

	for (i = 0; i < KPD_NUM_MEMS; i++) {
		change = new_state[i] ^ keypad->keymap_state[i];
		if (!change)
			continue;

		for (j = 0; j < 16U; j++) {
			mask = (u16) 1 << j;
			if (!(change & mask))
				continue;

			hw_keycode = (i << 4) + j;

			if (hw_keycode >= KPD_NUM_KEYS)
				continue;

			/* bit is 1: not pressed, 0: pressed */
			pressed = (new_state[i] & mask) == 0U;
			pr_debug("(%s) HW keycode = %d\n",
				(pressed) ? "pressed" : "released",
					hw_keycode);

			keycode = keypad->hw_init_map[hw_keycode];
			if (!keycode)
				continue;
			input_report_key(keypad->input_dev, keycode, pressed);
			input_sync(keypad->input_dev);
			pr_debug("report Linux keycode = %d\n", keycode);
		}
	}

	dest = memcpy(keypad->keymap_state, new_state, sizeof(new_state));
	enable_irq(keypad->irqnr);
}

static irqreturn_t kpd_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
	struct mtk_keypad *keypad = dev_id;

	disable_irq_nosync(keypad->irqnr);
	tasklet_schedule(&keypad->tasklet);
	return IRQ_HANDLED;
}

static int kpd_get_dts_info(struct mtk_keypad *keypad,
				struct device_node *node)
{
	int ret;

	ret = of_property_read_u32(node, "mediatek,kpd-key-debounce",
		&keypad->key_debounce);
	if (ret) {
		pr_debug("read mediatek,key-debounce-ms error.\n");
		return ret;
	}

	ret = of_property_read_u32(node, "mediatek,kpd-hw-map-num",
		&keypad->hw_map_num);
	if (ret) {
		pr_debug("read mediatek,hw-map-num error.\n");
		return ret;
	}

	if (keypad->hw_map_num > KPD_NUM_KEYS) {
		pr_debug("hw-map-num error, it cannot bigger than %d.\n",
			KPD_NUM_KEYS);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(node, "mediatek,kpd-hw-init-map",
		keypad->hw_init_map, keypad->hw_map_num);

	if (ret) {
		pr_debug("hw-init-map was not defined in dts.\n");
		return ret;
	}

	pr_debug("deb= %d\n", keypad->key_debounce);

	return 0;
}

static int kpd_gpio_init(struct device *dev)
{
	struct pinctrl *keypad_pinctrl;
	struct pinctrl_state *kpd_default;

	keypad_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(keypad_pinctrl)) {
		pr_debug("Cannot find keypad_pinctrl!\n");

		return (int)PTR_ERR(keypad_pinctrl);
	}

	kpd_default = pinctrl_lookup_state(keypad_pinctrl, "default");
	if (IS_ERR(kpd_default)) {
		pr_debug("Cannot find ecall_state!\n");

		return (int)PTR_ERR(kpd_default);
	}

	return pinctrl_select_state(keypad_pinctrl,
				kpd_default);
}

static int kpd_pdrv_probe(struct platform_device *pdev)
{
	struct mtk_keypad *keypad;
	struct resource *res;
	int i;
	int err;

	keypad = devm_kzalloc(&pdev->dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	keypad->clk = devm_clk_get(&pdev->dev, "kpd");
	if (IS_ERR(keypad->clk)) {
		pr_debug("get kpd-clk fail: %d\n", (int)PTR_ERR(keypad->clk));
		return (int)PTR_ERR(keypad->clk);
	}

	err = clk_prepare_enable(keypad->clk);
	if (err) {
		pr_debug("kpd-clk prepare enable failed.\n");
		return err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENODEV;
		goto err_unprepare_clk;
	}

	keypad->base = devm_ioremap(&pdev->dev, res->start,
			resource_size(res));
	if (!keypad->base) {
		pr_debug("KP iomap failed\n");
		err = -EBUSY;
		goto err_unprepare_clk;
	}

	keypad->irqnr = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!keypad->irqnr) {
		pr_debug("KP get irqnr failed\n");
		err = -ENODEV;
		goto err_unprepare_clk;
	}

	pr_info("kp base: 0x%p, addr:0x%p,  kp irq: %d\n",
			keypad->base, &keypad->base, keypad->irqnr);
	err = kpd_gpio_init(&pdev->dev);
	if (err) {
		pr_debug("gpio init failed\n");
		goto err_unprepare_clk;
	}

	err = kpd_get_dts_info(keypad, pdev->dev.of_node);
	if (err) {
		pr_debug("get dts info failed.\n");
		goto err_unprepare_clk;
	}

	memset(keypad->keymap_state, 0xff, sizeof(keypad->keymap_state));

	keypad->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!keypad->input_dev) {
		pr_notice("input allocate device fail.\n");
		err = -ENOMEM;
		goto err_unprepare_clk;
	}

	keypad->input_dev->name = KPD_NAME;
	keypad->input_dev->id.bustype = BUS_HOST;
	keypad->input_dev->dev.parent = &pdev->dev;

	__set_bit(EV_KEY, keypad->input_dev->evbit);

	for (i = 0; i < KPD_NUM_KEYS; i++) {
		if (keypad->hw_init_map[i])
			__set_bit(keypad->hw_init_map[i],
				keypad->input_dev->keybit);
	}

#if defined (TOUCHPANEL_GESTURE)  // xjl 2014-01-16
        kpd_input_dev = keypad->input_dev;
	__set_bit(KEY_TPGESTURE_UP, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_DOWN, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_LEFT, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_RIGHT, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_DOUBLE, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_C, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_E, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_M, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_O, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_S, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_V, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_W, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_Z, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_ARROWUP, keypad->input_dev->keybit);
	__set_bit(KEY_TPGESTURE_ARROWRIGHT, keypad->input_dev->keybit);
        __set_bit(KEY_F4, keypad->input_dev->keybit);//tplink
#endif

	err = input_register_device(keypad->input_dev);
	if (err) {
		pr_notice("register input device failed (%d)\n", err);
		goto err_unprepare_clk;
	}

	input_set_drvdata(keypad->input_dev, keypad);

	keypad->suspend_lock = wakeup_source_register(NULL, "kpd wakelock");
	if (!keypad->suspend_lock) {
		pr_notice("wakeup source init failed.\n");
		goto err_unregister_device;
	}

	tasklet_init(&keypad->tasklet, kpd_keymap_handler,
					(unsigned long)keypad);

	writew((u16)(keypad->key_debounce & KPD_DEBOUNCE_MASK),
			keypad->base + KP_DEBOUNCE);

	/* register IRQ */
	err = request_irq(keypad->irqnr, kpd_irq_handler, IRQF_TRIGGER_NONE,
			KPD_NAME, keypad);
	if (err) {
		pr_notice("register IRQ failed (%d)\n", err);
		goto err_irq;
	}

	pr_info("kpd_probe OK.\n");
	
	err = kpd_create_attr(&kpd_pdrv.driver);
	if (err) {
		pr_notice("create attr file fail\n");
		kpd_delete_attr(&kpd_pdrv.driver);
		return err;
	}

	return 0;

err_irq:
	tasklet_kill(&keypad->tasklet);

err_unregister_device:
	input_unregister_device(keypad->input_dev);

err_unprepare_clk:
	clk_disable_unprepare(keypad->clk);

	return err;
}

static int kpd_pdrv_remove(struct platform_device *pdev)
{
	struct mtk_keypad *keypad = platform_get_drvdata(pdev);

	tasklet_kill(&keypad->tasklet);
	wakeup_source_unregister(keypad->suspend_lock);
	input_unregister_device(keypad->input_dev);
	clk_disable_unprepare(keypad->clk);

	return 0;
}

static const struct of_device_id kpd_of_match[] = {
	{.compatible = "mediatek,mt6779-keypad"},
	{.compatible = "mediatek,kp"},
	{},
};

static struct platform_driver kpd_pdrv = {
	.probe = kpd_pdrv_probe,
	.remove = kpd_pdrv_remove,
	.driver = {
		   .name = KPD_NAME,
		   .of_match_table = kpd_of_match,
		   },
};

module_platform_driver(kpd_pdrv);

MODULE_AUTHOR("Mediatek Corporation");
MODULE_DESCRIPTION("MTK Keypad (KPD) Driver");
MODULE_LICENSE("GPL");

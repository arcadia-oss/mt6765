// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2013 Google, Inc.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/string.h>
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
#include <trusty/sm_err.h>
#else
#include <linux/trusty/sm_err.h>
#endif
#include <teei_trusty.h>
#include <teei_smcall.h>
#include "teei_bootprof.h"

static struct platform_device *trusty_mtee_dev;

s32 trusty_mtee_std_call32(u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	return trusty_std_call32(trusty_mtee_dev->dev.parent,
				 smcnr, a0, a1, a2);
}

static int trusty_mtee_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);

	trusty_mtee_dev = pdev;

	return 0;
}

static int trusty_mtee_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);

	return 0;
}

#define MODULE_NAME "trusty-mtee"
static const struct of_device_id trusty_mtee_of_match[] = {
	{ .compatible = "mediatek,trusty-mtee-v1", },
};
MODULE_DEVICE_TABLE(of, trusty_mtee_of_match);

static struct platform_driver trusty_mtee_driver = {
	.probe = trusty_mtee_probe,
	.remove = trusty_mtee_remove,
	.driver	= {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = trusty_mtee_of_match,
	},
};

static int __init register_trusty_mtee_driver(void)
{
	int ret = 0;

	if (platform_driver_register(&trusty_mtee_driver)) {
		ret = -ENODEV;
		pr_warn("[%s] could not register device for the device, ret:%d\n",
			MODULE_NAME,
			ret);
		return ret;
	}

	return ret;
}

static int __init trusty_mtee_init(void)
{
	int ret = 0;

	ret = register_trusty_mtee_driver();
	if (ret) {
		pr_warn("[%s] register device/driver failed, ret:%d\n",
			MODULE_NAME,
			ret);
		return ret;
	}

	return 0;
}
subsys_initcall(trusty_mtee_init);


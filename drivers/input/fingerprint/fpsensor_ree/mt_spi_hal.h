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
#ifndef __MT_SPI_HAL_H__
#define __MT_SPI_HAL_H__

#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#endif				/* !defined(CONFIG_MTK_CLKMGR) */

//#include <linux/wakelock.h>
#include <mt_spi.h>

struct mt_spi_t {
	struct platform_device *pdev;
	void __iomem *regs;
	int irq;
	int running;

	//struct wake_lock wk_lock;
	struct mt_chip_conf *config;
	struct spi_master *master;

	struct spi_transfer *cur_transfer;
	struct spi_transfer *next_transfer;

	spinlock_t lock;
	struct list_head queue;
#if !defined(CONFIG_MTK_CLKMGR)
	struct clk *clk_main;	/* main clock for spi bus */
#endif				/* !defined(CONFIG_MTK_CLKMGR) */
};


#endif


/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MICROTRUST Incorporated
 *
 */

#ifndef TEEI_BOOTPROF_H
#define TEEI_BOOTPROF_H
#include <linux/version.h>

#ifdef CONFIG_MTPROF
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
extern void bootprof_log_boot(char *str);
#define TEEI_BOOT_FOOTPRINT(str) bootprof_log_boot(str)
#else
extern void log_boot(char *str);
#define TEEI_BOOT_FOOTPRINT(str) log_boot(str)
#endif
#endif

#endif

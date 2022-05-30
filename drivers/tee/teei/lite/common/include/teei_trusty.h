/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MICROTRUST Incorporated
 *
 */
#ifndef _TEEI_TRUSTY_H_
#define _TEEI_TRUSTY_H_
#include <linux/version.h>
#include <linux/kernel.h>
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
#include <trusty/sm_err.h>
#else
#include <linux/trusty/sm_err.h> 
#endif
#include <linux/device.h>
#include <linux/pagemap.h>

#ifdef CONFIG_TRUSTY_INTERRUPT_MAP
extern void handle_trusty_ipi(int ipinr);
#endif
s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);
s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);
#ifdef CONFIG_TRUSTY_WDT_FIQ_ARMV7_SUPPORT
s32 trusty_fast_call32_nodev(u32 smcnr, u32 a0, u32 a1, u32 a2);
#endif
#ifdef CONFIG_64BIT
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2);
#endif

struct notifier_block;
enum {
	TRUSTY_CALL_PREPARE,
	TRUSTY_CALL_RETURNED,
};
int trusty_call_notifier_register(struct device *dev,
				  struct notifier_block *n);
int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n);
const char *trusty_version_str_get(struct device *dev);
u32 trusty_get_api_version(struct device *dev);

struct ns_mem_page_info {
	uint64_t attr;
};

int trusty_encode_page_info(struct ns_mem_page_info *inf,
			    struct page *page, pgprot_t pgprot);

int trusty_call32_mem_buf(struct device *dev, u32 smcnr,
			  struct page *page,  u32 size,
			  pgprot_t pgprot);

#endif /* end _TEEI_TRUSTY_H_ */

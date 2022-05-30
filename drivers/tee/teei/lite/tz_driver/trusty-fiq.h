/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2014 Google, Inc.
 *
 */

#ifndef __TRUSTY_FIQ_H__
#define __TRUSTY_FIQ_H__

int trusty_fiq_arch_probe(struct platform_device *pdev);
void trusty_fiq_arch_remove(struct platform_device *pdev);

#endif /* __TRUSTY_FIQ_H__ */

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Microtrust, Inc.
 *
 */

#ifndef _TEEI_LOADER_H_
#define _TEEI_LOADER_H_

struct TEEC_UUID {
	uint32_t timeLow;
	uint16_t timeMid;
	uint16_t timeHiAndVersion;
	uint8_t clockSeqAndNode[8];
};

extern struct TEEC_UUID uuid_fp;
extern s32 trusty_mtee_std_call32(u32 smcnr, u32 a0, u32 a1, u32 a2);

#endif

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __IMGSENSOR_PROC_H__
#define __IMGSENSOR_PROC_H__

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "imgsensor_common.h"

#define PROC_CAMERA_INFO "driver/camera_info"
#define PROC_SENSOR_STAT "driver/imgsensor_status_info"
// light start
#if defined(MACRO_CAM_DEV_NODE)
#define PROC_MACRO_CAM "driver/macro_cam"
#define macro_cam_size 50
extern char macro_cam[macro_cam_size];
#endif
#if defined(WIDE_ANGLE_CAM_DEV)
#define PROC_WIDE_ANGLE_CAM "driver/wide_angle_cam"
#define wide_angle_cam_size 50
extern char wide_angle_cam[wide_angle_cam_size];
#endif
// light end

#define IMGSENSOR_STATUS_INFO_LENGTH 128
#define camera_info_size 4096

extern char mtk_ccm_name[camera_info_size];
extern struct IMGSENSOR *pgimgsensor;

enum IMGSENSOR_RETURN imgsensor_proc_init(void);
#endif


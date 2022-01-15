// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "kd_imgsensor.h"
#include "imgsensor_sensor_list.h"

/* Add Sensor Init function here
 * Note:
 * 1. Add by the resolution from ""large to small"", due to large sensor
 *    will be possible to be main sensor.
 *    This can avoid I2C error during searching sensor.
 * 2. This should be the same as
 *     mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
 */
struct IMGSENSOR_INIT_FUNC_LIST kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR] = {
	/*OV*/
#if defined(OV13853_MIPI_RAW)
	{OV13853_SENSOR_ID, SENSOR_DRVNAME_OV13853_MIPI_RAW, OV13853_MIPI_RAW_SensorInit},
#endif

	/*HI*/
#if defined(HI846_MIPI_RAW)
	{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif

#if defined(HI846_2LANE_MIPI_RAW)
	{HI846_2LANE_SENSOR_ID, SENSOR_DRVNAME_HI846_2LANE_MIPI_RAW, HI846_2LANE_MIPI_RAW_SensorInit},
#endif

	/*GC*/
#if defined(GC5035_MIPI_RAW)
	{GC5035_SENSOR_ID, SENSOR_DRVNAME_GC5035_MIPI_RAW, GC5035_MIPI_RAW_SensorInit},
#endif

#if defined(GC02M1_MIPI_RAW)
	{GC02M1_SENSOR_ID, SENSOR_DRVNAME_GC02M1_MIPI_RAW, GC02M1_MIPI_RAW_SensorInit},
#endif

	/*IMX*/
#if defined(IMX214_MIPI_RAW)
	{IMX214_SENSOR_ID, SENSOR_DRVNAME_IMX214_MIPI_RAW, IMX214_MIPI_RAW_SensorInit},
#endif

#if defined(IMX134_MIPI_RAW)
	{IMX134_SENSOR_ID, SENSOR_DRVNAME_IMX134_MIPI_RAW,IMX134_MIPI_RAW_SensorInit},
#endif

#if defined(IMX134_2LANE_MIPI_RAW)
	{IMX134_2LANE_SENSOR_ID, SENSOR_DRVNAME_IMX134_2LANE_MIPI_RAW,IMX134_2LANE_MIPI_RAW_SensorInit},
#endif

	/*S5K*/
#if defined(S5K3L6_MIPI_RAW)
	{S5K3L6_SENSOR_ID, SENSOR_DRVNAME_S5K3L6_MIPI_RAW, S5K3L6_MIPI_RAW_SensorInit},
#endif
	/*  ADD sensor driver before this line */
	{0, {0}, NULL}, /* end of list */
};
/* e_add new sensor driver here */


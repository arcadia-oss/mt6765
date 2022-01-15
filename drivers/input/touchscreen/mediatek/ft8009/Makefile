# Makefile for the focaltech touchscreen drivers.

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
#ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/spi/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include


obj-y    +=  focaltech_core.o
obj-y    +=  focaltech_ex_fun.o
obj-y    +=  focaltech_ex_mode.o
obj-y    +=  focaltech_gesture.o
obj-y    +=  focaltech_esdcheck.o
obj-y    +=  focaltech_point_report_check.o
ifeq ($(TOUCHPANEL_TOUCHFELL_IN_PHONE_CALL),yes)
obj-y    +=  focaltech_proximity.o
endif

obj-y    +=  focaltech_flash.o
#obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS) +=     focaltech_flash/



obj-y    +=  focaltech_i2c.o

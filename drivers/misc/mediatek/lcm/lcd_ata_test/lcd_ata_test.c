#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK //xen 20160603
    #include <platform/mt_gpio.h>
    //#include "cust_gpio_usage.h"
#else
    //#include <linux/gpio.h>
    //#include <mt-plat/mtk_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

//modified by xen 20170414
#if defined(LCM_480_800)
 #define FRAME_WIDTH  										(480)
 #define FRAME_HEIGHT 										(800)
#elif defined(LCM_480_854)
 #define FRAME_WIDTH  										(480)
 #define FRAME_HEIGHT 										(854)
#elif defined(LCM_480_960)
 #define FRAME_WIDTH  										(480)
 #define FRAME_HEIGHT 										(960)
#elif defined(LCM_540_960)
 #define FRAME_WIDTH  										(540)
 #define FRAME_HEIGHT 										(960)
#elif defined(LCM_720_1280)
 #define FRAME_WIDTH  										(720)
 #define FRAME_HEIGHT 										(1280)
#elif defined(LCM_640_1280)
 #define FRAME_WIDTH  										(640)
 #define FRAME_HEIGHT 										(1280)
#elif defined(LCM_720_1440)
 #define FRAME_WIDTH  										(720)
 #define FRAME_HEIGHT 										(1440)
#elif defined(LCM_1080_1920)
 #define FRAME_WIDTH  										(1080)
 #define FRAME_HEIGHT 										(1920)
#elif defined(LCM_320_480)
 #define FRAME_WIDTH  										(320)
 #define FRAME_HEIGHT 										(480)
#else //default is 480X854
 #define FRAME_WIDTH  										(480)
 #define FRAME_HEIGHT 										(854)
#endif

#define LCM_PHYSICAL_WIDTH									(0)
#define LCM_PHYSICAL_HEIGHT									(0)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#ifdef BUILD_LK //xjl 20180531
static LCM_UTIL_FUNCS lcm_util = {0};
#else
static struct LCM_UTIL_FUNCS lcm_util = {0};
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

#ifdef BUILD_LK //xjl 20180531
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
#else
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}
#endif

#ifdef BUILD_LK //xjl 20180531
static void lcm_get_params(LCM_PARAMS *params)
#else
static void lcm_get_params(struct LCM_PARAMS *params)
#endif
{
#ifdef BUILD_LK //xjl 20180531
	memset(params, 0, sizeof(LCM_PARAMS));
#else
	memset(params, 0, sizeof(struct LCM_PARAMS));
#endif
	
    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
	
#ifndef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;

	params->dsi.switch_mode = CMD_MODE;
	params->dsi.switch_mode_enable = 0;	
#endif

    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    //Highly depends on LCD driver capability.
    //video mode timing

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 30;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 10;
	params->dsi.horizontal_frontporch				= 10;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    //noncontiune clk
    params->dsi.cont_clock=0;
    params->dsi.clk_lp_per_line_enable = 1;	
  
    //improve clk quality
    params->dsi.PLL_CLOCK = 200; //this value must be in MTK suggested table
    params->dsi.compatibility_for_nvk = 1;
    params->dsi.ssc_disable = 1;
}

static void lcm_init(void)
{
	
}

static void lcm_suspend(void)
{
	
}

static void lcm_resume(void)
{
    
}

static unsigned int lcm_compare_id(void)
{
	return 0;
}


#ifdef BUILD_LK //xjl 20180531
LCM_DRIVER lcd_ata_test_lcm_drv = 
#else
struct LCM_DRIVER lcd_ata_test_lcm_drv = 
#endif
{
	.name		= "lcd_ata_test",
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params	= lcm_get_params,
	.init		= lcm_init,
	.suspend	= lcm_suspend,
	.resume		= lcm_resume,
	.compare_id	= lcm_compare_id,
};


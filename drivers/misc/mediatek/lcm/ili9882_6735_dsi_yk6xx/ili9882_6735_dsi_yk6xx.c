/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
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

#define LCM_ID_ILI9882C (0x9882)

#define LCM_DBG(fmt, arg...)
	//LCM_PRINT ("[LCM-ILI9882-DSI-VDO] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0

#if defined(LCM_720_1440)
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)
#elif defined(LCM_640_1280)
#define FRAME_WIDTH  										(640)
#define FRAME_HEIGHT 										(1280)
#elif defined(LCM_600_1280)
#define FRAME_WIDTH  										(600)
#define FRAME_HEIGHT 										(1280)
#elif defined(LCM_720_1520)
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1520)
#elif defined(LCM_720_1528)
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1528)
#elif defined(LCM_720_1560)
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1560)
#elif defined(LCM_720_1600)
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1600)
#else
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#endif

#if defined(YK676_CUSTOMER_TRX_S606_HDPLUS)||defined(YK676_V60_CUSTOMER_TRX_S607_HDPLUS)//hzr add
#define LCM_PHYSICAL_WIDTH                                  (64800)
#define LCM_PHYSICAL_HEIGHT                                 (140400)
#else
#define LCM_PHYSICAL_WIDTH									(0)
#define LCM_PHYSICAL_HEIGHT									(0)
#endif

#if defined(YK736_CUSTOMER_XINJIDE_G1_HDPLUS)//zwl 20180314
#define REGFLAG_DELAY             							0XFD
#else
#define REGFLAG_DELAY             							0XFE
#endif
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#ifdef BUILD_LK //xjl 20180531
static LCM_UTIL_FUNCS lcm_util = {0};
#else
static struct LCM_UTIL_FUNCS lcm_util = {0};
#endif

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

#define YK_LCM_5V_IC_SUPPORT

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

#ifdef YK_LCM_5V_IC_SUPPORT

extern int set_gpio_lcm_enp_enn(int enable);

#ifdef BUILD_LK
extern int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value);
#else
extern int tps65132_write_bytes(unsigned char addr, unsigned char value);
#endif


#endif

struct LCM_setting_table {
    unsigned int cmd; //zxs 20151127
    unsigned char count;
    unsigned char para_list[64];//5];  //xen for less space 20170511
};


static unsigned char BufLcmInfo[] = "LCD_HUAJIACAI_ILI9882N_HDPLUS_65";
static struct LCM_setting_table lcm_initialization_setting[] = {
// GIP Setting
{0xFF,3,{0x98,0x82,0x01}},
{0x00,1,{0x46}},  
{0x01,1,{0x16}},
{0x02,1,{0x10}},  //10
{0x03,1,{0x10}},
{0x08,1,{0x80}},  
{0x09,1,{0x12}},  //DUMMT CK
{0x0a,1,{0x71}},
{0x0b,1,{0x00}},  //clk keep 10 off 00
{0x14,1,{0x8A}},  //KEEP
{0x15,1,{0x8A}},  //KEEP
{0x0c,1,{0x10}},  //01
{0x0d,1,{0x10}},  //01
{0x0e,1,{0x00}},  
{0x0f,1,{0x00}},
{0x10,1,{0x01}},
{0x11,1,{0x01}},
{0x12,1,{0x01}},
{0x24,1,{0x00}},  //01 
{0x25,1,{0x09}},  
{0x26,1,{0x10}},
{0x27,1,{0x10}},
{0x31,1,{0x21}},  //STV_C
{0x32,1,{0x07}},  
{0x33,1,{0x01}},  
{0x34,1,{0x00}},  
{0x35,1,{0x02}},  
{0x36,1,{0x07}},
{0x37,1,{0x07}},
{0x38,1,{0x07}}, 
{0x39,1,{0x07}},  
{0x3a,1,{0x17}},  //CLK8
{0x3b,1,{0x15}},  //CLK6
{0x3c,1,{0x07}},  
{0x3d,1,{0x07}},  
{0x3e,1,{0x13}},  //CLK4
{0x3f,1,{0x11}},  //CLK2
{0x40,1,{0x09}},  //STV_A  
{0x41,1,{0x07}},
{0x42,1,{0x07}},
{0x43,1,{0x07}}, 
{0x44,1,{0x07}},  
{0x45,1,{0x07}},  
{0x46,1,{0x07}},
{0x47,1,{0x20}},  //STV_C
{0x48,1,{0x07}},  
{0x49,1,{0x01}},  
{0x4a,1,{0x00}},
{0x4b,1,{0x02}},
{0x4c,1,{0x07}},
{0x4d,1,{0x07}},  
{0x4e,1,{0x07}},
{0x4f,1,{0x07}},
{0x50,1,{0x16}},  //CLK7
{0x51,1,{0x14}},  //CLK5
{0x52,1,{0x07}},
{0x53,1,{0x07}},  
{0x54,1,{0x12}},  //CLK3
{0x55,1,{0x10}},  //CLK1
{0x56,1,{0x08}},  //STV_A
{0x57,1,{0x07}},
{0x58,1,{0x07}},  
{0x59,1,{0x07}},  
{0x5a,1,{0x07}},
{0x5b,1,{0x07}},
{0x5c,1,{0x07}},
{0x61,1,{0x08}},  
{0x62,1,{0x07}},
{0x63,1,{0x01}},
{0x64,1,{0x00}},  
{0x65,1,{0x02}},  
{0x66,1,{0x07}},
{0x67,1,{0x07}},
{0x68,1,{0x07}},  
{0x69,1,{0x07}},  
{0x6a,1,{0x10}},
{0x6b,1,{0x12}},
{0x6c,1,{0x07}},  
{0x6d,1,{0x07}},  
{0x6e,1,{0x14}},
{0x6f,1,{0x16}},
{0x70,1,{0x20}},  
{0x71,1,{0x07}},  
{0x72,1,{0x07}},
{0x73,1,{0x07}},
{0x74,1,{0x07}},  
{0x75,1,{0x07}},  
{0x76,1,{0x07}},
{0x77,1,{0x09}},
{0x78,1,{0x07}},  
{0x79,1,{0x01}},  
{0x7a,1,{0x00}},
{0x7b,1,{0x02}},
{0x7c,1,{0x07}},
{0x7d,1,{0x07}},  
{0x7e,1,{0x07}},
{0x7f,1,{0x07}},
{0x80,1,{0x11}},  
{0x81,1,{0x13}},
{0x82,1,{0x07}},
{0x83,1,{0x07}},  
{0x84,1,{0x15}},
{0x85,1,{0x17}},
{0x86,1,{0x21}},
{0x87,1,{0x07}},
{0x88,1,{0x07}},  
{0x89,1,{0x07}},  
{0x8a,1,{0x07}},
{0x8b,1,{0x07}},
{0x8c,1,{0x07}},
{0xA0,1,{0x01}},
{0xA1,1,{0x10}},  
{0xA2,1,{0x08}},  
{0xA5,1,{0x10}},  
{0xA6,1,{0x10}},
{0xA7,1,{0x00}},
{0xA8,1,{0x00}},  
{0xA9,1,{0x09}},  
{0xAa,1,{0x09}},
{0xb9,1,{0x40}},
{0xd0,1,{0x01}},
{0xd1,1,{0x00}},
{0xdc,1,{0x35}},
{0xdd,1,{0x42}},
{0xe2,1,{0x00}},
{0xe6,1,{0x22}},
{0xe7,1,{0x54}},              //V-porch SRC=V0

// RTN. Internal VBP,0x Internal VFP
{0xFF,3,{0x98,0x82,0x02}},
{0xF1,1,{0x1C}},    // Tcon ESD option
{0x4B,1,{0x5A}},    // line_chopper
{0x50,1,{0xCA}},    // line_chopper
{0x51,1,{0x00}},    // line_chopper
{0x06,1,{0x91}},    // Internal Line Time (RTN)
{0x0B,1,{0xA0}},    // Internal VFP[9]
{0x0C,1,{0x00}},    // Internal VFP[8]
{0x0D,1,{0x14}},    // Internal VBP
{0x0E,1,{0xF6}},    // Internal VFP
{0x4E,1,{0x11}},    // SRC BIAS

// Power Setting
{0xFF,3,{0x98,0x82,0x05}},
{0x03,1,{0x00}},     //VCOM
{0x04,1,{0xDA}},     // VCOM
{0x58,1,{0x61}},     // VGL 2x
{0x63,1,{0x8A}},     // GVDDN = -5.24V
{0x64,1,{0x88}},     // GVDDP = 5.2V
{0x68,1,{0xA1}},     // VGHO = 15V
{0x69,1,{0xA7}},     // VGH = 16V
{0x6A,1,{0x79}},     // VGLO = -10V
{0x6B,1,{0x6B}},     // VGL = -11V
{0x85,1,{0x37}},     // HW RESET option
{0x46,1,{0x00}},     // LVD HVREG option
{0x23,1,{0x42}},     //MDT LPWG timing
{0x24,1,{0x52}},     //MDT LPWG timing

// Resolution
{0xFF,3,{0x98,0x82,0x06}},
{0xD9,1,{0x1F}},     // 1F：4Lane；10：3Lane；
{0xC0,1,{0x18}},     // NL = 1560
{0xC1,1,{0x16}},     // NL = 1560

// Gamma Register
{0xFF,3,{0x98,0x82,0x08}},
{0xE0,27,{0x00,0x24,0x83,0xBB,0xFB,0x55,0x2E,0x54,0x80,0xA2,0xA9,0xD8,0x02,0x28,0x4D,0xAA,0x74,0xA4,0xC4,0xEC,0xFF,0x0E,0x3A,0x70,0x9C,0x03,0xEC}},
{0xE1,27,{0x00,0x24,0x83,0xBB,0xFB,0x55,0x2E,0x54,0x80,0xA2,0xA9,0xD8,0x02,0x28,0x4D,0xAA,0x74,0xA4,0xC4,0xEC,0xFF,0x0E,0x3A,0x70,0x9C,0x03,0xEC}},

// OSC Auto Trim Setting
{0xFF,3,{0x98,0x82,0x0B}},
{0x9A,1,{0x44}},
{0x9B,1,{0xA2}},
{0x9C,1,{0x03}},
{0x9D,1,{0x03}},
{0x9E,1,{0x74}},
{0x9F,1,{0x74}},
{0xAB,1,{0xE0}},     // AutoTrimType

{0xFF,3,{0x98,0x82,0x0E}},
{0x00,1,{0xA0}},     // LV mode
{0x11,1,{0x10}},     // TSVD Rise Poisition
{0x13,1,{0x14}},     // TSHD Rise Poisition

{0xFF,3,{0x98,0x82,0x00}},
{0x35,1,{0x00}}, //TE enable

{0x11, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 20, {}}, 
{REGFLAG_END_OF_TABLE, 0x00, {}}
};




static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 30, {}},
	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

#ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
#else
static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
#endif
{
	unsigned int i;
    for(i = 0; i < count; i++) {	
        unsigned int cmd; //zxs 20151127
        cmd = table[i].cmd;
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

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

#if defined(DOUBLE_LCM_BY_IDPIN) //xen 20180117
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

		// enable tearing-free
		//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //BURST_VDO_MODE;
        #endif
		
        #if defined(YK676_CUSTOMER_TRX_S606_HDPLUS)||defined(YK676_V60_CUSTOMER_TRX_S607_HDPLUS)//hzr add
        params->physical_width = LCM_PHYSICAL_WIDTH/1000;
        params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
        #endif

        #ifndef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
        	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
        	params->physical_width_um = LCM_PHYSICAL_WIDTH;
        	params->physical_height_um = LCM_PHYSICAL_HEIGHT;	

        	params->dsi.switch_mode = CMD_MODE;
        	params->dsi.switch_mode_enable = 0;
        #endif

		if (get_lcd_id_state()==0)
		  params->dsi.g_StrLcmInfo = BufLcmInfo_0;
		else
		  params->dsi.g_StrLcmInfo = BufLcmInfo_1;

		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change,should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	

		params->dsi.vertical_sync_active				= 8;	//2;
		params->dsi.vertical_backporch					= 18;	//14;
		params->dsi.vertical_frontporch					= 20;	//16;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		
		params->dsi.horizontal_sync_active				= 40;	//2;
		params->dsi.horizontal_backporch				= 120;	//60;	//42;
		params->dsi.horizontal_frontporch				= 100;	//60;	//44;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 210;//208;	


        #if defined(MTK_ESD_TEST) //zxs 20160318 //zxs 20151107
		params->dsi.esd_check_enable =1;
		params->dsi.customization_esd_check_enable =1;
		params->dsi.lcm_esd_check_table[0].cmd   =0x0A;
		params->dsi.lcm_esd_check_table[0].count = 1;//1;
		params->dsi.lcm_esd_check_table[0].para_list[0] =0x9C;

		params->dsi.noncont_clock=0;
		params->dsi.clk_lp_per_line_enable=1;
        #endif		
}
#else
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

		// enable tearing-free
		//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //BURST_VDO_MODE;
        #endif
		
        #if defined(YK676_CUSTOMER_TRX_S606_HDPLUS)||defined(YK676_V60_CUSTOMER_TRX_S607_HDPLUS)//hzr add
        params->physical_width = LCM_PHYSICAL_WIDTH/1000;
        params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
        #endif

        #ifndef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
        	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
        	params->physical_width_um = LCM_PHYSICAL_WIDTH;
        	params->physical_height_um = LCM_PHYSICAL_HEIGHT;	

        	params->dsi.switch_mode = CMD_MODE;
        	params->dsi.switch_mode_enable = 0;
        #endif

		params->dsi.g_StrLcmInfo = BufLcmInfo;

		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	

        params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 236;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		
		params->dsi.horizontal_sync_active				= 8;
		params->dsi.horizontal_backporch				= 40;
		params->dsi.horizontal_frontporch				= 60;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 307;

#if defined(MTK_ESD_TEST) //zxs 20160318 //zxs 20151107
        params->dsi.esd_check_enable =1;
        params->dsi.customization_esd_check_enable =1;
        params->dsi.lcm_esd_check_table[0].cmd   =0x0A;
        params->dsi.lcm_esd_check_table[0].count = 1;//1;
        params->dsi.lcm_esd_check_table[0].para_list[0] =0x9C;

        params->dsi.noncont_clock=0;
        params->dsi.clk_lp_per_line_enable=1;
#endif		

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[3];
	unsigned int array[16];  

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);	

	array[0]=0x00063902;
	array[1]=0x008298FF;//1>6
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10); 

    array[0] = 0x00023700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x04,buffer, 2);

	//zrl modify for ili9806 read ID,121113       
	id = buffer[0]<<8 |buffer[1];

#if defined(BUILD_LK)
	//printf("ili9882c 0x%x , 0x%x , 0x%x \n",buffer[0],buffer[1],id);
#else
	printk("ili9882c 0x%x , 0x%x , 0x%x \n",buffer[0],buffer[1],id);
#endif
	//return 1;
    return (id == 0x0080)?1:0;
}                                     

static void lcm_init(void)
{
#ifdef YK_LCM_5V_IC_SUPPORT
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
#ifndef CONFIG_FPGA_EARLY_PORTING
	int ret = 0;
#endif
	cmd = 0x00;
	data = 0x0F;
	SET_RESET_PIN(0);
#ifndef BUILD_LK 
	//set_gpio_lcd_enp(1);
	set_gpio_lcm_enp_enn(1);
	MDELAY(5);
#endif
#ifdef BUILD_LK
	ret = TPS65132_write_byte(cmd, data);
#else
	ret = tps65132_write_bytes(cmd, data);
#endif
	cmd = 0x01;
	data = 0x0F;
#ifdef BUILD_LK
	ret = TPS65132_write_byte(cmd, data);
#else
	ret = tps65132_write_bytes(cmd, data);
#endif
#endif
#ifdef YK_INCELL_LCM_SUPPORT
#ifdef BUILD_LK
	int ret = 0;

	SET_RESET_PIN(1);
	ret = PMU_REG_MASK(0xB0, 0x00, 0x01);
	//ret = PMU_REG_MASK(0xB2, 0x28, 0x3f);
	ret = PMU_REG_MASK(0xB3, 0x24, 0x3F);

	ret = PMU_REG_MASK(0xB4, 0x24, 0x3F);

	ret = PMU_REG_MASK(0xB1, 0x40, 0x48);

	MDELAY(15);
	ret = PMU_REG_MASK(0xB1, 0x48, 0x48);
	MDELAY(2);

#else
	display_bias_enable();
	MDELAY(2);
#endif
#endif
	SET_RESET_PIN(1);	 
	MDELAY(10); //20
	SET_RESET_PIN(0);
	MDELAY(20);  // 50 
	SET_RESET_PIN(1);
	MDELAY(50);  // 150

#if defined(DOUBLE_LCM_BY_IDPIN) //xjl 20161105
    if (get_lcd_id_state()==1)
    {
 #ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        push_table(lcm_initialization_setting_1, sizeof(lcm_initialization_setting_1) / sizeof(struct LCM_setting_table), 1);
 #else
	push_table(NULL, lcm_initialization_setting_1, sizeof(lcm_initialization_setting_1) / sizeof(struct LCM_setting_table), 1);
 #endif
    }
    else
    {
 #ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        push_table(lcm_initialization_setting_0, sizeof(lcm_initialization_setting_0) / sizeof(struct LCM_setting_table), 1);
 #else
	push_table(NULL, lcm_initialization_setting_0, sizeof(lcm_initialization_setting_0) / sizeof(struct LCM_setting_table), 1);
 #endif
    }
#else
 #ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
 #else
	push_table(NULL, lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
 #endif
#endif
}


static void lcm_suspend(void)
{
	LCM_DBG("lcm_suspend");
#ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#else
	push_table(NULL, lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#endif
	//SET_RESET_PIN(0);
    //MDELAY(100);
   	//SET_RESET_PIN(1); 
#ifdef YK_INCELL_LCM_SUPPORT
#ifndef BUILD_LK
//if(!tpd_load_status) //xjl 20181026
	display_bias_disable();
#endif
#endif

#ifdef YK_LCM_5V_IC_SUPPORT
	MDELAY(10);
#ifndef BUILD_LK
	set_gpio_lcm_enp_enn(0);
#endif
#endif
}


static void lcm_resume(void)
{
	LCM_DBG("lcm_resume");
	//lcm_init();
	SET_RESET_PIN(1);	 
	MDELAY(10); //20
	SET_RESET_PIN(0);
	MDELAY(20);  // 50 
	SET_RESET_PIN(1);
	MDELAY(50);  // 150

#if defined(DOUBLE_LCM_BY_IDPIN) //xjl 20161105
    if (get_lcd_id_state()==1)
    {
 #ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        push_table(lcm_initialization_setting_1, sizeof(lcm_initialization_setting_1) / sizeof(struct LCM_setting_table), 1);
 #else
	push_table(NULL, lcm_initialization_setting_1, sizeof(lcm_initialization_setting_1) / sizeof(struct LCM_setting_table), 1);
 #endif
    }
    else
    {
 #ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        push_table(lcm_initialization_setting_0, sizeof(lcm_initialization_setting_0) / sizeof(struct LCM_setting_table), 1);
 #else
	push_table(NULL, lcm_initialization_setting_0, sizeof(lcm_initialization_setting_0) / sizeof(struct LCM_setting_table), 1);
 #endif
    }
#else
 #ifdef BUILD_LK //added by xen for softlink lk&kernel lcm driver 20171020
        push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
 #else
	push_table(NULL, lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
 #endif
#endif
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}
#endif

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
#ifdef BUILD_LK //xjl 20180531
LCM_DRIVER ili9882_6735_dsi_lcm_drv_yk6xx = 
#else
struct LCM_DRIVER ili9882_6735_dsi_lcm_drv_yk6xx = 
#endif
{
    .name		    = "ili9882n_67xx_dsi_video",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};

/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH										  (720)
#define FRAME_HEIGHT										(1280)

#define REGFLAG_DELAY								    0xFE
#define REGFLAG_END_OF_TABLE						0xFF	/* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE									0
#define LCM_ID_ILI9807  0x9807

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)					 lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		 lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg					 lcm_util.dsi_read_reg()


struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
{0xB9,3,{0xF1,0x12,0x83}},
{0xBA,27,{0x32,0x81,0x05,0xF9,0x0E,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x25,0x00,0x91,0x0A,0x00,0x00,0x02,0x4F,0xD1,0x00,0x00,0x37}},
{0xB8,4,{0x25,0x22,0x20,0x03}},
{0xBF,3,{0x02,0x10,0x00}},
{0xB3,10,{0x0C,0x10,0x0A,0x50,0x03,0xFF,0x00,0x00,0x00,0x00}},
{0xC0,9,{0x73,0x73,0x50,0x50,0x00,0x00,0x08,0x70,0x00}},
{0xBC,1,{0x4F}},
{0xCC,1,{0x0B}},
{0xB4,1,{0x80}},
{0xB2,3,{0xC8,0x12,0xF0}},
{0xE3,14,{0x07,0x07,0x0B,0x0B,0x03,0x03,0x03,0x03,0x00,0x00,0xFF,0x00,0xC0,0x10}},
//{0xC1,12,{0x55,0x00,0x1E,0x1E,0x77,0xD1,0xFF,0xFF,0xCC,0xCC,0x77,0x77}},
{0xC1,12,{0x55,0x00,0x1E,0x1E,0x77,0xE1,0xCC,0xCC,0xCC,0xCC,0x33,0x33}},
{0xB5,2,{0x09,0x09}},
{0xB6,2,{0x9e,0x9e}},//7C
{0xE9,63,{0xC2,0x10,0x06,0x05,0x01,0x05,0xA0,0x12,0x31,0x23,0x3F,0x82,0x05,0xA0,0x37,0x0A,0x20,0x00,0x01,0x00,0x00,0x00,0x20,0x00,0x01,0x00,0x00,0x00,0x00,0x24,0x68,0x88,0x88,0x88,0x88,0x88,0x88,0x8F,0x84,0x11,0x35,0x78,0x88,0x88,0x88,0x88,0x88,0x88,0x8F,0x85,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xEA,61,{0x00,0x1A,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x57,0x53,0x18,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0xF1,0x46,0x42,0x08,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0xF0,0x23,0x10,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x02,0xA0,0x00,0x00,0x00,0x00}},
{0xE0,34,{0x00,0x04,0x08,0x2A,0x30,0x38,0x39,0x39,0x06,0x0B,0x0D,0x11,0x13,0x12,0x13,0x12,0x19,0x00,0x04,0x08,0x2A,0x30,0x38,0x39,0x39,0x06,0x0B,0x0D,0x11,0x13,0x12,0x13,0x12,0x19}},


		{0X35,1,{0X00}},

		{0x11,1,{0x00}},  
		{REGFLAG_DELAY, 250, {}},

		{0x29,1, {0x00}},  
		{REGFLAG_DELAY, 50, {}},  



		// Setting ending by predefined flag
		{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*static struct LCM_setting_table lcm_sleep_out_setting[] = {
	 Sleep Out
	{ 0x11, 0, {} },
	{ REGFLAG_DELAY, 120, {} },

	 Display ON
	{ 0x29, 0, {} },
	{ REGFLAG_END_OF_TABLE, 0x00, {} }
};*/


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Sleep Mode On */
	{ 0x28, 0, {} },
	{ REGFLAG_DELAY, 200, {} },	
	
	{ 0x10, 0, {} },
	{ REGFLAG_DELAY, 120, {} },
	{ REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {

		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}

}


/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	// enable tearing-free
	
  params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
  params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_THREE_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;
	
	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;
	
	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;
	
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.vertical_sync_active     = 3;
	params->dsi.vertical_backporch		   = 10;
	params->dsi.vertical_frontporch 	   = 16;
	params->dsi.vertical_active_line	   = FRAME_HEIGHT; 
	
	params->dsi.horizontal_sync_active	 = 4;
	params->dsi.horizontal_backporch	   = 45;
	params->dsi.horizontal_frontporch	   = 45;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	params->dsi.PLL_CLOCK=230;
}

static void init_lcm_registers(void)
{
		push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_init(void)
{
		SET_RESET_PIN(1);
		MDELAY(10);
		SET_RESET_PIN(0);
		MDELAY(20);
		SET_RESET_PIN(1);
		MDELAY(200);

		init_lcm_registers();
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
}


static void lcm_resume(void)
{
	MDELAY(10);
	lcm_init();
	MDELAY(10);
}
#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
	return 1;
}
#endif
LCM_DRIVER st7703_hd720_dsi_vdo_lcm_drv = {

  	.name              = "st7703_hd720_dsi_vdo",
  	.set_util_funcs    = lcm_set_util_funcs,
  	.get_params        = lcm_get_params,
  	.init              = lcm_init,
  	.suspend           = lcm_suspend,
  	.resume            = lcm_resume,
  	#ifdef BUILD_LK
  	.compare_id        = lcm_compare_id,
  	#endif
};

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

#define FRAME_WIDTH										  (240)
#define FRAME_HEIGHT										(240)

#define REGFLAG_DELAY								    0xFE
#define REGFLAG_END_OF_TABLE						0xFF	/* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE									0

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

	//{ 0x11, 0, {}},
	{ 0xf0, 1, {0xc3}},
	{ 0xf0, 1, {0x96}},	
	{ 0xec, 3, {0x00,0x00,0x01}},		
	{ 0xb4, 1, {0x01}},	
	{ 0xb9, 2, {0x02,0xc0}},
	{ 0x21, 0, {}},	
	{ 0x3A, 1, {0x77}},	
	{ 0xb1, 2, {0x50,0x10}},
	//{ 0xb6, 3, {0x8a,0x07,0x1d}},
	{ 0xc5, 1, {0x1f}},	
	{ 0xe8, 8, {0x40,0x8a,0x1b,0x1b,0x10,
	            0x03,0xb8,0x33}},	
	{ 0x12, 0, {}},
	{ 0xe0, 14,{0x70,0x02,0x05,0x07,0x05,
	            0x12,0x21,0x32,0x38,0x27,
	            0x13,0x14,0x25,0x35,}},
	            	
	{ 0xe1, 14,{0x70,0x02,0x06,0x07,0x06,
	            0x22,0x23,0x33,0x39,0x17,
	            0x13,0x13,0x24,0x35,}},

	{ 0x36, 1, {0x48}},
	{ 0x35, 1, {0x00}},
	{ 0x2a, 4, {0x00,0x28,0x01,0x17}},
	{ 0x2b, 4, {0x00,0x00,0x00,0xef}},
	
	//{ 0x29, 0, {}},
	
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 0, {} },
	{ 0xc1, 1, {0x0f}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }	

	
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
	{ REGFLAG_DELAY, 20, {} },	
	
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
	params->dsi.mode  = BURST_VDO_MODE; //SYNC_EVENT_VDO_MODE;//SYNC_PULSE_VDO_MODE
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_ONE_LANE;
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
	
	params->dsi.vertical_sync_active     = 60;
	params->dsi.vertical_backporch		   = 60;
	params->dsi.vertical_frontporch 	   = 100;
	params->dsi.vertical_active_line	   = FRAME_HEIGHT; 
	
	params->dsi.horizontal_sync_active	 = 30;
	params->dsi.horizontal_backporch	   = 30;
	params->dsi.horizontal_frontporch	   = 200;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 168;

}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	
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

LCM_DRIVER st7796s_qvga_dsi_vdo_lcm_drv = {

  	.name              = "st7796s_qvga_dsi_vdo",
  	.set_util_funcs    = lcm_set_util_funcs,
  	.get_params        = lcm_get_params,
  	.init              = lcm_init,
  	.suspend           = lcm_suspend,
  	.resume            = lcm_resume,
};

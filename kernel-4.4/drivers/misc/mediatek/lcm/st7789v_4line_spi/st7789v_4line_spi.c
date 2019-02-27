#include "lcm_drv.h"
#include "lcm_define.h"
#include "lcm_i2c.h"
#ifdef BUILD_LK 
	#include <platform/mt_gpio.h>
	#include <platform/upmu_common.h>
	#include "boot_logo.h" // bootloader logo
#else
	#include <linux/string.h>
	#include <linux/gpio.h>
#endif



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_GC9305_ID										/*(0x8552)*/
#define FRAME_WIDTH  										(240)
#define FRAME_HEIGHT 										(320)

#define REGFLAG_DELAY             							(0X1FE)
#define REGFLAG_END_OF_TABLE      							(0x1FF)  // END OF REGISTERS MARKER
#define DCX_BIT 											(0x100)

#define LCM_DSI_CMD_MODE									 0

#ifndef TRUE
    #define   TRUE     1
#endif

#ifndef FALSE
    #define   FALSE    0
#endif

#define GPIO_OUT_ZERO 		0
#define GPIO_OUT_ONE 		1
// ---------------------------------------------------------------------------
//  Custom Define
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
#if defined(BUILD_LK)
  #define LCM_RESET_PIN                               (GPIO83 | 0x80000000)
	#define SET_RESET_PIN(v)                            {mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_00);mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);mt_set_gpio_out(LCM_RESET_PIN, (v));}
#else
	#define SET_RESET_PIN(v)    					    {lcm_util.set_reset_pin((v));}
#endif

#define UDELAY(n)										(lcm_util.udelay(n))
#define MDELAY(n)										(lcm_util.mdelay(n))
static LCM_UTIL_FUNCS lcm_util = {0};
														 
struct I2C_setting_table {
    unsigned char reg;
    unsigned char para;
};

struct SPI_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};


extern void lcm_spi_interface(unsigned char *sbuf, size_t spilen,unsigned char mode);
extern void spi_updata_screen(int w, int h, unsigned char *screen_data);
extern void clean_screen(void);
#if defined(CONFIG_LDO_BACKLIGHT_SUPPORT) // zhou add button backlight
extern void ldo_button_backlight_off(unsigned int off);
#endif
// ---------------------------------------------------------------------------
//  LCM Regs set
// ---------------------------------------------------------------------------


static struct SPI_setting_table spi_regs_setting[] = {
{0x11,0,{}},
{REGFLAG_DELAY,120,{}}, 

{0x36,1,{0x00}},
{0x3a,1,{0x05}},
{0xb2,5,{0x0c,0x0c,0x00,0x33,0x33}},

{0xb7,1,{0x35}},
{0xbb,1,{0x38}},
{0xc0,1,{0x2c}},
{0xc2,1,{0x01}},
{0xc3,1,{0x11}},
{0xc4,1,{0x20}},
{0xc6,1,{0x0f}},
{0xd0,2,{0xa4,0xa1}},

{0xe0,14,{0xd0,0x00,0x05,0x0e,0x15,
         0x0d,0x37,0x43,0x47,0x09,
         0x15,0x12,0x16,0x19}},
{0xe1,14,{0xd0,0x00,0x05,0x0d,0x0c,
         0x06,0x2d,0x44,0x40,0x0e,
         0x1c,0x18,0x16,0x19}},
    
{0x29,0,{}},
{REGFLAG_END_OF_TABLE, 0, {}}	             
};


static struct SPI_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct SPI_setting_table lcm_deep_sleep_mode_in_setting[] = {

	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

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

	params->dsi.mode   = SYNC_PULSE_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB565;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_16BIT_RGB565;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 4;
	params->dsi.vertical_frontporch					= 8;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 12;
	params->dsi.horizontal_frontporch				= 52;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	// Bit rate calculation
	//1 Every lane speed
	params->dsi.PLL_CLOCK=30;

    
   // params->dsi.g_StrLcmInfo = BufLcmInfo;
}


static void spi_push_table(struct SPI_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned char cmd;
	for(i = 0; i < count; i++) {
		switch (table[i].cmd) {	
		case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE :
			break;
		default:
			cmd=(unsigned char)table[i].cmd;
			lcm_spi_interface(&cmd, 1, 0);
			lcm_spi_interface(&table[i].para_list[0], table[i].count, 1);
		}
	}
}


static void lcm_set_regs(void)           
{

#if defined(BUILD_LK)
	printf ("=====lcm_set_regs=======");
#else
	printk ("=====lcm_set_regs=======");
#endif	

	SET_RESET_PIN(1);  
	MDELAY(20);	 
	SET_RESET_PIN(0);  
	MDELAY(50);
	SET_RESET_PIN(1);  
	MDELAY(150); 

	spi_push_table(spi_regs_setting, sizeof(spi_regs_setting)/sizeof(struct SPI_setting_table));
	MDELAY(40);
}

static void lcm_init(void)           
{
#ifdef BUILD_LK 
	printf("[%s]\n", __func__);
#else
  printk("[%s]\n", __func__);
#endif	
	lcm_set_regs();
	
	/* bootloader logo */
#ifdef BUILD_LK 	
	spi_updata_screen(240, 320, boot_logo);
#endif	
}


static void lcm_suspend(void)
{
#if defined(BUILD_LK)
	printf("[%s]\n", __func__);
#else
	printk("[%s]\n", __func__);
#endif
	spi_push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting)/sizeof(struct SPI_setting_table));

#if defined(CONFIG_LDO_BACKLIGHT_SUPPORT) // zhou add button backlight
   ldo_button_backlight_off(1);
#endif
}


static void lcm_resume(void)
{
#if defined(BUILD_LK)
printf("[%s]\n", __func__);
#else
	printk("[%s]\n", __func__);
#endif		
	spi_push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting)/sizeof(struct SPI_setting_table));
	
#if defined(CONFIG_LDO_BACKLIGHT_SUPPORT) // zhou add button backlight
   ldo_button_backlight_off(0);
#endif
}

#ifdef BUILD_LK	
static unsigned int lcm_compare_id(void)
{
#if 0
	unsigned char id_tmp0[2];
	unsigned char id_tmp1[2];
	unsigned int lcd_id;
	unsigned char cmd;
	
	SET_RESET_PIN(1);  
	MDELAY(150);	 
	SET_RESET_PIN(0);  
	MDELAY(200);
	SET_RESET_PIN(1);  
	MDELAY(250);
	id_tmp0[0]=0xdb;
	lcm_spi_interface(id_tmp0,2,2);
    MDELAY(10);
	id_tmp1[0]=0xdc;
	lcm_spi_interface(id_tmp1,2,2);
    MDELAY(10);

     printf("id_tmp0[0] = %d,id_tmp0[1]= %d \n",id_tmp0[0],id_tmp0[1]);
	 printf("id_tmp1[0] = %d,id_tmp1[1]= %d \n",id_tmp1[0],id_tmp1[1]);
    lcd_id = ((id_tmp0[0]&0xff)<<8)|(id_tmp1[0]&0xff);
    
   return (LCM_ST7789V_ID == lcd_id)?1:0;
#else
   return 1;
#endif     
}
#endif

LCM_DRIVER st7789v_4line_spi_lcm_drv =
{
	.name			= "st7789v_4line_spi",
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init			= lcm_init,
	.suspend		= lcm_suspend,
	.resume			= lcm_resume,
#ifdef BUILD_LK	
	.compare_id     = lcm_compare_id,
#endif
};


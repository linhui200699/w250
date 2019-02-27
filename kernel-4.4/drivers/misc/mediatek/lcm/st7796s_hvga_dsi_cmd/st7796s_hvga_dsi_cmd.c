#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include <platform/mt_i2c.h>
#include <platform/upmu_common.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include "ddp_hal.h"
#else
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/kernel.h>
#include <linux/delay.h>
//#include "extpwr_def.h"
#endif
#include "lcm_drv.h"
//#include "lk_logo.h"

#define FRAME_WIDTH  (128)
#define FRAME_HEIGHT (64)

#ifdef BUILD_LK
#define SPI_LCD_DEBUG(fmt, args...)    printf("spi_lcm func:%s line:%d : "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define SPI_LCD_DEBUG(fmt, args...)    printk("spi_lcm func:%s line:%d : "fmt, __FUNCTION__, __LINE__, ##args)
extern void lcm_send_cmd(unsigned char byte);
extern void lcm_send_data(unsigned char byte);

#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
//extern int mtk_gpio_base_teksun(void);
static LCM_UTIL_FUNCS lcm_util = {0};
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#ifdef BUILD_LK
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define lcm_send_cmd(cmd)  lcm_send_cmd(cmd)
#define lcm_send_data(data) lcm_send_data(data)
#else
#define UDELAY(n)  usleep_range((n),(n))
#define MDELAY(n)  msleep(n)
//#define lcm_send_cmd(cmd)  lcm_util.send_cmd(cmd)
//#define lcm_send_data(data) lcm_util.send_data(data)
#endif

#define LCM_DSI_CMD_MODE 0
extern int mt_set_gpio_out_base(unsigned long pin, unsigned long output);
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
	params->physical_width = FRAME_WIDTH;
	params->physical_height = FRAME_HEIGHT;
	params->lcm_if   = LCM_INTERFACE_DSI0;
	
	//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    	params->dsi.mode   = CMD_MODE;
#else
    	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_ONE_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB565;

	
	params->dsi.packet_size=256;
	params->dsi.intermediat_buffer_num = 2; 
	params->dsi.edp_panel=1;
	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_16BIT_RGB565;

    params->dsi.word_count = FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line = FRAME_HEIGHT; //480;

    params->dsi.vertical_sync_active				= 8;
    params->dsi.vertical_backporch					= 12;
    params->dsi.vertical_frontporch					= 10;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active			= 10;
    params->dsi.horizontal_backporch				= 51;
    params->dsi.horizontal_frontporch				= 40;
    params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

	// Bit rate calculation
	params->dsi.PLL_CLOCK				= 60;
}


#ifdef BUILD_LK
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	if (GPIO == 0xFFFFFFFF) {
		SPI_LCD_DEBUG("[LK/LCM] GPIO =   0x%x\n", GPIO);
		SPI_LCD_DEBUG("[LK/LCM] output =   0x%x\n", output);
		return;
	}
	mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
static int lcm_get_gpio_output( int gpio_number)
{

			mt_set_gpio_mode(gpio_number| 0x80000000, 0);
			mt_set_gpio_dir(gpio_number| 0x80000000,0);
			return	mt_get_gpio_in(gpio_number| 0x80000000);
}
unsigned short last_lcm_data[FRAME_HEIGHT*FRAME_WIDTH];
unsigned int LCD_LCM_RESET_PIN =(GPIO83 | 0x80000000);
unsigned int LCD_SPI_CS_PIN = (GPIO31 | 0x80000000);
unsigned int LCD_SPI_CLK_PIN = (GPIO32 | 0x80000000);
unsigned int LCD_SPI_OUT_DATA_PIN = (GPIO34 | 0x80000000);
unsigned int LCD_SPI_IN_DATA_PIN = (GPIO33 | 0x80000000);
#define SERIAL_DATA_SET_LOW  	do{	lcm_set_gpio_output(LCD_SPI_OUT_DATA_PIN,GPIO_OUT_ZERO);	}while(0)
#define SERIAL_DATA_SET_HIGH  	do{	lcm_set_gpio_output(LCD_SPI_OUT_DATA_PIN,GPIO_OUT_ONE);	}while(0)
#define SERIAL_DATA2_SET_LOW  	do{	lcm_set_gpio_output(LCD_SPI_IN_DATA_PIN,GPIO_OUT_ZERO);	}while(0)
#define SERIAL_DATA2_SET_HIGH  	do{	lcm_set_gpio_output(LCD_SPI_IN_DATA_PIN,GPIO_OUT_ONE);	}while(0)
#define DISABLE_MANAL_CS 	do{	lcm_set_gpio_output(LCD_SPI_CS_PIN,GPIO_OUT_ONE);	}while(0)	
#define ENABLE_MANAL_CS 	do{	lcm_set_gpio_output(LCD_SPI_CS_PIN,GPIO_OUT_ZERO);	}while(0)
#define SERIAL_RESET_LOW  	do{	lcm_set_gpio_output(LCD_LCM_RESET_PIN,GPIO_OUT_ZERO);}while(0)
#define SERIAL_RESET_HIGH  	do{	lcm_set_gpio_output(LCD_LCM_RESET_PIN,GPIO_OUT_ONE);}while(0)
#define SERIAL_CLK_SET_LOW  do{	lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ZERO);	}while(0)

#define SERIAL_CLK_CNT(cnt)   do{\
	int c; \
	for( c = 0; c < cnt; c++){\
		lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ZERO); \
		lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ONE);\
		}\
	} while(0)

static void lcm_send_cmd(unsigned char byte)
{
    int i;
    SERIAL_DATA2_SET_LOW;
    for( i = 7; i >= 0; i--)
    {
        lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ZERO);
        lcm_set_gpio_output(LCD_SPI_OUT_DATA_PIN,(byte & (1<<i)) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
	    lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ONE);
    }
}

static void lcm_send_data(unsigned char byte)
{
    int i;
    SERIAL_DATA2_SET_HIGH;
    for( i = 7; i >= 0; i--)
    {
        lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ZERO);
        lcm_set_gpio_output(LCD_SPI_OUT_DATA_PIN,(byte & (1<<i)) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
	    lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ONE);
    }
}

static void lcm_send_data_16bit(unsigned short data)
{
    int i;
    SERIAL_DATA2_SET_HIGH;
    for( i = 15; i >= 0; i--)
    {
        lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ZERO);
        lcm_set_gpio_output(LCD_SPI_OUT_DATA_PIN,(data & (1<<i)) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
	    lcm_set_gpio_output(LCD_SPI_CLK_PIN,GPIO_OUT_ONE);
    }    
}

static void lcm_send_data_array(unsigned char * data, int len)
{
     int i;
    SERIAL_DATA2_SET_HIGH;
    {
        for(i = 0; i < len; i++ )
        {
            lcm_send_data(data[i]);
        }
    }      

    return ;
}
static void gpio_init(void)
{
    mt_set_gpio_mode(LCD_SPI_CS_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(LCD_SPI_CS_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(LCD_SPI_CS_PIN, GPIO_OUT_ONE);
    mt_set_gpio_out(LCD_SPI_CS_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(LCD_SPI_CLK_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(LCD_SPI_CLK_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(LCD_SPI_CLK_PIN, GPIO_OUT_ONE);

    mt_set_gpio_mode(LCD_SPI_OUT_DATA_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(LCD_SPI_OUT_DATA_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(LCD_SPI_OUT_DATA_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(LCD_SPI_IN_DATA_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(LCD_SPI_IN_DATA_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(LCD_SPI_IN_DATA_PIN, GPIO_OUT_ONE);

    mt_set_gpio_mode(LCD_LCM_RESET_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(LCD_LCM_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(LCD_LCM_RESET_PIN,GPIO_OUT_ONE);
}
static void reset_gpio_mode(void)
{
//	return;
    mt_set_gpio_mode(LCD_SPI_CS_PIN,1);
    mt_set_gpio_mode(LCD_SPI_CLK_PIN, 1);
    mt_set_gpio_mode(LCD_SPI_OUT_DATA_PIN, 1);
    //mt_set_gpio_mode(LCD_SPI_IN_DATA_PIN, 1);
}
void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend)
{
//	return;
	lcm_send_cmd(0x2a);
	lcm_send_data((Xstart&0xff00)>>8);//start x
	lcm_send_data(Xstart&0x00ff);

	lcm_send_data((Xend&0xff00)>>8);//end x
	lcm_send_data(Xend&0x00ff);

	lcm_send_cmd(0x2b);
	lcm_send_data((Ystart&0xff00)>>8);//start x
	lcm_send_data(Ystart&0x00ff);

	lcm_send_data((Yend&0xff00)>>8);//end x
	lcm_send_data(Yend&0x00ff);
	lcm_send_cmd(0x2c);//transfor begin
}

void Blockclean(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend,unsigned int date)
{
	int i;
//	return;
	lcm_send_cmd(0x2a);
	lcm_send_data((Xstart&0xff00)>>8);//start x
	lcm_send_data(Xstart&0x00ff);

	lcm_send_data((Xend&0xff00)>>8);//end x
	lcm_send_data(Xend&0x00ff);

	lcm_send_cmd(0x2b);
	lcm_send_data((Ystart&0xff00)>>8);//start x
	lcm_send_data(Ystart&0x00ff);

	lcm_send_data((Yend&0xff00)>>8);//end x
	lcm_send_data(Yend&0x00ff);
	lcm_send_cmd(0x2c);//transfor begin
	
		for(i=0;i<FRAME_HEIGHT*FRAME_WIDTH;i++)
		{
           //
			lcm_send_data_16bit(date);
		}
}

void test_show_lcd(void)
{
	int i=0;
	//return;
	 BlockWrite(0,FRAME_WIDTH-1,0,FRAME_HEIGHT-1);
		//lcm_send_cmd(0x2c);//transfor begin
		for(i=0;i<FRAME_HEIGHT*FRAME_WIDTH;i++)
		{
            //last_lcm_data[i] = data[i];
			lcm_send_data_16bit(0xf800);
		}
	}
void spi_show_lk_center(unsigned short *data)
{
    int i=0;
	unsigned int waste_time=0;
	waste_time = get_timer(0);
    SPI_LCD_DEBUG("spi_show_lk_center begin\n");
    for(i=0;i<FRAME_HEIGHT*FRAME_WIDTH;i++)
    {
        if (data[i] != 0)
			break;
    }
    if (i != FRAME_HEIGHT*FRAME_WIDTH)
    {
        if (memcmp(last_lcm_data, data, sizeof(last_lcm_data)) == 0)
        {
            SPI_LCD_DEBUG("spi_show_lk_center same framebuffer return\n");
            return;
        }
		lcm_send_cmd(0x2c);//transfor begin
		for(i=0;i<FRAME_HEIGHT*FRAME_WIDTH;i++)
		{
            last_lcm_data[i] = data[i];
			lcm_send_data_16bit(data[i]);
		}
	}
    SPI_LCD_DEBUG("spi_show_lk_center end in %d ms\n",(int)get_timer(waste_time));    
    reset_gpio_mode();
}
#else
void Blockclean(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend,unsigned int date)
{
	int i;
//	return;
	lcm_send_cmd(0x2a);
	lcm_send_data((Xstart&0xff00)>>8);//start x
	lcm_send_data(Xstart&0x00ff);

	lcm_send_data((Xend&0xff00)>>8);//end x
	lcm_send_data(Xend&0x00ff);

	lcm_send_cmd(0x2b);
	lcm_send_data((Ystart&0xff00)>>8);//start x
	lcm_send_data(Ystart&0x00ff);

	lcm_send_data((Yend&0xff00)>>8);//end x
	lcm_send_data(Yend&0x00ff);
	lcm_send_cmd(0x2c);//transfor begin
	
		for(i=0;i<FRAME_HEIGHT*FRAME_WIDTH;i++)
		{
			lcm_send_data((date&0xff00)>>8);//end x
			lcm_send_data(date&0x00ff);
		}
}

#endif

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int Xend=width-1;
	unsigned int Yend=height-1;

	lcm_send_cmd(0x2a);
	lcm_send_data((x&0xff00)>>8);//start x
	lcm_send_data(x&0x00ff);

	lcm_send_data((Xend&0xff00)>>8);//end x
	lcm_send_data(Xend&0x00ff);

	lcm_send_cmd(0x2b);
	lcm_send_data((y&0xff00)>>8);//start x
	lcm_send_data(y&0x00ff);

	lcm_send_data((Yend&0xff00)>>8);//end x
	lcm_send_data(Yend&0x00ff);
	lcm_send_cmd(0x2c);//transfor begin
}
#endif


#ifndef  BUILD_LK
extern void lcm_set_gpio_output(int gpio, unsigned int output);
#endif
static void lcm_soft_reset(void)
{
	#ifdef  BUILD_LK
    SERIAL_RESET_HIGH;
    MDELAY(1);
    SERIAL_RESET_LOW;
    MDELAY(50);
    SERIAL_RESET_HIGH;
    MDELAY(120);
    #else
   /* spi_lcd_control_gpio(spi_gpio83_rst_high);
	mdelay(2);
	spi_lcd_control_gpio(spi_gpio83_rst_low);
	mdelay(50);
	spi_lcd_control_gpio(spi_gpio83_rst_high);
	mdelay(100);*/
	lcm_set_gpio_output(83,1);
	MDELAY(5);
	lcm_set_gpio_output(83,0);
	MDELAY(50);
	lcm_set_gpio_output(83,1);
	MDELAY(20);
    #endif
}
static void st7789_lcm_init_00(void)//HSD
{
 

lcm_send_cmd(0xE2);
MDELAY(5);
lcm_send_cmd(0x2c);
MDELAY(5);
lcm_send_cmd(0x2e);
MDELAY(5);
lcm_send_cmd(0x2F);
MDELAY(5);
lcm_send_cmd(0x24);
lcm_send_cmd(0xA2);
lcm_send_cmd(0xC8);
lcm_send_cmd(0xA0);
lcm_send_cmd(0x40);
lcm_send_cmd(0xAF);
}

/****************************************************/
//测试
void test(void)
{
 int page,i;

 for(page=0;page<9;page++)
  {
lcm_send_cmd(0x10);
lcm_send_cmd(0x00);
   lcm_send_cmd(page+0xb0);
    for(i=0;i<64;i++)
    {lcm_send_data(0x00);lcm_send_data(0xFF);}
  }
}
//测试斜线
  void Displine(int ph,int pl,int pw,int pd)
{
	   int i,j;

		   for(i=0;i<9;i++)
		   {

			    lcm_send_cmd(0x10);
			    lcm_send_cmd(0x00);
                            lcm_send_cmd(i+0xb0);

		       for(j=0;j<128;j++)
				    {
				     lcm_send_data(ph);
                                     lcm_send_data(pl);
                                     lcm_send_data(pw);
                                     lcm_send_data(pd);
			         }
           }
}
/****************************************************/
//图片写入
void graphic(char pic[])
{
 int i,page;

 for(page=0;page<2;page++)
 {
lcm_send_cmd(0x10);
lcm_send_cmd(0x00);
  lcm_send_cmd(page+0xb0);
   for(i=0;i<16 ;i++)
    {lcm_send_data(*pic);pic++;}
 }
}
/****************************************************/
//图片存放
 unsigned char  BMP[]={/*--  调入了一幅图像：D:\My Documents\My Pictures\128 X64 (1).bmp  --*/
/*--  宽度x高度=128x32  --*/
0x08,0x30,0x00,0xFF,0x20,0x20,0x20,0x20,0xFF,0x20,0xE1,0x26,0x2C,0x20,0x20,0x00,
0x04,0x02,0x01,0xFF,0x40,0x20,0x18,0x07,0x00,0x00,0x03,0x0C,0x30,0x60,0x20,0x00
};
static unsigned int lcm_compare_id(void)
{
    return 1;		 
}
#ifndef  BUILD_LK
extern int mtk_gpio_base_spi(void);
int lcm_get_gpio_value(int gpio)
{
	gpio += mtk_gpio_base_spi();
	if (gpio_is_valid(gpio)) {
		return gpio_get_value(gpio);
	}
	return -1;
}
#endif
void lcm_init(void)
{ 
	SPI_LCD_DEBUG("ST7789v lcm_init %s \n", __func__);    
#if 1
    lcm_soft_reset();
   st7789_lcm_init_00();
	SPI_LCD_DEBUG("----------yoyo--show test  logo-----------\n");
//#ifndef BUILD_LK 
 //Blockclean(0,FRAME_WIDTH-1,0,FRAME_HEIGHT-1,0x0000);
//#endif
#endif
}

static void lcm_suspend(void)
{

	SPI_LCD_DEBUG("ST7789v enter suspend %s \n", __func__);
	lcm_send_cmd(0x28);
	MDELAY(20);
	lcm_send_cmd(0x10);
        MDELAY(100);
		
}
extern void lcm_reset(void);
extern void spi_cleanblack(void);

static void lcm_resume(void)
{
		lcm_send_cmd(0x29);
	MDELAY(20);
	lcm_send_cmd(0x11);
    SPI_LCD_DEBUG("ST7789v exit suspend %s \n", __func__);
	lcm_init();

	
	
MDELAY(100);
lcm_send_cmd(0x81);
lcm_send_cmd(0x2f);
//graphic(BMP);
//test();
//	//Displine(0xc0,0x30,0x0c,0x03);
}

unsigned int st7796s_lcm_ata_check(unsigned char *buffer)
{
	return 1;	
}

LCM_DRIVER st7796s_hvga_dsi_cmd_drv = 
{
    .name			= "st7796s_hvga_dsi_cmd",
	.set_util_funcs = lcm_set_util_funcs,
	.compare_id     = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.ata_check	= st7796s_lcm_ata_check,
#if (LCM_DSI_CMD_MODE)
	 .update         = lcm_update,
#endif	
};

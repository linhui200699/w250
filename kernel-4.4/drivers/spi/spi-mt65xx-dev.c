/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/spi/spi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_data/spi-mt65xx.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#ifdef CONFIG_TRUSTONIC_TEE_SUPPORT
#define SPI_TRUSTONIC_TEE_SUPPORT
#endif

#ifdef SPI_TRUSTONIC_TEE_SUPPORT
#include <mobicore_driver_api.h>
#include <tlspi_Api.h>
#endif
#ifdef CONFIG_SPI_LCM_SUPPORT//by yoyo

#include "primary_display.h"
#include "DpDataType.h"
#include "disp_drv_platform.h"
#include "mtkfb.h"

#define DEBUG_SPI_LCM
#define   DELAY_MS_TIME         0
void *g_color_temp = NULL;
#define RGB888TO565(r,g,b)  ( ((((u16)(r)) &0x00F8)>>0) \
                            | ((((u16)(g)) &0x00E0)>>5) \
                            | ((((u16)(g)) &0x001C)<<11)\
                            | ((((u16)(b)) &0x00f8)<<5)  )
#define LCD_PHY_WIDTH  (128)
#define LCD_PHY_HEIGHT  (64)
#define MAX_PACK_SIZE  (0x400)
#define MAX_PACK_COUNT  (0x100)
#define LCM_DATA_8BIT_SIZE  (LCD_PHY_WIDTH *LCD_PHY_HEIGHT *2) 
u32 display_buffer[LCM_DATA_8BIT_SIZE/4]= {0};
u8  oled_display_buffer[LCD_PHY_WIDTH*LCD_PHY_HEIGHT]= {0};
u8  oled_display_buffer_tmp[LCD_PHY_WIDTH*LCD_PHY_HEIGHT]= {0};
#define LCM_DATA_9BIT_SIZE ((LCM_DATA_8BIT_SIZE  / 8 )* 9)
#define LCM_CMD_9BIT_SIZE  (9 * 4) //pading to dword
#define LCM_TOTAL_9BIT_SIZE (LCM_DATA_9BIT_SIZE + LCM_CMD_9BIT_SIZE )
//round to package size   
//#define LCM_TOTAL_8BIT_BUFF_SIZE (((LCM_TOTAL_9BIT_SIZE + MAX_PACK_SIZE -1)/ MAX_PACK_SIZE) * MAX_PACK_SIZE)
#define LCM_TOTAL_9BIT_BUFF_SIZE  (((LCM_TOTAL_9BIT_SIZE + MAX_PACK_SIZE -1)>>10 )<< 10)
u32 spi_buffer[LCM_TOTAL_9BIT_BUFF_SIZE/4]= {0};
#define rIndex 1
#define gIndex 2
#define bIndex 3
#define CMD_MODE 1
#define  DATA_MODE 0
#define CAPTURE_MODE_CUSTOM  2
#define CAPTURE_MODE_OVL  1
#define CAPTURE_MODE_FB  0


struct lcm_screen_info {
    int inited;
    int flush;
    int capture;
    int capture_state;
    int spi_mode;
    unsigned long update;
    unsigned long refresh;
};
struct lcm_screen_info lcm_scr_info = {0};
#define  g_start_flush  (lcm_scr_info.flush)
#define  g_start_capture  (lcm_scr_info.capture)
u8 lcd_backlight = 0;
#define  increase_update()  lcm_scr_info.update++
#define  increase_refresh()  lcm_scr_info.refresh++
#define  set_capture_state(b)  lcm_scr_info.capture_state = (b)
#define  get_capture_state()  lcm_scr_info.capture_state 
static struct task_struct *flushthread = NULL;
static struct task_struct *capturethread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(flush_waiter);
static DECLARE_WAIT_QUEUE_HEAD(capture_waiter);
DEFINE_SPINLOCK(capture_lock);
unsigned long primary_display_get_frame_buffer_va_address(void);
void spi_send_data(u16 x,u16 y,u16 w,u16 h,void *buff);
#endif

#define SPIDEV_LOG(fmt, args...) pr_err("[SPI-UT]: [%s]:[%d]" fmt, __func__, __LINE__, ##args)
#define SPIDEV_MSG(fmt, args...) pr_err(fmt, ##args)

static struct spi_device *spi_test;

struct mtk_spi {
	void __iomem *base;
	void __iomem *peri_regs;
	u32 state;
	int pad_num;
	u32 *pad_sel;
	struct clk *parent_clk, *sel_clk, *spi_clk;
	struct spi_transfer *cur_transfer;
	u32 xfer_len;
	struct scatterlist *tx_sgl, *rx_sgl;
	u32 tx_sgl_len, rx_sgl_len;
	const struct mtk_spi_compatible *dev_comp;
	u32 dram_8gb_offset;
};



#ifdef SPI_TRUSTONIC_TEE_SUPPORT
#define DEFAULT_HANDLES_NUM (64)
#define MAX_OPEN_SESSIONS (0xffffffff - 1)
/*
* Trustlet UUID.
*/
u8 spi_uuid[10][16] = {{0x09, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x09, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };

static struct mc_session_handle secspi_session = { 0 };

static u32 secspi_session_ref;
static u32 secspi_devid = MC_DEVICE_ID_DEFAULT;
static tciSpiMessage_t *secspi_tci;

static DEFINE_MUTEX(secspi_lock);

int secspi_session_open(u32 spinum)
{
	enum mc_result mc_ret = MC_DRV_OK;

	mutex_lock(&secspi_lock);


	pr_warn("secspi_session_open start\n");
	do {
		/* sessions reach max numbers ? */

		if (secspi_session_ref > MAX_OPEN_SESSIONS) {
			pr_warn("secspi_session > 0x%x\n", MAX_OPEN_SESSIONS);
			break;
		}

		if (secspi_session_ref > 0) {
			secspi_session_ref++;
			break;
		}

		/* open device */
		mc_ret = mc_open_device(secspi_devid);
		if (mc_ret != MC_DRV_OK) {
			pr_warn("mc_open_device failed: %d\n", mc_ret);
			break;
		}

		/* allocating WSM for DCI */
		mc_ret =
		    mc_malloc_wsm(secspi_devid, 0, sizeof(tciSpiMessage_t),
				  (uint8_t **) &secspi_tci, 0);
		if (mc_ret != MC_DRV_OK) {
			pr_warn("mc_malloc_wsm failed: %d\n", mc_ret);
			mc_close_device(secspi_devid);
			break;
		}

		/* open session */
		secspi_session.device_id = secspi_devid;
		/* mc_ret =
		 *   mc_open_session(&secspi_session, &secspi_uuid, (uint8_t *) secspi_tci,
		 *		    sizeof(tciSpiMessage_t));
		 */
		mc_ret = mc_open_session(&secspi_session, (struct mc_uuid_t *)&spi_uuid[spinum][0],
				(uint8_t *)secspi_tci, sizeof(tciSpiMessage_t));

		if (mc_ret != MC_DRV_OK) {
			pr_warn("secspi_session_open fail: %d\n", mc_ret);
			mc_free_wsm(secspi_devid, (uint8_t *) secspi_tci);
			mc_close_device(secspi_devid);
			secspi_tci = NULL;
			break;
		}
		secspi_session_ref = 1;

	} while (0);

	pr_warn("secspi_session_open: ret=%d, ref=%d\n", mc_ret, secspi_session_ref);

	mutex_unlock(&secspi_lock);
	pr_err("secspi_session_open end\n");

	if (mc_ret != MC_DRV_OK)
		return -ENXIO;

	return 0;
}

static int secspi_session_close(void)
{
	enum mc_result mc_ret = MC_DRV_OK;

	mutex_lock(&secspi_lock);

	do {
		/* session is already closed ? */
		if (secspi_session_ref == 0) {
			SPIDEV_MSG("spi_session already closed\n");
			break;
		}

		if (secspi_session_ref > 1) {
			secspi_session_ref--;
			break;
		}

		/* close session */
		mc_ret = mc_close_session(&secspi_session);
		if (mc_ret != MC_DRV_OK) {
			SPIDEV_MSG("SPI mc_close_session failed: %d\n", mc_ret);
			break;
		}

		/* free WSM for DCI */
		mc_ret = mc_free_wsm(secspi_devid, (uint8_t *) secspi_tci);
		if (mc_ret != MC_DRV_OK) {
			SPIDEV_MSG("SPI mc_free_wsm failed: %d\n", mc_ret);
			break;
		}
		secspi_tci = NULL;
		secspi_session_ref = 0;

		/* close device */
		mc_ret = mc_close_device(secspi_devid);
		if (mc_ret != MC_DRV_OK)
			SPIDEV_MSG("SPI mc_close_device failed: %d\n", mc_ret);

	} while (0);

	SPIDEV_MSG("secspi_session_close: ret=%d, ref=%d\n", mc_ret, secspi_session_ref);

	mutex_unlock(&secspi_lock);

	if (mc_ret != MC_DRV_OK)
		return -ENXIO;

	return 0;

}

void secspi_enable_clk(struct spi_device *spidev)
{
	int ret;
	struct spi_master *master;
	struct mtk_spi *ms;

	master = spidev->master;
	ms = spi_master_get_devdata(master);
	/*
	 * prepare the clock source
	 */
	ret = clk_prepare_enable(ms->spi_clk);
}

int secspi_execute(u32 cmd, tciSpiMessage_t *param)
{
	enum mc_result mc_ret;

	pr_warn("secspi_execute\n");
	mutex_lock(&secspi_lock);

	if (secspi_tci == NULL) {
		mutex_unlock(&secspi_lock);
		pr_warn("secspi_tci not exist\n");
		return -ENODEV;
	}

	/*set transfer data para */
	if (param == NULL) {
		pr_warn("secspi_execute parameter is NULL !!\n");
	} else {
		secspi_tci->tx_buf = param->tx_buf;
		secspi_tci->rx_buf = param->rx_buf;
		secspi_tci->len = param->len;
		secspi_tci->is_dma_used = param->is_dma_used;
		secspi_tci->tx_dma = param->tx_dma;
		secspi_tci->rx_dma = param->rx_dma;
		secspi_tci->tl_chip_config = param->tl_chip_config;
	}

	secspi_tci->cmd_spi.header.commandId = (tciCommandId_t) cmd;
	secspi_tci->cmd_spi.len = 0;

	pr_warn("mc_notify\n");

	/* enable_clock(MT_CG_PERI_SPI0, "spi"); */
	/* enable_clk(ms); */

	mc_ret = mc_notify(&secspi_session);

	if (mc_ret != MC_DRV_OK) {
		pr_warn("mc_notify failed: %d", mc_ret);
		goto exit;
	}

	pr_warn("SPI mc_wait_notification\n");
	mc_ret = mc_wait_notification(&secspi_session, -1);

	if (mc_ret != MC_DRV_OK) {
		pr_warn("SPI mc_wait_notification failed: %d", mc_ret);
		goto exit;
	}

exit:

	mutex_unlock(&secspi_lock);

	if (mc_ret != MC_DRV_OK)
		return -ENOSPC;

	return 0;
}

/*used for REE to detach IRQ of TEE*/
void spi_detach_irq_tee(u32 spinum)
{
	secspi_session_open(spinum);
	secspi_execute(2, NULL);
	pr_warn("secspi_execute 2 finished!!!\n");
}
#endif
#ifdef  CONFIG_SPI_LCM_SUPPORT//by yoyo
static int spi_send_cmd(struct spi_device *spi, u8 *tx, u32 spilen)
{
	struct spi_message m;

	struct spi_transfer t = {
		.tx_buf = tx,
		.len = spilen
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(spi, &m);
}

extern int mtk_gpio_base_spi(void);
void lcm_set_gpio_output(int gpio, unsigned int output)
{
	gpio += mtk_gpio_base_spi();
	if (gpio_is_valid(gpio)) {
		gpio_direction_output(gpio, output);
		gpio_set_value(gpio, output);
	}
}
void write_command(struct spi_device *spi, u8 cmd)
{
       spi_send_cmd(spi,(u8 *)&cmd,sizeof(cmd)/sizeof(char));
}


#define write_cmd(cmd)  write_command(spi, cmd)
#define write_dat(dat)  write_command(spi, dat)
static void copy_lk_logo(u16 *layer_buffer)
{
    int bmp_height  = LCD_PHY_HEIGHT;
    int bmp_width = LCD_PHY_WIDTH;
    int scr_width = LCD_PHY_WIDTH; 
    int line,j;
    u16 *dst;
    u8 *p;
    u8 *p_rgb_data = (u8*)primary_display_get_frame_buffer_va_address();
    for(line = 0 ; line < bmp_height ; line++ )
    {
        p = (u8*)( p_rgb_data + line * ALIGN_TO(scr_width,32) *32 /8);

        dst = &layer_buffer[ scr_width * line ];
        for(j = 0; j < bmp_width; j++ )
        {
            dst[j] = RGB888TO565(p[2], p[1], p[0]);//rgb bgr012 102 210
            p += 4;
        }
    }
}
void notify_spi_lklogo(void)
{
	//unsigned long flags;
	u16 *data_buff = (u16*)&display_buffer[0];
	copy_lk_logo(data_buff);
	printk("show-------notify_spi_lklogo-----yoyo-----------------\n");
    set_capture_state(1);
}
void lcm_hook_primary_display_trigger(int blocking)
{
	unsigned long flags;

    spin_lock_irqsave(&capture_lock,flags);
    increase_update();
    //if(!blocking)
    {
        set_capture_state(1);
    }
    printk("show-------lcm_hook_primary_display_trigger-----yoyo-----------------\n");
    spin_unlock_irqrestore(&capture_lock,flags);
    if(blocking )
    {
    }

	wake_up_interruptible(&capture_waiter);
}

static int capture_thread(void *data)
{
	int i,j;
	u8 R ,G,B;
	unsigned long flags;
    #define boot_completed   1
    #define set_boot_completed(b) do{;}while(0)
	struct sched_param param = {.sched_priority = 94 };
	sched_setscheduler(current, SCHED_RR, &param);
	do{
		int need_capture;
		u8 *p = (u8*) g_color_temp;
		//u16 *pOut = (u16*)&display_buffer[0];
		u8 *pOut = &oled_display_buffer[0];
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(capture_waiter, get_capture_state());
		set_current_state(TASK_RUNNING); 
         printk("yoyo -------capture_thread\n");
        spin_lock_irqsave(&capture_lock,flags);
        increase_refresh();

        need_capture = get_capture_state();
        set_capture_state(0);
        spin_unlock_irqrestore(&capture_lock,flags);
#if !defined( __USE_STATIC_BUFFER__)
        if(g_color_temp == NULL ) continue;
#endif
		if(boot_completed == 0)
        {
        }
       else if(need_capture == 1 )
        {
	            primary_display_capture_framebuffer_ovl((unsigned long)g_color_temp, UFMT_ARGB8888);//eARGB8888 eBGR888
		}
	      else if(need_capture == 2 )
        {
        }
        else
        {

        }
		wait_event_interruptible(capture_waiter, g_start_capture !=0);
        if( boot_completed == 0 )
        {

        }
       else for(i=0;i<LCD_PHY_HEIGHT;i++)
	  	{
	          	for(j=0;j<LCD_PHY_WIDTH*4;j+=8)
	          	{

	          		G = p[gIndex];
	          		R = p[rIndex];
	          		B = p[bIndex];
				//	printk("1 R G B is %u,%u,%u \n",R,G,B);
					if (R+G+B > 381)
						*(pOut++) = 0x00;
					else
						*(pOut++) = 0x01;
	          		//*(pOut++) = RGB888TO565(R,G,B);	          		
	          		p += 4; 
	          		G = p[gIndex];
	          		R = p[rIndex];
	          		B = p[bIndex];
					//printk("2 R G B is %u,%u,%u \n",R,G,B);
					//printk("2 oPUT is %u,%u,%u \n",R,G,B);
					if (R+G+B > 381)
						*(pOut++) = 0x00;
					else
						*(pOut++) = 0x01;
	          		//*(pOut++) =RGB888TO565(R,G,B); 
	          		p += 4;
	          	}
	      	}
	  	if( boot_completed == 0 )
	  	{
#ifdef NO_ANDROID_ANIMATION
	      	static int line_len = 10;
	      	if(line_len++ >= 200 ){
	      	    line_len = 10;
	      	}
	  		draw_logo_line(10, 300, line_len, RGB888TO565(255,0,0), &display_buffer[0]);
#else
	      	set_boot_completed(1);
#endif
	  	}
	  	 spin_lock_irqsave(&capture_lock,flags);
	 	 g_start_flush = 1;
		 g_start_capture = 0;
         spin_unlock_irqrestore(&capture_lock,flags);
	 	 wake_up_interruptible(&flush_waiter);
	}while(!kthread_should_stop());
	return 0;
}

void lcm_reset(void){
	lcm_set_gpio_output(83,0);
	mdelay(10);
    lcm_set_gpio_output(83,1); 
	mdelay(20);
	lcm_set_gpio_output(83,0); 
	mdelay(20);
}
void lcm_send_cmd(unsigned char byte)
{
	char cmd[1]={0};
	cmd[0]=byte;
	  lcm_set_gpio_output(33,0);
     spi_send_cmd(spi_test,(u8 *)&cmd,sizeof(cmd)/sizeof(char));
	}
void lcm_send_data(unsigned char byte)
{
	 char cmd[1]={0};
     	cmd[0]=byte;
	 lcm_set_gpio_output(33,1);
     spi_send_cmd(spi_test,(u8 *)&cmd,sizeof(cmd)/sizeof(char));
	}
void spi_cleanblack(void)
{
        u8 *data_buff = (u8*)&display_buffer[0];
        	    if(spi_test==NULL){
		    return;
	    }
         memset(data_buff , 0x00, LCM_DATA_8BIT_SIZE);//for test
         spi_send_cmd(spi_test,(u8 *)data_buff,(LCD_PHY_WIDTH *LCD_PHY_HEIGHT*2));
}
static int flushlcm_thread(void *data)
{
//	struct spi_device *spi=(struct spi_device *) data;
//	int i;
		//u8 *buff = (u8*)&spi_buffer[0];	
	//	u8 *data_buff = (u8*)&display_buffer[0];
		u8 *data_buff =  &oled_display_buffer[0];
	struct sched_param param = {.sched_priority = 94 };
	sched_setscheduler(current, SCHED_RR, &param);
		
	g_start_capture = 1;
	do{
		unsigned long flags;	
		
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(flush_waiter, g_start_flush !=0);
		
		
		set_current_state(TASK_RUNNING); 
	    spi_send_data(0, 0,0, 0,data_buff);
	  	 spin_lock_irqsave(&capture_lock,flags);
		 g_start_flush = 0;	 
		 g_start_capture = 1;
         spin_unlock_irqrestore(&capture_lock,flags);
       
		 wake_up_interruptible(&capture_waiter);
	}while(!kthread_should_stop());

	return 0;
}
void addset(struct spi_device *spi,unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend)
{
	u8 id_data_temp[2];
	u8 id_cmd_2a[1]={0x2a};
	u8 id_cmd_2b[1]={0x2b};
	
	lcm_set_gpio_output(33,0);
	spi_send_cmd(spi,id_cmd_2a,sizeof(id_cmd_2a)/sizeof(char));
	id_data_temp[0] = (Xstart&0xFF00)>>8;
	id_data_temp[1] = (Xstart&0x00FF);
	lcm_set_gpio_output(33,1);
	spi_send_cmd(spi,id_data_temp,sizeof(id_data_temp)/sizeof(char));
	
	id_data_temp[0] = (Xend&0xFF00)>>8;
	id_data_temp[1] = (Xend&0x00FF);
	lcm_set_gpio_output(33,1);
	spi_send_cmd(spi,id_data_temp,sizeof(id_data_temp)/sizeof(char));
	
	lcm_set_gpio_output(33,0);
	spi_send_cmd(spi,id_cmd_2b,sizeof(id_cmd_2b)/sizeof(char));
	
	id_data_temp[0] = (Ystart&0xFF00)>>8;
	id_data_temp[1] = (Ystart&0x00FF);
	lcm_set_gpio_output(33,1);
	spi_send_cmd(spi,id_data_temp,sizeof(id_data_temp)/sizeof(char));
	
	id_data_temp[0] = (Yend&0xFF00)>>8;
	id_data_temp[1] = (Yend&0x00FF);
	lcm_set_gpio_output(33,1);
	spi_send_cmd(spi,id_data_temp,sizeof(id_data_temp)/sizeof(char));
}
#if 0
void spi_send_data(u16 x,u16 y,u16 w,u16 h,void *buff)
{
	  lcm_set_gpio_output(33,0);
    write_command(spi_test, 0x2c);//transfor begin
   // printk("show_fb width=%d,height=%d",width,height);
	lcm_set_gpio_output(33,1);
	spi_send_cmd(spi_test,(u8 *)buff,(LCD_PHY_WIDTH*LCD_PHY_HEIGHT*2));
}
#endif 
void spi_send_data(u16 x,u16 y,u16 w,u16 h,void *buff)
{
 int i,page;
 u8 *pic =(u8*)buff;
 u8 *picture=&oled_display_buffer_tmp[0];
	for(i=0;i<128;i++)
	{
		#if 0
		*(picture+i)=(*(pic+i+128*0)<<7)|(*(pic+i+128*1)<<6)|(*(pic+i+128*2)<<5)|(*(pic+i+128*3)<<4)|(*(pic+i+128*4)<<3)|(*(pic+i+128*5)<<2)|(*(pic+i+128*6)<<1)|(*(pic+i+128*7));
		*(picture+i+128)=(*(pic+i+128*8)<<7)|(*(pic+i+128*9)<<6)|(*(pic+i+128*10)<<5)|(*(pic+i+128*11)<<4)|(*(pic+i+128*12)<<3)|(*(pic+i+128*13)<<2)|(*(pic+i+128*14)<<1)|(*(pic+i+128*15));
		*(picture+i+128*2)=(*(pic+i+128*16)<<7)|(*(pic+i+128*17)<<6)|(*(pic+i+128*18)<<5)|(*(pic+i+128*19)<<4)|(*(pic+i+128*20)<<3)|(*(pic+i+128*21)<<2)|(*(pic+i+128*22)<<1)|(*(pic+i+128*23));
		*(picture+i+128*3)=(*(pic+i+128*24)<<7)|(*(pic+i+128*25)<<6)|(*(pic+i+128*26)<<5)|(*(pic+i+128*27)<<4)|(*(pic+i+128*28)<<3)|(*(pic+i+128*29)<<2)|(*(pic+i+128*30)<<1)|(*(pic+i+128*31));
		*(picture+i+128*4)=(*(pic+i+128*32)<<7)|(*(pic+i+128*33)<<6)|(*(pic+i+128*34)<<5)|(*(pic+i+128*35)<<4)|(*(pic+i+128*36)<<3)|(*(pic+i+128*37)<<2)|(*(pic+i+128*38)<<1)|(*(pic+i+128*39));
		*(picture+i+128*5)=(*(pic+i+128*40)<<7)|(*(pic+i+128*41)<<6)|(*(pic+i+128*42)<<5)|(*(pic+i+128*43)<<4)|(*(pic+i+128*44)<<3)|(*(pic+i+128*45)<<2)|(*(pic+i+128*46)<<1)|(*(pic+i+128*47));
		*(picture+i+128*6)=(*(pic+i+128*48)<<7)|(*(pic+i+128*49)<<6)|(*(pic+i+128*50)<<5)|(*(pic+i+128*51)<<4)|(*(pic+i+128*52)<<3)|(*(pic+i+128*53)<<2)|(*(pic+i+128*54)<<1)|(*(pic+i+128*55));
		*(picture+i+128*7)=(*(pic+i+128*56)<<7)|(*(pic+i+128*57)<<6)|(*(pic+i+128*58)<<5)|(*(pic+i+128*59)<<4)|(*(pic+i+128*60)<<3)|(*(pic+i+128*61)<<2)|(*(pic+i+128*62)<<1)|(*(pic+i+128*63));
		#endif 
		*(picture+i)=(*(pic+i+128*0))|(*(pic+i+128*1)<<1)|(*(pic+i+128*2)<<2)|(*(pic+i+128*3)<<3)|(*(pic+i+128*4)<<4)|(*(pic+i+128*5)<<5)|(*(pic+i+128*6)<<6)|(*(pic+i+128*7)<<7);
		*(picture+i+128)=(*(pic+i+128*8))|(*(pic+i+128*9)<<1)|(*(pic+i+128*10)<<2)|(*(pic+i+128*11)<<3)|(*(pic+i+128*12)<<4)|(*(pic+i+128*13)<<5)|(*(pic+i+128*14)<<6)|(*(pic+i+128*15)<<7);
		*(picture+i+128*2)=(*(pic+i+128*16))|(*(pic+i+128*17)<<1)|(*(pic+i+128*18)<<2)|(*(pic+i+128*19)<<3)|(*(pic+i+128*20)<<4)|(*(pic+i+128*21)<<5)|(*(pic+i+128*22)<<6)|(*(pic+i+128*23)<<7);
		*(picture+i+128*3)=(*(pic+i+128*24))|(*(pic+i+128*25)<<1)|(*(pic+i+128*26)<<2)|(*(pic+i+128*27)<<3)|(*(pic+i+128*28)<<4)|(*(pic+i+128*29)<<5)|(*(pic+i+128*30)<<6)|(*(pic+i+128*31)<<7);
		*(picture+i+128*4)=(*(pic+i+128*32))|(*(pic+i+128*33)<<1)|(*(pic+i+128*34)<<2)|(*(pic+i+128*35)<<3)|(*(pic+i+128*36)<<4)|(*(pic+i+128*37)<<5)|(*(pic+i+128*38)<<6)|(*(pic+i+128*39)<<7);
		*(picture+i+128*5)=(*(pic+i+128*40))|(*(pic+i+128*41)<<1)|(*(pic+i+128*42)<<2)|(*(pic+i+128*43)<<3)|(*(pic+i+128*44)<<4)|(*(pic+i+128*45)<<5)|(*(pic+i+128*46)<<6)|(*(pic+i+128*47)<<7);
		*(picture+i+128*6)=(*(pic+i+128*48))|(*(pic+i+128*49)<<1)|(*(pic+i+128*50)<<2)|(*(pic+i+128*51)<<3)|(*(pic+i+128*52)<<4)|(*(pic+i+128*53)<<5)|(*(pic+i+128*54)<<6)|(*(pic+i+128*55)<<7);
		*(picture+i+128*7)=(*(pic+i+128*56))|(*(pic+i+128*57)<<1)|(*(pic+i+128*58)<<2)|(*(pic+i+128*59)<<3)|(*(pic+i+128*60)<<4)|(*(pic+i+128*61)<<5)|(*(pic+i+128*62)<<6)|(*(pic+i+128*63)<<7);		
	} 
		
	 for(page=0;page<8;page++)
	 {
		lcm_send_cmd(0x10);
		lcm_send_cmd(0x00);
		lcm_send_cmd(page+0xb0);
	
		for(i=0;i<128 ;i++)
		{
			lcm_send_data(*picture);
		//lcm_send_data(0xff);
		//lcm_send_data(0x00);
		   picture++;
		}
 }
}
#if 0
void lcm_send_data(unsigned char byte)
{
	 char cmd[1]={0};
     	cmd[0]=byte;
	 lcm_set_gpio_output(33,1);
     spi_send_cmd(spi_test,(u8 *)&cmd,sizeof(cmd)/sizeof(char));
	}
#endif 
#if 0
void lcm_send_cmd(unsigned char byte)
{
	//int i_data = byte;
	  lcm_set_gpio_output(33,0);
    write_command(spi_test, byte);//transfor begin
	}
void lcm_send_data(unsigned char byte)
{
	//int i_data = byte;
	  lcm_set_gpio_output(33,1);
    write_command(spi_test, byte);//transfor begin
	}
void spi_cleanblack(void){
        u8 *data_buff = (u8*)&display_buffer[0];
        	    if(spi_test==NULL){
		    return;
	    }
         memset(data_buff , 0x00, LCM_DATA_8BIT_SIZE);//for test
         spi_send_cmd(spi_test,(u8 *)data_buff,(LCD_PHY_WIDTH*LCD_PHY_HEIGHT*2));
}
void lcm_reset(void){
	/*
	spi_lcd_control_gpio(spi_gpio83_rst_high);
	mdelay(2);
	spi_lcd_control_gpio(spi_gpio83_rst_low);
	mdelay(50);
	spi_lcd_control_gpio(spi_gpio83_rst_high);
	mdelay(100);*/
}
#endif
#endif
void mt_spi_disable_master_clk(struct spi_device *spidev)
{
	struct mtk_spi *ms;

	ms = spi_master_get_devdata(spidev->master);
	/*
	 * unprepare the clock source
	 */
	clk_disable_unprepare(ms->spi_clk);
}
EXPORT_SYMBOL(mt_spi_disable_master_clk);

void mt_spi_enable_master_clk(struct spi_device *spidev)
{
	int ret;
	struct mtk_spi *ms;

	ms = spi_master_get_devdata(spidev->master);
	/*
	 * prepare the clock source
	 */
	ret = clk_prepare_enable(ms->spi_clk);
}
EXPORT_SYMBOL(mt_spi_enable_master_clk);

static int spi_setup_xfer(struct device *dev, struct spi_transfer *xfer, u32 len, u32 flag)
{
	u32 tx_buffer = 0x12345678;
	u32 cnt, i;

	xfer->len = len;

	xfer->tx_buf = kzalloc(len, GFP_KERNEL);
	xfer->rx_buf = kzalloc(len, GFP_KERNEL);
	xfer->speed_hz = 500000;
	xfer->len = len;

	/* Instead of using kzalloc, if using below
	 * dma_alloc_coherent to allocate tx_dma/rx_dma, remember:
	 * 1. remove kfree in spi_recv_check_all, otherwise KE reboot
	 * 2. set msg.is_dma_mapped = 1 before calling spi_sync();
	 */

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	SPIDEV_LOG("Transfer addr:Tx:0x%llx, Rx:0x%llx\n", xfer->tx_dma, xfer->rx_dma);
#else
	SPIDEV_LOG("Transfer addr:Tx:0x%x, Rx:0x%x\n", xfer->tx_dma, xfer->rx_dma);
#endif

	if ((xfer->tx_buf == NULL) || (xfer->rx_buf == NULL))
		return -1;

	cnt = (len%4)?(len/4 + 1):(len/4);

	if (flag == 0) {
		for (i = 0; i < cnt; i++)
			*((u32 *)xfer->tx_buf + i) = tx_buffer;
	} else if (flag == 1) {
		for (i = 0; i < cnt; i++)
			*((u32 *)xfer->tx_buf + i) = tx_buffer + i;
	} else if (flag == 2) {

	} else {
		return -EINVAL;
	}
	return 0;

}

static int spi_recv_check(struct spi_message *msg)
{

	struct spi_transfer *xfer;
	u32 cnt, i, err = 0;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {

		if (!xfer) {
			SPIDEV_MSG("rv msg is NULL.\n");
			return -1;
		}
		cnt = (xfer->len%4)?(xfer->len/4 + 1):(xfer->len/4);
		for (i = 0; i < cnt; i++) {
			if (*((u32 *) xfer->rx_buf + i) != *((u32 *) xfer->tx_buf + i)) {
				SPIDEV_LOG("tx xfer %d is:%.8x\n", i, *((u32 *) xfer->tx_buf + i));
				SPIDEV_LOG("rx xfer %d is:%.8x\n", i, *((u32 *) xfer->rx_buf + i));
				SPIDEV_LOG("\n");
				err++;
			}
		}
		kfree(xfer->tx_buf);
		kfree(xfer->rx_buf);
	}

		SPIDEV_LOG("Message:0x%p,error %d,actual xfer length is:%d\n", msg, err, msg->actual_length);

	return err;
}

static ssize_t spi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct spi_device *spi;

	struct mtk_chip_config *chip_config;

#ifdef CONFIG_TRUSTONIC_TEE_SUPPORT
	u32 spinum;
#endif

	spi = container_of(dev, struct spi_device, dev);

	SPIDEV_LOG("SPIDEV name is:%s\n", spi->modalias);

	chip_config = (struct mtk_chip_config *) spi->controller_data;

	if (!chip_config) {
		SPIDEV_LOG("chip_config is NULL.\n");
		chip_config = kzalloc(sizeof(struct mtk_chip_config), GFP_KERNEL);
		if (!chip_config)
			return -ENOMEM;
	}
	if (!buf) {
		SPIDEV_LOG("buf is NULL.\n");
		goto out;
	}
#ifdef CONFIG_TRUSTONIC_TEE_SUPPORT
	if (!strncmp(buf, "send", 4) && (sscanf(buf + 4, "%d", &spinum) == 1)) { /*TRANSFER*/
		SPIDEV_MSG("start to access TL SPI driver.\n");
		secspi_session_open(spinum);
		secspi_enable_clk(spi);
		secspi_execute(1, NULL);
		secspi_session_close();
		SPIDEV_MSG("secspi_execute 1 finished!!!\n");
	} else if (!strncmp(buf, "config", 6) && (sscanf(buf + 6, "%d", &spinum) == 1)) { /*HW CONFIG*/
		SPIDEV_MSG("start to access TL SPI driver.\n");
		secspi_session_open(spinum);
		secspi_execute(2, NULL);
		secspi_session_close();
		SPIDEV_MSG("secspi_execute 2 finished!!!\n");
	} else if (!strncmp(buf, "debug", 5) && (sscanf(buf + 5, "%d", &spinum) == 1)) { /*DEBUG*/
		SPIDEV_MSG("start to access TL SPI driver.\n");
		secspi_session_open(spinum);
		secspi_execute(3, NULL);
		secspi_session_close();
		SPIDEV_MSG("secspi_execute 3 finished!!!\n");
	} else if (!strncmp(buf, "test", 4) && (sscanf(buf + 4, "%d", &spinum) == 1)) { /*TEST*/
		SPIDEV_MSG("start to access TL SPI driver.\n");
		secspi_session_open(spinum);
		secspi_execute(4, NULL);
		secspi_session_close();
		SPIDEV_MSG("secspi_execute 4 finished!!!\n");
#else
	if (!strncmp(buf, "-h", 2)) {
		SPIDEV_MSG("Please input the parameters for this device.\n");
#endif
	} else if (!strncmp(buf, "-w", 2)) {

	}
out:
	if (!spi->controller_data)
		kfree(chip_config);
	return count;
}

static ssize_t
spi_msg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	struct spi_device *spi;

	struct spi_transfer transfer = {0,};
	struct spi_message msg;
	/*struct mtk_chip_config *chip_config;*/

	u32 len = 4;
	u32 tx_buffer = 0x12345678;
	u32 rx_buffer = 0xaaaaaaaa;

	transfer.tx_buf = &tx_buffer;
	transfer.rx_buf = &rx_buffer;
	transfer.len = 4;

	spi = container_of(dev, struct spi_device, dev);

	if (unlikely(!spi)) {
		SPIDEV_LOG("spi device is invalid\n");
		goto out;
	}
	if (unlikely(!buf)) {
		SPIDEV_LOG("buf is invalid\n");
		goto out;
	}
	spi_message_init(&msg);

	if (!strncmp(buf, "-h", 2)) {
		SPIDEV_MSG("Please input the message of this device to send and receive.\n");
	} else if (!strncmp(buf, "-func", 5)) {
		buf += 6;
		if (!strncmp(buf, "len=", 4) && 1 == sscanf(buf + 4, "%d", &len)) {
			spi_setup_xfer(&spi->dev, &transfer, len, 1);
			spi_message_add_tail(&transfer, &msg);
			ret = spi_sync(spi, &msg);
			if (ret < 0) {
				SPIDEV_LOG("Message transfer err:%d\n", ret);
			} else {
				ret = spi_recv_check(&msg);
				if (ret != 0) {
					ret = -ret;
					SPIDEV_LOG("Message transfer err:%d\n", ret);
					goto out;
				}
			}
		}
	} else{
		SPIDEV_LOG("Wrong parameters.\n");
		ret = -1;
		goto out;
	}
	ret = count;

out:
	return ret;

}

static DEVICE_ATTR(spi, 0200, NULL, spi_store);
static DEVICE_ATTR(spi_msg, 0200, NULL, spi_msg_store);



static struct device_attribute *spi_attribute[] = {
	&dev_attr_spi,
	&dev_attr_spi_msg,
};

static int spi_create_attribute(struct device *dev)
{
	int num, idx;
	int err = 0;

	num = (int)ARRAY_SIZE(spi_attribute);
	for (idx = 0; idx < num; idx++) {
		err = device_create_file(dev, spi_attribute[idx]);
		if (err)
			break;
	}
	return err;

}

static void spi_remove_attribute(struct device *dev)
{
	int num, idx;

	num = (int)ARRAY_SIZE(spi_attribute);
	for (idx = 0; idx < num; idx++)
		device_remove_file(dev, spi_attribute[idx]);
}

static int spi_test_remove(struct spi_device *spi)
{

	SPIDEV_LOG("spi_test_remove.\n");
	spi_remove_attribute(&spi->dev);
	return 0;
}

static int spi_test_probe(struct spi_device *spi)
{
#ifdef CONFIG_SPI_LCM_SUPPORT//by yoyo
	int ret;
#endif
	SPIDEV_LOG("spi test probe  enter\n");
	spi_test = spi;
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
#ifdef CONFIG_SPI_LCM_SUPPORT//by yoyo
	   while( get_capture_state() == 0 )
        {
            msleep(1);
        }
        set_capture_state(0);
#if !defined( __USE_STATIC_BUFFER__)
	   g_color_temp = (void*)kmalloc(LCD_PHY_HEIGHT * LCD_PHY_WIDTH * 4+64, GFP_KERNEL | GFP_DMA);
	if(g_color_temp== NULL){
        printk("Fail to alloc vmalloc 0\n");
        ret= -1;
    }
    else
    {
		memset(g_color_temp , 0, LCD_PHY_HEIGHT * LCD_PHY_WIDTH * 4+64);
    }		 
#endif //!defined( __USE_STATIC_BUFFER__)

	capturethread= kthread_run(capture_thread, (void *)spi, "capturethread");
    if(IS_ERR(capturethread)) 
    { 
      ret = PTR_ERR(capturethread);
      printk( " failed to create kernel thread: %d\n", ret);
    }
    	flushthread = kthread_run(flushlcm_thread, (void *)spi, "flushlcmthread");
    if(IS_ERR(flushthread)) 
    { 
      ret = PTR_ERR(flushthread);
      printk( " failed to create kernel thread: %d\n", ret);
    }
#endif

	return spi_create_attribute(&spi->dev);
	return 0;
}

struct spi_device_id spi_id_table[] = {
	{"spi-ut", 0},
	{},
};

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "mediatek,spi-mt65xx-test" },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static struct spi_driver spi_test_driver = {
	.driver = {
		.name = "test_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = spidev_dt_ids,
	},
	.probe = spi_test_probe,
	.remove = spi_test_remove,
	.id_table = spi_id_table,
};

static int __init spi_dev_init(void)
{
	SPIDEV_LOG("SPI_DEV_INIT.\n");
	return spi_register_driver(&spi_test_driver);
}

static void __exit spi_test_exit(void)
{
	SPIDEV_LOG("SPI_DEV_EXIT.\n");
	spi_unregister_driver(&spi_test_driver);
}

module_init(spi_dev_init);
module_exit(spi_test_exit);

MODULE_DESCRIPTION("mt SPI test device driver");
MODULE_AUTHOR("ZH Chen <zh.chen@mediatek.com>");
MODULE_LICENSE("GPL");

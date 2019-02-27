#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/signal.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <DpDataType.h>

#define PLATFORM_DRIVER_NAME   "rgbleds"

	
struct pinctrl * this_pinctrl = NULL; 
struct pinctrl_state *rgb_r_high = NULL;
struct pinctrl_state *rgb_r_low = NULL;
struct pinctrl_state *rgb_g_high = NULL;
struct pinctrl_state *rgb_g_low = NULL;
struct pinctrl_state *rgb_b_high = NULL;
struct pinctrl_state *rgb_b_low = NULL;

#define RGB_R_HIGH_EN()			  pinctrl_select_state(this_pinctrl, rgb_r_high);
#define RGB_R_LOW_EN()		    pinctrl_select_state(this_pinctrl, rgb_r_low);
#define RGB_G_HIGH_EN()			  pinctrl_select_state(this_pinctrl, rgb_g_high);
#define RGB_G_LOW_EN()		    pinctrl_select_state(this_pinctrl, rgb_g_low);
#define RGB_B_HIGH_EN()			  pinctrl_select_state(this_pinctrl, rgb_b_high);
#define RGB_B_LOW_EN()		    pinctrl_select_state(this_pinctrl, rgb_b_low);

enum mt65xx_led_mode {
  RGB_LED_ALL_OFF=0,
	RGB_LED_R_ON,
	RGB_LED_R_OFF,
	RGB_LED_G_ON,
	RGB_LED_G_OFF,
	RGB_LED_B_ON,
	RGB_LED_B_OFF,
  RGB_LED_ALL_ON	
};

static int rgb_mode=0;

void rgb_leds_interface(unsigned int mode)
{
	switch (mode)
	{ 
      case RGB_LED_ALL_OFF:	
           RGB_R_LOW_EN(); RGB_G_LOW_EN(); RGB_B_LOW_EN();	break;
           
	   	case RGB_LED_R_ON:
	   	         RGB_R_HIGH_EN(); break;
	   	         
	   	case RGB_LED_R_OFF:
	   	         RGB_R_LOW_EN(); break;
	   	         
	   	case RGB_LED_G_ON:
	   	         RGB_G_HIGH_EN();break;
	   	         
	   	case RGB_LED_G_OFF:
	   	         RGB_G_LOW_EN(); break;	
	   	         	   	           	          
	   	case RGB_LED_B_ON:
	   	         RGB_B_HIGH_EN(); break;
	   	     
	   	case RGB_LED_B_OFF:
	   	         RGB_B_LOW_EN(); break;	 	
	   	        	 
      case RGB_LED_ALL_ON:
           RGB_R_HIGH_EN(); RGB_G_HIGH_EN(); RGB_B_HIGH_EN();	break;
  }         
    return ;
}
EXPORT_SYMBOL(rgb_leds_interface);

struct class *rgbleds_class;
struct device *rgbleds_dev;

static ssize_t rgbleds_enable_show(struct device *dev, struct device_attribute *attr, char *buf) 
{           
     return scnprintf(buf, PAGE_SIZE, "%d\n", rgb_mode);
}

static ssize_t rgbleds_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int mode = 0;
    if(buf != NULL && size != 0)
    {
        mode = (int)simple_strtoul(buf, NULL, 0);
    }
    rgb_mode=mode;
#if 1  
	printk("[rgbleds]rgb_leds mode change!! mode=%d\n", mode);
    rgb_leds_interface(mode);
#endif	    
    return size;
}

static DEVICE_ATTR(rgbleds_enable, 0644, rgbleds_enable_show, rgbleds_enable_store);
static int create_node_file(void)
{

	rgbleds_class = class_create(THIS_MODULE, "rgbleds");
	rgbleds_dev = device_create(rgbleds_class, NULL, 0, NULL,  "rgbleds");
    device_create_file(rgbleds_dev, &dev_attr_rgbleds_enable);
	return 0;

}

static int RGB_probe(struct platform_device *pdev)
{
	int ret = 0;

    printk(" rgb_leds RGB_probe in!\n ");
    // dts read
    pdev->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,rgbleds");

	  this_pinctrl = devm_pinctrl_get(&pdev->dev);
                
	rgb_r_high = pinctrl_lookup_state(this_pinctrl, "rgb_r_high");
	if (IS_ERR(rgb_r_high)) {
		ret = PTR_ERR(rgb_r_high);
		printk("%s : pinctrl lookup rgb_r_high failed\n", __func__);
	}	                 
                 
	rgb_r_low = pinctrl_lookup_state(this_pinctrl, "rgb_r_low");
	if (IS_ERR(rgb_r_low)) {
		ret = PTR_ERR(rgb_r_low);
		printk("%s : pinctrl lookup rgb_r_low failed\n", __func__);
	}	                          

	rgb_g_high = pinctrl_lookup_state(this_pinctrl, "rgb_g_high");
	if (IS_ERR(rgb_g_high)) {
		ret = PTR_ERR(rgb_g_high);
		printk("%s : pinctrl lookup rgb_g_high failed\n", __func__);
	}	                 
                 
	rgb_g_low = pinctrl_lookup_state(this_pinctrl, "rgb_g_low");
	if (IS_ERR(rgb_g_low)) {
		ret = PTR_ERR(rgb_g_low);
		printk("%s : pinctrl lookup rgb_g_low failed\n", __func__);
	}	
	
	rgb_b_high = pinctrl_lookup_state(this_pinctrl, "rgb_b_high");
	if (IS_ERR(rgb_b_high)) {
		ret = PTR_ERR(rgb_b_high);
		printk("%s : pinctrl lookup rgb_b_high failed\n", __func__);
	}	                 
                 
	rgb_b_low = pinctrl_lookup_state(this_pinctrl, "rgb_b_low");
	if (IS_ERR(rgb_b_low)) {
		ret = PTR_ERR(rgb_b_low);
		printk("%s : pinctrl lookup rgb_b_low failed\n", __func__);
	}		
   create_node_file();

	return ret;
}

static int RGB_remove(struct platform_device *pdev)
{
	return 0;
}

static int RGB_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int RGB_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver rgb_leds_Driver = {
	.probe = RGB_probe,
	.remove = RGB_remove,
	.suspend = RGB_suspend,
	.resume = RGB_resume,
	.driver = {
		   .name = PLATFORM_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};

static struct platform_device rgb_leds_device = {
	.name = PLATFORM_DRIVER_NAME,
	.id = 0,
	.dev = {}
};

static int __init rgb_leds_i2C_init(void)
{
	
	if (platform_device_register(&rgb_leds_device)) {
		printk("failed to register rgb_leds driver\n"); 
		return -ENODEV;
	}

	if (platform_driver_register(&rgb_leds_Driver)) {
		printk("Failed to register rgb_leds driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit rgb_leds_i2C_exit(void)
{

	platform_driver_unregister(&rgb_leds_Driver);
}

module_init(rgb_leds_i2C_init);
module_exit(rgb_leds_i2C_exit);

MODULE_DESCRIPTION("MTK MT6735 breath rgb_leds driver");
MODULE_AUTHOR("zhoucheng <cheng.zhou@runyee.com.cn>");
MODULE_LICENSE("GPL");

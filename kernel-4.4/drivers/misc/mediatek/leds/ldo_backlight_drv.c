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
#include <linux/timer.h>

#define PLATFORM_DRIVER_NAME   "ldo_backlight"

static struct hrtimer ldo_timer;
struct pinctrl * ldo_pinctrl = NULL; 
struct pinctrl_state *ldo_high = NULL;
struct pinctrl_state *ldo_low = NULL;


#define LDO_HIGH_EN()			  pinctrl_select_state(ldo_pinctrl, ldo_high);
#define LDO_LOW_EN()		    pinctrl_select_state(ldo_pinctrl, ldo_low);

enum mt65xx_led_mode {
	LDO_OFF,
	LDO_ON
};

static int ldo_mode=0;
static int ldo_sleep=0;
void ldo_backlight_interface(unsigned int mode)
{
	switch (mode)
	{ 
      case LDO_OFF:	
           LDO_LOW_EN();	break;
           
	   	case LDO_ON:
	   	     LDO_HIGH_EN();  break;	   	    
  }         
    return ;
}
EXPORT_SYMBOL(ldo_backlight_interface);

void ldo_backlight_callback(void)
{
   if(ldo_sleep==0)
   {
     ktime_t ktime = ktime_set(6, 0);	/* 6s, 10* 1000 ms */
     hrtimer_start(&ldo_timer, ktime, HRTIMER_MODE_REL);
	   
	   ldo_backlight_interface(LDO_ON);
	 }  
}
EXPORT_SYMBOL(ldo_backlight_callback);


enum hrtimer_restart ldo_button_hrtimer_func(struct hrtimer *timer)
{
     ldo_backlight_interface(LDO_OFF); 
	   return HRTIMER_NORESTART;
}

void bide_kthread_hrtimer_init(void)
{
	ktime_t ktime = ktime_set(1, 0);

	hrtimer_init(&ldo_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ldo_timer.function = ldo_button_hrtimer_func;
	hrtimer_start(&ldo_timer, ktime, HRTIMER_MODE_REL);
} 

void ldo_button_backlight_off(unsigned int off)
{
   if(off)
   {
       ldo_sleep=1;
       ldo_backlight_interface(LDO_OFF);     
   }else{
       ldo_sleep=0;
   }
}
EXPORT_SYMBOL(ldo_button_backlight_off);

struct class *ldobacklight_class;
struct device *ldobacklight_dev;

static ssize_t ldobacklight_enable_show(struct device *dev, struct device_attribute *attr, char *buf) 
{           
     return scnprintf(buf, PAGE_SIZE, "%d\n", ldo_mode);
}

static ssize_t ldobacklight_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int mode = 0;
    if(buf != NULL && size != 0)
    {
        mode = (int)simple_strtoul(buf, NULL, 0);
    }
    ldo_mode=mode;
#if 1  
	printk("[ldobacklight]ldo_backlight mode change!! mode=%d\n", mode);
    ldo_backlight_interface(mode);
#endif	    
    return size;
}

static DEVICE_ATTR(ldobacklight_enable, 0644, ldobacklight_enable_show, ldobacklight_enable_store);


static int ldo_probe(struct platform_device *pdev)
{
	int ret = 0;

    printk(" ldo_backlight ldo_probe in!\n ");
    // dts read
    pdev->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,ldobacklight");

	  ldo_pinctrl = devm_pinctrl_get(&pdev->dev);
                
	ldo_high = pinctrl_lookup_state(ldo_pinctrl, "ldo_high");
	if (IS_ERR(ldo_high)) {
		ret = PTR_ERR(ldo_high);
		printk("%s : pinctrl lookup ldo_high failed\n", __func__);
	}	                 
 
	ldo_low = pinctrl_lookup_state(ldo_pinctrl, "ldo_low");
	if (IS_ERR(ldo_low)) {
		ret = PTR_ERR(ldo_low);
		printk("%s : pinctrl lookup ldo_low failed\n", __func__);
	}	
	                                         	
  ldobacklight_class = class_create(THIS_MODULE, "ldo_backlight");
	ldobacklight_dev = device_create(ldobacklight_class, NULL, 0, NULL,  "ldo_backlight");
    device_create_file(ldobacklight_dev, &dev_attr_ldobacklight_enable);
    
  bide_kthread_hrtimer_init();
    
	return ret;
}

static int ldo_remove(struct platform_device *pdev)
{
	return 0;
}

static int ldo_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int ldo_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver ldo_backlight_Driver = {
	.probe = ldo_probe,
	.remove = ldo_remove,
	.suspend = ldo_suspend,
	.resume = ldo_resume,
	.driver = {
		   .name = PLATFORM_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};

static struct platform_device ldo_backlight_device = {
	.name = PLATFORM_DRIVER_NAME,
	.id = 0,
	.dev = {}
};

static int __init ldo_backlight_i2C_init(void)
{
	
	if (platform_device_register(&ldo_backlight_device)) {
		printk("failed to register ldo_backlight driver\n"); 
		return -ENODEV;
	}

	if (platform_driver_register(&ldo_backlight_Driver)) {
		printk("Failed to register ldo_backlight driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit ldo_backlight_i2C_exit(void)
{

	platform_driver_unregister(&ldo_backlight_Driver);
}

module_init(ldo_backlight_i2C_init);
module_exit(ldo_backlight_i2C_exit);

MODULE_DESCRIPTION("MTK MT6735 breath ldo_backlight driver");
MODULE_AUTHOR("zhoucheng <cheng.zhou@runyee.com.cn>");
MODULE_LICENSE("GPL");

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

#define PLATFORM_DRIVER_NAME   "aw9364"

	
struct pinctrl * aw9364_pinctrl = NULL; 
struct pinctrl_state *aw9364_high = NULL;
struct pinctrl_state *aw9364_low = NULL;

#define aw9364_EN_HIGH()		  pinctrl_select_state(aw9364_pinctrl, aw9364_high);
#define aw9364_EN_LOW()		    pinctrl_select_state(aw9364_pinctrl, aw9364_low);

static unsigned int old_level_t=0;
static unsigned int old_aw9364_en=1;
static unsigned int new_aw9364_en=0;
static unsigned int lk_flag=0;
static DEFINE_SPINLOCK(aw9364_drv_lock);

static void aw9364_SetCount_en(unsigned int level_t)
{
	  unsigned long flags;
	  unsigned int i =0;
	  unsigned int SetCount=0;
	  
	  spin_lock_irqsave(&aw9364_drv_lock, flags);
	  if(lk_flag == 0)
	  {
	  	 old_level_t = 5;
	  	 lk_flag = 1;
	  }
	  if(old_level_t != level_t)
	  {
	    if(old_level_t < level_t)
	    {
	  	  SetCount = level_t - old_level_t;
	    }else if(old_level_t > level_t){
	    	SetCount = level_t + 16  - old_level_t;
	    }   
	  }  	   
	  printk("[aw9364] count =%d, old_level_t=%d, level_t=%d, new_enable=%d ,old_enable=%d\n", SetCount, old_level_t, level_t, new_aw9364_en, old_aw9364_en);
	   
     if(new_aw9364_en == 0)
     {
     		aw9364_EN_LOW();
     		mdelay(3);
     		old_aw9364_en =0; 
     		old_level_t = 0;   		
     		spin_unlock_irqrestore(&aw9364_drv_lock, flags);
     		return ;
     }else{	    	
     	
     	 if(old_level_t != level_t)
	     {     	
     	    if(old_aw9364_en == 0)
     	    {
     	 	     aw9364_EN_HIGH();
     	 	     udelay(30); 
             for(i=0; i< level_t-1; i++)
             {
     	          aw9364_EN_LOW();
     	          udelay(2);
     	          aw9364_EN_HIGH();
     	          udelay(1);
             }
          }else{
             for(i=0;  i< SetCount; i++)
            {
     	       aw9364_EN_LOW();
     	       udelay(2);
     	       aw9364_EN_HIGH();
     	       udelay(1);
            }       	
       	 
          }       
          old_aw9364_en =1;  
       } 
     }
     old_level_t = level_t;
     spin_unlock_irqrestore(&aw9364_drv_lock, flags); 
     return ;
}
void aw9364_set_backlight_level(unsigned int level)
{     
    unsigned int set;
    unsigned int level_t;
    unsigned char arr[16]={16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1};
    unsigned long flags;
    
    spin_lock_irqsave(&aw9364_drv_lock, flags);
    if(level == 0)
    {
       new_aw9364_en = 0;
       level_t = 16;
    }else if(level < 48){
    	 new_aw9364_en = 1;
    	 level_t = 16;
    	 
    }else if(level <= 768){
    	 new_aw9364_en = 1;
       set = level/48;
       set= set-1;
       level_t = arr[set];  
    }else if(level >768){
    	 new_aw9364_en = 1;
       level_t = 1;
    }
    spin_unlock_irqrestore(&aw9364_drv_lock, flags); 
    //printk("[aw9364_set_backlight_level] level =%d \n",level);
    
    aw9364_SetCount_en(level_t);
   
    return ;
}
EXPORT_SYMBOL(aw9364_set_backlight_level);


static int aw9364_probe(struct platform_device *pdev)
{
	int ret = 0;

    printk("aw9364_probe in!\n ");
    // dts read
    pdev->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,aw9364");

	  aw9364_pinctrl = devm_pinctrl_get(&pdev->dev);
                
	aw9364_high = pinctrl_lookup_state(aw9364_pinctrl, "aw9364_high");
	if (IS_ERR(aw9364_high)) {
		ret = PTR_ERR(aw9364_high);
		printk("%s : pinctrl lookup aw9364_high failed\n", __func__);
	}	                 
                 
	aw9364_low = pinctrl_lookup_state(aw9364_pinctrl, "aw9364_low");
	if (IS_ERR(aw9364_low)) {
		ret = PTR_ERR(aw9364_low);
		printk("%s : pinctrl lookup aw9364_low failed\n", __func__);
	}

	return ret;
}

static int aw9364_remove(struct platform_device *pdev)
{
	return 0;
}

static int aw9364_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int aw9364_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver aw9364_Driver = {
	.probe = aw9364_probe,
	.remove = aw9364_remove,
	.suspend = aw9364_suspend,
	.resume = aw9364_resume,
	.driver = {
		   .name = PLATFORM_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};

static struct platform_device aw9364_device = {
	.name = PLATFORM_DRIVER_NAME,
	.id = 0,
	.dev = {}
};

static int __init aw9364_init(void)
{
	
	if (platform_device_register(&aw9364_device)) {
		printk("failed to register aw9364 driver\n"); 
		return -ENODEV;
	}

	if (platform_driver_register(&aw9364_Driver)) {
		printk("Failed to register aw9364 driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit aw9364_exit(void)
{

	platform_driver_unregister(&aw9364_Driver);
}

module_init(aw9364_init);
module_exit(aw9364_exit);

MODULE_DESCRIPTION("MTK MT6735 breath aw9364 driver");
MODULE_AUTHOR("zhoucheng <cheng.zhou@runyee.com.cn>");
MODULE_LICENSE("GPL");

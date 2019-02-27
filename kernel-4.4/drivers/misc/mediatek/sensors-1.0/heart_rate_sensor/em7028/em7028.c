#if 1
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/sysfs.h>
#include <linux/device.h> 
#include <heart_rate.h>
#include "em7028.h"
//#include "libBpm.h"
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/wakelock.h>
//#include <batch.h>
#include <hwmsensor.h>
//#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <hwmsen_helper.h>


/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define EM7028_DEV_NAME     "em7028"

/*----------------------------------------------------------------------------*/
#define EM7028_DEBUG
#ifdef EM7028_DEBUG
#define APS_TAG                  "[HRM] "
#define APS_FUN(f)               printk(APS_TAG"%s %d \n", __FUNCTION__ , __LINE__)
#define APS_ERR(fmt, args...)    printk(APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)    
#else
#define APS_TAG                  "[HRM] "
#define APS_FUN(f)               
#define APS_ERR(fmt, args...)    printk(APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    
#define APS_DBG(fmt, args...)       
#endif             

/*----------------------------------------------------------------------------*/
static struct i2c_client *EM7028_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id EM7028_i2c_id[] = {{EM7028_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_EM7028={ I2C_BOARD_INFO(EM7028_DEV_NAME, 0x48>>1)};

static int EM7028_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int EM7028_i2c_remove(struct i2c_client *client);
//static int EM7028_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
//static int EM7028_i2c_suspend(struct i2c_client *client, pm_message_t msg);
//static int EM7028_i2c_resume(struct i2c_client *client);
int EM7028_read_hrs(struct i2c_client *client, u16 *data);
static int EM7028_enable_hrs(struct i2c_client *client, int enable);
static int  EM7028_local_init(void);
static int  EM7028_local_uninit(void);
//extern int em70xx_bpm_dynamic(int i, int j,int k,int kk);
//extern int em70xx_reset(int i);

static int	EM7028_init_flag = -1;	// 0<==>OK -1 <==> fail
static int faraway_hand_flag=1;
static int heart_enable=0;

static DEFINE_MUTEX(sensor_lock);
static struct hrm_init_info EM7028_init_info = {
		.name = "EM7028",
		.init = EM7028_local_init,
		.uninit = EM7028_local_uninit,
};

/*----------------------------------------------------------------------------*/
struct EM7028_priv {
	struct i2c_client *client;
	struct work_struct  eint_work;
	struct input_dev *hrs_dev;
	struct mutex hrm_mutex;
	/*misc*/
	atomic_t    i2c_retry;
	/*data*/
	u16          hrs;
	ulong       enable;         /*enable mask*/
	ulong       pending_intr;   /*pending interrupt*/

	struct hrtimer hrs_timer;	
	struct workqueue_struct *em_hrs_wq;
	struct work_struct em_hrs_work;
	ktime_t hrs_delay;	
	
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id heart_rate_of_match[] = {
	{.compatible = "mediatek,heart"},
	{},
};
#endif

static struct i2c_driver EM7028_i2c_driver = {	
	.probe      = EM7028_i2c_probe,
	.remove     = EM7028_i2c_remove,
	//.detect     = EM7028_i2c_detect,
	//.suspend    = EM7028_i2c_suspend,
	//.resume     = EM7028_i2c_resume,
	.id_table   = EM7028_i2c_id,
	//.address_data = &EM7028_addr_data,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = EM7028_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = heart_rate_of_match,
#endif
	},
};

static struct EM7028_priv *EM7028_obj = NULL;

static int EM7028_get_hrs_value(struct EM7028_priv *obj, u16 hrs);
#if 0
/******************************************************************************/
static ssize_t EM7028_store_enable(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{
	struct EM7028_priv *obj ;	 
	obj = EM7028_obj;
	APS_DBG("hrm_store_active buf=%s\n",buf);
	mutex_lock(&obj->hrm_mutex);

    if (!strncmp(buf, "1", 1)) 
     {
        EM7028_enable_hrs(obj->client,1);
       
      } 
	else if (!strncmp(buf, "0", 1))
	{
         EM7028_enable_hrs(obj->client,0);
      }
	else
	{
	  APS_DBG(" hrm_store_active error !!\n");
	}
	mutex_unlock(&obj->hrm_mutex);
	APS_DBG(" hrm_store_active done\n");
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t EM7028_show_enable(struct device* dev, struct device_attribute *attr, char *buf) 
{
	struct EM7028_priv *obj;	 
	int div = 0;
	obj = EM7028_obj;
		
	APS_DBG("hrm show active not support now\n");

	APS_DBG("EM7028 vender_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div); 
	
	//return len;
}

DEVICE_ATTR(enable,     S_IWUSR | S_IRUGO, EM7028_show_enable, EM7028_store_enable);

static struct attribute *EM7028_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};
static struct attribute_group EM7028_attribute_group = {
	.attrs = EM7028_attributes
};
#endif
/*----------------------------------------------------------------------------*/
int EM7028_check_version(struct i2c_client *client)
{
	//struct EM7028_priv *obj = i2c_get_clientdata(client);	 
	
	u8 version;
	u8 buffer[1];
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=EM7028_CMM_ID;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, &version, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
		 
	APS_DBG("EM7028 hrm version = 0x%x\t\n",version);//yjc 0x36
	return 0;	 

EXIT_ERR:
	APS_ERR("EM7028_check_version i2c fail\n");
	return -1;
}
int EM7028_read_h80(struct i2c_client *client)
{
	u8 hrs_value_low[1], hrs_value_high[1];
	u8 buffer[1];
	int res = 0,data=0;
	
	buffer[0]=0x20; //buffer[0]=0x20;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, hrs_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=0x21; //buffer[0]=0x21;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, hrs_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	data = hrs_value_low[0] | ((hrs_value_high[0])<<8);
	return data;
	
EXIT_ERR:
	APS_ERR("EM7028_read_h80 fail\n");
	return res;	
}
int EM7028_read_h08(struct i2c_client *client)
{
	u8 hrs_value_low[1], hrs_value_high[1];
	u8 buffer[1];
	int res = 0,data=0;
	
	buffer[0]=0x30;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, hrs_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=0x31;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, hrs_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	data = hrs_value_low[0] | ((hrs_value_high[0])<<8);
printk("[hrm]EM7028_read_h08 data=%d\n",data);	
	
	return data;
	
EXIT_ERR:
	APS_ERR("EM7028_read_h08 fail\n");
	return res;
}
static int far_cnt=0;
int EM7028_read_hrs(struct i2c_client *client, u16 *data)
{
	//struct EM7028_priv *obj = i2c_get_clientdata(client);	 
	u8 databuf[2]; 
	int res = 0,rawdata;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	if(faraway_hand_flag){
		
		rawdata=EM7028_read_h80(client);
		if(rawdata>6){
			databuf[0] = EM7028_CMM_ENABLE;	 
			databuf[1] = 0X08;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
			   APS_FUN();
			   goto EXIT_ERR;
			}
			//em70xx_reset(0);
			faraway_hand_flag=0; 
		}
		rawdata=0;
		em70xx_reset(0);
	}else{
		rawdata=EM7028_read_h08(client);
		if(rawdata<1000||rawdata>60000){
			if(far_cnt>33){
				databuf[0] = EM7028_CMM_ENABLE;	 
				databuf[1] = 0x80;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
				   APS_FUN();
				   goto EXIT_ERR;
				}
				em70xx_reset(0);
				faraway_hand_flag=1;
				far_cnt = 0;
			}else{
				far_cnt++;
			}	
		}else{
			far_cnt=0;
		}
	}
	
    *data = rawdata;
	 
	APS_DBG("EM7028_read_hrs faraway_hand_flag=%d,value = %d\t\n",faraway_hand_flag,*data);
	return 0;	 

EXIT_ERR:
	APS_ERR("EM7028_read_hrs fail\n");
	return res;
	}
/*----------------------------------------------------------------------------*/
//extern void em70xx_reset(int i);

static int EM7028_enable_hrs(struct i2c_client *client, int enable)
{
	struct EM7028_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;

      //  em70xx_reset(0);
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	APS_DBG("EM7028_enable_hrs, enable = %d\n", enable);

	if(enable)
	{
		databuf[1] = 0x80; //databuf[1] = 0X80;   
		databuf[0] = EM7028_CMM_ENABLE;    

		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

		APS_DBG("EM7028 hrs power on\n");
		hrtimer_start(&obj->hrs_timer, obj->hrs_delay, HRTIMER_MODE_REL);			
		faraway_hand_flag=1;
	}
	else
	{
		databuf[1] = 0X00;
		databuf[0] = EM7028_CMM_ENABLE;    
                       
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		faraway_hand_flag=1;
		//atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("EM7028hrs power off\n");
		hrtimer_cancel(&obj->hrs_timer);					
		cancel_work_sync(&obj->em_hrs_work);
	}
	
	return 0;
	
EXIT_ERR:
	APS_ERR("EM7028_enable_hrs fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int Bpm_value=0;
static void EM7028_work_func(struct work_struct *work)
{
	struct EM7028_priv *obj = (struct EM7028_priv *)container_of(work, struct EM7028_priv, em_hrs_work);		
	struct hwm_sensor_data data;

	int  bpm;
	EM7028_read_hrs(obj->client, &obj->hrs);
	
	 printk("EM7028_eint_work rawdata ps=%d \n",obj->hrs);
	 bpm= EM7028_get_hrs_value(obj, obj->hrs); 
	 printk("EM7028_eint_work bpm=%d \n",bpm);
	 Bpm_value = bpm;
	 data.values[0] = bpm;
	 data.values[1] = 2;
   data.values[2] = 0;
	 data.values[3] = 0;
	//input_report_abs(obj->hrs_dev,ABS_X,bpm);
	//input_sync(obj->hrs_dev);
  //hrm_data_report(data,2);
	return;
}	

static enum hrtimer_restart EM7028_timer_func(struct hrtimer *timer)
{
	struct EM7028_priv *obj = container_of(timer, struct EM7028_priv, hrs_timer);
	queue_work(obj->em_hrs_wq, &obj->em_hrs_work);	
	hrtimer_forward_now(&obj->hrs_timer, obj->hrs_delay);
	return HRTIMER_RESTART;	
}

/*----------------------------------------------------------------------------*/
static int EM7028_get_hrs_value(struct EM7028_priv *obj, u16 hrs)
{
	static int bpm;
	bpm =em70xx_bpm_dynamic(hrs, 0,0,0);	
	return bpm;
}

#if 1
static int EM7028_init_client(struct i2c_client *client)
{
	//struct EM7028_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	
	databuf[0] = EM7028_CMM_ENABLE;	 
	databuf[1] = 0x00; //databuf[1] = 0X80;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		APS_FUN();
	   goto EXIT_ERR;
	}

	databuf[0] = EM7028_CMM_OFFSET;	 
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		   APS_FUN();
	   goto EXIT_ERR;
	}
	databuf[0] = EM7028_CMM_GAIN;	 
	databuf[1] = 0X7F;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		APS_FUN();
	   goto EXIT_ERR;
	}
	databuf[0] = EM7028_CMM_CTR;	 
	databuf[1] = 0XC7;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		APS_FUN();
	   goto EXIT_ERR;
	}
	databuf[0] = 0x0e;	 
	databuf[1] = 0x06;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		APS_FUN();
	   goto EXIT_ERR;
	}
	return EM7028_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return -1;
}
#endif
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int EM7028_open(struct inode *inode, struct file *file)
{
	file->private_data = EM7028_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int EM7028_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/

static long EM7028_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct EM7028_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	char strbuf[3];

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				err = EM7028_enable_hrs(obj->client, 1);
				if(err )
				{
					APS_ERR("enable hrs fail: %d\n", err); 
					goto err_out;
				}
				
			}
			else
			{
				err = EM7028_enable_hrs(obj->client, 0);
				if(err)
				{
					APS_ERR("disable hrs fail: %d\n", err); 
					goto err_out;
				}
				
			}
			break;

		case ALSPS_GET_PS_MODE:
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			err = EM7028_read_hrs(obj->client, &obj->hrs);
			if(err)
			{
				goto err_out;
			}
			
			dat = EM7028_get_hrs_value(obj, obj->hrs);
			if(dat == -1)
			{
				err = -EFAULT;
				goto err_out;
			}
			
			sprintf(strbuf, "%d %d %d", dat, obj->hrs, obj->hrs);
			if(copy_to_user(ptr, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:
			err = EM7028_read_hrs(obj->client, &obj->hrs);
			if(err)
			{
				goto err_out;
			}
			
			dat = obj->hrs;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations EM7028_fops = {
	.owner = THIS_MODULE,
	.open = EM7028_open,
	.release = EM7028_release,
	.unlocked_ioctl = EM7028_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice EM7028_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "heart_rate",
	.fops = &EM7028_fops,
};
/*----------------------------------------------------------------------------*/
#if 0
static int EM7028_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
//	struct EM7028_priv *obj = i2c_get_clientdata(client);    
//	int err;
	APS_FUN(); 
///	if(ps_enabled)
//	{
//		return -EACCES;
//	}
	
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
#if 0
static int EM7028_i2c_resume(struct i2c_client *client)
{
//	struct EM7028_priv *obj = i2c_get_clientdata(client);        
	APS_FUN();
	return 0;
}
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
/*----------------------------------------------------------------------------*/
static void EM7028_early_suspend(struct early_suspend *h) 
{   
	int err;
	APS_FUN();    
}
/*----------------------------------------------------------------------------*/
static void EM7028_late_resume(struct early_suspend *h)
{           
	int err;
	APS_FUN();
}
#endif
/*----------------------------------------------------------------------------*/
/*static int EM7028_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, EM7028_DEV_NAME);
	return 0;
}*/

/*----------------------------------------------------------------------------*/
static int hrm_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , hrm only enabled but not report inputEvent to HAL */

static int hrm_enable_nodata(int en)
{
	int err = 0;
	APS_LOG("EM7028 hrm enable value = %d\n", en);
	if(EM7028_obj == NULL){
		APS_ERR("EM7028_obj is null\n");
		return -1;
	}

		if(en)
		{
			mutex_lock(&sensor_lock);
			err = EM7028_enable_hrs(EM7028_obj->client, 1);
			if(err)
			{
				APS_ERR("enable ps fail: %d\n", err); 
				
				mutex_unlock(&sensor_lock);
				return -1;
			}
			mutex_unlock(&sensor_lock);						
		}
		else
		{
		
			mutex_lock(&sensor_lock);
			err = EM7028_enable_hrs(EM7028_obj->client, 0);
			if(err)
			{
				APS_ERR("disable ps fail: %d\n", err); 
				
				mutex_unlock(&sensor_lock);
				return -1;
			}
			mutex_unlock(&sensor_lock);
			//clear_bit(CMC_BIT_ALS, &EM7028_obj->enable);
		}


	return 0;
}

static int hrm_set_delay(u64 ns)
{
	return 0;
}

static int hrm_get_data(u32 *value, int *status)
{
	int err = 0;

	APS_LOG("EM7028 hrs get data \n");

	if(EM7028_obj == NULL){
		APS_ERR("EM7028_obj is null\n");
		return -1;
	}
	
	err = EM7028_read_hrs(EM7028_obj->client, &EM7028_obj->hrs);			
	*value = EM7028_get_hrs_value(EM7028_obj, EM7028_obj->hrs);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	APS_LOG("EM7028 hrm_get_data, value = %d\n",*value);

	return err;
}
struct class *heart_class;
struct device *heart_dev;

static ssize_t heart_data_show(struct device *dev, struct device_attribute *attr, char *buf) 
{
     return scnprintf(buf, PAGE_SIZE, "%d\n", Bpm_value);
}

static ssize_t heart_data_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{ 
      return size;
}
static ssize_t heart_enable_show(struct device *dev, struct device_attribute *attr, char *buf) 
{
     
     return scnprintf(buf, PAGE_SIZE, "%d\n", heart_enable);
}

static ssize_t heart_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int enable = 0;
    if(buf != NULL && size != 0)
    {
        enable = (int)simple_strtoul(buf, NULL, 0);
    }
    if(enable)
    {
      heart_enable = 1;
      EM7028_enable_hrs(EM7028_i2c_client,1);//turn on hrm
    }else{
      heart_enable = 0;    
      EM7028_enable_hrs(EM7028_i2c_client,0);//turn off hrm
    }
    return size;
}
static DEVICE_ATTR(heart_data,   0644,   heart_data_show,   heart_data_store);
static DEVICE_ATTR(heart_enable, 0644,   heart_enable_show, heart_enable_store);

static int EM7028_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct EM7028_priv *obj;
	int err = 0;
	struct hrm_control_path hrm_ctl={0};
	struct hrm_data_path hrm_data = { 0 };
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	
	//APS_DBG("EM7028 probe %s\n",__func__);
printk("[hrm]EM7028_i2c_probe begin...\n");
		
	err = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE);	
	if(err < 0){
		goto  exit;
	}	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	EM7028_obj = obj;
	
	APS_LOG("EM7028_i2c_probe() in i2c addr=0x%x!\n",client->addr);//0x24
	EM7028_i2c_client = client;
	
	if(EM7028_check_version(client)!=0)
	{
		err = -EIO;
		goto exit;
	}

	obj->client = client;
	obj->enable = 0;
	obj->pending_intr = 0;
	atomic_set(&obj->i2c_retry, 3);
	i2c_set_clientdata(client, obj);	
	err = EM7028_init_client(client);
	if(err!=0)
	{
        APS_ERR("EM7028_init_client failed\n");
		goto exit;
	}

	APS_LOG("EM7028_init_client() OK!\n");

	err = misc_register(&EM7028_device);
	if(err)
	{
		APS_ERR("EM7028_device register failed\n");
		goto exit_create_attr_failed;
	}
	
	hrm_ctl.set_delay = hrm_set_delay;
	hrm_ctl.open_report_data=hrm_open_report_data;
	hrm_ctl.enable_nodata =hrm_enable_nodata;
	hrm_ctl.is_support_batch =false;
	err = hrm_register_control_path(&hrm_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit;
	}
	hrm_data.get_data = hrm_get_data;
	hrm_data.vender_div = 1;
	err = hrm_register_data_path(&hrm_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit;
	}
	
	//err = batch_register_support_info(ID_HEART_RATE, hrm_ctl.is_support_batch, 1, 0);
	//if (err)
		//APS_ERR("register light batch support err = %d\n", err);

	/*err = sysfs_create_group(&obj->hrs_dev->dev.kobj,&EM7028_attribute_group);
	if (err < 0)
	{
	   APS_ERR("unable to create hrm attribute file\n");
	   return -3;
	}*/
	
	obj->em_hrs_wq = create_singlethread_workqueue("em_hrs_wq");
	INIT_WORK(&obj->em_hrs_work,EM7028_work_func);
	hrtimer_init(&obj->hrs_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	obj->hrs_delay = ns_to_ktime(30 * NSEC_PER_MSEC);
	obj->hrs_timer.function = EM7028_timer_func;
	
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = EM7028_early_suspend,
	obj->early_drv.resume   = EM7028_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
	EM7028_init_flag = 0;
	APS_LOG("EM7028 probe %s OK\n",__func__);
	EM7028_enable_hrs(client,0);//turn off hrm
	//EM7028_enable_hrs(client,1);//add zhou test
	mdelay(30); 
	
	//printk("EM7028_read_h08=%d,\n",EM7028_read_h08(client));
	
  heart_class = class_create(THIS_MODULE, "heart_class");
	heart_dev = device_create(heart_class, NULL, 0, NULL,  "heart_dev");
    device_create_file(heart_dev, &dev_attr_heart_data);
    device_create_file(heart_dev, &dev_attr_heart_enable);
    	
	return 0;

	exit_create_attr_failed:
	misc_deregister(&EM7028_device);
	exit:
	EM7028_i2c_client = NULL;    
	EM7028_init_flag = -1;

	//APS_ERR("%s: err = %d\n", __func__, err);
	
printk("[hrm]EM7028_i2c_probe end...\n");	
	return err;
}
/*----------------------------------------------------------------------------*/
static int EM7028_i2c_remove(struct i2c_client *client)
{
	//int err;
	
	misc_deregister(&EM7028_device);
	//if(err)
	//{
	//	APS_ERR("misc_deregister fail: %d\n", err);    
	//}
	
	EM7028_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

/*----------------------------------------------------------------------------*/

static int EM7028_local_init(void) 
{

printk("[hrm]EM7028_local_init...\n");
	if(i2c_add_driver(&EM7028_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	if(-1 == EM7028_init_flag)
	{
	   return -1;
	}
	
	return 0;
}

static int EM7028_local_uninit(void)
{
	APS_FUN();    
	i2c_del_driver(&EM7028_i2c_driver);
	EM7028_i2c_client = NULL;
	return 0;
}

static int __init EM7028_init(void)
{
	APS_FUN();    
	//i2c_register_board_info(1, &i2c_EM7028, 1);
	hrm_driver_add(&EM7028_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit EM7028_exit(void)
{
	APS_FUN();  
}
/*----------------------------------------------------------------------------*/

module_init(EM7028_init);     
module_exit(EM7028_exit);   
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("binghua chen@epticore.com");
MODULE_DESCRIPTION("EM7028 driver");
MODULE_LICENSE("GPL");
#endif

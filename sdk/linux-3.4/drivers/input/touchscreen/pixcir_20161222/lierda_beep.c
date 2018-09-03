#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/pwm.h>


#include <linux/types.h>
#include <linux/cdev.h>

#define SET_PARAMETER	13
#define ENABLE_PWM		14
#define DISABlE_PWM		15

static struct timer_list mytimer;
static struct class *beep_cls;
static struct class_device	*beep_class_dev;
static struct cdev beep_cdev; 
static struct pwm_device *pwm_dev;

#define MAX_NUM	2

static int major;

static int beep_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
	int buf[2] = {0, 0};
	int a = 12500;
	int b = 25000;
	copy_from_user(buf, (int *)arg, 2);
	switch(cmd)
	{
	  case SET_PARAMETER:
	  				   pwm_config(pwm_dev, a, b);
					   break;
	  case ENABLE_PWM:
	  			   printk("beep_ioctl : enable_pwm\n");
					/*
				   pwm_dev = pwm_request(0, "lierda_beep");
	 			   if(NULL == pwm_dev)
	 			   {
				   		printk("pwm request error!\n");
						return -1;
	 			   }
	 			   */
				
				   pwm_enable(pwm_dev);

				   //mod_timer(&mytimer,jiffies + 10);//改变定时器的时间
				   //mytimer.expires = jiffies + 20;
				   //add_timer(&mytimer);
		break;
	  case DISABlE_PWM:
	  			   printk("beep_ioctl : disable_pwm\n");
				   pwm_disable(pwm_dev);
	  	break;
	}
	return 0;
}

static int beep_open (struct inode *inode, struct file *file)
{
	 printk("beep_open\n");
	  pwm_dev = pwm_request(0, "lierda_beep");
	  if(NULL == pwm_dev)
	 			   {
				   		printk("pwm request error!\n");
						return -1;
	 			   }
				//   pwm_config(pwm_dev, 0, 25000);
				 //  pwm_enable(pwm_dev);
	 return 0;
}

static int beep_close(struct inode *node, struct file *filp)
{
	printk("beep_close\n");
	pwm_disable(pwm_dev);
	pwm_free(pwm_dev);
	return 0;
}

static struct file_operations beep_drv_ops = {
	.owner  =   THIS_MODULE,  
   	 .open   =   beep_open,     
	.unlocked_ioctl	=	beep_ioctl,
	.release = beep_close,
};

void  mytimer_ok(unsigned long arg)
{
		//printk("enter timer\n");
           //pwm_disable(pwm_dev);
		   //pwm_free(pwm_dev);
		   pwm_config(pwm_dev, 0, 25000);
		   //pwm_enable(pwm_dev);
}


static int beep_init(void)
{
	int rc;
	dev_t devid;

	if (major) {
		devid = MKDEV(major, 0);
		rc = register_chrdev_region(devid, MAX_NUM, "lierda_beep");
	} else {
		rc = alloc_chrdev_region(&devid, 0, MAX_NUM, "lierda_beep");
		major = MAJOR(devid);
	}
	if (rc < 0) {
		return -ENODEV;
	}

	//init_timer(&mytimer);     //初始化定时器
    
    //mytimer.function = mytimer_ok;//设置定时器超时函数
    //add_timer(&mytimer); //添加定时器，定时器开始生效
	
	cdev_init(&beep_cdev, &beep_drv_ops);
	cdev_add(&beep_cdev, devid, MAX_NUM);

	beep_cls = class_create(THIS_MODULE, "lierda_beep");

	device_create(beep_cls, NULL, MKDEV(major, 0), NULL, "lierda_beep"); 
	return 0;
}

static void beep_exit(void)
{
	device_destroy(beep_cls,MKDEV(major, 0));
	class_destroy(beep_cls);

	cdev_del(&beep_cdev);
	unregister_chrdev_region(MKDEV(major, 0), MAX_NUM);
	
}

module_init(beep_init);
module_exit(beep_exit);
MODULE_LICENSE("GPL");







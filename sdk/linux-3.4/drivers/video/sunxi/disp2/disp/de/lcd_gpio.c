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
#include "disp_lcd.h"

#include <linux/types.h>
#include <linux/cdev.h>


#include "disp_lcd.h"
#define LCD_SET_PWM 11
#define LCD_PWM_ENABLE 12
#define PRINT_DEBUG_ON 0


#define PRINT_DEBUG(fmt,arg...)          do{\
                                         if(PRINT_DEBUG_ON)\
                                         printk("<<PRINT_DEBUG->> "fmt"", ##arg);\
                                       }while(0)


static struct class *uart_gpio_cls;
static struct class_device	*uart_gpio_class_dev;
static struct cdev uart_gpio_cdev; 
static struct disp_device *lcd;


#define MAX_NUM	1

static int major;

static int lcd_gpio_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{	
	int buf;
	copy_from_user(&buf, arg, sizeof(int));
	
	switch(cmd)
	{
	  case LCD_SET_PWM:
	  				PRINT_DEBUG("uart_gpio_ioctl LCD_SET_PWM\n");
					PRINT_DEBUG("ioctl get buf[0] is %d\n", buf);
                    disp_lcd_set_bright(lcd, buf);
			break;
		
	}
	return 0;
}

static int lcd_gpio_open (struct inode *inode, struct file *file)
{
	PRINT_DEBUG("lcd_gpio_open\n");
	 	
	lcd = disp_get_lcd(0);
	if(NULL == lcd)
		return -1;
	return 0;
}

static struct file_operations uart_gpio_drv_ops = {
	.owner  =   THIS_MODULE,  
   	 .open   =   lcd_gpio_open,     
	.unlocked_ioctl	=	lcd_gpio_ioctl,
};

static int lcd_gpio_init(void)
{
	int rc;
	dev_t devid;

	if (major) {
		devid = MKDEV(major, 0);
		rc = register_chrdev_region(devid, MAX_NUM, "lcd_gpio");
	} else {
		rc = alloc_chrdev_region(&devid, 0, MAX_NUM, "lcd_gpio");
		major = MAJOR(devid);
	}
	if (rc < 0) {
		return -ENODEV;
	}

	cdev_init(&uart_gpio_cdev, &uart_gpio_drv_ops);
	cdev_add(&uart_gpio_cdev, devid, MAX_NUM);

	uart_gpio_cls = class_create(THIS_MODULE, "lcd_gpio");

	device_create(uart_gpio_cls, NULL, MKDEV(major, 0), NULL, "lcd_gpio"); 
	 
	return 0;
}

static void lcd_gpio_exit(void)
{
	device_destroy(uart_gpio_cls,MKDEV(major, 0));
	class_destroy(uart_gpio_cls);

	cdev_del(&uart_gpio_cdev);
	unregister_chrdev_region(MKDEV(major, 0), MAX_NUM);
}

module_init(lcd_gpio_init);
module_exit(lcd_gpio_exit);
MODULE_LICENSE("GPL");






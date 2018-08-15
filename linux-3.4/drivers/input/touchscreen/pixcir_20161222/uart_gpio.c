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
#include <linux/types.h>
#include <linux/cdev.h>
#include <mach/uncompress.h>

#define UART_GPIO_ON 1
#define UART_GPIO_OFF 0
#define WIFI_GPIO_ON 10
#define WIFI_GPIO_OFF 3
#define LED_GPIO_ON 20
#define LED_GPIO_OFF 30


#define UARTGPIO_NUM 129
#define WIFIGPIO_NUM 130
#define LEDGPIO_NUM 135

static struct class *uart_gpio_cls;
static struct class_device	*uart_gpio_class_dev;
static struct cdev uart_gpio_cdev; 

#define MAX_NUM	2

static int major;

static int uart_gpio_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
	switch(cmd)
	{
	  case UART_GPIO_ON:
	  				//printk("uart_gpio_ioctl ON\n");
                    //gpio_direction_output(UARTGPIO_NUM,1);
			gpio_set_value(UARTGPIO_NUM,1);
			break;
	  case UART_GPIO_OFF:
	  			   //printk("uart_gpio_ioctl OFF\n");
                   gpio_direction_output(UARTGPIO_NUM,0);
		break;
	  case WIFI_GPIO_ON:
	  				//printk("wifi_gpio_ioctl ON\n");
                    gpio_direction_output(WIFIGPIO_NUM,1);
			break;
	  case WIFI_GPIO_OFF:
	  			   //printk("wifi_gpio_ioctl OFF\n");
                   gpio_direction_output(WIFIGPIO_NUM,0);
		break;
	case LED_GPIO_ON:
                                        //printk("wifi_gpio_ioctl ON\n");
                    gpio_direction_output(LEDGPIO_NUM,1);
                        break;
          case LED_GPIO_OFF:
                                   //printk("wifi_gpio_ioctl OFF\n");
                   gpio_direction_output(LEDGPIO_NUM,0);
                break;
		
	}
	return 0;
}

static int uart_gpio_open (struct inode *inode, struct file *file)
{
	 printk("uart_gpio_open\n");
	 gpio_request(UARTGPIO_NUM,"uartgpio");
	 gpio_request(WIFIGPIO_NUM,"wifigpio");
	 gpio_direction_output(WIFIGPIO_NUM,1);
	 gpio_direction_output(UARTGPIO_NUM,0);
	 
	 return 0;
}

static struct file_operations uart_gpio_drv_ops = {
	.owner  =   THIS_MODULE,  
   	 .open   =   uart_gpio_open,     
	.unlocked_ioctl	=	uart_gpio_ioctl,
};

static int uart_gpio_init(void)
{
	int rc;
	dev_t devid;

	if (major) {
		devid = MKDEV(major, 0);
		rc = register_chrdev_region(devid, MAX_NUM, "uart_gpio");
	} else {
		rc = alloc_chrdev_region(&devid, 0, MAX_NUM, "uart_gpio");
		major = MAJOR(devid);
	}
	if (rc < 0) {
		return -ENODEV;
	}

	cdev_init(&uart_gpio_cdev, &uart_gpio_drv_ops);
	cdev_add(&uart_gpio_cdev, devid, MAX_NUM);

	uart_gpio_cls = class_create(THIS_MODULE, "uart_gpio");

	device_create(uart_gpio_cls, NULL, MKDEV(major, 0), NULL, "uart_gpio"); 
	device_create(uart_gpio_cls, NULL, MKDEV(major, 1), NULL, "wifi_gpio"); 
	return 0;
}

static void uart_gpio_exit(void)
{
	device_destroy(uart_gpio_cls,MKDEV(major, 0));
	device_destroy(uart_gpio_cls,MKDEV(major, 1));
	class_destroy(uart_gpio_cls);

	cdev_del(&uart_gpio_cdev);
	unregister_chrdev_region(MKDEV(major, 0), MAX_NUM);
}

module_init(uart_gpio_init);
module_exit(uart_gpio_exit);
MODULE_LICENSE("GPL");



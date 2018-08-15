#ifndef	_PIXCIR_I2C_TS_H
#define	_PIXCIR_I2C_TS_H

#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
//#include "/resolution.h"		//Michael

#define SLAVE_ADDR			0x5c
#define	BOOTLOADER_ADDR			0x5d

//if your TP has button please define this
#define BUTTON


#define DEFAULT_X_MAX 800//13800//1024
#define DEFAULT_Y_MAX 480//8000//768

#define GTP_AUTO_UPDATE       0//auto update fw as default
#define GTP_DEBUG_ON 1
#define BTIME 200
#define bootloader_delay (1500*6)
//#define bootloader_delay  65535

//for debug
#define GTP_INFO(fmt,arg...)           printk("<<-PIXCIR-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)          printk("<<-PIXCIR-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-PIXCIR-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)

#define FAIL -1
#define SUCCESS 0


//#define BUTTON   //if have button on TP

/*********************Platform gpio define************************/
//#define	S5PC1XX
//#define 	MINI6410
//#define      QCOM              1  //qcom platform
//#define      CONFIG_OF      1 //enable dts
//#define 	MINI210
//#define		TINY210
//#define	AMLOGIC

#define get_attb_value gpio_get_value

//#define tiny4412
#ifdef tiny4412
#define ATTB  42 //EXYNOS4_GPX1(7)//eint15 63
#define get_attb_value gpio_get_value
//#define	RESETPIN_CFG	s3c_gpio_cfgpin(EXYNOS4_GPX1(6),S3C_GPIO_OUTPUT)
#define	RESETPIN_SET1 gpio_set_value(EXYNOS4_GPX1(6),1)
#define	RESETPIN_SET0 gpio_set_value(EXYNOS4_GPX1(6),0)

#endif 

#ifdef S5PC1XX

#include <plat/gpio-bank-e1.h> //reset pin GPE1_5
#include <plat/gpio-bank-h1.h> //attb pin GPH1_3
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#define ATTB		S5PC1XX_GPH1(3)
#define get_attb_value	gpio_get_value
#define	RESETPIN_CFG	s3c_gpio_cfgpin(S5PC1XX_GPE1(5),S3C_GPIO_OUTPUT)
#define	RESETPIN_SET0 	gpio_direction_output(S5PC1XX_GPE1(5),0)
#define	RESETPIN_SET1	gpio_direction_output(S5PC1XX_GPE1(5),1)
#endif

#ifdef MINI6410	//mini6410
#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-e.h>
#include <mach/gpio-bank-n.h>
#include <mach/gpio.h>

#define ATTB		S3C64XX_GPN(11)
#define get_attb_value	gpio_get_value
#define	RESETPIN_CFG	s3c_gpio_cfgpin(S3C64XX_GPE(1),S3C_GPIO_OUTPUT)
#define	RESETPIN_SET0 	gpio_direction_output(S3C64XX_GPE(1),0)
#define	RESETPIN_SET1	gpio_direction_output(S3C64XX_GPE(1),1)
#endif

#ifdef MINI210
#include <plat/gpio-cfg.h>
#include <mach/gpio-bank.h>
#include <mach/gpio.h>

#define ATTB		S5PV210_GPH0(1)
#define get_attb_value	gpio_get_value
#define	RESETPIN_CFG	s3c_gpio_cfgpin(S5PV210_GPJ3(3),S3C_GPIO_OUTPUT)
#define	RESETPIN_SET0 	gpio_direction_output(S5PV210_GPJ3(3),0)
#define	RESETPIN_SET1	gpio_direction_output(S5PV210_GPJ3(3),1)
#endif

#ifdef TINY210
#include <plat/gpio-cfg.h>
#include <mach/gpio.h>
#include <linux/ptp_clock_kernel.h>

#define ATTB  S5PV210_GPH1(7)
#define get_attb_value gpio_get_value
#define RESETPIN_CFG s3c_gpio_cfgpin(S5PV210_GPH3(3),S3C_GPIO_OUTPUT)
#define RESETPIN_SET0  gpio_direction_output(S5PV210_GPH3(3),0)
#define RESETPIN_SET1 gpio_direction_output(S5PV210_GPH3(3),1)
#endif

#ifdef AMLOGIC
#include <linux/workqueue.h>
#include <linux/smp_lock.h>
#include <linux/gpio.h>

#define	GPIO_PIXCIR_PENIRQ	((GPIOD_bank_bit2_24(24)<<16) |GPIOD_bit_bit2_24(24))
#define ATTB			GPIO_PIXCIR_PENIRQ
#define get_attb_value		gpio_get_value
#define	RESETPIN_CFG		set_gpio_mode(GPIOD_bank_bit2_24(23), GPIOD_bit_bit2_24(23), GPIO_OUTPUT_MODE);
#define	RESETPIN_SET0		set_gpio_val(GPIOD_bank_bit2_24(23), GPIOD_bit_bit2_24(23), 0);
#define	RESETPIN_SET1		set_gpio_val(GPIOD_bank_bit2_24(23), GPIOD_bit_bit2_24(23), 1);
#endif

#ifdef CONFIG_OF
struct pixcir_ts_platform_data {
	u32 x_max;
	u32 y_max;
	u32 gpio_attb;		/* GPIO connected to ATTB line */	
	u32 gpio_rst;
	int irq;
};
#endif

struct pixcir_i2c_ts_data {
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int irq;
	u8 enter_update;
	s32 irq_is_disable;
};

#endif

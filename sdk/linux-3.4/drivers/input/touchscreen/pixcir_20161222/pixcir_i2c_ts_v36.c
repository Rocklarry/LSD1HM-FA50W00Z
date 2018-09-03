/*
 * Driver for Pixcir I2C touchscreen controllers.
 *
 * Copyright (C) 2010-2013 Pixcir, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.If not, see <http://www.gnu.org/licenses/>.
 *
 * pixcir_i2c_ts.c V3.0	from v3.0 support TangoC solution and remove the previous soltutions
 *
 * pixcir_i2c_ts.c V3.1	Add bootloader function	7
 *			Add RESET_TP		9
 * 			Add ENABLE_IRQ		10
 *			Add DISABLE_IRQ		11
 * 			Add BOOTLOADER_STU	16
 *			Add ATTB_VALUE		17
 *			Add Write/Read Interface for APP software
 *
 * pixcir_i2c_ts.c V3.2.09	for INT_MODE 0x09
 *				change to workqueue for ISR
 *				adaptable report rate self
 *
 * pixcir_i2c_ts.c V3.3.09	Add Android early power management
 *				Add irq_flag for pixcir debug tool
 *				Add CRC attb check when bootloader
 *
 * pixcir_i2c_ts.c V3.4.09 	modify bootloader according to interrupt
 *                 		add printd function, show info when define PIXCIR_DEBUG
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/init-input.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "pixcir_i2c_ts_v36.h"

/*********************************Bee-0928-TOP****************************************/
#define PIXCIR_DEBUG  0

#ifndef I2C_MAJOR
#define I2C_MAJOR 			125
#endif

#define I2C_MINORS 			256

#define	CALIBRATION_FLAG		1
#define	BOOTLOADER			7
#define RESET_TP			9

#define	ENABLE_IRQ			10
#define	DISABLE_IRQ			11
#define	BOOTLOADER_STU			16
#define ATTB_VALUE			17
#define	MAX_FINGER_NUM			5
//#define     CONFIG_OF    1   //enable dts



unsigned char status_reg = 0;
int irq_num, irq_flag, work_pending=0;

struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

static struct i2c_driver pixcir_i2c_ts_driver;
static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);

static struct workqueue_struct *pixcir_wq;

static struct ctp_config_info config_info = {
	.input_type = CTP_TYPE,
	.name = NULL,
	.int_number = 0,
};

#define ATTB config_info.irq_gpio.gpio
#define	RESETPIN_SET1 gpio_set_value(config_info.wakeup_gpio.gpio,1)
#define	RESETPIN_SET0 gpio_set_value(config_info.wakeup_gpio.gpio,0)

struct point_node_t {
	unsigned char active;
	unsigned char finger_id;
	unsigned int posx;
	unsigned int posy;
};

static struct point_node_t point_slot[MAX_FINGER_NUM * 2];
int bootloader_irq = 0; // 0: no, 1: falling

#ifdef QCOM
static u32 global_gpio_rst;
static int gloable_irq;
#endif
u32 global_gpio_attb;
struct i2c_client * i2c_connect_client;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_i2c_ts_early_suspend(struct early_suspend *h);
static void pixcir_i2c_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct pixcir_i2c_ts_data *ts);
#endif

extern void gtp_irq_disable(struct pixcir_i2c_ts_data *ts);
extern void gtp_irq_enable(struct pixcir_i2c_ts_data *ts);
#if GTP_AUTO_UPDATE
extern bool auto_update_flag;
#endif

int attb_read_val(void) {
#ifdef QCOM
	return gpio_get_value(global_gpio_attb);
#else
    return get_attb_value(ATTB);
#endif
}

void pixcir_reset(void) {
#ifdef QCOM
	gpio_direction_output(global_gpio_rst, 1);
	mdelay(10);
	gpio_direction_output(global_gpio_rst, 0);
#else
//        RESETPIN_CFG;
        RESETPIN_SET1;
        mdelay(10);
        RESETPIN_SET0;
#endif
}

void pixcir_init(void) {
	
#ifdef QCOM
int ret = -1;
	ret = gpio_request(global_gpio_rst, "gpio_rst");
	if(ret < 0){
		printk(KERN_ERR "request reset gpio failed!\n");
		return ;
	}
	
	pixcir_reset();	
	
	ret = gpio_request(global_gpio_attb, "gpio_attb");
	if(ret < 0){
		printk(KERN_ERR "request attb gpio failed!\n");
		return ;
	}else{
		gpio_direction_input(global_gpio_attb);
		gloable_irq = gpio_to_irq(global_gpio_attb);
	}
#else
       pixcir_reset();
       mdelay(150);

#endif


}

void judge_int_gpio_status(void)
{		
	int time_out = 0;

	while(!(attb_read_val()))
	{
		time_out++;
		udelay(BTIME);
		if(time_out > bootloader_delay) 
			break;
	}
    GTP_DEBUG("judge_int_gpio_status timeout1=%d",time_out);
	time_out = 0;
	while(attb_read_val())
	{
		time_out++;
		udelay(BTIME);
		if(time_out > bootloader_delay) 
			break;
	}
     GTP_DEBUG("judge_int_gpio_status timeout2=%d",time_out);
}

static void return_i2c_dev(struct i2c_dev *i2c_dev) {
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index) {
	struct i2c_dev *i2c_dev;
	i2c_dev = NULL;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
	found: spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap) {
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
				adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);

	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static int printd(char *fmt, ...) {
#ifdef PIXCIR_DEBUG
	va_list args;
	int r = 0;
#ifdef CONFIG_KGDB_KDB
	if (unlikely(kdb_trap_printk)) {
		va_start(args, fmt);
		r = vkdb_printf(fmt, args);
		va_end(args);
		return r;
	}
#endif
	va_start(args, fmt);
	r = vprintk(fmt, args);
	va_end(args);

	return r;
#endif
}

static void pixcir_ts_poscheck(struct work_struct *work) {
	struct pixcir_i2c_ts_data
	*tsdata = container_of(work,
			struct pixcir_i2c_ts_data,
			work.work);

	unsigned char *p;
	unsigned char touch, button, pix_id, slot_id;	
	unsigned char rdbuf[32], wrbuf[1] = { 0 };
	int ret, i;

	if(work_pending==2) return;

	ret = i2c_master_send(tsdata->client, wrbuf, sizeof(wrbuf));
	if (ret != sizeof(wrbuf)) {
		dev_err(&tsdata->client->dev, "%s: i2c_master_send failed(), ret=%d\n",
				__func__, ret);
	}

	ret = i2c_master_recv(tsdata->client, rdbuf, sizeof(rdbuf));
	if (ret != sizeof(rdbuf)) {
		dev_err(&tsdata->client->dev, "%s: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
	}

	touch = rdbuf[0] & 0x07;
	button = rdbuf[1];
	//printd("touch=%d,button=%d\n", touch, button);

#ifdef BUTTON
	if (button) {
		switch (button) {
		case 1:
			input_report_key(tsdata->input, KEY_HOME, 1);
		case 2:
			//add other key down report
		case 4:
		case 8:
			//add case for other more key 
		default:
			break;
		}
	} else {
		input_report_key(tsdata->input, KEY_HOME, 0);
		//add other key up report
	}
#endif

	p = &rdbuf[2];
	for (i = 0; i < touch; i++) {
		pix_id = (*(p + 4));
		slot_id = ((pix_id & 7) << 1) | ((pix_id & 8) >> 3);
		//printk( KERN_EMERG "### slod_id=%x",slot_id);
#ifndef QCOM   //obtain slot_id > 5 on QCOM platform
		point_slot[slot_id].active = 1;
		point_slot[slot_id].finger_id = pix_id;
		point_slot[slot_id].posx = (*(p + 1) << 8) + (*(p));
        point_slot[slot_id].posy = (*(p + 3) << 8) + (*(p + 2));
#endif
		p += 5;
	}

	if (touch) {
		input_report_key(tsdata->input, BTN_TOUCH, 1);
		input_report_abs(tsdata->input, ABS_PRESSURE, 1000);
		for (i = 0; i < MAX_FINGER_NUM * 2; i++) {
			if (point_slot[i].active == 1) {
				point_slot[i].active = 0;
				input_report_abs(tsdata->input, ABS_X, point_slot[i].posx);
        		input_report_abs(tsdata->input, ABS_Y, point_slot[i].posy);
					input_sync(tsdata->input);

				/*printk("slot=%d,x%d=%d,y%d=%d  ", i, i / 2, point_slot[i].posx,
						i / 2, point_slot[i].posy);*/
	
			}
		}
		#if 0
		input_report_key(tsdata->input, BTN_TOUCH, 1);
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 15);
		for (i = 0; i < MAX_FINGER_NUM * 2; i++) {
			if (point_slot[i].active == 1) {
				point_slot[i].active = 0;
				input_report_key(tsdata->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(tsdata->input, ABS_MT_POSITION_X,
						point_slot[i].posx);
				input_report_abs(tsdata->input, ABS_MT_POSITION_Y,
						point_slot[i].posy);
				input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 15);
				input_mt_sync(tsdata->input);

				/*printd("slot=%d,x%d=%d,y%d=%d  ", i, i / 2, point_slot[i].posx,
						i / 2, point_slot[i].posy);*/
	
			}
		}
		//printd("\n");
		#endif

	} else {
		input_report_key(tsdata->input, BTN_TOUCH, 0);
		//input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(tsdata->input, ABS_PRESSURE, 0);
	}
	input_sync(tsdata->input);

#ifdef QCOM
	gtp_irq_enable(tsdata);
#endif
}

static irqreturn_t pixcir_ts_isr(int irq, void *dev_id) {
	struct pixcir_i2c_ts_data *tsdata = dev_id;

#ifdef QCOM
	gtp_irq_disable(tsdata);
#endif

	//printd("enter pixcir_ts_isr.work_pending=%d\n", work_pending);
	if (work_pending == 1) { // common
		queue_work(pixcir_wq, &tsdata->work.work);
	}
	else if (work_pending == 2) { // bootloader
		bootloader_irq = 1;
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static struct pixcir_ts_platform_data *pixcir_parse_dt(struct device *dev)
{	
	struct device_node *np = dev->of_node;
	struct pixcir_ts_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->gpio_attb = of_get_named_gpio(np, "attb-gpio", 0);
	global_gpio_attb = pdata->gpio_attb;
	
	pdata->gpio_rst = of_get_named_gpio(np, "rst-gpio", 0);
	global_gpio_rst = pdata->gpio_rst;

	if (of_property_read_u32(np, "touchscreen-size-x", &pdata->x_max)) {
		dev_err(dev, "can not to get touchscreen-size-x property\n");
		pdata->x_max = DEFAULT_X_MAX;
	}
	pdata->x_max -= 1;

	if (of_property_read_u32(np, "touchscreen-size-y", &pdata->y_max)) {
		dev_err(dev, "can not to get touchscreen-size-y property\n");
		pdata->y_max = DEFAULT_Y_MAX;
	}
	pdata->y_max -= 1;

    return pdata;
}
#endif

#if GTP_AUTO_UPDATE
#define pixcir_update_PROC_FILE "driver/pixcir_update"
ssize_t pixcir_update_proc_write(struct file *file_chip, const char __user *buf,
						size_t len, loff_t *data)
{
	int temp = -1;
	char messages[10];
	struct pixcir_i2c_ts_data *ts = NULL;

	 if(!i2c_connect_client){
        printk(KERN_ERR "%s:I2C client is NULL!\n", __func__);
        return -1;
    }

    ts = i2c_get_clientdata(i2c_connect_client);
    if(!ts){
        printk(KERN_ERR "%s:pixcir i2c data is NULL!\n", __func__);
        return -1;
    }

	if(len > 10)
		len = 10;

	if (copy_from_user(messages, buf, len))
	{
		pr_err("copy from bus failed!\n");
		return -EFAULT;
	}

	temp = (int)simple_strtol(messages, NULL, 10);
    GTP_DEBUG("%s obtain number=%d",__FUNCTION__,temp);
	if(temp == 1){
		gup_init_update_proc(ts); //
	}

	return len;
}


static const struct file_operations pixcir_update_fops = {
	.owner = THIS_MODULE,	
	.write = pixcir_update_proc_write,
	//.read = seq_read,
};

static void proc_inode_create_update(void){
	struct proc_dir_entry *pixcir_update = NULL;
	pixcir_update = proc_create(pixcir_update_PROC_FILE, 0666, NULL, &pixcir_update_fops);
	if(!pixcir_update)
		pr_err("creat pixcir update proc inode failed!\n");
	return ;
}
#endif

#ifdef QCOM
static int pixcir_i2c_ts_probe(struct i2c_client *client,const struct i2c_device_id *id)
#else
static int __devinit pixcir_i2c_ts_probe(struct i2c_client *client,const struct i2c_device_id *id)
#endif
{
#ifdef QCOM
 struct device *dev = &client->dev;
#else
 struct device *dev = NULL;
#endif
	
#ifdef CONFIG_OF
    struct pixcir_ts_platform_data *pdata =
			dev_get_platdata(&client->dev);
    struct device_node *np = dev->of_node; //add by qiuguochang
#endif

	struct pixcir_i2c_ts_data *tsdata;
	struct input_dev *input;
	struct i2c_dev *i2c_dev;
	int i, error;
	int ret = -1;

   printk(KERN_ERR "gauss-------pixcir_i2c_ts_probe\n");
#ifdef CONFIG_OF
	if (np && !pdata) {
		pdata = pixcir_parse_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}
#endif

	pixcir_init();//reset pin set to 0 or 1 and platform init	

	for(i=0; i<MAX_FINGER_NUM*2; i++) {
		point_slot[i].active = 0;
	}

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	input = input_allocate_device();
	if (!tsdata || !input) {
		dev_err(&client->dev, "Failed to allocate driver data!\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	tsdata->client = client;
	tsdata->input = input;
	//tsdata->chip = pdata;
	tsdata->irq_is_disable = 0;
#ifdef CONFIG_OF
    pdata->irq = gloable_irq;
    tsdata->irq = pdata->irq;
    irq_num = pdata->irq;
#else
     tsdata->irq = client->irq;
	 client->irq = gpio_to_irq(config_info.irq_gpio.gpio);
     irq_num = client->irq;
#endif


	i2c_connect_client = client;

    spin_lock_init(&tsdata->irq_lock);
	INIT_WORK(&tsdata->work.work, pixcir_ts_poscheck);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	/*
	__set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
	__set_bit(ABS_MT_TRACKING_ID, input->absbit);
	__set_bit(ABS_MT_POSITION_X, input->absbit);
	__set_bit(ABS_MT_POSITION_Y, input->absbit);
	*/
	//__set_bit(ABS_PRESSURE, input->absbit);
	__set_bit(ABS_X, input->absbit);
	__set_bit(ABS_Y, input->absbit);
#if 0
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#ifdef CONFIG_OF
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
#else
    input_set_abs_params(input, ABS_MT_POSITION_X, 0, DEFAULT_X_MAX, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, DEFAULT_Y_MAX, 0, 0);
#endif
#endif
	input_set_abs_params(input, ABS_X, 0, DEFAULT_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, DEFAULT_Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 1000, 0, 0);

	input_set_drvdata(input, tsdata);

    error = request_irq(irq_num, pixcir_ts_isr,
			IRQF_TRIGGER_FALLING, client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}

#ifdef QCOM
	gtp_irq_disable(tsdata);
#else
	disable_irq_nosync(client->irq);
#endif

	error = input_register_device(input);
	if (error)
	goto err_free_irq;
    printk(KERN_ERR "gauss-------b24\n");
	i2c_set_clientdata(client, tsdata);
	device_init_wakeup(&client->dev, 1);

#if GTP_AUTO_UPDATE
    GTP_DEBUG("call gup_init_update_proc start");
    ret = gup_init_update_proc(tsdata);
    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif

	/*********************************Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}

	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,
					client->adapter->nr), NULL, "pixcir_i2c_ts%d", 0);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		return error;
	}
	/*********************************Bee-0928-BOTTOM****************************************/

#ifdef CONFIG_HAS_EARLYSUSPEND
	tsdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tsdata->early_suspend.suspend = pixcir_i2c_ts_early_suspend;
	tsdata->early_suspend.resume = pixcir_i2c_ts_late_resume;
	register_early_suspend(&tsdata->early_suspend);
#endif

	dev_err(&tsdata->client->dev, "insmod successfully!\n");

	work_pending = 1;
	irq_flag = 1;

#ifdef QCOM
	gtp_irq_enable(tsdata);
#else
     enable_irq(client->irq);
#endif	


	//create node for debug
#if GTP_AUTO_UPDATE
	proc_inode_create_update();
#endif

	return 0;

err_free_irq:
	free_irq(client->irq, tsdata);
err_free_mem:
	input_free_device(input);
	kfree(tsdata);
	return error;
}

#ifdef QCOM
static int pixcir_i2c_ts_remove(struct i2c_client *client)
#else
static int __devexit pixcir_i2c_ts_remove(struct i2c_client *client)
#endif
{
	int error;
	struct i2c_dev *i2c_dev;
	struct pixcir_i2c_ts_data *tsdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tsdata->early_suspend);
#endif

	device_init_wakeup(&client->dev, 0);

	mb();
    free_irq(irq_num, tsdata);

	/*********************************Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}

	return_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	/*********************************Bee-0928-BOTTOM****************************************/

	input_unregister_device(tsdata->input);
	kfree(tsdata);

	return 0;
}

/*************************************Bee-0928****************************************/
/*                        	     pixcir_open                                     */
/*************************************Bee-0928****************************************/
static int pixcir_open(struct inode *inode, struct file *file) {
	int subminor;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;
	int ret = 0;
       #if GTP_AUTO_UPDATE
	if(auto_update_flag){
		printk("auto update running, please waiting\n");
		return -ETXTBSY;
	}
       #endif 
		
	//printd("enter pixcir_open function\n");
	subminor = iminor(inode);

	//lock_kernel();
	i2c_dev = i2c_dev_get_by_minor(subminor);
	if (!i2c_dev) {
		printk("error i2c_dev\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter) {
		return -ENODEV;
	}

	client = kzalloc(sizeof(*client), GFP_KERNEL);

	if (!client) {
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}

	snprintf(client->name, I2C_NAME_SIZE, "pixcir_i2c_ts%d", adapter->nr);
	client->driver = &pixcir_i2c_ts_driver;
	client->adapter = adapter;

	file->private_data = client;

	return 0;
}

/*************************************Bee-0928****************************************/
/*                        	     pixcir_ioctl                                    */
/*************************************Bee-0928****************************************/
static long pixcir_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	struct i2c_client *client = (struct i2c_client *) file->private_data;

	//printd("pixcir_ioctl(),cmd = %d, arg = %ld\n", cmd, arg);

	switch (cmd) {
	case CALIBRATION_FLAG: //CALIBRATION_FLAG = 1
		client->addr = SLAVE_ADDR;
		status_reg = CALIBRATION_FLAG;
		break;

	case BOOTLOADER: //BOOTLOADER = 7
		printd("bootloader\n");
		client->addr = BOOTLOADER_ADDR;
		status_reg = BOOTLOADER;

		work_pending = 0;
		bootloader_irq = 0;
		pixcir_reset();
		mdelay(3);
		irq_flag = 1;
		//enable_irq(irq_num);
                  disable_irq_nosync(irq_num);
		work_pending = 2;

		break;

	case RESET_TP: //RESET_TP = 9
		printd("reset tp\n");
		pixcir_reset();
		work_pending = 1;
		break;

	case ENABLE_IRQ: //ENABLE_IRQ = 10
		status_reg = 0;
		if (irq_flag == 0) {
			irq_flag = 1;
			enable_irq(irq_num);
		}
		break;

	case DISABLE_IRQ: //DISABLE_IRQ = 11
		if (irq_flag == 1) {
			irq_flag = 0;
			disable_irq_nosync(irq_num);
		}
		break;

	case BOOTLOADER_STU: //BOOTLOADER_STU = 16
		client->addr = BOOTLOADER_ADDR;
		status_reg = BOOTLOADER_STU;

		//pixcir_reset();
		//mdelay(3);
		break;

	case ATTB_VALUE: //ATTB_VALUE = 17
		//printd("ATTB_VALUE\n");
		client->addr = SLAVE_ADDR;
		status_reg = ATTB_VALUE;
		break;

	default:
		client->addr = SLAVE_ADDR;
		status_reg = 0;
		break;
	}
	return 0;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_read                                      */
/***********************************Bee-0928****************************************/
static ssize_t pixcir_read (struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	unsigned char *tmp, bootloader_stu[4], attb_value[1];
	int ret = 0;

	switch(status_reg)
	{
		case BOOTLOADER_STU:
		i2c_master_recv(client, bootloader_stu, sizeof(bootloader_stu));
		if (ret!=sizeof(bootloader_stu)) {
			dev_err(&client->dev,
					"%s: BOOTLOADER_STU: i2c_master_recv() failed, ret=%d\n",
					__func__, ret);
			return -EFAULT;
		}

		if (copy_to_user(buf, bootloader_stu, sizeof(bootloader_stu))) {
			dev_err(&client->dev,
					"%s: BOOTLOADER_STU: copy_to_user() failed.\n", __func__);
			return -EFAULT;
		} else {
			ret = 4;
		}
		break;

		case ATTB_VALUE:
               
		attb_value[0] = attb_read_val();
		//printd("attb_read,%d\n",attb_value[0]); 
		if(copy_to_user(buf, attb_value, sizeof(attb_value))) {
			dev_err(&client->dev,
					"%s: ATTB_VALUE: copy_to_user() failed.\n", __func__);
			return -EFAULT;
		} else {
			ret = 1;
		}
		break;

		default:
		tmp = kmalloc(count, GFP_KERNEL);
		if (tmp==NULL)
		return -ENOMEM;

		ret = i2c_master_recv(client, tmp, count);
		if (ret != count) {
			dev_err(&client->dev,
					"%s: default: i2c_master_recv() failed, ret=%d\n",
					__func__, ret);
			return -EFAULT;
		}
		//printd(">>>>>>>>>>%s:default:%d,%d\n",__func__, tmp[0], tmp[1]);
		if(copy_to_user(buf, tmp, count)) {
			dev_err(&client->dev,
					"%s: default: copy_to_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_write                                     */
/***********************************Bee-0928****************************************/
static ssize_t pixcir_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct i2c_client *client;
	unsigned char *tmp, bootload_data[143];
	int ret = 0;
	client = file->private_data;

	switch(status_reg)
	{
		case CALIBRATION_FLAG: //CALIBRATION_FLAG=1
		tmp = kmalloc(count, GFP_KERNEL);
		if (tmp == NULL)
		return -ENOMEM;

		if (copy_from_user(tmp, buf, count)) {
			dev_err(&client->dev,
					"%s: CALIBRATION_FLAG: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}

		ret = i2c_master_send(client, tmp, count);
		if (ret != count) {
			dev_err(&client->dev,
					"%s: CALIBRATION: i2c_master_send() failed, ret=%d\n",
					__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}

		kfree(tmp);
		break;

		case BOOTLOADER:
		memset(bootload_data, 0, sizeof(bootload_data));

		if (copy_from_user(bootload_data, buf, count)) {
			dev_err(&client->dev,
					"%s: BOOTLOADER: copy_from_user() failed.\n", __func__);
			return -EFAULT;
		}

		ret = i2c_master_send(client, bootload_data, count);
		if (ret != count) {
			printk("%s----->>>>%x\n", __func__, client->addr);
			dev_err(&client->dev,
					"%s: BOOTLOADER: i2c_master_send() failed, ret = %d\n",
					__func__, ret);
			return -EFAULT;
		} 
		judge_int_gpio_status();
        /*
		time_out = 0;
		while(bootloader_irq==0) {
			if(time_out > bootloader_delay) break;
			else {
				time_out++;
				udelay(BTIME);
			}
		}
		bootloader_irq = 0;
                 */
               
		//printd("time_out = %d ms\n", time_out/(1000/BTIME));
		break;

		default:
		tmp = kmalloc(count, GFP_KERNEL);
		if (tmp == NULL)
		return -ENOMEM;

		if (copy_from_user(tmp, buf, count)) {
			dev_err(&client->dev,
					"%s: default: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}
		//printd(">>>>>>>>>>%s:default:%d,%d\n",__func__, tmp[0], tmp[1]);
		ret = i2c_master_send(client,tmp,count);
		if (ret != count) {
			dev_err(&client->dev,
					"%s: default: i2c_master_send() failed, ret=%d\n",
					__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_release                                   */
/***********************************Bee-0928****************************************/
static int pixcir_release(struct inode *inode, struct file *file) {
	struct i2c_client *client = file->private_data;

	//printd("enter pixcir_release funtion\n");
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

/*********************************Bee-0928-TOP****************************************/
static const struct file_operations pixcir_i2c_ts_fops = {
	.owner = THIS_MODULE,
	.open = pixcir_open,
	.unlocked_ioctl = pixcir_ioctl,
	.read = pixcir_read,
	.write = pixcir_write,
	.release = pixcir_release, };
/*********************************Bee-0928-BOTTOM****************************************/

#ifdef QCOM
static int pixcir_i2c_ts_suspend(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);
#else
static int pixcir_i2c_ts_suspend(struct i2c_client *client) {
#endif
	struct pixcir_i2c_ts_data *tsdata = i2c_get_clientdata(client);
	unsigned char wrbuf[2] = { 0 };
	int ret;
	//printk("------------------pixcir TP early suspend--------------------\n");
	wrbuf[0] = 0x33;
	wrbuf[1] = 0x03; //enter into Sleep mode;
	/**************************************************************
	 wrbuf[1]:	0x00: Active mode

	 0x01: Sleep mode
	 0xA4: Sleep mode automatically switch
	 0x03: Freeze mode
	 More details see application note 710 power manangement section
	 ****************************************************************/
	ret = i2c_master_send(tsdata->client, wrbuf, 2);
	if (ret != 2) {
		dev_err(&tsdata->client->dev, "%s: i2c_master_send failed(), ret=%d\n",
				__func__, ret);
	}

	if (device_may_wakeup(&tsdata->client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

#ifdef QCOM
static int pixcir_i2c_ts_resume(struct device *dev) {	
	struct i2c_client *client = to_i2c_client(dev);
#else
static int pixcir_i2c_ts_resume(struct i2c_client *client) {
#endif
	struct pixcir_i2c_ts_data *tsdata = i2c_get_clientdata(client);
#if 1
	pixcir_reset();
#else
	unsigned char wrbuf[2] = { 0 };
	int ret;

	wrbuf[0] = 0x33;
	wrbuf[1] = 0;
	ret = i2c_master_send(tsdata->client, wrbuf, 2);
	if (ret != 2) {
		dev_err(&tsdata->client->dev, "%s: i2c_master_send failed(), ret=%d\n",
				__func__, ret);
	}
#endif
	if (device_may_wakeup(&tsdata->client->dev))
		disable_irq_wake(tsdata->irq);
	//printk("pixcir resume: disable_irq_wake\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_i2c_ts_early_suspend(struct early_suspend *h)
{
	struct pixcir_i2c_ts_data *tsdata;
	tsdata = container_of(h, struct pixcir_i2c_ts_data, early_suspend);
	pixcir_i2c_ts_suspend(tsdata->client);
}

static void pixcir_i2c_ts_late_resume(struct early_suspend *h)
{
	struct pixcir_i2c_ts_data *tsdata;
	tsdata = container_of(h, struct pixcir_i2c_ts_data, early_suspend);
	pixcir_i2c_ts_resume(tsdata->client);
}
#endif

#ifdef QCOM
#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(pixcir_dev_pm_ops,
			 pixcir_i2c_ts_suspend, pixcir_i2c_ts_resume);
#endif
#endif

#ifdef CONFIG_OF
static const struct of_device_id pixcir_match_table[] = {
		{.compatible = "pixcir_i2c_ts_v3.6.09",},
		{ },
};
#endif

static const struct i2c_device_id pixcir_i2c_ts_id[] =
//		{ { "pixcir,pixcir_ts", 0 }, { } };
                { { "pixcir-i2c-ts", 0 }, { } };
MODULE_DEVICE_TABLE( i2c, pixcir_i2c_ts_id);

static struct i2c_driver pixcir_i2c_ts_driver = { 
		.driver = { 
			.owner =THIS_MODULE, 
			.name = "pixcir_i2c_ts_v3.6.09", 
#ifdef CONFIG_OF
        	.of_match_table = pixcir_match_table,
#endif
#ifdef QCOM
        #ifdef CONFIG_PM_SLEEP
			.pm	= &pixcir_dev_pm_ops,
		#endif
#endif
		},
#ifndef QCOM
#ifndef CONFIG_HAS_EARLYSUSPEND
        .suspend = pixcir_i2c_ts_suspend,
        .resume = pixcir_i2c_ts_resume,
#endif
#endif
		.probe = pixcir_i2c_ts_probe, 
#ifdef QCOM
		.remove = pixcir_i2c_ts_remove,
#else
        .remove = __devexit_p(pixcir_i2c_ts_remove),
#endif
		.id_table = pixcir_i2c_ts_id, 
};

static int __init pixcir_i2c_ts_init(void)
{
	int ret;
	pixcir_wq = create_singlethread_workqueue("pixcir_wq");
	if (!pixcir_wq)
	return -ENOMEM;
	if (input_fetch_sysconfig_para(&(config_info.input_type))) {
		pr_err("%s: ctp_fetch_sysconfig_para err.\n", __func__);
		return 0;
	} else {
		ret = input_init_platform_resource(&(config_info.input_type));
		if (0 != ret)
			pr_err("%s:ctp_ops.init_platform_resource err.\n",
								__func__);
	}
	/*********************************Bee-0928-TOP****************************************/
	ret = register_chrdev(I2C_MAJOR, "pixcir_i2c_ts", &pixcir_i2c_ts_fops);
	if (ret) {
		printk(KERN_ERR "%s:register chrdev failed\n", __FILE__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "pixcir_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	/********************************Bee-0928-BOTTOM******************************************/

	return i2c_add_driver(&pixcir_i2c_ts_driver);
}
module_init( pixcir_i2c_ts_init);

static void __exit pixcir_i2c_ts_exit(void)
{
	i2c_del_driver(&pixcir_i2c_ts_driver);
	/********************************Bee-0928-TOP******************************************/
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"pixcir_i2c_ts");
	/********************************Bee-0928-BOTTOM******************************************/
	if(pixcir_wq)
	destroy_workqueue(pixcir_wq);
}
module_exit( pixcir_i2c_ts_exit);

MODULE_AUTHOR("Jianchun Bian <jcbian@pixcir.com.cn>");
MODULE_DESCRIPTION("Pixcir I2C Touchscreen Driver");
MODULE_LICENSE("GPL");

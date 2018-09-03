/*
 * (C) Copyright 2007-2013
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Char <yanjianbo@allwinnertech.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/arch/timer.h>
#include <sys_config.h>
#include <asm/arch/cpu.h>


struct timer_list TIMER0;
static int   sprite_led_status;
static __u32 sprite_led_hd;

/*
************************************************************************************************************
*
*                                             function
*
*    �������ƣ�
*
*    �����б���
*
*    ����ֵ  ��
*
*    ˵��    ��
*
*
************************************************************************************************************
*/
static void sprite_timer_func(void *p)
{
	gpio_write_one_pin_value(sprite_led_hd, sprite_led_status, "sprite_gpio0");
	sprite_led_status = (~sprite_led_status) & 0x01;

	//printf("sprite_time_func\n");
	del_timer(&TIMER0);
	add_timer(&TIMER0);
	return;
}

/*
************************************************************************************************************
*
*                                             function
*
*    �������ƣ�
*
*    �����б���
*
*    ����ֵ  ��
*
*    ˵��    ��
*
*
************************************************************************************************************
*/
int sprite_led_init(void)
{
	user_gpio_set_t	gpio_init;
	int	ret;

	sprite_led_status = 1;

	printf("try sprite_led_gpio config\n");
	memset(&gpio_init, 0, sizeof(user_gpio_set_t));
	//�������gpio��
	ret = script_parser_fetch("card_boot", "sprite_gpio0", (void *)&gpio_init, sizeof(user_gpio_set_t)>>2);
	if(!ret)
	{
		if(gpio_init.port)
		{
			sprite_led_hd = gpio_request(&gpio_init, 1);
			if(!sprite_led_hd)
			{
				printf("reuqest gpio for led failed\n");
				return 1;
			}
			printf("sprite_led_gpio start sprite_led_hd %d\n", sprite_led_hd);

			/*set gpio output*/
			gpio_set_one_pin_io_status(sprite_led_hd, 1, "sprite_gpio0");
			/*set 1 to gpio data*/
			gpio_write_one_pin_value(sprite_led_hd, sprite_led_status, "sprite_gpio0");

			return 0;
		}
	}
	return 0;
}

/*
************************************************************************************************************
*
*                                             function
*
*    �������ƣ�
*
*    �����б���
*
*    ����ֵ  ��
*
*    ˵��    ��
*
*
************************************************************************************************************
*/
int sprite_led_exit(int status)
{
	int ret;
	int delay;


	//������ʱ��led����˸�ӿ�
	if(status < 0)
	{
		ret = script_parser_fetch("card_boot", "sprite_err_delay", (void *)&delay, 1);
		if((ret) || (!delay))
		{
			delay = 100;
		}
		del_timer(&TIMER0);
		TIMER0.data = (unsigned long)&TIMER0;
		TIMER0.expires = delay;
		TIMER0.function = sprite_timer_func;
		//init_timer(&TIMER0);
		add_timer(&TIMER0);
	}

	return 0;
}

static inline void mdelay(unsigned long msec)
{
	unsigned long i;
	for (i = 0; i < msec; i++)
		udelay(1000);
}

void lierda_led_enable(void)
{
	gpio_write_one_pin_value(sprite_led_hd, 0, "sprite_gpio0");
}

int sunxi_flashing_led(void)
{
	int i;
	int delay = 200;
	for(i = 0; i < 10;i++)
	{
		gpio_write_one_pin_value(sprite_led_hd, 1, "sprite_gpio0");
		mdelay(delay);
		gpio_write_one_pin_value(sprite_led_hd, 0, "sprite_gpio0");
		mdelay(delay);
	}

	return 0;
}
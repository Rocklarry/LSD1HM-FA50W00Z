obj-m += pixcir_i2c_ts_v36.o
obj-m += pixcir_i2c_update.o
obj-m += uart_gpio.o
obj-m += lierda_beep.o
obj-y += gt9xx.o gt9xx_update.o goodix_tool.o
obj-y += led_gpio.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DHALL_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

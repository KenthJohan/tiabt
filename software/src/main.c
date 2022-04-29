#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <sys/__assert.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/pinctrl.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "adc.h"
#include "bt.h"




void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);
	
	init_adc();

	//gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	//gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	//gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
	/*
	gpio_pin_toggle_dt(&led1);
	gpio_pin_toggle_dt(&led2);
	gpio_pin_toggle_dt(&led3);
	*/
	
	while (1)
	{
		k_sleep(K_MSEC(1000));
		if (simulate_temp)
		{
			ess_simulate();
		}
		bas_notify();
	}
}

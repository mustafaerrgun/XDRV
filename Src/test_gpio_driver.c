/**
 ******************************************************************************
 * @file           : test_gpio_driver.c
 * @brief          : Test file for XDR GPIO Driver
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_gpio.h"

xdr_gpio led;

int main(void)
{
	led.xdr_gpiox 		 = GPIOB;
	led.xdr_gpio_portId  = XDR_GPIO_PORT_B;
	led.xdr_gpio_pin     = GPIO_PIN_NO_7;
	led.xdr_gpio_pinMode = GPIO_MODE_OUTPUT;
	led.xdr_gpio_pinPuPd = GPIO_NO_PUPD;

	XDR_GPIO_Init(&led);

	XDR_GPIO_Write_Pin(&led, led.xdr_gpio_pin, GPIO_PIN_SET);

	while(1){

		XDR_GPIO_Toggle(&led, led.xdr_gpio_pin);
		for (uint32_t i = 0; i < 1000000; i++) {}
	}
	return 0 ;
}


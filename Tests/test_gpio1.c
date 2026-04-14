/**
 ******************************************************************************
 * @file           : test_gpio1.c
 * @brief          : Test file for XDR GPIO Driver
 ******************************************************************************
 */

#include "xdr_gpio.h"
#include "xdr_systick.h"

xdr_gpio led;

void run_test(void)
{
	led.xdr_gpiox 		 = GPIOB;
	led.xdr_gpio_portId  = XDR_GPIO_PORT_B;
	led.xdr_gpio_pin     = GPIO_PIN_NO_7;
	led.xdr_gpio_pinMode = GPIO_MODE_OUTPUT;
	led.xdr_gpio_pinPuPd = GPIO_NO_PUPD;
	led.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

	XDR_GPIO_Init(&led);

	XDR_GPIO_Write_Pin(&led, led.xdr_gpio_pin, GPIO_PIN_SET);

	while(1){

		XDR_GPIO_Toggle(&led, led.xdr_gpio_pin);
		/*Delay for 2000ms*/
		XDR_SysTick_Delay(2000);
	}
}


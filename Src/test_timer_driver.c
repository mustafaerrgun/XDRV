/**
 ******************************************************************************
 * @file           : test_gpio_driver.c
 * @brief          : Test file for XDR TIM Driver
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_gpio.h"
#include "stm32f767xx_xdr_tim.h"

xdr_gpio led;
xdr_tim  tim;

int main(void)
{
	tim.tim_id = XDR_TIM_2;
	tim.presc = 16000U;
	tim.period = 1000U;

	XDR_TIM_Init(&tim);

	led.xdr_gpiox 		  = GPIOB;
	led.xdr_gpio_portId   = XDR_GPIO_PORT_B;
	led.xdr_gpio_pin      = GPIO_PIN_NO_7;
	led.xdr_gpio_pinMode  = GPIO_MODE_OUTPUT;
	led.xdr_gpio_pinPuPd  = GPIO_NO_PUPD;
	led.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

	XDR_GPIO_Init(&led);

	XDR_GPIO_Write_Pin(&led, led.xdr_gpio_pin, GPIO_PIN_SET);

	while(1){

		XDR_GPIO_Toggle(&led, led.xdr_gpio_pin);
        /*Wait 1 second period for Update Event(UEV) */
		XDR_TIM_WaitUpdate(&tim);

	}
	return 0 ;
}


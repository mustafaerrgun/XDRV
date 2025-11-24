/**
 ******************************************************************************
 * @file           : LED_Toggling.c
 * @brief          : Test file for XDR GPIO Driver
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_gpio.h"

void LED_Init(void);
void LED_Toggle(void);

XDR_GPIO_Handle Led;

int main(void)
{
	LED_Init();

	while(1){

		LED_Toggle();
	}
	return 0 ;
}

void LED_Init(void)
{
	Led.XDR_GPIOx = GPIOB;
	Led.XDR_PortId = XDR_GPIO_PORT_B;
	Led.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_7 ;
	Led.XDR_GPIO_Config.XDR_GPIO_PinMode = GPIO_MODE_OUTPUT;
	Led.XDR_GPIO_Config.XDR_GPIO_PinOType = GPIO_OP_TYPE_PP;
	Led.XDR_GPIO_Config.XDR_GPIO_PinPUPDType = GPIO_NO_PUPD;
	Led.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 0;
	XDR_GPIO_Init(&Led);

	XDR_GPIO_Write_Pin(Led.XDR_GPIOx, Led.XDR_GPIO_Config.XDR_GPIO_Pin, GPIO_PIN_SET);
}

void LED_Toggle(void)
{
	XDR_GPIO_Toggle(Led.XDR_GPIOx, Led.XDR_GPIO_Config.XDR_GPIO_Pin);
	for (uint32_t i = 0; i < 1000000; i++) {}
}


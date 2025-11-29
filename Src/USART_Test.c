/**
 ******************************************************************************
 * @file           : USART_Test.c
 * @brief          : Test XDR USART Driver module
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_usart.h"
#include "stm32f767xx_xdr_rcc.h"
#include "stm32f767xx_xdr_gpio.h"

void SysClk_Config(void);
void USART_Config(void);
void GPIO_Config(void);

XDR_RCC_Handle rccHandle;
XDR_USART_Handle usartHandle;
XDR_GPIO_Handle usart_gpio;

int main(void)
{
	SysClk_Config();

	GPIO_Config();

	USART_Config();


	while(1){

        XDR_USART_Send(&usartHandle, 'A');
        XDR_USART_Send(&usartHandle, '\r');
        XDR_USART_Send(&usartHandle, '\n');

        for (volatile uint32_t i = 0; i < 1000000UL; ++i)
        {
            /* crude delay */
        }
	}
	return 0 ;
}

/* ABP1 clock	*/
void SysClk_Config(void){

	rccHandle.XDR_RCC = RCC;
	rccHandle.XDR_RCC_Config.ClockSource = XDR_HSI_CLOCK;
	rccHandle.XDR_RCC_Config.SYSCLK_Freq = XDR_SYSCLK_48MHZ;
	rccHandle.XDR_RCC_Config.AHB_Prescaler = XDR_AHB_DIV8 ;
	rccHandle.XDR_RCC_Config.APB1_Prescaler = XDR_APB_DIV2;
	rccHandle.XDR_RCC_Config.APB2_Prescaler = XDR_APB_DIV2;

    XDR_RCC_Init(&rccHandle);
}

// GPIO Configuration for USART
void GPIO_Config(void){

    usart_gpio.XDR_GPIOx  = GPIOD;
    usart_gpio.XDR_PortId = XDR_GPIO_PORT_D;

	/// GPIO pin PD8 configured for USART
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_Pin        = GPIO_PIN_NO_8;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinMode    = GPIO_MODE_ALTFN;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinOSpeed  = GPIO_SPEED_FAST;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinOType   = GPIO_OP_TYPE_PP;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinPUPDType = GPIO_NO_PUPD;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinAFMode  = 7;

	XDR_GPIO_Init(&usart_gpio);

	// GPIO pin PD9 configured for USART
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_Pin        = GPIO_PIN_NO_9;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinMode    = GPIO_MODE_ALTFN;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinOSpeed  = GPIO_SPEED_FAST;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinOType   = GPIO_OP_TYPE_PP;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinPUPDType= GPIO_PIN_PU;
	usart_gpio.XDR_GPIO_Config.XDR_GPIO_PinAFMode  = 7;

	XDR_GPIO_Init(&usart_gpio);
}

// USART Configurations
void USART_Config(void){

	usartHandle.XDR_RCC_Handle = &rccHandle;
	usartHandle.XDR_USART_Instance = XDR_USART3;
	usartHandle.XDR_USART_Config.XDR_USART_BaudRate = XDR_USART_BAUD_9600;
	usartHandle.XDR_USART_Config.XDR_USART_Mode = XDR_USART_MODE_TX_RX;
	usartHandle.XDR_USART_Config.XDR_USART_WordLength  = XDR_USART_WORDLENGTH_8B;

    XDR_USART_Init(&usartHandle);
}

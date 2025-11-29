/**
 ******************************************************************************
 * @file           : USART_Test.c
 * @brief          : Test XDR USART Driver module
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_usart.h"
#include "stm32f767xx_xdr_rcc.h"

void SysClk_Config(void);
void USART_Config(void);

XDR_RCC_Handle rccHandle;
xdr_usart usart;

int main(void)
{
	SysClk_Config();

	USART_Config();


	while(1){

        XDR_USART_Send(&usart, 'A');
        XDR_USART_Send(&usart, '\r');
        XDR_USART_Send(&usart, '\n');

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
	rccHandle.XDR_RCC_Config.SYSCLK_Freq = XDR_SYSCLK_16MHZ;
	rccHandle.XDR_RCC_Config.AHB_Prescaler = XDR_AHB_DIV4 ;
	rccHandle.XDR_RCC_Config.APB1_Prescaler = XDR_APB_DIV2;
	rccHandle.XDR_RCC_Config.APB2_Prescaler = XDR_APB_DIV2;

    XDR_RCC_Init(&rccHandle);
}

// USART Configurations
void USART_Config(void){

	usart.XDR_RCC_Handle = &rccHandle;
	usart.xdr_usart_instance = XDR_USART3;
	usart.xdr_usart_baudrate = XDR_USART_BAUD_9600;

    XDR_USART_Init(&usart);
}

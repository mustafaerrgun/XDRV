/**
 ******************************************************************************
 * @file           : test_usart_driver_send.c
 * @brief          : Test XDR USART Driver send API
 ******************************************************************************
 */

#include "stm32f767xx_xdr_usart.h"

int main(void)
{
	xdr_usart usart;
	usart.xdr_usart_instance = XDR_USART3;
	usart.xdr_usart_baudrate = XDR_USART_BAUD_9600;

    XDR_USART_Init(&usart);

	while(1){

        XDR_USART_Send(&usart, 'A');
        XDR_USART_Send(&usart, '\r');
        XDR_USART_Send(&usart, '\n');

        for (volatile uint32_t i = 0; i < 1000000UL; ++i){ }
	}
	return 0 ;
}

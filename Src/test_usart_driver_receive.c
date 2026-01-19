/**
 ******************************************************************************
 * @file           : test_usart_driver_receive.c
 * @brief          : Test XDR USART Driver receive API
 ******************************************************************************
 */

#include "stm32f767xx_xdr_usart.h"
#include "stm32f767xx_xdr_systick.h"

int main(void)
{
	xdr_usart usart;
	usart.xdr_usart_instance = XDR_USART3;
	usart.xdr_usart_baudrate = XDR_USART_BAUD_9600;

    XDR_USART_Init(&usart);

    uint8_t received = 0;

	while(1){

		received = XDR_USART_Receive(&usart);

		if(received == 'B')
		{
			XDR_USART_Send(&usart, 'A');
			XDR_USART_Send(&usart, '\r');
			XDR_USART_Send(&usart, '\n');

			/*Delay for 2000ms*/
			XDR_SysTick_Delay(2000);

			received = 0;
		}
	}

	return 0 ;
}

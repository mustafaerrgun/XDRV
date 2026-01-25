/**
 ******************************************************************************
 * @file           : test_usart_driver_receive.c
 * @brief          : Test XDR USART Driver RX Interrupt
 ******************************************************************************
 */

#include "stm32f767xx_xdr_usart.h"
#include "stm32f767xx_xdr_systick.h"

xdr_usart usart;

int main(void)
{
	usart.xdr_usart_instance = XDR_USART3;
	usart.xdr_usart_baudrate = XDR_USART_BAUD_9600;

    XDR_USART_Init(&usart);

    XDR_USART3_EnableRxInterrupt(&usart);

	while(1){

	}

	return 0 ;
}

void XDR_USART3_RxCallback(uint8_t data){

	if(data == 'B')
	{
		XDR_USART_Send(&usart, 'A');
		XDR_USART_Send(&usart, '\r');
		XDR_USART_Send(&usart, '\n');
	}

}

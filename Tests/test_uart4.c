/**
 ******************************************************************************
 * @file           : test_uart4.c
 * @brief          : Test XDR UART Driver receive API
 ******************************************************************************
 */

#include "xdr_uart.h"
#include "xdr_systick.h"

void run_test(void)
{
	xdr_uart uart_test;
	uart_test.xdr_uart_instance = XDR_UART3;
	uart_test.xdr_uart_baudrate = XDR_UART_BAUD_9600;

    XDR_USART_Init(&uart_test);

    uint8_t received = 0;

	while(1){

		received = XDR_USART_Receive(&uart_test);

		if(received == 'B')
		{
			XDR_USART_Send(&uart_test, 'A');
			XDR_USART_Send(&uart_test, '\r');
			XDR_USART_Send(&uart_test, '\n');

			/*Delay for 2000ms*/
			XDR_SysTick_Delay(2000);

			received = 0;
		}
	}
}

/**
 ******************************************************************************
 * @file           : test_uart2.c
 * @brief          : Test XDR UART Driver send API
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

	while(1){

        XDR_UART_Send(&uart_test, 'A');
        XDR_UART_Send(&uart_test, '\r');
        XDR_UART_Send(&uart_test, '\n');
        /*Delay for 2000ms*/
        XDR_SysTick_Delay(2000);
	}
}

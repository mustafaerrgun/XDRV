/**
 ******************************************************************************
 * @file           : test_uart1.c
 * @brief          : Test XDR UART Driver module
 ******************************************************************************
 */

#include "xdr_uart.h"

void run_test(void)
{
	xdr_uart uart_test;
	uart_test.xdr_uart_instance = XDR_UART3;
	uart_test.xdr_uart_baudrate = XDR_UART_BAUD_9600;

    XDR_UART_Init(&uart_test);

	while(1){

        XDR_UART_Send(&uart_test, 'A');
        XDR_UART_Send(&uart_test, '\r');
        XDR_UART_Send(&uart_test, '\n');

        for (volatile uint32_t i = 0; i < 1000000UL; ++i){ }
	}
}

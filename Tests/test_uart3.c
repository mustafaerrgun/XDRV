/**
 ******************************************************************************
 * @file           : test_uart3.c
 * @brief          : Test XDR UART Driver RX Interrupt
 ******************************************************************************
 */

#include "xdr_uart.h"
#include "xdr_systick.h"

xdr_uart uart_test;

void run_test(void)
{
	uart_test.xdr_uart_instance = XDR_UART3;
	uart_test.xdr_uart_baudrate = XDR_UART_BAUD_9600;

    XDR_UART_Init(&uart_test);

    XDR_UART3_EnableRxInterrupt(&uart_test);

	while(1){

	}
}

void XDR_UART3_RxCallback(uint8_t data){

	if(data == 'B')
	{
		XDR_UART_Send(&uart_test, 'A');
		XDR_UART_Send(&uart_test, '\r');
		XDR_UART_Send(&uart_test, '\n');
	}

}

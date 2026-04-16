/**
 ******************************************************************************
 * @file           : test_uart1.c
 * @brief          : Test XDR UART Driver module
 ******************************************************************************
 */

#include "xdr_uart.h"
#include "xdr_systick.h"

void run_test(void)
{
	xdr_uart uart_test;
	uart_test.xdr_uart_instance = XDR_UART3;
	uart_test.xdr_uart_baudrate = XDR_UART_BAUD_9600;

    UART_Init(&uart_test);

	while(1){

        UART_SendString(&uart_test, "Mustafa\r\n");

        /*Delay for 2000ms*/
        XDR_SysTick_Delay(2000);
	}
}
    
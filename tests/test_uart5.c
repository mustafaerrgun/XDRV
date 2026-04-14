/**
 ******************************************************************************
 * @file           : test_uart5.c
 * @brief          : Test XDR UART with DMA
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "xdr_uart.h"

xdr_uart uart_test;

volatile uint8_t g_rx_cmplt;
volatile uint8_t g_tx_cmplt;
char msg_buff[150] ={'\0'};

void run_test(void)
{
	uart_test.xdr_uart_instance = XDR_UART3;
	uart_test.xdr_uart_baudrate = XDR_UART_BAUD_9600;
	uart_test.xdr_uart_interrupt = 0U;
	uart_test.xdr_uart_dma = 1U;

    XDR_USART_Init(&uart_test);

    XDR_UART3_DMA_Rx_Config();
    sprintf(msg_buff,"Initialization...cmplt\n\r");
    XDR_UART3_DMA_Tx_Config((uint32_t)msg_buff,strlen(msg_buff));
    while(!g_tx_cmplt){}

	while(1){

        if(g_rx_cmplt)
        {
            sprintf(msg_buff, "Message received : %s \r\n",UART3_Data_Buffer);
            g_rx_cmplt = 0;
            g_tx_cmplt = 0;
            XDR_UART3_DMA_Tx_Config((uint32_t)msg_buff,strlen(msg_buff));
            while(!g_tx_cmplt){}
        }

	}
}

void XDR_UART3_DMA_Rx_Callback(void){
	g_rx_cmplt = 1;
}

void XDR_UART3_DMA_Tx_Callback(void){
	g_tx_cmplt = 1;
}


/**
 ******************************************************************************
 * @file           : test_usart_driver_receive.c
 * @brief          : Test XDR USART with DMA
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "stm32f767xx_xdr_usart.h"

xdr_usart usart;

volatile uint8_t g_rx_cmplt;
volatile uint8_t g_tx_cmplt;
char msg_buff[150] ={'\0'};

int main(void)
{
	usart.xdr_usart_instance = XDR_USART3;
	usart.xdr_usart_baudrate = XDR_USART_BAUD_9600;
	usart.xdr_usart_interrupt = 0U;
	usart.xdr_usart_dma = 1U;

    XDR_USART_Init(&usart);

    XDR_USART3_DMA_Rx_Config();
    sprintf(msg_buff,"Initialization...cmplt\n\r");
    XDR_USART3_DMA_Tx_Config((uint32_t)msg_buff,strlen(msg_buff));
    while(!g_tx_cmplt){}

	while(1){

        if(g_rx_cmplt)
        {
            sprintf(msg_buff, "Message received : %s \r\n",USART3_Data_Buffer);
            g_rx_cmplt = 0;
            g_tx_cmplt = 0;
            XDR_USART3_DMA_Tx_Config((uint32_t)msg_buff,strlen(msg_buff));
            while(!g_tx_cmplt){}
        }

	}

	return 0 ;
}

void XDR_USART3_DMA_Rx_Callback(void){
	g_rx_cmplt = 1;
}

void XDR_USART3_DMA_Tx_Callback(void){
	g_tx_cmplt = 1;
}


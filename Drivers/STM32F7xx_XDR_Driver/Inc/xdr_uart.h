/**
  ******************************************************************************
  * @file    xdr_uart.h
  * @author  Mustafa Ergün
  * @brief   Header file of UART XDR module.
  ******************************************************************************
  */

#ifndef XDR_UART_H_
#define XDR_UART_H_

#include "stm32f767xx.h"
#include "xdr_rcc.h"
#include "xdr_gpio.h"

typedef enum
{
	XDR_UART1,
	XDR_UART2,
	XDR_UART3,
	XDR_UART6
}xdr_uart_instance;

typedef struct{
	USART_TypeDef 		*uart;
	xdr_uart_instance	 xdr_uart_instance;
	uint32_t 			 xdr_uart_baudrate;
	uint8_t				 xdr_uart_interrupt;
	uint8_t				 xdr_uart_dma;
}xdr_uart;

// Driver APIs
void XDR_UART_Init(xdr_uart *xdr_uart);
void XDR_UART_Send(xdr_uart *xdr_uart, uint8_t data);
uint8_t XDR_UART_Receive(xdr_uart *xdr_uart);
// Driver Interrupt APIs
void UART3_IRQHandler(void);
void XDR_UART3_Callback(uint8_t data);
// DMA Macros, variables and functions for UART3
#define UART_BUFF_SIZE		10U
extern char UART3_Data_Buffer[UART_BUFF_SIZE];
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void XDR_UART3_DMA_Rx_Config(void);
void XDR_UART3_DMA_Tx_Config(uint32_t data, uint32_t length);
void XDR_UART3_DMA_Rx_Callback(void);
void XDR_UART3_DMA_Tx_Callback(void);


__attribute__((weak)) void XDR_UART3_DMA_Rx_Callback(void) { }
__attribute__((weak)) void XDR_UART3_DMA_Tx_Callback(void) { }
__attribute__((weak)) void XDR_UART3_RxCallback(uint8_t data) { (void)data; }


// Macros for UART CR1
#define XDR_UART_CR1_UE		0U
#define XDR_UART_CR1_UESM		1U
#define XDR_UART_CR1_RE		2U
#define XDR_UART_CR1_TE		3U
#define XDR_UART_CR1_IDLEIE	4U
#define XDR_UART_CR1_RXNEIE	5U
#define XDR_UART_CR1_TCIE		6U
#define XDR_UART_CR1_TXEIE		7U
#define XDR_UART_CR1_PEIE		8U
#define XDR_UART_CR1_M0		12U
#define XDR_UART_CR1_OVER8		15U
#define XDR_UART_CR1_M1		28U

#define XDR_UART_CR1_CLEAR		0UL

// Macros for UART ISR
#define XDR_UART_ISR_RXNE		5U
#define XDR_UART_ISR_TC		6U
#define XDR_UART_ISR_TXE		7U

// Macros for UART BRR
#define XDR_UART_BRR_Pos			4U
#define XDR_UART_BRR_Over16Mask	0x0FU

#define XDR_UART_OVERSAMPLING_16   0UL

// Macros for Baud Rates
#define XDR_UART_BAUD_4800      4800UL
#define XDR_UART_BAUD_9600      9600UL
#define XDR_UART_BAUD_38400     38400UL
#define XDR_UART_BAUD_57600     57600UL
#define XDR_UART_BAUD_115200    115200UL
#define XDR_UART_BAUD_460800    460800UL

// Macro for Clock Source Freqeuncy
#define XDR_HSI_CLK				16000000U

// Macros GPIO AF bit positions for selected pins
#define XDR_UART_GPIO_AF_PA9	4U
#define XDR_UART_GPIO_AF_PA10	8U
#define XDR_UART_GPIO_AF_PD5	20U
#define XDR_UART_GPIO_AF_PD6	24U
#define XDR_UART_GPIO_AF_PD8	0U
#define XDR_UART_GPIO_AF_PD9	4U
#define XDR_UART_GPIO_AF_PC6	24U
#define XDR_UART_GPIO_AF_PC7	28U

// Clock source HSI selection macros for UARTx
#define UART1_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 0U)  )
#define UART2_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 2U)  )
#define UART3_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 4U)  )
#define UART6_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 10U) )

// Clock enable macros for UARTx
#define UART1_CLOCK_ENABLE()	(RCC->APB2ENR |= (1UL << 4U ) )
#define UART2_CLOCK_ENABLE()	(RCC->APB1ENR |= (1UL << 17U) )
#define UART3_CLOCK_ENABLE()	(RCC->APB1ENR |= (1UL << 18U) )
#define UART6_CLOCK_ENABLE()	(RCC->APB2ENR |= (1UL << 5U ) )

#endif /* XDR_UART_H_ */

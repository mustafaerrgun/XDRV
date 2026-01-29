/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_usart.h
  * @author  Mustafa Ergün
  * @brief   Header file of USART XDR module.
  ******************************************************************************
  */

#ifndef STM32F767XX__XDR_USART_H_
#define STM32F767XX__XDR_USART_H_

#include "stm32f767xx.h"
#include "stm32f767xx_xdr_rcc.h"
#include "stm32f767xx_xdr_gpio.h"

typedef enum
{
	XDR_USART1,
	XDR_USART2,
	XDR_USART3,
	XDR_USART6
}xdr_usart_instance;

typedef struct{
	USART_TypeDef 		*usart;
	xdr_usart_instance	 xdr_usart_instance;
	uint32_t 			 xdr_usart_baudrate;
	uint8_t				 xdr_usart_interrupt;
	uint8_t				 xdr_usart_dma;
}xdr_usart;

// Driver APIs
void XDR_USART_Init(xdr_usart *xdr_usart);
void XDR_USART_Send(xdr_usart *xdr_usart, uint8_t data);
uint8_t XDR_USART_Receive(xdr_usart *xdr_usart);
// Driver Interrupt APIs
void USART3_IRQHandler(void);
void XDR_USART3_Callback(uint8_t data);
// DMA Macros, variables and functions for USART3
#define USART_BUFF_SIZE		10U
extern char USART3_Data_Buffer[USART_BUFF_SIZE];
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void XDR_USART3_DMA_Rx_Config(void);
void XDR_USART3_DMA_Tx_Config(uint32_t data, uint32_t length);
void XDR_USART3_DMA_Rx_Callback(void);
void XDR_USART3_DMA_Tx_Callback(void);


// Macros for USART CR1
#define XDR_USART_CR1_UE		0U
#define XDR_USART_CR1_UESM		1U
#define XDR_USART_CR1_RE		2U
#define XDR_USART_CR1_TE		3U
#define XDR_USART_CR1_IDLEIE	4U
#define XDR_USART_CR1_RXNEIE	5U
#define XDR_USART_CR1_TCIE		6U
#define XDR_USART_CR1_TXEIE		7U
#define XDR_USART_CR1_PEIE		8U
#define XDR_USART_CR1_M0		12U
#define XDR_USART_CR1_OVER8		15U
#define XDR_USART_CR1_M1		28U

#define XDR_USART_CR1_CLEAR		0UL

// Macros for USART ISR
#define XDR_USART_ISR_RXNE		5U
#define XDR_USART_ISR_TC		6U
#define XDR_USART_ISR_TXE		7U

// Macrod for USART BRR
#define XDR_USART_BRR_Pos			4U
#define XDR_USART_BRR_Over16Mask	0x0FU

#define XDR_USART_OVERSAMPLING_16   0UL

// Macros for Baud Rates
#define XDR_USART_BAUD_4800      4800UL
#define XDR_USART_BAUD_9600      9600UL
#define XDR_USART_BAUD_38400     38400UL
#define XDR_USART_BAUD_57600     57600UL
#define XDR_USART_BAUD_115200    115200UL
#define XDR_USART_BAUD_460800    460800UL

// Macro for Clock Source Freqeuncy
#define XDR_HSI_CLK				16000000U

// Macros GPIO AF bit positions for selected pins
#define XDR_USART_GPIO_AF_PA9	4U
#define XDR_USART_GPIO_AF_PA10	8U
#define XDR_USART_GPIO_AF_PD5	20U
#define XDR_USART_GPIO_AF_PD6	24U
#define XDR_USART_GPIO_AF_PD8	0U
#define XDR_USART_GPIO_AF_PD9	4U
#define XDR_USART_GPIO_AF_PC6	24U
#define XDR_USART_GPIO_AF_PC7	28U

// Clock source HSI selection macros for USARTx
#define USART1_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 0U)  )
#define USART2_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 2U)  )
#define USART3_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 4U)  )
#define USART6_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 10U) )

//Clock enable macros for USARTx
#define USART1_CLOCK_ENABLE()	(RCC->APB2ENR |= (1UL << 4U ) )
#define USART2_CLOCK_ENABLE()	(RCC->APB1ENR |= (1UL << 17U) )
#define USART3_CLOCK_ENABLE()	(RCC->APB1ENR |= (1UL << 18U) )
#define USART6_CLOCK_ENABLE()	(RCC->APB2ENR |= (1UL << 5U ) )

#endif /* STM32F767XX__XDR_USART_H_ */

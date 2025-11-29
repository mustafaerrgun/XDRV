/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_usart.h
  * @author  Mustafa Ergün
  * @brief   Header file of USART XDR module.
  ******************************************************************************
  */

#ifndef STM32F407XX__XDR_USART_DRIVER_H_
#define STM32F407XX__XDR_USART_DRIVER_H_

#include "stm32f767xx.h"
#include "stm32f767xx_xdr_rcc.h"

typedef enum
{
	XDR_USART1,
	XDR_USART2,
	XDR_USART3,
	XDR_USART6
}XDR_USART_Instance;

typedef enum
{
	XDR_USART_WORDLENGTH_7B,
    XDR_USART_WORDLENGTH_8B,
    XDR_USART_WORDLENGTH_9B
} XDR_USART_WordLength;

typedef enum
{
    XDR_USART_OVERSAMPLING_16 = 0UL,
    XDR_USART_OVERSAMPLING_8  = 1UL
} XDR_USART_Oversamp;

typedef enum
{
    XDR_USART_MODE_RX,
    XDR_USART_MODE_TX,
    XDR_USART_MODE_TX_RX
} XDR_USART_Mode;

typedef struct{
	uint32_t 			  XDR_USART_BaudRate;
	XDR_USART_WordLength  XDR_USART_WordLength;
	XDR_USART_Mode        XDR_USART_Mode;
}XDR_USART_Config;

typedef struct{
	USART_TypeDef 		*XDR_USART;
	XDR_RCC_Handle      *XDR_RCC_Handle;
	XDR_USART_Instance	 XDR_USART_Instance;
	XDR_USART_Config 	 XDR_USART_Config;
}XDR_USART_Handle;


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
#define XDR_USART_BRR_Over16Mask	0x0FUL

#define XDR_USART_OVERSAMPLING_16   0UL

// Macros for Baud Rates
#define XDR_USART_BAUD_4800      4800UL
#define XDR_USART_BAUD_9600      9600UL
#define XDR_USART_BAUD_38400     38400UL
#define XDR_USART_BAUD_57600     57600UL
#define XDR_USART_BAUD_115200    115200UL
#define XDR_USART_BAUD_460800    460800UL


//Clock enable macros for USARTx
#define USART1_CLOCK_ENABLE()	(RCC->APB2ENR |= (1UL << 4U ) )
#define USART2_CLOCK_ENABLE()	(RCC->APB1ENR |= (1UL << 17U) )
#define USART3_CLOCK_ENABLE()	(RCC->APB1ENR |= (1UL << 18U) )
#define USART6_CLOCK_ENABLE()	(RCC->APB2ENR |= (1UL << 5U ) )

// Clock disable macros for USARTx
#define USART1_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1UL << 4U ) )
#define USART2_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1UL << 17U) )
#define USART3_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1UL << 18U) )
#define USART6_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1UL << 5U ) )

// Driver APIs
void XDR_USART_Init(XDR_USART_Handle *USART_Handle);
void XDR_USART_Send(XDR_USART_Handle *USART_Handle, uint8_t data);
uint8_t XDR_USART_Receive(XDR_USART_Handle *USART_Handle);

#endif /* STM32F407XX__XDR_USART_DRIVER_H_ */

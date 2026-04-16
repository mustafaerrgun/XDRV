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
#include "xdr_gpio.h"

typedef enum
{
	XDR_UART1,
	XDR_UART2,
	XDR_UART3,
	XDR_UART4,
	XDR_UART5,
	XDR_UART6,
	XDR_UART7
}xdr_uart_instance;

typedef struct{
	xdr_uart_instance	 xdr_uart_instance;
	uint32_t 			 xdr_uart_baudrate;
	uint8_t				 xdr_uart_interrupt;
	uint8_t				 xdr_uart_dma;
}xdr_uart;

// Driver APIs
void UART_Init(const xdr_uart *xdr_uart);
void UART_SendChar(const xdr_uart *xdr_uart, char c);
void UART_SendString(const xdr_uart *xdr_uart, const char *str);
uint8_t UART_Receive(const xdr_uart *xdr_uart);
// Driver Interrupt APIs
void USART3_IRQHandler(void);
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
#define XDR_UART_CR1_UE			0U
#define XDR_UART_CR1_RE			2U
#define XDR_UART_CR1_TE			3U
#define XDR_UART_CR1_CLEAR		0UL

// Macros for UART ISR
#define XDR_UART_ISR_RXNE		5U
#define XDR_UART_ISR_TC			6U
#define XDR_UART_ISR_TXE		7U

// Macros for UART BRR
#define XDR_UART_BRR_Pos			4U
#define XDR_UART_BRR_Over16Mask		0x0FU

// Macros for Baud Rates
#define XDR_UART_BAUD_4800      4800UL
#define XDR_UART_BAUD_9600      9600UL
#define XDR_UART_BAUD_38400     38400UL
#define XDR_UART_BAUD_57600     57600UL
#define XDR_UART_BAUD_115200    115200UL
#define XDR_UART_BAUD_460800    460800UL

// Macro for Clock Source Freqeuncy
#define XDR_HSI_CLK				16000000U

// Compute AFR register index and bit shift directly from pin number
#define UART_AFR_INDEX(pin)  ((uint8_t)((pin) >> 3U))
#define UART_AFR_SHIFT(pin)  ((uint8_t)(((pin) & 0x7U) << 2U))

// GPIO pin configuration struct for UART instances
typedef struct {
    GPIO_TypeDef    *tx_port;
    xdr_gpio_portId  tx_port_id;
    uint8_t          tx_pin;
    GPIO_TypeDef    *rx_port;
    xdr_gpio_portId  rx_port_id;
    uint8_t          rx_pin;
    uint8_t          af;
} uart_pin_cfg;

// Clock source (HSI) selection and clock enable macros for UARTx
// DCKCFGR2: value 0b10 = HSI clock source
#define UART1_CLOCK_INIT()  do {                                                    \
    RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~(3UL << 0U))  | (2UL << 0U);                  \
    RCC->APB2ENR |= (1UL << 4U);                                                    \
} while(0)

#define UART2_CLOCK_INIT()  do {                                                    \
    RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~(3UL << 2U))  | (2UL << 2U);                  \
    RCC->APB1ENR |= (1UL << 17U);                                                   \
} while(0)

#define UART3_CLOCK_INIT()  do {                                                    \
    RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~(3UL << 4U))  | (2UL << 4U);                  \
    RCC->APB1ENR |= (1UL << 18U);                                                   \
} while(0)

#define UART4_CLOCK_INIT()  do {                                                    \
    RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~(3UL << 6U))  | (2UL << 6U);                  \
    RCC->APB1ENR |= (1UL << 19U);                                                   \
} while(0)

#define UART5_CLOCK_INIT()  do {                                                    \
    RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~(3UL << 8U))  | (2UL << 8U);                  \
    RCC->APB1ENR |= (1UL << 20U);                                                   \
} while(0)

#define UART6_CLOCK_INIT()  do {                                                    \
    RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~(3UL << 10U)) | (2UL << 10U);                 \
    RCC->APB2ENR |= (1UL << 5U);                                                    \
} while(0)

#define UART7_CLOCK_INIT()  do {                                                    \
    RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~(3UL << 12U)) | (2UL << 12U);                 \
    RCC->APB1ENR |= (1UL << 30U);                                                   \
} while(0)

#endif /* XDR_UART_H_ */

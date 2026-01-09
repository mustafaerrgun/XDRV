/**
  ******************************************************************************
  * @file    stm32f767xx_xdr_spi.h
  * @author  Mustafa Ergün
  * @brief   Header file of SPI XDR module.
  ******************************************************************************
  */
#ifndef STM32F767XX__XDR_SPI_H_
#define STM32F767XX__XDR_SPI_H_

#include "stm32f767xx.h"
#include "stm32f767xx_xdr_gpio.h"


typedef struct{
	SPI_TypeDef 		*spi;
}xdr_spi;

// Public APIs
void XDR_SPI_Init(void);
void XDR_SPI_Transmit(uint8_t data);
uint8_t XDR_SPI_Receive(void);

// Macros for SPI CR1
#define XDR_SPI_CR1_CPHA		0U
#define XDR_SPI_CR1_CPOL		1U
#define XDR_SPI_CR1_MSTR		2U
#define XDR_SPI_CR1_BR			3U
#define XDR_SPI_CR1_SPE			6U
#define XDR_SPI_CR1_LSBFIRST	7U
#define XDR_SPI_CR1_SSI			8U
#define XDR_SPI_CR1_SSM			9U
#define XDR_SPI_CR1_RXONLY		10U

// Macros for SPI CR2
#define XDR_SPI_CR2_DS			8U
#define XDR_SPI_CR2_FRXTH		12U

// Macros for SPI SR
#define XDR_SPI_SR_RXNE			0U
#define XDR_SPI_SR_TXE			1U
#define XDR_SPI_SR_BSY			7U
#define XDR_SPI_SR_RXNE_MSK   (1UL << XDR_SPI_SR_RXNE)
#define XDR_SPI_SR_TXE_MSK    (1UL << XDR_SPI_SR_TXE)
#define XDR_SPI_SR_BSY_MSK    (1UL << XDR_SPI_SR_BSY)

// Macros GPIO AF bit positions for selected pins
#define XDR_SPI_GPIO_AF_PB10	8U
#define XDR_SPI_GPIO_AF_PB15	28U
#define XDR_SPI_GPIO_AF_PC2		8U


// Clock enable macros for SPIx
#define SPI1_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1UL << 12U))
#define SPI2_CLOCK_ENABLE()		(RCC->APB1ENR |= (1UL << 14U))
#define SPI3_CLOCK_ENABLE()		(RCC->APB1ENR |= (1UL << 15U))
#define SPI4_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1UL << 13U))
#define SPI5_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1UL << 20U))
#define SPI6_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1UL << 21U))



#endif /* STM32F767XX__XDR_SPI_H_ */

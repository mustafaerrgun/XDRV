/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_spi.c
  * @author  Mustafa Ergün
  * @brief   I2C XDR module driver.
  *          This file provides firmware functions to manage the SPI peripheral
  ******************************************************************************
  */

#include "stm32f767xx_xdr_spi.h"

// Private APIs
static void XDR_SPI_GPIO_Init(void);

// Global Variable
static xdr_spi  xdr_SPI;

void XDR_SPI_Init(void){

	// Enable Clock for SPI1
    SPI1_CLOCK_ENABLE();
    xdr_SPI.spi = SPI1;

    // Configure GPIO pins for SPI1
    XDR_SPI_GPIO_Init();

    // Clear CR1 register
    xdr_SPI.spi->CR1 = 0U;

    // Use default SPI Mode for CPOL (0) and CPHA (0)
    xdr_SPI.spi->CR1 &= ~(1UL << XDR_SPI_CR1_CPHA);
    xdr_SPI.spi->CR1 &= ~(1UL << XDR_SPI_CR1_CPOL);

    // Configure the SCK baud rate for fixed value PCLK/256
    xdr_SPI.spi->CR1 |= (7UL << XDR_SPI_CR1_BR);

    // Configure frame format for default LSBFIRST: 0, MSB first
    xdr_SPI.spi->CR1 &= ~(1UL << XDR_SPI_CR1_LSBFIRST);

    // Configure SSM and SSI for Software NSS Management
    xdr_SPI.spi->CR1 |= (1UL << XDR_SPI_CR1_SSI)
    				  |	 (1UL << XDR_SPI_CR1_SSM);

    // Enable Master Configuration
    xdr_SPI.spi->CR1 |= (1UL << XDR_SPI_CR1_MSTR);

    // Clear CR2 register and set data size as 8-bits
    xdr_SPI.spi->CR2 = 0U;
    xdr_SPI.spi->CR2 |= (7UL << XDR_SPI_CR2_DS);
    xdr_SPI.spi->CR2 |= (1UL << XDR_SPI_CR2_FRXTH);

    // Enable SPI2
    xdr_SPI.spi->CR1 |= (1UL << XDR_SPI_CR1_SPE);
}

void XDR_SPI_Transmit(uint8_t data)
{
    // CS low
	GPIOA->BSRR = (1UL << (4U + 16U));

    // Wait TX empty
    while (!(xdr_SPI.spi->SR & XDR_SPI_SR_TXE_MSK)) {}

    // Send byte
    xdr_SPI.spi->DR = data;

    // Wait SPI idle
    while ((xdr_SPI.spi->SR & XDR_SPI_SR_BSY_MSK) != 0U) {}

    // CS high
    GPIOA->BSRR = (1UL << 4U);
}

uint8_t XDR_SPI_Receive(void){

	uint8_t data = 0U;

    // CS low
	GPIOA->BSRR = (1UL << (4U + 16U));

	// Wait SPI idle
	while ((xdr_SPI.spi->SR & XDR_SPI_SR_BSY_MSK) != 0U) {}

	// Send dummy data
	xdr_SPI.spi->DR = 0;

	// Wait for RXNE is empty
	while (!(xdr_SPI.spi->SR & XDR_SPI_SR_RXNE_MSK)) {}

	data = (xdr_SPI.spi->DR);

    // CS high
    GPIOA->BSRR = (1UL << 4U);

	return data;

}

static void XDR_SPI_GPIO_Init(void){

	// Enable clock GPIO pins for port A and B
	RCC ->AHB1ENR |= (1UL << 0U) | (1UL << 1U);

	// Configure GPIO mode
	GPIOA ->MODER |= (1UL << 8U); // PA4 configured as general output
	GPIOB ->MODER |= (2UL << 6U)  // PB3, PB4 and PB5 configured as AF mode
					|(2UL << 8U)
					|(2UL << 10U);

	// Configure speed type for pins
	GPIOA ->OSPEEDR |= (2UL << 8U); // PA4 configured for high speed
	GPIOB ->OSPEEDR |= (2UL << 6U)  // PB3, PB4 and PB5 configured for high speed
					  |(2UL << 8U)
					  |(2UL << 10U);

	// CS initially HIGH (slave deselect)
	GPIOA->BSRR = (1UL << 4U);

	// Configure AF register for pins
	GPIOB ->AFR[0] |= (5UL << 12U)  // PB3 -> AF5
					 |(5UL << 16U)  // PB4 -> AF5
					 |(5UL << 20U); // PB5 -> AF5

}




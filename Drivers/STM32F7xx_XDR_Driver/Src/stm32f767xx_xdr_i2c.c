/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_i2c.c
  * @author  Mustafa Ergün
  * @brief   I2C XDR module driver.
  *          This file provides firmware functions to manage the I2C peripheral
  ******************************************************************************
  */

#include "stm32f767xx_xdr_i2c.h"

// Private APIs
static void XDR_I2C_Clock_Enable(xdr_i2c *xdr_I2C);
static void XDR_I2C_GPIO_Init(const xdr_i2c *xdr_I2C);
static void XDR_I2C_TimingConfig(xdr_i2c *xdr_I2C);

void XDR_I2C_Init(xdr_i2c *xdr_I2C){

	// Enable Clock for I2C
	XDR_I2C_Clock_Enable(xdr_I2C);

	// Configure GPIO pins for I2C
	XDR_I2C_GPIO_Init(xdr_I2C);

	// Clear PE bit in CR1 register
	xdr_I2C->i2c->CR1 &= ~(1U << XDR_I2C_CR1_PE);

	// Digital and Analog Filter enabled
	xdr_I2C->i2c->CR1 |= (1UL << XDR_I2C_CR1_DNF);
	xdr_I2C->i2c->CR1 |= (1UL << XDR_I2C_CR1_ANFOFF);

	// Configure TIMINGR for I2C
	XDR_I2C_TimingConfig(xdr_I2C);

	// NOSTRETCH = 0 → clock stretching allowed
	xdr_I2C->i2c->CR1 &= ~(1UL << XDR_I2C_CR1_NOSTRETCH);

	// Enable I2C
	xdr_I2C->i2c->CR1 |= (1U << XDR_I2C_CR1_PE);
}

void XDR_I2C_Write(xdr_i2c *xdr_I2C, uint8_t addr7, uint8_t data){

	// Wait bus free
	while ((xdr_I2C->i2c->ISR & ((uint32_t)1UL << XDR_I2C_ISR_BUSY)) != 0UL) {}

	// Clear flags
	xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_STOPCF) | (1U << XDR_I2C_ICR_NACKCF);

	// Configure: 7-bit addr, 1 byte, write, auto STOP
	xdr_I2C->i2c->CR2 = 0x0U;
	xdr_I2C->i2c->CR2 |= ((uint32_t)(addr7 << 1) << XDR_I2C_CR2_SADD);
	xdr_I2C->i2c->CR2 |= (1UL << XDR_I2C_CR2_NBYTES);
	xdr_I2C->i2c->CR2 &= ~(1UL << XDR_I2C_CR2_RD_WRN);
	xdr_I2C->i2c->CR2 |=  (1UL << XDR_I2C_CR2_AUTOEND);

	// Generate START
	xdr_I2C->i2c->CR2 |= (1UL << XDR_I2C_CR2_START);

    // Wait TXIS or NACK
    while (!(xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_TXIS))) {
        if ( (xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_NACKF)) != 0UL) {
            xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_NACKCF);
        }
    }

	// Send Data
	xdr_I2C->i2c->TXDR = data;

    // Wait for STOP
    while (!(xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_STOPF))) {
        if ((xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_NACKF)) != 0UL) {
            xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_NACKCF);
            break;
        }
    }

    // Clear STOP
    xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_STOPCF);
}

uint8_t XDR_I2C_Read(xdr_i2c *xdr_I2C, uint8_t addr7){

	uint8_t data=0;

	// Wait bus free
	while ((xdr_I2C->i2c->ISR & ((uint32_t)1UL << XDR_I2C_ISR_BUSY)) != 0UL) {}

	// Clear flags
	xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_STOPCF) | (1U << XDR_I2C_ICR_NACKCF);

	// Configure slave address and number of bytes
	xdr_I2C->i2c->CR2 = 0x0U;
	xdr_I2C->i2c->CR2 |= ((uint32_t)(addr7 << 1) << XDR_I2C_CR2_SADD);
	xdr_I2C->i2c->CR2 |= (1UL << XDR_I2C_CR2_NBYTES);
	xdr_I2C->i2c->CR2 |= (1UL << XDR_I2C_CR2_RD_WRN);
	xdr_I2C->i2c->CR2 |= (1UL << XDR_I2C_CR2_AUTOEND);

	// Generate START
	xdr_I2C->i2c->CR2 |= (1UL << XDR_I2C_CR2_START);

    // Wait RXNE or NACK
    while (!(xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_RXNE))) {
        if ( (xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_NACKF)) != 0UL) {
            xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_NACKCF);
        }
    }

    // Read byte
    data = (uint8_t)xdr_I2C->i2c->RXDR;

    // Wait for STOP
    while (!(xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_STOPF))) {
        if ( (xdr_I2C->i2c->ISR & (1U << XDR_I2C_ISR_NACKF)) != 0UL) {
            xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_NACKCF);
            break;
        }
    }

    // Clear STOP
    xdr_I2C->i2c->ICR |= (1U << XDR_I2C_ICR_STOPCF);

	return data;
}

static void XDR_I2C_Clock_Enable(xdr_i2c *xdr_I2C)
{
    switch (xdr_I2C->xdr_i2c_instance)
    {
        case XDR_I2C1:
        {
            I2C1_CLOCK_SOURCE();
            I2C1_CLOCK_ENABLE();
            xdr_I2C->i2c = I2C1;
            break;
        }

        case XDR_I2C2:
        {
            I2C2_CLOCK_SOURCE();
            I2C2_CLOCK_ENABLE();
            xdr_I2C->i2c = I2C2;
            break;
        }

        case XDR_I2C4:
        {
            I2C4_CLOCK_SOURCE();
            I2C4_CLOCK_ENABLE();
            xdr_I2C->i2c = I2C4;
            break;
        }

        default:
        {
            xdr_I2C->i2c = NULL;
            break;
        }
    }
}


static void XDR_I2C_GPIO_Init(const xdr_i2c *xdr_I2C){

	xdr_gpio gpio_scl;
	xdr_gpio gpio_sda;

	gpio_scl.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
	gpio_scl.xdr_gpio_pinPuPd = GPIO_PIN_PU;
	gpio_scl.xdr_gpio_pinOType = GPIO_OP_TYPE_OD;

	gpio_sda.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
	gpio_sda.xdr_gpio_pinPuPd = GPIO_PIN_PU;
	gpio_sda.xdr_gpio_pinOType = GPIO_OP_TYPE_OD;

    switch(xdr_I2C->xdr_i2c_instance){
    	case XDR_I2C1:
    		// PB8 (SCL) and PB9(SDA) for I2C1
    		gpio_scl.xdr_gpiox  = GPIOB;
    		gpio_scl.xdr_gpio_portId = XDR_GPIO_PORT_B;
    		gpio_scl.xdr_gpio_pin = GPIO_PIN_NO_8;
    		XDR_GPIO_Init(&gpio_scl);
    		gpio_scl.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_I2C_GPIO_AF_PB8);
    		gpio_scl.xdr_gpiox->AFR[1] |=  (4UL << XDR_I2C_GPIO_AF_PB8);

    		gpio_sda.xdr_gpiox  = GPIOB;
    		gpio_sda.xdr_gpio_portId = XDR_GPIO_PORT_B;
    		gpio_sda.xdr_gpio_pin = GPIO_PIN_NO_9;
    		XDR_GPIO_Init(&gpio_sda);
    		gpio_sda.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_I2C_GPIO_AF_PB9);
    		gpio_sda.xdr_gpiox->AFR[1] |=  (4UL << XDR_I2C_GPIO_AF_PB9);

    		break;
    	case XDR_I2C2:
    		// PB10 (SCL) and PB11(SDA) for I2C2
    		gpio_scl.xdr_gpiox  = GPIOB;
    		gpio_scl.xdr_gpio_portId = XDR_GPIO_PORT_B;
    		gpio_scl.xdr_gpio_pin = GPIO_PIN_NO_10;
    		XDR_GPIO_Init(&gpio_scl);
    		gpio_scl.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_I2C_GPIO_AF_PB10);
    		gpio_scl.xdr_gpiox->AFR[1] |=  (4UL << XDR_I2C_GPIO_AF_PB10);

    		gpio_sda.xdr_gpiox  = GPIOB;
    		gpio_sda.xdr_gpio_portId = XDR_GPIO_PORT_B;
    		gpio_sda.xdr_gpio_pin = GPIO_PIN_NO_11;
    		XDR_GPIO_Init(&gpio_sda);
    		gpio_sda.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_I2C_GPIO_AF_PB11);
    		gpio_sda.xdr_gpiox->AFR[1] |=  (4UL << XDR_I2C_GPIO_AF_PB11);

    		break;
    	case XDR_I2C4:
    		// PF14 (SCL) and PF15(SDA) for I2C4
    		gpio_scl.xdr_gpiox  = GPIOF;
    		gpio_scl.xdr_gpio_portId = XDR_GPIO_PORT_F;
    		gpio_scl.xdr_gpio_pin = GPIO_PIN_NO_14;
    		XDR_GPIO_Init(&gpio_scl);
    		gpio_scl.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_I2C_GPIO_AF_PF14);
    		gpio_scl.xdr_gpiox->AFR[1] |=  (4UL << XDR_I2C_GPIO_AF_PF14);

    		gpio_sda.xdr_gpiox  = GPIOF;
    		gpio_sda.xdr_gpio_portId = XDR_GPIO_PORT_F;
    		gpio_sda.xdr_gpio_pin = GPIO_PIN_NO_15;
    		XDR_GPIO_Init(&gpio_sda);
    		gpio_sda.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_I2C_GPIO_AF_PF15);
    		gpio_sda.xdr_gpiox->AFR[1] |=  (4UL << XDR_I2C_GPIO_AF_PF15);

    		break;
    	default: break;
    }

}

static void XDR_I2C_TimingConfig(xdr_i2c *xdr_I2C){

	// Configure I2C module standard mode timing settings for I2CCLK of 16MHz
	xdr_I2C->i2c->TIMINGR =  (0x3UL  << XDR_I2C_TIMINGR_PRESC)  |
							 (0x4UL  << XDR_I2C_TIMINGR_SCLDEL) |
							 (0x2UL  << XDR_I2C_TIMINGR_SDADEL) |
							 (0xFUL  << XDR_I2C_TIMINGR_SCLH)   |
							 (0x13U << XDR_I2C_TIMINGR_SCLL) ;
}

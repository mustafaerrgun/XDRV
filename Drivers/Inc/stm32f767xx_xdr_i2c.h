/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_i2c.h
  * @author  Mustafa Ergün
  * @brief   Header file of I2C XDR module.
  ******************************************************************************
  */
#ifndef STM32F407XX__XDR_I2C_H_
#define STM32F407XX__XDR_I2C_H_

#include "stm32f767xx.h"
#include "stm32f767xx_xdr_gpio.h"

typedef enum
{
	XDR_I2C1,
	XDR_I2C2,
	XDR_I2C4,
}xdr_i2c_instance;

typedef struct{
	I2C_TypeDef 		*i2c;
	xdr_i2c_instance	 xdr_i2c_instance;
}xdr_i2c;

// Public APIs
void XDR_I2C_Init(xdr_i2c *xdr_I2C);
void XDR_I2C_Write(xdr_i2c *xdr_I2C, uint8_t addr7, uint8_t data);
uint8_t XDR_I2C_Read(xdr_i2c *xdr_I2C, uint8_t addr7);

// Macros for I2C CR1
#define XDR_I2C_CR1_PE			0U
#define XDR_I2C_CR1_DNF			8U
#define XDR_I2C_CR1_ANFOFF		12U
#define XDR_I2C_CR1_NOSTRETCH	17U

// Macros for I2C CR2
#define XDR_I2C_CR2_SADD		0U
#define XDR_I2C_CR2_RD_WRN		10U
#define XDR_I2C_CR2_START		13U
#define XDR_I2C_CR2_STOP		13U
#define XDR_I2C_CR2_NBYTES		16U
#define XDR_I2C_CR2_AUTOEND		25U

// Macros for I2C TIMINGR
#define XDR_I2C_TIMINGR_SCLL	0U
#define XDR_I2C_TIMINGR_SCLH	8U
#define XDR_I2C_TIMINGR_SDADEL	16U
#define XDR_I2C_TIMINGR_SCLDEL  20U
#define XDR_I2C_TIMINGR_PRESC	28U

// Macros for I2C ISR
#define XDR_I2C_ISR_TXE			0U
#define XDR_I2C_ISR_TXIS		1U
#define XDR_I2C_ISR_RXNE		2U
#define XDR_I2C_ISR_NACKF		4U
#define XDR_I2C_ISR_STOPF		5U
#define XDR_I2C_ISR_TC			6U
#define XDR_I2C_ISR_BUSY		15U

// Macro for I2C ICR
#define XDR_I2C_ICR_STOPCF		5U
#define XDR_I2C_ICR_NACKCF		4U

// Macros GPIO AF bit positions for selected pins
#define XDR_I2C_GPIO_AF_PB8		0U
#define XDR_I2C_GPIO_AF_PB9		4U
#define XDR_I2C_GPIO_AF_PB10	8U
#define XDR_I2C_GPIO_AF_PB11	12U
#define XDR_I2C_GPIO_AF_PF14	24U
#define XDR_I2C_GPIO_AF_PF15	28U

// Clock source HSI selection macros for I2Cx
#define I2C1_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 16U) )
#define I2C2_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 18U) )
#define I2C4_CLOCK_SOURCE()		(RCC->DCKCFGR2 |= (2UL << 22U) )

// Clock enable macros for I2Cx
#define I2C1_CLOCK_ENABLE()		(RCC->APB1ENR |= (1UL << 21U) )
#define I2C2_CLOCK_ENABLE()		(RCC->APB1ENR |= (1UL << 22U) )
#define I2C4_CLOCK_ENABLE()		(RCC->APB1ENR |= (1UL << 24U) )


#endif /* STM32F407XX__XDR_I2C_H_ */

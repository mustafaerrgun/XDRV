/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_gpio.h
  * @author  Mustafa Ergün
  * @brief   Header file of GPIO XDR module.
  ******************************************************************************
  */

#ifndef STM32F407XX__XDR_GPIO_DRIVER_H_
#define STM32F407XX__XDR_GPIO_DRIVER_H_

#include "stm32f767xx.h"

typedef enum
{
    XDR_GPIO_PORT_A,
    XDR_GPIO_PORT_B,
    XDR_GPIO_PORT_C,
    XDR_GPIO_PORT_D,
    XDR_GPIO_PORT_E,
    XDR_GPIO_PORT_F,
    XDR_GPIO_PORT_G,
    XDR_GPIO_PORT_H,
    XDR_GPIO_PORT_I,
    XDR_GPIO_PORT_J,
    XDR_GPIO_PORT_K
} xdr_gpio_portId;

typedef struct{
	GPIO_TypeDef 	*xdr_gpiox;
	xdr_gpio_portId  xdr_gpio_portId;
	uint8_t 		 xdr_gpio_pin;
	uint8_t 		 xdr_gpio_pinMode;
	uint8_t 		 xdr_gpio_pinPuPd;
}xdr_gpio;

/* Driver Public APIs */

// Init and DeInit APIs
void XDR_GPIO_Init(xdr_gpio *xdr_gpio);
void XDR_GPIO_DeInit(const xdr_gpio *xdr_gpio);

// GPIO Read and Write APIs
uint8_t XDR_GPIO_Read_Pin(xdr_gpio *xdr_gpio, uint8_t xdr_gpio_pin);
uint16_t XDR_GPIO_Read_Port(xdr_gpio *xdr_gpio);
void XDR_GPIO_Write_Pin(xdr_gpio *xdr_gpio, uint8_t xdr_gpio_pin, uint8_t xdr_value);
void XDR_GPIO_Write_Port(xdr_gpio *xdr_gpio, uint16_t xdr_value);
void XDR_GPIO_Toggle(xdr_gpio *xdr_gpio, uint8_t xdr_gpio_pin);


/* Macro definitions for GPIO Config */
#define GPIO_PIN_RESET 	0U
#define GPIO_PIN_SET 	1U

//GPIO pin numbers
#define GPIO_PIN_NO_0		0U
#define GPIO_PIN_NO_1		1U
#define GPIO_PIN_NO_2		2U
#define GPIO_PIN_NO_3		3U
#define GPIO_PIN_NO_4		4U
#define GPIO_PIN_NO_5		5U
#define GPIO_PIN_NO_6		6U
#define GPIO_PIN_NO_7		7U
#define GPIO_PIN_NO_8		8U
#define GPIO_PIN_NO_9		9U
#define GPIO_PIN_NO_10		10U
#define GPIO_PIN_NO_11		11U
#define GPIO_PIN_NO_12		12U
#define GPIO_PIN_NO_13		13U
#define GPIO_PIN_NO_14		14U
#define GPIO_PIN_NO_15		15U

//  GPIO pin modes
#define GPIO_MODE_INPUT 	0U
#define GPIO_MODE_OUTPUT	1U
#define GPIO_MODE_ALTFN		2U
#define GPIO_MODE_ANALOGE	3U

// GPIO output types
#define GPIO_OP_TYPE_PP		0U
#define GPIO_OP_TYPE_OD		1U

// GPIO output pin speed
#define GPIO_SPEED_HIGH			2U

//GPIO pin pull up pull down
#define GPIO_NO_PUPD			0U
#define GPIO_PIN_PD				2U
#define GPIO_PIN_PU				1U

// Clock enable macros for GPIOx
#define GPIOA_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 0U) )
#define GPIOB_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 1U) )
#define GPIOC_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 2U) )
#define GPIOD_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 3U) )
#define GPIOE_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 4U) )
#define GPIOF_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 5U) )
#define GPIOG_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 6U) )
#define GPIOH_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 7U) )
#define GPIOI_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 8U) )
#define GPIOJ_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 9U) )
#define GPIOK_CLOCK_ENABLE()	(RCC->AHB1ENR  |= (1UL << 10U))

// Clock disable macros for GPIOx
#define GPIOA_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 0U) )
#define GPIOB_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 1U) )
#define GPIOC_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 2U) )
#define GPIOD_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 3U) )
#define GPIOE_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 4U) )
#define GPIOF_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 5U) )
#define GPIOG_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 6U) )
#define GPIOH_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 7U) )
#define GPIOI_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 8U) )
#define GPIOJ_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 9U) )
#define GPIOK_CLOCK_DISABLE()	(RCC->AHB1ENR  &= ~(1UL << 10U))

// GPIOx peripheral reset
#define GPIOA_REG_RESET()			do {(RCC->AHB1RSTR |=(1UL <<  0U)); (RCC->AHB1RSTR &= ~(1UL <<  0U));}while(0)
#define GPIOB_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  1U)); (RCC->AHB1RSTR &= ~(1UL <<  1U));}while(0)
#define GPIOC_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  2U)); (RCC->AHB1RSTR &= ~(1UL <<  2U));}while(0)
#define GPIOD_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  3U)); (RCC->AHB1RSTR &= ~(1UL <<  3U));}while(0)
#define GPIOE_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  4U)); (RCC->AHB1RSTR &= ~(1UL <<  4U));}while(0)
#define GPIOF_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  5U)); (RCC->AHB1RSTR &= ~(1UL <<  5U));}while(0)
#define GPIOG_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  6U)); (RCC->AHB1RSTR &= ~(1UL <<  6U));}while(0)
#define GPIOH_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  7U)); (RCC->AHB1RSTR &= ~(1UL <<  7U));}while(0)
#define GPIOI_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  8U)); (RCC->AHB1RSTR &= ~(1UL <<  8U));}while(0)
#define GPIOJ_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL <<  9U)); (RCC->AHB1RSTR &= ~(1UL <<  9U));}while(0)
#define GPIOK_REG_RESET()           do {(RCC->AHB1RSTR |=(1UL << 10U)); (RCC->AHB1RSTR &= ~(1UL << 10U));}while(0)

#endif /* STM32F407XX__XDR_GPIO_DRIVER_H_ */

/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_gpio.c
  * @author  Mustafa Ergün
  * @brief   GPIO XDR module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the General Purpose Input/Output (GPIO) peripheral:
  *           + Initialization functions
  *           + IO operation functions
  ******************************************************************************
  */

#include "stm32f767xx_xdr_gpio.h"

// Private APIs
static void XDR_GPIO_Clock_Enable(const xdr_gpio *xdr_gpio);


void XDR_GPIO_Init(xdr_gpio *xdr_gpio){

	// Enable the Peripheral Clock
	XDR_GPIO_Clock_Enable(xdr_gpio);


	// Configure GPIO port mode register
	xdr_gpio->xdr_gpiox->MODER &= ~(0x3U << (2U * (uint32_t)xdr_gpio->xdr_gpio_pin));
	xdr_gpio->xdr_gpiox->MODER |= (xdr_gpio->xdr_gpio_pinMode
							   << (2U * (uint32_t)xdr_gpio->xdr_gpio_pin));

	// Configure GPIO output type register
	xdr_gpio->xdr_gpiox->OTYPER &= ~(0x1U << ((uint32_t)xdr_gpio->xdr_gpio_pin));
	xdr_gpio->xdr_gpiox->OTYPER |= (xdr_gpio->xdr_gpio_pinOType << ((uint32_t)xdr_gpio->xdr_gpio_pin));


	// Configure GPIO port output speed as high speed
	xdr_gpio->xdr_gpiox->OSPEEDR  &= ~(0x3U << (2U * (uint32_t)xdr_gpio->xdr_gpio_pin)) ;
	xdr_gpio->xdr_gpiox->OSPEEDR  |= (GPIO_SPEED_HIGH << (2U * (uint32_t)xdr_gpio->xdr_gpio_pin));

	// Configure GPIO port pull-up/pull-down register
	xdr_gpio->xdr_gpiox->PUPDR  &= ~(0x3U << (2U * (uint32_t)xdr_gpio->xdr_gpio_pin)) ;
	xdr_gpio->xdr_gpiox->PUPDR  |= ((uint32_t)xdr_gpio->xdr_gpio_pinPuPd
								<< 	(2U * (uint32_t)xdr_gpio->xdr_gpio_pin));

}

// Clock Control APIs for GPIO
static void XDR_GPIO_Clock_Enable(const xdr_gpio *xdr_gpio){

    switch (xdr_gpio->xdr_gpio_portId)
    {
        case XDR_GPIO_PORT_A: GPIOA_CLOCK_ENABLE(); break;
        case XDR_GPIO_PORT_B: GPIOB_CLOCK_ENABLE(); break;
		case XDR_GPIO_PORT_C: GPIOC_CLOCK_ENABLE(); break;
        case XDR_GPIO_PORT_D: GPIOD_CLOCK_ENABLE(); break;
		case XDR_GPIO_PORT_E: GPIOE_CLOCK_ENABLE(); break;
        case XDR_GPIO_PORT_F: GPIOF_CLOCK_ENABLE(); break;
		case XDR_GPIO_PORT_G: GPIOG_CLOCK_ENABLE(); break;
        case XDR_GPIO_PORT_H: GPIOH_CLOCK_ENABLE(); break;
		case XDR_GPIO_PORT_I: GPIOI_CLOCK_ENABLE(); break;
        case XDR_GPIO_PORT_J: GPIOJ_CLOCK_ENABLE(); break;
		case XDR_GPIO_PORT_K: GPIOK_CLOCK_ENABLE(); break;
        default: break;
    }

}

// GPIO Read APIs for Pin or Port
uint8_t XDR_GPIO_Read_Pin(xdr_gpio *xdr_gpio, uint8_t xdr_gpio_pin){

	uint8_t value ;
	value = (uint8_t)((xdr_gpio->xdr_gpiox->IDR >> xdr_gpio_pin) & GPIO_PIN_SET);
	return value ;
}
uint16_t XDR_GPIO_Read_Port(xdr_gpio *xdr_gpio) {

	uint16_t value ;
	value = (uint16_t)(xdr_gpio->xdr_gpiox->IDR);
	return value ;
}

// GPIO Write APIs for Pin or Port
void XDR_GPIO_Write_Pin(xdr_gpio *xdr_gpio, uint8_t xdr_gpio_pin, uint8_t xdr_value) {

	if (xdr_value == GPIO_PIN_SET) {
		xdr_gpio->xdr_gpiox->ODR |= (1UL << xdr_gpio_pin) ;

	} else {
		xdr_gpio->xdr_gpiox->ODR &= ~(1UL << xdr_gpio_pin) ;
	}
}
void XDR_GPIO_Write_Port(xdr_gpio *xdr_gpio, uint16_t xdr_value) {

	xdr_gpio->xdr_gpiox->ODR = xdr_value ;

}

// GPIO Toggle API for Pin
void XDR_GPIO_Toggle(xdr_gpio *xdr_gpio, uint8_t xdr_gpio_pin) {

	xdr_gpio->xdr_gpiox->ODR = xdr_gpio->xdr_gpiox->ODR ^ (1UL << xdr_gpio_pin) ;

}

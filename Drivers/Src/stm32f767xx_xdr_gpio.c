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

// Driver Private APIs
static void XDR_GPIO_Clock_Enable(const XDR_GPIO_Handle *GPIO_Handle);
static void XDR_GPIO_Clock_Disable(const XDR_GPIO_Handle *GPIO_Handle);

// Clock Control APIs for GPIO
static void XDR_GPIO_Clock_Enable(const XDR_GPIO_Handle *GPIO_Handle){

    switch (GPIO_Handle->XDR_PortId)
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

static void XDR_GPIO_Clock_Disable(const XDR_GPIO_Handle *GPIO_Handle)
{

    switch (GPIO_Handle->XDR_PortId)
    {
        case XDR_GPIO_PORT_A: GPIOA_CLOCK_DISABLE(); break;
        case XDR_GPIO_PORT_B: GPIOB_CLOCK_DISABLE(); break;
		case XDR_GPIO_PORT_C: GPIOC_CLOCK_DISABLE(); break;
        case XDR_GPIO_PORT_D: GPIOD_CLOCK_DISABLE(); break;
		case XDR_GPIO_PORT_E: GPIOE_CLOCK_DISABLE(); break;
        case XDR_GPIO_PORT_F: GPIOF_CLOCK_DISABLE(); break;
		case XDR_GPIO_PORT_G: GPIOG_CLOCK_DISABLE(); break;
        case XDR_GPIO_PORT_H: GPIOH_CLOCK_DISABLE(); break;
		case XDR_GPIO_PORT_I: GPIOI_CLOCK_DISABLE(); break;
        case XDR_GPIO_PORT_J: GPIOJ_CLOCK_DISABLE(); break;
		case XDR_GPIO_PORT_K: GPIOK_CLOCK_DISABLE(); break;
        default: break;
    }
}


// GPIO Init and DeInit APIs
void XDR_GPIO_Init(XDR_GPIO_Handle *GPIO_Handle){

	// Enable the Peripheral Clock
	XDR_GPIO_Clock_Enable(GPIO_Handle);


	// Configure GPIO port mode register
	GPIO_Handle->XDR_GPIOx->MODER &= ~(0x3UL << (2UL * (uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));
	GPIO_Handle->XDR_GPIOx->MODER |= (GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinMode
									<< 	(2UL * (uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));


	// Configure GPIO port output speed register
	GPIO_Handle->XDR_GPIOx->OSPEEDR  &= ~(0x3UL << (2UL * (uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin)) ;
	GPIO_Handle->XDR_GPIOx->OSPEEDR  |= ((uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinOSpeed
										<< 	(2UL * (uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));

	// Configure GPIO port pull-up/pull-down register
	GPIO_Handle->XDR_GPIOx->PUPDR  &= ~(0x3UL << (2UL * (uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin)) ;
	GPIO_Handle->XDR_GPIOx->PUPDR  |= ((uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinPUPDType
										<< 	(2UL * (uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));

	// Configure GPIO port output type register
	GPIO_Handle->XDR_GPIOx->OTYPER &= ~(0x1UL << (uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin) ;
	GPIO_Handle->XDR_GPIOx->OTYPER |= ((uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinOType
										<< 	(uint32_t)GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin);

	// Configure GPIO alternate function registers
	if (GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinMode == GPIO_MODE_ALTFN)
	{
	    uint8_t pinNumber = GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin;
	    uint32_t afValue   = GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinAFMode;

	    // AFRL: pins 0..7
	    if (pinNumber <= GPIO_PIN_NO_7)
	    {
	        GPIO_Handle->XDR_GPIOx->AFR[0] &= ~(0xFUL << (4UL * (uint32_t)pinNumber));
	        GPIO_Handle->XDR_GPIOx->AFR[0] |=  (afValue << (4UL * (uint32_t)pinNumber));
	    }
	    // AFRH: pins 8..15
	    else
	    {
	        uint8_t shift = pinNumber - GPIO_PIN_NO_8;
	        GPIO_Handle->XDR_GPIOx->AFR[1] &= ~(0xFUL << (4UL * (uint32_t)shift));
	        GPIO_Handle->XDR_GPIOx->AFR[1] |=  (afValue << (4UL * (uint32_t)shift));
	    }
	}

}
void XDR_GPIO_DeInit(const XDR_GPIO_Handle *GPIO_Handle) {

	// Disable the Peripheral Clock
	XDR_GPIO_Clock_Disable(GPIO_Handle);

	switch (GPIO_Handle->XDR_PortId)
    {
        case XDR_GPIO_PORT_A: GPIOA_REG_RESET(); break;
        case XDR_GPIO_PORT_B: GPIOB_REG_RESET(); break;
		case XDR_GPIO_PORT_C: GPIOC_REG_RESET(); break;
        case XDR_GPIO_PORT_D: GPIOD_REG_RESET(); break;
		case XDR_GPIO_PORT_E: GPIOE_REG_RESET(); break;
        case XDR_GPIO_PORT_F: GPIOF_REG_RESET(); break;
		case XDR_GPIO_PORT_G: GPIOG_REG_RESET(); break;
        case XDR_GPIO_PORT_H: GPIOH_REG_RESET(); break;
		case XDR_GPIO_PORT_I: GPIOI_REG_RESET(); break;
        case XDR_GPIO_PORT_J: GPIOJ_REG_RESET(); break;
		case XDR_GPIO_PORT_K: GPIOK_REG_RESET(); break;
        default: break;
    }
}

// GPIO Read for Pin or Port APIs
uint8_t XDR_GPIO_Read_Pin(const GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin){

	uint8_t value ;
	value = (uint8_t)((XDR_GPIOx->IDR >> XDR_GPIO_Pin)& GPIO_PIN_SET);
	return value ;
}
uint16_t XDR_GPIO_Read_Port(GPIO_TypeDef *XDR_GPIOx) {

	uint16_t value ;
	value = (uint16_t)(XDR_GPIOx->IDR ) ;
	return value ;
}

// GPIO Write to Pin or Port APIs
void XDR_GPIO_Write_Pin(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin, uint8_t XDR_Value) {

	if (XDR_Value == GPIO_PIN_SET) {
		XDR_GPIOx->ODR |= (1UL << XDR_GPIO_Pin) ;

	} else {
		XDR_GPIOx->ODR &= ~(1UL << XDR_GPIO_Pin) ;
	}
}
void XDR_GPIO_Write_Port(GPIO_TypeDef *XDR_GPIOx, uint16_t XDR_Value) {

	XDR_GPIOx->ODR = XDR_Value ;

}
void XDR_GPIO_Toggle(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin) {

	XDR_GPIOx->ODR = XDR_GPIOx->ODR ^ (1UL << XDR_GPIO_Pin) ;

}

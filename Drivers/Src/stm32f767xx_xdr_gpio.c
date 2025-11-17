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
#include <stdio.h>

// Clock Control APIs for GPIO
void XDR_GPIO_Clock_Enable(GPIO_TypeDef *XDR_GPIOx){


		if (XDR_GPIOx == GPIOA) {
			GPIOA_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOB) {
			GPIOB_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOC){
			GPIOC_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOD){
			GPIOD_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOE){
			GPIOE_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOF){
			GPIOF_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOG){
			GPIOG_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOH){
			GPIOH_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOI){
			GPIOI_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOJ){
			GPIOJ_CLOCK_ENABLE();
		}else if (XDR_GPIOx == GPIOK){
			GPIOK_CLOCK_ENABLE();
		}

}

void XDR_GPIO_Clock_Disable(GPIO_TypeDef *XDR_GPIOx)
{

		if (XDR_GPIOx == GPIOA) {
			GPIOA_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOB) {
			GPIOB_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOC){
			GPIOC_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOD){
			GPIOD_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOE){
			GPIOE_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOF){
			GPIOF_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOG){
			GPIOG_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOH){
			GPIOH_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOI){
			GPIOI_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOJ){
			GPIOJ_CLOCK_DISABLE();
		}else if (XDR_GPIOx == GPIOK){
			GPIOK_CLOCK_DISABLE();
		}
}


// GPIO Init and DeInit APIs
void XDR_GPIO_Init(XDR_GPIO_Handle *GPIO_Handle){

	// Enable the Peripheral Clock
	XDR_GPIO_Clock_Enable(GPIO_Handle->XDR_GPIOx);


	// Configure GPIO port mode register
	GPIO_Handle->XDR_GPIOx->MODER &= ~(0x3 << (2 * GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));
	GPIO_Handle->XDR_GPIOx->MODER |= (GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinMode
									<< 	(2 * GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));


	// Configure GPIO port output speed register
	GPIO_Handle->XDR_GPIOx->OSPEEDR  &= ~(0x3 << (2 * GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin)) ;
	GPIO_Handle->XDR_GPIOx->OSPEEDR  |= (GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinOSpeed
										<< 	(2 * GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));

	// Configure GPIO port pull-up/pull-down register
	GPIO_Handle->XDR_GPIOx->PUPDR  &= ~(0x3 << (2 * GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin)) ;
	GPIO_Handle->XDR_GPIOx->PUPDR  |= (GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinPUPDType
										<< 	(2 * GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin));

	// Configure GPIO port output type register
	GPIO_Handle->XDR_GPIOx->OTYPER &= ~(0x1 << GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin) ;
	GPIO_Handle->XDR_GPIOx->OTYPER |= (GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinOType
										<< 	GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin);

	// Configure GPIO alternate function registers
	if (GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinMode == GPIO_MODE_ALTFN)
	{
	    uint8_t pinNumber = GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_Pin;
	    uint8_t afValue   = GPIO_Handle->XDR_GPIO_Config.XDR_GPIO_PinAFMode;

	    // AFRL: pins 0..7
	    if (pinNumber <= 7)
	    {
	        GPIO_Handle->XDR_GPIOx->AFR[0] &= ~(0xF << (4 * pinNumber));
	        GPIO_Handle->XDR_GPIOx->AFR[0] |=  (afValue << (4 * pinNumber));
	    }
	    // AFRH: pins 8..15
	    else
	    {
	        uint8_t shift = pinNumber - 8;
	        GPIO_Handle->XDR_GPIOx->AFR[1] &= ~(0xF << (4 * shift));
	        GPIO_Handle->XDR_GPIOx->AFR[1] |=  (afValue << (4 * shift));
	    }
	}

}
void XDR_GPIO_DeInit(GPIO_TypeDef *XDR_GPIOx) {

	if (XDR_GPIOx == GPIOA) {
		GPIOA_REG_RESET() ;
	} else if (XDR_GPIOx == GPIOB){
		GPIOB_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOC){
		GPIOC_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOD){
		GPIOD_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOE){
		GPIOE_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOF){
		GPIOF_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOG){
		GPIOG_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOH){
		GPIOH_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOI){
		GPIOI_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOJ){
		GPIOJ_REG_RESET() ;
	}else if (XDR_GPIOx == GPIOK){
		GPIOK_REG_RESET() ;
	}


}

// GPIO Read for Pin or Port APIs
uint8_t XDR_GPIO_Read_Pin(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin){

	uint8_t value ;
	value = (uint8_t)((XDR_GPIOx->IDR >> XDR_GPIO_Pin)& 0x00000001 );
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
		XDR_GPIOx->ODR |= (1 << XDR_GPIO_Pin) ;

	} else {
		XDR_GPIOx->ODR &= ~(1 << XDR_GPIO_Pin) ;
	}
}
void XDR_GPIO_Write_Port(GPIO_TypeDef *XDR_GPIOx, uint16_t XDR_Value) {

	XDR_GPIOx->ODR = XDR_Value ;

}
void XDR_GPIO_Toggle(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin) {

	XDR_GPIOx->ODR = XDR_GPIOx->ODR ^ (1 << XDR_GPIO_Pin) ;

}

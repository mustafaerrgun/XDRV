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


typedef struct{
	uint8_t XDR_GPIO_Pin;
	uint8_t XDR_GPIO_PinMode;
	uint8_t XDR_GPIO_PinOType;
	uint8_t XDR_GPIO_PinOSpeed;
	uint8_t XDR_GPIO_PinPUPDType;
	uint8_t XDR_GPIO_PinSetReset;
	uint8_t XDR_GPIO_PinAFMode;

}XDR_GPIO_Config;

typedef struct{

	GPIO_TypeDef *XDR_GPIOx;
	XDR_GPIO_Config XDR_GPIO_Config;

}XDR_GPIO_Handle;

/* Driver APIs */

// Clock Control for GPIOs
void XDR_GPIO_Clock(GPIO_TypeDef *XDR_GPIOx);

// Init GPIO
void XDR_GPIO_Init(XDR_GPIO_Handle *GPIO_Handle);

// GPIO Read and Write
void XDR_GPIO_Read_Pin(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin);
void XDR_GPIO_Read_Port(GPIO_TypeDef *XDR_GPIOx);
void XDR_GPIO_Write_Pin(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin, uint8_t XDR_Value);
void XDR_GPIO_Write_Port(GPIO_TypeDef *XDR_GPIOx, uint16_t XDR_Value);
void XDR_GPIO_Toggle(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin);

#endif /* STM32F407XX__XDR_GPIO_DRIVER_H_ */

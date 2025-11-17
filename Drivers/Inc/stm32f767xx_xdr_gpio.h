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

// Clock Control APIs
void XDR_GPIO_Clock_Enable(GPIO_TypeDef *XDR_GPIOx);
void XDR_GPIO_Clock_Disable(GPIO_TypeDef *XDR_GPIOx);

// Init and DeInit APIs
void XDR_GPIO_Init(XDR_GPIO_Handle *GPIO_Handle);
void XDR_GPIO_DeInit(GPIO_TypeDef *XDR_GPIOx);

// GPIO Read and Write APIs
uint8_t XDR_GPIO_Read_Pin(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin);
uint16_t XDR_GPIO_Read_Port(GPIO_TypeDef *XDR_GPIOx);
void XDR_GPIO_Write_Pin(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin, uint8_t XDR_Value);
void XDR_GPIO_Write_Port(GPIO_TypeDef *XDR_GPIOx, uint16_t XDR_Value);
void XDR_GPIO_Toggle(GPIO_TypeDef *XDR_GPIOx, uint8_t XDR_GPIO_Pin);


/* Macro definitions for GPIO Config */

#define GPIO_PIN_SET 	1
#define GPIO_PIN_RESET 	0

//GPIO pin numbers
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

//  GPIO pin modes

#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOGE	3

// GPIO output types
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// GPIO pin Speeds

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

//GPIO pin pull up pull down
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PD				2
#define GPIO_PIN_PU				1
#endif /* STM32F407XX__XDR_GPIO_DRIVER_H_ */

/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_rcc.h
  * @author  Mustafa Ergün
  * @brief   Header file of TIM XDR module.
  ******************************************************************************
  */

#ifndef STM32F767XX__XDR_TIM_DRIVER_H_
#define STM32F767XX__XDR_TIM_DRIVER_H_

#include "stm32f767xx.h"

typedef enum
{
    XDR_TIM_2,
    XDR_TIM_3,
    XDR_TIM_4,
    XDR_TIM_5
} xdr_tim_id;

typedef struct{
	TIM_TypeDef 		*tim;
	xdr_tim_id			tim_id;
	uint32_t			presc;
	uint32_t			period;
}xdr_tim;

void XDR_TIM_Init(xdr_tim *xdr_tim);
void XDR_TIM_WaitUpdate(xdr_tim *xdr_tim);

// Macro for TIM Counter Enable
#define XDR_TIM_CR1_CEN        (1UL<<0U)

// Macro of Update Flag for TIM Status Register
#define XDR_TIM_SR_UIF  	   (1UL<<0U)

// Clock enable macros for SPIx
#define TIM2_CLOCK_ENABLE()	    (RCC->APB1ENR |= (1UL << 0U))
#define TIM3_CLOCK_ENABLE()		(RCC->APB1ENR |= (1UL << 1U))
#define TIM4_CLOCK_ENABLE()	    (RCC->APB1ENR |= (1UL << 3U))
#define TIM5_CLOCK_ENABLE()	    (RCC->APB1ENR |= (1UL << 3U))

#endif /* STM32F767XX__XDR_TIM_DRIVER_H_ */

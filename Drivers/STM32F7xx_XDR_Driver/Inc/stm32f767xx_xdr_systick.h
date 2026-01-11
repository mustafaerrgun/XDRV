/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_systick.h
  * @author  Mustafa Ergün
  * @brief   Header file of SysTick XDR module.
  ******************************************************************************
  */

#ifndef STM32F767XX__XDR_SYSTICK_DRIVER_H_
#define STM32F767XX__XDR_SYSTICK_DRIVER_H_

#include "stm32f767xx.h"
#include "stm32f767xx_xdr_rcc.h"

void XDR_SysTick_Delay(uint32_t delay_ms);

#define CTRL_ENABLE        (1UL<<0U)
#define CTRL_CLCKSRC       (1UL<<2U)
#define CTRL_COUNTFLAG    (1UL<<16U)


#endif /* STM32F767XX__XDR_SYSTICK_DRIVER_H_ */

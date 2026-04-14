/**
  ******************************************************************************
  * @file    xdr_systick.h
  * @author  Mustafa Ergün
  * @brief   Header file of SysTick XDR module.
  ******************************************************************************
  */

#ifndef XDR_SYSTICK_H_
#define XDR_SYSTICK_H_

#include "stm32f767xx.h"
#include "xdr_rcc.h"

void XDR_SysTick_Delay(uint32_t delay_ms);

#define CTRL_ENABLE        (1UL<<0U)
#define CTRL_CLCKSRC       (1UL<<2U)
#define CTRL_COUNTFLAG    (1UL<<16U)


#endif /* XDR_SYSTICK_H_ */

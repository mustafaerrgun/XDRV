/**
  ******************************************************************************
  * @file    stm32f767xx_xdr_tim.c
  * @author  Mustafa Ergün
  * @brief   TIM XDR module driver.
  *          This file provides firmware functions to manage the TIM2/3/4/5 peripherals
  ******************************************************************************
  */

#include "stm32f767xx_xdr_tim.h"

static void XDR_TIM_Clock_Enable(xdr_tim *xdr_tim);

void XDR_TIM_Init(xdr_tim *xdr_tim){
    /*Enable clock access*/
	XDR_TIM_Clock_Enable(xdr_tim);
    /*Set prescaler value*/
    xdr_tim->tim->PSC =  xdr_tim->presc  - 1UL;
    /*Set auto-reload value*/
    xdr_tim->tim->ARR =  xdr_tim->period - 1UL;
    /*Clear counter*/
    xdr_tim->tim->CNT = 0UL;
    /*Enable timer*/
    xdr_tim->tim->CR1 = XDR_TIM_CR1_CEN;
}

void XDR_TIM_WaitUpdate(xdr_tim *xdr_tim){

    while ((xdr_tim->tim->SR & XDR_TIM_SR_UIF) == 0U){}

    xdr_tim->tim->SR &= ~XDR_TIM_SR_UIF;   /* clear update flag */
}

static void XDR_TIM_Clock_Enable(xdr_tim *xdr_tim){

    switch (xdr_tim->tim_id)
    {
        case XDR_TIM_2: TIM2_CLOCK_ENABLE(); xdr_tim->tim=TIM2; break;
        case XDR_TIM_3: TIM3_CLOCK_ENABLE(); xdr_tim->tim=TIM3; break;
        case XDR_TIM_4: TIM4_CLOCK_ENABLE(); xdr_tim->tim=TIM4; break;
        case XDR_TIM_5: TIM5_CLOCK_ENABLE(); xdr_tim->tim=TIM5; break;
        default: break;
    }
}

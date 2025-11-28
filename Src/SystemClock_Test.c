/**
 ******************************************************************************
 * @file           : SystemClock_Test.c
 * @brief          : Test XDR RCC Driver module for 48MHz System Clock
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_rcc.h"

int main(void)
{
    XDR_RCC_Handle testHandle;
    testHandle.XDR_RCC = RCC;
    testHandle.XDR_RCC_Config.ClockSource = XDR_PLL_CLOCK;
    testHandle.XDR_RCC_Config.SYSCLK_Freq = XDR_SYSCLK_48MHZ;
    testHandle.XDR_RCC_Config.AHB_Prescaler = XDR_AHB_DIV4;
    testHandle.XDR_RCC_Config.APB1_Prescaler = XDR_APB_DIV2;
    testHandle.XDR_RCC_Config.APB2_Prescaler = XDR_APB_DIV2;



    XDR_RCC_Init(&testHandle);

    volatile uint32_t system_clock = XDR_Get_SysClock(&testHandle);
    volatile uint32_t ahb_clock = XDR_Get_HCLK(&testHandle);
    volatile uint32_t apb1_clock = XDR_Get_PCLK1(&testHandle);
    volatile uint32_t apb2_clock = XDR_Get_PCLK2(&testHandle);


	while(1){


	}
	return 0 ;
}







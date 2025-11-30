/**
 ******************************************************************************
 * @file           : test_rcc_driver.c
 * @brief          : Test XDR RCC Driver module
 ******************************************************************************
 */

#include "stm32f767xx_xdr_rcc.h"

int main(void)
{
	xdr_rcc rcc;
	rcc.rcc = RCC;
	rcc.sysclk_freq = XDR_SYSCLK_48MHZ;
    rcc.ahb_prescaler = XDR_AHB_DIV4;
    rcc.apb1_prescaler = XDR_APB_DIV2;
    rcc.apb2_prescaler = XDR_APB_DIV2;

    XDR_RCC_Init(&rcc);

    volatile uint32_t system_clock = XDR_Get_SysClock();
    volatile uint32_t ahb_clock    = XDR_Get_HCLK();
    volatile uint32_t apb1_clock   = XDR_Get_PCLK1();
    volatile uint32_t apb2_clock   = XDR_Get_PCLK2();

	while(1){

	}
	return 0 ;
}







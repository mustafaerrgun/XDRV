/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_gpio.c
  * @author  Mustafa Ergün
  * @brief   RCC XDR module driver.
  *          This file provides firmware functions to configure the the system
  *          and peripheral clocks
  ******************************************************************************
  */

#include "stm32f767xx_xdr_rcc.h"


// Private APIs
static void XDR_Enable_ClockSource(XDR_RCC_Handle *RCC_Handle);
static void XDR_RCC_ConfigPLL(XDR_RCC_Handle *RCC_Handle);
static void XDR_RCC_ConfigBusClocks(XDR_RCC_Handle *RCC_Handle);
static void XDR_RCC_SelectSystemClock(XDR_RCC_Handle *RCC_Handle);

void XDR_RCC_Init(XDR_RCC_Handle *RCC_Handle){

    // 1) Configure AHB, APB1, APB2 prescalers
    XDR_RCC_ConfigBusClocks(RCC_Handle);

    // 2) Enable and configure PLL/source
    XDR_Enable_ClockSource(RCC_Handle);

    // 3) Switch SYSCLK to the selected source
    XDR_RCC_SelectSystemClock(RCC_Handle);


}
// Select the clock source (HSE/HSI/PLL) for MCU
static void XDR_Enable_ClockSource(XDR_RCC_Handle *RCC_Handle){

    switch (RCC_Handle->XDR_RCC_Config.ClockSource) {

    case XDR_HSI_CLOCK:
        // 1) set HSION bit
    	RCC_Handle->XDR_RCC->CR |= (XDR_SET  << XDR_RCC_CR_HSION);
        // 2) wait until HSIRDY is set
    	while(!(RCC_Handle->XDR_RCC->CR & (XDR_SET <<XDR_RCC_CR_HSIRDY))) { /* wait for HSI ready */ }
        break;

    case XDR_PLL_CLOCK:
        // 1) Configure PLLCFGR
    	XDR_RCC_ConfigPLL(RCC_Handle);
        // 2) set PLLON
    	RCC_Handle->XDR_RCC->CR |= (XDR_SET << XDR_RCC_CR_PLLON);
        // 3) wait PLLRDY
    	while(!(RCC_Handle->XDR_RCC->CR & (XDR_SET<<XDR_RCC_CR_PLLRDY))) { /* wait for PLL ready */ }
        break;
    
    default:
        break;
    }


}


static void XDR_RCC_ConfigPLL(XDR_RCC_Handle *RCC_Handle){
    // 1) Make sure HSI is ON if PLLSRC = HSI
    RCC_Handle->XDR_RCC->CR |= (XDR_SET << XDR_RCC_CR_HSION);
    while (!(RCC_Handle->XDR_RCC->CR & (XDR_SET << XDR_RCC_CR_HSIRDY))) { /* wait for HSI ready */ }

    // 2) Clear PLLON, wait PLLRDY = 0
 	RCC_Handle->XDR_RCC->CR &= ~(XDR_SET << XDR_RCC_CR_PLLON);

    /* MISRA deviation: Waiting for hardware PLL lock. Condition changes in hardware. */
    /* cppcheck-suppress misra-c2012-14.4 */
 	while((RCC_Handle->XDR_RCC->CR & (XDR_SET << XDR_RCC_CR_PLLRDY))) { /* wait for PLL not ready */ }

    // 3) Clear flash latency and insert hardcoded PLLCFGR values into register
 	XDR_FLASH_WAIT_CLEAR();
    switch (RCC_Handle->XDR_RCC_Config.SYSCLK_Freq)
    {
        case XDR_SYSCLK_48MHZ:
        	RCC_Handle->XDR_RCC->PLLCFGR = XDR_RCC_PLLCFGR_48MHZ;
        	XDR_FLASH_WAIT_48MHZ();
            break;

        case XDR_SYSCLK_96MHZ:
        	RCC_Handle->XDR_RCC->PLLCFGR = XDR_RCC_PLLCFGR_96MHZ;
        	XDR_FLASH_WAIT_96MHZ();
            break;

        case XDR_SYSCLK_144MHZ:
        	RCC_Handle->XDR_RCC->PLLCFGR = XDR_RCC_PLLCFGR_144MHZ;
        	XDR_FLASH_WAIT_144MHZ();
            break;

        case XDR_SYSCLK_216MHZ:
        	RCC_Handle->XDR_RCC->PLLCFGR = XDR_RCC_PLLCFGR_216MHZ;
        	XDR_FLASH_WAIT_216MHZ();
            break;
        
        default:
            break;
    }
}

static void XDR_RCC_ConfigBusClocks(XDR_RCC_Handle *RCC_Handle)
{
    uint32_t cfgr = RCC_Handle->XDR_RCC->CFGR;

    // Clear Prescaler bits
    cfgr &= ~(XDR_RCC_CFGR_HPRE_MASK  |
              XDR_RCC_CFGR_PPRE1_MASK |
              XDR_RCC_CFGR_PPRE2_MASK);

    // Set new prescalers
    cfgr |= ((uint32_t)RCC_Handle->XDR_RCC_Config.AHB_Prescaler  << XDR_RCC_CFGR_HPRE);
    cfgr |= ((uint32_t)RCC_Handle->XDR_RCC_Config.APB1_Prescaler << XDR_RCC_CFGR_PPRE1);
    cfgr |= ((uint32_t)RCC_Handle->XDR_RCC_Config.APB2_Prescaler << XDR_RCC_CFGR_PPRE2);

    RCC_Handle->XDR_RCC->CFGR = cfgr;
}

static void XDR_RCC_SelectSystemClock(XDR_RCC_Handle *RCC_Handle)
{
    // 1) Clear SW bits
    RCC_Handle->XDR_RCC->CFGR &= ~XDR_RCC_CFGR_SW_MASK;

    // 2) Set desired source
    switch (RCC_Handle->XDR_RCC_Config.ClockSource)
    {
        case XDR_HSI_CLOCK:
        	RCC_Handle->XDR_RCC->CFGR |= (XDR_HSI_CLOCK << XDR_RCC_CFGR_SW);
            break;
        case XDR_PLL_CLOCK:
        	RCC_Handle->XDR_RCC->CFGR |= (XDR_PLL_CLOCK << XDR_RCC_CFGR_SW);
            break;
        default:
            break;
    }

    // 3) Wait until SWS bits reflect new source
    uint32_t sws = (RCC_Handle->XDR_RCC_Config.ClockSource << XDR_RCC_CFGR_SWS);
    while ( (RCC_Handle->XDR_RCC->CFGR & XDR_RCC_CFGR_SWS_MASK) != sws ) { }

}

uint32_t XDR_Get_SysClock(XDR_RCC_Handle *RCC_Handle){

	uint32_t sysclk = 0;

	// 1) Read system clock source from SWS bits
	uint32_t sws = (RCC_Handle->XDR_RCC->CFGR & XDR_RCC_CFGR_SWS_MASK)
					>> XDR_RCC_CFGR_SWS;

    switch (sws)
    {
        case XDR_HSI_CLOCK:
            sysclk = XDR_HSI_VALUE;
            break;

        case XDR_PLL_CLOCK:
        {
            uint32_t pllcfgr = RCC_Handle->XDR_RCC->PLLCFGR;

            /* 2) Get PLL source: bit PLLSRC (1 = HSE, 0 = HSI) */
            uint32_t pll_src_freq = XDR_HSI_VALUE;

            /* 3) Decode PLLM, PLLN, PLLP from PLLCFGR using masks and shifts */
            uint32_t pllm = (pllcfgr >> XDR_RCC_PLLCFGR_PLLM) & XDR_RCC_PLLCFGR_PLLM_MASK;
            uint32_t plln = (pllcfgr >> XDR_RCC_PLLCFGR_PLLN) & XDR_RCC_PLLCFGR_PLLN_MASK;
            uint32_t pllp_bits =
            				(pllcfgr >> XDR_RCC_PLLCFGR_PLLP) & XDR_RCC_PLLCFGR_PLLP_MASK;

            /* PLLP is encoded: 0 → 2, 1 → 4, 2 → 6, 3 → 8 */
            uint32_t pllp = (pllp_bits + XDR_SET) * XDR_PLL_CLOCK;

            /* 4) Compute VCO and SYSCLK
             * VCO = (pll_src_freq / PLLM) * PLLN
             * SYSCLK = VCO / PLLP
             */
            uint32_t vco = (pll_src_freq / pllm) * plln;
            sysclk = vco / pllp;

            break;
        }

        default:
            sysclk = 0;
            break;
    }

    return sysclk;

}

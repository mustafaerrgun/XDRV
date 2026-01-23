/**
 ******************************************************************************
 * @file    stm32f7xx_xdr_rcc.c
 * @author  Mustafa Ergün
 * @brief   RCC XDR module driver.
 *          This file provides firmware functions to configure the the system
 *          and peripheral clocks
 ******************************************************************************
 */

#include "stm32f767xx_xdr_rcc.h"

// Private APIs
static void XDR_RCC_ConfigBusClocks(xdr_rcc *xdr_rcc);
static void XDR_Enable_ClockSource(xdr_rcc *xdr_rcc);
static void XDR_RCC_ConfigPLL(xdr_rcc *xdr_rcc);
static void XDR_RCC_SelectSystemClock(xdr_rcc *xdr_rcc);
static uint32_t XDR_RCC_AHBPrescaler(uint32_t ahb_prescaler);
static uint32_t XDR_RCC_APBPrescaler(uint32_t apb_prescaler);

void XDR_RCC_Init(xdr_rcc *xdr_RCC)
{

    // 1) Configure AHB, APB1, APB2 prescalers
    XDR_RCC_ConfigBusClocks(xdr_RCC);

    // 2) Enable and configure PLL/source
    XDR_Enable_ClockSource(xdr_RCC);

    // 3) Switch SYSCLK to the selected source
    XDR_RCC_SelectSystemClock(xdr_RCC);
}

// Configure bus clocks AHB/APB1/APB2
static void XDR_RCC_ConfigBusClocks(xdr_rcc *xdr_rcc)
{
    uint32_t cfgr = xdr_rcc->rcc->CFGR;

    // Clear Prescaler bits
    cfgr &= ~(XDR_RCC_CFGR_HPRE_MASK | XDR_RCC_CFGR_PPRE1_MASK | XDR_RCC_CFGR_PPRE2_MASK);

    // Set new prescalers
    cfgr |= ((uint32_t)xdr_rcc->ahb_prescaler << XDR_RCC_CFGR_HPRE);
    cfgr |= ((uint32_t)xdr_rcc->apb1_prescaler << XDR_RCC_CFGR_PPRE1);
    cfgr |= ((uint32_t)xdr_rcc->apb2_prescaler << XDR_RCC_CFGR_PPRE2);

    xdr_rcc->rcc->CFGR = cfgr;
}

// Select the clock source (HSE/PLL) for MCU
static void XDR_Enable_ClockSource(xdr_rcc *xdr_rcc)
{

    switch (xdr_rcc->sysclk_freq)
    {

    // Use HSI as the clock source
    case XDR_SYSCLK_16MHZ:
        // 1) set HSION bit
        xdr_rcc->rcc->CR |= (XDR_SET << XDR_RCC_CR_HSION);
        // 2) wait until HSIRDY is set
        while (!(xdr_rcc->rcc->CR & (XDR_SET << XDR_RCC_CR_HSIRDY)))
        {
        }
        break;

        // Use PLL as the clock source
    case XDR_SYSCLK_48MHZ:
    case XDR_SYSCLK_96MHZ:
    case XDR_SYSCLK_144MHZ:
    case XDR_SYSCLK_216MHZ:
        // 1) Configure PLLCFGR
        XDR_RCC_ConfigPLL(xdr_rcc);
        // 2) set PLLON
        xdr_rcc->rcc->CR |= (XDR_SET << XDR_RCC_CR_PLLON);
        // 3) wait PLLRDY
        while (!(xdr_rcc->rcc->CR & (XDR_SET << XDR_RCC_CR_PLLRDY)))
        {
        }

        break;

    default:
        break;
    }
}

static void XDR_RCC_ConfigPLL(xdr_rcc *xdr_rcc)
{
    // 1) Make sure HSI is ON if PLLSRC = HSI
    xdr_rcc->rcc->CR |= (XDR_SET << XDR_RCC_CR_HSION);
    while (!(xdr_rcc->rcc->CR & (XDR_SET << XDR_RCC_CR_HSIRDY)))
    {
    }

    // 2) Clear PLLON, wait PLLRDY = 0
    xdr_rcc->rcc->CR &= ~(XDR_SET << XDR_RCC_CR_PLLON);
    while ((xdr_rcc->rcc->CR & (XDR_SET << XDR_RCC_CR_PLLRDY)) != 0U)
    {
    }

    // 3) Clear flash latency and insert hardcoded PLLCFGR values into register
    XDR_FLASH_WAIT_CLEAR();
    switch (xdr_rcc->sysclk_freq)
    {
    case XDR_SYSCLK_48MHZ:
        xdr_rcc->rcc->PLLCFGR = XDR_RCC_PLLCFGR_48MHZ;
        XDR_FLASH_WAIT_48MHZ();

        break;

    case XDR_SYSCLK_96MHZ:
        xdr_rcc->rcc->PLLCFGR = XDR_RCC_PLLCFGR_96MHZ;
        XDR_FLASH_WAIT_96MHZ();
        break;

    case XDR_SYSCLK_144MHZ:
        xdr_rcc->rcc->PLLCFGR = XDR_RCC_PLLCFGR_144MHZ;
        XDR_FLASH_WAIT_144MHZ();
        break;

    case XDR_SYSCLK_216MHZ:
        xdr_rcc->rcc->PLLCFGR = XDR_RCC_PLLCFGR_216MHZ;
        XDR_FLASH_WAIT_216MHZ();
        break;

    default:
        break;
    }
}

static void XDR_RCC_SelectSystemClock(xdr_rcc *xdr_rcc)
{

    uint32_t sws = 0U;

    // 1) Clear SW bits
    xdr_rcc->rcc->CFGR &= ~XDR_RCC_CFGR_SW_MASK;

    // 2) Set clock source according to SysClk
    switch (xdr_rcc->sysclk_freq)
    {
    case XDR_SYSCLK_16MHZ:
        xdr_rcc->rcc->CFGR |= (XDR_HSI_CLOCK << XDR_RCC_CFGR_SW);
        sws = (XDR_HSI_CLOCK << XDR_RCC_CFGR_SWS);
        break;
    default:
        xdr_rcc->rcc->CFGR |= (XDR_PLL_CLOCK << XDR_RCC_CFGR_SW);
        sws = (XDR_PLL_CLOCK << XDR_RCC_CFGR_SWS);
        break;
    }

    // 3) Wait until SWS bits reflect new source
    while ((xdr_rcc->rcc->CFGR & XDR_RCC_CFGR_SWS_MASK) != sws)
    {
    }
}

uint32_t XDR_Get_SysClock(void)
{

    uint32_t sysclk = 0U;

    // 1) Read system clock source from SWS bits
    uint32_t sws = (RCC->CFGR & XDR_RCC_CFGR_SWS_MASK) >> XDR_RCC_CFGR_SWS;

    switch (sws)
    {
    case XDR_HSI_CLOCK:
        sysclk = XDR_HSI_VALUE;
        break;

    case XDR_PLL_CLOCK:
    {
        uint32_t pllcfgr = RCC->PLLCFGR;

        /* 2) Get HSI as PLL clock source */
        uint32_t pll_src_freq = XDR_HSI_VALUE;

        /* 3) Decode PLLM, PLLN, PLLP from PLLCFGR using masks and shifts */
        uint32_t pllm = (pllcfgr & XDR_RCC_PLLCFGR_PLLM_MASK) >> XDR_RCC_PLLCFGR_PLLM;
        uint32_t plln = (pllcfgr & XDR_RCC_PLLCFGR_PLLN_MASK) >> XDR_RCC_PLLCFGR_PLLN;
        uint32_t pllp_bits = (pllcfgr >> XDR_RCC_PLLCFGR_PLLP) & XDR_RCC_PLLCFGR_PLLP_MASK;

        /* PLLP is encoded: 0 → 2, 1 → 4, 2 → 6, 3 → 8 */
        uint32_t pllp = (uint32_t)((pllp_bits + XDR_SET) * XDR_PLL_CLOCK);

        /* 4) Compute VCO and SYSCLK
         * VCO = (pll_src_freq / PLLM) * PLLN
         * SYSCLK = VCO / PLLP
         */
        uint32_t vco = (pll_src_freq / pllm) * plln;
        sysclk = vco / pllp;

        break;
    }

    default:
        sysclk = XDR_HSI_VALUE;
        break;
    }
    return sysclk;
}

uint32_t XDR_Get_HCLK(void)
{

    uint32_t hclk = 0;
    uint32_t ahb_prescaler = (RCC->CFGR & XDR_RCC_CFGR_HPRE_MASK) >> XDR_RCC_CFGR_HPRE;
    uint32_t ahb_divider = XDR_RCC_AHBPrescaler(ahb_prescaler);

    hclk = XDR_Get_SysClock() / ahb_divider;

    return hclk;
}

uint32_t XDR_Get_PCLK1(void)
{

    uint32_t pclk1 = 0;
    uint32_t hclk = XDR_Get_HCLK();
    uint32_t apb_prescaler = (RCC->CFGR & XDR_RCC_CFGR_PPRE1_MASK) >> XDR_RCC_CFGR_PPRE1;
    uint32_t apb_divider = XDR_RCC_APBPrescaler(apb_prescaler);

    pclk1 = hclk / apb_divider;

    return pclk1;
}

uint32_t XDR_Get_PCLK2(void)
{

    uint32_t pclk2 = 0;
    uint32_t hclk = XDR_Get_HCLK();
    uint32_t apb_prescaler = (RCC->CFGR & XDR_RCC_CFGR_PPRE2_MASK) >> XDR_RCC_CFGR_PPRE2;
    uint32_t apb_divider = XDR_RCC_APBPrescaler(apb_prescaler);

    pclk2 = hclk / apb_divider;

    return pclk2;
}

static uint32_t XDR_RCC_AHBPrescaler(uint32_t ahb_prescaler)
{
    uint32_t div;

    switch (ahb_prescaler)
    {
    case XDR_AHB_DIV1:
        div = 1U;
        break;
    case XDR_AHB_DIV2:
        div = 2U;
        break;
    case XDR_AHB_DIV4:
        div = 4U;
        break;
    case XDR_AHB_DIV8:
        div = 8U;
        break;
    case XDR_AHB_DIV16:
        div = 16U;
        break;
    case XDR_AHB_DIV64:
        div = 64U;
        break;
    case XDR_AHB_DIV128:
        div = 128U;
        break;
    case XDR_AHB_DIV256:
        div = 256U;
        break;
    case XDR_AHB_DIV512:
        div = 512U;
        break;
    default:
        div = 1U;
        break;
    }
    return div;
}

static uint32_t XDR_RCC_APBPrescaler(uint32_t apb_prescaler)
{
    uint32_t div;

    switch (apb_prescaler)
    {
    case XDR_APB_DIV1:
        div = 1U;
        break;
    case XDR_APB_DIV2:
        div = 2U;
        break;
    case XDR_APB_DIV4:
        div = 4U;
        break;
    case XDR_APB_DIV8:
        div = 8U;
        break;
    case XDR_APB_DIV16:
        div = 16U;
        break;
    default:
        div = 1U;
        break;
    }

    return div;
}

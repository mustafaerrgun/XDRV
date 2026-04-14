/**
 ******************************************************************************
 * @file    xdr_systick.c
 * @author  Mustafa Ergün
 * @brief   SysTick XDR module driver.
 ******************************************************************************
 */

#include "xdr_systick.h"

void XDR_SysTick_Delay(uint32_t delay_ms)
{
    // cppcheck-suppress misra-c2012-11.4
    volatile SysTick_Type *const pSystick = SysTick; /* MISRA dev: 11.4 */

    uint32_t cycles_per_ms = XDR_Get_HCLK() / 1000U;

    pSystick->LOAD = cycles_per_ms - 1U;
    pSystick->VAL = 0U;
    pSystick->CTRL = CTRL_CLCKSRC | CTRL_ENABLE;

    for (uint32_t i = 0U; i < delay_ms; i++)
    {
        while ((pSystick->CTRL & CTRL_COUNTFLAG) == 0U)
        {
        }
    }

    pSystick->CTRL = 0U;
}

/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_gpio.h
  * @author  Mustafa Ergün
  * @brief   Header file of RCC XDR module.
  ******************************************************************************
  */

#ifndef STM32F407XX__XDR_RCC_DRIVER_H_
#define STM32F407XX__XDR_RCC_DRIVER_H_

#include "stm32f767xx.h"

typedef enum {
    XDR_AHB_DIV1  = 0x0U,
    XDR_AHB_DIV2  = 0x8U,
    XDR_AHB_DIV4  = 0x9U,
    XDR_AHB_DIV8  = 0xAU,
    XDR_AHB_DIV16 = 0xBU,
    XDR_AHB_DIV64 = 0xCU,
    XDR_AHB_DIV128= 0xDU,
    XDR_AHB_DIV256= 0xEU,
    XDR_AHB_DIV512= 0xFU
} XDR_AHB_Prescaler;

typedef enum {
    XDR_APB_DIV1  = 0x0U,
    XDR_APB_DIV2  = 0x4U,
    XDR_APB_DIV4  = 0x5U,
    XDR_APB_DIV8  = 0x6U,
    XDR_APB_DIV16 = 0x7U
} XDR_APB_Prescaler;

typedef enum {
    XDR_SYSCLK_16MHZ  = 0,
    XDR_SYSCLK_48MHZ  = 1,
    XDR_SYSCLK_96MHZ  = 2,
    XDR_SYSCLK_144MHZ = 3,
    XDR_SYSCLK_216MHZ = 4
} XDR_SYSCLK_Freq;

typedef struct{
	uint8_t 			ClockSource;
	uint8_t 			SYSCLK_Freq;
    XDR_AHB_Prescaler 	AHB_Prescaler;
    XDR_APB_Prescaler 	APB1_Prescaler;
    XDR_APB_Prescaler 	APB2_Prescaler;
}XDR_RCC_Config;

typedef struct{
	RCC_TypeDef *XDR_RCC;
	XDR_RCC_Config XDR_RCC_Config;
}XDR_RCC_Handle;



#define XDR_RESET  0UL
#define XDR_SET    1UL

#define XDR_HSI_CLOCK 0UL
#define XDR_PLL_CLOCK 2UL

// HSI clock frequency 16MHz
#define XDR_HSI_VALUE 16000000

// System clock frequencies
#define XDR_SYSCLK_16MHZ    0U
#define XDR_SYSCLK_48MHZ    1U
#define XDR_SYSCLK_96MHZ    2U
#define XDR_SYSCLK_144MHZ   3U
#define XDR_SYSCLK_216MHZ   4U

// Macros for CR
#define XDR_RCC_CR_HSION 		  0U
#define XDR_RCC_CR_HSIRDY 		  1U
#define XDR_RCC_CR_HSITRIM 		  3U
#define XDR_RCC_CR_HSICAL 		  8U
#define XDR_RCC_CR_HSEON 		  16U
#define XDR_RCC_CR_HSERDY 		  17U
#define XDR_RCC_CR_HSEBYP 		  18U
#define XDR_RCC_CR_CSSON 		  19U
#define XDR_RCC_CR_PLLON 		  24U
#define XDR_RCC_CR_PLLRDY		  25U
#define XDR_RCC_CR_PLLI2SON 	  26U
#define XDR_RCC_CR_PLLI2SRDY	  27U

// Macros for PLLCFGR
#define XDR_RCC_PLLCFGR_PLLM   0U
#define XDR_RCC_PLLCFGR_PLLN   6U
#define XDR_RCC_PLLCFGR_PLLP   16U
#define XDR_RCC_PLLCFGR_PLLSRC 22U
#define XDR_RCC_PLLCFGR_PLLQ   24U

#define XDR_RCC_PLLCFGR_PLLM_MASK		  63U
#define XDR_RCC_PLLCFGR_PLLN_MASK		  511U
#define XDR_RCC_PLLCFGR_PLLSRC_MASK 	  1U
#define XDR_RCC_PLLCFGR_PLLP_MASK		  3U

#define XDR_RCC_PLLCFGR_48MHZ  0x22000C08  // PLLM:8, PLLN:48,  PLLP:2, PLLSRC:0, PLLQ:2, PLLR:2
#define XDR_RCC_PLLCFGR_96MHZ  0x24001808  // PLLM:8, PLLN:96,  PLLP:2, PLLSRC:0, PLLQ:4, PLLR:2
#define XDR_RCC_PLLCFGR_144MHZ 0x26002408  // PLLM:8, PLLN:144, PLLP:2, PLLSRC:0, PLLQ:6, PLLR:2
#define XDR_RCC_PLLCFGR_216MHZ 0x29003608  // PLLM:8, PLLN:216, PLLP:2, PLLSRC:0, PLLQ:9, PLLR:2

// Macros for CFGR
#define XDR_RCC_CFGR_SW      0U
#define XDR_RCC_CFGR_SWS     2U
#define XDR_RCC_CFGR_HPRE    4U
#define XDR_RCC_CFGR_PPRE1   10U
#define XDR_RCC_CFGR_PPRE2   13U

#define XDR_RCC_CFGR_SW_MASK     (0x3U << XDR_RCC_CFGR_SW)
#define XDR_RCC_CFGR_SWS_MASK    (0x3U << XDR_RCC_CFGR_SWS)
#define XDR_RCC_CFGR_HPRE_MASK   (0xFUL << XDR_RCC_CFGR_HPRE)
#define XDR_RCC_CFGR_PPRE1_MASK  (0x7UL << XDR_RCC_CFGR_PPRE1)
#define XDR_RCC_CFGR_PPRE2_MASK  (0x7UL << XDR_RCC_CFGR_PPRE2)


/** Flash Latency
 * To correctly read data from flash memory, the number of wait states (LATENCY) must be
	correctly programmed in the Flash access control register (FLASH_ACR) according to the
	frequency of the CPU clock (HCLK) and the supply voltage(3.3V) of the device.
  * Reference Manual : Table 7. Number of wait states according to CPU clock (HCLK) frequency
  */
#define XDR_FLASH_WAIT		    0U
#define XDR_FLASH_WAIT_MASK	  15UL

#define XDR_FLASH_WAIT_16MHZ()   (FLASH->ACR |=  (0UL << XDR_FLASH_WAIT))
#define XDR_FLASH_WAIT_48MHZ()   (FLASH->ACR |=  (1UL << XDR_FLASH_WAIT))
#define XDR_FLASH_WAIT_96MHZ()   (FLASH->ACR |=  (3UL << XDR_FLASH_WAIT))
#define XDR_FLASH_WAIT_144MHZ()  (FLASH->ACR |=  (4UL << XDR_FLASH_WAIT))
#define XDR_FLASH_WAIT_216MHZ()  (FLASH->ACR |=  (7UL << XDR_FLASH_WAIT))

#define XDR_FLASH_WAIT_CLEAR()   (FLASH->ACR &= ~(XDR_FLASH_WAIT_MASK << XDR_FLASH_WAIT))

// Public APIs
void XDR_RCC_Init(XDR_RCC_Handle *RCC_Handle);
uint32_t XDR_Get_SysClock(XDR_RCC_Handle *RCC_Handle);
uint32_t XDR_Get_HCLK(XDR_RCC_Handle *RCC_Handle);
uint32_t XDR_Get_PCLK1(XDR_RCC_Handle *RCC_Handle);
uint32_t XDR_Get_PCLK2(XDR_RCC_Handle *RCC_Handle);


#endif /* STM32F407XX__XDR_RCC_DRIVER_H_ */

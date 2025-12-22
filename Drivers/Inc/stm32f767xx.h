/**
  ******************************************************************************
  * @file    stm32f7xx.h
  * @author  Mustafa Ergün
  * @brief   Data structures and the address mapping for limited peripherals
  ******************************************************************************
  */

#ifndef STM32F767XX_H_
#define STM32F767XX_H_


#include <stdint.h>
#include <stddef.h>

#define XDR_VOL volatile

/**
  * FLASH Registers
  */

typedef struct
{
  XDR_VOL uint32_t ACR;      /*!< FLASH access control register,     Address offset: 0x00 */
  XDR_VOL uint32_t KEYR;     /*!< FLASH key register,                Address offset: 0x04 */
  XDR_VOL uint32_t OPTKEYR;  /*!< FLASH option key register,         Address offset: 0x08 */
  XDR_VOL uint32_t SR;       /*!< FLASH status register,             Address offset: 0x0C */
  XDR_VOL uint32_t CR;       /*!< FLASH control register,            Address offset: 0x10 */
  XDR_VOL uint32_t OPTCR;    /*!< FLASH option control register ,    Address offset: 0x14 */
  XDR_VOL uint32_t OPTCR1;   /*!< FLASH option control register 1 ,  Address offset: 0x18 */
} FLASH_TypeDef;

/**
  * External Interrupt/Event Controller
  */

typedef struct
{
  XDR_VOL uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  XDR_VOL uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  XDR_VOL uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  XDR_VOL uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  XDR_VOL uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  XDR_VOL uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

/**
  * General Purpose I/O
  */

typedef struct
{
  XDR_VOL uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  XDR_VOL uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  XDR_VOL uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  XDR_VOL uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  XDR_VOL uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  XDR_VOL uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  XDR_VOL uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  XDR_VOL uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  XDR_VOL uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/**
  * System configuration controller
  */

typedef struct
{
  XDR_VOL uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  XDR_VOL uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  XDR_VOL uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED;     /*!< Reserved, 0x18                                                               */
  XDR_VOL uint32_t CBR;          /*!< SYSCFG Class B register,                           Address offset: 0x1C      */
  XDR_VOL uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

/**
  * Inter-integrated Circuit Interface
  */

typedef struct
{
  XDR_VOL uint32_t CR1;      /*!< I2C Control register 1,            Address offset: 0x00 */
  XDR_VOL uint32_t CR2;      /*!< I2C Control register 2,            Address offset: 0x04 */
  XDR_VOL uint32_t OAR1;     /*!< I2C Own address 1 register,        Address offset: 0x08 */
  XDR_VOL uint32_t OAR2;     /*!< I2C Own address 2 register,        Address offset: 0x0C */
  XDR_VOL uint32_t TIMINGR;  /*!< I2C Timing register,               Address offset: 0x10 */
  XDR_VOL uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
  XDR_VOL uint32_t ISR;      /*!< I2C Interrupt and status register, Address offset: 0x18 */
  XDR_VOL uint32_t ICR;      /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  XDR_VOL uint32_t PECR;     /*!< I2C PEC register,                  Address offset: 0x20 */
  XDR_VOL uint32_t RXDR;     /*!< I2C Receive data register,         Address offset: 0x24 */
  XDR_VOL uint32_t TXDR;     /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/**
  * Reset and Clock Control
  */

typedef struct
{
  XDR_VOL uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  XDR_VOL uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  XDR_VOL uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  XDR_VOL uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  XDR_VOL uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  XDR_VOL uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  XDR_VOL uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  XDR_VOL uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  XDR_VOL uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  XDR_VOL uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  XDR_VOL uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  XDR_VOL uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  XDR_VOL uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  XDR_VOL uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  XDR_VOL uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  XDR_VOL uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  XDR_VOL uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  XDR_VOL uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  XDR_VOL uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  XDR_VOL uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  XDR_VOL uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  XDR_VOL uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  XDR_VOL uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  XDR_VOL uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  XDR_VOL uint32_t DCKCFGR1;      /*!< RCC Dedicated Clocks configuration register1,                 Address offset: 0x8C */
  XDR_VOL uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x90 */

} RCC_TypeDef;

/**
  * Serial Peripheral Interface
  */

typedef struct
{
  XDR_VOL uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  XDR_VOL uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  XDR_VOL uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  XDR_VOL uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  XDR_VOL uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  XDR_VOL uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  XDR_VOL uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  XDR_VOL uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  XDR_VOL uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

/**
  * Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  XDR_VOL uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
  XDR_VOL uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
  XDR_VOL uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
  XDR_VOL uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
  XDR_VOL uint32_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  XDR_VOL uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  XDR_VOL uint32_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
  XDR_VOL uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
  XDR_VOL uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  XDR_VOL uint32_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
  XDR_VOL uint32_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
} USART_TypeDef;


/**
 * STM32F7xx Interrupt Number Definition, according to the selected device
 *
 */
typedef enum
{
/******  Cortex-M7 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M7 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M7 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M7 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M7 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M7 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M7 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M7 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                             */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                       */
  DMA2D_IRQn                  = 90,     /*!< DMA2D global Interrupt                                            */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< Quad SPI global interrupt                                         */
  LPTIM1_IRQn                 = 93,     /*!< LP TIM1 interrupt                                                 */
  CEC_IRQn                    = 94,     /*!< HDMI-CEC global Interrupt                                         */
  I2C4_EV_IRQn                = 95,     /*!< I2C4 Event Interrupt                                              */
  I2C4_ER_IRQn                = 96,     /*!< I2C4 Error Interrupt                                              */
  SPDIF_RX_IRQn               = 97,     /*!< SPDIF-RX global Interrupt                                         */
  DFSDM1_FLT0_IRQn	          = 99,     /*!< DFSDM1 Filter 0 global Interrupt                                  */
  DFSDM1_FLT1_IRQn	          = 100,    /*!< DFSDM1 Filter 1 global Interrupt                                  */
  DFSDM1_FLT2_IRQn	          = 101,    /*!< DFSDM1 Filter 2 global Interrupt                                  */
  DFSDM1_FLT3_IRQn	          = 102,    /*!< DFSDM1 Filter 3 global Interrupt                                  */
  SDMMC2_IRQn                 = 103,    /*!< SDMMC2 global Interrupt                                           */
  CAN3_TX_IRQn                = 104,    /*!< CAN3 TX Interrupt                                                 */
  CAN3_RX0_IRQn               = 105,    /*!< CAN3 RX0 Interrupt                                                */
  CAN3_RX1_IRQn               = 106,    /*!< CAN3 RX1 Interrupt                                                */
  CAN3_SCE_IRQn               = 107,    /*!< CAN3 SCE Interrupt                                                */
  JPEG_IRQn                   = 108,    /*!< JPEG global Interrupt                                             */
  MDIOS_IRQn                  = 109     /*!< MDIO Slave global Interrupt                                       */
} IRQn_Type;

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define RAMITCM_BASE           0x00000000UL /*!< Base address of : 16KB RAM reserved for CPU execution/instruction accessible over ITCM  */
#define FLASHITCM_BASE         0x00200000UL /*!< Base address of : (up to 2 MB) embedded FLASH memory  accessible over ITCM              */
#define FLASHAXI_BASE          0x08000000UL /*!< Base address of : (up to 2 MB) embedded FLASH memory accessible over AXI                */
#define RAMDTCM_BASE           0x20000000UL /*!< Base address of : 128KB system data RAM accessible over DTCM                            */
#define PERIPH_BASE            0x40000000UL /*!< Base address of : AHB/ABP Peripherals                                                   */
#define BKPSRAM_BASE           0x40024000UL /*!< Base address of : Backup SRAM(4 KB)                                                     */
#define QSPI_BASE              0x90000000UL /*!< Base address of : QSPI memories  accessible over AXI                                    */
#define FMC_R_BASE             0xA0000000UL /*!< Base address of : FMC Control registers                                                 */
#define QSPI_R_BASE            0xA0001000UL /*!< Base address of : QSPI Control  registers                                               */
#define SRAM1_BASE             0x20020000UL /*!< Base address of : 368KB RAM1 accessible over AXI/AHB                                    */
#define SRAM2_BASE             0x2007C000UL /*!< Base address of : 16KB RAM2 accessible over AXI/AHB                                     */
#define FLASH_END              0x081FFFFFUL /*!< FLASH end address */
#define FLASH_OTP_BASE         0x1FF0F000UL /*!< Base address of : (up to 1024 Bytes) embedded FLASH OTP Area                            */
#define FLASH_OTP_END          0x1FF0F41FUL /*!< End address of : (up to 1024 Bytes) embedded FLASH OTP Area                             */

/* Legacy define */
#define FLASH_BASE     FLASHAXI_BASE

/* ARM Cortex-M NVIC register addresses */
#define XDR_NVIC_ISER   ((XDR_VOL uint32_t *)0xE000E100UL)
#define XDR_NVIC_ICER   ((XDR_VOL uint32_t *)0xE000E180UL)

/*!< Peripheral memory map */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000UL)
#define GPIOJ_BASE            (AHB1PERIPH_BASE + 0x2400UL)
#define GPIOK_BASE            (AHB1PERIPH_BASE + 0x2800UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define FLASHSIZE_BASE        0x1FF0F442UL

/*!< APB1 peripherals */
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C4_BASE             (APB1PERIPH_BASE + 0x6000UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)

/*!< APB2 peripherals */
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define SPI5_BASE             (APB2PERIPH_BASE + 0x5000UL)
#define SPI6_BASE             (APB2PERIPH_BASE + 0x5400UL)

/** Peripheral Declaration */
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C4                ((I2C_TypeDef *) I2C4_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define USART1				((USART_TypeDef *) USART1_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define USART6				((USART_TypeDef *) USART6_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ               ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK               ((GPIO_TypeDef *) GPIOK_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)


	// Clock enable macro for SYSCFG
	#define SYSCFG_CLOCK_ENABLE()	(RCC->APB2ENR |= (1 << 14) )

	// clock disable macro for SYSCFG
	#define SYSCFG_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 14) )

#endif /* STM32F767XX_H_ */

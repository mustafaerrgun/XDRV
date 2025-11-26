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
#define NVIC_ISER0   ((XDR_VOL uint32_t*)0xE000E100)
#define NVIC_ISER1   ((XDR_VOL uint32_t*)0xE000E104)
#define NVIC_ISER2   ((XDR_VOL uint32_t*)0xE000E108)

#define NVIC_ICER0   ((XDR_VOL uint32_t*)0xE000E180)
#define NVIC_ICER1   ((XDR_VOL uint32_t*)0xE000E184)
#define NVIC_ICER2   ((XDR_VOL uint32_t*)0xE000E188)

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
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
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
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
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


/** Peripheral Clock Enable Macros
  *
  */
	// Clock enable macros for I2Cx
	#define I2C1_CLOCK_ENABLE()		(RCC->APB1ENR |= (1 << 21) )
	#define I2C2_CLOCK_ENABLE()		(RCC->APB1ENR |= (1 << 22) )
	#define I2C3_CLOCK_ENABLE()		(RCC->APB1ENR |= (1 << 23) )
	#define I2C4_CLOCK_ENABLE()		(RCC->APB1ENR |= (1 << 24) )

	// Clock enable macros for SPIx
	#define SPI1_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1 << 12) )
	#define SPI2_CLOCK_ENABLE()		(RCC->APB1ENR |= (1 << 14) )
	#define SPI3_CLOCK_ENABLE()		(RCC->APB1ENR |= (1 << 15) )
	#define SPI4_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1 << 13) )
	#define SPI5_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1 << 20) )
	#define SPI6_CLOCK_ENABLE()	    (RCC->APB2ENR |= (1 << 21) )

	//Clock enable macros for USARTx
	#define USART1_CLOCK_ENABLE()	(RCC->APB2ENR |= (1 << 4) )
	#define USART2_CLOCK_ENABLE()	(RCC->APB1ENR |= (1 << 17) )
	#define USART3_CLOCK_ENABLE()	(RCC->APB1ENR |= (1 << 18) )
	#define USART6_CLOCK_ENABLE()	(RCC->APB2ENR |= (1 << 5) )

	// Clock enable macro for SYSCFG
	#define SYSCFG_CLOCK_ENABLE()	(RCC->APB2ENR |= (1 << 14) )

/** Peripheral Clock Disable Macros
  *
  */
	// Clock disable macros for I2Cx
	#define I2C1_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 21) )
	#define I2C2_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 22) )
	#define I2C3_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 23) )
	#define I2C4_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 24) )

	// Clock disable macros for SPIx
	#define SPI1_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 12) )
	#define SPI2_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 14) )
	#define SPI3_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 15) )
	#define SPI4_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 13) )
	#define SPI5_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 20) )
	#define SPI6_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 21) )

	// Clock disable macros for USARTx
	#define USART1_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 4) )
	#define USART2_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 17) )
	#define USART3_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1 << 18) )
	#define USART6_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 5) )

	// clock disable macro for SYSCFG
	#define SYSCFG_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1 << 14) )

#endif /* STM32F767XX_H_ */

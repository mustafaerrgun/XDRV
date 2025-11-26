# XDR - eXperimental Driver
Bare-metal STM32F767 firmware drivers (GPIO, RCC, USART, I2C, SPI) built from scratch. 

## 1. Overview

**XDR (eXperimental Driver)** is a lightweight, low-level driver framework for the STM32F767 microcontroller family.

It is designed without HAL or LL, focusing on:

- Full control of registers
- Clear peripheral abstraction
- Continuous Integration (CI) pipeline with MISRA compliance checks

The project currently implements:

1. **GPIO Driver** — complete digital I/O configuration and runtime API
2. **RCC Driver** — system clock configuration using HSI/PLL, AHB/APB prescalers, and flash wait-states

The drivers rely on a custom reduced CMSIS-style header ([stm32f767xx.h](Drivers/Inc/stm32f767xx.h)) which defines all relevant register maps.

## 2. XDR-GPIO Driver

### The GPIO module provides:

- Pin configuration (Mode, Type, Speed, Pull-up/down, Alternate Function)
- Pin-level read/write
- Port-level read/write
- Pin toggle
- Clock enable/disable per GPIO port

### Public APIs:

```
void XDR_GPIO_Init(XDR_GPIO_Handle *GPIO_Handle);
void XDR_GPIO_DeInit(GPIO_TypeDef *GPIOx);

uint8_t  XDR_GPIO_Read_Pin(GPIO_TypeDef *GPIOx, uint8_t Pin);
uint16_t XDR_GPIO_Read_Port(GPIO_TypeDef *GPIOx);

void XDR_GPIO_Write_Pin(GPIO_TypeDef *GPIOx, uint8_t Pin, uint8_t Value);
void XDR_GPIO_Write_Port(GPIO_TypeDef *GPIOx, uint16_t Value);

void XDR_GPIO_Toggle(GPIO_TypeDef *GPIOx, uint8_t Pin);
```

## 3. XDR-RCC Driver

### The RCC module supports:

- Switching system clock source (HSI or PLL)
- Auto-configuration of:
  - PLL multipliers/dividers (hardcoded for stability)
  - AHB/APB1/APB2 prescalers
  - Required flash wait states

### Supported System Clock Options
- 16 MHz
- 48 MHz
- 96 MHz
- 144 MHz
- 216 MHz

### Public APIs
```
void     XDR_RCC_Init       (XDR_RCC_Handle *RCC_Handle);
uint32_t XDR_Get_SysClock   (XDR_RCC_Handle *RCC_Handle);
```

## 4. Implementation Notes

### Custom CMSIS Header

A minimal version of the STM32F767 register map is provided in [stm32f767xx.h](Drivers/Inc/stm32f767xx.h)

This header includes:

- RCC, GPIO, FLASH, EXTI, SYSCFG register definitions  
- Full peripheral memory map  
- NVIC register definitions  
- Peripheral base addresses  
- Peripheral register structures 

### PLL Hardcoded Configurations

For learning, PLL configurations are currently implemented as **predefined configurations**, such as:

- `XDR_RCC_PLLCFGR_48MHZ`  
- `XDR_RCC_PLLCFGR_96MHZ`  
- `XDR_RCC_PLLCFGR_144MHZ`  
- `XDR_RCC_PLLCFGR_216MHZ`  

These values are applied inside the RCC driver implementation  
(in [stm32f767xx_xdr_rcc.c](Drivers/Src/stm32f767xx_xdr_rcc.c)).

### Flash Wait States

Flash latency is automatically configured based on the selected **SYSCLK** frequency.  

## 5. Goals & Roadmap

### Completed
- GPIO Driver  
- RCC Driver  
- Minimal CMSIS Hardware Layer 
- MISRA Compliance Checker Integration 

### In Progress / Planned
- USART Driver  
- SPI Driver  
- I2C Driver
/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_usart.c
  * @author  Mustafa Ergün
  * @brief   USART XDR module driver.
  *          This file provides firmware functions to manage the USART peripheral
  ******************************************************************************
  */

#include "stm32f767xx_xdr_usart.h"
#include "stm32f767xx_xdr_gpio.h"
#include "stm32f767xx_xdr_rcc.h"

// Private APIs
static void XDR_USART_Clock_Enable(xdr_usart *xdr_usart);
static void XDR_USART_GPIO_Init(xdr_usart *xdr_usart);
static uint32_t XDR_USART_BRR_Calculation(xdr_usart *xdr_usart);


void XDR_USART_Init(xdr_usart *xdr_usart){

	// Enable Clock for USART
	XDR_USART_Clock_Enable(xdr_usart);

	// Configure GPIO pins for USART
	XDR_USART_GPIO_Init(xdr_usart);

	// Clear CR1 register
	xdr_usart->usart->CR1 = XDR_USART_CR1_CLEAR;

	// Use TX-RX Mode
	xdr_usart->usart->CR1 |= (1UL << XDR_USART_CR1_RE)
					      |  (1UL << XDR_USART_CR1_TE);

	// Use Oversampling 16
	xdr_usart->usart->CR1 |= (XDR_USART_OVERSAMPLING_16 << XDR_USART_CR1_OVER8);

	// Calculate BRR register value for specified baud rate
	xdr_usart->usart->BRR =XDR_USART_BRR_Calculation(xdr_usart);

	// Enable USART Module
	xdr_usart->usart->CR1 |= (1UL << XDR_USART_CR1_UE);

}

static void XDR_USART_GPIO_Init(xdr_usart *xdr_usart){

    XDR_GPIO_Handle gpio_tx;
    XDR_GPIO_Handle gpio_rx;

    gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinMode     = GPIO_MODE_ALTFN;
    gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinOSpeed   = GPIO_SPEED_FAST;
    gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinOType    = GPIO_OP_TYPE_PP;
    gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinPUPDType = GPIO_NO_PUPD;

    switch(xdr_usart->xdr_usart_instance){
    	case XDR_USART1:
    		// PA9 (TX) and PA10(RX) for USART1
    		gpio_tx.XDR_GPIOx  = GPIOA;
    		gpio_tx.XDR_PortId = XDR_GPIO_PORT_A;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_9;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 7;

    		gpio_rx.XDR_GPIOx  = GPIOA;
    		gpio_rx.XDR_PortId = XDR_GPIO_PORT_A;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_10;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 7;

    		break;
    	case XDR_USART2:
    		// PD5 (TX) and PD6(RX) for USART2
    		gpio_tx.XDR_GPIOx  = GPIOD;
    		gpio_tx.XDR_PortId = XDR_GPIO_PORT_D;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_5;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 7;

    		gpio_rx.XDR_GPIOx  = GPIOD;
    		gpio_rx.XDR_PortId = XDR_GPIO_PORT_D;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_6;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 7;

    		break;
    	case XDR_USART3:
    		// PD8 (TX) and PD9(RX) for USART3
    		gpio_tx.XDR_GPIOx  = GPIOD;
    		gpio_tx.XDR_PortId = XDR_GPIO_PORT_D;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_8;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 7;

    		gpio_rx.XDR_GPIOx  = GPIOD;
    		gpio_rx.XDR_PortId = XDR_GPIO_PORT_D;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_9;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 7;

    		break;
    	case XDR_USART6:
    		// PC6 (TX) and PC7 (RX) for USART6
    		gpio_tx.XDR_GPIOx  = GPIOC;
    		gpio_tx.XDR_PortId = XDR_GPIO_PORT_C;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_6;
    		gpio_tx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 8;

    		gpio_rx.XDR_GPIOx  = GPIOC;
    		gpio_rx.XDR_PortId = XDR_GPIO_PORT_C;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_Pin = GPIO_PIN_NO_7;
    		gpio_rx.XDR_GPIO_Config.XDR_GPIO_PinAFMode = 8;
    		break;
    	default: break;
    }

    XDR_GPIO_Init(&gpio_tx);
    XDR_GPIO_Init(&gpio_rx);
}

// Clock Enable API for USART Instances
static void XDR_USART_Clock_Enable(xdr_usart *xdr_usart){

    switch (xdr_usart->xdr_usart_instance)
    {
        case XDR_USART1: USART1_CLOCK_ENABLE(); xdr_usart->usart = USART1; break;
        case XDR_USART2: USART2_CLOCK_ENABLE(); xdr_usart->usart = USART2; break;
		case XDR_USART3: USART3_CLOCK_ENABLE(); xdr_usart->usart = USART3; break;
        case XDR_USART6: USART6_CLOCK_ENABLE(); xdr_usart->usart = USART6; break;
        default: break;
    }

}

static uint32_t XDR_USART_BRR_Calculation(xdr_usart *xdr_usart){

	uint32_t apb_clock = 0U;
	uint32_t baud      = xdr_usart->xdr_usart_baudrate;

	uint32_t mantissa = 0U;
	uint32_t fraction = 0U;
	uint32_t usartdiv = 0U;
	uint32_t brr = 0U;

	switch(xdr_usart->xdr_usart_instance)
	{
		case XDR_USART1: case XDR_USART6:
			apb_clock=XDR_Get_PCLK2(xdr_usart->XDR_RCC_Handle); break;
		case XDR_USART2: case XDR_USART3:
			apb_clock=XDR_Get_PCLK1(xdr_usart->XDR_RCC_Handle); break;
		default: break;
	}

	usartdiv = (apb_clock + (baud / 2U)) / baud;
	mantissa = usartdiv / 16U;
	fraction = usartdiv % 16U;
	brr = (mantissa << XDR_USART_BRR_Pos) | (fraction & XDR_USART_BRR_Over16Mask);

	return brr;
}

void XDR_USART_Send(xdr_usart *xdr_usart, uint8_t data){


	while((xdr_usart->usart->ISR & (1UL << XDR_USART_ISR_TXE)) == 0UL){ }

	xdr_usart->usart->TDR = data;

	while((xdr_usart->usart->ISR & (1UL << XDR_USART_ISR_TC)) == 0UL){ }
}

uint8_t XDR_USART_Receive(xdr_usart *xdr_usart){

	uint8_t data = 0;

	while ((xdr_usart->usart->ISR & (1UL << XDR_USART_ISR_RXNE)) == 0UL){ }

	data = (uint8_t)(xdr_usart->usart->RDR & 0xFFUL);

	return data;

}


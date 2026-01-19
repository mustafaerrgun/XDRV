/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_usart.c
  * @author  Mustafa Ergün
  * @brief   USART XDR module driver.
  *          This file provides firmware functions to manage the USART peripheral
  ******************************************************************************
  */

#include "stm32f767xx_xdr_usart.h"

// Private APIs
static void XDR_USART_Clock_Enable(xdr_usart *xdr_usart);
static void XDR_USART_GPIO_Init(const xdr_usart *xdr_usart);
static uint32_t XDR_USART_BRR_Calculation(const xdr_usart *xdr_usart);


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
	xdr_usart->usart->BRR = XDR_USART_BRR_Calculation(xdr_usart);

	// Enable USART Module
	xdr_usart->usart->CR1 |= (1UL << XDR_USART_CR1_UE);

}

static void XDR_USART_GPIO_Init(const xdr_usart *xdr_usart){

	xdr_gpio gpio_tx;
	xdr_gpio gpio_rx;

	gpio_tx.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
	gpio_tx.xdr_gpio_pinPuPd = GPIO_NO_PUPD;
	gpio_tx.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

	gpio_rx.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
	gpio_rx.xdr_gpio_pinPuPd = GPIO_NO_PUPD;
	gpio_rx.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

    switch(xdr_usart->xdr_usart_instance){
    	case XDR_USART1:
    		// PA9 (TX) and PA10(RX) for USART1
    		gpio_tx.xdr_gpiox  = GPIOA;
    		gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_A;
    		gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_9;
    		XDR_GPIO_Init(&gpio_tx);
    		gpio_tx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PA9);
    		gpio_tx.xdr_gpiox->AFR[1] |=  (7UL << XDR_USART_GPIO_AF_PA9);

    		gpio_rx.xdr_gpiox  = GPIOA;
    		gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_A;
    		gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_10;
    		XDR_GPIO_Init(&gpio_rx);
    		gpio_rx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PA10);
    		gpio_rx.xdr_gpiox->AFR[1] |=  (7UL << XDR_USART_GPIO_AF_PA10);

    		break;
    	case XDR_USART2:
    		// PD5 (TX) and PD6(RX) for USART2
    		gpio_tx.xdr_gpiox  = GPIOD;
    		gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_D;
    		gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_5;
    		XDR_GPIO_Init(&gpio_tx);
    		gpio_tx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PD5);
    		gpio_tx.xdr_gpiox->AFR[0] |=  (7UL << XDR_USART_GPIO_AF_PD5);

    		gpio_rx.xdr_gpiox  = GPIOD;
    		gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_D;
    		gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_6;
    		XDR_GPIO_Init(&gpio_rx);
    		gpio_rx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PD6);
    		gpio_rx.xdr_gpiox->AFR[0] |=  (7UL << XDR_USART_GPIO_AF_PD6);

    		break;
    	case XDR_USART3:
    		// PD8 (TX) and PD9(RX) for USART3
    		gpio_tx.xdr_gpiox  = GPIOD;
    		gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_D;
    		gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_8;
    		XDR_GPIO_Init(&gpio_tx);
    		gpio_tx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PD8);
    		gpio_tx.xdr_gpiox->AFR[1] |=  (7UL << XDR_USART_GPIO_AF_PD8);

    		gpio_rx.xdr_gpiox  = GPIOD;
    		gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_D;
    		gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_9;
    		XDR_GPIO_Init(&gpio_rx);
    		gpio_rx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PD9);
    		gpio_rx.xdr_gpiox->AFR[1] |=  (7UL << XDR_USART_GPIO_AF_PD9);

    		break;
    	case XDR_USART6:
    		// PC6 (TX) and PC7 (RX) for USART6
    		gpio_tx.xdr_gpiox  = GPIOC;
    		gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_C;
    		gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_6;
    		XDR_GPIO_Init(&gpio_tx);
    		gpio_tx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PC6);
    		gpio_tx.xdr_gpiox->AFR[0] |=  (8UL << XDR_USART_GPIO_AF_PC6);

    		gpio_rx.xdr_gpiox  = GPIOC;
    		gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_C;
    		gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_7;
    		XDR_GPIO_Init(&gpio_rx);
    		gpio_rx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PC7);
    		gpio_rx.xdr_gpiox->AFR[0] |=  (8UL << XDR_USART_GPIO_AF_PC7);
    		break;
    	default: break;
    }
}

// Clock Enable API for USART Instances
static void XDR_USART_Clock_Enable(xdr_usart *xdr_usart){

    switch (xdr_usart->xdr_usart_instance)
    {
        case XDR_USART1: USART1_CLOCK_SOURCE(); USART1_CLOCK_ENABLE(); xdr_usart->usart = USART1; break;
        case XDR_USART2: USART2_CLOCK_SOURCE(); USART2_CLOCK_ENABLE(); xdr_usart->usart = USART2; break;
		case XDR_USART3: USART3_CLOCK_SOURCE(); USART3_CLOCK_ENABLE(); xdr_usart->usart = USART3; break;
        case XDR_USART6: USART6_CLOCK_SOURCE(); USART6_CLOCK_ENABLE(); xdr_usart->usart = USART6; break;
        default: break;
    }

}

static uint32_t XDR_USART_BRR_Calculation(const xdr_usart *xdr_usart){


	uint32_t baud      = xdr_usart->xdr_usart_baudrate;

	uint32_t mantissa = 0U;
	uint32_t fraction = 0U;
	uint32_t usartdiv = 0U;
	uint32_t brr = 0U;

	usartdiv = (XDR_HSI_CLK + (baud / 2U)) / baud;
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


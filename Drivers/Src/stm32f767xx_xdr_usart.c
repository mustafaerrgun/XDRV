/**
  ******************************************************************************
  * @file    stm32f7xx_xdr_usart.c
  * @author  Mustafa Ergün
  * @brief   USART XDR module driver.
  *          This file provides firmware functions to manage the USART peripheral
  ******************************************************************************
  */

#include "stm32f767xx_xdr_usart.h"
#include "stm32f767xx_xdr_rcc.h"

// Private APIs
static void XDR_USART_Clock_Enable(XDR_USART_Handle *USART_Handle);
static uint32_t XDR_USART_BRR_Calculation(XDR_USART_Handle *USART_Handle);


void XDR_USART_Init(XDR_USART_Handle *USART_Handle){

	XDR_USART_Clock_Enable(USART_Handle);

	USART_Handle->XDR_USART->CR1 = XDR_USART_CR1_CLEAR;

	switch(USART_Handle->XDR_USART_Config.XDR_USART_Mode)
	{
		case XDR_USART_MODE_RX	 : USART_Handle->XDR_USART->CR1 |= (1UL << XDR_USART_CR1_RE); break;
		case XDR_USART_MODE_TX	 : USART_Handle->XDR_USART->CR1 |= (1UL << XDR_USART_CR1_TE); break;
		case XDR_USART_MODE_TX_RX: USART_Handle->XDR_USART->CR1 |= (1UL << XDR_USART_CR1_RE)
																 | (1UL << XDR_USART_CR1_TE); break;
		default: break;
	}

	USART_Handle->XDR_USART->CR1 |= (XDR_USART_OVERSAMPLING_16 << XDR_USART_CR1_OVER8);

	USART_Handle->XDR_USART->BRR =XDR_USART_BRR_Calculation(USART_Handle);

	switch(USART_Handle->XDR_USART_Config.XDR_USART_WordLength)
	{
		case XDR_USART_WORDLENGTH_7B : USART_Handle->XDR_USART->CR1 |= (1UL << XDR_USART_CR1_M1); break;
		case XDR_USART_WORDLENGTH_8B : break;
		case XDR_USART_WORDLENGTH_9B : USART_Handle->XDR_USART->CR1 |= (1UL << XDR_USART_CR1_M0); break;
		default: break;
	}

	// Enable USART Module
	USART_Handle->XDR_USART->CR1 |= (1UL << XDR_USART_CR1_UE);

}

// Clock Control APIs for USART
static void XDR_USART_Clock_Enable(XDR_USART_Handle *USART_Handle){

    switch (USART_Handle->XDR_USART_Instance)
    {
        case XDR_USART1: USART1_CLOCK_ENABLE(); USART_Handle->XDR_USART=USART1; break;
        case XDR_USART2: USART2_CLOCK_ENABLE(); USART_Handle->XDR_USART=USART2; break;
		case XDR_USART3: USART3_CLOCK_ENABLE(); USART_Handle->XDR_USART=USART3; break;
        case XDR_USART6: USART6_CLOCK_ENABLE(); USART_Handle->XDR_USART=USART6; break;
        default: break;
    }

}

static uint32_t XDR_USART_BRR_Calculation(XDR_USART_Handle *USART_Handle){

	uint32_t apb_clock = 0U;
	uint32_t baud      = USART_Handle->XDR_USART_Config.XDR_USART_BaudRate;

	uint32_t mantissa = 0U;
	uint32_t fraction = 0U;
	uint32_t usartdiv = 0U;
	uint32_t brr = 0U;

	switch(USART_Handle->XDR_USART_Instance)
	{
		case XDR_USART1: case XDR_USART6:
			apb_clock=XDR_Get_PCLK2(USART_Handle->XDR_RCC_Handle); break;
		case XDR_USART2: case XDR_USART3:
			apb_clock=XDR_Get_PCLK1(USART_Handle->XDR_RCC_Handle); break;
		default: break;
	}

	usartdiv = (apb_clock + (baud / 2UL)) / baud;
	mantissa = usartdiv / 16UL;
	fraction = usartdiv % 16UL;
	brr = (mantissa << XDR_USART_BRR_Pos) | (fraction & XDR_USART_BRR_Over16Mask);

	return brr;
}

void XDR_USART_Send(XDR_USART_Handle *USART_Handle, uint8_t data){


	while((USART_Handle->XDR_USART->ISR & (1UL << XDR_USART_ISR_TXE)) == 0UL){ }

	USART_Handle->XDR_USART->TDR = data;

	while((USART_Handle->XDR_USART->ISR & (1UL << XDR_USART_ISR_TC)) == 0UL){ }
}

uint8_t XDR_USART_Receive(XDR_USART_Handle *USART_Handle){

	uint8_t data = 0;

	while ((USART_Handle->XDR_USART->ISR & (1UL << XDR_USART_ISR_RXNE)) == 0UL){ }

	data = (uint8_t)(USART_Handle->XDR_USART->RDR & 0xFFUL);

	return data;

}


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
static void XDR_USART3_EnableRxInterrupt(xdr_usart *xdr_usart);
static void XDR_USART3_EnableDMA(xdr_usart *xdr_usart);
// Variables
static xdr_usart *usart_irq;
char USART3_Data_Buffer[USART_BUFF_SIZE];

void XDR_USART_Init(xdr_usart *xdr_usart)
{

    // Enable Clock for USART
    XDR_USART_Clock_Enable(xdr_usart);

    // Configure GPIO pins for USART
    XDR_USART_GPIO_Init(xdr_usart);

    // Clear CR1 register
    xdr_usart->usart->CR1 = XDR_USART_CR1_CLEAR;

    // Use TX-RX Mode
    xdr_usart->usart->CR1 |= (1UL << XDR_USART_CR1_RE) | (1UL << XDR_USART_CR1_TE);

    // Use Oversampling 16
    xdr_usart->usart->CR1 |= (XDR_USART_OVERSAMPLING_16 << XDR_USART_CR1_OVER8);

    // Calculate BRR register value for specified baud rate
    xdr_usart->usart->BRR = XDR_USART_BRR_Calculation(xdr_usart);

    // Enable USART Module
    xdr_usart->usart->CR1 |= (1UL << XDR_USART_CR1_UE);

    // Enable interrupt for USART3
    if (xdr_usart->xdr_usart_interrupt != 0U)
    {
        XDR_USART3_EnableRxInterrupt(xdr_usart);
    }

    if (xdr_usart->xdr_usart_dma != 0U)
    {
        XDR_USART3_EnableDMA(xdr_usart);
    }
}

static void XDR_USART_GPIO_Init(const xdr_usart *xdr_usart)
{

    xdr_gpio gpio_tx;
    xdr_gpio gpio_rx;

    gpio_tx.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
    gpio_tx.xdr_gpio_pinPuPd = GPIO_NO_PUPD;
    gpio_tx.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

    gpio_rx.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
    gpio_rx.xdr_gpio_pinPuPd = GPIO_NO_PUPD;
    gpio_rx.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

    switch (xdr_usart->xdr_usart_instance)
    {
    case XDR_USART1:
        // PA9 (TX) and PA10(RX) for USART1
        gpio_tx.xdr_gpiox = GPIOA;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_A;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_9;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PA9);
        gpio_tx.xdr_gpiox->AFR[1] |= (7UL << XDR_USART_GPIO_AF_PA9);

        gpio_rx.xdr_gpiox = GPIOA;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_A;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_10;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PA10);
        gpio_rx.xdr_gpiox->AFR[1] |= (7UL << XDR_USART_GPIO_AF_PA10);

        break;
    case XDR_USART2:
        // PD5 (TX) and PD6(RX) for USART2
        gpio_tx.xdr_gpiox = GPIOD;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_5;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PD5);
        gpio_tx.xdr_gpiox->AFR[0] |= (7UL << XDR_USART_GPIO_AF_PD5);

        gpio_rx.xdr_gpiox = GPIOD;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_6;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PD6);
        gpio_rx.xdr_gpiox->AFR[0] |= (7UL << XDR_USART_GPIO_AF_PD6);

        break;
    case XDR_USART3:
        // PD8 (TX) and PD9(RX) for USART3
        gpio_tx.xdr_gpiox = GPIOD;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_8;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PD8);
        gpio_tx.xdr_gpiox->AFR[1] |= (7UL << XDR_USART_GPIO_AF_PD8);

        gpio_rx.xdr_gpiox = GPIOD;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_9;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_USART_GPIO_AF_PD9);
        gpio_rx.xdr_gpiox->AFR[1] |= (7UL << XDR_USART_GPIO_AF_PD9);

        break;
    case XDR_USART6:
        // PC6 (TX) and PC7 (RX) for USART6
        gpio_tx.xdr_gpiox = GPIOC;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_C;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_6;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PC6);
        gpio_tx.xdr_gpiox->AFR[0] |= (8UL << XDR_USART_GPIO_AF_PC6);

        gpio_rx.xdr_gpiox = GPIOC;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_C;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_7;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_USART_GPIO_AF_PC7);
        gpio_rx.xdr_gpiox->AFR[0] |= (8UL << XDR_USART_GPIO_AF_PC7);
        break;
    default:
        break;
    }
}

// Clock Enable API for USART Instances
static void XDR_USART_Clock_Enable(xdr_usart *xdr_usart)
{

    switch (xdr_usart->xdr_usart_instance)
    {
    case XDR_USART1:
        USART1_CLOCK_SOURCE();
        USART1_CLOCK_ENABLE();
        xdr_usart->usart = USART1;
        break;
    case XDR_USART2:
        USART2_CLOCK_SOURCE();
        USART2_CLOCK_ENABLE();
        xdr_usart->usart = USART2;
        break;
    case XDR_USART3:
        USART3_CLOCK_SOURCE();
        USART3_CLOCK_ENABLE();
        xdr_usart->usart = USART3;
        break;
    case XDR_USART6:
        USART6_CLOCK_SOURCE();
        USART6_CLOCK_ENABLE();
        xdr_usart->usart = USART6;
        break;
    default:
        break;
    }
}

static uint32_t XDR_USART_BRR_Calculation(const xdr_usart *xdr_usart)
{

    uint32_t baud = xdr_usart->xdr_usart_baudrate;

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

void XDR_USART_Send(xdr_usart *xdr_usart, uint8_t data)
{

    while ((xdr_usart->usart->ISR & (1UL << XDR_USART_ISR_TXE)) == 0UL)
    {
    }

    xdr_usart->usart->TDR = data;

    while ((xdr_usart->usart->ISR & (1UL << XDR_USART_ISR_TC)) == 0UL)
    {
    }
}

uint8_t XDR_USART_Receive(xdr_usart *xdr_usart)
{

    uint8_t data = 0;

    while ((xdr_usart->usart->ISR & (1UL << XDR_USART_ISR_RXNE)) == 0UL)
    {
    }

    data = (uint8_t)(xdr_usart->usart->RDR & 0xFFUL);

    return data;
}

static void XDR_USART3_EnableDMA(xdr_usart *xdr_usart)
{
    // Enable DMA for USART3 CR3
    xdr_usart->usart->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
    /*Enable clock access to DMA1*/
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    /*Enable DMA Stream3 Interrupt in NVIC*/
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void XDR_USART3_DMA_Rx_Config(void)
{
    /*Disable DMA stream*/
    DMA1_Stream1->CR &= ~DMA_SxCR_EN;
    /*Wait till DMA Stream is disabled*/
    while ((DMA1_Stream1->CR & DMA_SxCR_EN) != 0U)
    {
    }
    /*Clear interrupt flags for stream 1*/
    DMA1->LIFCR = DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1;
    /*Set periph address*/
    DMA1_Stream1->PAR = (uint32_t)(&(USART3->RDR));
    /*Set mem address*/
    DMA1_Stream1->M0AR = (uint32_t)(&USART3_Data_Buffer);
    /*Set number of transfer*/
    DMA1_Stream1->NDTR = (uint16_t)USART_BUFF_SIZE;
    /*Select Channel 4*/
    DMA1_Stream1->CR &= ~DMA_SxCR_CHSEL_0;
    DMA1_Stream1->CR &= ~DMA_SxCR_CHSEL_1;
    DMA1_Stream1->CR |= DMA_SxCR_CHSEL_2;
    /*Enable memory addr increment*/
    DMA1_Stream1->CR |= DMA_SxCR_MINC;
    /*Enable transfer complete interrupt*/
    DMA1_Stream1->CR |= DMA_SxCR_TCIE;
    /*Enable Circular mode*/
    DMA1_Stream1->CR |= DMA_SxCR_CIRC;
    /*Set transfer direction : Periph to Mem*/
    DMA1_Stream1->CR &= ~DMA_SxCR_DIR_0;
    DMA1_Stream1->CR &= ~DMA_SxCR_DIR_1;
    /*Enable DMA stream*/
    DMA1_Stream1->CR |= DMA_SxCR_EN;
    /*Enable DMA Stream1 Interrupt in NVIC*/
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void XDR_USART3_DMA_Tx_Config(uint32_t data, uint32_t length)
{
    /*Disable DMA stream*/
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    /*Wait till  DMA Stream is disabled*/
    while ((DMA1_Stream3->CR & DMA_SxCR_EN) != 0U)
    {
    }
    /*Clear interrupt flags for stream 3*/
    DMA1->LIFCR = DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3;
    /*Set periph address*/
    DMA1_Stream3->PAR = (uint32_t)(&(USART3->TDR));
    /*Set mem address*/
    DMA1_Stream3->M0AR = data;
    /*Set number of transfer*/
    DMA1_Stream3->NDTR = length;
    /*Select Channel 4*/
    DMA1_Stream3->CR &= ~DMA_SxCR_CHSEL_0;
    DMA1_Stream3->CR &= ~DMA_SxCR_CHSEL_1;
    DMA1_Stream3->CR |= DMA_SxCR_CHSEL_2;
    /*Enable memory addr increment*/
    DMA1_Stream3->CR |= DMA_SxCR_MINC;
    /*Set transfer direction :Mem to Periph*/
    DMA1_Stream3->CR |= DMA_SxCR_DIR_0;
    DMA1_Stream3->CR &= ~DMA_SxCR_DIR_1;
    /*Set transfer complete interrupt*/
    DMA1_Stream3->CR |= DMA_SxCR_TCIE;
    /*Enable DMA stream*/
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void DMA1_Stream1_IRQHandler(void)
{
    if ((DMA1->LISR & DMA_LISR_TCIF1) != 0U)
    {
        /* Do Something */
        XDR_USART3_DMA_Rx_Callback();
        /*Clear the flag*/
        DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
    }
}

void DMA1_Stream3_IRQHandler(void)
{
    if ((DMA1->LISR & DMA_LISR_TCIF3) != 0U)
    {
        /* Do Something */
        XDR_USART3_DMA_Tx_Callback();
        /*Clear the flag*/
        DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
    }
}

void XDR_USART3_EnableRxInterrupt(xdr_usart *xdr_usart)
{
    /* Use global xdr_usart */
    usart_irq = xdr_usart;
    // Enable RXNE interrupt
    xdr_usart->usart->CR1 |= USART_CR1_RXNEIE;
    /*Enable USART3_IRQn in NVIC*/
    NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{

    // Check RXNE flag AND RXNE interrupt enable
    if (((usart_irq->usart->ISR & USART_ISR_RXNE) != 0U) &&
        ((usart_irq->usart->CR1 & USART_CR1_RXNEIE) != 0U))
    {
        // Reading RDR clears RXNE
        uint8_t ch = (uint8_t)usart_irq->usart->RDR;

        // Pass data to user hook (best: give parameter)
        XDR_USART3_RxCallback(ch);
    }
}
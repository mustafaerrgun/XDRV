/**
 ******************************************************************************
 * @file    xdr_uart.c
 * @author  Mustafa Ergün
 * @brief   UART XDR module driver.
 *          This file provides firmware functions to manage the UART peripheral
 ******************************************************************************
 */

#include "xdr_uart.h"

// Private APIs
static void XDR_UART_Clock_Enable(xdr_uart *xdr_uart);
static void XDR_UART_GPIO_Init(const xdr_uart *xdr_uart);
static uint32_t XDR_UART_BRR_Calculation(const xdr_uart *xdr_uart);
static void XDR_UART3_EnableRxInterrupt(xdr_uart *xdr_uart);
static void XDR_UART3_EnableDMA(xdr_uart *xdr_uart);
// Variables
static xdr_uart *uart_irq;
/* cppcheck-suppress misra-c2012-8.7 ; External linkage is required: buffer is declared extern in xdr_uart.h for DMA access by application code */
char UART3_Data_Buffer[UART_BUFF_SIZE];

void XDR_UART_Init(xdr_uart *xdr_uart)
{

    // Enable Clock for UART
    XDR_UART_Clock_Enable(xdr_uart);

    // Configure GPIO pins for UART
    XDR_UART_GPIO_Init(xdr_uart);

    // Clear CR1 register
    xdr_uart->uart->CR1 = XDR_UART_CR1_CLEAR;

    // Use TX-RX Mode
    xdr_uart->uart->CR1 |= (1UL << XDR_UART_CR1_RE) | (1UL << XDR_UART_CR1_TE);

    // Use Oversampling 16
    xdr_uart->uart->CR1 |= (XDR_UART_OVERSAMPLING_16 << XDR_UART_CR1_OVER8);

    // Calculate BRR register value for specified baud rate
    xdr_uart->uart->BRR = XDR_UART_BRR_Calculation(xdr_uart);

    // Enable UART Module
    xdr_uart->uart->CR1 |= (1UL << XDR_UART_CR1_UE);

    // Enable interrupt for UART3
    if (xdr_uart->xdr_uart_interrupt != 0U)
    {
        XDR_UART3_EnableRxInterrupt(xdr_uart);
    }

    if (xdr_uart->xdr_uart_dma != 0U)
    {
        XDR_UART3_EnableDMA(xdr_uart);
    }
}

static void XDR_UART_GPIO_Init(const xdr_uart *xdr_uart)
{

    xdr_gpio gpio_tx;
    xdr_gpio gpio_rx;

    gpio_tx.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
    gpio_tx.xdr_gpio_pinPuPd = GPIO_NO_PUPD;
    gpio_tx.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

    gpio_rx.xdr_gpio_pinMode = GPIO_MODE_ALTFN;
    gpio_rx.xdr_gpio_pinPuPd = GPIO_NO_PUPD;
    gpio_rx.xdr_gpio_pinOType = GPIO_OP_TYPE_PP;

    switch (xdr_uart->xdr_uart_instance)
    {
    case XDR_UART1:
        // PA9 (TX) and PA10(RX) for UART1
        gpio_tx.xdr_gpiox = GPIOA;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_A;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_9;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_UART_GPIO_AF_PA9);
        gpio_tx.xdr_gpiox->AFR[1] |= (7UL << XDR_UART_GPIO_AF_PA9);

        gpio_rx.xdr_gpiox = GPIOA;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_A;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_10;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_UART_GPIO_AF_PA10);
        gpio_rx.xdr_gpiox->AFR[1] |= (7UL << XDR_UART_GPIO_AF_PA10);

        break;
    case XDR_UART2:
        // PD5 (TX) and PD6(RX) for UART2
        gpio_tx.xdr_gpiox = GPIOD;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_5;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_UART_GPIO_AF_PD5);
        gpio_tx.xdr_gpiox->AFR[0] |= (7UL << XDR_UART_GPIO_AF_PD5);

        gpio_rx.xdr_gpiox = GPIOD;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_6;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_UART_GPIO_AF_PD6);
        gpio_rx.xdr_gpiox->AFR[0] |= (7UL << XDR_UART_GPIO_AF_PD6);

        break;
    case XDR_UART3:
        // PD8 (TX) and PD9(RX) for UART3
        gpio_tx.xdr_gpiox = GPIOD;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_8;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_UART_GPIO_AF_PD8);
        gpio_tx.xdr_gpiox->AFR[1] |= (7UL << XDR_UART_GPIO_AF_PD8);

        gpio_rx.xdr_gpiox = GPIOD;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_D;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_9;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[1] &= ~(0xFUL << XDR_UART_GPIO_AF_PD9);
        gpio_rx.xdr_gpiox->AFR[1] |= (7UL << XDR_UART_GPIO_AF_PD9);

        break;
    case XDR_UART6:
        // PC6 (TX) and PC7 (RX) for UART6
        gpio_tx.xdr_gpiox = GPIOC;
        gpio_tx.xdr_gpio_portId = XDR_GPIO_PORT_C;
        gpio_tx.xdr_gpio_pin = GPIO_PIN_NO_6;
        XDR_GPIO_Init(&gpio_tx);
        gpio_tx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_UART_GPIO_AF_PC6);
        gpio_tx.xdr_gpiox->AFR[0] |= (8UL << XDR_UART_GPIO_AF_PC6);

        gpio_rx.xdr_gpiox = GPIOC;
        gpio_rx.xdr_gpio_portId = XDR_GPIO_PORT_C;
        gpio_rx.xdr_gpio_pin = GPIO_PIN_NO_7;
        XDR_GPIO_Init(&gpio_rx);
        gpio_rx.xdr_gpiox->AFR[0] &= ~(0xFUL << XDR_UART_GPIO_AF_PC7);
        gpio_rx.xdr_gpiox->AFR[0] |= (8UL << XDR_UART_GPIO_AF_PC7);
        break;
    default:
        break;
    }
}

// Clock Enable API for UART Instances
static void XDR_UART_Clock_Enable(xdr_uart *xdr_uart)
{

    switch (xdr_uart->xdr_uart_instance)
    {
    case XDR_UART1:
        UART1_CLOCK_SOURCE();
        UART1_CLOCK_ENABLE();
        xdr_uart->uart = USART1;
        break;
    case XDR_UART2:
        UART2_CLOCK_SOURCE();
        UART2_CLOCK_ENABLE();
        xdr_uart->uart = USART2;
        break;
    case XDR_UART3:
        UART3_CLOCK_SOURCE();
        UART3_CLOCK_ENABLE();
        xdr_uart->uart = USART3;
        break;
    case XDR_UART6:
        UART6_CLOCK_SOURCE();
        UART6_CLOCK_ENABLE();
        xdr_uart->uart = USART6;
        break;
    default:
        break;
    }
}

static uint32_t XDR_UART_BRR_Calculation(const xdr_uart *xdr_uart)
{

    uint32_t baud = xdr_uart->xdr_uart_baudrate;

    uint32_t mantissa = 0U;
    uint32_t fraction = 0U;
    uint32_t uartdiv = 0U;
    uint32_t brr = 0U;

    uartdiv = (XDR_HSI_CLK + (baud / 2U)) / baud;
    mantissa = uartdiv / 16U;
    fraction = uartdiv % 16U;
    brr = (mantissa << XDR_UART_BRR_Pos) | (fraction & XDR_UART_BRR_Over16Mask);

    return brr;
}

void XDR_UART_Send(xdr_uart *xdr_uart, uint8_t data)
{

    while ((xdr_uart->uart->ISR & (1UL << XDR_UART_ISR_TXE)) == 0UL)
    {
    }

    xdr_uart->uart->TDR = data;

    while ((xdr_uart->uart->ISR & (1UL << XDR_UART_ISR_TC)) == 0UL)
    {
    }
}

uint8_t XDR_UART_Receive(xdr_uart *xdr_uart)
{

    uint8_t data = 0;

    while ((xdr_uart->uart->ISR & (1UL << XDR_UART_ISR_RXNE)) == 0UL)
    {
    }

    data = (uint8_t)(xdr_uart->uart->RDR & 0xFFUL);

    return data;
}

static void XDR_UART3_EnableDMA(xdr_uart *xdr_uart)
{
    // Enable DMA for UART3 CR3
    xdr_uart->uart->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
    /*Enable clock access to DMA1*/
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    /*Enable DMA Stream3 Interrupt in NVIC*/
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void XDR_UART3_DMA_Rx_Config(void)
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
    DMA1_Stream1->M0AR = (uint32_t)(&UART3_Data_Buffer);
    /*Set number of transfer*/
    DMA1_Stream1->NDTR = (uint16_t)UART_BUFF_SIZE;
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

void XDR_UART3_DMA_Tx_Config(uint32_t data, uint32_t length)
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
        XDR_UART3_DMA_Rx_Callback();
        /*Clear the flag*/
        DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
    }
}

void DMA1_Stream3_IRQHandler(void)
{
    if ((DMA1->LISR & DMA_LISR_TCIF3) != 0U)
    {
        /* Do Something */
        XDR_UART3_DMA_Tx_Callback();
        /*Clear the flag*/
        DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
    }
}

void XDR_UART3_EnableRxInterrupt(xdr_uart *xdr_uart)
{
    /* Use global xdr_uart */
    uart_irq = xdr_uart;
    // Enable RXNE interrupt
    xdr_uart->uart->CR1 |= USART_CR1_RXNEIE;
    /*Enable USART3_IRQn in NVIC*/
    NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{

    // Check RXNE flag AND RXNE interrupt enable
    if (((uart_irq->uart->ISR & USART_ISR_RXNE) != 0U) &&
        ((uart_irq->uart->CR1 & USART_CR1_RXNEIE) != 0U))
    {
        // Reading RDR clears RXNE
        uint8_t ch = (uint8_t)uart_irq->uart->RDR;

        // Pass data to user hook (best: give parameter)
        XDR_UART3_RxCallback(ch);
    }
}
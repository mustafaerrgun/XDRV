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
static inline USART_TypeDef *UART_Get_Instance(xdr_uart_instance instance);
static void UART_Clock_Enable(const xdr_uart *xdr_uart);
static void UART_GPIO_Init(const xdr_uart *xdr_uart);
static uint32_t UART_BRR_Calc(const xdr_uart *xdr_uart);
static void UART_SendByte(const xdr_uart *xdr_uart, uint8_t data);

/* UART3 Interrupt and DMA Config Functions*/
static void XDR_UART3_EnableRxInterrupt(xdr_uart *xdr_uart);
static void XDR_UART3_EnableDMA(const xdr_uart *xdr_uart);

// Variables
static xdr_uart *uart_irq;
char UART3_Data_Buffer[UART_BUFF_SIZE];

void UART_Init(const xdr_uart *xdr_uart)
{
    USART_TypeDef *uart = UART_Get_Instance(xdr_uart->xdr_uart_instance);

    // Enable Clock for UART
    UART_Clock_Enable(xdr_uart);

    // Configure GPIO pins for UART
    UART_GPIO_Init(xdr_uart);

    // Clear CR1 register
    uart->CR1 = 0UL;

    // Use TX-RX Mode
    uart->CR1 |= USART_CR1_RE | USART_CR1_TE;

    // Calculate BRR register value for specified baud rate
    uart->BRR = UART_BRR_Calc(xdr_uart);

    // Enable UART Module
    uart->CR1 |= USART_CR1_UE;

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

static inline USART_TypeDef *UART_Get_Instance(xdr_uart_instance instance)
{
    static USART_TypeDef * const uart_instance_map[] = {
        [XDR_UART1] = USART1,
        [XDR_UART2] = USART2,
        [XDR_UART3] = USART3,
        [XDR_UART4] = UART4,
        [XDR_UART5] = UART5,
        [XDR_UART6] = USART6,
        [XDR_UART7] = UART7,
    };
    return uart_instance_map[instance];
}

// GPIO Configuration for UART instances
static void UART_GPIO_Init(const xdr_uart *xdr_uart)
{
    static const uart_pin_cfg uart_pin_map[(uint8_t)XDR_UART7 + 1U] = {
        [XDR_UART1] = { GPIOA, XDR_GPIO_PORT_A, GPIO_PIN_NO_9,  GPIOA, XDR_GPIO_PORT_A, GPIO_PIN_NO_10, 7U },
        [XDR_UART2] = { GPIOD, XDR_GPIO_PORT_D, GPIO_PIN_NO_5,  GPIOD, XDR_GPIO_PORT_D, GPIO_PIN_NO_6,  7U },
        [XDR_UART3] = { GPIOD, XDR_GPIO_PORT_D, GPIO_PIN_NO_8,  GPIOD, XDR_GPIO_PORT_D, GPIO_PIN_NO_9,  7U },
        [XDR_UART4] = { GPIOD, XDR_GPIO_PORT_D, GPIO_PIN_NO_0,  GPIOD, XDR_GPIO_PORT_D, GPIO_PIN_NO_1,  8U },
        [XDR_UART5] = { GPIOB, XDR_GPIO_PORT_B, GPIO_PIN_NO_8,  GPIOB, XDR_GPIO_PORT_B, GPIO_PIN_NO_9,  7U },
        [XDR_UART6] = { GPIOC, XDR_GPIO_PORT_C, GPIO_PIN_NO_6,  GPIOC, XDR_GPIO_PORT_C, GPIO_PIN_NO_7,  8U },
        [XDR_UART7] = { GPIOE, XDR_GPIO_PORT_E, GPIO_PIN_NO_7,  GPIOE, XDR_GPIO_PORT_E, GPIO_PIN_NO_8,  8U },
    };
    const uart_pin_cfg *cfg = &uart_pin_map[xdr_uart->xdr_uart_instance];

    xdr_gpio gpio = {
        .xdr_gpio_pinMode  = GPIO_MODE_ALTFN,
        .xdr_gpio_pinPuPd  = GPIO_NO_PUPD,
        .xdr_gpio_pinOType = GPIO_OP_TYPE_PP,
    };

    /* TX */
    gpio.xdr_gpiox       = cfg->tx_port;
    gpio.xdr_gpio_portId = cfg->tx_port_id;
    gpio.xdr_gpio_pin    = cfg->tx_pin;
    XDR_GPIO_Init(&gpio);
    cfg->tx_port->AFR[UART_AFR_INDEX(cfg->tx_pin)] &= ~(0xFUL << UART_AFR_SHIFT(cfg->tx_pin));
    cfg->tx_port->AFR[UART_AFR_INDEX(cfg->tx_pin)] |=  ((uint32_t)cfg->af << UART_AFR_SHIFT(cfg->tx_pin));

    /* RX */
    gpio.xdr_gpiox       = cfg->rx_port;
    gpio.xdr_gpio_portId = cfg->rx_port_id;
    gpio.xdr_gpio_pin    = cfg->rx_pin;
    XDR_GPIO_Init(&gpio);
    cfg->rx_port->AFR[UART_AFR_INDEX(cfg->rx_pin)] &= ~(0xFUL << UART_AFR_SHIFT(cfg->rx_pin));
    cfg->rx_port->AFR[UART_AFR_INDEX(cfg->rx_pin)] |=  ((uint32_t)cfg->af << UART_AFR_SHIFT(cfg->rx_pin));
}

// Clock Enable for UART Instances
static void UART_Clock_Enable(const xdr_uart *xdr_uart)
{
    switch (xdr_uart->xdr_uart_instance)
    {
    case XDR_UART1:  UART1_CLOCK_INIT(); break;
    case XDR_UART2:  UART2_CLOCK_INIT(); break;
    case XDR_UART3:  UART3_CLOCK_INIT(); break;
    case XDR_UART4:  UART4_CLOCK_INIT(); break;
    case XDR_UART5:  UART5_CLOCK_INIT(); break;
    case XDR_UART6:  UART6_CLOCK_INIT(); break;
    case XDR_UART7:  UART7_CLOCK_INIT(); break;
    default:         break;
    }
}

static uint32_t UART_BRR_Calc(const xdr_uart *xdr_uart)
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

void UART_SendChar(const xdr_uart *xdr_uart, char c)
{
    UART_SendByte(xdr_uart, (uint8_t)c);
}

void UART_SendString(const xdr_uart *xdr_uart, const char *str)
{
    const char *p = str;
    while (*p != '\0') {
        UART_SendByte(xdr_uart, (uint8_t)*p);
        p++;
    }
}

static void UART_SendByte(const xdr_uart *xdr_uart, uint8_t data)
{
    USART_TypeDef *uart = UART_Get_Instance(xdr_uart->xdr_uart_instance);

    while ((uart->ISR & USART_ISR_TXE) == 0UL)
    {
    }

    uart->TDR = data;

    while ((uart->ISR & USART_ISR_TC) == 0UL)
    {
    }
}

uint8_t UART_Receive(const xdr_uart *xdr_uart)
{
    const USART_TypeDef *uart = UART_Get_Instance(xdr_uart->xdr_uart_instance);
    uint8_t data = 0;

    while ((uart->ISR & USART_ISR_RXNE) == 0UL)
    {
    }

    data = (uint8_t)(uart->RDR & 0xFFUL);

    return data;
}

static void XDR_UART3_EnableDMA(const xdr_uart *xdr_uart)
{
    USART_TypeDef *uart = UART_Get_Instance(xdr_uart->xdr_uart_instance);

    // Enable DMA for UART3 CR3
    uart->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
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
    USART_TypeDef *uart = UART_Get_Instance(xdr_uart->xdr_uart_instance);

    /* Use global xdr_uart */
    uart_irq = xdr_uart;
    // Enable RXNE interrupt
    uart->CR1 |= USART_CR1_RXNEIE;
    /*Enable USART3_IRQn in NVIC*/
    NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{
    USART_TypeDef *uart = UART_Get_Instance(uart_irq->xdr_uart_instance);

    // Check RXNE flag AND RXNE interrupt enable
    if (((uart->ISR & USART_ISR_RXNE) != 0U) &&
        ((uart->CR1 & USART_CR1_RXNEIE) != 0U))
    {
        // Reading RDR clears RXNE
        uint8_t ch = (uint8_t)uart->RDR;

        // Pass data to user hook (best: give parameter)
        XDR_UART3_RxCallback(ch);
    }
}
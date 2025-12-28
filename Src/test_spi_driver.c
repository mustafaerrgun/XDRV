/**
 ******************************************************************************
 * @file           : test_spi_driver.c
 * @brief          : Test file for XDR SPI Driver
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_spi.h"


int main(void)
{

    XDR_SPI_Init();

    uint8_t r1 = 0x55;
    while (1)
    {
        XDR_SPI_Transmit(r1);
        for (volatile uint32_t i = 0; i < 10000; i++) {}

    }
}


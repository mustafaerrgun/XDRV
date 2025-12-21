/**
 ******************************************************************************
 * @file           : test_i2c_driver.c
 * @brief          : Test file for XDR I2C Driver
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f767xx_xdr_i2c.h"

#define ARDUINO_ADDR_7BIT  (0x08u)

int main(void)
{
    xdr_i2c test_i2c;
    test_i2c.xdr_i2c_instance = XDR_I2C1;

    XDR_I2C_Init(&test_i2c);

    while (1)
    {
        XDR_I2C_Write(&test_i2c, 0x08, 30);
        for (volatile uint32_t i = 0; i < 100000; i++) {}

        uint8_t rx = XDR_I2C_Read(&test_i2c, 0x08);
        for (volatile uint32_t i = 0; i < 100000; i++) {}

        XDR_I2C_Write(&test_i2c, 0x08, rx);
        for (volatile uint32_t i = 0; i < 300000; i++) {}
    }
}


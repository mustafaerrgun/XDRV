/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Entry point — delegates to the selected test file
 *
 * To switch tests, change the #include below to the desired test file and
 * recompile. Only one test file should be compiled at a time.
 *
 * Available tests:
 *   Tests/test_button_exti.c
 *   Tests/test_gpio_driver.c
 *   Tests/test_i2c_driver.c
 *   Tests/test_rcc_driver.c
 *   Tests/test_spi_driver.c
 *   Tests/test_timer_driver.c
 *   Tests/test_usart_dma.c
 *   Tests/test_usart_driver.c
 *   Tests/test_usart_driver_receive.c
 *   Tests/test_usart_driver_rx_interrupt.c
 *   Tests/test_usart_driver_send.c
 ******************************************************************************
 */

#include "test_runner.h"

int main(void)
{
    run_test();
    return 0;
}

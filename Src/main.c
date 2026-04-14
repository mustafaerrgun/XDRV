/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Entry point — delegates to the selected test file
 *
 * To switch tests, change the #include below to the desired test file and
 * recompile. Only one test file should be compiled at a time.
 *
 * Available tests:
 *   Tests/test_gpio1.c
 *   Tests/test_gpio2.c
 *   Tests/test_uart1.c
 *   Tests/test_uart2.c
 *   Tests/test_uart3.c
 *   Tests/test_uart4.c
 *   Tests/test_uart5.c
 *   Tests/test_rcc.c
 *   Tests/test_i2c.c
 *   Tests/test_spi.c
 *   Tests/test_timer.c
 ******************************************************************************
 */

#include "test_runner.h"

int main(void)
{
    run_test();
    return 0;
}

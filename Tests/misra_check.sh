#!/usr/bin/env bash

set -e

# Get the directory of the script
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Driver Source files
SRC_FILES=$(find "$SCRIPT_DIR/../Drivers/STM32F7xx_XDR_Driver/Src" -name '*.c')

# Driver Include path
INCLUDE_PATHS="-I$SCRIPT_DIR/../Drivers/STM32F7xx_XDR_Driver/Inc -I$SCRIPT_DIR/../Drivers/CMSIS/Include -I$SCRIPT_DIR/../Drivers/CMSIS/Device/ST/STM32F7xx/Include"

# Get absolute paths for suppression
CMSIS_INCLUDE_ABS=$(realpath "$SCRIPT_DIR/../Drivers/CMSIS/Include")
CMSIS_DEVICE_ABS=$(realpath "$SCRIPT_DIR/../Drivers/CMSIS/Device/ST/STM32F7xx/Include")

# Run cppcheck with MISRA
cppcheck \
  --std=c11 \
  --language=c \
  --enable=warning,style,performance,portability \
  --addon=misra \
  --inline-suppr \
  "--suppress=*:$CMSIS_INCLUDE_ABS/*" \
  "--suppress=*:$CMSIS_DEVICE_ABS/*" \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_gpio.c  --suppress=misra-c2012-2.5:*stm32f767xx_xdr_gpio.h \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_rcc.c   --suppress=misra-c2012-2.3:*stm32f767xx_xdr_rcc.h   --suppress=misra-c2012-2.4:*stm32f767xx_xdr_rcc.h   --suppress=misra-c2012-2.5:*stm32f767xx_xdr_rcc.h \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_usart.c --suppress=misra-c2012-2.3:*stm32f767xx_xdr_usart.h --suppress=misra-c2012-2.4:*stm32f767xx_xdr_usart.h --suppress=misra-c2012-2.5:*stm32f767xx_xdr_usart.h \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_i2c.c   --suppress=misra-c2012-2.3:*stm32f767xx_xdr_i2c.h   --suppress=misra-c2012-2.4:*stm32f767xx_xdr_i2c.h   --suppress=misra-c2012-2.5:*stm32f767xx_xdr_i2c.h \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_spi.c   --suppress=misra-c2012-2.3:*stm32f767xx_xdr_spi.h   --suppress=misra-c2012-2.4:*stm32f767xx_xdr_spi.h   --suppress=misra-c2012-2.5:*stm32f767xx_xdr_spi.h \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_tim.c \
  --error-exitcode=1 \
  $INCLUDE_PATHS \
  $SRC_FILES 2> "$SCRIPT_DIR/misra_report.txt"

echo "MISRA/static analysis finished. See misra_report.txt"
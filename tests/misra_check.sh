#!/usr/bin/env bash

set -e

# Get the directory of the script
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Driver Source files
SRC_FILES=$(find "$SCRIPT_DIR/../Drivers/Src" -name '*.c')

# Driver Include path
INCLUDE_PATHS="-I$SCRIPT_DIR/../Drivers/Inc"


# Run cppcheck with MISRA
cppcheck \
  --std=c11 \
  --language=c \
  --enable=warning,style,performance,portability \
  --addon=misra \
  --inline-suppr \
  --suppress=misra-c2012-11.4:*stm32f767xx.h --suppress=misra-c2012-2.3:*stm32f767xx.h --suppress=misra-c2012-2.4:*stm32f767xx.h --suppress=misra-c2012-2.5:*stm32f767xx.h \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_gpio.c --suppress=misra-c2012-2.5:*stm32f767xx_xdr_gpio.h \
  --suppress=misra-c2012-11.4:*stm32f767xx_xdr_rcc.c  --suppress=misra-c2012-2.3:*stm32f767xx_xdr_rcc.h --suppress=misra-c2012-2.4:*stm32f767xx_xdr_rcc.h --suppress=misra-c2012-2.5:*stm32f767xx_xdr_rcc.h \
  --error-exitcode=1 \
  $INCLUDE_PATHS \
  $SRC_FILES 2> "$SCRIPT_DIR/misra_report.txt"

echo "MISRA/static analysis finished. See misra_report.txt"
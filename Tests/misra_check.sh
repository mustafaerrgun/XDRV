#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(dirname "$(realpath "$0")")

mapfile -t SRC_FILES < <(find "$SCRIPT_DIR/../Drivers/STM32F7xx_XDR_Driver/Src" -name '*.c')

INCLUDE_PATHS=(
  "-I$SCRIPT_DIR/../Drivers/STM32F7xx_XDR_Driver/Inc"
  "-I$SCRIPT_DIR/../Drivers/CMSIS/Include"
  "-I$SCRIPT_DIR/../Drivers/CMSIS/Device/ST/STM32F7xx/Include"
)

CMSIS_INCLUDE_ABS=$(realpath "$SCRIPT_DIR/../Drivers/CMSIS/Include")
CMSIS_DEVICE_ABS=$(realpath "$SCRIPT_DIR/../Drivers/CMSIS/Device/ST/STM32F7xx/Include")

cppcheck \
  -j$(nproc) \
  --std=c11 \
  --language=c \
  --enable=warning,style,performance,portability \
  --addon=misra \
  --inline-suppr \
  "--suppress=*:$CMSIS_INCLUDE_ABS/*" \
  "--suppress=*:$CMSIS_DEVICE_ABS/*" \
  --suppress=misra-c2012-11.4:*xdr_*.c \
  --suppress=misra-c2012-2.3:*xdr_*.h \
  --suppress=misra-c2012-2.4:*xdr_*.h \
  --suppress=misra-c2012-2.5:*xdr_*.h \
  --error-exitcode=1 \
  "${INCLUDE_PATHS[@]}" \
  "${SRC_FILES[@]}" 2> "$SCRIPT_DIR/misra_report.txt"

echo "MISRA/static analysis finished. See misra_report.txt"
# Compiler and tools
CC = arm-none-eabi-gcc
CFLAGS = -c -mcpu=cortex-m7 -mthumb -std=gnu11 -Wall -Wextra -I$(PATH_CMSIS_INC) -I$(PATH_CMSIS_DEVICE_INC)

# Directories
PATH_SRC = Src
PATH_DRV_SRC = Drivers/STM32F7xx_XDR_Driver/Src
PATH_DRV_INC = Drivers/STM32F7xx_XDR_Driver/Inc
PATH_CMSIS_INC = Drivers/CMSIS/Include
PATH_CMSIS_DEVICE_INC = Drivers/CMSIS/Device/ST/STM32F7xx/Include
BUILD_DIR = build

# Object files
DRIVER_OBJS = $(BUILD_DIR)/stm32f767xx_xdr_gpio.o \
              $(BUILD_DIR)/stm32f767xx_xdr_rcc.o \
              $(BUILD_DIR)/stm32f767xx_xdr_i2c.o \
              $(BUILD_DIR)/stm32f767xx_xdr_spi.o \
              $(BUILD_DIR)/stm32f767xx_xdr_usart.o

# Default target - builds all object files
all: $(DRIVER_OBJS)

# Create build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Driver object files
$(BUILD_DIR)/stm32f767xx_xdr_gpio.o: $(PATH_DRV_SRC)/stm32f767xx_xdr_gpio.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(PATH_DRV_INC) $< -o $@
	
$(BUILD_DIR)/stm32f767xx_xdr_rcc.o: $(PATH_DRV_SRC)/stm32f767xx_xdr_rcc.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(PATH_DRV_INC) $< -o $@

$(BUILD_DIR)/stm32f767xx_xdr_i2c.o: $(PATH_DRV_SRC)/stm32f767xx_xdr_i2c.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(PATH_DRV_INC) $< -o $@

$(BUILD_DIR)/stm32f767xx_xdr_spi.o: $(PATH_DRV_SRC)/stm32f767xx_xdr_spi.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(PATH_DRV_INC) $< -o $@

$(BUILD_DIR)/stm32f767xx_xdr_usart.o: $(PATH_DRV_SRC)/stm32f767xx_xdr_usart.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(PATH_DRV_INC) $< -o $@

# Clean target to remove build directory
clean:
	rm -rf $(BUILD_DIR)

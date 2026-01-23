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
DRIVER_SRCS = $(wildcard $(PATH_DRV_SRC)/*.c)
DRIVER_OBJS = $(DRIVER_SRCS:$(PATH_DRV_SRC)/%.c=$(BUILD_DIR)/%.o)

# Default target - builds all object files
all: $(DRIVER_OBJS)

# Create build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Driver object files
$(BUILD_DIR)/%.o: $(PATH_DRV_SRC)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(PATH_DRV_INC) $< -o $@
	
# Formatting
format:
	@echo "Formatting source files..."
	clang-format -i $(PATH_DRV_SRC)/*.c

format-check:
	@echo "Checking formatting of source files..."
	clang-format --Werror -i $(PATH_DRV_SRC)/*.c

# Clean target to remove build directory
clean:
	rm -rf $(BUILD_DIR)

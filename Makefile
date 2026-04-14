# Compiler and tools
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Target name
TARGET = XDR

# Directories
PATH_SRC              = Src
PATH_TEST             = Tests
PATH_DRV_SRC          = Drivers/STM32F7xx_XDR_Driver/Src
PATH_DRV_INC          = Drivers/STM32F7xx_XDR_Driver/Inc
PATH_CMSIS_INC        = Drivers/CMSIS/Include
PATH_CMSIS_DEVICE_INC = Drivers/CMSIS/Device/ST/STM32F7xx/Include
PATH_STARTUP          = Startup
BUILD_DIR             = build

# MCU flags (shared between compiler, assembler and linker)
MCU_FLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

# Test selection: override on the command line with TEST=<name>
# e.g.  make TEST=test_gpio1   → compiles Tests/test_gpio1.c
TEST    ?= gpio1
APP_SRC ?= $(PATH_TEST)/test_$(TEST).c

# Linker script
LDSCRIPT = STM32F767ZITX_FLASH.ld

# Compiler flags
CFLAGS = -std=gnu11 -g3 -O0 \
         -DSTM32F767xx \
         -Wall -Wextra \
         -ffunction-sections -fdata-sections \
         -fstack-usage \
         --specs=nano.specs \
         $(MCU_FLAGS) \
         -I$(PATH_DRV_INC) \
         -I$(PATH_CMSIS_INC) \
         -I$(PATH_CMSIS_DEVICE_INC) \
         -I$(PATH_TEST)

# Assembler flags (for startup file)
ASFLAGS = -g3 \
          -x assembler-with-cpp \
          --specs=nano.specs \
          $(MCU_FLAGS)

# Linker flags
LDFLAGS = -T$(LDSCRIPT) \
          --specs=nosys.specs \
          --specs=nano.specs \
          $(MCU_FLAGS) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
          -Wl,--gc-sections \
          -static \
          -Wl,--start-group -lc -lm -Wl,--end-group

# --- Sources ---

# Startup assembly
STARTUP_OBJ = $(BUILD_DIR)/startup_stm32f767zitx.o

# System sources (auto-discovered)
SYS_SRCS = $(shell find $(PATH_SRC) -name '*.c')
SYS_OBJS  = $(patsubst $(PATH_SRC)/%.c,$(BUILD_DIR)/%.o,$(SYS_SRCS))

# Driver sources (auto-discovered)
DRV_SRCS = $(shell find $(PATH_DRV_SRC) -name '*.c')
DRV_OBJS = $(patsubst $(PATH_DRV_SRC)/%.c,$(BUILD_DIR)/drv_%.o,$(DRV_SRCS))

# Application object (derived from APP_SRC)
APP_OBJ  = $(BUILD_DIR)/app_$(notdir $(APP_SRC:.c=.o))

# All objects
ALL_OBJS = $(STARTUP_OBJ) $(SYS_OBJS) $(DRV_OBJS) $(APP_OBJ)

# --- Targets ---

.PHONY: all clean format format-check

all: $(BUILD_DIR)/$(TARGET).bin
	@echo ""
	@$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# Link ELF
$(BUILD_DIR)/$(TARGET).elf: $(ALL_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^
	@echo "Built: $@"

# Generate BIN from ELF
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@
	@echo "Built: $@"

# Build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Startup assembly
$(STARTUP_OBJ): $(PATH_STARTUP)/startup_stm32f767zitx.s | $(BUILD_DIR)
	$(CC) $(ASFLAGS) -MMD -MP -MF $(@:.o=.d) -c -o $@ $<

# System C files
$(BUILD_DIR)/%.o: $(PATH_SRC)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -MMD -MP -MF $(@:.o=.d) -c -o $@ $<

# Driver C files
$(BUILD_DIR)/drv_%.o: $(PATH_DRV_SRC)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -MMD -MP -MF $(@:.o=.d) -c -o $@ $<

# Application source
$(APP_OBJ): $(APP_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -MMD -MP -MF $(@:.o=.d) -c -o $@ $<

# Include dependency files
-include $(wildcard $(BUILD_DIR)/*.d)

clean:
	rm -rf $(BUILD_DIR)

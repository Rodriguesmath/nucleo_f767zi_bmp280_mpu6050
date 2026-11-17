################################################################################
# Makefile for NUCLEO-F767ZI (STM32F767ZIT6) - BMP280 (I2C) + MPU-6050 (I2C)
################################################################################

TARGET = nucleo_f767zi_bmp280_mpu6050
BUILD_DIR = build

# Toolchain definitions
ST_TOOLCHAIN_PATH := /opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin
ST_CUBEPROG_PATH  := /opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI

PREFIX := $(shell which arm-none-eabi-gcc >/dev/null 2>&1 && echo "" || echo "$(ST_TOOLCHAIN_PATH)/")

CC      = $(PREFIX)arm-none-eabi-gcc
AS      = $(PREFIX)arm-none-eabi-gcc -x assembler-with-cpp
CP      = $(PREFIX)arm-none-eabi-objcopy
SZ      = $(PREFIX)arm-none-eabi-size
HEX     = $(CP) -O ihex
BIN     = $(CP) -O binary -S

CUBEPROG := $(shell which STM32_Programmer_CLI >/dev/null 2>&1 && echo "STM32_Programmer_CLI" || echo "$(ST_CUBEPROG_PATH)")

# Architecture & Optimization (Cortex-M7 with Double-Precision FPU)
CPU       = -mcpu=cortex-m7
FPU       = -mfpu=fpv5-d16
FLOAT-ABI = -mfloat-abi=hard
MCU       = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Compiler optimization
OPT       = -Og -g3 -gdwarf-2

# C Defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F767xx \
-DHSE_VALUE=8000000U

# Include paths
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
-IDrivers/CMSIS/Include

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Wextra -fdata-sections -ffunction-sections
ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections

# C Sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/bmp280.c \
Core/Src/Mpu6050.c \
Core/Src/ComplementaryFilter.c \
Core/Src/stm32f7xx_it.c \
Core/Src/stm32f7xx_hal_msp.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/system_stm32f7xx.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c

# ASM Sources
ASM_SOURCES =  \
Core/Startup/startup_stm32f767zitx.s

# Linker script
LDSCRIPT = STM32F767ZITX_FLASH.ld

# Libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# Object list
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile $(LDSCRIPT)
	@echo "LD $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "HEX $@"
	@$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "BIN $@"
	@$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

test: | $(BUILD_DIR)
	gcc -O2 -Wall -Wextra -I./Core/Inc -DUNIT_TEST tests/test_bmp280.c Core/Src/bmp280.c -lm -o $(BUILD_DIR)/test_bmp280
	gcc -O2 -Wall -Wextra -I./Core/Inc -DUNIT_TEST tests/test_speculative_edge_cases.c Core/Src/bmp280.c -lm -o $(BUILD_DIR)/test_speculative_edge_cases
	gcc -O2 -Wall -Wextra -I./Core/Inc -DUNIT_TEST tests/test_mpu6050_filter.c Core/Src/Mpu6050.c Core/Src/ComplementaryFilter.c Core/Src/bmp280.c -lm -o $(BUILD_DIR)/test_mpu6050_filter
	$(BUILD_DIR)/test_bmp280
	$(BUILD_DIR)/test_speculative_edge_cases
	$(BUILD_DIR)/test_mpu6050_filter

flash: $(BUILD_DIR)/$(TARGET).bin
	@echo "Flashing with STM32CubeProgrammer..."
	$(CUBEPROG) -c port=SWD -w $(BUILD_DIR)/$(TARGET).bin 0x08000000 -v -rst

viewer:
	./run_viewer.sh

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean test flash viewer

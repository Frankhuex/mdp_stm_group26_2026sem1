# Standalone STM32F407 build; run from this directory.
.DEFAULT_GOAL := build
TOOLCHAIN_DIR ?= .tools/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi/bin
PREFIX := $(if $(wildcard $(TOOLCHAIN_DIR)/arm-none-eabi-gcc),$(TOOLCHAIN_DIR)/,)arm-none-eabi-
CC := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size
OPENOCD ?= openocd
BUILD_DIR := build
TARGET := $(BUILD_DIR)/mdp21_8
include sources.mk
OBJECTS := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(SOURCES)))
INCLUDES := Core/Inc Drivers/STM32F4xx_HAL_Driver/Inc Drivers/STM32F4xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32F4xx/Include Drivers/CMSIS/Include Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS := $(addprefix -I,$(INCLUDES)) -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx
CFLAGS := $(CPU) -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs
LDFLAGS := $(CPU) -TSTM32F407VETX_FLASH.ld --specs=nosys.specs --specs=nano.specs -static -Wl,--gc-sections -Wl,-Map=$(TARGET).map
.PHONY: build flash verify probe backup debug clean
build: $(TARGET).elf $(TARGET).bin $(TARGET).hex
	$(SIZE) $(TARGET).elf
$(BUILD_DIR)/%.c.o: %.c Makefile
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c "$<" -o "$@"
$(BUILD_DIR)/%.s.o: %.s Makefile
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CPU) -g3 -x assembler-with-cpp -MMD -MP -c "$<" -o "$@"
$(TARGET).elf: $(OBJECTS) STM32F407VETX_FLASH.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -Wl,--start-group -lc -lm -Wl,--end-group -o "$@"
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary "$<" "$@"
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex "$<" "$@"
flash: build
	$(OPENOCD) -f openocd.cfg -c "program $(TARGET).elf verify reset exit"
verify: $(TARGET).elf
	$(OPENOCD) -f openocd.cfg -c "init; reset halt; verify_image $(TARGET).elf; reset run; shutdown"
probe:
	$(OPENOCD) -f openocd.cfg -c "init; targets; flash probe 0; shutdown"
backup:
	@mkdir -p backups
	$(OPENOCD) -f openocd.cfg -c "init; halt; dump_image backups/flash-$$(date +%Y%m%d-%H%M%S).bin 0x08000000 0x80000; resume; shutdown"
debug: $(TARGET).elf
	$(OPENOCD) -f openocd.cfg
clean:
	rm -rf $(BUILD_DIR)
-include $(OBJECTS:.o=.d)

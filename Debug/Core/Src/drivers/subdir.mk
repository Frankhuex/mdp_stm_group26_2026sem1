################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/drivers/oled.c \
../Core/Src/drivers/robot_hw.c 

OBJS += \
./Core/Src/drivers/oled.o \
./Core/Src/drivers/robot_hw.o 

C_DEPS += \
./Core/Src/drivers/oled.d \
./Core/Src/drivers/robot_hw.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/drivers/%.o Core/Src/drivers/%.su: ../Core/Src/drivers/%.c Core/Src/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-drivers

clean-Core-2f-Src-2f-drivers:
	-$(RM) ./Core/Src/drivers/oled.d ./Core/Src/drivers/oled.o ./Core/Src/drivers/oled.su ./Core/Src/drivers/robot_hw.d ./Core/Src/drivers/robot_hw.o ./Core/Src/drivers/robot_hw.su

.PHONY: clean-Core-2f-Src-2f-drivers


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/control/approach_control.c \
../Core/Src/control/lateral_control.c \
../Core/Src/control/bypass_control.c \
../Core/Src/control/motion_heading.c \
../Core/Src/control/straight_control.c \
../Core/Src/control/turn_control.c 

OBJS += \
./Core/Src/control/approach_control.o \
./Core/Src/control/lateral_control.o \
./Core/Src/control/bypass_control.o \
./Core/Src/control/motion_heading.o \
./Core/Src/control/straight_control.o \
./Core/Src/control/turn_control.o 

C_DEPS += \
./Core/Src/control/approach_control.d \
./Core/Src/control/lateral_control.d \
./Core/Src/control/bypass_control.d \
./Core/Src/control/motion_heading.d \
./Core/Src/control/straight_control.d \
./Core/Src/control/turn_control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/control/%.o Core/Src/control/%.su: ../Core/Src/control/%.c Core/Src/control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-control

clean-Core-2f-Src-2f-control:
	-$(RM) ./Core/Src/control/approach_control.d ./Core/Src/control/approach_control.o ./Core/Src/control/approach_control.su ./Core/Src/control/motion_heading.d ./Core/Src/control/motion_heading.o ./Core/Src/control/motion_heading.su ./Core/Src/control/straight_control.d ./Core/Src/control/straight_control.o ./Core/Src/control/straight_control.su ./Core/Src/control/turn_control.d ./Core/Src/control/turn_control.o ./Core/Src/control/turn_control.su

.PHONY: clean-Core-2f-Src-2f-control


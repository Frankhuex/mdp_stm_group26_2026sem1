################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/tests/component/test_accel.c \
../Core/Src/tests/component/test_encoder.c \
../Core/Src/tests/component/test_imu.c \
../Core/Src/tests/component/test_ir.c \
../Core/Src/tests/component/test_motor.c \
../Core/Src/tests/component/test_oled.c \
../Core/Src/tests/component/test_servo.c \
../Core/Src/tests/component/test_uart.c \
../Core/Src/tests/component/test_ultrasonic.c 

OBJS += \
./Core/Src/tests/component/test_accel.o \
./Core/Src/tests/component/test_encoder.o \
./Core/Src/tests/component/test_imu.o \
./Core/Src/tests/component/test_ir.o \
./Core/Src/tests/component/test_motor.o \
./Core/Src/tests/component/test_oled.o \
./Core/Src/tests/component/test_servo.o \
./Core/Src/tests/component/test_uart.o \
./Core/Src/tests/component/test_ultrasonic.o 

C_DEPS += \
./Core/Src/tests/component/test_accel.d \
./Core/Src/tests/component/test_encoder.d \
./Core/Src/tests/component/test_imu.d \
./Core/Src/tests/component/test_ir.d \
./Core/Src/tests/component/test_motor.d \
./Core/Src/tests/component/test_oled.d \
./Core/Src/tests/component/test_servo.d \
./Core/Src/tests/component/test_uart.d \
./Core/Src/tests/component/test_ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/tests/component/%.o Core/Src/tests/component/%.su: ../Core/Src/tests/component/%.c Core/Src/tests/component/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-tests-2f-component

clean-Core-2f-Src-2f-tests-2f-component:
	-$(RM) ./Core/Src/tests/component/test_accel.d ./Core/Src/tests/component/test_accel.o ./Core/Src/tests/component/test_accel.su ./Core/Src/tests/component/test_encoder.d ./Core/Src/tests/component/test_encoder.o ./Core/Src/tests/component/test_encoder.su ./Core/Src/tests/component/test_imu.d ./Core/Src/tests/component/test_imu.o ./Core/Src/tests/component/test_imu.su ./Core/Src/tests/component/test_ir.d ./Core/Src/tests/component/test_ir.o ./Core/Src/tests/component/test_ir.su ./Core/Src/tests/component/test_motor.d ./Core/Src/tests/component/test_motor.o ./Core/Src/tests/component/test_motor.su ./Core/Src/tests/component/test_oled.d ./Core/Src/tests/component/test_oled.o ./Core/Src/tests/component/test_oled.su ./Core/Src/tests/component/test_servo.d ./Core/Src/tests/component/test_servo.o ./Core/Src/tests/component/test_servo.su ./Core/Src/tests/component/test_uart.d ./Core/Src/tests/component/test_uart.o ./Core/Src/tests/component/test_uart.su ./Core/Src/tests/component/test_ultrasonic.d ./Core/Src/tests/component/test_ultrasonic.o ./Core/Src/tests/component/test_ultrasonic.su

.PHONY: clean-Core-2f-Src-2f-tests-2f-component


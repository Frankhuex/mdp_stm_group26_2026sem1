################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/tests/calibration/test_approach_control.c \
../Core/Src/tests/calibration/test_gyro_calibration.c \
../Core/Src/tests/calibration/test_ir_calibration.c \
../Core/Src/tests/calibration/test_min_turn_radius.c \
../Core/Src/tests/calibration/test_motion_sequence.c \
../Core/Src/tests/calibration/test_straight_control.c \
../Core/Src/tests/calibration/test_turn_angle.c 

OBJS += \
./Core/Src/tests/calibration/test_approach_control.o \
./Core/Src/tests/calibration/test_gyro_calibration.o \
./Core/Src/tests/calibration/test_ir_calibration.o \
./Core/Src/tests/calibration/test_min_turn_radius.o \
./Core/Src/tests/calibration/test_motion_sequence.o \
./Core/Src/tests/calibration/test_straight_control.o \
./Core/Src/tests/calibration/test_turn_angle.o 

C_DEPS += \
./Core/Src/tests/calibration/test_approach_control.d \
./Core/Src/tests/calibration/test_gyro_calibration.d \
./Core/Src/tests/calibration/test_ir_calibration.d \
./Core/Src/tests/calibration/test_min_turn_radius.d \
./Core/Src/tests/calibration/test_motion_sequence.d \
./Core/Src/tests/calibration/test_straight_control.d \
./Core/Src/tests/calibration/test_turn_angle.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/tests/calibration/%.o Core/Src/tests/calibration/%.su: ../Core/Src/tests/calibration/%.c Core/Src/tests/calibration/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-tests-2f-calibration

clean-Core-2f-Src-2f-tests-2f-calibration:
	-$(RM) ./Core/Src/tests/calibration/test_approach_control.d ./Core/Src/tests/calibration/test_approach_control.o ./Core/Src/tests/calibration/test_approach_control.su ./Core/Src/tests/calibration/test_gyro_calibration.d ./Core/Src/tests/calibration/test_gyro_calibration.o ./Core/Src/tests/calibration/test_gyro_calibration.su ./Core/Src/tests/calibration/test_ir_calibration.d ./Core/Src/tests/calibration/test_ir_calibration.o ./Core/Src/tests/calibration/test_ir_calibration.su ./Core/Src/tests/calibration/test_min_turn_radius.d ./Core/Src/tests/calibration/test_min_turn_radius.o ./Core/Src/tests/calibration/test_min_turn_radius.su ./Core/Src/tests/calibration/test_motion_sequence.d ./Core/Src/tests/calibration/test_motion_sequence.o ./Core/Src/tests/calibration/test_motion_sequence.su ./Core/Src/tests/calibration/test_straight_control.d ./Core/Src/tests/calibration/test_straight_control.o ./Core/Src/tests/calibration/test_straight_control.su ./Core/Src/tests/calibration/test_turn_angle.d ./Core/Src/tests/calibration/test_turn_angle.o ./Core/Src/tests/calibration/test_turn_angle.su

.PHONY: clean-Core-2f-Src-2f-tests-2f-calibration


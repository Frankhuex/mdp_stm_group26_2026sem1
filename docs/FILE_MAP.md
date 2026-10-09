# Active file map - direct naming version

## Generated / MCU infrastructure

- `Core/Src/main.c` - STM32 entry point and CubeMX peripheral initialization.
- `Core/Src/freertos.c` - FreeRTOS/CMSIS-RTOS infrastructure.
- `Core/Src/stm32f4xx_*.c`, `sys*.c`, `system_stm32f4xx.c` - generated/platform support.
- `Core/Inc/test_config.h` - select exactly one test or the assessment command interface.
- `Core/Src/tests/test_runner.c` - runs the selection from `test_config.h`.

## Reusable hardware / drivers

Located under `Core/Src/drivers/`:

- `robot_hw.c/.h` - motors, steering servo, encoders and gyro hardware abstraction.
- `oled.c/.h` - OLED driver.

## Reusable control algorithms

Located under `Core/Src/control/`:

- `straight_control.c/.h` - the single production straight-distance controller.
- `straight_tuning.h` - straight floor/behavior tuning values.
- `turn_control.c/.h` - the single production turn controller.
- `turn_tuning.h` - turn floor/behavior tuning values.

## Reusable sensors

Located under `Core/Src/sensors/`:

- `ir_sensor.c/.h` - LEFT Sharp IR on PC0/ADC1_IN10 and RIGHT Sharp IR on PC1/ADC1_IN11.
- `ultrasonic_sensor.c/.h` - HC-SR04 using TRIG PC7 and ECHO PC6.

## Component tests

Located under `Core/Src/tests/component/`:

- `test_motor.c` -> `TEST_MOTOR`
- `test_encoder.c` -> `TEST_ENCODER`
- `test_servo.c` -> `TEST_SERVO`
- `test_oled.c` -> `TEST_OLED`
- `test_uart.c` -> `TEST_UART`
- `test_imu.c` -> `TEST_IMU`
- `test_accel.c` -> `TEST_ACCEL`
- `test_ir.c` -> `TEST_IR`
- `test_ultrasonic.c` -> `TEST_ULTRASONIC`

These answer: **does this individual component work?**

## Calibration tests

Located under `Core/Src/tests/calibration/`:

- `test_gyro_calibration.c` -> `TEST_GYRO_CALIBRATION`
- `test_ir_calibration.c` -> `TEST_IR_CALIBRATION`
- `test_min_turn_radius.c` -> `TEST_MIN_TURN_RADIUS`
- `test_straight_control.c` -> `TEST_STRAIGHT_CONTROL`
- `test_turn_angle.c` -> `TEST_TURN_ANGLE`

These answer: **what measured/tuned value should the production code use?**

## Assessment-facing code

Located under `Core/Src/assessment/`:

- `command_interface.c` -> `ASSESSMENT_COMMAND_INTERFACE`

It accepts UART/PuTTY/RPi commands and calls the shared `control/straight_control.c`, `control/turn_control.c`, and `drivers/robot_hw.c` modules.

## Motion command documentation and host checks

- `docs/PROTOCOL_en.md`, `docs/PROTOCOL_zh.md`: UART commands and replies, including LEFTIR/RIGHTIR.
- `docs/IR_STRAIGHT.md`: side IR thresholds, FAR/ADC failure policy and validation.
- `docs/MOTION_SETTINGS.md`: global PWM scale, calibration/wait switch and braking.
- `docs/HARDCODE_COMMAND_EXECUTOR.md`: command arrays and sequence behavior.
- `tests/host/test_ir_straight.c`: simulated hardware test for the production straight controller; outside the embedded Core source tree.

## Archived sources

`Archive/` contains old/reference source files and donor history. It is intentionally outside `Core/` so those `.c` files do not compile alongside the active implementation.

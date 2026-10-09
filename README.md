# MDP21 STM32 Project - Direct Naming Structure

This version uses simple folder names and test names that match the actual files.
There is no `APP_MODE` naming anymore.

## 1. To select a test

Open:

```text
Core/Inc/test_config.h
```

Change only this line:

```c
#define ACTIVE_TEST TEST_IR
```

Examples:

```c
#define ACTIVE_TEST TEST_MOTOR
#define ACTIVE_TEST TEST_ENCODER
#define ACTIVE_TEST TEST_SERVO
#define ACTIVE_TEST TEST_OLED
#define ACTIVE_TEST TEST_UART
#define ACTIVE_TEST TEST_IMU
#define ACTIVE_TEST TEST_ACCEL
#define ACTIVE_TEST TEST_IR
#define ACTIVE_TEST TEST_ULTRASONIC

#define ACTIVE_TEST TEST_GYRO_CALIBRATION
#define ACTIVE_TEST TEST_IR_CALIBRATION
#define ACTIVE_TEST TEST_MIN_TURN_RADIUS

#define ACTIVE_TEST ASSESSMENT_COMMAND_INTERFACE
```

The names match the files directly. Example:

```text
TEST_MOTOR               -> tests/component/test_motor.c
TEST_IR                   -> tests/component/test_ir.c
TEST_IR_CALIBRATION       -> tests/calibration/test_ir_calibration.c
ASSESSMENT_COMMAND_INTERFACE -> assessment/command_interface.c
```

The current selection is `TEST_HARDCODE_COMMAND_EXECUTOR`; check `Core/Inc/test_config.h` before building.

## 2. Folder structure

```text
Core/
├── Inc/
│   ├── main.h
│   ├── test_config.h
│   ├── drivers/
│   │   ├── robot_hw.h
│   │   ├── oled.h
│   │   └── oledfont.h
│   ├── control/
│   │   ├── straight_control.h
│   │   └── turn_control.h
│   ├── sensors/
│   │   ├── ir_sensor.h
│   │   └── ultrasonic_sensor.h
│   ├── assessment/
│   │   └── command_interface.h
│   └── tests/
│       ├── test_runner.h
│       ├── component/
│       └── calibration/
│
└── Src/
    ├── main.c
    ├── freertos.c
    ├── drivers/
    │   ├── robot_hw.c
    │   └── oled.c
    ├── control/
    │   ├── straight_control.c
    │   └── turn_control.c
    ├── sensors/
    │   ├── ir_sensor.c
    │   └── ultrasonic_sensor.c
    ├── assessment/
    │   └── command_interface.c
    └── tests/
        ├── test_runner.c
        ├── component/
        │   ├── test_motor.c
        │   ├── test_encoder.c
        │   ├── test_servo.c
        │   ├── test_oled.c
        │   ├── test_uart.c
        │   ├── test_imu.c
        │   ├── test_accel.c
        │   ├── test_ir.c
        │   └── test_ultrasonic.c
        └── calibration/
            ├── test_gyro_calibration.c
            ├── test_ir_calibration.c
            └── test_min_turn_radius.c
```

## 3. What each folder means

- `drivers/` = direct reusable hardware functions such as motors, servo, encoders and OLED.
- `control/` = real robot movement algorithms. `straight_control.c` and `turn_control.c` are used by the final robot.
- `sensors/` = reusable IR and ultrasonic sensor reading code.
- `tests/component/` = check whether one individual component works.
- `tests/calibration/` = collect/tune measured values.
- `assessment/` = integrated robot interface used for PuTTY/RPi and assessment.
- `Archive/` = old/reference code only; it is outside `Core/` and is not part of the normal CubeIDE source tree.

## 4. IR sensors

Current wiring used by the project:

```text
LEFT Sharp IR
VCC -> 5V
GND -> GND
VO  -> PC0 / ADC1_IN10

RIGHT Sharp IR
VCC -> 5V
GND -> GND
VO  -> PC1 / ADC1_IN11
```

Use these in order:

```c
#define ACTIVE_TEST TEST_IR
```

First verify LEFT and RIGHT both respond.

Then:

```c
#define ACTIVE_TEST TEST_IR_CALIBRATION
```

Use the UART calibration commands documented in `docs/IR_CALIBRATION.md`.

## 5. Final assessment command interface

Select:

```c
#define ACTIVE_TEST ASSESSMENT_COMMAND_INTERFACE
```

Then PuTTY/RPi can send commands such as:

```text
STRAIGHT,GOCM,10
STRAIGHT,GOCM,80
STRAIGHT,GOCM,100
STRAIGHT,GOCM,120
TURN,RIGHT,R=43,A=90
STRAIGHT,STATUS
TURN,STATUS
LOOP,ONCE
STOP
```

`STRAIGHT,GOCM,10` uses the dedicated 10 cm short-distance profile. Longer distances use the integrated long-distance profile. Both are inside the single `control/straight_control.c` module.

`LOOP,ONCE` runs one offset loop through the same command executor used by the
hardcoded command-array test:

```text
TURN,RIGHT,R=35,A=90
STRAIGHT,GOCM,40
TURN,LEFT,R=20,A=180
STRAIGHT,UNTILUS,47
```

The only replies for a normal run are `OK,loop=ONCE,count=4` and
`DONE,LOOP,state=DONE,count=4`. On failure or `STOP`, the final line is
`DONE,LOOP,state=<reason>,index=<row>,count=4`. Detailed message formats are in
`docs/PROTOCOL_en.md` and `docs/PROTOCOL_zh.md`.


## Floor retuning

Use `TEST_STRAIGHT_CONTROL` and `TEST_TURN_ANGLE` in `Core/Inc/test_config.h`.
The test files call the same production controllers used by the assessment command interface.
Tuning values are grouped in `Core/Inc/control/straight_tuning.h` and `Core/Inc/control/turn_tuning.h`.
Current straight profile selection is 10-20 cm SHORT and 21-200 cm LONG.
See `docs/FLOOR_RETUNING.md`.

## Ultrasonic clearance command

`STRAIGHT,UNTILUS,20` drives forward until a fresh ultrasonic reading is strictly
below 20 cm. Threshold range: 5..200 cm. Supports `STRAIGHT,STATUS`,
`STRAIGHT,STOP`, and `STOP`, with asynchronous completion/fault reporting.
See [ULTRASONIC_STRAIGHT.md](docs/ULTRASONIC_STRAIGHT.md) for protocol, limits,
wiring and manual checks. Select ASSESSMENT_COMMAND_INTERFACE to use commands;
TEST_ULTRASONIC validates the shared PB15/PC8 TIM8_CH3 capture driver.

## Side IR straight commands

## IR GT / LT 更新

新格式：`STRAIGHT,LEFTIR/RIGHTIR,GT/LT,n`，n 为整数厘米。
例如 `STRAIGHT,LEFTIR,GT,50`（大于 50 cm 停车）、
`STRAIGHT,RIGHTIR,LT,20`（小于 20 cm 停车）。等于阈值继续。
左侧范围 10～50 cm，右侧 10～60 cm。旧三段格式保留为 GT。
所有 IR 回报增加 `op=GT`／`op=LT`。

LT 遇到 NEAR（小于标定下限 10 cm）也停车；FAR 继续；
ADC／转换故障则停车并报告 SENSOR_ERROR，中止硬编码序列。
起步前已满足条件则不驱动车轮。采样、纠偏及超时／行程限制不变。
下文“大于阈值、FAR／ADC 故障正常 DONE”规则仅适用于 GT。

New syntax: `STRAIGHT,LEFTIR/RIGHTIR,GT/LT,n`. GT stops strictly above n;
LT stops strictly below n. Equality continues. Legacy three-field commands
remain GT aliases. Replies include `op=GT|LT`.
LT also stops on NEAR, continues on FAR, and stops with SENSOR_ERROR on
ADC/conversion failure (aborting sequences). Bounds and motion limits are
unchanged. The greater-than and failure-as-DONE descriptions below apply
only to GT.



```text
STRAIGHT,LEFTIR,GT,50
STRAIGHT,RIGHTIR,GT,50
```

Drive forward until the selected IR distance is strictly greater than the target;
equality continues. FAR or ADC failure completes normally with DONE. An initial
qualifying sample completes without driving. LEFTIR accepts 10..50 cm; RIGHTIR
accepts 10..60 cm. Both work in the hardcoded executor, support STOP/status, and
retain 30-second/200 cm limits. See [IR_STRAIGHT.md](docs/IR_STRAIGHT.md) and the
[Chinese protocol](docs/PROTOCOL_zh.md) / [English protocol](docs/PROTOCOL_en.md).

## Overall speed and command waits

In `Core/Inc/drivers/robot_tuning.h`, `MOTOR_SPEED_SCALE_PERCENT` scales drive PWM
(currently 120; 100 means unchanged). Braking and servo pulses are unaffected.
`GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` currently equals 0: repeated calibration,
preparation stops/stillness checks, turn servo/post-stop waits and automatic
sequence gaps are skipped. First-use calibration, target braking and explicit
WAIT remain. Set 1 to restore calibration and waits. Rebuild and flash after
editing. See [MOTION_SETTINGS.md](docs/MOTION_SETTINGS.md) for full behavior.

## Hardcoded command-array test

Select `TEST_HARDCODE_COMMAND_EXECUTOR` in `Core/Inc/test_config.h` and edit
`commands[]` in `Core/Src/tests/test_hardcode_command_executor.c`.
Motion commands run sequentially, with completion/error handling and external
UART `STOP`. See [HARDCODE_COMMAND_EXECUTOR.md](docs/HARDCODE_COMMAND_EXECUTOR.md).

IR extension: use `STRAIGHT,LEFTIR,LT,20` or `STRAIGHT,RIGHTIR,LT,20` to stop below 20 cm. NEAR satisfies LT; FAR continues; ADC/conversion errors stop with SENSOR_ERROR. The greater-than/failure-as-DONE behavior above applies to GT only. Legacy three-field IR commands remain GT aliases. Replies include `op=GT|LT`.

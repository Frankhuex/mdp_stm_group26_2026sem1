# Side IR straight commands

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

## 中文速查

`STRAIGHT,LEFTIR,GT,50`／`STRAIGHT,RIGHTIR,GT,50`：向前行驶，选定侧 IR 距离严格
大于 50 cm 时停车，等于则继续。LEFTIR 阈值范围 10～50 cm，RIGHTIR 为 10～60 cm。
`FAR`、ADC 读取失败和转换错误均按无限远处理，正常回报 DONE，执行器继续下一条。
起步前已满足条件时不驱动车轮。此规则不改变陀螺仪故障处理。

支持 `STRAIGHT,STATUS`、`STRAIGHT,STOP` 和 `STOP`。运行上限为 30 秒、任一轮
200 cm；达到上限报告故障终态并中止序列。参数及全局开关见
[运动设置](MOTION_SETTINGS.md)，完整报文见 [中文协议](PROTOCOL_zh.md)。

## Command behavior

```text
STRAIGHT,LEFTIR,GT,50
STRAIGHT,RIGHTIR,GT,50
```

Drive forward with the existing yaw steering and wheel synchronization until
the selected side's IR distance is **strictly greater** than the threshold in
centimetres. Equality does not stop. LEFTIR accepts integer thresholds 10..50;
RIGHTIR accepts 10..60, matching the current side-specific calibration tables.

`FAR`, ADC read failure and conversion error are treated as infinite distance:
they trigger normal braking and `DONE`, not `SENSOR_ERROR`. This is specific to
these commands; it does not change the shared IR driver or gyro fault handling.
The initial sample is checked before drive power: if already above threshold,
the command completes without driving. FAR is necessary for LEFTIR,50 because
the driver's numeric distance saturates at 50 cm.

Samples are taken about every 40 ms. Two consecutive qualifying samples are
required. Tuning is in `Core/Inc/control/straight_tuning.h` (`IR_COMMAND_*`).
Every active IR comparison uses base PWM 1800, including startup and entry
from GOCM, subject to global motor scaling and normal wheel synchronization. Maximum travel is 200 cm on either
wheel and timeout is 30 seconds; these return DISTANCE_LIMIT/TIMEOUT, not DONE.

Replies:

```text
OK,mode=LEFTIR,op=GT,threshold_cm=50
DONE,STRAIGHT,mode=LEFTIR,op=GT,state=DONE,threshold_cm=50,ir_cm=FAR
```

`STRAIGHT,STATUS` reports mode, state, threshold and last IR distance (`FAR` also
represents ADC failure). `STOP` and `STRAIGHT,STOP` abort normally. Malformed
arguments, out-of-range thresholds and concurrent motion are rejected.

Both commands work unchanged as strings in the hardcoded executor. The executor
waits for completion and advances on normal DONE, including FAR/ADC failure.
Explicit STOP, timeout, distance limit and gyro faults interrupt the sequence.
Existing calibration/wait and motor speed scale settings also apply.

Floor checks: start below/equal/above threshold; cross it on the selected side;
change only the opposite side; test FAR/ADC failure before and during motion;
test STOP, timeout, travel limit and subsequent GOCM/UNTILUS mode transitions.

## Host verification

The production straight controller is linked against simulated hardware in
`tests/host/test_ir_straight.c`. From the repository root, with GCC on PATH:

```text
gcc -std=c11 -Wall -Wextra -Itests/host -ICore/Inc tests/host/test_ir_straight.c Core/Src/control/straight_control.c -o tests/host/test_ir_straight.exe
tests\host\test_ir_straight.exe
```

The host test cases cover strict threshold comparison, side selection, FAR and ADC
failure, initial no-drive completion, STOP, mode transitions, bounds, BUSY,
timeout and travel limit. This is controller simulation, not a UART parser or
physical robot test. STM32 firmware compilation/linking also passed during
implementation; hardware performance remains unverified.

GT/LT validation for this revision: STM32 firmware compilation and linking passed.
The expanded host suite compiled with -Wall -Wextra -Werror, but execution
could not be completed in this environment. UART integration and physical
robot behavior still require validation.

## Explicit direction / 显式方向

`STRAIGHT,FORWARD,LEFTIR,GT,30`、`STRAIGHT,BACKWARD,LEFTIR,LT,30`；RIGHTIR 同样支持两个方向和 GT/LT，方向参数固定紧跟 STRAIGHT。省略方向默认前进。回复／状态／完成事件增加 `dir=FORWARD|BACKWARD`。原有阈值范围、故障策略、缓存、STOP、超时和行程限制沿用，LATERAL 的内部 IR 接近阶段继续前进。

FORWARD and BACKWARD immediately follow STRAIGHT and independently combine with either IR side and GT/LT. Omitted direction defaults to forward. Responses include `dir=FORWARD|BACKWARD`; existing bounds, sensor rules, caches and termination limits remain.

## Edge detection / 障碍边缘检测

`STRAIGHT,FORWARD,LEFTIR,RISING,30`：先遇到 IR<30 cm，继续行驶，再遇到 IR>30 cm 时停车。`STRAIGHT,BACKWARD,RIGHTIR,FALLING,30`：先遇到 IR>30 cm，再遇到 IR<30 cm 时停车。前后方向和左右探头可任意组合，等于阈值继续等待；每条新指令清空阶段，停车缓存最终读数。NEAR 视为小于，FAR 视为大于；采集错误报 SENSOR_ERROR，不作为边缘。

RISING requires below then above; FALLING requires above then below. Equality preserves phase without triggering. Each command clears phase, and errors abort rather than create crossings. Status and completion preserve the requested operator name.

IR 确认策略：GT/LT 连续两次满足条件才完成；RISING/FALLING 两个阶段各确认两次。等于阈值或不满足当前阶段时清零当前计数，首阶段确认后保持至本条指令结束。采样间隔约 40 ms。初始 GT/LT 已满足条件时在停车状态间隔 40 ms 确认第二次，再决定完成或起步。

## 连续行程后检测 / Continuous distance gate

例：`STRAIGHT,GOCM,20,RIGHTIR,RISING,30`，先前进 20 cm 不检测 IR，再连续运动并检测右侧先小于 30、后大于 30 的边缘。负 x 表示后退；编码器门槛不含停车补偿。状态追加 `gocm_cm` 和 `phase=GOCM/IR`，总时间／行程仍有限制，详见 PROTOCOL。

`STRAIGHT,GOCM,30,LEFTIR,RISING,50,LATERAL`：先直行 30 cm，再确认左 IR<50 的阶段并缓存该确认组的第一条读数，随后确认 IR>50 并停车，再以缓存值 +10 cm 向左侧移。两阶段各连续确认两次，不读取新的 IR 作为侧移距离。缓存须有效且不超过 45 cm，补偿后不得超过 LATERAL 的 55 cm 上限。

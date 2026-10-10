# 运动速度、衔接与制动设置 / Motion settings

设置位置：[robot_tuning.h](../Core/Inc/drivers/robot_tuning.h)。以下是
2026-09-23 文档更新时的配置；实际值以源码为准。修改后重新编译、烧录。
这些参数不是 UART 命令。

```c
#define MOTOR_SPEED_SCALE_PERCENT 120U
#define GYRO_RECALIBRATE_BEFORE_EACH_COMMAND 0U
```

## 全局 PWM 比例 / Global drive scale

`MOTOR_SPEED_SCALE_PERCENT` 支持 1～200：100 保持原 PWM，80 为 0.8 倍，
120 为 1.2 倍。缩放发生在共享电机驱动入口，覆盖直行、转弯、IR、超声波、
前进、倒车、起步，以及手动 MOTOR 指令。结果四舍五入并限制在电机 PWM 上限。
状态中的电机 PWM 是缩放后的值。单独设置一侧电机不会再次缩放另一侧。

**制动和舵机不参与缩放。** PWM 比例不是实测车速比例；比例过低可能无法起步，
提高比例可能改变停车误差。调整后复测距离和转弯角度。

The scale applies once to all shared drive outputs, including kickstart and manual
MOTOR commands. Reported motor PWM is the scaled output. Braking and servo pulses
are unchanged. Actual speed is not proportional to PWM; saturation is possible.

## 独立控制开关 / Independent control switches

以下开关位于 `robot_tuning.h`，默认均为 `0U`，保持拆分前的快速模式。
`1U` 启用对应行为，`0U` 关闭；修改后重新编译烧录。

| 开关 / Switch | 控制内容 / Behavior |
|---|---|
| `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` | 每段运动刷新零偏 / Refresh gyro bias before each motion |
| `MOTION_PREPARE_STOP_ENABLE` | 额外准备制动，转弯准备回中 / Additional preparation stops and turn centring |
| `MOTION_STILL_CHECK_ENABLE` | 额外静止检查 / Additional stationary check |
| `TURN_POST_STOP_SETTLE_ENABLE` | 转弯制动后等 500 ms / Post-turn 500 ms settling |
| `TURN_RIGHT_RETURN_ENABLE` | 右转结束 1500 us 保持 500 ms，再回中保持 500 ms / RIGHT steering holds |
| `TURN_LEFT_HEADING_OFF_ENABLE` | 左转成功后自动 HEADING,OFF / LEFT automatically disables heading |
| `TURN_END_HEADING_RESET_ENABLE` | 转弯成功后清空航向历史 / Reset heading history after successful turns |
| `SEQUENCE_COMMAND_GAP_ENABLE` | 执行序列配置的 gap_ms / Apply configured sequence gaps |

首次没有有效零偏时总会制动、确认静止并完整校准；启用重复零偏校准时也必须制动并确认静止，关闭其他准备开关不会取消这些校准前提。单独启用静止检查也会先制动。航向重置保留 ON/OFF 设置；启用左转自动 HEADING,OFF 则按该命令语义同时重置航向。右转舵机保持可在保留航向历史时单独启用。

First-use calibration always brakes and verifies stillness. Repeat bias calibration has the same prerequisites even if preparation switches are off. A standalone stillness check also brakes first. Heading reset preserves ON/OFF; LEFT HEADING,OFF additionally resets the origin by its existing semantics. RIGHT steering holds can run without resetting heading history.

转弯起步按 PWM 行程估算的舵机等待始终保留；目标制动、每段 yaw／编码器起点、舵机回差定位、超声波初始确认、STOP、故障处理、显式 WAIT 和启动倒计时都保留。诊断 observer 前的等待及配置间隔不受序列开关取消。各段仍制动后重新起步，没有跨段连续速度规划。

Turn-start PWM-travel waiting, target braking, per-motion measurement resets, servo backlash take-up, ultrasound confirmation, STOP, faults, explicit WAIT and startup delay remain. Diagnostic observers retain their waits/gaps. Motion segments still stop and restart.

## 转弯与制动 / Turns and braking

参数见 [turn_tuning.h](../Core/Inc/control/turn_tuning.h)：

- 起步 4000，巡航 3600，中速 3300，末段 3100，均为缩放前 PWM。
- 右转角度绝对值小于 90°（前进／倒车均适用） 使用独立速度分支，剩余不大于 20° 时用末段 PWM。
- 右转角度绝对值小于 90°（前进／倒车均适用），以及其他不大于 15° 的转弯：起步最多 60 ms 或已转 2°，
  先达到者结束起步。其他转弯起步最多 120 ms。停车判断始终优先。
- 正常指令转弯制动保持 `TURN_BRAKE_HOLD_MS=150`，期间持续积分陀螺仪；
  之后释放并使舵机回中。其他共享 `RobotHW_MotorStop()` 调用保持 150 ms。
- 双高制动使用 `CCR=ARR+1`（当前 TIM4／TIM9 的 ARR=7199），输出持续高电平。
  延长保持时间避免过早释放，不表示瞬时制动力同比增大。
- 小角度右转提前量 `TURN_RIGHT_SHORT_LEAD_TIME_S=0.17`，普通右转 0.17、左转 0.22。
  这些是角速度预测补偿时间，不是制动保持时间；过冲时增大、转不足时减小，
  每次调整后实测。`TURN_*_STOP_RATIO` 只用于旧的最小半径标定流程。

Normal command turns use non-blocking 150 ms braking; other shared blocking stop
calls hold for 150 ms. Stop prediction remains active even during kickstart.
Test 15/30/45/60/90/180 degrees under the actual load after speed changes.

## 验证 / Verification

实现阶段已通过 STM32 编译链接及 IR 主机模拟测试。未烧录或完成实车验证。
IR 行为与测试说明见 [IR_STRAIGHT.md](IR_STRAIGHT.md)。

同侧转弯的正负角度使用相同的速度曲线、起步限制、目标补偿和停车提前量；
LEFT 倒车对齐 LEFT 前进，RIGHT 倒车对齐 RIGHT 前进。
角度符号只改变驱动方向及陀螺仪进度的符号换算。

直行 GOCM 的正负距离共用速度曲线、距离补偿和停车条件。
前进与倒车舵机微调独立设置：STRAIGHT_FORWARD_TRIM_US 为 +42 us，
STRAIGHT_REVERSE_TRIM_US 为 +8 us，对应基准脉宽 1767／1733 us。
前进 1767 us 加陀螺仪反馈仍需实车验证；改变前进微调不会同步改变倒车。
倒车仍反转电机方向和转向纠偏符号。

转弯目标角度比例支持放大／缩小，合法范围为 0 < scale <= 2.0。
前进 TURN_LEFT_TARGET_SCALE 当前为 0.990，TURN_RIGHT_TARGET_SCALE 为 0.978；
倒车使用独立的 LEFT/RIGHT_REVERSE_TARGET_SCALE，不随前进比例改变。
例如无继承误差时 LEFT A=90 的内部目标为 89.10 度，再应用停车提前量，
目标是补偿停车过程后实际转到 90 度；这次调整仍需实车验证。
TURN_*_STEER_SCALE 仍限制在 (0, 1]，不受目标角度比例调整影响。


### 转弯起步舵机等待独立控制 / Independent turn-start steering wait

转弯指令起步前始终等待，GYRO_RECALIBRATE_BEFORE_EACH_COMMAND=0 也保留。
按下发 PWM 变化量估算，当前最低 80 ms，全端点行程 500 ms，参数为
TURN_SERVO_WAIT_MIN_MS 和 TURN_SERVO_WAIT_FULL_SPAN_MS。无实际舵角反馈，
需要实车负载验证；旧固定 500 ms 仍用于独立最小半径诊断。
Command turns always wait before driving, including with gyro recalibration off.
The open-loop PWM-travel estimate ranges from 80 to 500 ms; configurable in
turn_tuning.h. It is not a physical servo-position measurement. The standalone
minimum-radius diagnostic retains its own fixed 500 ms wait.

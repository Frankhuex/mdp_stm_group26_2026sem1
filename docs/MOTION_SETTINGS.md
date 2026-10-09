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

## 校准与衔接开关 / Calibration and transition waits

| 行为 / Behavior | `1U` | `0U` |
|---|---|---|
| 首次没有零偏时静止校准 / First-use calibration | 保留 / retained | 保留 / retained |
| 每条运动前重复校准 / Repeat bias calibration | 开启 / on | 跳过 / skipped |
| 重复准备停车与至少 250 ms 静止检查 / Repeated preparation | 保留 / retained | 跳过 / skipped |
| 转弯前 800 ms 舵机等待 / Turn servo wait | 保留 / retained | 跳过 / skipped |
| 转弯后 1000 ms 稳定等待 / Post-turn settling | 保留 / retained | 跳过 / skipped |
| 右转结束 1500 us／500 ms，再中位／500 ms / RIGHT end steering holds | 保留 / retained | 跳过 / skipped |
| 左转结束 HEADING,OFF 与航向重置 / LEFT end heading disable/reset | 保留 / retained | 跳过 / skipped |
| 执行器自动指令间隔（当前 300 ms） / Automatic sequence gap | 保留 / retained | 跳过 / skipped |
| 到达目标时制动 / Target braking | 保留 / retained | 保留 / retained |
| 每段 yaw／编码器清零 / Per-motion reset | 保留 / retained | 保留 / retained |
| 启动倒计时、显式 WAIT、STOP、故障处理 / Startup, WAIT, STOP, faults | 保留 / retained | 保留 / retained |

关闭开关会减少停顿，**不等于不停轮衔接**。车轮仍在每段目标位置制动，下一段
重新起步；也没有跨指令速度规划。舵机尚未到位时就可能开始行驶，轨迹需实测。
`0U` 时仍在正常转弯制动释放后使舵机回中，但直接完成，不执行上述左右转结束动作。
首次陀螺仪校准仍保留；航向 ON/OFF 模式不再由左转结束动作自动改变。
本开关不取消超声波首次测距确认，也不取消独立元件／标定测试的专用等待。

Fast mode reuses gyro bias and removes automatic preparation/settling delays in
the production straight/turn command path and sequence gaps. It does not remove
target braking, explicit WAIT, startup delay, sensor readiness or fault handling.
Steering may still be moving when drive starts; this is not continuous motion blending.

## 转弯与制动 / Turns and braking

参数见 [turn_tuning.h](../Core/Inc/control/turn_tuning.h)：

- 起步 4000，巡航 3600，中速 3300，末段 3100，均为缩放前 PWM。
- 右转角度绝对值小于 90°（前进／倒车均适用） 使用独立速度分支，剩余不大于 20° 时用末段 PWM。
- 右转角度绝对值小于 90°（前进／倒车均适用），以及其他不大于 15° 的转弯：起步最多 60 ms 或已转 2°，
  先达到者结束起步。其他转弯起步最多 120 ms。停车判断始终优先。
- 正常指令转弯制动保持 `TURN_BRAKE_HOLD_MS=150`，期间持续积分陀螺仪；
  之后释放并使舵机回中。其他共享 `RobotHW_MotorStop()` 调用保持 80 ms。
- 双高制动使用 `CCR=ARR+1`（当前 TIM4／TIM9 的 ARR=7199），输出持续高电平。
  延长保持时间避免过早释放，不表示瞬时制动力同比增大。
- 小角度右转提前量 `TURN_RIGHT_SHORT_LEAD_TIME_S=0.17`，普通右转 0.17、左转 0.22。
  这些是角速度预测补偿时间，不是制动保持时间；过冲时增大、转不足时减小，
  每次调整后实测。`TURN_*_STOP_RATIO` 只用于旧的最小半径标定流程。

Normal command turns use non-blocking 150 ms braking; other shared blocking stop
calls hold for 80 ms. Stop prediction remains active even during kickstart.
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

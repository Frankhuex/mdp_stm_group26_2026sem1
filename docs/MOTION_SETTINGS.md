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

以下开关位于 `robot_tuning.h`，当前仅重复陀螺仪零偏刷新为 `0U`，其余开关为 `1U`；实际配置以源码为准。
`1U` 启用对应行为，`0U` 关闭；修改后重新编译烧录。

| 开关 / Switch | 控制内容 / Behavior |
|---|---|
| `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` | 每段运动刷新零偏 / Refresh gyro bias before each motion |
| `MOTION_PREPARE_STOP_ENABLE` | 额外准备制动，转弯准备回中 / Additional preparation stops and turn centring |
| `MOTION_STILL_CHECK_ENABLE` | 额外静止检查 / Additional stationary check |
| `TURN_POST_STOP_SETTLE_ENABLE` | 转弯制动后等 150 ms / Post-turn 150 ms settling |
| `TURN_RIGHT_RETURN_ENABLE` | 右转结束 1500 us 保持 100 ms，再回中保持 100 ms / RIGHT steering holds |
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
按下发 PWM 变化量估算，当前最低 60 ms，全端点行程 400 ms，参数为
TURN_SERVO_WAIT_MIN_MS 和 TURN_SERVO_WAIT_FULL_SPAN_MS。无实际舵角反馈，
需要实车负载验证；旧固定 500 ms 仍用于独立最小半径诊断。
Command turns always wait before driving, including with gyro recalibration off.
The open-loop PWM-travel estimate ranges from 60 to 400 ms; configurable in
turn_tuning.h. It is not a physical servo-position measurement. The standalone
minimum-radius diagnostic retains its own fixed 500 ms wait.

UNTILUS 末段速度与制动参数位于 `straight_tuning.h`：最后 20 cm 使用 `US_FINAL_APPROACH_PWM=1800`，成功制动最少保持 `MOTOR_BRAKE_PULSE_MS=150` ms；编码器连续 `US_BRAKE_QUIET_MS=60` ms 无变化后释放，最长 `US_BRAKE_MAX_MS=600` ms。达到上限仍有编码器运动则报 STALL。IR、GOCM 与转弯制动参数保持原值。

UNTILUS final approach and brake settings are in `straight_tuning.h`: 1800 PWM within 200 mm after startup, at least 150 ms braking, 60 ms of encoder quiet before release, and a 600 ms maximum. Continued encoder motion at the maximum reports STALL.


当前衔接优化：运动准备制动仅一次，保持至少 150 ms，同时检查编码器静止；普通运动要求连续静止 100 ms，首次或重复零偏校准仍要求 250 ms。静止检查最多等待 1000 ms，失败释放制动并返回 NOT_STILL。转弯后的固定稳定等待为 150 ms，右转两段定位各 100 ms；舵机回差定位各 60 ms，转弯到位估算为 60～400 ms，继承航向的直行舵机等待为 60 ms。LOOP 与 FAST 额外指令间隔均为 0；显式 WAIT 保留。IR 检测基础 PWM 为 1900，采样间隔保持 40 ms；目标停车制动仍至少 150 ms。首次校准样本数量与等待、UNTILUS 静止释放规则保持原配置。缩短后的舵机等待与定位仍需实车验证。

Motion preparation now uses one >=150 ms brake with concurrent encoder checks (100 ms quiet normally, 250 ms before bias calibration). Turn settling is 150 ms; right return holds are 100 ms each; backlash holds are 60 ms each. Estimated turn steering wait is 60..400 ms and inherited straight steering wait is 60 ms. LOOP/FAST extra gaps are zero. IR uses base PWM 1900 and 40 ms sampling; target braking, explicit WAIT and calibration remain. Servo timings require hardware verification.

UNTILUS 起步回正与 GOCM 一致：对中心值加方向微调执行两段回差定位，每段等待 `SERVO_TAKEUP_SETTLE_MS`，随后执行准备检查与初始超声确认，再起步。
UNTILUS now uses the same two-stage backlash positioning as GOCM before driving, targeting the shared centre plus direction trim. Each hold uses SERVO_TAKEUP_SETTLE_MS; preparation checks and initial ultrasonic confirmation follow.

缓存接口：`Bypass_ForwardDistanceMm` 为左右共用的理想纵向位移（毫米），按实际整数指令角度计算，首次初始化为 UINT32_MAX，成功启动 BYPASS 后更新，失败启动不覆盖；STOP/故障后保留计划值，不表示已走距离。`Straight_ForwardUSStartMm`、`Straight_BackwardUSStartMm` 保存各方向 UNTILUS 本次启动后、轮子起步前的第一个新鲜有效超声读数（毫米）；接受新指令时只清空对应方向，另一方向保留。若起步前无有效回波而进入慢速移动，该次起始缓存保持 UINT32_MAX，恢复信号不补写成起步读数。
Cache interfaces: Bypass_ForwardDistanceMm is shared by both sides and stores rounded-angle planned forward displacement in mm after successful start, not measured travel. Straight_ForwardUSStartMm / Straight_BackwardUSStartMm store the first fresh valid pre-drive US sample for each gear. Only the selected cache resets on accepted UNTILUS start; missing pre-drive measurement remains UINT32_MAX even if echoes recover during motion.

UNTILUS 现有四个毫米缓存：前进 `Straight_ForwardUSStartMm` / `Straight_ForwardUSEndMm`，后退 `Straight_BackwardUSStartMm` / `Straight_BackwardUSEndMm`。默认 `STRAIGHT,UNTILUS,n` 与显式 FORWARD 共用前进缓存。接受命令时只清空所选方向的一对缓存；开始值为起步前首个新鲜有效样本，结束值为正常完成时的最终条件样本（与 DONE 回报一致，在制动前采集，未重新测量停车后距离）。STOP 或失败不会写成功结束值，未获得对应值为 UINT32_MAX。
UNTILUS has separate start/end pairs per gear, in mm. Implicit direction uses the FORWARD pair. Only the selected pair resets at accepted start. End caches hold the successful final condition sample, matching DONE; they are not fresh post-brake measurements. STOP/fault leave the selected end unavailable (UINT32_MAX).

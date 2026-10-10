# MDP21 STM32 ↔ RPi 通信协议（中文版）

**版本**：v1.15  
**日期**：2026-10-10  
**对端**：Raspberry Pi / PuTTY  
**实现依据**：[command_interface.c](../Core/Src/assessment/command_interface.c)

## 1. 连接与运行模式

- USART3：PD8=TX、PD9=RX；115200 baud，8N1，无流控。
- 使用板载 USB-UART 或与 RPi UART 交叉连接并共地。
- 使用命令时，将 `Core/Inc/test_config.h` 设为 `ASSESSMENT_COMMAND_INTERFACE`；
  当前选择 `TEST_HARDCODE_COMMAND_EXECUTOR`，用于执行硬编码命令数组。
- 启动时发送 `HELLO,mdp21_v8\r\n`；此固件标识与文档版本独立。

## 2. 报文格式

- 大写 ASCII，逗号分隔，不在字段之间加空格。
- 请求以 `\r\n`、单独 `\r` 或 `\n` 结束；回复统一为 `\r\n`。
- 接收缓冲区为 128 字节，最多接受 127 字节正文（不含行结束符）。溢出返回 `ERR,RX_LINE_OVERFLOW`，丢弃直到下一行结束。
- 空字段、整数溢出和无法解析的数字返回 `ERR,BAD_ARGS`；数值超出命令范围返回 `ERR,BAD_VALUE`。
- 当前 `CMD_ECHO_ENABLE=0`，STM 不回显输入；PuTTY 如需显示键入内容，请启用本地回显。
- 数值遥测通常为带正负号的三位小数，例如 `+12.340`；目标厘米数、角度和毫米测距为整数。

| 类型 | 格式 |
|---|---|
| 接受/成功 | `OK[,key=value...]` |
| 拒绝 | `ERR,<reason>` |
| 查询结果 | `<NAME>,key=value,...` |
| 运动终态 | `DONE,<motion>,[mode=UNTILUS,]state=<S>,...` |

`OK` 表示命令已接受，不能据此判断运动完成。接受的运动进入终态后只发送一次 `DONE`；`DONE` 前缀也用于失败和中止，必须检查 `state`。立即被拒绝的命令没有对应的完成事件。

| 错误 | 含义 |
|---|---|
| `BAD_ARGS` | 参数格式或数量错误，包括整数溢出 |
| `BAD_VALUE` | 参数超出范围 |
| `BUSY` | 运动占用或操作与运动冲突 |
| `NO_DEVICE` | 所需设备初始化失败 |
| `UNKNOWN_CMD` | 空闲时收到未知命令；运动中未知命令返回 `BUSY` |
| `RX_LINE_OVERFLOW` | 正文超过接收缓冲区容量 |
| `IR_MEASUREMENT_INVALID` | LATERAL 起始对应侧 IR 不可用或超出标定范围 |

`TIMEOUT`、`SENSOR_ERROR`、`DISTANCE_LIMIT` 是运动终态，出现在 `DONE`/状态查询中，不是本接口独立发送的 `ERR` 原因。

## 3. 直行命令

### 3.1 固定行程 GOCM

```text
STRAIGHT,GOCM,<cm>
OK,target_cm=<cm>
DONE,STRAIGHT,state=<S>,target_cm=<cm>
```

- `cm` 为有符号整数，绝对值 10～200；正值前进、负值倒车。
- 绝对值 10～20 使用 SHORT，21～200 使用 LONG。
- SHORT：前 100 ms 使用 PWM 3200，随后按进度使用 2950→2980→2350 曲线，普通 GOCM 增加 2 cm 延后刹车行程，目标净补偿为前进 −2 cm、倒车 −1 cm；CAL 保留原补偿。
- LONG：PWM 3300，任一轮累计达到 0.8 圈后降至 2400；基础目标圈数减去前进 0.310 圈、倒车 0.260 圈，普通 GOCM 再加 2/20.4 圈（2 cm），净扣减约为前进 0.212 圈、倒车 0.162 圈。以上 PWM 尚未乘全局比例。减速点按已走圈数判断，不是剩余距离。
- 共用 yaw/角速度→舵机 PID 和左右轮编码器增量同步；左右轮均达到补偿目标后完成 150 ms 刹车，期间继续采样 yaw 和编码器。
- 自动直行与转弯共享计划航向和陀螺仪估计航向。前进、倒车保持计划车身航向；转弯按请求角度更新计划航向，先根据进入转弯时的误差修正局部角度（最多 5 度，小转角最多一半），再应用已有左右转弯校准系数。倒车转弯的航向变化符号相反。制动和停车等待期间继续跟踪航向，传感器局部清零不会删除路线误差。
- 手动电机/舵机、STOP、编码器/yaw 重置及陀螺仪重新配置会重置路线基准。历史采样间断或单次读取失败仅标记可信度降低，不会因此拒绝下一条指令；持续的实时传感器故障仍报错停止。航向修正不能消除横向位置偏差。
- `HEADING,STATUS` 返回 `HEADING,enabled=<0|1>,target=<deg>,measured=<deg>,error=<deg>,tracking=<0|1>,degraded=<0|1>`，正角度为左转。停车时可用 `HEADING,RESET` 重设基准，或 `HEADING,ON` / `HEADING,OFF` 切换模式并重设基准。默认 ON；OFF 保留每段局部 PID，仍报告累计航向，但不跨段继承修正。测试方法见 [MOTION_SEQUENCE_TESTS.md](MOTION_SEQUENCE_TESTS.md)。
- 终态：`DONE`、`ABORTED`、`TIMEOUT`；超时为 `abs(cm) × 300 + 5000` ms。

查询 GOCM 状态：

```text
STRAIGHT,STATUS
STRAIGHT,state=<S>,elapsed=<ms>,target_cm=<int>,l_cm=<f>,r_cm=<f>,diff_cm=<f>,yaw=<f>,route_heading=<0|1>
```

### 3.2 超声波阈值 UNTILUS

```text
STRAIGHT,UNTILUS,<cm>
STRAIGHT,FORWARD,UNTILUS,<cm>
STRAIGHT,BACKWARD,UNTILUS,<cm>
OK,mode=UNTILUS,threshold_cm=<cm>,dir=<FORWARD|BACKWARD>
```

`cm` 为 5～200 的正整数，表示探头到障碍物的距离，与 `US,GET` 使用相同参考。前进与倒车均直接比较输入目标，不添加安装偏移。`OK`、完成事件和状态查询均回报输入阈值。例如输入 35，实际比较 350 mm，各回报均为 `threshold_cm=35`。

- `STRAIGHT,UNTILUS,n` 默认前进；`STRAIGHT,FORWARD,UNTILUS,n` 显式前进至距离小于目标；`STRAIGHT,BACKWARD,UNTILUS,n` 显式后退至距离大于目标。全程固定指定方向，不根据初始距离自动换向。命令与回报的方向名称一致。
- 初始距离已满足指定条件时（前进：严格小于目标；倒车：严格大于目标），连续两次新鲜有效且相差不超过 50 mm 的读数确认后静止完成。相等时仍按指定方向执行，其余有效起始读数确认两次后起步。
- 行驶保留提前刹车策略：前进在目标 + 提前量、后退在目标 − 提前量处确认，两次有效读数相差不超过 50 mm。两方向配置提前量均为 5 cm；初始距离已知时最多取初始误差的一半，无回波起步使用配置提前量，最终距离需实车验证。
- 无效、零值、过期或无回波均不判到达，也不直接报 SENSOR_ERROR 停车；按指定方向以基础 PWM 1900 低速起步／继续，并持续测量。恢复有效读数后继续正常控制和确认。无回波可能有多种原因，恢复无法保证；初始化／触发驱动失败与陀螺仪故障仍报错。STOP、堵转保护、30 秒超时与 200 cm 行程限制继续生效。
- PB15=TRIG，PC8=ECHO 经 5V→3.3V 分压；TIM8_CH3 在 1 MHz 下双边沿捕获。触发间隔至少 60 ms，测量超时 60 ms，有效样本年龄不得超过 200 ms。
- 基础 PWM：远距离起步 4000、巡航 3500、接近目标 2400。起始距目标不超过 35 cm 时使用 2600 起步，两轮开始运动后即可结束起步阶段；接近目标时立即降速，加速仍限制变化率。前进和倒车均在距离内部目标不足或等于 35 cm 时降速，速度切换具有 5 cm 回差，并保留轮速同步。以上数值尚未乘全局 PWM 比例。
- 最后 20 cm 在两轮起步确认后降至基础 PWM 1900，此时回波丢失不会加速。所有停车阈值（包括越过实际目标）均需连续两次有效读数确认，且两次相差不超过 50 mm；突变读数重新计数，无效读数清零。UNTILUS 成功制动至少保持 150 ms，并等待两轮编码器连续 60 ms 无变化，最长 600 ms；达到上限时车轮仍在运动则返回 STALL。
- 总运行上限 30 秒，任一轮编码器行程达到 200 cm 时停止。不使用 GOCM 行程补偿；成功停止仍执行刹车和舵机回中，最终实际距离可能与触发读数不同。

完成/故障事件的字段顺序：

```text
DONE,STRAIGHT,mode=UNTILUS,state=<S>,reason=<R>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,dir=<FORWARD|BACKWARD|NONE|UNKNOWN>,us_age_ms=<uint>
```

| `state` | `reason` | 含义 |
|---|---|---|
| `DONE` | `US_THRESHOLD` | 确认提前刹车阈值，或起始已满足指定条件 |
| `ABORTED` | `ABORTED` | 用户发送停止命令 |
| `SENSOR_ERROR` | `SENSOR_ERROR` | 触发驱动失败或陀螺仪故障 |
| `TIMEOUT` | `TIMEOUT` | 达到总时间上限 |
| `DISTANCE_LIMIT` | `DISTANCE_LIMIT` | 达到编码器行程上限 |

成功完成时，`us_mm` 与 `us_status=0` 回报刹车前保存的条件触发样本，DONE 发送延迟超过 200 ms 也不会丢失。`dir` 为 FORWARD／BACKWARD，起始已满足条件并静止完成时为 NONE；`us_age_ms` 是触发时样本年龄。故障／中止仍按当前样本有效性和年龄回报，`dir=UNKNOWN`。这些字段表示触发停车的测量，停车后没有重新测量。实时 `STRAIGHT,STATUS` 仍将过期样本显示为 INF/-1。

查询 UNTILUS 状态（替代 GOCM 的状态格式）：

```text
STRAIGHT,STATUS
STRAIGHT,state=<S>,mode=UNTILUS,elapsed=<ms>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,us_age_ms=<uint>,l_cm=<f>,r_cm=<f>,diff_cm=<f>,yaw=<f>
```

`RUNNING` 包括启动测距等待和实际行驶。`us_mm` 单位为毫米，`us_age_ms` 为最后一份样本的年龄，`elapsed` 为命令运行毫秒数。`us_status=0` 表示有效且新鲜，`-1` 表示无样本/无上升沿/已过期，`-2` 表示无效或过长回波。非零状态时不要将 `us_mm` 当作有效距离。终态后保留最后样本，但其年龄继续增加，因此之后查询可能显示过期。

### 3.3 侧面 IR 阈值 LEFTIR / RIGHTIR

边缘检测指令：`STRAIGHT,FORWARD/BACKWARD,LEFTIR/RIGHTIR,RISING/FALLING,n`。RISING 先检测到严格小于 n，记录阶段但不停，再在后续读数严格大于 n 时停车；FALLING 先检测到严格大于 n，再在后续读数严格小于 n 时停车。等于阈值既不记录首阶段，也不停车，已记录的阶段继续保留。启动时的读数可以记录首阶段，仅处于最终一侧不会完成；每条新指令清除此前阶段。NEAR 视为小于，FAR 视为大于；采集／转换错误以 SENSOR_ERROR 终止，不能作为越沿信号。正常停车缓存最终越沿读数，FAR 用 UINT32_MAX。方向、阈值范围、STOP、超时和行程限制沿用，省略方向默认前进。两个阶段分别要求连续两次满足条件；等于阈值或不满足当前阶段时清零当前计数，已确认的首阶段保留。GT/LT 同样连续确认两次，新指令清空计数。未增加距离回差。

方向参数紧跟 STRAIGHT：`STRAIGHT,FORWARD,LEFTIR/RIGHTIR,GT/LT,n` 或 `STRAIGHT,BACKWARD,LEFTIR/RIGHTIR,GT/LT,n`。省略方向默认前进，左右探头与 GT/LT 条件均可独立组合；BACKWARD 回报 `dir=BACKWARD`。两方向使用相同的 IR 故障处理与左右缓存规则。

格式为 `STRAIGHT,LEFTIR/RIGHTIR,GT/LT,n`，n 为整数厘米。GT 严格大于阈值停车，LT 严格小于阈值停车，等于阈值继续；旧三段格式保留为 GT。左侧支持 10～50 cm，右侧支持 10～60 cm，回复包含 `op=GT|LT`。LT 遇到 NEAR 也正常完成，FAR 继续；ADC／转换错误以 SENSOR_ERROR 停止并中止序列。下文无限远及故障正常 DONE 规则仅适用于 GT。

```text
STRAIGHT,LEFTIR,GT,50
STRAIGHT,RIGHTIR,GT,50
OK,mode=<LEFTIR|RIGHTIR>,op=<GT|LT|RISING|FALLING>,threshold_cm=<int>,dir=<FORWARD|BACKWARD>
DONE,STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT|RISING|FALLING>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>,dir=<FORWARD|BACKWARD>
```

- 向前直行，选定侧距离**严格大于**阈值时停车；等于阈值继续。阈值是传感器距离，不加距离偏移量。
- 阈值为整数厘米：LEFTIR 支持 10～50，RIGHTIR 支持 10～60，与当前标定表一致。
- `FAR`、ADC 读取失败和转换错误均按无限远处理，正常回报 `state=DONE,ir_cm=FAR`，不作为 IR 传感器故障。
- 起步前先采样，已满足条件则直接完成而不驱动车轮。行驶时约每 40 ms 采样一次，一次满足即可停车。
- 所有启用 IR 比较的阶段从起步起统一使用基础 PWM 1900，涵盖前后方向、左右探头、GT/LT/RISING/FALLING，以及有无 GOCM／LATERAL 的组合；仍使用全局 PWM 比例和直行方向／左右轮同步控制。GOCM 前段沿用原速度曲线，到达 IR 检测切换点立即降速。
- STOP 返回 ABORTED；30 秒超时返回 TIMEOUT；任一轮行程达到 200 cm 返回 DISTANCE_LIMIT；陀螺仪故障仍可返回 SENSOR_ERROR。这些非成功终态会中止序列。
- 硬编码执行器收到正常 DONE 后进入下一行，包含 FAR／ADC 失败触发的正常完成。

运行中及完成后的查询格式：

```text
STRAIGHT,STATUS
STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT|RISING|FALLING>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>,dir=<FORWARD|BACKWARD>
```

数值距离保留一位小数；`FAR` 不区分超量程和 ADC 失败。详见 [IR_STRAIGHT.md](IR_STRAIGHT.md)。

条件满足并正常停止时，将对应侧读数保存到 `Straight_LeftIRStopCmX10`／`Straight_RightIRStopCmX10`，单位为 0.1 cm；未缓存、FAR 或 GT 读取失败用 `UINT32_MAX` 表示。左右独立更新，STOP、超时和其他故障保留旧值。LATERAL 的 IR 模式自行现场测量。

GT/LT 起始条件已满足时，电机保持停车，间隔 40 ms 进行第二次确认，再决定完成或起步。行驶期间两次相邻 IR 采样间隔约 40 ms。

### 3.3.1 连续 GOCM 后检测 IR

`STRAIGHT,GOCM,x,LEFTIR/RIGHTIR,GT/LT/RISING/FALLING,n`：先按编码器行驶 x cm，然后不停、不重新起步地启用 IR 检测。x 为有符号整数，正数前进、负数后退，绝对值 10～200 cm；n 沿用左侧 10～50 cm、右侧 10～60 cm。两轮均达到 |x| 前不采样 IR，越沿阶段与确认计数仅从启用检测后开始。切换点按标定编码器实际行程判断，不使用普通 GOCM 的停车距离补偿；起始段沿用 GOCM 速度曲线，再切换至 IR 巡航，编码器累计量和航向保持。

接受回报 `OK,mode=GOCM_IR,gocm_cm=x,ir=LEFTIR/RIGHTIR,op=...,threshold_cm=n,dir=FORWARD/BACKWARD`。状态与唯一的最终 IR 完成事件追加 `gocm_cm=x,phase=GOCM/IR`。GT/LT 及越沿的两个阶段均连续确认两次，左右缓存和故障规则沿用。STOP 可取消任一阶段；整条命令上限为 30 秒、总行程 200 cm，包含初始距离，x 达到行程上限时没有后续检测行驶空间。

### 3.3.2 越沿后自动侧移

`STRAIGHT,GOCM,x,LEFTIR/RIGHTIR,RISING/FALLING,n,LATERAL`：走满 x 后不停地检测 IR。RISING 从第一次有效读数小于阈值时开始缓存，此后每次读取有效 IR 都比较并维护最小值，直到确认越沿停车；首阶段计数被打断也保留最小值。FALLING 维持首阶段第一条读数的缓存规则，未确认的孤立读数会丢弃。两个阶段仍分别连续确认两次。第二阶段也确认两次，再制动停车，随后朝所选 IR 侧执行 LATERAL，侧移目标为缓存 IR +10 cm（探头到中线 5 cm + 障碍物半宽 5 cm）。不重新读 IR，也不执行单独 LATERAL,IR 的前进等待阶段。

侧移缓存必须为有效标定的有限距离，NEAR/FAR 不能作为侧移距离。缓存 +10 不得超过 55 cm，即缓存不得超过 45 cm；否则越沿停车后报 LATERAL START_ERROR，不启动转弯，无有效缓存时 start_rc=-5、越界时 -2。前段故障同样阻止转弯，STOP 取消任意阶段。整个动作在 LOOP 和命令数组中视为一条指令，只输出一次最终 `DONE,LATERAL,...`；`source=EDGE_IR`，`ir_cm_x10` 是 RISING 最小值或 FALLING 首阶段缓存，`d_cm` 含补偿。阶段 0 为前段连续直行／IR 检测，1／2 为侧移转弯；接受回报使用 `mode=GOCM_IR_LATERAL`。

### 3.4 停止与时序

- `STRAIGHT,STOP` → `OK,aborted=<0|1>`；若中止运行中的直行（含测距等待），随后发送一次 `state=ABORTED` 完成事件。
- `STOP` → `OK`；停止电机、舵机回中，并中止正在运行的直行、转弯或侧移，随后发送对应完成事件。
- 首次运动可能在回复 `OK` 前进行陀螺仪初始化和约 1.7 秒加初始化／读取耗时的静止校准；后续零偏刷新由 `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` 控制，其他等待和结束动作使用独立开关，详见 [MOTION_SETTINGS.md](MOTION_SETTINGS.md)。首次初始化阶段仍是阻塞的。
- 运动控制按至少 20 ms 间隔步进；UART 收发仍可能延迟主循环。运动期间跳过 OLED 命令刷新，独立陀螺轮询暂停，避免重复积分。没有硬实时停车时延保证。

## 4. 转弯命令

指令转弯的起步舵机等待始终启用，独立于 `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND`。下发目标舵角后电机保持零输出，估算到位时间结束才起步。当前估算为 `60 + ceil(340 × abs(目标PWM − 上次PWM) / 1600)` ms，范围 60～400 ms，上次指令超出配置端点时等待 400 ms；参数位于 `turn_tuning.h`。估算依据为指令 PWM 变化量，没有实际舵角反馈，需带负载验证。小幅调整等待短，大幅调整等待长；此等待采用非阻塞状态，可响应 STOP。

| 请求 | 参数 / 回复 |
|---|---|
| `TURN,LEFT,R=<cm>,A=<deg>` | R 为整数且 ≥18；A 为有符号整数，绝对值 5～360 |
| `TURN,RIGHT,R=<cm>,A=<deg>` | R 为整数且 ≥35；A 同上 |
| 接受回复 | `OK,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| 终态事件 | `DONE,TURN,state=<S>,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| `TURN,STOP` | `OK,aborted=<0\|1>`；若中止动作，随后发一次完成事件 |
| `TURN,STATUS` | `TURN,state=<S>,dir=<LEFT\|RIGHT>,elapsed=<ms>,r_cm=<int>,target_deg=<int>,yaw=<f>` |

LEFT/RIGHT 选择转向侧，A 正值前进、负值倒车。R 无额外配置上限，但必须能解析为 int32。终态文本包括 `DONE`、`ABORTED`、`TIMEOUT`、`WRONG_SIGN`、`NO_TURN`、`BAD_CONFIG`、`NO_IMU`（后两项为接口可识别状态）。当前转弯参数以 [turn_tuning.h](../Core/Inc/control/turn_tuning.h) 和 [turn_control.c](../Core/Src/control/turn_control.c) 为准。

转弯结束采用独立开关，当前均开启：`TURN_POST_STOP_SETTLE_ENABLE` 控制制动后 150 ms 等待；`TURN_RIGHT_RETURN_ENABLE` 控制右转结束两段 100 ms 舵机保持；`TURN_LEFT_HEADING_OFF_ENABLE` 控制左转成功后自动关闭航向；`TURN_END_HEADING_RESET_ENABLE` 独立控制成功转弯后的航向历史重置。前进和倒车转弯均适用。`GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` 只决定重复刷新零偏，校准所需停车和静止确认始终执行。目标制动与起步舵机估算等待始终保留。完整开关见 [MOTION_SETTINGS.md](MOTION_SETTINGS.md)。

### 4.1 侧向平移 LATERAL

| 请求 | 行为 |
|---|---|
| `LATERAL,LEFT,IR`／`LATERAL,RIGHT,IR` | 保存对应侧现场 IR，前进至该侧 IR >50 cm，再按起始 IR +10 cm 执行两次反向转弯 |
| `LATERAL,LEFT,n`／`LATERAL,RIGHT,n` | 直接按整数 n cm 侧移，范围 1～55；无 IR 直行阶段及补偿 |
| `LATERAL,STATUS` | 查询组合动作、起始 IR 和当前阶段 |
| `LATERAL,STOP` | 返回 `OK`，取消整个侧移动作 |

IR 补偿为探头到车体中线 5 cm，加上 10 cm 障碍宽的一半 5 cm，总计 10 cm。不补偿车长，也不增加额外直行距离。起步前检查初始测量及最终目标：初始 IR 必须为 VALID 且在标定范围 10～50 cm 内，起始 IR +10 不得超过 55 cm，因此可用起始距离为 10～45 cm。初始 ADC 失败、FAR、NEAR 或无效距离返回 `ERR,IR_MEASUREMENT_INVALID`；最终目标超过 55 cm 返回 `ERR,BAD_VALUE`。数值参数无效返回 `ERR,BAD_VALUE`，字段格式或数量错误返回 `ERR,BAD_ARGS`。

前进阶段复用直行控制和刹车，采用严格读取故障策略：等于 50 cm 继续，大于 50 cm 或 FAR 完成直行；ADC／对应侧转换错误会中止。STOP、30 秒超时、200 cm 行程上限及陀螺仪故障均阻止后续转弯。后续实时测量及直行停止缓存的变化不会覆盖本次起始 IR。

LEFT 先左转后右转，RIGHT 先右转后左转。左转采用 20 cm 软限制，右转半径 35 cm；普通 TURN 的左转硬极限保留 18 cm。目标侧移 d 的两段名义角度均为 `acos(1 − d/55) × 180/π`，四舍五入为整数度，每段最多 90°。两段圆弧理论前移为 `sqrt(110d − d²)`，IR 模式还包含之前长度由测量决定的直行。例如数值指令 `LATERAL,LEFT,20` 执行两次 50° 转弯，对应理想前移约 42.13 cm、左移约 19.65 cm。仍应用已有转弯校准、刹车和转弯结束动作，实际效果需地面验证。

接受后发送 `OK,LATERAL`，再发送当前 LATERAL 状态。整个动作结束时只发送一个组合事件，内部直行及转弯不单独发送 `DONE,STRAIGHT`／`DONE,TURN`：

`LATERAL,state=<S>,dir=<LEFT|RIGHT>,source=<IR|CM>,d_cm=<一位小数>,a=<整数>,stage=<0|1|2>,ir_cm_x10=<无符号整数>,straight_state=<S>,turn_state=<S>,start_rc=<整数>`

`DONE,LATERAL,state=<S>,dir=<LEFT|RIGHT>,source=<IR|CM>,d_cm=<一位小数>,a=<整数>,stage=<0|1|2>,ir_cm_x10=<无符号整数>,straight_state=<S>,turn_state=<S>,start_rc=<整数>`

阶段 0 为 IR 前进，1 为第一次转弯，2 为第二次转弯；数值模式从阶段 1 开始，`ir_cm_x10=0`。起始 `ir_cm_x10` 单位为 0.1 cm，300 表示 30 cm；`d_cm` 仅在 IR 模式包含补偿。状态包括 IDLE、RUNNING、DONE、ABORTED、STRAIGHT_ERROR、TURN_ERROR、START_ERROR。失败保留失败阶段以及底层控制器状态或启动返回码；`a` 为正整数指令角度，与实测 yaw 分别记录。

`STOP`、`STRAIGHT,STOP`、`TURN,STOP`、`LATERAL,STOP` 都取消整个侧移动作，阶段间取消后不会启动后续转弯。所有阶段保持运动互斥，其他运动返回 BUSY。硬编码／LOOP／FAST 序列等待整个动作完成，失败时停止，原有回复屏蔽规则仍有效。序列执行期间应使用全局 `STOP` 取消；可查询 `LATERAL,STATUS`，传入各子命令 STOP 形式会被序列入口拒绝为 BUSY。

### 4.2 三段转弯绕障：BYPASS

- `BYPASS,LEFT,n` / `BYPASS,RIGHT,n`：全程前进、三段转弯，`n` 为 1～55 cm 的整数，表示最大侧向偏移。理想终点回到原行进直线并恢复原朝向。数值模式不额外加 10 cm 补偿，当前不支持 IR 模式。
- RIGHT 按右→左→右执行，LEFT 按左→右→左执行。左转调用已有的 20 cm 软限制半径，18 cm 硬极限保持；右转调用已有的 35 cm 最小半径。
- 外侧角度 `a = round(acos(1 - n / (20 + 35)) × 180 / π)`，中间角度严格使用 `2a`，保证整数取整后的净转角为零。理想前进距离为 `2 × (20 + 35) × sin(a)`。`n=20` 时执行 50°、100°、50°。
- `BYPASS,STATUS` 输出 `state,dir,d_cm,a,middle_a,stage,turn_state,start_rc`，阶段 1～3 对应当前转弯。接受指令后输出 `OK,BYPASS` 和状态行，结束或中断时只输出一条 `DONE,BYPASS,...`；内部转弯不单独输出完成事件。
- `BYPASS,STOP` 或全局 `STOP` 取消剩余转弯，转弯异常会终止整段动作。运动中启动返回 `ERR,BUSY`，距离越界返回 `ERR,BAD_VALUE`。支持 `LOOP,ONCE` 与硬编码命令数组。
- 起始距离和障碍物纵向长度需要与路径匹配；最大侧移量无法单独保证整个车身的绕障净空，需实车验证。

## 5. 系统、手动控制与查询

以下回复均以 `\r\n` 结束。`<f>` 为三位小数，`<int>` 为整数。

| 请求 | 回复 / 说明 |
|---|---|
| `HELLO` | `HELLO,mdp21_v8` |
| `PING` | `PONG` |
| `UPTIME` | `UPTIME,ms=<int>` |
| `LOOP,ONCE` | 执行一轮偏移后的 assessment loop：`TURN,RIGHT,R=35,A=90`、`STRAIGHT,GOCM,40`、`TURN,LEFT,R=20,A=180`、`STRAIGHT,UNTILUS,47`；只输出一个 `OK` 和一个最终 `DONE,LOOP` |
| `MOTOR,L,<pwm>` | `OK`；左轮，正前进负倒车，绝对值不超过左电机 PWM 上限 |
| `MOTOR,R,<pwm>` | `OK`；右轮，对应右电机 PWM 上限 |
| `MOTOR,B,<l>,<r>` | `OK`；同时设置两轮 |
| `STEER,US,<us>` | `OK`；原始舵机比较值，没有软件范围钳位 |
| `STEER,CENTER` | `OK`；共享中心 1726 µs（robot_tuning.h） |
| `STEER,GET` | `STEER,us=<int>,center=<int>` |
| `ENC,RESET` | `OK`；清零编码器 |
| `ENC,GET` | `ENC,l=<int>,r=<int>`；正向归一化计数 |
| `ENC,GET,RAW` | `ENC,l=<int>,r=<int>`；原始有符号计数 |
| `ENC,GET,CM` | `ENC,l_cm=<f>,r_cm=<f>` |
| `GYRO,INIT` | `OK` 或 `ERR,NO_DEVICE` |
| `GYRO,CALIB` | `OK,offset=<int>`；需静止校准 |
| `GYRO,Z` | `GYRO,z=<f>,raw=<int>` |
| `GYRO,OFFSET` | `GYRO,offset=<int>` |
| `GYRO,SET_OFFSET,<int>` | `OK` |
| `YAW,RESET` | `OK` |
| `YAW,GET` | `YAW,deg=<f>` |
| `STATUS` | `STATUS,motor_l=<int>,motor_r=<int>,steer=<int>,enc_l=<int>,enc_r=<int>,yaw=<f>,gyro_z=<f>,uptime=<int>` |
| `STATUS,MOTOR` | `MOTOR,l=<int>,r=<int>` |
| `STATUS,ENC` | `ENC,l=<int>,r=<int>` |

运动中或 `LOOP,ONCE` 序列执行中互斥：新的运动命令、`MOTOR,*`、`STEER,US`、`STEER,CENTER`、`ENC,RESET`、`YAW,RESET`，以及除 `GYRO,OFFSET` 之外的 `GYRO` 操作均返回 `ERR,BUSY`。状态查询和停止命令仍可使用。

**当前未实现 `HB`、心跳停车、`RESET`、`PARAM,GET/SET` 和 `STEER,ANGLE`。** 不应依赖通信断开后自动停止手动 PWM。闭环运动依靠自身终止条件或上限结束，无需发送心跳；周期查询也不是运行条件。

## 6. LOOP,ONCE 序列命令

### FAST 最快车分组命令

支持 `FAST,APPROACH`、`FAST,LEFT1`、`FAST,LEFT2`、`FAST,RIGHT1`、`FAST,RIGHT2`。APPROACH 当前仅执行 `STRAIGHT,UNTILUS,28`（默认前进），接受回报 `OK,fast=APPROACH,count=1`，成功回报 `DONE,FAST,route=APPROACH,state=DONE,count=1`；BUSY、STOP 和失败处理沿用其他 FAST 命令组。
它们分别执行 `test_hardcode_command_executor.c` 中同名分组的动作，
LEFT1／RIGHT1 各 6 条，LEFT2／RIGHT2 各 12 条。

与 LOOP,ONCE 共用序列执行器，每条运动成功完成后才开始下一条。
FAST 不增加启动倒计时或额外指令间隔；运动准备、重复陀螺仪校准、
转弯结束动作使用 robot_tuning.h 中的独立开关。
不会强制切换航向模式。每组结束或失败后停车、回中。

例如发送 `FAST,LEFT1`，先返回 `OK,fast=LEFT1,count=6`，
成功结束返回 `DONE,FAST,route=LEFT1,state=DONE,count=6`。
失败返回 `DONE,FAST,route=LEFT1,state=<原因>,index=<失败步骤>,count=6`。
内部运动回复和 SEQ 进度行被屏蔽，诊断仍可在停止后用 `SEQ,LOG` 读取。
执行期间新运动／FAST／LOOP 命令返回 `ERR,BUSY`，`STOP` 取消剩余动作；
STOP 和原有状态查询仍可使用。未知分组或多余参数返回 `ERR,BAD_ARGS`。

这些指令也可作为硬编码数组中的一行：每组完成后再执行下一行，组内失败或
STOP 会终止外层剩余动作，日志追加到同一份序列记录。
硬编码测试模式仍自动启动数组，不会变成等待串口启动。

`LOOP,ONCE` 是 `ASSESSMENT_COMMAND_INTERFACE` 模式下可用的普通 UART 命令。
它复用 `TEST_HARDCODE_COMMAND_EXECUTOR` 的序列执行器，执行一轮偏移后的
assessment loop。固定命令列表为：

```text
TURN,RIGHT,R=35,A=90
STRAIGHT,GOCM,40
TURN,LEFT,R=20,A=180
STRAIGHT,UNTILUS,47
```

命令被接受后，立即返回：

```text
OK,loop=ONCE,count=4
```

这个 `OK` 只表示 `LOOP,ONCE` 已被接受，并且第一条排队的运动将开始执行，
不表示任何运动已经完成。`LOOP,ONCE` 会屏蔽 `SEQ,*` 进度行，也会屏蔽内部运动
子命令各自的 `OK` / `DONE`。

```text
DONE,LOOP,state=DONE,count=4
```

一次成功执行的完整形态如下：

```text
> LOOP,ONCE
< OK,loop=ONCE,count=4
< DONE,LOOP,state=DONE,count=4
```

最终的 `DONE,LOOP` 表示四条内部运动都到达了各自的成功终态。

如果某一行被拒绝、用户发送 `STOP`，或某个运动以非成功终态结束，loop 仍然只输出
一个 loop 级 `DONE`：

```text
DONE,LOOP,state=<reason>,index=<1..4>,count=4
```

例如超声波步骤超时：`DONE,LOOP,state=TIMEOUT,index=4,count=4`。
如果用户用 `STOP` 取消：`DONE,LOOP,state=USER_STOP,index=<current>,count=4`。

`LOOP,ONCE` 执行期间，UART 仍接受 `STOP`、`PING`、`STATUS`、
`STRAIGHT,STATUS`、`TURN,STATUS`、`LATERAL,STATUS` 和 `HEADING,STATUS`。其他传入命令返回 `ERR,BUSY`。

## 7. UNTILUS 交互示例

示意值，不代表实车测量。`>` 为发送，`<` 为接收，不属于报文本身。

```text
> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20,dir=FORWARD
> STRAIGHT,STATUS
< STRAIGHT,state=RUNNING,mode=UNTILUS,elapsed=420,threshold_cm=20,us_mm=320,us_status=0,us_age_ms=12,l_cm=+8.300,r_cm=+8.280,diff_cm=+0.020,yaw=+0.100
< DONE,STRAIGHT,mode=UNTILUS,state=DONE,reason=US_THRESHOLD,threshold_cm=20,us_mm=197,us_status=0,dir=FORWARD,us_age_ms=0

> STRAIGHT,UNTILUS,4
< ERR,BAD_VALUE

> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20,dir=FORWARD
> STOP
< OK
< DONE,STRAIGHT,mode=UNTILUS,state=ABORTED,reason=ABORTED,threshold_cm=20,us_mm=INF,us_status=-1,dir=UNKNOWN,us_age_ms=0
```

最后一个示例在取得首个有效样本前中止。RPi 应按字段名解析模式与终态，避免把 UNTILUS 的阈值当作 GOCM 的行程目标。

## 8. 配置与验证

- v1.15 同步双向 UNTILUS、左右 IR 停止缓存，以及两种 LATERAL 模式、IR 前进阶段和 10 cm 补偿。

- [MOTION_SETTINGS.md](MOTION_SETTINGS.md)：全局 PWM 比例、校准与指令衔接等待开关、主动制动。修改后须重新编译和烧录，不是 UART 动态参数。
- [IR_STRAIGHT.md](IR_STRAIGHT.md)：LEFTIR／RIGHTIR 指令及 `IR_COMMAND_*` 参数。
- v1.10 新增侧面 IR 指令及 FAR／ADC 失败正常完成规则。实现阶段已通过固件编译链接与主机模拟控制测试，尚未实车验证。

- [straight_tuning.h](../Core/Inc/control/straight_tuning.h)：SHORT/LONG 与 `US_*` 参数。
- [ULTRASONIC_STRAIGHT.md](ULTRASONIC_STRAIGHT.md)：接线、完整行为和手动验证步骤。
- [FLOOR_RETUNING.md](FLOOR_RETUNING.md)：地面变化后的控制参数校准。
- v1.9 同步了 UNTILUS、故障终态、运动互斥，以及当前无回显/无心跳的实现；修正了旧版 GOCM 范围和控制说明。
- 当前速度、转向衔接与停车余量仍需实车验证。

## 硬编码命令数组测试

`TEST_HARDCODE_COMMAND_EXECUTOR` 将命令数组执行一次，等待每个运动成功后再继续，遇到错误停止。此模式的 UART 仅接受 STOP、PING、STATUS、STRAIGHT,STATUS、TURN,STATUS、LATERAL,STATUS 和 HEADING,STATUS。WAIT,<ms> 仅可用于数组。详见 [HARDCODE_COMMAND_EXECUTOR.md](HARDCODE_COMMAND_EXECUTOR.md)。

`us_mm=INF` 表示未知/无限远，`us_status` 仍为非零。上位机解析此字段时须同时支持数值和字符串 INF。

`LEFTIR,GET`／`RIGHTIR,GET` 在停车状态现场读取对应侧 IR 一次。回报 `LEFTIR/RIGHTIR,cm=<小数|NEAR|FAR|INVALID>,raw=<原始ADC|NA>,mv=<毫伏|NA>,status=<VALID|NEAR|FAR|ERROR>,rc=<驱动返回码>`。近／远超量程使用 NEAR/FAR，不伪造数值距离；ADC 失败回报 ERROR 和非零 rc。运动中返回 ERR,BUSY；读取不修改越沿阶段、确认计数或停车缓存。

IR 状态与完成事件追加所选侧的 `ir_raw,ir_status,ir_age_ms,confirm,armed`，分别表示原始 ADC、VALID/NEAR/FAR/ERROR 状态、样本年龄、连续确认计数和越沿首阶段是否确认。均为控制器最后采样，查询不额外采样；原始 ADC 不可用时为 UINT32_MAX，带距离门槛的命令在启用检测前没有 IR 样本。

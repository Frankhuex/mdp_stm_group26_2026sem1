# MDP21 STM32 ↔ RPi 通信协议（中文版）

**版本**：v1.10  
**日期**：2026-09-23  
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
- SHORT：前 100 ms 使用 PWM 3200，随后按进度使用 2950→2980→2350 曲线，目标距离补偿为前进 −4 cm、倒车 −3 cm。
- LONG：PWM 3300，任一轮累计达到 0.8 圈后降至 2400；目标圈数减去前进 0.310 圈、倒车 0.260 圈。以上 PWM 尚未乘全局比例。减速点按已走圈数判断，不是剩余距离。
- 共用 yaw/角速度→舵机 PID 和左右轮编码器增量同步；左右轮均达到补偿目标后完成 80 ms 刹车，期间继续采样 yaw 和编码器。
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
OK,mode=UNTILUS,threshold_cm=<cm>
```

`cm` 为 **5～200 的正整数**，表示车头保险杠到障碍物的距离阈值，不是行驶距离。固件会在内部加上 8 cm，因为超声波探头位于保险杠后方；控制器实际比较的是探头到障碍物的距离。仅向前行驶。首次取得有效且新鲜的读数 **严格小于** 内部阈值时停车；等于阈值不完成。

- 先等待启动确认；连续两次有效读数低于阈值时，直接完成而不启动电机。连续两次有效读数高于阈值时开始前进。连续三次未知读数也会开始前进，并按无限远处理。
- PB15=TRIG、PC8=ECHO（经过 5V→3.3V 分压），与工作示例 `mdp21_8_ultrasonic` 一致。TIM8_CH3 在 1 MHz、ARR=65535 下双边沿捕获；驱动启动输入捕获中断并读取 CCR3。
- 触发间隔至少 60 ms，整次测量超时为 60 ms。2+10 µs 触发脉冲由 DWT 计时，不重置捕获定时器或 HAL 时基。采集逻辑按工作示例执行：仅将大于 25000 µs 的脉冲判为超量程，不再添加最小脉宽或边沿阶段过滤。
- 无回波、无效/零读数或超过 200 ms 的旧读数按无限远处理，保持巡航 PWM 并继续测距。取消 500 ms 启动失败及读数过期停车；STOP、30 秒超时和 200 cm 行程上限仍有效。
- 基础 PWM：kickstart 4000，巡航 3500，读数不大于“阈值 + 25 cm”时降至 2600，叠加原有轮速同步修正。
- 总运行上限 30 秒；任一轮编码器测得行程达到 200 cm 时停止。阈值与行程限制是不同概念。
- 不使用 GOCM 的行程补偿。停止时先短暂将每个电机的两个输入同时拉高进行刹车，然后将 PWM 归零并使舵机回中；最终静止距离可能小于触发停车时的读数。

完成/故障事件的字段顺序：

```text
DONE,STRAIGHT,mode=UNTILUS,state=<S>,reason=<R>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>
```

| `state` | `reason` | 含义 |
|---|---|---|
| `DONE` | `US_THRESHOLD` | 有效新鲜测距严格低于阈值 |
| `ABORTED` | `ABORTED` | 用户发送停止命令 |
| `SENSOR_ERROR` | `SENSOR_ERROR` | 传感器驱动不可用；无回波不会触发此状态 |
| `TIMEOUT` | `TIMEOUT` | 达到总时间上限 |
| `DISTANCE_LIMIT` | `DISTANCE_LIMIT` | 达到编码器行程上限 |

查询 UNTILUS 状态（替代 GOCM 的状态格式）：

```text
STRAIGHT,STATUS
STRAIGHT,state=<S>,mode=UNTILUS,elapsed=<ms>,threshold_cm=<int>,us_mm=<uint-or-INF>,us_status=<int>,us_age_ms=<uint>,l_cm=<f>,r_cm=<f>,diff_cm=<f>,yaw=<f>
```

`RUNNING` 包括启动测距等待和实际行驶。`us_mm` 单位为毫米，`us_age_ms` 为最后一份样本的年龄，`elapsed` 为命令运行毫秒数。`us_status=0` 表示有效且新鲜，`-1` 表示无样本/无上升沿/已过期，`-2` 表示无效或过长回波。非零状态时不要将 `us_mm` 当作有效距离。终态后保留最后样本，但其年龄继续增加，因此之后查询可能显示过期。

### 3.3 侧面 IR 阈值 LEFTIR / RIGHTIR

GT / LT

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
OK,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,threshold_cm=<int>
DONE,STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>
```

- 向前直行，选定侧距离**严格大于**阈值时停车；等于阈值继续。阈值是传感器距离，不加超声波的保险杠偏移量。
- 阈值为整数厘米：LEFTIR 支持 10～50，RIGHTIR 支持 10～60，与当前标定表一致。
- `FAR`、ADC 读取失败和转换错误均按无限远处理，正常回报 `state=DONE,ir_cm=FAR`，不作为 IR 传感器故障。
- 起步前先采样，已满足条件则直接完成而不驱动车轮。行驶时约每 40 ms 采样一次，一次满足即可停车。
- 起步 PWM 4000，持续 120 ms，之后巡航 3300；仍使用全局 PWM 比例和直行方向／左右轮同步控制。
- STOP 返回 ABORTED；30 秒超时返回 TIMEOUT；任一轮行程达到 200 cm 返回 DISTANCE_LIMIT；陀螺仪故障仍可返回 SENSOR_ERROR。这些非成功终态会中止序列。
- 硬编码执行器收到正常 DONE 后进入下一行，包含 FAR／ADC 失败触发的正常完成。

运行中及完成后的查询格式：

```text
STRAIGHT,STATUS
STRAIGHT,mode=<LEFTIR|RIGHTIR>,op=<GT|LT>,state=<S>,threshold_cm=<int>,ir_cm=<decimal-or-FAR>
```

数值距离保留一位小数；`FAR` 不区分超量程和 ADC 失败。详见 [IR_STRAIGHT.md](IR_STRAIGHT.md)。

### 3.4 停止与时序

- `STRAIGHT,STOP` → `OK,aborted=<0|1>`；若中止运行中的直行（含测距等待），随后发送一次 `state=ABORTED` 完成事件。
- `STOP` → `OK`；停止电机、舵机回中，并中止正在运行的直行或转弯，随后发送对应完成事件。
- 首次运动可能在回复 `OK` 前进行陀螺仪初始化和约 5.5 秒静止校准；后续是否重新校准、等待由 `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` 控制，详见 [MOTION_SETTINGS.md](MOTION_SETTINGS.md)。首次初始化阶段仍是阻塞的。
- 运动控制按至少 20 ms 间隔步进；UART 收发仍可能延迟主循环。运动期间跳过 OLED 命令刷新，独立陀螺轮询暂停，避免重复积分。没有硬实时停车时延保证。

## 4. 转弯命令

| 请求 | 参数 / 回复 |
|---|---|
| `TURN,LEFT,R=<cm>,A=<deg>` | R 为整数且 ≥18；A 为有符号整数，绝对值 5～360 |
| `TURN,RIGHT,R=<cm>,A=<deg>` | R 为整数且 ≥35；A 同上 |
| 接受回复 | `OK,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| 终态事件 | `DONE,TURN,state=<S>,dir=<LEFT\|RIGHT>,r_cm=<int>,a=<int>` |
| `TURN,STOP` | `OK,aborted=<0\|1>`；若中止动作，随后发一次完成事件 |
| `TURN,STATUS` | `TURN,state=<S>,dir=<LEFT\|RIGHT>,elapsed=<ms>,r_cm=<int>,target_deg=<int>,yaw=<f>` |

LEFT/RIGHT 选择转向侧，A 正值前进、负值倒车。R 无额外配置上限，但必须能解析为 int32。终态文本包括 `DONE`、`ABORTED`、`TIMEOUT`、`WRONG_SIGN`、`NO_TURN`、`BAD_CONFIG`、`NO_IMU`（后两项为接口可识别状态）。当前转弯参数以 [turn_tuning.h](../Core/Inc/control/turn_tuning.h) 和 [turn_control.c](../Core/Src/control/turn_control.c) 为准。

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

支持 `FAST,LEFT1`、`FAST,LEFT2`、`FAST,RIGHT1`、`FAST,RIGHT2`。
它们分别执行 `test_hardcode_command_executor.c` 中同名分组的动作，
LEFT1／RIGHT1 各 6 条，LEFT2／RIGHT2 各 12 条。

与 LOOP,ONCE 共用序列执行器，每条运动成功完成后才开始下一条。
FAST 不增加启动倒计时或额外指令间隔；运动准备、重复陀螺仪校准、
转弯结束动作仍由 `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` 控制。
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
`STRAIGHT,STATUS`、`TURN,STATUS` 和 `HEADING,STATUS`。其他传入命令返回 `ERR,BUSY`。

## 7. UNTILUS 交互示例

示意值，不代表实车测量。`>` 为发送，`<` 为接收，不属于报文本身。

```text
> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20
> STRAIGHT,STATUS
< STRAIGHT,state=RUNNING,mode=UNTILUS,elapsed=420,threshold_cm=20,us_mm=320,us_status=0,us_age_ms=12,l_cm=+8.300,r_cm=+8.280,diff_cm=+0.020,yaw=+0.100
< DONE,STRAIGHT,mode=UNTILUS,state=DONE,reason=US_THRESHOLD,threshold_cm=20,us_mm=197,us_status=0

> STRAIGHT,UNTILUS,4
< ERR,BAD_VALUE

> STRAIGHT,UNTILUS,20
< OK,mode=UNTILUS,threshold_cm=20
> STOP
< OK
< DONE,STRAIGHT,mode=UNTILUS,state=ABORTED,reason=ABORTED,threshold_cm=20,us_mm=INF,us_status=-1
```

最后一个示例在取得首个有效样本前中止。RPi 应按字段名解析模式与终态，避免把 UNTILUS 的阈值当作 GOCM 的行程目标。

## 8. 配置与验证

- [MOTION_SETTINGS.md](MOTION_SETTINGS.md)：全局 PWM 比例、校准与指令衔接等待开关、主动制动。修改后须重新编译和烧录，不是 UART 动态参数。
- [IR_STRAIGHT.md](IR_STRAIGHT.md)：LEFTIR／RIGHTIR 指令及 `IR_COMMAND_*` 参数。
- v1.10 新增侧面 IR 指令及 FAR／ADC 失败正常完成规则。实现阶段已通过固件编译链接与主机模拟控制测试，尚未实车验证。

- [straight_tuning.h](../Core/Inc/control/straight_tuning.h)：SHORT/LONG 与 `US_*` 参数。
- [ULTRASONIC_STRAIGHT.md](ULTRASONIC_STRAIGHT.md)：接线、完整行为和手动验证步骤。
- [FLOOR_RETUNING.md](FLOOR_RETUNING.md)：地面变化后的控制参数校准。
- v1.9 同步了 UNTILUS、故障终态、运动互斥，以及当前无回显/无心跳的实现；修正了旧版 GOCM 范围和控制说明。
- 当前速度、转向衔接与停车余量仍需实车验证。

## 硬编码命令数组测试

`TEST_HARDCODE_COMMAND_EXECUTOR` 将命令数组执行一次，等待每个运动成功后再继续，遇到错误停止。此模式的 UART 仅接受 STOP、PING、STATUS、STRAIGHT,STATUS、TURN,STATUS 和 HEADING,STATUS。WAIT,<ms> 仅可用于数组。详见 [HARDCODE_COMMAND_EXECUTOR.md](HARDCODE_COMMAND_EXECUTOR.md)。

`us_mm=INF` 表示未知/无限远，`us_status` 仍为非零。上位机解析此字段时须同时支持数值和字符串 INF。

# 提交变化总结 / Commit Change Summary

整理日期 / Prepared: 2026-10-12（Asia/Singapore）  
仓库 / Repository: `mdp21_8`  
范围 / Scope: `970ea99` → `b49a968`，共 12 个提交 / 12 commits  
核对版本 / Reviewed HEAD: `b49a968f5b5f998c830e58fc82bc11c4cd39ac31`

本文依据各提交的代码、参数、测试及文档差异整理，排除初始提交 `f30da7f initial`。历史行为与当前行为分别说明；提交日期来自 Git，均为 UTC+08:00。已有功能只在后续提交修改它们时列入，讨论过但没有进入提交的方案不计作实现。

This document summarizes code, tuning, test and documentation changes in every commit after `f30da7f initial`. Historical behavior is distinguished from the reviewed HEAD. Commit dates use Git timestamps in UTC+08:00. Features inherited from the initial commit are mentioned only where later commits changed them; uncommitted proposals are excluded.

## 1. 逐次提交 / Commit-by-commit changes

### 1.1 `970ea99` · 2026-10-09 · IR cache

**中文：** 新增左右独立的 IR 停车缓存 `Straight_LeftIRStopCmX10`、`Straight_RightIRStopCmX10`。IR 条件满足并正常停止时，在制动前保存触发条件的读数，单位为 0.1 cm；初始化为 `UINT32_MAX`。只更新对应侧，STOP、超时和故障不覆盖已有缓存。扩展主机测试验证左右独立、缓存保留及异常读数行为。

**English:** Added independent left/right IR stop caches. A successful condition-triggered stop captures the triggering reading before braking, in 0.1 cm units. Initialization uses `UINT32_MAX`; only the selected side is updated. STOP, timeout and faults preserve previous values. Host tests cover independent updates, retention and exceptional readings.

### 1.2 `05d3962` · 2026-10-09 · LATERAL,LEFT/RIGHT,n/IR

**中文：** 新增 LATERAL 控制器与 UART 命令，支持指定侧移距离和现场 IR 测距两种模式。两次相反方向的等角转弯完成侧移，使用左软限制半径 20 cm、右最小半径 35 cm，保留左硬限制 18 cm。角度为 `round(acos(1-d/55) × 180/π)`，数值侧移范围 1～55 cm。初版 IR 模式现场测距后加 15 cm，再直接转弯。接入运动互斥、STOP、状态回报、序列完成检测及构建清单，新增侧移测试；FAST 第一组路线开始使用 LATERAL。

**English:** Introduced a LATERAL controller and UART commands for numeric displacement or a fresh IR measurement. Two opposite turns use equal angles, with a 20 cm left soft radius and a 35 cm right minimum radius; the 18 cm left hard limit remains. The rounded angle is `acos(1-d/55)` in degrees, with numeric displacement limited to 1..55 cm. Initially, IR mode added 15 cm to a fresh reading and turned immediately. Added motion interlocks, STOP, status/completion reporting, sequence integration, build entries and lateral tests. First-stage FAST routes began using LATERAL.

### 1.3 `f92bd67` · 2026-10-09 · improve UNTILUS

**中文：** UNTILUS 从单向接近升级为按初始超声距离自动选择方向：距离大于目标前进，小于目标后退，相等则确认后原地完成。启动方向要求连续有效读数确认，运动开始后锁定方向，停车条件按方向判断；前后都按剩余距离减速。增加样本新鲜度检查；启动阶段连续无有效距离会报传感器错误。该自动选向和无回波启动报错策略后来由 `d9ced81` 替换。

**English:** UNTILUS temporarily gained automatic direction selection from the starting ultrasonic distance: forward above the target, backward below it, and stationary completion at equality after confirmation. Confirmed startup readings selected a direction that remained fixed during motion; stopping and slowing were direction-aware. Added sample freshness checks and an initial invalid-reading failure policy. Automatic selection and initial no-echo failure were later replaced by `d9ced81`.

### 1.4 `1f8227b` · 2026-10-09 · cache IR, GT 50, then lateral

**中文：** 修改独立 `LATERAL,LEFT/RIGHT,IR`：先保存现场 IR，再前进到对应侧 IR 严格大于 50 cm，停车后用起始缓存计算两次侧移转弯。增加阶段 0 的直行等待、直行故障状态及阶段取消。前进阶段使用严格故障策略：FAR 可表示离开障碍物，ADC/选定探头错误会中止；后续读数不覆盖起始距离。同步状态中的阶段、原始缓存及底层直行结果，更新路线和测试。

**English:** Changed standalone IR-based LATERAL to save a fresh reading, drive forward until the selected IR exceeds 50 cm, stop, then execute the two arcs using the saved starting distance. Added a straight approach stage, straight-failure reporting and stage-aware cancellation. FAR may complete the approach, while acquisition/selected-sensor errors abort it. Subsequent readings do not overwrite the starting distance. Updated status fields, routes and tests.

### 1.5 `16059b2` · 2026-10-09 · change lateral compensation

**中文：** IR 侧移补偿从半车宽 10 cm 加半障碍宽 5 cm，改为探头到车中线 5 cm 加半障碍宽 5 cm，总补偿由 15 cm 改为 10 cm。引入 `LATERAL_IR_TO_CENTER_CM`，同步控制器、回复说明、测试和双语协议。提交历史中该步直接由 15 改为 10，未形成独立的 8 cm 补偿提交。

**English:** Replaced half vehicle width with the measured IR-to-centre offset: 5 cm sensor offset plus 5 cm obstacle half-width. Total IR displacement compensation changed from 15 to 10 cm. Updated the shared constant, controller, reply descriptions, tests and bilingual protocol. The committed transition is directly 15→10 cm; there is no separate 8 cm compensation commit.

### 1.6 `3d507e7` · 2026-10-09 · tuned straight gocm and untilus

**中文：** 普通 GOCM 前后方向、长短距离模式都增加 2 cm 的晚刹车行程修正，固定舵角校准模式保留原行为。UNTILUS 移除命令层硬编码的 `+8 cm` 安装偏移，直接使用 `US,GET` 的传感器距离；加入双向 50 mm 停车提前量，小修正时最多取初始误差的一半。近目标起步采用较低 PWM，减速立即生效、加速仍渐变。完成回报保存制动前超声读数、方向和检测时样本年龄，避免制动耗时后显示 INF。共享制动保持从 80 增至 150 ms；FAST 第一组侧移距离由 20 调到 25 cm。此时方向仍自动选择。

**English:** Added 2 cm of later-braking travel to ordinary GOCM in both gears and distance profiles, leaving fixed-steering calibration unchanged. Removed UNTILUS's command-level +8 cm installation offset and used the same sensor distance as US,GET. Added 50 mm stopping leads in both directions, capped at half the starting error for small adjustments. Near starts use lower PWM; deceleration is immediate while acceleration remains gradual. Completion retains the pre-braking ultrasonic reading, direction and sample age to avoid reporting a stale INF result. Shared braking increased from 80 to 150 ms, and first-stage FAST lateral distances changed from 20 to 25 cm. Direction was still selected automatically at this stage.

### 1.7 `b41b4ee` · 2026-10-10 · tuned parameters

**中文：** 把 `GYRO_RECALIBRATE_BEFORE_EACH_COMMAND` 的职责收窄为重复零偏刷新，并默认关闭；准备制动、静止检查、转弯后稳定、右转回中动作、左转关闭航向、航向历史重置及序列间隔改用独立开关，提交中的这些开关开启。首次校准仍要求停车与静止。转弯起步始终保留舵机到位等待，按指令脉宽行程估算 80～500 ms，独立于陀螺仪开关。右转两段定位由各 500 改为各 150 ms，回差定位由各 120 改为各 80 ms；全局 PWM 比例由 120% 改为 140%。加入舵机等待及结束策略测试。

**English:** Limited the gyro recalibration switch to repeated bias refresh and disabled it by default. Separate enabled switches now control preparation braking, stillness checks, post-turn settling, right return positioning, left heading disable, heading-history reset and sequence gaps. First calibration still requires braking and stillness. Turn startup always waits for steering, independently of gyro recalibration, using an 80..500 ms estimate based on commanded pulse travel. Right return holds changed from 500 to 150 ms each, backlash holds from 120 to 80 ms each, and global drive scaling from 120% to 140%. Added steering-wait and end-policy tests.

### 1.8 `d9ced81` · 2026-10-10 · 组合运动、双向传感器命令及诊断 / Composite motion, explicit directions and diagnostics

**中文：** 这一提交集中实现了以下功能：

- 新增 `BYPASS,LEFT/RIGHT,n`：三次转弯角度为 `a,2a,a`，转向为左—右—左或右—左—右；沿用 20/35 cm 半径，n 为最大侧向偏移，范围 1～55 cm。加入状态、STOP、故障中止、序列及构建集成。
- UNTILUS 改为显式方向：默认前进，也支持 `STRAIGHT,FORWARD/BACKWARD,UNTILUS,n`。方向不再自动反转；无效、过期或无回波时按指定方向低速继续测量。硬件/陀螺仪故障及 STOP、失速、超时、行程上限仍可终止。
- 超声接近减速范围由 25 增至 35 cm，接近 PWM 2400、末段 20 cm 为 1800。所有停车越限需连续两次有效读数且差值不超过 50 mm；制动至少 150 ms、两轮静止连续 60 ms 后释放，最长 600 ms，仍运动则报 STALL。补上初始测量等待阶段的超时检查。
- 普通 IR 支持 FORWARD/BACKWARD，以及 GT/LT/RISING/FALLING；回复方向统一为 FORWARD/BACKWARD。GT/LT 连续确认两次，边缘检测两阶段分别连续确认两次，等于阈值不算越限。
- 新增连续 `STRAIGHT,GOCM,x,LEFTIR/RIGHTIR,GT/LT/RISING/FALLING,n`：两轮达到指定距离前不读 IR，到达后不停、不重置编码器或航向，直接开始 IR 检测。
- 新增上述边缘命令的 `,LATERAL` 后缀：越沿停车后按缓存 IR +10 cm 向探头侧平移。RISING 从首个有效低于阈值的读数起维护后续最小值；FALLING 保留首阶段确认对中的第一条读数。无有效距离或补偿后超过 55 cm 会阻止转弯。
- 新增 `LEFTIR,GET`、`RIGHTIR,GET`，回报距离、ADC 原值、毫伏、量程状态和驱动返回码。IR 运动状态追加原始读数、量程、年龄、确认计数和已确认阶段。
- 修复 FAST 预检仍限制 4 字段的问题，允许最多 7 字段；更新 FAST 路线、协议、运动文档与 IR/侧移/绕障/组合动作测试。

**English:** This commit introduced several coordinated changes:

- Added three-turn BYPASS using angles `a,2a,a` and alternating steering. Radii remain 20/35 cm; n is maximum lateral excursion, limited to 1..55 cm. Integrated status, STOP, fault handling, sequencing and build files.
- Replaced automatic UNTILUS direction selection with explicit FORWARD/BACKWARD, defaulting to forward. Invalid, stale or missing echoes cause slow motion in the requested direction while acquisition continues. Hardware/gyro errors, STOP, stall, timeout and travel limits still terminate motion.
- Expanded ultrasonic slowdown from 25 to 35 cm, with approach PWM 2400 and final PWM 1800 within 20 cm. Every stop crossing requires two valid echoes differing by no more than 50 mm. Braking lasts at least 150 ms and waits for 60 ms of encoder quiet, bounded to 600 ms; continued motion reports STALL. Added timeout coverage for initial measurement waiting.
- Added explicit directions and RISING/FALLING to ordinary IR commands. Direction replies use FORWARD/BACKWARD. GT/LT require two consecutive confirmations; each edge phase also requires two. Equality does not qualify.
- Added continuous GOCM→IR commands. IR acquisition starts only after both wheels reach the distance gate, without an intermediate stop or encoder/heading reset.
- Added the edge→LATERAL suffix. After stopping at the completed edge, lateral distance is cached IR +10 cm toward the selected sensor. RISING maintains the minimum valid reading from the first below-threshold sample onward; FALLING keeps the first reading in its confirmed initial phase. Missing finite distance or displacement above 55 cm prevents turning.
- Added left/right IR GET commands and detailed raw/range/age/confirmation diagnostics.
- Raised FAST preflight support from four to seven fields, updated routes and documentation, and expanded IR, lateral, bypass and composite-motion tests.

### 1.9 `8609bdf` · 2026-10-10 · IR减速 / IR slowdown

**中文：** 所有正在启用 IR 比较的阶段统一为基础 PWM 1800，移除 IR 的 4000 PWM/120 ms 起步和 3300 巡航；覆盖双向、左右探头、全部比较类型及 GOCM/LATERAL 组合。GOCM 前段保留原曲线，启用 IR 时立即降速。新增低速覆盖测试。FAST LEFT2/RIGHT2 暂时注释倒车 LT 步骤，组合侧移前的 GOCM 从 30 改为 50 cm。

**English:** Unified every active IR comparison stage at base PWM 1800, removing the 4000-PWM startup and 3300-PWM cruise. Coverage includes both gears/sensors, every comparison and GOCM/LATERAL combinations. GOCM retains its original profile before the IR gate and reduces speed at entry. Added speed coverage tests. Temporarily removed the backward LT row from LEFT2/RIGHT2 and changed their composite-command GOCM distances from 30 to 50 cm.

### 1.10 `0c83cdf` · 2026-10-10 · reduce delay

**中文：** IR 基础 PWM 从 1800 调到 1900。准备制动合并为一次，至少保持 150 ms，同时检查编码器静止；普通运动连续静止要求从 250 降到 100 ms，首次或重复零偏校准仍用 250 ms，最多等待 1000 ms。取消转弯前两处重复制动和 UNTILUS 外层重复准备制动。转弯后等待从 500 降到 150 ms，右转两段定位各 150→100 ms，回差定位各 80→60 ms，转弯舵机等待估算范围 80～500→60～400 ms，直行继承航向时的舵机等待 100→60 ms，LOOP 额外间隔 300→0 ms。保留目标停车制动、IR 40 ms 采样和校准要求，更新等待测试及文档。

**English:** Raised IR base PWM from 1800 to 1900. Preparation now uses one brake held for at least 150 ms with concurrent encoder checks. Normal stillness changed from 250 to 100 ms; calibration retains 250 ms, with a 1000 ms timeout. Removed duplicate turn preparation brakes and the outer UNTILUS preparation brake. Post-turn settling changed 500→150 ms, right holds 150→100 ms each, backlash holds 80→60 ms each, estimated turn steering wait 80..500→60..400 ms, inherited straight-heading steering wait 100→60 ms, and LOOP gaps 300→0 ms. Target braking, 40 ms IR acquisition and calibration requirements remain. Updated tests and documentation.

### 1.11 `6a9c337` · 2026-10-10 · FAST,APPROACH

**中文：** 新增 `FAST,APPROACH`，只有 `STRAIGHT,UNTILUS,28` 一条命令，默认前进。复用已有 FAST 执行、互斥、STOP、失败中止和完成回报机制，count 为 1，同步中英文协议。

**English:** Added FAST,APPROACH as a one-row group containing STRAIGHT,UNTILUS,28, defaulting to forward. Reuses FAST execution, interlocks, cancellation, failure and completion reporting, with count=1. Updated both protocol documents.

### 1.12 `b49a968` · 2026-10-10 · 分号批次与手动中心值 / Semicolon batches and manual centre setting

**中文：** 支持一行用分号提交多个命令，按序列执行器逐条等待运动完成，保留子命令回复，最后回报 `DONE,BATCH`；支持嵌套 FAST 和序列 WAIT，错误或另行发送 STOP 会终止后续步骤。最多 16 条，整行 511 字节，每条 127 字节、7 字段；允许子命令两端空白，拒绝空段及超限。接收行缓冲扩至 512 字节，串口环形缓冲扩至 1024 字节；批次保存到独立空间，并在分派前清零接收长度，防止递归轮询覆盖剩余命令。新增相关主机测试。新增 `SERVO,SETCENTER,n`，保留 `SERVO,CENTER,n` 别名；校验整数、端点范围和运动 BUSY，重置航向并更新运行时中心、执行回差定位，重启恢复源码默认值。另恢复 FAST LEFT2/RIGHT2 的倒车 LT 步骤，因此两组当前各 8 条。更新双语文档。

**English:** Added semicolon-separated batches that wait for each managed motion, retain individual replies and finish with DONE,BATCH. Supports nested FAST and sequence WAIT; errors or a separate STOP cancel remaining steps. Limits are 16 commands, 511 total bytes, 127 bytes/seven fields per command. Trims outer whitespace and rejects empty/oversized segments. Enlarged RX line storage to 512 bytes and the RX ring to 1024, saved batches independently, and cleared RX length before dispatch to prevent recursive polling from corrupting pending commands. Added host tests. Introduced SERVO,SETCENTER,n while retaining SERVO,CENTER,n; validates integer/range/BUSY, resets heading, updates the runtime centre and performs backlash positioning. Restart restores the compiled default. Restored backward LT rows in LEFT2/RIGHT2, bringing each to eight commands. Updated bilingual documentation.

## 2. 当前功能与限制 / Current behavior and limits

| 功能 / Feature | 截至 `b49a968` 的行为 / Behavior at reviewed HEAD |
|---|---|
| UNTILUS | 默认前进，可显式 FORWARD/BACKWARD；目标 5～200 cm，无 +8 cm 安装偏移 / Defaults forward, explicit gear, target 5..200 cm, no +8 cm installation offset |
| 普通 IR / Ordinary IR | 左阈值 10～50 cm、右 10～60 cm；双向 GT/LT/RISING/FALLING；40 ms 采样、两次确认 / Side-specific ranges, both gears and four comparisons, 40 ms acquisition, two confirmations |
| IR 速度 / IR speed | 基础 PWM 1900，仍乘全局 140% 并受轮同步控制影响 / Base PWM 1900, with 140% global scaling and wheel synchronization |
| LATERAL | 两次转弯，左 20/右 35 cm；目标最多 55 cm；IR 补偿 +10 cm / Two arcs, 20/35 cm radii, maximum 55 cm displacement, +10 cm IR offset |
| 独立 LATERAL,IR / Standalone IR lateral | 现场缓存起始距离，再前进到 IR>50 后转弯 / Fresh starting snapshot, forward approach until IR>50, then arcs |
| 边缘组合 / Edge composite | GOCM 到距离后连续检测；RISING 最小值/FALLING 首阶段值 +10；停车后侧移 / Continuous gated detection, cached minimum/first-phase value +10, stop then lateral |
| BYPASS | 三次转弯，数值偏移 1～55 cm；没有 IR 模式 / Three arcs, numeric excursion 1..55 cm; no IR mode |
| FAST | APPROACH 1 条、LEFT1/RIGHT1 各 2 条、LEFT2/RIGHT2 各 8 条 / Route counts: 1, 2/2, 8/8 |
| 停车 / Braking | 目标制动 150 ms；UNTILUS 额外静止确认，上限 600 ms / 150 ms target braking; UNTILUS adds quiet confirmation, maximum 600 ms |
| 舵机中心 / Servo centre | 编译默认 1725 µs；可运行时设置，直行方向微调仍生效 / Compiled default 1725 µs; runtime override with direction trims retained |
| 批次 / Batch | `;` 顺序执行；最多 16 条、511 字节，单条 127 字节 / Sequential execution, up to 16 rows/511 bytes, 127 bytes per row |

当前 LATERAL/BYPASS 都限制最大侧向距离 55 cm，未实现“超过 55 cm 时在两段 90° 转弯中间加入直行”。RISING 的最小缓存与正常停车时保存的最终 IR 缓存是不同用途。IR、GOCM 和 TURN 的目标制动仍按自身现有策略执行，只有 UNTILUS 增加了 60 ms 编码器静止释放条件。

LATERAL and BYPASS remain limited to 55 cm. The proposed straight segment between two 90° arcs for larger offsets is absent. RISING's minimum cache serves a different purpose from the final condition-triggered IR stop cache. IR, GOCM and TURN retain their respective braking policies; only UNTILUS adds the 60 ms encoder-quiet release criterion.

## 3. 代码、测试与文档 / Code, tests and documentation

主要实现集中于 [command_interface.c](../Core/Src/assessment/command_interface.c)、[straight_control.c](../Core/Src/control/straight_control.c)、[lateral_control.c](../Core/Src/control/lateral_control.c)、[bypass_control.c](../Core/Src/control/bypass_control.c)、[turn_control.c](../Core/Src/control/turn_control.c) 和 [robot_hw.c](../Core/Src/drivers/robot_hw.c)。新控制器同时接入 `sources.mk` 与 STM32 Debug 构建清单。

Implementation changes are concentrated in the command interface, straight/lateral/bypass/turn controllers and shared hardware driver linked above. New controllers are included in both the source list and STM32 Debug build entries.

新增或扩展的主机测试包括 `test_ir_straight.c`、`test_lateral.c`、`test_bypass.c`、`test_edge_lateral.c`、`test_turn_servo_wait.c` 与 `test_inline_batch.py`，覆盖方向、探头、条件确认、缓存、GOCM 门控、组合取消、异常、舵机等待和串口批次接收。测试文件的存在说明覆盖范围；本文整理期间未重新运行固件或实车测试，也不据此承诺实际停车精度。

Added/expanded host tests cover direction and sensor combinations, confirmation, caching, distance gates, composite cancellation, faults, steering waits and batch RX behavior. This history review did not rerun firmware or hardware tests; test coverage does not establish physical stopping accuracy.

文档更新涉及 [中文协议](PROTOCOL_zh.md)、[English protocol](PROTOCOL_en.md)、[运动设置](MOTION_SETTINGS.md)、[IR 直行说明](IR_STRAIGHT.md)、地面调参与 assessment README。旧说明中的路线条数、开关默认值及参数可能滞后，本总结中的当前值已对照 HEAD 源码核对。仅改动注释或换行格式的差异未逐条展开。

Documentation changes include the bilingual protocol, motion settings, IR straight behavior, floor retuning and assessment README. Some older descriptions lag route counts, switch defaults or tuning; current values in this summary were checked against HEAD source. Comment-only and line-ending changes are not individually expanded.
